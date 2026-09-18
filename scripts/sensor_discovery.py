#!/usr/bin/env python3
# sensor_discovery.py — 리그에 실제로 붙어 있는 센서를 네트워크에서 찾는다.
#
# 센서군마다 찾는 방법이 다르다. 공통 반환형은 아래 device dict 하나로 맞춘다:
#
#   {group, subset, nic, ip, identity, model, vendor, mac, state, note}
#
#   state: ok      설정(인벤토리)에 있고 실제로도 보임
#          new     보이는데 설정에 없음 (새로 꽂은 장비)
#          missing 설정에 있는데 안 보임 (전원/케이블)
#          subnet  보이긴 하는데 호스트와 다른 서브넷 (ForceIP 필요)
#
# 단독 실행:  python3 sensor_discovery.py [group_key ...]
#
# 주의: 이 모듈은 아무것도 바꾸지 않는다. 읽고 찾기만 한다.

import os
import socket
import struct
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import yaml

import net_tools

OK, NEW, MISSING, SUBNET = "ok", "new", "missing", "subnet"

# 설치본(share/clip_recorder/config) → 소스 트리(config) 순으로 찾는다.
# 설치본에서 이 파일은 <prefix>/lib/clip_recorder/ 에 놓이므로 prefix 는 parents[1].
_HERE = Path(__file__).resolve().parent
REGISTRY_CANDIDATES = (
    _HERE.parents[1] / "share" / "clip_recorder" / "config" / "sensors.yaml",
    _HERE.parent / "config" / "sensors.yaml",
    Path.home() / "DM_clipGUI" / "config" / "sensors.yaml",
)


def _ament_registry():
    """ROS 환경이면 ament 인덱스가 가장 정확하다. 없으면 조용히 건너뛴다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("clip_recorder")) / "config" / "sensors.yaml"
    except Exception:                                             # noqa: BLE001
        return None


def registry_path():
    ament = _ament_registry()
    for cand in ((ament,) if ament else ()) + REGISTRY_CANDIDATES:
        if cand.is_file():
            return cand
    return None


def load_registry(path=None):
    """sensors.yaml 로드. 못 찾으면 {'groups': []}."""
    path = Path(path) if path else registry_path()
    if not path or not path.is_file():
        return {"groups": []}
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {"groups": []}


def expand(path):
    """레지스트리에 적힌 ~ 경로 → 절대 경로. 빈 값은 None."""
    if not path:
        return None
    return Path(os.path.expanduser(str(path)))


# ---------- params / 인벤토리 읽기 ----------

def load_params(path):
    """ROS params YAML → (node_key, {키: 값}). 실패하면 (None, {})."""
    path = expand(path)
    if not path or not path.is_file():
        return None, {}
    try:
        with open(path, "r", encoding="utf-8") as stream:
            doc = yaml.safe_load(stream) or {}
    except yaml.YAMLError:
        return None, {}
    for node_key, body in doc.items():
        if isinstance(body, dict) and "ros__parameters" in body:
            return node_key, dict(body["ros__parameters"] or {})
    return None, {}


def load_inventory(path):
    """multicam_cameras.yaml → {serial: {name, namespace, force_ip_address, ...}}"""
    _, params = load_params(path)
    out = {}
    for cam in params.get("cameras") or []:
        if isinstance(cam, dict) and cam.get("serial"):
            out[str(cam["serial"])] = cam
    return out


# ---------- NIC 서브넷 ----------

def _nic_subnets(nics):
    """{iface: (network_int, mask_int)} — 감지된 IP가 우리 망 안인지 판정용."""
    out = {}
    for iface in nics:
        addr = net_tools.nic_ipv4(iface)
        if not addr:
            continue
        ip, plen = addr
        mask = (0xFFFFFFFF << (32 - plen)) & 0xFFFFFFFF
        out[iface] = (struct.unpack(">I", socket.inet_aton(ip))[0] & mask, mask)
    return out


def _in_subnet(ip, subnet):
    try:
        value = struct.unpack(">I", socket.inet_aton(ip))[0]
    except OSError:
        return False
    network, mask = subnet
    return value & mask == network


# ---------- GigE Vision 카메라 ----------

def discover_gvcp(group):
    """GVCP 브로드캐스트를 NIC마다 쏴서 카메라를 모으고, 모델명으로 subset을 가른다.

    net_tools.gvcp_discover()를 그대로 쓴다 — GUI의 네트워크 패널이 쓰는 것과 같은 코드이고,
    root 권한이 필요 없다. 다른 점은 NIC 하나가 아니라 전부 훑는다는 것뿐이다.
    """
    nics = group.get("discovery", {}).get("nics") or [n for n, _ in net_tools.nic_list()]
    subnets = _nic_subnets(nics)
    subsets = group.get("subsets") or []

    found = {}                       # ip -> (nic, info)
    for iface in nics:
        try:
            for ip, info in net_tools.gvcp_discover(iface, timeout=1.0).items():
                found.setdefault(ip, (iface, info))
        except OSError:
            continue

    def subset_of(model):
        for sub in subsets:
            for pattern in sub.get("model_patterns") or []:
                if pattern.lower() in (model or "").lower():
                    return sub
        return None

    devices, seen_serials = [], set()
    for ip, (iface, info) in sorted(found.items()):
        sub = subset_of(info.get("model"))
        inventory = load_inventory(sub.get("inventory")) if sub else {}
        serial = info.get("serial") or ""
        seen_serials.add(serial)
        entry = inventory.get(serial)
        if not _in_subnet(ip, subnets.get(iface, (0, 0))):
            state, note = SUBNET, "호스트와 다른 서브넷 — ForceIP 필요"
        elif entry:
            state, note = OK, entry.get("namespace") or entry.get("name") or ""
        else:
            state, note = NEW, "인벤토리에 없음"
        devices.append({
            "group": group["key"],
            "subset": sub["key"] if sub else None,
            "subset_label": sub["label"] if sub else "미분류 GigE",
            "nic": iface,
            "ip": ip,
            "identity": serial,
            "model": info.get("model") or "",
            "vendor": info.get("vendor") or "",
            "mac": info.get("mac") or "",
            "state": state,
            "note": note,
        })

    # 인벤토리에는 있는데 응답이 없는 카메라 — 8대여야 하는데 7대인 상황이 여기서 잡힌다.
    for sub in subsets:
        for serial, entry in load_inventory(sub.get("inventory")).items():
            if serial in seen_serials:
                continue
            devices.append({
                "group": group["key"],
                "subset": sub["key"],
                "subset_label": sub["label"],
                "nic": "",
                "ip": entry.get("force_ip_address") or "",
                "identity": serial,
                "model": "",
                "vendor": "",
                "mac": entry.get("mac_address") or "",
                "state": MISSING,
                "note": entry.get("namespace") or entry.get("name") or "",
            })
    return devices


# ---------- Ouster ----------

def _tcp_open(host, port, timeout):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (OSError, socket.timeout):
        return False


def _ouster_metadata(host, timeout):
    """http://<host>/api/v1/sensor/metadata → (serial, model, firmware). 실패 시 (,,)."""
    url = f"http://{host}/api/v1/sensor/metadata"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            doc = yaml.safe_load(response.read().decode("utf-8", "replace")) or {}
    except (urllib.error.URLError, OSError, yaml.YAMLError, ValueError):
        return "", "", ""
    info = doc.get("sensor_info") or doc
    return (str(info.get("prod_sn") or info.get("serial_number") or ""),
            str(info.get("prod_line") or info.get("product_line") or ""),
            str(info.get("build_rev") or info.get("image_rev") or ""))


def discover_ouster(group):
    """설정에 적힌 주소(+사용자 추가분)만 TCP로 두드린다.

    링크로컬 169.254.0.0/16 전수 스캔은 6만 5천 번의 connect라 하지 않는다.
    프로브 방식은 all_sensors.launch.py의 _check_lidar_reachable과 같다 —
    ICMP가 아니라 TCP 연결이어야 "부팅돼서 서비스 중"임이 증명된다.
    """
    cfg = group.get("discovery", {})
    port = int(cfg.get("port") or 80)
    timeout = float(cfg.get("timeout_s") or 3.0)

    hosts = list(cfg.get("hosts") or [])
    _, params = load_params(group.get("params"))
    if params.get("sensor_hostname"):
        hosts.append(str(params["sensor_hostname"]))
    for extra in group.get("_extra_hosts") or []:
        hosts.append(extra)

    devices, seen = [], set()
    for host in hosts:
        if not host or host in seen:
            continue
        seen.add(host)
        if _tcp_open(host, port, timeout):
            serial, model, firmware = _ouster_metadata(host, timeout)
            devices.append({
                "group": group["key"], "subset": None,
                "subset_label": group["label"], "nic": "", "ip": host,
                "identity": serial, "model": model or "Ouster",
                "vendor": "Ouster", "mac": "", "state": OK,
                "note": f"펌웨어 {firmware}" if firmware else f"TCP {port} 응답",
            })
        else:
            devices.append({
                "group": group["key"], "subset": None,
                "subset_label": group["label"], "nic": "", "ip": host,
                "identity": "", "model": "", "vendor": "Ouster", "mac": "",
                "state": MISSING, "note": f"TCP {port} 무응답",
            })
    return devices


# ---------- OxTS NCOM (RT2000) ----------

def discover_ncom(group):
    """UDP 3000을 잠깐 듣는다. NCOM은 브로드캐스트라 드라이버가 떠 있어도 같이 받는다.

    NCOM 패킷은 72바이트이고 첫 바이트가 동기 문자 0xE7이다. 소스 IP가 곧 장비 주소.
    """
    cfg = group.get("discovery", {})
    port = int(cfg.get("port") or 3000)
    timeout = float(cfg.get("timeout_s") or 2.0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    senders = {}
    try:
        sock.bind(("", port))
        sock.settimeout(0.3)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                data, src = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) >= 72 and data[0] == 0xE7:
                senders[src[0]] = senders.get(src[0], 0) + 1
    except OSError:
        pass
    finally:
        sock.close()

    if not senders:
        return [{
            "group": group["key"], "subset": None, "subset_label": group["label"],
            "nic": "", "ip": "", "identity": "", "model": "", "vendor": "OxTS",
            "mac": "", "state": MISSING,
            "note": f"UDP {port}에서 NCOM 미수신 (장비 이더넷 출력 확인)",
        }]

    own = net_tools.own_ipv4s()
    return [{
        "group": group["key"], "subset": None, "subset_label": group["label"],
        "nic": _nic_for_ip(ip), "ip": ip, "identity": "", "model": "NCOM",
        "vendor": "OxTS", "mac": "", "state": OK,
        "note": f"{count}패킷/{timeout:g}초" + (" (이 PC)" if ip in own else ""),
    } for ip, count in sorted(senders.items())]


def _nic_for_ip(ip):
    """그 IP와 같은 서브넷인 NIC 이름 (표시용). 없으면 ''."""
    for iface, _ in net_tools.nic_list():
        addr = net_tools.nic_ipv4(iface)
        if not addr:
            continue
        host_ip, plen = addr
        mask = (0xFFFFFFFF << (32 - plen)) & 0xFFFFFFFF
        network = struct.unpack(">I", socket.inet_aton(host_ip))[0] & mask
        if _in_subnet(ip, (network, mask)):
            return iface
    return ""


# ---------- 디스패치 ----------

_KINDS = {"gvcp": discover_gvcp, "ouster_probe": discover_ouster, "ncom_listen": discover_ncom}


def discover_group(group):
    """센서군 하나를 감지. 어떤 예외도 밖으로 내보내지 않는다 (GUI가 죽으면 안 된다)."""
    kind = (group.get("discovery") or {}).get("kind")
    handler = _KINDS.get(kind)
    if handler is None:
        return []
    try:
        return handler(group)
    except Exception as exc:                                  # noqa: BLE001
        return [{
            "group": group["key"], "subset": None, "subset_label": group.get("label", ""),
            "nic": "", "ip": "", "identity": "", "model": "", "vendor": "", "mac": "",
            "state": MISSING, "note": f"감지 실패: {exc}",
        }]


def discover_all(registry=None):
    """{group_key: [device, ...]}"""
    registry = registry or load_registry()
    return {g["key"]: discover_group(g) for g in registry.get("groups", [])}


def summarize_by_subset(devices):
    """[(subset_label, 정상, 전체, 문제 요약)] — 가시광 Blackfly 와 열화상 A70 은
    대수도 상태도 따로 세야 한다. 합쳐서 "2/10" 이라고 하면 어느 쪽이 빠졌는지 안 보인다."""
    order, buckets = [], {}
    for dev in devices:
        label = dev["subset_label"]
        if label not in buckets:
            order.append(label)
            buckets[label] = []
        buckets[label].append(dev)
    out = []
    for label in order:
        group_devices = buckets[label]
        ok = sum(1 for d in group_devices if d["state"] == OK)
        problems = []
        for state, text in ((MISSING, "안 보임"), (SUBNET, "서브넷 불일치"), (NEW, "설정에 없음")):
            count = sum(1 for d in group_devices if d["state"] == state)
            if count:
                problems.append(f"{count}대 {text}")
        out.append((label, ok, len(group_devices), ", ".join(problems)))
    return out


def summarize(devices):
    """센서군 요약: (정상 대수, 전체 대수, 한 줄 설명)"""
    live = [d for d in devices if d["state"] in (OK, SUBNET, NEW)]
    ok = [d for d in devices if d["state"] == OK]
    ips = [d["ip"] for d in live if d["ip"]]
    span = ""
    if ips:
        span = ips[0] if len(ips) == 1 else f"{ips[0]} ~ {ips[-1]}"
    problems = []
    for state, text in ((MISSING, "안 보임"), (SUBNET, "서브넷 불일치"), (NEW, "설정에 없음")):
        count = sum(1 for d in devices if d["state"] == state)
        if count:
            problems.append(f"{count}대 {text}")
    note = ", ".join(problems) if problems else "전부 정상"
    return len(ok), len(devices), f"{span}  {note}".strip()


STATE_MARK = {OK: "OK", NEW: "NEW", MISSING: "--", SUBNET: "!!"}


def main(argv):
    registry = load_registry()
    if not registry.get("groups"):
        print("sensors.yaml을 찾지 못했습니다:", [str(c) for c in REGISTRY_CANDIDATES])
        return 1
    wanted = set(argv[1:])
    for group in registry["groups"]:
        if wanted and group["key"] not in wanted:
            continue
        devices = discover_group(group)
        ok, total, note = summarize(devices)
        print(f"\n=== {group['label']} ({group['key']}) — {ok}/{total} 정상 · {note}")
        for label, sub_ok, sub_total, sub_note in summarize_by_subset(devices):
            print(f"    {label:16s} {sub_ok}/{sub_total} 정상"
                  + (f" · {sub_note}" if sub_note else ""))
        for d in devices:
            print(f"  [{STATE_MARK.get(d['state'], '?')}] {d['subset_label']:16s} "
                  f"{d['ip']:16s} {d['identity']:12s} {d['model']:20s} "
                  f"{d['nic']:10s} {d['note']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
