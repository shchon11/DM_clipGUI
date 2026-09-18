#!/usr/bin/env python3
# sensor_discovery.py — 리그에 실제로 붙어 있는 센서를 네트워크에서 찾는다.
#
# 센서군마다 찾는 방법이 다르다. 공통 반환형은 아래 device dict 하나로 맞춘다:
#
#   {group, subset, nic, ip, identity, model, vendor, mac, state, note}
#
# 보이는 장비만 돌려준다. 리그 대수는 고정이 아니라서 "몇 대여야 하는데 몇 대 빠졌다"는
# 판정은 하지 않는다 — 인벤토리 YAML은 이름(namespace)을 붙이는 데만 쓴다.
# 인벤토리에 없는 새 카메라도 문제가 아니다: multicam.launch.py 가 기동할 때
# auto_update_cameras_file / auto_update_thermal_cameras_file (기본 true) 로 알아서 추가한다.
#
#   state: ok       보임
#          subnet   보이긴 하는데 이대로는 못 연다 — 다른 서브넷, IP 미설정(169.254.x.x),
#                   또는 다른 카메라와 IP 충돌. [IP 할당] 으로 해결 (net_tools.gvcp_force_ip)
#          updater  펌웨어 업데이터 모드 (GVCP 모델명이 "Updater") — 전원을 다시 넣어야 한다
#          nic      장비는 보이는데(mDNS) PC 쪽 NIC 에 IPv4 가 없어서 못 붙는다 — 라이다를 새 NIC
#                   (USB 이더넷 어댑터 등)에 꽂았을 때. [NIC 설정] 으로 해결 (net_tools.nm_link_local)
#
# 단독 실행:  python3 sensor_discovery.py [group_key ...]
#
# 주의: 이 모듈은 아무것도 바꾸지 않는다. 읽고 찾기만 한다.

import json
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import yaml

import net_tools

OK, SUBNET, UPDATER, NIC = "ok", "subnet", "updater", "nic"

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

def wired_nics():
    """UP 상태의 유선 NIC. GigE Vision 카메라가 WiFi 너머에 있을 일은 없다."""
    return [n for n, _ in net_tools.nic_list()
            if not (Path("/sys/class/net") / n / "wireless").exists()]


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
    nics = group.get("discovery", {}).get("nics") or wired_nics()
    subnets = _nic_subnets(nics)
    subsets = group.get("subsets") or []

    found = {}                       # mac -> (nic, info) — IP 가 겹쳐도 둘 다 남는다
    for iface in nics:
        try:
            for info in net_tools.gvcp_discover_all(iface, timeout=1.0):
                found.setdefault(info["mac"], (iface, info))
        except OSError:
            continue
    ip_count = {}
    for _iface, info in found.values():
        ip_count[info["ip"]] = ip_count.get(info["ip"], 0) + 1

    def subset_of(model):
        for sub in subsets:
            for pattern in sub.get("model_patterns") or []:
                if pattern.lower() in (model or "").lower():
                    return sub
        return None

    def subset_of_updater(serial):
        """업데이터 모드면 모델명이 "Updater" 라 종류를 모른다 — 인벤토리에 있으면 그 종류,
        없으면 첫 종류(가시광). 업데이터 모드는 Spinnaker 카메라(Blackfly)의 상태다."""
        for sub in subsets:
            if serial in load_inventory(sub.get("inventory")):
                return sub
        return subsets[0] if subsets else None

    devices = []
    for iface, info in sorted(found.values(), key=lambda v: _ip_sort_key(v[1]["ip"])):
        ip = info["ip"]
        serial = info.get("serial") or ""
        updater = "updater" in (info.get("model") or "").lower()
        sub = subset_of_updater(serial) if updater else subset_of(info.get("model"))
        inventory = load_inventory(sub.get("inventory")) if sub else {}
        entry = inventory.get(serial) or {}
        name = entry.get("namespace") or entry.get("name") or ""
        if updater:
            state, note = UPDATER, "업데이터 모드 — 카메라 전원을 다시 넣어야 합니다"
        elif not _in_subnet(ip, subnets.get(iface, (0, 0))):
            kind = "IP 미설정(링크로컬)" if ip.startswith("169.254.") else "호스트와 다른 서브넷"
            state, note = SUBNET, f"{kind} — IP 할당 필요"
        elif ip_count[ip] > 1:
            # 같은 IP 를 여러 대가 쓰면 스트림이 엉뚱한 곳으로 가서 죽는다 (A70 에서 실제로 겪음)
            state, note = SUBNET, f"IP 충돌 — {ip} 를 {ip_count[ip]}대가 씀, IP 할당 필요"
        else:
            state, note = OK, name or "새 장비 (기동 시 인벤토리에 자동 추가)"
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
    return devices


def _ip_sort_key(ip):
    """192.168.1.2 가 192.168.1.11 보다 앞에 오게 (문자열 정렬이면 뒤집힌다)."""
    try:
        return struct.unpack(">I", socket.inet_aton(ip))[0]
    except OSError:
        return 0


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
    """Ouster 찾기: (1) 설정에 적힌 주소를 TCP 로 두드리고 (2) 센서가 스스로 알리는 mDNS 를 듣는다.

    링크로컬 169.254.0.0/16 전수 스캔은 6만 5천 번의 connect라 하지 않는다.
    TCP 프로브는 all_sensors.launch.py의 _check_lidar_reachable과 같다 —
    ICMP가 아니라 TCP 연결이어야 "부팅돼서 서비스 중"임이 증명된다.

    mDNS(_roger._tcp)는 라이다를 다른 NIC(USB 이더넷 어댑터 등)로 옮겨 꽂았을 때를 위한 것이다.
    그 NIC 에 PC 쪽 IPv4 가 없으면 TCP 로는 못 닿지만, IPv6 link-local 로 알리는 mDNS 는 들린다
    → 어느 NIC 에 붙었는지 알 수 있고, [NIC 설정] 으로 그 NIC 를 link-local 로 잡으면 붙는다.
    """
    cfg = group.get("discovery", {})
    port = int(cfg.get("port") or 80)
    timeout = float(cfg.get("timeout_s") or 3.0)

    devices, seen = [], set()
    for host in _ouster_hosts(group):
        if not _tcp_open(host, port, timeout):
            continue
        serial, model, firmware = _ouster_metadata(host, timeout)
        devices.append(_ouster_device(group, host, _nic_for_ip(host) or net_tools.route_dev(host),
                                      serial, model, OK,
                                      f"펌웨어 {firmware}" if firmware else f"TCP {port} 응답"))
        seen.add(serial or host)

    by_serial = {}
    for entry in _ouster_mdns(timeout):
        by_serial.setdefault(entry["serial"] or entry["host"], []).append(entry)
    for serial, entries in by_serial.items():
        if serial in seen:
            continue
        iface = entries[0]["iface"]
        v4 = [e["address"] for e in entries if e["proto"] == "IPv4"]
        v6 = [e for e in entries if e["proto"] == "IPv6"]
        if not v4 and v6:                      # PC 쪽이 IPv6 만 있으면 센서 API 로 IPv4 설정을 묻는다
            addr = _ouster_ipv4_via_ipv6(v6[0]["address"], v6[0]["iface"], timeout)
            v4 = [addr] if addr else []
        fw = entries[0]["fw"]
        reachable = next((ip for ip in v4 if _tcp_open(ip, port, timeout)), None)
        if reachable:
            devices.append(_ouster_device(group, reachable, iface, serial, "", OK,
                                          f"mDNS 로 찾음 · {fw}".strip(" ·")))
            continue
        ip = v4[0] if v4 else (v6[0]["address"] if v6 else "")
        if net_tools.nic_ipv4(iface):
            # NIC 에 IPv4 는 있는데 못 닿는다 — 그 주소로 가는 경로가 다른 NIC 로 나 있는 경우
            via = net_tools.route_dev(ip) if v4 else ""
            note = (f"{ip} 로 가는 경로가 {via} 로 나 있어 못 붙음 (라이다는 {iface})" if via and via != iface
                    else f"{iface} 에서 보이지만 TCP {port} 응답 없음")
            devices.append(_ouster_device(group, ip, iface, serial, "", NIC, note))
        else:
            dev = _ouster_device(group, ip, iface, serial, "", NIC,
                                 f"PC 의 {iface} 에 IPv4 가 없어 못 붙음 — [NIC 설정]")
            dev["fix_nic"] = iface
            devices.append(dev)
    return devices


def _ouster_device(group, ip, nic, serial, model, state, note):
    return {"group": group["key"], "subset": None, "subset_label": group["label"],
            "nic": nic, "ip": ip, "identity": serial, "model": model or "Ouster",
            "vendor": "Ouster", "mac": "", "state": state, "note": note}


def _ouster_mdns(timeout):
    """avahi 로 Ouster 가 알리는 _roger._tcp 를 모은다 → [{iface, proto, host, address, serial, fw}].

    avahi-browse 가 없거나 느리면 빈 목록 (TCP 프로브만으로 동작한다).
    """
    exe = shutil.which("avahi-browse")
    if not exe:
        return []
    try:
        out = subprocess.run([exe, "-rpt", "_roger._tcp"], capture_output=True,
                             timeout=max(2.0, timeout)).stdout
    except subprocess.TimeoutExpired as exc:
        out = exc.stdout or b""
    except OSError:
        return []
    found = []
    for line in out.decode("utf-8", "replace").splitlines():
        parts = line.split(";")
        if len(parts) < 10 or parts[0] != "=":
            continue
        txt = dict(re.findall(r'"(\w+)=([^"]*)"', parts[9]))
        found.append({"iface": parts[1], "proto": parts[2], "host": parts[6],
                      "address": parts[7], "serial": txt.get("sn", ""),
                      "fw": txt.get("fw", "").split("+")[0]})
    return found


def _ouster_ipv4_via_ipv6(address, iface, timeout):
    """IPv6 link-local 로 센서 API(/api/v1/system/network)에 물어 센서의 IPv4 주소를 얻는다."""
    try:
        scope = socket.if_nametoindex(iface)
        with socket.socket(socket.AF_INET6, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            sock.connect((address, 80, 0, scope))
            sock.sendall(b"GET /api/v1/system/network HTTP/1.0\r\nHost: sensor\r\n\r\n")
            data = b""
            while len(data) < 65536:
                chunk = sock.recv(8192)
                if not chunk:
                    break
                data += chunk
        doc = json.loads(data.split(b"\r\n\r\n", 1)[-1].decode("utf-8", "replace"))
        return str((doc.get("ipv4") or {}).get("addr") or "").split("/")[0]
    except (OSError, ValueError, AttributeError):
        return ""


def link_local_candidates():
    """링크는 올라왔는데 IPv4 가 없는 유선 NIC — link-local 장비(라이다)를 새로 꽂았을 수 있다.

    NetworkManager 가 DHCP 를 기다리다 실패하고 끊는 동안엔 IPv6 도 없어서 mDNS 도 안 들린다.
    그래서 장비가 안 보여도 "이 NIC 일 수 있다" 를 알려 줄 수 있게 따로 모은다.
    """
    out = []
    for iface, speed in net_tools.nic_list():
        if (Path("/sys/class/net") / iface / "wireless").exists():
            continue
        if not net_tools.nic_carrier(iface) or net_tools.nic_ipv4(iface):
            continue
        out.append({"nic": iface, "usb": net_tools.nic_is_usb(iface), "speed": speed})
    return out


def _ouster_hosts(group):
    cfg = group.get("discovery", {})
    hosts = list(cfg.get("hosts") or [])
    _, params = load_params(group.get("params"))
    if params.get("sensor_hostname"):
        hosts.append(str(params["sensor_hostname"]))
    return [h for i, h in enumerate(hosts) if h and h not in hosts[:i]]


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

    own = net_tools.own_ipv4s()
    return [{
        "group": group["key"], "subset": None, "subset_label": group["label"],
        "nic": _nic_for_ip(ip), "ip": ip, "identity": "", "model": "",
        "vendor": "OxTS", "mac": "", "state": OK,
        "note": f"NCOM {count}패킷/{timeout:g}초" + (" (이 PC)" if ip in own else ""),
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


def probe_description(group):
    """무엇을 찔러봤는지 — 0대일 때 "어디를 봤는데 없었다"를 보여주려는 것."""
    cfg = group.get("discovery") or {}
    kind = cfg.get("kind")
    if kind == "gvcp":
        nics = cfg.get("nics") or wired_nics()
        return "GVCP 브로드캐스트: " + (", ".join(nics) or "UP 상태 유선 NIC 없음")
    if kind == "ouster_probe":
        hosts = _ouster_hosts(group)
        return (f"TCP {cfg.get('port') or 80}: " + (", ".join(hosts) or "주소 없음 (sensor_hostname 비어 있음)")
                + " · mDNS _roger._tcp (모든 NIC)")
    if kind == "ncom_listen":
        return f"UDP {cfg.get('port') or 3000} NCOM 수신 {cfg.get('timeout_s') or 2.0:g}초"
    return kind or "감지 방법 없음"


def discover_group(group):
    """센서군 하나를 감지 -> {"devices": [...], "probe": 설명, "error": str|None}.

    어떤 예외도 밖으로 내보내지 않는다 (GUI가 죽으면 안 된다).
    """
    result = {"devices": [], "probe": "", "error": None}
    try:
        result["probe"] = probe_description(group)
    except Exception:                                         # noqa: BLE001
        pass
    kind = (group.get("discovery") or {}).get("kind")
    handler = _KINDS.get(kind)
    if handler is None:
        return result
    try:
        result["devices"] = handler(group)
    except Exception as exc:                                  # noqa: BLE001
        result["error"] = f"감지 실패: {exc}"
    if kind == "ouster_probe" and not any(d["state"] == OK for d in result["devices"]):
        # 라이다가 안 보이면 "IPv4 없는 유선 NIC" 를 후보로 — 거기 꽂았을 수 있다
        try:
            used = {d["nic"] for d in result["devices"]}
            result["candidates"] = [c for c in link_local_candidates() if c["nic"] not in used]
        except Exception:                                     # noqa: BLE001
            result["candidates"] = []
    return result


def discover_all(registry=None):
    """{group_key: discover_group() 결과}"""
    registry = registry or load_registry()
    return {g["key"]: discover_group(g) for g in registry.get("groups", [])}


def count_by_subset(group, devices):
    """[(라벨, 보이는 대수, 비고)].

    가시광 Blackfly 와 열화상 A70 은 따로 센다. subset 이 선언돼 있으면 0대여도 줄을
    낸다 — "A70 2대" 만 보이고 Blackfly 줄이 아예 없으면 안 본 건지 없는 건지 모른다.
    """
    subs = group.get("subsets") or []
    buckets = [(sub["key"], sub["label"]) for sub in subs] or [(None, group.get("label", ""))]
    out = []
    for key, label in buckets:
        mine = [d for d in devices if not subs or d["subset"] == key]
        out.append((label, len(mine), _count_note(mine)))
    # 모델 패턴에 안 걸린 GigE 장비 (예: 다른 회사 카메라)
    stray = [d for d in devices if subs and d["subset"] is None]
    if stray:
        out.append(("미분류 GigE", len(stray), _count_note(stray)))
    return out


def _count_note(devices):
    notes = []
    bad = sum(1 for d in devices if d["state"] == SUBNET)
    upd = sum(1 for d in devices if d["state"] == UPDATER)
    if bad:
        notes.append(f"{bad}대 IP 안 맞음")
    if upd:
        notes.append(f"{upd}대 업데이터 모드")
    nic = sum(1 for d in devices if d["state"] == NIC)
    if nic:
        notes.append(f"{nic}대 PC NIC 설정 필요")
    return ", ".join(notes)


def count_text(group, devices):
    """'열화상 A70 2대 · 가시광 Blackfly 0대' 한 줄."""
    return "  ·  ".join(f"{label} {count}대" + (f" ({note})" if note else "")
                        for label, count, note in count_by_subset(group, devices))


STATE_MARK = {OK: "OK", SUBNET: "!!", UPDATER: "UP", NIC: "NI"}


def main(argv):
    registry = load_registry()
    if not registry.get("groups"):
        print("sensors.yaml을 찾지 못했습니다:", [str(c) for c in REGISTRY_CANDIDATES])
        return 1
    wanted = set(argv[1:])
    for group in registry["groups"]:
        if wanted and group["key"] not in wanted:
            continue
        result = discover_group(group)
        devices = result["devices"]
        print(f"\n=== {group['label']} ({group['key']}) — {count_text(group, devices)}")
        print(f"    감지: {result['probe']}")
        if result["error"]:
            print(f"    {result['error']}")
        for d in devices:
            print(f"  [{STATE_MARK.get(d['state'], '?')}] {d['subset_label']:16s} "
                  f"{d['ip']:16s} {d['identity']:12s} {d['model']:20s} "
                  f"{d['nic']:10s} {d['note']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
