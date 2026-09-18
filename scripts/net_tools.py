#!/usr/bin/env python3
# net_tools.py — 네트워크 관측 유틸 (root 불필요한 부분).
#   - NIC 링크 속도/누적 바이트 (sysfs)
#   - GigE Vision 디스커버리 (GVCP DISCOVERY_CMD 브로드캐스트) → IP↔serial/model
#   - net_probe 도우미 바이너리 위치 찾기
#
# 단독 실행: python3 net_tools.py [iface]  → NIC 요약 + 카메라 디스커버리 출력

import os
import shutil
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

SYS_NET = Path("/sys/class/net")


# ---------- NIC ----------

def nic_list():
    """UP 상태의 물리 인터페이스 목록: [(iface, speed_mbps)]"""
    out = []
    for d in sorted(SYS_NET.iterdir()):
        name = d.name
        if name == "lo" or not (d / "device").exists():
            continue
        try:
            if (d / "operstate").read_text().strip() != "up":
                continue
            speed = int((d / "speed").read_text().strip() or -1)
        except (OSError, ValueError):
            speed = -1
        out.append((name, speed))
    return out


def auto_nic():
    """가장 빠른 UP 인터페이스 (카메라 업링크는 보통 최고속 포트)."""
    cands = nic_list()
    if not cands:
        return None
    return max(cands, key=lambda x: x[1])[0]


def nic_stats(iface):
    """(rx_bytes, tx_bytes, speed_mbps, mtu). 실패 시 None."""
    d = SYS_NET / iface
    try:
        rx = int((d / "statistics/rx_bytes").read_text())
        tx = int((d / "statistics/tx_bytes").read_text())
        speed = int((d / "speed").read_text().strip() or -1)
        mtu = int((d / "mtu").read_text())
        return rx, tx, speed, mtu
    except (OSError, ValueError):
        return None


def nic_ipv4(iface):
    """(ip, prefix_len) 또는 None"""
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr", "show", "dev", iface],
                             capture_output=True, text=True, timeout=3).stdout
        for tok in out.split():
            if "/" in tok and tok[0].isdigit():
                ip, plen = tok.split("/")
                return ip, int(plen)
    except (subprocess.SubprocessError, ValueError):
        pass
    return None


def nic_is_usb(iface):
    """USB 어댑터인가 (sysfs 장치 경로에 usb 가 끼어 있다)."""
    try:
        return "/usb" in os.path.realpath(SYS_NET / iface / "device")
    except OSError:
        return False


def nic_carrier(iface):
    """케이블이 꽂혀 링크가 올라왔는가."""
    try:
        return (SYS_NET / iface / "carrier").read_text().strip() == "1"
    except OSError:
        return False


# ---------- NetworkManager: link-local 장비용 NIC ----------

NM_LINK_LOCAL_PREFIX = "LiDAR link-local"


def nm_link_local(iface, timeout=30.0):
    """NIC 를 link-local(IPv4 169.254.x.x + IPv6 fe80::) 로 잡는 NetworkManager 프로필을 만들고 올린다.

    Ouster 같은 link-local 장비를 새 NIC(USB 이더넷 어댑터 등)에 꽂으면, NetworkManager 가 그 NIC 에
    기본 DHCP 프로필을 걸어 45초 기다리다 실패하고 연결을 끊는다 (IPv6 주소까지 사라진다). 그걸
    되풀이하니 PC 가 장비에 영영 못 붙는다. 이 프로필은 그 NIC 이름에 묶이고 autoconnect-priority 가
    기본 프로필(0)보다 높아서, 다음에 꽂을 때부터는 NetworkManager 가 알아서 이걸 쓴다.
    기본 경로는 만들지 않는다 (never-default). 되돌리기: nmcli connection delete '<프로필 이름>'.

    권한: 데스크톱 세션 사용자는 polkit 으로 허용된다 (sudo 불필요).
    반환 (ok, 메시지, 잡힌 IPv4|None)
    """
    nmcli = shutil.which("nmcli")
    if not nmcli:
        return False, "nmcli 가 없어 NIC 를 설정할 수 없습니다", None
    env = dict(os.environ, LC_ALL="C")
    name = f"{NM_LINK_LOCAL_PREFIX} ({iface})"

    def run(args, limit=15.0):
        return subprocess.run([nmcli] + args, capture_output=True, text=True, timeout=limit, env=env)

    settings = ["ipv4.method", "link-local", "ipv6.method", "link-local",
                "ipv4.never-default", "yes", "ipv6.never-default", "yes",
                "connection.autoconnect", "yes", "connection.autoconnect-priority", "50"]
    try:
        names = run(["-t", "-f", "NAME", "connection", "show"]).stdout.splitlines()
        if name in names:
            res = run(["connection", "modify", name, "connection.interface-name", iface] + settings)
        else:
            res = run(["connection", "add", "type", "ethernet", "con-name", name, "ifname", iface]
                      + settings)
        if res.returncode:
            return False, f"프로필 만들기 실패: {(res.stderr or res.stdout).strip()}", None
        res = run(["--wait", str(int(timeout)), "connection", "up", name, "ifname", iface],
                  limit=timeout + 10)
        if res.returncode:
            return False, f"연결 실패: {(res.stderr or res.stdout).strip()}", None
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"nmcli 실행 실패: {exc}", None
    deadline = time.time() + 15
    while time.time() < deadline:
        addr = nic_ipv4(iface)
        if addr and addr[0].startswith("169.254."):
            return True, f"{iface} ← {addr[0]}/{addr[1]} (link-local, 프로필 '{name}')", addr[0]
        time.sleep(0.5)
    return False, f"{iface} 에 link-local 주소가 안 잡혔습니다 (프로필 '{name}')", None


def route_dev(ip):
    """그 주소로 나가는 NIC 이름 (ip route get). 모르면 ''."""
    try:
        out = subprocess.run(["ip", "-o", "route", "get", ip], capture_output=True, text=True,
                             timeout=3).stdout.split()
        return out[out.index("dev") + 1] if "dev" in out else ""
    except (OSError, subprocess.SubprocessError, IndexError):
        return ""


def _broadcast(ip, plen):
    n = struct.unpack(">I", socket.inet_aton(ip))[0]
    mask = (0xFFFFFFFF << (32 - plen)) & 0xFFFFFFFF
    return socket.inet_ntoa(struct.pack(">I", n | (~mask & 0xFFFFFFFF)))


# ---------- GigE Vision 디스커버리 ----------

GVCP_PORT = 3956


def _gvcp_socket(iface, timeout):
    """(소켓, 보낼 주소들). NIC 에 묶은 브로드캐스트 소켓.

    제한 브로드캐스트(255.255.255.255)로 보내야 IP 가 아직 없는 새 카메라(링크로컬 169.254.x.x)도
    듣는다 — 서브넷 브로드캐스트(192.168.1.255)만 쓰면 다른 서브넷 장비는 못 듣는다 (Spinnaker 도
    제한 브로드캐스트로 찾는다). 그런 장비는 ACK 도 브로드캐스트로 돌려주므로 소켓은 INADDR_ANY 에
    묶고 SO_BINDTODEVICE 로 NIC 를 고른다 (리눅스 5.7+ 는 root 불필요). 안 되면 예전 방식.
    """
    addr = nic_ipv4(iface)
    if not addr:
        return None, []
    local_ip, plen = addr
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.settimeout(timeout)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface.encode())
        s.bind(("", 0))
        return s, ["255.255.255.255"]
    except OSError:
        s.bind((local_ip, 0))
        return s, [_broadcast(local_ip, plen)]


def gvcp_discover_all(iface, timeout=1.0):
    """[{ip, vendor, model, serial, user_name, mac, src}] — 장비마다 한 줄 (MAC 기준).

    IP 로 묶지 않는다: 두 카메라가 같은 IP 를 쓰면(ForceIP 충돌 등) 둘 다 보여야 한다.
    """
    s, targets = _gvcp_socket(iface, timeout)
    if s is None:
        return []
    found = {}
    try:
        # key 0x42, flags 0x11 (ack 요구 + 브로드캐스트 ack 허용), DISCOVERY_CMD, len 0, id 1
        for target in targets:
            s.sendto(struct.pack(">BBHHH", 0x42, 0x11, 0x0002, 0, 1), (target, GVCP_PORT))
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                data, src = s.recvfrom(1024)
            except socket.timeout:
                break
            # DISCOVERY_ACK = 0x0003, payload 248바이트
            if len(data) < 8 + 248 or struct.unpack(">H", data[2:4])[0] != 3:
                continue
            b = data[8:]

            def cstr(off, n):
                return b[off:off + n].split(b"\0")[0].decode(errors="replace")
            mac = ":".join(f"{x:02x}" for x in b[10:16])
            found[mac] = {"ip": socket.inet_ntoa(b[36:40]), "vendor": cstr(72, 32),
                          "model": cstr(104, 32), "serial": cstr(216, 16),
                          "user_name": cstr(232, 16), "mac": mac, "src": src[0]}
    except OSError:
        pass
    finally:
        s.close()
    return list(found.values())


def gvcp_discover(iface, timeout=1.0):
    """{ip: {'vendor','model','serial','user_name','mac'}} — IP 로 찾아보는 쪽(네트워크 표)용."""
    return {d["ip"]: d for d in gvcp_discover_all(iface, timeout)}


def gvcp_force_ip(iface, mac, ip, mask="255.255.255.0", gateway="0.0.0.0", timeout=1.0):
    """GVCP FORCEIP_CMD — MAC 으로 고른 카메라에 임시 IP 를 준다 (전원을 다시 넣으면 풀린다).

    지금 IP 가 무엇이든(다른 서브넷·링크로컬이어도) 브로드캐스트로 닿는다. Spinnaker 의 ForceIP 와
    같은 명령이다. ACK 를 받으면 True — 그래도 호출하는 쪽에서 다시 찾아 확인하는 게 맞다.
    """
    s, targets = _gvcp_socket(iface, timeout)
    if s is None:
        return False
    mac_b = bytes(int(x, 16) for x in mac.split(":"))
    payload = (b"\0\0" + mac_b + b"\0" * 12 + socket.inet_aton(ip) + b"\0" * 12 +
               socket.inet_aton(mask) + b"\0" * 12 + socket.inet_aton(gateway))
    req_id = 0x4242
    try:
        for target in targets:
            s.sendto(struct.pack(">BBHHH", 0x42, 0x01, 0x0004, len(payload), req_id) + payload,
                     (target, GVCP_PORT))
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                data, _src = s.recvfrom(1024)
            except socket.timeout:
                break
            # FORCEIP_ACK = 0x0005, ack_id 가 요청 id 와 같아야 한다
            if len(data) >= 8 and struct.unpack(">HHHH", data[:8])[1] == 0x0005 and \
                    struct.unpack(">H", data[6:8])[0] == req_id:
                return True
    except OSError:
        pass
    finally:
        s.close()
    return False


# ---------- 플로우 힌트 → 정체 추정 ----------

# 목적지 포트로 장치/프로토콜을 추정. (proto, port) 또는 port만.
PORT_NAMES = {
    (17, 3000): "OxTS NCOM (RT2000/3000)", (17, 3956): "GigE Vision 제어(GVCP)",
    (17, 319): "PTP 이벤트", (17, 320): "PTP 일반",
    (17, 7502): "Ouster LiDAR 데이터", (17, 7503): "Ouster IMU",
    (17, 2368): "Velodyne LiDAR 데이터", (17, 8308): "Velodyne 위치",
    (17, 6699): "Livox/Hesai LiDAR", (17, 2369): "Hesai GPS",
    (17, 5353): "mDNS", (17, 1900): "SSDP", (17, 137): "NetBIOS", (17, 138): "NetBIOS",
    (17, 67): "DHCP", (17, 68): "DHCP", (17, 123): "NTP", (17, 53): "DNS",
    (6, 22): "SSH", (6, 80): "HTTP", (6, 443): "HTTPS", (6, 5900): "VNC",
}
PROTO_NAMES = {6: "TCP", 17: "UDP", 1: "ICMP", 2: "IGMP"}


def describe_flows(flows):
    """net_probe 힌트 [[proto, dport, kind, pkts], ...] → 사람이 읽는 짧은 문자열."""
    if not flows:
        return ""
    parts = []
    for proto, dport, kind, pkts in sorted(flows, key=lambda f: -f[3])[:2]:
        name = PORT_NAMES.get((proto, dport))          # 알려진 포트가 최우선
        if not name and proto == 17 and 7400 <= dport <= 7500:
            name = "DDS(ROS 2) 디스커버리/데이터"
        elif not name and proto == 17 and dport > 1024 and pkts > 1000 and kind == 0:
            name = "고속 UDP 스트림(GVSP 등)"
        if not name:
            name = f"{PROTO_NAMES.get(proto, 'proto' + str(proto))}" + (f"/{dport}" if dport else "")
        txt = name + {1: " 브로드캐스트", 2: " 멀티캐스트"}.get(kind, "")
        if txt not in parts:
            parts.append(txt)
    return ", ".join(parts)


def own_ipv4s():
    """이 PC의 모든 IPv4 주소 (lo 포함)."""
    out = set()
    try:
        txt = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True,
                             text=True, timeout=3).stdout
        for tok in txt.split():
            if "/" in tok and tok[0].isdigit():
                out.add(tok.split("/")[0])
    except subprocess.SubprocessError:
        pass
    return out


# ---------- net_probe 도우미 ----------

def find_net_probe():
    """설치본(lib/clip_recorder) → 빌드 트리 → PATH 순으로 탐색."""
    here = Path(__file__).resolve().parent
    for cand in (here / "net_probe",
                 here.parent.parent.parent / "build" / "clip_recorder" / "net_probe",
                 Path.home() / "DM_clipGUI" / "build" / "clip_recorder" / "net_probe"):
        if cand.is_file() and os.access(cand, os.X_OK):
            return os.path.realpath(cand)      # setcap/getcap은 실제 파일에
    return shutil.which("net_probe")


def net_probe_has_cap(path):
    """getcap으로 cap_net_raw 부여 여부 확인 (getcap 없으면 None)."""
    if not path or not shutil.which("getcap"):
        return None
    out = subprocess.run(["getcap", path], capture_output=True, text=True).stdout
    return "cap_net_raw" in out


if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else auto_nic()
    print("NICs:", nic_list())
    print("선택:", iface, nic_stats(iface), nic_ipv4(iface))
    print("net_probe:", find_net_probe(), "cap:", net_probe_has_cap(find_net_probe()))
    for d in sorted(gvcp_discover_all(iface), key=lambda d: d["ip"]):
        print(f"  {d['ip']:15s} {d['model']:28s} serial={d['serial']:10s} mac={d['mac']}")
