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


def _broadcast(ip, plen):
    n = struct.unpack(">I", socket.inet_aton(ip))[0]
    mask = (0xFFFFFFFF << (32 - plen)) & 0xFFFFFFFF
    return socket.inet_ntoa(struct.pack(">I", n | (~mask & 0xFFFFFFFF)))


# ---------- GigE Vision 디스커버리 ----------

def gvcp_discover(iface, timeout=1.0):
    """{ip: {'vendor','model','serial','user_name','mac'}}
    GVCP DISCOVERY_CMD를 서브넷 브로드캐스트로 보내고 ACK를 모은다. root 불필요."""
    addr = nic_ipv4(iface)
    if not addr:
        return {}
    local_ip, plen = addr
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.settimeout(timeout)
    found = {}
    try:
        s.bind((local_ip, 0))
        # key 0x42, flags 0x11 (ack 요구 + 브로드캐스트 ack 허용), DISCOVERY_CMD, len 0, id 1
        s.sendto(struct.pack(">BBHHH", 0x42, 0x11, 0x0002, 0, 1),
                 (_broadcast(local_ip, plen), 3956))
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
            ip = socket.inet_ntoa(b[36:40])
            found[ip] = {"vendor": cstr(72, 32), "model": cstr(104, 32),
                         "serial": cstr(216, 16), "user_name": cstr(232, 16),
                         "mac": mac, "src": src[0]}
    except OSError:
        pass
    finally:
        s.close()
    return found


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
    for ip, d in sorted(gvcp_discover(iface).items()):
        print(f"  {ip:15s} {d['model']:28s} serial={d['serial']:10s} mac={d['mac']}")
