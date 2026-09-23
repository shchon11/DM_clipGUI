#!/usr/bin/env python3
# gpio_watch.py — Blackfly S 카메라 GPIO 핀에 지금 어떤 신호가 들어오는지 터미널에서 본다.
#
# 카메라의 LineStatusAll 레지스터(0x000D2064, 라인마다 1비트 — 지금 전압이 HIGH 면 1)를 GigE Vision 제어
# 프로토콜(GVCP READREG, UDP 3956)로 0.7 ms 간격쯤 계속 읽는다. 읽기만 해서 카메라 설정 · 영상에 영향이 없고,
# 노드(flir_spinnaker_camera_node)가 카메라를 잡고 있어도 읽힌다. 카메라마다 스레드 하나라 여러 대를 같이 재도
# 간격이 벌어지지 않는다 — 1 ms 짜리 트리거 펄스도 놓치지 않는다.
#
# 핀 배치 (FLIR Blackfly S GigE 6핀 Hirose HR10, BFS-PGE-51S5 "Input/Output Control" 문서의 표 — 원문 HTML 로 확인.
# 표의 핀 번호 옆 각주(1², 3²)를 핀 번호로 읽으면 틀린다):
#   핀 1 초록 Line3 비절연 입력 (GPI, HIGH 2.6–3.6 V) · 보조 전원 입력(VAUX 8–24 V) 겸용
#   핀 2 검정 Line0 옵토 절연 입력 (HIGH 2.6–30 V, 3.5–7 mA)
#   핀 3 빨강 Line2 비절연 입출력 (입력 HIGH 2.6–24 V, 출력은 오픈 드레인) · 3.3 V 출력(VOUT, V3_3Enable) 겸용
#   핀 4 흰색 Line1 옵토 절연 출력 (오픈 드레인, 풀업 필요)
#   핀 5 파랑 옵토 GND · 핀 6 갈색 카메라 GND
# 비트 n = Line n 은 카메라 XML 의 LineStatusAll 설명("Line 0 status corresponds to bit 0 …")과
# LineStatus = (LineStatusAll >> LineSelector) & 1 공식으로 확인했다.
#
# 사용:
#   python3 gpio_watch.py                       카메라 NIC 에서 Blackfly 를 찾아 전부 2초씩
#   python3 gpio_watch.py 192.168.1.1           한 대만
#   python3 gpio_watch.py 192.168.1.1 -w        한 대를 계속 (선을 옮기며 볼 때, Ctrl+C 로 끝)
#   python3 gpio_watch.py -t 5 -i enp3s0f1      5초씩 · NIC 지정
#   python3 gpio_watch.py --scope [IP ...]      실시간 파형 창 (gpio_scope.py)

import argparse
import socket
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import net_tools  # noqa: E402

LINE_STATUS_ALL = 0x000D2064
LINES = [("Line0", "핀2 검정 옵토입력"), ("Line1", "핀4 흰색 옵토출력"), ("Line2", "핀3 빨강 입출력/3.3V"),
         ("Line3", "핀1 초록 입력")]
INPUTS = (0, 2, 3)                   # 밖에서 신호를 받을 수 있는 라인 (Line2 는 3.3V 출력을 끈 입력 모드일 때)


def read_line_status(sock, ip, req):
    """(상태, 값). 응답이 없으면 socket.timeout."""
    sock.sendto(struct.pack(">BBHHHI", 0x42, 0x01, 0x0080, 4, req, LINE_STATUS_ALL), (ip, net_tools.GVCP_PORT))
    while True:
        data, _ = sock.recvfrom(64)
        if len(data) < 8:
            continue
        status, _ack, _length, ack_id = struct.unpack(">HHHH", data[:8])
        if ack_id == req:
            return status, (struct.unpack(">I", data[8:12])[0] if status == 0 and len(data) >= 12 else None)


def sample(ip, seconds):
    """[(시각, 값)] 과 오류 글 — seconds 동안 쉬지 않고 읽는다."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.05)
    out, err, req, t0 = [], None, 0, time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            req = req % 0xFFFF + 1
            try:
                status, value = read_line_status(sock, ip, req)
            except socket.timeout:
                err = "응답 없음"
                continue
            if status != 0:
                return out, f"GVCP 상태 0x{status:04x}"
            out.append((time.monotonic(), value))
    finally:
        sock.close()
    return out, (err if not out else None)


def analyse(samples, bit):
    """한 라인 -> {high 비율, 바뀐 횟수, 떨어진 엣지 Hz, LOW 폭 ms, 올라간 엣지 Hz, HIGH 폭 ms}."""
    values = [(v >> bit) & 1 for _, v in samples]
    times = [t for t, _ in samples]
    span = times[-1] - times[0] if len(times) > 1 else 0.0
    flips = sum(1 for a, b in zip(values, values[1:]) if a != b)
    runs = {0: [], 1: []}                # 끝까지 이어진 구간만 (양쪽이 잘린 첫 · 마지막 구간은 뺀다)
    start = None
    for i in range(1, len(values)):
        if values[i] != values[i - 1]:
            if start is not None:
                runs[values[i - 1]].append(times[i] - times[start])
            start = i
    falls = [times[i] for i in range(1, len(values)) if values[i - 1] == 1 and values[i] == 0]
    rises = [times[i] for i in range(1, len(values)) if values[i - 1] == 0 and values[i] == 1]

    def median(x):
        return sorted(x)[len(x) // 2] * 1000 if x else None

    def rate(edges):                     # 엣지 간격의 중앙값으로 — 재는 구간 양 끝에 잘린 주기에 안 흔들린다
        if len(edges) > 1:
            return 1000.0 / median([b - a for a, b in zip(edges, edges[1:])])
        return len(edges) / span if span else 0.0
    return {"high": sum(values) / len(values), "flips": flips, "span": span,
            "fall_hz": rate(falls), "rise_hz": rate(rises),
            "low_ms": median(runs[0]), "high_ms": median(runs[1])}


def verdict(lines):
    """입력 라인(0, 3)을 보고 한 줄 판정."""
    parts = []
    for bit in INPUTS:
        a = lines[bit]
        name, pin = LINES[bit]
        if a["flips"]:
            if a["high"] >= 0.5:         # 평소 HIGH, 잠깐 LOW -> LOW 펄스 (떨어지는 순간이 펄스 시작)
                width = f", LOW 폭 ≈{a['low_ms']:.1f} ms" if a["low_ms"] else ""
                parts.append(f"{name}({pin}) ← {a['fall_hz']:.1f} Hz LOW 펄스{width} → 엣지 FallingEdge")
            else:
                width = f", HIGH 폭 ≈{a['high_ms']:.1f} ms" if a["high_ms"] else ""
                parts.append(f"{name}({pin}) ← {a['rise_hz']:.1f} Hz HIGH 펄스{width} → 엣지 RisingEdge")
    if parts:
        return "  ✓ " + " · ".join(parts)
    why = ["Line0 늘 0 = 옵토 입력(핀2→핀5)에 전류가 안 흐름" if lines[0]["high"] < 0.5 else
           "Line0 늘 1 = 옵토 입력에 전류가 계속 흐름"]
    for bit, pin in ((2, "핀3"), (3, "핀1")):
        why.append(f"Line{bit} 늘 1 = {pin} 을 아무것도 GND 로 안 끌어내림" if lines[bit]["high"] >= 0.5 else
                   f"Line{bit} 늘 0 = {pin} 이 GND 쪽에 붙어 있음")
    return "  ✗ 펄스 없음 — " + " · ".join(why)


def names_by_serial():
    """시리얼 -> GUI 에서 쓰는 카메라 이름. 못 읽으면 {} (시리얼로 보여 준다)."""
    try:
        import yaml
        import sensor_config
        import sensor_discovery
        session = yaml.safe_load((Path.home() / ".config/dm_clip_gui/last_session.yaml").read_text()) or {}
        out = {}
        for group in sensor_discovery.load_registry().get("groups", []):
            overrides = (session.get("sensors") or {}).get(group["key"]) or {}
            for sub in sensor_config.subsets_of(group):
                if not sub.get("inventory"):
                    continue
                repo = sensor_config.repo_inventory(sub)
                for serial in set(repo) | set(sensor_config.camera_overrides(dict(overrides))):
                    out[serial] = sensor_config.camera_entry(sub, serial, dict(overrides), repo)["namespace"]
        return out
    except Exception:            # 이름은 보기 좋으라고 붙이는 것 — 없어도 잰다
        return {}


def find_cameras(iface):
    found = net_tools.gvcp_discover_all(iface, 1.0)
    cams = [d for d in found if "BFS" in d["model"] or "Blackfly" in d["model"]]
    return sorted(cams, key=lambda d: tuple(int(x) for x in d["ip"].split(".")))


def run_once(targets, seconds, names):
    results = {}

    def work(ip):
        results[ip] = sample(ip, seconds)
    threads = [threading.Thread(target=work, args=(ip,), daemon=True) for ip, _ in targets]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    head = "  ".join(f"{n}({p})" for n, p in LINES)
    print(f"{'카메라':22s} {'IP':15s}  {head}")
    print(f"{'':22s} {'':15s}  (각 칸: HIGH 비율 · 바뀐 횟수)")
    for ip, serial in targets:
        label = names.get(serial, serial or ip)
        samples, err = results.get(ip, ([], "안 잼"))
        if not samples:
            print(f"{label:22s} {ip:15s}  ✗ {err}")
            continue
        lines = [analyse(samples, bit) for bit in range(4)]
        span = lines[0]["span"] or seconds
        cells = "  ".join(f"{a['high']:>8.2f} · {a['flips']:<5d}" for a in lines)
        print(f"{label:22s} {ip:15s}  {cells}  ({len(samples)}회, {span / max(len(samples) - 1, 1) * 1000:.1f} ms 간격)")
        print(f"{'':22s} {'':15s}{verdict(lines)}")


def main():
    parser = argparse.ArgumentParser(description="Blackfly S GPIO 핀 신호 보기 (GVCP 로 LineStatusAll 읽기, 읽기 전용)")
    parser.add_argument("ips", nargs="*", help="카메라 IP (없으면 NIC 에서 Blackfly 를 찾는다)")
    parser.add_argument("-t", "--seconds", type=float, default=2.0, help="카메라마다 잴 시간 (초, 기본 2)")
    parser.add_argument("-i", "--iface", default=None, help="카메라 NIC (기본: 가장 빠른 NIC)")
    parser.add_argument("-w", "--watch", action="store_true", help="계속 반복 (Ctrl+C 로 끝)")
    parser.add_argument("-s", "--scope", action="store_true", help="실시간 파형 창으로 보기 (gpio_scope.py)")
    args = parser.parse_args()
    if args.scope:
        import gpio_scope
        targets = gpio_scope.resolve_targets(args.ips, args.iface)
        if not targets:
            print("Blackfly 카메라를 못 찾았습니다 (-i 로 NIC 지정, 또는 IP 를 직접)")
            return 1
        return gpio_scope.run(targets)

    names = names_by_serial()
    if args.ips:
        serial_of = {}
        iface = args.iface or net_tools.route_dev(args.ips[0]) or net_tools.auto_nic()
        if iface:
            serial_of = {d["ip"]: d["serial"] for d in net_tools.gvcp_discover_all(iface, 0.5)}
        targets = [(ip, serial_of.get(ip, "")) for ip in args.ips]
    else:
        iface = args.iface or net_tools.auto_nic()
        if not iface:
            print("카메라 NIC 를 못 찾았습니다 (-i 로 지정)")
            return 1
        targets = [(d["ip"], d["serial"]) for d in find_cameras(iface)]
        if not targets:
            print(f"{iface} 에서 Blackfly 카메라를 못 찾았습니다 (-i 로 NIC 지정, 또는 IP 를 직접)")
            return 1
    print(f"{len(targets)}대 · {args.seconds:g}초씩 · 읽기 전용 (GVCP READREG 0x{LINE_STATUS_ALL:08X})\n")
    try:
        while True:
            run_once(targets, args.seconds, names)
            if not args.watch:
                break
            print(f"\n--- {time.strftime('%H:%M:%S')} ---")
    except KeyboardInterrupt:
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
