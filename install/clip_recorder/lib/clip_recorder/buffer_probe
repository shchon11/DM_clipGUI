#!/usr/bin/env python3
# buffer_probe.py — measure how much ring-buffer memory clip_recorder needs.
#
# Listens to the recorder's status on /diagnostics for a while, then prints
# average / peak inflow, a recommended max_buffer_mb, and the heaviest topics.
#
#   ros2 run clip_recorder buffer_probe                 # 60 s
#   ros2 run clip_recorder buffer_probe --duration 300  # longer = catches peaks
#   ros2 run clip_recorder buffer_probe --node my_rec   # non-default node name
#
# Requires the recorder to be running with status_period_sec > 0.

import argparse
import math
import sys
import time
from collections import defaultdict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node

MARGIN = 1.3  # headroom on top of the peak requirement


def mem_total_mb():
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return None


class Probe(Node):
    def __init__(self, target, top):
        super().__init__("buffer_probe")
        self.want = f"{target}: ring buffer"
        self.top = top
        self.samples = []                       # dicts of the scalar fields
        self.topic_rates = defaultdict(list)    # topic -> [MB/s ...]
        self.t0 = time.time()
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)

    def on_diag(self, msg):
        for st in msg.status:
            if st.name != self.want:
                continue
            kv = {v.key: v.value for v in st.values}
            try:
                s = {
                    "buffer_mb": float(kv["buffer_mb"]),
                    "cap_mb": float(kv["cap_mb"]),
                    "span_sec": float(kv["span_sec"]),
                    "retain_sec": float(kv.get("retain_sec", kv["pre_sec"])),
                    "rate": float(kv["rate_mb_s"]),
                    "needed_mb": float(kv["needed_mb"]),
                    "cap_hit": kv.get("cap_hit") == "true",
                }
            except (KeyError, ValueError):
                return
            self.samples.append(s)
            for k, v in kv.items():
                if k.startswith("/") and v.endswith(" MB/s"):
                    self.topic_rates[k].append(float(v[:-5]))
            flag = "  ** CAP HIT **" if s["cap_hit"] else ""
            print(f"[{time.time() - self.t0:5.0f}s] in {s['rate']:7.1f} MB/s  "
                  f"need {s['needed_mb']:7.0f} MB  buf {s['buffer_mb']:6.0f}/{s['cap_mb']:.0f} MB  "
                  f"span {s['span_sec']:4.1f}/{s['retain_sec']:.1f} s{flag}", flush=True)

    def report(self):
        if not self.samples:
            print("\n수신된 상태 메시지가 없습니다. clip_recorder가 실행 중인지, "
                  "status_period_sec > 0 인지, 노드 이름(--node)이 맞는지 확인하세요.")
            return 1
        rates = [s["rate"] for s in self.samples]
        needed = [s["needed_mb"] for s in self.samples]
        retain = self.samples[-1]["retain_sec"]
        cap = self.samples[-1]["cap_mb"]
        cap_hits = sum(1 for s in self.samples if s["cap_hit"])
        peak_need = max(needed)
        rec = math.ceil(peak_need * MARGIN / 256.0) * 256   # round up to 256 MB steps

        print("\n" + "=" * 64)
        print(f"측정 {len(self.samples)}회, {time.time() - self.t0:.0f}초")
        print(f"유입량      평균 {sum(rates) / len(rates):.1f} MB/s   피크 {max(rates):.1f} MB/s")
        print(f"보관 목표   {retain:.1f} s (pre_sec + trigger_slack_sec)")
        print(f"필요 메모리 평균 {sum(needed) / len(needed):.0f} MB   피크 {peak_need:.0f} MB")
        print(f"현재 상한   {cap:.0f} MB" + (f"   ← {cap_hits}회 상한 초과 (pre 구간 잘림)" if cap_hits else "   (초과 없음)"))
        print(f"\n▶ 추천 max_buffer_mb: {rec}   (피크 {peak_need:.0f} MB × {MARGIN})")
        if cap_hits and cap >= rec:
            print("  주의: 상한 초과가 있었는데 측정 피크가 상한보다 작습니다. 더 길게 측정하세요.")
        total = mem_total_mb()
        if total:
            pct = rec / total * 100
            print(f"  이 PC RAM {total / 1024:.1f} GB 의 {pct:.0f}%"
                  + ("  ← 너무 큽니다. 아래 토픽을 줄이세요." if pct > 50 else ""))

        if self.topic_rates:
            avg = {t: sum(v) / len(v) for t, v in self.topic_rates.items()}
            tot = sum(avg.values()) or 1.0
            print(f"\n무거운 토픽 Top {self.top} (평균 MB/s, 비중, {retain:.0f}초 보관 시 MB):")
            for t, r in sorted(avg.items(), key=lambda x: -x[1])[: self.top]:
                print(f"  {r:7.1f} MB/s  {r / tot * 100:5.1f}%  {r * retain:7.0f} MB   {t}")
        print("=" * 64)
        return 0


def main():
    ap = argparse.ArgumentParser(description="clip_recorder 링 버퍼 필요량 측정")
    ap.add_argument("--duration", type=float, default=60.0, help="측정 시간(초), 기본 60")
    ap.add_argument("--node", default="clip_recorder", help="레코더 노드 이름")
    ap.add_argument("--top", type=int, default=10, help="표시할 토픽 수")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args or None)
    probe = Probe(args.node, args.top)
    print(f"{args.duration:.0f}초 동안 /diagnostics 의 '{probe.want}' 를 수집합니다 (Ctrl+C 로 조기 종료)")
    end = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(probe, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    code = probe.report()
    probe.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
