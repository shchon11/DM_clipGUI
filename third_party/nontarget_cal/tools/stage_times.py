#!/usr/bin/env python3
"""Per-stage wall clock of one or more runs from their events.jsonl (last run_start .. run_end).

    python tools/stage_times.py WORK_OR_EVENTS [WORK_OR_EVENTS ...] [--md]

Stages overlap (extraction streams into the LiDAR odometry and the trackers; the RGB and thermal chains
run in parallel), so the table gives each stage's start and end relative to the run start and its
wall time; 'total' is run_start -> run_end. Task-level sums (worker wall time, CPU) come from
<work>/tasks/<stage>/*.result.json.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def load(p: Path):
    p = Path(p)
    ev = p / "events.jsonl" if p.is_dir() else p
    rows = [json.loads(line) for line in ev.read_text().splitlines() if line.strip()]
    starts = [i for i, r in enumerate(rows) if r["ev"] == "run_start"]
    rows = rows[starts[-1]:] if starts else rows
    t0 = rows[0]["t"]
    st, out = {}, {}
    for r in rows:
        if r["ev"] == "stage_start":
            st.setdefault(r["stage"], r["t"])
        elif r["ev"] == "stage_end":
            s = r["stage"]
            a = st.get(s, r["t"] - r.get("wall_s", 0))
            o = out.get(s)
            out[s] = {"start_min": round((min(a, o["_a"]) if o else a) - t0, 1) / 60, "_a": min(a, o["_a"]) if o else a,
                      "end": r["t"], "disk_gb": r.get("disk_gb")}
    end = next((r["t"] for r in rows if r["ev"] == "run_end"), max((o["end"] for o in out.values()), default=t0))
    res = {s: {"start_min": round((o["_a"] - t0) / 60, 1), "end_min": round((o["end"] - t0) / 60, 1),
               "wall_min": round((o["end"] - o["_a"]) / 60, 1), "disk_gb": o["disk_gb"]} for s, o in out.items()}
    res["total"] = {"start_min": 0.0, "end_min": round((end - t0) / 60, 1), "wall_min": round((end - t0) / 60, 1)}
    return res


def tasks(work: Path):
    out = {}
    for d in sorted((Path(work) / "tasks").glob("*")):
        w = c = 0.0
        n = 0
        for f in d.glob("*.result.json"):
            r = json.loads(f.read_text())
            w += r.get("wall_s") or 0
            c += r.get("cpu_s") or 0
            n += 1
        out[d.name] = {"n": n, "task_wall_min": round(w / 60, 1), "cpu_min": round(c / 60, 1)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--tasks", action="store_true")
    a = ap.parse_args()
    res = [load(r) for r in a.runs]
    stages = []
    for r in res:
        stages += [s for s in r if s not in stages]
    print("| stage | " + " | ".join(str(r) for r in a.runs) + " |")
    print("|---|" + "---|" * len(a.runs))
    for s in stages:
        cells = []
        for r in res:
            x = r.get(s)
            cells.append("–" if x is None else f"{x['wall_min']:.1f} ({x['start_min']:.0f}-{x['end_min']:.0f})")
        print(f"| {s} | " + " | ".join(cells) + " |")
    if a.tasks:
        for r in a.runs:
            work = r if r.is_dir() else r.parent
            print(r, json.dumps(tasks(work), indent=None))


if __name__ == "__main__":
    main()
