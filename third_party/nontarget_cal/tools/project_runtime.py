#!/usr/bin/env python3
"""Project the wall time of a run on another machine from the per-task CPU time of a measured run.

    python tools/project_runtime.py WORK --cpus 20 --ram-gb 32 [--workers 6 8 12] [--cores-eff 14]
                                     [--extract-min 15] [--scale rgb_tracks=0.75 ...]

Model (a list-scheduling simulation of the pipeline's dependency graph, 1 s steps):
  * every task of WORK/tasks/*/*.result.json becomes a job of cpu_s CPU-seconds that runs on at most
    `par` cores (its measured CPU/wall parallelism when run alone: PAR below);
  * at most `workers` jobs run at once (the worker slots), jobs are admitted in the pipeline's priority
    order (extraction > LO > rest) and only while their memory estimate (resources.task_mem_gb of the
    package config) fits RAM - 6 GB;
  * the running jobs share `cores_eff` cores (logical CPUs are hyper-threads: 20 logical ~ 14 cores);
  * extraction is disk-bound: its tasks keep their measured wall time (or --extract-min spread over them);
  * dependencies as in pipeline.run(): LO and RGB tracks after the window's extraction, thermal tracks
    after its LO, the thermal chain (lens -> edges -> pos -> links -> final+halves -> held-out) and the
    RGB chain (zero-shot 1 -> 2 -> final + halves (validation.rgb_halves_start: start; else halves after the
    final) -> edges -> held-out) after all LO (+ all tracks).
It ignores disk contention and the GUI; it is a projection, not a measurement.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

PAR = {"lo": 2.4, "rgb_tracks": 2.2, "thermal_tracks": 1.2, "thermal_edges": 1.0, "thermal_links": 1.0,
       "extract": 1.0, "default": 2.5}


HALVES_NEXT_TO_FINAL = [True]      # validation.rgb_halves_start == "start" (set from the config in main)


def load(work: Path):
    jobs = []
    for f in sorted((work / "tasks").glob("*/*.result.json")):
        stage = f.parent.name
        key = f.name[:-len(".result.json")]
        r = json.loads(f.read_text())
        spec = json.loads(f.with_name(key + ".json").read_text()) if f.with_name(key + ".json").exists() else {}
        jobs.append({"stage": stage, "key": key, "cpu": float(r.get("cpu_s") or r.get("wall_s") or 0),
                     "wall": float(r.get("wall_s") or 0), "args": spec.get("args", {}), "task": spec.get("task", "")})
    return jobs


def mem_of(job, table):
    e = table.get(job["task"], table.get("default", 2.0))
    if isinstance(e, list):
        e = e[0] + e[1] * len(job["args"].get("segs") or [])
    if job["task"] == "rgb_tracks" and job["args"].get("cams"):
        e *= len(job["args"]["cams"])
    return float(e)


def win_of(job):
    a = job["args"]
    if job["stage"] not in ("lo", "rgb_tracks", "thermal_tracks", "extract", "thermal_edges", "thermal_links"):
        return None
    w = a.get("win") or (a.get("segs") or [None])[0]
    return w["name"] if isinstance(w, dict) else w


NICE_STAGES = ("rgb_tracks", "thermal_tracks")      # resources.tracks_nice
NICE_WEIGHT = [1 / 3.0]                           # CFS weight of nice 5 relative to nice 0 (1.25^-5)


def cpu_shares(run, cores):
    """Cores per running job: proportional to the CFS weight (nice), each capped by its parallelism
    (water-filling)."""
    out, left, todo = {}, float(cores), list(run)
    while todo:
        w = {id(r): (NICE_WEIGHT[0] if r["stage"] in NICE_STAGES else 1.0) for r in todo}
        tot = sum(w.values())
        capped = [r for r in todo if r["par"] <= left * w[id(r)] / tot]
        if not capped:
            for r in todo:
                out[id(r)] = left * w[id(r)] / tot
            break
        for r in capped:
            out[id(r)] = r["par"]
            left -= r["par"]
            todo.remove(r)
    return out


def simulate(jobs, workers, cores, ram_budget, extract_min=None, scale=None):
    scale = scale or {}
    ex = [j for j in jobs if j["stage"] == "extract"]
    if extract_min:
        for j in ex:
            j["wall"] = extract_min * 60 / len(ex) * 2          # 2 extraction workers
    for j in jobs:
        j["left"] = j["cpu"] * scale.get(j["stage"], 1.0)
        j["par"] = PAR.get(j["stage"], PAR["default"])
        j["done"] = False
        j["t_end"] = None
    by = lambda st: [j for j in jobs if j["stage"] == st]  # noqa: E731

    def ready(j, t):
        st = j["stage"]
        done_all = lambda s: all(x["done"] for x in by(s))  # noqa: E731
        if st in ("extract_scan",):
            return True
        if st == "extract":
            return all(x["done"] for x in by("extract_scan"))
        w = win_of(j)
        extracted = lambda w: any(x["done"] for x in by("extract") if win_of(x) == w) or not by("extract")  # noqa: E731
        if st in ("lo", "rgb_tracks"):
            return extracted(w)
        if st == "thermal_tracks":
            return any(x["done"] for x in by("lo") if win_of(x) == w)
        if st == "axes":
            return done_all("lo")
        lo_ok = done_all("lo") and done_all("axes")
        if st == "thermal_lens":
            return lo_ok and done_all("thermal_tracks")
        if st == "thermal_edges":
            return done_all("thermal_lens")
        if st == "thermal_pos":
            return done_all("thermal_edges")
        if st == "thermal_links":
            return done_all("thermal_pos")
        if st == "thermal_final":
            return done_all("thermal_links")
        if st == "rgb_zeroshot1":
            return lo_ok and done_all("rgb_tracks")
        if st == "rgb_zeroshot2":
            return done_all("rgb_zeroshot1")
        if st == "rgb_final":
            return done_all("rgb_zeroshot2")
        if st == "validation":
            if j["key"].startswith("thermal"):
                return done_all("thermal_final")
            if j["key"].startswith("rgb_half") and HALVES_NEXT_TO_FINAL[0]:
                return done_all("rgb_zeroshot2")
            return done_all("rgb_final")
        if st == "validation_heldout":
            src = "thermal" if j["key"].startswith("thermal") else "rgb"
            return all(x["done"] for x in by("validation") if x["key"].startswith(src))
        if st == "images":
            return all(x["done"] for x in jobs if x["stage"] != "images")
        return True

    prio = {"extract_scan": 3, "extract": 3, "lo": 2}
    running, t = [], 0
    mem_used = 0.0
    while not all(j["done"] for j in jobs):
        cand = [j for j in jobs if not j["done"] and j not in running and ready(j, t)]
        cand.sort(key=lambda j: -prio.get(j["stage"], 1))
        for j in cand:
            nex = sum(1 for r in running if r["stage"] == "extract")
            if len(running) >= workers + nex or (j["stage"] == "extract" and nex >= 2):
                continue
            if running and mem_used + j["mem"] > ram_budget:
                continue
            running.append(j)
            mem_used += j["mem"]
            j["t0"] = t
        share = cpu_shares([r for r in running if r["stage"] != "extract"], cores)
        for r in list(running):
            if r["stage"] == "extract":
                r["wall"] -= 1
                fin = r["wall"] <= 0
            else:
                r["left"] -= share[id(r)]
                fin = r["left"] <= 0
            if fin:
                r["done"], r["t_end"] = True, t + 1
                running.remove(r)
                mem_used -= r["mem"]
        t += 1
        if t > 50 * 3600:
            raise RuntimeError("stuck")
    stages = {}
    for j in jobs:
        name = j["stage"]
        if name.startswith("validation"):
            name += "_" + ("thermal" if j["key"].startswith("thermal") else "rgb")
        s = stages.setdefault(name, [1e18, 0])
        s[0] = min(s[0], j["t0"]); s[1] = max(s[1], j["t_end"])
    return t, stages


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work", type=Path)
    ap.add_argument("--cpus", type=int, default=20)
    ap.add_argument("--cores-eff", type=float, default=None, help="core-equivalents (default 0.7 x logical CPUs)")
    ap.add_argument("--ram-gb", type=float, default=32)
    ap.add_argument("--workers", type=int, nargs="+", default=None)
    ap.add_argument("--extract-min", type=float, default=None)
    ap.add_argument("--scale", nargs="*", default=[], help="stage=factor on the CPU time, e.g. rgb_tracks=0.5")
    a = ap.parse_args()
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from nontarget_cal.config import load_config
    cfg = load_config()
    table = cfg["resources"]["task_mem_gb"]
    HALVES_NEXT_TO_FINAL[0] = cfg["validation"].get("rgb_halves_start", "final") == "start"
    NICE_WEIGHT[0] = 1.25 ** -float(cfg["resources"].get("tracks_nice", 0))
    scale = {k: float(v) for k, v in (x.split("=") for x in a.scale)}
    cores = a.cores_eff or 0.7 * a.cpus
    for w in a.workers or [int(a.cpus // 2.5)]:
        jobs = load(a.work)
        for j in jobs:
            j["mem"] = mem_of(j, table)
        T, st = simulate(jobs, w, cores, a.ram_gb - 6.0, a.extract_min, scale)
        print(f"workers {w:2d}, {cores:.0f} cores, RAM {a.ram_gb:.0f} GB: total {T / 60:.0f} min; " +
              ", ".join(f"{k} {v[0] / 60:.0f}-{v[1] / 60:.0f}" for k, v in sorted(st.items(), key=lambda kv: kv[1][0])))


if __name__ == "__main__":
    main()
