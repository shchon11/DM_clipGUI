#!/usr/bin/env python3
"""Profile the heavy task types on ONE window of an existing run (cProfile + wall clock + CPU time).

    python tools/profile_stages.py --src WORK --dst SCRATCH [--win S01] [--tasks kiss refine tracks ba thermal_tracks
                                   edges links tba] [--threads 3]

SCRATCH becomes a workspace whose inputs are symlinks into WORK (extract, thermal16, windows, tracks, LO ...),
so every task recomputes only its own output. Writes SCRATCH/profile/<task>.prof and profile.json
(wall s, process CPU s, CPU utilisation = cpu/wall, top functions by cumulative and internal time).
"""
from __future__ import annotations

import argparse
import cProfile
import io
import json
import os
import pstats
import shutil
import time
from pathlib import Path


def link_ws(src: Path, dst: Path, win: str, keep_outputs: set):
    from nontarget_cal.workspace import Workspace
    ws = Workspace(dst)
    for d in ("windows", "names", "preflight"):
        for f in (src / d).glob("*"):
            t = dst / d / f.name
            if not t.exists():
                t.symlink_to(f)
    for f in list((src / "extract").glob("b*_*.npz")) + list((src / "extract").glob("b*_*.json")) + list((src / "thermal16").glob("b*_*.npz")):
        t = dst / f.relative_to(src)
        if not t.exists():
            t.symlink_to(f)
    for d in ("extract", "thermal16"):
        t = dst / d / win
        if not t.exists():
            t.symlink_to(src / d / win)
    for kind in ("kiss", "ref", "map"):
        if kind in keep_outputs:
            continue
        t = ws.lo(kind, win)
        if not t.exists():
            t.symlink_to(src / "lo" / f"{kind}_{win}.npz")
    for f in (src / "tracks").glob(f"tracks_{win}_*.npz"):
        if "tracks" in keep_outputs:
            continue
        t = dst / "tracks" / f.name
        if not t.exists():
            t.symlink_to(f)
    for sub in ("tracks", "edges", "links"):
        (dst / "thermal" / sub).mkdir(parents=True, exist_ok=True)
        if sub in keep_outputs or f"thermal_{sub}" in keep_outputs:
            continue
        for f in (src / "thermal" / sub).glob("*"):
            t = dst / "thermal" / sub / f.name
            if not t.exists():
                t.symlink_to(f)
    for f in ("lens", "pos_e", "final"):
        p = src / "thermal" / f / "result.json"
        if p.exists():
            (dst / "thermal" / f"src_{f}").mkdir(parents=True, exist_ok=True)
            t = dst / "thermal" / f"src_{f}" / "result.json"
            if not t.exists():
                t.symlink_to(p)
    t = dst / "lo" / "vehicle_axes.json"
    if not t.exists() and (src / "lo" / "vehicle_axes.json").exists():
        t.symlink_to(src / "lo" / "vehicle_axes.json")
    return ws


def top(prof, n=18):
    out = {}
    for key in ("cumulative", "tottime"):
        s = io.StringIO()
        pstats.Stats(prof, stream=s).sort_stats(key).print_stats(n)
        out[key] = s.getvalue().splitlines()[-n - 2:]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, required=True)
    ap.add_argument("--dst", type=Path, required=True)
    ap.add_argument("--win", default="S01")
    ap.add_argument("--tasks", nargs="+", default=["kiss", "refine", "tracks", "ba", "thermal_tracks", "edges", "links", "tba"])
    ap.add_argument("--threads", type=int, default=3)
    ap.add_argument("--refine-iters", type=int, default=2)
    a = ap.parse_args()
    for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "OPENCV_NUM_THREADS"):
        os.environ[k] = str(a.threads)
    import numpy as np  # noqa: F401
    import torch
    torch.set_num_threads(a.threads)
    from nontarget_cal.config import load_config
    cfg = load_config()
    cfg["resources"]["solver_threads"] = a.threads
    res = {}
    pdir = a.dst / "profile"
    pdir.mkdir(parents=True, exist_ok=True)
    rj = pdir / "profile.json"
    if rj.exists():
        res = json.loads(rj.read_text())
    w = a.win
    for t in a.tasks:
        keep = {"kiss": {"kiss"}, "refine": {"ref"}, "tracks": {"tracks"}, "thermal_tracks": {"thermal_tracks"},
                "edges": {"thermal_edges"}, "links": {"thermal_links"}}.get(t, set())
        ws = link_ws(a.src, a.dst, w, keep)
        log = lambda *x, **k: None  # noqa: E731
        from nontarget_cal import tasks as T
        if t == "kiss":
            from nontarget_cal.lo.kiss import run_kiss
            fn = lambda: run_kiss(ws.seg_dir(w), ws.lo("kiss", w), threads=a.threads, log=log)  # noqa: E731
        elif t == "refine":
            from nontarget_cal.lo.refine import refine
            fn = lambda: refine(ws.seg_dir(w), ws.lo("kiss", w), ws.lo("ref", w), iters=a.refine_iters, log=log)  # noqa: E731
        elif t == "tracks":
            fn = lambda: T.t_rgb_tracks(ws, cfg, log, w, "camera_front5", "b0", str(Path(cfg["paths"]["masks_dir"]) / "camera_front5.png"))  # noqa: E731
            if ws.tracks(w, "camera_front5").is_symlink() or ws.tracks(w, "camera_front5").exists():
                ws.tracks(w, "camera_front5").unlink()
        elif t == "thermal_tracks":
            p = ws.thermal_tracks(w, "thermal_left")
            if p.exists() or p.is_symlink():
                p.unlink()
            fn = lambda: T.t_thermal_tracks(ws, cfg, log, w, "thermal_left")  # noqa: E731
        elif t == "ba":
            shutil.rmtree(a.dst / "rgb" / "prof_ba", ignore_errors=True)
            cams = list(cfg["cameras"]["rgb"])
            opts = {"free": ["rot", "pos", "f", "k", "pp"], "ties": True, "pos_prior": 10.0, "iters": 25}
            fn = lambda: T.t_rgb_solve(ws, cfg, log, "prof_ba", [w], cams, {"kind": "result", "path": str(a.src / "rgb" / "final" / "result.json")}, opts)  # noqa: E731
        elif t == "edges":
            for c in ("thermal_left", "thermal_right"):
                p = ws.thermal_edges_dir() / f"{w}_{c}.npz"
                if p.exists() or p.is_symlink():
                    p.unlink()
            fn = lambda: T.t_thermal_edges(ws, cfg, log, w, str(a.dst / "thermal" / "src_lens" / "result.json"))  # noqa: E731
        elif t == "links":
            p = ws.root / "thermal" / "links" / "prof.npz"
            if p.exists():
                p.unlink()
            fn = lambda: T.t_thermal_links(ws, cfg, log, "prof", [w], str(a.dst / "thermal" / "src_pos_e" / "result.json"))  # noqa: E731
        elif t == "tba":
            shutil.rmtree(a.dst / "thermal" / "prof_tba", ignore_errors=True)
            p = a.dst / "thermal" / "links" / "links.npz"
            if not p.exists():
                p.symlink_to(a.src / "thermal" / "links" / "links.npz")
            opts = {"free": ["rot", "pos", "f", "aspect", "dt"], "ties": True, "edges": True, "stereo": "links",
                    "time": "smooth", "dt_mode": "seg"}
            fn = lambda: T.t_thermal_solve(ws, cfg, log, "prof_tba", [w], {"path": str(a.dst / "thermal" / "src_lens" / "result.json")}, opts)  # noqa: E731
        else:
            raise ValueError(t)
        prof = cProfile.Profile()
        c0, t0 = time.process_time(), time.time()
        prof.runcall(fn)
        wall, cpu = time.time() - t0, time.process_time() - c0
        prof.dump_stats(str(pdir / f"{t}.prof"))
        res[t] = {"wall_s": round(wall, 1), "cpu_s": round(cpu, 1), "cpu_util": round(cpu / wall, 2), **top(prof)}
        print(f"{t}: wall {wall:.1f} s, cpu {cpu:.1f} s ({cpu / wall:.2f} cores)", flush=True)
        rj.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
