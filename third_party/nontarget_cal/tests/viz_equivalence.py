#!/usr/bin/env python3
"""Opt-in real S01 CPU checks: viz on/off bit identity and matched wall time.

This is intentionally outside pytest discovery: it needs the read-only calibration
corpus on the calibration machine. Each run is a fresh process, one numerical job
at a time, using at most four CPU threads and no GPU. For balanced timing use two
repeats (off/on then on/off), and report the pair results without claiming that a
small noisy sample proves an end-to-end overhead bound.

    .venv/bin/python tests/viz_equivalence.py --scratch DIR --repeats 2

The historical SIFT comparison reproduces commit 1af77de's S01 check against
nt_regress/crosstime/cache_snap/S01.npz (la/lb/dist/dts/imp). Source workspaces
are never constructed or modified: only individual input paths are linked into
the scratch workspace. Numerical array payload bytes, including NaN payloads,
are compared; ZIP timestamps and runtime metadata are intentionally excluded.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
LO = Path("/hdd/DM_calib/lidar_odo")
NIGHT = Path("/hdd/DM_calib/online/night")
FULL = Path("/hdd/DM_calib/nt_regress/full/work")
SIFT_REFERENCE = Path("/hdd/DM_calib/nt_regress/crosstime/cache_snap/S01.npz")
CASES = ("refine", "rgb", "thermal", "sift")
SOURCES = ("nontarget_cal/viz.py", "nontarget_cal/viz_preview.py", "nontarget_cal/config/default.yaml",
           "nontarget_cal/lo/refine.py", "nontarget_cal/lo/maps.py",
           "nontarget_cal/rgb/solve.py", "nontarget_cal/rgb/rigba.py", "nontarget_cal/rgb/crosstime.py",
           "nontarget_cal/thermal/solve.py", "nontarget_cal/thermal/tba.py")


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=True) + "\n")


def source_fingerprint():
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCES}


def link(dst, src):
    dst, src = Path(dst), Path(src)
    if not src.exists():
        raise FileNotFoundError(src)
    if not dst.exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.symlink_to(src)


def workspace(scratch, historical=False):
    from nontarget_cal.workspace import Workspace

    ws = Workspace(scratch / "ws")
    source = LO / "lo" if historical else FULL / "lo"
    for kind in ("ref", "map"):
        link(ws.lo(kind, "S01"), source / f"{kind}_S01.npz")
    link(ws.seg_dir("S01"), NIGHT / "S01" if historical else FULL / "extract/S01")
    if historical:
        cams = json.loads((LO / "ba/ALL/result.json").read_text())["cams"]
        for cam in cams:
            link(ws.tracks("S01", cam), NIGHT / "tracks" / f"tracks_S01_{cam}.npz")
    else:
        plan = json.loads((FULL / "windows/plan.json").read_text())
        plan["windows"] = [w for w in plan["windows"] if w["name"] == "S01"]
        dump(ws.root / "windows/plan.json", plan)
        link(ws.thermal16("S01"), FULL / "thermal16/S01")
        for cam in ("thermal_left", "thermal_right"):
            link(ws.thermal_tracks("S01", cam), FULL / "thermal/tracks" / f"tracks_S01_{cam}.npz")
            for bag in plan["bags"]:
                fn = f"{bag['bag_id']}_clock_{cam}.npz"
                link(ws.root / "thermal16" / fn, FULL / "thermal16" / fn)
    return ws


def digest_array(value):
    import numpy as np

    a = np.asarray(value)
    return {"dtype": a.dtype.str, "shape": list(a.shape),
            "sha256": hashlib.sha256(a.tobytes(order="C")).hexdigest()}


def npz_fingerprint(path):
    import numpy as np

    with np.load(path, allow_pickle=False) as data:
        return {key: digest_array(data[key]) for key in data.files if key != "wall_s"}


def numerical_result(value):
    """Drop the sole nondeterministic report field, preserving all solve outputs."""
    return {key: val for key, val in value.items() if key != "wall_s"}


def publication_gaps(stream):
    """Measure first publication of distinct images, not repeated snapshot refs."""
    seen, times, by_camera = set(), [], {}
    if stream.exists():
        for line in stream.read_text().splitlines():
            event = json.loads(line)
            for camera, asset in event.get("assets", {}).get("matching", {}).items():
                key = asset.get("image")
                if key and key not in seen:
                    seen.add(key)
                    stamp = float(event["t"])
                    times.append(stamp)
                    by_camera.setdefault(camera, []).append(stamp)
    def minimum(values):
        return min((b - a for a, b in zip(values, values[1:])), default=None)
    return {"distinct_images": len(times), "global_min_gap_s": minimum(times),
            "per_camera": {camera: {"count": len(values), "min_gap_s": minimum(values)}
                           for camera, values in by_camera.items()}}


def configure_viz(scratch, enabled, camera=None):
    from nontarget_cal.config import load_config
    from nontarget_cal.viz import configure

    # Public API is shared by the real CLI and workers; scratch retains the stream.
    cfg = load_config(overrides={"viz": {"enabled": enabled}})
    emitter = configure(scratch, cfg,
                        context={"run_id": "s01-equivalence", "window_id": "S01", "bag_id": "b0"}, fresh=True)
    if emitter.enabled != enabled:
        raise RuntimeError("requested visualization configuration was not activated")
    if enabled and camera:
        control = scratch / "viz/control.json"
        tmp = control.with_suffix(".tmp")
        dump(tmp, {"camera": camera, "enabled": True})
        tmp.replace(control)
    return emitter


def worker(args):
    # An operator can defer the next numerical child while another calibration
    # uses shared-machine RAM. Existing solves are never interrupted.
    pauses = [args.scratch.parent / ".pause", args.scratch.parent / f".pause_{args.case}"]
    if any(path.exists() for path in pauses):
        print(f"Waiting for {pauses} to be removed before loading numerical inputs", flush=True)
    while any(path.exists() for path in pauses):
        time.sleep(1)
    if args.case == "sift" and args.enabled and (args.scratch.parent / ".sift_golden_only").exists():
        print("SKIP: only one fresh historical SIFT check requested; no viz-on SIFT solve executed", flush=True)
        raise SystemExit(77)
    import cv2
    import numpy as np

    cv2.setNumThreads(1)
    args.scratch.mkdir(parents=True, exist_ok=True)
    source = source_fingerprint()
    start = time.perf_counter()
    cpu_start = time.process_time()
    emitter = configure_viz(args.scratch, args.enabled, args.camera)
    if args.case == "refine":
        from nontarget_cal.lo.refine import refine

        refine(NIGHT / "S01", LO / "lo/kiss_S01.npz", args.scratch / "refine.npz", iters=1)
        numerical = npz_fingerprint(args.scratch / "refine.npz")
    elif args.case == "rgb":
        from nontarget_cal.rgb.solve import calib_cams, result_calib, run_ba

        ws = workspace(args.scratch, historical=True)
        initial = json.loads((LO / "ba/A_tie_nominal_p2/result.json").read_text())
        cams = initial["cams"]
        result = run_ba(ws, ["S01"], cams, args.scratch / "rgb",
                        start=calib_cams(cams, result_calib(initial)),
                        free=["rot", "pos", "f", "k", "pp"], ties=True,
                        pos_prior=10.0, init="board", threads=4, device="cpu", iters=args.iters)
        numerical = {"result": numerical_result(result),
                     "state": npz_fingerprint(args.scratch / "rgb/state.npz")}
    elif args.case == "thermal":
        from nontarget_cal.thermal.solve import run_tba
        from nontarget_cal.thermal.timemodel import Plan

        ws = workspace(args.scratch)
        initial = json.loads((FULL / "thermal/final/result.json").read_text())["cameras"]
        result = run_tba(ws, Plan(ws), ["S01"], args.scratch / "thermal",
                         start=initial, free=["rot", "pos", "f", "rs", "dt"],
                         ties=True, threads=4, iters=args.iters, rounds=1)
        numerical = {"result": numerical_result(result)}
        state = args.scratch / "thermal/state.npz"
        if state.exists():
            numerical["state"] = npz_fingerprint(state)
    else:
        from nontarget_cal.rgb.crosstime import associate

        ws = workspace(args.scratch)
        result = json.loads((FULL / "rgb/final_keys/result.json").read_text())
        links, stats = associate(ws, result, FULL / "rgb/final_keys/state.npz",
                                 cache_dir=args.scratch / "sift_cache", only={result["segs"].index("S01")},
                                 matcher="sift")
        np.savez(args.scratch / "sift_links.npz", **{k: np.asarray(v) for k, v in links.items()})
        fingerprint = npz_fingerprint(args.scratch / "sift_cache/S01.npz")
        numerical = {"cache": fingerprint, "links": npz_fingerprint(args.scratch / "sift_links.npz"),
                     "stats": stats}
        historical_same = fingerprint == npz_fingerprint(SIFT_REFERENCE)
        dump(args.scratch / "sift_historical.json",
             {"reference": str(SIFT_REFERENCE), "byte_identical": historical_same})
        if not historical_same:
            raise AssertionError("S01 SIFT cache differs from the historical la/lb/dist/dts/imp check")
    emitter.close()
    elapsed = time.perf_counter() - start
    cpu_elapsed = time.process_time() - cpu_start
    dump(args.scratch / "numerical.json", numerical)
    # This fingerprint canonicalizes JSON NaNs consistently; all NPZ comparisons
    # above hash the actual dtype+shape+payload, not approximate numerical values.
    sha = hashlib.sha256(json.dumps(numerical, sort_keys=True, allow_nan=True).encode()).hexdigest()
    stream = args.scratch / "viz/events.jsonl"
    events = len(stream.read_text().splitlines()) if stream.exists() else 0
    if args.enabled and args.case != "sift" and not events:
        raise RuntimeError("visualization was enabled but the numerical hooks emitted no state")
    dump(args.scratch / "measurement.json", {"case": args.case, "enabled": args.enabled,
         "wall_s": elapsed, "cpu_s": cpu_elapsed, "numerical_sha256": sha,
         "events": events,
         "preview_mode": args.camera or "round_robin",
         "image_publication": publication_gaps(stream),
         "image_snapshots": len(list((args.scratch / "viz/assets").glob("*.jpg"))),
         "npz_snapshots": len(list((args.scratch / "viz/assets").glob("*.npz"))),
         "source_sha256": source, "source_stable": source == source_fingerprint()})


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scratch", type=Path, required=True)
    p.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--iters", type=int, default=5, help="RGB/thermal solver iteration budget; LO uses one")
    p.add_argument("--sift-golden-only", action="store_true",
                   help="run SIFT once against the historical cache; do not claim SIFT viz on/off equivalence")
    p.add_argument("--camera", help="select a preview camera through control.json; default is round robin")
    p.add_argument("--case", choices=CASES, help=argparse.SUPPRESS)
    p.add_argument("--enabled", action="store_true", help=argparse.SUPPRESS)
    args = p.parse_args()
    if args.case:
        worker(args)
        return
    args.scratch.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, OMP_NUM_THREADS="4", OPENBLAS_NUM_THREADS="4", MKL_NUM_THREADS="4",
               NONTARGET_LO_THREADS="4", NONTARGET_TIE_THREADS="1", NONTARGET_DEVICE="cpu",
               OPENCV_FOR_THREADS_NUM="1", CUDA_VISIBLE_DEVICES="", PYTHONUNBUFFERED="1")
    report = {"inputs": {"window": "S01", "t0_s": 120, "t1_s": 160,
                        "historical_lo": str(LO), "historical_rgb": str(NIGHT), "thermal_sift": str(FULL)},
              "threads": 4, "rgb_thermal_iters": args.iters, "viewer_running": False, "pairs": []}
    for case in args.cases:
        for repeat in range(args.repeats):
            results = {}
            order = (False,) if case == "sift" and args.sift_golden_only else (
                (False, True) if repeat % 2 == 0 else (True, False))
            for enabled in order:
                output = args.scratch / f"{case}_{repeat}_{'on' if enabled else 'off'}"
                output.mkdir(parents=True, exist_ok=True)
                cmd = [sys.executable, str(Path(__file__).resolve()), "--scratch", str(output),
                       "--case", case, "--iters", str(args.iters)] + (["--enabled"] if enabled else [])
                if args.camera:
                    cmd += ["--camera", args.camera]
                print("$", " ".join(cmd), flush=True)
                with (output / "run.log").open("w") as log:
                    subprocess.run(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
                results[enabled] = json.loads((output / "measurement.json").read_text())
            if len(order) == 1:
                report.setdefault("historical_checks", []).append({"case": case, "measurement": results[False],
                    "comparison": "fresh S01 SIFT versus historical la/lb/dist/dts/imp; no viz-on solve"})
                dump(args.scratch / "report.json", report)
                break
            off, on = results[False], results[True]
            pair = {"case": case, "repeat": repeat, "off": off, "on": on,
                    "byte_identical": off["numerical_sha256"] == on["numerical_sha256"],
                    "same_source": off["source_sha256"] == on["source_sha256"],
                    "overhead_pct": 100 * (on["wall_s"] / off["wall_s"] - 1)}
            report["pairs"].append(pair)
            dump(args.scratch / "report.json", report)
            print(json.dumps(pair, indent=2), flush=True)
            if not pair["byte_identical"]:
                raise SystemExit(f"{case}: numerical output changed with visualization enabled")


if __name__ == "__main__":
    main()
