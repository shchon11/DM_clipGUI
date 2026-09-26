#!/usr/bin/env python3
"""Numerical equivalence of the refactored modules with the validated scripts they came from.

Each test runs the ORIGINAL script (read-only use of /hdd/DM_calib/...; outputs go to a scratch
directory) and the package function on the SAME inputs, and compares the outputs.

    python tests/test_equivalence.py --scratch DIR [refine] [ba] [tracks]

Original locations (never written to): /hdd/DM_calib/lidar_odo (lo_refine.py, run_ba.py),
/home/shchon11/ROSbag_nuscenes_parser/online_calib/tracks.py, /hdd/DM_calib/online/night (data).
"""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
LO = Path("/hdd/DM_calib/lidar_odo")
NIGHT = Path("/hdd/DM_calib/online/night")
OC = Path("/home/shchon11/ROSbag_nuscenes_parser/online_calib")
PY = sys.executable
ENV = dict(os.environ, OMP_NUM_THREADS="4", OPENBLAS_NUM_THREADS="4", MKL_NUM_THREADS="4", PYTHONUNBUFFERED="1")


def sh(cmd, cwd=None, log=None):
    print("$", " ".join(map(str, cmd)), flush=True)
    with open(log, "w") if log else open(os.devnull, "w") as f:
        subprocess.run(list(map(str, cmd)), cwd=cwd, env=ENV, check=True, stdout=f, stderr=subprocess.STDOUT)


def ws_links(scratch, seg):
    """A Workspace whose inputs are symlinks to the validated data of one segment."""
    from nontarget_cal.workspace import Workspace
    ws = Workspace(scratch / "ws")
    links = {ws.lo("ref", seg): LO / "lo" / f"ref_{seg}.npz", ws.lo("map", seg): LO / "lo" / f"map_{seg}.npz",
             ws.seg_dir(seg): NIGHT / seg}
    for c in json.loads((LO / "ba/ALL/result.json").read_text())["cams"]:
        links[ws.tracks(seg, c)] = NIGHT / "tracks" / f"tracks_{seg}_{c}.npz"
    for dst, src in links.items():
        if not dst.exists():
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.symlink_to(src)
    return ws


def t_refine(scratch):
    from nontarget_cal.lo.refine import refine
    seg = "S01"
    old = scratch / "refine_old.npz"
    if not old.exists():
        sh([PY, "lo_refine.py", NIGHT / seg, LO / "lo" / f"kiss_{seg}.npz", "--out", old, "--iters", "1"],
           cwd=LO, log=scratch / "refine_old.log")
    new = scratch / "refine_new.npz"
    refine(NIGHT / seg, LO / "lo" / f"kiss_{seg}.npz", new, iters=1)
    a, b = np.load(old), np.load(new)
    d = np.abs(a["T_w_L"] - b["T_w_L"]).max()
    print(f"refine (1 iteration, {seg}): max |T_old - T_new| = {d:.3e}")
    return {"refine_max_abs_diff": float(d), "pass": bool(d < 1e-9)}


def t_ba(scratch):
    """perseg recipe on S01: all 14 cameras, ties, lens free, started from ba/A_tie_nominal_p2."""
    from nontarget_cal.rgb.solve import calib_cams, result_calib, run_ba
    seg = "S01"
    init = json.loads((LO / "ba/A_tie_nominal_p2/result.json").read_text())
    old_dir = scratch / "ba_old"
    if not (old_dir / "result.json").exists():
        sh([PY, "run_ba.py", "--segs", seg, "--free", "rot", "pos", "f", "k", "pp", "--ties",
            "--init-from", LO / "ba/A_tie_nominal_p2/result.json", "--pos-prior", "10", "--out", old_dir],
           cwd=LO, log=scratch / "ba_old.log")
    ws = ws_links(scratch, seg)
    cams = init["cams"]
    new = run_ba(ws, [seg], cams, scratch / "ba_new", start=calib_cams(cams, result_calib(init)),
                 free=["rot", "pos", "f", "k", "pp"], ties=True, pos_prior=10.0, init="board", threads=4)
    old = json.loads((old_dir / "result.json").read_text())
    dp = max(np.abs(np.array(old["cameras"][c]["T_cam_lidar"]) - np.array(new["cameras"][c]["T_cam_lidar"])).max() for c in cams)
    di = max(np.abs(np.array(old["cameras"][c]["intr_kb"]) - np.array(new["cameras"][c]["intr_kb"])).max() for c in cams)
    print(f"run_ba perseg S01: max |dT| {dp:.3e}, max |d intr| {di:.3e}, n_obs {old['n_obs']} vs {new['n_obs']}")
    stored = json.loads((LO / "ba/perseg" / seg / "result.json").read_text())
    ds = max(np.abs(np.array(stored["cameras"][c]["T_cam_lidar"]) - np.array(new["cameras"][c]["T_cam_lidar"])).max() for c in cams)
    print(f"  vs the stored lidar_odo/ba/perseg/{seg}: max |dT| {ds:.3e}")
    return {"ba_max_abs_dT": float(dp), "ba_max_abs_dintr": float(di), "n_obs_old": old["n_obs"], "n_obs_new": new["n_obs"],
            "vs_stored_perseg_max_abs_dT": float(ds), "pass": bool(dp < 1e-9 and di < 1e-6 and old["n_obs"] == new["n_obs"])}


def t_tracks(scratch):
    from nontarget_cal.rgb.tracks import run_tracks
    seg, cam = "S01", "camera_front5"
    od = scratch / "tracks_old"
    if not (od / f"tracks_{seg}_{cam}.npz").exists():
        sh([PY, OC / "tracks.py", NIGHT / seg, cam, "--mask", "/hdd/DM_calib/online/work/masks/camera_front5.png",
            "--ins", NIGHT / "dm_ins.npz", "--out", od], log=scratch / "tracks_old.log")
    new = scratch / "tracks_new.npz"
    run_tracks(NIGHT / seg, cam, ROOT / "nontarget_cal/data/masks/camera_front5.png", NIGHT / "dm_ins.npz", new)
    a, b = np.load(od / f"tracks_{seg}_{cam}.npz"), np.load(new)
    same = all(np.array_equal(a[k], b[k]) for k in ("frame_ns", "obs_frame", "obs_track", "obs_xy"))
    stored = np.load(NIGHT / "tracks" / f"tracks_{seg}_{cam}.npz")
    same_st = all(np.array_equal(stored[k], b[k]) for k in ("frame_ns", "obs_frame", "obs_track", "obs_xy"))
    print(f"tracks {seg} {cam}: identical to the original script {same}; identical to the stored tracks {same_st}")
    return {"tracks_identical": bool(same), "tracks_identical_to_stored": bool(same_st), "pass": bool(same)}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("tests", nargs="*", default=["refine", "ba", "tracks"])
    ap.add_argument("--scratch", type=Path, required=True)
    a = ap.parse_args()
    a.scratch.mkdir(parents=True, exist_ok=True)
    out = {}
    for t in a.tests:
        out[t] = {"refine": t_refine, "ba": t_ba, "tracks": t_tracks}[t](a.scratch)
        (a.scratch / "equivalence.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))
