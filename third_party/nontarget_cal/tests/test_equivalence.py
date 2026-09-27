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


def t_kernels(scratch):
    """Compiled kernels vs the validated code on the same inputs: LO refinement pair terms (numba vs
    numpy, 1 iteration of S01), rig BA (numba vs torch, the t_ba recipe), exact voxel de-dup."""
    import os
    from nontarget_cal.fastops import _first_occurrence_np, first_occurrence
    from nontarget_cal.lo import refine as R
    from nontarget_cal.rgb.solve import calib_cams, result_calib, run_ba
    out = {}
    seg = "S01"
    for k in ("numpy", "numba"):
        f = scratch / f"refine_{k}.npz"
        if not f.exists():
            R.KERNEL = k
            R.refine(NIGHT / seg, LO / "lo" / f"kiss_{seg}.npz", f, iters=1)
    d = np.abs(np.load(scratch / "refine_numpy.npz")["T_w_L"] - np.load(scratch / "refine_numba.npz")["T_w_L"]).max()
    out["refine_numba_vs_numpy_max_abs_dT"] = float(d)
    init = json.loads((LO / "ba/A_tie_nominal_p2/result.json").read_text())
    ws = ws_links(scratch, seg)
    cams = init["cams"]
    res = {}
    for k in ("torch", "numba"):
        res[k] = run_ba(ws, [seg], cams, scratch / f"ba_{k}", start=calib_cams(cams, result_calib(init)),
                        free=["rot", "pos", "f", "k", "pp"], ties=True, pos_prior=10.0, init="board", threads=4, kernel=k)
    dp = max(np.abs(np.array(res["torch"]["cameras"][c]["T_cam_lidar"]) - np.array(res["numba"]["cameras"][c]["T_cam_lidar"])).max() for c in cams)
    out["ba_numba_vs_torch_max_abs_dT"] = float(dp)
    z = np.load(LO / "lo" / f"map_{seg}.npz")["P"][:2_000_000].astype(np.float64)
    kk = np.floor(z / 0.05).astype(np.int64) + (1 << 20)
    key = (kk[:, 0] << 42) | (kk[:, 1] << 21) | kk[:, 2]
    out["voxel_first_occurrence_identical"] = bool(np.array_equal(_first_occurrence_np(key), first_occurrence(key)))
    print(out)
    out["pass"] = bool(d < 1e-9 and dp < 1e-9 and out["voxel_first_occurrence_identical"])
    return out


def t_ties(scratch):
    """Landmark-to-plane association with the per-bin candidate cache (3 rounds, landmarks moved by
    ~3 mm and 2 % removed between them) vs a fresh SegMap per call (the validated selection)."""
    from nontarget_cal.rgb.data import SegMap
    seg = "S01"
    st = np.load(LO / "ba/perseg" / seg / "state.npz") if (LO / "ba/perseg" / seg / "state.npz").exists() else None
    ws = ws_links(scratch, seg)
    rng = np.random.default_rng(1)
    if st is not None and "obs_t" in st.files:
        X = st["X"]
        M = len(X)
        tm = np.zeros(M)
        np.add.at(tm, st["obs_lm"], st["obs_t"].astype(np.float64))
        tm /= np.maximum(np.bincount(st["obs_lm"], minlength=M), 1)
    else:                                   # synthetic landmarks on the map points
        mp = np.load(ws.lo("map", seg))
        i = rng.choice(len(mp["P"]), 20000, replace=False)
        X = mp["P"][i].astype(np.float64) + rng.normal(0, 0.01, (20000, 3))
        tm = mp["hdr"][np.searchsorted(mp["start"], i, side="right") - 1].astype(np.float64)
    calls = [(X, tm, 0.3)]
    keep = rng.random(len(X)) > 0.02
    calls.append((X[keep] + rng.normal(0, 0.003, (keep.sum(), 3)), tm[keep], 0.15))
    calls.append((calls[1][0] + rng.normal(0, 0.002, calls[1][0].shape), tm[keep], 0.15))
    cached = SegMap(ws.lo("map", seg))
    same = True
    for Xc, tc, g in calls:
        a = SegMap(ws.lo("map", seg))
        a.CACHE_GB = 0.0
        r0 = a.associate(Xc, tc.astype(np.int64), gate=g)
        r1 = cached.associate(Xc, tc.astype(np.int64), gate=g)
        same &= all(np.array_equal(u, v) for u, v in zip(r0, r1))
    print(f"ties with the candidate cache identical: {same} (cache {SegMap.stats})")
    return {"ties_identical": bool(same), "pass": bool(same)}


def t_lk(scratch):
    """CUDA Lucas-Kanade (rgb/cudalk.py) vs OpenCV's CPU LK (the validated tracker) on the same frame pairs
    and points (S01 camera_front5, 150 consecutive pairs, the tracker's own points), and the whole tracker
    with device=cuda vs the validated tracks (not bit-identical: float interpolation)."""
    import cv2
    from nontarget_cal.rgb import tracks as T
    from nontarget_cal.rgb.cudalk import CudaLK, available
    if not available():
        return {"skipped": "no CUDA GPU / Triton", "pass": True}
    seg, cam = "S01", "camera_front5"
    files = sorted((NIGHT / seg / "cam" / cam).glob("*.jpg"))[:151]
    imgs = [cv2.imread(str(f), cv2.IMREAD_GRAYSCALE) for f in files]
    lk = CudaLK.from_lk(T.LK)
    cv2.setNumThreads(1)
    pts = cv2.goodFeaturesToTrack(imgs[0], 1200, 0.005, 12).reshape(-1, 2).astype(np.float32)
    dp, db, mis, n = [], [], 0, 0
    pp = lk.pyramid(imgs[0])
    for a, b in zip(imgs[:-1], imgs[1:]):
        p1, st, pb, st2 = T.lk_pair(a, b, pts)
        pc = lk.pyramid(b)
        q1, qs, qb, qs2 = lk.pair(pp, pc, pts)
        pp = pc
        ok = (st.ravel() == 1) & (qs.ravel() == 1)
        dp.append(np.linalg.norm((p1 - q1).reshape(-1, 2)[ok], axis=1))
        ok2 = ok & (st2.ravel() == 1) & (qs2.ravel() == 1)
        db.append(np.linalg.norm((pb - qb).reshape(-1, 2)[ok2], axis=1))
        mis += int((st.ravel() != qs.ravel()).sum()); n += len(pts)
        pts = p1.reshape(-1, 2)[st.ravel() == 1].astype(np.float32)
        if len(pts) < 800:
            pts = np.concatenate([pts, cv2.goodFeaturesToTrack(b, 600, 0.005, 12).reshape(-1, 2).astype(np.float32)])
    dp, db = np.concatenate(dp), np.concatenate(db)
    out = {"pairs": len(imgs) - 1, "points": n, "fwd_px_median": float(np.median(dp)), "fwd_px_p99": float(np.percentile(dp, 99)),
           "bwd_px_median": float(np.median(db)), "status_mismatch_frac": mis / n}
    new = scratch / "tracks_cuda.npz"
    T.run_tracks(NIGHT / seg, cam, ROOT / "nontarget_cal/data/masks/camera_front5.png", NIGHT / "dm_ins.npz", new,
                 device="cuda")
    a, b = np.load(NIGHT / "tracks" / f"tracks_{seg}_{cam}.npz"), np.load(new)
    out.update({"tracks_obs_validated": int(len(a["obs_track"])), "tracks_obs_cuda": int(len(b["obs_track"])),
                "tracks_n_validated": int(len(np.unique(a["obs_track"]))), "tracks_n_cuda": int(len(np.unique(b["obs_track"])))})
    print(out)
    out["pass"] = bool(out["fwd_px_median"] < 0.01 and out["status_mismatch_frac"] < 0.01
                       and abs(out["tracks_obs_cuda"] / out["tracks_obs_validated"] - 1) < 0.02)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("tests", nargs="*", default=["refine", "ba", "tracks", "kernels", "ties", "lk"])
    ap.add_argument("--scratch", type=Path, required=True)
    a = ap.parse_args()
    a.scratch.mkdir(parents=True, exist_ok=True)
    out = {}
    for t in a.tests:
        out[t] = {"refine": t_refine, "ba": t_ba, "tracks": t_tracks, "kernels": t_kernels, "ties": t_ties, "lk": t_lk}[t](a.scratch)
        (a.scratch / "equivalence.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))
