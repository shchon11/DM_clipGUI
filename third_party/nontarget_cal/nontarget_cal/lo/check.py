"""How good is a LiDAR trajectory?  Two frame-free checks on the sweeps themselves.

Verbatim numerics of /hdd/DM_calib/lidar_odo/lo_check.py (the INS variant removed).

1. cross-sweep consistency: sweep i+g deskewed into W and compared point-to-plane with
   thin planar patches (< 2 cm, planarity > 0.7) of sweep i alone. The residual is the
   relative pose error between the two sweeps projected on the plane normals, on top of
   the sensor's own noise (the g = 0 row: two halves of the same sweep's points).
2. map thickness as online/map_thickness.py measures it: 11 consecutive sweeps (+-0.5 s)
   accumulated, 16 neighbours within 0.4 m, sqrt of the smallest eigenvalue.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

from .lotraj import LOTraj, deskew, load_sweep, local_planes, sweep_files, voxel_down


def lo_check(seg, traj, gaps=(1, 3, 10, 30), n: int = 24, no_thick: bool = False, log=print) -> dict:
    from types import SimpleNamespace
    a = SimpleNamespace(seg=Path(seg), gaps=list(gaps), n=n, no_thick=no_thick)
    print = lambda *x, **k: log(" ".join(str(y) for y in x))  # noqa: A001,E731
    tr = LOTraj.load(Path(traj))
    files, hdr = sweep_files(a.seg)
    t0, t1 = tr.span()
    ok = np.flatnonzero((hdr >= t0) & (hdr + 100_000_000 <= t1))
    cache = {}

    def W(k):
        if k not in cache:
            s = load_sweep(files[k], 3.0, 60.0)
            cache[k] = voxel_down(deskew(s, hdr[k], tr), 0.1)
        return cache[k]

    out = {}
    for g in [0] + a.gaps:
        cand = ok[np.isin(ok + g, ok)]
        sel = cand[:: max(1, len(cand) // a.n)][: a.n]
        res = []
        for i in sel:
            A = W(i)
            if g == 0:
                A, B = A[0::2], A[1::2]
            else:
                B = W(i + g)
            tree = cKDTree(A)
            okq, n, mu, th, pl = local_planes(A, B, tree, k=10, rmax=0.4)
            good = (th < 0.02) & (pl > 0.7)
            d = np.einsum("ni,ni->n", n[good], B[okq][good] - mu[good])
            res.append(d)
        r = np.concatenate(res)
        mad = 1.4826 * np.median(np.abs(r - np.median(r)))
        out[f"gap{g}"] = {"n": int(len(r)), "robust_std_mm": 1e3 * mad, "median_abs_mm": 1e3 * np.median(np.abs(r))}
        print(f"  gap {g / 10:4.1f} s: point-to-plane robust std {1e3 * mad:6.2f} mm, median |d| "
              f"{1e3 * np.median(np.abs(r)):6.2f} mm  ({len(r)} pts)", flush=True)
    if not a.no_thick:
        rng = np.random.default_rng(0)
        th_all = []
        for c in ok[5:-5][:: max(1, len(ok) // 8)][:8]:
            P = np.concatenate([W(k) for k in range(c - 5, c + 6) if k in set(ok)])
            tree = cKDTree(P)
            q = P[rng.choice(len(P), min(5000, len(P)), replace=False)]
            d, nn = tree.query(q, k=16, distance_upper_bound=0.4)
            full = np.isfinite(d).all(1)
            Q = P[nn[full]]
            mu = Q.mean(1)
            C = np.einsum("nki,nkj->nij", Q - mu[:, None], Q - mu[:, None]) / 16
            w = np.linalg.eigvalsh(C)
            th_all.append(np.sqrt(np.maximum(w[:, 0], 0)))
        th = np.concatenate(th_all)
        out["thickness_11sweeps_median_cm"] = 100 * float(np.median(th))
        print(f"  11-sweep map thickness median {100 * np.median(th):.2f} cm (p10 {100 * np.percentile(th, 10):.2f})")
    return out
