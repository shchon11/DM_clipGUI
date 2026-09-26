"""Vehicle axes expressed in the Ouster frame, from the LiDAR data alone (verbatim numerics of
/hdd/DM_calib/lidar_odo/vehicle_axes.py).

forward = mean direction of travel (LO velocity in the LiDAR frame, straight driving, > 3 m/s);
up = normal of the road around the car (RANSAC plane of points 3-12 m away, |lateral| < 4 m,
z < -1.2 m in the LiDAR frame). R_L_V has columns forward, left, up expressed in L.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .lotraj import LOTraj, load_sweep, sweep_files, xyz


def vehicle_axes(pairs, log=print) -> dict:
    """pairs: [(ref_npz, seg_dir), ...]"""
    fw, up = [], []
    for ref, seg_dir in pairs:
        tr = LOTraj.load(ref)
        T = tr.T[1:]
        v = np.einsum("nji,nj->ni", T[:-1, :3, :3], T[1:, :3, 3] - T[:-1, :3, 3]) / 0.1
        w = Rot.from_matrix(np.einsum("nji,njk->nik", T[:-1, :3, :3], T[1:, :3, :3])).magnitude() / 0.1
        ok = (np.linalg.norm(v, axis=1) > 3) & (np.degrees(w) < 2)
        if ok.any():
            fw.append((v[ok] / np.linalg.norm(v[ok], axis=1, keepdims=True)))
        files, hdr = sweep_files(seg_dir)
        for f in files[::40]:
            p = xyz(load_sweep(f, 3.0, 12.0))
            cand = p[(np.abs(p[:, 1]) < 4) & (p[:, 2] < -1.2)]
            if len(cand) < 200:
                continue
            best, bn = 0, None
            rng = np.random.default_rng(0)
            for _ in range(200):
                s = cand[rng.choice(len(cand), 3, replace=False)]
                n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
                if nn < 1e-6:
                    continue
                n /= nn
                inl = np.abs((cand - s[0]) @ n) < 0.03
                if inl.sum() > best:
                    best, bn, bi = inl.sum(), n, inl
            if bn is None:
                continue
            q = cand[bi]; mu = q.mean(0)
            n = np.linalg.eigh((q - mu).T @ (q - mu))[1][:, 0]
            if n[2] < 0:
                n = -n
            up.append((n, float(-(mu @ n))))
    if not fw or not up:
        raise RuntimeError("vehicle axes: no straight fast driving or no road plane in the windows")
    F = np.concatenate(fw).mean(0); F /= np.linalg.norm(F)
    U = np.array([u[0] for u in up]); Um = np.median(U, 0); Um /= np.linalg.norm(Um)
    h = np.median([u[1] for u in up])
    Z = Um; X = F - (F @ Z) * Z; X /= np.linalg.norm(X); Y = np.cross(Z, X)
    R_L_V = np.stack([X, Y, Z], 1)
    ang_up = np.degrees(np.arccos(np.clip(U @ Um, -1, 1)))
    out = {"R_L_V": R_L_V.tolist(), "forward_in_L": F.tolist(), "up_in_L": Um.tolist(),
           "lidar_height_above_road_m": float(h), "n_ground_fits": len(up),
           "up_scatter_deg_median": float(np.median(ang_up)),
           "forward_up_angle_deg": float(np.degrees(np.arccos(F @ Um))),
           "note": "V = vehicle axes (x forward along travel, y left, z up normal to the road), origin at the LiDAR"}
    log(f"vehicle axes: forward {np.round(F, 4).tolist()} up {np.round(Um, 4).tolist()}, LiDAR {h:.2f} m above road")
    return out
