"""Stage 1 of the LiDAR odometry: KISS-ICP on one extracted window, per-point deskew.

Verbatim numerics of /hdd/DM_calib/lidar_odo/lo_kiss.py (KISS-ICP v1.3, 0.5 m voxels, 2.5-100 m,
stamp = t / 0.1 s so the pose returned for sweep k is the LiDAR pose at tau_k = header_k + 0.1 s).
Input: SEG/lidar/<header ns>.npy (x y z intensity ring t). Output: tau (ns), header, T_w_L (N,4,4).
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from ..workspace import atomic_savez

SWEEP_S = 0.1


def load(path: Path, rmin: float, rmax: float):
    s = np.load(path)
    p = np.stack([s["x"], s["y"], s["z"]], -1).astype(np.float64)
    r = np.linalg.norm(p, axis=1)
    k = (r > rmin) & (r < rmax)
    return p[k], s["t"][k].astype(np.float64)


def run_kiss(seg: Path, out: Path, voxel: float = 0.5, rmin: float = 2.5, rmax: float = 100.0,
             threads: int = 3, log=print) -> dict:
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP

    seg = Path(seg)
    cfg = KISSConfig()
    cfg.data.max_range, cfg.data.min_range, cfg.data.deskew = rmax, rmin, True
    cfg.mapping.voxel_size = voxel
    cfg.registration.max_num_threads = threads
    odo = KissICP(cfg)
    files = sorted((seg / "lidar").glob("*.npy"))
    if len(files) < 20:
        raise RuntimeError(f"{seg.name}: only {len(files)} LiDAR sweeps")
    hdr = np.array([int(f.stem) for f in files], np.int64)
    poses = []
    t0 = time.time()
    for f in files:
        p, t = load(f, rmin, rmax)
        odo.register_frame(p, t / SWEEP_S)
        poses.append(odo.last_pose.copy())
    wall = time.time() - t0
    tau = hdr + int(SWEEP_S * 1e9)
    atomic_savez(out, tau=tau, header=hdr, T_w_L=np.array(poses), wall_s=wall, voxel=voxel)
    P = np.array(poses)
    d = np.linalg.norm(np.diff(P[:, :3, 3], axis=0), axis=1)
    info = {"sweeps": len(files), "wall_s": wall, "path_m": float(d.sum()), "mean_speed_mps": float(d.mean() / SWEEP_S)}
    log(f"{seg.name}: KISS {len(files)} sweeps in {wall:.1f} s ({1e3 * wall / len(files):.0f} ms/sweep); "
        f"path {d.sum():.1f} m, mean speed {d.mean() / SWEEP_S:.2f} m/s")
    return info
