"""Cache of every sweep of a window deskewed into the LO world (5 cm voxels, float32), for the
landmark-to-plane ties and the thermal edge visibility. Verbatim numerics of lidar_odo/build_maps.py.
Output: P (n,3) f4, start (K+1,), hdr (K,)."""
from __future__ import annotations

import logging
import time
from pathlib import Path

import numpy as np

from ..workspace import atomic_savez
from .lotraj import LOTraj, deskew, load_sweep, sweep_files, voxel_down

_VIZ_WARNING = False
_VIZ_MAX_POINTS = 32_768


def _sample_clouds(clouds, limit: int = _VIZ_MAX_POINTS):
    """Bounded, deterministic copies of already-computed clouds; no random draws."""
    if not clouds:
        return np.empty((0, 3), np.float32)
    clouds = clouds[::max(1, (len(clouds) + limit - 1) // limit)]
    per_cloud = max(1, limit // len(clouds))
    samples = [p[::max(1, (len(p) + per_cloud - 1) // per_cloud)][:per_cloud]
               for p in clouds if len(p)]
    if not samples:
        return np.empty((0, 3), np.float32)
    return np.concatenate(samples).astype(np.float32)[:limit]


def _sample_trajectory(poses):
    step = max(1, (len(poses) + 4095) // 4096)
    return np.asarray(poses[::step], np.float32)[:, :3, 3].copy()


def _emit_map(seg, capture, **progress):
    """Run an observation-only capture only when due; failures cannot reach the solver.

    The KISS world is the first LiDAR pose of this window, not a globally registered
    world. Keep that frame explicit instead of pretending unrelated windows align.
    """
    global _VIZ_WARNING
    try:
        from ..viz import get_viz
        viz = get_viz()
        if not viz.due("map"):
            return
        context = viz.context
        window = str(context.get("window_id") or Path(seg).name)
        bag = context.get("bag_id")
        frame = f"lidar_window:{str(bag) + '/' if bag else ''}{window}"
        data = capture()
        viz.publish({"stage": "lidar_odometry", "window_id": window,
                     "map_frame": frame, "status_text": "실제 LiDAR 주행 복원 · " + progress.get("lo_phase", ""),
                     **progress}, map_data=data, force=True)
    except Exception:
        if not _VIZ_WARNING:
            _VIZ_WARNING = True
            try:
                logging.getLogger(__name__).warning("LO visualization skipped after an error", exc_info=True)
            except Exception:
                pass


def build_map(seg: Path, ref: Path, out: Path, voxel: float = 0.05, rmin: float = 2.5, rmax: float = 80.0,
              log=print) -> dict:
    t0 = time.time()
    tr = LOTraj.load(ref)
    files, hdr = sweep_files(seg)
    a, b = tr.span()
    ok = (hdr >= a) & (hdr + 100_000_000 <= b)
    P, start, H = [], [0], []
    for f, h in zip([f for f, k in zip(files, ok) if k], hdr[ok]):
        s = load_sweep(f, rmin, rmax)
        q = voxel_down(deskew(s, h, tr), voxel).astype(np.float32)
        P.append(q); start.append(start[-1] + len(q)); H.append(h)
        _emit_map(seg, lambda: {"points": _sample_clouds(P),
                               "trajectory": _sample_trajectory(tr.T[tr.tau <= h + 100_000_000])},
                  lo_phase="map", sweep=len(H), total_sweeps=int(ok.sum()))
    atomic_savez(out, P=np.concatenate(P), start=np.array(start), hdr=np.array(H))
    log(f"{Path(seg).name}: map {len(H)} sweeps, {start[-1]} pts, {time.time() - t0:.0f} s")
    return {"sweeps": len(H), "points": int(start[-1])}
