"""Cache of every sweep of a window deskewed into the LO world (5 cm voxels, float32), for the
landmark-to-plane ties and the thermal edge visibility. Verbatim numerics of lidar_odo/build_maps.py.
Output: P (n,3) f4, start (K+1,), hdr (K,)."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from ..workspace import atomic_savez
from .lotraj import LOTraj, deskew, load_sweep, sweep_files, voxel_down


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
    atomic_savez(out, P=np.concatenate(P), start=np.array(start), hdr=np.array(H))
    log(f"{Path(seg).name}: map {len(H)} sweeps, {start[-1]} pts, {time.time() - t0:.0f} s")
    return {"sweeps": len(H), "points": int(start[-1])}
