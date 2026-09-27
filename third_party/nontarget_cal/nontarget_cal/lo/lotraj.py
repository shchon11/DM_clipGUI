"""LiDAR-odometry trajectory: knots T_w_L(tau_k), interpolation, deskew, sweep I/O.

Convention everywhere in /hdd/DM_calib/lidar_odo:
  * T_a_b maps a point in frame b to frame a (p_a = T_a_b @ p_b), metres.
  * L = Ouster `os_lidar` frame. W = the odometry world (= L at the first knot of a segment).
  * Knot k sits at tau_k = header_k + 0.1 s (end of sweep k); a point of sweep k with
    per-point time t (0..0.1 s) was measured at header_k + t, i.e. between knots k-1 and k.
  * Between knots: SLERP for rotation, linear for position (chord-vs-arc error at 20 deg/s
    and 5 m/s over 0.1 s is 2 mm).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

SWEEP_NS = 100_000_000


class LOTraj:
    def __init__(self, tau: np.ndarray, T: np.ndarray):
        o = np.argsort(tau)
        self.tau = np.asarray(tau, np.int64)[o]
        self.T = np.asarray(T, np.float64)[o]
        # first knot extrapolated backwards so sweep 0 can be deskewed too
        d = np.linalg.inv(self.T[0]) @ self.T[1]
        self.tau = np.concatenate([[self.tau[0] - (self.tau[1] - self.tau[0])], self.tau])
        self.T = np.concatenate([[self.T[0] @ np.linalg.inv(d)], self.T])
        self.rv = Rot.from_matrix(self.T[:, :3, :3])

    @classmethod
    def load(cls, path: Path, key: str | None = None) -> "LOTraj":
        z = np.load(path)
        key = key or ("T_w_L" if "T_w_L" in z.files else "T")
        tr = cls.__new__(cls)
        tr.tau = z["tau"].astype(np.int64)
        tr.T = z[key].astype(np.float64)
        if "ext" in z.files and bool(z["ext"]):
            tr.rv = Rot.from_matrix(tr.T[:, :3, :3])
            return tr
        return cls(tr.tau, tr.T)

    def save(self, path: Path, **extra) -> None:
        np.savez(path, tau=self.tau, T_w_L=self.T, ext=True, **extra)

    def _seg(self, t):
        t = np.atleast_1d(np.asarray(t, np.float64))
        i = np.clip(np.searchsorted(self.tau, t) - 1, 0, len(self.tau) - 2)
        a = (t - self.tau[i]) / (self.tau[i + 1] - self.tau[i])
        return i, a

    def R(self, t) -> np.ndarray:
        i, a = self._seg(t)
        R0 = self.T[i, :3, :3]
        dR = Rot.from_matrix(np.einsum("nji,njk->nik", R0, self.T[i + 1, :3, :3])).as_rotvec()
        return np.einsum("nij,njk->nik", R0, Rot.from_rotvec(dR * a[:, None]).as_matrix())

    def p(self, t) -> np.ndarray:
        i, a = self._seg(t)
        return self.T[i, :3, 3] * (1 - a)[:, None] + self.T[i + 1, :3, 3] * a[:, None]

    def Tm(self, t) -> np.ndarray:
        R, p = self.R(t), self.p(t)
        T = np.tile(np.eye(4), (len(R), 1, 1))
        T[:, :3, :3], T[:, :3, 3] = R, p
        return T

    def span(self):
        return int(self.tau[1]), int(self.tau[-1])


def sweep_files(seg: Path):
    files = sorted((Path(seg) / "lidar").glob("*.npy"))
    return files, np.array([int(f.stem) for f in files], np.int64)


_SWEEP_CACHE: dict | None = None      # path -> raw sweep array while a window's LO chain runs (read once)


class sweep_cache:
    """Context manager: inside it, read_sweep() keeps every raw sweep file in memory (~1.1 GB for a 40 s
    window), so KISS, the refinement, the map and the check read each file from disk once, not 4 times."""

    def __enter__(self):
        global _SWEEP_CACHE
        _SWEEP_CACHE = {}
        return self

    def __exit__(self, *a):
        global _SWEEP_CACHE
        _SWEEP_CACHE = None


def read_sweep(path: Path) -> np.ndarray:
    """np.load of a sweep file (cached inside `sweep_cache`; the arrays are never modified by callers)."""
    if _SWEEP_CACHE is None:
        return np.load(path)
    k = str(path)
    s = _SWEEP_CACHE.get(k)
    if s is None:
        s = _SWEEP_CACHE[k] = np.load(path)
    return s


def load_sweep(path: Path, rmin: float = 2.5, rmax: float = 100.0):
    s = read_sweep(path)
    r = np.sqrt(s["x"].astype(np.float64) ** 2 + s["y"] ** 2 + s["z"] ** 2)
    return s[(r > rmin) & (r < rmax)]


def xyz(s) -> np.ndarray:
    return np.stack([s["x"], s["y"], s["z"]], -1).astype(np.float64)


def deskew(s, header_ns: int, traj: LOTraj, shift_ns: int = 0) -> np.ndarray:
    """Every point into W at its own acquisition time header + t (+ shift)."""
    t = header_ns + shift_ns + s["t"].astype(np.float64) * 1e9
    ut, inv = np.unique(t, return_inverse=True)          # ~1024 column times per sweep
    R, p = traj.R(ut), traj.p(ut)
    return np.einsum("nij,nj->ni", R[inv], xyz(s)) + p[inv]


def voxel_down(p: np.ndarray, voxel: float, *extra):
    """First point (in input order) of every voxel, in input order. fastops.first_occurrence = the
    sorted return_index of np.unique (hash table instead of a stable sort; identical indices)."""
    from ..fastops import first_occurrence
    k = np.floor(p / voxel).astype(np.int64) + (1 << 20)
    key = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
    idx = first_occurrence(key)
    return (p[idx], *(e[idx] for e in extra)) if extra else p[idx]


def local_planes(P: np.ndarray, Q: np.ndarray, tree: cKDTree, k: int = 12, rmax: float = 0.5):
    """For query points Q: normal, centroid, thickness, planarity of their k-NN patch in P."""
    d, nn = tree.query(Q, k=k, distance_upper_bound=rmax)
    ok = np.isfinite(d).all(1)
    nb = P[nn[ok]]
    mu = nb.mean(1)
    C = np.einsum("nki,nkj->nij", nb - mu[:, None], nb - mu[:, None]) / k
    w, v = np.linalg.eigh(C)
    n = v[:, :, 0]
    thick = np.sqrt(np.maximum(w[:, 0], 0))
    planar = (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-12)
    return ok, n, mu, thick, planar
