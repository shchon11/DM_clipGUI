"""Loading for the RGB solve: KLT tracks, LO trajectories, LiDAR maps (verbatim numerics of
/hdd/DM_calib/lidar_odo/data.py; the fixed directories became a Workspace)."""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from ..lo.lotraj import LOTraj, voxel_down


def load_traj(ws, seg: str, kind: str = "ref") -> LOTraj:
    return LOTraj.load(ws.lo(kind, seg))


def load_tracks(ws, seg: str, cam: str, min_len: int = 12, step: int = 3,
                max_tracks: int = 4000, max_obs: int = 40, rng=None, t_range=None):
    """KLT tracks of one camera: (track id, frame ns, uv) subsampled every `step` frames.
    Tracks shorter than min_len frames dropped; at most max_tracks, longest first (ties random)."""
    rng = rng or np.random.default_rng(0)
    z = np.load(ws.tracks(seg, cam))
    fns, of, ot, xy = z["frame_ns"], z["obs_frame"], z["obs_track"], z["obs_xy"].astype(np.float64)
    if t_range is not None:
        k = (fns[of] >= t_range[0]) & (fns[of] <= t_range[1])
        of, ot, xy = of[k], ot[k], xy[k]
    o = np.lexsort((of, ot))
    of, ot, xy = of[o], ot[o], xy[o]
    uid, start, cnt = np.unique(ot, return_index=True, return_counts=True)
    ok = cnt >= min_len
    uid, start, cnt = uid[ok], start[ok], cnt[ok]
    pri = cnt + rng.random(len(cnt))
    sel = np.argsort(-pri)[:max_tracks]
    keep_idx = []
    for s, n in zip(start[sel], cnt[sel]):
        idx = np.arange(s, s + n)[::step]
        if len(idx) > max_obs:
            idx = idx[np.linspace(0, len(idx) - 1, max_obs).astype(int)]
        keep_idx.append(idx)
    keep_idx = np.concatenate(keep_idx) if keep_idx else np.zeros(0, int)
    return ot[keep_idx], fns[of[keep_idx]], xy[keep_idx]


class SegMap:
    """LiDAR map of a window for landmark-to-plane ties, from the cached deskewed sweeps
    (5 cm voxels, refined odometry). A landmark is compared with the map of the sweeps within
    [t_mid - back, t_mid + fwd] of its observations: the road 2-9 m ahead of a camera was scanned by
    the LiDAR ~1-2 s EARLIER, from further back."""

    CELL = 0.5          # candidate cells (m); a multiple of the 5 cm voxel and >= the 0.35 m radius

    def __init__(self, map_path, back_s: float = 3.0, fwd_s: float = 1.5, bin_s: float = 1.0):
        z = np.load(map_path)
        self.P, self.start, self.hdr = z["P"], z["start"], z["hdr"]
        self.back, self.fwd, self.bin = int(back_s * 1e9), int(fwd_s * 1e9), int(bin_s * 1e9)
        # Candidate points of a time bin = the map points in the 0.5 m cells around the landmarks.
        # lidar_odo/data.py used 1 m cells (27 m^3 around each landmark, ~40 % of the bin); 0.5 m cells
        # still contain every point within the 0.35 m search radius and consist of whole 5 cm voxels, so
        # the voxel representatives and the 16-NN within 0.35 m are unchanged, at ~1/8 of the points.
        # The cell keys are sorted once (cached next to the map): selection = a few range lookups.
        self.key = self._key(self.P, self.CELL)
        self.korder = self._korder(map_path)
        self.ksorted = self.key[self.korder]

    def _korder(self, map_path):
        """Stable argsort of the cell keys, cached next to the map (int32, computed once per window)."""
        import os
        from pathlib import Path
        side = Path(str(map_path).replace(".npz", f".korder{int(self.CELL * 100)}.npy"))
        if side.exists():
            try:
                k = np.load(side)
                if len(k) == len(self.key):
                    return k
            except Exception:  # noqa
                pass
        k = np.argsort(self.key, kind="stable").astype(np.int32 if len(self.key) < 2**31 else np.int64)
        try:
            tmp = side.with_name(side.stem + ".tmp.npy")
            np.save(tmp, k)
            os.replace(tmp, side)
        except OSError:
            pass
        return k

    def _in_cells(self, ckey, s0, s1):
        """Indices i in [s0, s1) with key[i] in ckey, ascending (== np.flatnonzero(np.isin(key[s0:s1], ckey)) + s0)."""
        lo = np.searchsorted(self.ksorted, ckey, side="left")
        hi = np.searchsorted(self.ksorted, ckey, side="right")
        n = hi - lo
        if not n.sum():
            return np.zeros(0, np.int64)
        rep = np.repeat(lo - np.cumsum(np.r_[0, n[:-1]]), n) + np.arange(n.sum())
        idx = self.korder[rep].astype(np.int64)
        idx = idx[(idx >= s0) & (idx < s1)]
        idx.sort()
        return idx

    @staticmethod
    def _key(P, cell=1.0):
        k = np.floor(P / cell).astype(np.int64) + (1 << 20)
        return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]

    def associate(self, X: np.ndarray, t_mid: np.ndarray, k: int = 16, radius: float = 0.35,
                  max_thick: float = 0.02, min_planar: float = 0.5, gate: float = 0.15):
        """For landmarks X (world) observed around t_mid (ns): (index, n, q, thickness)."""
        b = ((t_mid - self.hdr[0]) // self.bin).astype(np.int64)
        out_i, out_n, out_q, out_th = [], [], [], []
        off = np.array([(i, j, l) for i in (-1, 0, 1) for j in (-1, 0, 1) for l in (-1, 0, 1)])
        for bb in np.unique(b):
            sel = np.flatnonzero(b == bb)
            tc = self.hdr[0] + bb * self.bin + self.bin // 2
            ks = np.flatnonzero((self.hdr >= tc - self.back) & (self.hdr <= tc + self.fwd))
            if not len(ks):
                continue
            s0, s1 = self.start[ks[0]], self.start[ks[-1] + 1]
            kl = np.floor(X[sel] / self.CELL).astype(np.int64)
            cells = (kl[:, None, :] + off[None]).reshape(-1, 3) + (1 << 20)
            ckey = np.unique((cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2])
            P = self.P[self._in_cells(ckey, s0, s1)].astype(np.float64)
            if len(P) < k:
                continue
            P = voxel_down(P, 0.05)
            tree = cKDTree(P)
            d, nn = tree.query(X[sel], k=k, distance_upper_bound=radius, workers=4)
            ok = np.isfinite(d).all(1)
            if not ok.any():
                continue
            nb = P[nn[ok]]
            mu = nb.mean(1)
            Cv = np.einsum("nki,nkj->nij", nb - mu[:, None], nb - mu[:, None]) / k
            w, v = np.linalg.eigh(Cv)
            th = np.sqrt(np.maximum(w[:, 0], 0))
            pl = (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-12)
            n = v[:, :, 0]
            dist = np.einsum("ni,ni->n", n, X[sel][ok] - mu)
            good = (th < max_thick) & (pl > min_planar) & (np.abs(dist) < gate)
            out_i.append(sel[ok][good])
            out_n.append(n[good])
            out_q.append(mu[good])
            out_th.append(th[good])
        if not out_i:
            z = np.zeros((0, 3))
            return np.zeros(0, int), z, z, np.zeros(0)
        return (np.concatenate(out_i), np.concatenate(out_n), np.concatenate(out_q), np.concatenate(out_th))
