"""Loading for the RGB solve: KLT tracks, LO trajectories, LiDAR maps (verbatim numerics of
/hdd/DM_calib/lidar_odo/data.py; the fixed directories became a Workspace)."""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from ..lo.lotraj import LOTraj, voxel_down


def load_traj(ws, seg: str, kind: str = "ref") -> LOTraj:
    return LOTraj.load(ws.lo(kind, seg))


def load_tracks(ws, seg: str, cam: str, min_len: int = 12, step: int = 3,
                max_tracks: int = 4000, max_obs: int = 40, rng=None, t_range=None, seg_span_s=None,
                with_q: bool = False):
    """KLT tracks of one camera: (track id, frame ns, uv) subsampled every `step` frames.
    Tracks shorter than min_len frames dropped; at most max_tracks, longest first (ties random).
    seg_span_s: after the selection, split each track into round(duration / seg_span_s) equal-time pieces that
    become separate landmarks (limits the KLT drift a landmark has to absorb; None = off, ids unchanged).
    with_q: also return the corner strength obs_q of the anchored tracker (NaN when the file has none)."""
    rng = rng or np.random.default_rng(0)
    z = np.load(ws.tracks(seg, cam))
    fns, of, ot, xy = z["frame_ns"], z["obs_frame"], z["obs_track"], z["obs_xy"].astype(np.float64)
    oq = z["obs_q"].astype(np.float64) if (with_q and "obs_q" in z.files) else np.full(len(ot), np.nan)
    if t_range is not None:
        k = (fns[of] >= t_range[0]) & (fns[of] <= t_range[1])
        of, ot, xy, oq = of[k], ot[k], xy[k], oq[k]
    o = np.lexsort((of, ot))
    of, ot, xy, oq = of[o], ot[o], xy[o], oq[o]
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
    tid, tns = ot[keep_idx], fns[of[keep_idx]]
    if seg_span_s and len(tid):
        _, first, inv = np.unique(tid, return_index=True, return_inverse=True)
        last = np.r_[first[1:], len(tid)] - 1
        ts, te = tns[first].astype(np.float64), tns[last].astype(np.float64)
        npc = np.maximum(1, np.round((te - ts) * 1e-9 / seg_span_s)).astype(np.int64)
        frac = (tns - ts[inv]) / np.maximum(te - ts, 1.0)[inv]
        piece = np.minimum((frac * npc[inv]).astype(np.int64), npc[inv] - 1)
        tid = tid.astype(np.int64) * 64 + piece
    if with_q:
        return tid, tns, xy[keep_idx], oq[keep_idx]
    return tid, tns, xy[keep_idx]


class SegMap:
    """LiDAR map of a window for landmark-to-plane ties, from the cached deskewed sweeps
    (5 cm voxels, refined odometry). A landmark is compared with the map of the sweeps within
    [t_mid - back, t_mid + fwd] of its observations: the road 2-9 m ahead of a camera was scanned by
    the LiDAR ~1-2 s EARLIER, from further back."""

    CELL = 0.5          # candidate cells (m); a multiple of the 5 cm voxel and >= the 0.35 m radius

    def __init__(self, map_path, back_s: float = 3.0, fwd_s: float = 1.5, bin_s: float = 1.0):
        from ..fastops import load_npz_mmap
        z = load_npz_mmap(map_path)          # memory-mapped: shared page cache, no per-solve copy
        self.P, self.start, self.hdr = z["P"], np.asarray(z["start"]), np.asarray(z["hdr"])
        self.back, self.fwd, self.bin = int(back_s * 1e9), int(fwd_s * 1e9), int(bin_s * 1e9)
        # Candidate points of a time bin = the map points in the 0.5 m cells around the landmarks.
        # lidar_odo/data.py used 1 m cells (27 m^3 around each landmark, ~40 % of the bin); 0.5 m cells
        # still contain every point within the 0.35 m search radius and consist of whole 5 cm voxels, so
        # the voxel representatives and the 16-NN within 0.35 m are unchanged, at ~1/8 of the points.
        # Cell index (cached next to the map, computed once per window): the points sorted by cell key
        # (stable) and, per distinct cell, its range in that order. Selection = a few range lookups.
        self.korder, self.ukeys, self.ustart = self._cell_index(map_path)

    def _cell_index(self, map_path):
        import os
        from pathlib import Path
        side = Path(str(map_path).replace(".npz", f".cells{int(self.CELL * 100)}.npz"))
        if side.exists():
            try:
                from ..fastops import load_npz_mmap
                z = load_npz_mmap(side)
                if int(z["n"]) == len(self.P):
                    return z["korder"], np.asarray(z["ukeys"]), np.asarray(z["ustart"])
            except Exception:  # noqa
                pass
        key = self._key(self.P, self.CELL)
        korder = np.argsort(key, kind="stable").astype(np.int32 if len(key) < 2**31 else np.int64)
        ks = key[korder]
        first = np.flatnonzero(np.r_[True, ks[1:] != ks[:-1]]) if len(ks) else np.zeros(0, np.int64)
        ukeys, ustart = ks[first], np.r_[first, len(ks)].astype(np.int64)
        try:
            tmp = side.with_name(side.stem + ".tmp.npz")
            np.savez(tmp, korder=korder, ukeys=ukeys, ustart=ustart, n=len(self.P))
            os.replace(tmp, side)
        except OSError:
            pass
        return korder, ukeys, ustart

    def _in_cells(self, ckey, s0, s1):
        """Indices i in [s0, s1) with key[i] in ckey, ascending (== np.flatnonzero(np.isin(key[s0:s1], ckey)) + s0)."""
        j = np.searchsorted(self.ukeys, ckey)
        jc = np.minimum(j, len(self.ukeys) - 1)
        hit = (j < len(self.ukeys)) & (self.ukeys[jc] == ckey) if len(self.ukeys) else np.zeros(len(ckey), bool)
        lo = self.ustart[jc[hit]]
        n = self.ustart[jc[hit] + 1] - lo
        if not n.sum():
            return np.zeros(0, np.int64)
        rep = np.repeat(lo - np.cumsum(np.r_[0, n[:-1]]), n) + np.arange(n.sum())
        idx = self.korder[rep].astype(np.int64)
        idx = idx[(idx >= s0) & (idx < s1)]
        idx.sort()
        return idx

    # ---- per time bin: the de-duplicated candidate points, cached between the solve rounds
    _cache_bytes = [0]                    # all SegMaps of the process
    CACHE_GB = float(__import__("os").environ.get("NONTARGET_TIE_CACHE_GB", "3"))

    CACHE_MARGIN = int(__import__("os").environ.get("NONTARGET_TIE_CACHE_MARGIN", "1"))
    stats = {"hit": 0, "miss": 0}

    def _bin_points(self, bb, s0, s1, ckey, kl=None):
        """voxel_down(map points of the cells `ckey` in sweeps [s0, s1), 5 cm) as float64 - exactly the
        original selection. The first call of a bin keeps (within a process-wide memory budget) the first
        point of every (5 cm voxel, 0.5 m cell) part of the cells within CACHE_MARGIN more cells of the
        landmarks; a later call whose cells are covered (the landmarks move by mm between the solve rounds,
        outliers are removed) filters these by cell and keeps the first per voxel: the same points in the
        same order as the selection from the map (a voxel is split between two cells only for points
        exactly on a cell boundary, handled by keeping both parts), without reading the map again."""
        cache = self.__dict__.setdefault("_bins", {})
        c = cache.get(bb)
        if c is not None:
            cells, P32 = c
            j = np.searchsorted(cells, ckey)
            if len(ckey) and j.max() < len(cells) and np.array_equal(cells[j], ckey):
                self.stats["hit"] += 1
                return self._from_parts(P32, self._key(P32, self.CELL), ckey)
        self.stats["miss"] += 1
        if self._cache_bytes[0] >= self.CACHE_GB * 1e9 or kl is None:
            return voxel_down(self.P[self._in_cells(ckey, s0, s1)].astype(np.float64), 0.05)
        # cache the cells within CACHE_MARGIN more cells of the landmarks (they move between rounds)
        r = 1 + self.CACHE_MARGIN
        off = np.array([(i, j, l) for i in range(-r, r + 1) for j in range(-r, r + 1) for l in range(-r, r + 1)])
        cells = (kl[:, None, :] + off[None]).reshape(-1, 3) + (1 << 20)
        cells = np.unique((cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2])
        raw = self.P[self._in_cells(cells, s0, s1)]
        if len(raw):
            from ..fastops import first_index
            vk = np.floor(raw.astype(np.float64) / 0.05).astype(np.int64) + (1 << 20)
            vk = (vk[:, 0] << 42) | (vk[:, 1] << 21) | vk[:, 2]
            ckr = self._key(raw, self.CELL)
            rep = first_index(vk)
            keep = rep == np.arange(len(rep))
            mixed = np.flatnonzero(ckr != ckr[rep])            # points in another cell than their voxel's first
            if len(mixed):
                pair = np.stack([vk[mixed], ckr[mixed]], 1)
                _, fi = np.unique(pair, axis=0, return_index=True)
                keep[mixed[fi]] = True
            P32 = np.ascontiguousarray(raw[keep])
            cache[bb] = (cells, P32)
            self._cache_bytes[0] += P32.nbytes + 8 * len(cells)
            return self._from_parts(P32, ckr[keep], ckey)
        return np.zeros((0, 3))

    @staticmethod
    def _from_parts(P32, ck, ckey):
        jj = np.searchsorted(ckey, ck)
        m = (jj < len(ckey)) & (ckey[np.minimum(jj, len(ckey) - 1)] == ck)
        return voxel_down(P32[m].astype(np.float64), 0.05)

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
            P = self._bin_points(int(bb), s0, s1, ckey, kl)
            if len(P) < k:        # (was tested before the de-dup; with fewer than k voxels no landmark
                continue          #  gets k neighbours either, so the output is the same)
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
