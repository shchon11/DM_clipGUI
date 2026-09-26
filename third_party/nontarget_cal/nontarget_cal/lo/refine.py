"""Stage 2 of the LiDAR odometry: continuous-time multi-sweep point-to-plane refinement.

Verbatim numerics of /hdd/DM_calib/lidar_odo/lo_refine.py; only the command line became a function.

KISS-ICP registers each sweep to a 0.5 m voxel map, point-to-point, once. Here every
sweep is registered to many others at once (offsets 1,2,3,5,8,13,20 sweeps, both roles),
point-to-plane on thin planar patches, with the trajectory continuous in time:

  knots T_k at tau_k = header_k + 0.1 s; a point of sweep k at time header_k + t lies
  between knots k-1 and k (SLERP / linear), so moving a knot moves the END of one sweep
  and the START of the next - the deskew is re-estimated together with the poses.

Unknowns: 6 per knot (rotation about the knot's own centre, world-aligned axes, and a
translation). Gauss-Newton on dense normal equations (6 x 401 unknowns), Cauchy
weights, re-association every iteration. Gauge: knot 1 (= the first sweep's end) held
by a strong prior. A weak constant-velocity prior keeps featureless stretches stable.
"""
from __future__ import annotations

import os
import time
from types import SimpleNamespace
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

from .lotraj import LOTraj, load_sweep, sweep_files, voxel_down, xyz

OFFSETS = (1, 2, 3, 5, 8, 13, 20)
QUERY_WORKERS = int(os.environ.get("NONTARGET_KDTREE_WORKERS", "1"))  # thread start-up dominated small queries at 4
PAIR_THREADS = int(os.environ.get("NONTARGET_LO_THREADS", os.environ.get("OMP_NUM_THREADS", "3")))


def skew(v):
    O = np.zeros(v.shape[:-1] + (3, 3))
    O[..., 0, 1], O[..., 0, 2] = -v[..., 2], v[..., 1]
    O[..., 1, 0], O[..., 1, 2] = v[..., 2], -v[..., 0]
    O[..., 2, 0], O[..., 2, 1] = -v[..., 1], v[..., 0]
    return O


class Sweep:
    """Points of one sweep: body coords, knot pair (a, a+1) and weight alpha."""

    def __init__(self, s, header, tr: LOTraj, voxel):
        p = xyz(s)
        t = header + s["t"].astype(np.float64) * 1e9
        p, t = voxel_down(p, voxel, t)
        i, a = tr._seg(t)
        self.p, self.t, self.i, self.a = p, t, i, a
        self._uniq()

    def _uniq(self):
        """The ~1024 distinct column times of the sweep: the interpolated pose is computed once per time
        (not per point) in world(); same inputs per element, so the same numbers."""
        _, first, self.u_inv = np.unique(self.t, return_index=True, return_inverse=True)
        self.u_i, self.u_a = self.i[first], self.a[first]


def world(sw: Sweep, tr: LOTraj):
    i, a = sw.u_i, sw.u_a
    R0 = tr.T[i, :3, :3]
    dR = Rot.from_matrix(np.einsum("nji,njk->nik", R0, tr.T[i + 1, :3, :3])).as_rotvec()
    R = np.einsum("nij,njk->nik", R0, Rot.from_rotvec(dR * a[:, None]).as_matrix())
    c = tr.T[i, :3, 3] * (1 - a)[:, None] + tr.T[i + 1, :3, 3] * a[:, None]
    return np.einsum("nij,nj->ni", R[sw.u_inv], sw.p) + c[sw.u_inv]


def refine(seg, init, out, iters: int = 6, src_voxel: float = 0.3, tgt_voxel: float = 0.1,
           scale: float = 0.03, cv_sigma: float = 0.05, max_src: int = 3000, offsets=OFFSETS, log=print) -> dict:
    a = SimpleNamespace(seg=Path(seg), init=Path(init), out=Path(out), iters=iters, src_voxel=src_voxel,
                        tgt_voxel=tgt_voxel, scale=scale, cv_sigma=cv_sigma, max_src=max_src, offsets=list(offsets))
    print = lambda *x, **k: log(" ".join(str(y) for y in x))  # noqa: A001,E731  (progress -> run log)
    t_start = time.time()
    tr = LOTraj.load(a.init)
    files, hdr = sweep_files(a.seg)
    keep = (hdr >= tr.tau[0]) & (hdr + 100_000_000 <= tr.tau[-1])
    files, hdr = [f for f, k in zip(files, keep) if k], hdr[keep]
    def prep(fh):
        """Per sweep (thread): target sweep, source sweep and its planarity mask. The random thinning
        of the source points stays sequential below (same random draws, same order)."""
        f, h = fh
        s_ = load_sweep(f, 3.0, 80.0)
        tg = Sweep(s_, h, tr, a.tgt_voxel)
        sw = Sweep(s_, h, tr, a.src_voxel)
        tree = cKDTree(tg.p)
        d, nn = tree.query(sw.p, k=10, distance_upper_bound=0.5)
        ok = np.isfinite(d).all(1)
        nb = tg.p[nn[ok]]
        mu = nb.mean(1)
        w_ = np.linalg.eigvalsh(np.einsum("nki,nkj->nij", nb - mu[:, None], nb - mu[:, None]) / 10)
        pl = np.zeros(len(sw.p), bool)
        pl[ok] = (np.sqrt(np.maximum(w_[:, 0], 0)) < 0.03) & ((w_[:, 1] - w_[:, 0]) / np.maximum(w_[:, 2], 1e-12) > 0.6)
        return tg, sw, pl

    if PAIR_THREADS > 1:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(PAIR_THREADS) as ex:
            prepped = list(ex.map(prep, zip(files, hdr)))
    else:
        prepped = [prep(x) for x in zip(files, hdr)]
    tgt = [x[0] for x in prepped]
    # source points: planar within their own sweep, then thinned to <= max_src per sweep
    rng = np.random.default_rng(0)
    src = []
    for tg, sw, pl in prepped:
        keep = np.flatnonzero(pl)
        if len(keep) > a.max_src:
            keep = np.sort(rng.choice(keep, a.max_src, replace=False))
        sw.p, sw.t, sw.i, sw.a = sw.p[keep], sw.t[keep], sw.i[keep], sw.a[keep]
        sw._uniq()
        src.append(sw)
    raw = prepped
    del raw
    K = len(tr.tau)
    N = len(files)
    print(f"{a.seg.name}: {N} sweeps, {K} knots, src {np.mean([len(s.p) for s in src]):.0f} "
          f"tgt {np.mean([len(s.p) for s in tgt]):.0f} pts/sweep; load {time.time() - t_start:.1f} s", flush=True)
    pairs = [(i, i + o) for o in a.offsets for i in range(N - o)]
    pairs += [(j, i) for (i, j) in pairs]
    for it in range(a.iters):
        t_it = time.time()
        Wt = [world(s, tr) for s in tgt]
        Ws = [world(s, tr) for s in src]
        trees = [cKDTree(w) for w in Wt]
        H = np.zeros((6 * K, 6 * K))
        g = np.zeros(6 * K)
        cost, nres, allr = 0.0, 0, []

        def pair_terms(ij):
            """Residuals of one sweep pair -> (cost, r, [(knots, Hg, gg)]) or None. Pure function of the
            current state, run in worker threads; the accumulation below stays sequential in pair
            order, so the result does not depend on the number of threads."""
            i, j = ij
            P = Ws[i]
            d, nn = trees[j].query(P, k=8, distance_upper_bound=0.6, workers=QUERY_WORKERS)
            ok = np.isfinite(d).all(1)
            if ok.sum() < 30:
                return None
            nb = Wt[j][nn[ok]]
            mu = nb.mean(1)
            C = np.einsum("nki,nkj->nij", nb - mu[:, None], nb - mu[:, None]) / 8
            w_, v_ = np.linalg.eigh(C)
            thick = np.sqrt(np.maximum(w_[:, 0], 0))
            planar = (w_[:, 1] - w_[:, 0]) / np.maximum(w_[:, 2], 1e-12)
            good = (thick < 0.03) & (planar > 0.6)
            if good.sum() < 30:
                return None
            idx = np.flatnonzero(ok)[good]
            n = v_[good, :, 0]
            q = mu[good]
            p = P[idx]
            r = np.einsum("ni,ni->n", n, p - q)
            gate = np.abs(r) < 0.3
            idx, n, q, p, r = idx[gate], n[gate], q[gate], p[gate], r[gate]
            nnj = nn[ok][good][gate]
            # knots and weights: source point, and the target patch (mean of its neighbours' alphas)
            si, sa = src[i].i[idx], src[i].a[idx]
            ti_ = tgt[j].i[nnj[:, 0]]
            ta = tgt[j].a[nnj].mean(1)
            # Cauchy weights
            wgt = 1.0 / (1.0 + (r / a.scale) ** 2)
            c_ = float(np.sum(np.log1p((r / a.scale) ** 2)))
            # Jacobians: dr/d(knot) = +-(weight) * [ ((p - c) x n)^T , n^T ]
            blocks = []
            for kn, wt, pt, sgn in ((si, 1 - sa, p, 1.0), (si + 1, sa, p, 1.0),
                                    (ti_, 1 - ta, q, -1.0), (ti_ + 1, ta, q, -1.0)):
                c = tr.T[kn, :3, 3]
                Jr = np.cross(pt - c, n)          # d/d omega of n.(omega x (p-c)) = ((p-c) x n)
                J = np.concatenate([Jr, n], 1) * (sgn * wt)[:, None]
                blocks.append(J)
            # the four knots of a residual are (si, si+1, ti, ti+1) and only a handful of (si, ti)
            # combinations occur per sweep pair, so each combination's 24x24 block is ONE weighted
            # J^T J (BLAS) scattered into H (was: per-point einsum + np.add.at per 6x6 block).
            # Same sums in a different order: differences at floating-point rounding level.
            Jall = np.concatenate(blocks, 1)            # (n, 24)
            grp = si * K + ti_
            ug, ginv = np.unique(grp, return_inverse=True)
            out = []
            for gi, u in enumerate(ug):
                m = ginv == gi
                Jm = Jall[m]
                x0, t0_ = divmod(int(u), K)
                out.append(((x0, x0 + 1, t0_, t0_ + 1), Jm.T @ (Jm * wgt[m, None]), Jm.T @ (wgt[m] * r[m])))
            return c_, r, out

        if PAIR_THREADS > 1:
            from concurrent.futures import ThreadPoolExecutor
            with ThreadPoolExecutor(PAIR_THREADS) as ex:
                results = ex.map(pair_terms, pairs, chunksize=16)
                results = list(results)
        else:
            results = [pair_terms(ij) for ij in pairs]
        for res_ in results:
            if res_ is None:
                continue
            c_, r, out = res_
            cost += c_
            nres += len(r)
            allr.append(r)
            for kn, Hg, gg in out:
                for ia, ka in enumerate(kn):
                    g[6 * ka:6 * ka + 6] += gg[6 * ia:6 * ia + 6]
                    for ib, kb in enumerate(kn):
                        H[6 * ka:6 * ka + 6, 6 * kb:6 * kb + 6] += Hg[6 * ia:6 * ia + 6, 6 * ib:6 * ib + 6]
        del results
        # priors: gauge on knot 1, constant velocity (second difference) everywhere
        H[6:12, 6:12] += np.eye(6) * 1e8
        sig = a.cv_sigma
        for k in range(1, K - 1):
            # second difference of positions and of rotations (small-angle, rotvecs)
            for (kk, cf) in ((k - 1, 1.0), (k, -2.0), (k + 1, 1.0)):
                for (ll, cg) in ((k - 1, 1.0), (k, -2.0), (k + 1, 1.0)):
                    H[6 * kk + 3:6 * kk + 6, 6 * ll + 3:6 * ll + 6] += np.eye(3) * cf * cg / sig ** 2
                    H[6 * kk:6 * kk + 3, 6 * ll:6 * ll + 3] += np.eye(3) * cf * cg * 100 / sig ** 2
            pm, p0, pp = tr.T[k - 1, :3, 3], tr.T[k, :3, 3], tr.T[k + 1, :3, 3]
            e = pm - 2 * p0 + pp
            rm = Rot.from_matrix(tr.T[k, :3, :3].T @ tr.T[k - 1, :3, :3]).as_rotvec()
            rp = Rot.from_matrix(tr.T[k, :3, :3].T @ tr.T[k + 1, :3, :3]).as_rotvec()
            er = tr.T[k, :3, :3] @ (rm + rp)
            for (kk, cf) in ((k - 1, 1.0), (k, -2.0), (k + 1, 1.0)):
                g[6 * kk + 3:6 * kk + 6] += cf * e / sig ** 2
                g[6 * kk:6 * kk + 3] += cf * er * 100 / sig ** 2
        H[np.diag_indices_from(H)] += 1e-6
        dx = -np.linalg.solve(H, g)
        dx = dx.reshape(K, 6)
        T = tr.T.copy()
        T[:, :3, :3] = np.einsum("nij,njk->nik", Rot.from_rotvec(dx[:, :3]).as_matrix(), T[:, :3, :3])
        T[:, :3, 3] += dx[:, 3:]
        tr.T = T
        tr.rv = Rot.from_matrix(T[:, :3, :3])
        r = np.concatenate(allr)
        mad = 1.4826 * np.median(np.abs(r))
        print(f"  it {it}: {nres} residuals, robust std {1e3 * mad:.2f} mm, cost {cost:.0f}, "
              f"step max rot {np.degrees(np.abs(dx[:, :3]).max()):.4f} deg trans {1e3 * np.abs(dx[:, 3:]).max():.1f} mm "
              f"({time.time() - t_it:.0f} s)", flush=True)
    tmp = a.out.with_name(a.out.stem + ".tmp.npz")
    tr.save(tmp, wall_s=time.time() - t_start)
    os.replace(tmp, a.out)
    print(f"  done in {time.time() - t_start:.0f} s -> {a.out}")
    return {"wall_s": time.time() - t_start, "robust_std_mm_last": 1e3 * mad, "knots": K}
