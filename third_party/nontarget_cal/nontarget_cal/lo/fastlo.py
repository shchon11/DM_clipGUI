"""Fused (numba) kernel for one sweep pair of the LiDAR-odometry refinement (lo/refine.py pair_terms).

After the k-NN query (cKDTree, unchanged), everything else of a pair - the 8-neighbour plane of every
source point (mean, covariance, eigen decomposition), the thin-plane tests, the gated point-to-plane
residual, its Cauchy weight and the 24-column Jacobian of the four knots, and the per (source knot,
target knot) 24x24 normal-equation block - runs in one compiled loop that releases the GIL, so the
worker threads of the refinement run truly in parallel. Same formulas; the plane normal comes from
the same LAPACK symmetric eigen solver (its sign does not matter: r and J flip together), the block
sums are formed point by point instead of by BLAS (floating-point summation order only).
"""
from __future__ import annotations

import math
import os

os.environ.setdefault("NUMBA_THREADING_LAYER", "omp")
import numba as nb  # noqa: E402
import numpy as np  # noqa: E402


@nb.njit(nogil=True, cache=True)
def pair_core(P, d, nn, Wt, s_i, s_a, t_i, t_a, Tpos, K, scale, kq):
    """-> (status, cost, r (m,), keys (g,), Hs (g,24,24), gs (g,24)); status 0 = pair unused."""
    n = P.shape[0]
    okc = 0
    ok = np.zeros(n, np.bool_)
    for p_ in range(n):
        f = True
        for k in range(kq):
            if not np.isfinite(d[p_, k]):
                f = False
                break
        ok[p_] = f
        if f:
            okc += 1
    empty_r = np.zeros(0)
    empty_k = np.zeros(0, np.int64)
    if okc < 30:
        return 0, 0.0, empty_r, empty_k, np.zeros((0, 24, 24)), np.zeros((0, 24))
    nrm = np.empty((n, 3))
    mu = np.empty((n, 3))
    good = np.zeros(n, np.bool_)
    ngood = 0
    C = np.empty((3, 3))
    for p_ in range(n):
        if not ok[p_]:
            continue
        m0 = 0.0
        m1 = 0.0
        m2 = 0.0
        for k in range(kq):
            q = nn[p_, k]
            m0 += Wt[q, 0]
            m1 += Wt[q, 1]
            m2 += Wt[q, 2]
        m0 /= kq
        m1 /= kq
        m2 /= kq
        mu[p_, 0], mu[p_, 1], mu[p_, 2] = m0, m1, m2
        for a in range(3):
            for b in range(3):
                C[a, b] = 0.0
        for k in range(kq):
            q = nn[p_, k]
            e0, e1, e2 = Wt[q, 0] - m0, Wt[q, 1] - m1, Wt[q, 2] - m2
            C[0, 0] += e0 * e0
            C[0, 1] += e0 * e1
            C[0, 2] += e0 * e2
            C[1, 1] += e1 * e1
            C[1, 2] += e1 * e2
            C[2, 2] += e2 * e2
        C[0, 0] /= kq
        C[0, 1] /= kq
        C[0, 2] /= kq
        C[1, 1] /= kq
        C[1, 2] /= kq
        C[2, 2] /= kq
        C[1, 0] = C[0, 1]
        C[2, 0] = C[0, 2]
        C[2, 1] = C[1, 2]
        w, v = np.linalg.eigh(C)
        thick = math.sqrt(max(w[0], 0.0))
        planar = (w[1] - w[0]) / max(w[2], 1e-12)
        if thick < 0.03 and planar > 0.6:
            good[p_] = True
            ngood += 1
            nrm[p_, 0], nrm[p_, 1], nrm[p_, 2] = v[0, 0], v[1, 0], v[2, 0]
    if ngood < 30:
        return 0, 0.0, empty_r, empty_k, np.zeros((0, 24, 24)), np.zeros((0, 24))
    # gated residuals, in source-point order
    r = np.empty(ngood)
    sel = np.empty(ngood, np.int64)
    m = 0
    for p_ in range(n):
        if not good[p_]:
            continue
        rr = (nrm[p_, 0] * (P[p_, 0] - mu[p_, 0]) + nrm[p_, 1] * (P[p_, 1] - mu[p_, 1])
              + nrm[p_, 2] * (P[p_, 2] - mu[p_, 2]))
        if abs(rr) < 0.3:
            r[m] = rr
            sel[m] = p_
            m += 1
    r = r[:m]
    sel = sel[:m]
    # groups (source knot, target knot), ascending
    grp = np.empty(m, np.int64)
    for j in range(m):
        grp[j] = s_i[sel[j]] * K + t_i[nn[sel[j], 0]]
    keys = np.unique(grp)
    G = keys.shape[0]
    Hs = np.zeros((G, 24, 24))
    gs = np.zeros((G, 24))
    J = np.empty(24)
    cost = 0.0
    for j in range(m):
        p_ = sel[j]
        rr = r[j]
        u = rr / scale
        wgt = 1.0 / (1.0 + u * u)
        cost += math.log1p(u * u)
        si = s_i[p_]
        sa = s_a[p_]
        ti = t_i[nn[p_, 0]]
        ta = 0.0
        for k in range(kq):
            ta += t_a[nn[p_, k]]
        ta /= kq
        n0, n1, n2 = nrm[p_, 0], nrm[p_, 1], nrm[p_, 2]
        for blk in range(4):
            if blk == 0:
                kn, wt, sgn = si, 1 - sa, 1.0
            elif blk == 1:
                kn, wt, sgn = si + 1, sa, 1.0
            elif blk == 2:
                kn, wt, sgn = ti, 1 - ta, -1.0
            else:
                kn, wt, sgn = ti + 1, ta, -1.0
            if blk < 2:
                x0, x1, x2 = P[p_, 0] - Tpos[kn, 0], P[p_, 1] - Tpos[kn, 1], P[p_, 2] - Tpos[kn, 2]
            else:
                x0, x1, x2 = mu[p_, 0] - Tpos[kn, 0], mu[p_, 1] - Tpos[kn, 1], mu[p_, 2] - Tpos[kn, 2]
            s = sgn * wt
            b = 6 * blk
            J[b + 0] = (x1 * n2 - x2 * n1) * s
            J[b + 1] = (x2 * n0 - x0 * n2) * s
            J[b + 2] = (x0 * n1 - x1 * n0) * s
            J[b + 3] = n0 * s
            J[b + 4] = n1 * s
            J[b + 5] = n2 * s
        gi = np.searchsorted(keys, grp[j])
        for a in range(24):
            wa = J[a] * wgt
            for b in range(24):
                Hs[gi, a, b] += J[b] * wa
            gs[gi, a] += wa * rr
    return 1, cost, r, keys, Hs, gs
