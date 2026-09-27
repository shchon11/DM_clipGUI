"""Fused (numba) kernels for the thermal bundle adjustment of thermal/tba.py.

Same model and formulas as tba.Trajs.pose / obs_time / cam_points / project / residuals / accumulate /
cost / EdgeTerm, computed per observation (or LiDAR edge point) in one pass: SLERP + Catmull-Rom pose,
row time, pinhole + radial projection, the analytic Jacobians and the numerical time derivative (step h).
Sums are formed per landmark block (observations) or per fixed chunk (edge points) in a fixed order, so
the results are deterministic and differ from the torch path only by floating-point summation order.
"""
from __future__ import annotations

import math
import os

os.environ.setdefault("NUMBA_THREADING_LAYER", "omp")
import numba as nb  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

NCP = 13
NJ = NCP + 1            # camera parameters + the (camera, window) time offset
H_IMG = 480.0
LM_BLOCK = 512
CHUNK = 32768


def set_threads(n):
    n = int(n or os.environ.get("OMP_NUM_THREADS", "4"))
    nb.set_num_threads(max(1, min(n, nb.config.NUMBA_NUM_THREADS)))


@nb.njit(cache=True)
def _searchsorted_left(a, x):
    lo, hi = 0, a.shape[0]
    while lo < hi:
        mid = (lo + hi) >> 1
        if a[mid] < x:
            lo = mid + 1
        else:
            hi = mid
    return lo


@nb.njit(cache=True)
def _pose(seg, t, tau, KR, Kp, phi, s0, s1, R, p):
    """R (3,3), p (3,) of segment `seg` at time t (tba.Trajs.pose)."""
    i = _searchsorted_left(tau, t) - 1
    lo, hi = s0[seg], s1[seg] - 2
    i = max(min(i, hi), lo)
    a = (t - tau[i]) / (tau[i + 1] - tau[i])
    w0, w1, w2 = phi[i, 0] * a, phi[i, 1] * a, phi[i, 2] * a
    th = math.sqrt(w0 * w0 + w1 * w1 + w2 * w2)
    # so3_exp
    if th < 1e-12:
        E00, E01, E02 = 1.0, -w2, w1
        E10, E11, E12 = w2, 1.0, -w0
        E20, E21, E22 = -w1, w0, 1.0
    else:
        thc = max(th, 1e-12)
        k0, k1, k2 = w0 / thc, w1 / thc, w2 / thc
        sn, cs = math.sin(th), 1 - math.cos(th)
        # K = skew(k); K@K = k k^T - I (|k| = 1 up to rounding: use the explicit product)
        KK00 = -k2 * k2 - k1 * k1
        KK01 = k1 * k0
        KK02 = k2 * k0
        KK10 = k0 * k1
        KK11 = -k2 * k2 - k0 * k0
        KK12 = k2 * k1
        KK20 = k0 * k2
        KK21 = k1 * k2
        KK22 = -k1 * k1 - k0 * k0
        E00 = 1 + cs * KK00
        E01 = -sn * k2 + cs * KK01
        E02 = sn * k1 + cs * KK02
        E10 = sn * k2 + cs * KK10
        E11 = 1 + cs * KK11
        E12 = -sn * k0 + cs * KK12
        E20 = -sn * k1 + cs * KK20
        E21 = sn * k0 + cs * KK21
        E22 = 1 + cs * KK22
    for r in range(3):
        a0, a1, a2 = KR[i, r, 0], KR[i, r, 1], KR[i, r, 2]
        R[r, 0] = a0 * E00 + a1 * E10 + a2 * E20
        R[r, 1] = a0 * E01 + a1 * E11 + a2 * E21
        R[r, 2] = a0 * E02 + a1 * E12 + a2 * E22
    im = max(i - 1, lo)
    ip = min(i + 2, hi + 1)
    edge = (i == lo) or (i + 2 > hi + 1)
    for m in range(3):
        p0, p1, p2, p3 = Kp[im, m], Kp[i, m], Kp[i + 1, m], Kp[ip, m]
        if edge:
            p[m] = p1 * (1 - a) + p2 * a
        else:
            p[m] = 0.5 * (2 * p1 + (-p0 + p2) * a + (2 * p0 - 5 * p1 + 4 * p2 - p3) * a ** 2
                          + (-p0 + 3 * p1 - 3 * p2 + p3) * a ** 3)


@nb.njit(cache=True)
def _campt(seg, t, Xw, ci, tau, KR, Kp, phi, s0, s1, Rc, cc, R, p, y, Xc):
    _pose(seg, t, tau, KR, Kp, phi, s0, s1, R, p)
    d0, d1, d2 = Xw[0] - p[0], Xw[1] - p[1], Xw[2] - p[2]
    for m in range(3):
        y[m] = R[0, m] * d0 + R[1, m] * d1 + R[2, m] * d2 - cc[ci, m]
    for m in range(3):
        Xc[m] = Rc[ci, 0, m] * y[0] + Rc[ci, 1, m] * y[1] + Rc[ci, 2, m] * y[2]


@nb.njit(cache=True)
def _one(seg, dseg, ci, u_obs, v_obs, tf, Xw, tau, KR, Kp, phi, s0, s1, Rc, cc, fx, fy, cx, cy, k1, k2, rs, dt,
         h, jac, R, p, y, Xc, Xc2, y2, Jp, Jl, out):
    """Residual (out[0:2]) and camera z (out[2]) of one observation; with jac Jp (2,14) and Jl (2,3)."""
    t = tf + dt[ci, dseg] + rs[ci] * (v_obs / H_IMG - 0.5)
    _campt(seg, t, Xw, ci, tau, KR, Kp, phi, s0, s1, Rc, cc, R, p, y, Xc)
    z = Xc[2]
    iz = 1.0 / z
    x, yy = Xc[0] * iz, Xc[1] * iz
    r2 = x * x + yy * yy
    a1, a2 = k1[ci], k2[ci]
    d = 1 + a1 * r2 + a2 * r2 * r2
    f_x, f_y = fx[ci], fy[ci]
    u = f_x * x * d + cx[ci]
    v = f_y * yy * d + cy[ci]
    out[0] = u - u_obs
    out[1] = v - v_obs
    out[2] = z
    out[3] = u
    out[4] = v
    if not jac:
        return
    dd = 2 * a1 + 4 * a2 * r2
    du_dx = f_x * (d + x * dd * x)
    du_dy = f_x * x * dd * yy
    dv_dx = f_y * yy * dd * x
    dv_dy = f_y * (d + yy * dd * yy)
    J00, J01, J02 = du_dx * iz, du_dy * iz, -(du_dx * x + du_dy * yy) * iz
    J10, J11, J12 = dv_dx * iz, dv_dy * iz, -(dv_dx * x + dv_dy * yy) * iz
    # A = JX @ RLC^T
    A00 = J00 * Rc[ci, 0, 0] + J01 * Rc[ci, 0, 1] + J02 * Rc[ci, 0, 2]
    A01 = J00 * Rc[ci, 1, 0] + J01 * Rc[ci, 1, 1] + J02 * Rc[ci, 1, 2]
    A02 = J00 * Rc[ci, 2, 0] + J01 * Rc[ci, 2, 1] + J02 * Rc[ci, 2, 2]
    A10 = J10 * Rc[ci, 0, 0] + J11 * Rc[ci, 0, 1] + J12 * Rc[ci, 0, 2]
    A11 = J10 * Rc[ci, 1, 0] + J11 * Rc[ci, 1, 1] + J12 * Rc[ci, 1, 2]
    A12 = J10 * Rc[ci, 2, 0] + J11 * Rc[ci, 2, 1] + J12 * Rc[ci, 2, 2]
    y0_, y1_, y2_ = y[0], y[1], y[2]
    Jp[0, 0] = A01 * y2_ - A02 * y1_
    Jp[0, 1] = -A00 * y2_ + A02 * y0_
    Jp[0, 2] = A00 * y1_ - A01 * y0_
    Jp[1, 0] = A11 * y2_ - A12 * y1_
    Jp[1, 1] = -A10 * y2_ + A12 * y0_
    Jp[1, 2] = A10 * y1_ - A11 * y0_
    Jp[0, 3], Jp[0, 4], Jp[0, 5] = -A00, -A01, -A02
    Jp[1, 3], Jp[1, 4], Jp[1, 5] = -A10, -A11, -A12
    Jp[0, 6] = f_x * x * d
    Jp[1, 6] = f_y * yy * d
    Jp[0, 7], Jp[1, 7] = 1.0, 0.0
    Jp[0, 8], Jp[1, 8] = 0.0, 1.0
    Jp[0, 9] = f_x * x * r2
    Jp[1, 9] = f_y * yy * r2
    Jp[0, 10] = f_x * x * r2 * r2
    Jp[1, 10] = f_y * yy * r2 * r2
    # Jl = A @ R_WL^T at the observation time
    for m in range(3):
        Jl[0, m] = A00 * R[m, 0] + A01 * R[m, 1] + A02 * R[m, 2]
        Jl[1, m] = A10 * R[m, 0] + A11 * R[m, 1] + A12 * R[m, 2]
    # time: numerical derivative of the camera-frame point along the trajectory
    _campt(seg, t + h, Xw, ci, tau, KR, Kp, phi, s0, s1, Rc, cc, R, p, y2, Xc2)
    e0, e1, e2 = (Xc2[0] - Xc[0]) / h, (Xc2[1] - Xc[1]) / h, (Xc2[2] - Xc[2]) / h
    g0 = J00 * e0 + J01 * e1 + J02 * e2
    g1 = J10 * e0 + J11 * e1 + J12 * e2
    row = v_obs / H_IMG - 0.5
    Jp[0, 11] = g0 * row
    Jp[1, 11] = g1 * row
    Jp[0, 12] = 0.0
    Jp[1, 12] = v - cy[ci]
    Jp[0, 13] = g0
    Jp[1, 13] = g1


@nb.njit(parallel=True, cache=True)
def _acc_obs(perm, lm_start, cam, seg, dseg, lm, uv, tf, pair, X, tau, KR, Kp, phi, s0, s1, Rc, cc,
             fx, fy, cx, cy, k1, k2, rs, dt, h, huber, S, G, P):
    M = X.shape[0]
    nblk = (M + LM_BLOCK - 1) // LM_BLOCK
    Ub = np.zeros((nblk, G, NJ, NJ))
    bb = np.zeros((nblk, G, NJ))
    V = np.zeros((M, 3, 3))
    bl = np.zeros((M, 3))
    Wp = np.zeros((P, NJ, 3))
    for blk in nb.prange(nblk):
        R = np.empty((3, 3)); p = np.empty(3); y = np.empty(3); Xc = np.empty(3); Xc2 = np.empty(3); y2 = np.empty(3)
        Jp = np.zeros((2, NJ)); Jl = np.zeros((2, 3)); out = np.empty(5)
        l0 = blk * LM_BLOCK
        l1 = min(M, l0 + LM_BLOCK)
        for q in range(lm_start[l0], lm_start[l1]):
            k = perm[q]
            ci, li = cam[k], lm[k]
            _one(seg[k], dseg[k], ci, uv[k, 0], uv[k, 1], tf[k], X[li], tau, KR, Kp, phi, s0, s1, Rc, cc,
                 fx, fy, cx, cy, k1, k2, rs, dt, h, True, R, p, y, Xc, Xc2, y2, Jp, Jl, out)
            r0, r1 = out[0], out[1]
            e = math.sqrt(r0 * r0 + r1 * r1)
            w = 1.0 if e <= huber else huber / e
            if out[2] <= 0.2:
                w = 0.0
            g = ci * S + dseg[k]
            pk = pair[k]
            for a in range(NJ):
                wa0 = w * Jp[0, a]
                wa1 = w * Jp[1, a]
                for b in range(NJ):
                    Ub[blk, g, a, b] += wa0 * Jp[0, b] + wa1 * Jp[1, b]
                bb[blk, g, a] += wa0 * r0 + wa1 * r1
                for b in range(3):
                    Wp[pk, a, b] += wa0 * Jl[0, b] + wa1 * Jl[1, b]
            for a in range(3):
                wa0 = w * Jl[0, a]
                wa1 = w * Jl[1, a]
                for b in range(3):
                    V[li, a, b] += wa0 * Jl[0, b] + wa1 * Jl[1, b]
                bl[li, a] += wa0 * r0 + wa1 * r1
    Ug = np.zeros((G, NJ, NJ))
    bg = np.zeros((G, NJ))
    for blk in range(nblk):
        Ug += Ub[blk]
        bg += bb[blk]
    return Ug, bg, V, bl, Wp


@nb.njit(parallel=True, cache=True)
def _edges(sel, ecam, eseg, edseg, EX, q, n, tfe, tau, KR, Kp, phi, s0, s1, Rc, cc, fx, fy, cx, cy, k1, k2,
           rs, dt, h, S, G, jac, sigma, cauchy, weight):
    """Edge terms r = n . (pi(X) - q): (cost, Ug, bg) (Ug, bg only with jac)."""
    N = sel.shape[0]
    nch = (N + CHUNK - 1) // CHUNK
    Ub = np.zeros((nch if jac else 0, G, NJ, NJ))
    bb = np.zeros((nch if jac else 0, G, NJ))
    cost = np.zeros(nch)
    ccs = cauchy / sigma
    for ch in nb.prange(nch):
        R = np.empty((3, 3)); p = np.empty(3); y = np.empty(3); Xc = np.empty(3); Xc2 = np.empty(3); y2 = np.empty(3)
        Jp = np.zeros((2, NJ)); Jl = np.zeros((2, 3)); out = np.empty(5); J = np.empty(NJ)
        c_ = 0.0
        for j in range(ch * CHUNK, min(N, (ch + 1) * CHUNK)):
            i = sel[j]
            ci = ecam[i]
            _one(eseg[i], edseg[i], ci, q[j, 0], q[j, 1], tfe[i], EX[i], tau, KR, Kp, phi, s0, s1, Rc, cc,
                 fx, fy, cx, cy, k1, k2, rs, dt, h, jac, R, p, y, Xc, Xc2, y2, Jp, Jl, out)
            r = n[j, 0] * out[0] + n[j, 1] * out[1]
            s = r / sigma
            c_ += weight * 0.5 * ccs * ccs * math.log1p((s / ccs) ** 2)
            if jac:
                w = weight / (1.0 + (s / ccs) ** 2) / sigma ** 2
                g = ci * S + edseg[i]
                for a in range(NJ):
                    J[a] = n[j, 0] * Jp[0, a] + n[j, 1] * Jp[1, a]
                for a in range(NJ):
                    wa = w * J[a]
                    for b in range(NJ):
                        Ub[ch, g, a, b] += wa * J[b]
                    bb[ch, g, a] += wa * r
        cost[ch] = c_
    Ug = np.zeros((G, NJ, NJ))
    bg = np.zeros((G, NJ))
    tot = 0.0
    for ch in range(nch):
        tot += cost[ch]
        if jac:
            Ug += Ub[ch]
            bg += bb[ch]
    return tot, Ug, bg


@nb.njit(parallel=True, cache=True)
def _resid(cam, seg, dseg, lm, uv, tf, X, tau, KR, Kp, phi, s0, s1, Rc, cc, fx, fy, cx, cy, k1, k2, rs, dt,
           want_xc):
    N = cam.shape[0]
    r = np.empty((N, 2))
    zc = np.empty(N)
    XC = np.empty((N if want_xc else 0, 3))
    nch = (N + CHUNK - 1) // CHUNK
    for ch in nb.prange(nch):
        R = np.empty((3, 3)); p = np.empty(3); y = np.empty(3); Xc = np.empty(3); Xc2 = np.empty(3); y2 = np.empty(3)
        Jp = np.zeros((2, NJ)); Jl = np.zeros((2, 3)); out = np.empty(5)
        for k in range(ch * CHUNK, min(N, (ch + 1) * CHUNK)):
            _one(seg[k], dseg[k], cam[k], uv[k, 0], uv[k, 1], tf[k], X[lm[k]], tau, KR, Kp, phi, s0, s1, Rc, cc,
                 fx, fy, cx, cy, k1, k2, rs, dt, 0.0, False, R, p, y, Xc, Xc2, y2, Jp, Jl, out)
            r[k, 0] = out[0]
            r[k, 1] = out[1]
            zc[k] = out[2]
            if want_xc:
                XC[k, 0], XC[k, 1], XC[k, 2] = Xc[0], Xc[1], Xc[2]
    return r, zc, XC


@nb.njit(parallel=True, cache=True)
def _robust(r, zc, huber, zmin):
    N = r.shape[0]
    nch = (N + CHUNK - 1) // CHUNK
    part = np.zeros(nch)
    e = np.empty(N)
    for ch in nb.prange(nch):
        s = 0.0
        for k in range(ch * CHUNK, min(N, (ch + 1) * CHUNK)):
            ek = math.sqrt(r[k, 0] * r[k, 0] + r[k, 1] * r[k, 1])
            e[k] = ek
            if zc[k] <= zmin:
                s += 1e4
            elif ek <= huber:
                s += 0.5 * ek * ek
            else:
                s += huber * (ek - 0.5 * huber)
        part[ch] = s
    tot = 0.0
    for ch in range(nch):
        tot += part[ch]
    return tot, e


@nb.njit(parallel=True, cache=True)
def _edge_r(sel, ecam, eseg, edseg, EX, q, n, tfe, tau, KR, Kp, phi, s0, s1, Rc, cc, fx, fy, cx, cy, k1, k2, rs, dt):
    N = sel.shape[0]
    r = np.empty(N)
    nch = (N + CHUNK - 1) // CHUNK
    for ch in nb.prange(nch):
        R = np.empty((3, 3)); p = np.empty(3); y = np.empty(3); Xc = np.empty(3); Xc2 = np.empty(3); y2 = np.empty(3)
        Jp = np.zeros((2, NJ)); Jl = np.zeros((2, 3)); out = np.empty(5)
        for j in range(ch * CHUNK, min(N, (ch + 1) * CHUNK)):
            i = sel[j]
            _one(eseg[i], edseg[i], ecam[i], q[j, 0], q[j, 1], tfe[i], EX[i], tau, KR, Kp, phi, s0, s1, Rc, cc,
                 fx, fy, cx, cy, k1, k2, rs, dt, 0.0, False, R, p, y, Xc, Xc2, y2, Jp, Jl, out)
            r[j] = n[j, 0] * out[0] + n[j, 1] * out[1]
    return r


@nb.njit(parallel=True, cache=True)
def _predict(ecam, eseg, edseg, EX, tfe, tau, KR, Kp, phi, s0, s1, Rc, cc, fx, fy, cx, cy, k1, k2, rs, dt):
    """EdgeTerm.predict: projection with the row time taken from the predicted row (2 passes from row 240)."""
    N = ecam.shape[0]
    uv = np.empty((N, 2))
    z = np.empty(N)
    nch = (N + CHUNK - 1) // CHUNK
    for ch in nb.prange(nch):
        R = np.empty((3, 3)); p = np.empty(3); y = np.empty(3); Xc = np.empty(3); Xc2 = np.empty(3); y2 = np.empty(3)
        Jp = np.zeros((2, NJ)); Jl = np.zeros((2, 3)); out = np.empty(5)
        for i in range(ch * CHUNK, min(N, (ch + 1) * CHUNK)):
            u, v = 240.0, 240.0
            for _ in range(2):
                _one(eseg[i], edseg[i], ecam[i], u, v, tfe[i], EX[i], tau, KR, Kp, phi, s0, s1, Rc, cc,
                     fx, fy, cx, cy, k1, k2, rs, dt, 0.0, False, R, p, y, Xc, Xc2, y2, Jp, Jl, out)
                u, v = out[3], out[4]
            uv[i, 0], uv[i, 1] = u, v
            z[i] = out[2]
    return uv, z


# ----------------------------------------------------------------------------- wrappers
def _a(t):
    return np.ascontiguousarray(t.detach().cpu().numpy() if isinstance(t, torch.Tensor) else t)


def traj_arrays(trajs):
    if getattr(trajs, "_fast", None) is None:
        trajs._fast = (_a(trajs.tau), _a(trajs.R), _a(trajs.p), _a(trajs.phi), _a(trajs.s0).astype(np.int64),
                       _a(trajs.s1).astype(np.int64))
    return trajs._fast


def cam_arrays(cams):
    fx, fy, cx, cy, k1, k2 = [_a(v) for v in cams.intr()]
    return (_a(cams.R), _a(cams.c), fx, fy, cx, cy, k1, k2, _a(cams.rs), _a(cams.dt))


class ObsIndex:
    def __init__(self, obs, M=None):
        self.cam, self.seg, self.dseg, self.lm = _a(obs.cam), _a(obs.seg), _a(obs.dseg), _a(obs.lm)
        self.uv, self.tf = _a(obs.uv), _a(obs.tf)
        if M is not None:
            self.perm = np.argsort(self.lm, kind="stable")
            self.lm_start = np.zeros(M + 1, np.int64)
            np.cumsum(np.bincount(self.lm, minlength=M), out=self.lm_start[1:])


def accumulate_obs(cams, trajs, ox, X, pair, P, huber, h=2e-3):
    Ug, bg, V, bl, Wp = _acc_obs(ox.perm, ox.lm_start, ox.cam, ox.seg, ox.dseg, ox.lm, ox.uv, ox.tf, pair, _a(X),
                                 *traj_arrays(trajs), *cam_arrays(cams), h, float(huber), cams.S, cams.C * cams.S, int(P))
    return [torch.from_numpy(v) for v in (Ug, bg, V, bl, Wp)]


def edge_arrays(E):
    if getattr(E, "_fast", None) is None or E._fast[0] is not E.sel:
        E._fast = (E.sel, _a(E.sel), _a(E.cam), _a(E.seg), _a(E.dseg), _a(E.X), _a(E.q), _a(E.n), _a(E.tf))
    return E._fast[1:]


def edge_terms(E, cams, trajs, jac, h=2e-3):
    c, Ug, bg = _edges(*edge_arrays(E), *traj_arrays(trajs), *cam_arrays(cams), h, cams.S, cams.C * cams.S,
                       bool(jac), float(E.sigma), float(E.cauchy), float(E.weight))
    return c, torch.from_numpy(Ug), torch.from_numpy(bg)


def residuals(cams, trajs, ox, X, want_xc=True):
    return _resid(ox.cam, ox.seg, ox.dseg, ox.lm, ox.uv, ox.tf, _a(X), *traj_arrays(trajs), *cam_arrays(cams),
                  bool(want_xc))


def robust_cost(cams, trajs, ox, X, huber):
    r, zc, _ = residuals(cams, trajs, ox, X, want_xc=False)
    c, e = _robust(r, zc, float(huber), 0.2)
    return c, torch.from_numpy(e)


def predict(E, cams, trajs):
    uv, z = _predict(_a(E.cam), _a(E.seg), _a(E.dseg), _a(E.X), _a(E.tf), *traj_arrays(trajs), *cam_arrays(cams))
    return torch.from_numpy(uv), torch.from_numpy(z)


def edge_residuals(E, cams, trajs):
    return torch.from_numpy(_edge_r(*edge_arrays(E), *traj_arrays(trajs), *cam_arrays(cams)))
