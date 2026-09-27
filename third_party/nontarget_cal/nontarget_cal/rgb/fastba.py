"""Fused (numba) kernels for the rig bundle adjustment of rgb/rigba.py.

Same model and formulas as rigba.project / kb_project / accumulate / cost_terms, computed per
observation in one pass instead of ~40 batched torch operations on (N, 2, 13) tensors: no
intermediate arrays, all cores. The normal-equation blocks are summed per landmark block in a fixed
order (independent of the thread count), so results are deterministic; they differ from the torch
path only by floating-point summation order (~1e-15 relative per block).

Layout: observations are visited landmark by landmark (a stable argsort of the landmark ids, done
once per solve). Blocks of consecutive landmarks run in parallel; everything indexed by a landmark or a
(landmark, camera) pair is written by exactly one block; the per-camera blocks U, bc are summed per
block and reduced in block order.
"""
from __future__ import annotations

import math
import os

os.environ.setdefault("NUMBA_THREADING_LAYER", "omp")    # the TBB layer needs a newer TBB than the system's
import numba as nb  # noqa: E402
import numpy as np
import torch

NP = 13
LM_BLOCK = 1024


def set_threads(n: int | None):
    n = int(n or os.environ.get("OMP_NUM_THREADS", "4"))
    nb.set_num_threads(max(1, min(n, nb.config.NUMBA_NUM_THREADS)))


@nb.njit(cache=True, inline="always")
def _obs(k, cam, lm, uv, pidx, PR, Pp, X, R, c, fx, fy, cx, cy, k1, k2, k3, jac, Jc, Jl, res):
    """One observation: residual (res[0:2]), camera-frame z (res[2]); with jac also Jc (2,13), Jl (2,3)."""
    ci, li, pk = cam[k], lm[k], pidx[k]
    dx0 = X[li, 0] - Pp[pk, 0]
    dx1 = X[li, 1] - Pp[pk, 1]
    dx2 = X[li, 2] - Pp[pk, 2]
    # XL = RWL^T (X - pWL); y = XL - c; Xc = RLC^T y
    y0 = PR[pk, 0, 0] * dx0 + PR[pk, 1, 0] * dx1 + PR[pk, 2, 0] * dx2 - c[ci, 0]
    y1 = PR[pk, 0, 1] * dx0 + PR[pk, 1, 1] * dx1 + PR[pk, 2, 1] * dx2 - c[ci, 1]
    y2 = PR[pk, 0, 2] * dx0 + PR[pk, 1, 2] * dx1 + PR[pk, 2, 2] * dx2 - c[ci, 2]
    x = R[ci, 0, 0] * y0 + R[ci, 1, 0] * y1 + R[ci, 2, 0] * y2
    yy = R[ci, 0, 1] * y0 + R[ci, 1, 1] * y1 + R[ci, 2, 1] * y2
    z = R[ci, 0, 2] * y0 + R[ci, 1, 2] * y1 + R[ci, 2, 2] * y2
    f_x, f_y = fx[ci], fy[ci]
    a1, a2, a3 = k1[ci], k2[ci], k3[ci]
    r2 = x * x + yy * yy
    r = max(math.sqrt(r2), 1e-12)
    th = math.atan2(r, z)
    t2 = th * th
    poly = 1 + t2 * (a1 + t2 * (a2 + t2 * a3))
    thd = th * poly
    s = thd / r
    res[0] = f_x * s * x + cx[ci] - uv[k, 0]
    res[1] = f_y * s * yy + cy[ci] - uv[k, 1]
    res[2] = z
    if not jac:
        return
    dthd_dth = 1 + t2 * (3 * a1 + t2 * (5 * a2 + t2 * 7 * a3))
    rho2 = r2 + z * z
    dth_dr = z / rho2
    dth_dz = -r / rho2
    ds_dr = (dthd_dth * dth_dr) / r - thd / (r * r)
    ds_dx = ds_dr * x / r
    ds_dy = ds_dr * yy / r
    ds_dz = dthd_dth * dth_dz / r
    J00 = f_x * (s + x * ds_dx)
    J01 = f_x * x * ds_dy
    J02 = f_x * x * ds_dz
    J10 = f_y * yy * ds_dx
    J11 = f_y * (s + yy * ds_dy)
    J12 = f_y * yy * ds_dz
    # A = JX @ RLC^T  (2,3): A[i, m] = sum_j JX[i, j] * R[m, j]
    for m in range(3):
        A0 = J00 * R[ci, m, 0] + J01 * R[ci, m, 1] + J02 * R[ci, m, 2]
        A1 = J10 * R[ci, m, 0] + J11 * R[ci, m, 1] + J12 * R[ci, m, 2]
        Jc[0, 3 + m] = -A0
        Jc[1, 3 + m] = -A1
        res[3 + m] = A0          # stash A (row 0) for the skew and Jl products below
        res[6 + m] = A1
    # Jc[:, 0:3] = A @ skew(y); skew(y) = [[0,-y2,y1],[y2,0,-y0],[-y1,y0,0]]
    for i in range(2):
        b = 3 if i == 0 else 6
        A0, A1_, A2 = res[b], res[b + 1], res[b + 2]
        Jc[i, 0] = A1_ * y2 - A2 * y1
        Jc[i, 1] = -A0 * y2 + A2 * y0
        Jc[i, 2] = A0 * y1 - A1_ * y0
        # Jl = A @ RWL^T: Jl[i, m] = sum_j A[i, j] * PR[m, j]
        for m in range(3):
            Jl[i, m] = A0 * PR[pk, m, 0] + A1_ * PR[pk, m, 1] + A2 * PR[pk, m, 2]
    # intrinsics: logF, cx, cy, k1, k2, k3, logA
    Jc[0, 6] = f_x * s * x
    Jc[1, 6] = f_y * s * yy
    Jc[0, 7] = 1.0
    Jc[1, 7] = 0.0
    Jc[0, 8] = 0.0
    Jc[1, 8] = 1.0
    t3 = th ** 3 / r
    t5 = th ** 5 / r
    t7 = th ** 7 / r
    Jc[0, 9] = f_x * t3 * x
    Jc[1, 9] = f_y * t3 * yy
    Jc[0, 10] = f_x * t5 * x
    Jc[1, 10] = f_y * t5 * yy
    Jc[0, 11] = f_x * t7 * x
    Jc[1, 11] = f_y * t7 * yy
    Jc[0, 12] = 0.0
    Jc[1, 12] = f_y * s * yy


@nb.njit(parallel=True, cache=True)
def _accumulate(perm, lm_start, cam, lm, uv, pidx, pair, PR, Pp, X, R, c, fx, fy, cx, cy, k1, k2, k3,
                huber, C, P):
    M = X.shape[0]
    nblk = (M + LM_BLOCK - 1) // LM_BLOCK
    Ub = np.zeros((nblk, C, NP, NP))
    bb = np.zeros((nblk, C, NP))
    V = np.zeros((M, 3, 3))
    bl = np.zeros((M, 3))
    Wp = np.zeros((P, NP, 3))
    for blk in nb.prange(nblk):
        Jc = np.zeros((2, NP))
        Jl = np.zeros((2, 3))
        res = np.zeros(9)
        l0 = blk * LM_BLOCK
        l1 = min(M, l0 + LM_BLOCK)
        for q in range(lm_start[l0], lm_start[l1]):
            k = perm[q]
            _obs(k, cam, lm, uv, pidx, PR, Pp, X, R, c, fx, fy, cx, cy, k1, k2, k3, True, Jc, Jl, res)
            r0, r1 = res[0], res[1]
            e = math.sqrt(r0 * r0 + r1 * r1)
            w = 1.0 if e <= huber else huber / e
            if res[2] <= 0.05:
                w = 0.0
            ci, li, pk = cam[k], lm[k], pair[k]
            for a in range(NP):
                wa0 = w * Jc[0, a]
                wa1 = w * Jc[1, a]
                for b in range(NP):
                    Ub[blk, ci, a, b] += wa0 * Jc[0, b] + wa1 * Jc[1, b]
                bb[blk, ci, a] += wa0 * r0 + wa1 * r1
                for b in range(3):
                    Wp[pk, a, b] += wa0 * Jl[0, b] + wa1 * Jl[1, b]
            for a in range(3):
                wa0 = w * Jl[0, a]
                wa1 = w * Jl[1, a]
                for b in range(3):
                    V[li, a, b] += wa0 * Jl[0, b] + wa1 * Jl[1, b]
                bl[li, a] += wa0 * r0 + wa1 * r1
    U = np.zeros((C, NP, NP))
    bc = np.zeros((C, NP))
    for blk in range(nblk):
        U += Ub[blk]
        bc += bb[blk]
    return U, bc, V, bl, Wp


@nb.njit(parallel=True, cache=True)
def _residuals(cam, lm, uv, pidx, PR, Pp, X, R, c, fx, fy, cx, cy, k1, k2, k3, want_xc):
    N = cam.shape[0]
    r = np.empty((N, 2))
    zc = np.empty(N)
    Xc = np.empty((N if want_xc else 0, 3))
    nchunk = (N + 65535) // 65536
    for ch in nb.prange(nchunk):
        Jc = np.zeros((2, NP))
        Jl = np.zeros((2, 3))
        res = np.zeros(9)
        for k in range(ch * 65536, min(N, (ch + 1) * 65536)):
            _obs(k, cam, lm, uv, pidx, PR, Pp, X, R, c, fx, fy, cx, cy, k1, k2, k3, False, Jc, Jl, res)
            r[k, 0] = res[0]
            r[k, 1] = res[1]
            zc[k] = res[2]
            if want_xc:
                # recompute the full camera-frame point (only the report / outlier paths need it)
                ci, li, pk = cam[k], lm[k], pidx[k]
                d0 = X[li, 0] - Pp[pk, 0]
                d1 = X[li, 1] - Pp[pk, 1]
                d2 = X[li, 2] - Pp[pk, 2]
                y0 = PR[pk, 0, 0] * d0 + PR[pk, 1, 0] * d1 + PR[pk, 2, 0] * d2 - c[ci, 0]
                y1 = PR[pk, 0, 1] * d0 + PR[pk, 1, 1] * d1 + PR[pk, 2, 1] * d2 - c[ci, 1]
                y2 = PR[pk, 0, 2] * d0 + PR[pk, 1, 2] * d1 + PR[pk, 2, 2] * d2 - c[ci, 2]
                for m in range(3):
                    Xc[k, m] = R[ci, 0, m] * y0 + R[ci, 1, m] * y1 + R[ci, 2, m] * y2
    return r, zc, Xc


@nb.njit(parallel=True, cache=True)
def _robust_cost(r, zc, huber):
    N = r.shape[0]
    nchunk = (N + 65535) // 65536
    part = np.zeros(nchunk)
    e = np.empty(N)
    for ch in nb.prange(nchunk):
        s = 0.0
        for k in range(ch * 65536, min(N, (ch + 1) * 65536)):
            ek = math.sqrt(r[k, 0] * r[k, 0] + r[k, 1] * r[k, 1])
            e[k] = ek
            if zc[k] <= 0.05:
                s += 1e4
            elif ek <= huber:
                s += 0.5 * ek * ek
            else:
                s += huber * (ek - 0.5 * huber)
        part[ch] = s
    tot = 0.0
    for ch in range(nchunk):
        tot += part[ch]
    return tot, e


# ----------------------------------------------------------------------------- wrappers (torch in/out)
def _np(t):
    return t.detach().cpu().numpy() if isinstance(t, torch.Tensor) else np.asarray(t)


def _cam_arrays(cams):
    fx, fy, cx, cy, k1, k2, k3 = [np.ascontiguousarray(_np(a)) for a in cams.intr()]
    return np.ascontiguousarray(_np(cams.R)), np.ascontiguousarray(_np(cams.c)), fx, fy, cx, cy, k1, k2, k3


class ObsIndex:
    """Per-solve constant arrays: observations as numpy, and their landmark-major order."""

    def __init__(self, obs, M, order=True):
        self.cam = np.ascontiguousarray(_np(obs.cam))
        self.lm = np.ascontiguousarray(_np(obs.lm))
        self.uv = np.ascontiguousarray(_np(obs.uv))
        self.pidx = np.ascontiguousarray(_np(obs.pidx))
        self.PR = np.ascontiguousarray(_np(obs.PR))
        self.Pp = np.ascontiguousarray(_np(obs.Pp))
        self.n = len(self.cam)
        if not order:
            return
        self.perm = np.argsort(self.lm, kind="stable")
        self.lm_start = np.zeros(M + 1, np.int64)
        np.cumsum(np.bincount(self.lm, minlength=M), out=self.lm_start[1:])


def accumulate(cams, ox: ObsIndex, X, pair_np, P, huber):
    U, bc, V, bl, Wp = _accumulate(ox.perm, ox.lm_start, ox.cam, ox.lm, ox.uv, ox.pidx, pair_np, ox.PR, ox.Pp,
                                   np.ascontiguousarray(_np(X)), *_cam_arrays(cams), float(huber), cams.C, int(P))
    return (torch.from_numpy(U), torch.from_numpy(bc), torch.from_numpy(V), torch.from_numpy(bl), torch.from_numpy(Wp))


def residuals(cams, ox: ObsIndex, X, want_xc=True):
    r, zc, Xc = _residuals(ox.cam, ox.lm, ox.uv, ox.pidx, ox.PR, ox.Pp, np.ascontiguousarray(_np(X)),
                           *_cam_arrays(cams), bool(want_xc))
    return r, zc, Xc


def robust_cost(cams, ox: ObsIndex, X, huber):
    r, zc, _ = residuals(cams, ox, X, want_xc=False)
    c, e = _robust_cost(r, zc, float(huber))
    return c, torch.from_numpy(e)
