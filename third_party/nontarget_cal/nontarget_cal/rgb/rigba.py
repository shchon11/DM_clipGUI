"""Rig bundle adjustment with the trajectory FIXED from LiDAR odometry.

Unknowns
  per camera c (NP = 13):  w (3)   rotation perturbation of R_LC (left, LiDAR axes)
                           c (3)   camera centre in the LiDAR frame, metres
                           logF    common scale of (fx, fy)  -> aspect ratio kept
                           cx, cy  principal point, px
                           k1 k2 k3  Kannala-Brandt (OpenCV fisheye) radial terms (k4 = 0)
                           logA    fy/fx aspect change (normally held: square pixels)
  per landmark: X (3), world frame of its segment (the LO world)

Observation  uv = KB( R_LC^T ( R_WL(t)^T (X - p_WL(t)) - c ) )
             with T_WL(t) from the refined LiDAR odometry at the image mid-exposure time.

Optional LiDAR tie: landmark-to-plane residuals n.(X - q) / sigma against thin planar
patches of the LiDAR map of the same segment (built from the same odometry).

Solver: Levenberg-Marquardt, landmarks eliminated by the Schur complement (3x3 blocks),
reduced camera system dense (C*NP). Huber on pixels, Cauchy on planes.
Conventions: T_a_b maps b -> a; L = os_lidar; C = OpenCV optical (x right, y down, z fwd).
"""
from __future__ import annotations

import logging

import numpy as np
import torch

DT = torch.float64
NP = 13
PNAMES = ["wx", "wy", "wz", "cx_L", "cy_L", "cz_L", "logF", "cx", "cy", "k1", "k2", "k3", "logA"]


def skew(v: torch.Tensor) -> torch.Tensor:
    O = torch.zeros(v.shape[:-1] + (3, 3), dtype=v.dtype, device=v.device)
    O[..., 0, 1], O[..., 0, 2] = -v[..., 2], v[..., 1]
    O[..., 1, 0], O[..., 1, 2] = v[..., 2], -v[..., 0]
    O[..., 2, 0], O[..., 2, 1] = -v[..., 1], v[..., 0]
    return O


def so3_exp(w: torch.Tensor) -> torch.Tensor:
    th = w.norm(dim=-1, keepdim=True).clamp_min(1e-15)
    k = w / th
    K = skew(k)
    th = th[..., None]
    I = torch.eye(3, dtype=w.dtype, device=w.device).expand(K.shape)
    return I + torch.sin(th) * K + (1 - torch.cos(th)) * (K @ K)


def kb_project(Xc, fx, fy, cx, cy, k1, k2, k3, jac: bool = True):
    """KB projection and Jacobians. Xc (N,3); intrinsics (N,) each.
    Returns uv (N,2), J_X (N,2,3), J_i (N,2,7) wrt [logF, cx, cy, k1, k2, k3, logA(fy only)]."""
    x, y, z = Xc[:, 0], Xc[:, 1], Xc[:, 2]
    r2 = x * x + y * y
    r = torch.sqrt(r2).clamp_min(1e-12)
    th = torch.atan2(r, z)
    t2 = th * th
    poly = 1 + t2 * (k1 + t2 * (k2 + t2 * k3))
    thd = th * poly
    s = thd / r
    u = fx * s * x + cx
    v = fy * s * y + cy
    uv = torch.stack([u, v], -1)
    if not jac:
        return uv
    dthd_dth = 1 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * 7 * k3))
    rho2 = r2 + z * z
    dth_dr = z / rho2
    dth_dz = -r / rho2
    # s = thd/r ; ds/dx = (dthd/dth * dth/dr * x/r)/r - thd/r^2 * x/r
    ds_dr = (dthd_dth * dth_dr) / r - thd / (r * r)
    ds_dx = ds_dr * x / r
    ds_dy = ds_dr * y / r
    ds_dz = dthd_dth * dth_dz / r
    JX = torch.zeros(len(x), 2, 3, dtype=Xc.dtype, device=Xc.device)
    JX[:, 0, 0] = fx * (s + x * ds_dx)
    JX[:, 0, 1] = fx * x * ds_dy
    JX[:, 0, 2] = fx * x * ds_dz
    JX[:, 1, 0] = fy * y * ds_dx
    JX[:, 1, 1] = fy * (s + y * ds_dy)
    JX[:, 1, 2] = fy * y * ds_dz
    Ji = torch.zeros(len(x), 2, 7, dtype=Xc.dtype, device=Xc.device)
    Ji[:, 1, 6] = fy * s * y
    Ji[:, 0, 0] = fx * s * x
    Ji[:, 1, 0] = fy * s * y
    Ji[:, 0, 1] = 1.0
    Ji[:, 1, 2] = 1.0
    for j, pw in enumerate((3, 5, 7)):
        tp = th ** pw / r
        Ji[:, 0, 3 + j] = fx * tp * x
        Ji[:, 1, 3 + j] = fy * tp * y
    return uv, JX, Ji


class Cameras:
    """Per-camera state: R_LC (C,3,3), c (C,3), fx0 fy0 (aspect kept), logF, cx, cy, k1..k3."""

    def __init__(self, names, T_L_C: np.ndarray, intr: np.ndarray):
        self.names = list(names)
        T = torch.tensor(np.asarray(T_L_C), dtype=DT)
        I = torch.tensor(np.asarray(intr), dtype=DT)       # (C,8) fx fy cx cy k1 k2 k3 k4
        self.R = T[:, :3, :3].clone()
        self.c = T[:, :3, 3].clone()
        self.fx0, self.fy0 = I[:, 0].clone(), I[:, 1].clone()
        self.logF = torch.zeros(len(names), dtype=DT)
        self.logA = torch.zeros(len(names), dtype=DT)
        self.cx, self.cy = I[:, 2].clone(), I[:, 3].clone()
        self.k = I[:, 4:7].clone()
        self.theta0 = self.vector().clone()

    @property
    def C(self):
        return len(self.names)

    def vector(self) -> torch.Tensor:
        """Parameter values in the NP layout (rotation part = 0: perturbations are relative)."""
        v = torch.zeros(self.C, NP, dtype=DT)
        v[:, 3:6] = self.c
        v[:, 6] = self.logF
        v[:, 7], v[:, 8] = self.cx, self.cy
        v[:, 9:12] = self.k
        v[:, 12] = self.logA
        return v

    def intr(self):
        F = torch.exp(self.logF)
        return self.fx0 * F, self.fy0 * F * torch.exp(self.logA), self.cx, self.cy, self.k[:, 0], self.k[:, 1], self.k[:, 2]

    def copy(self) -> "Cameras":
        o = Cameras.__new__(Cameras)
        o.names = list(self.names)
        for k in ("R", "c", "fx0", "fy0", "logF", "logA", "cx", "cy", "k", "theta0"):
            setattr(o, k, getattr(self, k).clone())
        return o

    def apply(self, d: torch.Tensor) -> None:
        self.R = so3_exp(d[:, 0:3]) @ self.R
        self.c = self.c + d[:, 3:6]
        self.logF = self.logF + d[:, 6]
        self.cx = self.cx + d[:, 7]
        self.cy = self.cy + d[:, 8]
        self.k = self.k + d[:, 9:12]
        self.logA = self.logA + d[:, 12]

    def T_L_C(self) -> np.ndarray:
        T = np.tile(np.eye(4), (self.C, 1, 1))
        T[:, :3, :3] = self.R.cpu().numpy()
        T[:, :3, 3] = self.c.cpu().numpy()
        return T

    def intr_array(self) -> np.ndarray:
        fx, fy, cx, cy, k1, k2, k3 = self.intr()
        return torch.stack([fx, fy, cx, cy, k1, k2, k3, torch.zeros_like(fx)], -1).cpu().numpy()


class Obs:
    """Observations: cam (N), lm (N), uv (N,2) and a pose index pidx (N) into a shared table
    of LiDAR poses PR (K,3,3), Pp (K,3) (one per camera frame): RWL/pWL are gathered lazily."""

    def __init__(self, cam, lm, uv, pidx, PR, Pp):
        self.cam = torch.as_tensor(cam, dtype=torch.long)
        self.lm = torch.as_tensor(lm, dtype=torch.long)
        self.uv = torch.as_tensor(uv, dtype=DT)
        self.pidx = torch.as_tensor(pidx, dtype=torch.long)
        self.PR = torch.as_tensor(PR, dtype=DT)
        self.Pp = torch.as_tensor(Pp, dtype=DT)

    @property
    def RWL(self):
        return self.PR[self.pidx]

    @property
    def pWL(self):
        return self.Pp[self.pidx]

    def __len__(self):
        return len(self.cam)

    def subset(self, m) -> "Obs":
        m = torch.as_tensor(m)
        return Obs(self.cam[m], self.lm[m], self.uv[m], self.pidx[m], self.PR, self.Pp)

    def chunk(self, sl) -> "Obs":
        return Obs(self.cam[sl], self.lm[sl], self.uv[sl], self.pidx[sl], self.PR, self.Pp)


def project(cams: Cameras, obs: Obs, X: torch.Tensor, jac: bool = True, chunk=None):
    """Residuals (N,2) and, if jac, J_c (N,2,NP), J_l (N,2,3)."""
    ci, li = obs.cam, obs.lm
    Xw = X[li]
    XL = torch.einsum("nji,nj->ni", obs.RWL, Xw - obs.pWL)
    y = XL - cams.c[ci]
    RLC = cams.R[ci]
    Xc = torch.einsum("nji,nj->ni", RLC, y)
    fx, fy, cx, cy, k1, k2, k3 = [a[ci] for a in cams.intr()]
    if not jac:
        uv = kb_project(Xc, fx, fy, cx, cy, k1, k2, k3, jac=False)
        return uv - obs.uv, Xc
    uv, JX, Ji = kb_project(Xc, fx, fy, cx, cy, k1, k2, k3)
    RLCt = RLC.transpose(1, 2)
    dXc_dX = RLCt @ obs.RWL.transpose(1, 2)
    Jl = JX @ dXc_dX
    Jc = torch.zeros(len(ci), 2, NP, dtype=DT)
    Jc[:, :, 0:3] = JX @ (RLCt @ skew(y))
    Jc[:, :, 3:6] = -(JX @ RLCt)
    Jc[:, :, 6:13] = Ji
    return uv - obs.uv, Jc, Jl, Xc


class Ties:
    """Landmark-to-plane constraints: lm (P), n (P,3), q (P,3), sigma (m)."""

    def __init__(self, lm, n, q, sigma=0.03, scale=None):
        self.lm = torch.as_tensor(lm, dtype=torch.long)
        self.n = torch.as_tensor(n, dtype=DT)
        self.q = torch.as_tensor(q, dtype=DT)
        self.sigma = sigma
        self.scale = scale or 2.0 * sigma

    def __len__(self):
        return len(self.lm)


def huber_w(e, d):
    a = e.abs()
    return torch.where(a <= d, torch.ones_like(a), d / a)


def project_all(cams, obs, X, chunk=1_000_000):
    """Residuals and camera-frame points for all observations, chunked (no Jacobians)."""
    R, Q = [], []
    for s0 in range(0, len(obs), chunk):
        r, xc = project(cams, obs.chunk(slice(s0, s0 + chunk)), X, jac=False)
        R.append(r); Q.append(xc)
    return torch.cat(R), torch.cat(Q)


LIN_PRIOR = []      # list of (cam, axis(3), target, sigma): prior on axis . c_cam (profiles)


_LAST_E = [None]      # residual norms of the last cost evaluation (reused for the progress log)


def cost_terms(cams, obs, X, ties, prior_sig, huber):
    r, Xc = project_all(cams, obs, X)
    e = r.norm(dim=1)
    _LAST_E[0] = e
    rho = torch.where(e <= huber, 0.5 * e * e, huber * (e - 0.5 * huber))
    bad = Xc[:, 2] <= 0.05
    rho = torch.where(bad, torch.full_like(rho, 1e4), rho)
    c = rho.sum()
    if ties is not None and len(ties):
        d = torch.einsum("ni,ni->n", ties.n, X[ties.lm] - ties.q) / ties.sigma
        s = ties.scale / ties.sigma
        c = c + 0.5 * (s * s * torch.log1p((d / s) ** 2)).sum()
    dv = cams.vector() - cams.theta0
    c = c + 0.5 * ((dv / prior_sig) ** 2)[torch.isfinite(prior_sig) & (prior_sig > 0)].sum()
    for (ci, ax, tgt, sg) in LIN_PRIOR:
        c = c + 0.5 * ((float(cams.c[ci] @ torch.as_tensor(ax, dtype=DT)) - tgt) / sg) ** 2
    return float(c)


def accumulate(cams, obs, X, ties, prior_sig, free, huber, pair, P, chunk=200_000):
    """Normal-equation blocks, chunked over observations: U (C,NP,NP), bc (C,NP),
    V (M,3,3), bl (M,3), Wp (P,NP,3). Robust (Huber) weights; priors added to U, bc."""
    C, M = cams.C, len(X)
    U = torch.zeros(C, NP, NP, dtype=DT)
    bc = torch.zeros(C, NP, dtype=DT)
    V = torch.zeros(M, 3, 3, dtype=DT)
    bl = torch.zeros(M, 3, dtype=DT)
    Wp = torch.zeros(P, NP, 3, dtype=DT)
    for s0 in range(0, len(obs), chunk):
        sl = slice(s0, min(len(obs), s0 + chunk))
        o = obs.chunk(sl)
        r, Jc, Jl, Xc = project(cams, o, X)
        e = r.norm(dim=1)
        w = huber_w(e, huber)
        w = torch.where(Xc[:, 2] <= 0.05, torch.zeros_like(w), w)
        wJc = Jc * w[:, None, None]
        wJl = Jl * w[:, None, None]
        U.index_add_(0, o.cam, wJc.transpose(1, 2) @ Jc)
        bc.index_add_(0, o.cam, torch.einsum("nki,nk->ni", wJc, r))
        V.index_add_(0, o.lm, wJl.transpose(1, 2) @ Jl)
        bl.index_add_(0, o.lm, torch.einsum("nki,nk->ni", wJl, r))
        Wp.index_add_(0, pair[sl], wJc.transpose(1, 2) @ Jl)
        del r, Jc, Jl, Xc, wJc, wJl
    if ties is not None and len(ties):
        d = torch.einsum("ni,ni->n", ties.n, X[ties.lm] - ties.q)
        wt = 1.0 / (1.0 + (d / ties.scale) ** 2) / ties.sigma ** 2
        V.index_add_(0, ties.lm, wt[:, None, None] * ties.n[:, :, None] * ties.n[:, None, :])
        bl.index_add_(0, ties.lm, (wt * d)[:, None] * ties.n)
    has_prior = torch.isfinite(prior_sig) & (prior_sig > 0) & free
    ps = torch.where(has_prior, prior_sig, torch.full_like(prior_sig, np.inf))
    dv = cams.vector() - cams.theta0
    U = U + torch.diag_embed(1.0 / ps ** 2)
    bc = bc + dv / ps ** 2
    for (ci, ax, tgt, sg) in LIN_PRIOR:
        a_ = torch.as_tensor(ax, dtype=DT)
        U[ci, 3:6, 3:6] += torch.outer(a_, a_) / sg ** 2
        bc[ci, 3:6] += a_ * (float(cams.c[ci] @ a_) - tgt) / sg ** 2
    return U, bc, V, bl, Wp


def inv3(A: torch.Tensor) -> torch.Tensor:
    """Batched inverse of 3x3 matrices. CPU: torch.linalg.inv (the validated numbers). CUDA: the
    closed-form adjugate / determinant (cuSOLVER's batched float64 LU is ~50x slower for 3x3; the
    two agree to ~1e-15 relative for these well-conditioned damped landmark blocks)."""
    if not A.is_cuda:
        return torch.linalg.inv(A)
    a, b, c = A[:, 0, 0], A[:, 0, 1], A[:, 0, 2]
    d, e, f = A[:, 1, 0], A[:, 1, 1], A[:, 1, 2]
    g, h, i = A[:, 2, 0], A[:, 2, 1], A[:, 2, 2]
    co = torch.stack([torch.stack([e * i - f * h, c * h - b * i, b * f - c * e], -1),
                      torch.stack([f * g - d * i, a * i - c * g, c * d - a * f], -1),
                      torch.stack([d * h - e * g, b * g - a * h, a * e - b * d], -1)], -2)
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    return co / det[:, None, None]


def reduced(U, V, Wp, p_lm, p_cam, xp, xq, lam=0.0):
    """Schur complement S (C*NP square) with LM damping lam on both blocks; also Vi, Y."""
    C = U.shape[0]
    Ud = U + lam * torch.diag_embed(torch.diagonal(U, dim1=1, dim2=2).clamp_min(1e-9))
    Vd = V + lam * torch.diag_embed(torch.diagonal(V, dim1=1, dim2=2).clamp_min(1e-9)) \
        + 1e-9 * torch.eye(3, dtype=DT)
    Vi = inv3(Vd)
    Y = Wp @ Vi[p_lm]
    S = torch.zeros(C * NP, C * NP, dtype=DT)
    blk = torch.zeros(C, NP, NP, dtype=DT).index_add_(0, p_cam, Y @ Wp.transpose(1, 2))
    for c in range(C):
        S[c * NP:(c + 1) * NP, c * NP:(c + 1) * NP] = Ud[c] - blk[c]
    if len(xp):
        kk = p_cam[xp] * C + p_cam[xq]
        CC = torch.zeros(C * C, NP, NP, dtype=DT)
        for s0 in range(0, len(xp), 500_000):
            sl = slice(s0, s0 + 500_000)
            CC.index_add_(0, kk[sl], Y[xp[sl]] @ Wp[xq[sl]].transpose(1, 2))
        for k in torch.unique(kk).tolist():
            a, b = divmod(k, C)
            S[a * NP:(a + 1) * NP, b * NP:(b + 1) * NP] -= CC[k]
    return S, Vi, Y


def structure(obs, C, M):
    key = obs.lm * C + obs.cam
    ukey, pair = torch.unique(key, return_inverse=True)
    p_lm, p_cam = ukey // C, ukey % C
    order = torch.argsort(p_lm)
    cnt = torch.bincount(p_lm[order], minlength=M)
    multi = torch.nonzero(cnt > 1).flatten()
    xp, xq = [], []
    if len(multi):
        starts = torch.cumsum(cnt, 0) - cnt
        for m in multi.tolist():
            ids = order[starts[m]:starts[m] + cnt[m]]
            a, b = torch.meshgrid(ids, ids, indexing="ij")
            sel = a != b
            xp.append(a[sel])
            xq.append(b[sel])
    xp = torch.cat(xp) if xp else torch.zeros(0, dtype=torch.long)
    xq = torch.cat(xq) if xq else torch.zeros(0, dtype=torch.long)
    return pair, len(ukey), p_lm, p_cam, xp, xq


def solve(cams: Cameras, obs: Obs, X: torch.Tensor, free: torch.Tensor, prior_sig: torch.Tensor,
          ties: Ties | None = None, iters: int = 30, huber: float = 1.5, lam0: float = 1e-3,
          verbose: bool = True, tol: float = 1e-7, want_cov: bool = True, on_iteration=None):
    """LM. free (C,NP) bool; prior_sig (C,NP) (inf = no prior; applies to free params only).
    Rotation priors are not supported (keep them inf). Returns cams, X, info dict.
    on_iteration observes the initial and accepted states; its failures disable only the observer."""
    C, M = cams.C, len(X)
    pair, P, p_lm, p_cam, xp, xq = structure(obs, C, M)
    fidx = torch.nonzero(free.flatten()).flatten()
    lam = lam0
    cost = cost_terms(cams, obs, X, ties, prior_sig, huber)
    e_acc = _LAST_E[0]
    hist = [cost]

    def observe(iteration, accepted):
        nonlocal on_iteration
        if on_iteration is not None:
            try:
                on_iteration(cams, obs, e_acc, cost, iteration, accepted)
            except Exception:
                logging.getLogger(__name__).warning("Visualization observer disabled after an error", exc_info=True)
                on_iteration = None

    observe(0, False)
    rel = 1.0
    dc = torch.zeros(C, NP, dtype=DT)
    for it in range(iters):
        U, bc, V, bl, Wp = accumulate(cams, obs, X, ties, prior_sig, free, huber, pair, P)
        while True:
            S, Vi, Y = reduced(U, V, Wp, p_lm, p_cam, xp, xq, lam)
            rhs = bc - torch.zeros(C, NP, dtype=DT).index_add_(0, p_cam, torch.einsum("pij,pj->pi", Y, bl[p_lm]))
            try:
                dcf = (-torch.linalg.solve(S[fidx][:, fidx], rhs.flatten()[fidx]) if len(fidx)
                       else torch.zeros(0, dtype=DT))
            except RuntimeError:
                lam *= 10
                continue
            dc = torch.zeros(C * NP, dtype=DT)
            dc[fidx] = dcf
            dc = dc.view(C, NP)
            t = torch.zeros(M, 3, dtype=DT).index_add_(0, p_lm, torch.einsum("pij,pi->pj", Wp, dc[p_cam]))
            dl = -torch.einsum("mij,mj->mi", Vi, bl + t)
            nc = cams.copy()
            nc.apply(dc)
            nX = X + dl
            new = cost_terms(nc, obs, nX, ties, prior_sig, huber)
            if new < cost:
                cams, X = nc, nX
                e_acc = _LAST_E[0]
                rel = (cost - new) / cost
                cost = new
                lam = max(lam / 3, 1e-7)
                observe(it + 1, True)
                break
            lam *= 4
            if lam > 1e8:
                rel = 0.0
                break
        hist.append(cost)
        if verbose:
            e = e_acc          # = project_all(cams, obs, X)[0].norm(dim=1) of the accepted state (no recompute)
            print(f"    LM {it:2d}: cost {cost:.1f} lam {lam:.1e} | reproj median {e.median():.3f} px "
                  f"rms(<5px) {e[e < 5].pow(2).mean().sqrt():.3f} | step max rot "
                  f"{np.degrees(float(dc[:, :3].abs().max())):.4f} deg pos {1e3 * dc[:, 3:6].abs().max():.1f} mm "
                  f"logF {dc[:, 6].abs().max():.2e}", flush=True)
        if rel < tol or lam > 1e8:
            break
    info = {"cost": cost, "hist": hist, "fidx": fidx, "cov_free": None}
    if want_cov and len(fidx):
        try:
            U, bc, V, bl, Wp = accumulate(cams, obs, X, ties, prior_sig, free, huber, pair, P)
            S, _, _ = reduced(U, V, Wp, p_lm, p_cam, xp, xq, 0.0)
            info["cov_free"] = torch.linalg.inv(S[fidx][:, fidx])
        except Exception as ex:  # noqa
            print("    covariance failed:", ex)
    return cams, X, info


def triangulate(cams: Cameras, obs: Obs, M: int, unproject_fn, chunk: int = 1_000_000):
    """Least-squares midpoint of all rays of each landmark (chunked over observations).
    Returns X (M,3) and parallax (M,) rad (2 x the largest angle between a ray and the mean ray)."""
    I = torch.eye(3, dtype=DT)
    AA = torch.zeros(M, 3, 3, dtype=DT)
    Ab = torch.zeros(M, 3, dtype=DT)
    dm = torch.zeros(M, 3, dtype=DT)

    def rays(o):
        ci = o.cam
        R_WC = o.RWL @ cams.R[ci]
        d = torch.einsum("nij,nj->ni", R_WC, unproject_fn(o))
        c = torch.einsum("nij,nj->ni", o.RWL, cams.c[ci]) + o.pWL
        return d, c

    for s0 in range(0, len(obs), chunk):
        o = obs.chunk(slice(s0, s0 + chunk))
        d, c = rays(o)
        A = I - d[:, :, None] * d[:, None, :]
        AA.index_add_(0, o.lm, A)
        Ab.index_add_(0, o.lm, torch.einsum("nij,nj->ni", A, c))
        dm.index_add_(0, o.lm, d)
    X = torch.linalg.solve(AA + 1e-9 * I, Ab)
    dm = dm / dm.norm(dim=1, keepdim=True).clamp_min(1e-12)
    par = torch.zeros(M, dtype=DT)
    for s0 in range(0, len(obs), chunk):
        o = obs.chunk(slice(s0, s0 + chunk))
        d, _ = rays(o)
        ang = torch.arccos(torch.einsum("ni,ni->n", d, dm[o.lm]).clamp(-1, 1))
        par.scatter_reduce_(0, o.lm, ang, reduce="amax")
    return X, 2 * par


def kb_unproject(uv, fx, fy, cx, cy, k1, k2, k3):
    mx, my = (uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy
    thd = torch.sqrt(mx * mx + my * my).clamp_min(1e-12)
    th = thd.clone()
    for _ in range(20):
        t2 = th * th
        f = th * (1 + t2 * (k1 + t2 * (k2 + t2 * k3))) - thd
        df = 1 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * 7 * k3))
        th = (th - f / df).clamp(0, 3.1)
    s = torch.sin(th) / thd
    return torch.stack([s * mx, s * my, torch.cos(th)], -1)


def unproject_obs(cams: Cameras):
    def fn(obs: Obs):
        fx, fy, cx, cy, k1, k2, k3 = [a[obs.cam] for a in cams.intr()]
        return kb_unproject(obs.uv, fx, fy, cx, cy, k1, k2, k3)
    return fn
