"""Thermal bundle adjustment with the trajectory FIXED from LiDAR odometry (the lidar_odo recipe,
re-written for the A70's pinhole + radial lens, its time offset and its row readout).

Observation (camera c, segment s, landmark X in the LO world of s, pixel (u, v) of row v):
    t      = t_frame + dt[c, s] + rs[c] * (v / 480 - 0.5)
    X_L    = R_WL(t)^T (X - p_WL(t))                      refined LiDAR odometry, held fixed
    X_C    = R_LC^T (X_L - c_L)                            c_L = camera centre in os_lidar
    (u, v) = pi(X_C; fx0 F, fy0 F, cx, cy, k1, k2)        pinhole, radial k1 k2 (OpenCV plumb bob)
Trajectory between the 10 Hz knots: SLERP for rotation, uniform Catmull-Rom for position
(linear interpolation cuts the chord in turns: ~2-3 mm lateral at 20 deg/s).
Optional LiDAR tie: n . (X - q) / sigma for landmarks that lie on a thin planar patch of the
segment's LO map (lidar_odo/data.SegMap), Cauchy.

Parameters (global vector): per camera 12 = [w(3) rotation (left, LiDAR axes), c_L(3), logF,
cx, cy, k1, k2, rs], then one dt per (camera, segment); landmarks are Schur-eliminated.
Levenberg-Marquardt, Huber on the pixel residual.
Conventions: T_a_b maps b -> a; L = os_lidar (x backward); C = OpenCV optical.
"""
from __future__ import annotations

import numpy as np
import torch

DT = torch.float64
NCP = 13                                   # per-camera parameters
PN = ["wx", "wy", "wz", "cLx", "cLy", "cLz", "logF", "cx", "cy", "k1", "k2", "rs", "logA"]
H_IMG = 480.0


# ----------------------------------------------------------------------------- SO(3)
def skew(v):
    O = torch.zeros(v.shape[:-1] + (3, 3), dtype=v.dtype)
    O[..., 0, 1], O[..., 0, 2] = -v[..., 2], v[..., 1]
    O[..., 1, 0], O[..., 1, 2] = v[..., 2], -v[..., 0]
    O[..., 2, 0], O[..., 2, 1] = -v[..., 1], v[..., 0]
    return O


def so3_exp(w):
    th = w.norm(dim=-1, keepdim=True)
    small = th < 1e-12
    k = w / th.clamp_min(1e-12)
    K = skew(k)
    th = th[..., None]
    I = torch.eye(3, dtype=w.dtype).expand(K.shape)
    R = I + torch.sin(th) * K + (1 - torch.cos(th)) * (K @ K)
    return torch.where(small[..., None], I + skew(w), R)


def so3_log(R):
    from scipy.spatial.transform import Rotation as Rot
    return torch.as_tensor(Rot.from_matrix(R.numpy()).as_rotvec(), dtype=DT)


# ----------------------------------------------------------------------------- trajectory
class Trajs:
    """All segments' LO knots in one table. Times in seconds relative to t_ref (float64)."""

    def __init__(self, segs, lo_dir, t_ref_ns):
        tau, R, p, s0, s1 = [], [], [], [], []
        n = 0
        for s in segs:
            z = np.load(f"{lo_dir}/ref_{s}.npz")
            assert bool(z["ext"])
            ta = (z["tau"].astype(np.int64) - t_ref_ns).astype(np.float64) * 1e-9
            tau.append(ta); R.append(z["T_w_L"][:, :3, :3]); p.append(z["T_w_L"][:, :3, 3])
            s0.append(n); n += len(ta); s1.append(n)
        # abutting windows overlap by a knot or two (the extrapolated first knot), which the per-segment
        # clamp in pose() absorbs; windows out of time order do not
        if any(b[1] <= a[1] for a, b in zip(tau[:-1], tau[1:])):
            raise ValueError("Trajs: windows must be given in chronological order (one searchsorted over all knots)")
        self.tau = torch.as_tensor(np.concatenate(tau), dtype=DT)
        self.R = torch.as_tensor(np.concatenate(R), dtype=DT)
        self.p = torch.as_tensor(np.concatenate(p), dtype=DT)
        self.s0 = torch.as_tensor(s0)
        self.s1 = torch.as_tensor(s1)
        # relative rotation between consecutive knots (last knot of a segment: unused)
        Rn = torch.cat([self.R[1:], self.R[-1:]])
        self.phi = so3_log(self.R.transpose(1, 2) @ Rn)
        self.span = [(float(self.tau[a + 1]), float(self.tau[b - 1])) for a, b in zip(s0, s1)]

    def pose(self, seg, t):
        """seg (N,) long, t (N,) float64 s -> R_WL (N,3,3), p_WL (N,3)."""
        i = torch.searchsorted(self.tau, t.contiguous()) - 1
        lo, hi = self.s0[seg], self.s1[seg] - 2
        i = torch.maximum(torch.minimum(i, hi), lo)
        a = ((t - self.tau[i]) / (self.tau[i + 1] - self.tau[i]))
        R = self.R[i] @ so3_exp(self.phi[i] * a[:, None])
        # Catmull-Rom on positions (uniform knots); fall back to linear at segment ends
        im = torch.maximum(i - 1, lo)
        ip = torch.minimum(i + 2, hi + 1)
        p0, p1, p2, p3 = self.p[im], self.p[i], self.p[i + 1], self.p[ip]
        a1 = a[:, None]
        cr = 0.5 * (2 * p1 + (-p0 + p2) * a1 + (2 * p0 - 5 * p1 + 4 * p2 - p3) * a1 ** 2
                    + (-p0 + 3 * p1 - 3 * p2 + p3) * a1 ** 3)
        lin = p1 * (1 - a1) + p2 * a1
        edge = ((i == lo) | (i + 2 > hi + 1))[:, None]
        return R, torch.where(edge, lin, cr)


# ----------------------------------------------------------------------------- lens
def project(Xc, fx, fy, cx, cy, k1, k2, jac=True):
    z = Xc[:, 2]
    iz = 1.0 / z
    x, y = Xc[:, 0] * iz, Xc[:, 1] * iz
    r2 = x * x + y * y
    d = 1 + k1 * r2 + k2 * r2 * r2
    u = fx * x * d + cx
    v = fy * y * d + cy
    uv = torch.stack([u, v], -1)
    if not jac:
        return uv
    dd = 2 * k1 + 4 * k2 * r2                # d(d)/dx = dd * x
    du_dx = fx * (d + x * dd * x); du_dy = fx * x * dd * y
    dv_dx = fy * y * dd * x;       dv_dy = fy * (d + y * dd * y)
    JX = torch.empty(len(z), 2, 3, dtype=DT)
    JX[:, 0, 0] = du_dx * iz; JX[:, 0, 1] = du_dy * iz; JX[:, 0, 2] = -(du_dx * x + du_dy * y) * iz
    JX[:, 1, 0] = dv_dx * iz; JX[:, 1, 1] = dv_dy * iz; JX[:, 1, 2] = -(dv_dx * x + dv_dy * y) * iz
    Ji = torch.zeros(len(z), 2, 5, dtype=DT)    # logF, cx, cy, k1, k2
    Ji[:, 0, 0] = fx * x * d; Ji[:, 1, 0] = fy * y * d
    Ji[:, 0, 1] = 1.0; Ji[:, 1, 2] = 1.0
    Ji[:, 0, 3] = fx * x * r2; Ji[:, 1, 3] = fy * y * r2
    Ji[:, 0, 4] = fx * x * r2 * r2; Ji[:, 1, 4] = fy * y * r2 * r2
    return uv, JX, Ji


def unproject(uv, fx, fy, cx, cy, k1, k2):
    xd, yd = (uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy
    x, y = xd.clone(), yd.clone()
    for _ in range(15):
        r2 = x * x + y * y
        d = 1 + k1 * r2 + k2 * r2 * r2
        x, y = xd / d, yd / d
    v = torch.stack([x, y, torch.ones_like(x)], -1)
    return v / v.norm(dim=1, keepdim=True)


# ----------------------------------------------------------------------------- state
class Cams:
    def __init__(self, names, T_L_C, intr, rs, dt, nseg):
        """T_L_C (C,4,4); intr (C,6) fx fy cx cy k1 k2; rs (C,) s; dt (C,S) s."""
        self.names = list(names)
        T = torch.as_tensor(np.asarray(T_L_C), dtype=DT)
        I = torch.as_tensor(np.asarray(intr), dtype=DT)
        self.R = T[:, :3, :3].clone()
        self.c = T[:, :3, 3].clone()
        self.fx0, self.fy0 = I[:, 0].clone(), I[:, 1].clone()
        self.logF = torch.zeros(len(names), dtype=DT)
        self.cx, self.cy, self.k1, self.k2 = I[:, 2].clone(), I[:, 3].clone(), I[:, 4].clone(), I[:, 5].clone()
        self.rs = torch.as_tensor(np.asarray(rs), dtype=DT).clone()
        self.logA = torch.zeros(len(names), dtype=DT)
        self.dt = torch.as_tensor(np.asarray(dt), dtype=DT).reshape(len(names), nseg).clone()
        self.theta0 = self.vector().clone()

    @property
    def C(self):
        return len(self.names)

    @property
    def S(self):
        return self.dt.shape[1]

    @property
    def P(self):
        return self.C * NCP + self.C * self.S

    def vector(self):
        v = torch.zeros(self.P, dtype=DT)
        for i in range(self.C):
            b = i * NCP
            v[b + 3:b + 6] = self.c[i]
            v[b + 6] = self.logF[i]
            v[b + 7], v[b + 8], v[b + 9], v[b + 10], v[b + 11] = self.cx[i], self.cy[i], self.k1[i], self.k2[i], self.rs[i]
            v[b + 12] = self.logA[i]
        v[self.C * NCP:] = self.dt.flatten()
        return v

    def intr(self):
        F = torch.exp(self.logF)
        return self.fx0 * F, self.fy0 * F * torch.exp(self.logA), self.cx, self.cy, self.k1, self.k2

    def copy(self):
        o = Cams.__new__(Cams)
        o.names = list(self.names)
        for k in ("R", "c", "fx0", "fy0", "logF", "cx", "cy", "k1", "k2", "rs", "logA", "dt", "theta0"):
            setattr(o, k, getattr(self, k).clone())
        return o

    def apply(self, d):
        C = self.C
        dc = d[:C * NCP].view(C, NCP)
        self.R = so3_exp(dc[:, 0:3]) @ self.R
        self.c = self.c + dc[:, 3:6]
        self.logF = self.logF + dc[:, 6]
        self.cx = self.cx + dc[:, 7]; self.cy = self.cy + dc[:, 8]
        self.k1 = self.k1 + dc[:, 9]; self.k2 = self.k2 + dc[:, 10]
        self.rs = self.rs + dc[:, 11]
        self.logA = self.logA + dc[:, 12]
        self.dt = self.dt + d[C * NCP:].view(C, self.S)

    def T_L_C(self):
        T = np.tile(np.eye(4), (self.C, 1, 1))
        T[:, :3, :3] = self.R.numpy()
        T[:, :3, 3] = self.c.numpy()
        return T

    def intr_array(self):
        fx, fy, cx, cy, k1, k2 = self.intr()
        return torch.stack([fx, fy, cx, cy, k1, k2], -1).numpy()


class Obs:
    """cam, seg (trajectory segment), dseg (time-offset group), lm (N,) long; uv (N,2);
    tf (N,) frame time (s rel t_ref)."""

    def __init__(self, cam, seg, lm, uv, tf, dseg=None):
        self.cam = torch.as_tensor(cam, dtype=torch.long)
        self.seg = torch.as_tensor(seg, dtype=torch.long)
        self.dseg = self.seg.clone() if dseg is None else torch.as_tensor(dseg, dtype=torch.long)
        self.lm = torch.as_tensor(lm, dtype=torch.long)
        self.uv = torch.as_tensor(uv, dtype=DT)
        self.tf = torch.as_tensor(tf, dtype=DT)

    def __len__(self):
        return len(self.cam)

    def subset(self, m):
        m = torch.as_tensor(m)
        return Obs(self.cam[m], self.seg[m], self.lm[m], self.uv[m], self.tf[m], self.dseg[m])

    def chunk(self, sl):
        return Obs(self.cam[sl], self.seg[sl], self.lm[sl], self.uv[sl], self.tf[sl], self.dseg[sl])

    @staticmethod
    def cat(a, b):
        return Obs(torch.cat([a.cam, b.cam]), torch.cat([a.seg, b.seg]), torch.cat([a.lm, b.lm]),
                   torch.cat([a.uv, b.uv]), torch.cat([a.tf, b.tf]), torch.cat([a.dseg, b.dseg]))


class Ties:
    def __init__(self, lm, n, q, sigma=0.03):
        self.lm = torch.as_tensor(lm, dtype=torch.long)
        self.n = torch.as_tensor(n, dtype=DT)
        self.q = torch.as_tensor(q, dtype=DT)
        self.sigma = sigma
        self.scale = 2.0 * sigma

    def __len__(self):
        return len(self.lm)


def obs_time(cams, o):
    return o.tf + cams.dt[o.cam, o.dseg] + cams.rs[o.cam] * (o.uv[:, 1] / H_IMG - 0.5)


def cam_points(cams, trajs, o, X, t=None):
    t = obs_time(cams, o) if t is None else t
    R, p = trajs.pose(o.seg, t)
    XL = torch.einsum("nji,nj->ni", R, X[o.lm] - p)
    y = XL - cams.c[o.cam]
    Xc = torch.einsum("nji,nj->ni", cams.R[o.cam], y)
    return Xc, y, R


def residuals(cams, trajs, o, X, jac=True, h=2e-3):
    """r (N,2), and if jac: Jp (N,2,13) [12 camera params + dt], Jl (N,2,3), Xc."""
    Xc, y, RWL = cam_points(cams, trajs, o, X)
    fx, fy, cx, cy, k1, k2 = [a[o.cam] for a in cams.intr()]
    if not jac:
        return project(Xc, fx, fy, cx, cy, k1, k2, jac=False) - o.uv, Xc
    uv, JX, Ji = project(Xc, fx, fy, cx, cy, k1, k2)
    RLC = cams.R[o.cam]
    RLCt = RLC.transpose(1, 2)
    Jp = torch.zeros(len(o), 2, NCP + 1, dtype=DT)
    Jp[:, :, 0:3] = JX @ (RLCt @ skew(y))
    Jp[:, :, 3:6] = -(JX @ RLCt)
    Jp[:, :, 6:11] = Ji
    # time: numerical derivative of the camera-frame point along the trajectory
    t = obs_time(cams, o)
    Xc2, _, _ = cam_points(cams, trajs, o, X, t + h)
    dXc = (Xc2 - Xc) / h
    g = torch.einsum("nij,nj->ni", JX, dXc)                      # d uv / d t
    Jp[:, :, 11] = g * (o.uv[:, 1:2] / H_IMG - 0.5)               # rs
    Jp[:, :, 12, ] = 0.0
    Jp[:, 1, 12] = uv[:, 1] - cy                                  # logA (fy only)
    Jp[:, :, 13] = g                                              # dt[c, s]
    Jl = JX @ (RLCt @ RWL.transpose(1, 2))
    return uv - o.uv, Jp, Jl, Xc


def gidx(cams, cam, seg):
    """global parameter indices (N,13) for observations of (cam, seg)."""
    base = cam[:, None] * NCP + torch.arange(NCP)[None]
    d = cams.C * NCP + cam * cams.S + seg
    return torch.cat([base, d[:, None]], 1)


def huber_w(e, d):
    return torch.where(e <= d, torch.ones_like(e), d / e)


PRIOR_LIN = []      # (param index, axis vector over c_L (3) or None, target, sigma) - profiles


def prior_terms(cams, prior_sig, theta0=None):
    v = cams.vector()
    th0 = cams.theta0 if theta0 is None else theta0
    dv = v - th0
    ok = torch.isfinite(prior_sig) & (prior_sig > 0)
    c = 0.5 * ((dv[ok] / prior_sig[ok]) ** 2).sum()
    for (ci, ax, tgt, sg) in PRIOR_LIN:
        c = c + 0.5 * ((float(cams.c[ci] @ torch.as_tensor(ax, dtype=DT)) - tgt) / sg) ** 2
    return float(c)


_LAST_E = [None]      # residual norms of the last cost evaluation (reused for the progress log)


def cost(cams, trajs, obs, X, ties, prior_sig, huber, chunk=400_000):
    c = 0.0
    es = []
    for s0 in range(0, len(obs), chunk):
        o = obs.chunk(slice(s0, s0 + chunk))
        r, Xc = residuals(cams, trajs, o, X, jac=False)
        e = r.norm(dim=1)
        es.append(e)
        rho = torch.where(e <= huber, 0.5 * e * e, huber * (e - 0.5 * huber))
        rho = torch.where(Xc[:, 2] <= 0.2, torch.full_like(rho, 1e4), rho)
        c += float(rho.sum())
    if ties is not None and len(ties):
        d = torch.einsum("ni,ni->n", ties.n, X[ties.lm] - ties.q) / ties.sigma
        s = ties.scale / ties.sigma
        c += float(0.5 * (s * s * torch.log1p((d / s) ** 2)).sum())
    if EDGE is not None and len(EDGE.sel):
        c += EDGE.cost(cams, trajs)
    _LAST_E[0] = torch.cat(es) if es else torch.zeros(0, dtype=DT)
    return c + prior_terms(cams, prior_sig)


def all_residuals(cams, trajs, obs, X, chunk=400_000):
    R, Q = [], []
    for s0 in range(0, len(obs), chunk):
        r, xc = residuals(cams, trajs, obs.chunk(slice(s0, s0 + chunk)), X, jac=False)
        R.append(r); Q.append(xc)
    return torch.cat(R), torch.cat(Q)


class Structure:
    """(landmark, camera) pairs; groups = (cam, seg) blocks; cross pairs for multi-camera landmarks."""

    def __init__(self, obs, cams, M):
        C = cams.C
        key = obs.lm * C + obs.cam
        ukey, self.pair = torch.unique(key, return_inverse=True)
        self.p_lm, self.p_cam = ukey // C, ukey % C
        # segment of each pair: that of any of its observations
        pseg = torch.zeros(len(ukey), dtype=torch.long)
        pseg[self.pair] = obs.dseg
        self.p_seg = pseg
        self.p_g = gidx(cams, self.p_cam, self.p_seg)                 # (P,13)
        self.npair = len(ukey)
        cnt = torch.bincount(self.p_lm, minlength=M)
        assert int(cnt.max()) <= 2, "cross pairs implemented for at most 2 cameras per landmark"
        xp, xq = [], []
        if bool((cnt == 2).any()):
            order = torch.argsort(self.p_lm, stable=True)
            sm = self.p_lm[order]
            two = cnt[sm] == 2
            ids = order[two].view(-1, 2)                     # consecutive pair ids of one landmark
            xp = [torch.cat([ids[:, 0], ids[:, 1]])]
            xq = [torch.cat([ids[:, 1], ids[:, 0]])]
        self.xp = torch.cat(xp) if xp else torch.zeros(0, dtype=torch.long)
        self.xq = torch.cat(xq) if xq else torch.zeros(0, dtype=torch.long)


def accumulate(cams, trajs, obs, X, ties, prior_sig, free, huber, st, chunk=250_000):
    P, M = cams.P, len(X)
    U = torch.zeros(P, P, dtype=DT)
    bp = torch.zeros(P, dtype=DT)
    V = torch.zeros(M, 3, 3, dtype=DT)
    bl = torch.zeros(M, 3, dtype=DT)
    Wp = torch.zeros(st.npair, NCP + 1, 3, dtype=DT)
    G = cams.C * cams.S
    Ug = torch.zeros(G, NCP + 1, NCP + 1, dtype=DT)
    bg = torch.zeros(G, NCP + 1, dtype=DT)
    for s0 in range(0, len(obs), chunk):
        sl = slice(s0, min(len(obs), s0 + chunk))
        o = obs.chunk(sl)
        r, Jp, Jl, Xc = residuals(cams, trajs, o, X)
        e = r.norm(dim=1)
        w = huber_w(e, huber)
        w = torch.where(Xc[:, 2] <= 0.2, torch.zeros_like(w), w)
        wJp = Jp * w[:, None, None]
        wJl = Jl * w[:, None, None]
        grp = o.cam * cams.S + o.dseg
        Ug.index_add_(0, grp, wJp.transpose(1, 2) @ Jp)
        bg.index_add_(0, grp, torch.einsum("nki,nk->ni", wJp, r))
        V.index_add_(0, o.lm, wJl.transpose(1, 2) @ Jl)
        bl.index_add_(0, o.lm, torch.einsum("nki,nk->ni", wJl, r))
        Wp.index_add_(0, st.pair[sl], wJp.transpose(1, 2) @ Jl)
    if EDGE is not None and len(EDGE.sel):
        cc = EDGE.cauchy / EDGE.sigma
        for r, J, ecam, edseg in EDGE.terms(cams, trajs):
            s = r / EDGE.sigma
            w = EDGE.weight / (1.0 + (s / cc) ** 2) / EDGE.sigma ** 2
            grp = ecam * cams.S + edseg
            Ug.index_add_(0, grp, (J * w[:, None])[:, :, None] * J[:, None, :])
            bg.index_add_(0, grp, J * (w * r)[:, None])
    # scatter the (cam, seg) blocks
    for g in range(G):
        c, s = divmod(g, cams.S)
        gi = gidx(cams, torch.tensor([c]), torch.tensor([s]))[0]
        U[gi[:, None], gi[None, :]] += Ug[g]
        bp[gi] += bg[g]
    if ties is not None and len(ties):
        d = torch.einsum("ni,ni->n", ties.n, X[ties.lm] - ties.q)
        wt = 1.0 / (1.0 + (d / ties.scale) ** 2) / ties.sigma ** 2
        V.index_add_(0, ties.lm, wt[:, None, None] * ties.n[:, :, None] * ties.n[:, None, :])
        bl.index_add_(0, ties.lm, (wt * d)[:, None] * ties.n)
    ok = torch.isfinite(prior_sig) & (prior_sig > 0) & free
    ps = torch.where(ok, prior_sig, torch.full_like(prior_sig, np.inf))
    dv = cams.vector() - cams.theta0
    U += torch.diag(1.0 / ps ** 2)
    bp += dv / ps ** 2
    for (ci, ax, tgt, sg) in PRIOR_LIN:
        a_ = torch.as_tensor(ax, dtype=DT)
        b = ci * NCP + 3
        U[b:b + 3, b:b + 3] += torch.outer(a_, a_) / sg ** 2
        bp[b:b + 3] += a_ * (float(cams.c[ci] @ a_) - tgt) / sg ** 2
    return U, bp, V, bl, Wp


def reduced(U, V, Wp, st, lam):
    Ud = U + lam * torch.diag(torch.diagonal(U).clamp_min(1e-9))
    Vd = V + lam * torch.diag_embed(torch.diagonal(V, dim1=1, dim2=2).clamp_min(1e-9)) \
        + 1e-9 * torch.eye(3, dtype=DT)
    Vi = torch.linalg.inv(Vd)
    Y = Wp @ Vi[st.p_lm]                                   # (Pairs, 13, 3)
    S = Ud.clone()
    # diagonal pair terms, grouped by the pair's (cam, seg) block
    G = int(st.p_g[:, -1].max()) + 1
    blk = Y @ Wp.transpose(1, 2)
    key = st.p_g[:, -1]                                    # dt index identifies (cam, seg) uniquely
    uk, inv = torch.unique(key, return_inverse=True)
    acc = torch.zeros(len(uk), NCP + 1, NCP + 1, dtype=DT).index_add_(0, inv, blk)
    for j in range(len(uk)):
        gi = st.p_g[(inv == j).nonzero()[0, 0]]
        S[gi[:, None], gi[None, :]] -= acc[j]
    if len(st.xp):
        cb = Y[st.xp] @ Wp[st.xq].transpose(1, 2)
        k2 = st.p_g[st.xp, -1] * 100000 + st.p_g[st.xq, -1]
        uk2, inv2 = torch.unique(k2, return_inverse=True)
        acc2 = torch.zeros(len(uk2), NCP + 1, NCP + 1, dtype=DT).index_add_(0, inv2, cb)
        for j in range(len(uk2)):
            f = (inv2 == j).nonzero()[0, 0]
            ga, gb = st.p_g[st.xp[f]], st.p_g[st.xq[f]]
            S[ga[:, None], gb[None, :]] -= acc2[j]
    return S, Vi, Y


def solve(cams, trajs, obs, X, free, prior_sig, ties=None, iters=30, huber=1.0, lam0=1e-3,
          verbose=True, tol=1e-7, want_cov=False, log=print):
    M = len(X)
    st = Structure(obs, cams, M)
    fidx = torch.nonzero(free).flatten()
    lam = lam0
    cst = cost(cams, trajs, obs, X, ties, prior_sig, huber)
    e_acc = _LAST_E[0]
    hist = [cst]
    rel = 1.0
    for it in range(iters):
        U, bp, V, bl, Wp = accumulate(cams, trajs, obs, X, ties, prior_sig, free, huber, st)
        while True:
            S, Vi, Y = reduced(U, V, Wp, st, lam)
            rhs = bp - torch.zeros(cams.P, dtype=DT).index_put_(
                (st.p_g.flatten(),), torch.einsum("pij,pj->pi", Y, bl[st.p_lm]).flatten(), accumulate=True)
            try:
                dpf = -torch.linalg.solve(S[fidx][:, fidx], rhs[fidx]) if len(fidx) else torch.zeros(0, dtype=DT)
            except RuntimeError:
                lam *= 10
                continue
            dp = torch.zeros(cams.P, dtype=DT)
            dp[fidx] = dpf
            t = torch.zeros(M, 3, dtype=DT).index_add_(0, st.p_lm, torch.einsum("pij,pi->pj", Wp, dp[st.p_g]))
            dl = -torch.einsum("mij,mj->mi", Vi, bl + t)
            nc = cams.copy()
            nc.apply(dp)
            nX = X + dl
            new = cost(nc, trajs, obs, nX, ties, prior_sig, huber)
            if new < cst:
                cams, X = nc, nX
                e_acc = _LAST_E[0]
                rel = (cst - new) / cst
                cst = new
                lam = max(lam / 3, 1e-7)
                break
            lam *= 4
            if lam > 1e8:
                rel = 0.0
                break
        hist.append(cst)
        if verbose:
            e = e_acc          # = all_residuals(cams, trajs, obs, X)[0].norm(dim=1) of the accepted state
            C = cams.C
            dcam = dp[:C * NCP].view(C, NCP)
            log(f"    LM {it:2d}: cost {cst:.1f} lam {lam:.1e} | reproj median {e.median():.3f} px | step rot "
                f"{np.degrees(float(dcam[:, :3].abs().max())):.4f} deg pos {1e3 * float(dcam[:, 3:6].abs().max()):.1f} mm "
                f"logF {float(dcam[:, 6].abs().max()):.1e} dt {1e3 * float(dp[C * NCP:].abs().max()) if cams.S else 0:.2f} ms")
        if rel < tol or lam > 1e8:
            break
    info = {"cost": cst, "hist": hist, "fidx": fidx, "cov": None}
    if want_cov and len(fidx):
        U, bp, V, bl, Wp = accumulate(cams, trajs, obs, X, ties, prior_sig, free, huber, st)
        S, _, _ = reduced(U, V, Wp, st, 0.0)
        try:
            info["cov"] = torch.linalg.inv(S[fidx][:, fidx])
        except Exception as ex:  # noqa
            log(f"    covariance failed: {ex}")
    return cams, X, info


def triangulate(cams, trajs, obs, M, chunk=500_000):
    """Midpoint of all rays of each landmark; X (M,3), parallax (M,) rad."""
    I3 = torch.eye(3, dtype=DT)
    AA = torch.zeros(M, 3, 3, dtype=DT)
    Ab = torch.zeros(M, 3, dtype=DT)
    dm = torch.zeros(M, 3, dtype=DT)
    fx, fy, cx, cy, k1, k2 = cams.intr()

    def rays(o):
        R, p = trajs.pose(o.seg, obs_time(cams, o))
        v = unproject(o.uv, fx[o.cam], fy[o.cam], cx[o.cam], cy[o.cam], k1[o.cam], k2[o.cam])
        d = torch.einsum("nij,nj->ni", R @ cams.R[o.cam], v)
        c = torch.einsum("nij,nj->ni", R, cams.c[o.cam]) + p
        return d, c

    for s0 in range(0, len(obs), chunk):
        o = obs.chunk(slice(s0, s0 + chunk))
        d, c = rays(o)
        A = I3 - d[:, :, None] * d[:, None, :]
        AA.index_add_(0, o.lm, A)
        Ab.index_add_(0, o.lm, torch.einsum("nij,nj->ni", A, c))
        dm.index_add_(0, o.lm, d)
    X = torch.linalg.solve(AA + 1e-9 * I3, Ab)
    dm = dm / dm.norm(dim=1, keepdim=True).clamp_min(1e-12)
    par = torch.zeros(M, dtype=DT)
    for s0 in range(0, len(obs), chunk):
        o = obs.chunk(slice(s0, s0 + chunk))
        d, _ = rays(o)
        ang = torch.arccos(torch.einsum("ni,ni->n", d, dm[o.lm]).clamp(-1, 1))
        par.scatter_reduce_(0, o.lm, ang, reduce="amax")
    return X, 2 * par


# ----------------------------------------------------------------------------- edge-alignment term
EDGE = None          # set to an EdgeTerm to add LiDAR-edge <-> thermal-edge residuals to cost/accumulate


class EdgeTerm:
    """Point-to-line residuals between near-range LiDAR depth-edge points (accumulated with the LO,
    fixed world points, edge_prep.py) and the nearest orientation-compatible sub-pixel thermal edge
    of the same frame:  r = n_e . (pi(X; camera, t) - q_e),  Cauchy, re-associated between rounds.
    Only the camera parameters (rotation, position, lens, dt, rs) enter; no landmark."""

    def __init__(self, segs, cams_names, edge_dir, dt_mode="seg", sigma=1.0, cauchy=1.5, weight=1.0,
                 max_depth=40.0, min_depth=1.5):
        from scipy.spatial import cKDTree
        X, cam, seg, kind, tf, gfr = [], [], [], [], [], []
        self.E, self.trees = [], []
        ng = 0
        for si, s in enumerate(segs):
            for ci, c in enumerate(cams_names):
                f = f"{edge_dir}/{s}_{c}.npz"
                try:
                    z = np.load(f)
                except FileNotFoundError:
                    continue
                r = z["rng_cam"]
                m = (r >= min_depth) & (r <= max_depth)
                X.append(z["X"][m]); cam.append(np.full(m.sum(), ci)); seg.append(np.full(m.sum(), si))
                kind.append(z["kind"][m]); tf.append(z["frame_tf"][z["fi"][m]]); gfr.append(z["fi"][m] + ng)
                EE, eo = z["E"], z["eoff"]
                for k in range(len(z["frame_hdr"])):
                    e = EE[eo[k]:eo[k + 1]].copy()
                    self.E.append(e)
                    self.trees.append(cKDTree(e[:, :2]) if len(e) else None)
                ng += len(z["frame_hdr"])
        self.X = torch.as_tensor(np.concatenate(X), dtype=DT)
        self.cam = torch.as_tensor(np.concatenate(cam), dtype=torch.long)
        self.seg = torch.as_tensor(np.concatenate(seg), dtype=torch.long)
        self.dseg = self.seg.clone() if dt_mode == "seg" else torch.zeros_like(self.seg)
        self.kind = np.concatenate(kind)
        self.tf = torch.as_tensor(np.concatenate(tf), dtype=DT)
        self.gfr = np.concatenate(gfr)
        self.sigma, self.cauchy, self.weight = sigma, cauchy, weight
        self.sel = torch.zeros(0, dtype=torch.long)
        self.q = torch.zeros(0, 2, dtype=DT)
        self.n = torch.zeros(0, 2, dtype=DT)

    def __len__(self):
        return len(self.X)

    def _obs(self, idx, uv):
        return Obs(self.cam[idx], self.seg[idx], torch.arange(len(idx)), uv, self.tf[idx], self.dseg[idx])

    def predict(self, cams, trajs, idx=None, chunk=500_000):
        idx = torch.arange(len(self.X)) if idx is None else idx
        out, zz = [], []
        for s0 in range(0, len(idx), chunk):
            ii = idx[s0:s0 + chunk]
            uv = torch.full((len(ii), 2), 240.0, dtype=DT)
            for _ in range(2):                      # row time from the predicted row
                o = self._obs(ii, uv)
                Xc, _, _ = cam_points(cams, trajs, o, self.X[ii])
                fx, fy, cx, cy, k1, k2 = [a[o.cam] for a in cams.intr()]
                uv = project(Xc, fx, fy, cx, cy, k1, k2, jac=False)
            out.append(uv); zz.append(Xc[:, 2])
        return torch.cat(out), torch.cat(zz)

    def associate(self, cams, trajs, gate=3.0, orient=0.5):
        uv, z = self.predict(cams, trajs)
        uvn = uv.numpy()
        ok = (z.numpy() > 0.5) & (uvn[:, 0] > 2) & (uvn[:, 0] < 638) & (uvn[:, 1] > 2) & (uvn[:, 1] < 478)
        sel, q, n = [], [], []
        order = np.argsort(self.gfr, kind="stable")
        g_sorted = self.gfr[order]
        bounds = np.flatnonzero(np.r_[True, g_sorted[1:] != g_sorted[:-1], True])
        for a_, b_ in zip(bounds[:-1], bounds[1:]):
            ii = order[a_:b_]
            ii = ii[ok[ii]]
            g = int(self.gfr[order[a_]])
            tree = self.trees[g]
            if tree is None or not len(ii):
                continue
            d, nn = tree.query(uvn[ii], k=6, distance_upper_bound=gate)
            E = self.E[g]
            best = np.full(len(ii), -1)
            for k in range(6):
                valid = np.isfinite(d[:, k]) & (best < 0)
                if not valid.any():
                    continue
                e = E[np.minimum(nn[:, k], len(E) - 1)]
                comp = np.where(self.kind[ii] == 0, np.abs(e[:, 2]) >= orient, np.abs(e[:, 3]) >= orient)
                take = valid & comp
                best[take] = nn[take, k]
            m = best >= 0
            if not m.any():
                continue
            e = E[best[m]]
            sel.append(ii[m]); q.append(e[:, :2]); n.append(e[:, 2:4])
        self.sel = torch.as_tensor(np.concatenate(sel), dtype=torch.long)
        self.q = torch.as_tensor(np.concatenate(q), dtype=DT)
        nn_ = torch.as_tensor(np.concatenate(n), dtype=DT)
        self.n = nn_ / nn_.norm(dim=1, keepdim=True).clamp_min(1e-9)
        return len(self.sel)

    def terms(self, cams, trajs, jac=True, chunk=300_000):
        """yield (r (n,), J (n,13) or None, cam, dseg) over chunks of the associated points."""
        for s0 in range(0, len(self.sel), chunk):
            ii = self.sel[s0:s0 + chunk]
            q, n = self.q[s0:s0 + chunk], self.n[s0:s0 + chunk]
            o = self._obs(ii, q.clone())
            if jac:
                r2, Jp, _, _ = residuals(cams, trajs, o, self.X[ii])
                yield (n * r2).sum(1), torch.einsum("ni,nij->nj", n, Jp), o.cam, o.dseg
            else:
                r2, _ = residuals(cams, trajs, o, self.X[ii], jac=False)
                yield (n * r2).sum(1), None, o.cam, o.dseg

    def cost(self, cams, trajs):
        c = 0.0
        cc = self.cauchy / self.sigma
        for r, _, _, _ in self.terms(cams, trajs, jac=False):
            s = r / self.sigma
            c += float(self.weight * 0.5 * cc * cc * torch.log1p((s / cc) ** 2).sum())
        return c

    def stats(self, cams, trajs):
        rs = torch.cat([r for r, _, _, _ in self.terms(cams, trajs, jac=False)]) if len(self.sel) else torch.zeros(0)
        return {"n_assoc": int(len(self.sel)), "n_points": int(len(self.X)),
                "median_abs_px": float(rs.abs().median()) if len(rs) else None,
                "frac_lt1px": float((rs.abs() < 1).double().mean()) if len(rs) else None}
