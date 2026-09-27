"""Per-frame (camera-rate) correction of the LiDAR-odometry pose inside the RGB rig BA.

All RGB cameras of the rig are exposed at the same instant (PTP-synchronous global shutter), so an error of
the LiDAR pose T_WL at a camera frame is common to every observation of that frame (~230 observations from
~12 cameras). The 10 Hz LO (SLERP between knots) misses part of the road-induced body motion (mostly roll).
Here a small body-frame correction  T_WL(t_f) <- T_WL(t_f) exp([dth_f, dp_f])  is estimated per synchronous
frame f, alternating with the rig BA (landmarks, calibration held while the corrections are fitted):

  minimise  sum_obs huber(|uv(obs; dth_f, dp_f) - uv_meas|)
          + sum_f |dth_f / s_rot|^2 + |dp_f / s_pos|^2                    (magnitude prior)
          + sum_f |(d_{f+1} - d_f) / s_rw|^2                               (optional first-difference prior)

per window, one sparse block-tridiagonal Gauss-Newton step per call (the problem is nearly linear at the
0.05 deg / 3 mm level). Gauge: a correction that is constant (or slowly varying) over a window is the same
as moving the whole rig relative to the LiDAR, i.e. it would trade against the extrinsics that we want to
estimate. After the fit the low-frequency part of every correction component is removed (running mean over
`highpass_s` seconds, and the window mean), so only the frame-level (high-frequency) body motion is corrected
and the extrinsics keep being determined by the LO trajectory and the LiDAR ties as before.

Frames with fewer than `min_cams` cameras (or `min_obs` observations) keep a zero correction: a frame seen
by one camera cannot separate a pose error from that camera's measurement error.

The corrections live in the Obs pose table (Obs.PR / Obs.Pp) as the product with the LO pose, so the rig BA
itself is unchanged. Conventions as rigba.py.
"""
from __future__ import annotations

import numpy as np
import torch

from .rigba import DT, kb_project, so3_exp

DEFAULTS = dict(enabled=False, rot=True, pos=True, sigma_rot_deg=0.2, sigma_pos_m=0.02, sigma_rw_rot_deg=None,
                sigma_rw_pos_m=None, highpass_s=1.0, min_cams=3, min_obs=30, max_px=3.0, huber=1.5,
                group_ms=5.0, rounds=[1, 2], steps=2)


class FrameCorr:
    """Pose-table bookkeeping: frame id of every pose row, the LO table (uncorrected), current corrections."""

    def __init__(self, obs, opts: dict):
        self.o = {**DEFAULTS, **(opts or {})}
        P = obs.poses
        t = P["t"].astype(np.int64)
        seg = P["seg"].astype(np.int64)
        # synchronous frames: pose rows of one window whose times are within group_ms of each other
        order = np.lexsort((t, seg))
        ts, ss = t[order], seg[order]
        new = np.r_[True, (ss[1:] != ss[:-1]) | (np.diff(ts) > self.o["group_ms"] * 1e6)]
        fid_sorted = np.cumsum(new) - 1
        self.fid = np.empty(len(t), np.int64)
        self.fid[order] = fid_sorted
        self.F = int(fid_sorted[-1]) + 1 if len(t) else 0
        first = order[new]
        self.f_t = t[first].astype(np.float64)          # frame time (first camera's stamp)
        self.f_seg = seg[first]
        self.PR0 = obs.PR.clone()
        self.Pp0 = obs.Pp.clone()
        self.x = np.zeros((self.F, 6))                  # [dth (rad), dp (m)] body frame
        self.stats = []

    def apply(self, obs) -> None:
        """Pose table <- LO pose * exp(correction of its frame)."""
        x = torch.as_tensor(self.x[self.fid], dtype=DT)
        R0 = self.PR0
        obs.PR = R0 @ so3_exp(x[:, :3])
        obs.Pp = self.Pp0 + torch.einsum("nij,nj->ni", R0, x[:, 3:])

    def fit(self, cams, obs, X, log=print, chunk: int = 500_000) -> dict:
        """One Gauss-Newton step of all frame corrections about the current ones (landmarks, cameras held),
        then the high-pass. Returns stats."""
        o = self.o
        F = self.F
        A = torch.zeros(F, 6, 6, dtype=DT)
        b = torch.zeros(F, 6, dtype=DT)
        nobs = torch.zeros(F, dtype=DT)
        fid_t = torch.as_tensor(self.fid, dtype=torch.long)
        camset = torch.zeros(F, cams.C, dtype=torch.bool)
        e_all = []
        for s0 in range(0, len(obs), chunk):
            ob = obs.chunk(slice(s0, s0 + chunk))
            RWL, pWL = ob.RWL, ob.pWL
            XL = torch.einsum("nji,nj->ni", RWL, X[ob.lm] - pWL)
            ci = ob.cam
            y = XL - cams.c[ci]
            RLC = cams.R[ci]
            Xc = torch.einsum("nji,nj->ni", RLC, y)
            fx, fy, cx, cy, k1, k2, k3 = [a[ci] for a in cams.intr()]
            uv, JX, _ = kb_project(Xc, fx, fy, cx, cy, k1, k2, k3)
            r = uv - ob.uv
            e = r.norm(dim=1)
            e_all.append(e)
            # T_WL <- T_WL exp([dth, dp]):  XL' = exp(-dth) (XL - dp)  ->  dXL/ddth = [XL]x, dXL/ddp = -I
            JC = JX @ RLC.transpose(1, 2)                       # d uv / d XL
            J = torch.zeros(len(ci), 2, 6, dtype=DT)
            Sx = torch.zeros(len(ci), 3, 3, dtype=DT)
            Sx[:, 0, 1], Sx[:, 0, 2] = -XL[:, 2], XL[:, 1]
            Sx[:, 1, 0], Sx[:, 1, 2] = XL[:, 2], -XL[:, 0]
            Sx[:, 2, 0], Sx[:, 2, 1] = -XL[:, 1], XL[:, 0]
            J[:, :, :3] = JC @ Sx
            J[:, :, 3:] = -JC
            w = torch.where(e <= o["huber"], torch.ones_like(e), o["huber"] / e.clamp_min(1e-12))
            w = torch.where((e < o["max_px"]) & (Xc[:, 2] > 0.5), w, torch.zeros_like(w))
            f = fid_t[ob.pidx]
            wJ = J * w[:, None, None]
            A.index_add_(0, f, wJ.transpose(1, 2) @ J)
            b.index_add_(0, f, torch.einsum("nki,nk->ni", wJ, r))
            nobs.index_add_(0, f, (w > 0).to(DT))
            camset[f[w > 0], ci[w > 0]] = True
        ncam = camset.sum(1).cpu().numpy()
        nob = nobs.cpu().numpy()
        active = (ncam >= o["min_cams"]) & (nob >= o["min_obs"])
        A = A.cpu().numpy()
        b = b.cpu().numpy()
        # parameter mask and priors (the current x is the linearisation point: the priors act on x + dx)
        use = np.array([o["rot"]] * 3 + [o["pos"]] * 3)
        s0 = np.array([np.radians(o["sigma_rot_deg"])] * 3 + [o["sigma_pos_m"]] * 3)
        srw = None
        if o.get("sigma_rw_rot_deg") or o.get("sigma_rw_pos_m"):
            srw = np.array([np.radians(o["sigma_rw_rot_deg"] or 1e3)] * 3 + [o["sigma_rw_pos_m"] or 1e3] * 3)
        x_new = np.zeros_like(self.x)
        import scipy.sparse as sp
        import scipy.sparse.linalg as spl
        for sg in np.unique(self.f_seg):
            fr = np.flatnonzero((self.f_seg == sg) & active)
            if not len(fr):
                continue
            fr = fr[np.argsort(self.f_t[fr])]
            n = len(fr)
            H = A[fr] + np.diag(1.0 / s0 ** 2)[None]
            g = b[fr] + self.x[fr] / s0[None] ** 2          # gradient of the prior at x
            rows, cols, vals = [], [], []
            ii = np.arange(n)[:, None, None] * 6 + np.arange(6)[None, :, None]
            jj = np.arange(n)[:, None, None] * 6 + np.arange(6)[None, None, :]
            rows.append(np.broadcast_to(ii, (n, 6, 6)).ravel())
            cols.append(np.broadcast_to(jj, (n, 6, 6)).ravel())
            vals.append(H.ravel())
            if srw is not None and n > 1:
                # first-difference prior between consecutive active frames, scaled by the time gap (random walk)
                dtf = np.maximum(np.diff(self.f_t[fr]) * 1e-9 / (1 / 30.0), 1.0)
                wd = (1.0 / srw ** 2)[None] / dtf[:, None]               # (n-1, 6)
                k = np.arange(n - 1)
                for q in range(6):
                    a_, c_ = 6 * k + q, 6 * (k + 1) + q
                    rows += [a_, c_, a_, c_]
                    cols += [a_, c_, c_, a_]
                    vals += [wd[:, q], wd[:, q], -wd[:, q], -wd[:, q]]
                dx = self.x[fr][1:] - self.x[fr][:-1]
                gd = wd * dx
                g[:-1] -= gd
                g[1:] += gd
            Hs = sp.csc_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(6 * n, 6 * n))
            m = np.tile(use, n)
            sol = np.zeros(6 * n)
            sol[m] = -spl.spsolve(Hs[m][:, m].tocsc(), g.ravel()[m])
            x_new[fr] = self.x[fr] + sol.reshape(n, 6)
        x_new[:, ~use] = 0.0
        # high-pass per window (gauge: no slow / constant part that could trade against the extrinsics)
        hp = self.o["highpass_s"]
        for sg in np.unique(self.f_seg):
            fr = np.flatnonzero(self.f_seg == sg)
            fr = fr[np.argsort(self.f_t[fr])]
            if not len(fr):
                continue
            xv = x_new[fr]
            if hp and hp > 0:
                tt = self.f_t[fr] * 1e-9
                cs = np.vstack([np.zeros((1, 6)), np.cumsum(xv, 0)])
                lo = np.searchsorted(tt, tt - hp / 2, side="left")
                hi = np.searchsorted(tt, tt + hp / 2, side="right")
                xv = xv - (cs[hi] - cs[lo]) / (hi - lo)[:, None]
            xv = xv - xv.mean(0)
            x_new[fr] = xv
        x_new[~active] = 0.0
        e0 = torch.cat(e_all)
        self.x = x_new
        self.apply(obs)
        st = {"frames": int(F), "active": int(active.sum()),
              "rot_rms_mdeg": (np.degrees(np.sqrt((x_new[active, :3] ** 2).mean(0))) * 1e3).round(1).tolist(),
              "pos_rms_mm": (1e3 * np.sqrt((x_new[active, 3:] ** 2).mean(0))).round(2).tolist(),
              "reproj_median_before_px": float(e0.median())}
        self.stats.append(st)
        log(f"  frame corrections: {st['active']}/{F} frames; rms rot [mdeg] {st['rot_rms_mdeg']} pos [mm] "
            f"{st['pos_rms_mm']} (reproj before {st['reproj_median_before_px']:.3f} px)")
        return st

    def save(self, path) -> None:
        np.savez(path, x=self.x, f_t=self.f_t.astype(np.int64), f_seg=self.f_seg)
