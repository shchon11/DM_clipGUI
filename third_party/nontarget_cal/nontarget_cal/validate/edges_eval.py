"""LiDAR-edge alignment checks (validation only; nothing here is used by the solves).

RGB (lidar_odo/eval_edges.py, same metric): LiDAR depth-discontinuity points (near side of a range
jump along a ring) of the sweep nearest the image, deskewed with the LiDAR odometry and moved to the
image time (moving windows) or projected as measured (parked moments), z-buffered, body-masked;
distance transform of the Canny map (CLAHE first at night) at their projections. Median in px.
Note (lidar_odo README 3.5c): at night this metric is 12-16 px for every calibration, including
correct ones (edges on dark tree canopies); the parked value is the meaningful one.

Thermal: the edge term's own data (LiDAR depth edges accumulated with the LO over 1.6 s, sub-pixel
thermal Canny edges): point-to-edge distance with the final calibration, per camera.

Voting gate (thermal_online's Hough-style check): for each image region (3x3) and whole image, the
number of LiDAR edge points within 1.5 px of an image edge is counted for every whole-pixel shift in
[-R, R]^2; a correct calibration peaks at (0, 0). Pass: whole-image peak within 1 px, contrast >= 1.8.
"""
from __future__ import annotations

import cv2
import numpy as np
import torch

from ..lo.lotraj import LOTraj, deskew, load_sweep, sweep_files, xyz
from ..rgb.rigba import kb_project


def sweep_edges(s: np.ndarray, jump: float = 0.3, rel: float = 0.05) -> np.ndarray:
    """Mask of points on the near side of a range discontinuity along their ring (online_calib/evaluate.py)."""
    r = np.sqrt(s["x"] ** 2 + s["y"] ** 2 + s["z"] ** 2)
    edge = np.zeros(len(s), bool)
    order = np.lexsort((s["t"], s["ring"]))
    rr, ring = r[order], s["ring"][order]
    same = ring[1:] == ring[:-1]
    d = rr[1:] - rr[:-1]
    thr = np.maximum(jump, rel * np.minimum(rr[1:], rr[:-1]))
    big = same & (np.abs(d) > thr)
    near_prev = big & (d > 0)
    near_next = big & (d < 0)
    e = np.zeros(len(s), bool)
    e[:-1] |= near_prev
    e[1:] |= near_next
    edge[order] = e
    return edge


def zbuffer_visible(uv: np.ndarray, depth: np.ndarray, W: int, H: int, cell: int = 6, tol: float = 0.08):
    ok = np.isfinite(uv).all(1) & (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    idx = np.flatnonzero(ok)
    vis = np.zeros(len(uv), bool)
    if not len(idx):
        return vis
    cx = (uv[idx, 0] // cell).astype(int)
    cy = (uv[idx, 1] // cell).astype(int)
    key = cy * (W // cell + 1) + cx
    zmin = np.full(key.max() + 1, np.inf)
    np.minimum.at(zmin, key, depth[idx])
    vis[idx] = depth[idx] <= zmin[key] * (1 + tol) + 0.2
    return vis


def _kb(Xc, intr):
    X = torch.tensor(Xc, dtype=torch.float64)
    I = torch.tensor(intr, dtype=torch.float64)
    n = len(X)
    uv = kb_project(X, *[I[i].expand(n) for i in range(7)], jac=False).numpy()
    uv[Xc[:, 2] < 0.5] = np.nan
    return uv


class Vote:
    """Accumulates shift-vote surfaces: S[region, dv, du] (region 0..8 = 3x3, 9 = whole image)."""

    def __init__(self, W, H, R=6, tol=1.5):
        self.W, self.H, self.R, self.tol = W, H, R, tol
        n = 2 * R + 1
        self.S = np.zeros((10, n, n))
        self.N = np.zeros(10)

    def add(self, uv, dist_map):
        near = (dist_map < self.tol).astype(np.uint8)
        u, v = uv[:, 0], uv[:, 1]
        reg = np.clip((u / self.W * 3).astype(int), 0, 2) + 3 * np.clip((v / self.H * 3).astype(int), 0, 2)
        self.N[:9] += np.bincount(reg, minlength=9)
        self.N[9] += len(u)
        for iy, sy in enumerate(range(-self.R, self.R + 1)):
            for ix, sx in enumerate(range(-self.R, self.R + 1)):
                xx = np.clip((u + sx).astype(int), 0, self.W - 1)
                yy = np.clip((v + sy).astype(int), 0, self.H - 1)
                hit = near[yy, xx]
                self.S[:9, iy, ix] += np.bincount(reg, weights=hit, minlength=9)
                self.S[9, iy, ix] += hit.sum()

    def summary(self, nmin=200, max_off=1.0, min_con=1.8):
        def peak(Sg):
            a, b = np.unravel_index(np.argmax(Sg), Sg.shape)

            def para(fm, f0, fp):
                den = fm - 2 * f0 + fp
                return 0.5 * (fm - fp) / den if den < 0 else 0.0
            n = Sg.shape[0]
            dv = (a - self.R) + (para(Sg[a - 1, b], Sg[a, b], Sg[a + 1, b]) if 0 < a < n - 1 else 0)
            du = (b - self.R) + (para(Sg[a, b - 1], Sg[a, b], Sg[a, b + 1]) if 0 < b < n - 1 else 0)
            return float(du), float(dv), float(Sg[a, b] / max(np.median(Sg), 1e-9))
        cells = []
        for r in range(9):
            if self.N[r] >= nmin:
                du, dv, c = peak(self.S[r])
                cells.append({"region": r, "du": round(du, 2), "dv": round(dv, 2), "contrast": round(c, 2), "n": int(self.N[r])})
        if self.N[9] < nmin:
            return {"pass": None, "n": int(self.N[9]), "cells": cells, "note": "too few edge points"}
        du, dv, c = peak(self.S[9])
        at0 = sum(1 for x in cells if np.hypot(x["du"], x["dv"]) <= 1.0)
        return {"pass": bool(np.hypot(du, dv) <= max_off and c >= min_con), "du": round(du, 2), "dv": round(dv, 2),
                "contrast": round(c, 2), "n": int(self.N[9]), "cells_at_origin": at0, "cells": cells}


class OrientedVote:
    """thermal_lo/code/vote_check.py semantics: along-ring LiDAR jumps (kind 0, vertical-ish boundaries)
    are counted against the distance map of vertical-ish image edges and constrain du; across-ring jumps
    (kind 1) against horizontal-ish edges and constrain dv. Profiles: du with dv summed over +-band, and
    vice versa. Gate (thermal_online): both whole-image peaks within 1 px and contrast >= 1.8."""

    def __init__(self, W, H, R=8, tol=1.5, band=2):
        self.W, self.H, self.R, self.tol, self.band = W, H, R, tol, band
        n = 2 * R + 1
        self.S = np.zeros((2, 10, n, n))
        self.N = np.zeros((2, 10))

    def add(self, uv, kind, dtv, dth):
        R = self.R
        ok = (uv[:, 0] >= R) & (uv[:, 0] < self.W - R - 1) & (uv[:, 1] >= R) & (uv[:, 1] < self.H - R - 1)
        uv, kind = uv[ok], kind[ok]
        ui, vi = np.rint(uv[:, 0]).astype(int), np.rint(uv[:, 1]).astype(int)
        reg = np.clip(ui * 3 // self.W, 0, 2) + 3 * np.clip(vi * 3 // self.H, 0, 2)
        for kk, M in ((0, dtv), (1, dth)):
            m = kind == kk
            if not m.any():
                continue
            u_, v_, r_ = ui[m], vi[m], reg[m]
            self.N[kk, :9] += np.bincount(r_, minlength=9)
            self.N[kk, 9] += m.sum()
            for a, dv in enumerate(range(-R, R + 1)):
                for b, du in enumerate(range(-R, R + 1)):
                    hit = M[v_ + dv, u_ + du] <= self.tol
                    self.S[kk, :9, a, b] += np.bincount(r_, weights=hit, minlength=9)
                    self.S[kk, 9, a, b] += hit.sum()

    @staticmethod
    def _peak1(prof):
        i = int(np.argmax(prof))
        x = float(i)
        if 0 < i < len(prof) - 1:
            den = prof[i - 1] - 2 * prof[i] + prof[i + 1]
            if den < 0:
                x += 0.5 * (prof[i - 1] - prof[i + 1]) / den
        return x, float(prof[i] / max(np.median(prof), 1e-9))

    def summary(self, nmin=300, max_off=1.0, min_con=1.8, min_cell_frac=0.0):
        R, b = self.R, self.band
        c0 = R
        out = {"cells": []}
        for r in range(10):
            pu = self.S[0, r, c0 - b:c0 + b + 1, :].sum(0)
            pv = self.S[1, r, :, c0 - b:c0 + b + 1].sum(1)
            du, cu = self._peak1(pu)
            dv, cv = self._peak1(pv)
            rec = {"du": round(du - R, 2), "contrast_u": round(cu, 2), "n_u": int(self.N[0, r]),
                   "dv": round(dv - R, 2), "contrast_v": round(cv, 2), "n_v": int(self.N[1, r])}
            if r < 9:
                out["cells"].append(rec)
            else:
                out.update(rec)
        if self.N[0, 9] < nmin or self.N[1, 9] < nmin:
            out["pass"] = None
            out["note"] = "too few edge points"
            return out
        out["contrast"] = min(out["contrast_u"], out["contrast_v"])
        out["pass"] = bool(abs(out["du"]) <= max_off and abs(out["dv"]) <= max_off and out["contrast_u"] >= min_con
                           and out["contrast_v"] >= min_con)
        offs = [abs(x["du"]) for x in out["cells"] if x["n_u"] >= nmin] + [abs(x["dv"]) for x in out["cells"] if x["n_v"] >= nmin]
        out["cells_at_origin"] = int(sum(o <= 1.0 for o in offs))
        out["cells_counted"] = len(offs)
        if offs and out["cells_at_origin"] < min_cell_frac * len(offs):
            out["pass"] = False
        return out


def rgb_edge_eval(ws, cfg, res, wins, parked, cams, frames=12, vote=True, log=print, max_range=40.0, threads=3):
    """Cameras in parallel threads; each LiDAR sweep is read, edge-classified and deskewed once for all
    cameras (the cameras of one window pick the same sweeps). Same numbers as one camera at a time."""
    import threading
    from concurrent.futures import ThreadPoolExecutor
    from pathlib import Path
    W, H = cfg["cameras"]["rgb_size"]
    masks = Path(cfg["paths"]["masks_dir"])
    cache, locks, glock = {}, {}, threading.Lock()

    def sweep(seg, f, hdr, tr):
        key = (seg, str(f), tr is not None)
        with glock:
            lk = locks.setdefault(key, threading.Lock())
        with lk:
            if key not in cache:
                s = load_sweep(f, 2.0, 80.0)
                cache[key] = (s, sweep_edges(s), deskew(s, hdr, tr) if tr is not None else xyz(s))
            return cache[key]

    def one(c):
        v = res["cameras"][c]
        T_CL, intr = np.array(v["T_cam_lidar"]), np.array(v.get("intr_kb", v.get("intr")))
        mk = cv2.imread(str(masks / f"{c}.png"), 0)
        per = {}
        for kind, segs in (("moving", wins), ("parked", parked)):
            errs, vt = [], Vote(W, H)
            for seg in segs:
                sd = ws.seg_dir(seg)
                files = sorted((sd / "cam" / c).glob("*.jpg"))
                if not files:
                    continue
                lf, lh = sweep_files(sd)
                if not len(lf):
                    continue
                tr = None
                if kind == "moving":
                    tr = LOTraj.load(ws.lo("ref", seg))
                    t0, t1 = tr.span()
                    files = [f for f in files if t0 + 1e8 < int(f.stem) < t1 - 1e8]
                pick = files[:: max(1, len(files) // frames)][:frames] if kind == "moving" else files[:1]
                for f in pick:
                    t_img = int(f.stem)
                    k = int(np.argmin(np.abs(lh + 50_000_000 - t_img)))
                    s, edge, P = sweep(seg, lf[k], lh[k], tr)
                    if tr is None:
                        XL = P
                    else:
                        Pw = P
                        T = tr.Tm([t_img])[0]
                        XL = (Pw - T[:3, 3]) @ T[:3, :3]
                    Xc = XL @ T_CL[:3, :3].T + T_CL[:3, 3]
                    uv = _kb(Xc, intr)
                    dist = np.linalg.norm(Xc, axis=1)
                    vis = zbuffer_visible(uv, dist, W, H)
                    sel = vis & edge & (dist < max_range)
                    u = uv[sel]
                    ui, vi = u[:, 0].astype(int), u[:, 1].astype(int)
                    inside = (mk[vi, ui] > 0) if mk is not None else np.ones(len(ui), bool)
                    img = cv2.imread(str(f), 0)
                    if img.mean() < 60:
                        img = cv2.createCLAHE(3.0, (8, 8)).apply(img)
                        E = cv2.Canny(cv2.GaussianBlur(img, (0, 0), 1.5), 30, 90)
                    else:
                        E = cv2.Canny(cv2.GaussianBlur(img, (0, 0), 1.2), 40, 110)
                    dtf = cv2.distanceTransform((E == 0).astype(np.uint8), cv2.DIST_L2, 5)
                    errs.append(dtf[vi[inside], ui[inside]])
                    if vote and inside.sum() > 50:
                        vt.add(u[inside], dtf)
            if errs:
                e = np.concatenate(errs)
                per[kind] = {"median_px": float(np.median(e)) if len(e) else None, "n": int(len(e)),
                             "within_2px": float(np.mean(e <= 2)) if len(e) else None}
                if vote:
                    per[kind]["vote"] = vt.summary(nmin=100 if kind == "parked" else 200)
        return per

    todo = [c for c in cams if c in res["cameras"]]
    with ThreadPoolExecutor(max(1, int(threads))) as ex:
        pers = list(ex.map(one, todo))
    out = {}
    for c, per in zip(todo, pers):
        out[c] = per
        log(f"{c}: " + ", ".join(f"{k} {v['median_px']:.2f} px (n {v['n']})" for k, v in per.items() if v.get("median_px") is not None))
    return out


def thermal_edge_eval(ws, cfg, res, segs, log=print):
    """Point-to-edge residual of the thermal edge term with the final calibration, per camera, and the
    voting gate on the same data (LO-accumulated LiDAR edges vs thermal Canny edges)."""
    from ..thermal import tba
    from ..thermal.solve import init_cams
    from ..thermal.timemodel import Plan
    plan = Plan(ws)
    cams_n = list(res["cams"])
    trajs = tba.Trajs(list(segs), str(ws.root / "lo"), plan.t_ref)
    idx = {s: i for i, s in enumerate(res["segs"])}
    # per-window dt of the solve where available, else the camera mean
    start = {}
    for c in cams_n:
        cc = dict(res["cameras"][c])
        start[c] = cc
    cams = init_cams(start, cams_n, len(segs))
    for i, c in enumerate(cams_n):
        d = res["cameras"][c]["dt_s"]
        cams.dt[i] = torch.tensor([d[idx[s]] if (isinstance(d, list) and s in idx and len(d) == len(res["segs"]))
                                   else res["cameras"][c]["dt_s_mean"] for s in segs], dtype=torch.float64)
    et = tba.EdgeTerm(list(segs), cams_n, str(ws.thermal_edges_dir()), "seg")
    uv, z = et.predict(cams, trajs)
    uv, z = uv.numpy(), z.numpy()
    W, H = cfg["cameras"]["thermal_size"]
    out = {}
    for ci, c in enumerate(cams_n):
        vt = OrientedVote(W, H)
        dists = []
        m_c = (et.cam.numpy() == ci)
        for g in np.unique(et.gfr[m_c]):
            ii = np.flatnonzero(m_c & (et.gfr == g))
            ok = (z[ii] > 0.5) & (uv[ii, 0] > 1) & (uv[ii, 0] < W - 2) & (uv[ii, 1] > 1) & (uv[ii, 1] < H - 2)
            ii = ii[ok]
            E = et.E[g]
            if not len(ii) or not len(E):
                continue
            ex = np.clip(np.rint(E[:, 0]).astype(int), 0, W - 1); ey = np.clip(np.rint(E[:, 1]).astype(int), 0, H - 1)
            maps = []
            for sel in (np.ones(len(E), bool), np.abs(E[:, 2]) >= 0.5, np.abs(E[:, 3]) >= 0.5):
                emap = np.zeros((H, W), np.uint8)
                emap[ey[sel], ex[sel]] = 1
                maps.append(cv2.distanceTransform((emap == 0).astype(np.uint8), cv2.DIST_L2, 5))
            dtf, dtv, dth = maps
            p = uv[ii]
            dists.append(dtf[p[:, 1].astype(int), p[:, 0].astype(int)])
            vt.add(p, et.kind[ii], dtv, dth)
        d = np.concatenate(dists) if dists else np.zeros(0)
        n_as = et.associate(cams, trajs, gate=3.0)
        rr = torch.cat([r for r, _, cam_, _ in et.terms(cams, trajs, jac=False)]) if n_as else torch.zeros(0)
        cam_of = et.cam[et.sel].numpy() if n_as else np.zeros(0, int)
        rc = rr.numpy()[cam_of == ci] if n_as else np.zeros(0)
        g = cfg["validation"]["gate"]
        out[c] = {"dist_to_edge_median_px": float(np.median(d)) if len(d) else None, "n": int(len(d)),
                  "assoc_point_to_line_median_px": float(np.median(np.abs(rc))) if len(rc) else None,
                  "vote": vt.summary(max_off=g["vote_max_offset_px"], min_con=g["vote_min_contrast"],
                                     min_cell_frac=g.get("vote_min_cell_frac", 0.0))}
        log(f"{c}: LiDAR-edge distance median {out[c]['dist_to_edge_median_px']} px, point-to-line "
            f"{out[c]['assoc_point_to_line_median_px']} px, vote {out[c]['vote'].get('pass')}")
    return out
