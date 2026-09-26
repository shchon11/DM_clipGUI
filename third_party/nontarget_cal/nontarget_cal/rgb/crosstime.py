"""Cross-camera, cross-TIME landmark association (multi-camera landmarks for the joint BA).

A static point is tracked by one camera (KLT) while it is in view; seconds later another camera (side,
rear) may track the same point in its own KLT track. KLT cannot bridge the time gap, so the link is made
in 3-D and verified in the images, guided by the fixed LiDAR-odometry trajectory and the current
calibration:

  1. candidates: landmarks of DIFFERENT cameras of the same window whose triangulated positions agree
     within gate = a + b * depth (the triangulation uncertainty grows with depth);
  2. geometry: the merged point must reproject into the observations of BOTH tracks (median and 90th
     percentile of the pixel residual) - i.e. each camera sees it where the other camera's track and the
     trajectory say it is;
  3. appearance: SIFT descriptors at one observation of each track, computed at a scale proportional to
     1/depth (same physical patch) and with the orientation of the projected world vertical (so a rolled
     camera and a yawed one describe the patch in the same frame); accepted when the descriptor distance is
     below `desc_max` (calibrated against the distances to wrong nearby candidates, reported);
  4. groups by union-find; a group with two tracks of the same camera is dropped.
Output: the lidar_odo `--cross` merge format {seg, cam, track, group} consumed by rgb/solve.py.
"""
from __future__ import annotations

import ast
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree

from ..lo.lotraj import LOTraj


def _kb_project(Xc, intr):
    fx, fy, cx, cy, k1, k2, k3, k4 = intr
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    D = np.array([k1, k2, k3, k4], float)
    uv = cv2.fisheye.projectPoints(Xc.reshape(-1, 1, 3).astype(np.float64), np.zeros(3), np.zeros(3), K, D)[0]
    return uv.reshape(-1, 2)


def associate(ws, result: dict, state_path: Path, gate_a=0.03, gate_b=0.004, max_px_med=2.0, max_px_p90=4.0,
              min_dt_s=0.0, desc_max=None, patch_m=0.6, log=print, cache_dir: Path | None = None, snap_s=0.5, only=None):
    """cache_dir: per-window results (<win>.npz: accepted pairs + impostor distances) are written there and
    re-used, so an interrupted run resumes window by window. snap_s: the descriptor of a track is taken at
    the observation nearest a multiple of snap_s, so each camera image of a window is read at most once
    per snap_s (was: one image per track -> disk-bound)."""
    z = np.load(state_path, allow_pickle=True)
    X, lm_seg = z["X"], z["lm_seg"]
    ocam, olm, ouv, ot = z["obs_cam"], z["obs_lm"], z["obs_uv"], z["obs_t"]
    keys = [ast.literal_eval(k) for k in z["lm_key"]]
    segs, cams = result["segs"], result["cams"]
    T_LC = {c: np.array(result["cameras"][c]["T_lidar_cam"]) for c in cams}
    intr = {c: np.array(result["cameras"][c]["intr_kb"]) for c in cams}
    M = len(X)
    order = np.argsort(olm, kind="stable")
    starts = np.searchsorted(olm[order], np.arange(M + 1))
    lm_cam = np.full(M, -1)
    lm_cam[olm] = ocam
    stats = {"candidates": 0, "geometry_ok": 0, "desc_ok": 0, "same_time": 0, "cross_time": 0, "per_cam_pair": {}}
    pairs, desc_true, desc_false = [], [], []
    sift = cv2.SIFT_create()
    img_cache = {}

    def img(seg, cam, t):
        k = (seg, cam, int(t))
        if k not in img_cache:
            g = cv2.imread(str(ws.seg_dir(seg) / "cam" / cam / f"{int(t)}.jpg"), cv2.IMREAD_GRAYSCALE)
            if g is not None and g.mean() < 60:
                g = cv2.createCLAHE(3.0, (8, 8)).apply(g)
            img_cache[k] = g
            if len(img_cache) > 400:
                img_cache.pop(next(iter(img_cache)))
        return img_cache[k]

    def grouped_median_p90(gid, v, n):
        o = np.lexsort((v, gid))
        g, vs = gid[o], v[o]
        st = np.searchsorted(g, np.arange(n + 1))
        cnt = np.diff(st)
        med = np.where(cnt > 0, vs[np.minimum(st[:-1] + cnt // 2, len(vs) - 1)], np.inf)
        p90 = np.where(cnt > 0, vs[np.minimum(st[:-1] + (9 * cnt) // 10, len(vs) - 1)], np.inf)
        return med, p90

    obs_seg = lm_seg[olm]
    cached = {}
    if cache_dir is not None:
        Path(cache_dir).mkdir(parents=True, exist_ok=True)
        for si, seg in enumerate(segs):
            f = Path(cache_dir) / f"{seg}.npz"
            if f.exists():
                cached[si] = np.load(f)
    for si, seg in enumerate(segs):
        if si in cached or (only is not None and si not in only):
            continue
        tr = LOTraj.load(ws.lo("ref", seg))
        L = np.flatnonzero(lm_seg == si)
        if len(L) < 2:
            continue
        oi = np.flatnonzero(obs_seg == si)
        ut, uinv = np.unique(ot[oi].astype(np.int64), return_inverse=True)
        Tw = tr.Tm(ut)
        Rw, pw = Tw[uinv, :3, :3], Tw[uinv, :3, 3]
        cL = np.stack([T_LC[c][:3, 3] for c in cams])[ocam[oi]]
        cw = np.einsum("nij,nj->ni", Rw, cL) + pw
        dep = np.linalg.norm(X[olm[oi]] - cw, axis=1)
        dsum = np.bincount(olm[oi], dep, minlength=M)
        dcnt = np.bincount(olm[oi], minlength=M)
        tsum = np.bincount(olm[oi], ot[oi].astype(np.float64) * 1e-9 - 1.7e9, minlength=M)
        dmean = dsum / np.maximum(dcnt, 1)
        tmean = tsum / np.maximum(dcnt, 1)
        tree = cKDTree(X[L])
        pr = tree.query_pairs(r=gate_a + gate_b * 60.0, output_type="ndarray")
        if not len(pr):
            continue
        la, lb = L[pr[:, 0]], L[pr[:, 1]]
        keep = lm_cam[la] != lm_cam[lb]
        la, lb = la[keep], lb[keep]
        dmax = np.maximum(dmean[la], dmean[lb])
        keep = np.linalg.norm(X[la] - X[lb], axis=1) <= gate_a + gate_b * dmax
        la, lb = la[keep], lb[keep]
        dts = np.abs(tmean[la] - tmean[lb])
        keep = dts >= min_dt_s
        la, lb, dts = la[keep], lb[keep], dts[keep]
        stats["candidates"] += int(len(la))
        if not len(la):
            continue
        wa, wb = 1 / dmean[la] ** 2, 1 / dmean[lb] ** 2
        Xm = (wa[:, None] * X[la] + wb[:, None] * X[lb]) / (wa + wb)[:, None]
        # residuals of the merged point in every observation of both tracks
        pos_in_oi = {}
        # observations of each landmark within this window (oi sorted by landmark)
        oo = oi[np.argsort(olm[oi], kind="stable")]
        st = np.searchsorted(olm[oo], np.arange(M + 1))
        meds, p90s = [], []
        for side in (la, lb):
            cnt = st[side + 1] - st[side]
            gid = np.repeat(np.arange(len(side)), cnt)
            idx = oo[np.concatenate([np.arange(st[l], st[l + 1]) for l in side])] if len(side) else np.zeros(0, int)
            t = ot[idx].astype(np.int64)
            k = np.searchsorted(ut, t)
            R_, p_ = Tw[k, :3, :3], Tw[k, :3, 3]
            XL = np.einsum("nji,nj->ni", R_, Xm[gid] - p_)
            e = np.full(len(idx), np.inf)
            for ci, c in enumerate(cams):
                m = ocam[idx] == ci
                if not m.any():
                    continue
                Xc = (XL[m] - T_LC[c][:3, 3]) @ T_LC[c][:3, :3]
                ok = Xc[:, 2] > 0.5
                ee = np.full(m.sum(), np.inf)
                if ok.any():
                    ee[ok] = np.linalg.norm(_kb_project(Xc[ok], intr[c]) - ouv[idx[m]][ok], axis=1)
                e[m] = ee
            med, p90 = grouped_median_p90(gid, e, len(side))
            meds.append(med); p90s.append(p90)
        good = (meds[0] <= max_px_med) & (meds[1] <= max_px_med) & (p90s[0] <= max_px_p90) & (p90s[1] <= max_px_p90)
        stats["geometry_ok"] += int(good.sum())
        for q in np.flatnonzero(good):
            pairs.append((si, int(la[q]), int(lb[q]), Xm[q], float(dts[q]), [float(meds[0][q]), float(meds[1][q])]))
    log(f"cross-camera candidates {stats['candidates']}, geometry ok {stats['geometry_ok']}")
    # ---- appearance: descriptors at the middle observation of each track, physical scale, vertical-up angle
    up = np.array([0.0, 0.0, 1.0])            # os_lidar z is up (the rig is level to ~1.5 deg)

    # requests: (window, landmark, merged point) -> keypoint in the middle observation; computed per image
    rng = np.random.default_rng(0)
    reqs = []                      # (si, l, Xm)
    for (si, la, lb, Xm, dts, res_) in pairs:
        reqs.append((si, la, Xm)); reqs.append((si, lb, Xm))
    # impostors: random different-camera landmark pairs among the candidates of the same window
    imp = []
    by_seg = {}
    for p_ in pairs:
        by_seg.setdefault(p_[0], set()).update((p_[1], p_[2]))
    for si, ls in by_seg.items():
        ls = np.array(sorted(ls))
        for _ in range(min(300, len(ls))):
            a_, b_ = rng.choice(ls, 2, replace=False)
            if lm_cam[a_] != lm_cam[b_]:
                imp.append((si, int(a_), int(b_)))
    for (si, a_, b_) in imp:
        reqs.append((si, a_, X[a_])); reqs.append((si, b_, X[b_]))
    kps = {}                        # (si, cam, t) -> list of (req index, keypoint)
    trs = {}
    for ri, (si, l, Xm) in enumerate(reqs):
        ob = order[starts[l]:starts[l + 1]]
        tt = ot[ob].astype(np.int64)
        ph = (tt % int(snap_s * 1e9)).astype(np.float64)
        ph = np.minimum(ph, snap_s * 1e9 - ph)
        k = ob[int(np.argmin(ph + 1e-3 * np.abs(np.arange(len(ob)) - len(ob) // 2)))]
        c = cams[lm_cam[l]]
        tk = int(ot[k])
        if si not in trs:
            trs[si] = LOTraj.load(ws.lo("ref", segs[si]))
        Tw = trs[si].Tm([tk])[0]
        P = np.stack([Xm, Xm + 0.2 * (Tw[:3, :3] @ up)])
        XL = (P - Tw[:3, 3]) @ Tw[:3, :3]
        Xc = (XL - T_LC[c][:3, 3]) @ T_LC[c][:3, :3]
        if (Xc[:, 2] < 0.3).any():
            continue
        uv = _kb_project(Xc, intr[c])
        dirv = uv[1] - uv[0]
        ang = float(np.degrees(np.arctan2(dirv[1], dirv[0])) + 90.0) % 360.0
        size = float(np.clip(intr[c][0] * patch_m / max(np.linalg.norm(Xc[0]), 1.0), 8.0, 96.0))
        kps.setdefault((si, c, tk), []).append((ri, cv2.KeyPoint(float(ouv[k][0]), float(ouv[k][1]), size, ang)))
    des = {}
    for (si, c, tk), lst in kps.items():
        g = img(segs[si], c, tk)
        if g is None:
            continue
        for ri, kp in lst:            # one at a time: compute() may drop keypoints near the border
            _, d_ = sift.compute(g, [kp])
            if d_ is not None and len(d_):
                des[ri] = d_[0] / max(np.linalg.norm(d_[0]), 1e-9)
    accepted = []
    for q, (si, la, lb, Xm, dts, res_) in enumerate(pairs):
        a_, b_ = des.get(2 * q), des.get(2 * q + 1)
        if a_ is None or b_ is None:
            continue
        dist = float(np.linalg.norm(a_ - b_))
        desc_true.append(dist)
        accepted.append((si, la, lb, dist, dts, res_))
    base = 2 * len(pairs)
    imp_dist = []
    for q in range(len(imp)):
        a_, b_ = des.get(base + 2 * q), des.get(base + 2 * q + 1)
        d_ = float(np.linalg.norm(a_ - b_)) if (a_ is not None and b_ is not None) else np.nan
        imp_dist.append(d_)
        if np.isfinite(d_):
            desc_false.append(d_)
    if cache_dir is not None:
        for si, seg in enumerate(segs):
            if si in cached or (only is not None and si not in only):
                continue
            acc = [x for x in accepted if x[0] == si]
            imp_d = [d for (s_i, _, _), d in zip(imp, imp_dist) if s_i == si]
            np.savez(Path(cache_dir) / f"{seg}.npz", la=np.array([x[1] for x in acc], np.int64),
                     lb=np.array([x[2] for x in acc], np.int64), dist=np.array([x[3] for x in acc]),
                     dts=np.array([x[4] for x in acc]), imp=np.array(imp_d))
    for si, z_ in cached.items():
        for la_, lb_, d_, t_ in zip(z_["la"], z_["lb"], z_["dist"], z_["dts"]):
            accepted.append((si, int(la_), int(lb_), float(d_), float(t_), None))
            desc_true.append(float(d_))
        desc_false += list(z_["imp"])
    dt_arr, df_arr = np.array(desc_true), np.array(desc_false)
    if desc_max is None:
        # threshold: 10th percentile of the impostor distances (<= 10 % false accepts among impostors)
        desc_max = float(np.percentile(df_arr, 10)) if len(df_arr) else 1.0
    stats["desc_threshold"] = desc_max
    stats["desc_candidates_median"] = float(np.median(dt_arr)) if len(dt_arr) else None
    stats["desc_impostor_median"] = float(np.median(df_arr)) if len(df_arr) else None
    # ---- union-find
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for (si, la, lb, dist, dts, res_) in accepted:
        if dist > desc_max:
            continue
        stats["desc_ok"] += 1
        stats["cross_time" if dts >= 0.5 else "same_time"] += 1
        pk = f"{cams[lm_cam[la]]}|{cams[lm_cam[lb]]}"
        stats["per_cam_pair"][pk] = stats["per_cam_pair"].get(pk, 0) + 1
        parent[find(la)] = find(lb)
    groups = {}
    for l in list(parent):
        groups.setdefault(find(l), []).append(l)
    out = {"seg": [], "cam": [], "track": [], "group": []}
    ng, dropped = 0, 0
    for root, mem in groups.items():
        cs = [lm_cam[l] for l in mem]
        if len(mem) < 2 or len(set(cs)) != len(cs):
            dropped += len(mem) >= 2
            continue
        for l in mem:
            s_, c_, t_ = keys[l]
            out["seg"].append(s_); out["cam"].append(c_); out["track"].append(int(t_)); out["group"].append(ng)
        ng += 1
    stats["groups"] = ng
    stats["groups_dropped_same_camera"] = dropped
    stats["landmarks_linked"] = len(out["seg"])
    per_cam = {}
    for c_ in out["cam"]:
        per_cam[c_] = per_cam.get(c_, 0) + 1
    stats["linked_per_camera"] = per_cam
    log(json.dumps({k: v for k, v in stats.items() if k != "per_cam_pair"}))
    return out, stats


def associate_resumable(ws, result: dict, state_path: Path, cache_dir: Path, log=print, **kw):
    """One window at a time (each cached as soon as it is done), then the groups over all windows."""
    for si, seg in enumerate(result["segs"]):
        if not (Path(cache_dir) / f"{seg}.npz").exists():
            associate(ws, result, state_path, cache_dir=cache_dir, only={si}, log=log, **kw)
            log(f"cross-camera links: window {seg} done")
    return associate(ws, result, state_path, cache_dir=cache_dir, only=set(), log=log, **kw)
