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
              min_dt_s=0.0, desc_max=None, patch_m=0.6, log=print, cache_dir: Path | None = None, snap_s=0.5, only=None,
              matcher="sift", learned: dict | None = None):
    """cache_dir: per-window results (<win>.npz: accepted pairs + impostor distances) are written there and
    re-used, so an interrupted run resumes window by window. snap_s: the descriptor of a track is taken at
    the observation nearest a multiple of snap_s, so each camera image of a window is read at most once
    per snap_s (was: one image per track -> disk-bound).
    matcher: "sift" (descriptor distance, threshold from impostors) or one of lmatch.MATCHERS (rectified crop
    pairs + learned matching + centre transfer, absolute threshold `learned["tol_px"]`); the cache of a learned
    matcher also keeps the geometric residuals and the match statistics of every candidate."""
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
            if len(img_cache) > 1200:          # ~2.8 GB of 1920x1200 grey frames
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
        d3 = np.linalg.norm(X[la] - X[lb], axis=1)
        keep = d3 <= gate_a + gate_b * dmax
        la, lb, dmax, d3 = la[keep], lb[keep], dmax[keep], d3[keep]
        dts = np.abs(tmean[la] - tmean[lb])
        keep = dts >= min_dt_s
        la, lb, dts, dmax, d3 = la[keep], lb[keep], dts[keep], dmax[keep], d3[keep]
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
            pairs.append((si, int(la[q]), int(lb[q]), Xm[q], float(dts[q]), [float(meds[0][q]), float(meds[1][q])],
                          [float(p90s[0][q]), float(p90s[1][q])], float(d3[q]), float(dmax[q])))
    log(f"cross-camera candidates {stats['candidates']}, geometry ok {stats['geometry_ok']}")
    if matcher == "none":                      # candidates only (bake-off / diagnostics)
        return pairs, stats
    if matcher != "sift":
        return _associate_learned(ws, segs, cams, T_LC, intr, X, keys, lm_cam, order, starts, ot, ouv, pairs, stats,
                                  cached, cache_dir, only, img, matcher, dict(learned or {}), snap_s, log)
    # ---- appearance: descriptors at the middle observation of each track, physical scale, vertical-up angle
    up = np.array([0.0, 0.0, 1.0])            # os_lidar z is up (the rig is level to ~1.5 deg)

    # requests: (window, landmark, merged point) -> keypoint in the middle observation; computed per image
    rng = np.random.default_rng(0)
    reqs = []                      # (si, l, Xm)
    for (si, la, lb, Xm, dts, res_, _p90, _d3, _dm) in pairs:
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
    for q, (si, la, lb, Xm, dts, res_, _p90, _d3, _dm) in enumerate(pairs):
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
    return _groups(accepted, desc_max, cams, lm_cam, keys, stats, log)


def _groups(accepted, thr, cams, lm_cam, keys, stats, log):
    """Union-find over the accepted links (score <= thr); a group with two tracks of one camera is dropped."""
    # ---- union-find
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for (si, la, lb, dist, dts, res_) in accepted:
        if dist > thr:
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


# ---------------------------------------------------------------------------------------------------------
# learned matcher (rgb/lmatch.py): rectified crop pairs, learned matching, centre transfer


def select_views(tr, cams, T_LC, ot, order, starts, lm_cam, la, lb, Xm, snap_s):
    """One observation per track and link: among the observations nearest each snap_s grid point (few images
    per camera and window), the pair whose viewing rays to the merged point are most alike (same side of the
    object, similar scale). Returns per link (obs A, obs B, centre A, centre B, R_world_cam A, R_world_cam B)."""
    snap = int(snap_s * 1e9)
    L = np.unique(np.concatenate([la, lb]))
    cand = {}
    allo = []
    for l in L:
        ob = order[starts[l]:starts[l + 1]]
        t = ot[ob].astype(np.int64)
        g = np.rint(t / snap).astype(np.int64)
        ph = np.abs(t - g * snap)
        o = np.lexsort((ph, g))
        first = np.r_[True, g[o][1:] != g[o][:-1]]
        sel = ob[o[first]]
        cand[l] = (len(allo), len(sel))
        allo.extend(sel.tolist())
    allo = np.array(allo, np.int64)
    Tw = tr.Tm(ot[allo].astype(np.int64))
    cam_of = lm_cam[np.repeat(L, [cand[l][1] for l in L])]
    TL = np.stack([T_LC[c] for c in cams])[cam_of]
    Rwc = np.einsum("nij,njk->nik", Tw[:, :3, :3], TL[:, :3, :3])
    cw = np.einsum("nij,nj->ni", Tw[:, :3, :3], TL[:, :3, 3]) + Tw[:, :3, 3]
    oa, ob_ = np.empty(len(la), np.int64), np.empty(len(la), np.int64)
    for q in range(len(la)):
        a0, na = cand[la[q]]
        b0, nb = cand[lb[q]]
        ra = Xm[q] - cw[a0:a0 + na]
        rb = Xm[q] - cw[b0:b0 + nb]
        ra /= np.linalg.norm(ra, axis=1, keepdims=True)
        rb /= np.linalg.norm(rb, axis=1, keepdims=True)
        i, j = np.unravel_index(np.argmax(ra @ rb.T), (na, nb))
        oa[q], ob_[q] = a0 + i, b0 + j
    return allo[oa], allo[ob_], cw[oa], cw[ob_], Rwc[oa], Rwc[ob_]


def verify_pairs(seg_dir, tr, cams, T_LC, intr, ot, ouv, order, starts, lm_cam, la, lb, Xm, M, img, crop_px=160,
                 crop_m=1.5, snap_s=0.5, min_local=6, chunk=4000, shift_px=None, epi=False):
    """Learned verification of links (la[q], lb[q]) with merged points Xm[q] of one window.
    M: an object with extract(crops) / match(fa, fb, S) (lmatch.Matcher). shift_px: optional per-link offset
    (native px, random direction) of the centre of crop B (bake-off: can the check see a wrong point that
    close?). Returns a dict of per-link arrays: err_px (centre transfer error in native px of camera B, inf
    when unverified), n_match, n_inl, n_loc, view_deg (angle between the two viewing rays)[, epi_n, epi_ok2]."""
    from .lmatch import centre_transfer, kb_unproject, ncc_refine, rectify, virtual_rotation
    n = len(la)
    S = int(crop_px)
    out = {k: np.zeros(n) for k in ("err_px", "n_match", "n_inl", "n_loc", "view_deg", "epi_n", "epi_ok2", "dx", "dy", "ncc", "ncc_px")}
    out["ncc_px"][:] = np.inf
    out["err_px"][:] = np.inf
    if not n:
        return out
    oa, ob, ca, cb, Ra, Rb = select_views(tr, cams, T_LC, ot, order, starts, lm_cam, la, lb, Xm, snap_s)
    ra, rb = Xm - ca, Xm - cb
    da, db = np.linalg.norm(ra, axis=1), np.linalg.norm(rb, axis=1)
    out["view_deg"] = np.degrees(np.arccos(np.clip(np.einsum("ni,ni->n", ra, rb) / (da * db), -1, 1)))
    cam_a, cam_b = lm_cam[la], lm_cam[lb]
    fa = np.array([intr[cams[c]][0] for c in cam_a])
    fb = np.array([intr[cams[c]][0] for c in cam_b])
    sphys = np.maximum.reduce([np.full(n, crop_m / S), da / fa, db / fb])
    fva, fvb = da / sphys, db / sphys
    uva, uvb = ouv[oa].astype(np.float64), ouv[ob].astype(np.float64).copy()
    if shift_px is not None:
        ang = np.random.default_rng(1).uniform(0, 2 * np.pi, n)
        uvb += np.asarray(shift_px)[:, None] * np.c_[np.cos(ang), np.sin(ang)]
    up = np.array([0.0, 0.0, 1.0])
    rays_a, rays_b = np.empty((n, 3)), np.empty((n, 3))
    for ci, c in enumerate(cams):
        m = cam_a == ci
        if m.any():
            rays_a[m] = kb_unproject(uva[m], intr[c])
        m = cam_b == ci
        if m.any():
            rays_b[m] = kb_unproject(uvb[m], intr[c])
    Rcv_a = np.stack([virtual_rotation(rays_a[q], Ra[q].T @ up) for q in range(n)])
    Rcv_b = np.stack([virtual_rotation(rays_b[q], Rb[q].T @ up) for q in range(n)])
    for c0 in range(0, n, chunk):
        idx = np.arange(c0, min(n, c0 + chunk))
        # requests: (camera, frame time) -> crops; rows 0..len-1 side A, len.. side B
        req = [(cams[cam_a[q]], int(ot[oa[q]]), 0, q) for q in idx] + [(cams[cam_b[q]], int(ot[ob[q]]), 1, q) for q in idx]
        req.sort(key=lambda r: (r[0], r[1]))
        crops = np.zeros((2, len(idx), S, S), np.uint8)
        ok = np.ones((2, len(idx)), bool)
        for (c, t, side, q) in req:
            g = img(seg_dir, c, t)
            if g is None:
                ok[side, q - c0] = False
                continue
            if side == 0:
                crops[0, q - c0] = rectify(g, intr[c], Rcv_a[q], fva[q], S)
            else:
                crops[1, q - c0] = rectify(g, intr[c], Rcv_b[q], fvb[q], S)
        feats = M.extract(crops.reshape(-1, S, S))
        mt = M.match(feats[:len(idx)], feats[len(idx):], S)
        for j, q in enumerate(idx):
            if not (ok[0, j] and ok[1, j]):
                continue
            ka, kb = mt[j]
            nm, ni, nl, e, inl = centre_transfer(ka, kb, S, min_local=min_local)
            out["n_match"][q], out["n_inl"][q], out["n_loc"][q] = nm, ni, nl
            out["err_px"][q] = e * fb[q] / fvb[q]
            if np.isfinite(e):
                out["dx"][q], out["dy"][q] = centre_transfer.last * fb[q] / fvb[q]
            # precise check: NCC of the centre template of A in B around the learned prediction (or the
            # centre when the matcher gave none); scale/rotation/fisheye are already removed by the rectification
            cc_ = (S - 1) / 2.0
            pred = cc_ + centre_transfer.last if np.isfinite(e) else None
            if pred is not None and np.abs(pred - cc_).max() > 12:
                pred = None if not np.isfinite(e) else pred
            off, sc = ncc_refine(crops[0, j], crops[1, j], pred)
            if off is not None:
                out["ncc"][q], out["ncc_px"][q] = sc, float(np.linalg.norm(off)) * fb[q] / fvb[q]
            if epi and nm:
                # epipolar error of every match with the known trajectory and calibration (native px of B)
                cc = (S - 1) / 2.0
                wa = np.c_[(ka - cc) / fva[q], np.ones(nm)] @ (Ra[q] @ Rcv_a[q]).T
                wb = np.c_[(kb - cc) / fvb[q], np.ones(nm)] @ (Rb[q] @ Rcv_b[q]).T
                base = cb[q] - ca[q]
                if np.linalg.norm(base) > 0.05:
                    nrm = np.cross(wa, base)
                    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
                    wb /= np.linalg.norm(wb, axis=1, keepdims=True)
                    ee = np.abs(np.arcsin(np.clip(np.einsum("ni,ni->n", nrm, wb), -1, 1))) * fb[q]
                    out["epi_n"][q], out["epi_ok2"][q] = nm, int((ee <= 2.0).sum())
    return out


def _associate_learned(ws, segs, cams, T_LC, intr, X, keys, lm_cam, order, starts, ot, ouv, pairs, stats, cached,
                       cache_dir, only, img, matcher, opt, snap_s, log):
    from .lmatch import Matcher
    tol = float(opt.get("tol_px", 1.0))            # NCC centre offset (native px of camera B)
    gross = float(opt.get("gross_px", 3.0))        # learned-matcher centre transfer (gross check)
    ncc_min = float(opt.get("ncc_min", 0.8))
    min_local = int(opt.get("min_local", 6))
    M = None
    by_seg = {}
    for p_ in pairs:
        by_seg.setdefault(p_[0], []).append(p_)
    new = {}
    for si, pl in sorted(by_seg.items()):
        if M is None:
            M = Matcher(matcher, device=opt.get("device"), max_kpts=int(opt.get("max_kpts", 256)))
        la = np.array([p_[1] for p_ in pl], np.int64)
        lb = np.array([p_[2] for p_ in pl], np.int64)
        Xm = np.stack([p_[3] for p_ in pl])
        r = verify_pairs(segs[si], LOTraj.load(ws.lo("ref", segs[si])), cams, T_LC, intr, ot, ouv, order, starts,
                         lm_cam, la, lb, Xm, M, lambda sd, c, t: img(sd, c, t), crop_px=int(opt.get("crop_px", 160)),
                         crop_m=float(opt.get("crop_m", 1.5)), snap_s=snap_s, min_local=min_local)
        rec = dict(la=la, lb=lb, dts=np.array([p_[4] for p_ in pl]), med=np.array([p_[5] for p_ in pl]),
                   p90=np.array([p_[6] for p_ in pl]), d3=np.array([p_[7] for p_ in pl]),
                   dmax=np.array([p_[8] for p_ in pl]), **r)
        new[si] = rec
        log(f"cross-camera links {matcher}: window {segs[si]} {len(la)} candidates, "
            f"{int((((r['err_px'] <= gross) | (matcher == 'ncc')) & (r['ncc'] >= ncc_min) & (r['ncc_px'] <= tol)).sum())} verified (extract {M.t_extract:.0f} s, match {M.t_match:.0f} s)")
        if cache_dir is not None:
            np.savez(Path(cache_dir) / f"{segs[si]}.npz", matcher=matcher, **rec)
    if cache_dir is not None and only is not None:
        for si in only:              # windows without candidates still get a (empty) cache file
            f = Path(cache_dir) / f"{segs[si]}.npz"
            if not f.exists():
                np.savez(f, matcher=matcher, la=np.zeros(0, np.int64), lb=np.zeros(0, np.int64), dts=np.zeros(0),
                         med=np.zeros((0, 2)), p90=np.zeros((0, 2)), d3=np.zeros(0), dmax=np.zeros(0),
                         **{k: np.zeros(0) for k in ("err_px", "n_match", "n_inl", "n_loc", "view_deg", "epi_n",
                                                     "epi_ok2", "dx", "dy", "ncc", "ncc_px")})
    recs = dict(new)
    for si, z_ in cached.items():
        recs[si] = {k: z_[k] for k in z_.files}
    # acceptance: centre transfer within tol, optional stricter geometric gates than the candidate stage
    gm, g9 = float(opt.get("accept_px_med", np.inf)), float(opt.get("accept_px_p90", np.inf))
    ga, gb = float(opt.get("accept_gate_a_m", np.inf)), float(opt.get("accept_gate_b", 0.0))
    accepted = []
    n_all = 0
    for si, z_ in sorted(recs.items()):
        n_all += len(z_["la"])
        for q in range(len(z_["la"])):
            learned_ok = matcher == "ncc" or (z_["n_loc"][q] >= min_local and z_["err_px"][q] <= gross)
            ok = (learned_ok and z_["ncc"][q] >= ncc_min
                  and z_["med"][q].max() <= gm and z_["p90"][q].max() <= g9
                  and z_["d3"][q] <= ga + gb * z_["dmax"][q])
            accepted.append((si, int(z_["la"][q]), int(z_["lb"][q]), float(z_["ncc_px"][q]) if ok else np.inf,
                             float(z_["dts"][q]), None))
    stats["matcher"] = matcher
    stats["verified_candidates"] = n_all
    stats["desc_threshold"] = tol
    return _groups(accepted, tol, cams, lm_cam, keys, stats, log)
