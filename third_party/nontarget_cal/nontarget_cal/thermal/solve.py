"""Thermal BA with the trajectory FIXED from refined LiDAR odometry.

Verbatim numerics of thermal_lo/code/run_tba.py (load_tracks, build, init_cams, reindex, tie_stats,
lidar_report, main, add_stereo); the command line became `run_tba(ws, plan, segs, out, **options)`.
Removed: --xy-prior, --cold, --axis-offset (studies), the thermal_online start ("prev"): a start is
given explicitly (a previous result or the rig design).
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch

from ..rgb.solve import _viz_observer, _viz_result
from . import tba
from .tba import NCP, PN, Cams, Obs, Ties, Trajs, all_residuals, solve, triangulate
from .timemodel import time_model

CAMS = ("thermal_left", "thermal_right")
DEFAULTS = dict(cams=list(CAMS), free=["rot", "pos", "f", "rs", "dt"], ties=False, tie_sigma=0.03, stereo=None,
                edges=False, edge_weight=1.0, edge_sigma=1.0, edge_maxdepth=40.0, edge_mindepth=1.5,
                time="smooth", dt_mode="seg", min_len=12, step=3, max_tracks=4000, max_obs=40,
                min_parallax=1.5, max_dist=150.0, min_dist=0.0, huber=1.0, iters=25, rounds=3,
                pos_prior=1.0, f_prior=0.05, pp_prior=20.0, k_prior=0.1, seed=0, step_tol=None,
                lidar_report=True)


def load_tracks(ws, plan, seg, cam, a, rng, span, tmodel):
    ZERO_NS = plan.t_ref
    z = np.load(ws.thermal_tracks(seg, cam))
    hdr = z["header_ns"]
    bag = plan.bag(seg)
    h_all, tf_all, _ = time_model(ws, bag, cam, plan.wins_of_bag(bag), tmodel)
    k = np.searchsorted(h_all, hdr)
    assert np.all(h_all[k] == hdr)
    tf = (tf_all[k] - ZERO_NS) * 1e-9
    of, ot, xy = z["obs_frame"], z["obs_track"], z["obs_xy"].astype(np.float64)
    ok = (tf[of] > span[0] + 0.15) & (tf[of] < span[1] - 0.15)
    of, ot, xy = of[ok], ot[ok], xy[ok]
    o = np.lexsort((of, ot))
    of, ot, xy = of[o], ot[o], xy[o]
    uid, start, cnt = np.unique(ot, return_index=True, return_counts=True)
    good = cnt >= a.min_len
    uid, start, cnt = uid[good], start[good], cnt[good]
    pri = cnt + rng.random(len(cnt))
    sel = np.argsort(-pri)[:a.max_tracks]
    idx = []
    for s, n in zip(start[sel], cnt[sel]):
        i = np.arange(s, s + n)[::a.step]
        if len(i) > a.max_obs:
            i = i[np.linspace(0, len(i) - 1, a.max_obs).astype(int)]
        idx.append(i)
    idx = np.concatenate(idx) if idx else np.zeros(0, int)
    return ot[idx], tf[of[idx]], xy[idx], hdr[of[idx]]


def build(ws, plan, a, segs, cams, trajs, rng):
    cam_i, seg_i, lm_i, uv_l, tf_l, key_l = [], [], [], [], [], []
    nlm = 0
    lm_seg, lm_cam, lm_tid = [], [], []
    for si, seg in enumerate(segs):
        span = trajs.span[si]
        for ci, cam in enumerate(cams):
            if not ws.thermal_tracks(seg, cam).exists():
                continue
            tid, tf, uv, hdr = load_tracks(ws, plan, seg, cam, a, rng, span, a.time)
            if not len(tid):
                continue
            uu, inv = np.unique(tid, return_inverse=True)
            cam_i.append(np.full(len(tid), ci)); seg_i.append(np.full(len(tid), si))
            lm_i.append(inv + nlm); uv_l.append(uv); tf_l.append(tf)
            lm_seg.append(np.full(len(uu), si)); lm_cam.append(np.full(len(uu), ci)); lm_tid.append(uu)
            nlm += len(uu)
    seg_i = np.concatenate(seg_i)
    dseg = seg_i if a.dt_mode == "seg" else np.zeros_like(seg_i)
    obs = Obs(np.concatenate(cam_i), seg_i, np.concatenate(lm_i), np.concatenate(uv_l),
              np.concatenate(tf_l), dseg)
    return obs, np.concatenate(lm_seg), np.concatenate(lm_cam), np.concatenate(lm_tid)


def init_cams(start: dict, cams, nseg):
    """start: {cam: {"T_cam_lidar", "intr" (fx fy cx cy k1 k2), "rs_s", "dt_s"}} (a previous result's
    cameras, a --init calibration, or the rig design)."""
    T, I, rs, dt = [], [], [], []
    for c in cams:
        cc = start[c]
        T.append(np.linalg.inv(np.array(cc["T_cam_lidar"], float))); I.append(np.array(cc["intr"], float))
        rs.append(float(cc["rs_s"]))
        d = cc.get("dt_s_mean", cc.get("dt_s"))
        dt.append(np.full(nseg, float(np.mean(d))))
    return Cams(cams, np.array(T), np.array(I), np.array(rs), np.array(dt), nseg)


def reindex(obs, keep_obs, M, min_obs=3):
    obs = obs.subset(keep_obs)
    cnt = torch.bincount(obs.lm, minlength=M)
    ok_lm = cnt >= min_obs
    k2 = ok_lm[obs.lm]
    obs = obs.subset(k2)
    new_id = torch.full((M,), -1, dtype=torch.long)
    kept = torch.nonzero(ok_lm).flatten()
    new_id[kept] = torch.arange(len(kept))
    obs.lm = new_id[obs.lm]
    return obs, kept


def tie_stats(ws, plan, cams, trajs, obs, X, lm_seg, segs, gate, want_stats=False):
    """Landmark -> LiDAR-plane association in the LO map of its segment (lidar_odo SegMap)."""
    from ..rgb.data import SegMap
    ZERO_NS = plan.t_ref
    M = len(X)
    tm = torch.zeros(M, dtype=tba.DT).index_add_(0, obs.lm, tba.obs_time(cams, obs))
    tm = (tm / torch.bincount(obs.lm, minlength=M).clamp_min(1)).numpy()
    Xn = X.numpy()
    from ..rgb.solve import assoc_windows
    li, ln, lq = assoc_windows(ws, segs, lm_seg, Xn, (ZERO_NS + tm * 1e9).astype(np.int64), gate)
    if not want_stats:
        return (li, ln, lq), None
    # ray geometry from the first observation of each landmark
    ol = obs.lm.numpy()
    first = np.full(M, -1)
    first[ol[::-1]] = np.arange(len(ol))[::-1]
    o = obs.subset(torch.as_tensor(first[li]))
    R, p = trajs.pose(o.seg, tba.obs_time(cams, o))
    cw = (torch.einsum("nij,nj->ni", R, cams.c[o.cam]) + p).numpy()
    ray = Xn[li] - cw
    depth = np.linalg.norm(ray, axis=1)
    d = ray / depth[:, None]
    nd = np.einsum("ni,ni->n", ln, d)
    s = np.einsum("ni,ni->n", ln, lq - Xn[li]) / np.where(np.abs(nd) > 1e-3, nd, np.nan)
    return (li, ln, lq), {"ray_err": s, "depth": depth, "cam": o.cam.numpy(), "incid": np.abs(nd),
                          "plane_d": np.einsum("ni,ni->n", ln, Xn[li] - lq)}


def lidar_report(ts, cams_names):
    good = ts["incid"] > 0.3
    rel = ts["ray_err"][good] / ts["depth"][good]
    rep = {"n": int(len(ts["depth"])), "plane_abs_median_m": float(np.median(np.abs(ts["plane_d"]))), "by_depth": {}, "per_cam": {}}
    for lo_, hi_ in ((0, 5), (5, 10), (10, 20), (20, 60)):
        m = (ts["depth"][good] >= lo_) & (ts["depth"][good] < hi_)
        if m.sum() > 20:
            rep["by_depth"][f"{lo_}-{hi_}m"] = {"n": int(m.sum()), "ray_err_median_m": float(np.median(ts["ray_err"][good][m])),
                                             "rel_depth_err_median": float(np.median(rel[m]))}
    for i, c in enumerate(cams_names):
        m = ts["cam"][good] == i
        if m.sum() > 20:
            rep["per_cam"][c] = {"n": int(m.sum()), "abs_plane_median_m": float(np.median(np.abs(ts["plane_d"][good][m]))),
                                 "rel_depth_err_median": float(np.median(rel[m]))}
    return rep


def run_tba(ws, plan, segs, out, start: dict, rig=None, calib_in: bool = False, log=print, threads=None,
            kernel=None, **opts):
    """One thermal solve. start: see init_cams. rig: optional (T_rig_lidar, {cam: T_cam_lidar of the RGB
    cameras}, R_L_V) for rig-frame reporting. calib_in: hold the calibration (held-out; with "dt" in
    free only the per-window time offsets are re-fitted)."""
    a = SimpleNamespace(**{**DEFAULTS, **opts})
    a.out = Path(out)
    a.out.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(int(threads or os.environ.get("OMP_NUM_THREADS", "4")))
    tba.KERNEL[0] = kernel or os.environ.get("NONTARGET_BA_KERNEL") or "torch"
    if tba.KERNEL[0] == "numba":
        try:
            from . import fasttba
            fasttba.set_threads(threads)
        except Exception as ex:  # noqa  (numba missing / incompatible): the validated torch path
            log(f"numba kernels unavailable ({ex}); using torch")
            tba.KERNEL[0] = "torch"
    ZERO_NS = plan.t_ref
    LO_DIR = str(ws.root / "lo")
    tba.EDGE = None
    tba.PRIOR_LIN.clear()
    logf = open(a.out / "log.txt", "a")
    _log = log

    def log(*s, **k):
        msg = " ".join(str(x) for x in s)
        _log(msg)
        logf.write(msg + "\n"); logf.flush()

    t_start = time.time()
    log(f"=== run_tba segs={list(segs)} options={ {k: v for k, v in opts.items()} } kernel={tba.KERNEL[0]}")
    rng = np.random.default_rng(a.seed)
    segs, cams_n = list(segs), list(a.cams)
    trajs = Trajs(segs, LO_DIR, ZERO_NS)
    obs, lm_seg, lm_cam, lm_tid = build(ws, plan, a, segs, cams_n, trajs, rng)
    nseg_dt = len(segs) if a.dt_mode == "seg" else 1
    cams = init_cams(start, cams_n, nseg_dt)
    M = int(obs.lm.max()) + 1
    if a.stereo:
        obs, M, lm_seg = add_stereo(ws, plan, a, obs, M, lm_seg, lm_cam, lm_tid, segs, cams_n, log)
    log(f"built: {len(obs)} obs, {M} landmarks, {len(segs)} segments ({time.time() - t_start:.0f} s)")
    X, par = triangulate(cams, trajs, obs, M)
    r, Xc = all_residuals(cams, trajs, obs, X)
    e = r.norm(dim=1)
    far = (Xc.norm(dim=1) > a.max_dist) | (Xc.norm(dim=1) < a.min_dist)
    bad_obs = (Xc[:, 2] < 0.5) | (e > 20)
    bad_lm = par < np.radians(a.min_parallax)
    n_par = int(bad_lm.sum())
    bad_lm.index_put_((obs.lm[bad_obs],), torch.tensor(True))
    log(f"filter: {n_par} landmarks with parallax < {a.min_parallax} deg; bad obs: behind/near {int((Xc[:, 2] < 0.5).sum())}, "
        f"reproj > 20 px {int((e > 20).sum())}, farther than {a.max_dist} m {int((Xc.norm(dim=1) > a.max_dist).sum())}; "
        f"landmarks dropped in total {int(bad_lm.sum())} of {M}")
    obs, kept = reindex(obs, ~bad_lm[obs.lm] & ~far, M)
    X, lm_seg = X[kept], lm_seg[kept.numpy()]
    M = len(X)
    log(f"after triangulation filter: {len(obs)} obs, {M} landmarks; per camera "
        f"{torch.bincount(obs.cam, minlength=len(cams_n)).tolist()}; initial reproj median {e.median():.3f} px")
    # ---- free mask and priors
    P = cams.P
    free = torch.zeros(P, dtype=torch.bool)
    prior = torch.full((P,), np.inf, dtype=tba.DT)
    for i in range(cams.C):
        b = i * NCP
        if "rot" in a.free:
            free[b:b + 3] = True
        if "pos" in a.free:
            free[b + 3:b + 6] = True; prior[b + 3:b + 6] = a.pos_prior
        if "f" in a.free:
            free[b + 6] = True; prior[b + 6] = a.f_prior
        if "pp" in a.free:
            free[b + 7:b + 9] = True; prior[b + 7:b + 9] = a.pp_prior
        if "k" in a.free:
            free[b + 9:b + 11] = True; prior[b + 9:b + 11] = a.k_prior
        if "k1" in a.free:
            free[b + 9] = True; prior[b + 9] = a.k_prior
        if "rs" in a.free:
            free[b + 11] = True
        if "aspect" in a.free:
            free[b + 12] = True; prior[b + 12] = 0.02
    if "dt" in a.free:
        free[cams.C * NCP:] = True
    if calib_in:
        keep_dt = "dt" in a.free
        free[:] = False
        if keep_dt:
            free[cams.C * NCP:] = True
    if a.edges:
        tba.EDGE = tba.EdgeTerm(segs, cams_n, str(ws.thermal_edges_dir()), a.dt_mode,
                                sigma=a.edge_sigma, weight=a.edge_weight, max_depth=a.edge_maxdepth,
                                min_depth=a.edge_mindepth)
        n_as = tba.EDGE.associate(cams, trajs, gate=4.0)
        log(f"edge term: {len(tba.EDGE)} LiDAR edge points, {n_as} associated (gate 4 px), weight {a.edge_weight}, sigma {a.edge_sigma} px")
    # ---- stage A: rotation + time only
    freeA = free.clone()
    for i in range(cams.C):
        freeA[i * NCP + 3:i * NCP + NCP] = False
    log("stage A (rotation, dt)" if freeA.any() else "landmarks only")
    cams, X, _ = solve(cams, trajs, obs, X, freeA, prior, iters=8, huber=a.huber, log=log, step_tol=a.step_tol,
                       on_iteration=_viz_observer(ws, segs, a.out.name, "A", 8, a.huber,
                                                  thermal=True, held=calib_in))
    ties = None
    info = {"cov": None, "fidx": torch.nonzero(free).flatten()}
    for rnd in range(a.rounds):
        log(f"stage B round {rnd}")
        if a.ties:
            t_ = time.time()
            (li, ln, lq), _ = tie_stats(ws, plan, cams, trajs, obs, X, lm_seg, segs, 0.3 if rnd == 0 else 0.15)
            ties = Ties(li, ln, lq, sigma=a.tie_sigma)
            log(f"  ties: {len(ties)} landmarks on planes ({100 * len(ties) / M:.1f}%) ({time.time() - t_:.0f} s)")
        if a.edges:
            n_as = tba.EDGE.associate(cams, trajs, gate=4.0 if rnd == 0 else 3.0)
            log(f"  edges: {n_as} associated; {tba.EDGE.stats(cams, trajs)}")
        last = rnd == a.rounds - 1
        t_ = time.time()
        cams, X, info = solve(cams, trajs, obs, X, free, prior, ties=ties, iters=a.iters, huber=a.huber,
                              want_cov=last and free.any(), log=log, step_tol=a.step_tol,
                              on_iteration=_viz_observer(ws, segs, a.out.name, f"B{rnd + 1}", a.iters, a.huber,
                                                         thermal=True, held=calib_in))
        log(f"  LM: {info.get('iters')} iterations ({time.time() - t_:.0f} s)")
        r, Xc = all_residuals(cams, trajs, obs, X)
        e = r.norm(dim=1)
        if last:
            break
        thr = max(2.5, 3 * 1.4826 * float(e.median()))
        keep = (e < thr) & (Xc[:, 2] > 0.5)
        n0 = len(obs)
        obs, kept = reindex(obs, keep, M)
        X, lm_seg = X[kept], lm_seg[kept.numpy()]
        M = len(X)
        log(f"  outliers > {thr:.2f} px removed: {n0 - len(obs)} obs; {M} landmarks left")
    # ---- LiDAR check (a report only: landmark-to-plane association after the solve; nothing reads it for
    # the intermediate solves, whose association with a cold map cache costs ~45 s)
    rep = None
    if a.lidar_report:
        (li, ln, lq), ts = tie_stats(ws, plan, cams, trajs, obs, X, lm_seg, segs, 0.3, want_stats=True)
        rep = lidar_report(ts, cams_n)
        log("LiDAR check: n %d |plane| median %.3f m; %s" % (rep["n"], rep["plane_abs_median_m"], ", ".join(
            f"{k} {100 * v['rel_depth_err_median']:+.2f}% (n {v['n']})" for k, v in rep["by_depth"].items())))
    # ---- report
    r, Xc = all_residuals(cams, trajs, obs, X)
    e = r.norm(dim=1)
    sig0 = float(1.4826 * e.median() / np.sqrt(np.log(4)))
    cov, fidx = info["cov"], info["fidx"]
    sd = np.full(P, np.nan)
    if cov is not None:
        sd[fidx.numpy()] = torch.sqrt(torch.diagonal(cov)).numpy() * sig0
    T_L_C = cams.T_L_C()
    intr = cams.intr_array()
    T_rig_lidar, rgbT, R_L_V = rig if rig is not None else (None, None, None)
    res = {"argv": ["run_tba", list(segs), {k: str(v) for k, v in opts.items()}], "segs": segs, "cams": cams_n, "free": a.free, "ties": a.ties, "stereo": a.stereo,
           "time_model": a.time, "dt_mode": a.dt_mode, "n_obs": len(obs), "n_lm": M,
           "n_ties": 0 if ties is None else len(ties), "reproj_median_px": float(e.median()),
           "sigma_px": sig0, "wall_s": time.time() - t_start, "cost": info["cost"], "lidar_check": rep,
           "time_info": {f"{b}/{c}": time_model(ws, b, c, plan.wins_of_bag(b), a.time)[2]
                         for b in sorted({plan.bag(s_) for s_ in segs}) for c in cams_n}, "cameras": {},
           "edges": tba.EDGE.stats(cams, trajs) if a.edges else None}
    Xcn = Xc.norm(dim=1).numpy()
    for i, c in enumerate(cams_n):
        b = i * NCP
        m = (obs.cam == i).numpy()
        axis = T_L_C[i][:3, 2]
        Tcl = np.linalg.inv(T_L_C[i])
        if T_rig_lidar is not None:
            T_cam_rig = Tcl @ np.linalg.inv(T_rig_lidar)
            pos_rig = -T_cam_rig[:3, :3].T @ T_cam_rig[:3, 3]
            f5 = rgbT["camera_front5"]
            c5 = -f5[:3, :3].T @ f5[:3, 3]
        else:
            T_cam_rig, pos_rig, c5 = np.full((4, 4), np.nan), np.full(3, np.nan), np.full(3, np.nan)
        sd_axis = np.nan
        if cov is not None and bool(free[b + 3:b + 6].all()):
            fi = fidx.tolist()
            pi = [fi.index(b + k) for k in (3, 4, 5)]
            Sg = cov[pi][:, pi].numpy() * sig0 ** 2
            sd_axis = float(np.sqrt(axis @ Sg @ axis))
        dts = cams.dt[i].numpy()
        res["cameras"][c] = {
            "T_lidar_cam": T_L_C[i].tolist(), "T_cam_lidar": Tcl.tolist(), "T_cam_rig": T_cam_rig.tolist(),
            "pos_rig_m": pos_rig.tolist(), "centre_L_m": T_L_C[i][:3, 3].tolist(),
            "along_axis_vs_front5_m": float(axis @ (T_L_C[i][:3, 3] - c5)),
            "intr": intr[i].tolist(), "focal_px": float(intr[i][0]), "rs_s": float(cams.rs[i]),
            "aspect_fy_over_fx": float(intr[i][1] / intr[i][0]),
            "dt_s": dts.tolist(), "dt_s_mean": float(dts.mean()), "dt_s_sd": float(dts.std()),
            "sd": dict(zip(PN, sd[b:b + NCP].tolist())),
            "sd_dt_s": sd[cams.C * NCP + i * cams.S: cams.C * NCP + (i + 1) * cams.S].tolist(),
            "sd_along_axis_m": sd_axis, "n_obs": int(m.sum()), "reproj_median_px": float(e[torch.as_tensor(m)].median()),
            "depth_median_m": float(np.median(Xcn[m])), "frac_depth_lt8m": float(np.mean(Xcn[m] < 8)),
            "frac_depth_lt4m": float(np.mean(Xcn[m] < 4))}
    if len(cams_n) == 2:
        TA, TB = [np.array(res["cameras"][c]["T_cam_lidar"]) for c in cams_n]
        T_B_A = TB @ np.linalg.inv(TA)
        res["stereo"] = {"left_centre_in_right_mm": (1e3 * T_B_A[:3, 3]).tolist(),
                         "baseline_mm": float(1e3 * np.linalg.norm(T_B_A[:3, 3]))}
    res["segs_bag"] = {s_: plan.bag(s_) for s_ in segs}
    tmpj = a.out / "result.json.tmp"
    tmpj.write_text(json.dumps(res, indent=1))
    np.savez(a.out / "state.npz", X=X.numpy(), lm_seg=lm_seg, obs_cam=obs.cam.numpy(), obs_seg=obs.seg.numpy(),
             obs_lm=obs.lm.numpy(), obs_uv=obs.uv.numpy(), obs_tf=obs.tf.numpy(), obs_dseg=obs.dseg.numpy())
    os.replace(tmpj, a.out / "result.json")
    log(f"done in {time.time() - t_start:.0f} s: reproj median {e.median():.3f} px, sigma {sig0:.3f}")
    for i, c in enumerate(cams_n):
        rc = res["cameras"][c]
        log(f"  {c}: centre_L {np.round(rc['centre_L_m'], 4).tolist()} rig {np.round(rc['pos_rig_m'], 4).tolist()} "
            f"axis-vs-front5 {rc['along_axis_vs_front5_m']:+.4f} m (formal {1e3 * rc['sd_along_axis_m']:.1f} mm) "
            f"f {rc['focal_px']:.2f} cx {rc['intr'][2]:.1f} cy {rc['intr'][3]:.1f} k {rc['intr'][4]:+.4f} {rc['intr'][5]:+.4f} "
            f"rs {1e3 * rc['rs_s']:.1f} ms dt {1e3 * rc['dt_s_mean']:.2f} +- {1e3 * rc['dt_s_sd']:.2f} ms")
    if "stereo" in res and isinstance(res["stereo"], dict):
        log(f"  stereo: baseline {res['stereo']['baseline_mm']:.1f} mm, left in right {np.round(res['stereo']['left_centre_in_right_mm'], 1).tolist()}")
    logf.close()
    _viz_result(ws, segs, a.out.name, cams, obs, e, info["cost"], a.huber,
                thermal=True, held=calib_in)
    return res


def add_stereo(ws, plan, a, obs, M, lm_seg, lm_cam, lm_tid, segs, cams_n, log):
    """Cross-camera observations: link_stereo.py found, for landmarks (seg, cam, track id), their
    position in the OTHER camera's image at that camera's frame times."""
    z = np.load(a.stereo)
    zseg, zsrc, zdst, ztrack = z["seg"], z["src_cam"], z["dst_cam"], z["track"]

    def codes(col, names):
        """name -> index (-1 unknown), vectorised over the unique names (same as a per-row dict lookup)."""
        u, inv = np.unique(col, return_inverse=True)
        m = {n: i for i, n in enumerate(names)}
        return np.array([m.get(str(x), -1) for x in u], np.int64)[inv.ravel()] if len(u) else np.zeros(0, np.int64)
    si, src, dst = codes(zseg, segs), codes(zsrc, cams_n), codes(zdst, cams_n)
    # landmark of (segment, camera, track id): sorted composite keys + searchsorted (= the dict lookup)
    B = np.int64(1) << 40
    kl = (np.asarray(lm_seg, np.int64) * 8 + np.asarray(lm_cam, np.int64)) * B + np.asarray(lm_tid, np.int64)
    kq = (si * 8 + src) * B + ztrack.astype(np.int64)
    lmk = np.full(len(kq), -1, np.int64)
    if len(kl):
        order = np.argsort(kl, kind="stable")
        j = np.clip(np.searchsorted(kl[order], kq), 0, len(kl) - 1)
        hit = (si >= 0) & (src >= 0) & (kl[order][j] == kq)
        lmk[hit] = order[j][hit]
    ok = (lmk >= 0) & (dst >= 0)
    ZERO_NS = plan.t_ref
    tf = np.zeros(ok.sum())
    dcam = dst[ok]
    hdr = z["dst_header_ns"][ok]
    sbag = np.array([plan.bag(segs[i]) for i in si[ok]])
    for b in np.unique(sbag):
        for ci, c in enumerate(cams_n):
            m = (dcam == ci) & (sbag == b)
            if not m.any():
                continue
            hh, tt, _ = time_model(ws, b, c, plan.wins_of_bag(b), a.time)
            k = np.searchsorted(hh, hdr[m])
            tf[m] = (tt[k] - ZERO_NS) * 1e-9
    seg_arr = si[ok]
    dseg = seg_arr if a.dt_mode == "seg" else np.zeros_like(seg_arr)
    so = Obs(dcam, seg_arr, lmk[ok], z["dst_uv"][ok].astype(np.float64), tf, dseg)
    log(f"stereo: {int(ok.sum())} cross-camera observations of {len(np.unique(lmk[ok]))} landmarks")
    return Obs.cat(obs, so), M, lm_seg

