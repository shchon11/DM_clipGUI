"""Rig bundle adjustment on KLT tracks with the trajectory fixed from LiDAR odometry.

Verbatim numerics of /hdd/DM_calib/lidar_odo/run_ba.py (`build`, `init_cams`, `reindex`,
`tie_stats`, `main`); the command line became `run_ba(ws, segs, cams, out, **options)` and the
fixed directories a Workspace. Variants: motion (no ties), tie (ties), held-out (calib_in).
The cross-camera LoFTR merges of the lidar_odo product are not part of the tool (lidar_odo
README V5: they move cameras by <= 15 mm and do not improve repeatability).
"""
from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch

from .data import SegMap, load_tracks, load_traj
from .rigba import NP, PNAMES, Cameras, Obs, Ties, project_all, solve, triangulate, unproject_obs

DEFAULTS = dict(cross=None, free=["rot", "pos", "f"], ties=False, tie_sigma=0.03, init="nominal", traj="ref", min_len=12, step=3,
                max_tracks=3000, min_parallax=2.0, max_dist=60.0, huber=1.5, iters=25, dt_ms=0.0, t_window=None,
                pos_prior=1.0, f_prior=0.05, pp_prior=30.0, k_prior=0.05, seed=0, seed_aspect=False)


def build(ws, segs, cams, a, rng):
    cam_i, lm_i, uv_l, t_l, pid_l, lm_seg, lm_key = [], [], [], [], [], [], []
    PR, Pp = [], []
    npose = 0
    trajs = {}
    merge = None
    if getattr(a, "cross", None):
        # multi-camera landmarks: {(seg, cam, track): group} (lidar_odo run_ba.py --cross format)
        z = np.load(a.cross, allow_pickle=True)
        merge = {(str(s_), str(c_), int(t_)): int(g_) for s_, c_, t_, g_ in
                 zip(z["seg"], z["cam"], z["track"], z["group"])}
    nlm = 0
    gmap = {}
    for si, seg in enumerate(segs):
        tr = load_traj(ws, seg, a.traj)
        trajs[seg] = tr
        t0, t1 = tr.span()
        for ci, cam in enumerate(cams):
            if not ws.tracks(seg, cam).exists():
                continue
            tid, tns, uv = load_tracks(ws, seg, cam, min_len=a.min_len, step=a.step,
                                       max_tracks=a.max_tracks, rng=rng,
                                       t_range=(t0 - a.dt_ns, t1 - a.dt_ns))
            if a.t_window is not None:
                k = (tns >= t0 + int(a.t_window[0] * 1e9)) & (tns <= t0 + int(a.t_window[1] * 1e9))
                tid, tns, uv = tid[k], tns[k], uv[k]
            if not len(tid):
                continue
            uu, inv = np.unique(tid, return_inverse=True)
            lut = np.empty(len(uu), np.int64)
            for j, u in enumerate(uu):
                key = (seg, cam, int(u))
                g = merge.get(key) if merge else None
                gk = ("g", seg, g) if g is not None else key
                if gk not in gmap:
                    gmap[gk] = nlm
                    lm_seg.append(si)
                    lm_key.append(gk)
                    nlm += 1
                lut[j] = gmap[gk]
            ut, tinv = np.unique(tns, return_inverse=True)
            t = ut + a.dt_ns
            PR.append(tr.R(t)); Pp.append(tr.p(t))
            pid_l.append(npose + tinv)
            npose += len(ut)
            cam_i.append(np.full(len(tid), ci))
            lm_i.append(lut[inv])
            uv_l.append(uv)
            t_l.append(tns)
    obs = Obs(np.concatenate(cam_i), np.concatenate(lm_i), np.concatenate(uv_l),
              np.concatenate(pid_l), np.concatenate(PR), np.concatenate(Pp))
    return obs, np.concatenate(t_l), np.array(lm_seg), lm_key, trajs


def nominal_cams(cams, design: dict, square: bool = True) -> Cameras:
    """NO board information about position: every camera starts at the LiDAR origin, rotation = the
    design angles (the board's rounded to the nearest 10 deg per Euler angle), lens = the team's seed.
    Same numbers as lidar_odo run_ba.py --init nominal."""
    from scipy.spatial.transform import Rotation as Rot
    T, I = [], []
    for c in cams:
        d = design["rgb"][c]
        Tlc = np.eye(4)
        Tlc[:3, :3] = Rot.from_euler("ZYX", np.array(d["design_R_lidar_cam_euler_ZYX_deg"], float), degrees=True).as_matrix()
        T.append(Tlc)
        I.append(np.array(d["seed_intr_kb"], float))
    I = np.array(I)
    if square:
        I[:, 0] = I[:, 1] = 0.5 * (I[:, 0] + I[:, 1])
    return Cameras(cams, np.array(T), I)


def calib_cams(cams, calib: dict) -> Cameras:
    """calib: {cam: (T_cam_lidar 4x4, intr8)} -> Cameras (as run_ba.py --init-from / --calib-in)."""
    return Cameras(cams, np.array([np.linalg.inv(calib[c][0]) for c in cams]), np.array([calib[c][1] for c in cams]))


def reindex(obs, keep_obs, M):
    """Drop observations, then landmarks with < 3 remaining observations; compact ids."""
    obs = obs.subset(keep_obs)
    cnt = torch.bincount(obs.lm, minlength=M)
    ok_lm = cnt >= 3
    k2 = ok_lm[obs.lm]
    obs = obs.subset(k2)
    new_id = torch.full((M,), -1, dtype=torch.long)
    kept = torch.nonzero(ok_lm).flatten()
    new_id[kept] = torch.arange(len(kept))
    obs.lm = new_id[obs.lm]
    return obs, kept, torch.nonzero(keep_obs).flatten()[k2]


def tie_stats(ws, cams, obs, X, lm_seg, tns, segs, gate):
    """Associate every landmark with the LiDAR map of its window. Returns (Ties arrays, stats)
    where stats per tie: signed ray error (m, + = LiDAR surface farther than the landmark),
    depth (m) from the observing camera at the landmark's middle observation, camera index."""
    M = len(X)
    tm = np.zeros(M)
    np.add.at(tm, obs.lm.cpu().numpy(), tns.astype(np.float64))
    tm /= np.maximum(np.bincount(obs.lm.cpu().numpy(), minlength=M), 1)
    Xn = X.cpu().numpy()
    li, ln, lq = assoc_windows(ws, segs, lm_seg, Xn, tm.astype(np.int64), gate)
    first = np.full(M, -1)
    ol = obs.lm.cpu().numpy()
    first[ol[::-1]] = np.arange(len(ol))[::-1]
    o = first[li]
    ci = obs.cam.cpu().numpy()[o]
    c_w = np.einsum("nij,nj->ni", obs.RWL.cpu().numpy()[o], cams.c.cpu().numpy()[ci]) + obs.pWL.cpu().numpy()[o]
    ray = Xn[li] - c_w
    depth = np.linalg.norm(ray, axis=1)
    d = ray / depth[:, None]
    nd = np.einsum("ni,ni->n", ln, d)
    s = np.einsum("ni,ni->n", ln, lq - Xn[li]) / np.where(np.abs(nd) > 1e-3, nd, np.nan)
    return (li, ln, lq), {"ray_err": s, "depth": depth, "cam": ci, "incid": np.abs(nd),
                          "plane_d": np.einsum("ni,ni->n", ln, Xn[li] - lq)}


TIE_THREADS = int(os.environ.get("NONTARGET_TIE_THREADS", "3"))


def assoc_windows(ws, segs, lm_seg, Xn, tm_ns, gate, threads=None):
    """Landmark -> LiDAR-plane association, one window per thread (numpy sorting, cKDTree build/query
    release the GIL). Windows are independent and the results are concatenated in window order, so the
    output is identical to the sequential loop of lidar_odo/run_ba.py:tie_stats."""
    from concurrent.futures import ThreadPoolExecutor

    def one(si_seg):
        si, seg = si_seg
        sel = np.flatnonzero(lm_seg == si)
        if not len(sel):
            return None
        sm = SegMap(ws.lo("map", seg))
        i, n, q, th = sm.associate(Xn[sel], tm_ns[sel], gate=gate)
        return sel[i], n, q
    nt = max(1, min(threads or TIE_THREADS, len(segs)))
    if nt == 1:
        out = [one(x) for x in enumerate(segs)]
    else:
        with ThreadPoolExecutor(nt) as ex:
            out = list(ex.map(one, enumerate(segs)))
    out = [o for o in out if o is not None]
    return (np.concatenate([o[0] for o in out]), np.concatenate([o[1] for o in out]),
            np.concatenate([o[2] for o in out]))


def pick_device(device: str | None) -> str:
    """'cpu', 'cuda' or 'auto' (CUDA when available). The CPU path is bit-for-bit the validated one; on
    the GPU, float64 sums are accumulated in a different (and not deterministic) order, so results
    differ at the 1e-12 level."""
    device = (device or os.environ.get("NONTARGET_DEVICE") or "cpu").lower()
    if device in ("auto", "cuda", "gpu"):
        if torch.cuda.is_available():
            return "cuda"
        if device != "auto":
            raise RuntimeError("CUDA requested but not available")
    return "cpu"


_VIZ_OBSERVER_WARNED = False


def _viz_observer(ws, segs, solve_name, solver_pass, total_iterations, huber, *,
                  thermal=False, held=False, dt_s=0.0, terminal=False):
    """Observe cached accepted residuals without re-evaluating or mutating the solver.

    Each camera's cost is its Huber pixel loss, excluding depth penalties, ties and priors.
    The complete LM objective is emitted separately as objective_cost. No uncertainty is inferred.
    Held-out/validation calibration poses must never replace the production calibration in the UI.
    """
    global _VIZ_OBSERVER_WARNED
    try:
        if held or solve_name.startswith(("half", "heldout")):
            return None
        from ..viz import get_viz
        viz = get_viz()
        if not viz.enabled:
            return None
        windows = list(segs)
        camera_indices = None
        preview_index = 0

        def observe(cams, obs, residual_norms, objective_cost, iteration, accepted):
            nonlocal camera_indices, preview_index
            if not terminal and not viz.due("state"):
                return
            if camera_indices is None:
                ids = obs.cam.detach().cpu().numpy()
                camera_indices = [np.flatnonzero(ids == i) for i in range(cams.C)]
            errors = residual_norms.detach().cpu().numpy()
            transforms, intrinsics = cams.T_L_C(), cams.intr_array()
            states = {}
            for i, name in enumerate(cams.names):
                intr = intrinsics[i]
                ec = errors[camera_indices[i]]
                rho = np.where(ec <= huber, 0.5 * ec * ec, huber * (ec - 0.5 * huber))
                state = {
                    "T_cam_lidar": np.linalg.inv(transforms[i]).tolist(),
                    "K": [[float(intr[0]), 0.0, float(intr[2])],
                          [0.0, float(intr[1]), float(intr[3])], [0.0, 0.0, 1.0]],
                    "D": ([float(intr[4]), float(intr[5]), 0.0, 0.0, 0.0]
                          if thermal else intr[4:8].tolist()),
                    "model": "plumb_bob" if thermal else "equidistant",
                    "reprojection_px": float(np.sqrt(np.mean(ec * ec))) if len(ec) else None,
                    "reprojection_median_px": float(np.median(ec)) if len(ec) else None,
                    "cost": float(rho.sum()), "cost_source": "huber_reprojection_only",
                    "n_obs": len(ec),
                    "metric_source": ("solver.result_residuals" if terminal else
                                      "solver.accepted_residuals" if accepted else "solver.initial_residuals"),
                    "state": "converged" if terminal else "converging", "sigma_rot_deg": None, "sigma_pos_mm": None,
                    "dt_s": dt_s, "time_offset_s": dt_s,
                    "solve_name": solve_name, "solver_pass": solver_pass, "windows": windows,
                    "iteration": iteration, "total_iterations": total_iterations,
                    "purpose": "calibration", "accepted": accepted,
                }
                if thermal:
                    dts = cams.dt[i].detach().cpu().numpy()
                    state.update(time_offset_s=float(dts.mean()), dt_s=float(dts[0]),
                                 time_offsets_s=dts.tolist(), rs_s=float(cams.rs[i]),
                                 row_readout_s=float(cams.rs[i]),
                                 time_offset_by_window_s={window: float(dts[j if len(dts) > 1 else 0])
                                                         for j, window in enumerate(windows)})
                states[name] = state
            # Terminal state must survive the local sampler. The asynchronous writer
            # still applies its global output cap and coalesces the latest camera states.
            snapshot = {"stage": "thermal" if thermal else "rgb_ba", "cameras": states,
                        "solve_name": solve_name, "solver_pass": solver_pass, "windows": windows, "window": len(windows),
                        "iteration": iteration, "total_iterations": total_iterations,
                        "accepted": accepted, "objective_cost": float(objective_cost),
                        "purpose": "calibration",
                        "status_text": f"실제 계산 · {solve_name} / {solver_pass}"}
            if terminal:
                snapshot["pass"] = "result"
            viz.publish(snapshot, force=terminal)
            if windows:
                preview_window = windows[preview_index]
                scheduled = False
                for name, state in states.items():
                    if viz.preview(ws, preview_window, name, state):
                        scheduled = True
                # The producer applies camera selection and <=1 Hz scheduling.
                # Advance only when a preview request actually entered its mailbox.
                if scheduled:
                    preview_index = (preview_index + 1) % len(windows)

        return observe
    except Exception:
        if not _VIZ_OBSERVER_WARNED:
            logging.getLogger(__name__).warning("Visualization observer unavailable", exc_info=True)
            _VIZ_OBSERVER_WARNED = True
        return None


def _viz_result(ws, segs, solve_name, cams, obs, residual_norms, objective_cost, huber, **options):
    """Publish the already-computed final residuals without sampler loss or solver I/O waits."""
    global _VIZ_OBSERVER_WARNED
    try:
        observer = _viz_observer(ws, segs, solve_name, "result", None, huber, terminal=True, **options)
        if observer is not None:
            observer(cams, obs, residual_norms, objective_cost, None, None)
    except Exception:
        if not _VIZ_OBSERVER_WARNED:
            logging.getLogger(__name__).warning("Final visualization observation skipped", exc_info=True)
            _VIZ_OBSERVER_WARNED = True


def run_ba(ws, segs, cams, out: Path, start: Cameras | None = None, calib_in: dict | None = None,
           design: dict | None = None, log=print, threads: int | None = None, device: str | None = None,
           **opts) -> dict:
    """One solve. start: initial Cameras (None -> nominal from `design`); calib_in: hold this
    calibration ({cam: (T_cam_lidar, intr8)}) and only re-triangulate (held-out evaluation)."""
    a = SimpleNamespace(**{**DEFAULTS, **opts})
    print = lambda *x, **k: log(" ".join(str(y) for y in x))  # noqa: A001,E731
    torch.set_num_threads(int(threads or os.environ.get("OMP_NUM_THREADS", "4")))
    dev = pick_device(device)
    torch.set_default_device(dev)          # every tensor of this solve lives on `dev` (one solve per process)
    print(f"device: {dev}")
    a.dt_ns = int(a.dt_ms * 1e6)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    t_start = time.time()
    rng = np.random.default_rng(a.seed)
    cams = list(cams)
    obs, tns, lm_seg, lm_key, trajs = build(ws, segs, cams, a, rng)
    if calib_in is not None:
        cams_s = calib_cams(cams, calib_in)
    elif start is not None:
        cams_s = Cameras(start.names, start.T_L_C(), start.intr_array())    # same numbers, on `dev`
    else:
        cams_s = nominal_cams(cams, design, square=not a.seed_aspect)
    cams_ = cams_s
    C = cams_.C
    M = int(obs.lm.max()) + 1
    print(f"built: {len(obs)} obs, {M} landmarks, {C} cameras, {len(segs)} segments "
          f"({time.time() - t_start:.0f} s)", flush=True)
    # ---- triangulate, filter
    X, par = triangulate(cams_, obs, M, unproject_obs(cams_))
    r, Xc = project_all(cams_, obs, X)
    e = r.norm(dim=1)
    dist = torch.cat([(X[obs.lm[k:k + 1_000_000]] - obs.chunk(slice(k, k + 1_000_000)).pWL).norm(dim=1)
                      for k in range(0, len(obs), 1_000_000)])
    bad_obs = (Xc[:, 2] < 0.5) | (e > 30) | (dist > a.max_dist)
    bad_lm = torch.zeros(M, dtype=torch.bool)
    bad_lm |= par < np.radians(a.min_parallax)
    bad_lm.index_put_((obs.lm[bad_obs],), torch.tensor(True))
    keep = ~bad_lm[obs.lm]
    obs, kept, oi = reindex(obs, keep, M)
    X, lm_seg, tns = X[kept], lm_seg[kept.cpu().numpy()], tns[oi.cpu().numpy()]
    lm_key = [lm_key[i] for i in kept.tolist()]
    M = len(X)
    print(f"after triangulation filter: {len(obs)} obs, {M} landmarks; "
          f"per camera obs {torch.bincount(obs.cam, minlength=C).tolist()}", flush=True)
    # ---- free mask and priors
    free = torch.zeros(C, NP, dtype=torch.bool)
    prior = torch.full((C, NP), np.inf, dtype=torch.float64)
    if "rot" in a.free:
        free[:, 0:3] = True
    if "pos" in a.free:
        free[:, 3:6] = True
        prior[:, 3:6] = a.pos_prior
    if "f" in a.free:
        free[:, 6] = True
        prior[:, 6] = a.f_prior
    if "pp" in a.free:
        free[:, 7:9] = True
        prior[:, 7:9] = a.pp_prior
    if "k" in a.free:
        free[:, 9:11] = True
        prior[:, 9:11] = a.k_prior
    if "aspect" in a.free:
        free[:, 12] = True
        prior[:, 12] = 0.02
    if "k3" in a.free:
        free[:, 11] = True
        prior[:, 11] = a.k_prior
    if calib_in is not None:
        free[:] = False
    # ---- stage A: rotations (+landmarks) only, then everything requested
    freeA = free.clone()
    freeA[:, 3:] = False
    print("stage A (rotations only)" if freeA.any() else "landmarks only (calibration held)", flush=True)
    cams_, X, _ = solve(cams_, obs, X, freeA, prior, iters=8, huber=a.huber, want_cov=False, verbose=True,
                       on_iteration=_viz_observer(ws, segs, out.name, "A", 8, a.huber,
                                                  held=calib_in is not None, dt_s=a.dt_ns * 1e-9))
    ties = None
    info = {"cov_free": None, "fidx": torch.zeros(0, dtype=torch.long)}
    for rnd in range(3):
        print(f"stage B round {rnd}", flush=True)
        if a.ties:
            (li, ln, lq), _ = tie_stats(ws, cams_, obs, X, lm_seg, tns, segs, 0.3 if rnd == 0 else 0.15)
            ties = Ties(li, ln, lq, sigma=a.tie_sigma)
            print(f"  ties: {len(ties)} landmarks on planes ({100 * len(ties) / M:.1f}%)", flush=True)
        if free.any():
            cams_, X, info = solve(cams_, obs, X, free, prior, ties=ties, iters=a.iters, huber=a.huber,
                                   want_cov=(rnd == 2),
                                   on_iteration=_viz_observer(ws, segs, out.name, f"B{rnd + 1}", a.iters, a.huber,
                                                              held=calib_in is not None, dt_s=a.dt_ns * 1e-9))
        else:
            cams_, X, info = solve(cams_, obs, X, free, prior, ties=ties, iters=a.iters, huber=a.huber,
                                   want_cov=False,
                                   on_iteration=_viz_observer(ws, segs, out.name, f"B{rnd + 1}", a.iters, a.huber,
                                                              held=calib_in is not None, dt_s=a.dt_ns * 1e-9))
        r, Xc = project_all(cams_, obs, X)
        e = r.norm(dim=1)
        thr = max(3.0, 3 * 1.4826 * float(e.median()))
        keep = (e < thr) & (Xc[:, 2] > 0.5)
        if rnd == 2:
            break
        n_before = len(obs)
        obs, kept, oi = reindex(obs, keep, M)
        X, lm_seg, tns = X[kept], lm_seg[kept.cpu().numpy()], tns[oi.cpu().numpy()]
        lm_key = [lm_key[i] for i in kept.tolist()]
        M = len(X)
        print(f"  outliers > {thr:.2f} px removed: {n_before - len(obs)} obs; {M} landmarks left", flush=True)
    # ---- LiDAR check of the landmarks (used in the fit only with ties)
    (li, ln, lq), ts = tie_stats(ws, cams_, obs, X, lm_seg, tns, segs, 0.3)
    good = ts["incid"] > 0.3
    rel = ts["ray_err"][good] / ts["depth"][good]
    tie_rep = {"n": int(len(li)), "plane_abs_median_m": float(np.median(np.abs(ts["plane_d"]))),
               "by_depth": {}}
    for lo_, hi_ in ((0, 8), (8, 15), (15, 25), (25, 60)):
        m = (ts["depth"][good] >= lo_) & (ts["depth"][good] < hi_)
        if m.sum() > 20:
            tie_rep["by_depth"][f"{lo_}-{hi_}m"] = {"n": int(m.sum()),
                                                  "ray_err_median_m": float(np.median(ts["ray_err"][good][m])),
                                                  "rel_depth_err_median": float(np.median(rel[m]))}
    tie_rep["per_cam"] = {}
    for i, c in enumerate(cams):
        m = ts["cam"][good] == i
        if m.sum() > 20:
            tie_rep["per_cam"][c] = {"n": int(m.sum()), "ray_err_median_m": float(np.median(ts["ray_err"][good][m])),
                                     "abs_plane_median_m": float(np.median(np.abs(ts["plane_d"][good][m]))),
                                     "frac_depth_lt8m": float(np.mean(ts["depth"][good][m] < 8))}
    print("LiDAR check of landmarks: n %d, |plane dist| median %.3f m; by depth: %s" % (
        tie_rep["n"], tie_rep["plane_abs_median_m"],
        ", ".join(f"{k} ray {v['ray_err_median_m']:+.3f} m ({100 * v['rel_depth_err_median']:+.2f}%, n {v['n']})"
                  for k, v in tie_rep["by_depth"].items())), flush=True)
    # ---- report
    r, Xc = project_all(cams_, obs, X)
    e = r.norm(dim=1)
    sig0 = float(1.4826 * e.median() / np.sqrt(np.log(4)))    # 2D Rayleigh median -> sigma
    cov = info["cov_free"]
    fidx = info["fidx"]
    sd = np.full((C, NP), np.nan)
    if cov is not None:
        sdf = torch.sqrt(torch.diagonal(cov)).cpu().numpy() * sig0
        sd.reshape(-1)[fidx.cpu().numpy()] = sdf
    T_L_C = cams_.T_L_C()
    intr = cams_.intr_array()
    res = {"segs": list(segs), "cams": cams, "free": a.free, "ties": a.ties, "cross": str(a.cross),
           "init": a.init, "traj": a.traj, "n_obs": len(obs), "n_lm": M,
           "n_ties": 0 if ties is None else len(ties),
           "reproj_median_px": float(e.median()), "reproj_rms_px": float(e[e < 5].pow(2).mean().sqrt()),
           "sigma_px": sig0, "wall_s": time.time() - t_start, "dt_ms": a.dt_ms, "cameras": {},
           "lidar_check": tie_rep, "held": calib_in is not None}
    depth_obs = Xc.norm(dim=1).cpu().numpy()
    per_cam = torch.bincount(obs.cam, minlength=C)
    for i, c in enumerate(cams):
        ec = e[obs.cam == i]
        axis = T_L_C[i][:3, 2]
        sd_axis = np.nan
        if cov is not None and bool(free[i, 3:6].all()):
            pos_idx = [int((fidx == i * NP + k).nonzero()[0]) for k in (3, 4, 5)]
            Sig = cov[pos_idx][:, pos_idx].cpu().numpy() * sig0 ** 2
            sd_axis = float(np.sqrt(axis @ Sig @ axis))
        dc_ = depth_obs[(obs.cam == i).cpu().numpy()]
        res["cameras"][c] = {"sd_along_axis_m": sd_axis,
            "depth_hist": {"lt4": float(np.mean(dc_ < 4)), "lt8": float(np.mean(dc_ < 8)),
                           "8-15": float(np.mean((dc_ >= 8) & (dc_ < 15))), "15-30": float(np.mean((dc_ >= 15) & (dc_ < 30))),
                           "ge30": float(np.mean(dc_ >= 30)), "median_m": float(np.median(dc_)) if len(dc_) else None},
            "n_frames": int(len(np.unique(tns[(obs.cam == i).cpu().numpy()]))),
            "T_lidar_cam": T_L_C[i].tolist(), "T_cam_lidar": np.linalg.inv(T_L_C[i]).tolist(),
            "intr_kb": intr[i].tolist(), "sd": dict(zip(PNAMES, sd[i].tolist())),
            "n_obs": int(per_cam[i]), "reproj_median_px": float(ec.median()) if len(ec) else None}
    tmp = out / "result.json.tmp"
    np.savez(out / "state.npz", X=X.cpu().numpy(), lm_seg=lm_seg, obs_cam=obs.cam.cpu().numpy(),
             obs_lm=obs.lm.cpu().numpy(), obs_uv=obs.uv.cpu().numpy(), obs_t=tns,
             tie_lm=np.zeros(0) if ties is None else ties.lm.cpu().numpy(),
             lm_key=np.array([str(k) for k in lm_key]))
    tmp.write_text(json.dumps(res, indent=1))
    os.replace(tmp, out / "result.json")
    print(f"done in {time.time() - t_start:.0f} s: reproj median {e.median():.3f} px")
    for i, c in enumerate(cams):
        Tl = T_L_C[i]
        print(f"  {c:18s} pos_L [{Tl[0,3]:+.3f} {Tl[1,3]:+.3f} {Tl[2,3]:+.3f}] "
              f"axis+-{1e3*res['cameras'][c]['sd_along_axis_m']:.1f} mm (formal)  "
              f"f {intr[i,0]:.1f}  cx {intr[i,2]:.1f} cy {intr[i,3]:.1f} "
              f"k {intr[i,4]:+.4f} {intr[i,5]:+.4f} {intr[i,6]:+.4f}")
    _viz_result(ws, segs, out.name, cams_, obs, e, info["cost"], a.huber,
                held=calib_in is not None, dt_s=a.dt_ns * 1e-9)
    return res


def result_calib(res: dict) -> dict:
    """result.json -> {cam: (T_cam_lidar, intr8)} (lidar_odo eval_edges.load_calib for a result file)."""
    return {c: (np.array(v["T_cam_lidar"]), np.array(v["intr_kb"])) for c, v in res["cameras"].items()}


def cams_from_result(res: dict, cams) -> Cameras:
    return calib_cams(list(cams), result_calib(res))
