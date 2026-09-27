"""Cross-camera (thermal_left <-> thermal_right) observations for the joint BA.

Verbatim numerics of thermal_lo/code/link_stereo.py; the command line became
`link_stereo(ws, plan, segs, calib, out)`.

For every KLT track of one camera (>= 12 frames), its landmark is triangulated with the given
calibration and the fixed LO trajectory. At every `every`-th observation, the other camera's frame
nearest in time (|dt| < 20 ms) is taken, the landmark is projected into it (prediction), and
Lucas-Kanade (same contrast-normalised images as the tracker, pyramid, initial flow = prediction)
finds the point; a backward LK (other -> own image) must return within 0.4 px, and the result
must lie within `gate` px of the prediction (rejects wrong matches only; the position itself
comes from the images, so the calibration used for the prediction does not bias it).
Both directions (left tracks into right images, right tracks into left images).

Output: seg, src_cam, track (the source track id), dst_cam, dst_header_ns, dst_uv (N,2),
pred_uv (N,2), src_header_ns, src_uv.
"""
from __future__ import annotations

import os
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import torch

from . import tba
from .timemodel import time_model
from .tracks import prep_image

CAMS = ("thermal_left", "thermal_right")
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.005),
          flags=cv2.OPTFLOW_USE_INITIAL_FLOW)


def link_stereo(ws, plan, segs, res, out, every=6, gate=6.0, time_kind="smooth", log=print):
    cv2.setNumThreads(2)
    torch.set_num_threads(2)
    a = SimpleNamespace(segs=list(segs), out=Path(out), every=every, gate=gate, time=time_kind)
    ZERO_NS = plan.t_ref
    LO_DIR = str(ws.root / "lo")
    cams_n = list(CAMS)
    segs_cal = res["segs"]
    T = np.array([res["cameras"][c]["T_lidar_cam"] for c in cams_n])
    I = np.array([res["cameras"][c]["intr"] for c in cams_n])
    rs = np.array([res["cameras"][c]["rs_s"] for c in cams_n])
    out = {k: [] for k in ("seg", "src_cam", "track", "dst_cam", "dst_header_ns", "dst_uv", "pred_uv",
                           "src_header_ns", "src_uv")}
    for seg in a.segs:
        bag = plan.bag(seg)
        tm = {c: time_model(ws, bag, c, plan.wins_of_bag(bag), a.time) for c in cams_n}
        t0 = time.time()
        trajs = tba.Trajs([seg], LO_DIR, ZERO_NS)
        # per-camera dt of this segment if the calibration has it, else the camera mean
        dts = []
        for c in cams_n:
            d = res["cameras"][c]["dt_s"]
            dts.append(d[segs_cal.index(seg)] if (isinstance(d, list) and len(d) == len(segs_cal) and seg in segs_cal)
                       else res["cameras"][c]["dt_s_mean"])
        cams = tba.Cams(cams_n, T, I, rs, np.array(dts)[:, None], 1)
        span = trajs.span[0]
        data = {}
        for ci, c in enumerate(cams_n):
            z = np.load(ws.thermal_tracks(seg, c))
            hdr = z["header_ns"]
            h_all, tf_all, _ = tm[c]
            tf = (tf_all[np.searchsorted(h_all, hdr)] - ZERO_NS) * 1e-9
            data[c] = dict(hdr=hdr, tf=tf, of=z["obs_frame"], ot=z["obs_track"], xy=z["obs_xy"].astype(np.float64))
        imgs = {}
        from .frames import ThermalFrames
        frames = {c: ThermalFrames(ws.thermal16(seg), c).load_all() for c in cams_n}

        def img(c, h):
            k = (c, int(h))
            if k not in imgs:
                imgs[k] = prep_image(frames[c].read(h))
            return imgs[k]

        nlink = 0
        for si_, src in enumerate(cams_n):
            di_ = 1 - si_
            dst = cams_n[di_]
            D, E = data[src], data[dst]
            of, ot, xy = D["of"], D["ot"], D["xy"]
            ok = (D["tf"][of] > span[0] + 0.15) & (D["tf"][of] < span[1] - 0.15)
            of, ot, xy = of[ok], ot[ok], xy[ok]
            uid, inv, cnt = np.unique(ot, return_inverse=True, return_counts=True)
            keep = cnt[inv] >= 12
            of, ot, xy, inv = of[keep], ot[keep], xy[keep], inv[keep]
            uid2, lm = np.unique(ot, return_inverse=True)
            M = len(uid2)
            o = tba.Obs(np.full(len(of), si_), np.zeros(len(of)), lm, xy, D["tf"][of])
            X, par = tba.triangulate(cams, trajs, o, M)
            good_lm = (par > np.radians(1.5)).numpy()
            # link frames: every `every`-th observation of each track
            order = np.lexsort((of, lm))
            rank = np.zeros(len(of), int)
            _, st_, cn_ = np.unique(lm[order], return_index=True, return_counts=True)
            rank[order] = np.arange(len(order)) - np.repeat(st_, cn_)
            sel = np.flatnonzero((rank % a.every == 0) & good_lm[lm])
            # nearest destination frame in time
            j = np.clip(np.searchsorted(E["tf"], D["tf"][of[sel]]), 1, len(E["tf"]) - 1)
            j = np.where(np.abs(E["tf"][j - 1] - D["tf"][of[sel]]) < np.abs(E["tf"][j] - D["tf"][of[sel]]), j - 1, j)
            dtt = np.abs(E["tf"][j] - D["tf"][of[sel]])
            m = dtt < 0.020
            sel, j = sel[m], j[m]
            # predicted position in the destination frame (two passes for the row time)
            od = tba.Obs(np.full(len(sel), di_), np.zeros(len(sel)), lm[sel], np.zeros((len(sel), 2)) + 240.0,
                         E["tf"][j])
            fx, fy, cx, cy, k1, k2 = [q[di_] for q in cams.intr()]
            for _ in range(2):
                Xc, _, _ = tba.cam_points(cams, trajs, od, X)
                pred = tba.project(Xc, fx, fy, cx, cy, k1, k2, jac=False)
                od.uv = pred.clone()
            pred = pred.numpy()
            vis = (Xc[:, 2].numpy() > 1.0) & (pred[:, 0] > 12) & (pred[:, 0] < 628) & (pred[:, 1] > 12) & (pred[:, 1] < 468)
            sel, j, pred = sel[vis], j[vis], pred[vis]
            # LK per (source frame, destination frame) pair
            pair = of[sel] * 100000 + j
            for pk in np.unique(pair):
                mm = np.flatnonzero(pair == pk)
                fs, fd = divmod(int(pk), 100000)
                A = img(src, D["hdr"][fs]); B = img(dst, E["hdr"][fd])
                p0 = xy[sel[mm]].astype(np.float32).reshape(-1, 1, 2)
                p1 = pred[mm].astype(np.float32).reshape(-1, 1, 2).copy()
                p1, s1, _ = cv2.calcOpticalFlowPyrLK(A, B, p0, p1, **LK)
                pb = p0.copy()
                pb, s2, _ = cv2.calcOpticalFlowPyrLK(B, A, p1, pb, **LK)
                fb = np.linalg.norm((pb - p0).reshape(-1, 2), axis=1)
                d = np.linalg.norm(p1.reshape(-1, 2) - pred[mm], axis=1)
                okk = (s1.ravel() == 1) & (s2.ravel() == 1) & (fb < 0.4) & (d < a.gate)
                if not okk.any():
                    continue
                q = mm[okk]
                out["seg"].append(np.full(len(q), seg)); out["src_cam"].append(np.full(len(q), src))
                out["track"].append(ot[sel[q]]); out["dst_cam"].append(np.full(len(q), dst))
                out["dst_header_ns"].append(np.full(len(q), E["hdr"][fd], np.int64))
                out["dst_uv"].append(p1.reshape(-1, 2)[okk]); out["pred_uv"].append(pred[q])
                out["src_header_ns"].append(np.full(len(q), D["hdr"][fs], np.int64)); out["src_uv"].append(xy[sel[q]])
                nlink += len(q)
        imgs.clear()          # per window (was per direction: every frame used both ways was prepared twice)
        frames.clear()
        dd = np.concatenate(out["dst_uv"][-50:]) - np.concatenate(out["pred_uv"][-50:]) if out["dst_uv"] else np.zeros((0, 2))
        log(f"{seg}: {nlink} links, |LK - prediction| median {np.median(np.linalg.norm(dd, axis=1)) if len(dd) else np.nan:.2f} px "
              f"({time.time() - t0:.0f} s)")
    tmp = a.out.with_name(a.out.stem + ".tmp.npz")
    n = int(sum(len(v) for v in out["seg"]))
    np.savez(tmp, n=n, **{k: np.concatenate(v) for k, v in out.items() if n})
    os.replace(tmp, a.out)
    return {"links": n}


def merge_links(parts, out):
    """Concatenate per-window link files (link_stereo on one window each, run in parallel) in window
    order: the same arrays as one link_stereo over all windows."""
    zs = [np.load(p) for p in parts]
    zs = [z for z in zs if int(z["n"])]
    keys = ("seg", "src_cam", "track", "dst_cam", "dst_header_ns", "dst_uv", "pred_uv", "src_header_ns", "src_uv")
    out = Path(out)
    tmp = out.with_name(out.stem + ".tmp.npz")
    np.savez(tmp, **{k: np.concatenate([z[k] for z in zs]) for k in keys})
    os.replace(tmp, out)
    return {"links": int(sum(len(z["seg"]) for z in zs))}
