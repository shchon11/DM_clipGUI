"""KLT tracks on the A70 thermal stream (16-bit raw PNG from extract_thermal16.py).

Verbatim numerics of thermal_lo/code/tracks_thermal.py; the command line became
`run_thermal_tracks(ws, plan, win, cam)`.

Per frame the raw centi-kelvin image is turned into a contrast-normalised 8-bit image that
is stable from frame to frame (the brightness-constancy assumption of Lucas-Kanade):
  * 'lcn'  (default): Gaussian pre-smoothing (sigma 1 px, the A70's temporal noise is
           ~6 cK per pixel), then local contrast normalisation
               n = (I - G_s * I) / (sqrt(G_s * (I - G_s*I)^2) + eps),  s = 10 px, eps = 25 cK
           mapped to 8 bit.  Independent of the scene's temperature span and of slow
           gain/offset drift, and it does not amplify the noise in flat regions (eps).
  * 'stretch': 1-99.5 % per-frame stretch (what online/extract_win.py writes).
  * 'clahe': stretch + CLAHE(3, 8x8).
Shi-Tomasi corners refilled on a 12x9 grid, pyramidal LK (21 px, 4 levels), forward-backward
check < 0.4 px. Frames flagged by the duplicate/FFC rule are skipped; a track survives one
skipped frame (the LK step is then 67 ms) but not a freeze (flat-field correction).
Tracks that stay put while the car travels 2 m (fixed-pattern noise, own car) are dropped.

Output TRACKS/tracks_<SEG>_<cam>.npz:
    header_ns (F,)  frame header stamps (the file names)
    obs_frame, obs_track (N,), obs_xy (N,2) float32
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import cv2
import numpy as np

from ..lo.lotraj import LOTraj
from .timemodel import frame_index
LK = dict(winSize=(21, 21), maxLevel=4,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.005))
CLAHE = cv2.createCLAHE(3.0, (8, 8))


def prep_image(raw: np.ndarray, kind: str = "lcn", s: float = 10.0, eps: float = 25.0, gain: float = 45.0):
    I = raw.astype(np.float32)
    if kind == "lcn":
        I = cv2.GaussianBlur(I, (0, 0), 1.0)
        mu = cv2.GaussianBlur(I, (0, 0), s)
        d = I - mu
        sd = np.sqrt(cv2.GaussianBlur(d * d, (0, 0), s))
        return np.clip(d / (sd + eps) * gain + 128, 0, 255).astype(np.uint8)
    lo, hi = np.percentile(I, [1.0, 99.5])
    a = np.clip((I - lo) / max(hi - lo, 1.0) * 255, 0, 255).astype(np.uint8)
    return CLAHE.apply(a) if kind == "clahe" else a


def track(imgs, skip_gap, grid=(12, 9), per_cell=10, fb_max=0.4, min_len=5, qual=0.01, min_dist=10, device="cpu"):
    """imgs: list of 8-bit images; skip_gap[i] True if frame i follows a freeze (restart all).
    device: cpu (validated, bit-identical) | cuda | opencl | auto (rgb/tracks.lk_device); the refill mask is drawn
    only around the cells that are refilled (identical)."""
    from ..rgb.tracks import LKRunner, occupancy
    lk = LKRunner(device, LK)
    h, w = imgs[0].shape
    gx, gy = grid
    cw, ch = w / gx, h / gy
    obs_f, obs_t, obs_xy = [], [], []
    pts = np.zeros((0, 2), np.float32)
    ids = np.zeros(0, np.int64)
    nid = 0
    prev = None
    for fi, img in enumerate(imgs):
        cur = lk.frame(img)
        if skip_gap[fi]:
            pts = np.zeros((0, 2), np.float32); ids = np.zeros(0, np.int64); prev = None
        if prev is not None and len(pts):
            p1, st, p0, st2 = lk.pair(prev, cur, pts)
            fb = np.linalg.norm(p0.reshape(-1, 2) - pts, axis=1)
            p1 = p1.reshape(-1, 2)
            ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < fb_max)
            ok &= (p1[:, 0] >= 3) & (p1[:, 0] < w - 3) & (p1[:, 1] >= 3) & (p1[:, 1] < h - 3)
            pts, ids = p1[ok], ids[ok]
        cnt = np.zeros((gy, gx), int)
        if len(pts):
            np.add.at(cnt, (np.clip((pts[:, 1] / ch).astype(int), 0, gy - 1),
                            np.clip((pts[:, 0] / cw).astype(int), 0, gx - 1)), 1)
        cells = [(j, i) for j in range(gy) for i in range(gx) if per_cell - cnt[j, i] > per_cell // 2]
        boxes = [(int(i * cw), int((i + 1) * cw), int(j * ch), int((j + 1) * ch)) for j, i in cells]
        occ = occupancy(pts, h, w, boxes, min_dist, fill=0, base=255)
        new = []
        for (j, i), (x0, x1, y0, y1) in zip(cells, boxes):
            need = per_cell - cnt[j, i]
            c = cv2.goodFeaturesToTrack(img[y0:y1, x0:x1], need, qual, min_dist, mask=occ[y0:y1, x0:x1])
            if c is not None:
                new.append(c.reshape(-1, 2) + [x0, y0])
        if new:
            new = np.concatenate(new).astype(np.float32)
            new = cv2.cornerSubPix(img, new.reshape(-1, 1, 2), (5, 5), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01)).reshape(-1, 2)
            pts = np.concatenate([pts, new])
            ids = np.concatenate([ids, np.arange(nid, nid + len(new))])
            nid += len(new)
        obs_f.append(np.full(len(pts), fi)); obs_t.append(ids.copy()); obs_xy.append(pts.copy())
        prev = cur
    obs_f, obs_t, obs_xy = np.concatenate(obs_f), np.concatenate(obs_t), np.concatenate(obs_xy)
    n = np.bincount(obs_t)
    keep = n[obs_t] >= min_len
    return obs_f[keep], obs_t[keep], obs_xy[keep]


def drop_static(t_ns, obs_f, obs_t, obs_xy, ref_npz, min_px=3.0, min_travel=2.0):
    tr = LOTraj.load(ref_npz)
    a, b = tr.span()
    pos = tr.p(np.clip(t_ns, a, b))
    o = np.lexsort((obs_f, obs_t))
    f, t, xy = obs_f[o], obs_t[o], obs_xy[o]
    first = np.r_[True, t[1:] != t[:-1]]
    last = np.r_[t[1:] != t[:-1], True]
    disp = np.linalg.norm(xy[last] - xy[first], axis=1)
    trav = np.linalg.norm(pos[f[last]] - pos[f[first]], axis=1)
    bad = t[first][(disp < min_px) & (trav > min_travel)]
    keep = ~np.isin(obs_t, bad)
    return obs_f[keep], obs_t[keep], obs_xy[keep], len(bad)


def frames_of(ws, plan, seg, cam, margin_s=0.3):
    # this window's own extraction (window +- 0.5 s): with abutting windows the +- 0.3 s margin frames
    # also exist in the neighbour's extraction, but the images read below are this window's
    ix = frame_index(ws, plan.bag(seg), cam, [seg])
    h = ix["header_ns"]
    a, b = plan.span_ns(seg)
    m = (h >= a - int(margin_s * 1e9)) & (h <= b + int(margin_s * 1e9))
    idx = np.flatnonzero(m)
    return h[idx], ix["ffc"][idx]


def run_thermal_tracks(ws, plan, seg, cam, prep="lcn", log=print, **kw):
    cv2.setNumThreads(1)
    t0 = time.time()
    h, bad = frames_of(ws, plan, seg, cam)
    keep = ~bad
    # a gap of more than 3 frames (freeze / flat-field correction) restarts the tracker
    hk = h[keep]
    gap = np.r_[True, np.diff(hk) > 110_000_000]
    # read the window's frames in one pass, decode + prepare in threads (PNG decoding and the blurs
    # release the GIL)
    from concurrent.futures import ThreadPoolExecutor
    from .frames import ThermalFrames
    fr = ThermalFrames(ws.thermal16(seg), cam).load_all()
    with ThreadPoolExecutor(4) as ex:
        imgs = list(ex.map(lambda x: prep_image(fr.read(x), prep), hk))
    f, t, xy = track(imgs, gap, **kw)
    f, t, xy, nst = drop_static(hk.astype(np.float64), f, t, xy, ws.lo("ref", seg))
    out = ws.thermal_tracks(seg, cam)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.stem + ".tmp.npz")
    np.savez(tmp, header_ns=hk, obs_frame=f.astype(np.int32),
             obs_track=t.astype(np.int64), obs_xy=xy.astype(np.float32), prep=prep)
    os.replace(tmp, out)
    ntr = len(np.unique(t))
    L = np.bincount(t)[np.unique(t)]
    log(f"{seg} {cam} [{prep}]: {len(hk)} frames ({int(bad.sum())} dropped), {ntr} tracks, {len(t)} obs, "
        f"len median {np.median(L):.0f}, >=12: {(L >= 12).sum()}, >=30: {(L >= 30).sum()}, static dropped {nst}, "
        f"{time.time() - t0:.0f} s")
    return {"frames": int(len(hk)), "tracks": int(ntr), "obs": int(len(t))}
