"""KLT tracks with anchored (drift-free) refinement: rgb.tracks.anchor.

The validated tracker (tracks.py) chains pyramidal LK frame to frame. Each step has a small bias (the patch
changes: scale, perspective, night noise), and the biases add up along a track: forward-backward closure
over whole tracks 0.3 px after 1 s, 0.4-0.5 px after 2-4 s on the front cameras, up to 1 px on the side and
several px on the rear cameras (tests/time_offset_eval_20260927.txt section 4b). The BA then sees a residual
that grows with the time from the track centre (0.68 -> 1.13 px).

Here every point keeps a TEMPLATE: the image patch (41 x 41) around the position where it was last anchored,
and the sub-pixel position of the point in it. In each frame
  1. the frame-to-frame pyramidal LK (+ forward-backward check, exactly as tracks.py) predicts p1;
  2. the point is re-measured against its template: LK from the template to a patch of the current image
     around p1, started at p1 (single level: the prediction is within ~1 px), then back from the current
     patch to the template (closure gate anchor_fb_max, and |re-measured - p1| < max_jump);
     the re-measured position replaces p1, so the error no longer accumulates frame by frame;
  3. when that fails, or the template is older than max_age frames, the point is re-anchored: the new
     template is cut from the previous frame at the point's previous (anchored) position and the point is
     measured again; if that fails too the frame-to-frame p1 is kept and the template is cut there.
The drift thus grows with the number of re-anchorings (about one per max_age frames) instead of with every
frame. All templates / current patches of a frame are tiled into one mosaic image each, so a frame costs
three LK calls on ~1.7 Mpx mosaics (the OpenCV Python binding cannot reuse per-frame pyramids).
Detection, refill, masks, the static-track test and the output format are those of tracks.py; in addition
obs_q holds the corner strength of every observation (min eigenvalue of the patch gradient matrix / window
area, OpenCV's LK normalisation; NaN for the detection frame).
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from .tracks import LK

ADEFAULTS = dict(enabled=False, max_age=30, anchor_fb_max=0.3, max_jump=2.0)
HALF = 20                     # template / patch half size (41 x 41); LK window 21 x 21
PS = 2 * HALF + 1
NCOL = 32                     # tiles per mosaic row
LK0 = dict(winSize=(21, 21), maxLevel=0, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.005))


def _crop(img_p: np.ndarray, centres: np.ndarray) -> np.ndarray:
    """Patches (n, PS, PS) of the border-padded image around integer centres (n, 2) (x, y) of the unpadded image."""
    ar = np.arange(PS)
    centres = np.stack([np.clip(centres[:, 0], 0, img_p.shape[1] - PS), np.clip(centres[:, 1], 0, img_p.shape[0] - PS)], 1)
    ys = centres[:, 1, None] + ar[None]            # padded by HALF: row of pixel (y - HALF + i) is y + i
    xs = centres[:, 0, None] + ar[None]
    return img_p[ys[:, :, None], xs[:, None, :]]


def _mosaic(patches: np.ndarray) -> np.ndarray:
    n = len(patches)
    rows = max(1, -(-n // NCOL))
    m = np.zeros((rows * NCOL, PS, PS), np.uint8)
    m[:n] = patches
    return m.reshape(rows, NCOL, PS, PS).transpose(0, 2, 1, 3).reshape(rows * PS, NCOL * PS)


def _tile_origin(n: int) -> np.ndarray:
    i = np.arange(n)
    return np.stack([(i % NCOL) * PS, (i // NCOL) * PS], 1).astype(np.float32)


class Templates:
    """Per-point templates: patches (n, PS, PS), sub-pixel point position inside (n, 2), anchor frame (n,)."""

    def __init__(self):
        self.P = np.zeros((0, PS, PS), np.uint8)
        self.q = np.zeros((0, 2), np.float32)
        self.f = np.zeros(0, np.int64)

    @staticmethod
    def cut(img_p, pts):
        c = np.round(pts).astype(np.int64)
        c = np.stack([np.clip(c[:, 0], 0, img_p.shape[1] - PS), np.clip(c[:, 1], 0, img_p.shape[0] - PS)], 1)
        return _crop(img_p, c), (pts - c + HALF).astype(np.float32)

    def set(self, k, img_p, pts, fi):
        self.P[k], self.q[k] = self.cut(img_p, pts)
        self.f[k] = fi

    def append(self, img_p, pts, fi):
        P, q = self.cut(img_p, pts)
        self.P = np.concatenate([self.P, P])
        self.q = np.concatenate([self.q, q])
        self.f = np.concatenate([self.f, np.full(len(pts), fi)])

    def keep(self, m):
        self.P, self.q, self.f = self.P[m], self.q[m], self.f[m]


def _measure(T: Templates, k: np.ndarray, img_p: np.ndarray, guess: np.ndarray, fb_max: float, max_jump: float):
    """Re-measure points k against their templates, starting at guess (n, 2) in the current image.
    Returns (pos (n, 2), ok (n,), minEig (n,))."""
    n = len(k)
    if not n:
        return np.zeros((0, 2), np.float32), np.zeros(0, bool), np.zeros(0, np.float32)
    org = _tile_origin(n)
    c = np.round(guess).astype(np.int64)
    c = np.stack([np.clip(c[:, 0], 0, img_p.shape[1] - PS), np.clip(c[:, 1], 0, img_p.shape[0] - PS)], 1)
    cur = _mosaic(_crop(img_p, c))
    tpl = _mosaic(T.P[k])
    p0 = (org + T.q[k]).reshape(-1, 1, 2)
    g = (org + (guess - c + HALF)).astype(np.float32).reshape(-1, 1, 2)
    p, st, _ = cv2.calcOpticalFlowPyrLK(tpl, cur, p0, g.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **LK0)
    b, st2, ev = cv2.calcOpticalFlowPyrLK(cur, tpl, p, p0.copy(),
                                          flags=cv2.OPTFLOW_USE_INITIAL_FLOW | cv2.OPTFLOW_LK_GET_MIN_EIGENVALS, **LK0)
    loc = p.reshape(-1, 2) - org                            # position inside the current patch
    pos = loc + c - HALF
    ok = (st.ravel() == 1) & (st2.ravel() == 1)
    ok &= np.linalg.norm(b.reshape(-1, 2) - p0.reshape(-1, 2), axis=1) < fb_max
    ok &= np.linalg.norm(pos - guess, axis=1) < max_jump
    ok &= (np.abs(loc - HALF) < HALF - 11).all(1)           # the LK window stayed inside the tile
    return pos.astype(np.float32), ok, ev.ravel().astype(np.float32)


def track_anchor(files: list[Path], mask: np.ndarray, grid=(16, 9), per_cell: int = 8, fb_max: float = 0.5,
                 min_len: int = 5, max_age: int = 30, anchor_fb_max: float = 0.3, max_jump: float = 2.0,
                 stats: dict | None = None, **_):
    h, w = mask.shape
    gx, gy = grid
    cell_w, cell_h = w / gx, h / gy
    frame_ns = np.array([int(f.stem) for f in files], dtype=np.int64)
    obs_f, obs_t, obs_xy, obs_q = [], [], [], []
    pts = np.zeros((0, 2), np.float32)
    ids = np.zeros(0, np.int64)
    qual = np.zeros(0, np.float32)
    T = Templates()
    next_id = 0
    prev = prev_p = None
    n_ok = n_re = n_fail = 0
    for fi, f in enumerate(files):
        img = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        img_p = cv2.copyMakeBorder(img, HALF, HALF, HALF, HALF, cv2.BORDER_REPLICATE)
        if prev is not None and len(pts):
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, img, pts.reshape(-1, 1, 2), None, **LK)
            p0, st2, _ = cv2.calcOpticalFlowPyrLK(img, prev, p1, None, **LK)
            fb = np.linalg.norm(p0.reshape(-1, 2) - pts, axis=1)
            p1 = p1.reshape(-1, 2)
            ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < fb_max)
            ok &= (p1[:, 0] >= 2) & (p1[:, 0] < w - 2) & (p1[:, 1] >= 2) & (p1[:, 1] < h - 2)
            newp = p1.copy()
            q = np.full(len(pts), np.nan, np.float32)
            # templates older than max_age: re-anchor at the previous (anchored) position first
            old = ok & (fi - T.f > max_age)
            if old.any():
                T.set(np.flatnonzero(old), prev_p, pts[old], fi - 1)
            k = np.flatnonzero(ok)
            pos, g, ev = _measure(T, k, img_p, p1[k], anchor_fb_max, max_jump)
            newp[k[g]], q[k[g]] = pos[g], ev[g]
            n_ok += int(g.sum())
            # failed: re-anchor at the previous position (unless the template is from the previous frame already)
            kk = k[~g]
            kk = kk[T.f[kk] < fi - 1]
            if len(kk):
                T.set(kk, prev_p, pts[kk], fi - 1)
                pos, g2, ev = _measure(T, kk, img_p, p1[kk], anchor_fb_max, max_jump)
                newp[kk[g2]], q[kk[g2]] = pos[g2], ev[g2]
                n_re += int(g2.sum())
                g_all = np.zeros(len(pts), bool)
                g_all[k[g]] = True
                g_all[kk[g2]] = True
            else:
                g_all = np.zeros(len(pts), bool)
                g_all[k[g]] = True
            # still failed: keep the frame-to-frame position, the template restarts there
            bad = np.flatnonzero(ok & ~g_all)
            n_fail += len(bad)
            p1 = newp
            inside = (p1[:, 0] >= 2) & (p1[:, 0] < w - 2) & (p1[:, 1] >= 2) & (p1[:, 1] < h - 2)
            ok &= inside
            ok[inside] &= mask[p1[inside, 1].astype(int), p1[inside, 0].astype(int)] > 0
            if len(bad):
                T.set(bad, img_p, p1[bad], fi)
            pts, ids, qual = p1[ok], ids[ok], q[ok]
            T.keep(ok)
        # refill empty grid cells (tracks.py)
        count = np.zeros((gy, gx), int)
        if len(pts):
            cx = np.clip((pts[:, 0] / cell_w).astype(int), 0, gx - 1)
            cy = np.clip((pts[:, 1] / cell_h).astype(int), 0, gy - 1)
            np.add.at(count, (cy, cx), 1)
        occ = np.zeros((h, w), np.uint8)
        for x, y in pts:
            cv2.circle(occ, (int(x), int(y)), 12, 255, -1)
        new = []
        for j in range(gy):
            for i in range(gx):
                need = per_cell - count[j, i]
                if need <= per_cell // 2:
                    continue
                y0, y1 = int(j * cell_h), int((j + 1) * cell_h)
                x0, x1 = int(i * cell_w), int((i + 1) * cell_w)
                m = ((mask[y0:y1, x0:x1] > 0) & (occ[y0:y1, x0:x1] == 0)).astype(np.uint8) * 255
                c = cv2.goodFeaturesToTrack(img[y0:y1, x0:x1], need, 0.005, 12, mask=m)
                if c is not None:
                    new.append(c.reshape(-1, 2) + [x0, y0])
        if new:
            new = np.concatenate(new).astype(np.float32)
            new = cv2.cornerSubPix(img, new.reshape(-1, 1, 2), (5, 5), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01)).reshape(-1, 2)
            pts = np.concatenate([pts, new])
            ids = np.concatenate([ids, np.arange(next_id, next_id + len(new))])
            qual = np.concatenate([qual, np.full(len(new), np.nan, np.float32)])
            T.append(img_p, new, fi)
            next_id += len(new)
        obs_f.append(np.full(len(pts), fi))
        obs_t.append(ids.copy())
        obs_xy.append(pts.copy())
        obs_q.append(qual.copy())
        prev, prev_p = img, img_p
    obs_f, obs_t, obs_xy, obs_q = map(np.concatenate, (obs_f, obs_t, obs_xy, obs_q))
    n = np.bincount(obs_t)
    keep = n[obs_t] >= min_len
    if stats is not None:
        tot = max(n_ok + n_re + n_fail, 1)
        stats.update(anchored=round(n_ok / tot, 4), reanchored=round(n_re / tot, 4), f2f_only=round(n_fail / tot, 4))
    return frame_ns, obs_f[keep], obs_t[keep], obs_xy[keep], obs_q[keep]
