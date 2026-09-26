"""KLT feature tracks for one camera over a clip.

Verbatim numerics of online_calib/tracks.py (track, drop_static); the command line became
`run_tracks(seg_dir, channel, mask, ins, out)`.

Shi-Tomasi corners, refilled on a grid so every part of the image keeps
features (the edges matter most for distortion); pyramidal Lucas-Kanade with a
forward-backward check. Tracks that do not move while the car drives are the
car's own body or glass reflections and are dropped. Output DIR/tracks_<clip>_<CHANNEL>.npz:
    frame_ns (F,)        image timestamps
    obs_frame, obs_track (N,), obs_xy (N, 2)
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import cv2
import numpy as np

from ..ins import Trajectory

LK = dict(winSize=(21, 21), maxLevel=4,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


def track(files: list[Path], mask: np.ndarray, grid=(16, 9), per_cell: int = 8,
          fb_max: float = 0.5, min_len: int = 5):
    h, w = mask.shape
    gx, gy = grid
    cell_w, cell_h = w / gx, h / gy
    frame_ns = np.array([int(f.stem) for f in files], dtype=np.int64)
    obs_f, obs_t, obs_xy = [], [], []
    pts = np.zeros((0, 2), np.float32)
    ids = np.zeros(0, np.int64)
    next_id = 0
    prev = None
    for fi, f in enumerate(files):
        img = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        if prev is not None and len(pts):
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, img, pts.reshape(-1, 1, 2), None, **LK)
            p0, st2, _ = cv2.calcOpticalFlowPyrLK(img, prev, p1, None, **LK)
            fb = np.linalg.norm(p0.reshape(-1, 2) - pts, axis=1)
            p1 = p1.reshape(-1, 2)
            ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < fb_max)
            inside = (p1[:, 0] >= 2) & (p1[:, 0] < w - 2) & (p1[:, 1] >= 2) & (p1[:, 1] < h - 2)
            ok &= inside
            ok[inside] &= mask[p1[inside, 1].astype(int), p1[inside, 0].astype(int)] > 0
            pts, ids = p1[ok], ids[ok]
        # refill empty grid cells
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
            next_id += len(new)
        obs_f.append(np.full(len(pts), fi))
        obs_t.append(ids.copy())
        obs_xy.append(pts.copy())
        prev = img
    obs_f, obs_t, obs_xy = np.concatenate(obs_f), np.concatenate(obs_t), np.concatenate(obs_xy)
    # keep tracks of at least min_len frames
    n = np.bincount(obs_t)
    keep = n[obs_t] >= min_len
    return frame_ns, obs_f[keep], obs_t[keep], obs_xy[keep]


def drop_static(frame_ns, obs_f, obs_t, obs_xy, traj: Trajectory, min_px: float = 3.0,
                min_travel: float = 2.0):
    """Remove tracks that stay put while the car travels min_travel metres."""
    pos = traj.p(frame_ns)
    order = np.lexsort((obs_f, obs_t))
    f, t, xy = obs_f[order], obs_t[order], obs_xy[order]
    first = np.r_[True, t[1:] != t[:-1]]
    last = np.r_[t[1:] != t[:-1], True]
    tid = t[first]
    disp = np.linalg.norm(xy[last] - xy[first], axis=1)
    travel = np.linalg.norm(pos[f[last]] - pos[f[first]], axis=1)
    bad = tid[(disp < min_px) & (travel > min_travel)]
    keep = ~np.isin(obs_t, bad)
    return obs_f[keep], obs_t[keep], obs_xy[keep], len(bad)


def run_tracks(clip: Path, channel: str, mask_path: Path, ins: Path, out: Path, grid=(16, 9), per_cell: int = 8,
               fb_max: float = 0.5, min_len: int = 5, log=print) -> dict:
    t0 = time.time()
    clip = Path(clip)
    files = sorted((clip / "cam" / channel).glob("*.jpg"))
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(f"mask {mask_path}")
    frame_ns, f, t, xy = track(files, mask, grid=tuple(grid), per_cell=per_cell, fb_max=fb_max, min_len=min_len)
    f, t, xy, n_static = drop_static(frame_ns, f, t, xy, Trajectory(ins))
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.stem + ".tmp.npz")
    np.savez(tmp, frame_ns=frame_ns, obs_frame=f, obs_track=t, obs_xy=xy, image_wh=np.array(mask.shape[::-1]))
    os.replace(tmp, out)
    n_tr = len(np.unique(t))
    log(f"{channel} {clip.name}: {len(files)} frames, {n_tr} tracks, {len(t)} obs, "
        f"mean length {len(t) / max(n_tr, 1):.1f}, {n_static} static tracks dropped ({time.time() - t0:.0f} s)")
    return {"frames": len(files), "tracks": int(n_tr), "obs": int(len(t))}
