"""Ego-body mask for a camera that has no packaged mask (online_calib/masks.py method: pixels that
barely change while the car drives, plus corners that stay put, plus the always-dark lens border).
Built from the extracted frames of the run. Night frames make this less reliable (dark areas look
static): the packaged masks of the vehicle (data/masks, built from daytime data) are preferred."""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from ..ins import Trajectory


def body_mask(files, traj, n=60):
    """files: [(path, Trajectory of its bag)] or paths with one `traj`."""
    files = [(f, traj) if not isinstance(f, tuple) else f for f in files]
    moving = [f for f, tr in files if tr.speed(int(f.stem))[0] > 3.0]
    pick = moving[:: max(1, len(moving) // n)][:n]
    g = np.stack([cv2.GaussianBlur(cv2.imread(str(f), cv2.IMREAD_GRAYSCALE), (0, 0), 3).astype(np.float32) for f in pick])
    change = np.median(np.abs(np.diff(g, axis=0)), axis=0)
    static = (change < 5.0).astype(np.uint8)
    dark = (np.median(g, axis=0) < 12).astype(np.uint8)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
    static = cv2.morphologyEx(static, cv2.MORPH_OPEN, k)
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(static)
    h, w = static.shape
    body = np.zeros_like(static)
    for i in range(1, nlab):
        x, y, ww, hh, area = stats[i]
        touches_border = x == 0 or y == 0 or x + ww == w or y + hh == h
        if area > 0.01 * h * w and touches_border and y + hh > 0.5 * h:
            body[lab == i] = 1
    body |= dark
    body = cv2.dilate(body, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41)))
    return np.where(body > 0, 0, 255).astype(np.uint8)


def build_mask(ws, cam, wins, bag_of, out: Path):
    files, trajs = [], {}
    for w in wins:
        b = bag_of[w]
        trajs.setdefault(b, Trajectory(ws.ins(b)))
        files += [(f, trajs[b]) for f in sorted((ws.seg_dir(w) / "cam" / cam).glob("*.jpg"))]
    m = body_mask(files, None)
    cv2.imwrite(str(out), m)
    return out
