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


def occupancy(pts, h, w, boxes, radius, fill=255, base=0):
    """The refill mask (`fill` within `radius` of a tracked point) valid inside the given cell boxes:
    identical there to drawing every point on the full image, but only the points that can reach a
    box are drawn (the cells that need no refill are never read)."""
    occ = np.full((h, w), base, np.uint8)
    if not len(pts) or not boxes:
        return occ
    ix, iy = pts[:, 0].astype(int), pts[:, 1].astype(int)      # = int() of cv2.circle's centre
    b = np.array(boxes)                                        # (k, 4) x0 x1 y0 y1
    r = radius + 1
    near = ((ix[:, None] >= b[None, :, 0] - r) & (ix[:, None] < b[None, :, 1] + r) &
            (iy[:, None] >= b[None, :, 2] - r) & (iy[:, None] < b[None, :, 3] + r)).any(1)
    for x, y in zip(ix[near].tolist(), iy[near].tolist()):
        cv2.circle(occ, (x, y), radius, fill, -1)
    return occ


def prefetch(files, flag=cv2.IMREAD_GRAYSCALE, depth=4, fn=None):
    """Decode the images in a background thread (cv2.imread releases the GIL) while the tracker works."""
    from concurrent.futures import ThreadPoolExecutor
    fn = fn or (lambda f: cv2.imread(str(f), flag))
    with ThreadPoolExecutor(1) as ex:
        futs = [ex.submit(fn, f) for f in files[:depth]]
        for i in range(len(files)):
            img = futs[i].result()
            futs[i] = None
            if i + depth < len(files):
                futs.append(ex.submit(fn, files[i + depth]))
            yield img


def lk_device(device: str | None) -> str:
    """'cpu': the validated OpenCV CPU Lucas-Kanade (bit-identical tracks). 'cuda': the same algorithm as
    a Triton kernel on a CUDA GPU (rgb/cudalk.py; ~1.4 ms CPU per 1920x1200 frame pair instead of ~25-33;
    float interpolation: points differ from the CPU LK by 0.0002 px median / 0.02 px p99). 'opencl': OpenCV's
    LK through OpenCL (float interpolation, ~0.002 px median; on NVIDIA it costs about as much CPU as the
    CPU LK because the driver spins while it waits, so it only shortens the wall time of one tracker).
    'auto': cuda, else opencl (OpenCL GPU), else cpu."""
    device = (device or "cpu").lower()
    if device in ("auto", "cuda"):
        from . import cudalk
        if cudalk.available():
            return "cuda"
        if device == "cuda":
            raise RuntimeError("CUDA Lucas-Kanade requested but torch has no CUDA GPU / Triton")
    if device in ("auto", "opencl", "gpu"):
        ok = cv2.ocl.haveOpenCL()
        if ok:
            cv2.ocl.setUseOpenCL(True)
            try:
                ok = cv2.ocl.Device.getDefault().type() & cv2.ocl.Device_TYPE_GPU != 0
            except Exception:  # noqa
                ok = False
        if ok:
            return "opencl"
        if device != "auto":
            raise RuntimeError("OpenCL Lucas-Kanade requested but OpenCV has no OpenCL GPU")
    return "cpu"


class LKRunner:
    """Forward-backward LK of consecutive frames on the chosen device: frame(img) converts the image once
    (the GPU pyramid is reused as the previous frame of the next pair), pair(prev, cur, pts) -> as lk_pair."""

    def __init__(self, device: str, lk=LK):
        self.dev = lk_device(device)
        self.lk = lk
        self.cuda = None
        if self.dev == "cuda":
            from .cudalk import CudaLK
            self.cuda = CudaLK.from_lk(lk)

    def frame(self, img):
        if self.dev == "cuda":
            return self.cuda.pyramid(img)
        return cv2.UMat(img) if self.dev == "opencl" else img

    def pair(self, prev, cur, pts):
        if self.dev == "cuda":
            return self.cuda.pair(prev, cur, pts)
        return lk_pair(prev, cur, pts, self.lk)


def lk_pair(prev, img, pts, lk=LK):
    """Forward LK prev -> img and backward img -> prev from the forward result (numpy or cv2.UMat images)."""
    p0 = pts.reshape(-1, 1, 2)
    if isinstance(img, cv2.UMat):
        p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, img, cv2.UMat(p0), None, **lk)
        pb, st2, _ = cv2.calcOpticalFlowPyrLK(img, prev, p1, None, **lk)
        return p1.get(), st.get(), pb.get(), st2.get()
    p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, img, p0, None, **lk)
    pb, st2, _ = cv2.calcOpticalFlowPyrLK(img, prev, p1, None, **lk)
    return p1, st, pb, st2


def track(files: list[Path], mask: np.ndarray, grid=(16, 9), per_cell: int = 8,
          fb_max: float = 0.5, min_len: int = 5, device: str = "cpu"):
    """Speed (same tracks bit for bit as online_calib/tracks.py on the CPU): JPEG decoding in a background
    thread, the refill mask drawn only around the cells that are refilled; optionally the LK on the GPU
    (device='cuda' / 'opencl', see lk_device)."""
    lk = LKRunner(device)
    h, w = mask.shape
    gx, gy = grid
    cell_w, cell_h = w / gx, h / gy
    usable = np.array([[(mask[int(j * cell_h):int((j + 1) * cell_h), int(i * cell_w):int((i + 1) * cell_w)] > 0).any()
                        for i in range(gx)] for j in range(gy)])
    frame_ns = np.array([int(f.stem) for f in files], dtype=np.int64)
    obs_f, obs_t, obs_xy = [], [], []
    pts = np.zeros((0, 2), np.float32)
    ids = np.zeros(0, np.int64)
    next_id = 0
    prev = None
    for fi, img in enumerate(prefetch(files)):
        cur = lk.frame(img)
        if prev is not None and len(pts):
            p1, st, p0, st2 = lk.pair(prev, cur, pts)
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
        # (a cell whose ego mask is all zero never yields a corner: goodFeaturesToTrack is skipped there)
        cells = [(j, i) for j in range(gy) for i in range(gx) if per_cell - count[j, i] > per_cell // 2 and usable[j, i]]
        boxes = [(int(i * cell_w), int((i + 1) * cell_w), int(j * cell_h), int((j + 1) * cell_h)) for j, i in cells]
        occ = occupancy(pts, h, w, boxes, 12)
        new = []
        for (j, i), (x0, x1, y0, y1) in zip(cells, boxes):
            need = per_cell - count[j, i]
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
        prev = cur
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
               fb_max: float = 0.5, min_len: int = 5, log=print, anchor: dict | None = None, device: str = "cpu",
               frame_step: int = 1) -> dict:
    """anchor: rgb.tracks.anchor options (tracks_anchor.py); None / enabled false = the validated tracker
    (its Lucas-Kanade on `device`; the anchored tracker runs on the CPU).
    frame_step > 1: track every frame_step-th frame only (a speed option: the bundle adjustment uses
    every 3rd-4th observation of a track anyway; the LK steps become frame_step times longer)."""
    t0, c0 = time.time(), time.process_time()
    clip = Path(clip)
    if (clip / "cam" / channel / "PRUNED").exists():
        raise RuntimeError(f"{clip.name}/{channel}: the frames were thinned after tracking (extract.keep_rgb_every); "
                           f"delete {clip} to extract the window again")
    files = sorted((clip / "cam" / channel).glob("*.jpg"))[::max(1, int(frame_step))]
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(f"mask {mask_path}")
    extra, st = {}, {}
    if anchor and anchor.get("enabled"):
        from .tracks_anchor import track_anchor
        frame_ns, f, t, xy, q = track_anchor(files, mask, grid=tuple(grid), per_cell=per_cell, fb_max=fb_max,
                                             min_len=min_len, stats=st, **{k: v for k, v in anchor.items() if k != "enabled"})
        dev = "cpu"
    else:
        frame_ns, f, t, xy = track(files, mask, grid=tuple(grid), per_cell=per_cell, fb_max=fb_max, min_len=min_len,
                                   device=device)
        q = None
        dev = lk_device(device)
    t_all = t
    f, t, xy, n_static = drop_static(frame_ns, f, t, xy, Trajectory(ins))
    if q is not None:
        extra["obs_q"] = q[np.isin(t_all, t)]            # drop_static keeps whole tracks, order unchanged
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.stem + ".tmp.npz")
    np.savez(tmp, frame_ns=frame_ns, obs_frame=f, obs_track=t, obs_xy=xy, image_wh=np.array(mask.shape[::-1]), **extra)
    os.replace(tmp, out)
    n_tr = len(np.unique(t))
    log(f"{channel} {clip.name} [{dev}]: {len(files)} frames, {n_tr} tracks, {len(t)} obs, "
        f"mean length {len(t) / max(n_tr, 1):.1f}, {n_static} static tracks dropped ({time.time() - t0:.0f} s, "
        f"cpu {time.process_time() - c0:.0f} s)" + (f"; anchored {st}" if st else ""))
    return {"frames": len(files), "tracks": int(n_tr), "obs": int(len(t)), "cpu_s": time.process_time() - c0, **st}
