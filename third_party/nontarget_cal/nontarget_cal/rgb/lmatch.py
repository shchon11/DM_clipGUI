"""Learned local-feature verification of cross-camera / cross-time landmark links (rgb/crosstime.py).

A candidate link joins track A (camera a) and track B (camera b) whose triangulated points agree in 3-D and
reproject into both tracks. The appearance check here replaces the single SIFT descriptor distance:

  1. rectified crops: for one observation of each track, a virtual pinhole camera at the real camera centre
     looks at the merged 3-D point, with its image "up" along the world vertical and a focal length chosen
     so that both crops have the same metric scale at the point (never finer than the native resolution of
     either fisheye camera). The fisheye distortion, the camera roll/yaw and the depth ratio are removed,
     so the two crop centres show the same physical point when the link is right;
  2. a learned detector/descriptor + matcher on the crop pair (LightGlue with SuperPoint / ALIKED / DISK, or
     XFeat with LighterGlue / mutual nearest neighbour);
  3. centre transfer: a local affine map fitted (RANSAC homography inliers, the nearest ones to the centre)
     carries the centre of crop A into crop B; the link is accepted when it lands within `tol_px` (native
     pixels of camera B) of the centre of crop B, with at least `min_local` supporting matches.

Everything runs on the GPU when available (CPU fallback). The models are loaded lazily; the packages are
optional (`lightglue` from github.com/cvg/LightGlue, XFeat from github.com/verlab/accelerated_features via
`NONTARGET_XFEAT_DIR`).
"""
from __future__ import annotations

import os
import sys
import time

import cv2
import numpy as np

MATCHERS = ("superpoint_lightglue", "aliked_lightglue", "disk_lightglue", "xfeat_lighterglue", "xfeat_mnn", "ncc")
LICENSES = {
    "superpoint_lightglue": "SuperPoint weights: Magic Leap non-commercial research license; LightGlue code+weights: Apache-2.0",
    "aliked_lightglue": "ALIKED: BSD-3-Clause; LightGlue code+weights: Apache-2.0",
    "disk_lightglue": "DISK: Apache-2.0; LightGlue code+weights: Apache-2.0",
    "xfeat_lighterglue": "XFeat + LighterGlue: Apache-2.0 (LighterGlue built on LightGlue, Apache-2.0)",
    "xfeat_mnn": "XFeat: Apache-2.0",
    "ncc": "no learned model (NCC on the rectified crops only)",
}


def kb_project(Xc: np.ndarray, intr) -> np.ndarray:
    """Equidistant (Kannala-Brandt, cv2.fisheye) projection, vectorised. Xc (..., 3) -> (..., 2)."""
    fx, fy, cx, cy, k1, k2, k3, k4 = [float(v) for v in intr]
    x, y, z = Xc[..., 0], Xc[..., 1], Xc[..., 2]
    r = np.hypot(x, y)
    th = np.arctan2(r, z)
    t2 = th * th
    thd = th * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
    s = np.where(r > 1e-12, thd / np.maximum(r, 1e-12), 1.0 / np.maximum(z, 1e-12))
    return np.stack([fx * x * s + cx, fy * y * s + cy], -1)


def kb_unproject(uv: np.ndarray, intr) -> np.ndarray:
    """Inverse of kb_project: pixels (N, 2) -> unit rays (N, 3) (Newton on the equidistant polynomial)."""
    fx, fy, cx, cy, k1, k2, k3, k4 = [float(v) for v in intr]
    mx, my = (uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy
    thd = np.hypot(mx, my)
    th = thd.copy()
    for _ in range(20):
        t2 = th * th
        f = th * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))) - thd
        df = 1 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
        th = th - f / df
    s = np.where(thd > 1e-12, np.sin(th) / np.maximum(thd, 1e-12), 1.0)
    return np.stack([mx * s, my * s, np.cos(th)], -1)


def virtual_rotation(ray_c: np.ndarray, up_c: np.ndarray) -> np.ndarray:
    """R (virtual -> real camera): z along the ray, image-down (y) opposite the world vertical."""
    z = ray_c / np.linalg.norm(ray_c)
    y = -(up_c - (up_c @ z) * z)
    if np.linalg.norm(y) < 1e-3:                       # looking straight up/down: keep the camera's y
        y = np.array([0.0, 1.0, 0.0]) - z[1] * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], 1)


class CropGrid:
    """Pixel rays of an S x S virtual camera with unit focal length (cached per size)."""
    _cache: dict = {}

    @classmethod
    def rays(cls, S: int) -> np.ndarray:
        if S not in cls._cache:
            c = (S - 1) / 2.0
            u, v = np.meshgrid(np.arange(S) - c, np.arange(S) - c)
            cls._cache[S] = np.stack([u, v, np.ones_like(u)], -1).reshape(-1, 3)
        return cls._cache[S]


def rectify(img: np.ndarray, intr, R_cv: np.ndarray, fv: float, S: int) -> np.ndarray:
    """Resample an S x S virtual pinhole view (focal fv, rotation R_cv) out of a fisheye image."""
    rv = CropGrid.rays(S).copy()
    rv[:, :2] /= fv
    uv = kb_project(rv @ R_cv.T, intr).astype(np.float32).reshape(S, S, 2)
    return cv2.remap(img, uv[..., 0], uv[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


class Matcher:
    """Batch feature extraction on crops + pairwise matching. `name` in MATCHERS."""

    def __init__(self, name: str, device: str | None = None, max_kpts: int = 256, batch: int = 16):
        import torch
        if name not in MATCHERS:
            raise ValueError(f"unknown matcher {name}; one of {MATCHERS}")
        self.torch = torch
        self.name = name
        self.dev = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        if self.dev.type == "cpu":
            torch.set_num_threads(int(os.environ.get("OMP_NUM_THREADS", "4")))
        self.K = max_kpts
        self.batch = batch
        self.t_extract = 0.0
        self.t_match = 0.0
        if name == "ncc":                      # no gross check: NCC searched around the crop centre
            return
        if name.startswith("xfeat"):
            xdir = os.environ.get("NONTARGET_XFEAT_DIR", "/hdd/DM_calib/third_party/accelerated_features")
            if xdir not in sys.path:
                sys.path.insert(0, xdir)
            from modules.xfeat import XFeat
            import contextlib, io
            with contextlib.redirect_stdout(io.StringIO()):
                self.xf = XFeat(top_k=max_kpts, detection_threshold=0.0)
            self.xf.dev = self.dev
            self.xf.net = self.xf.net.to(self.dev)
            if name == "xfeat_lighterglue":
                from modules.lighterglue import LighterGlue
                with contextlib.redirect_stdout(io.StringIO()):
                    self.lg = LighterGlue().to(self.dev).eval()
        else:
            import lightglue as LG
            feat = name.split("_")[0]
            if feat == "superpoint":
                self.ex = LG.SuperPoint(max_num_keypoints=max_kpts, detection_threshold=-1.0)
            elif feat == "aliked":
                self.ex = LG.ALIKED(max_num_keypoints=max_kpts, detection_threshold=-1.0)
            else:
                self.ex = LG.DISK(max_num_keypoints=max_kpts, detection_threshold=-1.0, pad_if_not_divisible=True)
            self.ex = self.ex.eval().to(self.dev)
            # batched: no point pruning (per-batch index select); early stop over the batch is fine
            self.lg = LG.LightGlue(features=feat, width_confidence=-1, depth_confidence=0.95).eval().to(self.dev)
        self.t_extract = 0.0
        self.t_match = 0.0

    def _retry(self, fn, *a):
        """Run fn; on CUDA out-of-memory halve the batch and retry (the GPU may be shared)."""
        while True:
            try:
                return fn(*a)
            except self.torch.OutOfMemoryError:
                self.torch.cuda.empty_cache()
                if self.batch > 1:
                    self.batch //= 2
                else:                                   # GPU full (shared): continue on the CPU
                    self.to_device(self.torch.device("cpu"))
                    self.batch = 16

    def to_device(self, dev):
        self.dev = dev
        for k in ("ex", "lg"):
            if hasattr(self, k):
                setattr(self, k, getattr(self, k).to(dev))
        if hasattr(self, "xf"):
            self.xf.dev = dev
            self.xf.net = self.xf.net.to(dev)
        if dev.type == "cpu":
            self.torch.set_num_threads(int(os.environ.get("OMP_NUM_THREADS", "4")))

    def extract(self, crops: np.ndarray) -> list:
        return self._retry(self._extract, crops)

    def match(self, fa: list, fb: list, S: int) -> list:
        return self._retry(self._match, fa, fb, S)

    def _extract(self, crops: np.ndarray) -> list:
        if self.name == "ncc":
            return [None] * len(crops)
        """crops: (N, S, S) uint8 -> per crop dict of CPU/GPU tensors (keypoints, descriptors[, scales, oris])."""
        torch = self.torch
        out = []
        t0 = time.time()
        with torch.inference_mode():
            for i in range(0, len(crops), self.batch):
                x = torch.from_numpy(crops[i:i + self.batch]).to(self.dev).float().div_(255.0)[:, None]
                if self.name.startswith("xfeat"):
                    fe = self.xf.detectAndCompute(x, top_k=self.K, detection_threshold=0.0)
                    for f in fe:
                        out.append({"keypoints": f["keypoints"], "descriptors": f["descriptors"]})
                else:
                    if self.name.startswith("aliked") or self.name.startswith("disk"):
                        x = x.expand(-1, 3, -1, -1)
                    # DISK may return fewer than K points and then cannot stack a batch: one image at a time
                    parts = [x] if not self.name.startswith("disk") else [x[k:k + 1] for k in range(x.shape[0])]
                    for xp in parts:
                        fe = self.ex({"image": xp})
                        for k in range(xp.shape[0]):
                            out.append({k_: v[k] for k_, v in fe.items() if k_ in ("keypoints", "descriptors", "scales", "oris")})
        if self.dev.type == "cuda":
            torch.cuda.synchronize()
        self.t_extract += time.time() - t0
        return out

    def _match(self, fa: list, fb: list, S: int) -> list:
        """Pairs (fa[i], fb[i]) -> list of (kpA (M,2), kpB (M,2)) numpy arrays in crop pixels."""
        if self.name == "ncc":
            return [(np.zeros((0, 2)), np.zeros((0, 2)))] * len(fa)
        torch = self.torch
        res = []
        t0 = time.time()
        if fa and fa[0] is not None and fa[0]["keypoints"].device != self.dev:   # after a switch to the CPU
            fa = [{k: v.to(self.dev) for k, v in f.items()} for f in fa]
            fb = [{k: v.to(self.dev) for k, v in f.items()} for f in fb]
        size = torch.tensor([[S, S]], dtype=torch.float32, device=self.dev)
        with torch.inference_mode():
            if self.name == "xfeat_mnn":
                for a, b in zip(fa, fb):
                    i0, i1 = self.xf.match(a["descriptors"], b["descriptors"], min_cossim=0.82)
                    res.append((a["keypoints"][i0].cpu().numpy(), b["keypoints"][i1].cpu().numpy()))
            elif self.name == "xfeat_lighterglue":
                for a, b in zip(fa, fb):
                    data = {"keypoints0": a["keypoints"][None], "keypoints1": b["keypoints"][None],
                            "descriptors0": a["descriptors"][None], "descriptors1": b["descriptors"][None],
                            "image_size0": size, "image_size1": size}
                    o = self.lg(data, min_conf=0.1)
                    m = o["matches"][0]
                    res.append((a["keypoints"][m[:, 0]].cpu().numpy(), b["keypoints"][m[:, 1]].cpu().numpy()))
            else:
                for i in range(0, len(fa), self.batch):
                    A, B = fa[i:i + self.batch], fb[i:i + self.batch]
                    if len({len(f["keypoints"]) for f in A + B}) > 1:      # unequal counts: pair by pair
                        for a, b in zip(A, B):
                            o = self.lg({"image0": {**{k: v[None] for k, v in a.items()}, "image_size": size},
                                         "image1": {**{k: v[None] for k, v in b.items()}, "image_size": size}})
                            m = o["matches"][0]
                            res.append((a["keypoints"][m[:, 0]].cpu().numpy(), b["keypoints"][m[:, 1]].cpu().numpy()))
                        continue
                    n = len(A)
                    d0 = {k: torch.stack([a[k] for a in A]) for k in A[0]}
                    d1 = {k: torch.stack([b[k] for b in B]) for k in B[0]}
                    d0["image_size"] = size.expand(n, 2)
                    d1["image_size"] = size.expand(n, 2)
                    o = self.lg({"image0": d0, "image1": d1})
                    for k in range(n):
                        m = o["matches"][k]
                        res.append((d0["keypoints"][k][m[:, 0]].cpu().numpy(), d1["keypoints"][k][m[:, 1]].cpu().numpy()))
        if self.dev.type == "cuda":
            torch.cuda.synchronize()
        self.t_match += time.time() - t0
        return res


def centre_transfer(ka: np.ndarray, kb: np.ndarray, S: int, ransac_px: float = 2.0, radius: float = 0.3,
                    min_local: int = 6):
    """Map the centre of crop A into crop B with a local affine map of the matches that agree with a
    RANSAC homography. Returns (n_matches, n_inliers, n_local, err_crop_px [inf if unverified], inlier mask)."""
    n = len(ka)
    c = np.array([(S - 1) / 2.0, (S - 1) / 2.0])
    if n < max(min_local, 5):
        return n, 0, 0, np.inf, np.zeros(n, bool)
    H, m = cv2.findHomography(ka.astype(np.float64), kb.astype(np.float64), cv2.USAC_MAGSAC, ransac_px,
                              maxIters=2000, confidence=0.999)
    if H is None:
        return n, 0, 0, np.inf, np.zeros(n, bool)
    inl = m.ravel().astype(bool)
    d = np.linalg.norm(ka - c, axis=1)
    loc = np.flatnonzero(inl & (d <= radius * S))
    if len(loc) < min_local:
        return n, int(inl.sum()), int(len(loc)), np.inf, inl
    loc = loc[np.argsort(d[loc])[:max(min_local, 12)]]
    A = np.c_[ka[loc], np.ones(len(loc))]
    M, *_ = np.linalg.lstsq(A, kb[loc], rcond=None)
    pb = np.r_[c, 1.0] @ M
    centre_transfer.last = pb - c                       # signed offset (diagnostics)
    return n, int(inl.sum()), int(len(loc)), float(np.linalg.norm(pb - c)), inl


def ncc_refine(ca: np.ndarray, cb: np.ndarray, pred: np.ndarray | None = None, half: int = 10, search: int = 6):
    """Sub-pixel position in crop B of the (2*half+1)^2 template at the centre of crop A (normalised cross-
    correlation, parabola peak), searched +-search px around `pred` (default: the centre of B).
    Returns (offset from the centre of B (2,), peak NCC); (None, -1) when out of bounds or textureless."""
    S = ca.shape[0]
    c = (S - 1) // 2
    tpl = ca[c - half:c + half + 1, c - half:c + half + 1]
    if tpl.std() < 2.0:
        return None, -1.0
    p = np.array([c, c], float) if pred is None else np.asarray(pred, float)
    x0, y0 = int(round(p[0])) - half - search, int(round(p[1])) - half - search
    w = 2 * (half + search) + 1
    if x0 < 0 or y0 < 0 or x0 + w > S or y0 + w > S:
        return None, -1.0
    r = cv2.matchTemplate(cb[y0:y0 + w, x0:x0 + w], tpl, cv2.TM_CCOEFF_NORMED)
    _, mx, _, (ix, iy) = cv2.minMaxLoc(r)
    dx = dy = 0.0
    if 0 < ix < r.shape[1] - 1:
        a, b_, cc = r[iy, ix - 1], r[iy, ix], r[iy, ix + 1]
        den = a - 2 * b_ + cc
        dx = 0.5 * (a - cc) / den if den < 0 else 0.0
    if 0 < iy < r.shape[0] - 1:
        a, b_, cc = r[iy - 1, ix], r[iy, ix], r[iy + 1, ix]
        den = a - 2 * b_ + cc
        dy = 0.5 * (a - cc) / den if den < 0 else 0.0
    pos = np.array([x0 + ix + half + dx, y0 + iy + half + dy])
    return pos - c, float(mx)
