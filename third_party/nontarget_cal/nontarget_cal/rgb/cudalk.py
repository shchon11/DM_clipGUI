"""Pyramidal Lucas-Kanade on CUDA (Triton kernel + torch), the algorithm of OpenCV's calcOpticalFlowPyrLK.

Why: OpenCV's OpenCL LK on an NVIDIA GPU costs about as much CPU as the CPU LK - the calling thread spins
while it waits for every result and the driver's helper thread spins ~170 ms after each GPU call, i.e.
all the time (measured: 42 ms CPU per 1920x1200 frame with OpenCL, 48 ms with the CPU LK, of which the
LK is ~25-33 ms). Here the LK is one kernel launch per pyramid level and direction, the pyramid and the
Scharr derivatives are a few torch ops per image (each computed once and used by the forward step of
one frame pair and the backward step of the next), and the CPU waits for the GPU without spinning
(event query + sleep): the LK costs ~1 ms CPU per frame.

Same algorithm as OpenCV's CPU LKTrackerInvoker (lkpyramid.cpp): pyramid by pyrDown (5x5 binomial,
BORDER_REFLECT_101, uint8 rounding), levels read with BORDER_REFLECT_101, Scharr derivatives of the
previous image with a zero border, bilinear patches of winSize around the point, the spatial gradient
matrix once per level, minimum-eigenvalue test (minEigThreshold 1e-4, as OpenCV scales it), iterations
until |delta|^2 <= eps^2 or the oscillation test (|delta + previous delta| < 0.01 px -> half step back),
status 0 at level 0 when the point leaves the image or the matrix is degenerate. Differences to OpenCV's
CPU code: float instead of 14-bit fixed-point interpolation weights (as OpenCV's own OpenCL kernel).
"""
from __future__ import annotations

import threading
import time

import numpy as np

_STATE = {}
_LOCK = threading.Lock()


def _wait_event(ev):
    while not ev.query():             # no spin: the CPU sleeps while the GPU works
        time.sleep(0.0002)


def available() -> bool:
    """CUDA GPU + Triton importable (torch wheels ship Triton on Linux)."""
    if "ok" in _STATE:
        return _STATE["ok"]
    ok = False
    try:
        import torch
        if torch.cuda.is_available():
            import triton  # noqa: F401
            _kernel()
            ok = True
    except Exception:  # noqa
        ok = False
    _STATE["ok"] = ok
    return ok


def _kernel():
    if "k" in _STATE:
        return _STATE["k"]
    import triton
    import triton.language as tl

    @triton.jit
    def _refl(x, n):
        # BORDER_REFLECT_101 for -n < x < 2n - 1, then clamped (safety)
        x = tl.where(x < 0, -x, x)
        x = tl.where(x >= n, 2 * n - 2 - x, x)
        return tl.minimum(tl.maximum(x, 0), n - 1)

    @triton.jit(do_not_specialize=["H", "W", "top", "level0", "max_iter"])
    def lk_level(I, DX, DY, J, H, W, PREV, NEXT, STATUS, lvl_scale, top, level0, max_iter, eps2, min_eig,
                 WIN: tl.constexpr, BLOCK: tl.constexpr):
        pid = tl.program_id(0)
        FLT_SCALE: tl.constexpr = 1.0 / 1048576.0
        half = (WIN - 1) * 0.5
        px = tl.load(PREV + 2 * pid) * lvl_scale
        py = tl.load(PREV + 2 * pid + 1) * lvl_scale
        gx = tl.load(NEXT + 2 * pid)
        gy = tl.load(NEXT + 2 * pid + 1)
        nx = tl.where(top != 0, px, gx * 2.0)
        ny = tl.where(top != 0, py, gy * 2.0)
        outx = nx
        outy = ny
        st = tl.load(STATUS + pid)
        px = px - half
        py = py - half
        ipx = tl.floor(px).to(tl.int32)
        ipy = tl.floor(py).to(tl.int32)
        active = ((ipx >= -WIN) & (ipx < W) & (ipy >= -WIN) & (ipy < H)).to(tl.int32)
        if active == 0:
            if level0 != 0:
                st = st * 0
        if active != 0:
            a = px - ipx.to(tl.float32)
            b = py - ipy.to(tl.float32)
            w00 = (1.0 - a) * (1.0 - b)
            w01 = a * (1.0 - b)
            w10 = (1.0 - a) * b
            w11 = a * b
            lane = tl.arange(0, BLOCK)
            valid = lane < WIN * WIN
            lx = lane % WIN
            ly = lane // WIN
            x0 = ipx + lx
            y0 = ipy + ly
            rx0 = _refl(x0, W)
            rx1 = _refl(x0 + 1, W)
            ry0 = _refl(y0, H)
            ry1 = _refl(y0 + 1, H)
            i00 = tl.load(I + ry0 * W + rx0, mask=valid, other=0).to(tl.float32)
            i01 = tl.load(I + ry0 * W + rx1, mask=valid, other=0).to(tl.float32)
            i10 = tl.load(I + ry1 * W + rx0, mask=valid, other=0).to(tl.float32)
            i11 = tl.load(I + ry1 * W + rx1, mask=valid, other=0).to(tl.float32)
            ival = (w00 * i00 + w01 * i01 + w10 * i10 + w11 * i11) * 32.0
            # derivatives: zero outside the image (OpenCV: copyMakeBorder BORDER_CONSTANT)
            inx0 = (x0 >= 0) & (x0 < W)
            inx1 = (x0 + 1 >= 0) & (x0 + 1 < W)
            iny0 = (y0 >= 0) & (y0 < H)
            iny1 = (y0 + 1 >= 0) & (y0 + 1 < H)
            cx0 = tl.minimum(tl.maximum(x0, 0), W - 1)
            cx1 = tl.minimum(tl.maximum(x0 + 1, 0), W - 1)
            cy0 = tl.minimum(tl.maximum(y0, 0), H - 1)
            cy1 = tl.minimum(tl.maximum(y0 + 1, 0), H - 1)
            m00 = valid & iny0 & inx0
            m01 = valid & iny0 & inx1
            m10 = valid & iny1 & inx0
            m11 = valid & iny1 & inx1
            ix = (w00 * tl.load(DX + cy0 * W + cx0, mask=m00, other=0.0) + w01 * tl.load(DX + cy0 * W + cx1, mask=m01, other=0.0)
                  + w10 * tl.load(DX + cy1 * W + cx0, mask=m10, other=0.0) + w11 * tl.load(DX + cy1 * W + cx1, mask=m11, other=0.0))
            iy = (w00 * tl.load(DY + cy0 * W + cx0, mask=m00, other=0.0) + w01 * tl.load(DY + cy0 * W + cx1, mask=m01, other=0.0)
                  + w10 * tl.load(DY + cy1 * W + cx0, mask=m10, other=0.0) + w11 * tl.load(DY + cy1 * W + cx1, mask=m11, other=0.0))
            ix = tl.where(valid, ix, 0.0)
            iy = tl.where(valid, iy, 0.0)
            A11 = tl.sum(ix * ix, 0) * FLT_SCALE
            A12 = tl.sum(ix * iy, 0) * FLT_SCALE
            A22 = tl.sum(iy * iy, 0) * FLT_SCALE
            D = A11 * A22 - A12 * A12
            me = (A22 + A11 - tl.sqrt((A11 - A22) * (A11 - A22) + 4.0 * A12 * A12)) / (2.0 * WIN * WIN)
            if (me < min_eig) | (D < 1.1920928955078125e-07):
                if level0 != 0:
                    st = st * 0
                active = active * 0
            if active != 0:
                Dinv = 1.0 / D
                nx = nx - half
                ny = ny - half
                pdx = nx * 0.0
                pdy = nx * 0.0
                j = 0
                while (j < max_iter) & (active != 0):
                    inx = tl.floor(nx).to(tl.int32)
                    iny = tl.floor(ny).to(tl.int32)
                    inb = (inx >= -WIN) & (inx < W) & (iny >= -WIN) & (iny < H)
                    if inb == 0:
                        if level0 != 0:
                            st = st * 0
                        active = active * 0
                    else:
                        a2 = nx - inx.to(tl.float32)
                        b2 = ny - iny.to(tl.float32)
                        v00 = (1.0 - a2) * (1.0 - b2)
                        v01 = a2 * (1.0 - b2)
                        v10 = (1.0 - a2) * b2
                        v11 = a2 * b2
                        xj = inx + lx
                        yj = iny + ly
                        sx0 = _refl(xj, W)
                        sx1 = _refl(xj + 1, W)
                        sy0 = _refl(yj, H)
                        sy1 = _refl(yj + 1, H)
                        j00 = tl.load(J + sy0 * W + sx0, mask=valid, other=0).to(tl.float32)
                        j01 = tl.load(J + sy0 * W + sx1, mask=valid, other=0).to(tl.float32)
                        j10 = tl.load(J + sy1 * W + sx0, mask=valid, other=0).to(tl.float32)
                        j11 = tl.load(J + sy1 * W + sx1, mask=valid, other=0).to(tl.float32)
                        diff = (v00 * j00 + v01 * j01 + v10 * j10 + v11 * j11) * 32.0 - ival
                        diff = tl.where(valid, diff, 0.0)
                        b1 = tl.sum(diff * ix, 0) * FLT_SCALE
                        bb = tl.sum(diff * iy, 0) * FLT_SCALE
                        dx = (A12 * bb - A22 * b1) * Dinv
                        dy = (A12 * b1 - A11 * bb) * Dinv
                        nx = nx + dx
                        ny = ny + dy
                        outx = nx + half
                        outy = ny + half
                        if dx * dx + dy * dy <= eps2:
                            active = active * 0
                        elif (j > 0) & (tl.abs(dx + pdx) < 0.01) & (tl.abs(dy + pdy) < 0.01):
                            outx = outx - dx * 0.5
                            outy = outy - dy * 0.5
                            active = active * 0
                        pdx = dx
                        pdy = dy
                        j += 1
        tl.store(NEXT + 2 * pid, outx)
        tl.store(NEXT + 2 * pid + 1, outy)
        tl.store(STATUS + pid, st)

    _STATE["k"] = lk_level
    return lk_level


class CudaLK:
    """One tracker's GPU state (own CUDA stream; several trackers can run in threads of one process)."""

    def __init__(self, win: int = 21, max_level: int = 4, max_iter: int = 30, eps: float = 0.01,
                 min_eig: float = 1e-4):
        import torch
        import torch.nn.functional as F
        self.torch, self.F = torch, F
        self.win, self.max_level, self.max_iter = int(win), int(max_level), int(max_iter)
        self.eps2 = float(eps) ** 2
        self.min_eig = float(min_eig)
        self.dev = torch.device("cuda")
        self.stream = torch.cuda.Stream()
        k = torch.tensor([1., 4., 6., 4., 1.], device=self.dev)
        self.kdown = (k[:, None] * k[None, :]).view(1, 1, 5, 5)
        sx = torch.tensor([[-3., 0., 3.], [-10., 0., 10.], [-3., 0., 3.]], device=self.dev)
        self.kd = torch.stack([sx, sx.t()]).view(2, 1, 3, 3)
        self.kern = _kernel()
        self._stage = None
        self._up = None
        with _LOCK:                   # compile / load the kernel once per process, not in racing threads
            if "warm" not in _STATE:
                z = np.zeros((64, 64), np.uint8)
                z[20:40, 20:40] = 200
                pz = self.pyramid(z)
                self.pair(pz, pz, np.array([[20.0, 20.0]], np.float32))
                _STATE["warm"] = True

    @classmethod
    def from_lk(cls, lk: dict):
        """From the OpenCV-style parameters dict (winSize, maxLevel, criteria)."""
        crit = lk.get("criteria", (3, 30, 0.01))
        return cls(win=lk["winSize"][0], max_level=lk["maxLevel"], max_iter=int(crit[1]), eps=float(crit[2]))

    def pyramid(self, img: np.ndarray):
        """uint8 image -> [(level uint8 (H,W), dx float32, dy float32)] on the GPU (levels as OpenCV's
        buildOpticalFlowPyramid: pyrDown; levels smaller than the window are not built)."""
        torch, F = self.torch, self.F
        with torch.cuda.stream(self.stream):
            # one pinned staging buffer per tracker (pin_memory() per image costs ~10 ms CPU); the previous
            # upload from it is complete: pair() waited for the stream
            if self._stage is None or tuple(self._stage.shape) != img.shape:
                self._stage = torch.empty(img.shape, dtype=torch.uint8, pin_memory=True)
            elif self._up is not None:
                _wait_event(self._up)
            self._stage.numpy()[...] = img
            g = self._stage.to(self.dev, non_blocking=True)
            self._up = torch.cuda.Event()
            self._up.record(self.stream)
            levels = [g]
            for _ in range(self.max_level):
                p = levels[-1]
                h, w = p.shape
                if (h + 1) // 2 <= self.win or (w + 1) // 2 <= self.win:
                    break
                x = F.pad(p.float()[None, None], (2, 2, 2, 2), mode="reflect")
                y = F.conv2d(x, self.kdown, stride=2)
                levels.append(torch.floor((y[0, 0] + 128.0) / 256.0).to(torch.uint8))
            out = []
            for p in levels:
                x = F.pad(p.float()[None, None], (1, 1, 1, 1), mode="reflect")
                d = F.conv2d(x, self.kd)[0]
                out.append((p.contiguous(), d[0].contiguous(), d[1].contiguous()))
        return out

    def _wait(self):
        ev = self.torch.cuda.Event()
        ev.record(self.stream)
        _wait_event(ev)

    def flow(self, pyr_i, pyr_j, pts_dev):
        """LK from image I (pyramid with derivatives) to image J for points (N, 2) on the GPU ->
        (next points (N, 2), status (N,) int32) on the GPU."""
        torch = self.torch
        n = pts_dev.shape[0]
        nxt = torch.zeros_like(pts_dev)
        st = torch.ones(n, dtype=torch.int32, device=self.dev)
        L = min(len(pyr_i), len(pyr_j)) - 1
        for lvl in range(L, -1, -1):
            I, dx, dy = pyr_i[lvl]
            Jl = pyr_j[lvl][0]
            H, W = I.shape
            self.kern[(n,)](I, dx, dy, Jl, H, W, pts_dev, nxt, st, 1.0 / (1 << lvl), int(lvl == L), int(lvl == 0),
                            self.max_iter, self.eps2, self.min_eig, WIN=self.win, BLOCK=512 if self.win <= 22 else 1024)
        return nxt, st

    def pair(self, pyr_prev, pyr_cur, pts: np.ndarray):
        """Forward prev -> cur and backward cur -> prev from the forward result, as rgb/tracks.lk_pair:
        -> p1 (N,1,2), st (N,1), pb (N,1,2), st2 (N,1) numpy."""
        torch = self.torch
        n = len(pts)
        with torch.cuda.stream(self.stream):
            p0 = torch.from_numpy(np.ascontiguousarray(pts, dtype=np.float32).reshape(n, 2)).to(self.dev, non_blocking=False)
            p1, st = self.flow(pyr_prev, pyr_cur, p0)
            pb, st2 = self.flow(pyr_cur, pyr_prev, p1)
            res = torch.cat([p1, pb, st[:, None].float(), st2[:, None].float()], 1).to("cpu", non_blocking=True)
        self._wait()
        r = res.numpy()
        return (r[:, 0:2].reshape(n, 1, 2).copy(), r[:, 4:5].astype(np.uint8), r[:, 2:4].reshape(n, 1, 2).copy(),
                r[:, 5:6].astype(np.uint8))
