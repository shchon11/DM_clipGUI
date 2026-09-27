"""Near-range LiDAR edges for the thermal edge-alignment term, accumulated with the LO trajectory.

Verbatim numerics of thermal_lo/code/edge_prep.py and of the thermal_online helpers it uses
(thc.organise, thc.depth_edges, prep.support_count, prep.thermal_edges). See edge_prep.py's
docstring (reproduced in `process`) for the method. Output: <workdir>/thermal/edges/<win>_<cam>.npz
"""
from __future__ import annotations

import math
import os
import time

import cv2
import numpy as np

from ..config import pkg_data
from ..lo.lotraj import LOTraj
from .timemodel import frame_index, time_model

CAMS = ("thermal_left", "thermal_right")
W, H = 640, 480
DCOL = 0.1 / 1024              # s per firing column at 10 Hz
DAZ = 2 * np.pi / 1024
CLAHE = cv2.createCLAHE(3.0, (8, 8))
_RING_SHIFT = None


def ring_shift() -> np.ndarray:
    """Per-ring destagger shift of the OS-2-128 (sensor constant, thermal_online/ring_shift.npy)."""
    global _RING_SHIFT
    if _RING_SHIFT is None:
        _RING_SHIFT = np.load(os.environ.get("NONTARGET_RING_SHIFT", str(pkg_data("ring_shift.npy"))))
    return _RING_SHIFT


def organise(s: np.ndarray):
    """-> xyz (128,1024,3) float64 with NaN holes, t (128,1024) s, refl (128,1024)."""
    sh = ring_shift()
    m = np.rint(s["t"] / DCOL).astype(np.int64)
    r = s["ring"].astype(np.int64)
    c = (m - sh[r]) % 1024
    xyz = np.full((128, 1024, 3), np.nan)
    t = np.full((128, 1024), np.nan)
    rf = np.zeros((128, 1024), np.float32)
    xyz[r, c, 0] = s["x"]; xyz[r, c, 1] = s["y"]; xyz[r, c, 2] = s["z"]
    t[r, c] = s["t"]; rf[r, c] = s["intensity"]
    return xyz, t, rf


def depth_edges(xyz, jump_abs=0.5, jump_rel=0.08, ratio=4.0, rmin=2.5, rmax=60.0):
    """Occlusion (depth-discontinuity) edges of an organised sweep.

    Along each ring (axis 1, "h") and down each column (axis 0, "v"): a jump between
    neighbours a|b that is large in absolute and relative terms AND much larger than the
    range steps just outside it on both sides (so a surface seen at grazing incidence,
    e.g. the road across rings, is not an edge). The edge point is put on the foreground
    surface, in the direction half way between the last foreground and the first
    background sample (the true boundary is uniformly distributed between them).

    Returns (P (n,3) edge points, fg index pairs (n,2) row/col, kind (n,) 0=h 1=v,
             jump (n,) m, rng (n,) m, tpair (n,) index into the flattened grid of the fg sample).
    """
    rng = np.linalg.norm(xyz, axis=-1)
    out = []
    for axis in (1, 0):
        R = rng if axis == 1 else rng.T
        X = xyz if axis == 1 else xyz.transpose(1, 0, 2)
        if axis == 1:   # wrap around the ring
            Rp = np.concatenate([R[:, -2:], R, R[:, :3]], 1); Xp = np.concatenate([X[:, -2:], X, X[:, :3]], 1)
            o = 2
        else:
            Rp = np.concatenate([np.full((R.shape[0], 2), np.nan), R, np.full((R.shape[0], 3), np.nan)], 1)
            Xp = np.concatenate([np.full((X.shape[0], 2, 3), np.nan), X, np.full((X.shape[0], 3, 3), np.nan)], 1)
            o = 2
        n = R.shape[1]
        a = Rp[:, o:o + n]; b = Rp[:, o + 1:o + n + 1]
        am = Rp[:, o - 1:o + n - 1]; bp = Rp[:, o + 2:o + n + 2]
        d = b - a
        near = np.fmin(a, b)
        with np.errstate(invalid="ignore"):
            ok = (np.abs(d) > np.maximum(jump_abs, jump_rel * near))
            side = np.fmax(np.abs(a - am), np.abs(bp - b))
            ok &= np.abs(d) > ratio * np.maximum(side, 0.05)
            ok &= np.isfinite(am) & np.isfinite(bp)
            ok &= (near > rmin) & (near < rmax)
        ii, jj = np.nonzero(ok)
        if len(ii) == 0:
            continue
        fg_is_a = a[ii, jj] < b[ii, jj]
        Xa = Xp[ii, o + jj]; Xb = Xp[ii, o + jj + 1]
        ua = Xa / np.linalg.norm(Xa, axis=1, keepdims=True); ub = Xb / np.linalg.norm(Xb, axis=1, keepdims=True)
        um = ua + ub; um /= np.linalg.norm(um, axis=1, keepdims=True)
        rf = np.where(fg_is_a, a[ii, jj], b[ii, jj])
        P = um * rf[:, None]
        jf = np.where(fg_is_a, jj, (jj + 1) % n if axis == 1 else jj + 1)
        if axis == 1:
            rc = np.stack([ii, jf], 1)
        else:
            rc = np.stack([jf, ii], 1)
        out.append((P, rc, np.full(len(ii), 0 if axis == 1 else 1), np.abs(d[ii, jj]), rf))
    if not out:
        return (np.zeros((0, 3)), np.zeros((0, 2), int), np.zeros(0, int), np.zeros(0), np.zeros(0))
    return tuple(np.concatenate([o[k] for o in out]) for k in range(5))


def thermal_edges(img8):
    img = CLAHE.apply(img8).astype(np.float32)
    b = cv2.GaussianBlur(img, (0, 0), 1.2)
    gx = cv2.Sobel(b, cv2.CV_32F, 1, 0, ksize=3) / 8
    gy = cv2.Sobel(b, cv2.CV_32F, 0, 1, ksize=3) / 8
    g = np.hypot(gx, gy)
    lo, hi = np.percentile(g, [75, 90])
    sc = 256.0 / max(hi, 1e-6)
    e = cv2.Canny((gx * sc).astype(np.int16), (gy * sc).astype(np.int16), lo * sc, 256, L2gradient=True)
    ys, xs = np.nonzero(e)                      # raster order == DIST_LABEL_PIXEL label order
    gg = g[ys, xs]
    nx = gx[ys, xs] / np.maximum(gg, 1e-9); ny = gy[ys, xs] / np.maximum(gg, 1e-9)
    # subpixel: parabola through |g| at -1, 0, +1 along the normal
    def samp(dx, dy):
        return cv2.remap(g, (xs + dx).astype(np.float32).reshape(1, -1),
                         (ys + dy).astype(np.float32).reshape(1, -1), cv2.INTER_LINEAR).ravel()
    gm, gp = samp(-nx, -ny), samp(nx, ny)
    den = gm - 2 * gg + gp
    dl = np.where(den < -1e-9, 0.5 * (gm - gp) / np.where(den < -1e-9, den, -1), 0.0)
    dl = np.clip(dl, -0.5, 0.5)
    ex = np.stack([xs + dl * nx, ys + dl * ny, nx, ny, gg / max(hi, 1e-6)], 1).astype(np.float32)
    _, lab = cv2.distanceTransformWithLabels(255 - e, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
    lab = lab.astype(np.int64)
    # labels are 1..N over the zero pixels of (255-e) i.e. the edge pixels, in raster order
    lab[lab > len(xs)] = 0
    gmag = np.clip(g / max(hi, 1e-6) * 128, 0, 255).astype(np.uint8)
    return ex, lab.astype(np.uint16) if len(xs) < 65535 else None, gmag


def support_count(rc, kind):
    """How many other edges of the same kind touch this one along the boundary direction."""
    key = {}
    for i, (r, c, k) in enumerate(zip(rc[:, 0], rc[:, 1], kind)):
        key[(int(r), int(c), int(k))] = i
    sup = np.zeros(len(kind), np.int8)
    for i, (r, c, k) in enumerate(zip(rc[:, 0], rc[:, 1], kind)):
        n = 0
        if k == 0:     # along-ring jump -> boundary runs across rings
            for dr in (-1, 1):
                for dc in (-1, 0, 1):
                    n += (int(r) + dr, (int(c) + dc) % 1024, 0) in key
        else:
            for dc in (-1, 1):
                for dr in (-1, 0, 1):
                    n += (int(r) + dr, (int(c) + dc) % 1024, 1) in key
        sup[i] = min(n, 6)
    return sup


def proj(Pc, K):
    fx, fy, cx, cy, k1, k2 = K
    x, y = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]
    r2 = x * x + y * y
    d = 1 + k1 * r2 + k2 * r2 * r2
    return np.stack([fx * x * d + cx, fy * y * d + cy], -1)


def process(ws, plan, seg, calib, every=6, hist=1.6, maxp=1500, cams=CAMS, time_kind="smooth", log=print):
    cv2.setNumThreads(1)
    ZERO_NS = plan.t_ref
    bag = plan.bag(seg)
    t0 = time.time()
    tr = LOTraj.load(ws.lo("ref", seg))
    a_, b_ = tr.span()
    files = sorted((ws.seg_dir(seg) / "lidar").glob("*.npy"))
    hdr = np.array([int(f.stem) for f in files], np.int64)
    ok = (hdr >= a_ - 100_000_000) & (hdr + 100_000_000 <= b_)
    files, hdr = [f for f, k in zip(files, ok) if k], hdr[ok]
    # ---- 1. depth edges of every 2nd sweep, in the LO world (computed with the LO of the window, cached)
    sw = sweep_edges(ws, seg, tr, files, hdr)
    log(f"{seg}: {len(sw)} sweeps of edges, {sum(len(v[0]) for v in sw.values())} points, {time.time() - t0:.0f} s", flush=True)
    swh = np.array(sorted(sw))
    from ..fastops import load_npz_mmap
    mp = load_npz_mmap(ws.lo("map", seg))          # only the slices of each frame's history are read
    MP, mst, mhdr = mp["P"], mp["start"], mp["hdr"]
    from .frames import ThermalFrames
    for ci, cam in enumerate(cams):
        frames = ThermalFrames(ws.thermal16(seg), cam)
        cc = calib["cameras"][cam]
        T_cl = np.array(cc["T_cam_lidar"])
        K = cc["intr"]
        dt = cc["dt_s_mean"]
        ix = frame_index(ws, bag, cam, plan.wins_of_bag(bag))
        h_all, tf_all, _ = time_model(ws, bag, cam, plan.wins_of_bag(bag), time_kind)
        wa, wb = plan.span_ns(seg)
        sel = np.flatnonzero((h_all >= wa) & (h_all <= wb) & ~ix["ffc"])
        sel = sel[::every]
        out_X, out_k, out_fi, out_r, E, eoff, fh, ftf = [], [], [], [], [], [0], [], []
        for f in sel:
            tcap = tf_all[f] + dt * 1e9                   # centre-row capture time (ns, float)
            if tcap < a_ + 2e8 or tcap > b_ - 2e8:
                continue
            Rw, pw = tr.R(np.array([tcap]))[0], tr.p(np.array([tcap]))[0]
            use = swh[(swh >= tcap - hist * 1e9) & (swh <= tcap)]
            if not len(use):
                continue
            Pw = np.concatenate([sw[int(k)][0] for k in use]).astype(np.float64)
            kd = np.concatenate([sw[int(k)][1] for k in use])
            PL = (Pw - pw) @ Rw
            Pc = PL @ T_cl[:3, :3].T + T_cl[:3, 3]
            inf = (Pc[:, 2] > 1.5) & (Pc[:, 2] < 50) & (np.abs(Pc[:, 0]) < 0.62 * Pc[:, 2]) & (np.abs(Pc[:, 1]) < 0.46 * Pc[:, 2])
            Pw, kd, Pc = Pw[inf], kd[inf], Pc[inf]
            if len(Pw) < 30:
                continue
            uv = proj(Pc, K)
            # z-buffer of the accumulated map
            ks = np.flatnonzero((mhdr >= tcap - hist * 1e9) & (mhdr <= tcap + 1e8))
            zb = zbuffer(MP, int(mst[ks[0]]), int(mst[ks[-1] + 1]), pw, Rw, T_cl, K)
            ie = (uv[:, 0] >= 4) & (uv[:, 0] < W - 4) & (uv[:, 1] >= 4) & (uv[:, 1] < H - 4)
            cu = np.clip((uv[:, 0] // 8).astype(int), 0, W // 8); cv_ = np.clip((uv[:, 1] // 8).astype(int), 0, H // 8)
            vis = ie & (Pc[:, 2] <= 1.05 * zb[cv_, cu] + 0.2)
            Pw, kd, Pc = Pw[vis], kd[vis], Pc[vis]
            # 5 cm de-dup, near points first
            key = np.floor(Pw / 0.05).astype(np.int64)
            _, first = np.unique(key[:, 0] * 10**12 + key[:, 1] * 10**6 + key[:, 2] + kd * 7, return_index=True)
            Pw, kd, Pc = Pw[first], kd[first], Pc[first]
            if len(Pw) > maxp:
                near = Pc[:, 2] < 12
                rest = np.flatnonzero(~near)
                keep = np.r_[np.flatnonzero(near), np.random.default_rng(int(f)).permutation(rest)[:max(0, maxp - near.sum())]]
                Pw, kd, Pc = Pw[keep], kd[keep], Pc[keep]
            raw = frames.read(h_all[f]).astype(np.float32)
            lo, hi = np.percentile(raw, [1.0, 99.5])
            img8 = np.clip((raw - lo) / max(hi - lo, 1.0) * 255, 0, 255).astype(np.uint8)
            ex, _, _ = thermal_edges(img8)
            fi = len(fh)
            out_X.append(Pw.astype(np.float32)); out_k.append(kd); out_fi.append(np.full(len(Pw), fi, np.int32))
            out_r.append(Pc[:, 2].astype(np.float32))
            E.append(ex.astype(np.float32)); eoff.append(eoff[-1] + len(ex))
            fh.append(int(h_all[f])); ftf.append((tf_all[f] - ZERO_NS) * 1e-9)
        EDGES = ws.thermal_edges_dir()
        EDGES.mkdir(parents=True, exist_ok=True)
        if not out_X:
            log(f"{seg} {cam}: no usable frames for the edge term")
            continue
        X = np.concatenate(out_X); r = np.concatenate(out_r)
        tmp = EDGES / f"{seg}_{cam}.tmp.npz"
        np.savez(tmp, frame_hdr=np.array(fh, np.int64), frame_tf=np.array(ftf),
                 X=X, kind=np.concatenate(out_k), fi=np.concatenate(out_fi), rng_cam=r,
                 E=np.concatenate(E), eoff=np.array(eoff, np.int64), hist_s=hist)
        os.replace(tmp, EDGES / f"{seg}_{cam}.npz")
        log(f"{seg} {cam}: {len(fh)} frames, {len(X)} edge points (<5 m {np.mean(r < 5):.2f}, <10 m {np.mean(r < 10):.2f}), "
              f"{time.time() - t0:.0f} s")




def sweep_edges(ws, seg, tr, files, hdr):
    """{sweep header: (world edge points f4, kind i1, foreground range f4)} of every 2nd sweep, the part of the
    edge term that does not depend on the thermal calibration. Cached as lo/tedge_<win>.npz, written right
    after the window's LiDAR odometry (t_lo_chain; the sweeps are then in memory) - same arrays."""
    cache = ws.lo("tedge", seg)
    if cache.exists():
        z = np.load(cache)
        st, P, kind, rfg = z["start"], z["P"], z["kind"], z["rfg"]      # each member read once
        return {int(h): (P[st[i]:st[i + 1]], kind[st[i]:st[i + 1]], rfg[st[i]:st[i + 1]])
                for i, h in enumerate(z["hdr"])}
    a_, b_ = tr.span()
    from ..lo.lotraj import read_sweep
    sw = {}
    for i in range(0, len(files), 2):
        s = read_sweep(files[i])
        xyz, tt, _ = organise(s)
        P, rc, kind, jump, rfg = depth_edges(xyz)
        if not len(P):
            continue
        sup = support_count(rc, kind)
        m = sup >= 1
        P, rc, kind, rfg = P[m], rc[m], kind[m], rfg[m]
        tp = hdr[i] + np.rint(np.nan_to_num(tt[rc[:, 0], rc[:, 1]]) * 1e9).astype(np.int64)
        tp = np.clip(tp, a_, b_)
        q = np.rint(tp / 20_000).astype(np.int64)
        uq, inv = np.unique(q, return_inverse=True)
        R, p = tr.R(uq * 20_000), tr.p(uq * 20_000)
        Pw = np.einsum("nij,nj->ni", R[inv], P) + p[inv]
        sw[int(hdr[i])] = (Pw.astype(np.float32), kind.astype(np.int8), rfg.astype(np.float32))
    return sw


def save_sweep_edges(ws, seg):
    """Compute and cache sweep_edges for a window (called at the end of its LO chain)."""
    from ..workspace import atomic_savez
    tr = LOTraj.load(ws.lo("ref", seg))
    a_, b_ = tr.span()
    files = sorted((ws.seg_dir(seg) / "lidar").glob("*.npy"))
    hdr = np.array([int(f.stem) for f in files], np.int64)
    ok = (hdr >= a_ - 100_000_000) & (hdr + 100_000_000 <= b_)
    files, hdr = [f for f, k in zip(files, ok) if k], hdr[ok]
    sw = sweep_edges(ws, seg, tr, files, hdr)
    ks = sorted(sw)
    st = np.cumsum([0] + [len(sw[k][0]) for k in ks])
    cat = lambda j, dt, sh: np.concatenate([sw[k][j] for k in ks]) if ks else np.zeros(sh, dt)  # noqa: E731
    atomic_savez(ws.lo("tedge", seg), hdr=np.array(ks, np.int64), start=st, P=cat(0, np.float32, (0, 3)),
                 kind=cat(1, np.int8, 0), rfg=cat(2, np.float32, 0))
    return len(ks)


def _zbuffer_np(MP, a, b, pw, Rw, T_cl, K):
    M = MP[a:b].astype(np.float64)
    Mc = ((M - pw) @ Rw) @ T_cl[:3, :3].T + T_cl[:3, 3]
    im = (Mc[:, 2] > 0.8) & (np.abs(Mc[:, 0]) < 0.7 * Mc[:, 2]) & (np.abs(Mc[:, 1]) < 0.55 * Mc[:, 2])
    Mc = Mc[im]
    uvm = proj(Mc, K)
    zb = np.full((H // 8 + 1, W // 8 + 1), np.inf)
    ia = (uvm[:, 0] >= 0) & (uvm[:, 0] < W) & (uvm[:, 1] >= 0) & (uvm[:, 1] < H)
    np.minimum.at(zb, ((uvm[ia, 1] // 8).astype(int), (uvm[ia, 0] // 8).astype(int)), Mc[ia, 2])
    return zb


try:
    os.environ.setdefault("NUMBA_THREADING_LAYER", "omp")
    import numba as _nb

    @_nb.njit(cache=True)
    def _zbuffer_nb(MP, a, b, pw, Rw, R2, t2, K, W_, H_):
        fx, fy, cx, cy, k1, k2 = K[0], K[1], K[2], K[3], K[4], K[5]
        zb = np.full((H_ // 8 + 1, W_ // 8 + 1), np.inf)
        for i in range(a, b):
            d0, d1, d2 = MP[i, 0] - pw[0], MP[i, 1] - pw[1], MP[i, 2] - pw[2]
            l0 = d0 * Rw[0, 0] + d1 * Rw[1, 0] + d2 * Rw[2, 0]
            l1 = d0 * Rw[0, 1] + d1 * Rw[1, 1] + d2 * Rw[2, 1]
            l2 = d0 * Rw[0, 2] + d1 * Rw[1, 2] + d2 * Rw[2, 2]
            x = l0 * R2[0, 0] + l1 * R2[0, 1] + l2 * R2[0, 2] + t2[0]
            y = l0 * R2[1, 0] + l1 * R2[1, 1] + l2 * R2[1, 2] + t2[1]
            z = l0 * R2[2, 0] + l1 * R2[2, 1] + l2 * R2[2, 2] + t2[2]
            if not (z > 0.8 and abs(x) < 0.7 * z and abs(y) < 0.55 * z):
                continue
            xn, yn = x / z, y / z
            r2 = xn * xn + yn * yn
            dd = 1 + k1 * r2 + k2 * r2 * r2
            u = fx * xn * dd + cx
            v = fy * yn * dd + cy
            if u >= 0 and u < W_ and v >= 0 and v < H_:
                r, c = int(math.floor(v / 8)), int(math.floor(u / 8))
                if z < zb[r, c]:
                    zb[r, c] = z
        return zb
    _ZB = _zbuffer_nb
except Exception:  # noqa  (no numba)
    _ZB = False


def zbuffer(MP, a, b, pw, Rw, T_cl, K):
    """Nearest map depth per 8x8 pixel cell of the frame (the map points of the history [a, b)). Compiled
    with numba when available: the same formulas point by point in one pass, no temporaries (differs from
    the numpy version only by the rounding of the 3x3 products)."""
    if _ZB is False:
        return _zbuffer_np(MP, a, b, pw, Rw, T_cl, K)
    return _ZB(MP, a, b, np.ascontiguousarray(pw, np.float64), np.ascontiguousarray(Rw, np.float64),
               np.ascontiguousarray(T_cl[:3, :3], np.float64), np.ascontiguousarray(T_cl[:3, 3], np.float64),
               np.asarray(K, np.float64), W, H)
