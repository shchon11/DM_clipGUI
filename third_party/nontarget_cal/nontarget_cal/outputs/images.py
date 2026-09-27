"""LiDAR -> image projection images (after /hdd/DM_calib/deliverables/make_rgb_deliverables.py,
make_driving_images.py, make_thermal_deliverables.py).

parked : the car stands still: one sweep projected as measured (no trajectory involved).
driving: fastest moment, sharpest turn, moderate speed (from the LiDAR odometry): every point is moved
         to the world at its own measurement time (header + t) with the LO, then into the LiDAR frame
         at the image's effective time (RGB: header + exposure/2; thermal: t_frame + dt + rs*(v/480-0.5),
         two passes for the row), then projected. Colour = range (red near -> blue 40 m).
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from ..lo.lotraj import LOTraj


def _draw(img, uv, r, label, rmax=40.0, radius=2):
    vis = img.copy()
    if len(r):
        col = cv2.applyColorMap(np.uint8(255 * (1 - np.clip(r, 0, rmax) / rmax)).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
        o = np.argsort(-r)
        for (u, v), c in zip(uv[o].astype(int), col[o]):
            cv2.circle(vis, (int(u), int(v)), radius, tuple(int(x) for x in c), -1)
    s = max(0.6, img.shape[1] / 1400)
    cv2.putText(vis, label, (14, int(40 * s)), 0, s, (255, 255, 255), 4)
    cv2.putText(vis, label, (14, int(40 * s)), 0, s, (0, 0, 200), 2)
    return vis


def _visible(uv, r, cell=6):
    cellk = (uv[:, 1] // cell).astype(np.int64) * 100000 + (uv[:, 0] // cell).astype(np.int64)
    zmin = {}
    for c_, rr in zip(cellk, r):
        if rr < zmin.get(c_, 1e9):
            zmin[c_] = rr
    return r <= np.array([zmin[c_] for c_ in cellk]) * 1.05 + 0.3


def _mask(masks_dir, cam, uv, W, H):
    mk = cv2.imread(str(Path(masks_dir) / f"{cam}.png"), 0)
    if mk is None or mk.shape != (H, W):
        return np.ones(len(uv), bool)
    return mk[np.clip(uv[:, 1].astype(int), 0, H - 1), np.clip(uv[:, 0].astype(int), 0, W - 1)] > 127


def proj_rgb(Pc, v, W, H):
    fx, fy, cx, cy, k1, k2, k3, k4 = v["intr_kb"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]); D = np.array([k1, k2, k3, k4])
    th = np.degrees(np.arctan2(np.linalg.norm(Pc[:, :2], axis=1), Pc[:, 2]))
    m = (Pc[:, 2] > 0.2) & (th < 70)
    uv = np.full((len(Pc), 2), -1.0)
    if m.any():
        uv[m] = cv2.fisheye.projectPoints(Pc[m].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    ok = m & (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    return uv, ok


def proj_th(Pc, v, W, H):
    fx, fy, cx, cy, k1, k2 = v["intr"]
    m = (Pc[:, 2] > 0.3) & (np.abs(Pc[:, 0]) < 0.7 * Pc[:, 2]) & (np.abs(Pc[:, 1]) < 0.55 * Pc[:, 2])
    uv = np.full((len(Pc), 2), -1.0)
    x, y = Pc[m, 0] / Pc[m, 2], Pc[m, 1] / Pc[m, 2]
    r2 = x * x + y * y
    d = 1 + k1 * r2 + k2 * r2 * r2
    uv[m] = np.stack([fx * x * d + cx, fy * y * d + cy], -1)
    ok = m & (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    return uv, ok


def thermal8(raw):
    raw = raw.astype(np.float32)
    lo, hi = np.percentile(raw, [1.0, 99.5])
    g = (np.clip((raw - lo) / max(hi - lo, 1.0), 0, 1) * 255).astype(np.uint8)
    g = cv2.createCLAHE(2.0, (8, 8)).apply(g)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)


def _tile(tiles, per_row=4, size=(640, 400)):
    if not tiles:
        return None
    t = [cv2.resize(x, size) for x in tiles]
    while len(t) % per_row:
        t.append(np.zeros_like(t[0]))
    return np.vstack([np.hstack(t[i:i + per_row]) for i in range(0, len(t), per_row)])


def _pts(a):
    P = np.stack([a["x"], a["y"], a["z"]], -1).astype(np.float64)
    ok = np.isfinite(P).all(1) & (np.linalg.norm(P, axis=1) > 1.0)
    return P, ok


def make_images(ws, cfg, rgb_result, thermal_result, parked, moving, outdir: Path, masks_dir: Path, log=print):
    rgb = json.loads(Path(rgb_result).read_text()) if rgb_result else None
    th = json.loads(Path(thermal_result).read_text()) if thermal_result else None
    Wr, Hr = cfg["cameras"]["rgb_size"]
    Wt, Ht = cfg["cameras"]["thermal_size"]
    rmax = cfg["outputs"]["images"]["max_range_m"]
    made = []
    # ---------------- parked
    for pw in parked:
        d = ws.seg_dir(pw)
        sw = sorted((d / "lidar").glob("*.npy"))
        if not sw:
            continue
        a = np.load(sw[len(sw) // 2])
        P, ok = _pts(a)
        P = P[ok]
        for kind, res in (("rgb", rgb), ("thermal", th)):
            if not res:
                continue
            tiles = []
            for c, v in res["cameras"].items():
                fs = sorted((d / "cam" / c).glob("*.jpg" if kind == "rgb" else "*.png"))
                if not fs:
                    continue
                T = np.array(v["T_cam_lidar"])
                Pc = P @ T[:3, :3].T + T[:3, 3]
                if kind == "rgb":
                    img = cv2.imread(str(fs[0]))
                    if img.mean() < 60:
                        img = cv2.LUT(img, np.array([((i / 255.0) ** 0.55) * 255 for i in range(256)], np.uint8))
                    uv, m = proj_rgb(Pc, v, Wr, Hr)
                    W, H = Wr, Hr
                else:
                    img = thermal8(cv2.imread(str(fs[0]), cv2.IMREAD_UNCHANGED))
                    uv, m = proj_th(Pc, v, Wt, Ht)
                    W, H = Wt, Ht
                uv, r = uv[m], np.linalg.norm(P[m], axis=1)
                keep = _visible(uv, r) & _mask(masks_dir, c, uv, W, H)
                if kind == "thermal":
                    img = cv2.resize(img, (2 * W, 2 * H), interpolation=cv2.INTER_CUBIC)
                    uv = uv * 2
                vis = _draw(img, uv[keep], r[keep], f"{c}  parked", rmax)
                p = outdir / "images" / "parked" / f"{c}.jpg"
                p.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(p), vis, [cv2.IMWRITE_JPEG_QUALITY, 88])
                tiles.append(vis); made.append(str(p))
            t = _tile(tiles, per_row=4 if kind == "rgb" else 2)
            if t is not None:
                cv2.imwrite(str(outdir / "images" / f"parked_{kind}_all.jpg"), t, [cv2.IMWRITE_JPEG_QUALITY, 85])
        break
    # ---------------- driving moments from the LO
    cands = []
    for s in moving:
        tp = ws.lo("ref", s)
        if not tp.exists():
            continue
        tr = LOTraj.load(tp)
        tau = tr.tau
        ts = np.arange(tau[2] + int(5e8), tau[-3] - int(5e8), int(2e8))
        if len(ts) < 3:
            continue
        p = tr.p(ts); R = tr.R(ts)
        v = np.linalg.norm(np.gradient(p, axis=0), axis=1) / 0.2
        yaw = np.array([np.arctan2(Rk[1, 0], Rk[0, 0]) for Rk in R])
        yr = np.abs(np.gradient(np.unwrap(yaw), axis=0)) / 0.2 * 180 / np.pi
        for t_, v_, y_ in zip(ts, v, yr):
            cands.append((s, int(t_), float(v_), float(y_)))
    if not cands:
        return {"images": made}
    fast = max(cands, key=lambda c: c[2])
    turn = max(cands, key=lambda c: c[3] if c[2] > 2 else -1)
    rest = [c for c in cands if c[0] not in (fast[0], turn[0])] or cands
    mid = min(rest, key=lambda c: abs(c[2] - 6.0))
    moments = {"fastest": fast, "turning": turn, "moderate": mid}
    for name, (seg, t0, v, yr) in moments.items():
        tr = LOTraj.load(ws.lo("ref", seg))
        d = ws.seg_dir(seg)
        sweeps = sorted((d / "lidar").glob("*.npy"))
        hdr = np.array([int(f.stem) for f in sweeps])
        for kind, res in (("rgb", rgb), ("thermal", th)):
            if not res:
                continue
            tiles = []
            for c, cv_ in res["cameras"].items():
                T = np.array(cv_["T_cam_lidar"])
                if kind == "rgb":
                    fs = sorted((d / "cam" / c).glob("*.jpg"))
                    if not fs:
                        continue
                    eff = np.array([int(f.stem) for f in fs])
                    te = int(eff[np.argmin(np.abs(eff - t0))])
                    img = cv2.imread(str(d / "cam" / c / f"{te}.jpg"))
                    img = cv2.LUT(img, np.array([((i / 255.0) ** 0.55) * 255 for i in range(256)], np.uint8))
                    rs, W, H = 0.0, Wr, Hr
                    tc = float(te)
                else:
                    from ..thermal.frames import ThermalFrames
                    fr = ThermalFrames(ws.thermal16(seg), c)
                    if not len(fr.headers):
                        continue
                    from ..thermal.timemodel import Plan, t_frame
                    plan = Plan(ws)
                    hh = fr.headers
                    segs = res["segs"]
                    dts = cv_["dt_s"]
                    dt = dts[segs.index(seg)] if (isinstance(dts, list) and seg in segs and len(dts) == len(segs)) else cv_["dt_s_mean"]
                    tf = t_frame(ws, plan.bag(seg), c, hh, res.get("time_model", "smooth"))
                    k = int(np.argmin(np.abs(tf + dt * 1e9 - t0)))
                    tc = tf[k] + dt * 1e9
                    img = thermal8(fr.read(hh[k]))
                    rs, W, H = float(cv_["rs_s"]), Wt, Ht
                kk = int(np.argmin(np.abs(hdr + int(5e7) - tc)))
                Pw_all = []
                for j in (kk - 1, kk, kk + 1):
                    if j < 0 or j >= len(sweeps):
                        continue
                    a = np.load(sweeps[j])
                    Pl, ok = _pts(a)
                    tp = hdr[j] + (a["t"].astype(np.float64) * 1e9).astype(np.int64)
                    ok &= np.abs(tp - tc) < 6e7
                    Pl, tp = Pl[ok], tp[ok]
                    tp = np.clip(tp, *tr.span())
                    Pw_all.append(np.einsum("nij,nj->ni", tr.R(tp), Pl) + tr.p(tp))
                if not Pw_all:
                    continue
                Pw = np.vstack(Pw_all)
                rows = np.full(len(Pw), H / 2)
                for _ in range(2 if rs else 1):
                    tcap = np.clip(tc + rs * 1e9 * (rows / H - 0.5), *tr.span())
                    q = np.rint(tcap / 1e5).astype(np.int64)
                    uq, inv = np.unique(q, return_inverse=True)
                    Rt, pt = tr.R(uq * 1e5), tr.p(uq * 1e5)
                    PL = np.einsum("nji,nj->ni", Rt[inv], Pw - pt[inv])
                    Pc = PL @ T[:3, :3].T + T[:3, 3]
                    uv, m = proj_rgb(Pc, cv_, W, H) if kind == "rgb" else proj_th(Pc, cv_, W, H)
                    rows = np.clip(np.where(m, uv[:, 1], H / 2), 0, H - 1)
                r = np.linalg.norm(PL, axis=1)
                uv, r = uv[m], r[m]
                keep = _visible(uv, r) & _mask(masks_dir, c, uv, W, H)
                uv, r = uv[keep], r[keep]
                if kind == "thermal":
                    img = cv2.resize(img, (2 * W, 2 * H), interpolation=cv2.INTER_CUBIC)
                    uv = uv * 2
                vis = _draw(img, uv, r, f"{c}  driving {v * 3.6:.0f} km/h, yaw {yr:.0f} deg/s", rmax)
                p = outdir / "images" / "driving" / f"{name}_{c}.jpg"
                p.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(p), vis, [cv2.IMWRITE_JPEG_QUALITY, 88])
                tiles.append(vis); made.append(str(p))
            t = _tile(tiles, per_row=4 if kind == "rgb" else 2)
            if t is not None:
                cv2.imwrite(str(outdir / "images" / "driving" / f"{name}_{kind}_all.jpg"), t, [cv2.IMWRITE_JPEG_QUALITY, 85])
    (outdir / "images").mkdir(parents=True, exist_ok=True)
    (outdir / "images" / "moments.json").write_text(json.dumps(
        {n: {"window": c[0], "t_ns": c[1], "speed_kmh": round(c[2] * 3.6, 1), "yaw_rate_deg_s": round(c[3], 1)}
         for n, c in moments.items()}, indent=1))
    log(f"images: {len(made)} written")
    return {"images": made, "moments": {n: c[0] for n, c in moments.items()}}
