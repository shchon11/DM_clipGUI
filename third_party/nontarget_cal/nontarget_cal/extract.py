"""Extraction: everything the solves need is copied out of the bags once; later stages read only the
work directory.

Per bag, one pass over the small topics (`scan_bag`): the whole GNSS/INS (-> <bag>_ins.npz, same
numbers as online/extract_dm.py:build_ins), every camera metadata message (exposure, camera clock)
and every thermal metadata message (the A70 frame clock, for the thermal time model).

Per window, ONE range query on rosbag2's timestamp index (`extract_window`), filtered by topic id,
writes (layout of /hdd/DM_calib/online/extract_win.py, so the numerics downstream are unchanged):
  extract/<win>/cam/<camera>/<header + exposure/2 ns>.jpg   RGB, the recorded JPEG bytes unchanged
  extract/<win>/lidar/<header ns>.npy                        x y z intensity(=reflectivity) ring t
  extract/<win>/cam_index.npz, source.json
  thermal16/<win>/<cam>/<header ns>.png                       raw 16-bit centi-kelvin (thermal_lo layout)
  thermal16/<win>/index_<cam>.npz                             header_ns, sd, mad_prev, mean_K, ffc
The window is written under <name>.partial and renamed when complete, so an interrupted extraction
(crash, unplugged disk) leaves no half window behind.
"""
from __future__ import annotations

import json
import os
import shutil
import time
from pathlib import Path

import cv2
import numpy as np

from .bag import Bag, header_ns, sweep_from_msg
from .ins import build_ins
from .workspace import atomic_savez, write_json


def ffc_flags(sd, mad):
    """Frames to drop (verbatim thermal_lo/code/extract_thermal16.py): exact repeats (flat-field
    correction freezes and isolated duplicates), both frames of every identical pair, and the frame
    after a freeze."""
    z = np.isfinite(mad) & (mad < 1e-3)
    bad = z | np.r_[z[1:], False]
    run = np.convolve(z.astype(int), np.ones(3, int), "same") >= 2
    bad |= np.r_[False, run[:-1]] | np.r_[run[1:], False]
    return bad


def bag_zero_ns(bag: Bag, topics: dict) -> int:
    """Time zero of a bag: the first /gps/fix header stamp (as online/extract_win.py)."""
    r = bag.first(topics["gps_fix"])
    if r is None:
        raise RuntimeError(f"{bag.path}: no {topics['gps_fix']}")
    return header_ns(r[1])


def scan_bag(bag: Bag, bag_id: str, names: dict, cfg, ws, log=print) -> dict:
    """One pass over the small topics of the whole bag. names: {'rgb': {cam: ns}, 'thermal': {cam: ns}}."""
    tp = cfg["topics"]
    t0 = time.time()
    small = {tp["gps_fix"]: ("fix", None), tp["gps_vel"]: ("vel", None), tp["imu"]: ("imu", None)}
    for cam, ns in names.get("rgb", {}).items():
        small[tp["rgb_meta"].format(name=ns)] = ("meta", cam)
    for cam, ns in names.get("thermal", {}).items():
        small[tp["thermal_meta"].format(name=ns)] = ("tmeta", cam)
    small = {k: v for k, v in small.items() if k in bag.topics}
    fx, vl, im = [], [], []
    meta = {c: [] for c in names.get("rgb", {})}
    tmeta = {c: [] for c in names.get("thermal", {})}
    n = 0
    for name, _, data in bag.rows(list(small)):
        kind, cam = small[name]
        m = bag.deserialize(name, data)
        if kind == "fix":
            fx.append((header_ns(m), m.latitude, m.longitude, m.altitude, m.status.status))
        elif kind == "vel":
            vl.append((header_ns(m), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z))
        elif kind == "imu":
            im.append((header_ns(m), m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z,
                       m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                       m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        elif kind == "meta":
            meta[cam].append((header_ns(m), int(m.camera_timestamp_ns), float(m.exposure_time_us)))
        else:
            tmeta[cam].append((header_ns(m), int(m.camera_timestamp_ns), int(m.camera_frame_id)))
        n += 1
    info = {"messages": n, "scan_s": time.time() - t0}
    if fx and vl and im:
        # float64 arrays as online/extract_dm.py:build_ins (its stamps are rounded to 256 ns; kept for identity)
        info["ins"] = build_ins(np.array(fx), np.array(vl), np.array(im), ws.ins(bag_id), log=log)
    for cam, rows in meta.items():
        # int64 throughout: ns stamps (~1.8e18) do not survive a float64 round trip
        atomic_savez(ws.root / "extract" / f"{bag_id}_meta_{cam}.npz", header_ns=np.array([r[0] for r in rows], np.int64),
                     camera_timestamp_ns=np.array([r[1] for r in rows], np.int64),
                     exposure_us=np.array([r[2] for r in rows], np.float64))
    for cam, rows in tmeta.items():
        rows = sorted(rows)
        atomic_savez(ws.root / "thermal16" / f"{bag_id}_clock_{cam}.npz",
                     header_ns=np.array([r[0] for r in rows], np.int64),
                     camclock_ns=np.array([r[1] for r in rows], np.int64),
                     frame_id=np.array([r[2] for r in rows], np.int64))
    log(f"{bag_id}: scanned {n} small messages in {time.time() - t0:.0f} s")
    return info


def extract_window(bag: Bag, bag_id: str, win: dict, names: dict, cfg, ws, zero_ns: int,
                   rgb: bool = True, thermal: bool = False, lidar: bool = True, log=print) -> dict:
    tp = cfg["topics"]
    name = win["name"]
    w0, w1 = zero_ns + int(float(win["t0_s"]) * 1e9), zero_ns + int(float(win["t1_s"]) * 1e9)
    final = ws.seg_dir(name)
    part = final.with_name(name + ".partial")
    tfinal = ws.thermal16(name)
    tpart = tfinal.with_name(name + ".partial")
    for p in (part, tpart):
        if p.exists():
            shutil.rmtree(p)
    t_start = time.time()
    plan = {}
    cams = list(names["rgb"]) if rgb else []
    for c in cams:
        ns = names["rgb"][c]
        plan[tp["rgb_image"].format(name=ns)] = ("rgb", c)
    tcams = list(names.get("thermal", {})) if thermal else []
    for c in tcams:
        plan[tp["thermal_image"].format(name=names["thermal"][c])] = ("thermal", c)
    if lidar:
        plan[tp["lidar_points"]] = ("lidar", None)
    missing = [t for t in plan if t not in bag.topics]
    if missing:
        raise RuntimeError(f"{name}: topics not in the bag: {missing}")
    # exposure per (camera, header) from the full-bag metadata scan
    expo = {}
    for c in cams:
        z = np.load(ws.root / "extract" / f"{bag_id}_meta_{c}.npz")
        expo[c] = dict(zip(z["header_ns"].tolist(), zip(z["camera_timestamp_ns"].tolist(), z["exposure_us"].tolist())))
    (part / "lidar").mkdir(parents=True, exist_ok=True)
    for c in cams:
        (part / "cam" / c).mkdir(parents=True, exist_ok=True)
    for c in tcams:
        (tpart / c).mkdir(parents=True, exist_ok=True)
    idx, n_l = [], 0
    trec = {c: [] for c in tcams}
    prev = {c: None for c in tcams}
    # RGB/LiDAR: header within [w0, w1] (extract_win.py); thermal: +-0.5 s (extract_thermal16 --t0/--t1)
    lo, hi = w0 - int(5e8), w1 + int(5e8)
    lo3, hi3 = w0 - int(3e8), w1 + int(3e8)
    for topic, tbag, data in bag.rows(list(plan), lo, hi):
        kind, c = plan[topic]
        if kind != "thermal" and not (lo3 <= tbag <= hi3):
            continue                      # extract_win.py queried bag time [w0 - 0.3 s, w1 + 0.3 s]
        m = bag.deserialize(topic, data)
        h = header_ns(m)
        if kind == "thermal":
            raw = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width)
            cv2.imwrite(str(tpart / c / f"{h}.png"), raw, [cv2.IMWRITE_PNG_COMPRESSION,
                                                           int(cfg["extract"]["thermal_png_compression"])])
            f = raw.astype(np.float32)
            mad = float(np.mean(np.abs(f - prev[c]))) if prev[c] is not None else np.nan
            prev[c] = f
            trec[c].append((h, float(f.std()), mad, float(f.mean()) * 0.01))
            continue
        if not (w0 <= h <= w1):
            continue
        if kind == "lidar":
            np.save(part / "lidar" / f"{h}.npy", sweep_from_msg(m))
            n_l += 1
            continue
        camts, exp_us = expo[c].get(h, (0, float("nan")))
        if not np.isfinite(exp_us):
            continue                      # no metadata -> no reliable exposure time (extract_win.py)
        eff = h + int(round(exp_us * 500.0))
        (part / "cam" / c / f"{eff}.jpg").write_bytes(bytes(m.data))
        idx.append((c, h, camts, exp_us, eff))
    per = {c: sum(1 for r in idx if r[0] == c) for c in cams}
    if idx:
        np.savez(part / "cam_index.npz", channel=np.array([r[0] for r in idx]),
                 header_ns=np.array([r[1] for r in idx], np.int64),
                 camera_timestamp_ns=np.array([r[2] for r in idx], np.int64),
                 exposure_us=np.array([r[3] for r in idx]),
                 effective_ns=np.array([r[4] for r in idx], np.int64))
    ex = np.array([r[3] for r in idx]) if idx else np.array([np.nan])
    tinfo = {}
    for c in tcams:
        r = np.array(trec[c], dtype=np.float64).reshape(-1, 4)
        h = np.array([x[0] for x in trec[c]], np.int64)
        ffc = ffc_flags(r[:, 1], r[:, 2]) if len(r) else np.zeros(0, bool)
        np.savez(tpart / f"index_{c}.npz", header_ns=h, sd=r[:, 1], mad_prev=r[:, 2], mean_K=r[:, 3], ffc=ffc)
        tinfo[c] = {"frames": int(len(h)), "ffc_or_duplicate": int(ffc.sum())}
    src = {"bag": str(bag.path), "bag_id": bag_id, "window_s": [float(win["t0_s"]), float(win["t1_s"])],
           "window_ns": [w0, w1], "zero_ns": zero_ns, "names": names, "frames": per, "sweeps": n_l,
           "exposure_us_median": float(np.nanmedian(ex)) if np.isfinite(ex).any() else None,
           "thermal": tinfo, "wall_s": time.time() - t_start}
    (part / "source.json").write_text(json.dumps(src, indent=1))
    if tcams:
        (tpart / "source.json").write_text(json.dumps(src, indent=1))
        if tfinal.exists():
            shutil.rmtree(tfinal)
        os.replace(tpart, tfinal)
    if final.exists():
        shutil.rmtree(final)
    os.replace(part, final)
    log(f"  {name}: {win['t0_s']:.1f}-{win['t1_s']:.1f} s, {len(idx)} images "
        f"({min(per.values()) if per else 0}-{max(per.values()) if per else 0} per camera), {n_l} sweeps, "
        f"thermal {tinfo}, {time.time() - t_start:.0f} s")
    return src


def extract_standstill(bag: Bag, win: dict, names: dict, cfg, ws, zero_ns: int, log=print) -> dict:
    """A parked moment for the parked projection images: 3 sweeps and, per camera, the image nearest
    the middle sweep (RGB JPEG / thermal 16-bit PNG). Written to extract/<name>/ like a window."""
    tp = cfg["topics"]
    name = win["name"]
    final = ws.seg_dir(name)
    part = final.with_name(name + ".partial")
    if part.exists():
        shutil.rmtree(part)
    tm = zero_ns + int(0.5 * (win["t0_s"] + win["t1_s"]) * 1e9)
    plan = {tp["lidar_points"]: ("lidar", None)}
    for c, ns in names.get("rgb", {}).items():
        plan[tp["rgb_image"].format(name=ns)] = ("rgb", c)
    for c, ns in names.get("thermal", {}).items():
        plan[tp["thermal_image"].format(name=ns)] = ("thermal", c)
    plan = {k: v for k, v in plan.items() if k in bag.topics}
    (part / "lidar").mkdir(parents=True, exist_ok=True)
    best = {}
    n_l = 0
    for topic, _, data in bag.rows(list(plan), tm - int(2e8), tm + int(2e8)):
        kind, c = plan[topic]
        m = bag.deserialize(topic, data)
        h = header_ns(m)
        if kind == "lidar":
            if abs(h - tm) < 1.5e8:
                np.save(part / "lidar" / f"{h}.npy", sweep_from_msg(m))
                n_l += 1
            continue
        if c not in best or abs(h - tm) < abs(best[c][0] - tm):
            best[c] = (h, kind, bytes(m.data), getattr(m, "height", 0), getattr(m, "width", 0))
    for c, (h, kind, data, H, W) in best.items():
        (part / "cam" / c).mkdir(parents=True, exist_ok=True)
        if kind == "rgb":
            (part / "cam" / c / f"{h}.jpg").write_bytes(data)
        else:
            cv2.imwrite(str(part / "cam" / c / f"{h}.png"), np.frombuffer(data, np.uint16).reshape(H, W))
    src = {"bag": str(bag.path), "standstill": True, "t_mid_ns": tm, "sweeps": n_l, "cams": sorted(best)}
    (part / "source.json").write_text(json.dumps(src, indent=1))
    if final.exists():
        shutil.rmtree(final)
    os.replace(part, final)
    log(f"  {name}: parked moment, {n_l} sweeps, {len(best)} cameras")
    return src


def window_done(ws, name: str, thermal: bool) -> bool:
    ok = (ws.seg_dir(name) / "source.json").exists()
    if thermal:
        ok &= (ws.thermal16(name) / "source.json").exists()
    return ok


def write_plan(ws, plan):
    write_json(ws.root / "windows" / "plan.json", plan)
