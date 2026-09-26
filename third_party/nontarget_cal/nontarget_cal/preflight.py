"""Preflight: fast checks of the input bags before any heavy work (metadata + light sampling only).

Per bag and in total:
  * topics present, which cameras exist (after name mapping), message counts
  * moving time, total absolute yaw rotation, number of turns, speed range
    (from /gps/odom sampled every `sample_period_s` with range queries on the timestamp index;
    /imu/data + /gps/vel as a fallback)
  * Ouster timestamp_mode from /ouster/metadata when recorded (warn if missing / not PTP), and the
    LiDAR header vs recorder time (a LiDAR not on the PTP clock is off by seconds to hours)
  * inter-camera stamp spread (PTP-triggered cameras agree to << 1 ms), camera exposure (flag e.g.
    50 us at night: black frames), thermal frame rate and exact duplicates
  * near-range structure for the thermal edge term (a few sampled sweeps)
  * runtime and disk estimate; free disk in the work directory with a safety margin
Gates (config `preflight`): below the recommended amount -> warning; below the hard minimum ->
refusal (errors.Refusal) with a message in English and Korean.
"""
from __future__ import annotations

import json
import os
import shutil
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import errors as E
from .bag import Bag, header_ns
from .events import free_gb


def _yaw_from_quat(q):
    return Rot.from_quat([q.x, q.y, q.z, q.w]).as_euler("ZYX")[0]


def sample_series(bag: Bag, cfg, names: dict, log=print) -> dict:
    """Light sampling of the whole bag: one small range query every sample_period_s."""
    tp = cfg["topics"]
    pf = cfg["preflight"]
    t0b, t1b = bag.time_range()
    step = int(pf["sample_period_s"] * 1e9)
    odom = tp["gps_odom"] if tp["gps_odom"] in bag.topics else None
    imu = tp["imu"] if tp["imu"] in bag.topics else None
    vel = tp["gps_vel"] if tp["gps_vel"] in bag.topics else None
    metas = {c: tp["rgb_meta"].format(name=ns) for c, ns in names.get("rgb", {}).items()}
    metas = {c: t for c, t in metas.items() if t in bag.topics}
    tmetas = {c: tp["thermal_meta"].format(name=ns) for c, ns in names.get("thermal", {}).items()}
    tmetas = {c: t for c, t in tmetas.items() if t in bag.topics}
    lidar_imu = "/ouster/imu" if "/ouster/imu" in bag.topics else None
    want = [t for t in [odom, imu, vel, lidar_imu] if t] + list(metas.values()) + list(tmetas.values())
    rev = {t: c for c, t in metas.items()}
    trev = {t: c for c, t in tmetas.items()}
    T, YAW, SPD = [], [], []
    spreads, expo = [], {c: [] for c in metas}
    lid_off = []
    cam_off = []
    tstamps = {c: [] for c in tmetas}
    t_q = time.time()
    k = 0
    for t in range(t0b, t1b, step):
        got = {}
        cam_h = {}
        for name, tb, data in bag.rows(want, t, t + 70_000_000):
            if name in (odom, imu, vel) and name not in got:
                got[name] = (tb, bag.deserialize(name, data))
            elif name in rev and rev[name] not in cam_h:
                m = bag.deserialize(name, data)
                cam_h[rev[name]] = header_ns(m)
                expo[rev[name]].append(float(m.exposure_time_us))
                cam_off.append((header_ns(m) - tb) * 1e-6)
            elif name == lidar_imu and "lid" not in got:
                m = bag.deserialize(name, data)
                got["lid"] = True
                lid_off.append((header_ns(m) - tb) * 1e-6)
            elif name in trev and k % 20 == 0:
                m = bag.deserialize(name, data)
                tstamps[trev[name]].append((header_ns(m), int(m.camera_frame_id)))
        k += 1
        if odom in got:
            tb, m = got[odom]
            T.append(header_ns(m)); YAW.append(_yaw_from_quat(m.pose.pose.orientation))
            SPD.append(float(np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)))
        elif imu in got:
            tb, m = got[imu]
            T.append(header_ns(m)); YAW.append(_yaw_from_quat(m.orientation))
            if vel in got:
                v = got[vel][1].twist.twist.linear
                SPD.append(float(np.hypot(v.x, v.y)))
            else:
                SPD.append(np.nan)
        if len(cam_h) >= 2:
            h = np.array(list(cam_h.values()), np.int64)
            ref = int(np.median(h))
            per = 1e9 / 30.0
            d = (h - ref + per / 2) % per - per / 2           # phase difference to the nearest trigger
            spreads.append(float((d.max() - d.min()) * 1e-6))
    T = np.array(T, np.int64)
    o = np.argsort(T)
    out = {"t": T[o], "yaw": np.unwrap(np.array(YAW)[o]) if len(YAW) else np.zeros(0), "speed": np.array(SPD)[o],
           "cam_spread_ms": np.array(spreads), "exposure_us": {c: np.array(v) for c, v in expo.items()},
           "lidar_header_minus_bag_ms": np.array(lid_off), "cam_header_minus_bag_ms": np.array(cam_off),
           "thermal_stamps": tstamps, "t_range": (t0b, t1b), "query_s": time.time() - t_q, "samples": k}
    log(f"{bag.path.name}: {k} light samples in {out['query_s']:.0f} s")
    return out


def motion_summary(s: dict, cfg) -> dict:
    pf = cfg["preflight"]
    t, yaw, spd = s["t"], s["yaw"], s["speed"]
    if len(t) < 3:
        return {"moving_s": 0.0, "rotation_deg": 0.0, "turns": 0, "speed_mps": [0, 0], "duration_s": 0.0}
    dt = np.diff(t) * 1e-9
    moving = spd[1:] > pf["moving_speed_mps"]
    dyaw = np.degrees(np.diff(yaw))
    rot = float(np.abs(dyaw[moving]).sum())
    rate = dyaw / np.maximum(dt, 1e-3)
    turning = np.abs(rate) > pf["turn_rate_deg_s"]
    turns, acc = 0, 0.0
    for tr, dy in zip(turning, dyaw):
        if tr:
            acc += dy
        else:
            if abs(acc) >= pf["turn_min_deg"]:
                turns += 1
            acc = 0.0
    if abs(acc) >= pf["turn_min_deg"]:
        turns += 1
    sm = spd[np.isfinite(spd)]
    return {"duration_s": float((t[-1] - t[0]) * 1e-9), "moving_s": float(dt[moving].sum()),
            "rotation_deg": rot, "turns": int(turns),
            "speed_mps": [float(sm.min()) if len(sm) else 0.0, float(sm.max()) if len(sm) else 0.0],
            "speed_moving_median_mps": float(np.median(sm[sm > pf["moving_speed_mps"]])) if (sm > pf["moving_speed_mps"]).any() else 0.0}


def ouster_mode(bag: Bag, cfg) -> dict:
    tp = cfg["topics"]["lidar_metadata"]
    if tp not in bag.topics:
        return {"present": False, "timestamp_mode": None}
    r = bag.first(tp)
    try:
        meta = json.loads(r[1].data)
        mode = (meta.get("config_params", {}) or {}).get("timestamp_mode") or meta.get("timestamp_mode")
    except Exception:  # noqa
        mode = None
    return {"present": True, "timestamp_mode": mode}


def sample_images(bag: Bag, cfg, names: dict, times_ns) -> dict:
    """Mean grey level of one decoded image per camera at a few times (night detection)."""
    tp = cfg["topics"]
    out = {}
    for c, ns in names.get("rgb", {}).items():
        top = tp["rgb_image"].format(name=ns)
        if top not in bag.topics:
            continue
        g = []
        for t in times_ns:
            for _, _, data in bag.rows([top], t, t + 100_000_000):
                m = bag.deserialize(top, data)
                img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    g.append(float(img.mean()))
                break
        out[c] = float(np.median(g)) if g else None
    return out


def thermal_duplicates(bag: Bag, cfg, names: dict, times_ns, burst: int = 12) -> dict:
    tp = cfg["topics"]
    out = {}
    for c, ns in names.get("thermal", {}).items():
        top = tp["thermal_image"].format(name=ns)
        if top not in bag.topics:
            continue
        n, dup = 0, 0
        for t in times_ns:
            prev = None
            for i, (_, _, data) in enumerate(bag.rows([top], t, t + 500_000_000)):
                if i >= burst:
                    break
                m = bag.deserialize(top, data)
                f = np.frombuffer(bytes(m.data), np.uint16)
                if prev is not None:
                    n += 1
                    dup += int(np.array_equal(f, prev))
                prev = f
        out[c] = {"pairs": n, "duplicate_frac": dup / n if n else None}
    return out


def near_structure(bag: Bag, cfg, times_ns) -> float | None:
    """Share of LiDAR returns 2-10 m away horizontally and above the road (z > -1.6 m in os_lidar),
    median over a few sampled sweeps: the thermal edge term needs near-range structure."""
    from .bag import pointcloud
    tp = cfg["topics"]["lidar_points"]
    if tp not in bag.topics:
        return None
    fr = []
    for t in times_ns:
        for _, _, data in bag.rows([tp], t, t + 150_000_000):
            m = bag.deserialize(tp, data)
            a = pointcloud(m).reshape(-1)
            x, y, z = a["x"].astype(float), a["y"].astype(float), a["z"].astype(float)
            r = np.hypot(x, y)
            ok = np.isfinite(r) & (r > 0.5)
            near = ok & (r > 2) & (r < 10) & (z > -1.6)
            fr.append(near.sum() / max(ok.sum(), 1))
            break
    return float(np.median(fr)) if fr else None


def estimate(cfg, moving_s: float, sensors, n_bags: int) -> dict:
    est = cfg["preflight"]["estimate"]
    n40 = min(moving_s, cfg["windows"]["max_windows"] * cfg["windows"]["length_s"]) / 40.0
    disk = est["fixed_disk_gb"] + n40 * ((est["rgb_disk_gb"] if "rgb" in sensors else 0) +
                                         (est["thermal_disk_gb"] if "thermal" in sensors else 0))
    ncpu = os.cpu_count() or 8
    speed = min(1.0, 16.0 / ncpu) if ncpu < 16 else 1.0
    minutes = est["fixed_min"] + n40 * ((est["rgb_cpu_min"] if "rgb" in sensors else 0) +
                                        (est["thermal_cpu_min"] if "thermal" in sensors else 0))
    minutes = minutes / speed if ncpu >= 16 else minutes * 16.0 / ncpu
    return {"windows_40s_equiv": round(n40, 1), "disk_gb": round(disk, 1), "runtime_min": round(minutes),
            "cpu_count": ncpu}


def run_preflight(bags, cfg, names_per_bag: dict, sensors, workdir: Path, ev, force: bool = False,
                  data_seconds: float | None = None) -> dict:
    """names_per_bag: {bag_id: {"rgb": {cam: ns}, "thermal": {cam: ns}}}. Returns the report; raises
    Refusal for hard failures (unless force, which turns data-amount/sync refusals into warnings;
    disk is never forced)."""
    pf = cfg["preflight"]
    rep = {"bags": {}, "warnings": [], "refusals": [], "sensors": list(sensors)}

    def warn(code, msg, ko, **kw):
        rep["warnings"].append({"code": code, "msg": msg, "msg_ko": ko, **kw})
        ev.warn(code, msg, ko, **kw)

    def refuse(code, msg, ko, forceable=True, **kw):
        r = {"code": code, "msg": msg, "msg_ko": ko, "forceable": forceable, **kw}
        rep["refusals"].append(r)

    tot_moving, tot_rot, tot_turns, tot_turnwin = 0.0, 0.0, 0, 0
    for bag_id, path in bags.items():
        bag = Bag(path)
        names = names_per_bag[bag_id]
        b = {"path": str(path), "files": [str(f) for f in bag.files], "cameras_rgb": sorted(names.get("rgb", {})),
             "cameras_thermal": sorted(names.get("thermal", {}))}
        counts = bag.counts()
        tp = cfg["topics"]
        need = [tp["lidar_points"], tp["gps_fix"]]
        missing = [t for t in need if t not in bag.topics]
        if missing:
            refuse(E.MISSING_TOPIC, f"{bag_id}: required topics missing: {missing}",
                   f"{bag_id}: 필수 토픽이 없습니다: {missing}", forceable=False)
        s = sample_series(bag, cfg, names, log=ev.log)
        ms = motion_summary(s, cfg)
        b["motion"] = ms
        np.savez(workdir / "preflight" / f"series_{bag_id}.npz", t=s["t"], yaw=s["yaw"], speed=s["speed"])
        # windows the planner would make (for the thermal count)
        from .windows import candidate_windows
        cands = candidate_windows(s["t"], s["yaw"], s["speed"], cfg, zero_ns=int(s["t"][0]) if len(s["t"]) else 0)
        b["candidate_windows"] = len(cands)
        b["turning_windows"] = int(sum(c["rotation_deg"] >= cfg["windows"]["thermal_min_rotation_deg"] for c in cands))
        tot_moving += ms["moving_s"]; tot_rot += ms["rotation_deg"]; tot_turns += ms["turns"]
        tot_turnwin += b["turning_windows"]
        # sync
        om = ouster_mode(bag, cfg)
        b["ouster"] = om
        if not om["present"]:
            warn("ouster_metadata_missing", f"{bag_id}: /ouster/metadata not recorded: timestamp_mode unknown "
                 "(checked empirically from the LiDAR header vs recorder time instead)",
                 f"{bag_id}: /ouster/metadata가 녹화되지 않아 timestamp_mode를 알 수 없음(헤더-녹화시각 차이로 대신 확인)")
        elif om["timestamp_mode"] != "TIME_FROM_PTP_1588":
            warn("ouster_not_ptp", f"{bag_id}: Ouster timestamp_mode = {om['timestamp_mode']} (expected TIME_FROM_PTP_1588)",
                 f"{bag_id}: Ouster timestamp_mode가 {om['timestamp_mode']} (PTP가 아님)")
        lo = s["lidar_header_minus_bag_ms"]
        if len(lo):
            med = float(np.median(lo))
            b["lidar_header_minus_bag_ms_median"] = med
            lim = pf["sync"]["lidar_header_minus_receive_ms"]
            if not (lim[0] <= med <= lim[1]):
                refuse(E.SYNC, f"{bag_id}: LiDAR header stamps are {med / 1e3:.1f} s away from the recorder clock: "
                       "the Ouster is not on the PTP clock, LiDAR and cameras cannot be related in time",
                       f"{bag_id}: LiDAR 헤더 시각이 녹화 시각과 {med / 1e3:.1f} s 차이 남 — Ouster가 PTP 시계가 아님. "
                       "LiDAR와 카메라 시각을 맞출 수 없음", forceable=False)
        sp = s["cam_spread_ms"]
        if len(sp):
            b["camera_stamp_spread_ms"] = {"median": float(np.median(sp)), "p95": float(np.percentile(sp, 95)),
                                           "max": float(sp.max())}
            if np.median(sp) > pf["sync"]["max_camera_spread_ms"]:
                refuse(E.SYNC, f"{bag_id}: simultaneous camera stamps disagree by {np.median(sp):.1f} ms (median): "
                       "the cameras are not PTP-synchronised/triggered",
                       f"{bag_id}: 같은 순간 카메라 시각 차이가 {np.median(sp):.1f} ms(중앙값) — 카메라 PTP 동기가 깨짐")
        # exposure + night
        mid = [int(s["t_range"][0] + f * (s["t_range"][1] - s["t_range"][0])) for f in (0.3, 0.5, 0.7)]
        gray = sample_images(bag, cfg, names, mid)
        b["exposure_us_median"] = {c: (float(np.median(v)) if len(v) else None) for c, v in s["exposure_us"].items()}
        b["image_mean_gray"] = gray
        bad = []
        for c, e in b["exposure_us_median"].items():
            if e is None:
                continue
            night = gray.get(c) is not None and gray[c] < pf["exposure"]["night_mean_gray"]
            if (night and e < pf["exposure"]["min_us_night"]) or e > pf["exposure"]["max_us"]:
                bad.append((c, e, gray.get(c)))
        if bad:
            refuse(E.EXPOSURE, f"{bag_id}: absurd exposure: " + ", ".join(f"{c} {e:.0f} us (grey {g})" for c, e, g in bad),
                   f"{bag_id}: 노출이 비정상: " + ", ".join(f"{c} {e:.0f} us (밝기 {g})" for c, e, g in bad) +
                   " — 야간에 수십 us면 영상이 검게 나와 특징점이 없음. 노출(auto)을 켜고 다시 수집하세요")
        # thermal stream
        if "thermal" in sensors and names.get("thermal"):
            th = {}
            dur = (s["t_range"][1] - s["t_range"][0]) * 1e-9
            for c, ns in names["thermal"].items():
                top = tp["thermal_image"].format(name=ns)
                th[c] = {"fps": counts.get(top, 0) / dur if dur > 0 else None}
                st = s["thermal_stamps"].get(c) or []
                if len(st) > 3:
                    st = np.array(st)
                    th[c]["frame_id_per_s"] = float(np.median(np.diff(st[:, 1]) / (np.diff(st[:, 0]) * 1e-9)))
            dup = thermal_duplicates(bag, cfg, names, [int(s["t_range"][0] + f * (s["t_range"][1] - s["t_range"][0]))
                                                       for f in np.linspace(0.1, 0.9, 8)])
            for c in th:
                th[c].update(dup.get(c, {}))
                if th[c]["fps"] is not None and th[c]["fps"] < pf["thermal_stream"]["min_fps"]:
                    warn("thermal_fps", f"{bag_id} {c}: {th[c]['fps']:.1f} frames/s recorded (< {pf['thermal_stream']['min_fps']})",
                         f"{bag_id} {c}: 열화상 {th[c]['fps']:.1f} fps만 녹화됨")
                if th[c].get("duplicate_frac") is not None and th[c]["duplicate_frac"] > pf["thermal_stream"]["max_duplicate_frac"]:
                    warn("thermal_duplicates", f"{bag_id} {c}: {100 * th[c]['duplicate_frac']:.0f}% exact duplicate frames",
                         f"{bag_id} {c}: 열화상 동일 프레임 중복 {100 * th[c]['duplicate_frac']:.0f}%")
            b["thermal_stream"] = th
            ns_ = near_structure(bag, cfg, [int(s["t_range"][0] + f * (s["t_range"][1] - s["t_range"][0]))
                                           for f in np.linspace(0.1, 0.9, 9)])
            b["near_structure_frac"] = ns_
        b["counts"] = {t: counts.get(t) for t in sorted(counts) if not t.endswith("camera_info")}
        bag.close()
        rep["bags"][bag_id] = b
        ev.log(f"{bag_id}: moving {ms['moving_s']:.0f} s, rotation {ms['rotation_deg']:.0f} deg, {ms['turns']} turns, "
               f"speed {ms['speed_mps'][0]:.1f}-{ms['speed_mps'][1]:.1f} m/s, turning windows {b['turning_windows']}")
    # ---- totals and gates
    rep["total"] = {"moving_s": tot_moving, "rotation_deg": tot_rot, "turns": tot_turns, "turning_windows": tot_turnwin}
    if "rgb" in sensors:
        g = pf["rgb"]
        if tot_moving < g["min_moving_s"]:
            refuse(E.NOT_ENOUGH_MOTION, f"only {tot_moving:.0f} s of moving data (hard minimum {g['min_moving_s']} s, "
                   f"recommended >= {g['recommend_moving_s']} s)",
                   f"움직인 데이터가 {tot_moving:.0f} s뿐임(최소 {g['min_moving_s']} s, 권장 {g['recommend_moving_s']} s 이상)")
        elif tot_moving < g["recommend_moving_s"]:
            warn("little_motion", f"{tot_moving:.0f} s of moving data (< recommended {g['recommend_moving_s']} s): "
                 "expect along-axis 1-sigma of ~2-3 cm", f"움직인 데이터 {tot_moving:.0f} s(권장 {g['recommend_moving_s']} s 미만): 광축 방향 오차 약 2-3 cm 예상")
        if tot_rot < g["min_rotation_deg"]:
            refuse(E.NOT_ENOUGH_ROTATION, f"total rotation {tot_rot:.0f} deg (hard minimum {g['min_rotation_deg']}, "
                   f"recommended >= {g['recommend_rotation_deg']}): the lever arms are not observable without turns",
                   f"누적 회전 {tot_rot:.0f}°(최소 {g['min_rotation_deg']}°, 권장 {g['recommend_rotation_deg']}° 이상) — 회전 없이는 위치(레버암)를 알 수 없음")
        elif tot_rot < g["recommend_rotation_deg"]:
            warn("little_rotation", f"total rotation {tot_rot:.0f} deg (< recommended {g['recommend_rotation_deg']}): "
                 f"position error ~ 1/sqrt(rotation), expect ~{21 * np.sqrt(250 / max(tot_rot, 1)):.0f} mm",
                 f"누적 회전 {tot_rot:.0f}°(권장 {g['recommend_rotation_deg']}° 미만): 위치 오차 약 {21 * np.sqrt(250 / max(tot_rot, 1)):.0f} mm 예상")
    if "thermal" in sensors:
        g = pf["thermal"]
        if tot_turnwin < g["min_turning_windows"]:
            refuse(E.NOT_ENOUGH_THERMAL, f"only {tot_turnwin} turning windows (>= {cfg['windows']['thermal_min_rotation_deg']:.0f} deg each) "
                   f"for the thermal cameras (hard minimum {g['min_turning_windows']}, recommended {g['recommend_turning_windows']}, "
                   f"~{g['recommend_moving_s'] / 60:.0f} min with near-range structure)",
                   f"열화상용 회전 구간이 {tot_turnwin}개뿐임(구간당 {cfg['windows']['thermal_min_rotation_deg']:.0f}° 이상; 최소 {g['min_turning_windows']}개, "
                   f"권장 {g['recommend_turning_windows']}개, 근거리 구조물이 있는 곳에서 약 {g['recommend_moving_s'] / 60:.0f}분)")
        elif tot_turnwin < g["recommend_turning_windows"]:
            warn("few_thermal_windows", f"{tot_turnwin} turning windows for the thermal cameras (< recommended {g['recommend_turning_windows']})",
                 f"열화상 회전 구간 {tot_turnwin}개(권장 {g['recommend_turning_windows']}개 미만)")
        nsf = [b.get("near_structure_frac") for b in rep["bags"].values() if b.get("near_structure_frac") is not None]
        if nsf and max(nsf) < g["near_structure_frac_min"]:
            warn("no_near_structure", "little near-range structure (2-10 m) in the sampled sweeps: the thermal "
                 "along-axis position relies on it", "샘플 스윕에 2-10 m 근거리 구조물이 적음: 열화상 광축 방향 위치가 이것에 의존함")
    est = estimate(cfg, data_seconds if data_seconds is not None else tot_moving, sensors, len(bags))
    rep["estimate"] = est
    fg = free_gb(workdir)
    need = est["disk_gb"] * (1 + pf["disk_margin_frac"]) + pf["disk_margin_gb"]
    rep["disk"] = {"workdir": str(workdir), "free_gb": round(fg, 1), "needed_gb": round(need, 1)}
    if fg < need:
        refuse(E.DISK, f"not enough free disk in {workdir}: {fg:.0f} GB free, {need:.0f} GB needed "
               f"(estimate {est['disk_gb']:.0f} GB + {100 * pf['disk_margin_frac']:.0f}% + {pf['disk_margin_gb']:.0f} GB margin)",
               f"작업 폴더 {workdir}의 여유 공간 부족: {fg:.0f} GB 남음, {need:.0f} GB 필요", forceable=False)
    try:
        import torch
        rep["gpu"] = torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
    except Exception:  # noqa
        rep["gpu"] = None
    rep["ram_gb"] = round(os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 1e9, 1)
    hard = [r for r in rep["refusals"] if not (force and r["forceable"])]
    rep["ok"] = not hard
    rep["forced"] = bool(force and rep["refusals"] and not hard)
    return rep
