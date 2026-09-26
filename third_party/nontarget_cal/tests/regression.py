#!/usr/bin/env python3
"""Regression of a nontarget_cal run on the 2026-09-24 night bag against the validated results.

    python tests/regression.py --out OUT --work WORK [--json regression.json] [--md regression.md]

1. Components (same windows as the validated work, i.e. run with --windows S01:120:160,...):
   * extraction: file names and bytes of the images/sweeps vs /hdd/DM_calib/online/night/<S..>
     (or lidar_odo/night/<W..>); thermal 16-bit frames vs thermal_lo/extract/night16
   * KLT tracks vs online/night/tracks and thermal_lo/tracks (array equality)
   * refined LiDAR odometry vs lidar_odo/lo/ref_<S..>.npz (pose differences)
2. Products: per camera, the tool's T_cam_lidar / lens vs lidar_odo/final (RGB) and
   thermal_lo/thermal_lo_calib.yaml (thermal), judged against the empirical repeatability for the
   amount of data used (1/sqrt(data), lidar_odo README 3.3 / thermal_lo README):
       z = |difference| / sqrt(sigma_run^2 + sigma_ref^2);   pass if z <= 3
Nothing under /hdd/DM_calib other than the tool's own OUT/WORK is written.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

LO = Path("/hdd/DM_calib/lidar_odo")
NIGHT = Path("/hdd/DM_calib/online/night")
TH = Path("/hdd/DM_calib/thermal_lo")


def far_px(Ta_cl, Tb_cl, proj_a, unproj_a, proj_b, W, H, n=9):
    """Pixel effect of the rotation + lens difference: a grid of pixels of calibration A is unprojected,
    taken to infinity through A's rotation into the LiDAR frame and projected with B. Median and max
    distance in px (translation plays no role at infinity; this is what the principal-point <->
    rotation trade leaves in the image)."""
    u, v = np.meshgrid(np.linspace(0.1 * W, 0.9 * W, n), np.linspace(0.1 * H, 0.9 * H, n))
    uv = np.stack([u.ravel(), v.ravel()], 1)
    ray_a = unproj_a(uv)                                  # camera A frame
    ray_l = ray_a @ Ta_cl[:3, :3]                         # R_CL^T x  (LiDAR frame)
    ray_b = ray_l @ Tb_cl[:3, :3].T
    uvb = proj_b(ray_b)
    d = np.linalg.norm(uvb - uv, axis=1)
    return float(np.median(d)), float(d.max())


def kb_fns(intr):
    import cv2
    fx, fy, cx, cy, k1, k2, k3, k4 = intr
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]); D = np.array([k1, k2, k3, k4], float)
    def un(uv):
        n = cv2.fisheye.undistortPoints(uv.reshape(-1, 1, 2).astype(np.float64), K, D).reshape(-1, 2)
        r = np.c_[n, np.ones(len(n))]
        return r / np.linalg.norm(r, axis=1, keepdims=True)
    def pr(X):
        return cv2.fisheye.projectPoints(X.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    return pr, un


def pinhole_fns(K, D):
    import cv2
    Km = np.array(K, float); Dm = np.array(list(D)[:2] + [0, 0, 0], float)
    def un(uv):
        n = cv2.undistortPoints(uv.reshape(-1, 1, 2).astype(np.float64), Km, Dm).reshape(-1, 2)
        r = np.c_[n, np.ones(len(n))]
        return r / np.linalg.norm(r, axis=1, keepdims=True)
    def pr(X):
        return cv2.projectPoints(X.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), Km, Dm)[0].reshape(-1, 2)
    return pr, un


def old_seg_dir(seg):
    return (LO / "night" / seg) if seg.startswith("W") else (NIGHT / seg)


def cmp_extract(work, seg, cams, n_bytes=40):
    new, old = work / "extract" / seg, old_seg_dir(seg)
    out = {}
    for c in cams:
        a = sorted(p.name for p in (new / "cam" / c).glob("*.jpg"))
        b = sorted(p.name for p in (old / "cam" / c).glob("*.jpg"))
        same_names = a == b
        pick = a[:: max(1, len(a) // n_bytes)][:n_bytes] if same_names else []
        same_bytes = all((new / "cam" / c / f).read_bytes() == (old / "cam" / c / f).read_bytes() for f in pick)
        out[c] = {"n_new": len(a), "n_old": len(b), "same_names": same_names, "same_bytes_sampled": same_bytes}
    a = sorted(p.name for p in (new / "lidar").glob("*.npy"))
    b = sorted(p.name for p in (old / "lidar").glob("*.npy"))
    pick = a[:: max(1, len(a) // 10)][:10] if a == b else []
    same = all(np.array_equal(np.load(new / "lidar" / f), np.load(old / "lidar" / f)) for f in pick)
    out["lidar"] = {"n_new": len(a), "n_old": len(b), "same_names": a == b, "same_arrays_sampled": same}
    return out


def cmp_thermal_frames(work, seg, cams, n=20):
    out = {}
    for c in cams:
        a = sorted(p.name for p in (work / "thermal16" / seg / c).glob("*.png"))
        old = TH / "extract" / "night16" / c
        present = [f for f in a if (old / f).exists()]
        pick = present[:: max(1, len(present) // n)][:n]
        import cv2
        same = all(np.array_equal(cv2.imread(str(work / "thermal16" / seg / c / f), -1), cv2.imread(str(old / f), -1)) for f in pick)
        out[c] = {"n_new": len(a), "n_in_old_extract": len(present), "same_pixels_sampled": same}
    return out


def cmp_tracks(work, seg, cams, thermal=False):
    out = {}
    for c in cams:
        if thermal:
            a, b = work / "thermal" / "tracks" / f"tracks_{seg}_{c}.npz", TH / "tracks" / f"tracks_{seg}_{c}.npz"
            keys = ("header_ns", "obs_frame", "obs_track", "obs_xy")
        else:
            a, b = work / "tracks" / f"tracks_{seg}_{c}.npz", ((LO / "night" / "tracks") if seg.startswith("W") else (NIGHT / "tracks")) / f"tracks_{seg}_{c}.npz"
            keys = ("frame_ns", "obs_frame", "obs_track", "obs_xy")
        if not (a.exists() and b.exists()):
            out[c] = None
            continue
        za, zb = np.load(a), np.load(b)
        same = all(np.array_equal(za[k], zb[k]) for k in keys)
        rec = {"identical": bool(same), "n_obs_new": int(len(za["obs_frame"])), "n_obs_old": int(len(zb["obs_frame"]))}
        if not same:
            ta, tb = set(np.unique(za["obs_track"]).tolist()), set(np.unique(zb["obs_track"]).tolist())
            rec["n_tracks_new"], rec["n_tracks_old"] = len(ta), len(tb)
        out[c] = rec
    return out


def cmp_lo(work, seg):
    from nontarget_cal.lo.lotraj import LOTraj
    a, b = LOTraj.load(work / "lo" / f"ref_{seg}.npz"), LOTraj.load(LO / "lo" / f"ref_{seg}.npz")
    n = min(len(a.tau), len(b.tau))
    same_tau = bool(np.array_equal(a.tau[:n], b.tau[:n]))
    # compare relative motion over 1 s (the world frames are both the first knot)
    dp = np.linalg.norm(a.T[:n, :3, 3] - b.T[:n, :3, 3], axis=1)
    dr = np.degrees(Rot.from_matrix(np.einsum("nji,njk->nik", a.T[:n, :3, :3], b.T[:n, :3, :3])).magnitude())
    return {"knots": n, "same_knot_times": same_tau, "pos_diff_mm_median": float(1e3 * np.median(dp)),
            "pos_diff_mm_max": float(1e3 * dp.max()), "rot_diff_deg_max": float(dr.max())}


def rgb_products(out_dir, seconds_run):
    ref_dir = LO / "final"
    R_L_V = np.array(json.loads((LO / "vehicle_axes.json").read_text())["R_L_V"])
    f = np.sqrt(40.0 / max(seconds_run, 1.0))
    # empirical 1-sigma of the run (lidar_odo 3.3: 1 window 17-33 mm along the axis, 0.06-0.18 deg, f 1.4-1.9 px)
    s_run = {"axis_mm": max(25.0 * f, 10.0), "pos_mm": max(35.0 * f, 14.0), "rot_deg": max(0.15 * f, 0.06), "f_px": max(1.7 * f, 0.9)}
    # the product: halves of 450 s differ by 10 mm (axis) / 18 mm (pos) / 0.06 deg / 0.94 px -> sigma ~ diff/sqrt(2)/sqrt(2)
    s_ref = {"axis_mm": 5.0, "pos_mm": 9.0, "rot_deg": 0.03, "f_px": 0.5}
    rows = {}
    for p in sorted((out_dir / "calib_json").glob("*.json")):
        c = p.stem
        rf = ref_dir / f"{c}.json"
        if not rf.exists():
            continue
        a, b = json.loads(p.read_text()), json.loads(rf.read_text())
        Ta, Tb = np.linalg.inv(np.array(a["T_cam_lidar"])), np.linalg.inv(np.array(b["T_cam_lidar"]))
        d = Ta[:3, 3] - Tb[:3, 3]
        ax = Tb[:3, 2]
        dr = float(np.degrees(Rot.from_matrix(Ta[:3, :3].T @ Tb[:3, :3]).magnitude()))
        df = float(a["intr"][0] - b["intr"][0])
        r = {"dpos_V_mm": (1e3 * R_L_V.T @ d).round(1).tolist(), "dpos_mm": round(float(1e3 * np.linalg.norm(d)), 1),
             "daxis_mm": round(float(1e3 * d @ ax), 1), "drot_deg": round(dr, 3), "df_px": round(df, 2),
             "dcx_px": round(float(a["intr"][2] - b["intr"][2]), 1), "dcy_px": round(float(a["intr"][3] - b["intr"][3]), 1)}
        pa, ua = kb_fns(a["intr"]); pb, _ = kb_fns(b["intr"])
        r["far_px_median"], r["far_px_max"] = [round(x, 2) for x in far_px(np.array(a["T_cam_lidar"]), np.array(b["T_cam_lidar"]), pa, ua, pb, 1920, 1200)]
        z = {"axis": abs(r["daxis_mm"]) / np.hypot(s_run["axis_mm"], s_ref["axis_mm"]),
             "pos": r["dpos_mm"] / np.hypot(s_run["pos_mm"], s_ref["pos_mm"]),
             "rot": dr / np.hypot(s_run["rot_deg"], s_ref["rot_deg"]),
             "f": abs(df) / np.hypot(s_run["f_px"], s_ref["f_px"])}
        r["z"] = {k: round(float(v), 2) for k, v in z.items()}
        r["pass"] = bool(max(z.values()) <= 3.0)
        rows[c] = r
    return {"sigma_run": s_run, "sigma_ref": s_ref, "cameras": rows,
            "n_pass": sum(r["pass"] for r in rows.values()), "n": len(rows)}


def thermal_products(out_dir, n_windows):
    y = yaml.safe_load((TH / "thermal_lo_calib.yaml").read_text())
    f = np.sqrt(15.0 / max(n_windows, 1))
    s_run = {"axis_mm": 15.0 * f, "lat_mm": 5.0 * f, "rot_deg": 0.05 * f, "fx_px": 1.0 * f, "dt_ms": 1.0 * f}
    s_ref = {"axis_mm": 15.0, "lat_mm": 5.0, "rot_deg": 0.05, "fx_px": 1.0, "dt_ms": 1.0}
    rows = {}
    for c in ("thermal_left", "thermal_right"):
        e = out_dir / "extrinsic" / f"{c}.yaml"
        i = out_dir / "intrinsic" / f"{c}.yaml"
        if not e.exists():
            continue
        E, I = yaml.safe_load(e.read_text()), yaml.safe_load(i.read_text())
        ref = y["cameras"][c]
        Ta, Tb = np.linalg.inv(np.array(E["T_cam_lidar"])), np.linalg.inv(np.array(ref["T_cam_lidar"]))
        d = Ta[:3, 3] - Tb[:3, 3]
        dc = Tb[:3, :3].T @ d                  # in the camera frame: x lateral, y vertical, z along the axis
        dr = float(np.degrees(Rot.from_matrix(Ta[:3, :3].T @ Tb[:3, :3]).magnitude()))
        fx = float(np.array(I["camera_matrix"])[0, 0])
        fx_ref = float(np.array(ref["intrinsics"]["camera_matrix"])[0, 0])
        ddt = 1e3 * (float(E["time_offset_s"]) - float(ref["time"]["time_offset_s"]))
        r = {"d_cam_frame_mm": (1e3 * dc).round(1).tolist(), "daxis_mm": round(float(1e3 * dc[2]), 1),
             "drot_deg": round(dr, 3), "dfx_px": round(fx - fx_ref, 2), "d_time_offset_ms": round(ddt, 2)}
        pa, ua = pinhole_fns(np.array(I["camera_matrix"]), I["distortion_coefficients"])
        pb, _ = pinhole_fns(np.array(ref["intrinsics"]["camera_matrix"]), ref["intrinsics"]["distortion_coefficients"])
        r["far_px_median"], r["far_px_max"] = [round(x, 2) for x in far_px(np.array(E["T_cam_lidar"]), np.array(ref["T_cam_lidar"]), pa, ua, pb, 640, 480)]
        r["dcx_dcy_px"] = [round(float(np.array(I["camera_matrix"])[0, 2] - np.array(ref["intrinsics"]["camera_matrix"])[0, 2]), 1),
                           round(float(np.array(I["camera_matrix"])[1, 2] - np.array(ref["intrinsics"]["camera_matrix"])[1, 2]), 1)]
        z = {"axis": abs(1e3 * dc[2]) / np.hypot(s_run["axis_mm"], s_ref["axis_mm"]),
             "lateral": abs(1e3 * dc[0]) / np.hypot(s_run["lat_mm"], s_ref["lat_mm"]),
             "vertical": abs(1e3 * dc[1]) / np.hypot(s_run["lat_mm"], s_ref["lat_mm"]),
             # rotation judged through its image effect: the principal point and the rotation trade
             # (3 px of cx ~ 0.25 deg at f = 680 px); 1 px at infinity ~ 0.084 deg
             "rot_px": r["far_px_median"] / np.hypot(s_run["rot_deg"] * 680 * np.pi / 180, s_ref["rot_deg"] * 680 * np.pi / 180),
             "fx": abs(fx - fx_ref) / np.hypot(s_run["fx_px"], s_ref["fx_px"]),
             "dt": abs(ddt) / np.hypot(s_run["dt_ms"], s_ref["dt_ms"])}
        r["z"] = {k: round(float(v), 2) for k, v in z.items()}
        r["pass"] = bool(max(z.values()) <= 3.0)
        rows[c] = r
    return {"sigma_run": s_run, "sigma_ref": s_ref, "cameras": rows}


def to_md(R):
    L = ["# Regression vs the validated 2026-09-24 results\n"]
    if R.get("components"):
        L.append("## Components (same windows)\n")
        for seg, v in R["components"].items():
            ex = v["extract"]
            cam_ok = all(x["same_names"] and x["same_bytes_sampled"] for k, x in ex.items() if k != "lidar")
            L.append(f"- {seg}: images identical {cam_ok}; sweeps identical {ex['lidar']['same_names'] and ex['lidar']['same_arrays_sampled']}; "
                     f"RGB tracks identical {sum(1 for x in v['tracks'].values() if x and x['identical'])}/{len(v['tracks'])}; "
                     f"thermal frames {v.get('thermal_frames')}; thermal tracks identical "
                     f"{sum(1 for x in v.get('thermal_tracks', {}).values() if x and x['identical'])}/{len(v.get('thermal_tracks', {}))}; "
                     f"LO vs ref: median {v['lo']['pos_diff_mm_median']:.2f} mm, max {v['lo']['pos_diff_mm_max']:.1f} mm, rot max {v['lo']['rot_diff_deg_max']:.4f} deg")
    rg = R.get("rgb")
    if rg:
        L.append(f"\n## RGB vs lidar_odo/final ({rg['n_pass']}/{rg['n']} within 3 sigma)\n")
        L.append(f"sigma_run {rg['sigma_run']}, sigma_ref {rg['sigma_ref']}\n")
        L.append("| camera | dpos V fwd/left/up mm | |dpos| mm | d along-axis mm | drot deg | df px | dcx/dcy px | far-field px (med/max) | max z | pass |")
        L.append("|---|---|---|---|---|---|---|---|---|---|")
        for c, r in rg["cameras"].items():
            L.append(f"| {c} | {r['dpos_V_mm']} | {r['dpos_mm']} | {r['daxis_mm']:+} | {r['drot_deg']} | {r['df_px']:+} | "
                     f"{r['dcx_px']:+}/{r['dcy_px']:+} | {r['far_px_median']}/{r['far_px_max']} | {max(r['z'].values()):.2f} | {r['pass']} |")
    th = R.get("thermal")
    if th:
        L.append("\n## Thermal vs thermal_lo/thermal_lo_calib.yaml\n")
        L.append(f"sigma_run {th['sigma_run']}, sigma_ref {th['sigma_ref']}\n")
        L.append("| camera | d (cam x/y/z) mm | drot deg | dcx/dcy px | far-field px (med/max) | dfx px | d time offset ms | max z | pass |")
        L.append("|---|---|---|---|---|---|---|---|---|")
        for c, r in th["cameras"].items():
            L.append(f"| {c} | {r['d_cam_frame_mm']} | {r['drot_deg']} | {r['dcx_dcy_px']} | {r['far_px_median']}/{r['far_px_max']} | {r['dfx_px']:+} | "
                     f"{r['d_time_offset_ms']:+} | {max(r['z'].values()):.2f} | {r['pass']} |")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--work", type=Path, required=True)
    ap.add_argument("--components", action="store_true")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--md", type=Path, default=None)
    a = ap.parse_args()
    s = json.loads((a.out / "summary.json").read_text())
    plan = json.loads((a.work / "windows" / "plan.json").read_text())
    kept = s["windows"]["kept"]
    secs = sum(w["t1_s"] - w["t0_s"] for w in plan["windows"] if w["name"] in kept)
    R = {"windows": kept, "seconds": secs}
    if a.components:
        rgb_cams = json.loads((LO / "ba/ALL/result.json").read_text())["cams"]
        R["components"] = {}
        for seg in kept:
            v = {"extract": cmp_extract(a.work, seg, rgb_cams), "tracks": cmp_tracks(a.work, seg, rgb_cams), "lo": cmp_lo(a.work, seg)}
            if (a.work / "thermal16" / seg).exists():
                v["thermal_frames"] = cmp_thermal_frames(a.work, seg, ["thermal_left", "thermal_right"])
                v["thermal_tracks"] = cmp_tracks(a.work, seg, ["thermal_left", "thermal_right"], thermal=True)
            R["components"][seg] = v
            print(seg, json.dumps(v["lo"]), flush=True)
    if (a.out / "calib_json").exists():
        R["rgb"] = rgb_products(a.out, secs)
    if (a.out / "extrinsic" / "thermal_left.yaml").exists():
        R["thermal"] = thermal_products(a.out, len(s["windows"].get("thermal") or []))
    if a.json:
        a.json.write_text(json.dumps(R, indent=1))
    md = to_md(R)
    if a.md:
        a.md.write_text(md)
    print(md)


if __name__ == "__main__":
    main()
