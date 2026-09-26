"""Differences between calibrations (lidar_odo/compare.py) and the empirical 1-sigma table.

Per camera, A vs B: position difference in the vehicle frame V and along the camera's own optical
axis, rotation angle, focal and principal point differences.

Empirical 1-sigma from two disjoint halves (validation stage): for each scalar, sigma = |A - B| /
sqrt(2) = the scatter of a solve on HALF the data. The delivered solve uses all the data, so its
scatter is expected to be ~sqrt(2) smaller; the table quotes the half-data value (conservative),
as the lidar_odo / thermal_lo READMEs quote half-vs-half differences.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot


def cam_T_intr(res, c):
    v = res["cameras"][c]
    return np.array(v["T_lidar_cam"]), np.array(v.get("intr_kb", v.get("intr")))


def diff(ra: dict, rb: dict, R_L_V=np.eye(3)) -> dict:
    out = {}
    for c in ra["cameras"]:
        if c not in rb["cameras"]:
            continue
        (Ta, Ia), (Tb, Ib) = cam_T_intr(ra, c), cam_T_intr(rb, c)
        d = Ta[:3, 3] - Tb[:3, 3]
        ax = 0.5 * (Ta[:3, 2] + Tb[:3, 2])
        ax /= np.linalg.norm(ax)
        dr = np.degrees(Rot.from_matrix(Ta[:3, :3].T @ Tb[:3, :3]).magnitude())
        out[c] = {"dpos_V_mm": (1e3 * (R_L_V.T @ d)).round(1).tolist(), "dpos_mm": float(1e3 * np.linalg.norm(d)),
                  "daxis_mm": float(1e3 * d @ ax), "drot_deg": float(dr), "df_px": float(Ia[0] - Ib[0]),
                  "dcx_px": float(Ia[2] - Ib[2]), "dcy_px": float(Ia[3] - Ib[3])}
    return out


def sigma_from_halves(d: dict) -> dict:
    s = 1.0 / np.sqrt(2.0)
    return {c: {"rot_deg": v["drot_deg"] * s, "pos_mm": v["dpos_mm"] * s, "along_axis_mm": abs(v["daxis_mm"]) * s,
                "focal_px": abs(v["df_px"]) * s} for c, v in d.items()}


def summary(d: dict) -> dict:
    if not d:
        return {}
    return {"median_dpos_mm": float(np.median([v["dpos_mm"] for v in d.values()])),
            "max_dpos_mm": float(np.max([v["dpos_mm"] for v in d.values()])),
            "median_abs_daxis_mm": float(np.median([abs(v["daxis_mm"]) for v in d.values()])),
            "median_drot_deg": float(np.median([v["drot_deg"] for v in d.values()])),
            "median_abs_df_px": float(np.median([abs(v["df_px"]) for v in d.values()]))}


def expected_sigma(cfg, seconds: float) -> tuple[float, float]:
    """Empirical repeatability for a given amount of data (1/sqrt(data), floors): (axis mm, rot deg)."""
    p = cfg["rgb"]["perbag"]
    f = np.sqrt(40.0 / max(seconds, 1.0))
    return max(p["sigma_axis_mm_40s"] * f, p["floor_axis_mm"]), max(p["sigma_rot_deg_40s"] * f, p["floor_rot_deg"])


def bag_agreement(per_bag: dict, seconds: dict, cfg) -> dict:
    """per_bag: {bag_id: result}; seconds: {bag_id: s of data}. A bag 'disagrees' when more than
    max_cams_out cameras differ from the median of the other bags beyond k_sigma * the expected scatter."""
    p = cfg["rgb"]["perbag"]
    bags = list(per_bag)
    out = {"bags": {}, "disagreeing": []}
    if len(bags) < 2:
        return out
    cams = sorted(set.intersection(*[set(r["cameras"]) for r in per_bag.values()]))
    for b in bags:
        others = [o for o in bags if o != b]
        n_out, rows = 0, {}
        for c in cams:
            T = np.array(per_bag[b]["cameras"][c]["T_lidar_cam"])
            To = [np.array(per_bag[o]["cameras"][c]["T_lidar_cam"]) for o in others]
            pos_o = np.median([t[:3, 3] for t in To], 0)
            ax = T[:3, 2]
            daxis = float(1e3 * (T[:3, 3] - pos_o) @ ax)
            dpos = float(1e3 * np.linalg.norm(T[:3, 3] - pos_o))
            Rm = Rot.from_matrix(np.array([t[:3, :3] for t in To])).mean()
            drot = float(np.degrees((Rm.inv() * Rot.from_matrix(T[:3, :3])).magnitude()))
            sa_b, sr_b = expected_sigma(cfg, seconds[b])
            so = [expected_sigma(cfg, seconds[o]) for o in others]
            sa = np.sqrt(sa_b ** 2 + np.mean([x[0] for x in so]) ** 2 / len(others))
            sr = np.sqrt(sr_b ** 2 + np.mean([x[1] for x in so]) ** 2 / len(others))
            bad = abs(daxis) > p["k_sigma"] * sa or dpos > p["k_sigma"] * 2 * sa or drot > p["k_sigma"] * sr
            n_out += bad
            rows[c] = {"daxis_mm": round(daxis, 1), "dpos_mm": round(dpos, 1), "drot_deg": round(drot, 3),
                       "limit_axis_mm": round(p["k_sigma"] * sa, 1), "limit_rot_deg": round(p["k_sigma"] * sr, 3),
                       "outside": bool(bad)}
        dis = n_out > p["max_cams_out"]
        out["bags"][b] = {"cameras_outside": n_out, "disagrees": dis, "per_camera": rows}
        if dis:
            out["disagreeing"].append(b)
    return out
