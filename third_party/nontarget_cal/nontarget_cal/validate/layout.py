"""Physical-layout checks driven by config/layout_rules.yaml (rules are data, not code).

Positions: camera centres in the vehicle frame V (x forward, y left, z up, origin at the Ouster;
R_L_V from the LiDAR data of the run, lo/axes.py)."""
from __future__ import annotations

import numpy as np


def positions_V(results, R_L_V) -> dict:
    P = {}
    for res in results:
        if not res:
            continue
        for c, v in res["cameras"].items():
            T = np.array(v["T_lidar_cam"])
            P[c] = R_L_V.T @ T[:3, 3]
    return P


def check(rules: list, P: dict) -> dict:
    out = {"rules": [], "pass": True, "skipped": []}
    for r in rules:
        t = r["type"]
        rec = {"name": r["name"], "type": t}
        try:
            if t == "line":
                cs = [c for c in r["cameras"] if c in P]
                if len(cs) < 3:
                    raise KeyError(r["cameras"])
                x = np.array([P[c][0] for c in cs]); y = np.array([P[c][1] for c in cs])
                A = np.stack([y, np.ones_like(y)], 1)
                co = np.linalg.lstsq(A, x, rcond=None)[0]
                res_ = float(np.abs(x - A @ co).max())
                spr = float(np.ptp(x))
                rec.update({"line_resid_max_m": round(res_, 4), "forward_spread_m": round(spr, 4),
                            "row_yaw_deg": round(float(np.degrees(np.arctan(co[0]))), 2),
                            "pass": res_ <= r["max_resid_m"] and spr <= r["max_spread_m"],
                            "limits": {"max_resid_m": r["max_resid_m"], "max_spread_m": r["max_spread_m"]}})
            elif t == "box":
                p = P[r["camera"]]
                lo, hi = np.array(r["lo"]), np.array(r["hi"])
                rec.update({"position_V_m": np.round(p, 3).tolist(), "lo": r["lo"], "hi": r["hi"],
                            "pass": bool(np.all(p >= lo) and np.all(p <= hi))})
            elif t == "mirror_pair":
                a, b = P[r["a"]], P[r["b"]]
                dx, sy, dz = float(a[0] - b[0]), float(a[1] + b[1]), float(a[2] - b[2])
                rec.update({"dx_m": round(dx, 4), "sum_y_m": round(sy, 4), "dz_m": round(dz, 4),
                            "half_width_m": round(float((a[1] - b[1]) / 2), 4),
                            "pass": abs(dx) <= r["max_dx_m"] and abs(sy) <= r["max_sum_y_m"] and abs(dz) <= r["max_dz_m"]})
            elif t == "between":
                ax = "xyz".index(r["axis"])
                p = P[r["camera"]]
                lo, hi = sorted([P[r["between"][0]][ax], P[r["between"][1]][ax]])
                row = np.mean([P[c][0] for c in r["row"] if c in P])
                behind = float(row - p[0])
                rec.update({"value_m": round(float(p[ax]), 4), "between_m": [round(float(lo), 4), round(float(hi), 4)],
                            "behind_row_m": round(behind, 4),
                            "pass": bool(lo < p[ax] < hi and r["behind_min_m"] <= behind <= r["behind_max_m"])})
            else:
                raise ValueError(t)
        except KeyError:
            out["skipped"].append(r["name"])
            continue
        rec["pass"] = bool(rec["pass"])
        out["rules"].append(rec)
        out["pass"] &= rec["pass"]
    return out
