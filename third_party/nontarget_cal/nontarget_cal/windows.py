"""Window selection: where in the bags to extract data.

The moving part of every bag (speed > windows.moving_speed_mps, gaps < 3 s bridged) is cut into
windows of at most `length_s` (40 s, as S01-S10 / W00-W14 of the validated work: every window gets
its own LiDAR odometry, so drift never enters), equal lengths per moving interval, none shorter than
`min_length_s`. Each window carries its total absolute yaw rotation (sampled INS heading).

  RGB     : all windows (the most rotating ones first if more than max_windows)
  thermal : windows with >= thermal_min_rotation_deg of rotation (thermal_lo: >= 120 deg;
            the lever arm and the focal are seen through rotation)
  parked  : one standstill moment per bag (>= standstill_min_s) for the parked projection images

Explicit windows (`--windows NAME:T0:T1,...`, seconds from the bag's first /gps/fix stamp; a
"b1/" prefix selects the second bag) replace the automatic choice, e.g. to reproduce a study.
"""
from __future__ import annotations

import numpy as np


def _intervals(mask, t):
    out = []
    i = 0
    n = len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            out.append((t[i], t[j]))
            i = j + 1
        else:
            i += 1
    return out


def candidate_windows(t, yaw, speed, cfg, zero_ns: int):
    w = cfg["windows"]
    if len(t) < 3:
        return []
    ts = (np.asarray(t, np.int64) - zero_ns) * 1e-9
    mov = np.nan_to_num(np.asarray(speed, float)) > w["moving_speed_mps"]
    ivs = _intervals(mov, ts)
    merged = []
    for a, b in ivs:                            # bridge short stops (< 3 s)
        if merged and a - merged[-1][1] < 3.0:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    dyaw = np.abs(np.degrees(np.diff(np.asarray(yaw, float))))
    tm = 0.5 * (ts[1:] + ts[:-1])
    out = []
    for a, b in merged:
        L = b - a
        if L < w["min_length_s"]:
            continue
        n = int(np.ceil(L / w["length_s"]))
        edges = np.linspace(a, b, n + 1)
        for k in range(n):
            t0, t1 = float(edges[k]), float(edges[k + 1])
            if t1 - t0 < w["min_length_s"]:
                continue
            m = (tm >= t0) & (tm < t1)
            out.append({"t0_s": round(t0, 2), "t1_s": round(t1, 2), "rotation_deg": float(dyaw[m].sum()),
                        "length_s": round(t1 - t0, 2), "kind": "moving"})
    return out


def standstill(t, speed, cfg, zero_ns: int):
    w = cfg["windows"]
    ts = (np.asarray(t, np.int64) - zero_ns) * 1e-9
    still = np.nan_to_num(np.asarray(speed, float), nan=9.0) < 0.2
    best = None
    for a, b in _intervals(still, ts):
        if b - a >= w["standstill_min_s"] and (best is None or b - a > best[1] - best[0]):
            best = (a, b)
    if best is None:
        return None
    m = 0.5 * (best[0] + best[1])
    return {"t0_s": round(m - 0.5, 2), "t1_s": round(m + 0.5, 2), "kind": "standstill", "length_s": 1.0}


def parse_explicit(spec: str, bag_ids):
    out = []
    for item in [s for s in spec.replace(" ", ",").split(",") if s]:
        bag = bag_ids[0]
        if "/" in item:
            b, item = item.split("/", 1)
            bag = bag_ids[int(b[1:])] if b.startswith("b") and b[1:].isdigit() else b
        name, t0, t1 = item.split(":")
        out.append({"bag_id": bag, "name": name, "t0_s": float(t0), "t1_s": float(t1), "kind": "moving"})
    return out


def plan_windows(series: dict, zeros: dict, cfg, sensors, explicit: str | None = None,
                 standstill_needed: bool = True, rotation_of=None) -> dict:
    """series: {bag_id: {"t","yaw","speed"}}; zeros: {bag_id: zero_ns}. Returns the plan dict."""
    w = cfg["windows"]
    wins = []
    if explicit:
        wins = parse_explicit(explicit, list(series))
        for x in wins:
            s = series[x["bag_id"]]
            ts = (s["t"] - zeros[x["bag_id"]]) * 1e-9
            m = (ts >= x["t0_s"]) & (ts <= x["t1_s"])
            x["rotation_deg"] = float(np.abs(np.degrees(np.diff(s["yaw"][m]))).sum()) if m.sum() > 1 else 0.0
            x["length_s"] = x["t1_s"] - x["t0_s"]
    else:
        for bag_id, s in series.items():
            for k, c in enumerate(candidate_windows(s["t"], s["yaw"], s["speed"], cfg, zeros[bag_id])):
                wins.append({"bag_id": bag_id, "name": f"{bag_id}_w{k:02d}", **c})
        if len(wins) > w["max_windows"]:
            keep = sorted(range(len(wins)), key=lambda i: -wins[i]["rotation_deg"])[:w["max_windows"]]
            wins = [wins[i] for i in sorted(keep)]
    for x in wins:
        x["rgb"] = "rgb" in sensors
        x["thermal"] = ("thermal" in sensors) and x["rotation_deg"] >= w["thermal_min_rotation_deg"]
    parked = []
    if standstill_needed:
        for bag_id, s in series.items():
            p = standstill(s["t"], s["speed"], cfg, zeros[bag_id])
            if p:
                parked.append({"bag_id": bag_id, "name": f"{bag_id}_parked", **p})
    return {"windows": wins, "parked": parked,
            "bags": [{"bag_id": b, "zero_ns": int(zeros[b])} for b in series]}
