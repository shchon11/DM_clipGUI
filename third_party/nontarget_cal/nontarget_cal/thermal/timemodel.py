"""Thermal frame tables and the time model (thermal_lo/code/tlcommon.py:time_model, per bag).

Frames are NAMED BY header.stamp (no lead applied). The capture time of image row v is modelled as
    t(v) = t_frame + dt_cam[,win] + rs_cam * (v / 480 - 0.5)
'header': t_frame = header.stamp.
'smooth' (default): t_frame = camclock + (a + b * camclock), one robust linear fit of (header -
camclock) against the A70's own frame clock over the whole bag (all metadata messages). The frame
clock steps exactly 33/34 ms; the header is that clock plus an offset the host re-estimates now and
then (steps of 1-17 ms). The fit keeps the header's clock and mean offset and removes the steps.
"""
from __future__ import annotations

import json

import numpy as np

_TM = {}


def frame_index(ws, bag_id: str, cam: str, wins) -> dict:
    """All extracted frames of one camera of one bag (union over its windows), sorted by header."""
    h, ffc, sd, mad = [], [], [], []
    for w in wins:
        p = ws.thermal16(w) / f"index_{cam}.npz"
        if not p.exists():
            continue
        z = np.load(p)
        h.append(z["header_ns"]); ffc.append(z["ffc"]); sd.append(z["sd"]); mad.append(z["mad_prev"])
    if not h:
        return {"header_ns": np.zeros(0, np.int64), "ffc": np.zeros(0, bool)}
    h = np.concatenate(h); ffc = np.concatenate(ffc)
    o = np.argsort(h, kind="stable")
    h, ffc = h[o], ffc[o]
    u = np.r_[True, np.diff(h) > 0]                        # windows do not overlap, but be safe
    return {"header_ns": h[u], "ffc": ffc[u], "sd": np.concatenate(sd)[o][u], "mad_prev": np.concatenate(mad)[o][u]}


def clock_fit(ws, bag_id: str, cam: str):
    """Robust line of (header - camclock) [ms] vs camclock [s] over the whole bag -> (co, cc0, info)."""
    key = (str(ws.root), bag_id, cam)
    if key in _TM:
        return _TM[key]
    z = np.load(ws.root / "thermal16" / f"{bag_id}_clock_{cam}.npz")
    h = z["header_ns"].astype(np.int64)
    cc = z["camclock_ns"].astype(np.int64)
    ok = cc > 0
    x = (cc[ok] - cc[ok][0]).astype(np.float64) * 1e-9
    y = (h[ok] - cc[ok]).astype(np.float64) * 1e-6        # ms
    m = np.ones(len(x), bool)
    for _ in range(5):
        A = np.vstack([x[m], np.ones(m.sum())]).T
        co = np.linalg.lstsq(A, y[m], rcond=None)[0]
        r = y - (co[0] * x + co[1])
        s = 1.4826 * np.median(np.abs(r[m] - np.median(r[m])))
        m = np.abs(r) < max(3 * s, 0.5)
    info = {"kind": "smooth", "drift_ppm": float(co[0] * 1e3), "resid_ms_sd": float(r[m].std()),
            "resid_ms_p1_p99": [float(np.percentile(r, 1)), float(np.percentile(r, 99))],
            "n_metadata": int(len(h)), "n_with_clock": int(ok.sum())}
    out = (co, int(cc[ok][0]), h, cc, info)
    _TM[key] = out
    return out


def t_frame(ws, bag_id: str, cam: str, headers: np.ndarray, kind: str = "smooth") -> np.ndarray:
    """t_frame (float64 ns) for frames with the given header stamps."""
    headers = np.asarray(headers, np.int64)
    if kind == "header":
        return headers.astype(np.float64)
    co, cc0, h_all, cc_all, _ = clock_fit(ws, bag_id, cam)
    k = np.clip(np.searchsorted(h_all, headers), 0, len(h_all) - 1)
    hit = h_all[k] == headers
    cc = np.where(hit, cc_all[k], -1)
    ok = cc > 0
    xa = (cc - cc0).astype(np.float64) * 1e-9
    pred = co[0] * xa + co[1]
    return np.where(ok, cc.astype(np.float64) + pred * 1e6, headers.astype(np.float64))


def time_model(ws, bag_id: str, cam: str, wins, kind: str = "smooth"):
    """-> (header_ns array, t_frame_ns array (float64), info) over the extracted frames of the bag."""
    key = ("tm", str(ws.root), bag_id, cam, kind, tuple(wins))
    if key in _TM:
        return _TM[key]
    ix = frame_index(ws, bag_id, cam, wins)
    h = ix["header_ns"]
    tf = t_frame(ws, bag_id, cam, h, kind)
    info = {"kind": "header"} if kind == "header" else clock_fit(ws, bag_id, cam)[4]
    _TM[key] = (h, tf, info)
    return _TM[key]


class Plan:
    """Window -> bag bookkeeping for the thermal code (was tlcommon.SEG_WIN / ZERO_NS)."""

    def __init__(self, ws):
        p = json.loads((ws.root / "windows" / "plan.json").read_text())
        self.windows = {w["name"]: w for w in p["windows"]}
        self.zero = {b["bag_id"]: int(b["zero_ns"]) for b in p["bags"]}
        self.t_ref = min(self.zero.values())

    def bag(self, win: str) -> str:
        return self.windows[win]["bag_id"]

    def wins_of_bag(self, bag_id: str):
        return [w for w, v in self.windows.items() if v["bag_id"] == bag_id and v.get("kind", "moving") == "moving"]

    def span_ns(self, win: str):
        w = self.windows[win]
        z = self.zero[w["bag_id"]]
        return z + int(float(w["t0_s"]) * 1e9), z + int(float(w["t1_s"]) * 1e9)
