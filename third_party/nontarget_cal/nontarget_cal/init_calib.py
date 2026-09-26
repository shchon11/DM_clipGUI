"""Reading a previous calibration for --init (warm start) and for the name-identification reference.

Accepted (comma-separated list of paths; later entries fill cameras missing from earlier ones):
  * an output directory of this tool, or a team deliverable directory: extrinsic/<cam>.yaml +
    intrinsic/<cam>.yaml (RGB equidistant; thermal plumb_bob with time_offset_s / row_readout_s)
  * a directory of online_calib-format JSONs (camera_*.json with "intr" and "T_cam_lidar"),
    e.g. /hdd/DM_calib/lidar_odo/final or <out>/calib_json
  * a thermal_lo_calib.yaml file (thermal_lo product)
  * a result.json of an RGB (intr_kb) or thermal (intr, rs_s, dt_s) solve
Returns {"rgb": {cam: {"T_cam_lidar": 4x4, "intr": [fx fy cx cy k1 k2 k3 k4]}},
         "thermal": {cam: {"T_cam_lidar", "intr": [fx fy cx cy k1 k2], "rs_s", "dt_s"}}}
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml


def _from_team_dir(d: Path, out: dict):
    for f in sorted((d / "extrinsic").glob("*.yaml")):
        c = f.stem
        e = yaml.safe_load(f.read_text())
        ip = d / "intrinsic" / f"{c}.yaml"
        if not ip.exists():
            continue
        i = yaml.safe_load(ip.read_text())
        K = np.array(i["camera_matrix"], float)
        D = list(i["distortion_coefficients"])
        T = np.array(e["T_cam_lidar"], float).tolist()
        if i.get("model") == "equidistant":
            D = (D + [0, 0, 0, 0])[:4]
            out["rgb"].setdefault(c, {"T_cam_lidar": T, "intr": [K[0, 0], K[1, 1], K[0, 2], K[1, 2], *map(float, D)]})
        else:
            out["thermal"].setdefault(c, {"T_cam_lidar": T, "intr": [K[0, 0], K[1, 1], K[0, 2], K[1, 2], float(D[0]), float(D[1])],
                                          "rs_s": float(e.get("row_readout_s", -0.025)), "dt_s": float(e.get("time_offset_s", -0.1))})


def _from_json_dir(d: Path, out: dict):
    for f in sorted(d.glob("*.json")):
        try:
            v = json.loads(f.read_text())
        except Exception:  # noqa
            continue
        if "T_cam_lidar" in v and "intr" in v and v.get("model", "kb") == "kb":
            c = v.get("channel", f.stem.replace("final_", ""))
            out["rgb"].setdefault(c, {"T_cam_lidar": v["T_cam_lidar"], "intr": list(v["intr"])[:8]})


def _from_result(p: Path, out: dict):
    r = json.loads(p.read_text())
    for c, v in r.get("cameras", {}).items():
        if "intr_kb" in v:
            out["rgb"].setdefault(c, {"T_cam_lidar": v["T_cam_lidar"], "intr": v["intr_kb"]})
        elif "rs_s" in v:
            out["thermal"].setdefault(c, {"T_cam_lidar": v["T_cam_lidar"], "intr": v["intr"], "rs_s": v["rs_s"],
                                          "dt_s": v.get("dt_s_mean", v.get("dt_s"))})


def _from_thermal_yaml(p: Path, out: dict):
    y = yaml.safe_load(p.read_text())
    for c, v in (y.get("cameras") or {}).items():
        K = np.array(v["intrinsics"]["camera_matrix"], float)
        D = v["intrinsics"]["distortion_coefficients"]
        out["thermal"].setdefault(c, {"T_cam_lidar": v["T_cam_lidar"],
                                      "intr": [K[0, 0], K[1, 1], K[0, 2], K[1, 2], float(D[0]), float(D[1])],
                                      "rs_s": float(v["time"]["row_readout_s"]), "dt_s": float(v["time"]["time_offset_s"])})


def load_init(spec) -> dict:
    out = {"rgb": {}, "thermal": {}, "sources": []}
    for p in [Path(s.strip()) for s in str(spec).split(",") if s.strip()]:
        if not p.exists():
            raise FileNotFoundError(f"--init {p} does not exist")
        if p.is_dir():
            if (p / "extrinsic").is_dir():
                _from_team_dir(p, out)
            if (p / "calib_json").is_dir():
                _from_json_dir(p / "calib_json", out)
            _from_json_dir(p, out)
        elif p.suffix == ".json":
            _from_result(p, out)
        elif p.suffix in (".yaml", ".yml"):
            _from_thermal_yaml(p, out)
        out["sources"].append(str(p))
    if not out["rgb"] and not out["thermal"]:
        raise ValueError(f"--init {spec}: no calibration found")
    return out
