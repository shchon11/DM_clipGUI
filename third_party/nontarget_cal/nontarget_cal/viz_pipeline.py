"""Best-effort pipeline boundary observations; never used by numerical code."""
from __future__ import annotations

import hashlib
import json
import uuid

import numpy as np
import yaml

from .workspace import read_json


def _camera(value, sensor, size):
    intr = value.get("intr_kb", value.get("intr"))
    fx, fy, cx, cy = intr[:4]
    state = {"sensor": sensor, "width": size[0], "height": size[1],
             "model": "equidistant" if sensor == "rgb" else "plumb_bob",
             "K": [[fx, 0., cx], [0., fy, cy], [0., 0., 1.]],
             "D": list(intr[4:8]) if sensor == "rgb" else list(intr[4:6]) + [0., 0., 0.],
             "T_cam_lidar": value["T_cam_lidar"], "state": "pending", "gate_pass": None,
             "reprojection_px": value.get("reproj_median_px"), "sigma_rot_deg": None, "sigma_pos_mm": None}
    if sensor == "thermal":
        state["time_offset_s"] = float(np.mean(value.get("dt_s_mean", value.get("dt_s", 0.))))
        state["row_readout_s"] = value.get("rs_s")
    return state


def start(pl):
    # A run attempt has an identity even when visualization is disabled.
    pl.viz_run_id = uuid.uuid4().hex
    if not pl.cfg.get("viz", {}).get("enabled", True):
        return
    try:
        from pathlib import Path
        from scipy.spatial.transform import Rotation
        from .init_calib import load_init
        from .viz import configure
        design = yaml.safe_load(Path(pl.cfg["paths"]["rig_design"]).read_text())
        initial = load_init(pl.a.init) if pl.a.mode == "warm" else {}
        if pl.a.mode == "warm" and not all(c in initial.get("thermal", {}) for c in pl.cfg["cameras"]["thermal"]):
            # The thermal solver falls back as a pair, not camera by camera.
            initial["thermal"] = {}
        states = {}
        for sensor in pl.sensors:
            for cam in pl.cfg["cameras"][sensor]:
                value = initial.get(sensor, {}).get(cam)
                if value is None and sensor == "rgb" and pl.a.mode == "warm":
                    # Missing warm RGB calibration is a solver refusal; never
                    # display an invented design fallback as its actual input.
                    continue
                if value is None:
                    d = design[sensor][cam]
                    T = np.eye(4)
                    T[:3, :3] = Rotation.from_euler("ZYX", d["design_R_lidar_cam_euler_ZYX_deg"], degrees=True).as_matrix()
                    if sensor == "rgb":
                        intr = list(d["seed_intr_kb"])
                        intr[0] = intr[1] = .5 * (intr[0] + intr[1])
                        value = {"T_cam_lidar": np.linalg.inv(T).tolist(), "intr": intr}
                    else:
                        T[:3, 3] = d["design_centre_lidar_m"]
                        value = {"T_cam_lidar": np.linalg.inv(T).tolist(), "intr": d["nominal_intr_pinhole"],
                                 "dt_s": d["nominal_time_offset_s"], "rs_s": d["nominal_row_readout_s"]}
                states[cam] = _camera(value, sensor, pl.cfg["cameras"][sensor + "_size"])
                states[cam]["metric_source"] = "warm-start" if cam in initial.get(sensor, {}) else "solver-initial"
        manifest = {"cameras": {c: {k: v for k, v in s.items() if k in
                                    ("sensor", "width", "height", "model", "K", "D")} for c, s in states.items()},
                    "bags": pl.bags, "R_lidar_V": [[-1., 0., 0.], [0., -1., 0.], [0., 0., 1.]],
                    "provenance": {"synthetic": False, "pose_source": "solver"}}
        manifest["input_identity"] = hashlib.sha256(json.dumps(
            {"bags": pl.bags, "windows": getattr(pl.a, "windows", None), "lo": pl.cfg.get("lo", {})},
            sort_keys=True).encode()).hexdigest()
        pl.viz = configure(pl.ws.root, pl.cfg, context={"run_id": pl.viz_run_id, "manifest": manifest,
                           "bags": pl.bags}, log=pl.ev.log, fresh=True)
        pl.viz.publish({"stage": "extract", "progress": 0., "cameras": states,
                        "status_text": "실제 계산 · 초기값", "provenance": {"synthetic": False, "pose_source": "solver"}},
                       force=True)
    except Exception as exc:
        pl.ev.log(f"Visualization initialization skipped: {exc}")


def task_context(pl, stage, key, args):
    # All context construction is fail-open, including missing/corrupt metadata.
    context = {"run_id": pl.viz_run_id, "pipeline_stage": stage, "task": key,
               "pass": args.get("name", key), "solve_name": args.get("name", key),
               "enabled": not stage.startswith("validation")}
    try:
        windows = pl.plan()["windows"]
        selected = args.get("segs", args.get("wins", []))
        win = args.get("win")
        if win:
            selected = [win["name"] if isinstance(win, dict) else win]
        by_name = {w["name"]: w for w in windows}
        entries = [by_name[name] for name in selected if name in by_name]
        context.update(total_windows=len(windows), windows=entries)
        if entries:
            first = entries[0]
            context.update(window=windows.index(first) + 1, window_id=first["name"], bag_id=first["bag_id"])
        context["stage"] = ("lidar_odometry" if stage in ("lo", "axes") else
                            "rgb_ba" if stage.startswith("rgb") else
                            "thermal" if stage.startswith("thermal") else
                            "validation" if stage.startswith("validation") else "extract")
    except Exception:
        pass
    return context


def axes(pl):
    if pl.viz is None:
        return
    try:
        R = read_json(pl.ws.root / "lo" / "vehicle_axes.json")["R_L_V"]
        pl.viz.publish_manifest({"R_lidar_V": R})
    except Exception as exc:
        pl.ev.log(f"Visualization vehicle axes skipped: {exc}")


def cached_result(pl, sensor):
    """Restore completed production estimates on an interrupted-run resume."""
    if pl.viz is None:
        return
    try:
        result = read_json(pl.ws.root / sensor / "final" / "result.json")
        states = {}
        for cam, value in result["cameras"].items():
            state = _camera(value, sensor, pl.cfg["cameras"][sensor + "_size"])
            state.update(state="converged", metric_source="solver.cached-result", solve_name="final",
                         solver_pass="cached-result", purpose="calibration", iteration=None,
                         total_iterations=None, accepted=None, cost=None)
            states[cam] = state
        pl.viz.publish({"stage": "rgb_ba" if sensor == "rgb" else "thermal", "cameras": states,
                        "solve_name": "final", "solver_pass": "cached-result", "purpose": "calibration", "iteration": None,
                        "total_iterations": None, "accepted": None,
                        "status_text": "실제 계산 결과 복원 · " + sensor}, force=True)
    except Exception as exc:
        pl.ev.log(f"Visualization cached result skipped: {exc}")


def validation(pl, result):
    if pl.viz is None:
        return
    try:
        from .outputs.report import metrics_table
        metrics = metrics_table(result, pl.cfg)
        failures = result["gate"]["failures"]
        states = {}
        for sensor in pl.sensors:
            final = read_json(pl.ws.root / sensor / "final" / "result.json")
            sigma = result.get(sensor, {}).get("halves", {}).get("sigma", {})
            for cam, value in final["cameras"].items():
                state = _camera(value, sensor, pl.cfg["cameras"][sensor + "_size"])
                own = [f for f in failures if f.startswith(cam + ":")]
                sig, metric = sigma.get(cam, {}), metrics.get(cam, {})
                checked = np.isfinite(sig.get("along_axis_mm", np.nan)) and (
                    sensor != "rgb" or np.isfinite(sig.get("rot_deg", np.nan)))
                passed = False if own else (True if checked else None)
                state.update(gate_pass=passed, gate_reasons=own, state="failed" if own else "converged",
                             sigma_rot_deg=metric.get("rot_deg"), sigma_pos_mm=metric.get("pos_mm"),
                             validation_vote=(metric.get("vote") or {}).get("pass"),
                             informational_checks=["RGB edge vote is informational"] if sensor == "rgb" and metric.get("vote") else [],
                             metric_source="solver.validation", solve_name="final", solver_pass="final", cost=None,
                             iteration=None, total_iterations=None, accepted=None, purpose="validation")
                states[cam] = state
        pl.viz.publish({"stage": "validation", "pass": "final", "solver_pass": "final", "solve_name": "final",
                        "purpose": "validation", "cameras": states,
                        "iteration": None, "total_iterations": None, "accepted": None, "objective_cost": None,
                        "gate": {**result["gate"], "status": "pass" if result["gate"]["pass"] else "fail",
                         "source": "solver.validation"}, "status_text": "실제 검증 결과"}, force=True)
    except Exception as exc:
        pl.ev.log(f"Visualization validation skipped: {exc}")
