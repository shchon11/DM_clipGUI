"""Small, read-only snapshots for real jobs that do not publish a viz stream.

No solver, ROS, Qt, scans, or replay code is imported. Call on the viewer's
single I/O worker: initialization and finished results are read once, incomplete
finished outputs are retried, and progress projection performs no disk I/O.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml

import online_calib as oc
from . import SCHEMA
from .results import camera_gate

MAX_METADATA_BYTES = 4 * 1024 * 1024
MAX_CAMERAS = 64
NOMINAL_VEHICLE_AXES = np.diag([-1., -1., 1.]).tolist()


def _read(path):
    path = Path(path).expanduser()
    if path.stat().st_size > MAX_METADATA_BYTES:
        raise ValueError(f"시각화 메타데이터가 너무 큼: {path.name}")
    with path.open(encoding="utf-8") as handle:
        return (json.load(handle) if path.suffix == ".json" else yaml.safe_load(handle)) or {}


def _merge(base, overrides):
    result = deepcopy(base)
    for key, value in overrides.items():
        result[key] = (_merge(result[key], value) if isinstance(value, dict)
                       and isinstance(result.get(key), dict) else deepcopy(value))
    return result


def _recorded_executable(job):
    attempts = job.get("attempts") or []
    command = attempts[-1].get("cmd", []) if attempts else []
    return command[0] if isinstance(command, (list, tuple)) and command else job.get("executable")


def _source_identity(job):
    return (job.get("id"), job.get("mode"), job.get("init"), job.get("config"), job.get("out"),
            tuple(job.get("sensors", [])), _recorded_executable(job))


def _tool_config(job):
    """Read the executing installation first, then the bundled source fallback."""
    executable = _recorded_executable(job)
    roots = []
    custom = False
    if executable:
        path = Path(executable).expanduser()
        custom = path.absolute() != (oc.DEFAULT_VENV / "bin/nontarget_cal").absolute()
        for prefix in dict.fromkeys([path.parent.parent, path.resolve().parent.parent]):
            roots.extend(sorted(prefix.glob("lib/python3*/site-packages/nontarget_cal")))
    if not custom:
        roots.extend(sorted(oc.DEFAULT_VENV.glob("lib/python3*/site-packages/nontarget_cal")))
        roots.extend(d / "nontarget_cal" for d in oc.VENDORED_CANDIDATES)
    package = next((p for p in roots if (p / "config/default.yaml").is_file()), None)
    cfg = _read(package / "config/default.yaml") if package else {}
    if job.get("config"):
        cfg = _merge(cfg, _read(job["config"]))
    design = cfg.get("paths", {}).get("rig_design")
    if not design:
        if custom:
            raise FileNotFoundError(f"실행 도구의 초기 설계 설정을 찾을 수 없습니다: {executable}")
        raise FileNotFoundError("도구의 rig_design 설정을 찾을 수 없습니다")
    if str(design).startswith("@pkg/"):
        if package is None:
            raise FileNotFoundError("설치된 nontarget_cal 설계 파일을 찾을 수 없습니다")
        design = package / str(design)[5:]
    return cfg, _read(design)


def _rotation_zyx(degrees):
    """Intrinsic ZYX: same convention as the tool's design Euler angles."""
    z, y, x = np.radians(degrees)
    cz, sz, cy, sy, cx, sx = math.cos(z), math.sin(z), math.cos(y), math.sin(y), math.cos(x), math.sin(x)
    return np.array([[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                     [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                     [-sy, cy * sx, cy * cx]])


def _camera(sensor, pose, intr, size, model=None):
    pose = np.asarray(pose, float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1])
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-3)
            or np.linalg.det(pose[:3, :3]) < .99):
        raise ValueError("잘못된 초기/최종 카메라 변환")
    values = np.asarray(intr, float)
    if values.ndim != 1 or len(values) < 6 or not np.isfinite(values).all() or min(values[:2]) <= 0:
        raise ValueError("잘못된 카메라 내부 파라미터")
    model = model or ("equidistant" if sensor == "rgb" else "plumb_bob")
    dist = list(map(float, values[4:]))
    if model == "plumb_bob" and len(dist) == 2:
        dist += [0., 0., 0.]
    count = 4 if model == "equidistant" else 5
    dist = (dist + [0.] * count)[:count]
    width, height = map(int, size)
    if not 0 < width <= 8192 or not 0 < height <= 8192:
        raise ValueError("잘못된 카메라 영상 크기")
    return {"sensor": sensor, "model": model, "width": width, "height": height,
            "K": [[float(values[0]), 0., float(values[2])],
                  [0., float(values[1]), float(values[3])], [0., 0., 1.]],
            "D": dist, "T_cam_lidar": pose.tolist()}


def _design_cameras(cfg, design, sensors):
    result = {}
    for sensor in sensors:
        for name in cfg.get("cameras", {}).get(sensor, list(design.get(sensor, {})))[:MAX_CAMERAS]:
            item = design.get(sensor, {}).get(name)
            if item is None:
                continue
            R = _rotation_zyx(item["design_R_lidar_cam_euler_ZYX_deg"])
            centre = np.asarray(item["design_centre_lidar_m"] if sensor == "thermal" else [0., 0., 0.])
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = R.T, -R.T @ centre
            intr = list(item["seed_intr_kb"] if sensor == "rgb" else item["nominal_intr_pinhole"])
            if sensor == "rgb":
                # nominal_cams(square=True): all RGB centres start at the LiDAR
                # origin and fx/fy are averaged. Never substitute board positions.
                intr[0] = intr[1] = .5 * (intr[0] + intr[1])
            size = item.get("image_size", cfg.get("cameras", {}).get(sensor + "_size", [1920, 1200] if sensor == "rgb" else [640, 480]))
            result[name] = _camera(sensor, T, intr, size)
    return result


def _team_cameras(root):
    result = {}
    files = sorted((Path(root) / "extrinsic").glob("*.yaml"))
    if len(files) > MAX_CAMERAS:
        raise ValueError("카메라 수가 시각화 상한을 초과했습니다")
    for path in files:
        name = path.stem
        intr_path = Path(root) / "intrinsic" / (name + ".yaml")
        if not intr_path.is_file():
            continue
        ext, intr = _read(path), _read(intr_path)
        if ext.get("parent_frame", "os_lidar") != "os_lidar":
            raise ValueError(f"{name}: 지원하지 않는 부모 좌표계")
        K = np.asarray(intr["camera_matrix"], float)
        values = [K[0, 0], K[1, 1], K[0, 2], K[1, 2], *intr["distortion_coefficients"]]
        sensor = "thermal" if name.startswith("thermal") else "rgb"
        result[name] = _camera(sensor, ext["T_cam_lidar"], values,
                               [intr["image_width"], intr["image_height"]], intr.get("model"))
    return result


def _warm_cameras(spec, cfg):
    """The tool's --init formats, without importing the tool or its solver stack."""
    result = {}
    def insert(name, sensor, pose, intr):
        size = cfg.get("cameras", {}).get(sensor + "_size", [1920, 1200] if sensor == "rgb" else [640, 480])
        result.setdefault(name, _camera(sensor, pose, intr, size))
    for text in str(spec).split(","):
        path = Path(text.strip()).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"warm-start 초기값 없음: {path}")
        if path.is_dir():
            for name, cam in _team_cameras(path).items():
                result.setdefault(name, cam)
            for folder in (path / "calib_json", path):
                for file in sorted(folder.glob("*.json"))[:MAX_CAMERAS]:
                    item = _read(file)
                    if "T_cam_lidar" in item and "intr" in item and item.get("model", "kb") == "kb":
                        insert(item.get("channel", file.stem.replace("final_", "")), "rgb", item["T_cam_lidar"], item["intr"][:8])
        elif path.suffix == ".json":
            for name, item in _read(path).get("cameras", {}).items():
                if "intr_kb" in item:
                    insert(name, "rgb", item["T_cam_lidar"], item["intr_kb"])
                elif "rs_s" in item:
                    insert(name, "thermal", item["T_cam_lidar"], item["intr"])
        elif path.suffix in (".yaml", ".yml"):
            for name, item in _read(path).get("cameras", {}).items():
                intr = item["intrinsics"]
                K = np.asarray(intr["camera_matrix"], float)
                insert(name, "thermal", item["T_cam_lidar"],
                       [K[0, 0], K[1, 1], K[0, 2], K[1, 2], *intr["distortion_coefficients"]])
    if not result:
        raise ValueError("warm-start 카메라 초기값을 읽을 수 없습니다")
    return result


def _message(event):
    return str(event.get("msg_ko") or event.get("msg") or event.get("code") or "")


def _metric(value):
    try:
        value = float(value)
        return value if math.isfinite(value) and value >= 0 else None
    except (TypeError, ValueError):
        return None


class JobSnapshot:
    """Cache small initial/final metadata, project copied ``online_calib.Progress``.

    ``snapshot(progress=None, job=None)`` returns ``(manifest, event, decoded)``.
    Replace ``job`` with an updated copy to reflect state changes. No intermediate
    pose or metric is inferred from elapsed time or progress. ``interpolate=False``
    also asks the viewer to switch directly from actual initial to final poses.
    """
    def __init__(self, job):
        self.job = deepcopy(job)
        self.seq = 0
        self._initial = {}
        self._final = None
        self._warnings = []
        self._window = {}
        self._initial_axes = NOMINAL_VEHICLE_AXES
        self._axes_source = "nominal os_lidar mount axes; not measured"
        try:
            cfg, design = _tool_config(job)
            self._initial = _design_cameras(cfg, design, job.get("sensors", ["rgb", "thermal"]))
            if job.get("mode") == "warm":
                warm = _warm_cameras(job["init"], cfg)
                # The solver refuses missing RGB; only thermal explicitly falls
                # back to the entire thermal design when any thermal init is absent.
                wanted = {k for k, v in self._initial.items() if v["sensor"] == "thermal"}
                if not wanted.issubset(warm):
                    warm = {k: v for k, v in warm.items() if v["sensor"] != "thermal"}
                    warm.update({k: v for k, v in self._initial.items() if k in wanted})
                    if wanted:
                        self._warnings.append("열화상 warm-start 일부 없음: 도구와 동일하게 열화상 설계값 사용")
                expected = set(self._initial)
                missing_rgb = [k for k, v in self._initial.items() if v["sensor"] == "rgb" and k not in warm]
                if missing_rgb:
                    self._warnings.append("RGB warm-start 초기값 없음: " + ", ".join(missing_rgb))
                self._initial = {k: v for k, v in warm.items() if k in expected}
                paths = [Path(p.strip()).expanduser() / "rig.yaml" for p in str(job["init"]).split(",")]
                rig = next((p for p in paths if p.is_file()), None)
                if rig:
                    self._initial_axes = _read(rig).get("R_lidar_V", NOMINAL_VEHICLE_AXES)
                    self._axes_source = "warm-start rig.yaml"
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            # Never quietly show design poses as if they were a failed warm init.
            self._initial = {}
            self._warnings.append(f"초기값 표시 불가: {exc}")

    def _load_final(self):
        out = Path(self.job["out"])
        summary = _read(out / "summary.json")
        metrics_path = out / "metrics.json"
        metrics = _merge(summary.get("metrics", {}), _read(metrics_path)) if metrics_path.is_file() else summary.get("metrics", {})
        cameras = _team_cameras(out)
        expected = set(summary.get("metrics") or metrics)
        if not cameras or not expected.issubset(cameras):
            raise ValueError("최종 카메라 YAML 발행 대기")
        rig_path = out / "rig.yaml"
        axes = _read(rig_path).get("R_lidar_V", self._initial_axes) if rig_path.is_file() else self._initial_axes
        self._final = (summary, metrics, cameras, axes, "final rig.yaml" if rig_path.is_file() else self._axes_source)

    def snapshot(self, progress=None, job=None):
        if job is not None:
            if _source_identity(job) != _source_identity(self.job):
                # Selecting a pending job may precede its first recorded
                # executable. Refresh once when launch establishes that source.
                previous_seq = self.seq
                self.__init__(job)
                self.seq = previous_seq
            else:
                self.job = deepcopy(job)
        job = self.job
        progress = progress or oc.Progress(job.get("sensors", ["rgb", "thermal"]))
        finished = job.get("state") == oc.DONE
        warnings = list(self._warnings)
        if finished and self._final is None:
            try:
                self._load_final()
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
                warnings.append(f"최종 결과 표시 대기: {exc}")
        source = "warm-start" if job.get("mode") == "warm" else "initial-design"
        cameras, axes, axes_source = self._initial, self._initial_axes, self._axes_source
        summary, metrics = {}, {}
        if finished and self._final is not None:
            summary, metrics, cameras, axes, axes_source = self._final
            source = "final-result"
        manifest = {"schema": SCHEMA, "run_id": "job-" + str(job.get("id", job.get("workdir", "unknown"))),
                    "mode": "job", "parent_frame": "os_lidar", "display_frame": "vehicle",
                    "R_lidar_V": axes, "cameras": {k: {a: b for a, b in v.items() if a != "T_cam_lidar"} for k, v in cameras.items()},
                    "provenance": {"pose_source": source, "vehicle_axes": axes_source}}
        states = {}
        for name, camera in cameras.items():
            metric = metrics.get(name, summary.get("metrics", {}).get(name, {}))
            gate = camera_gate(summary, metrics, name) if source == "final-result" else {}
            passed = gate.get("pass")
            states[name] = {"T_cam_lidar": camera["T_cam_lidar"],
                            "sigma_rot_deg": _metric(metric.get("rot_deg")),
                            "sigma_pos_mm": _metric(metric.get("pos_mm")),
                            "reprojection_px": _metric(metric.get("track_reproj_px")),
                            "state": "converged" if passed is True else ("failed" if passed is False else "pending"),
                            "gate_pass": passed, "gate_reasons": gate.get("reasons", []),
                            "informational_checks": gate.get("informational", []),
                            "validation_vote": (metric.get("vote") or {}).get("pass"),
                            "validation_vote_gate": (metric.get("vote") or {}).get("gate"),
                            "metric_source": "final-result:metrics.json" if source == "final-result" else "unavailable"}
        current = progress.current()
        stage = current[-1][4] or current[-1][0] if current else "pending"
        last = progress.last or {}
        if last.get("stage") and str(last.get("ev", "")).startswith("stage_"):
            stage = last["stage"]
        if finished:
            stage = "outputs"
        label = oc.STAGE_LABEL.get(oc.main_stage(stage), stage)
        if stage != oc.main_stage(stage):
            label += f" ({stage})"
        warnings.extend(_message(ev) for ev in progress.warnings[-6:])
        warnings.extend(_message(ev) for ev in (progress.error, progress.refusal) if ev)
        warnings.extend(_message(ev) for ev in progress.task_failed[-3:])
        warnings = list(dict.fromkeys(w for w in warnings if w))[-8:]
        # Task counts for RGB tracks (camera x window), solves and validations
        # are not window counts. Only these stages are one task per window.
        for key in ("extract", "lo", "thermal_solve"):
            item = progress.stages.get(key, {})
            if item.get("sub") in ("extract", "lo", "thermal_edges") and item.get("total", 0) > 0:
                self._window = {"window": item["done"], "total_windows": item["total"]}
        retained = getattr(progress, "viz_progress", {})
        for key in ("window", "total_windows"):
            if isinstance(retained.get(key), int) and retained[key] >= 0:
                self._window[key] = retained[key]
        for key in ("window", "total_windows"):
            if isinstance(last.get(key), int) and last[key] >= 0:
                self._window[key] = last[key]
        if source == "final-result" and isinstance(summary.get("windows", {}).get("kept"), list):
            count = len(summary["windows"]["kept"])
            self._window = {"window": count, "total_windows": count}
        gate = summary.get("validation", {}).get("gate", {})
        passed = gate.get("pass") if source == "final-result" else None
        pose_text = {"initial-design": "설계 초기값 · 중간 해 미발행", "warm-start": "warm-start 초기값 · 중간 해 미발행", "final-result": "실제 최종 결과"}[source]
        if not cameras:
            pose_text = "카메라 초기값 표시 불가"
        status_text = f"{oc.STATE_LABEL.get(job.get('state'), job.get('state', ''))} · {pose_text}"
        if job.get("state_msg"):
            status_text += " · " + str(job["state_msg"])
        self.seq += 1
        event = {"schema": SCHEMA, "seq": self.seq, "t": time.time(), "stage": oc.main_stage(stage),
                 "stage_label": label, "status_text": status_text, "interpolate": False,
                 "progress": 1.0 if finished else progress.fraction(),
                 "eta_s": 0.0 if finished else _metric(last.get("eta_s", retained.get("eta_s"))),
                 "cameras": states, "assets": {}, "warnings": warnings,
                 "gate": {"status": "pass" if passed is True else ("fail" if passed is False else "pending"),
                          "pass": passed, "source": "summary.validation.gate" if source == "final-result" else "pending",
                          "warning": " · ".join(warnings[-2:])},
                 "provenance": {"pose_source": source, "synthetic": False, "vehicle_axes": axes_source},
                 **self._window}
        return manifest, event, {"matching": {}}
