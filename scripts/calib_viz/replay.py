"""A deterministic, explicitly synthetic convergence replay over real assets.

No solver is imported or executed. Only small source frames are mapped; large
``lo/map_*.npz`` archives are never loaded. Each window is sampled separately
and only a small asset cache is retained.
Frames are decoded on demand for the current replay step, never for the whole run.
"""
import argparse
from collections import OrderedDict, deque
import hashlib
import json
import math
import os
import shutil
import uuid
import zipfile
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import yaml

from . import SCHEMA
from .geometry import interpolate_pose, vehicle_from_camera
from .stream import append_snapshot, atomic_json, atomic_npz
from .results import camera_gate

CAMERA_NAMES = (["camera_front" + str(i) for i in range(1, 10)] +
                ["camera_top", "camera_side_left", "camera_side_right",
                 "camera_rear_left", "camera_rear_right", "thermal_left", "thermal_right"])
PREVIEW_CAMERAS = tuple(CAMERA_NAMES)
ASSET_STEPS = 12
POINTS_PER_SWEEP = 4800
PREVIEW_POINTS = 7000
MAX_SNAPSHOTS = 4096
MAX_ASSET_SETS = 3
MAX_TRAJECTORY_POSES = 4096
MAX_TRACK_BYTES = 32 * 1024 * 1024


def _read_json(path):
    with Path(path).open(encoding="utf-8") as handle:
        return json.load(handle)


def _read_yaml(path):
    with Path(path).open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _sample_scan(path, limit):
    """Memory-map one raw NPY scan, then materialize at most limit records."""
    scan = np.load(path, mmap_mode="r", allow_pickle=False)
    if not len(scan):
        return np.empty((0, 3)), np.empty(0)
    index = np.linspace(0, len(scan) - 1, min(limit, len(scan)), dtype=int)
    sample = scan[index]
    if scan.dtype.names:
        points = np.column_stack([sample[key] for key in ("x", "y", "z")])
        offsets = sample["t"].astype(float) if "t" in scan.dtype.names else np.zeros(len(sample))
    else:
        points = np.asarray(sample[:, :3], float)
        offsets = np.zeros(len(sample))
    radius = np.linalg.norm(points, axis=1)
    keep = np.isfinite(points).all(axis=1) & (radius > 2.5) & (radius < 100)
    return points[keep], offsets[keep]


class ReplayProducer:
    """Read a completed run and publish visualization-only mock iterations.

    ``prepare`` writes metadata only. ``snapshot_at`` lazily builds a bounded
    per-window asset cache and supports deterministic screenshot/seek without mutating the event log.
    ``run(stop_event)`` emits complete snapshots at <= 4 Hz with real event
    timing divided by speed. Do not call prepare/run from the GUI thread.
    """
    def __init__(self, workdir, output_dir, out_dir=None, speed=60):
        self.workdir = Path(workdir).resolve()
        self.output_dir = Path(output_dir).resolve()
        self.out_dir = Path(out_dir).resolve() if out_dir else self.workdir.parent / "out"
        if not np.isfinite(speed) or speed <= 0:
            raise ValueError("speed must be positive and finite")
        # A replay must never overwrite the completed solver run's real events.
        if self.output_dir == self.workdir or self.output_dir == self.out_dir:
            raise ValueError("replay output must be separate from source work/out")
        # OpenCV otherwise creates a machine-wide thread pool even for tiny
        # previews. This affects only the separate replay/viewer process.
        cv2.setNumThreads(1)
        self.speed = float(speed)
        self.manifest = None
        self.assets = OrderedDict()
        self._retired_assets = deque()
        self.window_name = None
        self.prepared = False

    def prepare(self):
        if self.prepared:
            return self.manifest
        rig = _read_yaml(self.out_dir / "rig.yaml")
        self.metrics = _read_json(self.out_dir / "metrics.json")
        summary = self.summary = _read_json(self.out_dir / "summary.json")
        self.final_gate = summary.get("validation", {}).get("gate", {})
        self.window_names = summary.get("windows", {}).get("kept", [])
        self.total_windows = len(self.window_names) or 1
        self.R_lidar_V = np.asarray(rig["R_lidar_V"], float)
        self.cameras = {}
        for name in CAMERA_NAMES:
            intr = _read_yaml(self.out_dir / "intrinsic" / (name + ".yaml"))
            ext = _read_yaml(self.out_dir / "extrinsic" / (name + ".yaml"))
            self.cameras[name] = {
                "sensor": "thermal" if name.startswith("thermal") else "rgb",
                "model": intr["model"], "width": intr["image_width"],
                "height": intr["image_height"], "K": intr["camera_matrix"],
                "D": intr["distortion_coefficients"],
                "T_cam_lidar_final": ext["T_cam_lidar"],
                "time_offset_s": ext.get("time_offset_s", 0),
                "row_readout_s": ext.get("row_readout_s", 0),
            }
        self.events = []
        with (self.workdir / "events.jsonl").open(encoding="utf-8") as handle:
            for line in handle:
                try:
                    event = json.loads(line)
                    if np.isfinite(float(event.get("t", float("nan")))):
                        self.events.append(event)
                except (ValueError, TypeError, AttributeError):
                    continue
        self.events.sort(key=lambda e: e["t"])
        if not self.events:
            raise ValueError("source events.jsonl has no timestamped events")
        self.start_time, self.end_time = self.events[0]["t"], self.events[-1]["t"]
        self.duration_s = max(1, self.end_time - self.start_time)
        starts = {}
        for event in self.events:
            if event.get("ev") == "stage_start":
                starts.setdefault(event.get("stage"), event["t"])
        # Keep the restarted source run's true elapsed time. Validation begins
        # at the last successful validation, not the premature first attempt.
        validation_times = [e["t"] for e in self.events if e.get("ev") == "stage_start"
                            and e.get("stage") == "validation"]
        self.stage_times = [
            ("extract", self.start_time),
            ("lidar_odometry", starts.get("lo", self.start_time + .08 * self.duration_s)),
            ("rgb_ba", starts.get("rgb_tracks", self.start_time + .25 * self.duration_s)),
            ("thermal", starts.get("thermal_solve", self.start_time + .7 * self.duration_s)),
            ("validation", max(validation_times) if validation_times else self.end_time - 1),
        ]
        validation_ends = [e["t"] for e in self.events if e.get("ev") == "stage_end"
                           and e.get("stage") == "validation" and e.get("ok")]
        self.validation_end_time = max(validation_ends) if validation_ends else self.end_time
        candidates = sorted((self.workdir / "lo").glob("ref_*.npz"))
        if not candidates:
            raise FileNotFoundError("replay needs lo/ref_<window>.npz")
        sources = {path.stem[4:]: path for path in candidates
                   if (self.workdir / "extract" / path.stem[4:] / "lidar").is_dir()}
        self.replay_windows = [name for name in self.window_names if name in sources]
        self.replay_windows += [name for name in sources if name not in self.replay_windows]
        if not self.replay_windows:
            raise FileNotFoundError("replay needs extracted raw LiDAR NPY frames")
        self._window_sources = sources
        digest = hashlib.sha256((str(self.workdir) + str(self.end_time)).encode()).hexdigest()[:12]
        self.manifest = {
            "schema": SCHEMA, "run_id": "replay-" + digest, "mode": "replay",
            "parent_frame": "os_lidar", "display_frame": "vehicle",
            "R_lidar_V": self.R_lidar_V.tolist(), "cameras": self.cameras,
            "preview_cameras": list(PREVIEW_CAMERAS),
            "map_frame": "vehicle_aligned_window:" + self.replay_windows[0],
            "windows": self.replay_windows,
            "source_workdir": str(self.workdir), "source_outdir": str(self.out_dir),
            "source_duration_s": self.duration_s, "replay_speed": self.speed,
            "rate_limits": {"snapshots_hz": 4, "map_points": ASSET_STEPS * POINTS_PER_SWEEP,
                            "preview_points": PREVIEW_POINTS, "asset_sets": MAX_ASSET_SETS,
                            "samples_per_window": ASSET_STEPS,
                            "max_snapshots_per_replay": MAX_SNAPSHOTS},
            "provenance": {
                "poses": "synthetic perturbation and decaying noise, exact measured final extrinsics",
                "metrics": "synthetic intermediate uncertainty, measured final empirical 1sigma and reprojection",
                "timing": "real events.jsonl wall clock, including source restarts, divided by replay_speed",
                "map": "all available windows, sampled independently with LO deskew; no unregistered fusion",
                "tracks": "real image tracks; not certified image-to-LiDAR correspondences",
                "thermal": "real 16-bit image normalized; center-row header+offset approximation; rolling shutter omitted",
            },
        }
        existing_manifest = self.output_dir / "manifest.json"
        if existing_manifest.exists():
            previous = _read_json(existing_manifest)
            if not isinstance(previous, dict) or any(previous.get(key) != self.manifest[key] for key in
                   ("run_id", "source_workdir", "source_outdir", "R_lidar_V", "cameras")):
                raise ValueError("output contains a different replay; choose a fresh output directory")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "assets").mkdir(exist_ok=True)
        # Only replay-owned directories are retired; unrelated stream files stay intact.
        self._retired_assets = deque(sorted(
            (path for path in (self.output_dir / "assets").glob("replay_*")
             if path.is_dir() and not path.is_symlink()), key=lambda path: path.stat().st_mtime_ns))
        self._trim_assets()
        atomic_json(self.output_dir / "manifest.json", self.manifest)
        self.prepared = True
        return self.manifest

    def _activate_window(self, name):
        if name == self.window_name:
            return
        with np.load(self._window_sources[name], allow_pickle=False) as archive:
            tau, poses = archive["tau"], archive["T_w_L"]
            take = np.linspace(0, len(tau) - 1, min(len(tau), MAX_TRAJECTORY_POSES), dtype=int)
            self._tau, self._poses = tau[take], poses[take]
        if not len(self._tau):
            raise ValueError("empty trajectory for " + name)
        self._lidar_files = sorted((self.workdir / "extract" / name / "lidar").glob("*.npy"))
        if not self._lidar_files:
            raise FileNotFoundError("no raw LiDAR frames for " + name)
        self._lidar_stamps = np.array([int(path.stem) for path in self._lidar_files], np.int64)
        self._indices = np.unique(np.linspace(0, len(self._lidar_files) - 1, ASSET_STEPS, dtype=int))
        self._sample_times = self._lidar_stamps[self._indices] + 50_000_000
        self.window_name = name
        self._camera_frames = {}
        self._camera_tracks = {}
        self._map_chunks = {}

    def _trim_assets(self):
        while len(self._retired_assets) + len(self.assets) > MAX_ASSET_SETS:
            if self._retired_assets:
                directory = self._retired_assets.popleft()
            else:
                _, (_, directory) = self.assets.popitem(last=False)
            shutil.rmtree(directory)

    def _pose_at(self, stamp):
        if len(self._tau) == 1:
            return self._poses[0].copy()
        idx = int(np.clip(np.searchsorted(self._tau, stamp) - 1, 0, len(self._tau) - 2))
        alpha = (float(stamp) - float(self._tau[idx])) / float(self._tau[idx + 1] - self._tau[idx])
        return interpolate_pose(self._poses[idx], self._poses[idx + 1], alpha)

    def _world_points(self, path, limit):
        points, offsets = _sample_scan(path, limit)
        result = np.empty_like(points, dtype=float)
        # 32 bins per 100ms sweep: bounded work and <=1.6ms deskew quantization.
        bins = np.rint(offsets * 320).astype(int)
        for bucket in np.unique(bins):
            mask = bins == bucket
            T = self._pose_at(int(path.stem) + bucket / 320 * 1e9)
            result[mask] = points[mask] @ T[:3, :3].T + T[:3, 3]
        return result

    def _tracks_for_frames(self, camera, selected_stamps):
        folder = "thermal/tracks" if camera.startswith("thermal") else "tracks"
        path = self.workdir / folder / ("tracks_" + self.window_name + "_" + camera + ".npz")
        empty = lambda: (np.empty((0, 2), np.float32), np.empty((0, 2), np.float32))
        if not path.exists():
            return [empty() for _ in selected_stamps]
        # One bounded archive at a time; retain only sampled observations for
        # this window. Missing/oversized tracks do not prevent image previews.
        with zipfile.ZipFile(path) as archive:
            if sum(item.file_size for item in archive.infolist()) > MAX_TRACK_BYTES:
                return [empty() for _ in selected_stamps]
        with np.load(path, allow_pickle=False) as archive:
            stamps = archive["header_ns" if camera.startswith("thermal") else "frame_ns"]
            frames, ids, xy = archive["obs_frame"], archive["obs_track"], archive["obs_xy"]
            result = []
            if not len(stamps):
                return [empty() for _ in selected_stamps]
            for stamp in selected_stamps:
                frame = int(np.argmin(np.abs(stamps - stamp)))
                if abs(int(stamps[frame]) - stamp) > 1_000_000:
                    result.append(empty())
                    continue
                # Same track IDs in consecutive observed frames, no invented matches.
                now, prev = np.flatnonzero(frames == frame), np.flatnonzero(frames == frame - 1)
                _, cur_idx, old_idx = np.intersect1d(ids[now], ids[prev], return_indices=True)
                take = np.linspace(0, len(cur_idx) - 1, min(180, len(cur_idx)), dtype=int)
                result.append((xy[now[cur_idx[take]]].copy(), xy[prev[old_idx[take]]].copy()))
            return result

    def _write_image(self, relative, image):
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if not ok:
            raise ValueError("could not encode replay image")
        path = self.output_dir / relative
        temporary = path.with_suffix(".jpg.tmp")
        temporary.write_bytes(encoded.tobytes())
        os.replace(temporary, path)

    def _assets_at(self, window, step):
        key = (window, step)
        if key in self.assets:
            self.assets.move_to_end(key)
            return self.assets[key][0]
        self._activate_window(window)
        step = min(step, len(self._indices) - 1)
        directory = self.output_dir / "assets" / ("replay_" + uuid.uuid4().hex)
        directory.mkdir()
        relative = directory.relative_to(self.output_dir).as_posix()
        asset = {"matching": {}}
        try:
            for index in range(step + 1):
                if index not in self._map_chunks:
                    points = self._world_points(self._lidar_files[self._indices[index]], POINTS_PER_SWEEP)
                    # W is this window's first LiDAR frame. Never fuse windows.
                    self._map_chunks[index] = (points @ self.R_lidar_V).astype(np.float32)
            end = max(1, int(np.searchsorted(self._tau, self._sample_times[step], side="right")))
            trajectory = (self._poses[:end, :3, 3] @ self.R_lidar_V).astype(np.float32)
            asset["map"] = relative + "/map.npz"
            atomic_npz(self.output_dir / asset["map"],
                       points=np.vstack([self._map_chunks[i] for i in range(step + 1)]),
                       trajectory=trajectory)
            world_scans = {}
            for camera in PREVIEW_CAMERAS:
                thermal = camera.startswith("thermal")
                if camera not in self._camera_frames:
                    folder = (self.workdir / "thermal16" / window / camera if thermal else
                              self.workdir / "extract" / window / "cam" / camera)
                    files = sorted(folder.glob("*.png" if thermal else "*.jpg"))
                    stamps = np.array([int(path.stem) for path in files], np.int64)
                    self._camera_frames[camera] = files, stamps
                files, stamps = self._camera_frames[camera]
                if not files:
                    continue
                offset = self.cameras[camera]["time_offset_s"] * 1e9
                chosen = [int(np.argmin(np.abs(stamps.astype(float) + offset - t))) for t in self._sample_times]
                if camera not in self._camera_tracks:
                    self._camera_tracks[camera] = self._tracks_for_frames(camera, [int(stamps[i]) for i in chosen])
                selected = chosen[step]
                image = cv2.imread(str(files[selected]), cv2.IMREAD_UNCHANGED)
                if image is None:
                    continue
                if thermal:
                    lo, hi = np.percentile(image, [2, 98])
                    gray = np.clip((image.astype(float) - lo) * (255 / max(hi - lo, 1)), 0, 255).astype(np.uint8)
                    image = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
                else:
                    image = cv2.LUT(image, np.array([(i / 255) ** .55 * 255 for i in range(256)], np.uint8))
                width, height = self.cameras[camera]["width"], self.cameras[camera]["height"]
                scale = min(1., 960 / width)
                image = cv2.resize(image, (round(width * scale), round(height * scale)), interpolation=cv2.INTER_AREA)
                stem = relative + "/" + camera
                self._write_image(stem + ".jpg", image)
                stamp = int(stamps[selected])
                capture_stamp = stamp + offset
                scan_idx = int(np.argmin(np.abs(self._lidar_stamps.astype(float) + 50_000_000 - capture_stamp)))
                if scan_idx not in world_scans:
                    world_scans[scan_idx] = self._world_points(self._lidar_files[scan_idx], PREVIEW_POINTS)
                T = self._pose_at(capture_stamp)
                points_lidar = ((world_scans[scan_idx] - T[:3, 3]) @ T[:3, :3]).astype(np.float32)
                tracks = self._camera_tracks[camera][step]
                arrays = {"points_lidar": points_lidar,
                          "tracks_uv": (tracks[0] * scale).astype(np.float32),
                          "tracks_prev_uv": (tracks[1] * scale).astype(np.float32)}
                mask_path = self.workdir / "masks" / (camera + ".png")
                if mask_path.exists():
                    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
                    if mask is not None:
                        arrays["mask"] = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
                atomic_npz(self.output_dir / (stem + ".npz"), **arrays)
                asset["matching"][camera] = {
                    "image": stem + ".jpg", "points": stem + ".npz",
                    "width": image.shape[1], "height": image.shape[0],
                    "source_stamp_ns": stamp, "capture_stamp_ns": int(capture_stamp),
                    "lidar_stamp_ns": int(self._lidar_stamps[scan_idx]),
                    "source_window": window,
                    "image_processing": "thermal percentile+inferno" if thermal else "gamma 0.55",
                    "points_frame": "os_lidar_at_image_capture",
                    "time_model": "header+center_offset; no row readout" if thermal else "effective exposure",
                }
        except Exception:
            shutil.rmtree(directory)
            raise
        self.assets[key] = (asset, directory)
        self._trim_assets()
        return asset

    def snapshot_at(self, fraction, window=None):
        """Deterministic seek; optional window selects its local fraction 0..1."""
        if not self.prepared:
            self.prepare()
        fraction = float(np.clip(fraction, 0, 1))
        source_time = self.start_time + fraction * self.duration_s
        stage = "extract"
        for name, at in self.stage_times:
            if source_time >= at:
                stage = name
        cameras = {}
        rgb_start = self.stage_times[2][1]
        thermal_start = self.stage_times[3][1]
        solve_end = self.stage_times[4][1]
        for index, (name, camera) in enumerate(self.cameras.items()):
            start = thermal_start if camera["sensor"] == "thermal" else rgb_start
            finish = solve_end if camera["sensor"] == "thermal" else thermal_start
            progress = float(np.clip((source_time - start) / max(finish - start, 1), 0, 1))
            # Reproducible, changing-axis residual models settling between solver steps.
            decay = ((math.exp(-4.5 * progress) - math.exp(-4.5)) /
                     (1 - math.exp(-4.5))) if progress < 1 else 0.
            phase = index * 1.73
            axis = np.array([math.sin(phase + .6), math.cos(phase), math.sin(phase * .7 + 1)])
            axis /= np.linalg.norm(axis)
            rotvec = axis * math.radians(7 + index % 4) * decay
            rotvec += .012 * decay * np.sin(np.arange(3) + phase + progress * 35)
            perturb, _ = cv2.Rodrigues(rotvec)
            final = np.asarray(camera["T_cam_lidar_final"], float)
            pose_V = vehicle_from_camera(final, self.R_lidar_V)
            pose_V[:3, :3] = perturb @ pose_V[:3, :3]
            pose_V[:3, 3] += decay * np.array([.11 * math.sin(phase + 1), .13 * math.cos(phase), .08 * math.sin(phase + 2)])
            T_lidar_V = np.eye(4)
            T_lidar_V[:3, :3] = self.R_lidar_V
            current = np.linalg.inv(T_lidar_V @ pose_V)
            if progress == 1:
                current = final.copy()  # preserve the exact measured endpoint
            metrics = self.metrics[name]
            vote = metrics.get("vote", {}).get("pass")
            result = camera_gate(self.summary, self.metrics, name)
            gate_pass = result["pass"] if source_time >= self.validation_end_time else None
            cameras[name] = {
                "T_cam_lidar": current.tolist(),
                "sigma_rot_deg": metrics.get("rot_deg", 0) + (2.8 + index % 3 * .3) * decay,
                "sigma_pos_mm": metrics.get("pos_mm", 0) + (110 + index % 4 * 13) * decay,
                "reprojection_px": metrics.get("track_reproj_px", 0) + (22 + index % 5 * 2) * decay,
                "state": ("failed" if gate_pass is False else "validated" if gate_pass is True else
                          "pending" if progress == 0 else "converging" if progress < 1 else "converged"),
                "validation_vote": vote if source_time >= self.validation_end_time else None,
                "gate_pass": gate_pass,
                "gate_reasons": result["reasons"] if source_time >= self.validation_end_time else [],
                "informational_checks": result["informational"] if source_time >= self.validation_end_time else [],
                "metric_source": "measured_final" if progress == 1 else "synthetic_replay",
            }
        failed_votes = [name for name, metric in self.metrics.items() if metric.get("vote", {}).get("pass") is False]
        validated = source_time >= self.validation_end_time
        gate_pass = self.final_gate.get("pass") if validated else None
        if window is None:
            window_index = min(len(self.replay_windows) - 1, int(fraction * len(self.replay_windows)))
            window = self.replay_windows[window_index]
            map_progress = min(1., fraction * len(self.replay_windows) - window_index)
        else:
            if window not in self._window_sources:
                raise ValueError("unknown replay window: " + window)
            map_progress = fraction
        self._activate_window(window)
        map_step = min(len(self._indices) - 1, int(map_progress * len(self._indices)))
        assets = self._assets_at(window, map_step)
        # Window counts reflect observed source completions, not simulated
        # fractional progress. Joint BA operates on all LO-completed windows.
        counter_stage = "extract" if stage == "extract" else "lo"
        completed_windows = set()
        for event in self.events:
            if event["t"] > source_time:
                break
            if event.get("stage") == counter_stage and event.get("ev") == "stage_progress":
                name = event.get("msg", "").split(" ")[0]
                if name in self.window_names:
                    completed_windows.add(name)
        return {
            "schema": SCHEMA, "seq": int(round(fraction * 10000)), "t": source_time,
            "source_time_s": source_time - self.start_time, "progress": fraction,
            "stage": stage, "window": len(completed_windows),
            "total_windows": self.total_windows, "eta_s": (1 - fraction) * self.duration_s / self.speed,
            "cameras": cameras,
            "gate": {"status": "pending" if gate_pass is None else ("pass" if gate_pass else "fail"),
                     "pass": gate_pass, "source": "summary.validation.gate",
                     "camera_vote_failures": failed_votes if validated else [],
                     "warning": ("RGB 영상 투표 %d개 미통과 · 종합 게이트와 별도" % len(failed_votes)) if validated and failed_votes else ""},
            "assets": assets,
            "map_frame": "vehicle_aligned_window:" + window, "map_window": window,
            "map_progress": float(map_progress),
            "provenance": {"mode": "replay", "synthetic_intermediate": True,
                           "label_ko": "실측 데이터 · 수렴 과정 모의 재생",
                           "map_label_ko": window + " 구간 표본 지도 · %d개 창 순회 · 창별 독립 좌표" % len(self.replay_windows)},
        }

    def run(self, stop_event=None):
        stop_event = stop_event or threading.Event()
        self.prepare()
        start = time.monotonic()
        seq = 0
        # At least one snapshot per window, even when requested speed would
        # otherwise jump from the first window straight to the final window.
        base_count = min(max(2, MAX_SNAPSHOTS - len(self.replay_windows)),
                         max(2, math.ceil(self.duration_s / self.speed * 4) + 1))
        fractions = sorted(set(np.linspace(0, 1, base_count)) |
                           {(index + .999) / len(self.replay_windows)
                            for index in range(len(self.replay_windows))})
        replay_duration = max(self.duration_s / self.speed, (len(fractions) - 1) * .25)
        # Replay can append after a previous invocation while keeping seq monotonic.
        existing = self.output_dir / "events.jsonl"
        if existing.exists():
            with existing.open("rb") as handle:
                handle.seek(max(0, existing.stat().st_size - 256 * 1024))
                for line in handle:
                    try:
                        seq = max(seq, int(json.loads(line).get("seq", -1)) + 1)
                    except (ValueError, TypeError, AttributeError):
                        pass
        for index, fraction in enumerate(fractions):
            if stop_event.is_set():
                return
            if index and stop_event.wait(max(.25, start + fraction * replay_duration - time.monotonic())):
                return
            snapshot = self.snapshot_at(fraction)
            snapshot["seq"] = seq
            append_snapshot(self.output_dir, snapshot)
            seq += 1


def main(argv=None):
    parser = argparse.ArgumentParser(description="실제 데이터 기반 보정 시각화 모의 스트림")
    parser.add_argument("--workdir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--out-dir")
    parser.add_argument("--speed", type=float, default=60)
    args = parser.parse_args(argv)
    ReplayProducer(args.workdir, args.output, args.out_dir, args.speed).run()


if __name__ == "__main__":
    main()
