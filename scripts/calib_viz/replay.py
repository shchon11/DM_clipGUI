"""A deterministic, explicitly synthetic convergence replay over real assets.

No solver is imported or executed. Only small source frames are mapped; large
``lo/map_*.npz`` archives are never loaded. Twelve source sweeps and three camera
previews keep both replay disk use and memory bounded independently of run size.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import yaml

from . import SCHEMA
from .geometry import interpolate_pose, vehicle_from_camera
from .stream import append_snapshot, atomic_json, atomic_npz

CAMERA_NAMES = (["camera_front" + str(i) for i in range(1, 10)] +
                ["camera_top", "camera_side_left", "camera_side_right",
                 "camera_rear_left", "camera_rear_right", "thermal_left", "thermal_right"])
PREVIEW_CAMERAS = ("camera_front5", "camera_side_left", "thermal_left")
ASSET_STEPS = 12
POINTS_PER_SWEEP = 4800
PREVIEW_POINTS = 7000
MAX_SNAPSHOTS = 4096


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

    ``prepare`` writes the manifest and a finite asset pool. ``snapshot_at``
    supports deterministic screenshot/seek without mutating the event log.
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
        self.speed = float(speed)
        self.manifest = None
        self.assets = []
        self.prepared = False

    def prepare(self):
        if self.prepared:
            return self.manifest
        rig = _read_yaml(self.out_dir / "rig.yaml")
        self.metrics = _read_json(self.out_dir / "metrics.json")
        summary = _read_json(self.out_dir / "summary.json")
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
        preferred = self.workdir / "lo/ref_S01.npz"
        trajectory_path = preferred if preferred.exists() else candidates[0]
        self.window_name = trajectory_path.stem[4:]
        with np.load(trajectory_path, allow_pickle=False) as archive:
            self._tau = archive["tau"]
            self._poses = archive["T_w_L"]
        self._lidar_files = sorted((self.workdir / "extract" / self.window_name / "lidar").glob("*.npy"))
        if not self._lidar_files:
            raise FileNotFoundError("replay needs extracted raw LiDAR NPY frames")
        self._lidar_stamps = np.array([int(p.stem) for p in self._lidar_files], dtype=np.int64)
        digest = hashlib.sha256((str(self.workdir) + str(self.end_time)).encode()).hexdigest()[:12]
        self.manifest = {
            "schema": SCHEMA, "run_id": "replay-" + digest, "mode": "replay",
            "parent_frame": "os_lidar", "display_frame": "vehicle",
            "R_lidar_V": self.R_lidar_V.tolist(), "cameras": self.cameras,
            "preview_cameras": list(PREVIEW_CAMERAS),
            "map_frame": "vehicle_aligned_window:" + self.window_name,
            "source_workdir": str(self.workdir), "source_outdir": str(self.out_dir),
            "source_duration_s": self.duration_s, "replay_speed": self.speed,
            "rate_limits": {"snapshots_hz": 4, "map_points": ASSET_STEPS * POINTS_PER_SWEEP,
                            "preview_points": PREVIEW_POINTS, "asset_sets": ASSET_STEPS,
                            "max_snapshots_per_replay": MAX_SNAPSHOTS},
            "provenance": {
                "poses": "synthetic perturbation and decaying noise, exact measured final extrinsics",
                "metrics": "synthetic intermediate uncertainty, measured final empirical 1sigma and reprojection",
                "timing": "real events.jsonl wall clock, including source restarts, divided by replay_speed",
                "map": "representative " + self.window_name + " only; sampled real sweeps with LO deskew",
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
        self._prepare_assets()
        atomic_json(self.output_dir / "manifest.json", self.manifest)
        self.prepared = True
        return self.manifest

    def _pose_at(self, stamp):
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
        # One track archive at a time, only the three selected previews.
        with np.load(path, allow_pickle=False) as archive:
            stamps = archive["header_ns" if camera.startswith("thermal") else "frame_ns"]
            frames, ids, xy = archive["obs_frame"], archive["obs_track"], archive["obs_xy"]
            result = []
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

    def _prepare_assets(self):
        indices = np.linspace(0, len(self._lidar_files) - 1, ASSET_STEPS, dtype=int)
        self._sample_times = self._lidar_stamps[indices] + 50_000_000
        map_chunks = []
        self.assets = [{"matching": {}} for _ in indices]
        for step, idx in enumerate(indices):
            points = self._world_points(self._lidar_files[idx], POINTS_PER_SWEEP)
            # W is the window's first LiDAR frame, rotated once into vehicle axes.
            map_chunks.append((points @ self.R_lidar_V).astype(np.float32))
            end = max(2, int(np.searchsorted(self._tau, self._sample_times[step], side="right")))
            trajectory = (self._poses[:end, :3, 3] @ self.R_lidar_V).astype(np.float32)
            relative = "assets/map_%02d.npz" % step
            atomic_npz(self.output_dir / relative, points=np.vstack(map_chunks), trajectory=trajectory)
            self.assets[step]["map"] = relative
        for camera in PREVIEW_CAMERAS:
            thermal = camera.startswith("thermal")
            folder = (self.workdir / "thermal16" / self.window_name / camera if thermal else
                      self.workdir / "extract" / self.window_name / "cam" / camera)
            files = sorted(folder.glob("*.png" if thermal else "*.jpg"))
            if not files:
                continue
            stamps = np.array([int(p.stem) for p in files], np.int64)
            offset = self.cameras[camera]["time_offset_s"] * 1e9
            chosen = [int(np.argmin(np.abs(stamps.astype(float) + offset - t))) for t in self._sample_times]
            tracks = self._tracks_for_frames(camera, [int(stamps[i]) for i in chosen])
            mask_path = Path("/hdd/DM_calib/online/work/masks") / (camera + ".png")
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE) if mask_path.exists() else None
            for step, selected in enumerate(chosen):
                image = cv2.imread(str(files[selected]), cv2.IMREAD_UNCHANGED)
                if image is None:
                    continue
                if thermal:
                    lo, hi = np.percentile(image, [2, 98])
                    gray = np.clip((image.astype(float) - lo) * (255 / max(hi - lo, 1)), 0, 255).astype(np.uint8)
                    image = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
                else:
                    # Record was collected at night: same gamma lift as source report.
                    image = cv2.LUT(image, np.array([(i / 255) ** .55 * 255 for i in range(256)], np.uint8))
                width, height = self.cameras[camera]["width"], self.cameras[camera]["height"]
                scale = min(1., 960 / width)
                image = cv2.resize(image, (round(width * scale), round(height * scale)), interpolation=cv2.INTER_AREA)
                stem = "assets/" + camera + "_%02d" % step
                self._write_image(stem + ".jpg", image)
                stamp = int(stamps[selected])
                capture_stamp = stamp + offset
                scan_idx = int(np.argmin(np.abs(self._lidar_stamps.astype(float) + 50_000_000 - capture_stamp)))
                world = self._world_points(self._lidar_files[scan_idx], PREVIEW_POINTS)
                T = self._pose_at(capture_stamp)
                points_lidar = ((world - T[:3, 3]) @ T[:3, :3]).astype(np.float32)
                arrays = {"points_lidar": points_lidar,
                          "tracks_uv": (tracks[step][0] * scale).astype(np.float32),
                          "tracks_prev_uv": (tracks[step][1] * scale).astype(np.float32)}
                if mask is not None:
                    arrays["mask"] = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
                atomic_npz(self.output_dir / (stem + ".npz"), **arrays)
                self.assets[step]["matching"][camera] = {
                    "image": stem + ".jpg", "points": stem + ".npz",
                    "width": image.shape[1], "height": image.shape[0],
                    "source_stamp_ns": stamp, "capture_stamp_ns": int(capture_stamp),
                    "lidar_stamp_ns": int(self._lidar_stamps[scan_idx]),
                    "source_window": self.window_name,
                    "image_processing": "thermal percentile+inferno" if thermal else "gamma 0.55",
                    "points_frame": "os_lidar_at_image_capture",
                    "time_model": "header+center_offset; no row readout" if thermal else "effective exposure",
                }

    def snapshot_at(self, fraction):
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
            cameras[name] = {
                "T_cam_lidar": current.tolist(),
                "sigma_rot_deg": metrics.get("rot_deg", 0) + (2.8 + index % 3 * .3) * decay,
                "sigma_pos_mm": metrics.get("pos_mm", 0) + (110 + index % 4 * 13) * decay,
                "reprojection_px": metrics.get("track_reproj_px", 0) + (22 + index % 5 * 2) * decay,
                "state": "pending" if progress == 0 else ("converging" if progress < 1 else "converged"),
                "validation_vote": vote if source_time >= self.validation_end_time else None,
                "metric_source": "measured_final" if progress == 1 else "synthetic_replay",
            }
        failed_votes = [name for name, metric in self.metrics.items() if metric.get("vote", {}).get("pass") is False]
        validated = source_time >= self.validation_end_time
        gate_pass = self.final_gate.get("pass") if validated else None
        map_progress = np.clip((source_time - self.stage_times[1][1]) /
                               max(rgb_start - self.stage_times[1][1], 1), 0, 1)
        map_step = min(ASSET_STEPS - 1, int(map_progress * ASSET_STEPS))
        preview_step = min(ASSET_STEPS - 1, int(fraction * ASSET_STEPS))
        matching = self.assets[preview_step]["matching"]
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
            "assets": {"map": self.assets[map_step]["map"], "matching": matching},
            "map_frame": self.manifest["map_frame"], "map_window": self.window_name,
            "map_progress": float(map_progress),
            "provenance": {"mode": "replay", "synthetic_intermediate": True,
                           "label_ko": "실측 데이터 · 수렴 과정 모의 재생",
                           "map_label_ko": self.window_name + " 구간 대표 지도 · 전체 주행 병합 아님"},
        }

    def run(self, stop_event=None):
        stop_event = stop_event or threading.Event()
        self.prepare()
        start = time.monotonic()
        seq = 0
        interval = max(.25, self.duration_s / self.speed / (MAX_SNAPSHOTS - 1))
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
        while not stop_event.is_set():
            fraction = min(1., (time.monotonic() - start) * self.speed / self.duration_s)
            snapshot = self.snapshot_at(fraction)
            snapshot["seq"] = seq
            append_snapshot(self.output_dir, snapshot)
            seq += 1
            if fraction >= 1:
                return
            stop_event.wait(interval)


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
