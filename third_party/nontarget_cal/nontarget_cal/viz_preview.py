"""Small real-data previews, evaluated only on the visualisation I/O thread.

Raw scans are mmap-sampled before deskew; cached LO is small and full map archives are
never opened. RGB filenames already encode exposure midpoint. Thermal uses the solver's
clock model, offset and (when available) iterative row-readout compensation.
"""
from __future__ import annotations

import heapq
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile

import cv2
import numpy as np

from .lo.lotraj import LOTraj, deskew

MAX_TRACK_BYTES = 32 * 1024 * 1024
MAX_PREVIEW_SOURCES = 16
MAX_LIDAR_FILES = 4096
MAX_TRAJECTORY_BYTES = 4 * 1024 * 1024


def _small_npz(path):
    with zipfile.ZipFile(path) as archive:
        if sum(info.file_size for info in archive.infolist()) > MAX_TRACK_BYTES:
            raise ValueError("preview archive exceeds 32 MiB")
    return np.load(path, allow_pickle=False)


def _tracks(path, stamp, thermal):
    empty = np.empty((0, 2), np.float32)
    if not path.exists():
        return empty, empty.copy()
    with _small_npz(path) as z:
        stamps = z["header_ns" if thermal else "frame_ns"]
        if not len(stamps):
            return empty, empty.copy()
        frame = int(np.argmin(np.abs(stamps - stamp)))
        if abs(int(stamps[frame]) - stamp) > 1_000_000:
            return empty, empty.copy()
        obs_frame, ids, xy = z["obs_frame"], z["obs_track"], z["obs_xy"]
        now = np.flatnonzero(obs_frame == frame)
        prev = np.flatnonzero(obs_frame == frame - 1)
        _, cur, old = np.intersect1d(ids[now], ids[prev], return_indices=True)
        take = np.linspace(0, len(cur) - 1, min(180, len(cur)), dtype=int)
        return xy[now[cur[take]]].copy(), xy[prev[old[take]]].copy()



def _bag_id(req):
    """Resolve the requested window, never inherit another window's bag clock."""
    context, window = req.get("context", {}), req["window"]
    for record in context.get("windows", []):
        if isinstance(record, dict) and record.get("name") == window:
            return record["bag_id"]
    plan_path = Path(req["workdir"]) / "windows" / "plan.json"
    if plan_path.exists():
        plan = json.loads(plan_path.read_text())
        for record in plan.get("windows", []):
            if record.get("name") == window:
                return record["bag_id"]
    return context.get("bag_id") if context.get("window_id") == window else None


def _source(req, cfg):
    root, window, camera = Path(req["workdir"]), req["window"], req["camera"]
    thermal = camera.startswith("thermal")
    directory = root / "thermal16" / window / camera if thermal else root / "extract" / window / "cam" / camera
    lo_path = root / "lo" / ("ref_" + window + ".npz")
    if not lo_path.exists():
        lo_path = root / "lo" / ("kiss_" + window + ".npz")
    if not lo_path.exists():
        return None
    with zipfile.ZipFile(lo_path) as archive:
        if sum(info.file_size for info in archive.infolist()) > MAX_TRAJECTORY_BYTES:
            return None
    tr = LOTraj.load(lo_path)
    mid = (int(tr.tau[0]) + int(tr.tau[-1])) // 2
    chosen = min(directory.glob("*.png" if thermal else "*.jpg"),
                 key=lambda f: abs(int(f.stem) - mid), default=None)
    if chosen is None:
        return None
    stamp = int(chosen.stem)
    image = cv2.imread(str(chosen), cv2.IMREAD_UNCHANGED)
    if image is None:
        return None
    height, width = image.shape[:2]
    if thermal:
        lo, hi = np.percentile(image, [2, 98])
        gray = np.clip((image.astype(np.float32) - lo) * (255 / max(hi - lo, 1)), 0, 255).astype(np.uint8)
        image = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
    else:
        image = cv2.LUT(image, np.array([(i / 255) ** .55 * 255 for i in range(256)], np.uint8))
    scale = min(1., min(960, max(1, int(cfg.get("viz", {}).get("image_width", 960)))) / max(width, height))
    image = cv2.resize(image, (round(width * scale), round(height * scale)), interpolation=cv2.INTER_AREA)
    okay, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not okay:
        raise ValueError("cannot encode live camera preview")
    tracks_path = root / ("thermal/tracks" if thermal else "tracks") / ("tracks_" + window + "_" + camera + ".npz")
    try:
        tracks, prev = _tracks(tracks_path, stamp, thermal)
    except (ValueError, OSError):
        tracks = prev = np.empty((0, 2), np.float32)
    arrays = {"tracks_uv": (tracks * scale).astype(np.float32), "tracks_prev_uv": (prev * scale).astype(np.float32)}
    mask_dir = cfg.get("paths", {}).get("masks_dir")
    if mask_dir:
        mask_path = Path(mask_dir) / (camera + ".png")
        if mask_path.exists():
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if mask is not None:
                arrays["mask"] = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
    source_stamp, base_stamp = stamp, stamp
    time_kind = "exposure_midpoint"
    bag_id = _bag_id(req)
    if thermal:
        from .thermal.timemodel import t_frame
        if not bag_id:
            raise ValueError("thermal preview has no bag identity for " + window)
        time_kind = req["camera_state"].get("time_model", cfg.get("thermal", {}).get("time_model", "smooth"))
        base_stamp = float(t_frame(SimpleNamespace(root=root), bag_id, camera, np.array([stamp]), kind=time_kind)[0])
    else:
        index = root / "extract" / window / "cam_index.npz"
        if index.exists():
            with _small_npz(index) as z:
                hit = np.flatnonzero((z["channel"] == camera) & (z["effective_ns"] == stamp))
                if len(hit):
                    source_stamp = int(z["header_ns"][hit[0]])
    lidar_files = heapq.nsmallest(MAX_LIDAR_FILES, (root / "extract" / window / "lidar").glob("*.npy"),
                                 key=lambda p: abs(int(p.stem) - mid))
    if not lidar_files:
        return None
    return {"image": encoded.tobytes(), "arrays": arrays, "trajectory": tr, "lidar_files": lidar_files,
            "base_stamp": base_stamp, "width": width, "height": height,
            "meta": {"width": image.shape[1], "height": image.shape[0], "source_stamp_ns": source_stamp,
                     "source_window": window, "bag_id": bag_id, "time_model": time_kind,
                     "image_processing": "thermal percentile+inferno" if thermal else "gamma 0.55"}}


def _thermal_rows(points, state, height):
    T = np.asarray(state["T_cam_lidar"], float)
    p = points @ T[:3, :3].T + T[:3, 3]
    z = np.where(np.abs(p[:, 2]) > 1e-6, p[:, 2], 1e-6)
    x, y = p[:, 0] / z, p[:, 1] / z
    D = list(state.get("D", [0., 0.])) + [0.] * 5
    r2 = x*x + y*y
    radial = 1 + D[0] * r2 + D[1] * r2*r2 + D[4] * r2*r2*r2
    yd = y * radial + D[2] * (r2 + 2*y*y) + 2*D[3]*x*y
    K = np.asarray(state["K"], float)
    return np.clip((K[1, 1] * yd + K[1, 2]) / height - .5, -.5, .5)


def build_preview(req, cfg, cache):
    """Return encoded JPG, detached small NPZ arrays, and truthful source metadata."""
    state = req["camera_state"]
    key = (req["workdir"], req["window"], req["camera"], state.get("time_model"))
    if key not in cache:
        source = _source(req, cfg)
        if source is None:
            return None
        # At most one pack per canonical camera stays hot during round-robin.
        # Retain only sampled tracks/clouds, never the decoded full track archive.
        if len(cache) >= MAX_PREVIEW_SOURCES:
            cache.pop(next(iter(cache)))
        cache[key] = source
    source = cache.pop(key)
    cache[key] = source              # insertion order is the small LRU
    thermal = req["camera"].startswith("thermal")
    offset = float(state.get("time_offset_by_window_s", {}).get(req["window"],
                       state.get("time_offset_s", state.get("dt_s", 0.))))
    capture = source["base_stamp"] + (offset * 1e9 if thermal else int(round(offset * 1e9)))
    path = min(source["lidar_files"], key=lambda p: abs(int(p.stem) + 50_000_000 - capture))
    if source.get("scan_path") != path:
        scan = np.load(path, mmap_mode="r", allow_pickle=False)
        step = max(1, int(np.ceil(len(scan) / min(7000, max(1, int(cfg.get("viz", {}).get("image_points", 7000)))))))
        sample = np.array(scan[::step], copy=True)
        radius = np.sqrt(sample["x"] ** 2 + sample["y"] ** 2 + sample["z"] ** 2)
        sample = sample[np.isfinite(radius) & (radius > 2.5) & (radius < 100.)]
        source["world"] = deskew(sample, int(path.stem), source["trajectory"])
        source["scan_path"] = path
    world, tr = source["world"], source["trajectory"]
    T = tr.Tm(capture)[0]
    points = (world - T[:3, 3]) @ T[:3, :3]
    rs = float(state.get("row_readout_s", state.get("rs_s", 0.))) if thermal else 0.
    if rs and len(points) and "K" in state and "T_cam_lidar" in state:
        # Solve exposure row with the CURRENT calibration, bounded to 3 fixed passes.
        for _ in range(3):
            stamps = capture + _thermal_rows(points, state, source["height"]) * rs * 1e9
            R, p = tr.R(stamps), tr.p(stamps)
            points = np.einsum("nji,nj->ni", R, world - p)
    arrays = {**source["arrays"], "points_lidar": points.astype(np.float32)}
    meta = {**source["meta"], "capture_stamp_ns": int(capture), "lidar_stamp_ns": int(path.stem),
            "points_frame": "os_lidar_at_image_row_capture" if rs else "os_lidar_at_image_capture",
            "row_readout_s": rs, "time_offset_s": offset, "row_projection_iterations": 3 if rs else 0}
    return source["image"], arrays, meta


def build_cached_map(req, cfg):
    """Three deterministic raw sweeps with actual cached LO, never map_*.npz."""
    root, window = Path(req["workdir"]), req["window"]
    trajectory_path = root / "lo" / ("ref_" + window + ".npz")
    if not trajectory_path.exists():
        return None
    with zipfile.ZipFile(trajectory_path) as archive:
        if sum(info.file_size for info in archive.infolist()) > MAX_TRAJECTORY_BYTES:
            return None
    tr = LOTraj.load(trajectory_path)
    middle = (int(tr.tau[0]) + int(tr.tau[-1])) // 2
    files = sorted(heapq.nsmallest(MAX_LIDAR_FILES, (root / "extract" / window / "lidar").glob("*.npy"),
                                  key=lambda path: abs(int(path.stem) - middle)))
    if not files:
        return None
    indices = np.unique(np.linspace(0, len(files) - 1, min(3, len(files)), dtype=int))
    limit = min(70000, max(3, int(cfg.get("viz", {}).get("max_points", 30000))))
    per_sweep = max(1, limit // len(indices))
    chunks = []
    for index in indices:
        path = files[index]
        scan = np.load(path, mmap_mode="r", allow_pickle=False)
        sample = np.array(scan[::max(1, int(np.ceil(len(scan) / per_sweep)))], copy=True)
        radius = np.sqrt(sample["x"] ** 2 + sample["y"] ** 2 + sample["z"] ** 2)
        sample = sample[np.isfinite(radius) & (radius > 2.5) & (radius < 100.)]
        if len(sample):
            chunks.append(deskew(sample, int(path.stem), tr).astype(np.float32))
    if not chunks:
        return None
    bag_id = _bag_id(req)
    frame = "lidar_window:" + ((str(bag_id) + "/") if bag_id else "") + window
    state = {"stage": "lidar_odometry", "window_id": window, "bag_id": bag_id,
             "map_frame": frame, "map_source": "cached-lo-sample", "lo_phase": "cached"}
    trajectory = tr.T[::max(1, int(np.ceil(len(tr.T) / 4096))), :3, 3].astype(np.float32)
    return state, {"points": np.concatenate(chunks), "trajectory": trajectory}
