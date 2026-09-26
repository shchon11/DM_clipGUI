"""Bounded, failure-tolerant file stream. No Qt or solver imports."""
from collections import deque
import json
import math
import os
from pathlib import Path
import uuid
import zipfile

import cv2
import numpy as np

from . import SCHEMA

MAX_RECORD_BYTES = 256 * 1024
MAX_ASSET_BYTES = 32 * 1024 * 1024


def atomic_json(path, value):
    """Publish a JSON document with an atomic rename on the same filesystem."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with tmp.open("w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, allow_nan=False)
            handle.write("\n")
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def atomic_npz(path, **arrays):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with tmp.open("wb") as handle:
            np.savez_compressed(handle, **arrays)
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def append_snapshot(root, snapshot):
    """Single-writer append; assets must already have been atomically published."""
    payload = (json.dumps(snapshot, ensure_ascii=False, allow_nan=False,
                          separators=(",", ":")) + "\n").encode("utf-8")
    if len(payload) > MAX_RECORD_BYTES:
        raise ValueError("snapshot exceeds the 256 KiB protocol limit")
    with (Path(root) / "events.jsonl").open("ab", buffering=0) as handle:
        handle.write(payload)


def validate_snapshot(event):
    if not isinstance(event, dict) or event.get("schema") != SCHEMA:
        raise ValueError("unsupported snapshot schema")
    if not isinstance(event.get("seq"), int) or event["seq"] < 0:
        raise ValueError("seq must be a nonnegative integer")
    if not isinstance(event.get("cameras"), dict):
        raise ValueError("cameras must be an object")
    for key in ("progress", "eta_s", "source_time_s", "t", "map_progress"):
        if key in event:
            value = event[key]
            if key == "eta_s" and value is None:
                continue
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError("invalid snapshot " + key)
            if key in ("progress", "map_progress") and value > 1:
                raise ValueError("fraction must be in [0, 1]")
    for key in ("window", "total_windows"):
        if key in event and (not isinstance(event[key], int) or event[key] < 0):
            raise ValueError("invalid window count")
    if "stage" in event and not isinstance(event["stage"], str):
        raise ValueError("stage must be a string")
    for key in ("assets", "gate", "provenance"):
        if key in event and not isinstance(event[key], dict):
            raise ValueError(key + " must be an object")
    assets = event.get("assets", {})
    gate = event.get("gate", {})
    if gate.get("pass") is not None and not isinstance(gate["pass"], bool):
        raise ValueError("gate pass must be boolean or null")
    if "status" in gate and gate["status"] not in ("pending", "pass", "fail"):
        raise ValueError("invalid gate status")
    if "map" in assets and not isinstance(assets["map"], str):
        raise ValueError("map asset must be a path")
    matching = assets.get("matching", {})
    if not isinstance(matching, dict):
        raise ValueError("matching assets must be an object")
    for preview in matching.values():
        if not isinstance(preview, dict):
            raise ValueError("preview must be an object")
        if any(not isinstance(preview.get(key), str) for key in ("image", "points")):
            raise ValueError("preview requires image and points paths")
        for key in ("width", "height"):
            if key in preview and (not isinstance(preview[key], int) or not 0 < preview[key] <= 3840):
                raise ValueError("invalid preview dimensions")
    for camera in event["cameras"].values():
        if not isinstance(camera, dict):
            raise ValueError("camera state must be an object")
        if "state" in camera and not isinstance(camera["state"], str):
            raise ValueError("camera state must be a string")
        for key in ("validation_vote", "gate_pass", "validation_vote_gate"):
            if camera.get(key) is not None and not isinstance(camera[key], bool):
                raise ValueError("camera " + key + " must be boolean or null")
        for key in ("informational_checks", "gate_reasons"):
            if key in camera and (not isinstance(camera[key], list)
                                  or any(not isinstance(note, str) for note in camera[key])):
                raise ValueError("camera " + key + " must be a list of text")
        T = np.asarray(camera.get("T_cam_lidar"), dtype=float)
        if T.shape != (4, 4) or not np.isfinite(T).all():
            raise ValueError("invalid camera pose")
        if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-5):
            raise ValueError("invalid homogeneous camera pose")
        R = T[:3, :3]
        if not np.allclose(R.T @ R, np.eye(3), atol=1e-3) or np.linalg.det(R) < 0.99:
            raise ValueError("camera pose rotation is not SO(3)")
        for key in ("sigma_rot_deg", "sigma_pos_mm", "reprojection_px"):
            value = camera.get(key)
            if value is not None and (not np.isfinite(float(value)) or value < 0):
                raise ValueError("invalid camera metric")
    return event


def validate_manifest(manifest):
    if not isinstance(manifest, dict) or manifest.get("schema") != SCHEMA:
        raise ValueError("unsupported manifest schema")
    cameras = manifest.get("cameras")
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError("manifest requires camera calibration")
    R = np.asarray(manifest.get("R_lidar_V"), float)
    if (R.shape != (3, 3) or not np.isfinite(R).all() or
            not np.allclose(R.T @ R, np.eye(3), atol=1e-3) or np.linalg.det(R) < .99):
        raise ValueError("invalid R_lidar_V rotation")
    for camera in cameras.values():
        if not isinstance(camera, dict):
            raise ValueError("manifest camera must be an object")
        K = np.asarray(camera.get("K"), float)
        D = np.asarray(camera.get("D"), float)
        model = camera.get("model")
        count = 4 if model == "equidistant" else (5 if model == "plumb_bob" else 0)
        if K.shape != (3, 3) or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
            raise ValueError("invalid camera matrix")
        if not count or D.shape != (count,) or not np.isfinite(D).all():
            raise ValueError("invalid distortion model or coefficients")
        if any(not isinstance(camera.get(key), int) or not 0 < camera[key] <= 8192
               for key in ("width", "height")):
            raise ValueError("invalid camera dimensions")
    return manifest


class StreamReader:
    """Tail complete records without blocking a producer or reading giant files.

    ``poll`` consumes at most max_bytes and returns at most max_events, retaining
    the newest snapshots from that batch. Replacement/truncation resets seq.
    Missing/malformed assets raise ValueError/OSError so the UI can retain its
    previous image; one bad line never terminates the reader.
    """
    def __init__(self, root, max_events=32, max_bytes=1024 * 1024):
        self.root = Path(root).resolve()
        self.max_events = max(1, int(max_events))
        self.max_bytes = max(MAX_RECORD_BYTES, int(max_bytes))
        self.errors = deque(maxlen=16)
        self.manifest = {}
        self._offset = 0
        self._identity = None
        self._partial = b""
        self._skip_line = False
        self._last_seq = -1
        self._anchor = b""
        self.refresh_manifest()

    def refresh_manifest(self):
        path = self.root / "manifest.json"
        if not path.exists():
            return self.manifest
        if path.stat().st_size > MAX_RECORD_BYTES:
            raise ValueError("manifest exceeds protocol size limit")
        with path.open(encoding="utf-8") as handle:
            manifest = json.load(handle)
        try:
            self.manifest = validate_manifest(manifest)
        except (TypeError, AttributeError) as exc:
            raise ValueError("invalid manifest fields") from exc
        return manifest

    def poll(self):
        path = self.root / "events.jsonl"
        try:
            stat = path.stat()
        except FileNotFoundError:
            return []
        identity = (stat.st_dev, stat.st_ino)
        changed = identity != self._identity or stat.st_size < self._offset
        if not changed and self._anchor:
            # Detect copy-truncate/rewrite that has already grown past offset
            # before the next poll; size/inode alone cannot distinguish it.
            with path.open("rb") as handle:
                handle.seek(self._offset - len(self._anchor))
                changed = handle.read(len(self._anchor)) != self._anchor
        if changed:
            self._offset = max(0, stat.st_size - self.max_bytes)
            self._partial, self._last_seq = b"", -1
            self._skip_line = self._offset > 0
            self._identity = identity
            self._anchor = b""
        elif stat.st_size - self._offset > self.max_bytes:
            # A suspended viewer may fall behind a healthy producer. Snapshots
            # are complete, so bounded latency is preferable to historical replay.
            self._offset = stat.st_size - self.max_bytes
            self._partial, self._skip_line = b"", True
        with path.open("rb") as handle:
            handle.seek(self._offset)
            chunk = handle.read(self.max_bytes)
            self._offset = handle.tell()
            anchor_size = min(64, self._offset)
            handle.seek(self._offset - anchor_size)
            self._anchor = handle.read(anchor_size)
        data = self._partial + chunk
        lines = data.split(b"\n")
        self._partial = lines.pop()
        events = deque(maxlen=self.max_events)
        for line in lines:
            if self._skip_line:
                self._skip_line = False
                continue
            if not line.strip():
                continue
            try:
                if len(line) > MAX_RECORD_BYTES:
                    raise ValueError("oversized snapshot skipped")
                event = validate_snapshot(json.loads(line))
                if event["seq"] > self._last_seq:
                    events.append(event)
                    self._last_seq = event["seq"]
            except (ValueError, TypeError, KeyError, AttributeError) as exc:
                self.errors.append(str(exc))
        if len(self._partial) > MAX_RECORD_BYTES:
            self._partial = b""
            self._skip_line = True
            self.errors.append("oversized partial snapshot skipped")
        return list(events)

    def asset_path(self, relative):
        if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
            raise ValueError("asset must be a nonempty relative path")
        path = (self.root / relative).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError("asset path escapes viz directory")
        return path

    def load_asset(self, relative):
        path = self.asset_path(relative)
        if path.stat().st_size > MAX_ASSET_BYTES:
            raise ValueError("asset exceeds 32 MiB limit")
        if path.suffix.lower() == ".npz":
            try:
                with zipfile.ZipFile(path) as archive:
                    if sum(item.file_size for item in archive.infolist()) > MAX_ASSET_BYTES:
                        raise ValueError("expanded NPZ exceeds 32 MiB limit")
                    for item in archive.infolist():
                        with archive.open(item) as handle:
                            version = np.lib.format.read_magic(handle)
                            shape, _, dtype = np.lib.format._read_array_header(handle, version)
                            size = math.prod(shape) * dtype.itemsize
                            if dtype.hasobject or size > MAX_ASSET_BYTES or size > item.file_size:
                                raise ValueError("unsafe array header")
                with np.load(path, allow_pickle=False) as archive:
                    arrays = {name: archive[name] for name in archive.files}
            except (zipfile.BadZipFile, EOFError, RuntimeError) as exc:
                raise ValueError("corrupt NPZ asset") from exc
            for name, array in arrays.items():
                columns = {"points": 3, "trajectory": 3, "points_lidar": 3,
                           "tracks_uv": 2, "tracks_prev_uv": 2}.get(name)
                if columns:
                    if (array.ndim != 2 or array.shape[1] != columns or
                            not np.issubdtype(array.dtype, np.number) or not np.isfinite(array).all()):
                        raise ValueError("invalid " + name + " array")
                elif name == "mask" and (array.ndim != 2 or array.dtype != np.uint8):
                    raise ValueError("mask must be a 2D uint8 image")
            if ("tracks_uv" in arrays and "tracks_prev_uv" in arrays and
                    arrays["tracks_uv"].shape != arrays["tracks_prev_uv"].shape):
                raise ValueError("track arrays must have equal shapes")
            return arrays
        if path.suffix.lower() in (".jpg", ".jpeg", ".png"):
            try:
                data = cv2.imread(str(path), cv2.IMREAD_COLOR)
            except cv2.error as exc:
                raise ValueError("unreadable image asset") from exc
            if data is None:
                raise ValueError("unreadable image asset")
            if data.shape[0] > 2160 or data.shape[1] > 3840:
                raise ValueError("preview image exceeds 3840 x 2160")
            return cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
        raise ValueError("unsupported asset extension")
