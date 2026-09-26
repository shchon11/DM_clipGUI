"""Fail-open, observation-only live visualisation stream.

Each process owns one bounded mailbox and one I/O thread. A filesystem lock serialises
state merges and complete JSONL writes across workers. No solver object is retained or
mutated, and neither random generators nor numerical thread settings are touched.
"""
from __future__ import annotations

import copy
import fcntl
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

import numpy as np

SCHEMA = "calib-viz/1"
MAX_JSON = 256 * 1024


def _plain(value):
    if isinstance(value, np.ndarray):
        return _plain(value.tolist())
    if isinstance(value, np.generic):
        return _plain(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    return value


def _merge(base, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


def _atomic(path, data):
    tmp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        tmp.write_bytes(data)
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def _json(value):
    raw = json.dumps(_plain(value), separators=(",", ":"), allow_nan=False).encode()
    if len(raw) > MAX_JSON:
        raise ValueError("visualisation JSON exceeds 256 KiB")
    return raw



def _recover_state(root, run_id):
    """Recover an interrupted append before assigning the next global sequence."""
    state_path = root / ".state.json"
    try:
        shared = json.loads(state_path.read_text())
    except (OSError, ValueError):
        shared = {"snapshot": {"schema": SCHEMA, "run_id": run_id, "seq": 0,
                                "cameras": {}, "assets": {}}, "last_emit": 0., "map_t": 0., "image_t": 0., "image_emit_t": 0.}
    event_path = root / "events.jsonl"
    if event_path.exists():
        with open(event_path, "r+b") as f:
            size = f.seek(0, os.SEEK_END)
            start = max(0, size - 2 * MAX_JSON)
            f.seek(start)
            tail = f.read()
            # A crashed process can leave a suffix even though concurrent writers
            # cannot interleave. Keep every complete append, discard only the suffix.
            end = tail.rfind(b"\n") + 1
            if tail and not tail.endswith(b"\n"):
                f.truncate(start + end)
            for line in reversed(tail[:end].splitlines()):
                try:
                    latest = json.loads(line)
                except ValueError:
                    continue
                if latest.get("run_id") == run_id and latest.get("seq", 0) > shared["snapshot"].get("seq", 0):
                    shared["snapshot"] = latest
                    shared["last_emit"] = latest.get("t", 0.)
                    if latest.get("assets", {}).get("matching"):
                        shared["image_emit_t"] = max(shared.get("image_emit_t", 0.), latest.get("t", 0.))
                break
    if "image_emit_t" not in shared and shared["snapshot"].get("assets", {}).get("matching"):
        shared["image_emit_t"] = shared["snapshot"].get("t", 0.)
    _atomic(state_path, _json(shared))
    return shared


def _default_manifest(cfg):
    import yaml
    path = cfg.get("paths", {}).get("rig_design")
    design = yaml.safe_load(Path(path).read_text()) if path else {}
    cameras = {}
    for sensor in ("rgb", "thermal"):
        width, height = cfg.get("cameras", {}).get(sensor + "_size", [1920, 1200] if sensor == "rgb" else [640, 480])
        for name in cfg.get("cameras", {}).get(sensor, []):
            d = design.get(sensor, {}).get(name, {})
            intr = d.get("seed_intr_kb" if sensor == "rgb" else "nominal_intr_pinhole")
            if intr is None:
                continue
            distortion = list(intr[4:8]) if sensor == "rgb" else list(intr[4:6]) + [0., 0., 0.]
            cameras[name] = {"sensor": sensor, "model": "equidistant" if sensor == "rgb" else "plumb_bob",
                             "width": width, "height": height,
                             "K": [[intr[0], 0, intr[2]], [0, intr[1], intr[3]], [0, 0, 1]], "D": distortion}
    return {"schema": SCHEMA, "mode": "live", "parent_frame": "os_lidar", "display_frame": "vehicle",
            "R_lidar_V": [[-1, 0, 0], [0, -1, 0], [0, 0, 1]], "cameras": cameras,
            "provenance": {"estimates": "live solver observations", "synthetic": False},
            "rate_limits": {"state_hz": 4, "map_hz": 1, "image_hz": 1}}



def _retained_map(root, old_manifest, new_manifest):
    """Carry only a safe map asset across an identical-input fresh attempt."""
    identity = new_manifest.get("input_identity")
    if not identity or old_manifest.get("input_identity") != identity or old_manifest.get("mode") != "live":
        return {}
    shared = _recover_state(root, old_manifest["run_id"])
    old = shared["snapshot"]
    if old.get("run_id") != old_manifest["run_id"]:
        return {}
    relative = old.get("assets", {}).get("map")
    if not isinstance(relative, str):
        return {}
    asset = Path(relative)
    if asset.is_absolute() or ".." in asset.parts or len(asset.parts) < 2 or asset.parts[0] != "assets":
        return {}
    path = root / asset
    if not path.is_file() or path.suffix != ".npz" or path.stat().st_size > 32 * 1024 * 1024:
        return {}
    if not path.resolve().is_relative_to(root.resolve() / "assets"):
        return {}
    if any((root / Path(*asset.parts[:i])).is_symlink() for i in range(1, len(asset.parts) + 1)):
        return {}
    if not old.get("map_frame"):
        return {}
    window = old.get("map_source_window", old.get("map_window", old.get("window_id")))
    return {"assets": {"map": relative}, "map_frame": old["map_frame"],
            "map_window": window, "map_source_window": window,
            "map_source": "previous-attempt-cache", "previous_map_run_id": old_manifest["run_id"]}


class NullViz:
    enabled = False
    context = {}

    def due(self, *args, **kwargs):
        return False

    def publish(self, *args, **kwargs):
        return False

    def publish_manifest(self, *args, **kwargs):
        return False

    def preview(self, *args, **kwargs):
        return False

    def cached_map(self, *args, **kwargs):
        return False

    def close(self):
        pass


class LiveViz:
    enabled = True

    def __init__(self, workdir, cfg, context=None, log=print, *, fresh=False):
        self.cfg = cfg
        limits = cfg.get("viz", {})
        self.intervals = {k: 1. / min(cap, max(.01, float(limits.get(k + "_hz", cap))))
                          for k, cap in (("state", 4.), ("map", 1.), ("image", 1.))}
        self.context = _plain(context or {})
        self.log = log
        self._warned = False
        self.root = Path(cfg.get("viz", {}).get("dir") or Path(workdir) / "viz")
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "assets").mkdir(exist_ok=True)
        self._cv = threading.Condition()
        self._pending = None
        self._closing = False
        self._due = {}
        self._seen_cameras = []
        self._last_camera = None
        self._next_preview = 0.
        self._control = None
        self._control_time = -math.inf
        self._last_prune = 0.
        self._preview_cache = {}
        with self._lock():
            manifest_path = self.root / "manifest.json"
            if not fresh and manifest_path.exists():
                self.manifest = json.loads(manifest_path.read_text())
                self.run_id = self.context.get("run_id") or self.manifest["run_id"]
                if self.run_id == self.manifest["run_id"]:
                    _recover_state(self.root, self.run_id)
            else:
                self.run_id = self.context.get("run_id") or uuid.uuid4().hex
                self.manifest = _default_manifest(cfg)
                supplied_manifest = self.context.get("manifest", {})
                _merge(self.manifest, supplied_manifest)
                # An explicit active sensor set is authoritative, including no
                # cameras; merging seed defaults would invent unused sensors.
                if "cameras" in supplied_manifest:
                    self.manifest["cameras"] = copy.deepcopy(supplied_manifest["cameras"])
                self.manifest.update(run_id=self.run_id, schema=SCHEMA, mode="live")
                retained = {}
                if fresh and manifest_path.exists():
                    try:
                        retained = _retained_map(self.root, json.loads(manifest_path.read_text()), self.manifest)
                    except Exception as error:
                        self._warn(error)
                _atomic(manifest_path, _json(self.manifest))
                initial = {"schema": SCHEMA, "run_id": self.run_id, "seq": 0, "t": time.time(),
                           "stage": "extract", "progress": 0., "cameras": {}, "assets": {}, "gate": {"status": "pending", "pass": None}}
                _merge(initial, retained)
                _atomic(self.root / "events.jsonl", b"")
                _atomic(self.root / ".state.json", _json({"snapshot": initial, "last_emit": 0., "map_t": 0., "image_t": 0., "image_emit_t": 0.}))
        self.context["run_id"] = self.run_id
        self._thread = threading.Thread(target=self._loop, name="calib-viz-io", daemon=True)
        self._thread.start()

    def _lock(self):
        class Lock:
            def __enter__(inner):
                inner.f = open(self.root / ".writer.lock", "a+b")
                fcntl.flock(inner.f, fcntl.LOCK_EX)
            def __exit__(inner, *exc):
                fcntl.flock(inner.f, fcntl.LOCK_UN)
                inner.f.close()
        return Lock()

    def _warn(self, error):
        if not self._warned:
            self._warned = True
            try:
                self.log(f"[viz] skipped visualisation output: {error}")
            except Exception:
                pass

    def due(self, kind="state", camera=None):
        """Reserve a cheap local sampling slot, before copying solver-owned arrays."""
        try:
            now = time.monotonic()
            key = (kind, camera)
            interval = self.intervals.get(kind, self.intervals["image"])
            if self._closing or now - self._due.get(key, -math.inf) < interval:
                return False
            self._due[key] = now
            return True
        except Exception as error:
            self._warn(error)
            return False

    def publish_manifest(self, patch):
        try:
            self._enqueue({"manifest": _plain(patch)})
            return True
        except Exception as error:
            self._warn(error)
            return False

    def publish(self, state, map_data=None, force=False):
        """Enqueue a detached state; latest camera updates coalesce without being lost.

        ``force`` documents milestone intent; the writer still enforces 4 Hz globally.
        Map callers should check ``due('map')`` before preparing their small samples.
        """
        try:
            patch = {k: self.context[k] for k in ("stage", "task", "pass", "bag_id", "window_id", "window", "total_windows") if k in self.context}
            _merge(patch, _plain(state))
            job = {"state": patch}
            if map_data is not None:
                job["map"] = {}
                for key, limit in (("points", min(70000, max(1, int(self.cfg.get("viz", {}).get("max_points", 70000))))), ("trajectory", 4096)):
                    a = np.asarray(map_data.get(key, np.empty((0, 3))))
                    if a.ndim != 2 or a.shape[1] != 3:
                        raise ValueError("map arrays must be N by 3")
                    step = max(1, math.ceil(len(a) / limit))
                    job["map"][key] = np.array(a[::step], dtype=np.float32, copy=True)
                job["map_frame"] = patch.get("map_frame")
            self._enqueue(job)
            return True
        except Exception as error:
            self._warn(error)
            return False

    def preview(self, ws, window, camera, camera_state, tracks=None):
        """Request one real exposure/deskew/KLT preview; all I/O is asynchronous."""
        try:
            if self._closing:
                return False
            if camera not in self._seen_cameras:
                self._seen_cameras.append(camera)
            now = time.monotonic()
            if now < self._next_preview:
                return False
            # Control is small and read by the I/O thread. First request may precede it.
            control = self._control or {}
            if control.get("enabled") is False:
                return False
            selected = control.get("camera")
            if selected and camera != selected:
                return False
            if not selected and len(self._seen_cameras) > 1 and self._last_camera in self._seen_cameras:
                wanted = self._seen_cameras[(self._seen_cameras.index(self._last_camera) + 1) % len(self._seen_cameras)]
                if wanted != camera:
                    return False
            self._next_preview = now + self.intervals["image"]
            self._last_camera = camera
            context = copy.deepcopy(self.context)
            records = [w for w in context.get("windows", []) if isinstance(w, dict) and w.get("name") == str(window)]
            if records:
                context["bag_id"] = records[0]["bag_id"]
            elif context.get("window_id") != str(window):
                context.pop("bag_id", None)
            context["window_id"] = str(window)
            self._enqueue({"preview": {"workdir": str(ws.root), "window": str(window), "camera": camera,
                                       "camera_state": _plain(camera_state), "context": context}})
            return True
        except Exception as error:
            self._warn(error)
            return False

    def cached_map(self, ws, window):
        """Schedule a bounded real LO cache sample only when no map is displayed."""
        try:
            if self._closing:
                return False
            self._enqueue({"cached_map": {"workdir": str(ws.root), "window": str(window),
                                          "context": copy.deepcopy(self.context)}})
            return True
        except Exception as error:
            self._warn(error)
            return False

    def _enqueue(self, job):
        with self._cv:
            if self._closing:
                return
            if self._pending is None:
                self._pending = job
            else:
                for key in ("state", "manifest"):
                    if key in job:
                        _merge(self._pending.setdefault(key, {}), job[key])
                for key in ("map", "map_frame", "preview", "cached_map"):
                    if key in job:
                        self._pending[key] = job[key]
            self._cv.notify()

    def _read_control(self):
        if time.monotonic() - self._control_time < .5:
            return
        self._control_time = time.monotonic()
        try:
            path = self.root / "control.json"
            control = json.loads(path.read_text()) if path.exists() and path.stat().st_size <= 4096 else {}
            self._control = control if control.get("run_id", self.run_id) == self.run_id else {}
        except (OSError, ValueError):
            self._control = {}

    def _reserve_image(self, camera):
        self._read_control()
        if (self._control or {}).get("enabled") is False:
            return False
        selected = (self._control or {}).get("camera")
        if selected and selected != camera:
            return False
        with self._lock():
            shared = json.loads((self.root / ".state.json").read_text())
            if shared["snapshot"]["run_id"] != self.run_id or time.time() - shared.get("image_t", 0) < self.intervals["image"]:
                return False
            shared["image_t"] = time.time()
            _atomic(self.root / ".state.json", _json(shared))
        return True

    def _npz(self, name, arrays):
        path = self.root / "assets" / (name + "_" + uuid.uuid4().hex + ".npz")
        tmp = path.with_suffix(".tmp")
        try:
            with open(tmp, "wb") as f:
                np.savez(f, **arrays)
            os.replace(tmp, path)
        finally:
            tmp.unlink(missing_ok=True)
        return path.relative_to(self.root).as_posix()

    def _loop(self):
        while True:
            with self._cv:
                if self._pending is None and not self._closing:
                    self._cv.wait(.25)
                if self._pending is None:
                    if self._closing:
                        return
                    try:
                        self._read_control()
                    except Exception as error:
                        self._warn(error)
                    continue
                job, self._pending = self._pending, None
            try:
                self._process(job)
            except Exception as error:
                self._warn(error)

    def _process(self, job):
        self._read_control()
        assets = {}
        cached_result = None
        if job.get("cached_map") and "map" not in job:
            with self._lock():
                current = json.loads((self.root / ".state.json").read_text())["snapshot"]
                needs_map = current["run_id"] == self.run_id and not current.get("assets", {}).get("map")
            if needs_map:
                try:
                    from .viz_preview import build_cached_map
                    cached_result = build_cached_map(job["cached_map"], self.cfg)
                except Exception as error:
                    self._warn(error)
        preview = job.get("preview")
        if preview and self._reserve_image(preview["camera"]):
            try:
                from .viz_preview import build_preview
                result = build_preview(preview, self.cfg, self._preview_cache)
                if result is not None:
                    image, arrays, meta = result
                    name = "preview_" + uuid.uuid4().hex
                    image_path = self.root / "assets" / (name + ".jpg")
                    _atomic(image_path, image)
                    meta.update(image=image_path.relative_to(self.root).as_posix(), points=self._npz(name, arrays))
                    assets["matching"] = {preview["camera"]: meta}
                    cameras = job.setdefault("state", {}).setdefault("cameras", {})
                    # A coalesced solver update may be newer than this rate-limited
                    # preview request. Fill missing values, never rewind that update.
                    cameras[preview["camera"]] = _merge(copy.deepcopy(preview["camera_state"]),
                                                        cameras.get(preview["camera"], {}))
            except Exception as error:
                self._warn(error)
        while True:
            delay = 0.
            with self._lock():
                shared = json.loads((self.root / ".state.json").read_text())
                current = shared["snapshot"]
                if current["run_id"] != self.run_id:
                    return
                now = time.time()
                delay = max(0., self.intervals["state"] - (now - shared.get("last_emit", 0.)))
                if assets.get("matching"):
                    # Preparation starts can be 1 s apart while variable decode /
                    # encoding latency makes publication much closer. Reserve the
                    # actual emission independently, across every worker process.
                    delay = max(delay, self.intervals["image"] - (now - shared.get("image_emit_t", 0.)))
                if delay == 0:
                    if cached_result is not None and not current.get("assets", {}).get("map") and "map" not in job:
                        cached_state, cached_arrays = cached_result
                        # Cache geometry can finish after calibration/validation.
                        # Never replay its earlier stage, progress or camera state.
                        job.update(map=cached_arrays, map_frame=cached_state["map_frame"],
                                   map_source="cached-lo-sample", map_window=cached_state.get("window_id"),
                                   map_bag_id=cached_state.get("bag_id"))
                    patch = job.get("state", {})
                    if patch.get("map_frame") and patch["map_frame"] != current.get("map_frame"):
                        current.setdefault("assets", {}).pop("map", None)
                    _merge(current, patch)
                    if "map" in job and now - shared.get("map_t", 0) >= self.intervals["map"]:
                        assets["map"] = self._npz("map", job["map"])
                        if job.get("map_frame"):
                            current["map_frame"] = job["map_frame"]
                        map_window = job.get("map_window", current.get("window_id"))
                        current.update(map_source=job.get("map_source", "solver"), map_window=map_window,
                                       map_source_window=map_window,
                                       map_bag_id=job.get("map_bag_id", current.get("bag_id")))
                        current.pop("previous_map_run_id", None)
                        shared["map_t"] = now
                    _merge(current.setdefault("assets", {}), assets)
                    current.update(schema=SCHEMA, run_id=self.run_id, seq=int(current.get("seq", 0)) + 1, t=now)
                    raw = _json(current) + b"\n"
                    event_path = self.root / "events.jsonl"
                    max_log = max(1024, int(self.cfg.get("viz", {}).get("max_log_bytes", 16 * 1024 * 1024)))
                    if event_path.exists() and event_path.stat().st_size + len(raw) > max_log:
                        _atomic(event_path, raw)
                    else:
                        with open(event_path, "ab", buffering=0) as f:
                            f.write(raw)
                    shared["last_emit"] = now
                    if assets.get("matching"):
                        shared["image_emit_t"] = now
                    _atomic(self.root / ".state.json", _json(shared))
                    manifest_patch = job.get("manifest", {})
                    # Camera models/intrinsics are also repeated in snapshots for tails
                    # that opened the manifest before optimisation changed the lens.
                    if manifest_patch:
                        manifest = json.loads((self.root / "manifest.json").read_text())
                        _merge(manifest, manifest_patch)
                        _atomic(self.root / "manifest.json", _json(manifest))
                    self._prune(current)
                    return
            # Sleep only the independent I/O thread, never a solver callback.
            time.sleep(min(delay, .25))

    def _prune(self, snapshot):
        now = time.time()
        if now - self._last_prune < 5:
            return
        self._last_prune = now
        active = set()
        def paths(value):
            if isinstance(value, str) and value.startswith("assets/"):
                active.add(value)
            elif isinstance(value, dict):
                for v in value.values():
                    paths(v)
        paths(snapshot.get("assets", {}))
        files = sorted((self.root / "assets").iterdir(), key=lambda p: p.stat().st_mtime, reverse=True)
        keep_s = max(2., float(self.cfg.get("viz", {}).get("retain_seconds", self.cfg.get("viz", {}).get("asset_keep_s", 120.))))
        for i, path in enumerate(files):
            if path.relative_to(self.root).as_posix() not in active and (i >= 512 or now - path.stat().st_mtime > keep_s):
                path.unlink(missing_ok=True)

    def close(self):
        try:
            with self._cv:
                self._closing = True
                self._cv.notify()
            self._thread.join(timeout=5.)
        except Exception as error:
            self._warn(error)


_viz = NullViz()


def get_viz():
    return _viz


def configure(workdir, cfg, context=None, log=print, *, fresh=False):
    """Configure this process. Every setup and publication failure remains optional."""
    global _viz
    _viz.close()
    _viz = NullViz()
    if not cfg.get("viz", {}).get("enabled", True) or (context or {}).get("enabled") is False:
        return _viz
    try:
        _viz = LiveViz(workdir, cfg, context=context, log=log, fresh=fresh)
    except Exception as error:
        try:
            log(f"[viz] disabled: {error}")
        except Exception:
            pass
    return _viz
