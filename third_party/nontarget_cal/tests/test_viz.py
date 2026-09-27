"""Publication, process isolation, rate and preview contracts of the live producer."""
import json
import multiprocessing
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np

from nontarget_cal import viz
from nontarget_cal.viz_preview import build_preview


def config(**options):
    return {"viz": {"enabled": True, **options}, "cameras": {"rgb": [], "thermal": []}}


def events(root):
    path = Path(root) / "viz/events.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_disabled_and_setup_errors_are_fail_open(tmp_path):
    off = viz.configure(tmp_path, config(enabled=False))
    assert not off.enabled and not off.due()
    assert not off.publish({"unserializable": object()})
    assert not (tmp_path / "viz").exists()
    log = []
    path = tmp_path / "file"
    path.write_text("block directory")
    bad = viz.configure(path, config(), log=log.append)
    assert not bad.enabled and len(log) == 1
    context_off = viz.configure(tmp_path, config(), {"enabled": False})
    assert not context_off.enabled


def test_coalesced_state_is_complete_and_arrays_are_detached(tmp_path):
    v = viz.LiveViz(tmp_path, config(), {"run_id": "a", "window": 2}, fresh=True)
    points = np.arange(6000, dtype=np.float64).reshape(-1, 3)
    before = points.copy()
    T = np.eye(4)
    for c in range(20):
        v.publish({"cameras": {str(c): {"T_cam_lidar": T}}, "map_frame": "lidar_window:S01"},
                  map_data={"points": points, "trajectory": points[:3]})
    T[0, 3] = 999
    points[:] = -1
    v.close()
    stream = events(tmp_path)
    assert set(stream[-1]["cameras"]) == {str(c) for c in range(20)}
    assert stream[-1]["window"] == 2
    assert all(c["T_cam_lidar"][0][3] == 0 for c in stream[-1]["cameras"].values())
    with np.load(tmp_path / "viz" / stream[-1]["assets"]["map"], allow_pickle=False) as data:
        assert np.array_equal(data["points"], before.astype(np.float32))
    assert not list((tmp_path / "viz").rglob("*.tmp"))


def test_due_and_global_state_rate(tmp_path):
    v = viz.LiveViz(tmp_path, config(state_hz=100), fresh=True)
    assert v.due("state")
    assert not v.due("state")
    assert v.due("map")
    assert not v.due("map")
    for i in range(18):
        v.publish({"iteration": i}, force=True)
        time.sleep(.03)
    v.close()
    stream = events(tmp_path)
    assert len(stream) >= 2
    assert all(b["t"] - a["t"] >= .249 for a, b in zip(stream, stream[1:]))
    assert stream[-1]["iteration"] == 17


def _process_publish(root, idx):
    v = viz.LiveViz(root, config(), {"run_id": "workers"})
    for i in range(20):
        v.publish({"cameras": {f"cam{idx}": {"iteration": i, "cost": float(i)}}})
    v.close()


def test_four_processes_preserve_every_camera_and_whole_lines(tmp_path):
    v = viz.LiveViz(tmp_path, config(), {"run_id": "workers"}, fresh=True)
    v.close()
    ctx = multiprocessing.get_context("spawn")
    procs = [ctx.Process(target=_process_publish, args=(str(tmp_path), i)) for i in range(4)]
    for p in procs:
        p.start()
    while any(p.is_alive() for p in procs):
        raw = (tmp_path / "viz/events.jsonl").read_bytes()
        if raw:
            assert raw.endswith(b"\n")
            for line in raw.splitlines():
                json.loads(line)
        time.sleep(.01)
    for p in procs:
        p.join(timeout=10)
        assert p.exitcode == 0
    stream = events(tmp_path)
    assert [e["seq"] for e in stream] == list(range(1, len(stream) + 1))
    assert all(c["iteration"] == 19 for c in stream[-1]["cameras"].values())
    assert len(stream[-1]["cameras"]) == 4
    assert all(b["t"] - a["t"] >= .249 for a, b in zip(stream, stream[1:]))


def test_resume_and_fresh_attempt_reject_old_worker(tmp_path):
    old = viz.LiveViz(tmp_path, config(), {"run_id": "old"}, fresh=True)
    old.publish({"iteration": 4})
    old.close()
    resumed = viz.LiveViz(tmp_path, config(), {"run_id": "old"})
    resumed.publish({"iteration": 5})
    resumed.close()
    assert events(tmp_path)[-1]["seq"] == 2
    stale = viz.LiveViz(tmp_path, config(), {"run_id": "old"})
    new = viz.LiveViz(tmp_path, config(), {"run_id": "new"}, fresh=True)
    stale.publish({"iteration": -1})
    stale.close()
    new.publish({"iteration": 0})
    new.close()
    assert [(e["run_id"], e["iteration"]) for e in events(tmp_path)] == [("new", 0)]


def test_atomic_assets_can_be_opened_as_events_arrive(tmp_path):
    v = viz.LiveViz(tmp_path, config(), fresh=True)
    errors = []
    stop = threading.Event()
    def reader():
        while not stop.is_set():
            try:
                for event in events(tmp_path):
                    if event.get("assets", {}).get("map"):
                        with np.load(tmp_path / "viz" / event["assets"]["map"], allow_pickle=False) as data:
                            assert data["points"].shape[1] == 3
            except Exception as exc:
                errors.append(exc)
            time.sleep(.001)
    thread = threading.Thread(target=reader)
    thread.start()
    for _ in range(12):
        v.publish({"map_frame": "lidar_window:S01"}, map_data={"points": np.ones((100, 3))})
        time.sleep(.02)
    v.close()
    stop.set()
    thread.join()
    assert not errors


def test_failures_log_once_and_do_not_escape(tmp_path, monkeypatch):
    log = []
    v = viz.LiveViz(tmp_path, config(), log=log.append, fresh=True)
    def failure(*args):
        raise OSError("disk unavailable")
    monkeypatch.setattr(v, "_process", failure)
    for _ in range(3):
        v.publish({"iteration": 0})
        time.sleep(.01)
    v.close()
    assert len(log) == 1 and "disk unavailable" in log[0]


def test_map_bounds_and_numerical_rng_unchanged(tmp_path):
    v = viz.LiveViz(tmp_path, config(max_points=101), fresh=True)
    rng = np.random.default_rng(7)
    points = rng.normal(size=(100000, 3))
    state = json.dumps(rng.bit_generator.state, sort_keys=True)
    original = points.tobytes()
    expected = points.T @ points
    v.publish({"cost": float("nan")}, map_data={"points": points, "trajectory": points})
    v.close()
    assert points.tobytes() == original
    assert json.dumps(rng.bit_generator.state, sort_keys=True) == state
    assert np.array_equal(points.T @ points, expected)
    event = events(tmp_path)[-1]
    assert event["cost"] is None
    with np.load(tmp_path / "viz" / event["assets"]["map"]) as z:
        assert len(z["points"]) <= 101 and len(z["trajectory"]) <= 4096


def test_log_rotation_retains_complete_snapshot_and_assets_prune(tmp_path):
    v = viz.LiveViz(tmp_path, config(max_log_bytes=1024, retain_seconds=2), fresh=True)
    old = tmp_path / "viz/assets/stale.npz"
    old.write_bytes(b"old")
    import os
    os.utime(old, (0, 0))
    v.publish({"long_note": "a" * 700, "cameras": {"one": {"cost": 1}}})
    time.sleep(.3)
    v.publish({"cameras": {"two": {"cost": 2}}})
    v.close()
    stream = events(tmp_path)
    assert len(stream) == 1
    assert stream[0]["seq"] == 2
    assert set(stream[0]["cameras"]) == {"one", "two"}
    assert not old.exists()


def test_camera_control_and_image_rate(tmp_path):
    v = viz.LiveViz(tmp_path, config(), {"run_id": "a"}, fresh=True)
    control = tmp_path / "viz/control.json"
    control.write_text(json.dumps({"run_id": "a", "camera": "front", "enabled": True}))
    v._read_control()
    assert not v.preview(SimpleNamespace(root=tmp_path), "S01", "rear", {})
    assert v._reserve_image("front")
    assert not v._reserve_image("front")
    assert not v._reserve_image("rear")
    v.close()


def test_rgb_preview_uses_exposure_midpoint_deskew_and_actual_track_ids(tmp_path):
    root = tmp_path
    camera, window = "camera_front5", "S01"
    cam_dir = root / "extract" / window / "cam" / camera
    lidar_dir = root / "extract" / window / "lidar"
    for d in (cam_dir, lidar_dir, root / "tracks", root / "lo"):
        d.mkdir(parents=True, exist_ok=True)
    stamp = 2_000_000_000
    cv2.imwrite(str(cam_dir / f"{stamp}.jpg"), np.full((40, 60, 3), 100, np.uint8))
    np.savez(root / "extract" / window / "cam_index.npz", channel=[camera], effective_ns=[stamp], header_ns=[stamp - 1000000])
    tau = np.array([1_900_000_000, 2_000_000_000, 2_100_000_000])
    T = np.tile(np.eye(4), (3, 1, 1)); T[:, 0, 3] = [-1., 0., 1.]
    np.savez(root / "lo/ref_S01.npz", tau=tau, T_w_L=T, ext=True)
    scan = np.zeros(2, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("t", "f4")])
    scan["x"] = 5.; scan["t"] = [0., .1]
    np.save(lidar_dir / "1950000000.npy", scan)
    np.savez(root / "tracks/tracks_S01_camera_front5.npz", frame_ns=[stamp - 10000000, stamp],
             obs_frame=[0, 1, 1], obs_track=[7, 7, 9], obs_xy=[[10., 20.], [11., 21.], [50., 20.]])
    req = {"workdir": str(root), "window": window, "camera": camera, "camera_state": {"dt_s": .01}, "context": {}}
    encoded, arrays, meta = build_preview(req, config(), {})
    assert encoded.startswith(b"\xff\xd8")
    assert meta["source_stamp_ns"] == stamp - 1000000
    assert meta["capture_stamp_ns"] == stamp + 10000000
    assert np.allclose(arrays["points_lidar"][:, 0], [4.4, 5.4])
    assert np.array_equal(arrays["tracks_uv"], [[11., 21.]])
    assert np.array_equal(arrays["tracks_prev_uv"], [[10., 20.]])


def test_resume_recovers_committed_append_and_discards_only_partial_suffix(tmp_path):
    v = viz.LiveViz(tmp_path, config(), {"run_id": "run"}, fresh=True)
    v.publish({"iteration": 1})
    v.close()
    last = events(tmp_path)[-1]
    last.update(seq=2, iteration=2)
    with open(tmp_path / "viz/events.jsonl", "ab") as f:
        f.write(json.dumps(last).encode() + b"\n{\"seq\":3")
    # Simulates interruption after an append, before .state.json atomic replacement.
    resumed = viz.LiveViz(tmp_path, config(), {"run_id": "run"})
    resumed.publish({"iteration": 3})
    resumed.close()
    assert [e["seq"] for e in events(tmp_path)] == [1, 2, 3]
    assert events(tmp_path)[-1]["iteration"] == 3


def test_preview_cannot_overwrite_newer_coalesced_solver_estimate(tmp_path, monkeypatch):
    import nontarget_cal.viz_preview as preview_module
    v = viz.LiveViz(tmp_path, config(), fresh=True)
    def preview(*args):
        return b"jpg", {"points_lidar": np.ones((2, 3))}, {"width": 2, "height": 2}
    monkeypatch.setattr(preview_module, "build_preview", preview)
    v._process({"state": {"cameras": {"front": {"iteration": 9, "cost": 1.}}},
                "preview": {"camera": "front", "camera_state": {"iteration": 2, "cost": 50., "model": "equidistant"}}})
    v.close()
    state = events(tmp_path)[-1]["cameras"]["front"]
    assert state == {"iteration": 9, "cost": 1., "model": "equidistant"}


def test_configured_rates_are_respected_below_hard_caps(tmp_path):
    v = viz.LiveViz(tmp_path, config(state_hz=2, image_hz=.5, map_hz=.25), fresh=True)
    assert v.intervals == {"state": .5, "image": 2., "map": 4.}
    v.close()


def test_identical_input_new_attempt_carries_only_map(tmp_path):
    context = {"run_id": "old", "manifest": {"input_identity": "same-inputs"}}
    v = viz.LiveViz(tmp_path, config(), context, fresh=True)
    v.publish({"stage": "validation", "progress": 1., "window_id": "S02", "map_frame": "lidar_window:bag/S02",
               "cameras": {"front": {"cost": 99, "gate_pass": False}}, "gate": {"pass": False, "status": "fail"}},
              map_data={"points": np.ones((10, 3))})
    v.close()
    old_map = events(tmp_path)[-1]["assets"]["map"]
    fresh = viz.LiveViz(tmp_path, config(), {**context, "run_id": "new"}, fresh=True)
    fresh.publish({"status_text": "new attempt"})
    fresh.close()
    event = events(tmp_path)[-1]
    assert event["run_id"] == "new" and event["seq"] == 1
    assert event["assets"] == {"map": old_map}
    assert event["map_frame"] == "lidar_window:bag/S02"
    assert event["map_source"] == "previous-attempt-cache" and event["previous_map_run_id"] == "old"
    assert event["map_source_window"] == event["map_window"] == "S02"
    assert event["cameras"] == {} and event["gate"] == {"status": "pending", "pass": None}
    assert event["stage"] == "extract" and event["progress"] == 0.
    mismatch = viz.LiveViz(tmp_path, config(), {"run_id": "changed", "manifest": {"input_identity": "different"}}, fresh=True)
    mismatch.publish({"status_text": "changed input"})
    mismatch.close()
    assert events(tmp_path)[-1]["assets"] == {}
    assert "map_source" not in events(tmp_path)[-1]


def test_retained_map_rejects_symlink_escape(tmp_path):
    context = {"run_id": "old", "manifest": {"input_identity": "same"}}
    v = viz.LiveViz(tmp_path, config(), context, fresh=True)
    v.publish({"map_frame": "lidar_window:S01"}, map_data={"points": np.ones((2, 3))})
    v.close()
    event = events(tmp_path)[-1]
    path = tmp_path / "viz" / event["assets"]["map"]
    outside = tmp_path / "outside.npz"
    path.rename(outside)
    path.symlink_to(outside)
    fresh = viz.LiveViz(tmp_path, config(), {**context, "run_id": "new"}, fresh=True)
    fresh.publish({"stage": "extract"})
    fresh.close()
    assert events(tmp_path)[-1]["assets"] == {}


def test_preview_window_selects_its_bag_instead_of_first_worker_bag(tmp_path, monkeypatch):
    from nontarget_cal.viz_preview import _bag_id
    records = [{"name": "S01", "bag_id": "day"}, {"name": "S02", "bag_id": "night"}]
    context = {"windows": records, "window_id": "S01", "bag_id": "day"}
    v = viz.LiveViz(tmp_path, config(), context, fresh=True)
    jobs = []
    monkeypatch.setattr(v, "_enqueue", jobs.append)
    assert v.preview(SimpleNamespace(root=tmp_path), "S02", "camera_front5", {})
    request = jobs[0]["preview"]
    assert request["context"]["bag_id"] == "night"
    assert request["context"]["window_id"] == "S02"
    assert _bag_id(request) == "night"
    (tmp_path / "windows").mkdir()
    (tmp_path / "windows/plan.json").write_text(json.dumps({"windows": records}))
    request["context"] = {"window_id": "S01", "bag_id": "day"}
    assert _bag_id(request) == "night"
    v.close()


def test_preview_source_cache_is_bounded_lru(tmp_path, monkeypatch):
    import nontarget_cal.viz_preview as preview_module
    scan_path = tmp_path / "1000000000.npy"
    def source(*args):
        return {"image": b"jpg", "arrays": {}, "trajectory": SimpleNamespace(Tm=lambda _: np.eye(4)[None]),
                "lidar_files": [scan_path], "scan_path": scan_path, "world": np.ones((2, 3)),
                "base_stamp": 1_000_000_000, "height": 100, "meta": {}}
    monkeypatch.setattr(preview_module, "_source", source)
    cache = {}
    def preview(index):
        request = {"workdir": str(tmp_path), "window": "S01", "camera": f"camera_{index}", "camera_state": {}}
        preview_module.build_preview(request, config(), cache)
    for i in range(20):
        preview(i)
    assert len(cache) == 16
    assert [key[2] for key in cache] == [f"camera_{i}" for i in range(4, 20)]
    preview(4)
    preview(20)
    assert len(cache) == 16 and next(iter(cache))[2] == "camera_6"
    assert list(cache)[-2][2] == "camera_4"


def test_cached_map_samples_raw_sweeps_and_never_reads_full_map_archive(tmp_path):
    from nontarget_cal.viz_preview import build_cached_map
    window = "S01"
    lidar = tmp_path / "extract" / window / "lidar"
    lidar.mkdir(parents=True)
    (tmp_path / "lo").mkdir()
    stamps = np.array([1_000_000_000, 1_100_000_000, 1_200_000_000])
    T = np.tile(np.eye(4), (3, 1, 1)); T[:, 0, 3] = [0., .1, .2]
    np.savez(tmp_path / "lo/ref_S01.npz", tau=stamps, T_w_L=T, ext=True)
    (tmp_path / "lo/map_S01.npz").write_bytes(b"must never be read")
    scan = np.zeros(100, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("t", "f4")])
    scan["x"] = 5.; scan["t"] = .05
    for stamp in stamps:
        np.save(lidar / f"{stamp}.npy", scan)
    req = {"workdir": str(tmp_path), "window": window,
           "context": {"windows": [{"name": "S01", "bag_id": "night"}]}}
    state, arrays = build_cached_map(req, config(max_points=12))
    assert state["map_source"] == "cached-lo-sample"
    assert state["map_frame"] == "lidar_window:night/S01"
    assert arrays["points"].shape == (12, 3)
    assert arrays["trajectory"].shape == (3, 3)
    assert np.allclose(arrays["points"][:, 0], np.repeat([5.05, 5.15, 5.25], 4))
    v = viz.LiveViz(tmp_path, config(max_points=12), req["context"], fresh=True)
    assert v.cached_map(SimpleNamespace(root=tmp_path), "S01")
    v.close()
    event = events(tmp_path)[-1]
    assert event["map_source"] == "cached-lo-sample" and event["map_window"] == "S01"
    assert (tmp_path / "viz" / event["assets"]["map"]).exists()


def test_cached_map_cannot_replace_existing_solver_map(tmp_path, monkeypatch):
    import nontarget_cal.viz_preview as preview_module
    context = {"run_id": "a"}
    first = viz.LiveViz(tmp_path, config(), context, fresh=True)
    first.publish({"map_frame": "lidar_window:night/S01", "window_id": "S01"}, map_data={"points": np.ones((2, 3))})
    first.close()
    old = events(tmp_path)[-1]
    def must_not_build(*args):
        raise AssertionError("cached source must not load when a solver map exists")
    monkeypatch.setattr(preview_module, "build_cached_map", must_not_build)
    next_worker = viz.LiveViz(tmp_path, config(), context)
    next_worker.cached_map(SimpleNamespace(root=tmp_path), "S02")
    next_worker.close()
    final = events(tmp_path)[-1]
    assert final["map_frame"] == old["map_frame"]
    assert final["assets"]["map"] == old["assets"]["map"]
    assert final["map_source"] == "solver"
    assert not next_worker._warned


def test_cached_map_coalescing_preserves_newer_complete_state(tmp_path, monkeypatch):
    import nontarget_cal.viz_preview as preview_module
    def cached(*args):
        return ({"stage": "lidar_odometry", "progress": .1, "window_id": "S01", "bag_id": "night",
                 "map_frame": "lidar_window:night/S01", "pass": "old", "cameras": {"front": {"cost": 99}}},
                {"points": np.ones((2, 3), np.float32), "trajectory": np.zeros((2, 3), np.float32)})
    monkeypatch.setattr(preview_module, "build_cached_map", cached)
    v = viz.LiveViz(tmp_path, config(), fresh=True)
    # The mailbox lock forces these two calls to coalesce before the I/O thread
    # can dequeue, matching a fully cached run that completes immediately.
    with v._cv:
        assert v.cached_map(SimpleNamespace(root=tmp_path), "S01")
        v.publish({"stage": "validation", "progress": 1., "pass": "final", "status": "complete",
                   "window_id": "S02", "bag_id": "day", "cameras": {"front": {"cost": 1.}},
                   "gate": {"pass": True, "status": "pass"}})
    v.close()
    event = events(tmp_path)[-1]
    assert event["stage"] == "validation" and event["status"] == "complete"
    assert event["progress"] == 1. and event["pass"] == "final"
    assert event["window_id"] == "S02" and event["bag_id"] == "day"
    assert event["cameras"]["front"]["cost"] == 1. and event["gate"]["pass"] is True
    assert event["map_frame"] == "lidar_window:night/S01"
    assert event["map_source_window"] == "S01" and event["map_bag_id"] == "night"
    assert event["map_source"] == "cached-lo-sample"


def test_explicit_manifest_cameras_replace_default_sensor_set(tmp_path):
    from nontarget_cal.config import load_config
    cfg = load_config()
    default = viz._default_manifest(cfg)
    assert len(default["cameras"]) == 16
    rgb = {name: value for name, value in default["cameras"].items() if value["sensor"] == "rgb"}
    for i, supplied in enumerate((rgb, {"camera_front5": rgb["camera_front5"]}, {})):
        root = tmp_path / str(i)
        v = viz.LiveViz(root, cfg, {"manifest": {"cameras": supplied}}, fresh=True)
        assert set(v.manifest["cameras"]) == set(supplied)
        v.close()
        on_disk = json.loads((root / "viz/manifest.json").read_text())
        assert set(on_disk["cameras"]) == set(supplied)
        worker = viz.LiveViz(root, cfg)
        assert set(worker.manifest["cameras"]) == set(supplied)
        worker.close()


def test_image_publication_rate_survives_variable_encoding_delay(tmp_path, monkeypatch):
    import nontarget_cal.viz_preview as preview_module
    first_started = threading.Event()
    calls = []
    def build(*args):
        calls.append(time.monotonic())
        if len(calls) == 1:
            first_started.set()
            time.sleep(.35)
        return b"jpg", {"points_lidar": np.ones((2, 3))}, {"width": 2, "height": 2}
    monkeypatch.setattr(preview_module, "build_preview", build)
    v = viz.LiveViz(tmp_path, config(), fresh=True)
    ws = SimpleNamespace(root=tmp_path)
    assert v.preview(ws, "S01", "front", {"cost": 50.})
    assert first_started.wait(2)
    # The second preparation starts > 1 s later, but its encoding is fast; without
    # an emission reservation the published image gap would be roughly 0.7 s.
    deadline = calls[0] + 1.05
    while time.monotonic() < deadline:
        time.sleep(min(.01, deadline - time.monotonic()))
    with v._cv:
        assert v.preview(ws, "S01", "front", {"cost": 10.})
        v.publish({"stage": "validation", "progress": 1., "status": "complete",
                   "cameras": {"front": {"cost": 1.}}, "gate": {"status": "pass", "pass": True}})
    v.close()
    stream = events(tmp_path)
    images = []
    for event in stream:
        image = event.get("assets", {}).get("matching", {}).get("front", {}).get("image")
        if image and (not images or image != images[-1][1]):
            images.append((event["t"], image))
    assert len(images) == 2
    assert images[1][0] - images[0][0] >= 1.
    assert stream[-1]["status"] == "complete" and stream[-1]["stage"] == "validation"
    assert stream[-1]["gate"]["pass"] is True and stream[-1]["cameras"]["front"]["cost"] == 1.
