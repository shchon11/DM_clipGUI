"""LO hooks observe real solver state without changing its numerical outputs."""
import sys
from types import SimpleNamespace

import numpy as np

from nontarget_cal.lo import kiss, maps, refine


class Recorder:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.context = {"bag_id": "b1", "window_id": "S01"}
        self.events = []

    def due(self, key):
        assert key == "map"
        return self.enabled

    def publish(self, event, *, map_data, force):
        assert force
        # A producer must only receive isolated copies, never solver state.
        self.events.append((event, {k: v.copy() for k, v in map_data.items()}))
        for value in map_data.values():
            value[:] = 99


def install(monkeypatch, recorder):
    monkeypatch.setitem(sys.modules, "nontarget_cal.viz", SimpleNamespace(get_viz=lambda: recorder))


def test_capture_runs_only_when_due_and_failure_is_contained(monkeypatch):
    recorder = Recorder(False)
    install(monkeypatch, recorder)
    monkeypatch.setattr(maps, "_VIZ_WARNING", False)
    calls = []
    warnings = []
    monkeypatch.setattr(maps.logging.getLogger(maps.__name__), "warning",
                        lambda message, **kwargs: warnings.append(message))

    def capture():
        calls.append(1)
        raise RuntimeError("observation failure")

    maps._emit_map("S01", capture)
    assert not calls
    recorder.enabled = True
    maps._emit_map("S01", capture)
    maps._emit_map("S01", capture)
    assert calls == [1, 1]
    assert len(warnings) == 1


def test_cloud_sampling_is_bounded_deterministic_and_does_not_mutate():
    cloud = np.arange(90_000, dtype=np.float64).reshape(-1, 3)
    original = cloud.copy()
    a = maps._sample_clouds([cloud] * 25, limit=1234)
    b = maps._sample_clouds([cloud] * 25, limit=1234)
    assert a.shape[0] <= 1234
    assert a.dtype == np.float32
    assert np.array_equal(a, b)
    a[:] = 0
    assert np.array_equal(cloud, original)
    poses = np.tile(np.eye(4), (9000, 1, 1))
    path = maps._sample_trajectory(poses)
    assert len(path) <= 4096
    path[:] = 99
    assert not poses[:, :3, 3].any()


def test_real_lo_numerics_bit_identical_with_observation_enabled(tmp_path, monkeypatch):
    """Exercise installed KISS, one GN iteration, and map deskew on identical inputs."""
    seg = tmp_path / "S01"
    (seg / "lidar").mkdir(parents=True)
    x, y = np.meshgrid(np.linspace(3.0, 9.0, 41), np.linspace(-3.0, 3.0, 41))
    dtype = [("x", "f4"), ("y", "f4"), ("z", "f4"), ("t", "f4")]
    for i in range(20):
        s = np.zeros(x.size, dtype=dtype)
        s["x"], s["y"], s["z"] = x.ravel() - i * 0.002, y.ravel(), -2.5
        s["t"] = np.linspace(0, 0.1, len(s))
        np.save(seg / "lidar" / f"{1_000_000_000 + i * 100_000_000}.npy", s)
    monkeypatch.setattr(refine, "PAIR_THREADS", 1)
    paths = []
    for enabled in (False, True):
        recorder = Recorder(enabled)
        install(monkeypatch, recorder)
        out = tmp_path / str(enabled)
        out.mkdir()
        kiss.run_kiss(seg, out / "kiss.npz", threads=1, log=lambda _: None)
        refine.refine(seg, out / "kiss.npz", out / "ref.npz", iters=1,
                      offsets=(1,), max_src=120, log=lambda _: None)
        maps.build_map(seg, out / "ref.npz", out / "map.npz", log=lambda _: None)
        paths.append(out)
    for name, keys in (("kiss", ("tau", "header", "T_w_L")),
                       ("ref", ("tau", "T_w_L", "ext")),
                       ("map", ("P", "start", "hdr"))):
        with np.load(paths[0] / f"{name}.npz") as a, np.load(paths[1] / f"{name}.npz") as b:
            for key in keys:
                assert np.array_equal(a[key], b[key]), (name, key)
    assert {e["lo_phase"] for e, _ in recorder.events} == {"kiss", "refine", "map"}
    assert all(e["map_frame"] == "lidar_window:b1/S01" for e, _ in recorder.events)
    assert all(len(data["points"]) <= maps._VIZ_MAX_POINTS for _, data in recorder.events)
    assert max(len(data["trajectory"]) for e, data in recorder.events if e["lo_phase"] == "kiss") == 20
