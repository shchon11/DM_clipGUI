#!/usr/bin/env python3
"""실시간 투영 확인 계산 시험 (ROS 없이).

    python3 -m pytest -q test/test_projection_check.py
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import projection_check as pc     # noqa: E402

cv2 = pytest.importorskip("cv2")


def _pts(n=500, seed=0):
    rng = np.random.default_rng(seed)
    return np.c_[rng.uniform(-8, 8, n), rng.uniform(-5, 5, n), rng.uniform(2, 40, n)]


def test_equidistant_matches_opencv_fisheye():
    K = [1055.4, 0, 961.2, 0, 1054.9, 598.7, 0, 0, 1]
    D = [-0.0396, 0.0021, -0.0009, 0.0001]
    X = _pts()
    uv, depth, idx = pc.project(X, K, D, "equidistant", 1920, 1200)
    ref, _ = cv2.fisheye.projectPoints(X[idx].reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                                       np.array(K).reshape(3, 3), np.array(D))
    assert len(uv) > 100 and np.abs(uv - ref.reshape(-1, 2)).max() < 1e-6


def test_plumb_bob_matches_opencv():
    K = [682.1, 0, 321.0, 0, 681.5, 239.2, 0, 0, 1]
    D = [0.0116, -0.03, 0.0005, -0.0003, 0.01]
    X = _pts(seed=1)
    uv, _, idx = pc.project(X, K, D, "plumb_bob", 640, 480)
    ref, _ = cv2.projectPoints(X[idx].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), np.array(K).reshape(3, 3),
                               np.array(D))
    assert len(uv) > 50 and np.abs(uv - ref.reshape(-1, 2)).max() < 1e-6


def test_tf_lookup_chain_and_disconnected():
    g = pc.TfGraph()
    g.add("os_sensor", "os_lidar", (0, 0, 0.03), (0, 0, 1, 0))           # 180° about z
    g.add("os_lidar", "cam_optical", (0.1, 0.2, 0.3), (0, 0, 0, 1))
    g.add("flir_rig_frame", "old_optical", (0, 0, 0), (0, 0, 0, 1))
    T, path = g.lookup("cam_optical", "os_lidar")
    assert path == ["os_lidar", "cam_optical"] and np.allclose(T[:3, 3], [-0.1, -0.2, -0.3])
    T, path = g.lookup("os_sensor", "cam_optical")                      # x_sensor = E1 E2 x_cam
    assert np.allclose(T[:3, 3], [-0.1, -0.2, 0.33])
    ok, _ = pc.judge_tf(*g.lookup("cam_optical", "os_lidar"), "os_lidar")
    assert ok
    ok, why = pc.judge_tf(*g.lookup("old_optical", "os_lidar"), "os_lidar")
    assert ok is False and "flir_rig_frame" in why
    ok, why = pc.judge_tf(*g.lookup("missing_optical", "os_lidar"), "os_lidar")
    assert ok is False and "TF 가 없음" in why
    g.add("os_lidar", "ident_optical", (0, 0, 0), (0, 0, 0, 1))
    ok, why = pc.judge_tf(*g.lookup("ident_optical", "os_lidar"), "os_lidar")
    assert ok is False and "항등" in why


def test_judge_camera_info():
    assert pc.judge_camera_info(None)[0] is None
    ph = {"model": "plumb_bob", "d": [0.0] * 5, "k": [1045.0, 0, 960, 0, 1045.0, 600, 0, 0, 1]}
    assert pc.judge_camera_info(ph)[0] is False
    assert pc.judge_camera_info(dict(ph, k=[0.0] * 9))[0] is False
    ok, txt = pc.judge_camera_info({"model": "equidistant", "d": [-0.04, 0, 0, 0], "k": [1055.4] + [0] * 8})
    assert ok and "equidistant" in txt


def _cloud(xyz_hw):
    h, w, _ = xyz_hw.shape
    rec = np.zeros((h, w), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("pad", "<f4")])
    rec["x"], rec["y"], rec["z"] = xyz_hw[..., 0], xyz_hw[..., 1], xyz_hw[..., 2]
    fields = [SimpleNamespace(name=n, offset=o) for n, o in (("x", 0), ("y", 4), ("z", 8))]
    return SimpleNamespace(fields=fields, is_bigendian=False, point_step=16, width=w, height=h, data=rec.tobytes())


def test_cloud_edges_finds_foreground_outline():
    # 한 줄 32칸: 20 m 벽 앞 5 m 기둥 (10..14 칸) — 윤곽은 기둥 양 끝 (앞쪽 점)만
    w = 32
    az = np.linspace(-0.5, 0.5, w)
    r = np.full(w, 20.0)
    r[10:15] = 5.0
    xyz = np.stack([r * np.cos(az), r * np.sin(az), np.zeros(w)], 1)[None].repeat(2, 0)
    e = pc.cloud_edges_xyz(_cloud(xyz))
    rr = np.linalg.norm(e, axis=1)
    assert len(e) == 4 and np.allclose(rr, 5.0, atol=1e-4)            # 두 줄 × 양 끝
    assert len(pc.cloud_xyz(_cloud(xyz))) == 64


def test_overlay_draws_inside_image():
    K = [500.0, 0, 320, 0, 500.0, 240, 0, 0, 1]
    info = {"model": "plumb_bob", "d": [0.0] * 5, "k": K, "width": 640, "height": 480}
    img = np.zeros((240, 320, 3), np.uint8)                            # 반으로 줄인 영상
    out, n = pc.overlay(img, np.array([[0.0, 0.0, 10.0], [0.0, 0.0, -5.0]]), info, 0.5)
    assert n == 1 and out[120, 160].any()                              # 광축 위 점 → 가운데
