"""Headless protocol/geometry/replay regression tests (no Qt or GPU required)."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from calib_viz import SCHEMA
from calib_viz.geometry import frustum_corners, interpolate_pose, project_camera, project_fisheye, vehicle_from_camera
from calib_viz.replay import CAMERA_NAMES, MAX_ASSET_SETS, ReplayProducer, _sample_scan
from calib_viz.stream import (MAX_RECORD_BYTES, StreamReader, append_snapshot, atomic_json, atomic_npz,
                              snapshot_calibrations, validate_snapshot)


def snapshot(seq=0):
    return {"schema": SCHEMA, "seq": seq, "cameras": {"test": {
        "T_cam_lidar": np.eye(4).tolist(), "sigma_rot_deg": .1,
        "sigma_pos_mm": 1, "reprojection_px": .8}}}


def manifest():
    return {"schema": SCHEMA, "R_lidar_V": np.eye(3).tolist(), "cameras": {
        "test": {"K": np.eye(3).tolist(), "D": [0, 0, 0, 0],
                 "model": "equidistant", "width": 640, "height": 480}}}


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        atomic_json(self.root / "manifest.json", manifest())
        self.reader = StreamReader(self.root, max_events=3)

    def tearDown(self):
        self.tmp.cleanup()

    def test_partial_line_only_publishes_after_newline(self):
        payload = json.dumps(snapshot()).encode()
        path = self.root / "events.jsonl"
        path.write_bytes(payload[:40])
        self.assertEqual(self.reader.poll(), [])
        with path.open("ab") as handle:
            handle.write(payload[40:])
        self.assertEqual(self.reader.poll(), [])
        with path.open("ab") as handle:
            handle.write(b"\n")
        self.assertEqual([e["seq"] for e in self.reader.poll()], [0])

    def test_malformed_bad_rotation_and_duplicate_do_not_poison_tail(self):
        bad = snapshot(1)
        bad["cameras"]["test"]["T_cam_lidar"][0][0] = 3
        (self.root / "events.jsonl").write_text("no json\n" + json.dumps(bad) + "\n")
        append_snapshot(self.root, snapshot(2))
        append_snapshot(self.root, snapshot(2))
        append_snapshot(self.root, snapshot(3))
        self.assertEqual([e["seq"] for e in self.reader.poll()], [2, 3])
        self.assertEqual(len(self.reader.errors), 2)

    def test_replacement_and_truncation_reset_sequence(self):
        append_snapshot(self.root, snapshot(99))
        self.reader.poll()
        path = self.root / "events.jsonl"
        path.write_text(json.dumps(snapshot()) + "\n")
        self.assertEqual(self.reader.poll()[0]["seq"], 0)
        replacement = self.root / "replacement"
        replacement.write_text(json.dumps(snapshot()) + "\n")
        replacement.replace(path)
        self.assertEqual(self.reader.poll()[0]["seq"], 0)

    def test_poll_bounds_and_overlong_partial_recovery(self):
        for seq in range(8):
            append_snapshot(self.root, snapshot(seq))
        self.assertEqual([e["seq"] for e in self.reader.poll()], [5, 6, 7])
        with (self.root / "events.jsonl").open("ab") as handle:
            handle.write(b"x" * (MAX_RECORD_BYTES + 1))
        self.assertEqual(self.reader.poll(), [])
        self.assertLessEqual(len(self.reader._partial), MAX_RECORD_BYTES)
        with (self.root / "events.jsonl").open("ab") as handle:
            handle.write(b"tail\n")
        append_snapshot(self.root, snapshot(8))
        self.assertEqual(self.reader.poll()[0]["seq"], 8)

    def test_missing_manifest_events_and_assets(self):
        absent = StreamReader(self.root / "absent")
        self.assertEqual(absent.poll(), [])
        self.assertEqual(absent.manifest, {})
        with self.assertRaises(FileNotFoundError):
            self.reader.load_asset("assets/missing.npz")

    def test_asset_containment_including_symlinks(self):
        for path in ("../outside.npz", "/tmp/outside.npz", ""):
            with self.assertRaises(ValueError):
                self.reader.asset_path(path)
        (self.root / "external").symlink_to(self.root.parent)
        with self.assertRaises(ValueError):
            self.reader.asset_path("external/outside.npz")

    def test_numeric_npz_and_rgb_image_load(self):
        atomic_npz(self.root / "asset.npz", points=np.arange(12).reshape(4, 3))
        np.testing.assert_equal(self.reader.load_asset("asset.npz")["points"], np.arange(12).reshape(4, 3))
        image = np.zeros((8, 8, 3), np.uint8)
        image[:, :, 2] = 255
        cv2.imwrite(str(self.root / "red.png"), image)
        np.testing.assert_equal(self.reader.load_asset("red.png")[0, 0], [255, 0, 0])
        np.savez(self.root / "object.npz", value=np.array([{}], dtype=object))
        with self.assertRaises(ValueError):
            self.reader.load_asset("object.npz")

    def test_corrupt_asset_does_not_poison_next_snapshot(self):
        (self.root / "bad.npz").write_bytes(b"not a zip archive")
        with self.assertRaises(ValueError):
            self.reader.load_asset("bad.npz")
        atomic_npz(self.root / "wrong_shape.npz", points=np.zeros((5, 2)))
        with self.assertRaises(ValueError):
            self.reader.load_asset("wrong_shape.npz")
        atomic_npz(self.root / "nan.npz", points=np.array([[np.nan, 0., 0.]]))
        with self.assertRaises(ValueError):
            self.reader.load_asset("nan.npz")
        append_snapshot(self.root, snapshot(5))
        self.assertEqual(self.reader.poll()[0]["seq"], 5)

    def test_optional_fields_validated_before_ui(self):
        for index, (key, value) in enumerate([
                ("progress", []), ("eta_s", "unknown"), ("progress", float("nan")),
                ("assets", []), ("gate", []), ("provenance", []), ("stage", [])]):
            event = snapshot(index)
            event[key] = value
            with (self.root / "events.jsonl").open("a") as handle:
                handle.write(json.dumps(event) + "\n")
        append_snapshot(self.root, snapshot(10))
        self.assertEqual([e["seq"] for e in self.reader.poll()], [10])
        self.assertEqual(len(self.reader.errors), 7)

    def test_manifest_validation_rejects_nonobject_and_invalid_intrinsics(self):
        for bad in ([], {"schema": SCHEMA}, {**manifest(), "R_lidar_V": []}):
            atomic_json(self.root / "manifest.json", bad)
            with self.assertRaises(ValueError):
                self.reader.refresh_manifest()

    def test_attach_seeks_latest_bounded_tail(self):
        path = self.root / "events.jsonl"
        with path.open("w") as handle:
            for index in range(3000):
                handle.write(json.dumps(snapshot(index)) + "\n")
        reader = StreamReader(self.root, max_events=3, max_bytes=MAX_RECORD_BYTES)
        self.assertEqual([e["seq"] for e in reader.poll()], [2997, 2998, 2999])

    def test_appended_attempt_and_rotated_resume_keep_latest_generation(self):
        first = dict(manifest(), run_id='first')
        atomic_json(self.root / 'manifest.json', first)
        self.reader.refresh_manifest()
        append_snapshot(self.root, dict(snapshot(99), run_id='first'))
        self.assertEqual(self.reader.poll()[0]['seq'], 99)
        atomic_json(self.root / 'manifest.json', dict(first, run_id='second'))
        self.reader.refresh_manifest()
        append_snapshot(self.root, dict(snapshot(100), run_id='first'))
        append_snapshot(self.root, dict(snapshot(0), run_id='second'))
        self.assertEqual([(e['run_id'], e['seq']) for e in self.reader.poll()], [('second', 0)])
        append_snapshot(self.root, dict(snapshot(1), run_id='second'))
        self.assertEqual(self.reader.poll()[0]['seq'], 1)
        path = self.root / 'replacement'
        path.write_text(json.dumps(dict(snapshot(1), run_id='second')) + '\n')
        path.replace(self.root / 'events.jsonl')
        self.assertEqual(self.reader.poll()[0]['seq'], 1)
        append_snapshot(self.root, dict(snapshot(2), run_id='second'))
        self.assertEqual(self.reader.poll()[0]['seq'], 2)

    def test_copy_truncate_that_has_regrown_is_detected(self):
        append_snapshot(self.root, snapshot(99))
        self.reader.poll()
        payload = json.dumps(dict(snapshot(0), note='new attempt' * 100)) + '\n'
        (self.root / 'events.jsonl').write_text(payload)
        self.assertEqual(self.reader.poll()[0]['seq'], 0)

    def test_current_intrinsics_override_static_manifest_and_validate(self):
        baseline = manifest()
        current = snapshot()
        current['cameras']['test'].update(K=[[20., 0, 16], [0, 25., 16], [0, 0, 1]],
                                          D=[.01, 0, 0, 0], cost=5., time_offset_s=-.02)
        validate_snapshot(current)
        merged = snapshot_calibrations(baseline, current)
        self.assertEqual(merged['test']['K'][0][0], 20.)
        self.assertEqual(baseline['cameras']['test']['K'][0][0], 1.)
        self.assertEqual(merged['test']['D'][0], .01)
        for key, invalid in [('K', [[-1., 0, 0], [0, 1., 0], [0, 0, 1]]),
                             ('D', [0, 0]), ('time_offset_s', float('inf')),
                             ('cost', -1), ('row_readout_s', float('nan')),
                             ('iteration', -1), ('total_iterations', '12')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                bad = snapshot()
                bad['cameras']['test'][key] = invalid
                validate_snapshot(bad)

    def test_final_validation_clears_iteration_with_null(self):
        event = dict(snapshot(22), run_id='real-run', stage='validation', progress=1.,
                     solver_pass='final', iteration=None, total_iterations=None, complete=True,
                     gate={'status': 'pass', 'pass': True, 'source': 'solver.validation'})
        event['cameras']['test'].update(gate_pass=True, state='converged', cost=None,
                                       metric_source='solver.validation', solver_pass='final',
                                       iteration=None, total_iterations=None)
        append_snapshot(self.root, event)
        self.assertEqual(self.reader.poll(), [event])
        self.assertEqual(list(self.reader.errors), [])


class GeometryTests(unittest.TestCase):
    def test_frustum_responds_to_current_intrinsics(self):
        camera = {'width': 640, 'height': 480,
                  'K': [[500, 0, 320], [0, 500, 240], [0, 0, 1]]}
        wide = frustum_corners(camera, .3)
        camera['K'] = [[1000, 0, 320], [0, 1000, 240], [0, 0, 1]]
        narrow = frustum_corners(camera, .3)
        self.assertLess(abs(narrow[0, 0]), abs(wide[0, 0]))
        np.testing.assert_allclose(np.linalg.norm(narrow, axis=1), .3)

    def test_lidar_backward_axis_and_optical_pose_inverse(self):
        T = np.eye(4)
        T[:3, 3] = [1, 2, 3]
        pose = vehicle_from_camera(T)
        np.testing.assert_allclose(pose[:3, 3], [1, 2, -3])
        np.testing.assert_allclose(pose[:3, :3], np.diag([-1, -1, 1]))

    def test_so3_midpoint_and_exact_endpoints(self):
        a, b = np.eye(4), np.eye(4)
        b[:3, :3], _ = cv2.Rodrigues(np.array([0., 0., np.pi / 2]))
        b[:3, 3] = [2, 0, 0]
        middle = interpolate_pose(a, b, .5)
        np.testing.assert_allclose(middle[:3, 3], [1, 0, 0])
        np.testing.assert_allclose(middle[:3, :3].T @ middle[:3, :3], np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.arctan2(middle[1, 0], middle[0, 0]), np.pi / 4)
        np.testing.assert_array_equal(interpolate_pose(a, b, 1), b)
        np.testing.assert_array_equal(interpolate_pose(a, b, 0), a)

    def test_fisheye_matches_opencv_and_rejects_behind_camera(self):
        points = np.array([[0, 0, 2], [.5, .4, 1], [-.4, .8, 2], [0, 0, -1.]])
        K = np.array([[500., 0, 320], [0, 490, 240], [0, 0, 1]])
        D = np.array([.01, -.03, .002, .0001])
        uv, depth, valid = project_fisheye(points, np.eye(4), K, D)
        reference, _ = cv2.fisheye.projectPoints(points[:3, None], np.zeros(3), np.zeros(3), K, D)
        np.testing.assert_allclose(uv[:3], reference[:, 0], atol=1e-9)
        np.testing.assert_array_equal(valid, [True, True, True, False])
        self.assertTrue(np.isnan(uv[-1]).all())
        self.assertEqual(depth[-1], -1)

    def test_thermal_uses_native_pinhole_distortion(self):
        points = np.array([[.7, .4, 2.]])
        K = np.array([[500., 0, 320], [0, 490, 240], [0, 0, 1]])
        D = np.array([.01, .08, 0, 0, 0])
        uv, _, _ = project_camera(points, np.eye(4), K, D, "plumb_bob")
        reference, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K, D)
        np.testing.assert_allclose(uv, reference[:, 0])


REAL = Path("/hdd/DM_calib/nt_regress/full")


@unittest.skipUnless((REAL / "out/rig.yaml").exists(), "real calibration fixture unavailable")
class RealReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.producer = ReplayProducer(REAL / "work", cls.tmp.name)
        cls.manifest = cls.producer.prepare()
        cls.reader = StreamReader(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_actual_rig_frame_front_left_and_top(self):
        rig = yaml.safe_load((REAL / "out/rig.yaml").read_text())
        positions = {name: vehicle_from_camera(camera["T_cam_lidar_final"], self.manifest["R_lidar_V"])[:3, 3]
                     for name, camera in self.manifest["cameras"].items()}
        self.assertEqual(set(positions), set(CAMERA_NAMES))
        for i in range(1, 10):
            self.assertGreater(positions["camera_front" + str(i)][0], .5)
        for name in ("camera_side_left", "camera_rear_left", "thermal_left"):
            self.assertGreater(positions[name][1], 0)
        self.assertGreater(positions["camera_top"][2], positions["camera_front5"][2])
        self.assertLess(positions["camera_top"][2], 0)  # top camera is below Ouster
        for name, position in positions.items():
            np.testing.assert_allclose(position, rig["cameras"][name]["position_V_m"], atol=1e-4)

    def test_deterministic_snapshots_exact_final_and_honest_gates(self):
        a, b = self.producer.snapshot_at(.63), self.producer.snapshot_at(.63)
        self.assertEqual(a, b)
        final = self.producer.snapshot_at(1)
        metrics = json.loads((REAL / "out/metrics.json").read_text())
        for name, camera in final["cameras"].items():
            np.testing.assert_array_equal(camera["T_cam_lidar"], self.manifest["cameras"][name]["T_cam_lidar_final"])
            self.assertEqual(camera["sigma_rot_deg"], metrics[name]["rot_deg"])
            self.assertEqual(camera["sigma_pos_mm"], metrics[name]["pos_mm"])
            self.assertEqual(camera["reprojection_px"], metrics[name]["track_reproj_px"])
            self.assertEqual(camera["validation_vote"], metrics[name]["vote"]["pass"])
            self.assertTrue(camera["gate_pass"], name)
            self.assertEqual(camera["state"], "validated")
        self.assertTrue(final["gate"]["pass"])
        self.assertEqual(len(final["gate"]["camera_vote_failures"]), 14)
        self.assertTrue(a["provenance"]["synthetic_intermediate"])

    def test_real_progress_timing_asset_bounds_and_tracks(self):
        snapshot = self.producer.snapshot_at(.13)
        self.assertEqual(snapshot["stage"], "lidar_odometry")
        self.assertAlmostEqual(self.manifest["source_duration_s"], 28332.818, places=2)
        final = self.producer.snapshot_at(1)
        cloud = self.reader.load_asset(final["assets"]["map"])
        self.assertLessEqual(len(cloud["points"]), 12 * 4800)
        self.assertGreater(np.ptp(cloud["trajectory"], axis=0).max(), 10)
        self.assertEqual(final["map_window"], "W14")
        self.assertEqual(len(self.manifest["windows"]), 25)
        self.assertEqual(len(final["assets"]["matching"]), 16)
        for name, asset in final["assets"]["matching"].items():
            image = self.reader.load_asset(asset["image"])
            points = self.reader.load_asset(asset["points"])
            self.assertEqual(image.shape[:2], (asset["height"], asset["width"]))
            self.assertLessEqual(len(points["points_lidar"]), 7000)
            if not name.startswith("thermal"):
                self.assertGreater(len(points["tracks_uv"]), 0, name)
            self.assertEqual(points["tracks_uv"].shape, points["tracks_prev_uv"].shape)
        # All-window traversal and eviction are exercised by bounded fixtures;
        # real-data validation need not decode hundreds of redundant previews.
        self.assertLessEqual(len(list((Path(self.tmp.name) / "assets").glob("replay_*"))), MAX_ASSET_SETS)

    def test_source_scan_is_memory_mapped_and_sampled(self):
        path = next((REAL / "work/extract/S01/lidar").glob("*.npy"))
        original_load = np.load
        with mock.patch("calib_viz.replay.np.load", wraps=original_load) as load:
            points, offsets = _sample_scan(path, 101)
            self.assertLessEqual(len(points), 101)
            self.assertEqual(len(points), len(offsets))
            self.assertEqual(load.call_args.kwargs["mmap_mode"], "r")

    def test_run_stops_without_writing_when_cancelled(self):
        stop = threading.Event()
        stop.set()
        self.producer.run(stop)
        self.assertFalse((Path(self.tmp.name) / "events.jsonl").exists())

    def test_run_publishes_final_snapshot_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as directory:
            producer = ReplayProducer(REAL / "work", directory, speed=1e9)
            producer.prepare()
            source_stat = (REAL / "work/events.jsonl").stat()
            # Test the real publisher/terminal state cheaply; all-window replay
            # traversal is covered with synthetic source fixtures in test/.
            producer.replay_windows = [producer.replay_windows[-1]]
            producer.run()
            events = StreamReader(directory).poll()
            self.assertEqual(events[-1]["progress"], 1.)
            self.assertEqual(events[-1]["stage"], "validation")
            self.assertGreater(len(events), 0)
            self.assertEqual((REAL / "work/events.jsonl").stat().st_mtime_ns, source_stat.st_mtime_ns)

    def test_final_validation_not_exposed_at_stage_start(self):
        p = self.producer
        at_start = (p.stage_times[4][1] - p.start_time) / p.duration_s
        result = p.snapshot_at(at_start)
        self.assertEqual(result["stage"], "validation")
        self.assertIsNone(result["gate"]["pass"])
        self.assertTrue(all(c["validation_vote"] is None for c in result["cameras"].values()))


if __name__ == "__main__":
    unittest.main()
