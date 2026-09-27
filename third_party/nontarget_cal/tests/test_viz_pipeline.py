"""Pipeline visualization must describe the actual initialization and validation."""
from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import yaml

from nontarget_cal import viz_pipeline
from nontarget_cal.config import load_config
from nontarget_cal.init_calib import load_init
from nontarget_cal.pipeline import Pipeline
from nontarget_cal.rgb.solve import nominal_cams


class Recorder:
    def __init__(self):
        self.snapshots = []

    def publish(self, snapshot, **kwargs):
        self.snapshots.append(copy.deepcopy(snapshot))


class VizPipelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.cfg = load_config(overrides={"cameras": {
            "rgb": ["camera_front1", "camera_front2", "camera_front3"],
            "thermal": ["thermal_left", "thermal_right"],
        }})
        self.recorder = Recorder()
        self.pl = SimpleNamespace(
            ws=SimpleNamespace(root=self.root), cfg=self.cfg, sensors=["rgb", "thermal"],
            bags={"b0": "/bags/later", "b1": "/bags/earlier"},
            a=SimpleNamespace(mode="zeroshot", init=None), viz=None, viz_run_id=None,
            ev=SimpleNamespace(log=Mock(), warn=Mock()),
        )
        self.windows = [
            {"name": "S01", "bag_id": "b0", "t0_s": 10.0, "t1_s": 20.0},
            {"name": "S02", "bag_id": "b0", "t0_s": 30.0, "t1_s": 40.0},
            {"name": "S03", "bag_id": "b1", "t0_s": 10.0, "t1_s": 20.0},
        ]
        self.pl.plan = lambda: {"windows": self.windows}

    def start(self):
        with patch("nontarget_cal.viz.configure", return_value=self.recorder) as configure:
            viz_pipeline.start(self.pl)
        self.assertEqual(self.pl.ev.log.call_count, 0)
        self.assertTrue(configure.call_args.kwargs["fresh"])
        manifest = configure.call_args.kwargs["context"]["manifest"]
        self.assertEqual(manifest["bags"], self.pl.bags)
        self.assertFalse(manifest["provenance"]["synthetic"])
        self.assertEqual(configure.call_args.kwargs["context"]["run_id"], self.pl.viz_run_id)
        return self.recorder.snapshots[-1]["cameras"]

    def warm_file(self, omit=()):
        cameras = {}
        for i, cam in enumerate(self.cfg["cameras"]["rgb"] + self.cfg["cameras"]["thermal"]):
            if cam in omit:
                continue
            T = np.eye(4)
            T[:3, 3] = [0.01 * i, -0.2, 0.3]
            value = {"T_cam_lidar": T.tolist()}
            if cam.startswith("thermal"):
                value.update(intr=[700., 710., 320., 240., .03, -.002], rs_s=-.02,
                             dt_s=[-.12, -.1], dt_s_mean=-.11)
            else:
                value["intr_kb"] = [800., 820., 960., 600., .01, -.02, .003, .004]
            cameras[cam] = value
        path = self.root / "warm.json"
        path.write_text(json.dumps({"cameras": cameras}))
        self.pl.a.mode, self.pl.a.init = "warm", str(path)
        return path

    def assert_state_matches(self, state, value, sensor):
        np.testing.assert_array_equal(state["T_cam_lidar"], value["T_cam_lidar"])
        intr = value["intr"]
        self.assertEqual(state["K"], [[intr[0], 0., intr[2]], [0., intr[1], intr[3]], [0., 0., 1.]])
        self.assertEqual(state["D"], intr[4:8] if sensor == "rgb" else intr[4:6] + [0., 0., 0.])
        self.assertEqual(state["model"], "equidistant" if sensor == "rgb" else "plumb_bob")
        self.assertIsNone(state["gate_pass"])
        self.assertIsNone(state["sigma_rot_deg"])
        self.assertIsNone(state["sigma_pos_mm"])
        if sensor == "thermal":
            self.assertEqual(state["time_offset_s"], float(np.mean(value["dt_s"])))
            self.assertEqual(state["row_readout_s"], value["rs_s"])

    def test_zero_shot_initial_matches_actual_rgb_and_thermal_initializers(self):
        states = self.start()
        design = yaml.safe_load(Path(self.cfg["paths"]["rig_design"]).read_text())
        rgb = nominal_cams(self.cfg["cameras"]["rgb"], design)
        for i, cam in enumerate(rgb.names):
            self.assert_state_matches(states[cam], {"T_cam_lidar": np.linalg.inv(rgb.T_L_C()[i]).tolist(),
                                                    "intr": rgb.intr_array()[i].tolist()}, "rgb")
            np.testing.assert_array_equal(np.asarray(states[cam]["T_cam_lidar"])[:3, 3], np.zeros(3))
            self.assertEqual(states[cam]["K"][0][0], states[cam]["K"][1][1])
        thermal, from_design = Pipeline._thermal_start(self.pl, self.cfg["cameras"]["thermal"], None)
        self.assertTrue(from_design)
        for cam, value in thermal.items():
            self.assert_state_matches(states[cam], value, "thermal")
        self.assertTrue(all(state["metric_source"] == "solver-initial" for state in states.values()))

    def test_warm_initial_matches_loaded_extrinsics_intrinsics_and_timing(self):
        path = self.warm_file()
        states = self.start()
        actual = load_init(path)
        for sensor in self.pl.sensors:
            for cam, value in actual[sensor].items():
                self.assert_state_matches(states[cam], value, sensor)
                self.assertEqual(states[cam]["metric_source"], "warm-start")

    def test_partial_thermal_warm_start_matches_all_camera_design_fallback(self):
        self.warm_file(omit=("thermal_right",))
        states = self.start()
        actual, from_design = Pipeline._thermal_start(self.pl, self.cfg["cameras"]["thermal"], None)
        self.assertTrue(from_design)
        for cam, value in actual.items():
            self.assert_state_matches(states[cam], value, "thermal")
            self.assertEqual(states[cam]["metric_source"], "solver-initial")

    def test_missing_warm_rgb_has_no_invented_solver_initial_pose(self):
        self.warm_file(omit=("camera_front2",))
        states = self.start()
        self.assertNotIn("camera_front2", states)
        self.assertEqual(states["camera_front1"]["metric_source"], "warm-start")

    def test_disabled_start_still_delimits_each_attempt(self):
        self.cfg["viz"]["enabled"] = False
        with patch("nontarget_cal.viz.configure") as configure:
            viz_pipeline.start(self.pl)
            first = self.pl.viz_run_id
            viz_pipeline.start(self.pl)
        self.assertTrue(first)
        self.assertNotEqual(first, self.pl.viz_run_id)
        self.assertIsNone(self.pl.viz)
        configure.assert_not_called()

    def test_visualization_initialization_failure_does_not_raise(self):
        with patch("nontarget_cal.viz.configure", side_effect=OSError("read-only output")):
            viz_pipeline.start(self.pl)
        self.assertTrue(self.pl.viz_run_id)
        self.pl.ev.log.assert_called_once()

    def test_multi_bag_task_context_preserves_solver_window_order(self):
        self.pl.viz_run_id = "attempt"
        context = viz_pipeline.task_context(self.pl, "thermal_lens", "lens", {
            "name": "lens", "segs": ["S03", "S01"],
        })
        self.assertEqual(context["run_id"], "attempt")
        self.assertEqual(context["stage"], "thermal")
        self.assertEqual(context["pass"], "lens")
        self.assertEqual(context["window_id"], "S03")
        self.assertEqual(context["bag_id"], "b1")
        self.assertEqual(context["window"], 3)
        self.assertEqual(context["total_windows"], 3)
        self.assertEqual([entry["name"] for entry in context["windows"]], ["S03", "S01"])
        self.assertTrue(context["enabled"])

    def test_context_single_window_and_validation_suppression(self):
        for win in ("S02", self.windows[1]):
            context = viz_pipeline.task_context(self.pl, "lo", "S02", {"win": win})
            self.assertEqual(context["stage"], "lidar_odometry")
            self.assertEqual(context["window_id"], "S02")
            self.assertEqual(context["bag_id"], "b0")
        for stage in ("validation_rgb", "validation_thermal", "validation_heldout"):
            context = viz_pipeline.task_context(self.pl, stage, "half1", {"segs": ["S01"]})
            self.assertFalse(context["enabled"])
            self.assertEqual(context["stage"], "validation")
        self.pl.plan = Mock(side_effect=OSError("missing plan"))
        context = viz_pipeline.task_context(self.pl, "validation_heldout", "held", {})
        self.assertFalse(context["enabled"])
        from nontarget_cal.viz import configure
        with patch("nontarget_cal.viz.LiveViz") as writer:
            disabled = configure(self.root, self.cfg, context=context)
        self.assertFalse(disabled.enabled)
        writer.assert_not_called()

    def final_and_validation(self):
        initial = load_init(self.warm_file())
        result = {"layout": {"rules": [{"name": "global_alignment", "pass": False}]}}
        for sensor in self.pl.sensors:
            final = {"cameras": {cam: {**value, "reproj_median_px": 1.2}
                                  for cam, value in initial[sensor].items()}}
            path = self.root / sensor / "final" / "result.json"
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps(final))
            result[sensor] = {"track_reproj": dict.fromkeys(initial[sensor], 1.2),
                              "halves": {"sigma": {}}, "edges": {}}
        result["rgb"]["halves"]["sigma"] = {
            "camera_front1": {"rot_deg": .01, "along_axis_mm": 1., "pos_mm": 2.},
            "camera_front2": {"rot_deg": .01, "along_axis_mm": 10000., "pos_mm": 10001.},
        }
        result["rgb"]["edges"]["camera_front1"] = {"parked": {"vote": {"pass": False}}}
        result["thermal"]["halves"]["sigma"] = {
            "thermal_left": {"rot_deg": .1, "along_axis_mm": 1., "pos_mm": 2.},
        }
        result["thermal"]["edges"] = {"thermal_left": {"vote": {"pass": True}},
                                        "thermal_right": {"vote": {"pass": False}}}
        result["gate"] = Pipeline._gate(self.pl, result)
        self.pl.viz = self.recorder
        return result

    def test_cached_result_restores_exact_production_geometry_and_timing_without_gates(self):
        self.final_and_validation()
        for sensor in self.pl.sensors:
            with self.subTest(sensor=sensor):
                path = self.root / sensor / "final" / "result.json"
                result = json.loads(path.read_text())
                expected = {}
                for cam, value in result["cameras"].items():
                    value["T_cam_lidar"][0][3] += .456
                    value["intr"][0] += 35.
                    value["intr"][1] += 12.
                    value.update(gate_pass=False, sigma_pos_mm=999., iteration=45,
                                 total_iterations=50, accepted=True, cost=12345.)
                    if sensor == "thermal":
                        value.update(dt_s=[.012, .020], dt_s_mean=.016, rs_s=-.035)
                    expected[cam] = copy.deepcopy(value)
                    if sensor == "rgb":
                        value["intr_kb"] = value.pop("intr")
                path.write_text(json.dumps(result))
                # Cache restoration reads result files only; it must not solve again.
                with patch("nontarget_cal.rgb.solve.run_ba", side_effect=AssertionError("must not solve")), \
                        patch("nontarget_cal.thermal.solve.run_tba", side_effect=AssertionError("must not solve")):
                    viz_pipeline.cached_result(self.pl, sensor)
                snapshot = self.recorder.snapshots[-1]
                self.assertEqual(snapshot["stage"], "rgb_ba" if sensor == "rgb" else "thermal")
                self.assertEqual(snapshot["solve_name"], "final")
                self.assertEqual(snapshot["solver_pass"], "cached-result")
                self.assertIsNone(snapshot["iteration"])
                self.assertIsNone(snapshot["total_iterations"])
                self.assertIsNone(snapshot["accepted"])
                self.assertIn(sensor, snapshot["status_text"])
                self.assertNotIn("gate", snapshot)
                for cam, state in snapshot["cameras"].items():
                    self.assert_state_matches(state, expected[cam], sensor)
                    self.assertEqual(state["state"], "converged")
                    self.assertEqual(state["metric_source"], "solver.cached-result")
                    self.assertEqual(state["purpose"], "calibration")
                    self.assertIsNone(state["iteration"])
                    self.assertIsNone(state["total_iterations"])
                    self.assertIsNone(state["accepted"])
                    self.assertIsNone(state["cost"])
        self.assertEqual(self.pl.ev.log.call_count, 0)

    def test_cached_result_missing_or_malformed_file_is_fail_open(self):
        self.pl.viz = self.recorder
        viz_pipeline.cached_result(self.pl, "rgb")
        path = self.root / "thermal" / "final" / "result.json"
        path.parent.mkdir(parents=True)
        path.write_text("not json")
        viz_pipeline.cached_result(self.pl, "thermal")
        self.assertEqual(self.recorder.snapshots, [])
        self.assertEqual(self.pl.ev.log.call_count, 2)
        self.assertTrue(all("cached result skipped" in call.args[0] for call in self.pl.ev.log.call_args_list))

    def test_cached_result_with_no_visualization_does_not_read_cache(self):
        with patch("nontarget_cal.viz_pipeline.read_json", side_effect=AssertionError("must not read")) as read:
            viz_pipeline.cached_result(self.pl, "rgb")
        read.assert_not_called()
        self.assertEqual(self.recorder.snapshots, [])
        self.pl.ev.log.assert_not_called()

    def test_validation_preserves_individual_pass_fail_unknown_and_vote_roles(self):
        result = self.final_and_validation()
        viz_pipeline.validation(self.pl, result)
        self.assertEqual(self.pl.ev.log.call_count, 0)
        snapshot = self.recorder.snapshots[-1]
        states = snapshot["cameras"]
        self.assertTrue(states["camera_front1"]["gate_pass"])
        self.assertFalse(states["camera_front1"]["validation_vote"])
        self.assertTrue(states["camera_front1"]["informational_checks"])
        self.assertFalse(states["camera_front2"]["gate_pass"])
        self.assertIsNone(states["camera_front3"]["gate_pass"])
        self.assertTrue(states["thermal_left"]["gate_pass"])
        self.assertFalse(states["thermal_right"]["gate_pass"])
        self.assertEqual(states["camera_front2"]["sigma_pos_mm"], 10001.)
        self.assertEqual(snapshot["gate"]["failures"], result["gate"]["failures"])
        self.assertEqual(snapshot["gate"]["pass"], result["gate"]["pass"])
        self.assertIsNone(snapshot["iteration"])
        self.assertEqual(snapshot["solver_pass"], "final")
        self.assertTrue(all(state["metric_source"] == "solver.validation" for state in states.values()))
        self.assertTrue(all(not any(reason.startswith("layout rule") for reason in state["gate_reasons"])
                            for state in states.values()))

    def test_global_layout_failure_does_not_fail_every_camera(self):
        result = self.final_and_validation()
        for sensor in self.pl.sensors:
            result[sensor]["edges"] = {}
            result[sensor]["halves"]["sigma"] = {
                cam: {"rot_deg": .01, "along_axis_mm": 1., "pos_mm": 2.}
                for cam in self.cfg["cameras"][sensor]
            }
        result["gate"] = Pipeline._gate(self.pl, result)
        self.assertFalse(result["gate"]["pass"])
        viz_pipeline.validation(self.pl, result)
        snapshot = self.recorder.snapshots[-1]
        self.assertFalse(snapshot["gate"]["pass"])
        self.assertTrue(all(state["gate_pass"] is True for state in snapshot["cameras"].values()))
        self.assertTrue(all(state["gate_reasons"] == [] for state in snapshot["cameras"].values()))


if __name__ == "__main__":
    unittest.main()
