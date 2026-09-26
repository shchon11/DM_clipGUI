"""Observation-only LM hooks: accepted states, current intrinsics, and bit identity."""
from __future__ import annotations

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import torch

from nontarget_cal.rgb import rigba
from nontarget_cal.rgb.solve import _viz_observer, _viz_result
from nontarget_cal.thermal import tba


class RecordingViz:
    enabled = True

    def __init__(self, due=True, preview_results=None):
        self.snapshots = []
        self.publish_options = []
        self.previews = []
        self.is_due = due
        self.preview_results = iter(preview_results) if preview_results is not None else None

    def due(self, key):
        return self.is_due

    def publish(self, snapshot, **kwargs):
        self.snapshots.append(snapshot)
        self.publish_options.append(kwargs)

    def preview(self, *args):
        self.previews.append(args)
        return next(self.preview_results) if self.preview_results is not None else True


class MovingTrajectory:
    def pose(self, seg, t):
        w = torch.stack([0 * t, 0 * t, 0.02 * t * t], dim=1)
        return tba.so3_exp(w), torch.stack([0.4 * t, 0.1 * t * t, 0 * t], dim=1)


def problem(thermal=False):
    rng = np.random.default_rng(91)
    X = torch.tensor(np.column_stack([rng.uniform(-1, 1, 40), rng.uniform(-1, 1, 40),
                                     rng.uniform(4, 8, 40)]), dtype=torch.float64)
    lm = np.tile(np.arange(len(X)), 5)
    frame = np.repeat(np.arange(5), len(X))
    n = len(lm)
    intr = np.array([[300.0, 310.0, 320.0, 240.0, 0.01, -0.001, 0.0001, 0.0]])
    T = np.eye(4)[None]
    if thermal:
        cameras = tba.Cams(["thermal_left"], T, intr[:, :6], [0.0], [[0.003]], 1)
        obs = tba.Obs(np.zeros(n), np.zeros(n), lm, np.zeros((n, 2)), frame * 0.4)
        trajs = MovingTrajectory()
        obs.uv = tba.all_residuals(cameras, trajs, obs, X)[0]
        free = torch.zeros(cameras.P, dtype=torch.bool)
        free[:7] = True
        free[-1] = True
        prior = torch.full((cameras.P,), np.inf, dtype=torch.float64)
        prior[3:6], prior[6] = 1.0, 0.05
        perturbation = torch.zeros(cameras.P, dtype=torch.float64)
        perturbation[:7] = torch.tensor([0.002, -0.004, 0.001, 0.02, -0.01, 0.01, 0.01])
        perturbation[-1] = 0.004
        cameras.apply(perturbation)
    else:
        cameras = rigba.Cameras(["camera_front5"], T, intr)
        positions = np.array([[0.3 * i, 0.03 * i * i, 0.0] for i in range(5)])
        obs = rigba.Obs(np.zeros(n), lm, np.zeros((n, 2)), frame, np.tile(np.eye(3), (5, 1, 1)), positions)
        obs.uv = rigba.project_all(cameras, obs, X)[0]
        trajs = None
        free = torch.zeros((1, rigba.NP), dtype=torch.bool)
        free[:, :7] = True
        prior = torch.full((1, rigba.NP), np.inf, dtype=torch.float64)
        prior[:, 3:6], prior[:, 6] = 1.0, 0.05
        perturbation = torch.zeros((1, rigba.NP), dtype=torch.float64)
        perturbation[:, :7] = torch.tensor([0.002, -0.004, 0.001, 0.02, -0.01, 0.01, 0.01])
        cameras.apply(perturbation)
    X = X + torch.tensor(rng.normal(0.0, 0.02, size=X.shape), dtype=torch.float64)
    return cameras, trajs, obs, X, free, prior


class SolverVizTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.previous_threads = torch.get_num_threads()
        torch.set_num_threads(1)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.previous_threads)

    def run_solver(self, inputs, observer=None, thermal=False):
        cameras, trajs, obs, X, free, prior = inputs
        common = dict(iters=4, want_cov=True, verbose=False, on_iteration=observer)
        if thermal:
            return tba.solve(cameras.copy(), trajs, obs, X.clone(), free, prior, **common)
        return rigba.solve(cameras.copy(), obs, X.clone(), free, prior, **common)

    def assert_same(self, off, on):
        np.testing.assert_array_equal(off[0].T_L_C(), on[0].T_L_C())
        np.testing.assert_array_equal(off[0].intr_array(), on[0].intr_array())
        self.assertTrue(torch.equal(off[1], on[1]))
        self.assertEqual(off[2]["hist"], on[2]["hist"])
        for key in ("cov", "cov_free"):
            if key in off[2] and off[2][key] is not None:
                self.assertTrue(torch.equal(off[2][key], on[2][key]))
        if hasattr(off[0], "dt"):
            self.assertTrue(torch.equal(off[0].dt, on[0].dt))
            self.assertTrue(torch.equal(off[0].rs, on[0].rs))

    def test_live_observation_is_bit_identical_and_current(self):
        for thermal in (False, True):
            with self.subTest(thermal=thermal):
                inputs = problem(thermal)
                baseline = self.run_solver(inputs, thermal=thermal)
                viz = RecordingViz()
                with patch.dict(sys.modules, {"nontarget_cal.viz": SimpleNamespace(get_viz=lambda: viz)}):
                    observer = _viz_observer(None, ["S01"], "final", "B1", 4, 1.5, thermal=thermal)
                    live = self.run_solver(inputs, observer, thermal)
                self.assert_same(baseline, live)
                self.assertFalse(viz.snapshots[0]["accepted"])
                accepted = viz.snapshots[1:]
                self.assertGreater(len(accepted), 0)
                self.assertTrue(all(s["accepted"] for s in accepted))
                costs = [s["objective_cost"] for s in viz.snapshots]
                self.assertTrue(all(b < a for a, b in zip(costs, costs[1:])))
                name = live[0].names[0]
                state = viz.snapshots[-1]["cameras"][name]
                np.testing.assert_array_equal(state["T_cam_lidar"], np.linalg.inv(live[0].T_L_C()[0]))
                self.assertEqual(state["K"][0][0], live[0].intr_array()[0, 0])
                self.assertEqual(state["K"][1][1], live[0].intr_array()[0, 1])
                self.assertEqual(state["model"], "plumb_bob" if thermal else "equidistant")
                self.assertIsNone(state["sigma_pos_mm"])
                self.assertEqual(state["cost_source"], "huber_reprojection_only")
                for key in ("solve_name", "solver_pass", "iteration", "total_iterations", "purpose", "accepted", "windows"):
                    self.assertEqual(state[key], viz.snapshots[-1][key])
                self.assertGreater(len(viz.previews), 0)
                if thermal:
                    self.assertEqual(state["time_offset_by_window_s"]["S01"], float(live[0].dt[0, 0]))
                    self.assertEqual(state["row_readout_s"], float(live[0].rs[0]))
                    self.assertEqual(state["row_readout_s"], state["rs_s"])

    def test_observer_failure_cannot_change_solver(self):
        for thermal, module in ((False, rigba), (True, tba)):
            with self.subTest(thermal=thermal):
                inputs = problem(thermal)
                baseline = self.run_solver(inputs, thermal=thermal)
                calls = []

                def broken(*args):
                    calls.append(None)
                    raise OSError("visualization disk unavailable")

                with self.assertLogs(module.__name__, level="WARNING") as logs:
                    failed = self.run_solver(inputs, broken, thermal)
                self.assert_same(baseline, failed)
                self.assertEqual(len(calls), 1)
                self.assertEqual(len(logs.output), 1)

    def test_rejected_candidates_are_not_published(self):
        for thermal, module in ((False, rigba), (True, tba)):
            with self.subTest(thermal=thermal):
                inputs = problem(thermal)
                self.run_solver(inputs, thermal=thermal)
                states = []
                # Returning one unchanged objective forces the actual rejection branch;
                # cached residuals remain available from the preceding real baseline solve.
                cost_name = "cost" if thermal else "cost_terms"
                with patch.object(module, cost_name, return_value=1.0):
                    self.run_solver(inputs, lambda *args: states.append((args[-2], args[-1])), thermal)
                self.assertEqual(states, [(0, False)])

    def test_preview_cycles_actual_windows_only_after_a_request_is_scheduled(self):
        viz = RecordingViz(preview_results=[True, False, True, True])
        cameras, _, obs, _, _, _ = problem(thermal=True)
        cameras.dt = torch.tensor([[.003, .007]], dtype=torch.float64)
        cameras.rs[:] = -.031
        with patch.dict(sys.modules, {"nontarget_cal.viz": SimpleNamespace(get_viz=lambda: viz)}):
            observer = _viz_observer(None, ["S03", "S01"], "joint", "B2", 8, 1.5, thermal=True)
            for iteration in range(1, 5):
                observer(cameras, obs, torch.ones(len(obs), dtype=torch.float64), 10., iteration, True)
        self.assertEqual([request[1] for request in viz.previews], ["S03", "S01", "S01", "S03"])
        for snapshot in viz.snapshots:
            state = snapshot["cameras"]["thermal_left"]
            self.assertEqual(state["time_offset_by_window_s"], {"S03": .003, "S01": .007})
            self.assertEqual(state["row_readout_s"], -.031)
            self.assertEqual(state["row_readout_s"], state["rs_s"])
            self.assertEqual(state["solve_name"], "joint")
            self.assertEqual(state["solver_pass"], "B2")
            self.assertEqual(state["iteration"], snapshot["iteration"])
            self.assertEqual(state["total_iterations"], 8)
            self.assertEqual(state["purpose"], "calibration")

    def test_terminal_result_bypasses_sampling_without_reprojection_or_fake_iteration(self):
        for thermal in (False, True):
            with self.subTest(thermal=thermal):
                inputs = problem(thermal)
                cameras, X, info = self.run_solver(inputs, thermal=thermal)
                _, trajs, obs, _, _, _ = inputs
                residuals = (tba.all_residuals(cameras, trajs, obs, X)[0] if thermal else
                             rigba.project_all(cameras, obs, X)[0]).norm(dim=1)
                T, intr = cameras.T_L_C().copy(), cameras.intr_array().copy()
                viz = RecordingViz(due=False)
                with patch.dict(sys.modules, {"nontarget_cal.viz": SimpleNamespace(get_viz=lambda: viz)}), \
                        patch.object(rigba, "project_all", side_effect=AssertionError("must not reproject")), \
                        patch.object(tba, "all_residuals", side_effect=AssertionError("must not reproject")):
                    _viz_result(None, ["S01"], "final", cameras, obs, residuals, info["cost"], 1.5, thermal=thermal)
                    _viz_result(None, ["S01"], "half1", cameras, obs, residuals, info["cost"], 1.5, thermal=thermal)
                    _viz_result(None, ["S01"], "held", cameras, obs, residuals, info["cost"], 1.5,
                                thermal=thermal, held=True)
                self.assertEqual(len(viz.snapshots), 1)
                snapshot = viz.snapshots[0]
                state = snapshot["cameras"][cameras.names[0]]
                self.assertEqual(snapshot["pass"], "result")
                self.assertEqual(snapshot["solver_pass"], "result")
                self.assertIsNone(snapshot["iteration"])
                self.assertIsNone(snapshot["total_iterations"])
                self.assertIsNone(snapshot["accepted"])
                self.assertEqual(state["solver_pass"], "result")
                self.assertIsNone(state["iteration"])
                self.assertEqual(state["state"], "converged")
                self.assertEqual(state["metric_source"], "solver.result_residuals")
                np.testing.assert_array_equal(state["T_cam_lidar"], np.linalg.inv(T[0]))
                self.assertEqual(state["K"][0][0], intr[0, 0])
                self.assertTrue(viz.publish_options[0]["force"])
                np.testing.assert_array_equal(cameras.T_L_C(), T)
                np.testing.assert_array_equal(cameras.intr_array(), intr)
                if thermal:
                    self.assertEqual(state["time_offset_s"], float(cameras.dt.mean()))
                    self.assertEqual(state["row_readout_s"], float(cameras.rs[0]))

    def test_terminal_observer_failure_is_contained_and_logged_once(self):
        broken = Mock(side_effect=OSError("visualization failed"))
        with patch("nontarget_cal.rgb.solve._viz_observer", return_value=broken), \
                patch("nontarget_cal.rgb.solve._VIZ_OBSERVER_WARNED", False), \
                self.assertLogs("nontarget_cal.rgb.solve", level="WARNING") as logs:
            for _ in range(2):
                _viz_result(None, [], "final", None, None, None, 1.0, 1.5)
        self.assertEqual(len(logs.output), 1)

    def test_rate_limit_skips_all_state_preparation(self):
        viz = RecordingViz(due=False)
        with patch.dict(sys.modules, {"nontarget_cal.viz": SimpleNamespace(get_viz=lambda: viz)}):
            observer = _viz_observer(None, ["S01"], "final", "A", 8, 1.5)
            observer(None, None, None, 0.0, 1, True)
        self.assertEqual(viz.snapshots, [])
        self.assertEqual(viz.previews, [])

    def test_disabled_and_validation_solvers_have_no_observer(self):
        viz = RecordingViz()
        with patch.dict(sys.modules, {"nontarget_cal.viz": SimpleNamespace(get_viz=lambda: viz)}):
            for solve_name, held in (("half1", False), ("heldout_1on2", False), ("final", True)):
                self.assertIsNone(_viz_observer(None, ["S01"], solve_name, "A", 8, 1.5, held=held))
            viz.enabled = False
            self.assertIsNone(_viz_observer(None, ["S01"], "final", "A", 8, 1.5))


if __name__ == "__main__":
    unittest.main()
