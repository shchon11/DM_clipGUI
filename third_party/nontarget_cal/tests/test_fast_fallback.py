"""Rig BA with the numba kernel selected: the fused kernels (rgb/fastba.py) run only for the validated
13-parameter model without observation weights; the time model (NPT layout, Obs.tm; bit-identical to kernel
"torch") and Obs.wt (normal equations) fall back to the torch path, and the viz observer sees the same calls."""
from __future__ import annotations

import unittest

import numpy as np
import torch

from nontarget_cal.rgb import rigba

try:
    from nontarget_cal.rgb import fastba  # noqa: F401
    HAVE_NUMBA = True
except Exception:  # noqa
    HAVE_NUMBA = False


def problem(time_model=False, weights=False):
    rng = np.random.default_rng(7)
    X = torch.tensor(np.column_stack([rng.uniform(-1, 1, 60), rng.uniform(-1, 1, 60),
                                     rng.uniform(4, 8, 60)]), dtype=torch.float64)
    lm = np.tile(np.arange(len(X)), 6)
    frame = np.repeat(np.arange(6), len(X))
    n = len(lm)
    intr = np.array([[300.0, 310.0, 320.0, 240.0, 0.01, -0.001, 0.0001, 0.0]])
    cams = rigba.Cameras(["camera_front5"], np.eye(4)[None], intr)
    pos = np.array([[0.3 * i, 0.03 * i * i, 0.0] for i in range(6)])
    tm = None
    if time_model:
        tm = {"W": torch.tensor(rng.normal(0, 0.05, (6, 3))), "V": torch.tensor(rng.normal(0, 3.0, (6, 3))),
              "T0": torch.zeros(6, dtype=torch.float64), "H": 480.0}
        cams.enable_time()
    obs = rigba.Obs(np.zeros(n), lm, np.zeros((n, 2)), frame, np.tile(np.eye(3), (6, 1, 1)), pos, tm)
    obs.uv = rigba.project_all(cams, obs, X)[0] + torch.tensor(rng.normal(0, 0.3, (n, 2)))
    if weights:
        obs.wt = torch.tensor(rng.uniform(0.5, 2.0, n))
    NP = cams.np_
    free = torch.zeros((1, NP), dtype=torch.bool)
    free[:, :7] = True
    prior = torch.full((1, NP), np.inf, dtype=torch.float64)
    prior[:, 3:6], prior[:, 6] = 1.0, 0.05
    d = torch.zeros((1, NP), dtype=torch.float64)
    d[:, :7] = torch.tensor([0.002, -0.004, 0.001, 0.02, -0.01, 0.01, 0.01])
    if time_model:
        free[:, 13] = True
        prior[:, 13] = 0.05
        d[:, 13] = 0.004
    cams.apply(d)
    X = X + torch.tensor(rng.normal(0.0, 0.02, size=X.shape), dtype=torch.float64)
    return cams, obs, X, free, prior


def run(inp, kernel):
    cams, obs, X, free, prior = inp
    calls = []
    old = rigba.KERNEL[0]
    rigba.KERNEL[0] = kernel
    try:
        out = rigba.solve(cams.copy(), obs, X.clone(), free, prior, iters=6, verbose=False, want_cov=True,
                          on_iteration=lambda c, o, e, cost, it, acc: calls.append((it, acc, cost, e.clone())),
                          step_tol={"rot": 2e-6, "pos": 1e-5, "f": 1e-6, "pp": 1e-3, "k": 1e-6, "dt": 1e-6})
    finally:
        rigba.KERNEL[0] = old
    return out, calls


@unittest.skipUnless(HAVE_NUMBA, "numba not installed")
class FastFallbackTests(unittest.TestCase):
    def check_identical(self, **kw):
        inp = problem(**kw)
        (ct, Xt, it_), calls_t = run(inp, "torch")
        (cn, Xn, in_), calls_n = run(inp, "numba")
        np.testing.assert_array_equal(ct.T_L_C(), cn.T_L_C())
        np.testing.assert_array_equal(ct.intr_array(), cn.intr_array())
        self.assertTrue(torch.equal(ct.dt, cn.dt))
        self.assertTrue(torch.equal(Xt, Xn))
        self.assertEqual(it_["hist"], in_["hist"])
        self.assertEqual([c[:3] for c in calls_t], [c[:3] for c in calls_n])

    def test_time_model_falls_back_bit_identical(self):
        self.check_identical(time_model=True)

    def test_observation_weights_fall_back(self):
        # normal equations on the torch path; the plain residuals (project_all) may still come from the
        # kernel, so the costs agree to rounding only
        inp = problem(weights=True)
        (ct, Xt, it_), calls_t = run(inp, "torch")
        (cn, Xn, in_), calls_n = run(inp, "numba")
        np.testing.assert_allclose(ct.T_L_C(), cn.T_L_C(), atol=1e-12)
        np.testing.assert_allclose(ct.intr_array(), cn.intr_array(), rtol=1e-12)
        np.testing.assert_allclose(it_["hist"], in_["hist"], rtol=1e-12)
        self.assertEqual([c[:2] for c in calls_t], [c[:2] for c in calls_n])

    def test_plain_model_uses_kernel_same_answer_and_observer(self):
        inp = problem()
        (ct, Xt, it_), calls_t = run(inp, "torch")
        (cn, Xn, in_), calls_n = run(inp, "numba")
        np.testing.assert_allclose(ct.T_L_C(), cn.T_L_C(), atol=1e-10)
        np.testing.assert_allclose(ct.intr_array(), cn.intr_array(), rtol=1e-10)
        self.assertEqual([c[:2] for c in calls_t], [c[:2] for c in calls_n])
        self.assertGreater(len(calls_n), 1)
        for a, b in zip(calls_t, calls_n):
            np.testing.assert_allclose(a[2], b[2], rtol=1e-9)
            np.testing.assert_allclose(a[3].numpy(), b[3].numpy(), atol=1e-9)

    def test_fast_predicate(self):
        cams, obs, X, _, _ = problem()
        rigba.KERNEL[0] = "numba"
        try:
            self.assertTrue(rigba._fast(X, obs, cams))
            obs.wt = torch.ones(len(obs), dtype=torch.float64)
            self.assertFalse(rigba._fast(X, obs, cams))
            self.assertTrue(rigba._fast(X, obs, cams, weights=False))
            cams_t, obs_t, *_ = problem(time_model=True)
            self.assertFalse(rigba._fast(X, obs_t, cams_t, weights=False))
        finally:
            rigba.KERNEL[0] = "torch"


if __name__ == "__main__":
    unittest.main()
