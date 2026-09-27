"""Empirical per-observation weights for the RGB rig BA (rgb.solve obs_weight, off by default).

The KLT measurement error is not uniform (tests/time_offset_eval_20260927.txt section 4b): it grows with the
time from the track centre (drift; 0.68 -> 1.13 px) and is larger for weak corners (min-eigenvalue quintiles
1.00 ... 0.67 px). Instead of assuming a noise model, the weights are measured from the residuals of the
previous solve round: per camera, the observations are binned by quantiles of each feature (|t - track
centre|, and the corner strength obs_q of the anchored tracker when available), and every bin gets the weight
(median residual of the camera / median residual of the bin)^2 (inverse variance, relative), clipped to
[floor, 1/floor] and normalised to mean 1 per camera, so the balance between cameras is unchanged.
"""
from __future__ import annotations

import numpy as np
import torch

from .rigba import DT

DEFAULTS = dict(enabled=False, by=["age"], nbins=5, floor=0.25, rounds=[1, 2])


def features(obs, tns: np.ndarray, q: np.ndarray | None, by) -> dict:
    lm = obs.lm.cpu().numpy()
    M = int(lm.max()) + 1
    t = tns.astype(np.float64)
    tmid = np.bincount(lm, weights=t, minlength=M) / np.maximum(np.bincount(lm, minlength=M), 1)
    out = {}
    if "age" in by:
        out["age"] = np.abs(t - tmid[lm]) * 1e-9
    if "q" in by and q is not None and np.isfinite(q).any():
        out["q"] = np.log(np.where(np.isfinite(q) & (q > 0), q, np.nan))
    return out


def weights(obs, e: torch.Tensor, feats: dict, nbins: int = 5, floor: float = 0.25):
    """Per-observation weights (N,) and a table for the log: {cam: [(bin key, med px, weight)]}."""
    cam = obs.cam.cpu().numpy()
    e = e.cpu().numpy()
    w = np.ones(len(cam))
    for ci in np.unique(cam):
        k = np.flatnonzero(cam == ci)
        key = np.zeros(len(k), np.int64)
        for name, v in feats.items():
            vk = v[k]
            fin = np.isfinite(vk)
            b = np.full(len(k), nbins // 2)                  # missing values (e.g. no corner strength): middle bin
            if fin.sum() > 10 * nbins:
                edges = np.quantile(vk[fin], np.linspace(0, 1, nbins + 1)[1:-1])
                b[fin] = np.searchsorted(edges, vk[fin])
            key = key * nbins + b
        med_all = np.median(e[k])
        wk = np.ones(len(k))
        for u in np.unique(key):
            m = key == u
            if m.sum() > 50:
                wk[m] = (med_all / max(np.median(e[k[m]]), 1e-6)) ** 2
        wk = np.clip(wk, floor, 1 / floor)
        w[k] = wk / wk.mean()
    return torch.as_tensor(w, dtype=DT)
