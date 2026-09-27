"""Configuration: one YAML file for every path, threshold and topic name.

The packaged default is nontarget_cal/config/default.yaml. `--config FILE` is deep-merged on top
of it (only the keys you want to change). Strings starting with "@pkg/" are resolved relative to
the installed package, so the default works wherever the package is installed; nothing in the
code refers to /hdd or any other absolute location.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parent


def pkg_data(rel: str) -> Path:
    return PKG / "data" / rel


def _merge(a: dict, b: dict) -> dict:
    out = copy.deepcopy(a)
    for k, v in (b or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _resolve(x):
    if isinstance(x, str) and x.startswith("@pkg/"):
        return str(PKG / x[len("@pkg/"):])
    if isinstance(x, dict):
        return {k: _resolve(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_resolve(v) for v in x]
    return x


class Config(dict):
    """dict with attribute-free helpers: cfg['a']['b'] or cfg.get_path('a.b')."""

    def get_path(self, dotted: str, default=None):
        cur = self
        for k in dotted.split("."):
            if not isinstance(cur, dict) or k not in cur:
                return default
            cur = cur[k]
        return cur

    def digest(self) -> str:
        return hashlib.sha1(json.dumps(self, sort_keys=True, default=str).encode()).hexdigest()[:12]


def load_config(user: str | Path | None = None, overrides: dict | None = None) -> Config:
    base = yaml.safe_load((PKG / "config" / "default.yaml").read_text())
    for u in (str(user).split(",") if user else []):
        # comma-separated overrides, applied in order; "fast" / "@pkg/config/fast.yaml" = the packaged preset
        u = u.strip()
        p = PKG / "config" / f"{u}.yaml" if u in ("fast",) else Path(_resolve(u))
        base = _merge(base, yaml.safe_load(p.read_text()) or {})
    if overrides:
        base = _merge(base, overrides)
    cfg = Config(_resolve(base))
    lr = cfg["paths"].get("layout_rules")
    cfg["layout_rules"] = yaml.safe_load(Path(lr).read_text()) if lr and Path(lr).exists() else {}
    return cfg
