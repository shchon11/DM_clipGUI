"""Work directory layout and the per-stage cache.

<workdir>/
  run.json                      arguments of the latest run
  events.jsonl, log.txt         progress (see events.py)
  stages/<stage>.json           "done" markers: written LAST, atomically, with a key of the inputs
  preflight/report.json
  names/mapping.json            canonical camera name -> topic namespace, per bag
  windows/plan.json             the windows (bag, t0, t1, name)
  extract/<win>/               cam/<camera>/<ns>.jpg, lidar/<ns>.npy, cam_index.npz, source.json, DONE
  extract/<bag>_ins.npz         INS of the whole bag (for the tracker's static-track test)
  thermal16/<win>/<cam>/<header ns>.png, thermal16/<bag>_index_<cam>.npz
  lo/kiss_<win>.npz ref_<win>.npz map_<win>.npz check_<win>.json
  tracks/tracks_<win>_<cam>.npz
  thermal/tracks|edges|links|<solve>/
  rgb/<solve>/result.json state.npz
  validation/...

A stage is done when its marker exists and its key matches; sub-steps (one window, one camera)
write their outputs to a temporary name and rename, so a crash or an unplugged disk leaves either a
complete file or none, and a re-run redoes only what is missing.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path


class Workspace:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        for d in ("stages", "preflight", "names", "windows", "extract", "thermal16", "lo", "tracks",
                  "thermal", "rgb", "validation"):
            (self.root / d).mkdir(exist_ok=True)

    # ------------------------------------------------------------------ paths
    def seg_dir(self, win: str) -> Path:
        return self.root / "extract" / win

    def ins(self, bag_id: str) -> Path:
        return self.root / "extract" / f"{bag_id}_ins.npz"

    def lo(self, kind: str, win: str) -> Path:
        return self.root / "lo" / f"{kind}_{win}.npz"

    def lo_check(self, win: str) -> Path:
        return self.root / "lo" / f"check_{win}.json"

    def tracks(self, win: str, cam: str) -> Path:
        return self.root / "tracks" / f"tracks_{win}_{cam}.npz"

    def thermal16(self, win: str) -> Path:
        return self.root / "thermal16" / win

    def thermal_index(self, bag_id: str, cam: str) -> Path:
        return self.root / "thermal16" / f"{bag_id}_index_{cam}.npz"

    def thermal_tracks(self, win: str, cam: str) -> Path:
        return self.root / "thermal" / "tracks" / f"tracks_{win}_{cam}.npz"

    def thermal_edges_dir(self) -> Path:
        return self.root / "thermal" / "edges"

    def solve_dir(self, sensor: str, name: str) -> Path:
        d = self.root / sensor / name
        d.mkdir(parents=True, exist_ok=True)
        return d

    # ------------------------------------------------------------------ stage cache
    def _marker(self, stage: str) -> Path:
        return self.root / "stages" / f"{stage}.json"

    def done(self, stage: str, key=None) -> dict | None:
        m = self._marker(stage)
        if not m.exists():
            return None
        d = json.loads(m.read_text())
        if key is not None and d.get("key") != key:
            return None
        return d

    def mark(self, stage: str, key=None, **info):
        write_json(self._marker(stage), {"stage": stage, "key": key, "time": time.time(), **info})

    def invalidate(self, *stages):
        for s in stages:
            m = self._marker(s)
            if m.exists():
                m.unlink()


def write_json(path: Path, obj, indent=1):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(obj, indent=indent, default=_default, ensure_ascii=False))
    os.replace(tmp, path)


def _default(o):
    import numpy as np
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(type(o))


def atomic_savez(path: Path, **arrays):
    import numpy as np
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.stem + ".tmp.npz")
    np.savez(tmp, **arrays)
    os.replace(tmp, path)


def read_json(path: Path):
    return json.loads(Path(path).read_text())
