"""Progress events (one JSON line per event, for a GUI), human log, per-stage wall clock and disk.

Every event is written to stdout as ONE line prefixed with nothing but the JSON object, e.g.
  {"ev": "stage_start", "stage": "lo", "t": 1790000000.1, ...}
and appended to <workdir>/events.jsonl. Human-readable log lines go to stderr and <workdir>/log.txt,
so a GUI can read stdout line by line and json.loads() every line.

Event kinds: run_start, stage_start, stage_skip (cached), stage_progress, stage_end, warning,
refusal, error, run_end. Fields: stage, msg, msg_ko (Korean), done/total, wall_s, disk_gb, ...
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import time
from contextlib import contextmanager
from pathlib import Path


def dir_size_gb(p: Path) -> float:
    tot = 0
    for root, _, files in os.walk(p):
        for f in files:
            try:
                tot += os.lstat(os.path.join(root, f)).st_size
            except OSError:
                pass
    return tot / 1e9


def free_gb(p: Path) -> float:
    p = Path(p)
    while not p.exists():
        p = p.parent
    return shutil.disk_usage(p).free / 1e9


class Events:
    def __init__(self, workdir: Path | None = None, quiet: bool = False):
        self.workdir = Path(workdir) if workdir else None
        self.quiet = quiet
        self._fj = None
        self._fl = None
        if self.workdir:
            self.workdir.mkdir(parents=True, exist_ok=True)
            self._fj = open(self.workdir / "events.jsonl", "a", buffering=1)
            self._fl = open(self.workdir / "log.txt", "a", buffering=1)

    def emit(self, ev: str, **kw):
        rec = {"ev": ev, "t": round(time.time(), 3), **kw}
        line = json.dumps(rec, ensure_ascii=False, default=str)
        sys.stdout.write(line + "\n")
        sys.stdout.flush()
        if self._fj:
            self._fj.write(line + "\n")
        return rec

    def log(self, msg: str):
        s = f"[{time.strftime('%H:%M:%S')}] {msg}"
        if not self.quiet:
            sys.stderr.write(s + "\n")
            sys.stderr.flush()
        if self._fl:
            self._fl.write(s + "\n")

    def warn(self, code: str, msg: str, msg_ko: str = "", **kw):
        self.log(f"WARNING {code}: {msg}")
        return self.emit("warning", code=code, msg=msg, msg_ko=msg_ko, **kw)

    def progress(self, stage: str, done: int, total: int, msg: str = ""):
        self.emit("stage_progress", stage=stage, done=done, total=total, msg=msg)

    @contextmanager
    def stage(self, name: str, disk_path: Path | None = None):
        t0 = time.time()
        self.emit("stage_start", stage=name)
        self.log(f"=== stage {name}")
        info = {}
        try:
            yield info
        except Exception:
            self.emit("stage_end", stage=name, ok=False, wall_s=round(time.time() - t0, 1))
            raise
        rec = {"stage": name, "ok": True, "wall_s": round(time.time() - t0, 1)}
        if disk_path is not None and Path(disk_path).exists():
            rec["disk_gb"] = round(dir_size_gb(Path(disk_path)), 3)
        if self.workdir:
            rec["workdir_free_gb"] = round(free_gb(self.workdir), 1)
        rec.update(info)
        self.emit("stage_end", **rec)
        self.log(f"=== stage {name} done in {rec['wall_s']:.0f} s" +
                 (f", {rec['disk_gb']:.2f} GB" if "disk_gb" in rec else ""))

    def close(self):
        for f in (self._fj, self._fl):
            if f:
                f.close()
