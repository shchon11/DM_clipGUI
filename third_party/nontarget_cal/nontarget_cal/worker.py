"""Worker process: runs ONE heavy task in its own process (own thread limits, own log file).

    python -m nontarget_cal.worker TASK.json

TASK.json: {"task": name, "workdir": ..., "config": snapshot path, "args": {...}, "result": path, "log": path}
The worker writes `result` (JSON) atomically at the end; the parent treats a missing result as a
failure and shows the tail of `log`.
"""
from __future__ import annotations

import json
import os
import sys
import time
import traceback
from pathlib import Path


def _log_to(path):
    f = open(path, "a", buffering=1)

    def log(*a, **k):
        f.write(" ".join(str(x) for x in a) + "\n")
    return log, f


class MemPeak:
    """Peak private (anonymous) and file-backed resident memory of this process, sampled every 0.5 s from
    /proc/self/status. ru_maxrss counts pages of memory-mapped files (the LiDAR maps are mapped and shared
    between processes through the page cache), which the OS can drop; RssAnon is what must fit in RAM."""

    def __init__(self):
        import threading
        self.anon = self.file = 0.0
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def sample(self):
        try:
            for line in open("/proc/self/status"):
                if line.startswith("RssAnon:"):
                    self.anon = max(self.anon, int(line.split()[1]) / 1e6)
                elif line.startswith("RssFile:"):
                    self.file = max(self.file, int(line.split()[1]) / 1e6)
        except OSError:
            pass

    def _run(self):
        while not self._stop.wait(0.5):
            self.sample()

    def stop(self):
        self._stop.set()
        self.sample()
        return {"max_anon_gb": round(self.anon, 3), "max_file_gb": round(self.file, 3)}


def main(task_file: str) -> int:
    spec = json.loads(Path(task_file).read_text())
    log, fh = _log_to(spec["log"])
    sys.stdout = fh
    sys.stderr = fh
    from .config import Config
    from .workspace import Workspace, write_json
    cfg = Config(json.loads(Path(spec["config"]).read_text()))
    ws = Workspace(Path(spec["workdir"]))
    from .viz import configure
    viz = configure(ws.root, cfg, context=spec.get("viz_context"), log=log)
    t0 = time.time()
    mem = MemPeak()
    try:
        from . import tasks
        fn = getattr(tasks, "t_" + spec["task"])
        if os.environ.get("NONTARGET_PROFILE"):
            # cProfile of the task -> <log>.prof (python -m pstats FILE; sort cumtime)
            import cProfile
            prof = cProfile.Profile()
            out = prof.runcall(fn, ws, cfg, log=log, **spec["args"])
            prof.dump_stats(spec["log"] + ".prof")
        else:
            out = fn(ws, cfg, log=log, **spec["args"])
        import resource
        ru = resource.getrusage(resource.RUSAGE_SELF)
        write_json(Path(spec["result"]), {"ok": True, "wall_s": time.time() - t0, "cpu_s": ru.ru_utime + ru.ru_stime,
                                          "max_rss_gb": ru.ru_maxrss / 1e6, **mem.stop(), "out": out})
        return 0
    except Exception as ex:  # noqa
        log(traceback.format_exc())
        write_json(Path(spec["result"]), {"ok": False, "wall_s": time.time() - t0, "error": repr(ex)})
        return 1
    finally:
        viz.close()


if __name__ == "__main__":
    for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(k, "3")
    sys.exit(main(sys.argv[1]))
