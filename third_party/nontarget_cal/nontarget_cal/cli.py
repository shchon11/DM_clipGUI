"""Command line.

  nontarget_cal run   --bags BAG [BAG ...] --out DIR [--mode warm|zeroshot] [--init CALIB[,CALIB2]]
                      [--sensors rgb,thermal] [--workdir DIR] [--name-map FILE] [--windows SPEC]
                      [--config FILE] [--force]
  nontarget_cal check --bags BAG [BAG ...] [--sensors ...] [--workdir DIR] [--name-map FILE]
  nontarget_cal report DIR

stdout carries one JSON object per line (progress events for a GUI); human-readable logging goes
to stderr and <workdir>/log.txt. Exit status: 0 ok, 2 refused (reason in the last JSON line, in
English and Korean), 1 error.
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
import traceback
from pathlib import Path


def _common(p):
    p.add_argument("--bags", nargs="+", required=True, help="rosbag2 directories (or .db3 files)")
    p.add_argument("--sensors", default="rgb,thermal", help="rgb, thermal or rgb,thermal")
    p.add_argument("--workdir", default=None, help="work directory (large; default: <out>/../<out name>_work)")
    p.add_argument("--name-map", default=None, help="camera name mapping YAML (see README)")
    p.add_argument("--no-verify-names", action="store_true", help="skip the geometric check of the name map")
    p.add_argument("--config", default=None, help="YAML overriding parts of the default configuration")
    p.add_argument("--init", default=None, help="previous calibration(s) for --mode warm (comma separated)")
    p.add_argument("--windows", default=None, help="explicit windows NAME:T0:T1[,...] (s from the first /gps/fix)")
    p.add_argument("--force", action="store_true", help="continue past data-amount/sync/exposure refusals and "
                   "disagreeing bags (never past insufficient disk)")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="nontarget_cal", description="Targetless camera/thermal <-> LiDAR calibration")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="full pipeline")
    _common(r)
    r.add_argument("--out", required=True)
    r.add_argument("--viz-dir", default=None, help="live visualization stream (default: <workdir>/viz)")
    r.add_argument("--no-viz", action="store_true", help="disable best-effort live visualization")
    r.add_argument("--mode", choices=["warm", "zeroshot"], default=None,
                   help="warm: start from --init; zeroshot: board-free start (default: warm if --init else zeroshot)")
    c = sub.add_parser("check", help="preflight only (fast)")
    _common(c)
    c.add_argument("--out", default=None)
    c.add_argument("--mode", default="zeroshot")
    rep = sub.add_parser("report", help="print / regenerate the report of an output directory")
    rep.add_argument("dir")
    rep.add_argument("--config", default=None)
    a = ap.parse_args(argv)

    from .config import load_config
    from .errors import Refusal
    from .events import Events

    if a.cmd == "report":
        from .outputs.report import write_all
        d = Path(a.dir)
        s = json.loads((d / "summary.json").read_text())
        write_all(d, s, load_config(a.config))
        sys.stdout.write((d / "report.md").read_text())
        return 0

    cfg = load_config(a.config)
    if getattr(a, "viz_dir", None):
        cfg.setdefault("viz", {})["dir"] = str(Path(a.viz_dir).resolve())
    if getattr(a, "no_viz", False):
        cfg.setdefault("viz", {})["enabled"] = False
    if a.mode is None:
        a.mode = "warm" if a.init else "zeroshot"
    if a.mode == "warm" and not a.init:
        ap.error("--mode warm needs --init")
    if a.workdir is None:
        if a.out:
            o = Path(a.out).resolve()
            a.workdir = str(o.parent / (o.name + "_work"))
        else:
            a.workdir = tempfile.mkdtemp(prefix="nontarget_cal_check_")
    ev = Events(Path(a.workdir))
    from .pipeline import Pipeline
    pl = None
    try:
        pl = Pipeline(a, cfg)
        pl.ev = ev
        if a.cmd == "check":
            rep = pl.stage_check(force=a.force)
            ev.emit("check_done", ok=rep["ok"], total=rep["total"], estimate=rep["estimate"], disk=rep["disk"],
                    warnings=rep["warnings"], names={b: n["source"] for b, n in rep["names"].items()},
                    report=str(Path(a.workdir) / "preflight" / "report.json"))
            return 0
        pl.run()
        return 0
    except Refusal as ex:
        ev.log(f"REFUSED [{ex.code}] {ex.msg}\n  {ex.msg_ko}")
        ev.emit("refusal", **ex.as_dict())
        return 2
    except Exception as ex:  # noqa
        ev.log(traceback.format_exc())
        ev.emit("error", msg=repr(ex), msg_ko="처리 중 오류: 로그를 확인하세요", log=str(Path(a.workdir) / "log.txt"))
        return 1
    finally:
        if pl is not None:
            pl.close_viz()


if __name__ == "__main__":
    sys.exit(main())
