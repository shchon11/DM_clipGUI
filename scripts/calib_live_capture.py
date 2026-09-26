#!/usr/bin/env python3
"""Capture an actual live solver stream without starting sensors or synthetic replay."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt5")

import numpy as np
from PyQt5 import QtCore, QtWidgets

from calib_viz.viewer import CalibrationVizWindow


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stream", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prefix", default="calib_live")
    parser.add_argument("--timeout", type=float, default=7200.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    app = QtWidgets.QApplication(sys.argv[:1])
    window = CalibrationVizWindow(stream_dir=args.stream, max_fps=12, max_points=30000)
    window.resize(1800, 1160)
    window.viewer.select_camera("camera_front5")
    window.show()
    start = time.monotonic()
    records, captured, pending = [], set(), set()
    previous = args.output / f"{args.prefix}_capture.json"
    if previous.exists():
        records = json.loads(previous.read_text())
        manifest = json.loads((args.stream / "manifest.json").read_text())
        records = [r for r in records if r.get("run_id") == manifest.get("run_id")]
        captured = {r["file"][len(args.prefix) + 1:-4] for r in records}
    first_pose = {c: np.asarray(s["T_cam_lidar"]) for c, s in (
        records[0]["event"].get("cameras", {}) if records else {}).items() if "T_cam_lidar" in s}
    last_pose = {}
    changes = dict(records[-1].get("camera_pose_change_max_abs", {})) if records else {}
    selected_thermal = False

    def save(label):
        pending.discard(label)
        viewer = window.viewer
        event = viewer.event
        if not event or event.get("provenance", {}).get("synthetic"):
            return
        if not viewer.rig.isValid() or not viewer.map.isValid():
            raise RuntimeError("Capture requires an OpenGL context (use xvfb-run with GLX)")
        path = args.output / f"{args.prefix}_{label}.png"
        if not window.grab().save(str(path)):
            raise RuntimeError(f"Cannot save {path}")
        record = {"file": path.name, "elapsed_s": time.monotonic() - start, "captured_at": time.time(),
                  "run_id": event.get("run_id"), "seq": event.get("seq"), "stage": event.get("stage"),
                  "pass": event.get("solver_pass", event.get("pass")), "iteration": event.get("iteration"),
                  "map_frame": event.get("map_frame"), "window_id": event.get("window_id"),
                  "point_count": viewer.map.point_count, "projected_points": len(viewer.image_panel.uv),
                  "selected_camera": viewer.selected, "fps": viewer.actual_fps,
                  "camera_pose_change_max_abs": dict(changes), "event": event}
        records.append(record)
        (args.output / f"{args.prefix}_capture.json").write_text(json.dumps(records, indent=2) + "\n")
        captured.add(label)
        print(json.dumps({k: v for k, v in record.items() if k != "event"}), flush=True)
        if label == "final":
            QtCore.QTimer.singleShot(500, app.quit)

    def schedule(label):
        if label not in captured and label not in pending:
            pending.add(label)
            QtCore.QTimer.singleShot(300, lambda: save(label))

    def applied(_seq):
        nonlocal selected_thermal
        viewer = window.viewer
        e = viewer.event
        for cam, state in e.get("cameras", {}).items():
            if "T_cam_lidar" not in state:
                continue
            T = np.asarray(state["T_cam_lidar"])
            first_pose.setdefault(cam, T.copy())
            if cam in last_pose:
                changes[cam] = max(changes.get(cam, 0.), float(np.max(np.abs(T - first_pose[cam]))))
            last_pose[cam] = T.copy()
        if e.get("stage") == "lidar_odometry" and viewer.map.point_count >= 1000:
            schedule("mid_lo")
        rgb = [v for k, v in e.get("cameras", {}).items() if k.startswith("camera_")]
        if e.get("stage") == "rgb_ba" and e.get("accepted") and any(
                (s.get("iteration") or e.get("iteration") or 0) >= 2 for s in rgb) and any(
                value > 1e-8 for cam, value in changes.items() if cam.startswith("camera_")):
            schedule("mid_rgb")
        if e.get("stage") == "rgb_ba" and any(
                np.linalg.norm(T[:3, 3] - first_pose[cam][:3, 3]) > .1
                for cam, T in last_pose.items() if cam.startswith("camera_") and cam in first_pose):
            schedule("rgb_converging")
        if len(viewer.image_panel.uv) >= 100 and "mid_rgb" in captured:
            schedule("projection")
        if ("projection" in captured and not selected_thermal
                and "thermal_right" in viewer.manifest.get("cameras", {})):
            viewer.select_camera("thermal_right")
            selected_thermal = True
        thermal = e.get("cameras", {}).get("thermal_right", {})
        if (selected_thermal and thermal.get("metric_source", "").startswith("solver")
                and changes.get("thermal_right", 0.) > 1e-8 and viewer.image_panel.image is not None
                and len(viewer.image_panel.uv) >= 100):
            schedule("mid_thermal")
        if e.get("complete"):
            schedule("final")

    window.viewer.snapshot_applied.connect(applied)
    timer = QtCore.QTimer()
    # Projection is updated on a render tick after snapshot_applied. Recheck
    # ready assets even if the solver is busy and no newer snapshot has arrived.
    timer.timeout.connect(lambda: app.quit() if time.monotonic() - start > args.timeout else applied(None))
    timer.start(1000)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    app.exec_()
    window.viewer.shutdown()
    return 0 if "final" in captured else 1


if __name__ == "__main__":
    raise SystemExit(main())
