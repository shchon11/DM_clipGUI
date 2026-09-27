#!/usr/bin/env python3
"""Standalone viewer/replay. No ROS initialization and no solver modifications."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading

# Limit only this viewer's libraries; never change the solver's environment.
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')

from PyQt5 import QtCore, QtWidgets

from calib_viz.replay import ReplayProducer
from calib_viz.stream import append_snapshot
from calib_viz.viewer import CalibrationVizWindow


class Bridge(QtCore.QObject):
    error = QtCore.pyqtSignal(str)
    prepared = QtCore.pyqtSignal()


def main(argv=None):
    parser = argparse.ArgumentParser(description='온라인 캘리브레이션 3D 뷰어')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--replay', type=Path, help='완료된 실행의 work 디렉터리')
    source.add_argument('--stream', type=Path, help='실시간 viz 디렉터리')
    parser.add_argument('--out', type=Path, help='완료된 결과 디렉터리 (기본: work 옆 out)')
    parser.add_argument('--speed', type=float, default=60, help='실제 이벤트 시간 압축 배율')
    parser.add_argument('--viz-dir', type=Path, help='리플레이 출력 (기본: 별도 /tmp 디렉터리)')
    parser.add_argument('--max-fps', type=int, default=40)
    parser.add_argument('--max-points', type=int, default=70000)
    parser.add_argument('--screenshots', type=Path, help='리플레이 오프스크린 PNG 저장 후 종료')
    parser.add_argument('--fractions', default='0.06,0.55,1.0', help='스크린샷 시점 0..1')
    parser.add_argument('--camera', default='camera_front5')
    parser.add_argument('--window', help='스크린샷용 특정 원본 구간 (기본: 재생 시점 구간)')
    args = parser.parse_args(argv)
    if args.screenshots and not args.replay:
        parser.error('--screenshots requires --replay')
    if not 1 <= args.max_points <= 200000:
        parser.error('--max-points must be 1..200000')
    fractions = [float(value) for value in args.fractions.split(',')]
    if not fractions or any(not 0 <= value <= 1 for value in fractions):
        parser.error('--fractions must be comma-separated numbers in 0..1')
    temporary = tempfile.TemporaryDirectory(prefix='calib-viz-') if args.replay and not args.viz_dir else None
    stream_dir = args.stream or args.viz_dir or Path(temporary.name)
    stop_event = threading.Event()
    shot_done = threading.Event()
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName('calib-viz')
    window = CalibrationVizWindow(stream_dir=stream_dir, max_fps=args.max_fps, max_points=args.max_points)
    window.viewer.select_camera(args.camera)
    window.show()
    bridge = Bridge()
    failure = []
    producer = ReplayProducer(args.replay, stream_dir, out_dir=args.out, speed=args.speed) if args.replay else None

    def on_error(message):
        failure.append(message)
        window.viewer.status_label.setText('리플레이 오류: ' + message)
        print('calib-viz:', message, file=sys.stderr)
        if args.screenshots:
            app.exit(1)

    bridge.error.connect(on_error)
    pending_shots = []
    shot_lock = threading.Lock()
    shot_stats = []

    def produce():
        try:
            producer.prepare()
            bridge.prepared.emit()
            if args.screenshots:
                args.screenshots.mkdir(parents=True, exist_ok=True)
                previous_fraction, seq = 0., 0
                for index, fraction in enumerate(fractions):
                    if stop_event.is_set():
                        break
                    # Exercise the real tail/animation path and populate genuine
                    # synthetic replay histories before each deterministic capture.
                    for step in range(6):
                        warmup = producer.snapshot_at(previous_fraction + (fraction-previous_fraction)*step/6, window=args.window)
                        warmup['seq'] = seq
                        seq += 1
                        append_snapshot(stream_dir, warmup)
                        if stop_event.wait(.075):
                            return
                    snapshot = producer.snapshot_at(fraction, window=args.window)
                    # A monotonically increasing transport sequence permits arbitrary seeks.
                    snapshot['seq'] = seq
                    seq += 1
                    with shot_lock:
                        pending_shots.append((index, fraction))
                    shot_done.clear()
                    append_snapshot(stream_dir, snapshot)
                    if not shot_done.wait(30):
                        raise RuntimeError('screenshot timed out; check the OpenGL context / Xvfb')
                    previous_fraction = fraction
                return
            producer.run(stop_event)
        except Exception as exc:
            bridge.error.emit(f'{type(exc).__name__}: {exc}')

    capturing = set()

    def on_snapshot(seq):
        if not args.screenshots or seq in capturing:
            return
        with shot_lock:
            if not pending_shots:
                return
            index, fraction = pending_shots.pop(0)
        capturing.add(seq)

        def capture():
            if not window.viewer.rig.isValid() or not window.viewer.map.isValid():
                on_error('OpenGL 컨텍스트 없음: xvfb-run + LIBGL_ALWAYS_SOFTWARE=1로 실행하세요')
                shot_done.set()
                return
            name = f'calib_viz_{index + 1:02d}_{int(fraction * 100):03d}.png'
            path = args.screenshots / name
            if not window.grab().save(str(path)):
                on_error('스크린샷 저장 실패: ' + str(path))
            else:
                print(str(path), flush=True)
            shot_stats.append({'progress': fraction, 'fps': window.viewer.actual_fps,
                               'points': window.viewer.map.point_count,
                               'projected_points': len(window.viewer.image_panel.uv),
                               'gl_valid': window.viewer.rig.isValid(),
                               'map_window': window.viewer.event.get('map_window'),
                               'available_cameras': window.viewer.camera_combo.count(),
                               'selected_camera': window.viewer.selected,
                               'camera_passes': sum(c.get('gate_pass') is True for c in window.viewer.event['cameras'].values())})
            shot_done.set()
            if len(shot_stats) == len(fractions):
                (args.screenshots / 'calib_viz_capture.json').write_text(
                    json.dumps(shot_stats, indent=2) + '\n')
                QtCore.QTimer.singleShot(100, app.quit)
        # Let the pose transition settle and collect rendered-frame FPS, not timer ticks.
        QtCore.QTimer.singleShot(3400, capture)

    window.viewer.snapshot_applied.connect(on_snapshot)
    worker = threading.Thread(target=produce, name='calib-viz-replay', daemon=True) if producer else None
    if worker:
        worker.start()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    exit_code = app.exec_()
    stop_event.set()
    shot_done.set()
    window.viewer.shutdown()
    if worker:
        worker.join(timeout=3)
    # A still-preparing daemon may own files; do not remove its directory beneath it.
    if temporary and (not worker or not worker.is_alive()):
        temporary.cleanup()
    return 1 if failure else exit_code


if __name__ == '__main__':
    raise SystemExit(main())
