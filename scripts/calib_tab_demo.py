#!/usr/bin/env python3
"""Open a completed result in only the calibration tab; no sensors or recorder.

Uses a temporary job store. Input work/out directories are always read-only.
For screenshots use Xvfb/GLX and software rendering, not Qt's GL-less offscreen.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile


def main(argv=None):
    parser = argparse.ArgumentParser(description='온라인 보정 탭 · 저장된 실제 결과 보기')
    parser.add_argument('--workdir', type=Path, required=True)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args(argv)
    out = args.out or args.workdir.parent / 'out'
    summary = json.loads((out / 'summary.json').read_text())
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')
    with tempfile.TemporaryDirectory(prefix='calib-tab-') as directory:
        os.environ['XDG_DATA_HOME'] = directory
        from PyQt5 import QtCore, QtWidgets
        import online_calib as oc
        from calib_tab import CalibTab
        import ui_theme
        app = QtWidgets.QApplication(sys.argv[:1])
        ui_theme._ARROW_DIR = Path(directory) / "theme"
        ui_theme.apply(app)
        bags = summary.get('bags') or []
        if isinstance(bags, dict):
            bags = list(bags.values())
        job = oc.new_job([], 'zeroshot', directory, directory)
        job.update(id='저장된_실제_보정_결과', state=oc.DONE, mode=summary.get('mode', 'zeroshot'),
                   bags=[str(b) for b in bags], workdir=str(args.workdir.resolve()), out=str(out.resolve()))
        events = args.workdir / 'events.jsonl'
        if events.is_file():
            job['attempts'] = [{'n': 1, 'stdout': str(events), 'stderr': str(args.workdir / 'log.txt'), 't0': 0}]
        store = oc.JobStore()
        store.add(job)
        tab = CalibTab({'recorder': {'output_dir': directory}, 'ui': {'calib': {'bag_root': directory}}})
        tab.setWindowTitle('온라인 캘리브레이션 · 실제 저장 결과 (센서/레코더 미실행)')
        tab.resize(1680, 1080)
        tab.show()
        failed = []
        if args.screenshot:
            done = []
            def capture(_seq):
                if done or tab.viz.event.get('gate', {}).get('pass') is None:
                    return
                done.append(True)
                def save():
                    if not tab.viz.rig.isValid():
                        failed.append('OpenGL context unavailable: use xvfb-run + LIBGL_ALWAYS_SOFTWARE=1')
                    else:
                        args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                        if not tab.grab().save(str(args.screenshot)):
                            failed.append('Screenshot save failed')
                        else:
                            evidence = {'job_mode': tab.viz.manifest.get('mode'),
                                        'cameras': len(tab.viz.event['cameras']),
                                        'camera_passes': sum(c.get('gate_pass') is True for c in tab.viz.event['cameras'].values()),
                                        'informational_badges': sum(bool(c.get('informational_checks')) for c in tab.viz.event['cameras'].values()),
                                        'pose_source': tab.viz.event.get('provenance', {}).get('pose_source'),
                                        'fps': tab.viz.actual_fps, 'fps_cap': tab.viz.max_fps,
                                        'gl_valid': tab.viz.rig.isValid()}
                            args.screenshot.with_suffix('.json').write_text(json.dumps(evidence, indent=2) + '\n')
                            print(args.screenshot, flush=True)
                    app.quit()
                QtCore.QTimer.singleShot(3500, save)
            tab.viz.snapshot_applied.connect(capture)
            def timeout():
                if not done:
                    failed.append('Timed out waiting for final result')
                    app.quit()
            QtCore.QTimer.singleShot(15000, timeout)
        app.exec_()
        tab.shutdown()
        if failed:
            print('\n'.join(failed), file=sys.stderr)
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
