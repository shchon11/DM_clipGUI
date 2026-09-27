"""Mailbox integration: actual job progress, late streams and lazy camera selection."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from calib_viz.stream import atomic_json, atomic_npz, append_snapshot
from calib_viz.viewer import StreamLoader, CalibrationVizWidget
from test_calib_viz_job import setup as job_setup, finished_result
setup = job_setup
import online_calib as oc


def wait_for(loader, predicate):
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        result, error = loader.take()
        if result and predicate(result):
            return result
        time.sleep(.02)
    raise AssertionError(f'No expected snapshot: {error}')


def publish(root, manifest, seq=1, camera_offset=4., mode='live', run_id='stream'):
    root.mkdir(parents=True, exist_ok=True)
    manifest = dict(manifest, mode=mode, run_id=run_id)
    atomic_json(root / 'manifest.json', manifest)
    pose = np.eye(4)
    pose[0, 3] = camera_offset
    event = {'schema': 'calib-viz/1', 'run_id': run_id, 'seq': seq, 'stage': 'rgb_ba', 'progress': .9,
             'cameras': {name: {'T_cam_lidar': pose.tolist()} for name in manifest['cameras']},
             'assets': {'matching': {}}, 'map_frame': 'W1'}
    for index, name in enumerate(manifest['cameras']):
        cv2.imwrite(str(root / f'{name}.jpg'), np.full((32, 32, 3), 50 + 100 * index, np.uint8))
        atomic_npz(root / f'{name}.npz', points_lidar=np.array([[0., 0., 3.]]))
        event['assets']['matching'][name] = {'image': f'{name}.jpg', 'points': f'{name}.npz'}
    append_snapshot(root, event)
    return event


def test_real_job_fallback_then_late_stream_then_final(setup):
    job, _, tmp = setup
    root = tmp / 'work/viz'
    loader = StreamLoader(root, job=job)
    try:
        initial = wait_for(loader, lambda r: bool(r[1]['cameras']))
        assert initial[1]['interpolate'] is False
        progress = oc.Progress()
        progress.feed({'ev': 'stage_progress', 'stage': 'extract', 'done': 3, 'total': 9, 'eta_s': 100})
        progress.feed({'ev': 'warning', 'msg_ko': '야간 영상 참고'})
        loader.update_job(job, progress)
        update = wait_for(loader, lambda r: r[1].get('window') == 3)
        assert update[1]['cameras'] == initial[1]['cameras']
        assert update[1]['eta_s'] == 100
        publish(root, initial[0])
        live = wait_for(loader, lambda r: r[1]['cameras']['camera_front5']['T_cam_lidar'][0][3] == 4.)
        assert live[1]['stage'] == 'extract'  # actual process event wins
        assert live[1]['window'] == 3
        assert live[1]['warnings'] == ['야간 영상 참고']
        finished_result(tmp / 'out')
        loader.update_job({**job, 'state': oc.DONE}, progress)
        final = wait_for(loader, lambda r: r[1].get('gate', {}).get('pass') is True)
        assert final[1]['cameras']['camera_front5']['T_cam_lidar'][0][3] == .1
        assert final[1]['cameras']['camera_front5']['informational_checks']
        assert final[1]['interpolate'] is False
        assert final[1]['progress'] == 1
    finally:
        loader.close()
        assert not loader.thread.is_alive()


def test_bad_existing_manifest_recovers_and_replay_is_rejected_for_job(setup):
    job, _, tmp = setup
    root = tmp / 'work/viz'
    root.mkdir(parents=True)
    (root / 'manifest.json').write_text('{')
    loader = StreamLoader(root, job=job)
    try:
        initial = wait_for(loader, lambda r: bool(r[1]['cameras']))
        assert loader.thread.is_alive()
        publish(root, initial[0], mode='replay')
        loader.update_job(job, oc.Progress())
        fallback = wait_for(loader, lambda r: '합성' in r[1].get('status_text', ''))
        assert fallback[1]['cameras'] == initial[1]['cameras']
        publish(root, initial[0], seq=2)
        wait_for(loader, lambda r: r[1]['cameras']['camera_front5']['T_cam_lidar'][0][3] == 4.)
    finally:
        loader.close()


def test_only_selected_image_decoded_and_switch_works_without_new_event(setup, monkeypatch):
    from calib_viz.job import JobSnapshot
    from calib_viz.stream import StreamReader
    job, _, tmp = setup
    manifest, _, _ = JobSnapshot(job).snapshot()
    root = tmp / 'viz'
    publish(root, manifest)
    loaded = []
    original = StreamReader.load_asset
    def record(self, path):
        loaded.append(path)
        return original(self, path)
    monkeypatch.setattr(StreamReader, 'load_asset', record)
    loader = StreamLoader(root)
    try:
        wait_for(loader, lambda r: 'camera_front5' in r[2]['matching'])
        assert not any('thermal_left' in path for path in loaded)
        loader.select_camera('thermal_left')
        switched = wait_for(loader, lambda r: 'thermal_left' in r[2]['matching'])
        assert list(switched[2]['matching']) == ['thermal_left']
        assert switched[1]['seq'] == 1
    finally:
        loader.close()


def test_widget_reset_clears_previous_job_data(setup):
    from PyQt5.QtWidgets import QApplication
    from calib_viz.job import JobSnapshot
    app = QApplication.instance() or QApplication([])
    job, _, tmp = setup
    finished_result(tmp / 'out')
    widget = CalibrationVizWidget(embedded=True, max_fps=20)
    try:
        manifest, event, decoded = JobSnapshot({**job, 'state': oc.DONE}).snapshot()
        widget.apply_snapshot(manifest, event, decoded)
        assert widget.camera_combo.count() == 2
        assert widget.event['gate']['pass'] is True
        widget.reset_source()
        assert not widget.event and not widget.targets and not widget.sensors.cameras
        assert widget.map.point_count == 0 and widget.image_panel.image is None
        assert widget.camera_combo.count() == 0
        app.processEvents()
    finally:
        widget.close()


def test_same_embedded_job_new_stream_attempt_resets_pose_history(setup):
    from PyQt5.QtWidgets import QApplication
    from calib_viz.job import JobSnapshot
    app = QApplication.instance() or QApplication([])
    job, _, tmp = setup
    root = tmp / 'work/viz'
    manifest, _, _ = JobSnapshot(job).snapshot()
    publish(root, manifest, seq=90, camera_offset=4., run_id='attempt-1')
    loader = StreamLoader(root, job=job)
    widget = CalibrationVizWidget(embedded=True)
    try:
        first = wait_for(loader, lambda r: r[1].get('run_id') == 'attempt-1')
        widget.apply_snapshot(*first)
        first_manifest_run_id, delivery_seq = first[0]['run_id'], first[1]['seq']
        widget.rig.history['camera_front5'] = ['old ghost']
        widget.current['camera_front5'] = np.array(first[1]['cameras']['camera_front5']['T_cam_lidar'])
        widget.sensors.histories['camera_front5'].append(99.)
        publish(root, manifest, seq=0, camera_offset=2., run_id='attempt-2')
        resumed = wait_for(loader, lambda r: r[1].get('run_id') == 'attempt-2')
        assert resumed[0]['run_id'] == first_manifest_run_id  # stable job selection identity
        assert resumed[1]['seq'] > delivery_seq  # delivery seq alone cannot detect restart
        widget.apply_snapshot(*resumed)
        assert not widget.rig.history
        assert not widget.current
        assert widget.initial['camera_front5'][0, 3] == 2.
        assert len(widget.sensors.histories['camera_front5']) == 1
        assert 99. not in widget.sensors.histories['camera_front5']
        app.processEvents()
    finally:
        widget.close()
        loader.close()


def test_new_attempt_without_viz_ignores_previous_stream_until_matching_run(setup):
    from calib_viz.job import JobSnapshot
    job, _, tmp = setup
    root = tmp / 'work/viz'
    manifest, initial, _ = JobSnapshot(job).snapshot()
    publish(root, manifest, seq=90, camera_offset=4., run_id='old-attempt')
    loader = StreamLoader(root, job=job)
    try:
        wait_for(loader, lambda r: r[1].get('run_id') == 'old-attempt')
        progress = oc.Progress()
        progress.feed({'ev': 'run_start', 'run_id': 'new-no-viz-attempt'})
        loader.update_job(job, progress)
        fallback = wait_for(loader, lambda r: '이전 실행 시도' in r[1].get('status_text', ''))
        assert fallback[1]['cameras'] == initial['cameras']
        assert fallback[1].get('run_id') != 'old-attempt'
        assert fallback[2]['map'] is None and fallback[2]['matching'] == {}
        # Updating real process progress must not bring historical poses back.
        progress.feed({'ev': 'stage_start', 'stage': 'rgb_solve'})
        loader.update_job(job, progress)
        still_initial = wait_for(loader, lambda r: r[1].get('stage') == 'rgb_solve')
        assert still_initial[1]['cameras'] == initial['cameras']
        # Opening the tab after the no-viz restart must gate the FIRST delivery,
        # rather than briefly displaying old poses before update_job arrives.
        reopened = StreamLoader(root, job=job, progress=progress)
        try:
            first_delivery = wait_for(reopened, lambda r: True)
            assert first_delivery[1]['cameras'] == initial['cameras']
            assert first_delivery[2]['matching'] == {}
        finally:
            reopened.close()
        # A matching stream may start late after a resumed/enabled invocation.
        publish(root, manifest, seq=0, camera_offset=2., run_id='new-no-viz-attempt')
        live = wait_for(loader, lambda r: r[1].get('run_id') == 'new-no-viz-attempt')
        assert live[1]['cameras']['camera_front5']['T_cam_lidar'][0][3] == 2.
    finally:
        loader.close()


def test_progress_keeps_window_eta_across_subtasks_and_bounds_reads(tmp_path):
    progress = oc.Progress()
    progress.feed({'ev': 'stage_progress', 'stage': 'extract', 'done': 2, 'total': 25, 'eta_s': 90})
    progress.feed({'ev': 'stage_progress', 'stage': 'extract_scan', 'done': 1, 'total': 1})
    assert progress.viz_progress == {'window': 2, 'total_windows': 25, 'eta_s': 90}
    events = tmp_path / 'events.jsonl'
    line = json.dumps({'ev': 'warning', 'msg': 'x' * 1000}) + '\n'
    events.write_text(line * 1000)
    progress.feed_file(events)
    assert progress._offset <= 256 * 1024
    assert len(progress.warnings) <= 200


@pytest.mark.parametrize('key,value', [('gate_pass', 'false'), ('informational_checks', 'weak'),
                                      ('gate_reasons', [3])])
def test_stream_rejects_malformed_camera_gate_fields(key, value):
    from calib_viz.stream import validate_snapshot
    event = {'schema': 'calib-viz/1', 'seq': 1, 'cameras': {
        'camera_front5': {'T_cam_lidar': np.eye(4).tolist(), key: value}}}
    with pytest.raises(ValueError):
        validate_snapshot(event)
