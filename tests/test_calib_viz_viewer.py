"""Qt headless checks for mailbox resilience and operator state (no GL context)."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')

from pathlib import Path
from copy import deepcopy
import json
import sys
import tempfile
import time
import unittest

import cv2
import numpy as np
from PyQt5 import QtWidgets

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from calib_viz import SCHEMA
from calib_viz.panels import ImagePanel
from calib_viz.stream import append_snapshot, atomic_json, atomic_npz
from calib_viz.viewer import CalibrationVizWidget, StreamLoader, window_context


def manifest():
    return {'schema': SCHEMA, 'run_id': 'test', 'R_lidar_V': np.eye(3).tolist(), 'cameras': {
        'camera_front5': {'K': [[20, 0, 16], [0, 20, 16], [0, 0, 1]], 'D': [0]*4,
                          'model': 'equidistant', 'width': 32, 'height': 32}}}


def snapshot(seq=0):
    return {'schema': SCHEMA, 'seq': seq, 'progress': .5, 'stage': 'rgb_ba', 'cameras': {
        'camera_front5': {'T_cam_lidar': np.eye(4).tolist(), 'reprojection_px': 1.,
                          'sigma_rot_deg': .1, 'sigma_pos_mm': 5.}},
        'assets': {'map': 'map.npz', 'matching': {'camera_front5': {
            'image': 'image.jpg', 'points': 'points.npz', 'width': 32, 'height': 32}}}}


class LoaderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        atomic_json(self.root/'manifest.json', manifest())
        atomic_npz(self.root/'map.npz', points=np.ones((3, 3)), trajectory=np.zeros((1, 3)))
        atomic_npz(self.root/'points.npz', points_lidar=np.array([[0., 0., 2.]]))
        cv2.imwrite(str(self.root/'image.jpg'), np.full((32, 32, 3), 80, np.uint8))
        self.loader = StreamLoader(self.root)

    def tearDown(self):
        self.loader.close()
        self.assertFalse(self.loader.thread.is_alive())
        self.tmp.cleanup()

    def wait_snapshot(self, seq):
        until = time.monotonic() + 3
        while time.monotonic() < until:
            result, error = self.loader.take()
            if result and result[1]['seq'] == seq:
                return result, error
            time.sleep(.02)
        self.fail('reader did not deliver seq ' + str(seq))

    def test_corrupt_map_does_not_block_poses_or_recovery(self):
        (self.root/'bad.npz').write_bytes(b'not a zip file')
        event = snapshot()
        event['assets']['map'] = 'bad.npz'
        append_snapshot(self.root, event)
        result, error = self.wait_snapshot(0)
        self.assertIsNone(result[2]['map'])
        self.assertIn('NPZ', error)
        append_snapshot(self.root, snapshot(1))
        result, error = self.wait_snapshot(1)
        self.assertEqual(len(result[2]['map']['points']), 3)
        self.assertIsNone(error)

    def test_missing_image_retains_last_good_and_marks_stale(self):
        append_snapshot(self.root, snapshot())
        self.wait_snapshot(0)
        event = snapshot(1)
        event['assets']['matching']['camera_front5']['image'] = 'missing.jpg'
        append_snapshot(self.root, event)
        result, error = self.wait_snapshot(1)
        preview = result[2]['matching']['camera_front5']
        self.assertTrue(preview['metadata']['stale'])
        self.assertEqual(preview['image'].shape, (32, 32, 3))
        self.assertIn('missing.jpg', error)

    def test_new_bag_frame_does_not_retain_previous_map_or_image(self):
        first = dict(snapshot(), map_frame='lidar_window:bag-A/S01')
        append_snapshot(self.root, first)
        self.wait_snapshot(0)
        second = dict(snapshot(1), map_frame='lidar_window:bag-B/S01', bag_id='bag-B', window_id='S01')
        second['assets']['map'] = 'next_map.npz'
        second['assets']['matching']['camera_front5']['image'] = 'next_image.jpg'
        append_snapshot(self.root, second)
        result, error = self.wait_snapshot(1)
        self.assertIsNone(result[2]['map'])
        self.assertEqual(result[2]['matching'], {})
        self.assertTrue(error)

    def test_live_camera_control_is_atomic_rate_limited_and_not_for_replay(self):
        live = dict(manifest(), mode='live')
        live['cameras']['thermal_left'] = live['cameras']['camera_front5']
        self.loader._publish_selection(live, 'camera_front5')
        path = self.root / 'control.json'
        before = path.read_bytes()
        control = json.loads(before)
        self.assertEqual(control['schema'], 'calib-viz-control/1')
        self.assertEqual(control['camera'], 'camera_front5')
        self.assertEqual(control['run_id'], 'test')
        self.loader._publish_selection(live, 'thermal_left')
        self.assertEqual(path.read_bytes(), before)
        self.loader._control_at -= 1.1
        self.loader._publish_selection(live, 'thermal_left')
        self.assertEqual(json.loads(path.read_text())['camera'], 'thermal_left')
        self.assertEqual(list(self.root.glob('control.json.*.tmp')), [])
        self.loader._control_at -= 1.1
        self.loader._publish_selection(dict(live, mode='replay'), 'camera_front5')
        self.assertEqual(json.loads(path.read_text())['camera'], 'thermal_left')


class WidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_sequence_reset_refreshes_reused_map_name_and_failure_gate(self):
        widget = CalibrationVizWidget()
        data = {'map': {'points': np.ones((3, 3))}, 'matching': {}}
        widget.apply_snapshot(manifest(), snapshot(9), data)
        self.assertEqual(widget.map.point_count, 3)
        event = snapshot(0)
        event['gate'] = {'status': 'fail'}
        widget.apply_snapshot(manifest(), event, {'map': {'points': np.ones((5, 3))}, 'matching': {}})
        self.assertEqual(widget.map.point_count, 5)
        self.assertIn('#ff737c', widget.gate_label.styleSheet())
        widget.pause_button.setChecked(True)
        self.assertTrue(widget.paused)
        widget.shutdown()
        self.assertFalse(widget.timer.isActive())
        widget.close()

    def test_mismatched_mask_is_ignored_without_projection_crash(self):
        panel = ImagePanel()
        panel.set_frame(np.zeros((32, 32, 3), np.uint8),
                        {'points_lidar': np.array([[0., 0., 2.]]), 'mask': np.zeros((2, 2), np.uint8)},
                        manifest()['cameras']['camera_front5'], {})
        panel.project(np.eye(4))
        self.assertEqual(len(panel.uv), 1)
        panel.close()

    def test_camera_switch_without_image_clears_previous_projection(self):
        widget = CalibrationVizWidget()
        decoded = {'matching': {'camera_front5': {
            'image': np.zeros((32, 32, 3), np.uint8),
            'points': {'points_lidar': np.array([[0., 0., 2.]]),
                       'tracks_uv': np.array([[16., 16.]]),
                       'tracks_prev_uv': np.array([[15., 16.]]),
                       'mask': np.full((32, 32), 255, np.uint8)},
            'metadata': {'source_window': 'S01'}}}}
        try:
            source = manifest()
            source['cameras']['thermal_left'] = dict(source['cameras']['camera_front5'],
                                                      model='plumb_bob', D=[0.] * 5, sensor='thermal')
            widget.apply_snapshot(source, snapshot(), decoded)
            panel = widget.image_panel
            panel.show_initial = True
            panel.project(np.eye(4), np.eye(4))
            self.assertEqual(len(panel.uv), 1)
            self.assertEqual(len(panel.tracks), 1)
            widget.select_camera('thermal_left')
            self.assertIsNone(panel.image)
            for name in ('points', 'tracks', 'tracks_prev', 'uv', 'depth', 'initial_uv'):
                self.assertEqual(len(getattr(panel, name)), 0, name)
            self.assertIsNone(panel.calibration)
            self.assertIsNone(panel.initial_calibration)
            self.assertIsNone(panel.mask)
            self.assertEqual(panel.asset_size, (1, 1))
            self.assertEqual(widget.metrics.metrics, {})
            panel.project(np.eye(4), np.eye(4))
            self.assertEqual(len(panel.uv), 0)
        finally:
            widget.close()

    def test_iteration_intrinsics_drive_projection_and_metrics(self):
        widget = CalibrationVizWidget()
        data = {'matching': {'camera_front5': {
            'image': np.zeros((32, 32, 3), np.uint8),
            'points': {'points_lidar': np.array([[.5, 0., 2.]])}, 'metadata': {}}}}
        initial = snapshot()
        widget.apply_snapshot(manifest(), initial, data)
        widget.image_panel.show_initial = True
        widget.image_panel.project(np.eye(4), np.eye(4))
        initial_uv = widget.image_panel.uv.copy()
        event = deepcopy(snapshot(1))
        event.update(solver_pass='B2', iteration=3, total_iterations=8,
                     bag_id='bag-B', window_id='S02')
        event['cameras']['camera_front5'].update(
            K=[[40, 0, 16], [0, 40, 16], [0, 0, 1]], D=[.01, 0, 0, 0],
            cost=10.3, time_offset_s=-.003)
        widget.apply_snapshot(manifest(), event, data)
        widget.image_panel.project(np.eye(4), np.eye(4))
        self.assertGreater(widget.image_panel.uv[0, 0], initial_uv[0, 0])
        np.testing.assert_allclose(widget.image_panel.initial_uv, initial_uv)
        self.assertIn('10.3', widget.solver_label.text())
        self.assertIn('-3.000 ms', widget.solver_label.text())
        self.assertIn('B2', widget.detail_label.text())
        self.assertIn('bag-B', widget.detail_label.text())
        self.assertIn('3 / 8', widget.detail_label.text())
        self.assertEqual(widget.calibrations['camera_front5']['K'][0][0], 40)
        self.assertEqual(widget.initial_calibrations['camera_front5']['K'][0][0], 20)
        resumed = dict(event, run_id='resumed', seq=2)
        widget.apply_snapshot(manifest(), resumed, data)
        self.assertEqual(widget.initial_calibrations['camera_front5']['K'][0][0], 40)
        widget.close()

    def test_explicit_synthetic_intermediate_is_always_labelled(self):
        widget = CalibrationVizWidget()
        event = dict(snapshot(), provenance={'synthetic_intermediate': True})
        widget.apply_snapshot(manifest(), event, {'matching': {}})
        self.assertIn('합성', widget.mode_label.text())
        widget.close()

    def test_selected_camera_context_is_distinct_from_latest_parallel_task(self):
        widget = CalibrationVizWidget()
        source = manifest()
        source['cameras']['thermal_left'] = dict(source['cameras']['camera_front5'],
                                                  model='plumb_bob', D=[0.] * 5)
        event = snapshot()
        event.update(solve_name='rgb_joint', solver_pass='B3', iteration=8, total_iterations=12,
                     windows=['S01', 'S02'], window_id='S01', window=2, total_windows=2)
        event['cameras']['thermal_left'] = {
            'T_cam_lidar': np.eye(4).tolist(), 'solve_name': 'thermal_left_joint',
            'solver_pass': 'B1', 'purpose': 'calibration', 'iteration': 2, 'total_iterations': 4,
            'time_offset_s': -.02, 'row_readout_s': 13e-6, 'rs_s': 13e-6, 'windows': ['S01', 'S02']}
        try:
            widget.apply_snapshot(source, event, {'matching': {}})
            widget.select_camera('thermal_left')
            self.assertIn('rgb_joint', widget.detail_label.text())
            self.assertIn('B3', widget.detail_label.text())
            self.assertIn('공동 2개 구간: S01, S02', widget.detail_label.text())
            self.assertIn('2 / 2', widget.window_label.text())
            self.assertIn('thermal_left_joint', widget.solver_label.text())
            self.assertIn('pass B1', widget.solver_label.text())
            self.assertIn('반복 2 / 4', widget.solver_label.text())
            self.assertIn('목적 calibration', widget.solver_label.text())
            self.assertIn('공동 2개 구간: S01, S02', widget.solver_label.text())
            self.assertIn('13.00 µs', widget.solver_label.text())
            final = deepcopy(event)
            final['seq'] += 1
            final['cameras']['thermal_left'].update(solver_pass='final', iteration=None,
                                                     total_iterations=None)
            widget.apply_snapshot(source, final, {'matching': {}})
            self.assertIn('pass final', widget.solver_label.text())
            self.assertNotIn('반복', widget.solver_label.text())
            self.assertNotIn('None', widget.solver_label.text())
        finally:
            widget.close()

    def test_window_context_bounds_long_joint_solve_lists(self):
        self.assertEqual(window_context([]), '')
        self.assertEqual(window_context(['S01']), '구간 S01')
        self.assertEqual(window_context([f'S{i:02}' for i in range(1, 26)]),
                         '공동 25개 구간: S01, S02, S03, …')

    def test_rgb_only_manifest_counts_and_rejects_inactive_camera(self):
        widget = CalibrationVizWidget()
        source = manifest()
        camera = source['cameras']['camera_front5']
        names = [f'camera_front{i}' for i in range(1, 10)] + [
            'camera_top', 'camera_side_left', 'camera_side_right', 'camera_rear_left', 'camera_rear_right']
        source['cameras'] = {name: dict(camera, sensor='rgb') for name in names}
        event = snapshot()
        try:
            self.assertTrue(widget.select_camera('thermal_right'))  # caller may select before first manifest
            widget.apply_snapshot(source, event, {'matching': {}})
            self.assertEqual(widget.selected, 'camera_front5')
            self.assertEqual(widget.rig_sensor_label.text(), 'RGB 14 · 열화상 0 · 카메라 14대 · Ouster')
            self.assertEqual(widget.camera_combo.count(), 14)
            self.assertFalse(widget.select_camera('thermal_left'))
            self.assertEqual(widget.selected, 'camera_front5')
            self.assertTrue(widget.select_camera('camera_side_left'))
            self.assertEqual(widget.selected, 'camera_side_left')
            widget.reset_source()
            self.assertEqual(widget.rig_sensor_label.text(), '센서 구성 대기')
        finally:
            widget.close()


if __name__ == '__main__':
    unittest.main()
