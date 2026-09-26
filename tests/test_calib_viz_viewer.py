"""Qt headless checks for mailbox resilience and operator state (no GL context)."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')

from pathlib import Path
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
from calib_viz.viewer import CalibrationVizWidget, StreamLoader


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


if __name__ == '__main__':
    unittest.main()
