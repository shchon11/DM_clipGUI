"""Small Qt-painted HUDs; no plotting workers or growing histories."""
from collections import deque

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .geometry import project_camera

INK = '#e0e9f5'
MUTED = '#8496af'
TEAL = '#49dfd6'
PURPLE = '#b0a8ff'
STAGES = [('extract', '데이터 추출'), ('lidar_odometry', 'LiDAR 주행 복원'),
          ('rgb_ba', 'RGB 공동 최적화'), ('thermal', '열화상 정합'), ('validation', '최종 검증')]


def font(size=10, bold=False, mono=False):
    return QtGui.QFont('DejaVu Sans Mono' if mono else 'Noto Sans CJK KR', size,
                       QtGui.QFont.Bold if bold else QtGui.QFont.Normal)


def short_name(name):
    if name.startswith('camera_front'):
        return '전방 ' + name[12:].zfill(2)
    return {'camera_top': '상단', 'camera_side_left': '측면 좌', 'camera_side_right': '측면 우',
            'camera_rear_left': '후방 좌', 'camera_rear_right': '후방 우',
            'thermal_left': '열화상 좌', 'thermal_right': '열화상 우'}.get(name, name)


def number(value, places=2):
    return '—' if value is None else f'{float(value):.{places}f}'


class StageBar(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(67)
        self.stage = ''
        self.progress = 0

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(p.Antialiasing)
        active = next((i for i, (key, _) in enumerate(STAGES) if key == self.stage), -1)
        step = self.width() / 5
        p.setPen(QtGui.QPen(QtGui.QColor('#26364c'), 1))
        for i in range(4):
            p.drawLine(int(i * step + 192), 22, int((i+1) * step), 22)
        for i, (_, label) in enumerate(STAGES):
            x = int(i * step) + 18
            col = TEAL if i <= active else '#41536e'
            p.setBrush(QtGui.QColor('#13383d' if i == active else '#142133'))
            p.setPen(QtGui.QPen(QtGui.QColor(col), 1.5))
            p.drawEllipse(QtCore.QPointF(x, 22), 12, 12)
            p.setFont(font(9, True))
            p.drawText(QtCore.QRectF(x - 12, 9, 24, 24), QtCore.Qt.AlignCenter,
                       '✓' if i < active else str(i + 1))
            p.setPen(QtGui.QColor(INK if i == active else MUTED))
            p.drawText(x + 22, 27, label)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor('#1b2b40'))
        p.drawRoundedRect(QtCore.QRectF(4, 53, self.width() - 8, 3), 1.5, 1.5)
        p.setBrush(QtGui.QColor(TEAL))
        p.drawRoundedRect(QtCore.QRectF(4, 53, (self.width() - 8) * self.progress, 3), 1.5, 1.5)


class SensorGrid(QtWidgets.QWidget):
    selected = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(550, 266)
        self.cameras = {}
        self.histories = {}
        self.active = 'camera_front5'
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setMouseTracking(True)

    def ingest(self, cameras):
        self.cameras = cameras
        for name, metrics in cameras.items():
            hist = self.histories.setdefault(name, deque(maxlen=80))
            hist.append(metrics.get('reprojection_px'))
        self.update()

    def mousePressEvent(self, event):
        names = list(self.cameras)
        row_height = (self.height() - 24) / 8
        index = int((event.y() - 24) // row_height) + (8 if event.x() >= self.width() / 2 else 0)
        if event.y() >= 24 and 0 <= index < len(names):
            self.active = names[index]
            self.selected.emit(self.active)
            self.update()

    def mouseMoveEvent(self, event):
        names = list(self.cameras)
        row_height = (self.height() - 24) / 8
        index = int((event.y() - 24) // row_height) + (8 if event.x() >= self.width() / 2 else 0)
        if event.y() < 24 or not 0 <= index < len(names):
            self.setToolTip("")
            return
        metric = self.cameras[names[index]]
        status = {True: "카메라 게이트 통과", False: "카메라 게이트 실패", None: "검증 대기"}.get(
            metric.get("gate_pass"), "검증 대기")
        notes = [status] + list(metric.get("gate_reasons") or []) + list(metric.get("informational_checks") or [])
        if metric.get('cost') is not None:
            notes.append(f"비용 {metric['cost']:.6g} ({metric.get('cost_source', 'solver')})")
        if metric.get('time_offset_s') is not None:
            notes.append(f"시간 offset {metric['time_offset_s'] * 1000:+.3f} ms")
        self.setToolTip("\n".join(notes))

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(p.Antialiasing)
        half = self.width() / 2
        row_height = (self.height() - 24) / 8
        show_cost = (any(m.get('cost') is not None for m in self.cameras.values())
                     and not any(m.get('sigma_pos_mm') is not None for m in self.cameras.values()))
        for col in range(2):
            x = col * half
            p.setFont(font(8))
            p.setPen(QtGui.QColor(MUTED))
            for offset, label in [(12, '센서'), (.34 * half, '회전 °'), (.51 * half, '비용' if show_cost else '위치 mm'),
                                   (.70 * half, '오차 px'), (.87 * half, '추이')]:
                p.drawText(int(x + offset), 15, label)
        for i, (name, m) in enumerate(self.cameras.items()):
            x, y = (i // 8) * half, 24 + (i % 8) * row_height
            active = name == self.active
            color = (TEAL if m.get('gate_pass') is True else MUTED)
            if m.get('state') == 'failed' or m.get('gate_pass') is False:
                color = '#ff737c'
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor('#202747' if active else ('#111e30' if i % 2 == 0 else '#101a2b')))
            p.drawRoundedRect(QtCore.QRectF(x + 2, y, half - 8, row_height - 2), 4, 4)
            p.setBrush(QtGui.QColor(color))
            p.drawEllipse(QtCore.QPointF(x + 13, y + row_height / 2 - 1), 2.5, 2.5)
            p.setFont(font(9, active))
            p.setPen(QtGui.QColor(INK if active else '#b4c5d9'))
            p.drawText(int(x + 25), int(y + row_height / 2 + 4), short_name(name))
            if m.get('informational_checks'):
                p.setFont(font(8))
                p.setPen(QtGui.QColor(MUTED))
                p.drawText(int(x + .29 * half), int(y + row_height / 2 + 4), 'ⓘ')
            p.setPen(QtGui.QColor(INK if active else '#b4c5d9'))
            p.setFont(font(9, mono=True))
            for offset, key, places in [(.34, 'sigma_rot_deg', 2), (.51, 'cost' if show_cost else 'sigma_pos_mm', 1),
                                         (.70, 'reprojection_px', 2)]:
                value = m.get(key)
                display = f'{value:.2g}' if key == 'cost' and value is not None else number(value, places)
                p.drawText(int(x + offset * half), int(y + row_height / 2 + 4), display)
            values = [v for v in self.histories.get(name, []) if v is not None]
            if len(values) > 1:
                low, high = min(values), max(values)
                points = [QtCore.QPointF(x + .86 * half + j / (len(values) - 1) * .10 * half,
                                         y + row_height - 7 - (v - low) / max(high-low, .1) * (row_height-13))
                          for j, v in enumerate(values)]
                p.setPen(QtGui.QPen(QtGui.QColor(PURPLE if active else color), 1.2))
                p.drawPolyline(QtGui.QPolygonF(points))


class MetricStrip(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(74)
        self.metrics = {}
        self.histories = {key: deque(maxlen=100) for key in ['sigma_rot_deg', 'sigma_pos_mm', 'reprojection_px']}
        self.name = None

    def ingest(self, name, metrics):
        if name != self.name:
            for values in self.histories.values():
                values.clear()
        self.name, self.metrics = name, metrics
        for key, values in self.histories.items():
            value = metrics.get(key)
            if value is not None:
                values.append(float(value))
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(p.Antialiasing)
        for i, (key, title, unit) in enumerate([('sigma_rot_deg', '회전 1σ', '°'),
                                                ('sigma_pos_mm', '위치 1σ', 'mm'),
                                                ('reprojection_px', '재투영', 'px')]):
            x, w = i * self.width() / 3, self.width() / 3
            p.setPen(QtGui.QColor(MUTED))
            p.setFont(font(8))
            p.drawText(int(x + 12), 18, title)
            p.setPen(QtGui.QColor(INK))
            p.setFont(font(17, mono=True))
            p.drawText(int(x + 10), 44, number(self.metrics.get(key), 2))
            p.setFont(font(9))
            p.setPen(QtGui.QColor(MUTED))
            p.drawText(int(x + w - 31), 43, unit)
            values = self.histories[key]
            if len(values) > 1:
                low, high = min(values), max(values)
                points = [QtCore.QPointF(x + 12 + j/(len(values)-1)*(w-25),
                                         67-(v-low)/max(high-low, .001)*13) for j, v in enumerate(values)]
                p.setPen(QtGui.QPen(QtGui.QColor(PURPLE), 1.3))
                p.drawPolyline(QtGui.QPolygonF(points))


class ImagePanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(360, 220)
        self.image = None
        self.points = np.empty((0, 3))
        self.tracks = np.empty((0, 2))
        self.tracks_prev = np.empty((0, 2))
        self.uv = np.empty((0, 2))
        self.depth = np.empty(0)
        self.initial_uv = np.empty((0, 2))
        self.show_initial = False
        self.show_tracks = True
        self.show_points = True
        self.calibration = None
        self.initial_calibration = None
        self.asset_size = (1, 1)
        self.initial_pose = None
        self.mask = None
        self.note = '동기화된 이미지 대기 중'
        self.depth_colors = [QtGui.QColor.fromHsvF(.70 * (1 - i/15), .82, 1) for i in range(16)]

    def set_frame(self, rgb, data, calibration, metadata, initial_calibration=None):
        if rgb is None:
            self.image = None
            self.points = np.empty((0, 3))
            self.tracks = np.empty((0, 2))
            self.tracks_prev = np.empty((0, 2))
            self.uv = np.empty((0, 2))
            self.depth = np.empty(0)
            self.initial_uv = np.empty((0, 2))
            self.calibration = None
            self.initial_calibration = None
            self.initial_pose = None
            self.mask = None
            self.asset_size = (1, 1)
            self.note = '동기화된 이미지 대기 중'
            self.update()
            return
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        h, w = rgb.shape[:2]
        self.image = QtGui.QImage(rgb.data, w, h, rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        self.points = np.asarray(data.get('points_lidar', np.empty((0, 3))))[:5000]
        self.tracks = np.asarray(data.get('tracks_uv', np.empty((0, 2))))[:100]
        self.tracks_prev = np.asarray(data.get('tracks_prev_uv', np.empty((0, 2))))[:100]
        self.calibration = calibration
        self.initial_calibration = initial_calibration or calibration
        self.mask = data.get('mask')
        if self.mask is not None and self.mask.shape != (h, w):
            self.mask = None
        self.asset_size = (w, h)
        self.note = f"{metadata.get('source_window', '실시간')} · 관측 특징 {len(self.tracks)}개"
        if metadata.get('bag_id'):
            self.note = f"{metadata['bag_id']} · " + self.note
        if metadata.get('stale'):
            self.note += ' · 이전 영상 (자산 대기)'
        a, b = metadata.get('source_stamp_ns'), metadata.get('lidar_stamp_ns')
        if a is not None and b is not None:
            self.note += f' · 입력 헤더차 {abs(float(a)-float(b))/1e6:.1f} ms'

    def project(self, pose, initial_pose=None):
        if not self.calibration or self.image is None:
            return
        c = self.calibration
        uv, depth, valid = project_camera(self.points, pose, c['K'], c['D'], c.get('model', 'equidistant'))
        scale = np.array(self.asset_size) / [c['width'], c['height']]
        uv = uv * scale
        valid &= np.isfinite(uv).all(axis=1)
        valid &= (uv[:, 0] >= 0) & (uv[:, 0] < self.asset_size[0])
        valid &= (uv[:, 1] >= 0) & (uv[:, 1] < self.asset_size[1])
        if self.mask is not None:
            indices = np.flatnonzero(valid)
            pix = uv[indices].astype(int)
            valid[indices] &= self.mask[pix[:, 1], pix[:, 0]] > 127
        self.uv, self.depth = uv[valid], depth[valid]
        if self.show_initial and initial_pose is not None:
            c0 = self.initial_calibration
            initial, _, ok = project_camera(self.points, initial_pose, c0['K'], c0['D'], c0.get('model', 'equidistant'))
            initial_scale = np.array(self.asset_size) / [c0['width'], c0['height']]
            self.initial_uv = initial[valid] * initial_scale
            self.initial_uv[~ok[valid]] = np.nan
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor('#080f1b'))
        if self.image is None:
            p.setPen(QtGui.QColor(MUTED))
            p.setFont(font(11))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, '선택한 카메라의 영상 스트림 대기 중')
            return
        w, h = self.asset_size
        area_h = self.height() - 26
        scale = min(self.width() / w, area_h / h)
        x, y = (self.width() - w*scale)/2, (area_h - h*scale)/2
        p.drawImage(QtCore.QRectF(x, y, w*scale, h*scale), self.image)
        p.save()
        p.setClipRect(QtCore.QRectF(x, y, w*scale, h*scale))
        p.translate(x, y)
        p.scale(scale, scale)
        if self.show_initial and len(self.initial_uv) == len(self.uv):
            p.setPen(QtGui.QPen(QtGui.QColor(195, 190, 255, 130), 1 / scale))
            for start, end in zip(self.initial_uv[::12], self.uv[::12]):
                if np.isfinite(start).all():
                    p.drawLine(QtCore.QPointF(*start), QtCore.QPointF(*end))
                    p.drawEllipse(QtCore.QPointF(*start), 2/scale, 2/scale)
        if self.show_points:
            bins = np.clip((self.depth / 35 * 15).astype(int), 0, 15)
            for i, color in enumerate(self.depth_colors):
                p.setPen(QtGui.QPen(color, 2.1 / scale, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                p.drawPoints(QtGui.QPolygonF([QtCore.QPointF(*point) for point in self.uv[bins == i]]))
        if self.show_tracks:
            p.setPen(QtGui.QPen(QtGui.QColor('#c6fa9d'), 1 / scale))
            for i, point in enumerate(self.tracks):
                if not np.isfinite(point).all():
                    continue
                if i < len(self.tracks_prev) and np.isfinite(self.tracks_prev[i]).all():
                    p.drawLine(QtCore.QPointF(*self.tracks_prev[i]), QtCore.QPointF(*point))
                p.drawRect(QtCore.QRectF(point[0]-3/scale, point[1]-3/scale, 6/scale, 6/scale))
        p.restore()
        p.setFont(font(8))
        p.setPen(QtGui.QColor(MUTED))
        p.drawText(10, self.height() - 8, f'{len(self.uv):,} 투영점 · {self.note}')
        # Depth legend is drawn over a dark plate so night images remain readable.
        p.fillRect(QtCore.QRectF(self.width()-166, 9, 157, 28), QtGui.QColor(8, 15, 27, 220))
        for i, color in enumerate(self.depth_colors):
            p.fillRect(QtCore.QRectF(self.width()-157+i*6, 20, 6, 3), color)
        p.setPen(QtGui.QColor('#d0dceb'))
        p.drawText(self.width()-57, 26, '35 m')
