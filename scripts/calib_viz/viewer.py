"""Embeddable calibration viewer. The solver never imports Qt or waits for it."""
from collections import OrderedDict
from copy import deepcopy
from pathlib import Path
import threading
import time

import numpy as np
from PyQt5 import QtCore, QtWidgets

from .geometry import interpolate_pose, vehicle_from_camera
from .panels import ImagePanel, MetricStrip, SensorGrid, StageBar, short_name
from .scene import MapScene, RigScene
from .stream import StreamReader, atomic_json, snapshot_calibrations


STYLE = '''
QWidget#CalibViz { background: #080f1c; color: #e0e9f5; }
QWidget { color: #d6e2f1; font-family: "Noto Sans CJK KR"; font-size: 12px; }
QFrame#Card { background: #101a2b; border: 1px solid #253449; border-radius: 10px; }
QLabel { border: none; background: transparent; }
QLabel#Title { font-size: 24px; font-weight: 700; color: #edf5ff; }
QLabel#Subtle { color: #8496af; font-size: 11px; }
QLabel#CardTitle { color: #dce8f6; font-size: 13px; font-weight: 600; }
QLabel#Badge { background: #133632; color: #5ee5cc; padding: 6px 11px; border-radius: 5px; }
QPushButton { background: #18263c; border: 1px solid #2c405c; border-radius: 5px; padding: 5px 11px; }
QPushButton:hover { background: #263857; border-color: #64809d; }
QPushButton:checked { background: #303456; border-color: #7970be; color: #c8c1ff; }
QComboBox { background: #172439; border: 1px solid #344761; border-radius: 4px; padding: 4px 10px; }
QComboBox QAbstractItemView { background: #172439; selection-background-color: #3a4569; }
QCheckBox { color: #9cafc7; spacing: 5px; font-size: 11px; }
QToolTip { background: #203148; color: white; border: 1px solid #668199; }
'''


def label(text, kind=None):
    widget = QtWidgets.QLabel(text)
    if kind:
        widget.setObjectName(kind)
    return widget


def window_context(windows):
    if not isinstance(windows, (list, tuple)) or not windows:
        return ''
    shown = ', '.join(str(window) for window in windows[:3])
    if len(windows) > 3:
        shown += ', …'
    return (f'공동 {len(windows)}개 구간: ' if len(windows) > 1 else '구간 ') + shown


def card(title, subtitle='', controls=()):
    frame = QtWidgets.QFrame()
    frame.setObjectName('Card')
    layout = QtWidgets.QVBoxLayout(frame)
    layout.setContentsMargins(14, 11, 14, 10)
    layout.setSpacing(7)
    header = QtWidgets.QHBoxLayout()
    header.addWidget(label(title, 'CardTitle'))
    if subtitle:
        header.addWidget(label(subtitle, 'Subtle') if isinstance(subtitle, str) else subtitle)
    header.addStretch()
    for widget in controls:
        header.addWidget(widget)
    layout.addLayout(header)
    return frame, layout


class StreamLoader:
    """One bounded I/O mailbox; only the selected camera's frames are decoded.

    Real jobs share this worker with their small initial/final YAML adapter. No
    solver imports, replay generation or workdir scan runs on the GUI thread.
    """
    def __init__(self, root, job=None, progress=None):
        self.root = Path(root)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.selected = 'camera_front5'
        self.job = deepcopy(job)
        self.progress = deepcopy(progress)
        self.revision = 0
        self.active = True
        self._control = None
        self._control_at = -float('inf')
        self.thread = threading.Thread(target=self._run, name='calib-viz-reader', daemon=True)
        self.thread.start()

    def update_job(self, job, progress):
        with self.lock:
            self.job, self.progress = deepcopy(job), deepcopy(progress)
            self.revision += 1

    def select_camera(self, name):
        with self.lock:
            self.selected = name

    def take(self):
        with self.lock:
            latest, self.latest = self.latest, None
            return latest, self.error

    def _publish_selection(self, manifest, camera):
        """Optional producer hint; read-only streams remain fully viewable."""
        if manifest.get('mode') != 'live' or camera not in manifest.get('cameras', {}):
            return
        selection = (manifest.get('run_id'), camera)
        now = time.monotonic()
        if selection == self._control or now - self._control_at < 1.:
            return
        self._control_at = now
        try:
            atomic_json(self.root / 'control.json', {
                'schema': 'calib-viz-control/1', 'run_id': selection[0],
                'camera': camera, 'enabled': True, 'updated_at': time.time()})
            self._control = selection
        except (OSError, ValueError):
            pass

    def _run(self):
        reader, cache = None, OrderedDict()
        previous = {'map': None, 'matching': {}}
        generation, last_seq, selected, revision = None, -1, None, -1
        stream_event, fallback, adapter = None, None, None
        delivery = 0
        retry = False
        while not self.stop_event.is_set():
            if not self.active:
                self.stop_event.wait(.25)
                continue
            try:
                with self.lock:
                    job, progress, current_revision, camera = self.job, self.progress, self.revision, self.selected
                changed = current_revision != revision or camera != selected
                if job is not None and current_revision != revision:
                    if adapter is None:
                        from .job import JobSnapshot
                        adapter = JobSnapshot(job)
                    fallback = adapter.snapshot(progress, job=job)
                revision, selected = current_revision, camera
                stream_error = None
                try:
                    if reader is None:
                        reader = StreamReader(self.root)
                    reader.refresh_manifest()
                    run_id = reader.manifest.get('run_id')
                    attempt_id = (getattr(progress, 'run_start', None) or {}).get('run_id') if job else None
                    obsolete = bool(attempt_id and run_id != attempt_id)
                    if obsolete:
                        # A new --no-viz attempt intentionally leaves the old
                        # stream directory untouched. Its poses are historical.
                        changed = changed or stream_event is not None
                        stream_event, last_seq = None, -1
                        cache.clear()
                        previous = {'map': None, 'matching': {}}
                        if reader.manifest:
                            stream_error = '이전 실행 시도의 스트림은 표시하지 않습니다'
                    else:
                        self._publish_selection(reader.manifest, camera)
                    if generation is not None and generation != run_id:
                        cache.clear()
                        previous = {'map': None, 'matching': {}}
                        stream_event, last_seq, changed = None, -1, True
                    generation = run_id
                    events = [] if obsolete else reader.poll()
                    if job and reader.manifest.get('mode') == 'replay':
                        events = []
                        stream_event = None
                        stream_error = '실제 작업에서는 합성 리플레이 스트림을 표시하지 않습니다'
                    if events:
                        candidate = events[-1]
                        provenance = candidate.get('provenance', {})
                        synthetic = provenance.get('synthetic_intermediate') or provenance.get('synthetic')
                        if job and (synthetic or provenance.get('mode') == 'replay'):
                            stream_error = '실제 작업에서는 합성 중간 추정을 표시하지 않습니다'
                            stream_event = None
                        else:
                            stream_event = candidate
                            changed = True
                            if candidate['seq'] <= last_seq or generation != run_id:
                                cache.clear()
                                previous = {'map': None, 'matching': {}}
                            generation, last_seq = run_id, candidate['seq']
                except (OSError, ValueError, KeyError, EOFError) as exc:
                    stream_error = str(exc)
                if not changed and not retry:
                    self.stop_event.wait(.25)
                    continue
                if stream_event is None and fallback is None:
                    if stream_error:
                        with self.lock:
                            self.error = stream_error
                    self.stop_event.wait(.25)
                    continue
                manifest = dict(reader.manifest) if stream_event else dict(fallback[0])
                event = dict(stream_event) if stream_event else dict(fallback[1])
                if fallback:
                    # Progress/warnings come from the actual job even when poses
                    # arrive separately. The final output supersedes an old pose.
                    status = fallback[1]
                    for key in ('stage', 'stage_label', 'progress', 'window', 'total_windows',
                                'eta_s', 'warnings', 'status_text', 'job_state'):
                        if key in status:
                            event[key] = status[key]
                    final = status.get('provenance', {}).get('pose_source') == 'final-result'
                    if final:
                        event.update(cameras=status['cameras'], gate=status.get('gate', {}), interpolate=False)
                        manifest.update(cameras=fallback[0]['cameras'], R_lidar_V=fallback[0]['R_lidar_V'])
                    manifest['mode'] = 'job'
                    manifest['run_id'] = fallback[0]['run_id']
                    event['seq'] = delivery
                    delivery += 1
                    if stream_event and not final:
                        event['status_text'] = '실제 작업 진행 · 발행된 중간 포즈'
                    if stream_error:
                        event['status_text'] = event.get('status_text', '') + ' · ' + stream_error

                def asset(path):
                    if not path:
                        return None
                    if path not in cache:
                        cache[path] = reader.load_asset(path)
                        while len(cache) > 5:
                            cache.popitem(last=False)
                    else:
                        cache.move_to_end(path)
                    return cache[path]

                assets = event.get('assets', {})
                errors = []
                frame = event.get('map_frame', manifest.get('map_frame'))
                if frame != previous.get('frame'):
                    previous = {'map': None, 'matching': {}}
                decoded = {'map': previous.get('map'), 'map_asset': previous.get('map_asset'),
                           'frame': frame, 'matching': {}}
                try:
                    decoded['map'] = asset(assets.get('map'))
                    decoded['map_asset'] = assets.get('map')
                except (OSError, ValueError, KeyError) as exc:
                    errors.append(f'지도 자산 대기: {exc}')
                metadata = assets.get('matching', {}).get(camera)
                if metadata:
                    try:
                        decoded['matching'][camera] = {'image': asset(metadata.get('image')),
                                                       'points': asset(metadata.get('points')) or {},
                                                       'metadata': metadata}
                    except (OSError, ValueError, KeyError) as exc:
                        errors.append(f'영상 자산 대기: {exc}')
                        if camera in previous['matching']:
                            old = previous['matching'][camera]
                            decoded['matching'][camera] = dict(old, metadata=dict(old['metadata'], stale=True))
                previous = decoded
                retry = bool(errors)
                snapshot_calibrations(manifest, event)
                with self.lock:
                    self.latest = (manifest, event, decoded)
                    self.error = errors[-1] if errors else (str(reader.errors[-1]) if reader and reader.errors else None)
                if reader:
                    reader.errors.clear()
            except (OSError, ValueError, KeyError, EOFError, TypeError) as exc:
                with self.lock:
                    self.error = str(exc)
                # Retry partially published initial/final output files.
                revision = -1
            self.stop_event.wait(.25)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=.5)


class CalibrationVizWidget(QtWidgets.QWidget):
    """Add this QWidget to an existing tab, then call set_stream(workdir / 'viz').

    Timers and the optional reader thread stop in closeEvent()/shutdown(). If a
    tab is removed without being closed, the host must call shutdown() explicitly.
    """
    snapshot_applied = QtCore.pyqtSignal(int)

    def __init__(self, stream_dir=None, parent=None, max_fps=40, max_points=70000, embedded=False):
        super().__init__(parent)
        self.setObjectName('CalibViz')
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.setStyleSheet(STYLE)
        self.manifest, self.event, self.decoded = {}, {}, {}
        self.calibrations, self.initial_calibrations = {}, {}
        self.targets, self.origins, self.current, self.initial = {}, {}, {}, {}
        self.selected = 'camera_front5'
        self.embedded = embedded
        self.loader = None
        self.max_fps = max(10, min(60, int(max_fps)))
        self.actual_fps = 0.
        self.frames = 0
        self.frame_epoch = time.monotonic()
        self.pose_epoch = self.frame_epoch
        self.last_receive = None
        self.last_map = None
        self.paused = False
        self._build(max_points)
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.timeout.connect(self._tick)
        self.timer.start(round(1000 / self.max_fps))
        if stream_dir is not None:
            self.set_stream(stream_dir)

    def _build(self, max_points):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(*((12, 8, 12, 8) if self.embedded else (22, 15, 22, 13)))
        root.setSpacing(6 if self.embedded else 10)
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QVBoxLayout()
        title.addWidget(label('온라인 캘리브레이션', 'Title'))
        title.addWidget(label('센서의 위치가 맞춰지는 과정을, 한눈에.', 'Subtle'))
        if not self.embedded:
            header.addLayout(title)
        header.addStretch()
        self.mode_label = label('스트림 연결 대기', 'Badge')
        self.mode_label.setFixedHeight(32)
        header.addWidget(self.mode_label)
        self.window_label = label('윈도 — / —')
        self.eta_label = label('남은 시간 —')
        header.addSpacing(15)
        header.addWidget(self.window_label)
        header.addSpacing(15)
        header.addWidget(self.eta_label)
        self.pause_button = QtWidgets.QPushButton('화면 일시정지')
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self._pause)
        header.addSpacing(10)
        header.addWidget(self.pause_button)
        root.addLayout(header)
        self.stages = StageBar()
        root.addWidget(self.stages)
        self.detail_label = label('', 'Subtle')
        self.detail_label.setWordWrap(True)
        root.addWidget(self.detail_label)
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(12)
        reset = QtWidgets.QPushButton('시점 복원')
        ghost = QtWidgets.QCheckBox('이전 추정 잔상')
        ghost.setChecked(True)
        self.rig_sensor_label = label('센서 구성 대기', 'Subtle')
        rig_card, rig_layout = card('01  센서 리그', self.rig_sensor_label, [ghost, reset])
        self.rig = RigScene()
        if self.embedded:
            self.rig.setMinimumSize(280, 190)
        self.rig.frameSwapped.connect(self._rendered)
        reset.clicked.connect(self.rig.reset_view)
        ghost.toggled.connect(lambda checked: setattr(self.rig, 'show_ghosts', checked))
        rig_layout.addWidget(self.rig, 1)
        rig_layout.addWidget(label('<span style="color:#49dfd6">●</span> 게이트 통과　'
                                  '<span style="color:#8496af">●</span> 미판정　'
                                  '<span style="color:#ff737c">●</span> 게이트 실패　'
                                  '<span style="color:#b0a8ff">●</span> 선택', 'Subtle'))
        rig_layout.addWidget(label('차량 좌표 · m  |  차량 외형은 개략도  |  드래그 회전 · 휠 확대', 'Subtle'))
        top.addWidget(rig_card, 3)
        self.camera_combo = QtWidgets.QComboBox()
        self.camera_combo.setMinimumWidth(142)
        self.camera_combo.currentIndexChanged.connect(self._combo_changed)
        image_card, image_layout = card('02  영상 ↔ LiDAR', controls=[self.camera_combo])
        self.image_panel = ImagePanel()
        if self.embedded:
            self.image_panel.setMinimumSize(300, 140)
        image_layout.addWidget(self.image_panel, 1)
        image_toggles = QtWidgets.QHBoxLayout()
        for title, attr, checked in [('깊이 투영', 'show_points', True), ('관측 특징', 'show_tracks', True),
                                      ('초기 추정 비교', 'show_initial', False)]:
            toggle = QtWidgets.QCheckBox(title)
            toggle.setChecked(checked)
            toggle.toggled.connect(lambda checked, key=attr: setattr(self.image_panel, key, checked))
            image_toggles.addWidget(toggle)
        image_layout.addLayout(image_toggles)
        self.metrics = MetricStrip()
        image_layout.addWidget(self.metrics)
        self.solver_label = label('', 'Subtle')
        self.solver_label.setWordWrap(True)
        image_layout.addWidget(self.solver_label)
        image_layout.addWidget(label('깊이점과 관측 특징을 함께 표시 · 점별 정답 대응을 뜻하지 않음', 'Subtle'))
        top.addWidget(image_card, 2)
        root.addLayout(top, 3)
        bottom = QtWidgets.QHBoxLayout()
        bottom.setSpacing(12)
        map_card, map_layout = card('03  주행 지도', 'LiDAR 누적점 · 궤적')
        self.map = MapScene(max_points=max_points)
        map_layout.addWidget(self.map, 1)
        self.map_label = label('지도 스트림 대기 중', 'Subtle')
        map_layout.addWidget(self.map_label)
        bottom.addWidget(map_card, 2)
        sensors_card, sensors_layout = card('04  센서별 수렴', '1σ · 재투영 오차', [])
        self.sensors = SensorGrid()
        if self.embedded:
            self.sensors.setMinimumSize(500, 200)
        self.sensors.selected.connect(self.select_camera)
        sensors_layout.addWidget(self.sensors, 1)
        self.gate_label = label('검증 대기 — 아직 판정하지 않았습니다', 'Subtle')
        self.gate_label.setWordWrap(True)
        sensors_layout.addWidget(self.gate_label)
        bottom.addWidget(sensors_card, 3)
        root.addLayout(bottom, 2)
        footer = QtWidgets.QHBoxLayout()
        self.status_label = label('작은 스냅샷을 읽어 표시합니다. 캘리브레이션 연산과 독립적으로 실행됩니다.', 'Subtle')
        footer.addWidget(self.status_label, 1)
        self.fps_label = label('— FPS', 'Subtle')
        footer.addWidget(self.fps_label)
        root.addLayout(footer)
        if self.embedded:
            for frame in (rig_card, image_card, map_card, sensors_card):
                frame.layout().setContentsMargins(10, 7, 10, 7)
                frame.layout().setSpacing(4)

    def reset_source(self):
        if self.loader:
            self.loader.close()
            self.loader = None
        self.manifest, self.event, self.decoded = {}, {}, {}
        self.calibrations.clear()
        self.initial_calibrations.clear()
        self.last_receive = None
        self.last_map = None
        self.current.clear()
        self.initial.clear()
        self.targets.clear()
        self.origins.clear()
        self.rig.history.clear()
        self.rig.last_seq = None
        self.rig.clear_poses()
        self.sensors.histories.clear()
        self.sensors.ingest({})
        for values in self.metrics.histories.values():
            values.clear()
        self.metrics.ingest(self.selected, {})
        self.image_panel.set_frame(None, {}, {}, {})
        self.map.set_map({}, None)
        self.map_label.setText('지도 스트림 대기 중')
        self.gate_label.setText('검증 대기')
        self.detail_label.clear()
        self.solver_label.clear()
        self.rig_sensor_label.setText('센서 구성 대기')
        self.camera_combo.clear()
        self.stages.progress = 0
        self.stages.stage = ''
        self.stages.update()
        self.window_label.setText('윈도 — / —')
        self.eta_label.setText('남은 시간 —')
        self.mode_label.setText('작업을 고르세요')
        self.pause_button.setChecked(False)

    def set_stream(self, path):
        self.reset_source()
        self.loader = StreamLoader(path)
        self.loader.select_camera(self.selected)
        self.loader.active = self.isVisible()

    def set_job(self, job, progress=None):
        self.reset_source()
        self.loader = StreamLoader(Path(job['workdir']) / 'viz', job=job, progress=progress)
        self.loader.select_camera(self.selected)
        self.loader.update_job(job, progress)
        self.loader.active = self.isVisible()

    def update_job(self, job, progress=None):
        if self.loader:
            self.loader.update_job(job, progress)

    def _pause(self, checked):
        self.paused = checked
        self.pause_button.setText('화면 다시 재생' if checked else '화면 일시정지')

    def _rendered(self):
        self.frames += 1

    def _combo_changed(self, index):
        name = self.camera_combo.itemData(index)
        if name:
            self.select_camera(name)

    def select_camera(self, name):
        if self.manifest.get('cameras') and name not in self.manifest['cameras']:
            return False
        self.selected = name
        if self.loader:
            self.loader.select_camera(name)
        self.sensors.active = name
        self.sensors.update()
        index = self.camera_combo.findData(name)
        self.camera_combo.blockSignals(True)
        self.camera_combo.setCurrentIndex(index)
        self.camera_combo.blockSignals(False)
        self._set_image()
        self.metrics.ingest(name, self.event.get('cameras', {}).get(name, {}))
        self._set_solver_metrics()
        return True

    def _set_solver_metrics(self):
        state = self.event.get('cameras', {}).get(self.selected, {})
        calibration = self.calibrations.get(self.selected, {})
        context = []
        for key, title in (('solve_name', 'solve'), ('solver_pass', 'pass'), ('purpose', '목적')):
            if state.get(key) is not None:
                context.append(f'{title} {state[key]}')
        windows = window_context(state.get('windows'))
        if windows:
            context.append(windows)
        if state.get('iteration') is not None:
            total = state.get('total_iterations')
            context.append(f"반복 {state['iteration']} / {total if total is not None else '—'}")
        parts = []
        if state.get('cost') is not None:
            parts.append(f"비용 {state['cost']:.5g}")
        if state.get('time_offset_s') is not None:
            parts.append(f"시간 offset {state['time_offset_s'] * 1000:+.3f} ms")
        row_time = state.get('row_readout_s', state.get('rs_s'))
        if row_time is not None:
            parts.append(f'행 readout {row_time * 1e6:.2f} µs')
        if 'K' in calibration:
            parts.append(f"fx/fy {calibration['K'][0][0]:.2f} / {calibration['K'][1][1]:.2f} px")
        context_line = '선택 센서 · ' + ' · '.join(context) if context else ''
        self.solver_label.setText('\n'.join(line for line in (context_line, ' · '.join(parts)) if line))
        self.solver_label.setToolTip(str(state.get('cost_source') or state.get('metric_source') or ''))

    def _set_image(self):
        matching = self.decoded.get('matching', {}).get(self.selected)
        calibration = self.calibrations.get(self.selected)
        if matching and calibration:
            self.image_panel.set_frame(matching['image'], matching['points'],
                                       calibration, matching['metadata'],
                                       self.initial_calibrations.get(self.selected))
        else:
            self.image_panel.set_frame(None, {}, {}, {})

    def apply_snapshot(self, manifest, event, decoded):
        """Apply already-decoded arrays on the GUI thread (also useful for tests)."""
        first = not self.event
        if self.event and (event['seq'] < self.event['seq']
                           or manifest.get('run_id') != self.manifest.get('run_id')
                           or event.get('run_id') != self.event.get('run_id')):
            self.rig.history.clear()
            self.sensors.histories.clear()
            self.initial.clear()
            self.initial_calibrations.clear()
            self.last_map = None
            self.current.clear()
            self.rig.last_seq = None
            for values in self.metrics.histories.values():
                values.clear()
        if first:
            self.frame_epoch = time.monotonic()
            self.frames = 0
        cameras_changed = self.event.get('cameras') != event.get('cameras')
        self.manifest, self.event, self.decoded = manifest, event, decoded
        self.calibrations = snapshot_calibrations(manifest, event)
        for name, calibration in self.calibrations.items():
            self.initial_calibrations.setdefault(name, deepcopy(calibration))
        self.last_receive = time.monotonic()
        self.pose_epoch = self.last_receive
        cameras = event.get('cameras', {})
        self.origins = {name: self.current.get(name, np.array(c['T_cam_lidar'], float))
                        for name, c in cameras.items()}
        self.targets = {name: np.array(c['T_cam_lidar'], float) for name, c in cameras.items()}
        for name, pose in self.targets.items():
            self.initial.setdefault(name, pose.copy())
        self.stages.stage = {'lo': 'lidar_odometry', 'axes': 'lidar_odometry',
                             'rgb': 'rgb_ba', 'rgb_tracks': 'rgb_ba', 'rgb_solve': 'rgb_ba',
                             'thermal_tracks': 'thermal', 'thermal_solve': 'thermal',
                             'outputs': 'validation'}.get(event.get('stage'), event.get('stage'))
        self.stages.progress = float(np.clip(event.get('progress', 0), 0, 1))
        self.stages.update()
        self.window_label.setText(f"윈도 {event.get('window', '—')} / {event.get('total_windows', '—')}")
        eta = event.get('eta_s')
        self.eta_label.setText('남은 시간 —' if eta is None else f'남은 시간 {int(eta)//60:02d}:{int(eta)%60:02d}')
        provenance = event.get('provenance', {})
        replay = (manifest.get('mode') == 'replay' or provenance.get('synthetic', False)
                  or provenance.get('synthetic_intermediate', False))
        # Explicitly label a replay even when a producer uses a more detailed provenance schema.
        replay = replay or bool(manifest.get('replay')) or 'replay' in str(event.get('provenance', {})).lower()
        self.mode_label.setText('실제 데이터 · 합성 수렴 리플레이' if replay else
                                ('실제 보정 작업' if manifest.get('mode') == 'job' else '● 실시간 스트림'))
        warnings = event.get('warnings', [])
        detail = event.get('stage_label', '')
        context = []
        windows = window_context(event.get('windows')) if event.get('stage') not in ('lo', 'lidar_odometry') else ''
        for key, title in (('bag_id', 'bag'), ('window_id', '구간'), ('solver_pass', 'pass'),
                           ('solve_name', 'solve'), ('purpose', '목적')):
            if key == 'window_id' and windows:
                continue
            if event.get(key) is not None:
                context.append(f'{title} {event[key]}')
        if windows:
            context.append(windows)
        if event.get('iteration') is not None:
            total = event.get('total_iterations')
            context.append(f"반복 {event['iteration']} / {total if total is not None else '—'}")
        if context:
            detail = ' · '.join(([detail] if detail else []) + context)
        if warnings:
            detail += f' · 기록된 경고 {len(warnings)}개 · ' + str(warnings[-1])[:100] + ('…' if len(str(warnings[-1])) > 100 else '')
        self.detail_label.setToolTip('\n'.join(str(w) for w in warnings))
        self.detail_label.setText(detail)
        self.detail_label.setVisible(bool(detail))
        if cameras_changed:
            self.sensors.ingest(cameras)
            self.metrics.ingest(self.selected, cameras.get(self.selected, {}))
        gate = event.get('gate', {})
        status = gate.get('status', 'pending')
        title = {'pass': '종합 검증 통과', 'fail': '종합 검증 실패', 'pending': '검증 대기'}.get(status, str(status))
        warning = gate.get('warning', '')
        self.gate_label.setToolTip(warning)
        warning = warning[:110] + ('…' if len(warning) > 110 else '')
        self.gate_label.setText(f'{title}  ·  {warning}' if warning else title)
        gate_color = '#ff737c' if status == 'fail' else ('#ffc06d' if warning else
                     ('#58d6c9' if status == 'pass' else '#8496af'))
        self.gate_label.setStyleSheet(f'color: {gate_color};')
        names = list(manifest.get('cameras', {}))
        sensor_types = [camera.get('sensor', 'thermal' if name.startswith('thermal_') else 'rgb')
                        for name, camera in manifest.get('cameras', {}).items()]
        self.rig_sensor_label.setText((f"RGB {sensor_types.count('rgb')} · 열화상 {sensor_types.count('thermal')}"
                                       f" · 카메라 {len(names)}대 · Ouster") if names else '센서 구성 대기')
        if names != [self.camera_combo.itemData(i) for i in range(self.camera_combo.count())]:
            self.camera_combo.blockSignals(True)
            self.camera_combo.clear()
            for name in names:
                self.camera_combo.addItem(short_name(name), name)
            self.camera_combo.setCurrentIndex(self.camera_combo.findData(self.selected))
            self.camera_combo.blockSignals(False)
        if names and self.selected not in names:
            self.select_camera('camera_front5' if 'camera_front5' in names else names[0])
        self._set_image()
        self._set_solver_metrics()
        map_path = decoded.get('map_asset', event.get('assets', {}).get('map'))
        frame = event.get('map_frame', manifest.get('map_frame', '미지정 지도 좌표'))
        if frame != self.map.frame_id and decoded.get('map') is None:
            self.map.set_map({}, frame)
            self.map_label.setText('지도 스트림 대기 중')
        if (map_path != self.last_map or frame != self.map.frame_id) and decoded.get('map') is not None:
            self.map.set_map(decoded['map'], frame)
            self.last_map = map_path
            map_note = event.get('provenance', {}).get('map_label_ko', frame)
            self.map_label.setText(f'{self.map.point_count:,} 표시점 · {map_note}')
        self.snapshot_applied.emit(int(event['seq']))

    def _tick(self):
        if not self.isVisible():
            return
        now = time.monotonic()
        if self.loader and not self.paused:
            latest, error = self.loader.take()
            if latest:
                self.apply_snapshot(*latest)
            if error:
                self.status_label.setText(f'스트림 확인: {error[:140]}')
        if not self.paused and self.targets:
            alpha = min(1., (now - self.pose_epoch) / .48) if self.event.get('interpolate', True) else 1.
            alpha = alpha * alpha * (3 - 2 * alpha)
            self.current = {name: interpolate_pose(self.origins[name], target, alpha)
                            for name, target in self.targets.items()}
            poses = {name: vehicle_from_camera(pose, self.manifest.get('R_lidar_V'))
                     for name, pose in self.current.items()}
            states = {name: dict(self.calibrations.get(name, {}), **state)
                      for name, state in self.event['cameras'].items()}
            self.rig.update_poses(poses, states, self.selected, self.event['seq'])
            if self.selected in self.current:
                self.image_panel.project(self.current[self.selected], self.initial.get(self.selected))
        elapsed = now - self.frame_epoch
        if elapsed >= 3:
            self.actual_fps = self.frames / elapsed
            self.fps_label.setText(f'{self.actual_fps:.0f} FPS · 최대 {self.max_fps} · 지도 {self.map.point_count:,}점')
            # Slow hardware automatically reduces work; this never touches the producer.
            if self.actual_fps < self.max_fps * .65 and self.max_fps > 15 and not self.paused and self.isVisible():
                self.max_fps = max(15, self.max_fps - 5)
                self.map.max_points = max(12000, int(self.map.max_points * .75))
                self.timer.setInterval(round(1000 / self.max_fps))
            self.frames, self.frame_epoch = 0, now
        if (self.last_receive and now - self.last_receive > 5 and self.stages.progress < 1
                and self.manifest.get('mode') != 'job'):
            self.status_label.setText('새 스냅샷 대기 중 · 마지막 추정치를 유지합니다')
        elif self.event and (not self.loader or not self.loader.error):
            self.status_label.setText(self.event.get('status_text') or ('합성 중간 포즈·1σ · 실측 최종값 / 관측 특징은 독립 표시' if
                                      '리플레이' in self.mode_label.text() else '스트림 연결됨 · 최신 스냅샷 표시'))

    def hideEvent(self, event):
        if self.loader:
            self.loader.active = False
        super().hideEvent(event)

    def showEvent(self, event):
        if self.loader:
            self.loader.active = True
        super().showEvent(event)

    def shutdown(self):
        self.timer.stop()
        if self.loader:
            self.loader.close()
            self.loader = None

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)


class CalibrationVizWindow(QtWidgets.QMainWindow):
    def __init__(self, stream_dir=None, **kwargs):
        super().__init__()
        self.setWindowTitle('온라인 캘리브레이션 · 3D 수렴 뷰어')
        self.viewer = CalibrationVizWidget(stream_dir=stream_dir, **kwargs)
        self.setCentralWidget(self.viewer)
        self.resize(1600, 1000)

    def closeEvent(self, event):
        self.viewer.shutdown()
        super().closeEvent(event)
