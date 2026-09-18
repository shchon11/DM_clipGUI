#!/usr/bin/env python3
# sensor_stage.py — "센서 기동" 탭.
#
#   감지 (어떤 센서가 어떤 IP로 몇 대)  ->  설정  ->  기동  ->  [녹화] 탭으로
#
# 이 탭이 하는 일은 세 모듈에 나눠져 있다. 여기는 그걸 화면에 붙이는 층이다:
#   sensor_discovery : 네트워크에서 센서 찾기
#   sensor_config    : 폼 값 <-> params 파일 / 런치 인자
#   sensor_launcher  : 센서군 프로세스
#
# 설정 값은 clip_gui 의 cfg["sensors"] 에 그대로 저장된다 (last_session.yaml).

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QPushButton,
    QSpinBox, QTabWidget, QTableWidget, QTableWidgetItem, QTextEdit,
    QVBoxLayout, QWidget)

import sensor_config
import sensor_discovery
import sensor_launcher
from sensor_discovery import MISSING, NEW, OK, SUBNET
from sensor_launcher import FAILED, RUNNING, STARTING, STATE_LABEL, STOPPED

STATE_COLOR = {OK: "#2e7d32", NEW: "#1565c0", MISSING: "#c62828", SUBNET: "#ef6c00"}
STATE_TEXT = {OK: "정상", NEW: "설정에 없음", MISSING: "안 보임", SUBNET: "서브넷 불일치"}
RUN_COLOR = {STOPPED: "#666666", STARTING: "#b58900", RUNNING: "#2e7d32", FAILED: "#c62828"}


class DiscoveryWorker(QThread):
    """GVCP 브로드캐스트와 TCP 프로브는 초 단위로 블록하므로 별도 스레드."""

    sig_done = pyqtSignal(dict)

    def __init__(self, registry):
        super().__init__()
        self.registry = registry

    def run(self):
        try:
            self.sig_done.emit(sensor_discovery.discover_all(self.registry))
        except Exception:                                          # noqa: BLE001
            self.sig_done.emit({})


class FieldEditor(QWidget):
    """설정 필드 하나 = [지정 체크박스] + 편집 위젯 + 단위.

    optional 필드는 체크를 풀면 값이 None 이 되고, 생성 params 에서 키 자체가 빠진다
    (= 원본 파일이 그 키를 안 쓰는 상태 그대로).
    """

    sig_changed = pyqtSignal()

    def __init__(self, field, value, parent=None):
        super().__init__(parent)
        self.field = field
        self.optional = bool(field.get("optional"))
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)

        self.use = None
        if self.optional:
            self.use = QCheckBox("지정")
            self.use.setChecked(value is not None)
            self.use.toggled.connect(self._on_toggle)
            row.addWidget(self.use)

        self.editor = self._make_editor(field, value)
        row.addWidget(self.editor, 1)
        if field.get("unit"):
            row.addWidget(QLabel(field["unit"]))
        if field.get("help"):
            self.setToolTip(field["help"])
        self._on_toggle()

    def _make_editor(self, field, value):
        kind = field.get("type", "string")
        if kind == "bool":
            widget = QCheckBox()
            widget.setChecked(bool(value))
            widget.toggled.connect(self.sig_changed)
            return widget
        if kind == "enum":
            widget = QComboBox()
            choices = [str(c) for c in field.get("choices") or []]
            widget.addItems(choices)
            text = "" if value is None else str(value)
            if text not in choices:
                widget.addItem(text)
            widget.setCurrentText(text)
            widget.currentTextChanged.connect(self.sig_changed)
            return widget
        if kind in ("float", "int"):
            widget = QDoubleSpinBox() if kind == "float" else QSpinBox()
            widget.setMinimum(field.get("min", 0))
            widget.setMaximum(field.get("max", 1_000_000))
            widget.setSingleStep(field.get("step", 1))
            if kind == "float":
                widget.setDecimals(2)
            try:
                widget.setValue(float(value) if kind == "float" else int(value))
            except (TypeError, ValueError):
                pass
            widget.valueChanged.connect(self.sig_changed)
            return widget
        widget = QLineEdit("" if value is None else str(value))
        widget.textChanged.connect(self.sig_changed)
        return widget

    def _on_toggle(self, *_):
        if self.use is not None:
            self.editor.setEnabled(self.use.isChecked())
        self.sig_changed.emit()

    def set_enabled_by_dependency(self, enabled):
        """enabled_when 조건이 깨지면 회색 처리 (값은 유지 — 되돌리면 그대로 살아난다)."""
        self.setEnabled(enabled)

    def value(self):
        if self.use is not None and not self.use.isChecked():
            return None
        kind = self.field.get("type", "string")
        if kind == "bool":
            return self.editor.isChecked()
        if kind == "enum":
            return self.editor.currentText()
        if kind == "float":
            return float(self.editor.value())
        if kind == "int":
            return int(self.editor.value())
        return self.editor.text()


class GroupPanel(QGroupBox):
    """센서군 하나 — 감지 표 + 일괄 설정 폼 + 기동 버튼."""

    sig_changed = pyqtSignal()
    sig_start = pyqtSignal(str)
    sig_stop = pyqtSignal(str)

    def __init__(self, group, overrides, parent=None):
        super().__init__(group["label"], parent)
        self.group = group
        self.key = group["key"]
        self.overrides = overrides
        self.editors = {}          # (subset, key) -> FieldEditor
        self.devices = []          # 마지막 감지 결과
        self._base = sensor_config.base_values(group)
        self._labels = {}          # (subset, key) -> QLabel
        self._build()

    # --- UI ---

    def _build(self):
        outer = QVBoxLayout(self)

        head = QHBoxLayout()
        self.lbl_state = QLabel("정지")
        self.lbl_state.setMinimumWidth(150)
        self.lbl_summary = QLabel("감지 전")
        self.lbl_summary.setStyleSheet("color:#666;")
        self.btn_start = QPushButton("기동")
        self.btn_stop = QPushButton("중지")
        self.btn_stop.setEnabled(False)
        self.btn_start.clicked.connect(lambda: self.sig_start.emit(self.key))
        self.btn_stop.clicked.connect(lambda: self.sig_stop.emit(self.key))
        head.addWidget(self.lbl_state)
        head.addWidget(self.lbl_summary, 1)
        head.addWidget(self.btn_start)
        head.addWidget(self.btn_stop)
        outer.addLayout(head)

        gaps = sensor_config.missing_paths(self.group)
        if gaps:
            warn = QLabel("경로 없음: " + ", ".join(gaps))
            warn.setWordWrap(True)
            warn.setStyleSheet("color:#c62828;")
            outer.addWidget(warn)
            self.btn_start.setEnabled(False)

        # subset 사용 체크박스 (가시광/열화상)
        subs = sensor_config.subsets_of(self.group)
        self.sub_checks = {}
        if subs:
            row = QHBoxLayout()
            row.addWidget(QLabel("띄울 대상:"))
            enabled = self.overrides.setdefault("subsets", {})
            for sub in subs:
                box = QCheckBox(sub["label"])
                box.setChecked(bool(enabled.get(sub["key"], True)))
                box.toggled.connect(self._on_edit)
                self.sub_checks[sub["key"]] = box
                row.addWidget(box)
            row.addStretch()
            outer.addLayout(row)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["종류", "IP", "시리얼", "모델", "NIC", "상태"])
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setMaximumHeight(150)
        outer.addWidget(self.table)

        for subset, title in self._form_sections():
            outer.addWidget(self._make_form(subset, title))

    def _form_sections(self):
        """[(subset, 제목)] — 폼을 나눌 단위."""
        sections = []
        for sub in sensor_config.subsets_of(self.group):
            if sensor_config.fields_for(self.group, sub["key"]):
                sections.append((sub["key"], f"{sub['label']} 일괄 설정"))
        if sensor_config.fields_for(self.group, sensor_config.NO_SUBSET):
            sections.append((sensor_config.NO_SUBSET, "설정"))
        if sensor_config.fields_for(self.group, "launch"):
            sections.append(("launch", "런치 옵션"))
        return sections

    def _make_form(self, subset, title):
        box = QGroupBox(title)
        if subset not in ("launch", sensor_config.NO_SUBSET):
            box.setToolTip("이 값은 해당 센서군의 모든 카메라에 똑같이 적용됩니다.")
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignRight)
        for field in sensor_config.fields_for(self.group, subset):
            value = sensor_config.effective_value(self.group, field, self.overrides, self._base)
            editor = FieldEditor(field, value)
            editor.sig_changed.connect(self._on_edit)
            label = QLabel(field.get("label") or field["key"])
            if field.get("danger"):
                label.setStyleSheet("color:#c62828;")
            if field.get("help"):
                label.setToolTip(field["help"])
            self.editors[(subset, field["key"])] = editor
            self._labels[(subset, field["key"])] = label
            form.addRow(label, editor)
        return box

    # --- 값 수집 ---

    def _on_edit(self, *_):
        self.collect()
        self._refresh_marks()
        self.sig_changed.emit()

    def collect(self):
        """폼 -> self.overrides (clip_gui 의 cfg["sensors"][key] 그 자체)."""
        if self.sub_checks:
            self.overrides["subsets"] = {k: b.isChecked() for k, b in self.sub_checks.items()}
        params = self.overrides.setdefault("params", {})
        launch = {}
        for (subset, key), editor in self.editors.items():
            value = editor.value()
            if subset == "launch":
                launch[key] = value
            else:
                params.setdefault(subset, {})[key] = value
        self.overrides["launch_args"] = launch
        return self.overrides

    def _refresh_marks(self):
        """원본과 다른 값은 굵게 + 원본값 툴팁."""
        changed = set(sensor_config.changed_keys(self.group, self.overrides, self._base))
        for (subset, key), label in self._labels.items():
            field = next((f for f in sensor_config.fields_for(self.group, subset)
                          if f["key"] == key), None)
            font = label.font()
            font.setBold(key in changed)
            label.setFont(font)
            if field is not None and key in changed:
                original = sensor_config.base_value(self.group, field, self._base)
                label.setToolTip(f"원본: {original}\n{field.get('help', '')}".strip())

        # enabled_when: 다른 키 값에 따라 회색 처리 (노출 auto=Continuous 면 노출시간 무의미)
        for (subset, key), editor in self.editors.items():
            condition = editor.field.get("enabled_when") or {}
            if not condition:
                continue
            ok = True
            for dep_key, dep_value in condition.items():
                dep = self.editors.get((subset, dep_key))
                if dep is not None and dep.value() != dep_value:
                    ok = False
            editor.set_enabled_by_dependency(ok)

    def group_subset_label(self, subset_key):
        for sub in sensor_config.subsets_of(self.group):
            if sub["key"] == subset_key:
                return sub["label"]
        return subset_key

    # --- 표시 갱신 ---

    def expectations(self):
        """[(토픽 정규식, 최소 대수)] — 감지된 대수만큼 떠야 '기동 완료'로 본다.

        가시광 8대를 켜 두고 기동했는데 3대만 올라왔다면 그건 완료가 아니다.
        """
        live = [d for d in self.devices if d["state"] in (OK, NEW, SUBNET)]
        subs = sensor_config.subsets_of(self.group)
        if not subs:
            pattern = self.group.get("topic_regex")
            return [(pattern, max(1, len(live)))] if pattern else []

        enabled = self.overrides.get("subsets") or {}
        out = []
        for sub in subs:
            if not sub.get("topic_regex") or not enabled.get(sub["key"], True):
                continue
            out.append((sub["topic_regex"],
                        max(1, sum(1 for d in live if d["subset"] == sub["key"]))))
        return out

    def enabled_without_devices(self):
        """켜 두었는데 한 대도 안 잡힌 대상 — 기동 전에 경고할 거리."""
        live = [d for d in self.devices if d["state"] in (OK, NEW, SUBNET)]
        enabled = self.overrides.get("subsets") or {}
        return [sub["label"] for sub in sensor_config.subsets_of(self.group)
                if enabled.get(sub["key"], True)
                and not any(d["subset"] == sub["key"] for d in live)]

    def set_devices(self, devices):
        self.devices = list(devices)
        self.table.setRowCount(len(devices))
        for row, dev in enumerate(devices):
            cells = [dev["subset_label"], dev["ip"], dev["identity"],
                     dev["model"], dev["nic"], STATE_TEXT.get(dev["state"], dev["state"])]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col == 5:
                    item.setForeground(Qt.black)
                    item.setToolTip(dev["note"])
                    font = item.font()
                    font.setBold(dev["state"] != OK)
                    item.setFont(font)
                self.table.setItem(row, col, item)
            if dev["note"]:
                self.table.item(row, 0).setToolTip(dev["note"])
        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)

        # 가시광 Blackfly 와 열화상 A70 은 따로 센다. 합치면 어느 쪽이 빠졌는지 안 보인다.
        parts, all_ok = [], True
        for label, ok, total, note in sensor_discovery.summarize_by_subset(devices):
            parts.append(f"{label} {ok}/{total}" + (f" ({note})" if note else ""))
            if ok != total:
                all_ok = False
        if not parts:
            parts, all_ok = ["감지된 장비 없음"], False
        color = STATE_COLOR[OK] if all_ok else STATE_COLOR[MISSING]
        self.lbl_summary.setText("  ·  ".join(parts))
        self.lbl_summary.setStyleSheet(f"color:{color};")
        self.lbl_summary.setToolTip("\n".join(parts))

    def set_run_state(self, state):
        # 어떤 대상을 띄운 상태인지까지 보여준다 — 한 프로세스가 가시광/열화상을
        # 같이 띄우므로 상태만으로는 지금 무엇이 도는지 알 수 없다.
        text = STATE_LABEL.get(state, state)
        if state in (STARTING, RUNNING) and self.sub_checks:
            active = [self.group_subset_label(k)
                      for k, box in self.sub_checks.items() if box.isChecked()]
            if active:
                text += " (" + ", ".join(active) + ")"
        self.lbl_state.setText(text)
        self.lbl_state.setStyleSheet(f"color:{RUN_COLOR.get(state, '#666')}; font-weight:bold;")
        active = state in (STARTING, RUNNING)
        self.btn_start.setEnabled(not active and not sensor_config.missing_paths(self.group))
        self.btn_stop.setEnabled(active)


class SensorStageWidget(QWidget):
    """센서 기동 탭 전체."""

    sig_log = pyqtSignal(str, str)         # level, text
    sig_ready = pyqtSignal()               # 기동한 센서군이 전부 RUNNING
    sig_state = pyqtSignal()               # 상태 요약이 바뀜

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.registry = sensor_discovery.load_registry()
        self.supervisor = sensor_launcher.SensorSupervisor(self.registry, self)
        self.supervisor.sig_state.connect(self._on_run_state)
        self.supervisor.sig_output.connect(self._on_output)
        self.panels = {}
        self.logs = {}
        self._disc = None
        self._announced_ready = False
        self._build()
        self.refresh_discovery()

    # --- UI ---

    def _build(self):
        outer = QVBoxLayout(self)

        top = QHBoxLayout()
        self.btn_scan = QPushButton("센서 다시 감지")
        self.btn_scan.clicked.connect(self.refresh_discovery)
        self.lbl_scan = QLabel("")
        self.lbl_scan.setStyleSheet("color:#666;")
        self.btn_start_all = QPushButton("전체 기동")
        self.btn_start_all.clicked.connect(self._start_all)
        self.btn_stop_all = QPushButton("전체 중지")
        self.btn_stop_all.clicked.connect(self.stop_all)
        top.addWidget(self.btn_scan)
        top.addWidget(self.lbl_scan, 1)
        top.addWidget(self.btn_start_all)
        top.addWidget(self.btn_stop_all)
        outer.addLayout(top)

        if not self.registry.get("groups"):
            warn = QLabel("sensors.yaml 을 찾지 못했습니다 — 센서 기동을 쓸 수 없습니다.\n"
                          "colcon build 후 다시 실행하거나 config/sensors.yaml 을 확인하세요.")
            warn.setStyleSheet("color:#c62828;")
            outer.addWidget(warn)
            outer.addStretch(1)
            return

        sensors_cfg = self.cfg.setdefault("sensors", {})
        grid = QGridLayout()
        for index, group in enumerate(self.registry["groups"]):
            overrides = sensors_cfg.setdefault(group["key"], {})
            panel = GroupPanel(group, overrides)
            panel.sig_start.connect(self.start_group)
            panel.sig_stop.connect(self.stop_group)
            panel.sig_changed.connect(self.sig_state)
            self.panels[group["key"]] = panel
            grid.addWidget(panel, index // 2, index % 2)
        outer.addLayout(grid)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        outer.addWidget(line)

        self.log_tabs = QTabWidget()
        for group in self.registry["groups"]:
            view = QTextEdit(readOnly=True)
            view.setFont(QFont("Monospace", 9))
            view.document().setMaximumBlockCount(2000)
            self.logs[group["key"]] = view
            self.log_tabs.addTab(view, group["label"])
        outer.addWidget(self.log_tabs, 1)

    # --- 감지 ---

    def refresh_discovery(self):
        if self._disc and self._disc.isRunning():
            return
        self.btn_scan.setEnabled(False)
        self.lbl_scan.setText("감지 중… (GVCP 브로드캐스트 / TCP 프로브)")
        self._disc = DiscoveryWorker(self.registry)
        self._disc.sig_done.connect(self._on_discovered)
        self._disc.start()

    def _on_discovered(self, result):
        self.btn_scan.setEnabled(True)
        lines = []
        for key, devices in result.items():
            panel = self.panels.get(key)
            if panel:
                panel.set_devices(devices)
            for label, ok, count, _ in sensor_discovery.summarize_by_subset(devices):
                lines.append(f"{label} {ok}/{count}")
        text = "  ·  ".join(lines) if lines else "감지된 장비 없음"
        self.lbl_scan.setText("감지 완료 — " + text)
        self.sig_log.emit("GUI", "센서 감지: " + text)
        self.sig_state.emit()

    # --- 기동 ---

    def start_group(self, key):
        panel = self.panels.get(key)
        if not panel:
            return
        overrides = panel.collect()
        self._announced_ready = False
        for label in panel.enabled_without_devices():
            self.sig_log.emit(
                "WARN", f"{label}: 감지된 장비가 없는데 기동 대상으로 켜져 있습니다 "
                        "— 기동은 하지만 완료 판정은 실패할 수 있습니다")
        self.sig_log.emit("GUI", f"{panel.group['label']} 기동")
        self.log_tabs.setCurrentWidget(self.logs[key])
        self.supervisor.start(key, overrides, panel.expectations())

    def stop_group(self, key):
        self.supervisor.stop(key)

    def _start_all(self):
        for key in self.panels:
            if self.supervisor.state(key) not in (STARTING, RUNNING):
                self.start_group(key)

    def stop_all(self):
        self.supervisor.stop_all()

    # --- 상태 ---

    def set_topics(self, topic_names):
        """clip_gui 가 주기적으로 현재 토픽 목록을 넘겨준다 (기동 완료 판정용)."""
        self.supervisor.set_topics(topic_names)

    def _on_run_state(self, key, state):
        panel = self.panels.get(key)
        if panel:
            panel.set_run_state(state)
        label = self.registry and next(
            (g["label"] for g in self.registry["groups"] if g["key"] == key), key)
        self.sig_log.emit("ERROR" if state == FAILED else "GUI",
                          f"{label}: {STATE_LABEL.get(state, state)}")
        self.sig_state.emit()

        active = [k for k in self.panels if self.supervisor.procs[k].is_active()]
        if active and all(self.supervisor.state(k) == RUNNING for k in active):
            if not self._announced_ready:
                self._announced_ready = True
                self.sig_ready.emit()

    def _on_output(self, key, line):
        view = self.logs.get(key)
        if view:
            view.append(line)

    def has_starting(self):
        """아직 기동 중인 센서군이 있는가 (토픽 폴링을 돌릴지 판단)."""
        return any(self.supervisor.state(k) == STARTING for k in self.panels)

    def summary(self):
        return self.supervisor.summary()
