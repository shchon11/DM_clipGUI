#!/usr/bin/env python3
# sensor_stage.py — "센서 기동" 탭.
#
#   ┌ 다시 감지 ─────────────────────── 선택한 센서 기동 ─ 전체 중지 ┐
#   ├ 센서 카드 ┬ 상세 ─────────────────────────────────────────────┤
#   │ ☑ A70  2대│ 열화상 A70  [실행 중]            ▶ 기동  ■ 중지   │
#   │ ☐ BFS  0대│ ┌ 감지된 장비 ────────────────────────────────┐   │
#   │ ☐ Ouster  │ ├ 설정 (이 종류 전체에 일괄) ──────────────────┤   │
#   │ ☐ GNSS    │ └ 런치 로그 ──────────────────────────────────┘   │
#   └───────────┴───────────────────────────────────────────────────┘
#
# 칸 경계는 전부 끌어서 크기를 바꿀 수 있고 (QSplitter), 위치는 설정에 저장된다.
#
# 카드 = 센서 "종류". 가시광 Blackfly 와 열화상 A70 은 카드가 따로지만 실제 프로세스는
# multicam.launch.py 하나다 (sensors.yaml 머리말 참고) — 두 카드의 체크가
# enable_visible_cameras / enable_thermal_cameras 로 간다.
#
# 감지/설정/기동 로직은 sensor_discovery / sensor_config / sensor_launcher 에 있고,
# 여기는 그걸 화면에 붙이는 층이다.

import re
import time
from pathlib import Path

import yaml
from PyQt5.QtCore import QByteArray, QEvent, QObject, QSize, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QPixmap
from PyQt5.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout,
    QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit, QPushButton, QScrollArea,
    QSizePolicy, QSpinBox, QSplitter, QStackedWidget, QTableWidget, QTabWidget,
    QTableWidgetItem, QToolButton, QVBoxLayout, QWidget)

import net_tools
import preview_panel
import sensor_config
import sensor_discovery
import sensor_launcher
import ui_theme
from sensor_discovery import NIC, OK, SUBNET, UPDATER
from sensor_launcher import FAILED, RUNNING, STARTING, STOPPED

RUN_TEXT = {STOPPED: "정지", STARTING: "기동 중…", RUNNING: "실행 중", FAILED: "실패"}
SPLITTER_MAIN = "stage/main"
SPLITTER_DETAIL = "stage/detail"


def build_cards(registry):
    """센서 '종류' 단위 카드 목록. subset 이 있는 센서군은 subset 마다 한 장."""
    cards = []
    for group in registry.get("groups", []):
        subs = sensor_config.subsets_of(group)
        if subs:
            for sub in subs:
                cards.append({"key": f"{group['key']}:{sub['key']}", "group": group,
                              "subset": sub["key"], "label": sub["label"]})
        else:
            cards.append({"key": group["key"], "group": group,
                          "subset": None, "label": group["label"]})
    return cards


def card_devices(card, devices):
    if card["subset"] is None:
        return list(devices)
    return [d for d in devices if d["subset"] == card["subset"]]


def ip_span(devices):
    """'192.168.1.11 ~ .12' 처럼 짧게."""
    ips = [d["ip"] for d in devices if d["ip"]]
    if not ips:
        return ""
    if len(ips) == 1:
        return ips[0]
    first, last = ips[0], ips[-1]
    head, _, tail = last.rpartition(".")
    if first.rpartition(".")[0] == head:
        return f"{first} ~ .{tail}"
    return f"{first} ~ {last}"


def _same(a, b):
    """폼 값과 원본 값 비교. 빈 문자열과 None 은 같은 것으로 본다 (둘 다 '지정 안 함')."""
    if a in ("", None) and b in ("", None):
        return True
    return a == b


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


def _answers_ping(ip):
    import subprocess
    try:
        return subprocess.run(["ping", "-c", "1", "-W", "1", ip], capture_output=True,
                              timeout=3).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


class NicSetupWorker(QThread):
    """[NIC 설정] — 라이다를 꽂은 NIC 를 NetworkManager link-local 프로필로 잡는다 (nmcli, 수 초)."""

    sig_done = pyqtSignal(bool, str)

    def __init__(self, iface):
        super().__init__()
        self.iface = iface

    def run(self):
        ok, message, _addr = net_tools.nm_link_local(self.iface)
        self.sig_done.emit(ok, message)


class ForceIpWorker(QThread):
    """[IP 할당] — 카메라에 한 대씩 IP 를 주고, 다시 찾아서 정말 그 주소로 바뀌었는지 확인한다.

    한 대씩 하는 이유: A70 두 대를 동시에 바꾸다 같은 IP 에 둘 다 앉은 적이 있다.
    """

    sig_progress = pyqtSignal(str)
    sig_done = pyqtSignal(list)            # [(serial, ip, ok)]

    def __init__(self, tasks):
        super().__init__()
        self.tasks = tasks                 # [{serial, mac, nic, ip, mask, label}]

    def run(self):
        results = []
        for task in self.tasks:
            ok = False
            # 그 주소에 이미 뭔가 대답하면(감지 안 되는 다른 장비) 주지 않는다 — 충돌을 만들지 않게
            if _answers_ping(task["ip"]):
                self.sig_progress.emit(f"{task['serial']}: {task['ip']} 에 이미 다른 장비가 있어 건너뜁니다")
                results.append((task["serial"], task["ip"], False))
                continue
            for attempt in (1, 2):
                self.sig_progress.emit(f"{task['label']} {task['serial']}: {task['ip']} 할당 중 "
                                       f"({attempt}/2)…")
                net_tools.gvcp_force_ip(task["nic"], task["mac"], task["ip"], task["mask"])
                deadline = time.time() + 6.0
                while time.time() < deadline and not ok:
                    time.sleep(0.5)
                    now = [d for d in net_tools.gvcp_discover_all(task["nic"], 0.5)
                           if d["mac"] == task["mac"]]
                    ok = bool(now) and now[0]["ip"] == task["ip"]
                if ok:
                    break
            results.append((task["serial"], task["ip"], ok))
        self.sig_done.emit(results)


class LiveView(QFrame):
    """[감지된 장비] 표에서 고른 카메라 한 대의 라이브 — 어떤 그림이 어떤 카메라인지 보며 이름 짓기.

    고른 카메라 하나만 계속 구독한다 (그림 + camera_info 로 Hz). 화면에서 사라지면(다른 카드·탭)
    구독을 내린다. 디코드는 8 fps 로 제한한다.
    """

    sig_rename = pyqtSignal(str, str)      # serial, 새 이름

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("Panel")
        self.worker = None
        self.hub = None
        self.serial = None
        self.topics = {}                   # {"image": (topic, type), "info": topic}
        self.thermal = False
        self.temp_scale = None
        self.rate = preview_panel.RateTracker()
        self.drawn = None
        self.image = None
        v = QVBoxLayout(self)
        v.setContentsMargins(10, 8, 10, 10)
        v.setSpacing(6)
        head = QHBoxLayout()
        self.title = QLabel("라이브")
        self.title.setObjectName("Section")
        self.title.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.hz = QLabel("")
        self.hz.setObjectName("PillSmall")
        head.addWidget(self.title, 1)
        head.addWidget(self.hz)
        v.addLayout(head)
        self.view = QLabel("")
        self.view.setObjectName("TileImage")
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setMinimumSize(120, 60)
        self.view.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        self.view.setWordWrap(True)
        v.addWidget(self.view, 1)
        self.line = QLabel("")
        self.line.setObjectName("Hint")
        self.line.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        v.addWidget(self.line)
        row = QHBoxLayout()
        row.addWidget(QLabel("이름"))
        self.name = QLineEdit()
        self.name.setPlaceholderText("이 카메라의 이름 (ROS 네임스페이스)")
        self.name.returnPressed.connect(self._apply)
        self.btn_name = QPushButton("적용")
        self.btn_name.clicked.connect(self._apply)
        row.addWidget(self.name, 1)
        row.addWidget(self.btn_name)
        v.addLayout(row)
        self.hint = QLabel("")
        self.hint.setObjectName("Hint")
        self.hint.setWordWrap(True)
        v.addWidget(self.hint)
        self.timer = QTimer(self, interval=125, timeout=self._tick)
        self.clear_camera("표에서 카메라를 고르세요.")

    def attach(self, worker):
        self.worker = worker
        self.hub = preview_panel.PreviewHub(worker) if worker is not None else None

    def _apply(self):
        if self.serial:
            self.sig_rename.emit(self.serial, self.name.text().strip())

    def clear_camera(self, text, serial=None, next_name=""):
        """스트림 없이 이름만 고칠 수 있는 상태 (실행 전이거나 이번 기동에 없는 카메라)."""
        self._unsubscribe()
        self.serial = serial
        self.topics = {}
        self.image = None
        self.view.clear()
        self.view.setText(text)
        self.hz.setText("")
        self.hz.setStyleSheet("")
        self.line.setText("")
        self.title.setText(f"{next_name} · {serial}" if serial else "라이브")
        self.name.setText(next_name)
        self.name.setEnabled(bool(serial))
        self.btn_name.setEnabled(bool(serial))
        self.hint.setText("")

    def show_camera(self, serial, running_name, next_name, thermal, temp_scale):
        self._unsubscribe()
        self.serial, self.thermal, self.temp_scale = serial, thermal, temp_scale
        self.image, self.drawn = None, None
        self.rate = preview_panel.RateTracker()
        self.title.setText(f"{running_name} · {serial}")
        self.name.setText(next_name)
        self.name.setEnabled(True)
        self.btn_name.setEnabled(True)
        self.hint.setText("" if next_name == running_name else
                          f"지금은 /{running_name}/ 로 발행 중 — 다음 기동부터 /{next_name}/")
        self.topics = self._resolve(running_name)
        if not self.topics:
            self.view.clear()
            self.view.setText(f"/{running_name}/ 의 이미지 토픽이 아직 없습니다.")
            return
        self.view.setText("수신 대기…")
        self._subscribe()

    def _resolve(self, ns):
        if self.worker is None:
            return {}
        node = self.worker.node
        live = {n: t[0] for n, t in node.get_topic_names_and_types()
                if t and node.count_publishers(n) > 0}
        out = {}
        jpeg, raw, info = f"/{ns}/image_rgb/compressed", f"/{ns}/image_raw", f"/{ns}/camera_info"
        if live.get(jpeg) == "sensor_msgs/msg/CompressedImage":
            out["image"] = (jpeg, live[jpeg])
        elif live.get(raw) == "sensor_msgs/msg/Image":
            out["image"] = (raw, live[raw])
        else:
            return {}
        if live.get(info) == "sensor_msgs/msg/CameraInfo":
            out["info"] = info
        return out

    def _subscribe(self):
        if self.hub is None or not self.topics or not self.isVisible():
            return
        wanted = {self.topics["image"][0]: self.topics["image"][1]}
        if self.topics.get("info"):
            wanted[self.topics["info"]] = "sensor_msgs/msg/CameraInfo"
        self.hub.sync(wanted)
        self.timer.start()

    def _unsubscribe(self):
        self.timer.stop()
        if self.hub is not None and self.hub.meters:
            self.hub.clear()

    def showEvent(self, event):
        super().showEvent(event)
        self._subscribe()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._unsubscribe()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._paint()

    def _tick(self):
        now = time.monotonic()
        info = self.hub.meters.get(self.topics.get("info")) if self.topics.get("info") else None
        image = self.hub.meters.get(self.topics["image"][0]) if self.topics else None
        hz_meter = info or image
        if hz_meter is not None:
            hz = self.rate.update(hz_meter.count, now)
            age = now - hz_meter.latest_t if hz_meter.latest_t else None
            if age is not None and age > preview_panel.STALE_S:
                self.hz.setText(f"끊김 {age:.0f}s")
                self.hz.setStyleSheet(ui_theme.pill_css("failed"))
            elif hz is not None:
                self.hz.setText(f"{hz:.1f} Hz")
                self.hz.setStyleSheet(ui_theme.pill_css("running"))
        if image is None or image.latest is None or image.latest is self.drawn:
            return
        self.drawn = image.latest
        width = max(80, self.view.width())
        try:
            if self.topics["image"][1] == "sensor_msgs/msg/CompressedImage":
                qimage, line, tip = preview_panel.decode_jpeg(image.latest, width)
            else:
                qimage, line, tip = preview_panel.decode_image(
                    image.latest, width, colormap=True,
                    temp_scale=self.temp_scale if self.thermal else None)
        except Exception as exc:                                 # noqa: BLE001
            self.view.setText(f"미리보기 실패: {exc}")
            return
        self.image = qimage
        self.line.setText(line or tip)
        self._paint()

    def _paint(self):
        if self.image is None or self.image.isNull():
            return
        self.view.setPixmap(QPixmap.fromImage(self.image).scaled(
            self.view.width(), self.view.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


# 장비 행의 상태 점. 카드의 색 띠 · 점과 같은 색.
WAITING, EXCLUDED = "waiting", "excluded"
ROW_STATE_TEXT = {RUNNING: "실행 중 — 영상 수신 중", STARTING: "기동 중 — 카메라를 여는 중",
                  WAITING: "대기 — 순차 기동 차례를 기다리는 중", FAILED: "실패",
                  EXCLUDED: "이번 기동에 미포함", STOPPED: "정지"}
_DOT_ICONS = {}


def dot_icon(state, lit=True):
    """상태 점 아이콘 (16px 칸에 10px 원). 대기는 빈 주황 원, 정지·미포함은 빈 회색 원, 기동 중은 깜빡임(lit)."""
    key = (state, lit)
    if key not in _DOT_ICONS:
        from PyQt5.QtGui import QColor, QIcon, QPen
        color = {RUNNING: ui_theme.OK, STARTING: ui_theme.WARN if lit else "#fde68a",
                 WAITING: ui_theme.WARN, FAILED: ui_theme.ERR}.get(state, "#c4c9d0")
        hollow = state in (WAITING, EXCLUDED, STOPPED)
        pix = QPixmap(16, 16)
        pix.fill(Qt.transparent)
        painter = QPainter(pix)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(QColor(color), 2.0) if hollow else Qt.NoPen)
        painter.setBrush(Qt.NoBrush if hollow else QColor(color))
        painter.drawEllipse(3, 3, 10, 10)
        painter.end()
        _DOT_ICONS[key] = QIcon(pix)
    return _DOT_ICONS[key]


INVALID = object()          # 글자로 적는 칸에 아직 값이 안 되는 글자가 있을 때 (예: 숫자 칸에 "1.2.3")


class _WheelGuard(QObject):
    """설정 목록을 휠로 내리다가 지나가는 콤보/스핀 박스의 값이 바뀌지 않게.

    포커스가 없으면 휠을 무시(ignore)하고 걸러낸다 — 무시된 휠 이벤트는 부모로 올라가
    스크롤 영역이 받는다. 칸을 한 번 누르면(포커스) 휠이 그 칸에 먹는다.
    """

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Wheel and not obj.hasFocus():
            event.ignore()
            return True
        return False


_WHEEL_GUARD = None


def guard_wheel(widget):
    global _WHEEL_GUARD
    if _WHEEL_GUARD is None:
        _WHEEL_GUARD = _WheelGuard()
    widget.setFocusPolicy(Qt.StrongFocus)
    widget.installEventFilter(_WHEEL_GUARD)


class ElidedLabel(QLabel):
    """긴 키 이름은 앞을 줄인다 ('…master.line_3v3_enable_nodes') — 뒤쪽이 더 구체적이다."""

    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self._full = text
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.setMinimumWidth(60)

    def minimumSizeHint(self):
        hint = super().minimumSizeHint()
        return QSize(60, hint.height())

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setPen(self.palette().color(self.foregroundRole()))
        painter.setFont(self.font())
        rect = self.contentsRect()
        text = self.fontMetrics().elidedText(self._full, Qt.ElideLeft, rect.width())
        painter.drawText(rect, int(Qt.AlignVCenter | Qt.AlignLeft), text)


def _list_text(value):
    return yaml.safe_dump(value, default_flow_style=True, allow_unicode=True, width=10_000).strip()


class FieldEditor(QWidget):
    """설정 필드 하나 = [지정 체크박스] + 편집 위젯 + 단위.

    optional 필드는 체크를 풀면 값이 None 이 되고, 생성 params 에서 키 자체가 빠진다
    (= 원본 파일이 그 키를 안 쓰는 상태 그대로). 원본에서 주석 처리된 키가 그렇다 — 예시값을
    미리 채워 두고, '지정' 을 켜야 파일에 실린다.

    타입: bool · enum(콤보) · int/float(스핀, 주요 설정) · int_text/float_text/list/string(글자 칸,
    전체 설정). 전체 설정의 숫자를 스핀으로 안 하는 이유: 범위를 모른다 (group_mask 는 2^32-1).
    ROS 파라미터는 타입이 엄격해서 값은 원래 타입으로 되돌려 준다 (18000 -> 18000.0).
    """

    sig_changed = pyqtSignal()

    def __init__(self, field, value, parent=None):
        super().__init__(parent)
        self.field = field
        self.kind = field.get("type", "string")
        self.scale = field.get("scale", 1)      # 파일 값 = 폼 값 × scale (예: MB/s ↔ B/s)
        sample = value if value is not None else field.get("example")
        self._elem = type(sample[0]) if isinstance(sample, list) and sample else None
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)

        self.use = None
        if field.get("optional"):
            self.use = QCheckBox("지정")
            self.use.setToolTip("켜야 이 값이 파일에 실립니다. 끄면 키를 아예 넣지 않습니다 (카메라에 저장된 값 그대로).")
            self.use.toggled.connect(self._on_toggle)
            row.addWidget(self.use)

        self.editor = self._make_editor(field)
        row.addWidget(self.editor, 1)
        if field.get("unit"):
            unit = QLabel(field["unit"])
            unit.setObjectName("Hint")
            row.addWidget(unit)
        if field.get("example") is not None:
            self._show(field["example"])         # '지정' 전에도 무슨 값을 넣게 되는지 보이게
        self.set_value(value)

    def _make_editor(self, field):
        kind = self.kind
        if kind == "bool":
            widget = QCheckBox("사용")
            widget.toggled.connect(self.sig_changed)
        elif kind == "enum":
            widget = QComboBox()
            widget.addItems([str(c) for c in field.get("choices") or []])
            if field.get("editable_choices"):
                # 주석에서 뽑은 목록은 전부가 아닐 수 있다 (Line0 | Line2 | ...) — 직접 적을 수도 있게
                widget.setEditable(True)
                widget.setInsertPolicy(QComboBox.NoInsert)
            widget.currentTextChanged.connect(self.sig_changed)
            guard_wheel(widget)
        elif kind in ("float", "int"):
            widget = QDoubleSpinBox() if kind == "float" else QSpinBox()
            widget.setMinimum(field.get("min", 0))
            widget.setMaximum(field.get("max", 1_000_000))
            widget.setSingleStep(field.get("step", 1))
            if kind == "float":
                widget.setDecimals(2)
            widget.valueChanged.connect(self.sig_changed)
            guard_wheel(widget)
        else:
            widget = QLineEdit()
            if kind == "list":
                widget.setPlaceholderText("[값, 값, …]")
            widget.textChanged.connect(self._on_text)
        widget.setMinimumWidth(90)
        return widget

    def _on_toggle(self, *_):
        self.editor.setEnabled(self.use is None or self.use.isChecked())
        self.sig_changed.emit()

    def _on_text(self, *_):
        self._mark_invalid()
        self.sig_changed.emit()

    def _mark_invalid(self):
        bad = self.value() is INVALID
        if bool(self.editor.property("invalid")) != bad:
            self.editor.setProperty("invalid", bad)
            ui_theme.repolish(self.editor)
        self.editor.setToolTip({"int_text": "정수", "float_text": "실수", "list": "YAML 목록 — 예: [1.0, 2.0]"}
                               .get(self.kind, "") + (" 로 적어야 합니다" if bad else ""))

    def _show(self, value):
        """편집 위젯에 값을 보여주기만 한다 (시그널 없음은 호출하는 쪽이 책임)."""
        kind = self.kind
        if kind == "bool":
            self.editor.setChecked(bool(value))
        elif kind == "enum":
            text = "" if value is None else str(value)
            if self.editor.findText(text) < 0:
                self.editor.addItem(text)
            self.editor.setCurrentText(text)
        elif kind in ("float", "int"):
            try:
                shown = float(value) / self.scale
                self.editor.setValue(shown if kind == "float" else int(round(shown)))
            except (TypeError, ValueError):
                pass
        elif kind == "list":
            self.editor.setText(_list_text(value) if isinstance(value, list) else str(value))
        else:
            self.editor.setText("" if value is None else str(value))

    def set_value(self, value):
        """시그널 없이 값만 바꾼다 (되돌리기 / 다른 카드와 동기화)."""
        self.blockSignals(True)
        try:
            if self.use is not None:
                self.use.setChecked(value is not None)
            if value is not None:
                self._show(value)
            self.editor.setEnabled(self.use is None or self.use.isChecked())
            if self.kind in ("int_text", "float_text", "list"):
                self._mark_invalid()
        finally:
            self.blockSignals(False)

    def _coerce_list(self, items):
        if self._elem is float:
            return [float(x) for x in items]
        if self._elem is int:
            return [int(x) for x in items]
        if self._elem is str:
            return [str(x) for x in items]
        return items

    def value(self):
        if self.use is not None and not self.use.isChecked():
            return None
        kind = self.kind
        if kind == "bool":
            return self.editor.isChecked()
        if kind == "enum":
            return self.editor.currentText()
        if kind == "float":
            return float(self.editor.value()) * self.scale
        if kind == "int":
            return int(round(self.editor.value() * self.scale))
        text = self.editor.text()
        try:
            if kind == "int_text":
                return int(text.strip())
            if kind == "float_text":
                return float(text.strip())
            if kind == "list":
                items = yaml.safe_load(text) if text.strip() else []
                if not isinstance(items, list):
                    return INVALID
                return self._coerce_list(items)
        except (ValueError, TypeError, yaml.YAMLError):
            return INVALID
        return text


class SettingsSection(QFrame):
    """설정 묶음 하나 (주요 설정 · 전체 설정의 파일 섹션 · 직접 추가).

    머리를 누르면 접히고(collapsible), 검색하면 맞는 줄만 남기고 저절로 펼쳐진다.
    """

    def __init__(self, title, hint="", collapsible=False, expanded=True, primary=False, parent=None):
        super().__init__(parent)
        self.setObjectName("SettingsBox")
        self.setProperty("primary", "true" if primary else "false")
        self.title = title
        self.collapsible = collapsible
        self._expanded = expanded
        self._changed = 0
        self._searching = False
        self.rows = []              # [(key, [widgets], 검색용 글)]
        self.subheads = []
        self.always = []            # 검색해도 안 숨기는 줄 (직접 추가 입력 줄)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 4, 8, 8)
        outer.setSpacing(4)
        # QToolButton 은 QSS 의 text-align 을 안 받아 제목이 가운데로 간다 — 평평한 QPushButton 으로
        self.head = QPushButton()
        self.head.setObjectName("SectionHead")
        self.head.setFlat(True)
        self.head.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        if not collapsible:
            self.head.setFocusPolicy(Qt.NoFocus)
        if collapsible:
            self.head.setCursor(Qt.PointingHandCursor)
            self.head.clicked.connect(lambda: self.set_expanded(not self._expanded))
        outer.addWidget(self.head)
        self.hint = None
        if hint:
            self.hint = QLabel(hint)
            self.hint.setObjectName("Hint")
            self.hint.setWordWrap(True)
            outer.addWidget(self.hint)
        self.body = QWidget()
        self.grid = QGridLayout(self.body)
        self.grid.setContentsMargins(4, 0, 0, 0)
        self.grid.setHorizontalSpacing(10)
        self.grid.setVerticalSpacing(5)
        self.grid.setColumnStretch(0, 2)
        self.grid.setColumnStretch(1, 3)
        outer.addWidget(self.body)
        self._next = 0
        self._update()

    # --- 줄 ---

    def add_subhead(self, text):
        label = QLabel(text)
        label.setObjectName("SubHead")
        self.grid.addWidget(label, self._next, 0, 1, 3)
        self._next += 1
        self.subheads.append(label)

    def add_row(self, key, holder, editor, trailing, blob):
        self.grid.addWidget(holder, self._next, 0)
        self.grid.addWidget(editor, self._next, 1)
        self.grid.addWidget(trailing, self._next, 2)
        self._next += 1
        self.rows.append((key, [holder, editor, trailing], blob.lower()))
        self._update()

    def add_wide(self, widget, always=False):
        self.grid.addWidget(widget, self._next, 0, 1, 3)
        self._next += 1
        if always:
            self.always.append(widget)

    def remove_row(self, key):
        for index, (row_key, widgets, _blob) in enumerate(self.rows):
            if row_key == key:
                for widget in widgets:
                    self.grid.removeWidget(widget)
                    widget.deleteLater()
                del self.rows[index]
                break
        self._update()

    # --- 접기 · 검색 · 표시 ---

    def set_expanded(self, expanded):
        self._expanded = expanded
        self._update()

    def set_changed(self, count):
        if count != self._changed:
            self._changed = count
            self._update()

    def set_filter(self, text):
        """맞는 줄 수를 돌려준다. 검색 중엔 맞는 줄만, 소제목은 숨긴다."""
        text = text.strip().lower()
        self._searching = bool(text)
        shown = 0
        for _key, widgets, blob in self.rows:
            match = not text or all(word in blob for word in text.split())
            for widget in widgets:
                widget.setVisible(match)
            shown += match
        for label in self.subheads:
            label.setVisible(not text)
        self.setVisible(not text or shown > 0 or bool(self.always))
        self._update()
        return shown

    def _update(self):
        open_ = self._expanded or self._searching or not self.collapsible
        self.body.setVisible(open_)
        if self.hint is not None:
            self.hint.setVisible(open_)
        arrow = ("▾  " if open_ else "▸  ") if self.collapsible else ""
        count = f"   ({len(self.rows)})" if self.collapsible and self.rows else ""
        changed = f"   ● {self._changed}" if self._changed else ""
        self.head.setText(f"{arrow}{self.title}{count}{changed}")


class Pill(QLabel):
    """실행 상태 알약."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("Pill")
        self.set_state(STOPPED)

    def set_state(self, state, text=None):
        self.setText(text or RUN_TEXT.get(state, state))
        self.setStyleSheet(ui_theme.pill_css(state))


class SensorCard(QFrame):
    """왼쪽 목록의 카드 한 장: 체크(기동 대상) · 이름 · 상태 · 보이는 대수."""

    sig_clicked = pyqtSignal(str)
    sig_toggled = pyqtSignal(str, bool)

    def __init__(self, card, parent=None):
        super().__init__(parent)
        self.card = card
        self.key = card["key"]
        self.setObjectName("SensorCard")
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)

        grid = QGridLayout(self)
        grid.setContentsMargins(12, 10, 12, 10)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(2)

        self.check = QCheckBox()
        self.check.setCursor(Qt.ArrowCursor)
        self.check.setToolTip("체크한 센서가 [선택한 센서 기동] 대상입니다")
        self.check.toggled.connect(lambda on: self.sig_toggled.emit(self.key, on))
        self.title = QLabel(card["label"])
        self.title.setObjectName("CardTitle")
        self.title.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.title.setMinimumWidth(60)
        # 실행 상태 점: 실행 중 초록 · 기동 중 주황(깜빡임) · 실패 빨강 · 정지 회색 테두리.
        # 카드 왼쪽 색 띠(QSS [run=…])와 같이 — 목록을 훑어도 어느 센서가 도는지 바로 보인다.
        self.dot = QLabel("●")
        self.dot.setObjectName("RunDot")
        self._blink = QTimer(self, interval=500, timeout=self._toggle_dot)
        self._dot_on = True
        head = QHBoxLayout()
        head.setSpacing(6)
        head.addWidget(self.dot)
        head.addWidget(self.title, 1)
        self.pill = Pill()
        grid.addWidget(self.check, 0, 0)
        grid.addLayout(head, 0, 1)
        grid.addWidget(self.pill, 0, 2, Qt.AlignRight)

        count_row = QHBoxLayout()
        count_row.setSpacing(4)
        self.count = QLabel("–")
        self.count.setObjectName("CardCount")
        unit = QLabel("대 감지")
        unit.setObjectName("CardSub")
        count_row.addWidget(self.count, 0, Qt.AlignBottom)
        count_row.addWidget(unit, 0, Qt.AlignBottom)
        count_row.addStretch()
        grid.addLayout(count_row, 1, 1, 1, 2)

        self.sub = QLabel("감지 전")
        self.sub.setObjectName("CardSub")
        self.sub.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        grid.addWidget(self.sub, 2, 1, 1, 2)
        self.warn = QLabel("")
        self.warn.setObjectName("CardSub")
        self.warn.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.warn.setProperty("warn", "true")
        self.warn.hide()
        grid.addWidget(self.warn, 3, 1, 1, 2)
        grid.setColumnStretch(1, 1)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.sig_clicked.emit(self.key)
        super().mousePressEvent(event)

    def set_selected(self, selected):
        self.setProperty("selected", "true" if selected else "false")
        ui_theme.repolish(self)

    def set_devices(self, devices, probe, error, candidates=None):
        n = len(devices)
        self.count.setText(str(n))
        self.count.setProperty("zero", "true" if n == 0 else "false")
        ui_theme.repolish(self.count)

        # IP 범위는 정상인 것만. 서브넷이 다른 장비는 따로 한 줄로 — 섞으면 범위가 엉터리가 된다.
        good = [d for d in devices if d["state"] == OK]
        bad = sum(1 for d in devices if d["state"] == SUBNET)
        upd = sum(1 for d in devices if d["state"] == UPDATER)
        nic = sum(1 for d in devices if d["state"] == NIC)
        self.sub.setText(ip_span(good))
        self.sub.setVisible(bool(good))
        spare = ", ".join(c["nic"] for c in candidates or [])
        warn = error or " · ".join(x for x in (f"⚠ {bad}대 IP 안 맞음" if bad else "",
                                               f"⚠ {upd}대 업데이터 모드" if upd else "",
                                               "⚠ PC 쪽 NIC 설정 필요" if nic else "",
                                               f"⚠ IP 없는 유선 NIC {spare}" if spare and not good else "")
                                   if x)
        self.warn.setText(warn)
        self.warn.setVisible(bool(warn))
        ui_theme.repolish(self.warn)
        self.setToolTip("\n".join(f"{d['ip']}  {d['identity']}  {d['note']}" for d in devices)
                        or probe)

    def set_run(self, state, text=None):
        self.pill.set_state(state, text)
        if self.property("run") != state:
            self.setProperty("run", state)
            ui_theme.repolish(self)
            self.dot.setProperty("run", state)
            ui_theme.repolish(self.dot)
        self.dot.setToolTip(RUN_TEXT.get(state, state))
        if state == STARTING:
            if not self._blink.isActive():
                self._blink.start()
        else:
            self._blink.stop()
            self._dot_on = True
            self.dot.setProperty("dim", "false")
            ui_theme.repolish(self.dot)

    def _toggle_dot(self):
        self._dot_on = not self._dot_on
        self.dot.setProperty("dim", "false" if self._dot_on else "true")
        ui_theme.repolish(self.dot)

    def set_locked(self, locked):
        """실행 중에는 기동 대상을 못 바꾼다 (카메라는 한 프로세스라 재기동이 필요하다)."""
        self.check.setEnabled(not locked)
        self.check.setToolTip("실행 중에는 바꿀 수 없습니다 — 중지 후 변경하세요" if locked
                              else "체크한 센서가 [선택한 센서 기동] 대상입니다")


class DetailPage(QWidget):
    """오른쪽 상세: 헤더(이름·상태·기동/중지) + [감지된 장비 | 설정 | 런치 로그] 세로 분할."""

    sig_start = pyqtSignal(str)                 # card key
    sig_stop = pyqtSignal(str)                  # group key
    sig_launch_edited = pyqtSignal(str, str)    # group key, field key (카메라 두 카드 동기화)
    sig_cameras_edited = pyqtSignal(str)        # group key — 카메라 이름/역할을 바꿈
    sig_assign_ip = pyqtSignal(str)             # card key — [IP 할당]
    sig_promote = pyqtSignal(str)               # card key — [기본값으로 저장] (설정)
    sig_promote_inventory = pyqtSignal(str)     # card key — [인벤토리에 저장] (이름 · 역할 · IP)
    sig_splitter = pyqtSignal(QByteArray)

    def __init__(self, card, overrides, ui_state=None, parent=None):
        super().__init__(parent)
        self.card = card
        self.group = card["group"]
        self.overrides = overrides
        # 장비 표 정렬 — cfg["ui"]["device_sort"][카드] = "ip:asc". 다음 실행에도 그대로.
        self._sorts = (ui_state if ui_state is not None else {}).setdefault("device_sort", {})
        self._col_ids = []
        self._base = sensor_config.base_values(self.group)
        self.rows = {}            # (scope, key) -> (field, editor, label, mark, reset)
        self.row_section = {}     # (scope, key) -> SettingsSection
        self.sections = []
        self.subset = next((s for s in sensor_config.subsets_of(self.group)
                            if s["key"] == card["subset"]), None)
        # 인벤토리가 있는 카메라군만 이름/역할을 GUI 에서 정한다 (라이다·GNSS 는 해당 없음)
        self.cameras_editable = bool(self.subset and self.subset.get("inventory"))
        self.sync_roles = bool(self.subset and self.subset.get("sync_roles"))
        self._devices, self._probe, self._error = [], "", None
        self._candidates = []            # 라이다: IPv4 없는 유선 NIC (여기 꽂았을 수 있다)
        self._marks = {}                 # {시리얼: (상태, 설명)} — 표의 행마다 점
        self._blink_on = True
        self._blink = QTimer(self, interval=500, timeout=self._blink_marks)
        self.fix_nic = None              # 라이다: [NIC 설정] 을 누르면 잡을 NIC
        self._locked = False
        self._included = False           # 지금 돌고 있는 프로세스가 이 카드의 종류를 띄웠는가
        self.launched = {}               # {serial: 지금 실행 중인 네임스페이스}
        self._selected = None            # 표에서 고른 시리얼 (다시 그려도 유지)
        self._build()

    # --- UI ---

    def _build(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)

        head = QHBoxLayout()
        title = QLabel(self.card["label"])
        title.setObjectName("PageTitle")
        self.pill = Pill()
        self.btn_start = QPushButton("▶  기동")
        ui_theme.set_variant(self.btn_start, "primary")
        self.btn_stop = QPushButton("■  중지")
        ui_theme.set_variant(self.btn_stop, "danger")
        self.btn_stop.setEnabled(False)
        self.btn_start.clicked.connect(lambda: self.sig_start.emit(self.card["key"]))
        self.btn_stop.clicked.connect(lambda: self.sig_stop.emit(self.group["key"]))
        head.addWidget(title)
        head.addWidget(self.pill, 0, Qt.AlignVCenter)
        head.addStretch()
        head.addWidget(self.btn_start)
        head.addWidget(self.btn_stop)
        outer.addLayout(head)

        subs = sensor_config.subsets_of(self.group)
        if subs:
            others = [s["label"] for s in subs if s["key"] != self.card["subset"]]
            note = QLabel(f"ⓘ  {', '.join(others)}와(과) 한 프로세스로 뜹니다")
            note.setObjectName("Hint")
            note.setToolTip(
                f"{self.card['label']}와(과) {', '.join(others)}는 {self.group['launch_file']} "
                "하나로 뜹니다.\n중지하면 같이 멈추고, 기동 대상을 바꾸려면 중지 후 다시 기동하세요.\n"
                "(따로 띄우면 ptp4l 이 두 번 뜨고 ForceIP 가 서로의 IP를 덮어씁니다.)")
            outer.addWidget(note)

        gaps = sensor_config.missing_paths(self.group)
        if gaps:
            warn = QFrame()
            warn.setObjectName("Warn")
            wl = QVBoxLayout(warn)
            text = QLabel("경로가 없어 기동할 수 없습니다:\n" + "\n".join(gaps))
            text.setWordWrap(True)
            wl.addWidget(text)
            outer.addWidget(warn)
            self.btn_start.setEnabled(False)

        # 위: [감지된 장비 | 설정] 탭 — 각자 전체 높이를 쓴다 (800x600 화면에서도 읽힌다)
        # 아래: 런치 로그 — 탭을 넘겨도 계속 보인다. 경계는 끌어서 조절.
        self.tabs = QTabWidget()
        self.tabs.setObjectName("Inner")
        self.tabs.addTab(self._devices_panel(), "감지된 장비")
        self.tabs.addTab(self._settings_panel(), "설정")
        self.split = QSplitter(Qt.Vertical)
        self.split.setChildrenCollapsible(False)
        self.split.addWidget(self.tabs)
        self.split.addWidget(self._log_panel())
        self.split.setStretchFactor(0, 2)
        self.split.setStretchFactor(1, 1)
        self.split.setSizes([460, 140])
        self._refresh_marks()          # 탭 제목의 변경 개수 표시
        self.split.splitterMoved.connect(lambda *_: self.sig_splitter.emit(self.split.saveState()))
        outer.addWidget(self.split, 1)

    def _panel(self, title, hint=""):
        frame = QFrame()
        frame.setObjectName("Panel")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(6)
        head = QHBoxLayout()
        label = QLabel(title)
        label.setObjectName("Section")
        head.addWidget(label)
        if hint:
            h = QLabel(hint)
            h.setObjectName("Hint")
            head.addWidget(h)
        head.addStretch()
        layout.addLayout(head)
        return frame, layout, head

    def _tab_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)
        return page, layout

    def _devices_panel(self):
        frame, layout = self._tab_page()
        self.sub = QLabel("")
        self.sub.setObjectName("Hint")
        layout.addWidget(self.sub)

        # 이대로는 못 여는 카메라(IP 미설정 · 다른 서브넷 · IP 충돌)가 있으면 뜨는 줄
        self.ip_banner = QFrame()
        self.ip_banner.setObjectName("Warn")
        bl = QHBoxLayout(self.ip_banner)
        bl.setContentsMargins(10, 6, 8, 6)
        self.ip_text = QLabel("")
        self.ip_text.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.ip_text.setWordWrap(True)          # 라이다 안내는 길다 — 버튼에 잘리지 않게
        self.btn_ip = QPushButton("IP 할당")
        ui_theme.set_variant(self.btn_ip, "primary")
        self.btn_ip.clicked.connect(lambda: self.sig_assign_ip.emit(self.card["key"]))
        bl.addWidget(self.ip_text, 1)
        bl.addWidget(self.btn_ip)
        self.ip_banner.hide()
        layout.addWidget(self.ip_banner)

        self.table = QTableWidget(0, 5)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self.table.itemChanged.connect(self._on_table_item)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        # 헤더를 누르면 그 열로 오름차순, 한 번 더 누르면 내림차순 (다시 오름…). Qt 의 자체 정렬
        # (setSortingEnabled)은 쓰지 않는다 — PTP 동기 칸의 콤보(셀 위젯)가 행을 못 따라가고,
        # 이름을 고치는 중에 행이 튀어 다닌다. 감지 목록을 정렬해서 다시 그린다.
        # 정렬 방향은 제목 글자에 ▲/▼ 로 붙인다 — Qt 의 정렬 표시는 오름차순을 ▾(아래)로 그려 헷갈린다.
        header = self.table.horizontalHeader()
        header.setSectionsClickable(True)
        header.setSortIndicatorShown(False)
        header.sectionClicked.connect(self._on_sort)
        header.setToolTip("열 제목을 누르면 정렬 (한 번 더 누르면 반대로)")
        self.table.setMinimumHeight(70)
        self.table.itemSelectionChanged.connect(self._on_select)
        # 카메라군이면 오른쪽에 고른 카메라의 라이브 (이름 짓기용). 경계는 끌어서 조절.
        self.live = LiveView()
        self.live.sig_rename.connect(self._apply_name)
        self.dev_split = QSplitter(Qt.Horizontal)
        self.dev_split.setChildrenCollapsible(False)
        self.dev_split.addWidget(self.table)
        self.dev_split.addWidget(self.live)
        self.dev_split.setStretchFactor(0, 3)
        self.dev_split.setStretchFactor(1, 2)
        self.live.setVisible(self.cameras_editable)
        layout.addWidget(self.dev_split, 1)
        self.empty = QLabel("감지 전")
        self.empty.setObjectName("Hint")
        self.empty.setAlignment(Qt.AlignCenter)
        self.empty.setWordWrap(True)
        layout.addWidget(self.empty, 1)
        self.dev_split.hide()
        self.table_msg = QLabel("")
        self.table_msg.setObjectName("Hint")
        self.table_msg.setWordWrap(True)
        foot = QHBoxLayout()
        foot.addWidget(self.table_msg, 1)
        # 카메라 이름 · PTP 역할 · IP 를 리포 인벤토리에 — GUI 없이 ros2 launch 로 띄워도 같은 이름이 된다
        self.btn_promote_inv = QPushButton("이름·역할을 인벤토리에 저장")
        self.btn_promote_inv.setToolTip(
            "GUI 에서 정한 카메라 이름 · PTP 역할 · IP 를 리포의 인벤토리 YAML 에 씁니다\n"
            "(multicam_cameras.yaml 등 — 원본은 ~/.config/dm_clip_gui/backups 에 복사해 둡니다).")
        self.btn_promote_inv.clicked.connect(lambda: self.sig_promote_inventory.emit(self.card["key"]))
        self.btn_promote_inv.setVisible(self.cameras_editable)
        self.btn_promote_inv.setEnabled(False)
        foot.addWidget(self.btn_promote_inv, 0, Qt.AlignTop)
        layout.addLayout(foot)
        return frame

    def _settings_panel(self):
        """설정 탭: [검색 · 바뀐 개수 · 모두 원본값으로] + 스크롤
             주요 설정 (한국어 이름, 소제목)      ← sensors.yaml fields
             카메라 공통 옵션 (런치 인자)
             전체 설정 — 원본 YAML 섹션마다 접히는 상자 + 런치 인자 전부   ← 파일에서 직접 읽음
             직접 추가 — 원본에 없는 키
        """
        frame, layout = self._tab_page()
        head = QHBoxLayout()
        self.search = QLineEdit()
        self.search.setPlaceholderText("설정 검색 — 이름 · 키 · 설명  (예: exposure, 대역폭, ptp, packet)")
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self._apply_filter)
        head.addWidget(self.search, 1)
        self.lbl_changed = QLabel("")
        self.lbl_changed.setObjectName("Changed")
        head.addWidget(self.lbl_changed)
        self.btn_reset_all = QPushButton("모두 원본값으로")
        self.btn_reset_all.setToolTip("이 카드에서 바꾼 값을 전부 버리고 리포의 원본 YAML / 런치 기본값으로 되돌립니다")
        self.btn_reset_all.clicked.connect(self.reset_all)
        head.addWidget(self.btn_reset_all)
        self.btn_promote = QPushButton("기본값으로 저장")
        self.btn_promote.setToolTip(
            "지금 바꾼 값을 리포의 원본(params YAML · 런치 파일 · sensors.yaml)에 써서 아예 기본값으로 만듭니다.\n"
            "GUI 없이 ros2 launch 로 띄워도 같은 값이 되고, 여기의 '원본과 다른 값' 은 0 이 됩니다.\n"
            "원본 파일은 ~/.config/dm_clip_gui/backups 에 복사해 두고, 주석 · 순서는 그대로 둡니다.")
        self.btn_promote.clicked.connect(lambda: self.sig_promote.emit(self.card["key"]))
        head.addWidget(self.btn_promote)
        layout.addLayout(head)

        body = QWidget()
        col = QVBoxLayout(body)
        col.setContentsMargins(0, 0, 6, 0)
        col.setSpacing(8)
        self.sections = []
        main = self.card["subset"] or sensor_config.NO_SUBSET
        subs = sensor_config.subsets_of(self.group)

        # 1) 주요 설정
        curated = sensor_config.fields_for(self.group, main)
        if curated:
            hint = "이 종류의 모든 장비에 똑같이 적용됩니다. 칸에 마우스를 올리면 설명이 나옵니다." \
                if self.card["subset"] else "칸에 마우스를 올리면 설명이 나옵니다."
            self._add_section(SettingsSection(f"주요 설정 — {self.card['label']}", hint, primary=True),
                              col, main, curated)
        launch = sensor_config.fields_for(self.group, "launch")
        if launch:
            title, hint = "런치 옵션", ""
            if subs:
                title = "카메라 공통 옵션"
                hint = " · ".join(s["label"] for s in subs) + "에 함께 적용됩니다 (한 프로세스)."
            self._add_section(SettingsSection(title, hint, primary=True), col, "launch", launch)

        # 2) 전체 설정 — 원본 파일에서 읽은 전부
        full = [(main, t, f) for t, f in sensor_config.full_fields(self.group, main)]
        full += [("launch", t, f) for t, f in sensor_config.full_fields(self.group, "launch")]
        if full:
            target = next((t for t in sensor_config.param_targets(self.group) if t["subset"] == main), None)
            source = sensor_config.expand(target["source"]).name if target else ""
            bar = QHBoxLayout()
            title = QLabel("전체 설정")
            title.setObjectName("BigTitle")
            bar.addWidget(title)
            note = QLabel(f"{source} 의 모든 키와 런치 인자 · 회색 = 원본에서 주석 처리된 키 ('지정' 해야 실림) "
                          "· 잠긴 칸 = 런치/GUI 가 정함")
            note.setObjectName("Hint")
            note.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            note.setToolTip(note.text())
            bar.addWidget(note, 1)
            for text, expand in (("모두 펼치기", True), ("모두 접기", False)):
                button = QToolButton()
                button.setText(text)
                button.clicked.connect(lambda _=False, e=expand: self._expand_all(e))
                bar.addWidget(button)
            col.addSpacing(6)
            col.addLayout(bar)
            for scope, title, fields in full:
                self._add_section(SettingsSection(title, collapsible=True, expanded=False),
                                  col, scope, fields)

        # 3) 직접 추가 — params 파일이 있는 센서군만 (런치 인자는 선언된 것만 받는다)
        if any(t["subset"] == main for t in sensor_config.param_targets(self.group)):
            col.addWidget(self._custom_section(main))

        self.no_match = QLabel("")
        self.no_match.setObjectName("Hint")
        self.no_match.setWordWrap(True)
        self.no_match.hide()
        col.addWidget(self.no_match)
        col.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(body)
        layout.addWidget(scroll, 1)
        self._refresh_marks()
        return frame

    def _add_section(self, section, column, scope, fields):
        last = None
        for field in fields:
            if field.get("section") and field["section"] != last:
                section.add_subhead(field["section"])
                last = field["section"]
            self._add_field_row(section, scope, field)
        self.sections.append(section)
        column.addWidget(section)
        return section

    def _add_field_row(self, section, scope, field):
        key = field["key"]
        value = sensor_config.effective_value(self.group, field, self.overrides, self._base)
        editor = FieldEditor(field, value)
        editor.sig_changed.connect(lambda s=scope, k=key: self._on_edit(s, k))

        generic = field.get("generic")
        text = field.get("label") or key
        if field.get("caution"):
            text = "⚠ " + text
        label = ElidedLabel(text) if generic else QLabel(text)
        label.setObjectName("KeyLabel" if generic else "FieldLabel")
        if field.get("caution"):
            label.setProperty("caution", "true")

        mark = QLabel("●")
        mark.setObjectName("Changed")
        mark.setToolTip("원본과 다른 값")
        holder = QWidget()
        left = QHBoxLayout(holder)
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(4)
        left.addWidget(mark)
        left.addWidget(label, 1)

        # 되돌리기(↺)는 바뀐 줄에만 보이지만, 칸 너비는 늘 잡아 둔다 — 줄마다 편집 칸이 흔들리지 않게
        trailing = QWidget()
        trailing.setFixedWidth(46 if field.get("custom") else 24)
        tl = QHBoxLayout(trailing)
        tl.setContentsMargins(0, 0, 0, 0)
        tl.setSpacing(0)
        reset = QToolButton()
        reset.setText("↺")
        reset.setToolTip("원본값으로")
        reset.clicked.connect(lambda _=False, s=scope, k=key: self.reset_field(s, k))
        tl.addWidget(reset)
        if field.get("custom"):
            reset.hide()
            drop = QToolButton()
            drop.setText("✕")
            drop.setToolTip("이 키를 빼기")
            drop.clicked.connect(lambda _=False, s=scope, k=key: self._remove_custom(s, k))
            tl.addWidget(drop)
        tl.addStretch()

        if field.get("managed"):
            editor.setEnabled(False)
            editor.setToolTip(f"🔒 {field['managed']}")
            label.setProperty("unset", "true")

        blob = " ".join(str(x) for x in (key, field.get("label", ""), field.get("help", ""),
                                         section.title, field.get("section", "")))
        section.add_row(key, holder, editor, trailing, blob)
        self.rows[(scope, key)] = (field, editor, label, mark, reset)
        self.row_section[(scope, key)] = section

    def _custom_section(self, scope):
        """원본 YAML 에 없는 키를 직접 넣는 상자. 넣은 키는 GUI 설정에 저장된다."""
        known = sensor_config.known_keys(self.group, scope)
        mine = {k: v for k, v in self._store(scope).items() if k not in known}
        hint = "원본 YAML 에 없는 ROS 파라미터를 넣습니다. 값은 YAML 로: 1.0 · 30 · true · \"Off\" · [1, 2]."
        if self.cameras_editable:
            hint += (" 카메라 노드는 camera.* / stream.* / tl_device.* 를 GenICam 노드 이름으로 그대로 "
                     "적용합니다 (이름은 SpinView 나 ros2 param list 로 확인).")
        section = SettingsSection("직접 추가 — 원본에 없는 키", hint, collapsible=True, expanded=bool(mine))
        self.custom_section = section
        for key, value in mine.items():
            self._add_field_row(section, scope, sensor_config.custom_field(scope, key, value))

        entry = QWidget()
        row = QHBoxLayout(entry)
        row.setContentsMargins(0, 4, 0, 0)
        row.setSpacing(6)
        self.custom_key = QLineEdit()
        self.custom_key.setPlaceholderText("camera.BalanceWhiteAuto" if self.cameras_editable else "파라미터 이름")
        self.custom_value = QLineEdit()
        self.custom_value.setPlaceholderText("값 (YAML)")
        add = QPushButton("+ 추가")
        add.clicked.connect(lambda: self._add_custom(scope))
        self.custom_value.returnPressed.connect(lambda: self._add_custom(scope))
        row.addWidget(self.custom_key, 3)
        row.addWidget(self.custom_value, 2)
        row.addWidget(add)
        section.add_wide(entry, always=True)
        self.custom_msg = QLabel("")
        self.custom_msg.setObjectName("Hint")
        self.custom_msg.setWordWrap(True)
        self.custom_msg.hide()
        section.add_wide(self.custom_msg, always=True)
        self.sections.append(section)
        return section

    def _custom_error(self, text):
        self.custom_msg.setText(text)
        self.custom_msg.setStyleSheet(f"color:{ui_theme.ERR};" if text else "")
        self.custom_msg.setVisible(bool(text))

    def _add_custom(self, scope):
        key = self.custom_key.text().strip()
        text = self.custom_value.text().strip()
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_.]*", key):
            return self._custom_error("키는 영문자로 시작하고 영문 · 숫자 · 밑줄 · 점만 씁니다 (예: camera.GammaEnable)")
        if (scope, key) in self.rows:
            self.search.setText(key)
            return self._custom_error(f"{key} 는 이미 목록에 있습니다 — 검색창에 넣었으니 거기서 바꾸세요")
        reason = sensor_config.managed_reason(self.group, scope, key)
        if reason:
            return self._custom_error(f"{key} 는 GUI 에서 정할 수 없습니다 — {reason}")
        try:
            value = yaml.safe_load(text) if text else None
        except yaml.YAMLError:
            value = None
        if value is None or isinstance(value, dict):
            return self._custom_error('값을 YAML 로 적으세요 — 예: 1.0 · 30 · true · "Off" · [1, 2]')
        self._custom_error("")
        self._store(scope)[key] = value
        self._add_field_row(self.custom_section, scope, sensor_config.custom_field(scope, key, value))
        self.custom_section.set_expanded(True)
        self.custom_key.clear()
        self.custom_value.clear()
        self._apply_filter(self.search.text())
        self._refresh_marks()

    def _remove_custom(self, scope, key):
        self._store(scope).pop(key, None)
        self.rows.pop((scope, key), None)
        section = self.row_section.pop((scope, key), None)
        if section is not None:
            section.remove_row(key)
        self._refresh_marks()

    def _expand_all(self, expanded):
        for section in self.sections:
            if section.collapsible:
                section.set_expanded(expanded)

    def _apply_filter(self, text):
        shown = sum(section.set_filter(text) for section in self.sections)
        if text.strip() and not shown:
            self.no_match.setText(f"'{text.strip()}' 에 맞는 설정이 없습니다. 원본 YAML 에 없는 키(예: GenICam 노드)라면 "
                                  "위 '직접 추가' 에 넣으세요.")
            if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_.]*", text.strip()) and hasattr(self, "custom_key"):
                self.custom_key.setText(text.strip())
            self.no_match.show()
        else:
            self.no_match.hide()

    def _log_panel(self):
        frame, layout, head = self._panel("런치 로그")
        clear = QToolButton()
        clear.setText("지우기")
        head.addWidget(clear)
        self.log = QPlainTextEdit()
        self.log.setObjectName("Log")
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        self.log.setLineWrapMode(QPlainTextEdit.NoWrap)
        font = self.log.font()
        font.setFamily("Monospace")
        font.setStyleHint(font.Monospace)
        font.setPointSize(9)
        self.log.setFont(font)
        clear.clicked.connect(self.log.clear)
        layout.addWidget(self.log, 1)
        return frame

    # --- 설정 값 ---

    def _store(self, scope):
        if scope == "launch":
            return self.overrides.setdefault("launch_args", {})
        return self.overrides.setdefault("params", {}).setdefault(scope, {})

    def _on_edit(self, scope, key):
        """원본과 같으면 오버라이드에서 뺀다 — 그래야 리포 원본이 바뀌면 그대로 따라간다."""
        field, editor, *_ = self.rows[(scope, key)]
        value = editor.value()
        if value is INVALID:
            return                      # 칸이 빨갛게 표시돼 있다. 고칠 때까지 저장된 값을 건드리지 않는다.
        store = self._store(scope)
        if not field.get("custom") and _same(value, sensor_config.base_value(self.group, field, self._base)):
            store.pop(key, None)
        else:
            store[key] = value
        self._refresh_marks()
        if scope == "launch":
            self.sig_launch_edited.emit(self.group["key"], key)

    def reset_field(self, scope, key):
        field, editor, *_ = self.rows[(scope, key)]
        if field.get("custom"):
            return self._remove_custom(scope, key)
        self._store(scope).pop(key, None)
        editor.set_value(sensor_config.base_value(self.group, field, self._base))
        self._refresh_marks()
        if scope == "launch":
            self.sig_launch_edited.emit(self.group["key"], key)

    def reset_all(self):
        for scope, key in list(self.rows):
            if key in self._store(scope):
                self.reset_field(scope, key)

    def sync_launch_field(self, key):
        """다른 카메라 카드에서 공통 옵션을 바꿨을 때 이쪽 폼도 맞춘다."""
        row = self.rows.get(("launch", key))
        if not row:
            return
        field, editor, *_ = row
        editor.set_value(sensor_config.effective_value(self.group, field, self.overrides, self._base))
        self._refresh_marks()

    def _tooltip(self, field, is_changed):
        parts = [field["key"]]
        if field.get("managed"):
            parts.append(f"🔒 {field['managed']}")
        if field.get("optional") and field.get("generic"):
            parts.append(f"원본 YAML 에서 주석 처리된 키 — '지정' 을 켜야 파일에 실립니다 (예시값 {field.get('example')!r})")
        if is_changed and not field.get("custom"):
            parts.append(f"원본: {sensor_config.base_value(self.group, field, self._base)!r}")
        if field.get("help"):
            parts.append(field["help"])
        return "\n".join(parts)

    def _refresh_marks(self):
        changed = 0
        per_section = {}
        for (scope, key), (field, editor, label, mark, reset) in self.rows.items():
            is_changed = key in self._store(scope)
            changed += is_changed
            section = self.row_section.get((scope, key))
            if section is not None:
                per_section[section] = per_section.get(section, 0) + is_changed
            mark.setVisible(is_changed)
            reset.setVisible(is_changed and not field.get("custom"))
            font = label.font()
            if font.bold() != is_changed:
                font.setBold(is_changed)
                label.setFont(font)
            unset = bool(field.get("managed")) or (bool(field.get("optional")) and editor.value() is None)
            if (label.property("unset") == "true") != unset:
                label.setProperty("unset", "true" if unset else "false")
                ui_theme.repolish(label)
            tip = self._tooltip(field, is_changed)
            label.setToolTip(tip)
            if not field.get("managed"):
                editor.setToolTip(tip)
        for section in self.sections:
            section.set_changed(per_section.get(section, 0))

        # enabled_when: 노출 auto=Continuous 면 노출 시간 칸은 의미가 없다 → 회색
        for (scope, _key), (field, editor, *_rest) in self.rows.items():
            condition = field.get("enabled_when") or {}
            if not condition:
                continue
            ok = all(self.rows.get((scope, dk)) is None or
                     self.rows[(scope, dk)][1].value() == dv
                     for dk, dv in condition.items())
            editor.setEnabled(ok)

        self.lbl_changed.setText(f"● 원본과 다른 값 {changed}개" if changed
                                 else "원본 그대로")
        self.lbl_changed.setObjectName("Changed" if changed else "Hint")
        ui_theme.repolish(self.lbl_changed)
        self.btn_reset_all.setEnabled(bool(changed))
        self.btn_promote.setEnabled(bool(changed))
        if hasattr(self, "tabs"):
            self.tabs.setTabText(1, f"설정  ●{changed}" if changed else "설정")

    # --- 표시 ---

    NAME_COL = 0          # 이름 · 시리얼 · IP · (PTP 동기) · 모델 · NIC — 이름 짓기가 목적이라 맨 앞
    SERIAL_COL = 1

    def set_devices(self, devices, probe, error, candidates=None):
        self._devices, self._probe, self._error = list(devices), probe, error
        self._candidates = list(candidates or [])
        self._render_devices()

    def _render_devices(self):
        devices = self._devices
        self.sub.setText(("⚠ " + self._error) if self._error else f"감지 방법 — {self._probe}")
        edited = sum(1 for d in devices
                     if d["identity"] in sensor_config.camera_overrides(self.overrides))
        self.tabs.setTabText(0, f"감지된 장비  {len(devices)}" + (f"  ●{edited}" if edited else ""))
        bad = [d for d in devices if d["state"] == SUBNET]
        upd = [d for d in devices if d["state"] == UPDATER]
        if (bad or upd) and self.cameras_editable:
            parts = []
            if bad:
                parts.append(f"⚠ {len(bad)}대 IP 안 맞음 — " +
                             ", ".join(f"{d['identity']} ({d['ip']})" for d in bad) +
                             ("  · 실행 중이라 중지 후 할당" if self._locked else ""))
            if upd:
                parts.append(f"⚠ {len(upd)}대 업데이터 모드 — " +
                             ", ".join(d["identity"] for d in upd) + " · 전원을 다시 넣으세요")
            self.ip_text.setText("   ".join(parts))
            self.btn_ip.setVisible(bool(bad))
            self.ip_banner.setToolTip(
                "\n".join(f"{d['identity']}: {d['note']}" for d in bad + upd) +
                "\n\n[IP 할당] 은 한 대씩 호스트 서브넷의 빈 주소를 주고 다시 찾아 확인합니다.\n"
                "카메라 전원을 다시 넣으면 풀리고, 그때 다시 누르면 같은 주소로 맞춥니다.\n"
                "툴바의 'IP 자동 맞춤' 을 켜 두면 기동할 때 알아서 합니다.")
            self.btn_ip.setEnabled(not self._locked)
            self.ip_banner.show()
        elif not self.cameras_editable and self._lidar_banner(devices):
            self.ip_banner.show()
        else:
            self.ip_banner.hide()

        if not devices:
            self.dev_split.hide()
            self.empty.show()
            self.empty.setText("감지된 장비가 없습니다.\n전원·케이블을 확인하고 [다시 감지]를 누르세요.")
            self.table_msg.hide()
            return
        self.empty.hide()
        self.dev_split.show()

        headers = ([("이름" if self.cameras_editable else "비고"), "시리얼", "IP"]
                   + (["PTP 동기"] if self.sync_roles else []) + ["모델", "NIC"])
        self._col_ids = ["name", "serial", "ip"] + (["ptp"] if self.sync_roles else []) + ["model", "nic"]
        tail = 4 if self.sync_roles else 3          # 모델 칸 위치
        col_id, descending = self._sort_spec()
        if col_id:
            devices = sorted(devices, key=lambda d: self._sort_key(d, col_id), reverse=descending)
            index = self._col_ids.index(col_id)
            headers[index] += "  ▼" if descending else "  ▲"
        self.table.blockSignals(True)
        self.table.clear()
        self.table.setColumnCount(len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        self.table.setRowCount(len(devices))
        # 실행 중에도 고칠 수 있다 — 라이브를 보며 이름을 짓는 게 목적이다. 적용은 다음 기동부터.
        editable = self.cameras_editable
        for r, dev in enumerate(devices):
            for c, text in ((1, dev["identity"]), (2, dev["ip"]), (tail, dev["model"]),
                            (tail + 1, dev["nic"])):
                item = QTableWidgetItem(text)
                item.setData(Qt.UserRole, dev["identity"])
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                if dev["state"] != OK:
                    item.setForeground(Qt.darkYellow if dev["state"] in (SUBNET, NIC) else Qt.red)
                    item.setToolTip(dev["note"])
                self.table.setItem(r, c, item)

            if not self.cameras_editable:
                item = QTableWidgetItem(dev["note"])
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(r, self.NAME_COL, item)
                continue

            serial = dev["identity"]
            entry = sensor_config.camera_entry(self.subset, serial, self.overrides)
            default = sensor_config.default_camera_entry(self.subset, serial)
            item = QTableWidgetItem(entry["namespace"])
            item.setData(Qt.UserRole, serial)
            if not editable:
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            changed = entry["namespace"] != default["namespace"]
            font = item.font()
            font.setBold(changed)
            item.setFont(font)
            running = self.launched.get(serial)
            item.setToolTip((f"원본: {default['namespace']}\n" if changed else "") +
                            (f"지금 실행 중인 이름: {running}\n" if running and running != entry["namespace"]
                             else "") + "더블클릭해서 이름을 바꿉니다 (다음 기동부터 적용)")
            if dev["state"] != OK:
                item.setForeground(Qt.darkYellow if dev["state"] == SUBNET else Qt.red)
            self.table.setItem(r, self.NAME_COL, item)

            if self.sync_roles:
                combo = QComboBox()
                combo.addItems(sensor_config.PTP_ROLES)
                role = entry.get("ptp_action_role", "none")
                if combo.findText(role) < 0:
                    combo.addItem(role)
                combo.setCurrentText(role)
                combo.setEnabled(editable)
                combo.setToolTip(f"원본: {default.get('ptp_action_role', 'none')}\n"
                                 "sender 는 하나. 보이는 카메라 중 sender 가 없으면 기동 때 "
                                 "첫 카메라가 sender 로 쓰입니다.")
                combo.currentTextChanged.connect(
                    lambda text, sn=serial: self._on_role(sn, text))
                self.table.setCellWidget(r, 3, combo)
        # 고른 행 유지 (다시 그려도 라이브가 끊기지 않게)
        if self._selected:
            for r in range(self.table.rowCount()):
                cell = self.table.item(r, self.SERIAL_COL)
                if cell and cell.text() == self._selected:
                    self.table.selectRow(r)
                    break
        self.table.blockSignals(False)
        self._apply_marks()
        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setStretchLastSection(True)

        if self.cameras_editable:
            edited = [d for d in self._devices if d["identity"] in sensor_config.camera_overrides(self.overrides)]
            self.btn_promote_inv.setEnabled(bool(edited))
            self.btn_promote_inv.setText(f"이름·역할을 인벤토리에 저장 ({len(edited)})" if edited
                                         else "이름·역할을 인벤토리에 저장")
            self.table_msg.show()
            if not self.table_msg.text():          # 방금 한 동작의 결과 메시지는 남겨 둔다
                self.table_msg.setText("행을 고르면 오른쪽에 라이브 (기동 중일 때) · 이름은 더블클릭 "
                                       "또는 오른쪽 칸 · 다음 기동부터 적용")
                self.table_msg.setToolTip("이름·PTP 역할은 GUI 설정에 저장됩니다 — 리포 인벤토리 YAML 은 "
                                          "바뀌지 않습니다.")
        else:
            self.table_msg.hide()

    def _lidar_banner(self, devices):
        """라이다가 PC 에 못 붙는 이유와 [NIC 설정] 버튼. 보일 게 없으면 False."""
        if any(d["state"] == OK for d in devices):
            self.fix_nic = None
            return False
        stuck = [d for d in devices if d["state"] == NIC]
        fixable = [d for d in stuck if d.get("fix_nic")]
        self.fix_nic = (fixable[0]["fix_nic"] if fixable
                        else self._candidates[0]["nic"] if self._candidates else None)
        if stuck:
            text = "⚠ " + " · ".join(f"{d['identity'] or '라이다'}: {d['note']}" for d in stuck)
        elif self._candidates:
            spare = ", ".join(f"{c['nic']} ({'USB, ' if c['usb'] else ''}"
                              f"{c['speed']} Mb/s)" if c["speed"] > 0 else c["nic"]
                              for c in self._candidates)
            text = (f"⚠ 라이다가 안 보입니다. IPv4 없는 유선 NIC: {spare} — "
                    "라이다를 여기 꽂았다면 link-local 로 잡아야 붙습니다")
        else:
            return False
        self.ip_text.setText(text)
        self.btn_ip.setText(f"{self.fix_nic} NIC 설정" if self.fix_nic else "NIC 설정")
        self.btn_ip.setVisible(bool(self.fix_nic))
        self.btn_ip.setEnabled(not self._locked)
        self.ip_banner.setToolTip(
            "라이다(Ouster)는 DHCP 가 없는 link-local(169.254.x.x) 장비입니다. 새 NIC(USB 이더넷 어댑터 등)에\n"
            "꽂으면 NetworkManager 가 그 NIC 에 DHCP 를 걸어 45초 기다리다 실패하고 끊기를 되풀이해서, PC 가\n"
            "라이다에 붙지 못합니다.\n\n"
            "[NIC 설정] 은 그 NIC 에 link-local 프로필('LiDAR link-local (NIC)')을 만들고 올립니다 (nmcli,\n"
            "sudo 불필요). 다음에 꽂을 때부터는 자동으로 쓰입니다. 되돌리기: nmcli connection delete '<프로필>'\n"
            "툴바의 'IP 자동 맞춤' 을 켜 두면 기동할 때 알아서 합니다.")
        return True

    def reload_settings(self):
        """원본이 바뀐 뒤(기본값으로 저장) 설정 탭을 새로 만든다 — 원본값 · 전체 설정 목록을 다시 읽는다."""
        self._base = sensor_config.base_values(self.group)
        search = self.search.text() if hasattr(self, "search") else ""
        self.rows, self.row_section, self.sections = {}, {}, []
        old = self.tabs.widget(1)
        self.tabs.removeTab(1)
        old.deleteLater()
        self.tabs.insertTab(1, self._settings_panel(), "설정")
        self.search.setText(search)
        self._refresh_marks()

    def set_row_marks(self, marks):
        """{시리얼: (상태, 설명)} — 감지된 장비 표의 이름 칸 앞 점."""
        if marks != self._marks:
            self._marks = dict(marks)
            self._apply_marks()

    def _apply_marks(self):
        starting = False
        self.table.blockSignals(True)          # 아이콘만 바꾼다 — 이름 편집 신호로 새지 않게
        try:
            for r in range(self.table.rowCount()):
                serial_item = self.table.item(r, self.SERIAL_COL)
                name_item = self.table.item(r, self.NAME_COL)
                if not serial_item or not name_item:
                    continue
                state, detail = self._marks.get(serial_item.text(), (STOPPED, ""))
                starting |= state == STARTING
                name_item.setIcon(dot_icon(state, self._blink_on))
                base = name_item.data(Qt.UserRole + 1)
                if base is None:
                    base = name_item.toolTip()
                    name_item.setData(Qt.UserRole + 1, base)
                status = ROW_STATE_TEXT.get(state, state) + (f" ({detail})" if detail else "")
                name_item.setToolTip(f"상태: {status}" + (f"\n{base}" if base else ""))
        finally:
            self.table.blockSignals(False)
        if starting and not self._blink.isActive():
            self._blink.start()
        elif not starting:
            self._blink.stop()
            self._blink_on = True

    def _blink_marks(self):
        self._blink_on = not self._blink_on
        self.table.blockSignals(True)
        try:
            for r in range(self.table.rowCount()):
                serial_item = self.table.item(r, self.SERIAL_COL)
                name_item = self.table.item(r, self.NAME_COL)
                if serial_item and name_item and \
                        self._marks.get(serial_item.text(), (None,))[0] == STARTING:
                    name_item.setIcon(dot_icon(STARTING, self._blink_on))
        finally:
            self.table.blockSignals(False)

    def _sort_spec(self):
        """(열 id, 내림차순?) — 저장된 게 없거나 지금 표에 없는 열이면 (None, False) = 감지 순서."""
        col_id, _, order = str(self._sorts.get(self.card["key"]) or "").partition(":")
        if col_id not in (self._col_ids or ["name", "serial", "ip", "ptp", "model", "nic"]):
            return None, False
        return col_id, order == "desc"

    def _on_sort(self, index):
        if not 0 <= index < len(self._col_ids):
            return
        col_id = self._col_ids[index]
        current, descending = self._sort_spec()
        descending = (not descending) if current == col_id else False
        self._sorts[self.card["key"]] = f"{col_id}:{'desc' if descending else 'asc'}"
        self._render_devices()

    @staticmethod
    def _natural(text):
        """'front_right10' 이 'front_right2' 뒤에 오게 — 숫자는 숫자로 비교."""
        return [int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", str(text or ""))]

    def _sort_key(self, dev, col_id):
        serial = self._natural(dev["identity"])
        if col_id == "ip":
            try:
                return [int(x) for x in dev["ip"].split(".")], serial
            except ValueError:
                return [999], serial
        if col_id == "name":
            if self.cameras_editable:
                name = sensor_config.camera_entry(self.subset, dev["identity"], self.overrides)["namespace"]
            else:
                name = dev["note"]
            return self._natural(name), serial
        if col_id == "ptp":
            role = sensor_config.camera_entry(self.subset, dev["identity"], self.overrides).get(
                "ptp_action_role", "none")
            return [{"sender": 0, "receiver": 1}.get(role, 2)], serial
        if col_id == "model":
            return self._natural(dev["model"]), serial
        if col_id == "nic":
            return self._natural(dev["nic"]), serial
        return serial

    def _camera_store(self, serial):
        return sensor_config.camera_overrides(self.overrides).setdefault(serial, {})

    def _drop_empty(self, serial):
        cams = sensor_config.camera_overrides(self.overrides)
        if not cams.get(serial):
            cams.pop(serial, None)

    def _show_table_msg(self, text, error=False):
        self.table_msg.setProperty("error", error)
        self.table_msg.setText(text)
        self.table_msg.setStyleSheet(f"color:{ui_theme.ERR};" if error else "")

    def _on_table_item(self, item):
        if item.column() != self.NAME_COL or not self.cameras_editable:
            return
        serial = item.data(Qt.UserRole)
        if serial:
            self._apply_name(serial, item.text().strip())

    def _apply_name(self, serial, name):
        """표(더블클릭)와 라이브의 이름 칸이 둘 다 여기로 온다."""
        default = sensor_config.default_camera_entry(self.subset, serial)["namespace"]
        store = self._camera_store(serial)
        if not name or name == default:
            store.pop("name", None)
            self._show_table_msg(f"{serial}: 원본 이름({default})으로 되돌렸습니다.")
        else:
            problem = sensor_config.validate_camera_name(self.group, self.overrides, serial, name)
            if problem:
                self._show_table_msg(f"'{name}' 은(는) 쓸 수 없습니다 — {problem}", error=True)
                self._drop_empty(serial)
                self._render_devices()
                self._on_select()
                return
            store["name"] = name
            self._show_table_msg(f"{serial} → {name}  (다음 기동부터 /{name}/… 로 발행)")
        self._drop_empty(serial)
        self._render_devices()
        self._on_select()
        self.sig_cameras_edited.emit(self.group["key"])

    def _on_select(self):
        """고른 카메라를 라이브에 — 실행 중이고 이번 기동에 들어간 카메라면 스트림, 아니면 이름만."""
        if not self.cameras_editable:
            return
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        item = self.table.item(rows[0].row(), self.SERIAL_COL) if rows else None
        serial = item.text() if item else None
        if not serial:
            self.live.clear_camera("표에서 카메라를 고르세요.")
            return
        changed = serial != self._selected or self.live.serial != serial
        self._selected = serial
        next_name = sensor_config.camera_entry(self.subset, serial, self.overrides)["namespace"]
        running = self.launched.get(serial) if (self._locked and self._included) else None
        if running:
            if changed or not self.live.topics:
                self.live.show_camera(serial, running, next_name, self.subset["key"] == "thermal",
                                      sensor_config.temperature_scale(self.group, self.overrides))
            else:                                     # 이름만 바뀐 경우 — 스트림은 그대로
                self.live.name.setText(next_name)
                self.live.hint.setText("" if next_name == running else
                                       f"지금은 /{running}/ 로 발행 중 — 다음 기동부터 /{next_name}/")
        else:
            dev = next((d for d in self._devices if d["identity"] == serial), {})
            why = ("이번 기동에 없는 카메라입니다" if self._locked else
                   "기동하면 여기서 라이브로 볼 수 있습니다")
            if dev.get("state") == SUBNET:
                why = "이대로는 못 여는 카메라입니다 — 위의 [IP 할당] 먼저"
            elif dev.get("state") == UPDATER:
                why = "업데이터 모드입니다 — 카메라 전원을 다시 넣으세요"
            self.live.clear_camera(f"{why}.\n이름은 지금 정해 둘 수 있습니다.", serial, next_name)

    def set_launched(self, mapping):
        self.launched = dict(mapping)

    def _on_role(self, serial, role):
        default = sensor_config.default_camera_entry(self.subset, serial).get("ptp_action_role", "none")
        store = self._camera_store(serial)
        if role == default:
            store.pop("ptp_action_role", None)
        else:
            store["ptp_action_role"] = role
        self._drop_empty(serial)
        self._show_table_msg(f"{serial}: PTP 역할 {role}" + ("" if role != default else " (원본값)"))
        self._render_devices()
        self.sig_cameras_edited.emit(self.group["key"])

    def set_run(self, state, group_active, included, text=None):
        """included: 지금 돌고 있는 프로세스가 이 카드의 종류를 띄웠는가."""
        if group_active and not included:
            self.pill.set_state(STOPPED, "이번 기동에 미포함")
        else:
            self.pill.set_state(state, text)
        self.btn_start.setEnabled(not group_active and not sensor_config.missing_paths(self.group))
        self.btn_stop.setEnabled(group_active)
        if self._locked != group_active or self._included != included:
            self._locked = group_active
            self._included = included
            self._show_table_msg("")
            self._render_devices()
            self._on_select()

    def restore_splitter(self, state):
        if state:
            self.split.restoreState(state)


class SensorStageWidget(QWidget):
    """센서 기동 탭 전체."""

    sig_log = pyqtSignal(str, str)         # level, text
    sig_ready = pyqtSignal()               # 기동한 센서군이 전부 RUNNING
    sig_state = pyqtSignal()               # 상태 요약이 바뀜
    sig_go_record = pyqtSignal()           # [녹화 탭으로] 버튼

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.registry = sensor_discovery.load_registry()
        self.groups = {g["key"]: g for g in self.registry.get("groups", [])}
        self.cards = build_cards(self.registry)
        self.supervisor = sensor_launcher.SensorSupervisor(self.registry, self)
        self.supervisor.sig_state.connect(self._on_run_state)
        self.supervisor.sig_output.connect(self._on_output)
        self.supervisor.sig_progress.connect(self._on_progress)
        self.supervisor.sig_devices.connect(self._refresh_row_marks)
        self._progress = {}            # group_key -> (뜬 것, 기대) — "기동 중 5/15"
        self._ros = None
        self._stale_pubs = {}          # group_key -> 기동 직전에 있던 {(토픽, 발행자 gid)}
        self.card_widgets = {}
        self.pages = {}
        self._result = {}
        self._touched = set()          # 사용자가 직접 체크를 바꾼 카드 — 자동 체크가 덮지 않는다
        self._included = {}            # group_key -> 이번 기동에 포함된 subset 들
        self._launched = {}            # group_key -> {serial: 실행 중인 네임스페이스}
        self._announced_ready = False
        self._disc = None
        self._ip_worker = None
        self._after_discovery = None      # 다음 감지가 끝나면 할 일 (IP 할당 뒤 기동 이어가기)
        self._rescan_after = False
        self._build()
        self.refresh_discovery()

    # --- 공용 ---

    def _splitters(self):
        return self.cfg.setdefault("ui", {}).setdefault("splitters", {})

    def _overrides(self, group_key):
        return self.cfg.setdefault("sensors", {}).setdefault(group_key, {})

    def _cards_of(self, group_key):
        return [c for c in self.cards if c["group"]["key"] == group_key]

    def _active(self, group_key):
        return self.supervisor.procs[group_key].is_active()

    # --- UI ---

    def _build(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)

        bar = QFrame()
        bar.setObjectName("Toolbar")
        tb = QHBoxLayout(bar)
        tb.setContentsMargins(10, 8, 10, 8)
        self.btn_scan = QPushButton("↻  다시 감지")
        self.btn_scan.clicked.connect(self.refresh_discovery)
        self.lbl_scan = QLabel("")
        self.lbl_scan.setObjectName("Hint")
        self.btn_start_sel = QPushButton("▶  선택한 센서 기동")
        ui_theme.set_variant(self.btn_start_sel, "primary")
        self.btn_start_sel.clicked.connect(self.start_selected)
        self.btn_stop_all = QPushButton("■  전체 중지")
        ui_theme.set_variant(self.btn_stop_all, "danger")
        self.btn_stop_all.clicked.connect(self.stop_all)
        # 기동한 센서가 전부 올라오면 나타나는 버튼. 별도 배너 줄을 두지 않는다 — 작은 화면에서
        # 세로 한 줄이 아깝다.
        self.btn_go = QPushButton("✓  모두 실행 중 · 녹화로  →")
        ui_theme.set_variant(self.btn_go, "success")
        self.btn_go.clicked.connect(self.sig_go_record)
        self.btn_go.hide()
        self.lbl_scan.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        ui = self.cfg.setdefault("ui", {})
        self.chk_fix_ip = QCheckBox("IP 자동 맞춤")
        self.chk_fix_ip.setChecked(bool(ui.get("auto_fix_ip", True)))
        self.chk_fix_ip.setToolTip(
            "기동하기 전에, IP 가 없거나(169.254.x.x) 다른 서브넷이거나 다른 카메라와 겹치는 카메라에\n"
            "한 대씩 호스트 서브넷의 빈 주소를 주고 확인한 뒤 띄웁니다 (GVCP ForceIP, 전원 재인가 시 풀림).\n"
            "라이다를 IPv4 가 없는 NIC(USB 이더넷 어댑터 등)에 꽂았으면 그 NIC 를 link-local 로 잡습니다.")
        self.chk_fix_ip.toggled.connect(lambda on: ui.__setitem__("auto_fix_ip", on))
        tb.addWidget(self.btn_scan)
        tb.addWidget(self.lbl_scan, 1)
        tb.addWidget(self.chk_fix_ip)
        tb.addWidget(self.btn_go)
        tb.addWidget(self.btn_start_sel)
        tb.addWidget(self.btn_stop_all)
        outer.addWidget(bar)

        if not self.cards:
            warn = QLabel("sensors.yaml 을 찾지 못했습니다 — 센서 기동을 쓸 수 없습니다.\n"
                          "colcon build 후 다시 실행하거나 config/sensors.yaml 을 확인하세요.")
            warn.setStyleSheet(f"color:{ui_theme.ERR};")
            outer.addWidget(warn)
            outer.addStretch(1)
            self.btn_scan.setEnabled(False)
            self.btn_start_sel.setEnabled(False)
            return

        self.split = QSplitter(Qt.Horizontal)
        self.split.setChildrenCollapsible(False)

        # 왼쪽: 카드 목록
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 4, 0)
        lv.setSpacing(8)
        for card in self.cards:
            widget = SensorCard(card)
            widget.sig_clicked.connect(self.select_card)
            widget.sig_toggled.connect(self._on_card_toggled)
            self.card_widgets[card["key"]] = widget
            lv.addWidget(widget)
        hint = QLabel("카드를 누르면 오른쪽에 장비·설정·로그가 나옵니다.\n"
                      "체크한 센서만 [선택한 센서 기동]으로 띄웁니다.")
        hint.setObjectName("Hint")
        hint.setWordWrap(True)
        lv.addWidget(hint)
        lv.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(left)
        scroll.setMinimumWidth(220)
        self.split.addWidget(scroll)

        # 오른쪽: 카드별 상세
        self.stack = QStackedWidget()
        self.stack.setMinimumWidth(360)
        detail_state = self._load_state(SPLITTER_DETAIL)
        for card in self.cards:
            page = DetailPage(card, self._overrides(card["group"]["key"]), self.cfg.setdefault("ui", {}))
            page.sig_start.connect(self._start_from_page)
            page.sig_stop.connect(self.stop_group)
            page.sig_launch_edited.connect(self._sync_launch_field)
            page.sig_assign_ip.connect(self.assign_ips)
            page.sig_promote.connect(self.promote_defaults)
            page.sig_promote_inventory.connect(self.promote_inventory)
            page.sig_cameras_edited.connect(lambda _k: self.sig_state.emit())
            page.sig_splitter.connect(self._on_detail_splitter)
            page.restore_splitter(detail_state)
            self.pages[card["key"]] = page
            self.stack.addWidget(page)
        self.split.addWidget(self.stack)
        self.split.setStretchFactor(0, 0)
        self.split.setStretchFactor(1, 1)
        self.split.setSizes([270, 730])
        main_state = self._load_state(SPLITTER_MAIN)
        if main_state:
            self.split.restoreState(main_state)
        self.split.splitterMoved.connect(
            lambda *_: self._save_state(SPLITTER_MAIN, self.split.saveState()))
        outer.addWidget(self.split, 1)

        for group_key in self.groups:
            self._apply_run_state(group_key)
        self.select_card(self.cards[0]["key"])
        self._update_start_button()

    def attach_ros(self, worker):
        """라이브 보기(이름 짓기)가 쓸 ROS 연결. clip_gui 가 RosWorker 를 넘겨준다."""
        self._ros = worker
        for page in self.pages.values():
            page.live.attach(worker)

    def stale_publishers(self):
        """기동 중인 센서군이 기동 직전에 이미 있던 발행자들 — 완료 판정에서 세지 않는다."""
        out = set()
        for key, snapshot in self._stale_pubs.items():
            if self.supervisor.state(key) == STARTING:
                out |= snapshot
        return out

    # --- 기본값으로 저장 ---

    def _confirm(self, title, text, detail):
        from PyQt5.QtWidgets import QMessageBox
        box = QMessageBox(QMessageBox.Question, title, text, QMessageBox.Yes | QMessageBox.Cancel, self)
        box.setDetailedText(detail)
        box.setDefaultButton(QMessageBox.Cancel)
        return box.exec_() == QMessageBox.Yes

    @staticmethod
    def _short(path):
        return str(path).replace(str(Path.home()), "~")

    def promote_defaults(self, card_key):
        """설정 탭의 [기본값으로 저장] — 이 카드의 params + 런치 옵션 차이를 원본 파일에."""
        card = next(c for c in self.cards if c["key"] == card_key)
        group = self.groups[card["group"]["key"]]
        overrides = self._overrides(group["key"])
        scopes = [card["subset"] or sensor_config.NO_SUBSET, "launch"]
        plan = sensor_config.promote_plan(group, overrides, scopes)
        if not plan:
            self.sig_log.emit("GUI", "원본과 다른 값이 없습니다")
            return
        fmt = lambda v: "(없음 — 주석 처리)" if v is None else repr(v)      # noqa: E731
        detail = "\n\n".join(
            f"{self._short(step['file'])}\n" + "\n".join(f"  {k}: {fmt(old)} → {fmt(new)}"
                                                        for k, old, new in step["items"])
            for step in plan)
        count = sum(len(step["items"]) for step in plan)
        files = ", ".join(Path(step["file"]).name for step in plan)
        if not self._confirm("기본값으로 저장",
                             f"{count}개 값을 리포 원본에 씁니다 ({files}).\n"
                             "GUI 없이 띄워도 이 값이 기본이 됩니다. 원본은 ~/.config/dm_clip_gui/backups 에 "
                             "복사해 둡니다 (git 으로도 되돌릴 수 있음).\n\n자세히 보기에서 바뀌는 값을 확인하세요.",
                             detail):
            return
        for step, backup, error in sensor_config.promote_apply(group, overrides, plan):
            name = self._short(step["file"])
            if error:
                self.sig_log.emit("ERROR", f"기본값 저장 실패 {name}: {error}")
            else:
                self.sig_log.emit("OK", f"기본값 저장: {name} ({len(step['items'])}개, 백업 {self._short(backup)})")
        for page_card in self._cards_of(group["key"]):
            self.pages[page_card["key"]].reload_settings()
        self.sig_state.emit()

    def promote_inventory(self, card_key):
        """감지된 장비 탭의 [이름·역할을 인벤토리에 저장]."""
        card = next(c for c in self.cards if c["key"] == card_key)
        group = self.groups[card["group"]["key"]]
        overrides = self._overrides(group["key"])
        devices = (self._result.get(group["key"]) or {}).get("devices", [])
        path, _node, entries, changes = sensor_config.promote_inventory_plan(
            group, overrides, card["subset"], devices)
        if not changes:
            self.sig_log.emit("GUI", "인벤토리와 다른 이름 · 역할이 없습니다")
            return
        detail = f"{self._short(path)}\n" + "\n".join(f"  {sn}: {what}" for sn, what in changes)
        if not self._confirm("인벤토리에 저장",
                             f"카메라 {len(changes)}대의 이름 · 역할 · IP 를 리포 인벤토리에 씁니다 "
                             f"({Path(path).name}, 모두 {len(entries)}대).\n"
                             "원본은 ~/.config/dm_clip_gui/backups 에 복사해 둡니다.", detail):
            return
        try:
            path, backup, changes = sensor_config.promote_inventory_apply(group, overrides, card["subset"], devices)
        except Exception as exc:                                  # noqa: BLE001
            self.sig_log.emit("ERROR", f"인벤토리 저장 실패: {exc}")
            return
        self.sig_log.emit("OK", f"인벤토리 저장: {self._short(path)} ({len(changes)}대"
                                + (f", 백업 {self._short(backup)})" if backup else ")"))
        for page_card in self._cards_of(group["key"]):
            self.pages[page_card["key"]]._render_devices()
        self.sig_state.emit()

    # --- IP 할당 ---

    def _ip_tasks(self, card):
        """이 카드에서 이대로는 못 여는 카메라마다 줄 주소를 정해 과제로. 정한 주소는 바로 기록한다
        (다음 카메라가 그 주소를 피하고, 인벤토리 사본에도 실린다)."""
        group_key = card["group"]["key"]
        group = self.groups[group_key]
        sub = next((x for x in sensor_config.subsets_of(group) if x["key"] == card["subset"]), None)
        if sub is None or not sub.get("inventory"):
            return []
        overrides = self._overrides(group_key)
        devices = (self._result.get(group_key) or {}).get("devices", [])
        tasks = []
        for dev in card_devices(card, devices):
            if dev["state"] != SUBNET or not dev.get("mac"):
                continue
            addr = net_tools.nic_ipv4(dev["nic"])
            if not addr:
                continue
            host_ip, prefix = addr
            ip = sensor_config.pick_force_ip(group, overrides, sub, dev["identity"], devices,
                                              host_ip, prefix)
            if not ip:
                self.sig_log.emit("ERROR", f"{dev['identity']}: {host_ip}/{prefix} 에 빈 주소가 없습니다")
                continue
            mask = ".".join(str((((0xFFFFFFFF << (32 - prefix)) & 0xFFFFFFFF) >> s_) & 255)
                            for s_ in (24, 16, 8, 0))
            mine = sensor_config.camera_overrides(overrides).setdefault(dev["identity"], {})
            mine.update({"force_ip_address": ip, "force_ip_subnet_mask": mask})
            tasks.append({"serial": dev["identity"], "mac": dev["mac"], "nic": dev["nic"],
                          "ip": ip, "mask": mask, "label": card["label"]})
        return tasks

    def assign_ips(self, card_key):
        """[IP 할당] 버튼 (라이다 카드에선 [NIC 설정])."""
        card = next(c for c in self.cards if c["key"] == card_key)
        if self._active(card["group"]["key"]):
            self.sig_log.emit("WARN", "실행 중에는 IP 를 바꾸지 않습니다 — 중지 후 다시 누르세요")
            return
        if self.pages[card_key].fix_nic and not self.pages[card_key].cameras_editable:
            self._run_nic_fix(card_key, self.pages[card_key].fix_nic, None)
            return
        tasks = self._ip_tasks(card)
        if tasks:
            self._run_ip_tasks(tasks, [card_key], None)

    def _run_ip_tasks(self, tasks, card_keys, then):
        """한 대씩 ForceIP → 다시 감지 → then() (기동 이어가기 등)."""
        if self._ip_worker is not None and self._ip_worker.isRunning():
            return
        for key in card_keys:
            self.pages[key].btn_ip.setEnabled(False)
            self.pages[key].btn_ip.setText("할당 중…")
        self.btn_start_sel.setEnabled(False)
        self.sig_log.emit("GUI", "IP 할당: " + ", ".join(f"{t['serial']} → {t['ip']}" for t in tasks))
        progress_page = self.pages[card_keys[0]]
        self._ip_worker = ForceIpWorker(tasks)
        self._ip_worker.sig_progress.connect(lambda text: progress_page._show_table_msg(text))
        self._ip_worker.sig_done.connect(lambda results: self._on_ip_done(card_keys, results, then))
        self._ip_worker.start()

    def _on_ip_done(self, card_keys, results, then):
        ok = [f"{sn} → {ip}" for sn, ip, good in results if good]
        bad = [f"{sn} ({ip})" for sn, ip, good in results if not good]
        if ok:
            self.sig_log.emit("OK", "IP 할당 완료: " + ", ".join(ok))
        if bad:
            self.sig_log.emit("ERROR", "IP 할당 실패 — 전원/케이블 확인 후 다시: " + ", ".join(bad))
        for key in card_keys:
            page = self.pages[key]
            page.btn_ip.setText("IP 할당")
            page._show_table_msg(("완료: " + ", ".join(ok) if ok else "") +
                                 ("  실패: " + ", ".join(bad) if bad else ""), error=bool(bad))
        self.sig_state.emit()
        self._after_discovery = then          # 새 주소로 다시 감지한 뒤에 이어서 (예: 기동)
        self.refresh_discovery(force=True)

    # --- 칸 크기 저장 ---

    def _load_state(self, name):
        text = self._splitters().get(name)
        return QByteArray.fromBase64(text.encode()) if text else None

    def _save_state(self, name, state):
        self._splitters()[name] = bytes(state.toBase64()).decode()

    def _on_detail_splitter(self, state):
        """상세 칸 비율은 카드마다가 아니라 하나로 — 카드를 옮겨 다녀도 레이아웃이 그대로다."""
        self._save_state(SPLITTER_DETAIL, state)
        for page in self.pages.values():
            if page is not self.stack.currentWidget():
                page.restore_splitter(state)

    # --- 카드 ---

    def select_card(self, key):
        for k, widget in self.card_widgets.items():
            widget.set_selected(k == key)
        page = self.pages.get(key)
        if page:
            self.stack.setCurrentWidget(page)

    def _on_card_toggled(self, key, _on):
        self._touched.add(key)
        self._update_start_button()

    def _checked_subsets(self, group_key):
        return [c["subset"] for c in self._cards_of(group_key)
                if self.card_widgets[c["key"]].check.isChecked()]

    def _update_start_button(self):
        if not self.cards:
            return
        self.btn_stop_all.setEnabled(any(self._active(k) for k in self.groups))
        pending = [c for c in self.cards
                   if self.card_widgets[c["key"]].check.isChecked()
                   and not self._active(c["group"]["key"])]
        self.btn_start_sel.setEnabled(bool(pending))
        self.btn_start_sel.setText(f"▶  선택한 센서 기동 ({len(pending)})" if pending
                                   else "▶  선택한 센서 기동")

    def _sync_launch_field(self, group_key, field_key):
        for card in self._cards_of(group_key):
            self.pages[card["key"]].sync_launch_field(field_key)
        self.sig_state.emit()

    # --- 감지 ---

    def refresh_discovery(self, force=False):
        if not self.cards:
            return
        if self._disc and self._disc.isRunning():
            if force:
                self._rescan_after = True      # IP 를 바꾼 뒤라면 지금 도는 감지는 옛 결과다
            return
        self.btn_scan.setEnabled(False)
        self.btn_scan.setText("감지 중…")
        self.lbl_scan.setText("GVCP 브로드캐스트 · TCP 프로브 · NCOM 수신")
        self._disc = DiscoveryWorker(self.registry)
        self._disc.sig_done.connect(self._on_discovered)
        self._disc.start()

    def _on_discovered(self, result):
        if self._rescan_after:
            self._rescan_after = False
            self._disc = None
            self.refresh_discovery()
            return
        self._result = result
        self.btn_scan.setEnabled(True)
        self.btn_scan.setText("↻  다시 감지")

        total = kinds = 0
        for card in self.cards:
            group_key = card["group"]["key"]
            res = result.get(group_key) or {"devices": [], "probe": "", "error": None}
            devices = card_devices(card, res["devices"])
            spare = res.get("candidates")
            self.card_widgets[card["key"]].set_devices(devices, res["probe"], res["error"], spare)
            self.pages[card["key"]].set_devices(devices, res["probe"], res["error"], spare)
            total += len(devices)
            kinds += bool(devices)
            # 보이는 종류는 기동 대상으로 기본 체크. 사용자가 손댄 카드와 실행 중인 건 그대로 둔다.
            widget = self.card_widgets[card["key"]]
            if card["key"] not in self._touched and not self._active(group_key):
                widget.check.blockSignals(True)
                widget.check.setChecked(bool(devices))
                widget.check.blockSignals(False)

        # 모델 패턴에 안 걸린 GigE 장비 (예: 다른 회사 카메라) — 카드가 없으니 숫자로만 알린다
        stray = sum(1 for key, group in self.groups.items() if sensor_config.subsets_of(group)
                    for d in (result.get(key) or {}).get("devices", []) if d["subset"] is None)

        text = f"{time.strftime('%H:%M:%S')} 감지 · {kinds}종 {total}대"
        if stray:
            text += f" · 미분류 GigE {stray}대"
        self.lbl_scan.setText(text)
        self.lbl_scan.setToolTip(text)
        self.sig_log.emit("GUI", "센서 감지: " + "  ·  ".join(
            f"{c['label']} {len(card_devices(c, (result.get(c['group']['key']) or {}).get('devices', [])))}대"
            for c in self.cards))
        self._update_start_button()
        self.sig_state.emit()
        if self._after_discovery is not None:
            then, self._after_discovery = self._after_discovery, None
            then()

    # --- 기동 ---

    def _expectations(self, group_key, subsets):
        """[(토픽 정규식, 최소 대수)] — 보이는 대수만큼 떠야 '기동 완료'.

        서브넷이 안 맞는 카메라는 열 수 없으니 세지 않는다.
        """
        group = self.groups[group_key]
        devices = [d for d in (self._result.get(group_key) or {}).get("devices", [])
                   if d["state"] == OK]
        subs = sensor_config.subsets_of(group)
        if not subs:
            pattern = group.get("topic_regex")
            return [(pattern, max(1, len(devices)))] if pattern else []
        return [(s["topic_regex"], max(1, sum(1 for d in devices if d["subset"] == s["key"])))
                for s in subs if s["key"] in subsets and s.get("topic_regex")]

    def _start_group(self, group_key, subsets):
        group = self.groups[group_key]
        overrides = self._overrides(group_key)
        subs = sensor_config.subsets_of(group)
        devices = (self._result.get(group_key) or {}).get("devices", [])

        if subs:
            # 인벤토리로 띄우는 종류는 "지금 보이는 카메라"만 띄운다. 한 대도 안 보이면 뺀다 —
            # 넣어 봐야 저장된 카메라들의 노드가 떠서 죽을 뿐이다.
            included = sensor_config.included_devices(group, overrides, devices)
            for card in self._cards_of(group_key):
                sub = next(s for s in subs if s["key"] == card["subset"])
                if card["subset"] in subsets and sub.get("inventory") and not included.get(card["subset"]):
                    subsets = [k for k in subsets if k != card["subset"]]
                    self.sig_log.emit("WARN", f"{card['label']}: 감지된 장비가 없어 이번 기동에서 뺍니다")
                left_out = len(card_devices(card, devices)) - len(included.get(card["subset"], []))
                if card["subset"] in subsets and left_out:
                    self.sig_log.emit("WARN", f"{card['label']}: 못 여는 카메라 {left_out}대는 빼고 띄웁니다 "
                                              "(IP 안 맞음은 툴바의 IP 자동 맞춤, 업데이터 모드는 전원 재인가)")
            if not subsets:
                self.sig_log.emit("WARN", f"{group['label']}: 띄울 장비가 없습니다")
                return
            overrides["subsets"] = {s["key"]: s["key"] in subsets for s in subs}
        elif not devices:
            self.sig_log.emit("WARN", f"{group['label']}: 감지된 장비가 없는데 기동합니다 "
                                      "— 완료 판정은 실패할 수 있습니다")

        self._included[group_key] = set(subsets)
        # 라이브 보기가 시리얼 → 실행 중인 네임스페이스를 알아야 한다 (이름을 바꿔도 지금 건 그대로)
        launched = {}
        if subs:
            included = sensor_config.included_devices(group, overrides, devices)
            for sub in subs:
                if sub["key"] in subsets and sub.get("inventory"):
                    for dev in included.get(sub["key"], []):
                        launched[dev["identity"]] = sensor_config.camera_entry(
                            sub, dev["identity"], overrides)["namespace"]
        self._launched[group_key] = launched
        self._announced_ready = False
        # 지금 그래프에 있는 발행자는 이번 기동 것이 아니다 (앞 실행의 죽은 노드가 임대 시간 동안 남는다)
        self._stale_pubs[group_key] = set()
        if self._ros is not None:
            try:
                self._stale_pubs[group_key] = self._ros.publisher_gids()
            except Exception:                                  # noqa: BLE001
                pass
        labels = ", ".join(c["label"] for c in self._cards_of(group_key) if c["subset"] in subsets)
        self.sig_log.emit("GUI", f"기동: {labels}")
        self.supervisor.start(group_key, overrides, self._expectations(group_key, subsets), devices)

    def _run_nic_fix(self, card_key, iface, then):
        """라이다 NIC 를 link-local 로 잡고 → 다시 감지 → then()."""
        if getattr(self, "_nic_worker", None) is not None and self._nic_worker.isRunning():
            return
        page = self.pages[card_key]
        page.btn_ip.setEnabled(False)
        page.btn_ip.setText("설정 중…")
        self.btn_start_sel.setEnabled(False)
        self.sig_log.emit("GUI", f"{iface} 를 라이다용 link-local 로 잡습니다 (NetworkManager)")
        self._nic_worker = NicSetupWorker(iface)
        self._nic_worker.sig_done.connect(lambda ok, msg: self._on_nic_done(card_key, ok, msg, then))
        self._nic_worker.start()

    def _on_nic_done(self, card_key, ok, message, then):
        self.sig_log.emit("OK" if ok else "ERROR", ("NIC 설정: " if ok else "NIC 설정 실패: ") + message)
        self.pages[card_key]._show_table_msg(message, error=not ok)
        self._after_discovery = then
        self.refresh_discovery(force=True)

    def _lidar_fix(self, cards):
        """기동할 라이다 카드 중 PC 쪽 NIC 를 잡아야 붙는 게 있으면 (카드 키, NIC)."""
        for card in cards:
            page = self.pages[card["key"]]
            if card["group"].get("discovery", {}).get("kind") == "ouster_probe" and page.fix_nic:
                return card["key"], page.fix_nic
        return None

    def _fix_ip_then(self, cards, then):
        """IP 자동 맞춤이 켜져 있으면 기동 전에 라이다 NIC · 카메라 IP 를 맞춘 뒤 then()."""
        if not self.chk_fix_ip.isChecked():
            return then()
        lidar = self._lidar_fix(cards)
        if lidar:
            self.sig_log.emit("GUI", f"라이다가 {lidar[1]} 에 있는데 PC 쪽 IPv4 가 없어 먼저 NIC 를 잡습니다")
            return self._run_nic_fix(lidar[0], lidar[1], lambda: self._fix_camera_ips_then(cards, then))
        self._fix_camera_ips_then(cards, then)

    def _fix_camera_ips_then(self, cards, then):
        tasks, keys = [], []
        for card in cards:
            found = self._ip_tasks(card)
            if found:
                tasks += found
                keys.append(card["key"])
        if not tasks:
            return then()
        self.sig_log.emit("GUI", f"기동 전에 IP 가 안 맞는 카메라 {len(tasks)}대를 맞춥니다")
        self._run_ip_tasks(tasks, keys, then)

    def start_selected(self):
        cards = [c for c in self.cards if self.card_widgets[c["key"]].check.isChecked()
                 and not self._active(c["group"]["key"])]
        self._fix_ip_then(cards, self._start_selected_now)

    def _start_selected_now(self):
        started = []
        for group_key in self.groups:
            subsets = self._checked_subsets(group_key)
            if subsets and not self._active(group_key):
                self._start_group(group_key, subsets)
                started.append(group_key)
        # 보고 있던 카드가 이번 기동에 없으면, 기동한 첫 카드로 옮겨 로그가 보이게 한다
        current = self.stack.currentWidget()
        if started and current is not None and current.group["key"] not in started:
            for card in self.cards:
                if card["group"]["key"] in started and card["subset"] in self._included[card["group"]["key"]]:
                    self.select_card(card["key"])
                    break

    def _start_from_page(self, card_key):
        """상세 화면의 [기동] — 이 카드는 포함시키고, 같은 프로세스의 체크된 카드도 같이 띄운다."""
        card = next(c for c in self.cards if c["key"] == card_key)
        widget = self.card_widgets[card_key]
        self._touched.add(card_key)
        if not widget.check.isChecked():
            widget.check.setChecked(True)
        group_key = card["group"]["key"]
        cards = [c for c in self._cards_of(group_key) if self.card_widgets[c["key"]].check.isChecked()]
        self._fix_ip_then(cards, lambda: self._start_group(group_key, self._checked_subsets(group_key)))

    def stop_group(self, group_key):
        self.supervisor.stop(group_key)

    def stop_all(self):
        self.supervisor.stop_all()

    # --- 상태 ---

    def set_topics(self, topic_names):
        """clip_gui 가 주기적으로 현재 토픽 목록을 넘겨준다 (기동 완료 판정용)."""
        self.supervisor.set_topics(topic_names)

    def has_starting(self):
        return any(self.supervisor.state(k) == STARTING for k in self.groups)

    def _apply_run_state(self, group_key):
        state = self.supervisor.state(group_key)
        active = self._active(group_key)
        included = self._included.get(group_key, set())
        if not active:
            self._launched.pop(group_key, None)
            self._progress.pop(group_key, None)
        # 순차 기동은 몇 분이 걸린다 — 몇 대째인지 보여야 멈춘 건지 도는 건지 안다
        text = None
        if state in (STARTING, FAILED) and active and self._progress.get(group_key):
            have, want = self._progress[group_key]
            text = f"{'기동 중' if state == STARTING else '실패'} {have}/{want}"
        marks = self._row_marks(group_key)
        for card in self._cards_of(group_key):
            self.pages[card["key"]].set_row_marks(marks)
        if state == RUNNING and active:
            dead = sum(1 for st, _ in marks.values() if st == FAILED)
            if dead:
                text = f"실행 중 · {dead}대 실패"
        for card in self._cards_of(group_key):
            self.pages[card["key"]].set_launched(self._launched.get(group_key, {}))
        for card in self._cards_of(group_key):
            inc = card["subset"] in included
            widget = self.card_widgets[card["key"]]
            widget.set_run(state if (inc or not active) else STOPPED, text if inc else None)
            widget.set_locked(active)
            self.pages[card["key"]].set_run(state, active, inc, text)

    def _row_marks(self, group_key):
        """{시리얼: (상태, 설명)} — 장비 하나하나가 지금 어떤지."""
        proc = self.supervisor.procs[group_key]
        if not proc.is_active():
            return {}
        devices = (self._result.get(group_key) or {}).get("devices", [])
        launched = self._launched.get(group_key, {})
        marks = {}
        if not launched:
            # 라이다 · GNSS: 장비 하나 = 센서군 하나
            state = proc.state if proc.state in (RUNNING, STARTING, FAILED) else STOPPED
            return {d["identity"]: (state, "") for d in devices}
        for dev in devices:
            serial = dev["identity"]
            ns = launched.get(serial)
            if ns is None:
                marks[serial] = (EXCLUDED, "못 여는 장비라 뺐음" if dev["state"] != OK else "")
            elif ns in proc.ns_dead:
                marks[serial] = (FAILED, "노드가 죽음 — 런치 로그 확인")
            elif ns in proc.ns_seen and ns not in proc.ns_present:
                marks[serial] = (FAILED, "토픽이 사라짐")
            elif ns in proc.ns_ready or (not proc.ready_log and ns in proc.ns_present):
                marks[serial] = (RUNNING, f"/{ns}/")
            elif ns in proc.ns_logged:
                marks[serial] = (STARTING, "")
            elif proc.state == FAILED:
                marks[serial] = (FAILED, "제한 시간 안에 안 뜸")
            else:
                marks[serial] = (WAITING, "")
        return marks

    def _refresh_row_marks(self, group_key):
        self._apply_run_state(group_key)

    def _on_progress(self, group_key, have, want):
        self._progress[group_key] = (have, want)
        if self.supervisor.state(group_key) in (STARTING, FAILED):
            self._apply_run_state(group_key)
            self.sig_state.emit()

    def _on_run_state(self, group_key, state):
        self._apply_run_state(group_key)
        labels = ", ".join(c["label"] for c in self._cards_of(group_key)
                           if c["subset"] in self._included.get(group_key, set()))
        self.sig_log.emit("ERROR" if state == FAILED else "GUI",
                          f"{labels or group_key}: {RUN_TEXT.get(state, state)}")
        self._update_start_button()

        active = [k for k in self.groups if self._active(k)]
        all_up = bool(active) and all(self.supervisor.state(k) == RUNNING for k in active)
        self.btn_go.setVisible(all_up)
        if all_up and not self._announced_ready:
            self._announced_ready = True
            self.sig_ready.emit()
        self.sig_state.emit()

    def _on_output(self, group_key, line):
        for card in self._cards_of(group_key):
            self.pages[card["key"]].log.appendPlainText(line)

    def device_at(self, ip):
        """감지된 센서 중 이 IP 의 장비 (녹화 탭 네트워크 표가 이름을 붙이는 데 쓴다)."""
        for res in self._result.values():
            for dev in (res or {}).get("devices", []):
                if dev["ip"] == ip:
                    return dev
        return None

    def describe_device(self, dev, live_name=None):
        """'camera1 · BFS' / 'thermal1 · A70' / 'Ouster LiDAR OS-2-128' / 'GNSS RT2000'

        카메라는 떠 있는 드라이버의 네임스페이스(live_name)가 있으면 그것, 없으면 GUI 설정·인벤토리로
        정해질 이름을 쓴다 — 녹화되는 토픽 이름과 같은 이름이 보여야 한다.
        """
        group = self.groups.get(dev.get("group"))
        sub = next((s for s in sensor_config.subsets_of(group)
                    if s["key"] == dev.get("subset")), None) if group else None
        if sub and sub.get("inventory"):
            name = live_name or sensor_config.camera_entry(
                sub, dev["identity"], self._overrides(dev["group"]))["namespace"]
            return f"{name} · {sub.get('short_label') or dev['subset_label']}"
        # 시리얼은 표 칸을 잡아먹으니 툴팁에만 (녹화 탭이 따로 보여준다)
        return f"{dev['subset_label']} {dev.get('model') or ''}".strip()

    def namespace_labels(self):
        """{네임스페이스: 종류 라벨} — 보이는 카메라가 어떤 이름으로 뜨는지 (미리보기 타일용)."""
        out = {}
        for key, res in self._result.items():
            group = self.groups.get(key)
            if not group:
                continue
            for dev in (res or {}).get("devices", []):
                sub = next((s for s in sensor_config.subsets_of(group)
                            if s["key"] == dev.get("subset")), None)
                if sub and sub.get("inventory"):
                    entry = sensor_config.camera_entry(sub, dev["identity"], self._overrides(key))
                    out[entry["namespace"]] = dev["subset_label"]
        return out

    def namespace_serials(self):
        """{네임스페이스: 시리얼} — 미리보기 타일 순서를 이름이 아니라 시리얼로 기억하려고.

        지금 실행 중인 이름(= 지금 토픽 이름)이 우선이고, 없으면 다음 기동 때 쓸 이름.
        """
        out = {}
        for key, res in self._result.items():
            group = self.groups.get(key)
            if not group:
                continue
            for dev in (res or {}).get("devices", []):
                sub = next((s for s in sensor_config.subsets_of(group)
                            if s["key"] == dev.get("subset")), None)
                if sub and sub.get("inventory"):
                    out[sensor_config.camera_entry(sub, dev["identity"], self._overrides(key))
                        ["namespace"]] = dev["identity"]
        for mapping in self._launched.values():
            for serial, namespace in mapping.items():
                out[namespace] = serial
        return out

    def thermal_temperature_scale(self):
        """A70 IRFormat → mono16 한 칸이 몇 켈빈인가 (10mK → 0.01). 온도 포맷이 아니면 None."""
        for group in self.groups.values():
            scale = sensor_config.temperature_scale(group, self._overrides(group["key"]))
            if scale:
                return scale
        return None

    def summary(self):
        return self.supervisor.summary()
