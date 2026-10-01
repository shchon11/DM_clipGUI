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

import copy
import math
import re
import time
from pathlib import Path

import numpy as np

import yaml
from PyQt5.QtCore import (QByteArray, QEvent, QItemSelectionModel, QObject, QSize, Qt, QThread, QTimer,
                          pyqtSignal)
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
import sync_check
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


class SensorTimeWorker(QThread):
    """라이다 센서의 시각 상태 (/api/v1/time) 를 한 번 읽는다 — 읽기만 한다."""

    sig_done = pyqtSignal(object)

    def __init__(self, host):
        super().__init__()
        self.host = host

    def run(self):
        self.sig_done.emit(sensor_discovery.ouster_time_status(self.host))


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
                # A70 은 새 IP 로 네트워크를 다시 올리는 데 Blackfly(1~2초)보다 훨씬 오래 걸린다. 6초 만에 포기하고
                # 한 번 더 ForceIP 를 보내면 재시작 중인 카메라를 또 재시작시켜, 결국 '실패' 로 끝나고 감지에서도
                # 빠졌다 (2026-09-22 thermal1 — 1분쯤 뒤 192.168.1.12 로 멀쩡히 대답). 보이는 즉시 끝나므로
                # 넉넉히 잡아도 정상일 때는 느려지지 않는다.
                settle = task.get("settle_s", 6.0)
                if settle > 6.0:
                    self.sig_progress.emit(f"{task['label']} {task['serial']}: 새 IP 로 다시 뜨기를 기다립니다 "
                                           f"(최대 {settle:.0f}초)…")
                deadline = time.time() + settle
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
        self.name_label = QLabel("이름")
        row.addWidget(self.name_label)
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


# ISP 자동 맞추기의 밝기 목표 — 8비트 영상의 밝기 중앙값(0~1). 중간 회색 근처.
AUTO_TARGET = 0.40


def _condition_keys(condition):
    """enabled_when / relevant_when 이 보는 키들 (목록이면 OR 의 모든 갈래)."""
    if isinstance(condition, list):
        return {k for c in condition for k in _condition_keys(c)}
    return set((condition or {}).keys())


class TuningPanel(QWidget):
    """[ISP 튜닝] 탭 — 카메라 한 대를 띄워 놓고 ISP · 노출 칸을 바꾸면 그 자리에서 카메라에 들어간다.

    설정 탭(전역 설정)과 따로 논다. 여기서 바꾼 값은 보고 있는 카메라에만 들어가고, [모든 카메라에 적용] 을
    누르면 설정 탭(전역 설정 — 다음 기동)에 저장하면서 지금 떠 있는 이 종류의 카메라 전부에 바로 넣는다 (재기동 없음,
    2026-09-22 전: 설정에만 저장해 리그의 다른 카메라는 재기동 전까지 옛 값이었다). [되돌리기] 는 전역 값으로 돌려
    카메라에도 다시 넣는다.
    카메라 노드는 camera.* 파라미터를 실행 중에 받는 즉시 카메라에 쓴다 (OnSetControlParameters).
    픽셀 포맷은 실행 중에 못 바꾸므로 Bayer 로 떠 있으면 RGB 출력 전용 칸은 잠근다.
    올릴 칸은 레지스트리 subset 의 tuning: {panels, sections} 이 정한다.
    """

    sig_start = pyqtSignal(str)                 # serial — 이 카메라만 기동
    sig_live = pyqtSignal(str, str, object)     # serial, 키, 값 — 실행 중인 카메라에 넣기
    sig_promote = pyqtSignal(dict)              # {키: 값} — 설정 탭(전역 설정)에 덮어쓰기
    sig_restart = pyqtSignal(str)               # serial — 튜닝 카메라 재기동 (재기동해야 들어가는 값 적용)

    def __init__(self, page):
        super().__init__()
        self.page = page
        self.group, self.subset = page.group, page.subset
        conf = self.subset.get("tuning") or {}
        self.fields = [f for f in sensor_config.fields_for(self.group, self.subset["key"])
                       if f["key"].startswith("camera.")
                       and (f.get("panel") in (conf.get("panels") or [])
                            or f.get("section") in (conf.get("sections") or []))]
        self.rows = {}               # 키 -> (field, editor, label)
        self.baseline = {}           # 키 -> 전역 설정의 값 (반영 · 되돌리기 기준)
        self.global_values = {}      # 이 종류의 주요 설정 전부의 전역 값 (픽셀 포맷 등 조건 판정용)
        self.serial = None
        self.running_ns = None
        self._pending = {}           # 키 -> 값. 스핀을 돌리는 동안 매 칸 보내지 않게 모았다가 보낸다
        # 사용자가 이 탭에서 직접 바꾼 칸. 변경 · 반영 · 되돌리기는 이것만 본다 — 전역 값이 없는 칸(원본에 없는 노출
        # 시간)은 스핀 최소값(10 µs)이 보이는데, 그걸 '바뀐 값' 으로 쳐서 전역 설정에 반영하면 안 된다.
        self.touched = set()
        self._retried = set()
        self._debounce = QTimer(self, singleShot=True, interval=150, timeout=self._flush)
        # 카메라가 막 떴을 때는 영상 토픽이 아직 없다 — 생길 때까지 1초마다 다시 붙는다
        self._live_retry = QTimer(self, interval=1000, timeout=self._retry_live)
        self._reapply = False        # 재기동 뒤 영상이 나오면 튜닝 값(반영 안 한 것)을 다시 넣는다
        # 자동 맞추기 — 값을 넣고, 새 프레임을 기다려 재고, 고친다. 제너레이터가 '기다릴 조건' 을 내놓는다.
        self._job = None
        self._job_wait = None
        self._job_timer = QTimer(self, interval=120, timeout=self._job_tick)
        self.restart_keys = {f["key"] for f in self.fields
                             if sensor_config.restart_only(self.group, self.subset["key"], f)}

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)
        bar = QHBoxLayout()
        bar.addWidget(QLabel("카메라"))
        # 이 탭의 값 칸만 마우스 휠로 바로 바꿔 볼 수 있다 (칸을 눌러 포커스를 준 뒤). 다른 곳은 ui_theme.lock_wheel 이 막는다
        ui_theme.allow_wheel(self)
        self.combo = QComboBox()
        self.combo.setMinimumWidth(220)
        self.combo.currentIndexChanged.connect(lambda _i: self.refresh_target())
        ui_theme.allow_wheel(self.combo, False)      # 튜닝할 카메라 고르기는 휠로 안 바뀌게
        guard_wheel(self.combo)
        bar.addWidget(self.combo)
        self.btn_start = QPushButton("▶  이 카메라만 기동")
        ui_theme.set_variant(self.btn_start, "primary")
        self.btn_start.setToolTip("고른 카메라 한 대만 띄웁니다 (열화상 · 다른 카메라는 안 뜸). 멈출 때는 위의 [■ 중지].")
        self.btn_start.clicked.connect(lambda: self.serial and self.sig_start.emit(self.serial))
        bar.addWidget(self.btn_start)
        self.status = QLabel("")
        self.status.setObjectName("Hint")
        self.status.setWordWrap(True)
        self.status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        bar.addWidget(self.status, 1)
        self.btn_restart = QPushButton("⟳  재기동")
        self.btn_restart.setToolTip("튜닝 중인 카메라를 내렸다 다시 올립니다 — 🔒 칸(재기동해야 들어가는 값)과 설정 탭의 값이 "
                                    "이때 들어갑니다. 올라오면 이 탭에서 바꾼 값(전역에 반영 안 한 것)을 다시 넣습니다.")
        self.btn_restart.clicked.connect(lambda: self.serial and self.sig_restart.emit(self.serial))
        bar.addWidget(self.btn_restart)
        outer.addLayout(bar)

        auto = QHBoxLayout()
        auto.addWidget(QLabel("자동 맞추기"))
        self.auto_buttons = {}
        for name, text, tip in (
                ("wb", "화이트밸런스", "화면 가운데 절반의 R · G · B 평균이 같아지게 R · B 비율을 맞춥니다 (WB auto 는 Off 로). "
                                    "흰 판이나 회색 판을 가운데에 두면 정확합니다."),
                ("exposure", "노출", "화면 밝기 중앙값이 중간 회색이 되게 노출 시간을 맞춥니다 (노출 auto 는 Off 로). "
                                   "트리거 주기의 90% 를 넘지 않습니다."),
                ("gamma", "감마", "노출은 그대로 두고 화면 밝기 중앙값이 중간 회색이 되게 감마를 찾습니다 (감마 보정 켬)."),
                ("black", "블랙 레벨", "가장 어두운 곳(하위 1%)이 0 에 붙어 잘리지 않고 살짝 뜨게 블랙 레벨을 맞춥니다. "
                                      "그림자가 있는 장면이어야 합니다 — 렌즈를 가리면 가장 정확합니다.")):
            button = QPushButton(text)
            button.setToolTip(tip + "\n결과는 그 칸의 값을 덮어쓰고 바로 카메라에 들어갑니다 (전역 반영 대상에도 포함).")
            button.clicked.connect(lambda _=False, n=name: self.start_auto(n))
            auto.addWidget(button)
            self.auto_buttons[name] = button
        self.btn_cancel = QToolButton()
        self.btn_cancel.setText("그만")
        self.btn_cancel.clicked.connect(lambda: self._finish_job("중간에 멈췄습니다 — 지금 값 그대로"))
        self.btn_cancel.hide()
        auto.addWidget(self.btn_cancel)
        self.auto_status = QLabel("")
        self.auto_status.setObjectName("Hint")
        self.auto_status.setWordWrap(True)
        self.auto_status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        auto.addWidget(self.auto_status, 1)
        outer.addLayout(auto)

        split = QSplitter(Qt.Horizontal)
        self.live = LiveView()
        for widget in (self.live.name_label, self.live.name, self.live.btn_name, self.live.hint):
            widget.hide()                                     # 이름 짓기는 감지된 장비 탭에서
        split.addWidget(self.live)
        body = QWidget()
        col = QVBoxLayout(body)
        col.setContentsMargins(0, 0, 6, 0)
        self.section = SettingsSection(
            "튜닝할 값", "바꾸는 즉시 위에서 고른 카메라에 들어갑니다 (재기동 없음). 맞으면 아래 [모든 카메라에 적용] — "
            "떠 있는 카메라 전부에 바로 넣고 설정(다음 기동)에도 저장합니다. '지정' 을 끈 칸은 카메라에 아무것도 보내지 "
            "않습니다.", primary=True)
        last = None
        for field in self.fields:
            if field.get("section") != last:
                self.section.add_subhead(field.get("section") or "")
                last = field.get("section")
            self._add_row(field)
        col.addWidget(self.section)
        col.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(body)
        split.addWidget(scroll)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        outer.addWidget(split, 1)

        foot = QHBoxLayout()
        self.result = QLabel("")
        self.result.setObjectName("Hint")
        self.result.setWordWrap(True)
        self.result.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        foot.addWidget(self.result, 1)
        self.btn_revert = QPushButton("되돌리기")
        self.btn_revert.setToolTip("튜닝 칸을 전역 설정 값으로 돌리고, 실행 중이면 카메라에도 다시 넣습니다.")
        self.btn_revert.clicked.connect(self.revert)
        foot.addWidget(self.btn_revert)
        self.btn_promote = QPushButton("모든 카메라에 적용")
        ui_theme.set_variant(self.btn_promote, "primary")
        self.btn_promote.setToolTip("여기서 바꾼 값을 이 종류의 모든 카메라에 적용합니다.\n"
                                    "· 설정 탭(전역 설정)에 저장 — 다음 기동부터 리그 전체에 들어갑니다.\n"
                                    "· 지금 떠 있는 카메라가 있으면 전부에 바로 넣습니다 (재기동 없음).\n"
                                    "바로 못 넣은 값이 있으면 위에 재기동 안내와 [⟳ 지금 재기동] 이 뜹니다.\n"
                                    "리포 원본 파일까지 바꾸려면 설정 탭의 [기본값으로 저장].")
        self.btn_promote.clicked.connect(self.promote)
        foot.addWidget(self.btn_promote)
        outer.addLayout(foot)
        self.refresh_baseline(reset=True)

    # --- 칸 ---

    def _add_row(self, field):
        editor = FieldEditor(field, None)
        editor.sig_changed.connect(lambda k=field["key"]: self._on_edit(k))
        label = QLabel(field.get("label") or field["key"])
        label.setObjectName("FieldLabel")
        holder = QWidget()
        left = QHBoxLayout(holder)
        left.setContentsMargins(0, 0, 0, 0)
        left.addWidget(label, 1)
        trailing = QWidget()
        trailing.setFixedWidth(4)
        self.section.add_row(field["key"], holder, editor, trailing, f"{field['key']} {field.get('label', '')}")
        self.rows[field["key"]] = (field, editor, label)

    def _value_of(self, key):
        row = self.rows.get(key)
        return row[1].value() if row is not None else self.global_values.get(key)

    def _changes(self):
        return {k: self.rows[k][1].value() for k in self.touched
                if self.rows[k][1].value() is not INVALID and not _same(self.rows[k][1].value(), self.baseline.get(k))}

    def refresh_baseline(self, reset=False):
        """전역 설정 값을 다시 읽는다. 튜닝으로 바꾸지 않은 칸은 새 전역 값을 따라간다."""
        page = self.page
        fields = sensor_config.fields_for(self.group, self.subset["key"])
        old = dict(self.baseline)
        self.global_values = {f["key"]: sensor_config.effective_value(self.group, f, page.overrides, page._base)
                              for f in fields}
        self.baseline = {k: self.global_values.get(k) for k in self.rows}
        if reset:
            self.touched.clear()
        for key, (_field, editor, _label) in self.rows.items():
            if key not in self.touched or _same(editor.value(), self.baseline[key]):
                editor.set_value(self.baseline[key])
                self.touched.discard(key)
        self._refresh_state()

    def _refresh_state(self):
        pixel = self.global_values.get("pixel_format")
        for key, (field, editor, label) in self.rows.items():
            cond = field.get("enabled_when")
            ok = sensor_config.condition_met(cond, self._value_of) if cond else True
            locked = bool(cond) and not ok and "pixel_format" in _condition_keys(cond)
            restart = key in self.restart_keys
            editor.setEnabled(ok and not restart)
            label.setText(("🔒 " if locked or restart else "") + (field.get("label") or key))
            tip = [key, "경로: " + sensor_config.field_route(field)[1]]
            if restart:
                tip.append("영상을 받는 동안 카메라가 잠그는 값이라 튜닝 중에는 못 바꿉니다 (카메라 XML: TLParamsLocked). "
                           "설정 탭에서 바꾸고 [⟳ 재기동].")
            elif locked:
                tip.append(f"픽셀 포맷이 {pixel} 이라 잠겼습니다 — RGB 출력에서만 카메라가 켜는 기능입니다. 픽셀 포맷은 "
                           "튜닝 중에 못 바꿉니다 (설정 탭에서 바꾸고 다시 기동).")
            elif cond and not ok:
                tip.append("위 칸의 조건이 안 맞아 잠겼습니다 (예: WB auto 가 Off 여야 비율을 넣을 수 있음).")
            if field.get("help"):
                tip.append(field["help"])
            label.setToolTip("\n".join(tip))
            idle = bool(field.get("relevant_when")) and not sensor_config.condition_met(
                field["relevant_when"], self._value_of)
            if (label.property("idle") == "true") != idle:
                label.setProperty("idle", "true" if idle else "false")
                ui_theme.repolish(label)
            changed = key in self.touched and not _same(editor.value(), self.baseline.get(key))
            font = label.font()
            if font.bold() != changed:
                font.setBold(changed)
                label.setFont(font)
        changes = self._changes()
        self.btn_promote.setText(f"모든 카메라에 적용 ({len(changes)})" if changes else "모든 카메라에 적용")
        self.btn_promote.setEnabled(bool(changes))
        self.btn_revert.setEnabled(bool(changes))

    # --- 대상 카메라 ---

    def refresh_target(self):
        """콤보(감지된 카메라) · 실행 상태 · 라이브를 맞춘다. 페이지가 장비 · 실행 상태가 바뀔 때 부른다."""
        page = self.page
        devices = [d for d in page._devices if d.get("subset", self.subset["key"]) == self.subset["key"]]
        wanted = [(d["identity"], sensor_config.camera_entry(self.subset, d["identity"], page.overrides)["namespace"])
                  for d in devices]
        have = [(self.combo.itemData(i), self.combo.itemText(i)) for i in range(self.combo.count())]
        keep = self.combo.currentData() or self.serial or page._selected
        labels = [(sn, f"{name} · {sn}") for sn, name in wanted]
        if [h[0] for h in have] != [w[0] for w in labels] or [h[1] for h in have] != [w[1] for w in labels]:
            self.combo.blockSignals(True)
            self.combo.clear()
            for sn, text in labels:
                self.combo.addItem(text, sn)
            index = self.combo.findData(keep)
            self.combo.setCurrentIndex(index if index >= 0 else 0)
            self.combo.blockSignals(False)
        self.serial = self.combo.currentData()
        running = page.launched.get(self.serial) if (self.serial and page._locked and page._included) else None
        dev = next((d for d in devices if d["identity"] == self.serial), {})
        if running and not self.running_ns and self.touched:
            self._reapply = True
        if running != self.running_ns or (running and (self.live.serial != self.serial or not self.live.topics)):
            self.running_ns = running
            if running:
                self.live.show_camera(self.serial, running, running, False, None)
            else:
                self.live.clear_camera("카메라가 실행 중이 아닙니다.\n[▶ 이 카메라만 기동] 을 누르세요.")
        if running and not self.live.topics:
            self._live_retry.start()
        else:
            self._live_retry.stop()
            self._reapply_touched()
        self.btn_start.setEnabled(bool(self.serial) and not page._locked and dev.get("state") == OK)
        # 재기동은 튜닝 기동(이 카메라 한 대만 떠 있음)일 때만 — 리그 전체를 내리지 않게
        solo = bool(running) and set(page.launched) == {self.serial}
        self.btn_restart.setEnabled(solo and self._job is None)
        self.btn_restart.setToolTip(self.btn_restart.toolTip().split("\n")[0] + (
            "" if solo or not running else "\n리그 전체가 떠 있어 재기동은 막아 둡니다 — [■ 중지] 후 [이 카메라만 기동]."))
        self._update_auto_buttons()
        if not self.serial:
            text = "감지된 카메라가 없습니다."
        elif running:
            others = len(page.launched) - 1
            text = (f"실행 중 — /{running}/ 에 바로 적용됩니다"
                    + (f" (리그 전체가 떠 있어 이 카메라만 다른 값이 됩니다: 다른 {others}대는 그대로)" if others > 0 else ""))
        elif page._locked:
            text = "리그가 실행 중인데 이 카메라는 이번 기동에 없습니다 — 중지한 뒤 [이 카메라만 기동]"
        elif dev.get("state") != OK:
            text = "이대로는 못 여는 카메라입니다 (감지된 장비 탭 확인)"
        else:
            text = "멈춰 있습니다 — [▶ 이 카메라만 기동] 후 값을 바꾸면 바로 들어갑니다"
        self.status.setText(text)

    # --- 적용 ---

    def _on_edit(self, key):
        self.touched.add(key)
        self._queue(key)
        # 이 칸 때문에 풀린 칸의 값도 같이 넣는다 (WB auto Off → R · B 비율, 노출 auto Off → 노출 시간).
        # 전역 값도 없고 손대지도 않은 칸은 보이는 숫자가 스핀 최소값일 뿐이라 보내지 않는다.
        for other, (field, _editor, _label) in self.rows.items():
            cond = field.get("enabled_when")
            if other != key and cond and key in _condition_keys(cond) and \
                    sensor_config.condition_met(cond, self._value_of) and \
                    (other in self.touched or self.baseline.get(other) is not None):
                self._queue(other)
        self._refresh_state()

    def _queue(self, key):
        field, editor, _label = self.rows[key]
        value = editor.value()
        if value is None or value is INVALID or not self.running_ns or not editor.isEnabled():
            return
        self._pending[key] = value
        self._debounce.start()

    def _flush(self):
        pending, self._pending = self._pending, {}
        for key, value in pending.items():
            self._retried.discard(key)
            self.sig_live.emit(self.serial, key, value)
        if pending:
            self.result.setText("보내는 중… " + ", ".join(pending))

    def on_result(self, key, ok, reason):
        """카메라 노드의 응답. 기동 때 잠겨 등록이 안 된 노드면 레지스트리 live_alias(늘 등록되는 별칭)로 다시 보낸다."""
        field = next((row[0] for k, row in self.rows.items() if key in (k, row[0].get("live_alias"))), None)
        label = (field or {}).get("label") or key
        if not ok and field and field.get("live_alias") and key == field["key"] and key not in self._retried \
                and "declared" in reason:
            self._retried.add(key)
            self.sig_live.emit(self.serial, field["live_alias"], self.rows[key][1].value())
            return
        if ok:
            self.result.setText(f"✓ {label} → 카메라에 들어감")
        elif "not writable" in reason:
            self.result.setText(f"✗ {label}: 지금 카메라 상태에서 잠겨 있습니다 — 영상을 받는 중에는 못 바꾸는 값일 수 "
                                "있습니다 ([모든 카메라에 적용] 뒤 재기동하면 들어갑니다)")
        else:
            self.result.setText(f"✗ {label}: {reason}")

    def _reapply_touched(self):
        """재기동 뒤 — 카메라는 전역 설정으로 돌아왔다. 이 탭에서 바꿔 둔 값(전역에 반영 안 한 것)을 다시 넣는다."""
        if not self._reapply or not self.running_ns or not self.live.topics:
            return
        self._reapply = False
        keys = [k for k in self.touched if self.rows[k][1].isEnabled()]
        for key in keys:
            self._queue(key)
        if keys:
            self.result.setText(f"재기동 뒤 이 탭에서 바꾼 {len(keys)}칸을 다시 넣었습니다")

    # --- 자동 맞추기 ---

    def _update_auto_buttons(self):
        ready = bool(self.running_ns) and self._image_meter() is not None and self._job is None
        for button in self.auto_buttons.values():
            button.setEnabled(ready)

    def _image_meter(self):
        if not self.live.topics or self.live.hub is None:
            return None
        topic, type_name = self.live.topics["image"]
        if type_name != "sensor_msgs/msg/CompressedImage":
            return None
        return self.live.hub.meters.get(topic)

    def _frames(self, n=5, min_s=0.4):
        """새 프레임 n 장이 올 때까지 (값을 넣은 뒤 카메라가 그 값으로 찍은 프레임을 재려고)."""
        meter = self._image_meter()
        start = meter.count if meter else 0
        t0 = time.monotonic()

        def ready():
            m = self._image_meter()
            if time.monotonic() - t0 > 8.0:
                raise RuntimeError("영상이 안 들어옵니다 (미리보기 확인)")
            return m is not None and m.count >= start + n and time.monotonic() - t0 >= min_s
        return ready

    def _grab(self, center=False):
        meter = self._image_meter()
        if meter is None or meter.latest is None:
            raise RuntimeError("미리보기 영상이 없습니다")
        img = preview_panel.jpeg_to_array(meter.latest, 320).astype(np.float32) / 255.0
        if center:
            h, w, _ = img.shape
            img = img[h // 4: h - h // 4, w // 4: w - w // 4]
        return img

    def _set_live(self, key, value):
        """자동 맞추기의 결과를 칸에 넣고(수동 값을 덮어씀) 바로 카메라에 보낸다."""
        _field, editor, _label = self.rows[key]
        editor.set_value(value)
        self.touched.add(key)
        value = editor.value()
        if value is not None and value is not INVALID:
            self.sig_live.emit(self.serial, key, value)
        self._refresh_state()

    def _say(self, text):
        self.auto_status.setText(text)

    def _max_exposure(self):
        g = self.global_values
        fps = g.get("camera.AcquisitionFrameRate") if g.get("camera.AcquisitionFrameRateEnable", True) else None
        fps = fps or g.get("timestamp.trigger_grid_hz") or g.get("ptp_action.rate_hz") or 30.0
        return min(0.9e6 / float(fps), 100000.0)

    def start_auto(self, name):
        if self._job is not None or not self.running_ns:
            return
        jobs = {"wb": self._job_wb, "exposure": self._job_exposure, "gamma": self._job_gamma, "black": self._job_black}
        self._job = jobs[name]()
        self._job_wait = None
        for button in self.auto_buttons.values():
            button.setEnabled(False)
        self.btn_restart.setEnabled(False)
        self.btn_cancel.show()
        self._job_timer.start()
        self._job_step()

    def _job_tick(self):
        try:
            if self._job_wait is not None and not self._job_wait():
                return
        except Exception as exc:                                  # noqa: BLE001
            self._finish_job(f"✗ {exc}")
            return
        self._job_step()

    def _job_step(self):
        try:
            self._job_wait = next(self._job)
        except StopIteration as done:
            self._finish_job(done.value or "끝")
        except Exception as exc:                                  # noqa: BLE001
            self._finish_job(f"✗ {exc}")

    def _finish_job(self, text):
        self._job_timer.stop()
        self._job, self._job_wait = None, None
        self.btn_cancel.hide()
        self._say(text)
        self.refresh_target()

    def _job_wb(self):
        red, blue = "camera.BalanceRatioRed_Val", "camera.BalanceRatioBlue_Val"
        self._set_live("camera.BalanceWhiteAuto", "Off")
        r = (self.rows[red][1].value() or 13926) / 8192.0
        b = (self.rows[blue][1].value() or 13107) / 8192.0
        self._set_live(red, int(round(r * 8192)))
        self._set_live(blue, int(round(b * 8192)))
        yield self._frames()
        for i in range(8):
            img = self._grab(center=True)
            ok = (img.max(axis=2) < 0.97) & (img.mean(axis=2) > 0.06)
            if ok.sum() < 300:
                raise RuntimeError("화면 가운데가 너무 어둡거나 하얗게 날아갔습니다 — 노출을 먼저 맞추세요")
            mr, mg, mb = (float(img[..., c][ok].mean()) for c in range(3))
            er, eb = mg / mr, mg / mb
            self._say(f"화이트밸런스 {i + 1}회 — R {r:.2f} · B {b:.2f} (G 대비 R {1 / er:.3f} · B {1 / eb:.3f})")
            if abs(er - 1) < 0.01 and abs(eb - 1) < 0.01:
                break
            r = min(4.0, max(0.5, r * min(1.4, max(0.7, er ** 1.2))))
            b = min(4.0, max(0.5, b * min(1.4, max(0.7, eb ** 1.2))))
            self._set_live(red, int(round(r * 8192)))
            self._set_live(blue, int(round(b * 8192)))
            yield self._frames()
        return f"✓ 화이트밸런스 — R {r:.2f} · B {b:.2f} (WB auto Off)"

    def _job_exposure(self):
        key = "camera.ExposureTime"
        self._set_live("camera.ExposureAuto", "Off")
        known = key in self.touched or self.baseline.get(key) is not None
        e = float(self.rows[key][1].value()) if known else 5000.0
        e_max = self._max_exposure()
        e = min(max(e, 20.0), e_max)
        self._set_live(key, e)
        yield self._frames()
        note = ""
        for i in range(10):
            m = float(np.median(self._grab().mean(axis=2)))
            self._say(f"노출 {i + 1}회 — {e:.0f} µs · 밝기 중앙값 {m:.2f} (목표 {AUTO_TARGET:.2f})")
            if abs(m - AUTO_TARGET) < 0.025:
                break
            factor = 4.0 if m < 0.005 else min(4.0, max(0.25, (AUTO_TARGET / m) ** 1.25))
            new = min(e_max, max(20.0, e * factor))
            if abs(new - e) < 1.0:
                note = f" — 한계({e_max:.0f} µs, 트리거 주기의 90%)에 붙음: 더 밝히려면 게인 · 감마" if new >= e_max else ""
                break
            e = float(round(new))
            self._set_live(key, e)
            yield self._frames()
        return f"✓ 노출 — {e:.0f} µs (노출 auto Off){note}"

    def _job_gamma(self):
        key = "camera.Gamma_FloatVal"
        self._set_live("camera.GammaEnable", True)
        g = float(self.rows[key][1].value() or 0.8)
        self._set_live(key, g)
        yield self._frames()
        history = []
        for i in range(8):
            m = float(np.median(self._grab().mean(axis=2)))
            history.append((g, m))
            self._say(f"감마 {i + 1}회 — {g:.2f} · 밝기 중앙값 {m:.2f} (목표 {AUTO_TARGET:.2f})")
            if abs(m - AUTO_TARGET) < 0.02:
                break
            if m <= 0.003 or m >= 0.997:
                raise RuntimeError("화면이 너무 어둡거나 밝아 감마로는 못 맞춥니다 — 노출을 먼저 맞추세요")
            if len(history) >= 2 and abs(history[-1][1] - history[-2][1]) > 1e-3:
                (g1, m1), (g2, m2) = history[-2], history[-1]
                new = g2 + (AUTO_TARGET - m2) * (g2 - g1) / (m2 - m1)     # 잰 기울기로 — 감마 방향 관례와 무관
            else:
                new = g * math.log(AUTO_TARGET) / math.log(m)             # 첫 걸음: 출력 = 입력^감마 가정
            new = min(2.0, max(0.5, new))
            if abs(new - g) < 0.005:
                break
            g = new
            self._set_live(key, g)
            yield self._frames()
        return f"✓ 감마 — {g:.2f} (감마 보정 켬)"

    def _job_black(self):
        key = "camera.BlackLevel"
        bl = float(self.rows[key][1].value() or 0.0)
        self._set_live(key, bl)
        yield self._frames()
        target = 0.03
        for i in range(8):
            low = float(np.percentile(self._grab().mean(axis=2), 1))
            self._say(f"블랙 레벨 {i + 1}회 — {bl:.2f}% · 하위 1% 밝기 {low:.3f} (목표 {target:.3f})")
            if abs(low - target) < 0.008:
                break
            new = min(5.0, max(0.0, bl + (target - low) * 100 * 0.8))
            if abs(new - bl) < 0.05:
                break
            bl = new
            self._set_live(key, round(bl, 2))
            yield self._frames()
        return f"✓ 블랙 레벨 — {bl:.2f}%"

    def revert(self):
        """전역 값으로 되돌린다. 실행 중이면 카메라에도 다시 넣는다 (지정 안 함인 칸은 넣을 값이 없어 그대로)."""
        skipped = []
        for key in sorted(self.touched):
            _field, editor, label = self.rows[key]
            if _same(editor.value(), self.baseline.get(key)):
                continue
            editor.set_value(self.baseline.get(key))
            if self.baseline.get(key) is None:
                skipped.append(label.text())
            else:
                self._queue(key)
        self.touched.clear()
        self._refresh_state()
        self.result.setText("전역 값으로 되돌렸습니다" + (f" — '지정 안 함' 인 {len(skipped)}칸은 카메라에 이미 들어간 값이 "
                                                     "그대로입니다 (다시 기동하면 카메라 값)" if skipped else ""))

    def promote(self):
        """[모든 카메라에 적용] — 설정에 저장하고 떠 있는 카메라 전부에 넣는다. 결과는 페이지가 show_applied 로 알린다."""
        changes = self._changes()
        if not changes:
            return
        self.result.setText(f"{len(changes)}칸을 모든 카메라에 적용하는 중…")
        self.sig_promote.emit(changes)
        self.touched -= set(changes)
        self.refresh_baseline()

    def show_applied(self, cameras, failed, labels):
        """cameras: 값을 넣으려 한 떠 있는 카메라 수 (0 = 떠 있는 카메라 없음). failed: [(노드, 키, 사유)]."""
        if not cameras:
            self.result.setText("✓ 설정에 저장했습니다 — 지금 떠 있는 카메라가 없어 다음 기동부터 모든 카메라에 들어갑니다 "
                                "(재기동할 필요 없음)")
        elif not failed:
            self.result.setText(f"✓ 떠 있는 카메라 {cameras}대 전부에 바로 들어갔고 설정에도 저장했습니다 — 재기동할 필요 "
                                "없습니다")
        else:
            nodes = sorted({node.strip("/").split("/")[0] for node, _key, _why in failed})
            keys = sorted({labels.get(key, key) for _node, key, _why in failed})
            self.result.setText(f"⚠ {cameras}대 중 {len(nodes)}대({', '.join(nodes[:4])}{' …' if len(nodes) > 4 else ''})에 "
                                f"{', '.join(keys)} 이(가) 바로 안 들어갔습니다 ({failed[0][2]}). 설정에는 저장됨 — 위의 "
                                "[⟳ 지금 재기동] 을 누르면 들어갑니다")

    def _retry_live(self):
        if not self.running_ns or self.live.topics:
            self._live_retry.stop()
            return
        if self.isVisible():
            self.live.show_camera(self.serial, self.running_ns, self.running_ns, False, None)
            if self.live.topics:
                self._live_retry.stop()
                self._reapply_touched()
                self._update_auto_buttons()

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh_baseline()
        self.refresh_target()


# 장비 행의 상태 점. 카드의 색 띠 · 점과 같은 색.
WAITING, EXCLUDED, DEGRADED = "waiting", "excluded", "degraded"
ROW_STATE_TEXT = {RUNNING: "실행 중 — 영상 수신 중", STARTING: "기동 중 — 카메라를 여는 중",
                  DEGRADED: "실행 중이지만 문제가 있음 — 런치 로그 확인",
                  WAITING: "대기 — 순차 기동 차례를 기다리는 중", FAILED: "실패",
                  EXCLUDED: "이번 기동에 미포함", STOPPED: "정지"}
_DOT_ICONS = {}

# 카메라 한 대의 image_raw (BayerRG8 1920×1200 × 30 Hz ≈ 69 MB/s) 와 녹화 디스크(SATA SSD) 쓰기 한계.
# 2026-09-30 합성 raw 7 스트림 녹화: 489 MB/s 까지 손실 0, 디스크 ~545 MB/s 가 천장.
RAW_MBPS, RAW_DISK_MBPS = 69, 480


def dot_icon(state, lit=True):
    """상태 점 아이콘 (16px 칸에 10px 원). 대기는 빈 주황 원, 정지·미포함은 빈 회색 원, 기동 중은 깜빡임(lit)."""
    key = (state, lit)
    if key not in _DOT_ICONS:
        from PyQt5.QtGui import QColor, QIcon, QPen
        color = {RUNNING: ui_theme.OK, STARTING: ui_theme.WARN if lit else "#fde68a",
                 WAITING: ui_theme.WARN, DEGRADED: ui_theme.WARN, FAILED: ui_theme.ERR}.get(state, "#c4c9d0")
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
    """ISP 튜닝 탭(휠 허용)에서도 칸을 눌러 포커스가 있을 때만 휠이 값에 먹게 — 튜닝 칸 목록을 휠로 내리다가
    지나가는 칸의 값이 바뀌지 않게. 포커스가 없으면 휠을 목록(스크롤 영역)에 넘긴다.
    그 밖의 곳은 ui_theme.lock_wheel 이 휠을 아예 막는다."""

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Wheel and not obj.hasFocus():
            ui_theme.forward_wheel(obj, event)
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
            # choice_labels: {값: 보이는 이름} — 콤보는 이름을 보이고 value() 는 원래 값을 돌려준다
            labels = field.get("choice_labels") or {}
            for choice in field.get("choices") or []:
                widget.addItem(str(labels.get(choice, choice)), choice)
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
            elif self.field.get("placeholder"):             # 비워 두면 어떻게 되는지 (예: '자동')
                widget.setPlaceholderText(self.field["placeholder"])
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
            index = self.editor.findData(value)
            if index < 0:
                text = "" if value is None else str(value)
                self.editor.addItem(text, text)
                index = self.editor.count() - 1
            self.editor.setCurrentIndex(index)
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
            data = self.editor.currentData()
            if data is None or (self.editor.isEditable() and self.editor.currentText() != self.editor.itemText(
                    self.editor.currentIndex())):
                return self.editor.currentText()         # 직접 적은 값 (전체 설정의 고르기 칸)
            return data
        if kind == "float":
            value = float(self.editor.value()) * self.scale
            # 폼은 실수, 파일은 정수 레지스터 (BalanceRatioRed_Val = 비율 × 8192) — ROS 파라미터 타입이 엄격하다
            return int(round(value)) if self.field.get("file_type") == "int" else value
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
    sig_tune = pyqtSignal(str, str)             # card key, serial — [ISP 튜닝] 이 카메라만 기동
    sig_retune = pyqtSignal(str, str)           # card key, serial — [ISP 튜닝] 재기동
    sig_live_param = pyqtSignal(str, str, str, object)   # card key, serial, 키, 값 — 실행 중인 카메라에 넣기
    sig_push_params = pyqtSignal(str, dict)     # card key, {키: 값} — [모든 카메라에 적용]: 떠 있는 카메라 전부에 넣기
    sig_restart = pyqtSignal(str)               # group key — [⟳ 지금 재기동] (기동 뒤 바꾼 설정 넣기)
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
        # 이번 기동 때의 설정 (스테이지가 넘겨 줌) — 지금 설정과 달라진 칸은 재기동해야 들어간다 → 머리에 안내 띠
        self.launch_snapshot = None
        self._selected = None            # 표에서 고른 시리얼 (다시 그려도 유지)
        self._picked = set()             # Ctrl·Shift 로 여러 행을 고른 시리얼들 (동기 방식 일괄 적용 대상)
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
        self.btn_start.clicked.connect(self._on_start_clicked)
        self.btn_stop.clicked.connect(lambda: self.sig_stop.emit(self.group["key"]))
        head.addWidget(title)
        head.addWidget(self.pill, 0, Qt.AlignVCenter)
        head.addStretch()
        # ISP 튜닝은 탭 줄 끝의 작은 탭이라 아무도 못 찾았다 (2026-09-22) — 머리에 버튼으로 꺼내 둔다
        self.btn_isp = QPushButton("🎨  ISP 튜닝 · 화질")
        self.btn_isp.setToolTip("카메라 영상을 보면서 노출 · 게인 · 화이트밸런스 · 감마 · 채도 · 샤프닝을 바꾸면 그 자리에서 "
                                "들어갑니다 (재기동 없음).\n맞으면 [모든 카메라에 적용] — 떠 있는 카메라 전부에 바로 넣고 "
                                "설정에도 저장합니다.")
        self.btn_isp.clicked.connect(self.open_tuning)
        self.btn_isp.hide()
        head.addWidget(self.btn_isp)
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

        # 어느 체크아웃이 뜨는지 — ~/FLIR_control 을 심볼릭 링크로 바꿔 끼우면 화면에 안 보인다
        where = sensor_launcher.work_dir_label(self.group)
        if where:
            repo = QLabel(f"리포  {where}")
            repo.setObjectName("Hint")
            repo.setTextInteractionFlags(Qt.TextSelectableByMouse)
            repo.setToolTip("런치가 도는 작업 디렉터리 (sensors.yaml 의 workdir) — 실제 경로와 git 브랜치")
            outer.addWidget(repo)

        # PTP 에 기대는 센서군(카메라가 PTP slave 로 촬영 · 라이다 타임스탬프가 PTP)일 때만 뜨는 상태 줄.
        # dm 이 터미널에서 묻던 것을 여기로 옮겼다 — 기동을 막지 않고, 켜기 전에 눈으로 확인만 시킨다.
        self.ptp_bar = sync_check.PtpBar(lambda: sync_check.ptp_reasons(self.group, self.overrides))
        outer.addWidget(self.ptp_bar)
        QTimer.singleShot(1500, self.ptp_bar.refresh)

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

        # 링크 대역폭 경고 — 설정을 바꿀 때마다 다시 계산 (탭을 넘겨도 보이게 머리에)
        self.budget_banner = QFrame()
        self.budget_banner.setObjectName("Warn")
        budget_layout = QVBoxLayout(self.budget_banner)
        budget_layout.setContentsMargins(10, 6, 8, 6)
        self.budget_text = QLabel("")
        self.budget_text.setWordWrap(True)
        self.budget_text.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        budget_layout.addWidget(self.budget_text)
        self.budget_banner.hide()
        outer.addWidget(self.budget_banner)

        # 기동한 뒤 바꾼 설정 — 실행 중인 노드는 기동 때 값을 쓰고 있다. 무엇이 아직 안 들어갔는지와 [⟳ 지금 재기동]
        self.restart_banner = QFrame()
        self.restart_banner.setObjectName("Warn")
        rb = QHBoxLayout(self.restart_banner)
        rb.setContentsMargins(10, 6, 8, 6)
        self.restart_text = QLabel("")
        self.restart_text.setWordWrap(True)
        self.restart_text.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        rb.addWidget(self.restart_text, 1)
        self.btn_restart_group = QPushButton("⟳  지금 재기동")
        self.btn_restart_group.setToolTip("이 센서군을 내렸다가 같은 구성으로 다시 올립니다 — 바꾼 설정이 전부 들어갑니다.\n"
                                          "그동안(수십 초) 이 센서군의 데이터가 끊깁니다.")
        self.btn_restart_group.clicked.connect(lambda: self.sig_restart.emit(self.group["key"]))
        rb.addWidget(self.btn_restart_group, 0, Qt.AlignVCenter)
        self.restart_banner.hide()
        outer.addWidget(self.restart_banner)

        # 위: [감지된 장비 | 설정] 탭 — 각자 전체 높이를 쓴다 (800x600 화면에서도 읽힌다)
        # 아래: 런치 로그 — 탭을 넘겨도 계속 보인다. 경계는 끌어서 조절.
        self.tabs = QTabWidget()
        self.tabs.setObjectName("Inner")
        self.sync_state = None
        if self.group.get("single_device"):
            # 장비가 하나뿐인 센서(라이다) — 장비 표와 설정을 탭으로 나누지 않고 한 화면에:
            # 위에 장비 한 줄 · 센서 동기 상태, 아래 설정
            combined = QWidget()
            cv = QVBoxLayout(combined)
            cv.setContentsMargins(0, 0, 0, 0)
            cv.setSpacing(4)
            devices = self._devices_panel()
            devices.setMaximumHeight(150)
            cv.addWidget(devices)
            if self.group.get("sync_status"):
                self.sync_state = QLabel("센서 동기 상태 — 읽는 중…")
                self.sync_state.setObjectName("Hint")
                self.sync_state.setWordWrap(True)
                self.sync_state.setContentsMargins(10, 0, 10, 0)
                self.sync_state.setToolTip("센서 HTTP API (/api/v1/time) 에서 5초마다 읽습니다 — 읽기만 하고 센서 설정은 "
                                           "바꾸지 않습니다.")
                cv.addWidget(self.sync_state)
                self._time_worker = None
                self._pulse_prev = None          # (센서 monotonic, 펄스 수) — 지금 PPS 가 들어오는지 재려고
                self._time_status = None
                self._time_timer = QTimer(self, interval=5000, timeout=self._poll_sensor_time)
            cv.addWidget(self._settings_panel(), 1)
            self.tabs.addTab(combined, "장비 · 설정")
            self.tabs.tabBar().hide()
        else:
            self.tabs.addTab(self._devices_panel(), "감지된 장비")
            self.tabs.addTab(self._settings_panel(), "설정")
        # 동기 역할이 있는 카메라군만 — 실행 중인 카메라들이 같은 트리거로 찍는지 GUI 에서 잰다
        self.sync_panel = None
        if self.sync_roles and self.group.get("sync_check"):
            self.sync_panel = sync_check.SyncCheckPanel(self.group)
            self.tabs.addTab(self.sync_panel, "동기 검증")
            self._update_sync_targets()
        # ISP 튜닝 — 레지스트리에 tuning 이 있는 카메라 종류만 (카메라 한 대를 띄워 놓고 값을 바로 넣어 본다)
        self.tuning = None
        if self.cameras_editable and self.subset.get("tuning"):
            self.tuning = TuningPanel(self)
            if self.tuning.fields:
                self.tuning.sig_start.connect(lambda sn: self.sig_tune.emit(self.card["key"], sn))
                self.tuning.sig_restart.connect(lambda sn: self.sig_retune.emit(self.card["key"], sn))
                self.tuning.sig_live.connect(
                    lambda sn, key, value: self.sig_live_param.emit(self.card["key"], sn, key, value))
                self.tuning.sig_promote.connect(self.apply_tuned)
                self.tabs.addTab(self.tuning, "🎨 ISP 튜닝")
                self.btn_isp.show()
            else:
                self.tuning = None
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

        # 동기 방식 일괄 — 여러 대를 한 대씩 바꾸다 한 대를 빠뜨리면 그 카메라만 트리거를 안 받는다.
        # 대상: 표에서 2대 이상 골랐으면 고른 카메라, 아니면 표의 전부 (한 대 고른 건 라이브 보기라 전체로 본다).
        self.sync_bar = QWidget()
        sb = QHBoxLayout(self.sync_bar)
        sb.setContentsMargins(0, 0, 0, 0)
        sb.setSpacing(6)
        title = QLabel("동기 방식 일괄")
        title.setObjectName("Section")
        sb.addWidget(title)
        self.sync_target = QLabel("")
        self.sync_target.setObjectName("Hint")
        sb.addWidget(self.sync_target)
        sb.addStretch()
        for mode, label in (("hw_trigger", "HW 트리거"), ("ptp", "PTP 액션"), ("free", "자유 실행")):
            btn = QPushButton(label)
            btn.setToolTip({"hw_trigger": "GPIO 트리거 입력으로 찍습니다 (전부 받는 쪽)",
                            "ptp": "PTP 액션 명령으로 찍습니다 — 보내기 1대 + 나머지 받기.\n"
                                   "이미 보내기인 카메라가 있으면 그대로 두고, 없으면 대상의 첫 카메라가 맡습니다.",
                            "free": "트리거 없이 각자 프레임레이트대로 찍습니다"}[mode]
                           + "\n\n다음 기동부터 적용")
            btn.clicked.connect(lambda _=False, m=mode: self._bulk_sync_mode(m))
            sb.addWidget(btn)
        self.sync_bar.setVisible(self.sync_roles)
        layout.addWidget(self.sync_bar)

        self.table = QTableWidget(0, 5)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self.table.itemChanged.connect(self._on_table_item)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        # Ctrl·Shift 로 여러 대를 골라 동기 방식을 한 번에 (고른 행의 콤보를 바꾸면 고른 행 전부 바뀐다)
        self.table.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        # 헤더를 누르면 그 열로 오름차순, 한 번 더 누르면 내림차순 (다시 오름…). Qt 의 자체 정렬
        # (setSortingEnabled)은 쓰지 않는다 — 동기 방식 칸의 콤보(셀 위젯)가 행을 못 따라가고,
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
        # 카메라 이름 · 동기 방식 · IP 를 리포 인벤토리에 — GUI 없이 ros2 launch 로 띄워도 같은 이름이 된다
        self.btn_promote_inv = QPushButton("이름·역할을 인벤토리에 저장")
        self.btn_promote_inv.setToolTip(
            "GUI 에서 정한 카메라 이름 · 동기 방식 · IP 를 리포의 인벤토리 YAML 에 씁니다\n"
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

        # 1) 주요 설정 — panel 이 붙은 필드(ISP 등)는 레지스트리 panels 의 제목으로 따로 묶는다
        curated = sensor_config.fields_for(self.group, main)
        plain = [f for f in curated if not f.get("panel")]
        if plain:
            hint = "이 종류의 모든 장비에 똑같이 적용됩니다. 칸에 마우스를 올리면 설명이 나옵니다." \
                if self.card["subset"] else "칸에 마우스를 올리면 설명이 나옵니다."
            self._add_section(SettingsSection(f"주요 설정 — {self.card['label']}", hint, primary=True),
                              col, main, plain)
        tuned_panels = ((self.subset or {}).get("tuning") or {}).get("panels") or []
        for name, panel in (self.group.get("panels") or {}).items():
            fields = [f for f in curated if f.get("panel") == name]
            if fields and name in tuned_panels and self.cameras_editable:
                col.addWidget(self._tuning_link(panel["title"]))
            if fields:
                self._add_section(SettingsSection(f"{panel['title']} — {self.card['label']}", panel.get("hint", ""),
                                                  collapsible=True, expanded=panel.get("expanded", True),
                                                  primary=True),
                                  col, main, fields)
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
            xml = sensor_config.genicam_source(self.group, main)
            extra = f" · GenICam = 카메라 XML({xml.name})의 노드" if xml else ""
            note = QLabel(f"{source} 의 모든 키와 런치 인자{extra} · 회색 = 원본에서 주석 처리된 키 ('지정' 해야 실림) "
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
        # 같은 소제목끼리 모은다 (처음 나온 순서) — 레지스트리에서 흩어져 있어도 소제목이 두 번 나오지 않게
        order = {}
        for field in fields:
            order.setdefault(field.get("section"), len(order))
        fields = sorted(fields, key=lambda f: order[f.get("section")])
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
        if not generic:
            # 값이 카메라에 닿는 경로 — GenICam 에 그대로 / 노드 로직이 GenICam 여러 개를 / PC 에서만
            kind = sensor_config.field_route(field)[0]
            node_tag = "노드→GenICam" if field.get("writes_to", "GenICam") == "GenICam" else "노드→센서"
            tag = QLabel({"genicam": "GenICam", "node": node_tag, "pc": "PC"}[kind])
            tag.setObjectName("RouteTag")
            tag.setProperty("route", kind)
            left.addWidget(tag)

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

    def _condition_ok(self, scope, condition):
        """폼의 지금 값으로 enabled_when / relevant_when 판정 (목록이면 OR). 폼에 없는 키는 맞는 것으로 친다."""
        if isinstance(condition, list):
            return any(self._condition_ok(scope, c) for c in condition)
        return all(self.rows.get((scope, key)) is None or
                   sensor_config.matches(self.rows[(scope, key)][1].value(), wanted)
                   for key, wanted in condition.items())

    def _tooltip(self, field, is_changed):
        parts = [field["key"]]
        if not field.get("generic"):
            parts.append("경로: " + sensor_config.field_route(field)[1])
            scope = field.get("subset") or sensor_config.NO_SUBSET
            depends = sensor_config.genicam_dependencies(self.group, scope, field)
            if depends:
                parts.append(f"잠김이 바뀌는 조건 (카메라 XML): {', '.join(depends)}")
        if field.get("relevant_when") and not self._condition_ok(
                "launch" if field.get("target") == "launch_arg" else (field.get("subset") or sensor_config.NO_SUBSET),
                field["relevant_when"]):
            parts.append("지금 설정에서는 효과가 없습니다 (값은 그대로 파일에 실립니다)")
        if field.get("managed"):
            parts.append(f"🔒 {field['managed']}")
        if field.get("genicam"):
            parts.append("원본 YAML 에 없는 카메라 노드 — '지정' 을 켜야 파일에 실립니다. 범위는 카메라가 정해서, "
                         "범위 밖 값이면 노드가 기동 중에 죽습니다.")
        elif field.get("optional") and field.get("generic"):
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
        self._refresh_restart_banner()

        # enabled_when: 잠긴 노드라 쓰면 안 되는 칸 (노출 auto 면 노출 시간) → 칸 전체가 회색, 파일에서도 빠진다
        # relevant_when: 지금 설정에선 효과가 없을 뿐인 칸 → 이름만 흐리게, 편집 가능, 파일에는 그대로
        for (scope, _key), (field, editor, label, *_rest) in self.rows.items():
            if field.get("enabled_when"):
                editor.setEnabled(self._condition_ok(scope, field["enabled_when"]))
            if field.get("relevant_when"):
                idle = not self._condition_ok(scope, field["relevant_when"])
                if (label.property("idle") == "true") != idle:
                    label.setProperty("idle", "true" if idle else "false")
                    ui_theme.repolish(label)

        self._refresh_budget()
        if getattr(self, "sync_state", None) is not None:
            self._show_sensor_time()
        self.lbl_changed.setText(f"● 원본과 다른 값 {changed}개" if changed
                                 else "원본 그대로")
        self.lbl_changed.setObjectName("Changed" if changed else "Hint")
        ui_theme.repolish(self.lbl_changed)
        self.btn_reset_all.setEnabled(bool(changed))
        self.btn_promote.setEnabled(bool(changed))
        if hasattr(self, "tabs"):
            self.tabs.setTabText(1, f"설정  ●{changed}" if changed else "설정")

    # --- 표시 ---

    NAME_COL = 0          # 이름 · 시리얼 · IP · (동기 방식) · 모델 · NIC — 이름 짓기가 목적이라 맨 앞
    SERIAL_COL = 1

    def set_devices(self, devices, probe, error, candidates=None):
        self._devices, self._probe, self._error = list(devices), probe, error
        self._candidates = list(candidates or [])
        self._render_devices()

    # --- 센서 동기 상태 (라이다) ---

    def showEvent(self, event):
        super().showEvent(event)
        if self.sync_state is not None:
            self._poll_sensor_time()
            self._time_timer.start()

    def hideEvent(self, event):
        super().hideEvent(event)
        if self.sync_state is not None:
            self._time_timer.stop()

    def _poll_sensor_time(self):
        host = next((d.get("ip") for d in self._devices if d.get("ip")), None)
        if not host:
            scope = sensor_config.NO_SUBSET
            row = self.rows.get((scope, "sensor_hostname"))
            host = row[1].value() if row else None
        if not host or (self._time_worker is not None and self._time_worker.isRunning()):
            if not host:
                self.sync_state.setText("센서 동기 상태 — 감지된 라이다가 없습니다")
            return
        self._time_worker = SensorTimeWorker(host)
        self._time_worker.sig_done.connect(self._on_sensor_time)
        self._time_worker.start()

    def _on_sensor_time(self, status):
        rate = None
        if status is not None:
            now = (status["monotonic"], status["pps_count"])
            if self._pulse_prev and now[0] > self._pulse_prev[0]:
                rate = (now[1] - self._pulse_prev[1]) / (now[0] - self._pulse_prev[0])
            self._pulse_prev = now
        self._time_status = (status, rate)
        self._show_sensor_time()

    def _show_sensor_time(self):
        if self.sync_state is None or self._time_status is None:
            return
        status, rate = self._time_status
        row = self.rows.get((sensor_config.NO_SUBSET, "timestamp_mode"))
        mode = row[1].value() if row else ""
        offset_row = self.rows.get((sensor_config.NO_SUBSET, "ptp_utc_tai_offset"))
        if offset_row:
            offset = offset_row[1].value()
        else:                  # 전체 설정에 칸이 안 보이면 생성 파일에 들어갈 값으로
            target = next((t for t in sensor_config.param_targets(self.group)
                           if t["subset"] == sensor_config.NO_SUBSET), None)
            offset = (sensor_config._merged_params(self.group, target, self.overrides) or {}).get(
                "ptp_utc_tai_offset") if target else None
        lock_row = self.rows.get((sensor_config.NO_SUBSET, "phase_lock_enable"))
        angle_row = self.rows.get((sensor_config.NO_SUBSET, "phase_lock_offset"))
        # 칸의 value() 는 파일 값(밀리도) — 도로 바꿔 센서 값과 비교한다
        angle = (float(angle_row[1].value() or 0) / (angle_row[0].get("scale") or 1)) if angle_row else 0.0
        wanted_lock = (lock_row[1].value(), angle) if lock_row else None
        line, warns = sensor_discovery.ouster_sync_summary(status, mode, rate, offset, wanted_lock)
        self.sync_state.setText(line + "".join(f"\n⚠ {w}" for w in warns))
        self.sync_state.setObjectName("Hint" if not warns else "")
        self.sync_state.setStyleSheet("" if not warns else f"color: {ui_theme.WARN};")

    def _refresh_budget(self):
        """엮인 설정끼리의 문제 (sensor_config.check_settings). 문제가 없으면 숨긴다."""
        if not self.card["subset"]:
            return
        modes = [sensor_config.sync_mode(sensor_config.camera_entry(self.subset, d["identity"], self.overrides))
                 for d in self._devices] if self.subset else []
        synced = sum(1 for m in modes if m in sensor_config.SYNCED_MODES)
        ptp_cams = sum(1 for m in modes if m in ("ptp_sender", "ptp_receiver"))
        found = sensor_config.check_settings(self.group, self.card["subset"], self.overrides, len(self._devices),
                                             sensor_config.nic_mtus(self._devices), synced, ptp_cams)
        self.budget_text.setText("\n".join(f"{'⛔' if level == 'error' else '⚠'} {text}" for level, text in found))
        self.budget_banner.setVisible(bool(found))

    def _render_devices(self):
        self._refresh_budget()
        self._refresh_restart_banner()
        if getattr(self, "tuning", None) is not None:
            self.tuning.refresh_target()
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

        raw = self.cameras_editable                 # 카메라별 image_raw 칸 (라이다 표엔 없음)
        headers = ([("이름" if self.cameras_editable else "비고"), "시리얼", "IP"]
                   + (["동기 방식"] if self.sync_roles else []) + (["raw"] if raw else []) + ["모델", "NIC"])
        self._col_ids = (["name", "serial", "ip"] + (["ptp"] if self.sync_roles else [])
                         + (["raw"] if raw else []) + ["model", "nic"])
        self._raw_col = self._col_ids.index("raw") if raw else None
        tail = self._col_ids.index("model")         # 모델 칸 위치
        raw_default = self._raw_default() if raw else False
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

            on = self._raw_on(serial, raw_default)
            item = QTableWidgetItem("")
            item.setData(Qt.UserRole, serial)
            item.setFlags((item.flags() | Qt.ItemIsUserCheckable) & ~Qt.ItemIsEditable)
            if not editable:
                item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
            item.setCheckState(Qt.Checked if on else Qt.Unchecked)
            item.setToolTip(
                f"이 카메라의 /…/image_raw 발행 (다음 기동부터) — 공통 설정: {'켬' if raw_default else '끔'}"
                + ("  · 이 카메라만 따로 정함" if "publish_raw" in (
                    sensor_config.camera_overrides(self.overrides).get(serial) or {}) else "")
                + f"\nraw 1920×1200 30 Hz 는 한 대에 약 {RAW_MBPS} MB/s — 녹화 디스크(SATA SSD ~540 MB/s)와 "
                  "클립 링버퍼 메모리를 그만큼 씁니다.\n여러 대를 고른 채로 한 칸을 바꾸면 고른 카메라 전부 바뀝니다."
                + "\n켠 카메라의 image_raw 는 녹화 토픽에도 넣어야 bag 에 들어갑니다.")
            self.table.setItem(r, self._raw_col, item)

            if self.sync_roles:
                combo = QComboBox()
                for mode_key, label, _hw, _ptp in sensor_config.SYNC_MODES:
                    combo.addItem(label, mode_key)
                key = sensor_config.sync_mode(entry)
                if key is None:
                    # 표에 없는 조합 (GPIO master 등 인벤토리 값) — 보여만 주고 고르면 교체
                    combo.addItem(sensor_config.sync_mode_label(entry), None)
                    combo.setCurrentIndex(combo.count() - 1)
                else:
                    combo.setCurrentIndex(combo.findData(key))
                combo.setEnabled(editable)
                combo.setToolTip(
                    f"원본: {sensor_config.sync_mode_label(default)}\n"
                    "GPIO 트리거와 PTP 액션은 함께 켤 수 없어 늘 한 쌍으로 바뀝니다.\n"
                    "PTP 보내기(sender)는 하나면 됩니다 — 보이는 카메라 중 없으면 "
                    "기동 때 첫 카메라가 맡습니다.")
                combo.currentIndexChanged.connect(
                    lambda idx, sn=serial, cb=combo:
                        self._on_sync_mode(sn, cb.itemData(idx)))
                self.table.setCellWidget(r, self._col_ids.index("ptp"), combo)
        # 고른 행 유지 (다시 그려도 라이브가 끊기지 않고, 여러 대 고른 것도 풀리지 않게)
        keep = self._picked | ({self._selected} if self._selected else set())
        if keep:
            model = self.table.selectionModel()
            for r in range(self.table.rowCount()):
                cell = self.table.item(r, self.SERIAL_COL)
                if cell and cell.text() in keep:
                    model.select(self.table.model().index(r, 0),
                                 QItemSelectionModel.Select | QItemSelectionModel.Rows)
                    if cell.text() == self._selected:
                        self.table.setCurrentCell(r, self.SERIAL_COL, QItemSelectionModel.NoUpdate)
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
                self.table_msg.setToolTip("이름·동기 방식은 GUI 설정에 저장됩니다 — 리포 인벤토리 YAML 은 "
                                          "바뀌지 않습니다.")
        else:
            self.table_msg.hide()
        self._update_bulk_target()
        self._update_sync_targets()

    def _lidar_banner(self, devices):
        """라이다가 PC 에 못 붙는 이유와 [NIC 설정] 버튼. 보일 게 없으면 False."""
        if any(d["state"] == OK for d in devices):
            self.fix_nic = None
            return False
        stuck = [d for d in devices if d["state"] == NIC]
        fixable = [d for d in stuck if d.get("fix_nic")]
        self.fix_nic = (fixable[0]["fix_nic"] if fixable
                        else self._candidates[0]["nic"] if self._candidates else None)
        # 라이다를 실제로 본 NIC 인가, 'IPv4 없는 유선 NIC' 이라 짐작한 후보인가. 기동 전 자동 설정은 본 경우에만 —
        # 짐작 후보는 부팅 직후 링크가 늦게 올라온 카메라 NIC 일 수 있다 (2026-09-22, 아래 _lidar_fix).
        self.fix_nic_seen = bool(fixable)
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
        if col_id not in (self._col_ids or ["name", "serial", "ip", "ptp", "raw", "model", "nic"]):
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
            entry = sensor_config.camera_entry(self.subset, dev["identity"], self.overrides)
            key = sensor_config.sync_mode(entry)
            order = [k for k, *_ in sensor_config.SYNC_MODES]
            return [order.index(key) if key in order else len(order)], serial
        if col_id == "raw":
            return [0 if self._raw_on(dev["identity"], self._raw_default()) else 1], serial
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

    def _raw_default(self):
        """이 subset 의 공통 publish_raw (설정 탭 'image_raw 발행')."""
        value = sensor_config._param_value(self.group, (self.subset or {}).get("key"), "publish_raw",
                                           self.overrides)
        return str(value).strip().lower() in ("true", "1", "yes", "on")

    def _raw_on(self, serial, default):
        mine = sensor_config.camera_overrides(self.overrides).get(serial) or {}
        return bool(mine["publish_raw"]) if "publish_raw" in mine else default

    def _on_raw(self, serial, on):
        """raw 칸 — 공통값과 같으면 카메라별 값을 지워 공통 설정을 따르게 한다."""
        picked, is_picked = self._bulk_targets()
        targets = sorted(picked) if is_picked and serial in picked else [serial]
        default = self._raw_default()
        for sn in targets:
            store = self._camera_store(sn)
            if on == default:
                store.pop("publish_raw", None)
            else:
                store["publish_raw"] = on
            self._drop_empty(sn)
        count = sum(1 for d in self._devices if self._raw_on(d["identity"], default))
        names = (f"{len(targets)}대" if len(targets) > 1 else
                 sensor_config.camera_entry(self.subset, serial, self.overrides)["namespace"])
        self._show_table_msg(f"{names}: image_raw {'켬' if on else '끔'} (다음 기동부터)  ·  raw 켠 카메라 "
                             f"{count}대 ≈ {count * RAW_MBPS} MB/s",
                             error=count * RAW_MBPS > RAW_DISK_MBPS)
        self._render_devices()
        self.sig_cameras_edited.emit(self.group["key"])

    def _on_table_item(self, item):
        if self.cameras_editable and item.column() == getattr(self, "_raw_col", None):
            serial = item.data(Qt.UserRole)
            if serial:
                self._on_raw(serial, item.checkState() == Qt.Checked)
            return
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
        rows = [i.row() for i in self.table.selectionModel().selectedRows()] if self.table.selectionModel() else []
        self._picked = {self.table.item(r, self.SERIAL_COL).text() for r in rows
                        if self.table.item(r, self.SERIAL_COL)}
        self._update_bulk_target()
        # 여러 대를 고르면 마지막으로 누른 행(현재 행)을 라이브에
        current = self.table.currentRow()
        row = current if current in rows else (rows[0] if rows else None)
        item = self.table.item(row, self.SERIAL_COL) if row is not None else None
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
        self._update_sync_targets()
        if self.tuning is not None:
            self.tuning.refresh_target()

    def open_tuning(self):
        if self.tuning is not None:
            self.tabs.setCurrentWidget(self.tuning)

    def _tuning_link(self, title):
        """설정 탭의 ISP 상자 위 — 여기서 바꾸면 다음 기동부터라는 것과, 영상을 보며 바로 바꾸는 탭이 따로 있다는 것."""
        frame = QFrame()
        frame.setObjectName("Info")
        row = QHBoxLayout(frame)
        row.setContentsMargins(10, 6, 8, 6)
        text = QLabel(f"🎨  {title} 값은 <b>ISP 튜닝</b> 탭에서 카메라 영상을 보면서 바로 바꿔 볼 수 있습니다 — 맞으면 "
                      "[모든 카메라에 적용] 으로 떠 있는 카메라 전부에 재기동 없이 들어갑니다. 여기서 바꾸면 다음 기동부터 "
                      "(실행 중이면 재기동해야) 들어갑니다.")
        text.setWordWrap(True)
        text.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        row.addWidget(text, 1)
        button = QPushButton("ISP 튜닝 열기")
        button.clicked.connect(self.open_tuning)
        row.addWidget(button, 0, Qt.AlignVCenter)
        return frame

    def set_launch_snapshot(self, snapshot):
        self.launch_snapshot = snapshot
        self._push_failed = set()        # 방금 [모든 카메라에 적용] 에서 카메라가 거부한 칸 이름 — 재기동밖에 없다
        self._refresh_restart_banner()

    def _pending_restart(self):
        """이번 기동 뒤에 바꾼 칸 이름들 — 실행 중인 노드에는 아직 안 들어갔다."""
        snap = self.launch_snapshot or {}
        names = []
        scope = self.card["subset"] or sensor_config.NO_SUBSET
        for sc, now, before in (
                (scope, (self.overrides.get("params") or {}).get(scope) or {},
                 (snap.get("params") or {}).get(scope) or {}),
                ("launch", self.overrides.get("launch_args") or {}, snap.get("launch_args") or {})):
            for key in sorted(set(now) | set(before)):
                if not _same(now.get(key), before.get(key)):
                    row = self.rows.get((sc, key))
                    names.append((row[0].get("label") if row else None) or key)
        mine = {d["identity"] for d in self._devices}
        now_cams = sensor_config.camera_overrides(self.overrides)
        before_cams = (snap.get("cameras") or {})
        for serial in sorted(mine):
            if (now_cams.get(serial) or {}) != (before_cams.get(serial) or {}):
                entry = sensor_config.camera_entry(self.subset, serial, self.overrides) if self.subset else {}
                names.append(f"{entry.get('name') or serial} 이름 · 동기 방식")
        return names

    def _refresh_restart_banner(self):
        if not hasattr(self, "restart_banner"):
            return
        names = self._pending_restart() if (self._locked and self._included and self.launch_snapshot is not None) else []
        if names:
            tuned = {f.get("label") for f in (self.tuning.fields if self.tuning is not None else [])}
            tuned -= getattr(self, "_push_failed", set())
            shown = ", ".join(names[:5]) + (f" 외 {len(names) - 5}개" if len(names) > 5 else "")
            self.restart_text.setText(
                f"⟳ 기동한 뒤 바꾼 설정 {len(names)}개는 아직 실행 중인 센서에 안 들어갔습니다 — 재기동해야 들어갑니다: "
                f"{shown}"
                + (".  (ISP · 노출 값은 🎨 ISP 튜닝 탭의 [모든 카메라에 적용] 으로 재기동 없이 넣을 수 있습니다)"
                   if tuned & set(names) else ""))
        self.restart_banner.setVisible(bool(names))

    def apply_tuned(self, values):
        """[ISP 튜닝] 의 [모든 카메라에 적용] — 튜닝한 값을 이 종류의 설정(모든 카메라, 다음 기동)에 저장하고, 떠 있는
        카메라 전부에 바로 넣어 달라고 스테이지에 알린다 (결과는 applied_live → 튜닝 탭 show_applied)."""
        scope = self.card["subset"] or sensor_config.NO_SUBSET
        store = self._store(scope)
        for key, value in values.items():
            row = self.rows.get((scope, key))
            if row is None:
                continue
            field, editor = row[0], row[1]
            editor.set_value(value)
            if _same(value, sensor_config.base_value(self.group, field, self._base)):
                store.pop(key, None)
            else:
                store[key] = value
        self._refresh_marks()
        self.sig_push_params.emit(self.card["key"], dict(values))

    def applied_live(self, cameras, failed):
        if self.tuning is not None:
            labels = {f["key"]: f.get("label") or f["key"] for f in self.tuning.fields}
            labels.update({f["live_alias"]: f.get("label") or f["key"] for f in self.tuning.fields if f.get("live_alias")})
            self.tuning.show_applied(cameras, failed, labels)
            self._push_failed = {labels.get(key, key) for _node, key, _why in failed}
        self._refresh_restart_banner()

    def _on_start_clicked(self):
        """기동은 그대로 하고, PTP 에 기대는 센서인데 PTP 가 성치 않으면 로그에 남긴다 (막지 않는다)."""
        bar = getattr(self, "ptp_bar", None)
        problem = bar.problem() if bar else None
        if problem:
            self.log.appendPlainText(
                f"[GUI] {problem} — 이 센서는 PTP 에 기댑니다. 그대로 기동하지만, 시각이 틀어지거나 "
                "카메라가 PTP 를 못 맞춰 죽을 수 있습니다 (터미널에서 ptp).")
        self.sig_start.emit(self.card["key"])

    def _update_sync_targets(self):
        """동기 검증 탭에 지금 떠 있는 이 카드의 카메라와 그 동기 방식을 알려 준다."""
        if not getattr(self, "sync_panel", None):
            return
        running = self._locked and self._included
        mine = {d["identity"] for d in self._devices}      # launched 는 센서군 전체(열화상 포함)
        modes = {}
        if running:
            for serial, namespace in self.launched.items():
                if serial in mine:
                    entry = sensor_config.camera_entry(self.subset, serial, self.overrides)
                    modes[namespace] = sensor_config.sync_mode_label(entry)
        fields = {f["key"]: f for f in sensor_config.fields_for(self.group, self.card["subset"])}
        stamp = {name: sensor_config.effective_value(self.group, fields[key], self.overrides, self._base)
                 for name, key in (("mode", "timestamp.mode"), ("grid_hz", "timestamp.trigger_grid_hz"))
                 if key in fields}
        self.sync_panel.set_targets(modes, running, stamp)

    def _bulk_targets(self):
        """동기 방식 일괄 적용 대상 — 표에서 2대 이상 골랐으면 그 카메라들, 아니면 표의 전부."""
        shown = [d["identity"] for d in self._devices]
        picked = [sn for sn in shown if sn in self._picked]
        return (picked, True) if len(picked) > 1 else (shown, False)

    def _update_bulk_target(self):
        if not self.sync_roles:
            return
        serials, picked = self._bulk_targets()
        self.sync_target.setText(f"대상: 고른 {len(serials)}대" if picked else
                                 f"대상: 전체 {len(serials)}대  (Ctrl·Shift 로 여러 대를 고르면 고른 카메라만)")
        self.sync_target.setStyleSheet(f"color:{ui_theme.WARN};" if picked else "")

    def _bulk_sync_mode(self, mode, serials=None):
        """여러 대의 동기 방식을 한 번에. PTP 액션이면 보내는 쪽은 하나 — 표에 이미 보내기인 카메라가 있으면
        (대상 밖이어도) 그대로 두고 대상은 전부 받기, 없으면 대상의 첫 카메라가 보내기."""
        if serials is None:
            serials, _picked = self._bulk_targets()
        if not serials:
            return
        if mode == "ptp":
            shown = [d["identity"] for d in self._devices]
            sender = next((sn for sn in serials + shown if sensor_config.sync_mode(
                sensor_config.camera_entry(self.subset, sn, self.overrides)) == "ptp_sender"), serials[0])
            mode = {sn: "ptp_sender" if sn == sender else "ptp_receiver" for sn in serials}
            name = sensor_config.camera_entry(self.subset, sender, self.overrides)["namespace"]
            text = f"{len(serials)}대 → PTP 액션 (보내기: {name})"
        else:
            label = next(lbl for k, lbl, *_ in sensor_config.SYNC_MODES if k == mode)
            text = f"{len(serials)}대 → {label}"
            mode = dict.fromkeys(serials, mode)
        self._apply_sync_modes(mode, text)

    def _apply_sync_modes(self, modes, text):
        """{시리얼: 동기 방식 키} 를 GUI 설정에 적고 표를 다시 그린다."""
        for sn, key in modes.items():
            sensor_config.set_sync_mode(self.subset, self.overrides, sn, key)
        self._show_table_msg(text + "  (다음 기동부터 적용)" + self._mixed_sync_note(modes))
        self._render_devices()
        self.sig_cameras_edited.emit(self.group["key"])

    def _mixed_sync_note(self, changed):
        """방금 바꾼 카메라(changed) 밖에 다른 동기 방식이 남았으면 '  ·  ⚠ …' — HW 트리거 · PTP 액션 · 자유 실행이
        섞이면 한 순간에 같이 찍히지 않는다. 고른 카메라만 바꿨을 때 나머지를 빠뜨린 걸 알아채게."""
        family = {"hw_trigger": "HW 트리거", "ptp_sender": "PTP 액션", "ptp_receiver": "PTP 액션", "free": "자유 실행"}
        mode_of = {d["identity"]: sensor_config.sync_mode(
            sensor_config.camera_entry(self.subset, d["identity"], self.overrides)) for d in self._devices}
        mine = {family.get(mode_of.get(sn), "기타") for sn in changed}
        rest = {}
        for sn, key in mode_of.items():
            name = family.get(key, "기타")
            if sn not in changed and name not in mine:
                rest[name] = rest.get(name, 0) + 1
        return ("  ·  ⚠ 나머지 " + ", ".join(f"{n} {c}대" for n, c in sorted(rest.items()))) if rest else ""

    def _on_sync_mode(self, serial, key):
        if not key:                # "기타 (…)" 표시용 항목 — 값 아님
            return
        # 여러 대를 고른 채로 그중 한 대의 콤보를 바꾸면 고른 카메라 전부
        picked, is_picked = self._bulk_targets()
        if is_picked and serial in picked:
            if key == "ptp_sender":         # 보내기는 한 대 — 바꾼 그 카메라, 나머지는 받기
                name = sensor_config.camera_entry(self.subset, serial, self.overrides)["namespace"]
                self._apply_sync_modes({sn: "ptp_sender" if sn == serial else "ptp_receiver" for sn in picked},
                                       f"{len(picked)}대 → PTP 액션 (보내기: {name})")
            else:
                self._bulk_sync_mode(key, picked)
            return
        sensor_config.set_sync_mode(self.subset, self.overrides, serial, key)
        label = next(lbl for k, lbl, *_ in sensor_config.SYNC_MODES if k == key)
        default_key = sensor_config.sync_mode(
            sensor_config.default_camera_entry(self.subset, serial))
        self._show_table_msg(f"{serial}: 동기 방식 {label}"
                             + (" (원본값)" if key == default_key else ""))
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
        self._update_sync_targets()
        if self.tuning is not None:
            self.tuning.refresh_target()
        self._refresh_restart_banner()

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
            page.sig_tune.connect(self._start_tuning)
            page.sig_retune.connect(self._restart_tuning)
            page.sig_live_param.connect(self._live_param)
            page.sig_push_params.connect(self._push_params)
            page.sig_restart.connect(self.restart_group)
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
            if page.tuning is not None:
                page.tuning.live.attach(worker)
        if worker is not None and hasattr(worker, "sig_node_param"):
            worker.sig_node_param.connect(self._on_node_param)

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
                          "ip": ip, "mask": mask, "label": card["label"],
                          # 새 IP 로 다시 대답하기까지 기다릴 시간 (sensors.yaml subset 의 force_ip_settle_s)
                          "settle_s": float(sub.get("force_ip_settle_s", 6.0))})
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

    def _start_group(self, group_key, subsets, only=None):
        """only: 이 시리얼들만 띄운다 (ISP 튜닝 — 카메라 한 대). 저장된 설정(띄울 종류)은 건드리지 않는다."""
        group = self.groups[group_key]
        overrides = self._overrides(group_key)
        subs = sensor_config.subsets_of(group)
        devices = (self._result.get(group_key) or {}).get("devices", [])
        if only is not None:
            devices = [d for d in devices if d["identity"] in only]
            overrides = dict(overrides)

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
        # 튜닝 기동은 녹화하려고 띄운 게 아니다 — 다 떠도 녹화 탭으로 넘기거나 토픽 선택 창을 열지 않는다
        self._tuning_launch = getattr(self, "_tuning_launch", {})
        self._tuning_launch[group_key] = only is not None
        self._announced_ready = False
        # 지금 그래프에 있는 발행자는 이번 기동 것이 아니다 (앞 실행의 죽은 노드가 임대 시간 동안 남는다)
        self._stale_pubs[group_key] = set()
        if self._ros is not None:
            try:
                self._stale_pubs[group_key] = self._ros.publisher_gids()
            except Exception:                                  # noqa: BLE001
                pass
        labels = ", ".join(c["label"] for c in self._cards_of(group_key) if c["subset"] in subsets)
        if only is not None:
            labels += f" — ISP 튜닝: {', '.join(launched.get(sn, sn) for sn in only)} 한 대만"
        self.sig_log.emit("GUI", f"기동: {labels}")
        # 이번 기동의 설정 — 나중에 바꾼 칸을 '재기동해야 들어감' 으로 알리고, [⟳ 지금 재기동] 이 같은 구성으로 다시 올린다
        self._launch_snapshot = getattr(self, "_launch_snapshot", {})
        self._launch_snapshot[group_key] = copy.deepcopy(overrides)
        self._launch_args = getattr(self, "_launch_args", {})
        self._launch_args[group_key] = (list(subsets), list(only) if only is not None else None)
        for card in self._cards_of(group_key):
            self.pages[card["key"]].set_launch_snapshot(self._launch_snapshot[group_key])
        self.supervisor.start(group_key, overrides, self._expectations(group_key, subsets), devices)

    def _run_nic_fix(self, card_key, iface, then):
        """라이다 NIC 를 link-local 로 잡고 → 다시 감지 → then()."""
        if getattr(self, "_nic_worker", None) is not None and self._nic_worker.isRunning():
            return
        cameras = self._camera_nic_devices(iface)
        if cameras:
            # 카메라 NIC 에 169.254 주소를 얹으면 카메라 IP 자동 맞춤이 그걸 기준 서브넷으로 잡아 카메라를 옮긴다.
            message = (f"{iface} 에는 카메라 {len(cameras)}대가 붙어 있어 라이다용 link-local 주소를 얹지 않습니다 "
                       "(얹으면 카메라 IP 가 169.254 로 옮겨져 기동이 실패합니다). 라이다가 꽂힌 NIC 를 확인하세요.")
            self.sig_log.emit("ERROR", message)
            self.pages[card_key]._show_table_msg(message, error=True)
            if then:
                then()
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
        """기동할 라이다 카드 중 PC 쪽 NIC 를 잡아야 붙는 게 있으면 (카드 키, NIC).

        라이다를 그 NIC 에서 실제로 본 경우만 자동으로 잡는다. 라이다가 안 보여 'IPv4 없는 유선 NIC' 을 후보로
        짐작한 경우는 건너뛴다 — 2026-09-22 재부팅 직후 카메라 스위치 링크가 늦게 올라와 카메라 NIC(enp3s0f1)가
        그 후보가 됐고, 기동 전 자동 설정이 거기에 169.254 주소를 영구로 얹었다. 이어서 카메라 IP 자동 맞춤이 그
        주소를 기준으로 16대를 169.254.0.x 로 옮겨 전부 기동 실패. 짐작 후보는 [NIC 설정] 을 직접 누를 때만.
        """
        for card in cards:
            page = self.pages[card["key"]]
            if card["group"].get("discovery", {}).get("kind") != "ouster_probe" or not page.fix_nic:
                continue
            if not getattr(page, "fix_nic_seen", False):
                self.sig_log.emit("WARN", f"라이다가 안 보여 NIC 자동 설정은 건너뜁니다 ({page.fix_nic} 은 짐작한 후보 — "
                                          "라이다가 거기 꽂혔다면 라이다 카드의 [NIC 설정] 을 직접 누르세요)")
                continue
            return card["key"], page.fix_nic
        return None

    def _camera_nic_devices(self, iface):
        """지금 감지 결과에서 그 NIC 에 붙어 있는 카메라 (GVCP 카메라 카드 전부)."""
        return [d for page in self.pages.values() if page.cameras_editable
                for d in page._devices if d.get("nic") == iface]

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

    def _start_tuning(self, card_key, serial):
        """[ISP 튜닝] 의 [이 카메라만 기동] — 그 카드의 종류에서 이 카메라 한 대만 띄운다."""
        card = next(c for c in self.cards if c["key"] == card_key)
        group_key = card["group"]["key"]
        if self._active(group_key):
            self.sig_log.emit("WARN", f"{card['group']['label']} 이(가) 실행 중입니다 — 중지한 뒤 튜닝 기동하세요")
            return
        self._fix_ip_then([card], lambda: self._start_group(group_key, [card["subset"]], only=[serial]))

    def _restart_tuning(self, card_key, serial):
        """[ISP 튜닝] 의 [⟳ 재기동] — 내렸다가 같은 카메라 한 대만 다시 올린다 (재기동해야 들어가는 값 적용)."""
        card = next(c for c in self.cards if c["key"] == card_key)
        group_key = card["group"]["key"]
        if not self._active(group_key):
            return self._start_tuning(card_key, serial)
        self._restart_after_stop = getattr(self, "_restart_after_stop", {})
        self._restart_after_stop[group_key] = (card_key, serial)
        self.sig_log.emit("GUI", "ISP 튜닝: 카메라 재기동 — 내렸다가 다시 올립니다")
        self.stop_group(group_key)

    def _live_param(self, card_key, serial, key, value):
        """튜닝 탭에서 바꾼 값을 실행 중인 그 카메라 노드에 넣는다 (카메라 노드는 받는 즉시 카메라에 쓴다)."""
        card = next(c for c in self.cards if c["key"] == card_key)
        page = self.pages[card_key]
        ns = self._launched.get(card["group"]["key"], {}).get(serial)
        if ns is None or self._ros is None or not hasattr(self._ros, "set_node_param"):
            page.tuning.on_result(key, False, "이 카메라가 실행 중이 아니거나 ROS 연결이 없습니다")
            return
        node = f"/{ns}/flir_camera"
        self._live_targets = getattr(self, "_live_targets", {})
        self._live_targets[node] = card_key
        self._ros.set_node_param(node, key, value)

    def restart_group(self, group_key):
        """[⟳ 지금 재기동] — 기동 뒤 바꾼 설정을 넣으려고 같은 구성(종류 · 튜닝 한 대)으로 내렸다 다시 올린다."""
        args = getattr(self, "_launch_args", {}).get(group_key)
        if not args or not self._active(group_key):
            return
        label = self.groups[group_key]["label"]
        if not self._confirm("재기동", f"{label} 을(를) 내렸다가 같은 구성으로 다시 올립니다.",
                             "기동한 뒤 바꾼 설정이 전부 들어갑니다. 다시 뜰 때까지(수십 초) 이 센서군의 데이터가 끊기므로, "
                             "녹화 중이면 녹화에 빈 구간이 생깁니다."):
            return
        subsets, only = args
        cards = [c for c in self._cards_of(group_key) if not c["subset"] or c["subset"] in subsets]
        self._restart_after_stop = getattr(self, "_restart_after_stop", {})
        self._restart_after_stop[group_key] = lambda: self._fix_ip_then(
            cards, lambda: self._start_group(group_key, subsets, only))
        self.sig_log.emit("GUI", f"{label}: 바꾼 설정을 넣으려고 재기동 — 내렸다가 같은 구성으로 다시 올립니다")
        self.stop_group(group_key)

    def _push_params(self, card_key, values):
        """[모든 카메라에 적용] — 떠 있는 이 카드의 카메라 전부에 값을 넣는다 (카메라 노드는 받는 즉시 카메라에 쓴다).
        카메라별로 따로 두는 ISP 값은 없어서(카메라별 항목은 이름 · 동기 역할 · IP 뿐) 다음 기동 값과 어긋나지 않는다."""
        card = next(c for c in self.cards if c["key"] == card_key)
        group_key = card["group"]["key"]
        page = self.pages[card_key]
        mine = {d["identity"] for d in page._devices}
        running = {}
        if self._active(group_key) and card["subset"] in self._included.get(group_key, set()):
            running = {sn: ns for sn, ns in self._launched.get(group_key, {}).items() if sn in mine}
        if not running or not values:
            page.applied_live(0, [])
            return
        if self._ros is None or not hasattr(self._ros, "set_node_param"):
            page.applied_live(len(running), [("ROS", k, "ROS 연결이 없어 실행 중인 카메라에 못 넣음") for k in values])
            return
        fields = {f["key"]: f for f in (page.tuning.fields if page.tuning is not None else [])}
        job = {"card": card_key, "group": group_key, "values": dict(values), "fields": fields, "cams": len(running),
               "pending": set(), "failed": [], "alias_of": {}, "done": False}
        self._node_jobs = getattr(self, "_node_jobs", {})
        for sn, ns in running.items():
            node = f"/{ns}/flir_camera"
            for key, value in values.items():
                job["pending"].add((node, key))
                self._node_jobs[(node, key)] = job
                self._ros.set_node_param(node, key, value)
        self.sig_log.emit("GUI", f"{card['label']}: {len(values)}칸을 떠 있는 카메라 {len(running)}대에 바로 넣습니다")
        QTimer.singleShot(15000, lambda j=job: self._finish_push(j))

    def _push_result(self, job, node, key, ok, reason):
        job["pending"].discard((node, key))
        original = job["alias_of"].get((node, key), key)
        field = job["fields"].get(original)
        # 기동 때 잠겨 등록이 안 된 노드면 늘 등록되는 별칭(live_alias)으로 한 번 더 — 튜닝 탭 on_result 와 같은 규칙
        if not ok and field and field.get("live_alias") and key == original and "declared" in reason:
            alias = field["live_alias"]
            job["pending"].add((node, alias))
            job["alias_of"][(node, alias)] = original
            self._node_jobs[(node, alias)] = job
            self._ros.set_node_param(node, alias, job["values"][original])
            return
        if not ok:
            job["failed"].append((node, original, reason or "거부됨"))
        if not job["pending"]:
            self._finish_push(job)

    def _finish_push(self, job):
        if job["done"]:
            return
        job["done"] = True
        for pending in list(job["pending"]):
            self._node_jobs.pop(pending, None)
            job["failed"].append((pending[0], job["alias_of"].get(pending, pending[1]), "응답 없음 (15초)"))
        page = self.pages[job["card"]]
        failed_keys = {key for _node, key, _why in job["failed"]}
        # 모든 카메라에 들어간 칸은 이미 적용된 것 — 기동 때 설정(스냅숏)에 옮겨 '재기동해야 들어감' 에서 뺀다
        snap = getattr(self, "_launch_snapshot", {}).get(job["group"])
        if snap is not None:
            scope = page.card["subset"] or sensor_config.NO_SUBSET
            store = (page.overrides.get("params") or {}).get(scope) or {}
            params = snap.setdefault("params", {}).setdefault(scope, {})
            for key in job["values"]:
                if key in failed_keys:
                    continue
                if key in store:
                    params[key] = copy.deepcopy(store[key])
                else:
                    params.pop(key, None)
        page.applied_live(job["cams"], job["failed"])
        label = page.card["label"]
        if job["failed"]:
            cams = len({node for node, _k, _w in job["failed"]})
            self.sig_log.emit("WARN", f"{label}: 카메라 {cams}대에 {', '.join(sorted(failed_keys))} 이(가) 바로 안 들어갔습니다 "
                                      f"({job['failed'][0][2]}) — 설정에는 저장됨, 재기동하면 들어갑니다")
        else:
            self.sig_log.emit("OK", f"{label}: 떠 있는 카메라 {job['cams']}대 전부에 바로 들어갔습니다 (설정에도 저장)")

    def _on_node_param(self, node, key, ok, reason):
        job = getattr(self, "_node_jobs", {}).pop((node, key), None)
        if job is not None:
            if not job["done"]:
                self._push_result(job, node, key, ok, reason)
            return
        card_key = getattr(self, "_live_targets", {}).get(node)
        page = self.pages.get(card_key)
        if page is not None and page.tuning is not None:
            page.tuning.on_result(key, ok, reason)
        if not ok:
            self.sig_log.emit("WARN", f"{node} {key}: {reason}")

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

    def start_groups(self, group_keys):
        """다른 탭에서 '이 센서군들을 켜 줘' ([실시간 projection 보기] 등). [기동] 과 같은 길 (IP 맞추기 → 기동).
        센서군 안에서 체크된 카드만 — 하나도 체크 안 돼 있으면 그 센서군의 카드 전부. 반환: 켜기 시작한 센서군 이름."""
        started = []
        for gk in group_keys:
            if gk not in self.groups or self._active(gk):
                continue
            cards = self._cards_of(gk)
            if not cards:
                continue
            if not any(self.card_widgets[c["key"]].check.isChecked() for c in cards):
                for c in cards:
                    self.card_widgets[c["key"]].check.setChecked(True)
            chosen = [c for c in cards if self.card_widgets[c["key"]].check.isChecked()]
            self._fix_ip_then(chosen, lambda gk=gk: self._start_group(gk, self._checked_subsets(gk)))
            started.append(self.groups[gk]["label"])
        if started:
            self.sig_log.emit("GUI", f"다른 탭 요청으로 센서 기동: {', '.join(started)}")
        return started

    def group_state(self, group_key):
        """(한글 상태, 켜져 있나) — stopped/starting/running/failed."""
        st = self.supervisor.state(group_key)
        return {"stopped": "꺼짐", "starting": "켜는 중…", "running": "켜짐", "failed": "기동 실패"}.get(st, st), st == "running"

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
            elif ns in proc.ns_issues and (ns in proc.ns_ready or ns in proc.ns_present):
                issue = proc.ns_issues[ns]
                marks[serial] = (DEGRADED, issue["text"] + (f": {issue['detail']}" if issue["detail"] else ""))
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

        pending = getattr(self, "_restart_after_stop", {}).get(group_key)
        if pending and not self._active(group_key):
            del self._restart_after_stop[group_key]
            QTimer.singleShot(800, pending if callable(pending) else (lambda p=pending: self._start_tuning(*p)))
        active = [k for k in self.groups if self._active(k)]
        all_up = bool(active) and all(self.supervisor.state(k) == RUNNING for k in active)
        tuning = any(getattr(self, "_tuning_launch", {}).get(k) for k in active)
        self.btn_go.setVisible(all_up and not tuning)
        if tuning and all_up and not self._announced_ready:
            self._announced_ready = True
            self.sig_log.emit("OK", "ISP 튜닝용 카메라가 떴습니다 — ISP 튜닝 탭에서 값을 바꾸면 바로 들어갑니다")
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
