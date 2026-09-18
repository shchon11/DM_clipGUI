#!/usr/bin/env python3
# preview_panel.py — 녹화 탭 오른쪽 "센서 미리보기".
#
# 리그 목표 규모는 가시광 14 + 열화상 2 + LiDAR 1 + GNSS 1 = 타일 18개다. 그래서 두 가지를 지킨다.
#
# 1) 계산: [그림] 설정으로 고른다.
#      - 실시간 N fps: 화면에 보이는 카메라 타일만 이미지 토픽을 계속 구독하고, N fps 로 최신 한 장만
#        디코드한다 (받는 건 30 Hz 전부지만 raw 바이트를 담아 두기만 한다).
#      - 느리게/보통/빠르게/1초: "스냅샷" — 구독을 만들어 한 장 받고 바로 내린다. 보이는 타일만, 돌아가며,
#        동시에 MAX_IN_FLIGHT 개까지.
#    예전엔 스냅샷뿐이었다 — 카메라 노드마다 리더가 하나씩 더 붙는 게 부담이었다 (그땐 카메라 노드의
#    디모자이킹 + JPEG 이 CPU 를 거의 다 먹었다). 2026-09-19 부터 그 일을 GPU 가 하고 점보 프레임으로
#    커널 수신도 줄어 CPU 유휴가 3.5% → 77% 가 되어서 실시간을 기본으로 둔다.
#      - Hz 는 /<ns>/camera_info (프레임마다 오는 수백 바이트) 를 계속 받아 실측한다 (모드와 무관).
#      - LiDAR 신호 이미지(256 KB × 10 Hz, 한 대)와 GNSS 값(작은 문자열)은 계속 받는다.
#    받은 데이터는 raw(직렬화된 바이트) 그대로 두고, 그릴 때만 역직렬화·디코드한다.
#    JPEG 은 QImageReader.setScaledSize 로 디코드 단계에서 줄인다 (libjpeg DCT 스케일링).
#    녹화 탭이 안 보이거나 미리보기를 끄면 구독을 전부 내린다.
#    구독 생성/해제는 전부 ROS 스레드에서 한다 (RosWorker.call_in_ros_thread) — GUI 스레드에서
#    하면 executor 가 대기 집합을 만드는 도중에 목록이 바뀌어 스핀 루프가 죽을 수 있다.
#
# 2) 공간: 종류별 섹션(가시광 / 열화상 / LiDAR / GNSS) 에 작은 타일. 섹션 머리에 대수·평균 Hz·
#    끊김 수. 타일 크기는 고를 수 있고, LiDAR 띠와 GNSS 는 한 줄 전체를 쓴다.
#
# 무엇을 타일로 만들지는 ROS 그래프에서 정한다 (퍼블리셔가 실제로 있는 토픽만).

import math
import queue
import time
from collections import deque

import numpy as np
from PyQt5.QtCore import QBuffer, QByteArray, QIODevice, QMimeData, QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QDrag, QImage, QImageReader, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QScrollArea,
    QSizePolicy, QToolButton, QVBoxLayout, QWidget)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import ui_theme

# best_effort 구독은 reliable / best_effort 퍼블리셔 모두와 붙는다. 깊이 1 — 밀린 건 버린다.
PREVIEW_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

LIDAR_IMAGES = ("signal_image", "reflec_image", "nearir_image", "range_image")
STALE_S = 2.0              # 이만큼 안 들어오면 '끊김'
HZ_WINDOW_S = 2.0          # Hz 는 최근 2초 평균
SCAN_PERIOD_MS = 3000      # 그래프(어떤 센서가 있나) 다시 보기
SNAP_TICK_MS = 250         # 스냅샷 스케줄러
STATUS_TICK_MS = 500       # Hz · LiDAR · GNSS 갱신
SNAP_TIMEOUT_S = 2.0       # 스냅샷이 이 안에 안 오면 포기 (카메라가 안 내보내는 중)
MAX_IN_FLIGHT = 4          # 동시에 열어 두는 스냅샷 구독 수

# [그림] 설정: ("live", 초당 그리는 장수) 또는 ("snap", 카메라 한 대가 새 그림을 받는 간격 초)
CADENCE = {"실시간 15 fps": ("live", 15), "실시간 10 fps": ("live", 10), "실시간 5 fps": ("live", 5),
           "1초": ("snap", 1.0), "빠르게": ("snap", 2.0), "보통": ("snap", 4.0), "느리게": ("snap", 8.0)}
DEFAULT_CADENCE = "실시간 5 fps"
BUDGET_VERSION = 2         # 예산이 바뀐 판 — 예전 빠듯한 예산으로 고른 설정을 한 번 새 기본값으로 올린다
TILE_WIDTH = {"작게": 150, "보통": 210, "크게": 300}       # 카메라 타일 최소 폭(px)
SECTIONS = (("visible", "가시광 카메라"), ("thermal", "열화상"), ("lidar", "LiDAR"), ("gnss", "GNSS"))
ASPECT = {"visible": 1200 / 1920, "thermal": 480 / 640, "lidar": 0.25}   # 첫 그림 전 자리 높이
TILE_MIME = "application/x-dmclip-tile"        # 타일 끌어 옮기기


def merge_order(saved, displayed):
    """저장할 순서 = 지금 보이는 순서, 단 지금 안 보이는 센서는 원래 바로 앞에 있던 센서 뒤에 끼운다.

    그래야 잠깐 뺀 카메라를 다시 꽂았을 때 제자리로 돌아온다. 새 센서는 저장된 순서에 없으니
    (정렬할 때) 섹션 끝에 붙는다.
    """
    shown = set(displayed)
    result = list(displayed)
    prev = None
    for key in saved:
        if key in shown:
            prev = key
            continue
        if key in result:
            continue
        result.insert(result.index(prev) + 1 if prev in result else 0, key)
        prev = key
    return result


def _inferno_lut():
    """열화상용 256단계 컬러맵 (inferno 근사) — 한 번만 만든다."""
    anchors = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
    rgb = np.array([[0, 0, 4], [87, 16, 110], [188, 55, 84], [249, 142, 9], [252, 255, 164]],
                   dtype=float)
    x = np.linspace(0.0, 1.0, 256)
    return np.stack([np.interp(x, anchors, rgb[:, c]) for c in range(3)], axis=1).astype(np.uint8)


INFERNO = _inferno_lut()


# ---------------- 구독 ----------------

class Meter:
    """계속 받는 raw 구독 하나. ROS 스레드는 push 만, GUI 스레드는 count/latest 만 읽는다.

    GUI 스레드가 목록을 순회하지 않게 정수 카운터와 참조 하나만 둔다 (파이썬 대입은 원자적).
    """

    __slots__ = ("count", "latest", "latest_t")

    def __init__(self):
        self.count = 0
        self.latest = None
        self.latest_t = 0.0

    def push(self, data):
        self.latest = data
        self.latest_t = time.monotonic()
        self.count += 1


class RateTracker:
    """GUI 스레드 전용: (시각, 누적 개수) 표본으로 최근 HZ_WINDOW_S 평균 Hz."""

    def __init__(self):
        self.samples = deque()

    def update(self, count, now):
        self.samples.append((now, count))
        while len(self.samples) > 2 and now - self.samples[0][0] > HZ_WINDOW_S:
            self.samples.popleft()
        if len(self.samples) < 2:
            return None
        (t0, c0), (t1, c1) = self.samples[0], self.samples[-1]
        return (c1 - c0) / (t1 - t0) if t1 > t0 else None


class PreviewHub:
    """미리보기 구독. 계속 받는 것(meters)과 한 장만 받는 것(스냅샷) 두 종류.

    생성/해제는 전부 ROS 스레드에 맡긴다. ROS 스레드 전용 상태는 _ 로 시작한다.
    """

    def __init__(self, worker):
        self.worker = worker
        self.meters = {}                       # GUI: topic -> Meter
        self.in_flight = set()                 # GUI: 스냅샷 요청 중인 토픽
        self.snap_results = queue.SimpleQueue()  # ROS → GUI: (topic, data | None)
        self._subs = {}                        # ROS: topic -> Subscription
        self._snaps = {}                       # ROS: topic -> [Subscription, 시작 시각, data]

    # --- 계속 받는 구독 ---

    def sync(self, wanted):
        """wanted: {topic: type 문자열}. 없는 건 만들고 빠진 건 내린다."""
        wanted = dict(wanted)
        for topic in wanted:
            self.meters.setdefault(topic, Meter())
        for topic in [t for t in self.meters if t not in wanted]:
            self.meters.pop(topic)
        meters = dict(self.meters)
        self.worker.call_in_ros_thread(lambda: self._apply(wanted, meters))

    def _apply(self, wanted, meters):          # ROS 스레드
        node = self.worker.node
        for topic in [t for t in self._subs if t not in wanted]:
            node.destroy_subscription(self._subs.pop(topic))
        for topic, type_name in wanted.items():
            if topic in self._subs or topic not in meters:
                continue
            try:
                cls = get_message(type_name)
            except Exception:                                    # noqa: BLE001
                continue                                         # 타입 패키지가 안 보이는 토픽
            self._subs[topic] = node.create_subscription(
                cls, topic, meters[topic].push, PREVIEW_QOS, raw=True)

    # --- 스냅샷: 한 장 받고 바로 내린다 ---

    def snapshot(self, topic, type_name):
        if topic in self.in_flight:
            return
        self.in_flight.add(topic)
        self.worker.call_in_ros_thread(lambda: self._snap_start(topic, type_name))

    def _snap_start(self, topic, type_name):   # ROS 스레드
        if topic in self._snaps:
            return
        try:
            cls = get_message(type_name)
        except Exception:                                        # noqa: BLE001
            self.snap_results.put((topic, None))
            return
        slot = [None, time.monotonic(), None]

        def keep(data, slot=slot):
            slot[2] = data

        slot[0] = self.worker.node.create_subscription(cls, topic, keep, PREVIEW_QOS, raw=True)
        self._snaps[topic] = slot

    def reap(self):
        """받았거나 시간이 다 된 스냅샷 구독을 내리라고 ROS 스레드에 부탁한다."""
        self.worker.call_in_ros_thread(self._reap)

    def _reap(self):                           # ROS 스레드
        now = time.monotonic()
        for topic, (sub, started, data) in list(self._snaps.items()):
            if data is not None or now - started > SNAP_TIMEOUT_S:
                self.worker.node.destroy_subscription(sub)
                del self._snaps[topic]
                self.snap_results.put((topic, data))

    def take_snapshots(self):                  # GUI 스레드
        out = []
        while True:
            try:
                topic, data = self.snap_results.get_nowait()
            except queue.Empty:
                return out
            self.in_flight.discard(topic)
            out.append((topic, data))

    def clear(self):
        self.sync({})
        self.worker.call_in_ros_thread(self._drop_snaps)

    def _drop_snaps(self):                     # ROS 스레드
        for topic, (sub, _started, _data) in list(self._snaps.items()):
            self.worker.node.destroy_subscription(sub)
            self.snap_results.put((topic, None))
        self._snaps.clear()


# ---------------- 디코더 (GUI 스레드, 새 데이터가 왔을 때만) ----------------

def decode_jpeg(data, width):
    """CompressedImage 바이트 → 가로 width 로 줄인 QImage (디코드 단계에서 축소)."""
    msg = deserialize_message(data, get_message("sensor_msgs/msg/CompressedImage"))
    raw = QByteArray(bytes(msg.data))
    buf = QBuffer(raw)
    buf.open(QIODevice.ReadOnly)
    reader = QImageReader(buf)
    full = reader.size()
    if full.isValid() and full.width() > width:
        reader.setScaledSize(QSize(width, max(1, round(full.height() * width / full.width()))))
    image = reader.read()
    return image, "", f"{full.width()}×{full.height()} {msg.format or ''} · {len(raw) / 1024:.0f} KB"


def _to_qimage_gray(gray):
    gray = np.ascontiguousarray(gray)
    h, w = gray.shape
    return QImage(gray.data, w, h, w, QImage.Format_Grayscale8).copy()


def _to_qimage_rgb(rgb):
    rgb = np.ascontiguousarray(rgb)
    h, w, _ = rgb.shape
    return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()


def _stretch(values):
    """1~99 백분위로 늘려 0..255 — 한두 픽셀의 극값에 끌려가지 않게."""
    lo, hi = np.percentile(values, (1, 99))
    if hi <= lo:
        hi = lo + 1
    return np.clip((values.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)


def decode_image(data, width, colormap=False, temp_scale=None):
    """sensor_msgs/Image 바이트 → (QImage, 한 줄 값, 툴팁). mono16 · mono8 · rgb/bgr · 베이어.

    temp_scale 이 있으면 (A70 TemperatureLinear10mK → 0.01) 중앙/최소/최대 온도를 낸다.
    """
    msg = deserialize_message(data, get_message("sensor_msgs/msg/Image"))
    w, h, enc, step = msg.width, msg.height, msg.encoding.lower(), msg.step
    if not w or not h:
        return None, "빈 이미지", ""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    k = max(1, math.ceil(w / max(1, width)))      # 솎아내기 간격
    tip = f"{w}×{h} {msg.encoding}"

    if enc in ("mono16", "16uc1"):
        dtype = ">u2" if msg.is_bigendian else "<u2"
        full = buf.view(dtype).reshape(h, step // 2)[:, :w]
        small = full[::k, ::k]
        stretched = _stretch(small)
        image = _to_qimage_rgb(INFERNO[stretched]) if colormap else _to_qimage_gray(stretched)
        line = ""
        if temp_scale:
            c = lambda v: float(v) * temp_scale - 273.15              # noqa: E731
            line = (f"중앙 {c(full[h // 2, w // 2]):.1f}°C · "
                    f"{c(small.min()):.1f} ~ {c(small.max()):.1f}°C")
        return image, line, tip
    if enc in ("mono8", "8uc1"):
        return _to_qimage_gray(buf.reshape(h, step)[:, :w][::k, ::k]), "", tip
    if enc in ("rgb8", "bgr8"):
        rgb = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)[::k, ::k]
        return _to_qimage_rgb(rgb[..., ::-1] if enc == "bgr8" else rgb), "", tip
    if enc.startswith("bayer_") and enc.endswith("8"):
        # 2x2 패턴에서 한 칸만 — 미리보기로는 흑백이면 충분하고 디모자이크보다 훨씬 싸다
        return _to_qimage_gray(buf.reshape(h, step)[:, :w][::2 * k, ::2 * k]), "", tip
    return None, f"미리보기 미지원: {msg.encoding}", tip


# ---------------- 타일 · 섹션 ----------------

class PreviewTile(QFrame):
    """센서 하나: [이름 · Hz] 한 줄 + 그림(또는 값) + (열화상/GNSS) 값 한 줄."""

    def __init__(self, spec, parent=None):
        super().__init__(parent)
        self.spec = spec
        self.key = spec["key"]                  # 순서 저장용 — 카메라는 시리얼 기준 (cam:<serial>)
        self._press = None
        self.setObjectName("Tile")
        self.setCursor(Qt.OpenHandCursor)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 5, 6, 6)
        v.setSpacing(4)

        head = QHBoxLayout()
        head.setSpacing(4)
        self.name = QLabel(spec["title"])
        self.name.setObjectName("TileName")
        self.name.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.hz = QLabel("대기")
        self.hz.setObjectName("PillSmall")
        self.hz.setStyleSheet(ui_theme.pill_css("stopped"))
        head.addWidget(self.name, 1)
        head.addWidget(self.hz)
        v.addLayout(head)

        self.view = QLabel()
        self.view.setAlignment(Qt.AlignCenter)
        if spec["section"] == "gnss":
            self.view.setTextFormat(Qt.RichText)
            self.view.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.view.setWordWrap(True)
        else:
            self.view.setObjectName("TileImage")
            self.view.setText("…")
        v.addWidget(self.view)

        self.line = QLabel("")
        self.line.setObjectName("Hint")
        self.line.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.line.setVisible(spec["section"] in ("thermal", "lidar"))
        v.addWidget(self.line)

        self.rate = RateTracker()
        self.last_snap = 0.0          # GUI: 마지막 스냅샷 요청 시각
        self.drawn = None             # 마지막으로 그린 바이트 (같은 걸 두 번 풀지 않는다)
        self.image = None             # 마지막 디코드 결과 (폭이 바뀌면 다시 풀지 않고 늘이기만)
        self.aspect = ASPECT.get(spec["section"], 0.6)
        self.image_width = 100

    def set_width(self, width):
        self.setFixedWidth(width)
        self.image_width = max(40, width - 12)
        if self.spec["section"] != "gnss":
            self.view.setFixedHeight(max(20, round(self.image_width * self.aspect)))
            self._paint()

    def set_hz(self, hz, age):
        if hz is None and age is None:
            text, state = "대기", "stopped"
        elif age is not None and age > STALE_S:
            text, state = f"끊김 {age:.0f}s", "failed"
        else:
            text, state = (f"{hz:.1f} Hz" if hz is not None else "…"), "running"
        self.hz.setText(text)
        self.hz.setStyleSheet(ui_theme.pill_css(state))
        return state

    def set_image(self, image, line, tip):
        if image is None or image.isNull():
            self.view.setText(line or "디코드 실패")
            return
        self.image = image
        if self.spec["section"] != "lidar":                  # LiDAR 띠는 세로로 늘려 보여준다
            self.aspect = image.height() / image.width()
            self.view.setFixedHeight(max(20, round(self.image_width * self.aspect)))
        self.line.setText(line)
        serial = f"시리얼 {self.spec['serial']}\n" if self.spec.get("serial") else ""
        self.setToolTip(f"{serial}{self.spec['image_topic']}\n{tip}\n끌어서 순서 변경".strip())
        self._paint()

    def _paint(self):
        if self.image is None:
            return
        pix = QPixmap.fromImage(self.image).scaled(
            self.image_width, self.view.height(),
            Qt.IgnoreAspectRatio if self.spec["section"] == "lidar" else Qt.KeepAspectRatio,
            Qt.SmoothTransformation)
        self.view.setPixmap(pix)

    def set_text(self, html, tip=""):
        self.view.setText(html)
        self.setToolTip(tip)

    def visible_on_screen(self):
        return self.isVisible() and not self.visibleRegion().isEmpty()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._press = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._press is None or not (event.buttons() & Qt.LeftButton):
            return
        if (event.pos() - self._press).manhattanLength() < QApplication.startDragDistance():
            return
        self._press = None
        mime = QMimeData()
        mime.setData(TILE_MIME, self.key.encode())
        drag = QDrag(self)
        drag.setMimeData(mime)
        pix = self.grab()
        drag.setPixmap(pix.scaledToWidth(min(160, pix.width()), Qt.SmoothTransformation))
        drag.exec_(Qt.MoveAction)

    def set_drop_target(self, on):
        self.setProperty("dropTarget", "true" if on else "false")
        ui_theme.repolish(self)


class Section(QWidget):
    """종류 하나: 머리(이름 · 대수 · 평균 Hz · 끊김) + 타일 격자. 섹션 안에서 끌어서 순서를 바꾼다."""

    sig_reorder = pyqtSignal(str, list)        # section key, 새 타일 키 순서

    def __init__(self, key, title, parent=None):
        super().__init__(parent)
        self.key = key
        self.title = title
        self.setAcceptDrops(True)
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(5)
        head = QHBoxLayout()
        self.label = QLabel(title)
        self.label.setObjectName("Section")
        self.summary = QLabel("")
        self.summary.setObjectName("Hint")
        head.addWidget(self.label)
        head.addWidget(self.summary, 1)
        v.addLayout(head)
        self.grid = QGridLayout()
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(6)
        self.grid.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        v.addLayout(self.grid)
        self.tiles = []

    def place(self, tiles, width, min_tile):
        """폭에 맞춰 열 수를 정하고 타일 폭을 행에 꽉 차게."""
        self.tiles = tiles
        while self.grid.count():
            self.grid.takeAt(0)
        self.setVisible(bool(tiles))
        if not tiles:
            return
        spacing = self.grid.spacing()
        full_row = self.key in ("lidar", "gnss")
        cols = 1 if full_row else max(1, (width + spacing) // (min_tile + spacing))
        tile_w = max(80, (width - spacing * (cols - 1)) // cols)
        for i, tile in enumerate(tiles):
            tile.set_width(tile_w)
            self.grid.addWidget(tile, i // cols, i % cols)

    # --- 끌어 놓기 ---

    def _dragged_key(self, event):
        if not event.mimeData().hasFormat(TILE_MIME):
            return None
        key = bytes(event.mimeData().data(TILE_MIME)).decode()
        return key if key in [t.key for t in self.tiles] else None   # 다른 섹션 타일은 안 받는다

    def _tile_at(self, pos):
        widget = self.childAt(pos)
        while widget is not None and not isinstance(widget, PreviewTile):
            widget = widget.parentWidget()
        return widget

    def _mark(self, target):
        for tile in self.tiles:
            tile.set_drop_target(tile is target)

    def dragEnterEvent(self, event):
        if self._dragged_key(event):
            event.acceptProposedAction()

    def dragMoveEvent(self, event):
        if self._dragged_key(event):
            self._mark(self._tile_at(event.pos()))
            event.acceptProposedAction()

    def dragLeaveEvent(self, event):
        self._mark(None)

    def dropEvent(self, event):
        key = self._dragged_key(event)
        self._mark(None)
        if not key:
            return
        target = self._tile_at(event.pos())
        if target is not None and target.key == key:
            return                                # 제자리에 놓음
        keys = [t.key for t in self.tiles if t.key != key]
        if target is None:
            keys.append(key)                      # 빈 곳에 놓으면 맨 끝으로
        else:
            index = keys.index(target.key)
            if target.mapFrom(self, event.pos()).x() > target.width() / 2:
                index += 1                      # 대상 타일 오른쪽 절반에 놓으면 그 뒤로
            keys.insert(index, key)
        event.acceptProposedAction()
        self.sig_reorder.emit(self.key, keys)

    def set_summary(self, live, stale, hz_values):
        parts = [f"{len(self.tiles)}대"]
        if hz_values:
            parts.append(f"평균 {sum(hz_values) / len(hz_values):.1f} Hz")
        if stale:
            parts.append(f"끊김 {stale}")
        self.summary.setText("  ·  ".join(parts))
        self.summary.setStyleSheet(f"color:{ui_theme.ERR};" if stale else "")


# ---------------- 패널 ----------------

class PreviewPanel(QWidget):
    """녹화 탭 오른쪽 전체. 켜져 있고 화면에 보일 때만 구독한다."""

    def __init__(self, worker, cfg, stage, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.cfg = cfg
        self.stage = stage
        self.hub = PreviewHub(worker)
        self.tiles = {}             # key(이미지 토픽) -> PreviewTile
        self.gnss_topics = {}       # {"status", "pos_type", "fix", "vel": topic}
        self._last_wanted = None
        self._active = False
        self._by_topic = {}
        ui = cfg.setdefault("ui", {})
        self._build(ui)
        self.scan_timer = QTimer(self, interval=SCAN_PERIOD_MS, timeout=self._scan)
        self.snap_timer = QTimer(self, interval=SNAP_TICK_MS, timeout=self._snap_tick)
        self.status_timer = QTimer(self, interval=STATUS_TICK_MS, timeout=self._status_tick)
        self.live_timer = QTimer(self, timeout=self._live_tick)       # 실시간 모드에서 그리기
        self._base_wanted = {}       # 계속 받는 것 (Hz · LiDAR · GNSS) — 실시간 이미지는 여기에 더한다
        self._apply_cadence()

    # --- UI ---

    def _build(self, ui):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(6)

        head = QHBoxLayout()
        title = QLabel("센서 미리보기")
        title.setObjectName("Section")
        self.enabled = QCheckBox("켜기")
        self.enabled.setChecked(bool(ui.get("preview_enabled", True)))
        self.enabled.setToolTip("끄면 미리보기 구독을 전부 내립니다 (데이터가 안 흐름)")
        self.enabled.toggled.connect(self._on_toggle)
        self.cadence = QComboBox()
        self.cadence.addItems(list(CADENCE))
        if ui.get("preview_budget") != BUDGET_VERSION:
            ui["preview_cadence"] = DEFAULT_CADENCE          # 예전 빠듯한 예산에서 고른 값 → 한 번 올린다
            ui["preview_budget"] = BUDGET_VERSION
        self.cadence.setCurrentText(ui.get("preview_cadence", DEFAULT_CADENCE))
        self.cadence.setToolTip(
            "실시간 N fps: 화면에 보이는 카메라를 계속 받아 초당 N장 그립니다.\n"
            "1초 / 빠르게 2초 / 보통 4초 / 느리게 8초: 그 간격으로 한 장씩만 받습니다 (부하가 가장 작음).\n"
            "Hz 는 이와 상관없이 실시간입니다. LiDAR·GNSS 는 늘 실시간.")
        self.cadence.currentTextChanged.connect(self._on_cadence)
        self.size = QComboBox()
        self.size.addItems(list(TILE_WIDTH))
        self.size.setCurrentText(ui.get("preview_tile", "보통"))
        self.size.setToolTip("카메라 타일 크기")
        self.size.currentTextChanged.connect(self._on_size)
        reset = QToolButton()
        reset.setText("순서 초기화")
        reset.setToolTip("끌어서 바꾼 타일 순서를 지우고 이름순으로")
        reset.clicked.connect(self._reset_order)
        head.addWidget(title)
        head.addStretch()
        head.addWidget(reset)
        head.addWidget(QLabel("그림"))
        head.addWidget(self.cadence)
        head.addWidget(QLabel("타일"))
        head.addWidget(self.size)
        head.addWidget(self.enabled)
        outer.addLayout(head)

        self.summary = QLabel("")
        self.summary.setObjectName("Hint")
        outer.addWidget(self.summary)

        self.body = QWidget()
        bv = QVBoxLayout(self.body)
        bv.setContentsMargins(0, 0, 4, 0)
        bv.setSpacing(12)
        self.sections = {}
        for key, name in SECTIONS:
            section = Section(key, name)
            section.sig_reorder.connect(self._on_reorder)
            section.hide()
            self.sections[key] = section
            bv.addWidget(section)
        self.empty = QLabel("")
        self.empty.setObjectName("Hint")
        self.empty.setAlignment(Qt.AlignCenter)
        self.empty.setWordWrap(True)
        bv.addWidget(self.empty)
        bv.addStretch(1)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setWidget(self.body)
        outer.addWidget(self.scroll, 1)
        self._update_empty()

    # --- 켜고 끄기: 보일 때만 구독 ---

    def showEvent(self, event):
        super().showEvent(event)
        self._set_active(self.enabled.isChecked())

    def hideEvent(self, event):
        super().hideEvent(event)
        self._set_active(False)

    def _on_toggle(self, on):
        self.cfg["ui"]["preview_enabled"] = on
        self._set_active(on and self.isVisible())

    def _on_cadence(self, text):
        self.cfg["ui"]["preview_cadence"] = text
        self._apply_cadence()
        self._sync_wanted()

    def _mode(self):
        return CADENCE.get(self.cadence.currentText(), CADENCE[DEFAULT_CADENCE])

    def _apply_cadence(self):
        kind, value = self._mode()
        if kind == "live":
            self.live_timer.setInterval(max(20, int(1000 / value)))
            if self._active and not self.live_timer.isActive():
                self.live_timer.start()
        else:
            self.live_timer.stop()

    def _on_size(self, text):
        self.cfg["ui"]["preview_tile"] = text
        self._relayout()

    def _set_active(self, on):
        if on == self._active:
            return
        self._active = on
        if on:
            self._scan()
            for timer in (self.scan_timer, self.snap_timer, self.status_timer):
                timer.start()
            self._apply_cadence()
        else:
            for timer in (self.scan_timer, self.snap_timer, self.status_timer, self.live_timer):
                timer.stop()
            self.hub.clear()
            self._last_wanted = None
            for tile in self.tiles.values():
                tile.set_hz(None, None)
        self._update_empty()

    def shutdown(self):
        self._set_active(False)

    # --- 어떤 센서가 있나 (ROS 그래프) ---

    def _live_topics(self):
        node = self.worker.node
        out = {}
        for name, types in node.get_topic_names_and_types():
            if types and node.count_publishers(name) > 0:       # 죽은 노드의 흔적은 뺀다
                out[name] = types[0]
        return out

    def _plan(self, topics):
        """{key: spec} — spec: section, title, image_topic, image_type, hz_topic, mode."""
        labels = self.stage.namespace_labels() if self.stage else {}
        serials = self.stage.namespace_serials() if self.stage else {}
        temp_scale = self.stage.thermal_temperature_scale() if self.stage else None
        plans = {}
        namespaces = sorted({n.strip("/").split("/")[0] for n in topics if n.count("/") >= 2})
        for ns in namespaces:
            base = f"/{ns}"
            jpeg, raw, info = f"{base}/image_rgb/compressed", f"{base}/image_raw", f"{base}/camera_info"
            has_jpeg = topics.get(jpeg) == "sensor_msgs/msg/CompressedImage"
            has_raw = topics.get(raw) == "sensor_msgs/msg/Image" and f"{raw}/metadata" in topics
            if has_jpeg or has_raw:
                label = labels.get(ns, "")
                thermal = "열화상" in label or (not label and not has_jpeg and ns.startswith("thermal"))
                image = jpeg if has_jpeg else raw
                serial = serials.get(ns)
                key = f"cam:{serial}" if serial else f"ns:{ns}"
                plans[key] = {
                    "key": key, "serial": serial,
                    "section": "thermal" if thermal else "visible", "title": ns,
                    "image_topic": image, "image_type": topics[image], "mode": "snapshot",
                    "hz_topic": info if topics.get(info) == "sensor_msgs/msg/CameraInfo" else None,
                    "temp_scale": temp_scale if thermal else None}
                continue
            if f"{base}/lidar_packets" in topics or f"{base}/points" in topics:
                image = next((f"{base}/{n}" for n in LIDAR_IMAGES
                              if topics.get(f"{base}/{n}") == "sensor_msgs/msg/Image"), None)
                if image:
                    plans[f"lidar:{ns}"] = {"key": f"lidar:{ns}", "section": "lidar", "title": ns,
                                    "image_topic": image,
                                    "image_type": topics[image], "mode": "stream", "hz_topic": image}

        ui = self.cfg.get("ui", {})
        fix = ui.get("gnss_fix_topic")
        status_topics = [t for t in ui.get("gnss_status_topics") or [] if t in topics]
        if (fix and fix in topics) or status_topics:
            gnss = {"fix": fix if topics.get(fix) == "sensor_msgs/msg/NavSatFix" else None,
                    "vel": ui.get("gnss_vel_topic") if ui.get("gnss_vel_topic") in topics else None}
            for t in status_topics:
                gnss[t.rsplit("/", 1)[-1]] = t          # pos_type, nav_status
            self.gnss_topics = {k: v for k, v in gnss.items() if v}
            # 위치 해가 없어도 NCOM 은 흐른다 — Hz 는 nav_status 에서 (없으면 fix)
            hz = self.gnss_topics.get("nav_status") or self.gnss_topics.get("fix")
            plans["gnss"] = {"key": "gnss", "section": "gnss",
                             "title": (fix or hz).strip("/").split("/")[0],
                             "image_topic": hz, "mode": "gnss", "hz_topic": hz}
        else:
            self.gnss_topics = {}
        return plans

    def _scan(self):
        if not self._active:
            return
        try:
            topics = self._live_topics()
        except Exception:                                        # noqa: BLE001
            return
        plans = self._plan(topics)

        # 계속 받을 것: 카메라 camera_info(Hz), LiDAR 이미지, GNSS 값들
        wanted = {}
        for spec in plans.values():
            if spec.get("hz_topic"):
                wanted[spec["hz_topic"]] = topics[spec["hz_topic"]]
            if spec["mode"] == "stream":
                wanted[spec["image_topic"]] = spec["image_type"]
        for topic in self.gnss_topics.values():
            wanted[topic] = topics[topic]
        self._base_wanted = wanted

        self._by_topic = {spec["image_topic"]: key for key, spec in plans.items()}
        for key in [k for k in self.tiles if k not in plans]:
            self.tiles.pop(key).deleteLater()
        for key, spec in plans.items():
            tile = self.tiles.get(key)
            if tile is None or tile.spec["section"] != spec["section"]:
                if tile is not None:
                    tile.deleteLater()
                self.tiles[key] = PreviewTile(spec)
            else:
                tile.spec = spec                   # 온도 배율 등이 바뀌었을 수 있다
        self._relayout()
        self._update_empty()
        self._sync_wanted()

    def _live_tiles(self):
        """실시간 모드에서 계속 받을 카메라 타일 = 화면에 보이는 것만 (스크롤 밖은 안 받는다)."""
        if self._mode()[0] != "live":
            return []
        return [t for t in self.tiles.values() if t.spec["mode"] == "snapshot" and t.visible_on_screen()]

    def _sync_wanted(self):
        """계속 받을 구독 = 기본(Hz · LiDAR · GNSS) + 실시간 모드의 보이는 카메라 이미지. 바뀔 때만 다시 건다."""
        if not self._active:
            return
        wanted = dict(self._base_wanted)
        for tile in self._live_tiles():
            wanted[tile.spec["image_topic"]] = tile.spec["image_type"]
        if wanted != self._last_wanted:
            self._last_wanted = dict(wanted)
            self.hub.sync(wanted)

    def _live_tick(self):
        """실시간 모드: 새로 온 최신 한 장만 디코드한다 (그 사이 온 프레임은 건너뜀)."""
        for tile in self._live_tiles():
            meter = self.hub.meters.get(tile.spec["image_topic"])
            if meter is not None and meter.latest is not None and meter.latest is not tile.drawn:
                self._draw(tile, meter.latest)

    def _relayout(self):
        width = self.scroll.viewport().width() - 6
        min_tile = TILE_WIDTH.get(self.size.currentText(), 210)
        order = self.cfg["ui"].get("preview_order") or []
        pos = {k: i for i, k in enumerate(order)}
        for key, section in self.sections.items():
            tiles = sorted((t for t in self.tiles.values() if t.spec["section"] == key),
                           key=lambda t: (pos.get(t.key, len(pos)), t.spec["title"]))
            section.place(tiles, width, min_tile)

    def _on_reorder(self, section_key, keys):
        displayed = []
        for key, _name in SECTIONS:
            displayed += keys if key == section_key else [t.key for t in self.sections[key].tiles]
        self.cfg["ui"]["preview_order"] = merge_order(self.cfg["ui"].get("preview_order") or [],
                                                      displayed)
        self._relayout()

    def _reset_order(self):
        self.cfg["ui"]["preview_order"] = []
        self._relayout()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._relayout()

    def _update_empty(self):
        if not self.enabled.isChecked():
            text = "미리보기가 꺼져 있습니다."
        elif not self.tiles:
            text = "미리보기할 센서 토픽이 없습니다.\n[센서 기동] 탭에서 센서를 띄우세요."
        else:
            text = ""
        self.empty.setText(text)
        self.empty.setVisible(bool(text))

    # --- 스냅샷: 보이는 타일만, 돌아가며, 동시에 MAX_IN_FLIGHT 개까지 ---

    def _snap_tick(self):
        self.hub.reap()
        for topic, data in self.hub.take_snapshots():
            tile = self.tiles.get(self._by_topic.get(topic))
            if tile is not None and data is not None:
                self._draw(tile, data)

        self._sync_wanted()                 # 스크롤로 보이는 타일이 바뀌었으면 실시간 구독도 따라간다
        kind, period = self._mode()
        if kind == "live":
            return
        now = time.monotonic()
        free = MAX_IN_FLIGHT - len(self.hub.in_flight)
        if free <= 0:
            return
        due = [t for t in self.tiles.values()
               if t.spec["mode"] == "snapshot" and t.spec["image_topic"] not in self.hub.in_flight
               and now - t.last_snap >= period and t.visible_on_screen()]
        for tile in sorted(due, key=lambda t: t.last_snap)[:free]:
            tile.last_snap = now
            self.hub.snapshot(tile.spec["image_topic"], tile.spec["image_type"])

    # --- Hz · LiDAR · GNSS (계속 받는 것) ---

    def _status_tick(self):
        now = time.monotonic()
        per_section = {key: [0, 0, []] for key, _ in SECTIONS}     # live, stale, hz 목록
        for tile in self.tiles.values():
            spec = tile.spec
            meter = self.hub.meters.get(spec.get("hz_topic"))
            if meter is None:
                state = tile.set_hz(None, None)
            else:
                hz = tile.rate.update(meter.count, now)
                age = now - meter.latest_t if meter.latest_t else None
                state = tile.set_hz(hz if meter.count else None, age)
                if state == "running" and hz:
                    per_section[spec["section"]][2].append(hz)
            stats = per_section[spec["section"]]
            if state == "running":
                stats[0] += 1
            elif state == "failed":
                stats[1] += 1

            if spec["mode"] == "stream" and meter is not None and meter.latest is not tile.drawn:
                self._draw(tile, meter.latest)
            elif spec["mode"] == "gnss":
                tile.set_text(*self._gnss_text())

        total_stale = 0
        for key, section in self.sections.items():
            live, stale, hz_values = per_section[key]
            section.set_summary(live, stale, hz_values)
            total_stale += stale
        counts = [f"{name} {len(self.sections[key].tiles)}" for key, name in SECTIONS
                  if self.sections[key].tiles]
        self.summary.setText("  ·  ".join(counts) + (f"   —   끊김 {total_stale}" if total_stale else ""))
        self.summary.setStyleSheet(f"color:{ui_theme.ERR};" if total_stale else "")

    def _draw(self, tile, data):
        if data is None:
            return
        tile.drawn = data
        spec = tile.spec
        try:
            if spec["image_type"] == "sensor_msgs/msg/CompressedImage":
                tile.set_image(*decode_jpeg(data, tile.image_width))
            elif spec["section"] == "lidar":
                image, line, tip = decode_image(data, tile.image_width)
                tile.set_image(image, spec["image_topic"].rsplit("/", 1)[-1], tip)
            else:
                tile.set_image(*decode_image(data, tile.image_width, colormap=True,
                                             temp_scale=spec.get("temp_scale")))
        except Exception as exc:                                 # noqa: BLE001
            tile.view.setText(f"미리보기 실패: {exc}")

    def _gnss_msg(self, key, type_name):
        meter = self.hub.meters.get(self.gnss_topics.get(key))
        if meter is None or meter.latest is None or time.monotonic() - meter.latest_t > STALE_S:
            return None
        try:
            return deserialize_message(meter.latest, get_message(type_name))
        except Exception:                                        # noqa: BLE001
            return None

    def _gnss_text(self):
        """위치 해가 있으면 위치·속도·정확도, 없으면 nav_status/pos_type 만이라도."""
        nav = self._gnss_msg("nav_status", "std_msgs/msg/String")
        pos = self._gnss_msg("pos_type", "std_msgs/msg/String")
        fix = self._gnss_msg("fix", "sensor_msgs/msg/NavSatFix")
        vel = self._gnss_msg("vel", "geometry_msgs/msg/TwistWithCovarianceStamped")
        status = " · ".join(x.data for x in (nav, pos) if x is not None and x.data) or "상태 수신 대기"
        good = fix is not None and fix.status.status >= 0 and \
            not (abs(fix.latitude) < 1e-9 and abs(fix.longitude) < 1e-9)
        color = ui_theme.OK if good else ui_theme.WARN
        lines = [f"<b style='color:{color}'>{status}</b>"]
        if good:
            speed = ""
            if vel is not None:
                v = vel.twist.twist.linear
                speed = f" · {math.hypot(v.x, v.y) * 3.6:.1f} km/h"
            sigma = math.sqrt(fix.position_covariance[0]) if fix.position_covariance[0] > 0 else None
            lines.append(f"{fix.latitude:.6f}, {fix.longitude:.6f} · {fix.altitude:.1f} m{speed}"
                         + (f" · ±{sigma:.2f} m" if sigma is not None else ""))
        else:
            lines.append("아직 위치 해 없음 (fix 토픽 미발행)")
        return "<br>".join(lines), "\n".join(self.gnss_topics.values())
