#!/usr/bin/env python3
# projection_check.py — [온라인 캘리브레이션] 탭의 "실시간 projection 보기".
#
# 적용된 캘값이 실제로 쓰이고 있는지 눈으로 본다: 카메라 노드가 지금 내보내는 camera_info 와 /tf_static
# (flir_camera_extrinsics_tf_node) 으로 /ouster/points 를 카메라 영상 위에 거리 색으로 그린다.
# 파일이 아니라 노드가 낸 값을 쓰므로 "적용했는데 노드를 다시 안 띄웠다", "이름을 바꿔 frame 이 안 맞는다",
# "자리표시 값이 그대로다" 도 여기서 드러난다. 수 cm 위치 오차는 눈으로 안 보인다 (정량 검증은 도구 쪽).
#
# 구독은 preview_panel 과 같은 규칙: raw 바이트로 받아 두고 그릴 때만 역직렬화, 생성/해제는 ROS 스레드.

import math
import queue
import time
from collections import deque

import numpy as np

PLACEHOLDER_FX = 1045.0          # FLIR_control 자리표시 camera_info (1920x1200 이상 핀홀, 왜곡 0)
MAX_DT_S = 0.06                  # 라이다 스캔과 영상 시각 차 — 이보다 크면 경고 (정차 중이면 상관없음)
CLOUD_FRAME_DEFAULT = "os_lidar"


# ---------------------------------------------------------------- 계산 (ROS · Qt 없음)
def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def cloud_xyz(msg, max_range=80.0, min_range=0.5):
    """sensor_msgs/PointCloud2 → (N,3) float32. x,y,z 는 float32 필드여야 한다. 원점 · NaN · 범위 밖은 뺀다."""
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in "xyz"):
        raise ValueError(f"x/y/z 필드 없음: {sorted(off)}")
    dt = np.dtype({"names": ["x", "y", "z"], "formats": [("<f4" if not msg.is_bigendian else ">f4")] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    n = msg.width * msg.height
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    xyz = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32)
    r = np.linalg.norm(xyz, axis=1)
    keep = np.isfinite(r) & (r > min_range) & (r < max_range)
    return xyz[keep]


def cloud_edges_xyz(msg, max_range=60.0, min_range=0.5, jump_m=0.3, jump_rel=0.05):
    """정돈된 점군(height × width, Ouster 128 × 1024)에서 깊이가 끊기는 곳의 앞쪽 점만 (물체 윤곽).
    정렬을 볼 때는 점 전체보다 이게 낫다 — 영상의 윤곽과 겹치는지만 보면 된다."""
    if msg.height <= 1:
        raise ValueError("정돈되지 않은 점군 (height 1) — 윤곽을 못 찾음")
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": [("<f4" if not msg.is_bigendian else ">f4")] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    xyz = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32).reshape(msg.height, msg.width, 3)
    r = np.linalg.norm(xyz, axis=2)
    valid = np.isfinite(r) & (r > min_range) & (r < max_range)
    r = np.where(valid, r, np.inf)
    edge = np.zeros_like(valid)
    with np.errstate(invalid="ignore"):          # inf - inf (둘 다 빈 칸)
        for shift in (1, -1):                    # 좌우 이웃 (같은 레이저 줄)
            nb = np.roll(r, shift, axis=1)
            edge |= valid & (nb - r > np.maximum(jump_m, jump_rel * r))   # 이웃이 뒤쪽이면 나는 앞쪽 윤곽
    return xyz[edge]


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class TfGraph:
    """정적 TF 모음. parent→child 변환 T (x_parent = T · x_child)."""

    def __init__(self):
        self.edges = {}                 # child -> (parent, 4x4)

    def add(self, parent, child, t, q):
        T = np.eye(4)
        T[:3, :3] = quat_to_R(*q)
        T[:3, 3] = t
        self.edges[child.lstrip("/")] = (parent.lstrip("/"), T)

    def add_msg(self, tf_msg):
        for tr in tf_msg.transforms:
            t, r = tr.transform.translation, tr.transform.rotation
            self.add(tr.header.frame_id, tr.child_frame_id, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def _to_root(self, frame):
        """frame 에서 뿌리까지: [(frame, T_root←frame 누적)], 사이클 방지."""
        chain, T, f, seen = [(frame, np.eye(4))], np.eye(4), frame, {frame}
        while f in self.edges:
            p, E = self.edges[f]
            if p in seen:
                break
            T = E @ T
            chain.append((p, T.copy()))
            seen.add(p)
            f = p
        return chain

    def lookup(self, target, source):
        """x_target = T · x_source 인 T 와 거쳐 간 frame 목록. 이어지지 않으면 (None, 이유)."""
        target, source = target.lstrip("/"), source.lstrip("/")
        src, tgt = self._to_root(source), self._to_root(target)     # [(f, T_f←source)], [(f, T_f←target)]
        src_frames = [f for f, _ in src]
        for j, (f, T_f_target) in enumerate(tgt):
            if f in src_frames:                                     # 가장 가까운 공통 조상
                i = src_frames.index(f)
                path = src_frames[:i + 1] + [g for g, _ in tgt[:j]][::-1]
                return np.linalg.inv(T_f_target) @ src[i][1], path
        parents = {p for p, _ in self.edges.values()}
        if target not in self.edges and target not in parents:
            return None, (f"{target} 을(를) 내는 TF 가 없음 — 이 카메라 캘값이 적용 안 됐거나, 적용 뒤 FLIR 센서군을 "
                          "다시 안 띄웠거나, frame 이름이 바뀜")
        return None, (f"{target} 이(가) '{tgt[-1][0]}' 아래에 있어 라이다('{src[-1][0]}')와 이어지지 않음 "
                      "(자리표시 TF 일 수 있음)")


def project(Xc, K, D, model, width, height):
    """카메라 좌표 점 (N,3) → (uv (M,2), 깊이 (M,), 원래 인덱스). model: equidistant | plumb_bob."""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    z = Xc[:, 2]
    front = z > 0.2
    X = Xc[front].astype(np.float64)
    idx = np.nonzero(front)[0]
    x, y = X[:, 0] / X[:, 2], X[:, 1] / X[:, 2]
    D = list(D) + [0.0] * 8
    if model == "equidistant":
        r = np.hypot(x, y)
        th = np.arctan(r)
        th2 = th * th
        thd = th * (1 + D[0] * th2 + D[1] * th2 ** 2 + D[2] * th2 ** 3 + D[3] * th2 ** 4)
        s = np.where(r > 1e-9, thd / np.maximum(r, 1e-9), 1.0)
        xd, yd = x * s, y * s
    elif model in ("plumb_bob", "rational_polynomial", ""):
        k1, k2, p1, p2, k3 = D[:5]
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        if model == "rational_polynomial":
            rad = rad / (1 + D[5] * r2 + D[6] * r2 * r2 + D[7] * r2 ** 3)
        xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    else:
        raise ValueError(f"지원하지 않는 왜곡 모델: {model}")
    u, v = fx * xd + cx, fy * yd + cy
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    # 광각에서 뒤로 접히는 점 (equidistant 는 θ 가 크면 단조가 깨질 수 있다) — 광축과 80° 넘으면 뺀다
    inside &= np.hypot(x, y) < math.tan(math.radians(80))
    return np.stack([u[inside], v[inside]], 1), X[inside, 2], idx[inside]


def depth_colors(depth, near=2.0, far=40.0):
    """거리 → RGB (가까우면 빨강, 멀면 파랑). 로그 눈금."""
    t = np.clip((np.log(np.maximum(depth, 1e-3)) - math.log(near)) / (math.log(far) - math.log(near)), 0, 1)
    h = t * 240.0 / 60.0                       # HSV hue 0(빨강)..240(파랑), S=V=1
    i = np.floor(h).astype(int) % 6
    f = h - np.floor(h)
    q, tt = 1 - f, f
    one, zero = np.ones_like(f), np.zeros_like(f)
    r = np.choose(i, [one, q, zero, zero, tt, one])
    g = np.choose(i, [tt, one, one, q, zero, zero])
    b = np.choose(i, [zero, zero, tt, one, one, q])
    return (np.stack([r, g, b], 1) * 255).astype(np.uint8)


def draw_points(rgb, uv, colors, size=2):
    """rgb (H,W,3) uint8 위에 점을 그린다 (제자리). 먼 점부터 그려 가까운 점이 위에 오게 하려면 미리 정렬."""
    h, w = rgb.shape[:2]
    u = np.round(uv[:, 0]).astype(int)
    v = np.round(uv[:, 1]).astype(int)
    for du in range(size):
        for dv in range(size):
            uu, vv = u + du, v + dv
            ok = (uu >= 0) & (uu < w) & (vv >= 0) & (vv < h)
            rgb[vv[ok], uu[ok]] = colors[ok]
    return rgb


def overlay(rgb, Xc, info, scale, max_points=60000, size=2):
    """rgb: 줄인 영상 (H,W,3). Xc: 카메라 좌표 점. info: dict(k, d, model, width, height). scale: 영상/camera_info 배율.
    반환 (그린 영상, 영상 안에 든 점 수)."""
    uv, depth, _ = project(Xc, info["k"], info["d"], info["model"], info["width"], info["height"])
    if len(uv) > max_points:
        sel = np.random.default_rng(0).choice(len(uv), max_points, replace=False)
        uv, depth = uv[sel], depth[sel]
    order = np.argsort(-depth)                 # 먼 것부터
    uv, depth = uv[order] * scale, depth[order]
    draw_points(rgb, uv, depth_colors(depth), size=size)
    return rgb, len(order)


def camera_info_dict(msg):
    return {"model": msg.distortion_model, "d": list(msg.d), "k": list(msg.k), "width": msg.width,
            "height": msg.height, "frame_id": msg.header.frame_id}


def judge_camera_info(info):
    """(ok, 쉬운 말). ok: True 정상 · False 문제 · None 모름."""
    if info is None:
        return None, "camera_info 안 옴 (카메라 노드가 꺼져 있음)"
    fx = info["k"][0] if info["k"] else 0.0
    if not fx:
        return False, "캘값 없음 (fx 0) — 이 카메라 시리얼이 camera_info 파일에 없음"
    if abs(fx - PLACEHOLDER_FX) < 1e-6 and not any(info["d"]):
        return False, "자리표시 값 (fx 1045 · 왜곡 0) — 캘값이 안 들어감"
    return True, f"{info['model']} · fx {fx:.1f}"


def judge_tf(T, path_or_why, cloud_frame):
    if T is None:
        return False, path_or_why
    if np.allclose(T, np.eye(4), atol=1e-9):
        return False, "항등 변환 (자리표시) — 캘값이 안 들어감"
    hops = " → ".join(path_or_why)
    direct = len(path_or_why) == 2 and path_or_why[0] == cloud_frame
    return True, (f"{hops}" if direct else f"{hops} (거쳐서)")


# ---------------------------------------------------------------- ROS 구독 (preview_panel 과 같은 규칙)
class ProjectionHub:
    """/ouster/points(최신 한 장) · 고른 카메라 영상(최근 몇 장) · 모든 camera_info(최신) · /tf_static(전부)."""

    KEEP_IMAGES = 12            # 30 Hz 에서 0.4 s — 라이다 스캔 시각에 가장 가까운 영상을 고른다

    def __init__(self, worker):
        self.worker = worker
        self.cloud = [None]                     # 최신 raw
        self.images = deque(maxlen=self.KEEP_IMAGES)
        self.infos = {}                         # ns -> raw
        self.tf_static = queue.SimpleQueue()    # raw TFMessage 들 (ROS → GUI)
        self._subs = {}                         # ROS 스레드 전용: 이름 -> Subscription
        self.image_topic = None

    def start(self, info_topics, cloud_topic="/ouster/points"):
        self.worker.call_in_ros_thread(lambda: self._start(list(info_topics), cloud_topic))

    def _start(self, info_topics, cloud_topic):        # ROS 스레드
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        node = self.worker.node
        be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if "tf" not in self._subs:
            self._subs["tf"] = node.create_subscription(get_message("tf2_msgs/msg/TFMessage"), "/tf_static",
                                                        self.tf_static.put, latched, raw=True)
        if "cloud" not in self._subs:
            self._subs["cloud"] = node.create_subscription(
                get_message("sensor_msgs/msg/PointCloud2"), cloud_topic,
                lambda d: self.cloud.__setitem__(0, d), be, raw=True)
        for t in info_topics:
            key = "info:" + t
            if key not in self._subs:
                ns = t.strip("/").split("/")[0]
                self._subs[key] = node.create_subscription(
                    get_message("sensor_msgs/msg/CameraInfo"), t,
                    lambda d, ns=ns: self.infos.__setitem__(ns, d), be, raw=True)

    def set_image(self, topic, type_name):
        self.image_topic = topic
        self.images.clear()
        self.worker.call_in_ros_thread(lambda: self._set_image(topic, type_name))

    def _set_image(self, topic, type_name):            # ROS 스레드
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        old = self._subs.pop("image", None)
        if old is not None:
            self.worker.node.destroy_subscription(old)
        if topic != self.image_topic:          # 그사이 또 바뀜
            return
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._subs["image"] = self.worker.node.create_subscription(
            get_message(type_name), topic, self.images.append, qos, raw=True)

    def stop(self):
        self.worker.call_in_ros_thread(self._stop)

    def _stop(self):                                   # ROS 스레드
        for s in self._subs.values():
            self.worker.node.destroy_subscription(s)
        self._subs.clear()


# ---------------------------------------------------------------- 창 (GUI 스레드)
from PyQt5.QtCore import Qt, QTimer                                                  # noqa: E402
from PyQt5.QtGui import QColor, QImage, QPixmap                                      # noqa: E402
from PyQt5.QtWidgets import (QAbstractItemView, QComboBox, QDialog, QFrame, QHBoxLayout,  # noqa: E402
                             QHeaderView, QLabel, QPushButton, QSizePolicy, QTableWidget,
                             QTableWidgetItem, QVBoxLayout)

OK_C, WARN_C, ERR_C, MUTED_C = "#16a34a", "#d97706", "#dc2626", "#6b7280"
RENDER_MS = 500                 # 2 Hz — 점군 역직렬화 · 투영이 GUI 스레드에서 50–100 ms
DISPLAY_W = 1280                # RGB 는 이 폭으로 줄여 디코드


def _thermal_rgb(msg):
    a = np.frombuffer(bytes(msg.data), ">u2" if msg.is_bigendian else "<u2")
    a = a.reshape(msg.height, msg.step // 2)[:, :msg.width].astype(np.float32)
    lo, hi = np.percentile(a, (1, 99))
    g = np.clip((a - lo) * (255.0 / max(hi - lo, 1.0)), 0, 255).astype(np.uint8)
    return np.repeat(g[..., None], 3, axis=2)


def _item(text, color=None, tip=""):
    it = QTableWidgetItem(text)
    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
    if color:
        it.setForeground(QColor(color))
    if tip:
        it.setToolTip(tip)
    return it


class ProjectionDialog(QDialog):
    """카메라별 판정 표 (camera_info · TF) + 고른 카메라 영상에 라이다 윤곽/점."""

    NEED_GROUPS = (("flir_cameras", "카메라"), ("ouster", "라이다"))

    def __init__(self, worker, parent=None, labels=None, stage=None, busy=None):
        super().__init__(parent)
        self.setWindowTitle("실시간 projection 보기 — 지금 적용된 캘값 (camera_info + /tf_static)")
        self.worker = worker
        self.labels = labels or {}                     # ns -> "가시광"/"열화상" (센서 기동 탭)
        self.stage = stage                             # SensorStage — 꺼져 있으면 여기서 켠다
        self.busy = busy or (lambda: None)             # () -> 켜면 안 되는 이유 (녹화 · 캘리브레이션 중 등) 또는 None
        self._have_cloud = False
        self.hub = ProjectionHub(worker)
        self.graph = TfGraph()
        self.cams = {}                                 # ns -> {"image", "type", "info", "thermal"}
        self._last_cloud = None
        self._paused = False

        v = QVBoxLayout(self)
        # 센서가 꺼져 있으면: 무엇이 꺼졌는지 + 그 자리에서 켜기
        self.guide = QFrame()
        self.guide.setObjectName("projGuide")
        self.guide.setStyleSheet("QFrame#projGuide{background:#fef3c7; border:1px solid #fcd34d; border-radius:8px;}")
        gl = QHBoxLayout(self.guide)
        gl.setContentsMargins(12, 10, 12, 10)
        self.lbl_guide = QLabel()
        self.lbl_guide.setWordWrap(True)
        self.lbl_guide.setTextFormat(Qt.RichText)
        self.lbl_guide.setStyleSheet("font-size:15px;")
        gl.addWidget(self.lbl_guide, 1)
        self.btn_on = QPushButton("카메라 · 라이다 켜기")
        self.btn_on.setMinimumHeight(40)
        self.btn_on.setStyleSheet("QPushButton{background:#2563eb; color:white; font-size:15px; font-weight:700; "
                                  "border-radius:6px; padding:6px 16px;} QPushButton:disabled{background:#93c5fd;}")
        self.btn_on.clicked.connect(self._turn_on)
        gl.addWidget(self.btn_on)
        v.addWidget(self.guide)
        self.lbl_verdict = QLabel("센서를 찾는 중…")
        self.lbl_verdict.setWordWrap(True)
        self.lbl_verdict.setStyleSheet("font-weight:600; font-size:14px;")
        v.addWidget(self.lbl_verdict)
        self.tbl = QTableWidget(0, 3)
        self.tbl.setHorizontalHeaderLabels(["카메라", "camera_info (렌즈)", "TF (라이다 → 카메라)"])
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.tbl.horizontalHeader().setStretchLastSection(True)
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl.setMaximumHeight(260)
        self.tbl.itemSelectionChanged.connect(self._row_picked)
        v.addWidget(self.tbl)

        row = QHBoxLayout()
        self.cb_cam = QComboBox()
        self.cb_cam.currentIndexChanged.connect(self._cam_changed)
        self.cb_mode = QComboBox()
        self.cb_mode.addItem("윤곽만 (정렬 보기)", "edges")
        self.cb_mode.addItem("점 전체", "all")
        self.cb_mode.setToolTip("윤곽: 라이다 깊이가 끊기는 곳(기둥 · 나무 · 차 옆면) — 영상의 윤곽과 겹치면 정렬이 맞다")
        self.btn_pause = QPushButton("멈춤")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(lambda on: (setattr(self, "_paused", on),
                                                   self.btn_pause.setText("계속" if on else "멈춤")))
        self._controls = (QLabel("카메라"), self.cb_cam, QLabel("표시"), self.cb_mode, self.btn_pause)
        for w in self._controls:
            row.addWidget(w)
        row.addStretch(1)
        v.addLayout(row)
        self.lbl_status = QLabel("")
        self.lbl_status.setStyleSheet(f"color:{MUTED_C};")
        self.lbl_status.setWordWrap(True)
        v.addWidget(self.lbl_status)
        self.view = QLabel("영상 대기 중")
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setMinimumSize(640, 400)
        self.view.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)      # 그림 크기가 칸을 키우지 않게
        self.view.setStyleSheet("background:#111; color:#aaa;")
        v.addWidget(self.view, 1)
        hint = QLabel("차가 <b>멈춰 있을 때</b> 보세요 — 달리는 중에는 라이다 스캔(0.1 s)과 영상 시각 차만큼 어긋나 보입니다. "
                      "수 cm 위치 오차는 눈으로 보이지 않습니다. 가까우면 빨강, 멀면 파랑.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color:{MUTED_C};")
        v.addWidget(hint)
        self.resize(1300, 1000)

        self._scan_graph()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(RENDER_MS)
        self.graph_timer = QTimer(self)
        self.graph_timer.timeout.connect(self._scan_graph)
        self.graph_timer.start(3000)

    # --- 어떤 카메라가 있나 ---
    def _scan_graph(self):
        node = self.worker.node
        topics = {}
        for name, types in node.get_topic_names_and_types():
            if types and node.count_publishers(name) > 0:
                topics[name] = types[0]
        cams = {}
        for ns in sorted({n.strip("/").split("/")[0] for n in topics if n.count("/") >= 2}):
            jpeg, raw, info = f"/{ns}/image_rgb/compressed", f"/{ns}/image_raw", f"/{ns}/camera_info"
            if info not in topics:
                continue
            if topics.get(jpeg) == "sensor_msgs/msg/CompressedImage":
                cams[ns] = {"image": jpeg, "type": topics[jpeg], "info": info, "thermal": False}
            elif topics.get(raw) == "sensor_msgs/msg/Image":
                cams[ns] = {"image": raw, "type": topics[raw], "info": info,
                            "thermal": "열화상" in self.labels.get(ns, "") or ns.startswith("thermal")}
        if set(cams) != set(self.cams):
            self.cams = cams
            self.hub.start([c["info"] for c in cams.values()])
            cur = self.cb_cam.currentData()
            self.cb_cam.blockSignals(True)
            self.cb_cam.clear()
            for ns in sorted(cams, key=lambda n: (cams[n]["thermal"], n)):
                self.cb_cam.addItem(("열화상 · " if cams[ns]["thermal"] else "") + ns, ns)
            i = self.cb_cam.findData(cur)
            self.cb_cam.setCurrentIndex(max(i, 0))
            self.cb_cam.blockSignals(False)
            self._cam_changed()
        self._have_cloud = "/ouster/points" in topics
        self._update_guide(bool(cams))
        for w in (self.tbl, self.lbl_verdict) + self._controls:     # 카메라가 없으면 빈 표 · 빈 선택칸 대신 안내만
            w.setVisible(bool(cams))
        if not cams:
            self.lbl_verdict.setText("카메라 영상이 안 옵니다 — 위의 [카메라 · 라이다 켜기]")
            self.lbl_verdict.setStyleSheet(f"font-weight:600; font-size:14px; color:{ERR_C};")
            self.view.setText("센서가 켜지면 여기에 카메라 영상 위 라이다가 보입니다")
        elif not self._have_cloud:
            self.lbl_status.setText("라이다 점군(/ouster/points)이 안 옵니다 — 위의 [카메라 · 라이다 켜기]")

    def _update_guide(self, have_cams):
        """카메라 · 라이다 중 안 켜진 것을 알리고 [켜기]. 둘 다 오면 숨긴다."""
        parts, off, starting = [], [], False
        for gk, name in self.NEED_GROUPS:
            if self.stage is not None and gk in getattr(self.stage, "groups", {}):
                txt, on = self.stage.group_state(gk)
            else:
                on = have_cams if gk == "flir_cameras" else self._have_cloud
                txt = "켜짐" if on else "안 보임"
            # 프로세스는 떠 있어도 토픽이 아직 안 오면 켜는 중으로
            if on and ((gk == "flir_cameras" and not have_cams) or (gk == "ouster" and not self._have_cloud)):
                txt, on = "켜는 중… (토픽 기다림)", False
                starting = True
            starting |= txt.startswith("켜는 중")
            color = OK_C if on else (WARN_C if txt.startswith("켜는 중") else ERR_C)
            parts.append(f"{name}: <b style='color:{color}'>{txt}</b>")
            if not on and not txt.startswith("켜는 중"):
                off.append(gk)
        if have_cams and self._have_cloud:
            self.guide.hide()
            return
        self.guide.show()
        why = self.busy()
        head = "센서가 켜져 있어야 볼 수 있습니다" if off else "센서를 켜는 중입니다 — 30초–1분 걸립니다"
        self.lbl_guide.setText(f"<b>{head}</b><br>" + " · ".join(parts)
                               + (f"<br><span style='color:{ERR_C}'>{why}</span>" if (why and off) else ""))
        self._off = off
        self.btn_on.setVisible(bool(off) and self.stage is not None)
        self.btn_on.setEnabled(not why)
        if self.stage is None and off:
            self.lbl_guide.setText(self.lbl_guide.text() + "<br>[센서 기동] 탭에서 FLIR 카메라와 라이다를 켜세요")

    def _turn_on(self):
        why = self.busy()
        if why or self.stage is None:
            return
        started = self.stage.start_groups([g for g in getattr(self, "_off", []) if g])
        self.btn_on.setEnabled(False)
        self.lbl_guide.setText(f"<b>켜는 중… ({', '.join(started) or '이미 켜는 중'})</b> — 30초–1분 뒤 자동으로 보입니다")

    def _cam_changed(self):
        ns = self.cb_cam.currentData()
        if ns and ns in self.cams:
            self.hub.set_image(self.cams[ns]["image"], self.cams[ns]["type"])
            self.view.setText("영상 대기 중")

    def _row_picked(self):
        rows = self.tbl.selectionModel().selectedRows()
        if rows:
            ns = self.tbl.item(rows[0].row(), 0).data(Qt.UserRole)
            i = self.cb_cam.findData(ns)
            if i >= 0 and i != self.cb_cam.currentIndex():
                self.cb_cam.setCurrentIndex(i)

    # --- 판정 ---
    def _infos(self):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        cls = get_message("sensor_msgs/msg/CameraInfo")
        out = {}
        for ns, raw in list(self.hub.infos.items()):
            try:
                out[ns] = camera_info_dict(deserialize_message(raw, cls))
            except Exception:                                   # noqa: BLE001
                out[ns] = None
        return out

    def _judge_all(self, cloud_frame):
        infos = self._infos()
        rows = []
        for ns in sorted(self.cams, key=lambda n: (self.cams[n]["thermal"], n)):
            info = infos.get(ns)
            ci_ok, ci_txt = judge_camera_info(info)
            frame = (info or {}).get("frame_id") or f"{ns}_optical_frame"
            T, why = self.graph.lookup(frame, cloud_frame)
            tf_ok, tf_txt = judge_tf(T, why, cloud_frame)
            rows.append((ns, ci_ok, ci_txt, tf_ok, tf_txt))
        return rows, infos

    def _show_table(self, rows):
        sel = self.cb_cam.currentData()
        self.tbl.blockSignals(True)
        self.tbl.setRowCount(len(rows))
        for i, (ns, ci_ok, ci_txt, tf_ok, tf_txt) in enumerate(rows):
            name = _item(("열화상 · " if self.cams[ns]["thermal"] else "") + ns)
            name.setData(Qt.UserRole, ns)
            self.tbl.setItem(i, 0, name)
            for col, ok, txt in ((1, ci_ok, ci_txt), (2, tf_ok, tf_txt)):
                mark = "✓ " if ok else ("✗ " if ok is False else "… ")
                self.tbl.setItem(i, col, _item(mark + txt, OK_C if ok else (ERR_C if ok is False else MUTED_C)))
            if ns == sel:
                self.tbl.selectRow(i)
        self.tbl.blockSignals(False)
        good = sum(1 for r in rows if r[1] and r[3])
        bad = [r[0] for r in rows if r[1] is False or r[3] is False]
        if not rows:
            return
        if not bad and good == len(rows):
            txt, color = f"✓ 카메라 {len(rows)}대 모두 캘값이 들어가 있습니다 — 아래 영상에서 윤곽이 맞는지 보세요", OK_C
        elif bad:
            txt, color = (f"✗ {len(bad)}대 캘값이 안 쓰이고 있습니다: {', '.join(bad)} — 표의 이유를 보세요 "
                          "(적용 뒤 FLIR 카메라 센서군을 다시 기동했는지)"), ERR_C
        else:
            txt, color = f"… {len(rows) - good}대는 아직 camera_info 를 기다리는 중", MUTED_C
        self.lbl_verdict.setText(txt)
        self.lbl_verdict.setStyleSheet(f"font-weight:600; font-size:14px; color:{color};")

    # --- 주기 ---
    def _tick(self):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        while True:                                            # /tf_static (latched) 모으기
            try:
                raw = self.hub.tf_static.get_nowait()
            except queue.Empty:
                break
            try:
                self.graph.add_msg(deserialize_message(raw, get_message("tf2_msgs/msg/TFMessage")))
            except Exception:                                   # noqa: BLE001
                pass
        raw_cloud = self.hub.cloud[0]
        cloud = None
        if raw_cloud is not None and (raw_cloud is not self._last_cloud and not self._paused):
            cloud = deserialize_message(raw_cloud, get_message("sensor_msgs/msg/PointCloud2"))
            self._last_cloud = raw_cloud
            self._cloud_frame = cloud.header.frame_id or CLOUD_FRAME_DEFAULT
        rows, infos = self._judge_all(getattr(self, "_cloud_frame", CLOUD_FRAME_DEFAULT))
        self._show_table(rows)
        if cloud is not None:
            try:
                self._render(cloud, infos)
            except Exception as e:                              # noqa: BLE001
                self.lbl_status.setText(f"그리기 실패: {e}")
                self.lbl_status.setStyleSheet(f"color:{ERR_C};")

    def _render(self, cloud, infos):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        import preview_panel
        ns = self.cb_cam.currentData()
        if not ns or ns not in self.cams:
            return
        cam = self.cams[ns]
        info = infos.get(ns)
        ci_ok, ci_txt = judge_camera_info(info)
        if info is None or not info["k"] or not info["k"][0]:
            self.lbl_status.setText(f"{ns}: {ci_txt} — 투영할 수 없음")
            self.lbl_status.setStyleSheet(f"color:{ERR_C};")
            return
        T, why = self.graph.lookup(info["frame_id"] or f"{ns}_optical_frame", self._cloud_frame)
        if T is None:
            self.lbl_status.setText(f"{ns}: TF 없음 — {why}")
            self.lbl_status.setStyleSheet(f"color:{ERR_C};")
            return
        imgs = list(self.hub.images)
        if not imgs:
            self.lbl_status.setText(f"{ns}: 영상이 안 옵니다")
            return
        cls = get_message(cam["type"])
        tc = stamp_s(cloud.header)
        best = min(((abs(stamp_s(m.header) - tc), m, raw) for raw in imgs
                    for m in [deserialize_message(raw, cls)]), key=lambda x: x[0])
        dt, msg, raw = best
        W = info["width"] or (msg.width if hasattr(msg, "width") else 0)
        H = info["height"] or (msg.height if hasattr(msg, "height") else 0)
        if cam["thermal"]:
            rgb = _thermal_rgb(msg)
            W, H = W or msg.width, H or msg.height
            scale = rgb.shape[1] / W
        else:
            rgb = preview_panel.jpeg_to_array(raw, DISPLAY_W)
            W, H = W or 1920, H or 1200
            scale = rgb.shape[1] / W
        xyz = cloud_edges_xyz(cloud) if self.cb_mode.currentData() == "edges" else cloud_xyz(cloud)
        Xc = (T[:3, :3] @ xyz.T).T + T[:3, 3]
        info2 = dict(info, width=W, height=H)
        rgb, n = overlay(np.ascontiguousarray(rgb), Xc, info2, scale,
                         size=2 if (cam["thermal"] or self.cb_mode.currentData() == "all") else 3)
        h, w, _ = rgb.shape
        img = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        self.view.setPixmap(QPixmap.fromImage(img).scaled(self.view.contentsRect().size(), Qt.KeepAspectRatio,
                                                          Qt.SmoothTransformation))
        warn = dt > MAX_DT_S
        self.lbl_status.setText(f"{ns}: 영상 안 라이다 점 {n}개 · 라이다–영상 시각 차 {dt * 1000:.0f} ms"
                                + (" — 크다: 멈춰 있을 때만 믿으세요" if warn else "")
                                + f" · {ci_txt}")
        self.lbl_status.setStyleSheet(f"color:{WARN_C if warn else MUTED_C};")

    def done(self, r):
        """X · Esc · close() 모두 여기로 온다 (QDialog.closeEvent → reject → done): 구독을 내리고 끝낸다."""
        self.timer.stop()
        self.graph_timer.stop()
        self.hub.stop()
        super().done(r)
