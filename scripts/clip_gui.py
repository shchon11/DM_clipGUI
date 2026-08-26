#!/usr/bin/env python3
# clip_gui.py — clip_recorder 데이터 로깅 GUI (PyQt5).
#
# 구조: 이 GUI는 레코더가 아니다. 링 버퍼/트리거/저장은 전부 C++ clip_recorder
# 노드가 하고, GUI는 그 노드를 조종한다:
#   - 토픽 선택/QoS  → topics / topic_qos 파라미터 (실행 중에도 반영)
#   - 트리거         → /clip_recorder/trigger (std_msgs/Header, 버튼·단축키)
#   - 버퍼 상태      → /diagnostics 구독
#   - 레코더 로그    → /rosout 구독 (하단 로그창)
#   - 저장 진행/완료 → /clip_recorder/clip_event 구독 → 완료 시 bag_diagnostics 자동 실행
# 레코더 프로세스는 GUI가 직접 띄우거나(기본), 이미 떠 있는 외부 노드에 붙는다.
#
# 설정은 ~/.config/dm_clip_gui/last_session.yaml 에 자동 저장/복원되고,
# 파일 메뉴로 다른 프로파일을 저장/불러올 수 있다.

import json
import math
import os
import shutil
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

import yaml

from PyQt5.QtCore import QObject, Qt, QThread, QTimer, QProcess, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QKeySequence, QPixmap, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox,
    QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QKeySequenceEdit, QLabel, QLineEdit,
    QListWidget, QListWidgetItem,
    QMainWindow, QMessageBox, QProgressBar, QPushButton, QScrollArea,
    QShortcut, QSpinBox, QSplitter, QTableWidget, QTableWidgetItem,
    QTextEdit, QVBoxLayout, QWidget)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String

import bag_diagnostics
import gnss_tools
import net_tools
from map_widget import MapWidget

CONFIG_DIR = Path.home() / ".config" / "dm_clip_gui"
LAST_SESSION = CONFIG_DIR / "last_session.yaml"
RECORDER_PARAMS = CONFIG_DIR / "recorder_params.yaml"

# GUI 목록에서 숨기는 토픽 (레코더 내부/ROS 인프라)
HIDDEN_TOPICS = ("/rosout", "/parameter_events",
                 "/clip_recorder/trigger", "/clip_recorder/clip_event")

DEFAULT_CFG = {
    "topics": [],           # [{name, type, reliability, durability}]
    "recorder": {
        "pre_sec": 20.0, "post_sec": 10.0, "trigger_slack_sec": 2.0,
        "max_buffer_mb": 16384.0, "queue_depth": 50,
        "output_dir": str(Path.home() / "DM_clipGUI" / "clips"),
        "storage_id": "sqlite3", "status_period_sec": 2.0,
    },
    "ui": {
        "shortcut": "F9", "auto_diagnose": True, "auto_start_recorder": True,
        "default_reliability": "best_effort", "default_durability": "volatile",
        # 네트워크: 스위치 업링크 NIC ("auto" = 가장 빠른 UP 인터페이스),
        # 장치(카메라/라이다) 쪽 포트 링크 속도, IP→이름 수동 별칭 (라이다 등)
        "nic": "auto", "per_ip_link_mbps": 1000, "ip_aliases": {},
        # GNSS 실시간 표시용 토픽
        "gnss_fix_topic": "/gps/fix",
        "gnss_status_topics": ["/gps/pos_type", "/gps/nav_status"],
        "gnss_gga_topic": "/gps/gga",
        "gnss_vel_topic": "/gps/vel",
        "map_clips": [],          # 누적 궤적 지도에 체크된 클립 폴더들
    },
}

GGA_QUALITY = {0: "INVALID", 1: "GPS (SPS)", 2: "DGPS", 3: "PPS", 4: "RTK FIXED",
               5: "RTK FLOAT", 6: "DR (추측항법)", 7: "MANUAL", 8: "SIMULATION"}
ROSOUT_LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
LOG_COLORS = {"INFO": None, "DEBUG": "#888888", "WARN": "#b58900",
              "ERROR": "#dc322f", "FATAL": "#dc322f", "GUI": "#268bd2",
              "OK": "#859900"}


def fmt_bw(bytes_per_s):
    """bytes/s → 사람이 읽기 좋은 단위 (B/s, KB/s, MB/s, GB/s)"""
    v = float(bytes_per_s)
    for unit, div in (("GB/s", 1 << 30), ("MB/s", 1 << 20), ("KB/s", 1 << 10)):
        if v >= div:
            return f"{v / div:.2f} {unit}" if v / div < 100 else f"{v / div:.0f} {unit}"
    return f"{v:.0f} B/s"


def load_config(path=LAST_SESSION):
    cfg = json.loads(json.dumps(DEFAULT_CFG))  # deep copy
    try:
        saved = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        for key in ("recorder", "ui"):
            cfg[key].update(saved.get(key) or {})
        cfg["topics"] = saved.get("topics") or []
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"설정 읽기 실패({path}): {e} — 기본값 사용", file=sys.stderr)
    return cfg


def save_config(cfg, path=LAST_SESSION):
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(
        yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False),
        encoding="utf-8")


# ---------------- ROS 브리지 ----------------

class RosWorker(QThread):
    """별도 스레드에서 rclpy 스핀. GUI와는 시그널로만 통신."""
    sig_diag = pyqtSignal(dict)          # ring buffer 상태 key/value
    sig_rosout = pyqtSignal(str, str)    # (level, message) — clip_recorder만
    sig_event = pyqtSignal(str)          # clip_event 원문
    sig_params = pyqtSignal(bool, str)   # 파라미터 적용 결과
    sig_gnss = pyqtSignal(dict)          # 실시간 GNSS 상태 조각
    sig_serials = pyqtSignal(dict)       # {camera_serial: 노드 네임스페이스}

    def __init__(self, cfg):
        super().__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node("clip_gui")
        ui = cfg["ui"]
        # GNSS: best_effort 구독은 reliable/best_effort 퍼블리셔 모두와 호환
        self.node.create_subscription(
            NavSatFix, ui["gnss_fix_topic"], self._on_fix, qos_profile_sensor_data)
        for t in ui.get("gnss_status_topics", []):
            self.node.create_subscription(
                String, t,
                lambda m, key=t.rsplit("/", 1)[-1]:
                    self.sig_gnss.emit({"kind": key, "text": m.data}),
                qos_profile_sensor_data)
        if ui.get("gnss_gga_topic"):
            self.node.create_subscription(
                String, ui["gnss_gga_topic"], self._on_gga, qos_profile_sensor_data)
        if ui.get("gnss_vel_topic"):
            self.node.create_subscription(
                TwistWithCovarianceStamped, ui["gnss_vel_topic"], self._on_vel,
                qos_profile_sensor_data)
        self._param_clients = {}
        self._last_emit = {}        # kind -> 마지막 emit 시각 (고주파 토픽 스로틀)
        self.node.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.node.create_subscription(Log, "/rosout", self._on_rosout, 50)
        self.node.create_subscription(
            String, "/clip_recorder/clip_event", self._on_event, 10)
        self._trigger_pub = self.node.create_publisher(
            Header, "/clip_recorder/trigger", 10)
        self._param_cli = self.node.create_client(
            SetParameters, "/clip_recorder/set_parameters")
        self._stop = False

    def run(self):
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)
        while not self._stop and rclpy.ok():
            ex.spin_once(timeout_sec=0.2)

    def stop(self):
        self._stop = True
        self.wait(3000)
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    # --- 콜백 (executor 스레드) → 시그널 ---
    def _on_diag(self, msg):
        for st in msg.status:
            if st.name.endswith(": ring buffer"):
                kv = {v.key: v.value for v in st.values}
                kv["_message"] = st.message
                self.sig_diag.emit(kv)

    def _on_rosout(self, m):
        if m.name == "clip_recorder":
            self.sig_rosout.emit(ROSOUT_LEVELS.get(m.level, "INFO"), m.msg)

    def _on_event(self, m):
        self.sig_event.emit(m.data)

    def _throttled(self, kind, period=0.1):
        """RT2000처럼 100 Hz+로 오는 토픽을 GUI로는 10 Hz만 넘긴다.
        (지도/라벨 갱신은 어차피 타이머 주기라 더 자주 보내봐야 큐만 쌓인다)"""
        now = time.monotonic()
        if now - self._last_emit.get(kind, 0.0) < period:
            return True
        self._last_emit[kind] = now
        return False

    def _on_fix(self, m):
        if self._throttled("fix"):
            return
        hacc = None
        if m.position_covariance_type > 0:
            c = m.position_covariance
            if c[0] >= 0 and c[4] >= 0:
                hacc = math.sqrt(max(c[0], c[4]))
        self.sig_gnss.emit({"kind": "fix", "lat": m.latitude, "lon": m.longitude,
                            "alt": m.altitude, "status": int(m.status.status),
                            "hacc": hacc, "t": time.time()})

    def _on_gga(self, m):
        if self._throttled("gga", 0.5):
            return
        # $GxGGA,time,lat,N,lon,E,quality,sats,hdop,alt,...
        f = m.data.strip().split(",")
        if len(f) < 9 or not f[0].endswith("GGA"):
            return
        try:
            self.sig_gnss.emit({"kind": "gga", "quality": int(f[6] or 0),
                                "sats": int(f[7] or 0), "hdop": float(f[8] or 0)})
        except ValueError:
            pass

    def _on_vel(self, m):
        if self._throttled("vel", 0.5):
            return
        v = m.twist.twist.linear
        self.sig_gnss.emit({"kind": "vel", "speed_mps": math.hypot(v.x, v.y),
                            "t": time.time()})

    def resolve_camera_serials(self):
        """flir_camera 노드들의 camera_serial 파라미터를 모아 sig_serials로."""
        nodes = [(n, ns) for n, ns in self.node.get_node_names_and_namespaces()
                 if n == "flir_camera"]
        result, pending = {}, [len(nodes)]

        def finish():
            self.sig_serials.emit(dict(result))
        if not nodes:
            finish()
            return
        for n, ns in nodes:
            svc = f"{ns.rstrip('/')}/{n}/get_parameters"
            cli = self._param_clients.get(svc)
            if cli is None:
                cli = self.node.create_client(GetParameters, svc)
                self._param_clients[svc] = cli
            if not cli.service_is_ready():
                pending[0] -= 1
                continue

            def done(fut, ns=ns):
                try:
                    vals = fut.result().values
                    if vals and vals[0].string_value:
                        result[vals[0].string_value] = ns.strip("/") or "camera"
                except Exception:
                    pass
                pending[0] -= 1
                if pending[0] <= 0:
                    finish()
            cli.call_async(GetParameters.Request(names=["camera_serial"])) \
                .add_done_callback(done)
        if pending[0] <= 0:
            finish()

    # --- GUI 스레드에서 호출하는 동작 ---
    def trigger(self, label=""):
        h = Header()
        h.stamp = self.node.get_clock().now().to_msg()
        h.frame_id = label
        self._trigger_pub.publish(h)

    def recorder_alive(self):
        try:
            return "clip_recorder" in \
                [n for n, _ in self.node.get_node_names_and_namespaces()]
        except Exception:
            return False

    def list_topics(self):
        out = []
        for name, types in sorted(self.node.get_topic_names_and_types()):
            if not types or name in HIDDEN_TOPICS:
                continue
            transient = False
            try:
                for info in self.node.get_publishers_info_by_topic(name):
                    if info.qos_profile.durability == \
                            DurabilityPolicy.TRANSIENT_LOCAL:
                        transient = True
            except Exception:
                pass
            out.append({"name": name, "type": types[0],
                        "transient_pub": transient})
        return out

    def set_recorder_params(self, topics, qos_entries):
        """topics/topic_qos 파라미터를 비동기로 적용. 결과는 sig_params."""
        if not self._param_cli.service_is_ready():
            self.sig_params.emit(False, "레코더 파라미터 서비스에 연결 안 됨")
            return

        def str_arr(name, values):
            return Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                string_array_value=values))

        req = SetParameters.Request()
        req.parameters = [str_arr("topics", topics),
                          str_arr("topic_qos", qos_entries)]

        def done(fut):
            try:
                results = fut.result().results
                bad = [r.reason for r in results if not r.successful]
                if bad:
                    self.sig_params.emit(False, "; ".join(bad))
                else:
                    self.sig_params.emit(
                        True, f"토픽 {len(topics)}개 적용 (2초 내 구독 재구성)")
            except Exception as e:
                self.sig_params.emit(False, str(e))

        self._param_cli.call_async(req).add_done_callback(done)


# ---------------- 레코더 프로세스 ----------------

class RecorderManager(QProcess):
    """GUI가 소유하는 clip_recorder 프로세스 (외부 노드에 붙으면 미사용)."""

    def start_recorder(self, cfg):
        rec = dict(cfg["recorder"])
        params = {k: rec[k] for k in
                  ("pre_sec", "post_sec", "trigger_slack_sec", "max_buffer_mb",
                   "queue_depth", "output_dir", "storage_id",
                   "status_period_sec")}
        topics = [t["name"] for t in cfg["topics"]]
        # 주의: ROS 2 파라미터 YAML은 빈 리스트의 타입을 추론하지 못해 노드가
        # 죽는다("No parameter value set"). 비어 있으면 키 자체를 생략한다.
        if topics:
            params["topics"] = topics
            params["topic_qos"] = [
                f"{t['name']} {t['reliability']} {t['durability']}"
                for t in cfg["topics"]]
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        RECORDER_PARAMS.write_text(
            yaml.safe_dump({"clip_recorder": {"ros__parameters": params}}),
            encoding="utf-8")
        self.setProcessChannelMode(QProcess.MergedChannels)
        self.start("ros2", ["run", "clip_recorder", "clip_recorder",
                            "--ros-args", "-r", "__node:=clip_recorder",
                            "--params-file", str(RECORDER_PARAMS)])

    def stop_recorder(self):
        if self.state() != QProcess.NotRunning:
            self.terminate()
            if not self.waitForFinished(5000):
                self.kill()
                self.waitForFinished(2000)


# ---------------- 진단 실행 스레드 ----------------

class DiagRunner(QThread):
    sig_progress = pyqtSignal(str)
    sig_done = pyqtSignal(dict)
    sig_error = pyqtSignal(str)

    def __init__(self, bag_dir):
        super().__init__()
        self.bag_dir = str(bag_dir)

    def run(self):
        try:
            rep = bag_diagnostics.analyze(
                self.bag_dir, progress=self.sig_progress.emit)
            # 보고서를 클립 옆에 저장
            d = Path(self.bag_dir)
            (d / "diagnostics.json").write_text(
                json.dumps(rep, ensure_ascii=False, indent=1),
                encoding="utf-8")
            (d / "diagnostics.txt").write_text(
                bag_diagnostics.render_text(rep) + "\n", encoding="utf-8")
            self.sig_done.emit(rep)
        except Exception as e:
            self.sig_error.emit(f"{type(e).__name__}: {e}")


# ---------------- 네트워크 모니터 ----------------

class DiscoveryThread(QThread):
    sig_found = pyqtSignal(dict)

    def __init__(self, iface):
        super().__init__()
        self.iface = iface

    def run(self):
        try:
            self.sig_found.emit(net_tools.gvcp_discover(self.iface, timeout=1.0))
        except Exception:
            self.sig_found.emit({})


class NetMonitor(QObject):
    """스위치 업링크 NIC 합계(sysfs) + 송신 IP별 트래픽(net_probe) + GigE 디스커버리."""
    sig_total = pyqtSignal(str, float, int)   # iface, rx Mbps, link Mbps
    sig_ips = pyqtSignal(dict)                # {ip: (Mbps, pps)}
    sig_state = pyqtSignal(str)               # 도우미 상태 ("" = 정상)
    sig_cams = pyqtSignal(dict)               # GVCP {ip: info}

    def __init__(self, iface, parent=None):
        super().__init__(parent)
        self.iface = iface
        self._last = None
        self._prev = {}
        self._buf = ""
        self.timer = QTimer(self, interval=1000, timeout=self._tick)
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._on_out)
        self.proc.finished.connect(self._on_exit)
        self.disc = DiscoveryThread(iface)
        self.disc.sig_found.connect(self.sig_cams)
        self.disc_timer = QTimer(self, interval=30000, timeout=self._discover)

    def start(self):
        self.timer.start()
        path = net_tools.find_net_probe()
        if path:
            self.proc.start(path, [self.iface])
        else:
            self.sig_state.emit("net_probe 도우미 없음 — colcon build 필요")
        self._discover()
        self.disc_timer.start()

    def stop(self):
        self.timer.stop()
        self.disc_timer.stop()
        if self.proc.state() != QProcess.NotRunning:
            self.proc.kill()
            self.proc.waitForFinished(1000)
        self.disc.wait(2000)

    def _discover(self):
        if not self.disc.isRunning():
            self.disc.start()

    def _tick(self):
        st = net_tools.nic_stats(self.iface)
        if not st:
            return
        rx, _, speed, _ = st
        now = time.monotonic()
        if self._last:
            dt = now - self._last[0]
            if dt > 0:
                self.sig_total.emit(self.iface, (rx - self._last[1]) * 8 / dt / 1e6, speed)
        self._last = (now, rx)

    def _on_out(self):
        self._buf += bytes(self.proc.readAllStandardOutput()).decode(errors="replace")
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            if not line.startswith("{"):
                self.sig_state.emit(line)
                continue
            try:
                j = json.loads(line)
            except ValueError:
                continue
            dt = max(float(j.get("dt", 1.0)), 1e-3)
            rates = {}
            for ip, v in j.get("ips", {}).items():
                b, pk = v[0], v[1]
                flows = v[2] if len(v) > 2 else []
                pb, pp = self._prev.get(ip, (b, pk))
                rates[ip] = ((b - pb) * 8 / dt / 1e6, (pk - pp) / dt, flows)
                self._prev[ip] = (b, pk)
            self.sig_ips.emit(rates)
            self.sig_state.emit("")

    def _on_exit(self, code, _status):
        if code != 0:
            path = net_tools.find_net_probe() or "net_probe"
            self.sig_state.emit(
                f"IP별 측정 불가 — 한 번만 실행: sudo setcap cap_net_raw+ep {path}")


# ---------------- 누적 궤적 지도 ----------------

class MapLoadThread(QThread):
    """체크된 클립들의 궤적을 읽는다 (캐시 없으면 bag에서 추출 → 캐시)."""
    sig_progress = pyqtSignal(str)
    sig_done = pyqtSignal(list)          # [(라벨, 경로, 유효 fix 리스트)]
    sig_error = pyqtSignal(str)

    def __init__(self, clips):
        super().__init__()
        self.clips = clips               # [(라벨, 경로)]

    def run(self):
        try:
            out = []
            for lbl, path in self.clips:
                self.sig_progress.emit(f"궤적 읽는 중: {lbl}")
                fx = gnss_tools.get_track(path)
                out.append((lbl, path, [f for f in fx if gnss_tools._valid(f)]))
            self.sig_done.emit(out)
        except Exception as e:
            self.sig_error.emit(f"{type(e).__name__}: {e}")


class PngExportThread(QThread):
    sig_done = pyqtSignal(str)

    def __init__(self, tracks, out_png):
        super().__init__()
        self.tracks, self.out_png = tracks, out_png

    def run(self):
        try:
            self.sig_done.emit(gnss_tools.render_map_multi(self.tracks, self.out_png) or "")
        except Exception as e:
            self.sig_done.emit(f"ERROR: {e}")


class GnssMapWindow(QDialog):
    """GNSS 상태 + 지도(현재 위치, pre 구간 궤적, 누적 클립 궤적)를 한 창에.

    성능 원칙: 이 창은 메시지 콜백에서 절대 그리지 않는다. MainWindow의 타이머가
    주기적으로 update_live()를 호출하고, 클립 궤적은 체크 상태가 실제로 바뀔 때만 다시 읽는다.
    """

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.setWindowTitle("GNSS — 상태 · 현재 위치 · 클립 궤적")
        self.resize(1400, 860)
        self.cfg = cfg
        self.thread = None
        self._dirty = False
        self._tracks = []
        self._last_sel = None
        self._first_fix = True
        self._has_live = False

        h = QHBoxLayout(self)
        left = QVBoxLayout()
        left.setSpacing(6)

        # --- 상태 ---
        self.lbl_overall = QLabel("수신 대기…")
        self.lbl_overall.setFixedHeight(46)
        self.lbl_overall.setWordWrap(True)
        self.lbl_overall.setStyleSheet("font-size:14pt; font-weight:bold; color:#888;")
        left.addWidget(self.lbl_overall)
        grp = QGroupBox("수신 상태")
        form = QFormLayout(grp)
        form.setVerticalSpacing(3)
        self.rows = {}
        for key, name in (("pos_type", "솔루션 (pos_type)"), ("nav_status", "항법 상태"),
                          ("fix", "NavSatFix 상태"), ("quality", "GGA 품질"),
                          ("sats", "위성 수"), ("hdop", "HDOP"),
                          ("pos", "위도, 경도"), ("alt", "고도"),
                          ("hacc", "수평 정확도"), ("speed", "속도"),
                          ("rate", "fix 수신율"), ("age", "마지막 fix")):
            lab = QLabel("-")
            lab.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.rows[key] = lab
            form.addRow(name, lab)
        left.addWidget(grp)
        self.chk_follow = QCheckBox("현재 위치 따라가기")
        self.chk_follow.setChecked(True)
        left.addWidget(self.chk_follow)
        self.lbl_hint = QLabel("")
        self.lbl_hint.setFixedHeight(34)
        self.lbl_hint.setWordWrap(True)
        self.lbl_hint.setStyleSheet("color:#666; font-size:9pt;")
        left.addWidget(self.lbl_hint)

        # --- 클립 궤적 ---
        grp_c = QGroupBox("클립 궤적 (체크 = 지도에 표시)")
        vc = QVBoxLayout(grp_c)
        self.list = QListWidget()
        self.list.itemChanged.connect(self._schedule)
        self.list.itemDoubleClicked.connect(self._focus_item)
        vc.addWidget(self.list, 1)
        grid = QGridLayout()
        for r, c, text, fn in ((0, 0, "새로고침", self.refresh),
                               (0, 1, "다른 폴더의 클립 추가…", self._add_dir),
                               (1, 0, "전체 선택", lambda: self._set_all(Qt.Checked)),
                               (1, 1, "전체 해제", lambda: self._set_all(Qt.Unchecked)),
                               (2, 0, "클립 전체 보기", self._fit_all),
                               (2, 1, "PNG 내보내기…", self._export_png)):
            btn = QPushButton(text)
            btn.clicked.connect(fn)
            grid.addWidget(btn, r, c)
        vc.addLayout(grid)
        self.lbl_status = QLabel("휠: 확대/축소 · 드래그: 이동 · 목록 더블클릭: 해당 클립으로")
        self.lbl_status.setFixedHeight(34)          # 텍스트가 바뀌어도 레이아웃 고정
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setStyleSheet("color:#666; font-size:9pt;")
        vc.addWidget(self.lbl_status)
        left.addWidget(grp_c, 1)

        lw = QWidget()
        lw.setLayout(left)
        lw.setFixedWidth(390)
        h.addWidget(lw)
        self.map = MapWidget()
        h.addWidget(self.map, 1)

        self.debounce = QTimer(self, singleShot=True, interval=400, timeout=self.render)
        self.refresh()

    # ---------- 클립 목록 ----------
    def _clip_dirs(self):
        out = []
        base = Path(self.cfg["recorder"]["output_dir"])
        if base.is_dir():
            out += sorted(p for p in base.iterdir()
                          if p.is_dir() and p.name.startswith("clip_"))
        for extra in self.cfg["ui"].get("map_clips", []):
            p = Path(extra)
            if p.is_dir() and p not in out:
                out.append(p)
        return out

    def _make_item(self, path, checked):
        it = QListWidgetItem(Path(path).name)
        it.setData(Qt.UserRole, str(path))
        it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
        it.setCheckState(Qt.Checked if checked else Qt.Unchecked)
        cached = (Path(path) / gnss_tools.TRACK_FILE).exists()
        it.setToolTip(str(path) + ("" if cached else
                      "\n(궤적 캐시 없음 — 체크하면 bag에서 추출, 큰 bag은 수십 초)"))
        it.setForeground(QColor("black" if cached else "#888"))
        return it

    def _item_for(self, path):
        for i in range(self.list.count()):
            if self.list.item(i).data(Qt.UserRole) == path:
                return self.list.item(i)
        return None

    def refresh(self):
        checked = set(self.cfg["ui"].get("map_clips", []))
        self.list.blockSignals(True)
        self.list.clear()
        for p in self._clip_dirs():
            self.list.addItem(self._make_item(p, str(p) in checked))
        self.list.blockSignals(False)
        self._schedule()

    def add_clip(self, path, checked=True):
        path = str(Path(path))
        it = self._item_for(path)
        self.list.blockSignals(True)
        if it is None:
            self.list.addItem(self._make_item(path, checked))
        else:
            it.setCheckState(Qt.Checked if checked else Qt.Unchecked)
        self.list.blockSignals(False)
        self._schedule()

    def _selected(self):
        return [(self.list.item(i).text(), self.list.item(i).data(Qt.UserRole))
                for i in range(self.list.count())
                if self.list.item(i).checkState() == Qt.Checked]

    def _set_all(self, state):
        self.list.blockSignals(True)
        for i in range(self.list.count()):
            self.list.item(i).setCheckState(state)
        self.list.blockSignals(False)
        self._schedule()

    def _add_dir(self):
        d = QFileDialog.getExistingDirectory(self, "클립 폴더",
                                             self.cfg["recorder"]["output_dir"])
        if not d:
            return
        if not (list(Path(d).glob("*.db3")) or list(Path(d).glob("*.mcap"))):
            QMessageBox.warning(self, "클립", "rosbag2 클립 폴더가 아닙니다 (.db3/.mcap 없음)")
            return
        self.add_clip(d, checked=True)

    def _focus_item(self, item):
        path = item.data(Qt.UserRole)
        for _, p, fixes in self._tracks:
            if p == path and len(fixes) >= 2:
                self.chk_follow.setChecked(False)
                self.map.fit_bounds([(f["lat"], f["lon"]) for f in fixes])
                return

    def _fit_all(self):
        pts = [(f["lat"], f["lon"]) for _, _, fx in self._tracks for f in fx]
        if pts:
            self.chk_follow.setChecked(False)
            self.map.fit_bounds(pts)

    # ---------- 클립 궤적 렌더 ----------
    def _schedule(self, *_):
        # 체크 집합이 실제로 바뀐 경우에만 (색상 변경 등 다른 itemChanged는 무시 → 루프 방지)
        sel = [p for _, p in self._selected()]
        if sel == self._last_sel:
            return
        self._last_sel = sel
        self.cfg["ui"]["map_clips"] = sel
        save_config(self.cfg)
        self.debounce.start()

    def render(self):
        if self.thread and self.thread.isRunning():
            self._dirty = True
            return
        sel = self._selected()
        if not sel:
            self._clear_clip_overlays()
            self._tracks = []
            self.lbl_status.setText("클립을 체크하면 지도에 표시됩니다")
            return
        self.thread = MapLoadThread(sel)
        self.thread.sig_progress.connect(self.lbl_status.setText)
        self.thread.sig_done.connect(self._done)
        self.thread.sig_error.connect(lambda e: self.lbl_status.setText(f"궤적 읽기 실패: {e}"))
        self.thread.start()

    def _clear_clip_overlays(self):
        for oid in [o for o in list(self.map._overlays)
                    if o.startswith(("clip:", "s:", "e:"))]:
            self.map.remove(oid)

    def _done(self, tracks):
        self._tracks = tracks
        self._clear_clip_overlays()
        shown, none, allpts = [], [], []
        self.list.blockSignals(True)               # 색상 변경이 itemChanged를 쏘지 않게
        try:
            for i in range(self.list.count()):
                it = self.list.item(i)
                cached = (Path(it.data(Qt.UserRole)) / gnss_tools.TRACK_FILE).exists()
                it.setForeground(QColor("black" if cached else "#888"))
            for i, (lbl, path, fixes) in enumerate(tracks):
                if len(fixes) < 2:
                    none.append(lbl)
                    continue
                pts = [(f["lat"], f["lon"]) for f in fixes]
                color = gnss_tools.TRACK_COLORS[i % len(gnss_tools.TRACK_COLORS)]
                self.map.add_track(f"clip:{path}", pts, color, 3.0, z=5)
                self.map.set_marker(f"s:{path}", *pts[0], color=color, radius=5, z=6)
                self.map.set_marker(f"e:{path}", *pts[-1], color=color, radius=5, label=lbl, z=6)
                it = self._item_for(path)
                if it is not None:
                    it.setForeground(QColor(color))
                shown.append(lbl)
                allpts += pts
        finally:
            self.list.blockSignals(False)
        # 실시간 fix를 따라가는 중이면 뷰를 뺏지 않는다
        if allpts and not (self._has_live and self.chk_follow.isChecked()):
            self.map.fit_bounds(allpts)
        txt = f"클립 {len(shown)}개 표시"
        if none:
            txt += f" — GNSS 없음: {', '.join(none)}"
        self.lbl_status.setText(txt)
        if self._dirty:
            self._dirty = False
            self.render()

    def _export_png(self):
        tracks = [(lbl, fx) for lbl, _, fx in self._tracks if len(fx) >= 2]
        if not tracks:
            QMessageBox.information(self, "PNG", "표시 중인 클립 궤적이 없습니다")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "PNG 내보내기", str(Path(self.cfg["recorder"]["output_dir"]) / "gnss_map.png"),
            "PNG (*.png)")
        if not path:
            return
        self._png = PngExportThread(tracks, path)
        self._png.sig_done.connect(lambda p: self.lbl_status.setText(
            f"PNG 저장: {p}" if p and not p.startswith("ERROR") else f"PNG 실패: {p}"))
        self._png.start()

    # ---------- 실시간 (타이머에서만 호출) ----------
    def update_live(self, g, trail, pre_sec):
        fix, gga, vel = g.get("fix"), g.get("gga"), g.get("vel")
        pt = (g.get("pos_type") or {}).get("text")
        ns = (g.get("nav_status") or {}).get("text")
        now = time.time()
        age = (now - fix["t"]) if fix else None
        live = fix is not None and age < 3.0 and fix["status"] >= 0
        self._has_live = live
        r = self.rows
        r["pos_type"].setText(pt or "-")
        r["nav_status"].setText(ns or "-")
        if fix is None:
            r["fix"].setText("수신 없음")
        else:
            nm = {-1: "NO_FIX", 0: "FIX", 1: "SBAS_FIX", 2: "GBAS_FIX"}.get(
                fix["status"], str(fix["status"]))
            r["fix"].setText(nm + ("" if age < 3.0 else f"  (끊김 {age:.0f}s)"))
        r["quality"].setText(GGA_QUALITY.get(gga["quality"], str(gga["quality"])) if gga else "-")
        r["sats"].setText(str(gga["sats"]) if gga else "-")
        r["hdop"].setText(f"{gga['hdop']:.1f}" if gga else "-")
        r["pos"].setText(f"{fix['lat']:.7f}, {fix['lon']:.7f}" if fix else "-")
        r["alt"].setText(f"{fix['alt']:.1f} m" if fix else "-")
        r["hacc"].setText(f"{fix['hacc']:.2f} m"
                          if fix and fix.get("hacc") is not None else "-")
        r["speed"].setText(f"{vel['speed_mps'] * 3.6:.1f} km/h"
                           if vel and now - vel["t"] < 3.0 else "-")
        recent = sum(1 for t, _, _ in trail if t > now - 5.0)
        r["rate"].setText(f"{recent / 5.0:.1f} Hz (GUI 10 Hz 제한)" if recent else "-")
        r["age"].setText("-" if age is None else f"{age:.1f} s 전")

        if not live:
            txt, col = ("GNSS 없음 — fix 수신 안 됨" if fix is None else "fix 끊김"), "#dc322f"
        else:
            q = gga["quality"] if gga else None
            up = (pt or "").upper()
            if q == 4 or ("RTK" in up and "FIX" in up):
                txt, col = "양호 — RTK FIXED", "#859900"
            elif q in (2, 5) or "FLOAT" in up or "DGPS" in up:
                txt, col = "보통 — RTK FLOAT / DGPS", "#b58900"
            else:
                txt, col = "단독 측위 (SPS)", "#cb4b16"
        self.lbl_overall.setText(txt)
        self.lbl_overall.setStyleSheet(f"font-size:14pt; font-weight:bold; color:{col};")

        # 지도: pre 구간 궤적 (그리기용으로 최대 ~400점으로 솎음)
        pts = [(la, lo) for _, la, lo in trail]
        step = max(1, len(pts) // 400)
        draw_pts = pts[::step]
        if len(pts) > 1 and (len(pts) - 1) % step:
            draw_pts.append(pts[-1])
        self.lbl_hint.setText(
            f"주황: 지금 트리거하면 담기는 pre {pre_sec:.0f}초 궤적 ({len(pts)} fix) · "
            f"초록: pre 시작 · 파랑: 현재 위치")
        if len(draw_pts) >= 2:
            self.map.add_track("trail", draw_pts, "#ff6f00", width=4.0, z=15)
        else:
            self.map.remove("trail")
        if pts:
            self.map.set_marker("trail_start", *pts[0], color="#2e7d32", radius=6,
                                label=f"-{pre_sec:.0f}s")
        else:
            self.map.remove("trail_start")
        if live:
            self.map.set_marker("me", fix["lat"], fix["lon"], color="#2962ff",
                                radius=8, label="현재")
            if self._first_fix:
                self._first_fix = False
                self.map.set_view(fix["lat"], fix["lon"], 16)
            elif self.chk_follow.isChecked():
                self.map.set_view(fix["lat"], fix["lon"])


# ---------------- 토픽 선택 다이얼로그 ----------------

class TopicDialog(QDialog):
    COL_CHECK, COL_NAME, COL_TYPE, COL_REL, COL_DUR = range(5)

    def __init__(self, worker, cfg, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.cfg = cfg
        self.setWindowTitle("녹화 토픽 선택")
        self.resize(860, 560)

        v = QVBoxLayout(self)
        top = QHBoxLayout()
        self.filter_edit = QLineEdit(placeholderText="필터…")
        self.filter_edit.textChanged.connect(self._apply_filter)
        btn_refresh = QPushButton("새로고침")
        btn_refresh.clicked.connect(self.refresh)
        btn_all = QPushButton("전체 선택")
        btn_all.clicked.connect(lambda: self._set_all(Qt.Checked))
        btn_none = QPushButton("전체 해제")
        btn_none.clicked.connect(lambda: self._set_all(Qt.Unchecked))
        for w in (self.filter_edit, btn_refresh, btn_all, btn_none):
            top.addWidget(w)
        v.addLayout(top)

        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(
            ["녹화", "토픽", "타입", "reliability", "durability"])
        self.table.horizontalHeader().setSectionResizeMode(
            self.COL_NAME, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        v.addWidget(self.table)

        hint = QLabel(
            "기본 QoS는 best_effort / volatile (모든 퍼블리셔와 호환). "
            "transient_local 퍼블리셔(tf_static 등)가 감지된 토픽은 자동으로 "
            "durability=transient_local 로 잡아준다 — volatile로 받으면 "
            "구독 이전에 발행된 latched 메시지를 놓치기 때문.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #666;")
        v.addWidget(hint)

        bot = QHBoxLayout()
        btn_load = QPushButton("프로파일 불러오기…")
        btn_load.clicked.connect(self._load_profile)
        btn_save = QPushButton("프로파일 저장…")
        btn_save.clicked.connect(self._save_profile)
        bot.addWidget(btn_load)
        bot.addWidget(btn_save)
        bot.addStretch()
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        bot.addWidget(bb)
        v.addLayout(bot)

        self.refresh()
        # 디스커버리가 아직 도는 중일 수 있으니 잠시 후 한 번 더
        QTimer.singleShot(1200, self.refresh)

    # 이전 설정 { name: (rel, dur) }
    def _prev_map(self):
        return {t["name"]: (t.get("reliability", "best_effort"),
                            t.get("durability", "volatile"))
                for t in self.cfg.get("topics", [])}

    def refresh(self):
        prev = self._prev_map()
        # 화면에 이미 있는 행의 현재 상태 보존
        for row in range(self.table.rowCount()):
            name = self.table.item(row, self.COL_NAME).text()
            prev[name] = self._row_qos(row) if self._row_checked(row) \
                else prev.get(name, None) if name in prev else None
            if not self._row_checked(row) and name in prev and prev[name] is None:
                del prev[name]

        topics = self.worker.list_topics()
        self.table.setRowCount(0)
        ui = self.cfg["ui"]
        for t in topics:
            row = self.table.rowCount()
            self.table.insertRow(row)
            chk = QTableWidgetItem()
            chk.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            chk.setCheckState(Qt.Checked if t["name"] in prev else Qt.Unchecked)
            self.table.setItem(row, self.COL_CHECK, chk)
            self.table.setItem(row, self.COL_NAME,
                               QTableWidgetItem(t["name"]))
            self.table.setItem(row, self.COL_TYPE,
                               QTableWidgetItem(t["type"]))
            rel_c = QComboBox()
            rel_c.addItems(["best_effort", "reliable", "auto"])
            dur_c = QComboBox()
            dur_c.addItems(["volatile", "transient_local", "auto"])
            if t["name"] in prev and prev[t["name"]]:
                rel, dur = prev[t["name"]]
            else:
                rel = ui["default_reliability"]
                dur = "transient_local" if t["transient_pub"] \
                    else ui["default_durability"]
            rel_c.setCurrentText(rel)
            dur_c.setCurrentText(dur)
            self.table.setCellWidget(row, self.COL_REL, rel_c)
            self.table.setCellWidget(row, self.COL_DUR, dur_c)
        self._apply_filter(self.filter_edit.text())

    def _row_checked(self, row):
        return self.table.item(row, self.COL_CHECK).checkState() == Qt.Checked

    def _row_qos(self, row):
        return (self.table.cellWidget(row, self.COL_REL).currentText(),
                self.table.cellWidget(row, self.COL_DUR).currentText())

    def _apply_filter(self, text):
        text = text.strip().lower()
        for row in range(self.table.rowCount()):
            name = self.table.item(row, self.COL_NAME).text().lower()
            self.table.setRowHidden(row, bool(text) and text not in name)

    def _set_all(self, state):
        for row in range(self.table.rowCount()):
            if not self.table.isRowHidden(row):
                self.table.item(row, self.COL_CHECK).setCheckState(state)

    def selected(self):
        out = []
        for row in range(self.table.rowCount()):
            if not self._row_checked(row):
                continue
            rel, dur = self._row_qos(row)
            out.append({
                "name": self.table.item(row, self.COL_NAME).text(),
                "type": self.table.item(row, self.COL_TYPE).text(),
                "reliability": rel, "durability": dur})
        return out

    def _load_profile(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "프로파일 불러오기", str(CONFIG_DIR), "YAML (*.yaml *.yml)")
        if not path:
            return
        self.cfg = load_config(path)
        self.refresh()

    def _save_profile(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "프로파일 저장", str(CONFIG_DIR / "profile.yaml"),
            "YAML (*.yaml *.yml)")
        if not path:
            return
        cfg = dict(self.cfg)
        cfg["topics"] = self.selected()
        save_config(cfg, path)


# ---------------- 녹화/앱 설정 다이얼로그 ----------------

class SettingsDialog(QDialog):
    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.setWindowTitle("설정")
        rec, ui = cfg["recorder"], cfg["ui"]
        form = QFormLayout(self)

        def dspin(val, lo, hi, step=1.0, suffix=""):
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setValue(val)
            s.setSingleStep(step)
            s.setSuffix(suffix)
            return s

        self.pre = dspin(rec["pre_sec"], 0, 600, 1, " s")
        self.post = dspin(rec["post_sec"], 0, 600, 1, " s")
        self.slack = dspin(rec["trigger_slack_sec"], 0, 60, 0.5, " s")
        self.cap = dspin(rec["max_buffer_mb"], 64, 131072, 512, " MB")
        self.qd = QSpinBox()
        self.qd.setRange(1, 1000)
        self.qd.setValue(rec["queue_depth"])
        self.outdir = QLineEdit(rec["output_dir"])
        btn_dir = QPushButton("…")
        btn_dir.setFixedWidth(28)
        btn_dir.clicked.connect(self._pick_dir)
        dir_row = QHBoxLayout()
        dir_row.addWidget(self.outdir)
        dir_row.addWidget(btn_dir)
        self.storage = QComboBox()
        self.storage.addItems(["sqlite3", "mcap"])
        self.storage.setCurrentText(rec["storage_id"])
        self.key = QKeySequenceEdit(QKeySequence(ui["shortcut"]))
        self.auto_diag = QCheckBox("클립 저장 완료 시 자동 진단")
        self.auto_diag.setChecked(ui["auto_diagnose"])
        self.auto_start = QCheckBox("시작 시 레코더 자동 실행 (외부 노드 없을 때)")
        self.auto_start.setChecked(ui["auto_start_recorder"])
        self.nic = QComboBox()
        self.nic.addItem("auto")
        for name, speed in net_tools.nic_list():
            self.nic.addItem(f"{name}" if speed < 0 else f"{name}", name)
        idx = self.nic.findText(ui.get("nic", "auto"))
        self.nic.setCurrentIndex(max(0, idx))
        self.ip_link = QSpinBox()
        self.ip_link.setRange(10, 100000)
        self.ip_link.setSuffix(" Mbps")
        self.ip_link.setValue(int(ui.get("per_ip_link_mbps", 1000)))

        form.addRow("pre_sec (트리거 이전)", self.pre)
        form.addRow("post_sec (트리거 이후)", self.post)
        form.addRow("trigger_slack_sec", self.slack)
        form.addRow("max_buffer_mb", self.cap)
        form.addRow("queue_depth", self.qd)
        form.addRow("저장 경로", dir_row)
        form.addRow("스토리지", self.storage)
        form.addRow("트리거 단축키", self.key)
        form.addRow(self.auto_diag)
        form.addRow(self.auto_start)
        form.addRow("스위치 업링크 NIC", self.nic)
        form.addRow("장치 포트 링크 속도", self.ip_link)
        note = QLabel("pre/post/버퍼 설정은 레코더 재시작 시 적용됩니다 "
                      "(GUI가 띄운 레코더는 자동 재시작).")
        note.setStyleSheet("color:#666;")
        form.addRow(note)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def _pick_dir(self):
        d = QFileDialog.getExistingDirectory(self, "저장 경로",
                                             self.outdir.text())
        if d:
            self.outdir.setText(d)

    def apply_to(self, cfg):
        rec, ui = cfg["recorder"], cfg["ui"]
        before = dict(rec)
        rec.update(pre_sec=self.pre.value(), post_sec=self.post.value(),
                   trigger_slack_sec=self.slack.value(),
                   max_buffer_mb=self.cap.value(),
                   queue_depth=self.qd.value(),
                   output_dir=self.outdir.text(),
                   storage_id=self.storage.currentText())
        ui.update(shortcut=self.key.keySequence().toString() or "F9",
                  auto_diagnose=self.auto_diag.isChecked(),
                  auto_start_recorder=self.auto_start.isChecked(),
                  nic=self.nic.currentText(),
                  per_ip_link_mbps=self.ip_link.value())
        return before != rec        # 레코더 재시작 필요 여부


# ---------------- 진단 결과 다이얼로그 ----------------

class DiagDialog(QDialog):
    def __init__(self, report, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"클립 진단 — {Path(report['bag']).name}")
        self.resize(940, 620)
        v = QVBoxLayout(self)

        lvl = report["level"]
        color = {"OK": "#859900", "WARN": "#b58900", "FAIL": "#dc322f"}[lvl]
        head = QLabel(
            f"<b style='color:{color}; font-size:14pt'>{lvl}</b> — "
            f"{report['duration']:.1f}s, {report['total_msgs']}개, "
            f"{report['total_mb']:.0f} MB"
            + (f" | 문제 토픽: {', '.join(report['fail_topics'] + report['warn_topics'])}"
               if lvl != "OK" else " | 모든 토픽 정상"))
        head.setWordWrap(True)
        v.addWidget(head)

        table = QTableWidget(0, 7)
        table.setHorizontalHeaderLabels(
            ["판정", "토픽", "수신", "중간유실", "경계잘림", "Hz", "지연 p95"])
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        table.verticalHeader().setVisible(False)
        order = {"FAIL": 0, "WARN": 1, "OK": 2}
        for name, t in sorted(report["topics"].items(),
                              key=lambda kv: (order[kv[1]["level"]], kv[0])):
            row = table.rowCount()
            table.insertRow(row)
            lvl_item = QTableWidgetItem(t["level"])
            lvl_item.setForeground(QColor(
                {"OK": "#859900", "WARN": "#b58900",
                 "FAIL": "#dc322f"}[t["level"]]))
            table.setItem(row, 0, lvl_item)
            table.setItem(row, 1, QTableWidgetItem(name))
            table.setItem(row, 2, QTableWidgetItem(str(t["count"])))
            table.setItem(row, 3, QTableWidgetItem(str(t["lost_mid"])))
            table.setItem(row, 4, QTableWidgetItem(
                f"{t['head_trunc']}/{t['tail_trunc']}"))
            table.setItem(row, 5, QTableWidgetItem(
                str(t.get("rate_hz", "-"))))
            table.setItem(row, 6, QTableWidgetItem(
                f"{t['lat_p95_ms']:.0f} ms" if "lat_p95_ms" in t else "-"))
        v.addWidget(table, 2)

        # GNSS 요약 + 지도
        self.report = report
        self.map_thread = None
        if report.get("gnss"):
            gl = QHBoxLayout()
            parts = []
            for tname, q in report["gnss"].items():
                seg = f"{tname}: [{q['level']}]"
                if "fix_ratio" in q:
                    seg += f" 유효 fix {q['fix_ratio']*100:.0f}%"
                if "hacc_p95_m" in q:
                    seg += f", 수평정확도 p95 {q['hacc_p95_m']} m"
                if "track_m" in q:
                    seg += f", 이동 {q['track_m']} m"
                if q.get("pos_type_hist"):
                    seg += f", pos_type {q['pos_type_hist']}"
                parts.append(seg)
            lbl = QLabel("GNSS — " + " | ".join(parts))
            lbl.setWordWrap(True)
            gl.addWidget(lbl, 1)
            self.btn_map = QPushButton("궤적 지도 (OSM)")
            has_track = any("track_m" in q for q in report["gnss"].values())
            self.btn_map.setEnabled(has_track)
            if not has_track:
                self.btn_map.setToolTip("유효한 fix가 없어 지도를 그릴 수 없음")
            self.btn_map.clicked.connect(self._show_map)
            gl.addWidget(self.btn_map)
            v.addLayout(gl)

        detail = QTextEdit(readOnly=True)
        detail.setFont(QFont("Monospace", 9))
        detail.setPlainText(bag_diagnostics.render_text(report))
        v.addWidget(detail, 1)

        bot = QHBoxLayout()
        note = QLabel(f"보고서 저장됨: {report['bag']}/diagnostics.txt / .json")
        note.setStyleSheet("color:#666;")
        bot.addWidget(note)
        bot.addStretch()
        btn = QPushButton("닫기")
        btn.clicked.connect(self.accept)
        bot.addWidget(btn)
        v.addLayout(bot)

    def _show_map(self):
        try:
            fixes = gnss_tools.get_track(self.report["bag"])
        except Exception as e:
            QMessageBox.warning(self, "지도", f"궤적 읽기 실패: {e}")
            return
        pts = [(f["lat"], f["lon"]) for f in fixes if gnss_tools._valid(f)]
        if len(pts) < 2:
            QMessageBox.information(self, "지도", "유효한 fix가 없어 지도를 그릴 수 없습니다.")
            return
        dlg = QDialog(self)
        dlg.setWindowTitle(f"GNSS 궤적 — {Path(self.report['bag']).name}")
        lay = QVBoxLayout(dlg)
        m = MapWidget()
        m.add_track("clip", pts, "#1565c0", 3.0)
        m.set_marker("s", *pts[0], color="#2e7d32", label="시작")
        m.set_marker("e", *pts[-1], color="#c62828", label="끝")
        lay.addWidget(m)
        dlg.resize(1000, 800)
        dlg.show()
        QTimer.singleShot(50, lambda: m.fit_bounds(pts))


# ---------------- 메인 창 ----------------

class MainWindow(QMainWindow):
    def __init__(self, worker, cfg):
        super().__init__()
        self.worker = worker
        self.cfg = cfg
        self.recorder = RecorderManager(self)
        self.owns_recorder = False
        self.busy = False
        self.last_clip = None
        self.diag_thread = None
        self._cap_warned = False
        self._disk_warned = False
        self._last_rate = 0.0     # 최근 유입 MB/s (디스크 여유 → 클립 수 환산용)
        self._cams = {}           # GVCP 디스커버리 {ip: info}
        self._own_ips = net_tools.own_ipv4s()
        self._ip_seen = {}        # ip -> 마지막으로 트래픽이 있었던 시각
        self._ip_flows = {}       # ip -> 플로우 힌트
        self._serials = {}        # {camera_serial: 드라이버 네임스페이스}
        self._gnss = {"pos_type": None, "nav_status": None, "fix": None, "gga": None}
        self.gnss_win = None       # GnssMapWindow (상태 + 지도 + 클립 궤적, 한 창)
        self._trail = deque()      # (t, lat, lon) — 최근 pre_sec 초의 유효 fix

        self.setWindowTitle("DM Clip GUI — 데이터 로깅")
        self.resize(1000, 900)
        self._build_ui()
        self._build_menu()

        worker.sig_diag.connect(self._on_diag)
        worker.sig_rosout.connect(lambda lv, m: self.log(lv, m))
        worker.sig_event.connect(self._on_clip_event)
        worker.sig_params.connect(self._on_params_result)
        worker.sig_gnss.connect(self._on_gnss)
        worker.sig_serials.connect(self._on_serials)
        self.recorder.readyReadStandardOutput.connect(self._on_proc_out)

        # 네트워크 모니터 (스위치 업링크 NIC)
        nic = cfg["ui"].get("nic", "auto")
        self.netmon = NetMonitor(net_tools.auto_nic() if nic == "auto" else nic, self)
        self.netmon.sig_total.connect(self._on_net_total)
        self.netmon.sig_ips.connect(self._on_net_ips)
        self.netmon.sig_state.connect(self._on_net_state)
        self.netmon.sig_cams.connect(self._on_cams)
        self.netmon.start()
        QApplication.instance().aboutToQuit.connect(self.netmon.stop)
        # GNSS 표시는 메시지마다가 아니라 2 Hz 타이머로만 갱신 (RT2000 100 Hz 대비)
        self.gnss_timer = QTimer(self, interval=500, timeout=self._refresh_gnss)
        self.gnss_timer.start()
        self.recorder.finished.connect(self._on_proc_exit)

        # 레코더 생존 감시
        self.alive_timer = QTimer(self, interval=2000,
                                  timeout=self._update_alive)
        self.alive_timer.start()

        # busy 안전 해제 (post_sec + 여유 후에도 완료 이벤트가 없으면)
        self.busy_timer = QTimer(self, singleShot=True,
                                 timeout=self._busy_timeout)

        self._apply_shortcut()
        QTimer.singleShot(300, self._startup_recorder)

    # --- UI 구성 ---
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        v = QVBoxLayout(central)

        # 상단: 레코더 상태
        top = QHBoxLayout()
        self.lbl_alive = QLabel("레코더: 확인 중…")
        self.btn_recorder = QPushButton("레코더 시작")
        self.btn_recorder.clicked.connect(self._toggle_recorder)
        btn_map = QPushButton("GNSS · 지도")
        btn_map.clicked.connect(self.open_gnss)
        top.addWidget(self.lbl_alive)
        top.addStretch()
        top.addWidget(btn_map)
        top.addWidget(self.btn_recorder)
        v.addLayout(top)

        # 버퍼 상태
        grp_buf = QGroupBox("링 버퍼")
        g = QGridLayout(grp_buf)
        self.bar_mem = QProgressBar(format="%v / %m MB")
        self.bar_span = QProgressBar(format="버퍼링 준비 중…")
        self.lbl_inflow = QLabel("유입 - MB/s")
        g.addWidget(QLabel("메모리"), 0, 0)
        g.addWidget(self.bar_mem, 0, 1)
        g.addWidget(QLabel("보관 구간"), 1, 0)
        g.addWidget(self.bar_span, 1, 1)
        g.addWidget(self.lbl_inflow, 2, 1)
        self.bar_disk = QProgressBar()
        self.lbl_disk = QLabel("")
        self.lbl_disk.setStyleSheet("color:#666;")
        g.addWidget(QLabel("디스크"), 3, 0)
        g.addWidget(self.bar_disk, 3, 1)
        g.addWidget(self.lbl_disk, 4, 1)
        v.addWidget(grp_buf)

        # 네트워크 (스위치 업링크 + 장치별) / GNSS
        row = QHBoxLayout()
        grp_net = QGroupBox("네트워크 — 스위치 업링크")
        gn = QGridLayout(grp_net)
        self.lbl_nic = QLabel("NIC: -")
        self.bar_nic = QProgressBar(format="- / - Gbps")
        gn.addWidget(self.lbl_nic, 0, 0)
        gn.addWidget(self.bar_nic, 0, 1)
        self.net_table = QTableWidget(0, 5)
        self.net_table.setHorizontalHeaderLabels(
            ["장치", "IP", "Mbps", "포트 사용률", "pps"])
        self.net_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.net_table.verticalHeader().setVisible(False)
        self.net_table.setEditTriggers(QTableWidget.DoubleClicked)   # 장치 이름 직접 지정
        self.net_table.itemChanged.connect(self._on_net_alias_edited)
        self.net_table.setToolTip("장치 칸을 더블클릭하면 이름을 직접 지정할 수 있습니다 (설정에 저장)")
        self.net_table.setMaximumHeight(190)
        gn.addWidget(self.net_table, 1, 0, 1, 2)
        self.lbl_net_state = QLabel("")
        self.lbl_net_state.setWordWrap(True)
        self.lbl_net_state.setStyleSheet("color:#b58900;")
        self.btn_grant = QPushButton("권한 부여 (비밀번호)")
        self.btn_grant.setToolTip("net_probe에 cap_net_raw 부여 — pkexec 로 관리자 인증 창이 뜹니다")
        self.btn_grant.clicked.connect(self._grant_net_probe)
        self.btn_grant.hide()
        state_row = QHBoxLayout()
        state_row.addWidget(self.lbl_net_state, 1)
        state_row.addWidget(self.btn_grant)
        gn.addLayout(state_row, 2, 0, 1, 2)
        row.addWidget(grp_net, 3)

        grp_gnss = QGroupBox("GNSS")
        gg = QGridLayout(grp_gnss)
        self.lbl_gnss_status = QLabel("수신 대기…")
        self.lbl_gnss_status.setWordWrap(True)
        self.lbl_gnss_pos = QLabel("-")
        self.lbl_gnss_q = QLabel("-")
        for r_, (k, w) in enumerate((("상태", self.lbl_gnss_status),
                                     ("위치", self.lbl_gnss_pos),
                                     ("품질", self.lbl_gnss_q))):
            gg.addWidget(QLabel(k), r_, 0, Qt.AlignTop)
            gg.addWidget(w, r_, 1)
        btn_g = QPushButton("GNSS · 지도 열기")
        btn_g.clicked.connect(self.open_gnss)
        gg.addWidget(btn_g, 3, 0, 1, 2)
        gg.setRowStretch(4, 1)
        row.addWidget(grp_gnss, 2)
        v.addLayout(row)

        # 트리거
        grp_trig = QGroupBox("클립 트리거")
        h = QHBoxLayout(grp_trig)
        h.addWidget(QLabel("라벨:"))
        self.label_edit = QLineEdit(
            placeholderText="클립 폴더명에 붙일 라벨 (선택)")
        h.addWidget(self.label_edit, 1)
        self.btn_trigger = QPushButton()
        self.btn_trigger.setMinimumHeight(56)
        f = self.btn_trigger.font()
        f.setPointSize(13)
        f.setBold(True)
        self.btn_trigger.setFont(f)
        self.btn_trigger.setStyleSheet(
            "QPushButton {background:#c62828; color:white; border-radius:6px;}"
            "QPushButton:disabled {background:#888;}")
        self.btn_trigger.clicked.connect(self.trigger_clip)
        h.addWidget(self.btn_trigger, 1)
        v.addWidget(grp_trig)

        self.lbl_state = QLabel("대기 중")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        v.addWidget(self.lbl_state)

        # 토픽별 유입량 + 로그
        split = QSplitter(Qt.Vertical)
        self.topic_table = QTableWidget(0, 3)
        self.topic_table.setHorizontalHeaderLabels(["토픽", "Hz", "대역폭"])
        self.topic_table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.Stretch)
        self.topic_table.verticalHeader().setVisible(False)
        self.topic_table.setEditTriggers(QTableWidget.NoEditTriggers)
        split.addWidget(self.topic_table)

        self.log_view = QTextEdit(readOnly=True)
        self.log_view.setFont(QFont("Monospace", 9))
        self.log_view.document().setMaximumBlockCount(3000)
        split.addWidget(self.log_view)
        split.setSizes([260, 220])
        v.addWidget(split, 1)

    def _build_menu(self):
        m_file = self.menuBar().addMenu("파일(&F)")
        m_file.addAction("토픽 선택…", self.open_topic_dialog)
        m_file.addAction("설정…", self.open_settings)
        m_file.addSeparator()
        m_file.addAction("프로파일 불러오기…", self._load_profile)
        m_file.addAction("프로파일 저장…", self._save_profile)
        m_file.addSeparator()
        m_file.addAction("종료", self.close)
        m_tool = self.menuBar().addMenu("도구(&T)")
        m_tool.addAction("마지막 클립 진단", self._diag_last)
        m_tool.addAction("클립 폴더에서 진단…", self._diag_pick)
        m_tool.addAction("클립 폴더 열기", self._open_clip_dir)
        m_tool.addSeparator()
        m_tool.addAction("GNSS · 지도 (상태 / 현재 위치 / 클립 궤적)…", self.open_gnss)

    # --- 로그 ---
    def log(self, level, text):
        color = LOG_COLORS.get(level)
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] [{level}] {text}"
        if color:
            self.log_view.append(
                f'<span style="color:{color}">{line}</span>')
        else:
            self.log_view.append(line)
        self.log_view.moveCursor(QTextCursor.End)

    # --- 레코더 관리 ---
    def _startup_recorder(self):
        if self.worker.recorder_alive():
            self.owns_recorder = False
            self.log("GUI", "외부 clip_recorder 노드에 연결 — 토픽 설정을 적용합니다")
            self._apply_topics_runtime()
        elif self.cfg["ui"]["auto_start_recorder"]:
            self._start_recorder()
        self._update_alive()

    def _start_recorder(self):
        if self.worker.recorder_alive():
            self.log("WARN", "clip_recorder가 이미 실행 중 — 그 노드에 붙습니다")
            self.owns_recorder = False
            return
        self.recorder.start_recorder(self.cfg)
        self.owns_recorder = True
        self.log("GUI", f"레코더 시작 (pre={self.cfg['recorder']['pre_sec']}s "
                        f"post={self.cfg['recorder']['post_sec']}s, "
                        f"토픽 {len(self.cfg['topics'])}개)")

    def _toggle_recorder(self):
        if self.worker.recorder_alive():
            if self.owns_recorder:
                self.recorder.stop_recorder()
                self.log("GUI", "레코더 정지")
            else:
                self.log("WARN", "외부에서 띄운 레코더는 GUI에서 정지하지 않습니다")
        else:
            self._start_recorder()

    def _restart_recorder(self):
        if self.owns_recorder:
            self.recorder.stop_recorder()
            QTimer.singleShot(500, self._start_recorder)
        else:
            self.log("WARN", "pre/post 등 정적 설정 변경은 외부 레코더에는 "
                             "적용 불가 — 레코더를 재시작하세요")

    def _apply_topics_runtime(self):
        topics = [t["name"] for t in self.cfg["topics"]]
        qos = [f"{t['name']} {t['reliability']} {t['durability']}"
               for t in self.cfg["topics"]]
        self.worker.set_recorder_params(topics, qos)

    def _on_params_result(self, ok, msg):
        self.log("OK" if ok else "ERROR", f"파라미터 적용: {msg}")

    def _on_proc_out(self):
        # 레코더 stdout은 /rosout과 중복되므로 rosout에 안 나오는 것만 (크래시 등)
        text = bytes(self.recorder.readAllStandardOutput()).decode(
            errors="replace")
        for line in text.splitlines():
            if line and "[INFO]" not in line and "[WARN]" not in line \
                    and "[ERROR]" not in line:
                self.log("GUI", f"(레코더) {line}")

    def _on_proc_exit(self, code, _status):
        if self.owns_recorder:
            lv = "ERROR" if code != 0 else "GUI"
            self.log(lv, f"레코더 프로세스 종료 (exit={code})")
        self._update_alive()

    def _update_alive(self):
        alive = self.worker.recorder_alive()
        if alive:
            src = "GUI 소유" if self.owns_recorder else "외부"
            self.lbl_alive.setText(f"레코더: 실행 중 ({src})")
            self.lbl_alive.setStyleSheet("color:#859900; font-weight:bold;")
            self.btn_recorder.setText("레코더 정지")
        else:
            self.lbl_alive.setText("레코더: 정지됨")
            self.lbl_alive.setStyleSheet("color:#dc322f; font-weight:bold;")
            self.btn_recorder.setText("레코더 시작")
        self.btn_trigger.setEnabled(alive and not self.busy)
        self._update_disk()

    # --- 네트워크 ---
    def _on_net_total(self, iface, mbps, link):
        cap = link if link > 0 else 10000
        self.bar_nic.setMaximum(cap)
        self.bar_nic.setValue(min(int(mbps), cap))
        pct = mbps / cap
        self.bar_nic.setFormat(f"{mbps/1000:.2f} / {cap/1000:.0f} Gbps  ({pct*100:.0f}%)")
        self.lbl_nic.setText(f"NIC {iface}")
        self.bar_nic.setStyleSheet(
            "QProgressBar::chunk{background:#dc322f;}" if pct > 0.9 else
            "QProgressBar::chunk{background:#b58900;}" if pct > 0.75 else "")

    def _on_net_ips(self, rates):
        link = float(self.cfg["ui"].get("per_ip_link_mbps", 1000))
        now = time.monotonic()
        for ip, (mbps, pps, flows) in rates.items():
            if flows:
                self._ip_flows[ip] = flows
            if pps > 0:
                self._ip_seen[ip] = now
            self._ip_seen.setdefault(ip, now)
        # 5분 넘게 조용한 IP는 목록에서 뺀다 (net_probe는 한 번 본 IP를 계속 보고함)
        rows = sorted(((ip, v) for ip, v in rates.items()
                       if now - self._ip_seen.get(ip, now) < 300),
                      key=lambda kv: -kv[1][0])
        self.net_table.blockSignals(True)        # 채우는 동안 itemChanged 무시
        self.net_table.setRowCount(len(rows))
        for r, (ip, (mbps, pps, _)) in enumerate(rows):
            pct = mbps / link * 100
            idle = now - self._ip_seen.get(ip, now)
            cells = [self._alias(ip), ip,
                     f"{mbps:.0f}" if mbps >= 10 else f"{mbps:.2f}",
                     f"{pct:.0f}% / {link:.0f}M",
                     f"{pps:.0f}" if idle < 10 else f"유휴 {idle:.0f}s"]
            for c, txt in enumerate(cells):
                it = QTableWidgetItem(txt)
                if c != 0:
                    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if idle >= 10:
                    it.setForeground(QColor("#999"))
                elif pct > 90:
                    it.setForeground(QColor("#dc322f"))
                elif pct > 75:
                    it.setForeground(QColor("#b58900"))
                self.net_table.setItem(r, c, it)
            self.net_table.item(r, 0).setToolTip(
                f"{ip}\n힌트: {net_tools.describe_flows(self._ip_flows.get(ip)) or '-'}"
                "\n더블클릭해서 이름 지정")
        self.net_table.blockSignals(False)

    def _on_net_alias_edited(self, item):
        if item.column() != 0:
            return
        ip_item = self.net_table.item(item.row(), 1)
        if ip_item is None:
            return
        ip, name = ip_item.text(), item.text().strip()
        aliases = self.cfg["ui"].setdefault("ip_aliases", {})
        if name and name != self._auto_alias(ip):
            aliases[ip] = name
            self.log("GUI", f"{ip} 이름 지정: {name}")
        else:
            aliases.pop(ip, None)
        save_config(self.cfg)

    def _on_net_state(self, text):
        self.lbl_net_state.setText(text)
        self.btn_grant.setVisible("setcap" in text)

    def _grant_net_probe(self):
        """pkexec(그래픽 인증)로 net_probe에 cap_net_raw를 주고 네트워크 모니터 재시작."""
        path = net_tools.find_net_probe()
        if not path:
            self.log("ERROR", "net_probe 바이너리를 찾을 수 없음 (colcon build 필요)")
            return
        if not shutil.which("pkexec"):
            self.log("ERROR", f"pkexec 없음 — 터미널에서: sudo setcap cap_net_raw+ep {path}")
            return
        self.btn_grant.setEnabled(False)
        proc = QProcess(self)
        proc.finished.connect(lambda code, _s: self._grant_done(code, path))
        proc.start("pkexec", ["setcap", "cap_net_raw+ep", path])
        self._grant_proc = proc

    def _grant_done(self, code, path):
        self.btn_grant.setEnabled(True)
        if code != 0:
            self.log("WARN", f"권한 부여 취소/실패 (exit {code}) — sudo setcap cap_net_raw+ep {path}")
            return
        self.log("OK", "net_probe 권한 부여 완료 — IP별 측정 시작")
        self.netmon.stop()
        nic = self.cfg["ui"].get("nic", "auto")
        self.netmon = NetMonitor(net_tools.auto_nic() if nic == "auto" else nic, self)
        self.netmon.sig_total.connect(self._on_net_total)
        self.netmon.sig_ips.connect(self._on_net_ips)
        self.netmon.sig_state.connect(self._on_net_state)
        self.netmon.sig_cams.connect(self._on_cams)
        self.netmon.start()

    def _on_cams(self, cams):
        self._cams = cams
        if cams:
            self.worker.resolve_camera_serials()

    def _on_serials(self, serials):
        self._serials = serials

    def _alias(self, ip):
        """IP → 표시 이름: 수동 별칭 > 자동 추정"""
        manual = (self.cfg["ui"].get("ip_aliases") or {}).get(ip)
        return manual if manual else self._auto_alias(ip)

    def _auto_alias(self, ip):
        """이 PC > 카메라(serial→노드 이름) > 모델명 > 플로우 힌트(포트로 추정) > ?"""
        if ip.startswith("127."):
            return "이 PC (localhost 루프백)"
        if ip in self._own_ips:
            return "이 PC"
        cam = self._cams.get(ip)
        if cam:
            name = self._serials.get(cam["serial"])
            if name:
                return f"{name} ({cam['serial']})"
            model = (cam["model"] or cam["vendor"] or "GigE").split(" ")[0]
            return f"{model} {cam['serial']}".strip()
        hint = net_tools.describe_flows(self._ip_flows.get(ip))
        return f"? {hint}" if hint else "?"

    # --- GNSS ---
    def _on_gnss(self, d):
        self._gnss[d["kind"]] = d
        if d["kind"] == "fix" and d["status"] >= 0 and \
                math.isfinite(d["lat"]) and math.isfinite(d["lon"]) and \
                not (abs(d["lat"]) < 1e-9 and abs(d["lon"]) < 1e-9):
            self._trail.append((d["t"], d["lat"], d["lon"]))
        # 그리기는 gnss_timer가 한다 — 여기서는 상태만 갱신

    def _refresh_gnss(self):
        g = self._gnss
        pre = float(self.cfg["recorder"]["pre_sec"])
        cut = time.time() - pre
        while self._trail and self._trail[0][0] < cut:
            self._trail.popleft()
        if self.gnss_win is not None and self.gnss_win.isVisible():
            self.gnss_win.update_live(g, list(self._trail), pre)
        pt = (g.get("pos_type") or {}).get("text")
        ns = (g.get("nav_status") or {}).get("text")
        fix = g.get("fix")
        gga = g.get("gga")
        age = (time.time() - fix["t"]) if fix else None
        live = fix is not None and age < 3.0 and fix["status"] >= 0
        parts = []
        if pt is not None:
            parts.append(f"pos_type={pt}")
        if ns is not None:
            parts.append(f"nav={ns}")
        if fix is None:
            parts.append("fix 토픽 수신 없음")
        elif age >= 3.0:
            parts.append(f"fix 끊김 {age:.0f}s")
        else:
            parts.append({-1: "NO_FIX", 0: "FIX", 1: "SBAS", 2: "GBAS"}.get(
                fix["status"], str(fix["status"])))
        self.lbl_gnss_status.setText("  ".join(parts) if parts else "수신 대기…")
        self.lbl_gnss_status.setStyleSheet(
            "color:#859900; font-weight:bold;" if live else "color:#dc322f;")
        if fix:
            self.lbl_gnss_pos.setText(
                f"{fix['lat']:.6f}, {fix['lon']:.6f}  alt {fix['alt']:.1f} m")
        q = []
        if fix and fix.get("hacc") is not None:
            q.append(f"수평정확도 {fix['hacc']:.2f} m")
        if gga:
            q.append(f"위성 {gga['sats']}  HDOP {gga['hdop']:.1f}")
        self.lbl_gnss_q.setText("  ".join(q) if q else "-")

    # --- 상태 표시 ---
    def _update_disk(self):
        """저장 경로가 있는 파일시스템의 실시간 용량. 2초마다 갱신."""
        d = Path(self.cfg["recorder"]["output_dir"])
        while not d.exists() and d != d.parent:   # 아직 안 만든 경로면 상위로
            d = d.parent
        try:
            du = shutil.disk_usage(d)
        except OSError:
            self.lbl_disk.setText("디스크 정보를 읽을 수 없음")
            return
        total_gb, free_gb = du.total / 1e9, du.free / 1e9
        used_gb = total_gb - free_gb
        self.bar_disk.setMaximum(max(1, int(total_gb)))
        self.bar_disk.setValue(min(int(used_gb), int(total_gb)))
        self.bar_disk.setFormat(f"사용 {used_gb:.0f} / {total_gb:.0f} GB")

        # 현재 유입 속도로 클립 하나가 차지할 용량 → 몇 개 더 저장 가능한지
        rec = self.cfg["recorder"]
        clip_gb = self._last_rate * (rec["pre_sec"] + rec["post_sec"]) / 1000.0
        est = ""
        if clip_gb > 0.01:
            est = (f" — 클립당 약 {clip_gb:.1f} GB, "
                   f"{int(free_gb / clip_gb)}개 저장 가능")
        self.lbl_disk.setText(f"여유 {free_gb:.1f} GB{est}")

        low = (clip_gb > 0.01 and free_gb < 2 * clip_gb) or free_gb < 5.0
        self.lbl_disk.setStyleSheet(
            "color:#dc322f; font-weight:bold;" if low else "color:#666;")
        if low and not self._disk_warned:
            self._disk_warned = True
            self.log("ERROR", f"디스크 여유 부족: {free_gb:.1f} GB — "
                              "클립 저장 실패 위험, 오래된 클립을 정리하세요")
        elif not low:
            self._disk_warned = False

    def _on_diag(self, kv):
        try:
            buf, cap = float(kv["buffer_mb"]), float(kv["cap_mb"])
            span, retain = float(kv["span_sec"]), float(kv["retain_sec"])
            rate = float(kv["rate_mb_s"])
            need = float(kv["needed_mb"])
            cap_hit = kv.get("cap_hit") == "true"
        except (KeyError, ValueError):
            return
        self._last_rate = rate
        self.bar_mem.setMaximum(max(1, int(cap)))
        self.bar_mem.setValue(min(int(buf), int(cap)))
        self.bar_span.setMaximum(max(1, int(retain * 10)))
        self.bar_span.setValue(min(int(span * 10), int(retain * 10)))
        self.bar_span.setFormat(f"{span:.1f} / {retain:.1f} s")
        self.lbl_inflow.setText(
            f"유입 {rate:.1f} MB/s — {retain:.0f}초 보관에 {need:.0f} MB 필요")
        if cap_hit and not self._cap_warned:
            self._cap_warned = True
            self.log("ERROR", "버퍼 상한 도달 — pre 구간이 잘리는 중! "
                              "max_buffer_mb를 키우거나 토픽을 줄이세요")
        elif not cap_hit:
            self._cap_warned = False

        # 토픽별: "<topic>"="X MB/s", "<topic>|hz", "<topic>|bps"(정수 bytes/s)
        per = {}
        for k, v in kv.items():
            if not k.startswith("/"):
                continue
            name, _, field = k.partition("|")
            d = per.setdefault(name, {"bps": None, "hz": None, "mb": 0.0})
            try:
                if field == "hz":
                    d["hz"] = float(v)
                elif field == "bps":
                    d["bps"] = float(v)
                elif v.endswith(" MB/s"):
                    d["mb"] = float(v[:-5])
            except ValueError:
                pass
        rows = sorted(per.items(),
                      key=lambda kv_: -(kv_[1]["bps"] if kv_[1]["bps"] is not None
                                        else kv_[1]["mb"] * 1048576))
        self.topic_table.setRowCount(len(rows))
        for row, (name, d) in enumerate(rows):
            bps = d["bps"] if d["bps"] is not None else d["mb"] * 1048576
            self.topic_table.setItem(row, 0, QTableWidgetItem(name))
            self.topic_table.setItem(
                row, 1, QTableWidgetItem("-" if d["hz"] is None else f"{d['hz']:.1f}"))
            self.topic_table.setItem(row, 2, QTableWidgetItem(fmt_bw(bps)))

    # --- 트리거 / 클립 이벤트 ---
    def _apply_shortcut(self):
        key = self.cfg["ui"]["shortcut"]
        if hasattr(self, "_shortcut"):
            self._shortcut.setKey(QKeySequence(key))
        else:
            self._shortcut = QShortcut(QKeySequence(key), self)
            self._shortcut.setContext(Qt.ApplicationShortcut)
            self._shortcut.activated.connect(self.trigger_clip)
        self.btn_trigger.setText(f"● 클립 저장  ({key})")

    def trigger_clip(self):
        if self.busy:
            self.log("WARN", "이미 클립 처리 중 — 트리거 무시")
            return
        if not self.worker.recorder_alive():
            self.log("ERROR", "레코더가 실행 중이 아님 — 트리거 불가")
            return
        label = self.label_edit.text().strip()
        self.worker.trigger(label)
        self.log("GUI", f"트리거 전송 (label='{label}')")

    def _on_clip_event(self, data):
        parts = data.split("|")
        kind = parts[0]
        if kind == "started":
            self.busy = True
            post = self.cfg["recorder"]["post_sec"]
            self.lbl_state.setText(
                f"클립 녹화 중… (post {post:.0f}초 + 디스크 기록 대기)")
            self.lbl_state.setStyleSheet("color:#b58900; font-weight:bold;")
            self.busy_timer.start(int((post + 300) * 1000))
        elif kind == "writing":
            self.lbl_state.setText(f"디스크 기록 중: {parts[1]}")
            self.log("GUI", f"디스크 기록 중… ({parts[1]})")
        elif kind == "done":
            self.busy = False
            self.busy_timer.stop()
            uri, nmsg, dur = parts[1], parts[2], parts[3]
            self.last_clip = uri
            self.lbl_state.setText("대기 중")
            self.lbl_state.setStyleSheet("")
            self.log("OK", f"클립 저장 완료: {uri} ({nmsg}개, {dur}s)")
            if self.cfg["ui"]["auto_diagnose"]:
                self.run_diagnostics(uri)
        elif kind == "busy":
            self.log("WARN", f"레코더 거부: {parts[1]}")
        elif kind == "error":
            self.busy = False
            self.busy_timer.stop()
            self.lbl_state.setText("대기 중")
            self.lbl_state.setStyleSheet("")
            self.log("ERROR", f"클립 저장 실패: {parts[1]}")
        self._update_alive()

    def _busy_timeout(self):
        if self.busy:
            self.busy = False
            self.log("ERROR", "클립 완료 이벤트가 오지 않음 — 상태 초기화 "
                              "(레코더 로그를 확인하세요)")
            self.lbl_state.setText("대기 중")
            self._update_alive()

    # --- 진단 ---
    def run_diagnostics(self, bag_dir):
        if self.diag_thread and self.diag_thread.isRunning():
            self.log("WARN", "이전 진단이 아직 실행 중")
            return
        p = Path(bag_dir)
        if not p.is_absolute():
            # 레코더의 상대 output_dir은 레코더 cwd 기준
            p = Path(self.cfg["recorder"]["output_dir"]).parent / bag_dir
        self.log("GUI", f"진단 시작: {p}")
        self.diag_thread = DiagRunner(p)
        self.diag_thread.sig_progress.connect(
            lambda s: self.lbl_state.setText(f"진단 중: {s}"))
        self.diag_thread.sig_done.connect(self._on_diag_done)
        self.diag_thread.sig_error.connect(
            lambda e: (self.log("ERROR", f"진단 실패: {e}"),
                       self.lbl_state.setText("대기 중")))
        self.diag_thread.start()

    def open_gnss(self):
        if self.gnss_win is None:
            self.gnss_win = GnssMapWindow(self.cfg, self)
        self.gnss_win.show()
        self.gnss_win.raise_()
        self._refresh_gnss()

    open_map = open_gnss

    def _add_to_map(self, bag_dir):
        """진단이 끝난 클립을 누적 지도에 추가 (창이 닫혀 있으면 설정에만 기록)."""
        if self.gnss_win is not None:
            self.gnss_win.add_clip(bag_dir, checked=True)
        else:
            lst = self.cfg["ui"].setdefault("map_clips", [])
            if str(bag_dir) not in lst:
                lst.append(str(bag_dir))
                save_config(self.cfg)

    def _on_diag_done(self, report):
        self.lbl_state.setText("대기 중")
        if any("track_m" in q for q in (report.get("gnss") or {}).values()):
            self._add_to_map(report["bag"])
            self.log("GUI", "GNSS 궤적을 누적 지도에 추가")
        lvl = report["level"]
        self.log({"OK": "OK", "WARN": "WARN", "FAIL": "ERROR"}[lvl],
                 f"진단 완료 [{lvl}] — FAIL {len(report['fail_topics'])}, "
                 f"WARN {len(report['warn_topics'])} "
                 f"(보고서: {report['bag']}/diagnostics.txt)")
        DiagDialog(report, self).show()

    def _diag_last(self):
        if self.last_clip:
            self.run_diagnostics(self.last_clip)
        else:
            self._diag_pick()

    def _diag_pick(self):
        d = QFileDialog.getExistingDirectory(
            self, "진단할 클립 폴더", self.cfg["recorder"]["output_dir"])
        if d:
            self.run_diagnostics(d)

    def _open_clip_dir(self):
        d = self.cfg["recorder"]["output_dir"]
        Path(d).mkdir(parents=True, exist_ok=True)
        subprocess.Popen(["xdg-open", d])

    # --- 설정/토픽 ---
    def open_topic_dialog(self):
        dlg = TopicDialog(self.worker, self.cfg, self)
        if dlg.exec_() != QDialog.Accepted:
            return
        self.cfg["topics"] = dlg.selected()
        save_config(self.cfg)
        self.log("GUI", f"토픽 {len(self.cfg['topics'])}개 선택됨")
        if self.worker.recorder_alive():
            self._apply_topics_runtime()

    def open_settings(self):
        dlg = SettingsDialog(self.cfg, self)
        if dlg.exec_() != QDialog.Accepted:
            return
        nic_before = self.cfg["ui"].get("nic", "auto")
        need_restart = dlg.apply_to(self.cfg)
        save_config(self.cfg)
        self._apply_shortcut()
        if self.cfg["ui"].get("nic", "auto") != nic_before:
            self.netmon.stop()
            nic = self.cfg["ui"]["nic"]
            self.netmon = NetMonitor(net_tools.auto_nic() if nic == "auto" else nic, self)
            self.netmon.sig_total.connect(self._on_net_total)
            self.netmon.sig_ips.connect(self._on_net_ips)
            self.netmon.sig_state.connect(self._on_net_state)
            self.netmon.sig_cams.connect(self._on_cams)
            self.netmon.start()
        if need_restart and self.worker.recorder_alive():
            self.log("GUI", "정적 설정 변경 — 레코더 재시작")
            self._restart_recorder()

    def _load_profile(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "프로파일 불러오기", str(CONFIG_DIR), "YAML (*.yaml *.yml)")
        if not path:
            return
        self.cfg = load_config(path)
        save_config(self.cfg)
        self._apply_shortcut()
        self.log("GUI", f"프로파일 적용: {path} "
                        f"(토픽 {len(self.cfg['topics'])}개)")
        if self.worker.recorder_alive():
            self._apply_topics_runtime()

    def _save_profile(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "프로파일 저장", str(CONFIG_DIR / "profile.yaml"),
            "YAML (*.yaml *.yml)")
        if path:
            save_config(self.cfg, path)
            self.log("GUI", f"프로파일 저장: {path}")

    # --- 종료 ---
    def closeEvent(self, ev):
        save_config(self.cfg)
        if self.diag_thread and self.diag_thread.isRunning():
            self.diag_thread.wait(1000)
        self.netmon.stop()
        if self.owns_recorder:
            self.recorder.stop_recorder()
        self.worker.stop()
        super().closeEvent(ev)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("DM Clip GUI")
    cfg = load_config()
    worker = RosWorker(cfg)
    worker.start()

    # 시작 시 토픽 선택 (이전 세션 선택이 미리 체크된 상태)
    dlg = TopicDialog(worker, cfg)
    if dlg.exec_() != QDialog.Accepted:
        worker.stop()
        return 0
    cfg["topics"] = dlg.selected()
    save_config(cfg)

    win = MainWindow(worker, cfg)
    win.show()
    rc = app.exec_()
    return rc


if __name__ == "__main__":
    sys.exit(main())
