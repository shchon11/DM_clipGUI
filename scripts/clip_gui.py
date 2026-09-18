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
import queue
import re
import shutil
import signal
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

import yaml

from PyQt5.QtCore import QByteArray, QObject, Qt, QThread, QTimer, QProcess, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QKeySequence, QPixmap, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox,
    QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QKeySequenceEdit, QLabel, QLineEdit,
    QListWidget, QListWidgetItem,
    QMainWindow, QMessageBox, QProgressBar, QPushButton, QScrollArea,
    QShortcut, QSizePolicy, QSpinBox, QSplitter, QTableWidget, QTableWidgetItem, QTabWidget,
    QToolButton,
    QTextEdit, QVBoxLayout, QWidget)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String

import bag_diagnostics
import gnss_tools
import net_tools
import preview_panel
import sensor_launcher
import sensor_stage
import ui_theme
from map_widget import MapWidget

CONFIG_DIR = Path.home() / ".config" / "dm_clip_gui"
LAST_SESSION = CONFIG_DIR / "last_session.yaml"
RECORDER_PARAMS = CONFIG_DIR / "recorder_params.yaml"

# GUI 목록에서 숨기는 토픽 (레코더 내부/ROS 인프라)
HIDDEN_TOPICS = ("/rosout", "/parameter_events",
                 "/clip_recorder/trigger", "/clip_recorder/clip_event", "/clip_recorder/record")

DEFAULT_CFG = {
    "topics": [],           # [{name, type, reliability, durability}]
    # 센서군별 설정 오버라이드 — sensor_stage/sensor_config 가 읽고 쓴다.
    #   {group_key: {subsets: {k: bool}, params: {subset: {키: 값}}, launch_args: {}}}
    "sensors": {},
    "recorder": {
        "pre_sec": 20.0, "post_sec": 10.0, "trigger_slack_sec": 2.0,
        "max_buffer_mb": 16384.0, "queue_depth": 50,
        "output_dir": str(Path.home() / "DM_clipGUI" / "clips"),
        "storage_id": "sqlite3", "status_period_sec": 2.0,
        # 수동 녹화: 시작부터 중지까지 bag 하나 (나누지 않는다). rosbag2 쓰기 캐시 크기
        "record_cache_mb": 256.0,
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
# 녹화 탭 로그는 센서 기동 탭처럼 어두운 배경(#0f172a)이라 밝은 톤을 쓴다. INFO 는 기본 글자색.
LOG_COLORS = {"INFO": None, "DEBUG": "#94a3b8", "WARN": "#fbbf24",
              "ERROR": "#f87171", "FATAL": "#f87171", "GUI": "#7dd3fc",
              "OK": "#4ade80"}


def load_config(path=LAST_SESSION):
    cfg = json.loads(json.dumps(DEFAULT_CFG))  # deep copy
    try:
        saved = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        for key in ("recorder", "ui"):
            cfg[key].update(saved.get(key) or {})
        # 한때 수동 녹화 파일을 60초마다 나누던 설정 — 이제 나누지 않는다 (녹화 하나 = bag 하나)
        cfg["recorder"].pop("record_split_sec", None)
        cfg["topics"] = saved.get("topics") or []
        cfg["sensors"] = saved.get("sensors") or {}
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
        # rclpy 는 기본으로 SIGINT/SIGTERM 에 ROS 컨텍스트를 내려 버린다. 그러면 Ctrl+C 뒤 종료 과정에서
        # 녹화 중지 명령(토픽)을 보낼 수 없다 — 시그널은 main() 의 처리기가 받아 순서대로 정리한다.
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
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
        self._record_pub = self.node.create_publisher(
            String, "/clip_recorder/record", 10)
        self._param_cli = self.node.create_client(
            SetParameters, "/clip_recorder/set_parameters")
        self._calls = queue.SimpleQueue()     # ROS 스레드에서 돌릴 작업 (구독 생성/해제)
        self._stop = False

    def call_in_ros_thread(self, fn):
        """구독을 만들고 내리는 일은 스핀하는 스레드에서 한다.

        GUI 스레드에서 하면 executor 가 대기 집합을 만드는 도중에 목록이 바뀌어 스핀 루프가
        예외로 죽을 수 있다 (그러면 GUI 가 ROS 를 통째로 잃는다). 최대 0.2초 뒤에 실행된다.
        """
        self._calls.put(fn)

    def _drain_calls(self):
        while True:
            try:
                fn = self._calls.get_nowait()
            except queue.Empty:
                return
            try:
                fn()
            except Exception as e:
                print(f"[clip_gui] ROS 스레드 작업 실패: {e}", file=sys.stderr)

    def run(self):
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)
        while not self._stop and rclpy.ok():
            self._drain_calls()
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

    def record(self, start, label=""):
        """수동 녹화 시작/중지 — 결과는 clip_event 의 rec_started / rec_stopped 로 온다."""
        text = ("start|" + label if label else "start") if start else "stop"
        self._record_pub.publish(String(data=text))

    def recorder_alive(self):
        try:
            return "clip_recorder" in \
                [n for n, _ in self.node.get_node_names_and_namespaces()]
        except Exception:
            return False

    def published_topics(self, exclude=None):
        """발행자가 실제로 있는 토픽 이름만. exclude={(토픽, 발행자 gid)} 는 없는 셈 친다.

        get_topic_names_and_types() 는 구독만 있는 토픽도 돌려준다. 센서를 한 번 띄웠다 내리면 레코더와
        미리보기가 그 토픽을 계속 구독하고 있어서, 다시 기동할 때 카메라가 뜨기도 전에 토픽이 '있다'고
        나와 곧바로 '실행 중'이 됐다 (GUI 를 껐다 켜면 구독이 사라져서 정상으로 돌아오던 증상).
        exclude 는 기동 직전에 이미 있던 발행자 — 강제 종료된 노드의 발행자는 DDS 임대 시간(~20초)
        동안 그래프에 남는다.
        """
        out = []
        for name, types in self.node.get_topic_names_and_types():
            if not types:
                continue
            if not exclude:
                if self.node.count_publishers(name) > 0:
                    out.append(name)
                continue
            infos = self.node.get_publishers_info_by_topic(name)
            if any((name, tuple(i.endpoint_gid)) not in exclude for i in infos):
                out.append(name)
        return out

    def publisher_gids(self):
        """{(토픽, 발행자 gid)} — 센서 기동 직전 스냅숏."""
        out = set()
        for name, types in self.node.get_topic_names_and_types():
            if types:
                for info in self.node.get_publishers_info_by_topic(name):
                    out.add((name, tuple(info.endpoint_gid)))
        return out

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

    def set_recorder_string(self, name, value):
        """레코더 문자열 파라미터 하나를 실행 중에 바꾼다 (output_dir 등). 결과는 sig_params."""
        if not self._param_cli.service_is_ready():
            self.sig_params.emit(False, "레코더 파라미터 서비스에 연결 안 됨 — 다음 레코더 시작부터 적용")
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=value))]

        def done(fut):
            try:
                bad = [r.reason for r in fut.result().results if not r.successful]
                self.sig_params.emit(not bad, "; ".join(bad) if bad else f"{name} = {value}")
            except Exception as e:
                self.sig_params.emit(False, str(e))

        self._param_cli.call_async(req).add_done_callback(done)

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
                   "status_period_sec", "record_cache_mb")}
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
        # setsid: 터미널 Ctrl+C 가 레코더에 바로 가지 않게 (센서 런치와 같은 이유 — sensor_launcher 참고).
        # 종료는 GUI 가 녹화 정리 → 센서 → 레코더 순으로 한다.
        self.start("setsid", ["ros2", "run", "clip_recorder", "clip_recorder",
                              "--ros-args", "-r", "__node:=clip_recorder",
                              "--params-file", str(RECORDER_PARAMS)])

    def stop_recorder(self, wait_ms=20000):
        """레코더를 내린다. 수동 녹화 중이면 레코더가 bag 을 닫고(캐시 flush) 내려갈 때까지 기다린다.

        `ros2 run` 은 받은 시그널을 레코더에 넘기지 않는다 — SIGTERM 을 받으면 자기만 죽고 레코더는
        고아로 남아 계속 돈다 (예전 terminate() 가 그랬다). 그래서 레코더 바이너리에 직접 SIGINT 를 보낸다.
        """
        if self.state() == QProcess.NotRunning:
            return
        pid = int(self.processId())
        family = sensor_launcher.descendants(pid) if pid > 0 else []
        for child in family:
            try:
                os.kill(child, signal.SIGINT)
            except OSError:
                pass
        if not sensor_launcher.wait_finished(self, wait_ms):
            self.terminate()
            if not sensor_launcher.wait_finished(self, 3000):
                self.kill()
                self.waitForFinished(2000)
        left = [c for c in family if sensor_launcher._alive(c)]
        if left:
            sensor_launcher.terminate(left, grace_s=0.5 if sensor_launcher.FORCE_STOP else 3.0)


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
        ui_theme.fit_to_screen(self, 1400, 860)
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
                self.map.add_track(f"clip:{path}", pts, color, 4.5, z=5)
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
            self.map.add_track("trail", draw_pts, "#ff6f00", width=5.5, z=15)
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
        ui_theme.fit_to_screen(self, 860, 560)

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
        self.auto_diag = QCheckBox("클립 녹화 완료 시 자동 진단")
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
                      "(레코더는 자동으로 재시작 — 외부에서 띄운 레코더는 제외).")
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
        ui_theme.fit_to_screen(self, 940, 620)
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
        m.add_track("clip", pts, "#2563eb", 4.5)
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
        self.quit_signal = None       # 터미널 신호로 끄는 중이면 그 이름 (SIGINT …) — 종료 대화상자를 건너뛴다
        self.last_clip = None
        self.diag_thread = None
        self._cap_warned = False
        self._disk_warned = False
        self._last_rate = 0.0     # 최근 유입 MB/s (디스크 여유 → 클립 수 환산용)
        # 링 버퍼 상태 — 레코더는 status_period_sec(2초)마다 알려주고, 그 사이는 GUI 가 시간으로 채워
        # 막대가 부드럽게 오른다. 보관 구간이 다 차야 클립 녹화를 받는다 (덜 찬 클립은 pre 가 잘린다).
        self._ring = {"t": 0.0, "span": 0.0, "retain": 0.0, "buf": 0.0, "cap": 1.0,
                      "rate": 0.0, "need": 0.0, "cap_hit": False, "have": False}
        self._ring_ready = False
        self._alive = False
        self._last_net_rates = None
        self.ring_timer = QTimer(self, interval=100, timeout=self._tick_ring)
        # 수동 녹화 상태 — 레코더의 /diagnostics(rec_*)가 기준이다 (GUI 를 다시 켜도 이어 보인다)
        self.recording = None     # {"uri", "t0", "sec", "mb", "closing"}
        self._rec_pending = False
        self.rec_timer = QTimer(self, interval=1000, timeout=self._tick_recording)
        self._cams = {}           # GVCP 디스커버리 {ip: info}
        self._own_ips = net_tools.own_ipv4s()
        self._ip_seen = {}        # ip -> 마지막으로 트래픽이 있었던 시각
        self._ip_flows = {}       # ip -> 플로우 힌트
        self._last_sensor_rescan = time.monotonic()   # 모르는 IP 때문에 센서를 다시 감지한 시각
        self._serials = {}        # {camera_serial: 드라이버 네임스페이스}
        self._gnss = {"pos_type": None, "nav_status": None, "fix": None, "gga": None}
        self.gnss_win = None       # GnssMapWindow (상태 + 지도 + 클립 궤적, 한 창)
        self._trail = deque()      # (t, lat, lon) — 최근 pre_sec 초의 유효 fix

        self.setWindowTitle("DM Clip GUI — 데이터 로깅")
        ui_theme.fit_to_screen(self, 1000, 900)
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

        # 센서군이 떠 있는 동안 도는 토픽 폴링 (기동 완료 판정 · 장비별 상태)
        self.sensor_timer = QTimer(self, interval=2000,
                                   timeout=self._poll_sensor_topics)
        self.sensor_timer.start()
        self._refresh_sensor_summary()

        self._apply_shortcut()
        QTimer.singleShot(300, self._startup_recorder)

    # --- UI 구성 ---
    def _build_ui(self):
        # 1탭 센서 기동 -> 2탭 녹화. 녹화 UI는 예전 그대로이고 탭 안으로만 들어갔다.
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.stage = sensor_stage.SensorStageWidget(self.cfg)
        self.stage.attach_ros(self.worker)      # 기동 중인 카메라 라이브 보기 (이름 짓기)
        self.stage.sig_log.connect(self.log)
        self.stage.sig_ready.connect(self._on_sensors_ready)
        self.stage.sig_go_record.connect(self._go_record)
        self.stage.sig_state.connect(self._refresh_sensor_summary)
        self.tabs.addTab(self.stage, "센서 기동")

        # 왼쪽: 레코더·버퍼·네트워크·GNSS·트리거·로그 / 오른쪽 전체: 센서 미리보기
        page = QWidget()
        self.tabs.addTab(page, "녹화")
        outer = QVBoxLayout(page)
        outer.setContentsMargins(10, 10, 10, 10)
        self.rec_hsplit = QSplitter(Qt.Horizontal)
        self.rec_hsplit.setChildrenCollapsible(False)
        outer.addWidget(self.rec_hsplit)
        left = QWidget()
        left.setMinimumWidth(380)
        self.rec_hsplit.addWidget(left)
        v = QVBoxLayout(left)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(8)

        # 센서 상태 한 줄 — 녹화 중에 센서가 죽으면 여기서 먼저 보인다
        self.lbl_sensors = QLabel("센서 정지")
        self.lbl_sensors.setStyleSheet("color:#666;")
        v.addWidget(self.lbl_sensors)

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
        self.bar_span.setRange(0, 1000)
        self.bar_span.setToolTip("지금 트리거하면 담길 과거 구간 (pre + 여유). 다 차야 클립 녹화를 받습니다 —\n"
                                 "덜 찬 채로 받으면 클립의 앞부분(pre)이 잘립니다.")
        self.lbl_inflow = QLabel("유입 - MB/s")
        # 수치는 막대 오른쪽에 — 줄 수를 줄여 아래 네트워크 표(센서 18대)에 자리를 준다
        self.bar_disk = QProgressBar()
        self.lbl_disk = QLabel("")
        self.lbl_disk.setStyleSheet("color:#666;")
        for w in (self.lbl_inflow, self.lbl_disk):
            w.setMinimumWidth(110)
        g.addWidget(QLabel("메모리"), 0, 0)
        g.addWidget(self.bar_mem, 0, 1)
        g.addWidget(self.lbl_inflow, 0, 2)
        g.addWidget(QLabel("보관 구간"), 1, 0)
        g.addWidget(self.bar_span, 1, 1, 1, 2)
        g.addWidget(QLabel("디스크"), 2, 0)
        g.addWidget(self.bar_disk, 2, 1)
        g.addWidget(self.lbl_disk, 2, 2)
        g.setColumnStretch(1, 1)

        # 아래 박스들은 전부 한 세로 분할 안에 있다 — 경계를 끌어서 크기를 바꾸고, 위치는 저장된다.
        self.rec_split = QSplitter(Qt.Vertical)
        self.rec_split.setChildrenCollapsible(False)
        self.rec_split.addWidget(grp_buf)

        # 네트워크 (스위치 업링크 + 장치별) / GNSS — 좌우도 끌어서 조절
        row = QSplitter(Qt.Horizontal)
        row.setChildrenCollapsible(False)
        self.net_split = row
        grp_net = QGroupBox("네트워크 — 스위치 업링크")
        gn = QGridLayout(grp_net)
        self.lbl_nic = QLabel("NIC: -")
        self.bar_nic = QProgressBar(format="- / - Gbps")
        gn.addWidget(self.lbl_nic, 0, 0)
        gn.addWidget(self.bar_nic, 0, 1)
        # 센서 18대 — 장치 이름이 제일 중요하다. Mbps·사용률은 한 칸으로, pps 는 툴팁으로.
        self.net_table = QTableWidget(0, 3)
        self.net_table.setHorizontalHeaderLabels(["장치", "IP", "대역폭"])
        self.net_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        for col in (1, 2):
            self.net_table.horizontalHeader().setSectionResizeMode(col, QHeaderView.ResizeToContents)
        self.net_table.verticalHeader().setVisible(False)
        self.net_table.setEditTriggers(QTableWidget.DoubleClicked)   # 장치 이름 직접 지정
        self.net_table.itemChanged.connect(self._on_net_alias_edited)
        self.net_table.setToolTip("장치 칸을 더블클릭하면 이름을 직접 지정할 수 있습니다 (설정에 저장)\n"
                                  "열 제목을 누르면 정렬 (한 번 더 누르면 반대로)")
        # 열 제목 클릭 = 그 열로 오름차순 ▲, 한 번 더 = 내림차순 ▼ (센서 기동 탭의 장비 표와 같게).
        # 표는 매초 새로 채우므로 Qt 정렬 대신 채울 때 정렬한다. ui.net_sort 에 저장.
        self.net_table.horizontalHeader().setSectionsClickable(True)
        self.net_table.horizontalHeader().sectionClicked.connect(self._on_net_sort)
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
        row.addWidget(grp_net)

        # GNSS: 상태 한 줄 + 위치/품질 한 줄 + 미니 지도 (현재 위치 · 지금 트리거하면 담길 pre 궤적)
        grp_gnss = QGroupBox("GNSS")
        gg = QVBoxLayout(grp_gnss)
        gg.setSpacing(4)
        head = QHBoxLayout()
        self.lbl_gnss_status = QLabel("수신 대기…")
        self.lbl_gnss_status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        btn_g = QToolButton()
        btn_g.setText("크게 ↗")
        btn_g.setToolTip("GNSS · 지도 창 (상태 전체 + 클립 궤적)")
        btn_g.clicked.connect(self.open_gnss)
        head.addWidget(self.lbl_gnss_status, 1)
        head.addWidget(btn_g)
        gg.addLayout(head)
        self.lbl_gnss_pos = QLabel("-")
        self.lbl_gnss_pos.setStyleSheet("color:#6b7280;")
        self.lbl_gnss_pos.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        gg.addWidget(self.lbl_gnss_pos)
        self.minimap = MapWidget()
        self.minimap.setMinimumSize(160, 110)
        last = self.cfg["ui"].get("last_fix")
        if last:
            self.minimap.set_view(last[0], last[1], 15)
        self._minimap_zoomed = False     # 첫 fix 에서 한 번만 확대하고, 그 뒤로는 따라가기만
        # [GNSS · 지도] 창에서 체크한 클립 경로를 미니 지도에도 (고르기는 그 창에서)
        self._mini_clips = None          # 지금 그려 둔 클립 목록
        self._mini_thread = None
        gg.addWidget(self.minimap, 1)
        row.addWidget(grp_gnss)
        # 장치 이름 칸이 넓어야 한다 (센서 18대). 오른쪽은 GNSS 미니 지도.
        row.setStretchFactor(0, 3)
        row.setStretchFactor(1, 2)
        row.setSizes([440, 280])
        self.rec_split.addWidget(row)

        # 녹화: 클립(트리거 앞뒤 창) + 수동 녹화(시작~중지 전부)
        grp_trig = QGroupBox("녹화")
        h = QHBoxLayout(grp_trig)
        # 저장 위치 — 누르면 폴더 선택. 설정에 바로 저장돼 GUI 를 다시 켜도 유지되고,
        # 떠 있는 레코더에도 바로 알려 다음 클립·녹화부터 새 위치에 쓴다.
        self.btn_outdir = QPushButton()
        self.btn_outdir.setMinimumHeight(56)
        self.btn_outdir.setMaximumWidth(260)
        self.btn_outdir.clicked.connect(self.pick_output_dir)
        h.addWidget(self.btn_outdir)
        self._show_output_dir()
        h.addWidget(QLabel("라벨:"))
        self.label_edit = QLineEdit(
            placeholderText="클립·녹화 폴더명에 붙일 라벨 (선택)")
        h.addWidget(self.label_edit, 1)
        self.btn_trigger = QPushButton()
        self.btn_trigger.setMinimumHeight(56)
        f = self.btn_trigger.font()
        f.setPointSize(13)
        f.setBold(True)
        self.btn_trigger.setFont(f)
        self.btn_trigger.setStyleSheet(
            "QPushButton {background:#dc2626; color:white; border:none; border-radius:8px;}"
            "QPushButton:hover {background:#b91c1c;}"
            "QPushButton:disabled {background:#d1d5db; color:#f9fafb;}")
        self.btn_trigger.clicked.connect(self.trigger_clip)
        h.addWidget(self.btn_trigger, 1)
        self.btn_record = QPushButton()
        self.btn_record.setMinimumHeight(56)
        self.btn_record.setFont(f)
        self.btn_record.setToolTip(
            "수동 녹화: 누른 때부터 다시 누를 때까지 선택한 토픽을 전부 bag 하나로 씁니다 "
            "(rec_<시각>[_라벨], 중간에 나누지 않음).\n클립과 같은 구독을 쓰므로 센서 쪽 부하는 "
            "늘지 않고, 녹화 중에도 클립 녹화가 됩니다.\n"
            "디스크 여유가 1 GB 밑으로 내려가면 자동으로 멈춥니다.")
        self.btn_record.clicked.connect(self.toggle_recording)
        h.addWidget(self.btn_record, 1)

        # 현재 상태 줄: 상태마다 바탕색이 바뀌는 띠 (ui_theme 의 QLabel#StateBar[kind=...])
        self.lbl_state = QLabel()
        self.lbl_state.setObjectName("StateBar")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self._set_state("대기 중")
        self.lbl_rec = QLabel("")
        self.lbl_rec.setObjectName("StateBar")
        self.lbl_rec.setProperty("kind", "rec")
        self.lbl_rec.setAlignment(Qt.AlignCenter)
        self.lbl_rec.hide()
        self._style_record_button()
        trig_box = QWidget()
        tv = QVBoxLayout(trig_box)
        tv.setContentsMargins(0, 0, 0, 0)
        tv.addWidget(grp_trig)
        tv.addWidget(self.lbl_rec)
        tv.addWidget(self.lbl_state)
        trig_box.setMaximumHeight(trig_box.sizeHint().height() + 40)
        self.rec_split.addWidget(trig_box)

        # (예전의 토픽별 Hz/대역폭 표는 없앴다 — 대역폭은 네트워크 표가 장치별로, 수신 Hz 는
        #  오른쪽 미리보기가 센서별로 보여준다.)
        self.log_view = QTextEdit(readOnly=True)
        self.log_view.setObjectName("Log")             # 센서 기동 탭 런치 로그와 같은 어두운 배경
        self.log_view.setFont(QFont("Monospace", 9))
        self.log_view.document().setMaximumBlockCount(3000)
        self.rec_split.addWidget(self.log_view)
        self.rec_split.setSizes([110, 300, 90, 160])
        v.addWidget(self.rec_split, 1)

        self.preview = preview_panel.PreviewPanel(self.worker, self.cfg, self.stage)
        self.preview.setMinimumWidth(280)
        self.rec_hsplit.addWidget(self.preview)
        self.rec_hsplit.setStretchFactor(0, 3)
        self.rec_hsplit.setStretchFactor(1, 2)
        self.rec_hsplit.setSizes([620, 460])

        # 칸 크기 복원 + 끌 때마다 저장 ("record/v" — 토픽 표가 빠져 칸 수가 바뀌어 새 키)
        states = self.cfg["ui"].setdefault("splitters", {})
        for name, splitter in (("record/v", self.rec_split), ("record/net", self.net_split),
                               ("record/h", self.rec_hsplit)):
            if states.get(name):
                splitter.restoreState(QByteArray.fromBase64(states[name].encode()))
            splitter.splitterMoved.connect(
                lambda *_, n=name, sp=splitter:
                    states.__setitem__(n, bytes(sp.saveState().toBase64()).decode()))

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
        m_tool.addAction("센서 다시 감지", lambda: self.stage.refresh_discovery())
        m_tool.addSeparator()
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
    # --- 센서 기동 탭 연동 ---

    def _poll_sensor_topics(self):
        """기동 완료 판정 · 장비별 상태(토픽이 사라짐)에 쓸 토픽 목록을 supervisor 에 넘긴다.

        센서군이 떠 있는 동안만 돈다 — 다 뜬 뒤에도 도는 이유는, 도중에 카메라 노드 하나가
        죽거나 토픽이 사라지면 감지된 장비 표의 그 행을 빨갛게 바꾸려는 것이다.
        """
        if not self.stage.supervisor.any_active():
            return
        try:
            self.stage.set_topics(self.worker.published_topics(self.stage.stale_publishers()))
        except Exception:
            pass

    def _on_sensors_ready(self):
        """기동한 센서군이 전부 올라왔다 — 녹화 탭으로 넘기고 토픽을 다시 고르게 한다."""
        self.log("OK", "센서 기동 완료 — 녹화할 토픽을 선택하세요")
        self._refresh_sensor_summary()
        self._go_record()

    def _go_record(self):
        self.tabs.setCurrentIndex(1)
        QTimer.singleShot(300, self.open_topic_dialog)

    def _refresh_sensor_summary(self):
        text = self.stage.summary()
        running = "실행" in text
        self.lbl_sensors.setText(text)
        self.lbl_sensors.setStyleSheet(
            "color:#2e7d32;" if running else "color:#666;")

    def _startup_recorder(self):
        if self.worker.recorder_alive():
            self.owns_recorder = False
            if not self.cfg["topics"]:
                # 토픽을 아직 안 골랐으면 밀어넣지 않는다. 빈 목록을 보내면 레코더는 그걸
                # "전체 토픽 녹화"로 받아들여, 이미 돌고 있던 레코더의 선택을 덮어쓴다.
                self.log("GUI", "외부 clip_recorder 노드에 연결 — 토픽 선택 전이라 "
                                "레코더의 현재 토픽 설정은 그대로 둡니다")
            else:
                self.log("GUI", "외부 clip_recorder 노드에 연결 — 토픽 설정을 적용합니다")
                self._apply_topics_runtime()
        elif not self.cfg["topics"]:
            # 예전에는 시작할 때 토픽 선택 창이 먼저 떠서 선택이 보장됐다. 이제는
            # 센서를 먼저 띄우는 흐름이라, 선택 전에 자동 시작하면 전체 토픽을 녹화한다.
            self.log("GUI", "녹화할 토픽이 아직 없습니다 — 센서 기동 후 토픽을 선택하세요")
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
                if not self._confirm_end_recording("레코더를 멈추면"):
                    return
                self._stop_recording_and_wait()
                self.recorder.stop_recorder()
                self.log("GUI", "레코더 정지")
            else:
                self.log("WARN", "외부에서 띄운 레코더는 GUI에서 정지하지 않습니다")
        else:
            self._start_recorder()

    def _restart_recorder(self):
        if self.owns_recorder:
            if self.recording:
                self.log("WARN", "수동 녹화 중이라 레코더를 재시작하지 않습니다 — 바꾼 설정은 녹화를 "
                                 "멈춘 뒤 [레코더 정지]→[레코더 시작] 하면 적용됩니다")
                return
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
        self._alive = alive
        if not alive:
            self._ring["have"] = False           # 레코더가 없으면 버퍼도 없다 — 다시 뜨면 처음부터 찬다
        self._set_ring_ready(self._ring_ready and alive)
        self.btn_record.setEnabled(alive and not self._rec_pending and
                                   not (self.recording or {}).get("closing"))
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
        self._last_net_rates = rates
        link = float(self.cfg["ui"].get("per_ip_link_mbps", 1000))
        now = time.monotonic()
        for ip, (mbps, pps, flows) in rates.items():
            if flows:
                self._ip_flows[ip] = flows
            if pps > 0:
                self._ip_seen[ip] = now
            self._ip_seen.setdefault(ip, now)
        # 5분 넘게 조용한 IP는 목록에서 뺀다 (net_probe는 한 번 본 IP를 계속 보고함)
        rows = [(ip, v) for ip, v in rates.items() if now - self._ip_seen.get(ip, now) < 300]
        col, descending = self._net_sort_spec()
        rows.sort(key=lambda kv: self._net_sort_key(col, kv), reverse=descending)
        labels = ["장치", "IP", "대역폭"]
        labels[col] += "  ▼" if descending else "  ▲"
        self.net_table.setHorizontalHeaderLabels(labels)
        self.net_table.blockSignals(True)        # 채우는 동안 itemChanged 무시
        self.net_table.setRowCount(len(rows))
        for r, (ip, (mbps, pps, _)) in enumerate(rows):
            pct = mbps / link * 100
            idle = now - self._ip_seen.get(ip, now)
            rate = f"{mbps:.0f}" if mbps >= 10 else f"{mbps:.2f}"
            cells = [self._alias(ip), ip,
                     f"{rate} Mbps · {pct:.0f}%" if idle < 10 else f"유휴 {idle:.0f}s"]
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
            sensor = self.stage.device_at(ip)
            detail = (f"\n{sensor['subset_label']} · {sensor['model'] or '-'} · "
                      f"시리얼 {sensor['identity'] or '-'} · {sensor['nic'] or '-'}") if sensor else ""
            self.net_table.item(r, 0).setToolTip(
                f"{ip}{detail}\n{rate} Mbps · 포트({link:.0f}M) 사용률 {pct:.0f}% · {pps:.0f} pps"
                f"\n힌트: {net_tools.describe_flows(self._ip_flows.get(ip)) or '-'}"
                "\n더블클릭해서 이름 지정")
        self.net_table.blockSignals(False)

        # 정체 모를 장치가 트래픽을 내고 있으면 (센서 탭 감지 뒤에 켜진 라이다 등) 다시 감지한다.
        unknown = any(self._alias(ip).startswith("?") for ip, _ in rows)
        if unknown and now - self._last_sensor_rescan > 60:
            self._last_sensor_rescan = now
            self.stage.refresh_discovery()

    NET_COLS = ("name", "ip", "bw")

    def _net_sort_spec(self):
        """(열 번호, 내림차순?) — 기본은 대역폭 내림차순 (예전 동작)."""
        key, _, order = str(self.cfg["ui"].get("net_sort") or "bw:desc").partition(":")
        col = self.NET_COLS.index(key) if key in self.NET_COLS else 2
        return col, order == "desc"

    def _on_net_sort(self, index):
        if not 0 <= index < len(self.NET_COLS):
            return
        col, descending = self._net_sort_spec()
        descending = (not descending) if col == index else False
        self.cfg["ui"]["net_sort"] = f"{self.NET_COLS[index]}:{'desc' if descending else 'asc'}"
        save_config(self.cfg)
        if self._last_net_rates is not None:
            self._on_net_ips(self._last_net_rates)

    def _net_sort_key(self, col, kv):
        ip, (mbps, _pps, _flows) = kv
        try:
            ip_key = tuple(int(x) for x in ip.split("."))
        except ValueError:
            ip_key = (999,)
        if col == 0:
            name = self._alias(ip)
            return ([int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", name)], ip_key)
        if col == 1:
            return ip_key
        return (mbps, ip_key)

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
        """이 PC > 센서 기동 탭이 감지한 센서 > GigE 카메라(serial→노드 이름) > 모델명
        > 플로우 힌트(포트로 추정) > ?

        라이다·GNSS 는 GigE 디스커버리에 안 잡혀서 예전에는 포트로만 추정했다
        ("? Ouster LiDAR 데이터"). 라이다 데이터 패킷은 MTU 보다 커서 IP 단편화되고, 뒤쪽 단편에는
        UDP 헤더가 없어 "? UDP" 로도 보였다. 센서 탭이 IP 로 이미 알고 있으니 그걸 먼저 쓴다.
        """
        if ip.startswith("127."):
            return "이 PC (localhost 루프백)"
        if ip in self._own_ips:
            return "이 PC"
        sensor = self.stage.device_at(ip)
        if sensor:
            return self.stage.describe_device(sensor, self._serials.get(sensor.get("identity")))
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
            "color:#16a34a; font-weight:bold;" if live else "color:#dc2626; font-weight:bold;")
        line = []
        if fix:
            line.append(f"{fix['lat']:.6f}, {fix['lon']:.6f} · {fix['alt']:.1f} m")
        if fix and fix.get("hacc") is not None:
            line.append(f"±{fix['hacc']:.2f} m")
        if gga:
            line.append(f"위성 {gga['sats']} · HDOP {gga['hdop']:.1f}")
        self.lbl_gnss_pos.setText("  ·  ".join(line) if line else "위치 없음")
        self._update_minimap(fix, live, pre)

    def _update_minimap_clips(self):
        """[GNSS · 지도] 창에서 체크한 클립(ui.map_clips)의 경로를 미니 지도에 — 목록이 바뀔 때만 읽는다."""
        sel = list(self.cfg["ui"].get("map_clips") or [])
        if sel == self._mini_clips or (self._mini_thread and self._mini_thread.isRunning()):
            return
        self._mini_clips = sel
        clips = [(Path(p).name.replace("clip_", "").replace("rec_", "녹화 "), p) for p in sel if Path(p).is_dir()]
        if not clips:
            self._draw_minimap_clips([])
            return
        self._mini_thread = MapLoadThread(clips)
        self._mini_thread.sig_done.connect(self._draw_minimap_clips)
        self._mini_thread.start()

    def _draw_minimap_clips(self, tracks):
        for oid in [o for o in list(self.minimap._overlays) if o.startswith(("clip:", "s:", "e:"))]:
            self.minimap.remove(oid)
        allpts = []
        for i, (_lbl, path, fixes) in enumerate(tracks):
            if len(fixes) < 2:
                continue
            pts = [(f["lat"], f["lon"]) for f in fixes]
            step = max(1, len(pts) // 300)                  # 작은 지도 — 점을 솎는다
            pts = pts[::step] + ([pts[-1]] if (len(pts) - 1) % step else [])
            color = gnss_tools.TRACK_COLORS[i % len(gnss_tools.TRACK_COLORS)]
            self.minimap.add_track(f"clip:{path}", pts, color, 3.5, z=5)
            self.minimap.set_marker(f"s:{path}", *pts[0], color=color, radius=3, z=6)
            self.minimap.set_marker(f"e:{path}", *pts[-1], color=color, radius=4, z=6)
            allpts += pts
        # 아직 실시간 위치로 확대하지 않았으면 클립들이 다 보이게
        if allpts and not self._minimap_zoomed:
            self.minimap.fit_bounds(allpts)

    def _update_minimap(self, fix, live, pre):
        """현재 위치(파랑) + 지금 트리거하면 담길 pre 구간 궤적(주황) + [GNSS · 지도] 창에서 체크한 클립 경로.
        녹화 탭이 보일 때만 그린다."""
        if not self.minimap.isVisible():
            return
        self._update_minimap_clips()
        pts = [(la, lo) for _, la, lo in self._trail]
        step = max(1, len(pts) // 200)
        draw = pts[::step]
        if len(pts) > 1 and (len(pts) - 1) % step:
            draw.append(pts[-1])
        if len(draw) >= 2:
            self.minimap.add_track("trail", draw, "#ff6f00", width=4.5, z=15)
        else:
            self.minimap.remove("trail")
        if fix and fix["status"] >= 0 and not (abs(fix["lat"]) < 1e-9 and abs(fix["lon"]) < 1e-9):
            color = "#2962ff" if live else "#9ca3af"          # 끊기면 회색 = 마지막 위치
            self.minimap.set_marker("me", fix["lat"], fix["lon"], color=color, radius=6)
            if live:
                self.cfg["ui"]["last_fix"] = [fix["lat"], fix["lon"]]   # 다음 실행 때 여기서 시작
                self.minimap.set_view(fix["lat"], fix["lon"], None if self._minimap_zoomed else 16)
                self._minimap_zoomed = True

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
        if self.recording and self._last_rate > 0.1:
            est = f" — 지금 녹화 속도로 약 {free_gb * 1000 / self._last_rate / 60:.0f}분 더"
        self.lbl_disk.setText(f"여유 {free_gb:.1f} GB{est}")
        if self.recording and free_gb < 1.0 and not self.recording.get("closing"):
            self.log("ERROR", f"디스크 여유 {free_gb:.2f} GB — 수동 녹화를 자동으로 멈춥니다")
            self._request_stop_recording()

        low = (clip_gb > 0.01 and free_gb < 2 * clip_gb) or free_gb < 5.0
        self.lbl_disk.setStyleSheet(
            "color:#dc322f; font-weight:bold;" if low else "color:#666;")
        if low and not self._disk_warned:
            self._disk_warned = True
            self.log("ERROR", f"디스크 여유 부족: {free_gb:.1f} GB — "
                              "녹화 실패 위험, 오래된 클립·녹화를 정리하세요")
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
        self._sync_recording(kv)
        self._ring.update(t=time.monotonic(), span=span, retain=retain, buf=buf, cap=cap,
                          rate=rate, need=need, cap_hit=cap_hit, have=True)
        if not self.ring_timer.isActive():
            self.ring_timer.start()
        self._tick_ring()
        self.lbl_inflow.setText(
            f"유입 {rate:.1f} MB/s — {retain:.0f}초 보관에 {need:.0f} MB 필요")
        if cap_hit and not self._cap_warned:
            self._cap_warned = True
            self.log("ERROR", "버퍼 상한 도달 — pre 구간이 잘리는 중! "
                              "max_buffer_mb를 키우거나 토픽을 줄이세요")
        elif not cap_hit:
            self._cap_warned = False

    def _tick_ring(self):
        """링 버퍼 막대 (0.1초마다). 보고 사이에는 보관 구간이 실시간으로 찬다고 보고 채운다."""
        ring = self._ring
        if not ring["have"]:
            self.bar_span.setValue(0)
            self.bar_span.setFormat("버퍼링 준비 중…")
            self._set_ring_ready(False)
            return
        dt = min(time.monotonic() - ring["t"], 5.0)
        retain = max(ring["retain"], 0.1)
        span = ring["span"] if ring["span"] >= retain else min(retain, ring["span"] + dt)
        buf = ring["buf"]
        if ring["span"] < retain and ring["rate"] > 0:
            buf = min(ring["cap"], max(ring["buf"], ring["need"]), ring["buf"] + ring["rate"] * dt)
        self.bar_mem.setMaximum(max(1, int(ring["cap"])))
        self.bar_mem.setValue(min(int(buf), int(ring["cap"])))
        full = span >= retain - 0.3
        self.bar_span.setValue(int(min(span / retain, 1.0) * 1000))
        if ring["cap_hit"]:
            color, text = ui_theme.ERR, f"{span:.1f} / {retain:.1f} s — 메모리 상한에 걸려 더 못 채움"
        elif full:
            color, text = ui_theme.OK, f"{span:.1f} / {retain:.1f} s — 클립 녹화 가능"
        else:
            color, text = ui_theme.WARN, f"{span:.1f} / {retain:.1f} s — 채우는 중 ({retain - span:.0f}초 남음)"
        self.bar_span.setFormat(text)
        if self.bar_span.property("ringColor") != color:
            self.bar_span.setProperty("ringColor", color)
            self.bar_span.setStyleSheet(f"QProgressBar::chunk {{ background: {color}; border-radius: 4px; }}")
        # 메모리 상한에 걸리면 영영 안 차므로 (경고는 따로 뜬다) 그 상태로 받는다
        self._set_ring_ready(full or ring["cap_hit"], retain - span)

    def _set_ring_ready(self, ready, remaining=0.0):
        self._ring_ready = ready
        self.btn_trigger.setEnabled(self._alive and not self.busy and ready)
        rec = self.cfg["recorder"]
        key = self.cfg["ui"]["shortcut"]
        length = f"{rec['pre_sec'] + rec['post_sec']:g}"
        if ready or not self._alive:
            text = f"● 클립({length}초) 녹화  ({key})"
        else:
            text = f"● 클립({length}초) 녹화 — 준비 중 {max(remaining, 0):.0f}초"
        if self.btn_trigger.text() != text:
            self.btn_trigger.setText(text)

    # --- 트리거 / 클립 이벤트 ---
    def _apply_shortcut(self):
        key = self.cfg["ui"]["shortcut"]
        if hasattr(self, "_shortcut"):
            self._shortcut.setKey(QKeySequence(key))
        else:
            self._shortcut = QShortcut(QKeySequence(key), self)
            self._shortcut.setContext(Qt.ApplicationShortcut)
            self._shortcut.activated.connect(self.trigger_clip)
        rec = self.cfg["recorder"]
        self.btn_trigger.setToolTip(f"사건 순간 앞 {rec['pre_sec']:g}초 + 뒤 {rec['post_sec']:g}초를 "
                                    "clip_<시각>[_라벨] 로 씁니다 (길이는 설정 창의 pre/post).\n"
                                    "링 버퍼의 보관 구간이 다 차야 눌립니다.")
        self._set_ring_ready(self._ring_ready)

    def trigger_clip(self):
        if self.busy:
            self.log("WARN", "이미 클립 처리 중 — 트리거 무시")
            return
        if not self.worker.recorder_alive():
            self.log("ERROR", "레코더가 실행 중이 아님 — 트리거 불가")
            return
        if not self._ring_ready:
            ring = self._ring
            self.log("WARN", f"보관 구간이 아직 {ring['span']:.0f}/{ring['retain']:.0f}초 — 다 차면 클립 녹화를 "
                             "받습니다 (지금 받으면 앞부분이 잘림)")
            return
        label = self.label_edit.text().strip()
        self.worker.trigger(label)
        self.log("GUI", f"트리거 전송 (label='{label}')")

    def _on_clip_event(self, data):
        parts = data.split("|")
        kind = parts[0]
        if kind.startswith("rec_"):
            self._on_record_event(kind, parts)
            self._update_alive()
            return
        if kind == "started":
            self.busy = True
            post = self.cfg["recorder"]["post_sec"]
            self._set_state(f"클립 녹화 중… (post {post:.0f}초 + 디스크 기록 대기)", "clip")
            self.busy_timer.start(int((post + 300) * 1000))
        elif kind == "writing":
            self._set_state(f"디스크 기록 중: {parts[1]}", "write")
            self.log("GUI", f"디스크 기록 중… ({parts[1]})")
        elif kind == "done":
            self.busy = False
            self.busy_timer.stop()
            uri, nmsg, dur = parts[1], parts[2], parts[3]
            self.last_clip = uri
            self._set_state("대기 중")
            self.log("OK", f"클립 녹화 완료: {uri} ({nmsg}개, {dur}s)")
            if self.cfg["ui"]["auto_diagnose"]:
                self.run_diagnostics(uri)
        elif kind == "busy":
            self.log("WARN", f"레코더 거부: {parts[1]}")
        elif kind == "error":
            self.busy = False
            self.busy_timer.stop()
            self._set_state("대기 중")
            self.log("ERROR", f"클립 녹화 실패: {parts[1]}")
        self._update_alive()

    # --- 저장 위치 ---
    def _show_output_dir(self):
        path = Path(self.cfg["recorder"]["output_dir"]).expanduser()
        try:
            short = "~/" + str(path.relative_to(Path.home()))
        except ValueError:
            short = str(path)
        if len(short) > 28:
            short = "…" + short[-27:]
        self.btn_outdir.setText(f"📁 {short}")
        self.btn_outdir.setToolTip(f"저장 위치: {path}\n누르면 바꿉니다 — 다음 클립·녹화부터 새 위치에 씁니다 "
                                   "(설정에 저장, GUI 를 다시 켜도 유지)")

    def pick_output_dir(self):
        current = str(Path(self.cfg["recorder"]["output_dir"]).expanduser())
        chosen = QFileDialog.getExistingDirectory(self, "저장 위치", current)
        if not chosen or chosen == current:
            return
        try:
            Path(chosen).mkdir(parents=True, exist_ok=True)
        except OSError as e:
            self.log("ERROR", f"저장 위치를 만들 수 없음: {e}")
            return
        self.cfg["recorder"]["output_dir"] = chosen
        save_config(self.cfg)
        self._show_output_dir()
        self._update_disk()
        if self.worker.recorder_alive():
            self.worker.set_recorder_string("output_dir", chosen)
        note = " (진행 중인 수동 녹화는 원래 위치에 계속 씁니다)" if self.recording else ""
        self.log("GUI", f"저장 위치: {chosen}{note}")

    # --- 수동 녹화 ---
    def _style_record_button(self):
        if self.recording:
            self.btn_record.setStyleSheet(
                "QPushButton {background:#ffffff; color:#b91c1c; border:2px solid #dc2626;"
                " border-radius:8px;} QPushButton:hover {background:#fef2f2;}"
                "QPushButton:disabled {color:#9ca3af; border-color:#d1d5db;}")
        else:
            self.btn_record.setStyleSheet(
                "QPushButton {background:#1f2937; color:white; border:none; border-radius:8px;}"
                "QPushButton:hover {background:#111827;}"
                "QPushButton:disabled {background:#d1d5db; color:#f9fafb;}")
        self._tick_recording()

    def toggle_recording(self):
        if self.recording:
            self._request_stop_recording()
            return
        if not self.worker.recorder_alive():
            self.log("ERROR", "레코더가 실행 중이 아님 — 녹화 불가")
            return
        if not self.cfg["topics"]:
            self.log("WARN", "녹화할 토픽을 고르지 않아 레코더가 보는 토픽 전부를 녹화합니다")
        label = self.label_edit.text().strip()
        self._rec_pending = True
        self.btn_record.setEnabled(False)
        self.btn_record.setText("시작 중…")
        self.worker.record(True, label)
        self.log("GUI", f"수동 녹화 시작 요청 (label='{label}')")
        QTimer.singleShot(5000, self._record_request_timeout)

    def _record_request_timeout(self):
        if self._rec_pending:
            self._rec_pending = False
            self.log("ERROR", "레코더가 수동 녹화 명령에 답이 없습니다 — 레코더가 이 기능을 모르는 옛 "
                              "빌드일 수 있습니다 (레코더를 재시작하세요)")
            self._style_record_button()
            self._update_alive()

    def _request_stop_recording(self):
        if not self.recording or self.recording.get("closing"):
            return
        self.recording["closing"] = True
        self.worker.record(False)
        self.btn_record.setEnabled(False)
        self.btn_record.setText("파일 닫는 중…")
        self.log("GUI", "수동 녹화 중지 요청")

    def _stop_recording_and_wait(self, timeout=30.0):
        """녹화를 멈추고 파일이 닫힐 때까지 기다린다 (레코더 정지 · GUI 종료 전)."""
        if not self.recording:
            return
        self._request_stop_recording()
        deadline = time.time() + timeout
        while self.recording and time.time() < deadline and not sensor_launcher.FORCE_STOP:
            QApplication.processEvents()
            time.sleep(0.05)

    def _confirm_end_recording(self, what):
        if not self.recording:
            return True
        answer = QMessageBox.question(
            self, "수동 녹화 중",
            f"수동 녹화 중입니다 ({Path(self.recording['uri']).name}).\n"
            f"{what} 녹화를 끝내고 파일을 닫습니다. 계속할까요?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        return answer == QMessageBox.Yes

    def _on_record_event(self, kind, parts):
        if kind == "rec_started":
            self._rec_pending = False
            self.recording = {"uri": parts[1], "t0": time.time(), "sec": 0.0, "mb": 0.0}
            self.rec_timer.start()
            self.log("OK", f"수동 녹화 시작: {parts[1]}")
        elif kind == "rec_closing":
            if self.recording:
                self.recording["closing"] = True
        elif kind == "rec_stopped":
            uri, nmsg, dur, mb = (parts + ["", "", "", ""])[1:5]
            self.recording = None
            self.rec_timer.stop()
            self.last_clip = uri
            self.log("OK", f"수동 녹화 저장 완료: {uri} ({nmsg}개, {dur}s, {float(mb or 0) / 1024:.1f} GB) "
                           "— 진단은 [클립 진단]에서 이 폴더를 고르세요")
        elif kind == "rec_busy":
            self._rec_pending = False
            reason = parts[1] if len(parts) > 1 else ""
            if reason.startswith("already recording: ") and not self.recording:
                # GUI 를 다시 켰는데 레코더는 녹화 중이던 경우 — 이어서 보여준다
                self.recording = {"uri": reason.split(": ", 1)[1], "t0": time.time(), "sec": 0.0, "mb": 0.0}
                self.rec_timer.start()
            self.log("WARN", f"레코더: {reason}")
        elif kind == "rec_error":
            self._rec_pending = False
            if self.recording and self.recording.get("closing"):
                self.recording = None
                self.rec_timer.stop()
            self.log("ERROR", f"수동 녹화 오류: {parts[1] if len(parts) > 1 else ''}")
        self._style_record_button()

    def _sync_recording(self, kv):
        """레코더가 2초마다 알려주는 녹화 상태가 기준 — GUI 가 놓친 이벤트도 여기서 맞춘다."""
        active = kv.get("rec_active")
        if active == "true":
            try:
                sec, mb = float(kv.get("rec_sec", 0)), float(kv.get("rec_mb", 0))
            except ValueError:
                sec, mb = 0.0, 0.0
            if not self.recording:
                self.recording = {"uri": kv.get("rec_uri", ""), "t0": time.time() - sec}
                self.rec_timer.start()
                self._style_record_button()
            self.recording.update(sec=sec, mb=mb, t0=time.time() - sec)
            errors = kv.get("rec_errors", "0")
            if errors not in ("0", "") and not self.recording.get("warned"):
                self.recording["warned"] = True
                self.log("ERROR", f"수동 녹화 쓰기 오류 {errors}건 — 레코더 로그를 확인하세요")
        elif active == "false" and self.recording and not self.recording.get("closing"):
            self.log("WARN", "레코더가 녹화 중이 아닙니다 (레코더가 재시작됐을 수 있음) — 녹화 표시를 끕니다")
            self.recording = None
            self.rec_timer.stop()
            self._style_record_button()

    def _tick_recording(self):
        rec = self.recording
        if not rec:
            self.btn_record.setText("⏺  수동 녹화")
            self.lbl_rec.hide()
            return
        sec = time.time() - rec["t0"]
        clock = f"{int(sec // 3600):d}:{int(sec % 3600 // 60):02d}:{int(sec % 60):02d}"
        gb = rec.get("mb", 0.0) / 1024
        if rec.get("closing"):
            self.btn_record.setText("파일 닫는 중…")
        else:
            self.btn_record.setText(f"■  녹화 중지   {clock}")
        self.lbl_rec.setText(f"● 수동 녹화 중  {Path(rec['uri']).name}  ·  {clock}  ·  {gb:.1f} GB"
                             + (f"  ·  {self._last_rate:.0f} MB/s" if self._last_rate else ""))
        self.lbl_rec.show()

    def _set_state(self, text, kind="idle"):
        """현재 상태 줄. kind: idle · clip(클립 녹화) · write(디스크 기록) · diag(진단)."""
        self.lbl_state.setText(text)
        if self.lbl_state.property("kind") != kind:
            self.lbl_state.setProperty("kind", kind)
            ui_theme.repolish(self.lbl_state)

    def _busy_timeout(self):
        if self.busy:
            self.busy = False
            self.log("ERROR", "클립 완료 이벤트가 오지 않음 — 상태 초기화 "
                              "(레코더 로그를 확인하세요)")
            self._set_state("대기 중")
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
            lambda s: self._set_state(f"진단 중: {s}", "diag"))
        self.diag_thread.sig_done.connect(self._on_diag_done)
        self.diag_thread.sig_error.connect(
            lambda e: (self.log("ERROR", f"진단 실패: {e}"),
                       self._set_state("대기 중")))
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
        self._set_state("대기 중")
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
    def quit_by_signal(self, name):
        """터미널 Ctrl+C · SIGTERM · SIGHUP: 묻지 않고 녹화 정리 → 센서 → 레코더 순으로 내리고 끈다."""
        if self.quit_signal:
            return
        self.quit_signal = name
        if not self.close():            # closeEvent 가 거부할 일은 없지만, 창이 이미 닫혔으면 여기서 끝낸다
            QApplication.instance().quit()

    def _quit_note(self, text):
        self.log("GUI", text)
        if self.quit_signal:
            say(text)

    def closeEvent(self, ev):
        by_signal = bool(self.quit_signal)
        if self.recording:
            if not by_signal and not self._confirm_end_recording("GUI 를 닫으면"):
                ev.ignore()
                return
            self._quit_note(f"수동 녹화를 끝내고 파일을 닫는 중… ({Path(self.recording['uri']).name})")
            self._stop_recording_and_wait()
        if self.stage.supervisor.any_active():
            # "센서는 두고 닫기"는 두지 않는다. 창이 닫히면 QProcess 가 런치를 SIGKILL 하고, 로그 파이프가
            # 끊긴 노드들도 곧 쓰러져서 (2026-09-19 확인) 센서가 계속 도는 게 아니라 지저분하게 죽을 뿐이다.
            if not by_signal:
                box = QMessageBox(QMessageBox.Question, "센서 종료",
                                  "센서가 아직 실행 중입니다.\n센서를 모두 종료하고 닫을까요?",
                                  QMessageBox.Yes | QMessageBox.Cancel, self)
                box.button(QMessageBox.Yes).setText("센서 종료하고 닫기")
                box.button(QMessageBox.Cancel).setText("취소")
                box.setDefaultButton(QMessageBox.Yes)
                if box.exec_() != QMessageBox.Yes:
                    ev.ignore()
                    return
            names = [self.stage.groups[k]["label"] for k, p in self.stage.supervisor.procs.items()
                     if p.is_active() and k in self.stage.groups]
            self._quit_note(f"센서 종료 중… ({', '.join(names)})")
            self.stage.stop_all()
        save_config(self.cfg)
        self.preview.shutdown()
        if self.diag_thread and self.diag_thread.isRunning():
            self.diag_thread.wait(1000)
        self.netmon.stop()
        if self.owns_recorder:
            if self.recorder.state() != QProcess.NotRunning:
                self._quit_note("레코더 종료 중…")
            self.recorder.stop_recorder()
        self.worker.stop()
        super().closeEvent(ev)
        if by_signal:
            say("종료 완료")
            # 모달 창(토픽 선택 등)이 떠 있어도 그 이벤트 루프까지 같이 빠져나온다
            QApplication.instance().quit()


def say(text):
    """터미널에 한 줄. 터미널이 닫혀(SIGHUP) 못 쓰면 조용히 넘어간다."""
    try:
        print(f"[clip_gui] {text}", file=sys.stderr, flush=True)
    except (OSError, ValueError):
        pass


def install_signal_handlers(win):
    """터미널 Ctrl+C · SIGTERM · SIGHUP(터미널 닫힘) → 센서부터 정리하고 끈다.

    센서 런치와 레코더는 setsid 로 따로 세션에 떠 있어 터미널 신호를 직접 받지 않는다 — 이 처리기가
    받아서 win.quit_by_signal 로 녹화 정리 → 센서 → 레코더 순으로 내린다. 종료 중에 한 번 더 받으면
    기다리지 않고 강제로 끝낸다 (sensor_launcher.FORCE_STOP).
    """
    def handler(signum, _frame):
        name = signal.Signals(signum).name
        if not win.quit_signal:
            say(f"{name} 받음 — 녹화 정리 → 센서 종료 → 레코더 종료 순으로 끕니다 "
                "(기다리지 않으려면 Ctrl+C 한 번 더)")
            QTimer.singleShot(0, lambda: win.quit_by_signal(name))
        elif not sensor_launcher.FORCE_STOP:
            sensor_launcher.FORCE_STOP = True
            say(f"{name} 다시 받음 — 기다리지 않고 강제로 끝냅니다")

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, handler)
    # 파이썬 시그널 처리기는 파이썬 코드가 돌 때만 불린다. Qt 이벤트 루프가 조용할 때도 곧바로
    # 불리도록 빈 타이머로 주기적으로 깨운다.
    wake = QTimer(win)
    wake.timeout.connect(lambda: None)
    wake.start(200)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("DM Clip GUI")
    ui_theme.apply(app)
    cfg = load_config()
    worker = RosWorker(cfg)
    worker.start()

    # 예전에는 여기서 토픽 선택 창이 먼저 떴다. 이제는 센서를 띄우는 것이 먼저이고
    # (아직 안 뜬 센서의 토픽은 목록에 없다), 기동이 끝나면 자동으로 선택 창이 열린다.
    # 이미 떠 있는 센서에 붙어 바로 녹화하려면 [녹화] 탭 -> 파일 -> 토픽 선택.
    win = MainWindow(worker, cfg)
    install_signal_handlers(win)
    win.show()
    rc = app.exec_()
    return rc


if __name__ == "__main__":
    sys.exit(main())
