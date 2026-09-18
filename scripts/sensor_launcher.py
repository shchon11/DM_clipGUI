#!/usr/bin/env python3
# sensor_launcher.py — 센서군을 자식 프로세스로 띄우고 상태를 본다.
#
# 센서군마다 소싱해야 하는 워크스페이스가 다르다 (FLIR_control / rt2000_ws / ROS 언더레이).
# 한 런치 파일로 합칠 수 없는 이유가 그것이고, 그래서 센서군당 프로세스 하나로 간다:
#
#   bash -c 'source <ws1>; source <ws2>; exec ros2 launch <pkg> <file> arg:=val ...'
#
# exec 이므로 우리가 쥐는 PID가 곧 ros2 launch 다. 종료는 SIGINT (ros2 launch 가
# 자식 노드들을 정리하는 유일한 신호) -> 5초 후 SIGKILL.
#
# 부수 효과 하나: ouster_ros 는 센서에 못 붙으면 launch.events.Shutdown 을 쏘는데 이 이벤트는
# 런치 서비스 전역이라, 한 런치에 카메라와 같이 넣으면 카메라까지 죽는다
# (all_sensors.launch.py 가 사전 TCP 프로브로 막고 있는 문제). 센서군을 프로세스로 갈라두면
# 그 폭발이 라이다 프로세스 안에 갇힌다.

import os
import re
import shlex
import signal

from PyQt5.QtCore import QObject, QProcess, QProcessEnvironment, QTimer, pyqtSignal

import sensor_config
from sensor_discovery import expand

STOPPED, STARTING, RUNNING, FAILED = "stopped", "starting", "running", "failed"

STATE_LABEL = {STOPPED: "정지", STARTING: "기동 중", RUNNING: "실행 중", FAILED: "실패"}

# 토픽이 뜰 때까지 기다려주는 시간. 8대 카메라는 camera_start_stagger 만큼 순차 기동하므로
# 넉넉해야 한다 (3초 x 10대 + Init 재시도).
READY_TIMEOUT_S = 90.0

ROS_UNDERLAY = "/opt/ros/humble/setup.bash"

# 모든 센서군을 같은 미들웨어로 묶는다. ~/flir_ouster_ws 는 cyclonedds 가 기본이라
# 섞이면 레코더가 토픽을 아예 못 찾는다 (all_sensors_bagging.sh 에 같은 이유의 주석이 있다).
RMW = "rmw_fastrtps_cpp"


def work_dir(group):
    """런치를 돌릴 디렉터리.

    카메라 런치는 camera_info.yaml_path 같은 값이 리포 루트 기준 상대 경로라 CWD가
    거기여야 한다 (아니면 노드가 "Failed to open camera_info YAML file" 로 죽는다).
    지정이 없으면 생성 파일 디렉터리 — ouster_ros 가 CWD 에 <sensor-ip>-metadata.json 을
    떨어뜨리는데 그게 리포 안에 생기지 않게 하려는 것이다.
    """
    path = expand(group.get("workdir"))
    if path and path.is_dir():
        return path
    sensor_config.GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    return sensor_config.GENERATED_DIR


def build_command(group, overrides):
    """(program, [args]) — bash -c 한 줄로 소싱 + 런치."""
    sources = []
    workspaces = list(group.get("workspaces") or [])
    if ROS_UNDERLAY not in workspaces:
        workspaces.insert(0, ROS_UNDERLAY)
    for workspace in workspaces:
        path = expand(workspace)
        if path and path.is_file():
            sources.append(f"source {shlex.quote(str(path))}")

    launch = ["ros2", "launch", group["launch_package"], group["launch_file"]]
    launch += sensor_config.launch_args(group, overrides)

    # ROS setup.bash 는 nounset 에 안전하지 않다 — set -u 를 켜면 안 된다.
    script = "; ".join(sources + ["exec " + " ".join(shlex.quote(t) for t in launch)])
    return "bash", ["-c", script]


class SensorGroupProcess(QObject):
    """센서군 하나의 런치 프로세스."""

    sig_state = pyqtSignal(str, str)      # group_key, state
    sig_output = pyqtSignal(str, str)     # group_key, 줄

    def __init__(self, group, parent=None):
        super().__init__(parent)
        self.group = group
        self.key = group["key"]
        self.state = STOPPED
        self._buf = ""
        self._elapsed = 0.0
        self._expect = []
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._on_output)
        self.proc.finished.connect(self._on_finished)

        env = QProcessEnvironment.systemEnvironment()
        env.insert("RMW_IMPLEMENTATION", RMW)
        self.proc.setProcessEnvironment(env)
        self.proc.setWorkingDirectory(str(work_dir(group)))

    # --- 상태 ---

    def _set_state(self, state):
        if state != self.state:
            self.state = state
            self.sig_state.emit(self.key, state)

    def is_active(self):
        return self.proc.state() != QProcess.NotRunning

    # --- 기동/종료 ---

    def start(self, overrides, expectations=None):
        """expectations: [(토픽 정규식, 최소 대수)].

        패턴이 하나라도 매칭되면 끝이 아니라, 감지된 대수만큼 네임스페이스가 떠야
        RUNNING 이다. 카메라는 camera_start_stagger 로 한 대씩 올라오므로, 첫 대만 보고
        완료라고 하면 나머지가 아직 Init 중인데 녹화를 시작하게 된다.
        """
        if self.is_active():
            return
        gaps = sensor_config.missing_paths(self.group)
        if gaps:
            self.sig_output.emit(self.key, "[GUI] 경로가 없어 기동할 수 없습니다: " + ", ".join(gaps))
            self._set_state(FAILED)
            return

        try:
            program, args = build_command(self.group, overrides)
        except Exception as exc:                                   # noqa: BLE001
            self.sig_output.emit(self.key, f"[GUI] 런치 명령 생성 실패: {exc}")
            self._set_state(FAILED)
            return

        if expectations is None:
            expectations = [(p, 1) for p in sensor_config.topic_regexes(self.group, overrides)]
        self._expect = [(re.compile(p), max(1, int(n))) for p, n in expectations]
        self._elapsed = 0.0
        self.sig_output.emit(self.key, "[GUI] " + args[-1])
        self._set_state(STARTING)
        self.proc.start(program, args)

    def stop(self):
        if not self.is_active():
            self._set_state(STOPPED)
            return
        pid = int(self.proc.processId())
        if pid > 0:
            try:
                # exec 로 바꿔치기했으므로 이 PID 가 ros2 launch 다. SIGINT 여야
                # 런치가 자식 노드들을 정리하고 내려간다.
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
        if not self.proc.waitForFinished(5000):
            self.proc.kill()
            self.proc.waitForFinished(2000)
        self._set_state(STOPPED)

    # --- 준비 판정 ---

    def tick(self, topic_names, dt):
        """주기 호출. 프로세스가 살아있고 기대 토픽이 다 떴으면 RUNNING."""
        if not self.is_active():
            return
        if self.state == RUNNING:
            return
        self._elapsed += dt
        if self._ready(topic_names):
            self._set_state(RUNNING)
        elif self._elapsed > READY_TIMEOUT_S:
            short = ", ".join(f"{p} {have}/{want}" for p, have, want in self.pending(topic_names))
            self.sig_output.emit(
                self.key,
                f"[GUI] {READY_TIMEOUT_S:.0f}초 안에 기대 토픽이 다 뜨지 않았습니다 "
                f"({short}). 프로세스는 살아 있습니다 — 위 로그를 확인하세요.")
            self._set_state(FAILED)

    @staticmethod
    def _namespaces(rx, topic_names):
        """패턴에 걸린 토픽들의 네임스페이스 집합. /thermal0/image_raw -> /thermal0/"""
        found = set()
        for name in topic_names:
            match = rx.match(name)
            if match:
                found.add(match.group(0))
        return found

    def _ready(self, topic_names):
        if not self._expect:
            return True
        return all(len(self._namespaces(rx, topic_names)) >= minimum
                   for rx, minimum in self._expect)

    def pending(self, topic_names):
        """아직 모자란 것 (표시용): [(패턴, 현재, 기대)]"""
        out = []
        for rx, minimum in self._expect:
            have = len(self._namespaces(rx, topic_names))
            if have < minimum:
                out.append((rx.pattern, have, minimum))
        return out

    # --- 출력 ---

    def _on_output(self):
        self._buf += bytes(self.proc.readAllStandardOutput()).decode("utf-8", "replace")
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line.strip():
                self.sig_output.emit(self.key, line.rstrip())

    def _on_finished(self, code, _status):
        if self._buf.strip():
            self.sig_output.emit(self.key, self._buf.rstrip())
            self._buf = ""
        if self.state == STOPPED:
            return
        self.sig_output.emit(self.key, f"[GUI] 프로세스 종료 (exit {code})")
        self._set_state(FAILED if code else STOPPED)


class SensorSupervisor(QObject):
    """센서군 프로세스 묶음. GUI는 이것 하나만 들고 있으면 된다."""

    sig_state = pyqtSignal(str, str)
    sig_output = pyqtSignal(str, str)

    TICK_MS = 1000

    def __init__(self, registry, parent=None):
        super().__init__(parent)
        self.groups = {g["key"]: g for g in registry.get("groups", [])}
        self.procs = {}
        for key, group in self.groups.items():
            proc = SensorGroupProcess(group, self)
            proc.sig_state.connect(self.sig_state)
            proc.sig_output.connect(self.sig_output)
            self.procs[key] = proc
        self._topics = []
        self.timer = QTimer(self, interval=self.TICK_MS, timeout=self._tick)
        self.timer.start()

    def set_topics(self, topic_names):
        """GUI가 주기적으로 넘겨주는 현재 토픽 목록 (RosWorker.list_topics 결과)."""
        self._topics = list(topic_names)

    def _tick(self):
        for proc in self.procs.values():
            proc.tick(self._topics, self.TICK_MS / 1000.0)

    def start(self, key, overrides, expectations=None):
        proc = self.procs.get(key)
        if proc:
            proc.start(overrides, expectations)

    def stop(self, key):
        proc = self.procs.get(key)
        if proc:
            proc.stop()

    def stop_all(self):
        for proc in self.procs.values():
            proc.stop()

    def state(self, key):
        proc = self.procs.get(key)
        return proc.state if proc else STOPPED

    def any_active(self):
        return any(p.is_active() for p in self.procs.values())

    def summary(self):
        """'3/4 실행 중' 같은 한 줄."""
        running = sum(1 for p in self.procs.values() if p.state == RUNNING)
        started = sum(1 for p in self.procs.values() if p.is_active())
        if not started:
            return "센서 정지"
        failed = sum(1 for p in self.procs.values() if p.state == FAILED)
        text = f"센서 {running}/{started} 실행 중"
        return text + (f" · {failed} 실패" if failed else "")
