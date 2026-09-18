#!/usr/bin/env python3
# sensor_launcher.py — 센서군을 자식 프로세스로 띄우고 상태를 본다.
#
# 센서군마다 소싱해야 하는 워크스페이스가 다르다 (FLIR_control / rt2000_ws / ROS 언더레이).
# 한 런치 파일로 합칠 수 없는 이유가 그것이고, 그래서 센서군당 프로세스 하나로 간다:
#
#   setsid bash -c 'source <ws1>; source <ws2>; exec ros2 launch <pkg> <file> arg:=val ...'
#
# exec 이므로 우리가 쥐는 PID가 곧 ros2 launch 다. 종료는 SIGINT (ros2 launch 가
# 자식 노드들을 정리하는 유일한 신호) -> STOP_GRACE_MS 후 SIGKILL. setsid 로 따로 세션에 띄워
# 터미널 Ctrl+C 는 GUI 만 받는다 (GUI 가 센서부터 순서대로 내린다).
#
# 부수 효과 하나: ouster_ros 는 센서에 못 붙으면 launch.events.Shutdown 을 쏘는데 이 이벤트는
# 런치 서비스 전역이라, 한 런치에 카메라와 같이 넣으면 카메라까지 죽는다
# (all_sensors.launch.py 가 사전 TCP 프로브로 막고 있는 문제). 센서군을 프로세스로 갈라두면
# 그 폭발이 라이다 프로세스 안에 갇힌다.

import os
import re
import shlex
import signal
import time

from PyQt5.QtCore import QObject, QProcess, QProcessEnvironment, QTimer, pyqtSignal

import sensor_config
from sensor_discovery import expand

STOPPED, STARTING, RUNNING, FAILED = "stopped", "starting", "running", "failed"

# 런치가 emulate_tty 로 찍는 ANSI 색 코드 — 로그 창에 [33m 같은 찌꺼기로 보인다
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

STATE_LABEL = {STOPPED: "정지", STARTING: "기동 중", RUNNING: "실행 중", FAILED: "실패"}

# 기대 토픽이 하나도 더 늘지 않고 이만큼 지나면 실패로 본다. 전체 시간 제한이 아니다 —
# 순차 기동이면 카메라가 한 대씩 (대당 열거 ~13초 + 설정) 올라와서 15대면 몇 분이 걸린다.
# 한 대라도 새로 뜨면 시계가 다시 돈다.
READY_TIMEOUT_S = 90.0

ROS_UNDERLAY = "/opt/ros/humble/setup.bash"

# 모든 센서군을 같은 미들웨어로 묶는다. ~/flir_ouster_ws 는 cyclonedds 가 기본이라
# 섞이면 레코더가 토픽을 아예 못 찾는다 (all_sensors_bagging.sh 에 같은 이유의 주석이 있다).
RMW = "rmw_fastrtps_cpp"

# 워크스페이스 소싱이 쌓는 변수들. 센서군마다 깨끗한 상태에서 다시 소싱한다.
LEAKY_ENV = ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
             "LD_LIBRARY_PATH", "PKG_CONFIG_PATH", "ROS_PACKAGE_PATH")


STOP_GRACE_MS = 10000     # SIGINT 뒤 ros2 launch 가 자식들을 정리하고 내려갈 시간 (카메라 13대면 5초로는 모자랐다)

# "기다리지 말고 끝내라" — 종료 중에 터미널 Ctrl+C 를 한 번 더 누르면 clip_gui 의 시그널 처리기가 켠다.
# 종료 대기 루프들이 짧게 끊어 기다리며 이 값을 본다.
FORCE_STOP = False


def wait_finished(proc, timeout_ms):
    """QProcess 가 끝날 때까지 기다린다. True = 제시간에 끝남.

    waitForFinished(긴 시간) 한 번으로 기다리면 그동안 파이썬 시그널 처리기가 돌지 못해 강제 종료
    요청(FORCE_STOP)을 못 받는다. 100 ms 씩 끊어서 기다린다.
    """
    deadline = time.time() + timeout_ms / 1000
    while proc.state() != QProcess.NotRunning:
        if FORCE_STOP or time.time() > deadline:
            return False
        proc.waitForFinished(100)
    return True


def _proc_table():
    """{pid: (ppid, cmdline 토큰들)} — /proc 에서 한 번에 읽는다."""
    table = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/stat", "rb") as stream:
                stat = stream.read().decode(errors="replace")
            ppid = int(stat[stat.rindex(")") + 2:].split()[1])
            with open(f"/proc/{entry}/cmdline", "rb") as stream:
                cmd = [t.decode(errors="replace") for t in stream.read().split(b"\0") if t]
        except (OSError, ValueError, IndexError):
            continue
        table[int(entry)] = (ppid, cmd)
    return table


def descendants(pid, table=None):
    """pid 아래 모든 자손 (런치 → 카메라 노드들 → ...)."""
    table = table or _proc_table()
    children = {}
    for child, (ppid, _cmd) in table.items():
        children.setdefault(ppid, []).append(child)
    out, stack = [], [pid]
    while stack:
        for child in children.get(stack.pop(), []):
            out.append(child)
            stack.append(child)
    return out


def _alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def orphans(group):
    """이전 기동에서 남은 이 센서군의 노드 — 부모 ros2 launch 는 죽고 자기만 남은 것.

    이런 게 카메라를 쥐고 있으면 새로 띄운 노드가 프레임을 못 받는다 (실제로 한 대가 그랬다).
    실행 파일이 <prefix>/lib/<launch 패키지>/ 아래에 있고 --ros-args 로 떴는데, 부모가
    ros2 launch 가 아니면 고아로 본다. 터미널에서 따로 띄운 런치의 노드는 부모가 ros2 launch 라 안 건드린다.
    """
    table = _proc_table()
    marker = f"/lib/{group['launch_package']}/"
    out = []
    for pid, (ppid, cmd) in table.items():
        if not cmd or marker not in cmd[0] or "--ros-args" not in cmd:
            continue
        parent = table.get(ppid, (0, []))[1]
        if not any(t.endswith("ros2") for t in parent[:2]) or "launch" not in parent:
            out.append((pid, cmd))
    return out


def terminate(pids, grace_s=3.0):
    """SIGTERM → grace_s 기다렸다 → 남은 건 SIGKILL."""
    pids = [p for p in pids if _alive(p)]
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.time() + grace_s
    while time.time() < deadline and any(_alive(p) for p in pids):
        time.sleep(0.1)
    for pid in pids:
        if _alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass


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


def build_command(group, overrides, devices=None, notes=None):
    """(program, [args], 기대 네임스페이스) — bash -c 한 줄로 소싱 + 런치.

    devices(감지 결과)를 주면 보이는 카메라만 담은 인벤토리 사본으로 띄운다 (sensor_config.plan_launch).
    """
    sources = []
    workspaces = list(group.get("workspaces") or [])
    if ROS_UNDERLAY not in workspaces:
        workspaces.insert(0, ROS_UNDERLAY)
    for workspace in workspaces:
        path = expand(workspace)
        if path and path.is_file():
            sources.append(f"source {shlex.quote(str(path))}")

    args, namespaces = sensor_config.plan_launch(group, overrides, devices, notes)
    launch = ["ros2", "launch", group["launch_package"], group["launch_file"]] + args

    # ROS setup.bash 는 nounset 에 안전하지 않다 — set -u 를 켜면 안 된다.
    script = "; ".join(sources + ["exec " + " ".join(shlex.quote(t) for t in launch)])
    return "bash", ["-c", script], namespaces


class SensorGroupProcess(QObject):
    """센서군 하나의 런치 프로세스."""

    sig_state = pyqtSignal(str, str)      # group_key, state
    sig_output = pyqtSignal(str, str)     # group_key, 줄
    sig_progress = pyqtSignal(str, int, int)   # group_key, 뜬 것, 기대 — "기동 중 5/15"
    sig_devices = pyqtSignal(str)              # group_key — 장비(네임스페이스)별 상태가 바뀜

    def __init__(self, group, parent=None):
        super().__init__(parent)
        self.group = group
        self.key = group["key"]
        self.state = STOPPED
        self._buf = ""
        self._elapsed = 0.0          # 기동 시작부터
        self._stalled = 0.0          # 마지막으로 새 토픽이 뜬 뒤부터
        self._progress = -1
        self._expect = []
        # 장비(네임스페이스)별 상태 — 감지된 장비 표의 행마다 점으로 보인다.
        # 카메라 노드는 뜨자마자(카메라를 열기 전에) 발행자를 만들어서 "토픽이 있다"만으로는 영상이
        # 나오는지 모른다. 그래서 레지스트리의 ready_log(예: "Camera acquisition started")를 그 노드의
        # 로그에서 봐야 '실행 중'이다.
        self.ready_log = group.get("ready_log") or ""
        self.expected_ns = []           # 이번 기동의 네임스페이스들
        self.ns_logged = set()          # 로그가 나오기 시작 = 노드가 떠서 카메라를 여는 중
        self.ns_ready = set()           # ready_log 가 찍힘 = 영상 수신 시작
        self.ns_dead = set()            # "process has died"
        self.ns_present = set()         # 지금 발행자가 있음
        self.ns_seen = set()            # 한 번이라도 발행자가 있었음 (사라지면 실패)
        self._stopping = None           # begin_stop 이 적어 둔 (프로세스 가족, 기한)
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._on_output)
        self.proc.finished.connect(self._on_finished)

        env = QProcessEnvironment.systemEnvironment()
        # 셸에서 소싱해 둔 워크스페이스가 새어 들어오지 않게 경로 변수를 비우고, sensors.yaml 이
        # 선언한 워크스페이스만 소싱한다. ~/.bashrc 가 ~/flir_ouster_ws 를 소싱하고 있어서, 안 비우면
        # ouster_ros 가 선언한 언더레이(0.14.2)가 아니라 그 워크스페이스 빌드(0.14.0)로 뜬다.
        # ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY 는 그대로 둔다 — 레코더와 같아야 서로 보인다.
        for var in LEAKY_ENV:
            env.remove(var)
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

    def start(self, overrides, expectations=None, devices=None):
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

        stale = orphans(self.group)
        if stale:
            names = ", ".join(next((t.split("=", 1)[1] for t in c if t.startswith("__ns:=")), c[0].rsplit("/", 1)[-1])
                              for _p, c in stale)
            self.sig_output.emit(self.key, f"[GUI] 이전 기동에서 남은 노드 {len(stale)}개 정리: {names}")
            terminate([p for p, _c in stale])

        notes = []
        try:
            program, args, namespaces = build_command(self.group, overrides, devices, notes)
        except Exception as exc:                                   # noqa: BLE001
            self.sig_output.emit(self.key, f"[GUI] 런치 명령 생성 실패: {exc}")
            self._set_state(FAILED)
            return
        for note in notes:
            self.sig_output.emit(self.key, "[GUI] " + note)

        if namespaces:
            # 이번에 띄우는 카메라 하나하나의 네임스페이스에 토픽이 떠야 완료. 이름을 GUI 에서
            # 바꿔도 (camera_ 로 시작하지 않아도) 정확히 맞는다.
            expectations = [(f"^/{re.escape(ns)}/", 1) for ns in namespaces]
        elif expectations is None:
            expectations = [(p, 1) for p in sensor_config.topic_regexes(self.group, overrides)]
        self._expect = [(re.compile(p), max(1, int(n))) for p, n in expectations]
        self._elapsed = self._stalled = 0.0
        self._progress = -1
        self.expected_ns = list(namespaces or [])
        for bucket in (self.ns_logged, self.ns_ready, self.ns_dead, self.ns_present, self.ns_seen):
            bucket.clear()
        self.sig_output.emit(self.key, "[GUI] " + args[-1])
        # 프로세스를 먼저 띄우고 상태를 알린다. 거꾸로 하면 STARTING 을 받은 쪽이
        # is_active() 를 물었을 때 아직 NotRunning 이라, 기동 중인데 [중지]가 잠기고
        # 체크박스는 안 잠긴다.
        # setsid: 런치를 따로 세션(프로세스 그룹)에 띄워, 터미널 Ctrl+C 가 GUI 만 거치게 한다. 전에는 같은
        # 그룹이라 Ctrl+C 가 GUI 와 런치에 동시에 가서, GUI 가 먼저 죽고 로그 파이프가 끊긴 런치가 정리
        # 도중 쓰러져 카메라 노드가 고아로 남곤 했다. 이제 GUI 가 받아서 순서대로 내린다 (clip_gui 참고).
        # 포크 없이 exec 하므로 우리가 쥐는 PID 는 그대로 ros2 launch 다.
        self.proc.start("setsid", [program] + args)
        self._set_state(STARTING)
        # 장비별 상태도 프로세스가 뜬 뒤에 알린다. 먼저 알리면 받는 쪽(stage)이 is_active()=False 를 보고
        # 카메라별 매핑(시리얼 → 네임스페이스)을 지워 버려, 모든 행이 센서군 상태 하나로 똑같이 칠해졌다.
        self.sig_devices.emit(self.key)

    def stop(self):
        self.begin_stop()
        self.finish_stop()

    def begin_stop(self):
        """SIGINT 만 보내고 돌아온다. 기다리는 건 finish_stop — 센서군 여럿을 한꺼번에 내릴 때 (stop_all)."""
        self._stopping = None
        if not self.is_active():
            return
        pid = int(self.proc.processId())
        # 가족을 먼저 적어 둔다. ros2 launch 가 먼저 죽으면 자식들의 부모가 systemd 로 바뀌어
        # 누가 누구 자식이었는지 알 수 없게 된다. 예전에는 5초 뒤 런치만 SIGKILL 해서, 종료가
        # 오래 걸리는 카메라 노드가 고아로 남아 카메라를 계속 쥐고 있었다.
        family = descendants(pid) if pid > 0 else []
        if pid > 0:
            try:
                # exec 로 바꿔치기했으므로 이 PID 가 ros2 launch 다. SIGINT 여야
                # 런치가 자식 노드들을 정리하고 내려간다.
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
        self._stopping = family

    def finish_stop(self):
        """begin_stop 뒤: 런치가 내려가길 (최대 STOP_GRACE_MS) 기다리고 남은 자식을 정리한다."""
        family, self._stopping = self._stopping or [], None
        if not wait_finished(self.proc, STOP_GRACE_MS):
            self.proc.kill()
            self.proc.waitForFinished(2000)
        leftovers = [p for p in family if _alive(p)]
        if leftovers:
            self.sig_output.emit(self.key, f"[GUI] 런치가 내려간 뒤 남은 프로세스 {len(leftovers)}개 정리")
            terminate(leftovers, grace_s=0.5 if FORCE_STOP else 3.0)
        self._set_state(STOPPED)

    # --- 준비 판정 ---

    def tick(self, topic_names, dt):
        """주기 호출. 프로세스가 살아있고 기대 토픽이 다 떴으면 RUNNING."""
        if not self.is_active():
            return
        present = {ns for ns in self.expected_ns
                   if any(name.startswith(f"/{ns}/") for name in topic_names)}
        if present != self.ns_present:
            self.ns_present = present
            self.ns_seen |= present
            self.sig_devices.emit(self.key)
        if self.state == RUNNING:
            return
        self._elapsed += dt
        self._stalled += dt
        have, want = self.progress(topic_names)
        if have != self._progress:
            if have > self._progress:
                self._stalled = 0.0
            self._progress = have
            self.sig_progress.emit(self.key, have, want)
        if self._ready(topic_names):
            self.sig_output.emit(self.key, f"[GUI] 기대 토픽이 다 떴습니다 ({want}개, {self._elapsed:.0f}초)")
            self._set_state(RUNNING)
        elif self._stalled > READY_TIMEOUT_S and self.state != FAILED:
            short = ", ".join(f"{p} {have}/{want}" for p, have, want in self.pending(topic_names))
            self.sig_output.emit(
                self.key,
                f"[GUI] {READY_TIMEOUT_S:.0f}초 동안 새로 뜬 토픽이 없습니다 ({have}/{want} · {short}). "
                "프로세스는 살아 있습니다 — 위 로그를 확인하세요.")
            self._set_state(FAILED)

    def _log_gated(self):
        return bool(self.ready_log and self.expected_ns)

    def progress(self, topic_names):
        """(뜬 것, 기대) — 기대 항목마다 최소 대수까지만 센다. ready_log 가 있으면 영상이 시작된 대수."""
        if self._log_gated():
            return len(self.ns_ready & set(self.expected_ns)), len(self.expected_ns)
        have = sum(min(len(self._namespaces(rx, topic_names)), minimum)
                   for rx, minimum in self._expect)
        return have, sum(minimum for _rx, minimum in self._expect)

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
        if self._log_gated() and not set(self.expected_ns) <= self.ns_ready:
            return False
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
            line = _ANSI.sub("", line).rstrip()
            if line.strip():
                self._track_line(line)
                self.sig_output.emit(self.key, line)

    _LOGGER = re.compile(r"\[([A-Za-z0-9_.]+)\.[A-Za-z0-9_]+\]:")
    _DIED_NS = re.compile(r"__ns:=/?([A-Za-z0-9_/]+)")

    def _track_line(self, line):
        """노드 로그 한 줄에서 장비별 상태를 읽는다 (로거 이름 = <네임스페이스>.<노드>)."""
        if not self.expected_ns:
            return
        changed = False
        if "process has died" in line:
            match = self._DIED_NS.search(line)
            if match and match.group(1) in self.expected_ns and match.group(1) not in self.ns_dead:
                self.ns_dead.add(match.group(1))
                changed = True
        else:
            match = self._LOGGER.search(line)
            ns = match.group(1).replace(".", "/") if match else None
            if ns in self.expected_ns:
                if ns not in self.ns_logged:
                    self.ns_logged.add(ns)
                    changed = True
                if self.ready_log and self.ready_log in line and ns not in self.ns_ready:
                    self.ns_ready.add(ns)
                    changed = True
        if changed:
            self.sig_devices.emit(self.key)

    def _on_finished(self, code, _status):
        if self._buf.strip():
            self.sig_output.emit(self.key, _ANSI.sub("", self._buf).rstrip())
            self._buf = ""
        if self.state == STOPPED:
            return
        self.sig_output.emit(self.key, f"[GUI] 프로세스 종료 (exit {code})")
        self._set_state(FAILED if code else STOPPED)


class SensorSupervisor(QObject):
    """센서군 프로세스 묶음. GUI는 이것 하나만 들고 있으면 된다."""

    sig_state = pyqtSignal(str, str)
    sig_output = pyqtSignal(str, str)
    sig_progress = pyqtSignal(str, int, int)
    sig_devices = pyqtSignal(str)

    TICK_MS = 1000

    def __init__(self, registry, parent=None):
        super().__init__(parent)
        self.groups = {g["key"]: g for g in registry.get("groups", [])}
        self.procs = {}
        for key, group in self.groups.items():
            proc = SensorGroupProcess(group, self)
            proc.sig_state.connect(self.sig_state)
            proc.sig_output.connect(self.sig_output)
            proc.sig_progress.connect(self.sig_progress)
            proc.sig_devices.connect(self.sig_devices)
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

    def start(self, key, overrides, expectations=None, devices=None):
        proc = self.procs.get(key)
        if proc:
            proc.start(overrides, expectations, devices)

    def stop(self, key):
        proc = self.procs.get(key)
        if proc:
            proc.stop()

    def stop_all(self):
        # 한꺼번에 SIGINT 를 보내고 기다린다. 하나씩 하면 센서군마다 종료 시간이 쌓인다.
        for proc in self.procs.values():
            proc.begin_stop()
        for proc in self.procs.values():
            proc.finish_stop()

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
