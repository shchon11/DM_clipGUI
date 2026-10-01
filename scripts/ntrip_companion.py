# ntrip_companion.py — GNSS(RT2000) 센서군과 같이 뜨는 NTRIP 보정(str2str).
#
#   NGII VRS 캐스터 ─(인터넷)→ str2str ─(USB-시리얼 9600)→ RT2000 COM2 (RTCM 입력)
#
# 센서군이 켜질 때 같이 켜고 꺼질 때 같이 끈다. 이미 다른 str2str 이 돌고 있으면 새로 띄우지 않는다 —
# 같은 계정으로 둘이 붙으면 캐스터가 서로를 끊어 보정이 안 들어온다 (2026-09-29).
# VRS 는 위치(GGA)를 받아야 보정을 보내는데 RT2000 → PC 로 GGA 가 안 나오므로 -p 로 좌표를 준다
# (-b 1 은 쓰면 안 됨). 좌표는 GUI 가 마지막으로 받은 GNSS 위치, 없으면 설정의 기본 좌표.
# 설정(계정 포함): ~/.config/dm_clip_gui/ntrip.yaml — 저장소에 두지 않는다.

import os
import re
import socket
import threading
import signal
import subprocess
import time
from pathlib import Path

import yaml
from PyQt5.QtCore import QObject, QProcess, QTimer, pyqtSignal

CONFIG = Path.home() / ".config" / "dm_clip_gui" / "ntrip.yaml"
GUI_SESSION = Path.home() / ".config" / "dm_clip_gui" / "last_session.yaml"
DEFAULTS = {
    "enable": True,
    "host": "RTS1.ngii.go.kr",
    "port": 2101,
    "mount": "VRS-RTCM31",           # RTCM34 는 ~9 kbps 라 9600 baud 를 넘친다
    "user": "",
    "password": "",
    "serial": "ttyUSB0",
    "baud": 9600,
    "position": [37.55677, 127.04629, 72.5],   # GNSS 위치를 아직 모를 때 캐스터에 알릴 좌표
    "position_from_gnss": True,
    # NTRIP 포트(2101)를 막는 와이파이 — 여기 붙어 있으면 켜자마자 핫스팟으로 바꾸라고 안내한다
    "blocked_wifi": ["IRCV_Basement_5G"],
}
_STATUS = re.compile(r"\[([A-Z-]{2,})\]\s+(\d+)\s+B\s+(\d+)\s+bps")
RESTART_S = 5.0
NO_INPUT_S = 20.0            # 이만큼 캐스터 연결이 안 되면 문제
STUCK_S = 10.0               # 받은 바이트가 이만큼 안 늘면 문제


def load_config():
    cfg = dict(DEFAULTS)
    try:
        cfg.update(yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {})
    except FileNotFoundError:
        CONFIG.parent.mkdir(parents=True, exist_ok=True)
        CONFIG.write_text(yaml.safe_dump(DEFAULTS, allow_unicode=True, sort_keys=False), encoding="utf-8")
        os.chmod(CONFIG, 0o600)
    return cfg


def last_fix():
    try:
        ui = (yaml.safe_load(GUI_SESSION.read_text(encoding="utf-8")) or {}).get("ui") or {}
        lat, lon = ui.get("last_fix") or (None, None)
        return (float(lat), float(lon)) if lat is not None else None
    except Exception:
        return None


def current_wifi():
    """지금 붙어 있는 와이파이 이름 (없으면 "")."""
    try:
        # 활성 연결 목록은 스캔을 안 해서 즉시 나온다 ("dev wifi" 는 스캔을 기다려 3초 넘게 걸렸다)
        out = subprocess.run(["nmcli", "-t", "-f", "NAME,TYPE", "con", "show", "--active"], capture_output=True,
                             text=True, timeout=2).stdout
        return next((l.rsplit(":", 1)[0] for l in out.splitlines() if l.endswith(":802-11-wireless")), "")
    except Exception:
        return ""


def network_signature():
    """지금 인터넷이 나가는 길 — (기본 경로 인터페이스, 게이트웨이, 와이파이 이름). 바뀌면 네트워크가 바뀐 것."""
    iface = gw = ""
    try:
        for line in Path("/proc/net/route").read_text().splitlines()[1:]:
            f = line.split()
            if f[1] == "00000000" and int(f[3], 16) & 2:          # 기본 경로 · RTF_GATEWAY
                iface, gw = f[0], socket.inet_ntoa(bytes.fromhex(f[2])[::-1])
                break
    except Exception:
        pass
    return iface, gw, current_wifi()


def caster_reachable(host, port, timeout=4.0):
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except OSError:
        return False


def network_hint(cfg, ssid):
    where = f"지금 와이파이 '{ssid}'" if ssid else "지금 연결된 네트워크"
    return (f"NTRIP: 캐스터({cfg['host']}:{cfg['port']})에 연결이 안 됩니다 — {where} 가 NTRIP 을 막습니다. "
            "휴대폰 핫스팟에 연결하세요 (바꾸면 20초 안에 알아서 다시 붙습니다)")


def other_str2str():
    """이 GUI 가 띄우지 않은 str2str PID 들."""
    try:
        out = subprocess.run(["pgrep", "-x", "str2str"], capture_output=True, text=True).stdout
        return [int(p) for p in out.split()]
    except Exception:
        return []


class NtripCompanion(QObject):
    """sig_log(줄), sig_status() — status = {"level": off|wait|ok|warn|err, "text": 한 줄}."""

    sig_log = pyqtSignal(str)
    sig_status = pyqtSignal()
    _sig_net = pyqtSignal(bool, str)      # 망 확인 스레드 → (캐스터 닿음?, 와이파이 이름)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._on_output)
        self.proc.finished.connect(self._on_finished)
        self._buf = ""
        self._want = False
        self._external = False
        self.status = {"level": "off", "text": "NTRIP 보정 꺼짐", "kind": ""}
        self._net_ok, self._ssid = None, ""
        self._sig_net.connect(self._on_net)
        self._started = 0.0
        self._last_bytes, self._bytes_t = -1, 0.0
        self._last_logged = 0.0
        self._restart = QTimer(self, singleShot=True, timeout=self._launch)
        self._watch = QTimer(self, interval=2000, timeout=self._check)
        # 네트워크가 바뀌면(내부망 와이파이 ↔ 핫스팟 등) str2str 을 바로 다시 켠다 — 켜져 있는 동안 3초마다 본다
        self._net_sig = None
        self._netwatch = QTimer(self, interval=3000, timeout=self._check_network)

    # --- 켜기 · 끄기 ---
    def start(self):
        cfg = load_config()
        if not cfg.get("enable", True):
            self._set("off", "NTRIP 보정 꺼짐 (ntrip.yaml enable: false)")
            return
        self._want = True
        self._net_sig = network_signature()
        self._netwatch.start()
        self._launch()

    def _check_network(self):
        now = network_signature()
        if now == self._net_sig:
            return
        before, self._net_sig = self._net_sig, now
        desc = lambda s: (f"와이파이 '{s[2]}'" if s[2] else (s[0] or "연결 없음")) + (f" ({s[1]})" if s[1] else "")
        self.sig_log.emit(f"[NTRIP] 네트워크가 바뀜: {desc(before)} → {desc(now)} — str2str 을 다시 켭니다")
        if not now[0]:
            self._set("err", "NTRIP: 인터넷 연결이 없습니다 — 휴대폰 핫스팟에 연결하세요", "network")
            return                                   # 연결이 생기면 다시 바뀜으로 잡힌다
        self._net_ok = None
        self._restart_now()

    def _restart_now(self):
        """지금 str2str 을 내리고 바로 다시 켠다 (위치 · 망 확인도 새로)."""
        self._restart.stop()
        if self.proc.state() != QProcess.NotRunning:
            self._relaunch_pending = True
            try:
                os.kill(int(self.proc.processId()), signal.SIGINT)
            except OSError:
                pass
            if not self.proc.waitForFinished(3000):
                self.proc.kill()
                self.proc.waitForFinished(1000)
        self._relaunch_pending = False
        self._launch()

    def stop(self):
        self._want = False
        self._netwatch.stop()
        self._restart.stop()
        self._watch.stop()
        if self.proc.state() != QProcess.NotRunning:
            pid = int(self.proc.processId())
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
            if not self.proc.waitForFinished(3000):
                self.proc.kill()
                self.proc.waitForFinished(1000)
            self.sig_log.emit("[NTRIP] str2str 종료")
        self._external = False
        self._set("off", "NTRIP 보정 꺼짐")

    def _launch(self):
        if not self._want or self.proc.state() != QProcess.NotRunning:
            return
        cfg = load_config()
        others = other_str2str()
        if others:
            # 손으로 띄운 str2str 이 이미 있다 — 둘이면 캐스터가 서로를 끊는다
            self._external = True
            self._set("warn", f"NTRIP: 이미 돌고 있는 str2str(PID {', '.join(map(str, others))})을 씁니다 — GUI 가 상태는 못 봅니다")
            self.sig_log.emit(f"[NTRIP] 이미 str2str 이 돌고 있어 새로 띄우지 않습니다 (PID {others}) — 같은 계정으로 둘이면 서로 끊깁니다")
            return
        if not cfg.get("user") or not cfg.get("password"):
            self._set("err", f"NTRIP: 계정이 없습니다 — {CONFIG} 에 user · password 를 넣으세요")
            self.sig_log.emit(f"[NTRIP] ✖ 계정 없음 — {CONFIG}")
            return
        dev = Path("/dev") / cfg["serial"]
        if not dev.exists():
            self._set("err", f"NTRIP: 시리얼 {dev} 가 없습니다 — USB-시리얼 케이블 확인")
            self.sig_log.emit(f"[NTRIP] ✖ {dev} 없음 — {RESTART_S:.0f}초 뒤 다시 시도")
            self._restart.start(int(RESTART_S * 1000))
            return
        lat, lon, hgt = cfg["position"]
        fix = last_fix() if cfg.get("position_from_gnss", True) else None
        if fix:
            lat, lon = fix
        url = f"ntrip://{cfg['user']}:{cfg['password']}@{cfg['host']}:{cfg['port']}/{cfg['mount']}"
        args = ["-in", url, "-p", f"{lat:.5f}", f"{lon:.5f}", f"{float(hgt):.1f}", "-n", "1000",
                "-out", f"serial://{cfg['serial']}:{cfg['baud']}:8:n:1:off"]
        self.sig_log.emit(f"[NTRIP] str2str 시작 — {cfg['host']}/{cfg['mount']} → /dev/{cfg['serial']} "
                          f"{cfg['baud']} baud · 위치 {lat:.5f}, {lon:.5f} ({'GNSS 마지막 위치' if fix else '기본 좌표'})")
        self._started = time.time()
        self._last_bytes, self._bytes_t = -1, time.time()
        ssid = current_wifi()
        if ssid and ssid in (cfg.get("blocked_wifi") or []):
            self._set("err", network_hint(cfg, ssid), "network")
            self.sig_log.emit(f"[NTRIP] ✖ 와이파이 '{ssid}' 는 NTRIP 포트를 막습니다 — 휴대폰 핫스팟에 연결하세요 "
                              "(str2str 은 켜 두고, 망이 바뀌면 알아서 붙습니다)")
        else:
            self._set("wait", "NTRIP: 캐스터에 연결 중…")
        self._probe_network(cfg)
        self.proc.start("str2str", args)
        self._watch.start()

    # --- 상태 ---
    def _set(self, level, text, kind=""):
        """kind: "network" = 망 문제(핫스팟으로 바꾸라는 안내 창을 띄울 것)."""
        if (level, text) != (self.status["level"], self.status["text"]):
            self.status = {"level": level, "text": text, "kind": kind}
            self.sig_status.emit()

    def _probe_network(self, cfg):
        def work():
            self._sig_net.emit(caster_reachable(cfg["host"], cfg["port"]), current_wifi())
        threading.Thread(target=work, daemon=True).start()

    def _on_net(self, ok, ssid):
        self._net_ok, self._ssid = ok, ssid
        if not ok and self._want:
            cfg = load_config()
            self._set("err", network_hint(cfg, ssid), "network")
            self.sig_log.emit(f"[NTRIP] ✖ 캐스터 {cfg['host']}:{cfg['port']} 에 TCP 연결 안 됨 (와이파이 '{ssid or '-'}') "
                              "— 내부망 와이파이는 NTRIP 을 막습니다. 휴대폰 핫스팟에 연결하세요")

    def _on_output(self):
        self._buf += bytes(self.proc.readAllStandardOutput()).decode("utf-8", "replace")
        while True:
            cut = min([i for i in (self._buf.find("\n"), self._buf.find("\r")) if i >= 0], default=-1)
            if cut < 0:
                break
            line, self._buf = self._buf[:cut].strip(), self._buf[cut + 1:]
            if line:
                self._line(line)

    def _line(self, line):
        m = _STATUS.search(line)
        if not m:
            self.sig_log.emit(f"[NTRIP] {line}")
            return
        flags, nbytes, bps = m.group(1), int(m.group(2)), int(m.group(3))
        now = time.time()
        if nbytes != self._last_bytes:
            self._last_bytes, self._bytes_t = nbytes, now
        inp, out = flags[0], flags[1]
        if out in "WE" or (out == "-" and now - self._started > NO_INPUT_S):
            self._set("err", f"NTRIP: 시리얼 출력 문제 [{flags}] — USB-시리얼 · 크로스 점퍼(2↔3, 3↔2, 5↔5) 확인")
        elif inp != "C":
            if now - self._started > NO_INPUT_S or self._net_ok is False:
                if self.status.get("kind") != "network" or now - self._started > NO_INPUT_S:
                    self._probe_network(load_config())          # 망이 바뀌었는지 다시 본다
                if self._net_ok is False:
                    self._set("err", network_hint(load_config(), self._ssid), "network")
                else:
                    self._set("err", f"NTRIP: 캐스터가 접속을 안 받습니다 [{flags}] — 계정(ntrip.yaml) · 같은 계정으로 "
                                     "다른 곳에서 접속 중인지 확인")
            else:
                self._set("wait", "NTRIP: 캐스터에 연결 중…")
        elif now - self._bytes_t > STUCK_S or bps == 0:
            self._set("warn", f"NTRIP: 연결은 됐는데 보정 데이터가 안 들어옴 [{flags}] (VRS 는 위치를 받아야 보냄)")
        else:
            self._set("ok", f"NTRIP 보정 ✓ {bps / 1000:.1f} kbps")
        if now - self._last_logged > 60:                   # 로그는 1분에 한 줄
            self._last_logged = now
            self.sig_log.emit(f"[NTRIP] [{flags}] {nbytes} B · {bps} bps")

    def _check(self):
        # str2str 이 아무것도 안 찍는 경우 (출력이 멈춤)
        if self.proc.state() == QProcess.Running and self._last_bytes >= 0 and time.time() - self._bytes_t > 30:
            self._set("warn", "NTRIP: 30초 넘게 보정 데이터가 안 늘어남")

    def _on_finished(self, code, _status):
        self._watch.stop()
        if not self._want or getattr(self, "_relaunch_pending", False):
            return
        self.sig_log.emit(f"[NTRIP] ✖ str2str 이 끝났습니다 (exit {code}) — {RESTART_S:.0f}초 뒤 다시 켭니다")
        self._set("err", f"NTRIP: str2str 이 멈춤 (exit {code}) — 다시 켜는 중")
        self._restart.start(int(RESTART_S * 1000))
