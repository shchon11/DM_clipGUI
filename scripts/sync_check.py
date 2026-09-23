#!/usr/bin/env python3
# sync_check.py — 가시광 카드의 '동기 검증' 탭 (docs/gnss_pps_sync_gui.md §6).
#
# 운행 직전에 "카메라들이 정말 같은 트리거로 찍고 있나" 를 GUI 에서 확인한다. 계산은 FLIR 리포의
# scripts/check_trigger_sync.py 가 한다 — 메시지 타입(FlirMetadata)이 그 워크스페이스에 있어서, 런치와
# 똑같이 그 워크스페이스를 소싱해 돌리고 --json 결과를 표로 보여준다. 판정선은 sensors.yaml 의 sync_check.
#
# 카메라 PTP 없이 GPIO 트리거만 쓰면 camera_timestamp_ns 는 카메라마다 전원을 넣은 뒤부터 세는
# 카운터라 서로 직접 비교할 수 없다. 스크립트는 두 카메라 카운터의 차이가 직선(발진기 ppm 차이)을
# 따르는지, 그 직선에서 얼마나 벗어나는지(= 트리거 정렬 오차)를 잰다.
#
# 단독 실행:  python3 sync_check.py <check_trigger_sync --json 출력 파일>   — 판정만 출력

import json
import sys

from PyQt5.QtCore import QProcess, Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView, QHBoxLayout, QLabel, QPushButton, QSpinBox, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget)

import sensor_launcher
import ui_theme
from sensor_discovery import expand

OK, WARN, FAIL = "ok", "warn", "fail"
_RANK = {None: 0, OK: 0, WARN: 1, FAIL: 2}
_COLOR = {OK: ui_theme.OK, WARN: ui_theme.WARN, FAIL: ui_theme.ERR}
_MARK = {OK: "●", WARN: "▲", FAIL: "✕", None: "·"}

DEFAULTS = {"duration_s": 20, "jitter_warn_us": 10.0, "jitter_fail_us": 1000.0,
            "arrival_warn_ms": 10.0, "rate_tolerance": 0.02}

COLUMNS = ["카메라", "동기 방식", "프레임", "레이트", "유실", "빠진 라운드", "도착 차이", "지터", "드리프트", "노출"]
COLUMN_TIPS = {
    "유실": "camera_frame_id 가 건너뛴 수 — 카메라가 아예 보내지 않은 프레임",
    "빠진 라운드": "기준 카메라가 찍은 트리거 중 이 카메라 프레임이 PC 에 안 온 수 (유실 + 전송 중 빠짐)",
    "도착 차이": "PC 도착 시각의 기준 카메라 대비 차이(중앙값). 트리거를 받는 카메라는 수 ms 안 —\n"
               "트리거를 못 받고 자유 실행하는 카메라는 트리거와 무관한 위상(최대 반 주기)에 있다",
    "지터": "기준 카메라와의 카운터 차이에서 오프셋·드리프트를 빼고 남은 흔들림(표준편차) = 트리거 정렬 오차.\n"
           "두 카메라의 흔들림이 합쳐진 값이다. 비절연 입력(Line3) 1 µs 안, 옵토(Line0) µs 단위",
    "드리프트": "두 카메라 발진기의 속도 차이. 수십 ppm 안이면 정상 — 트리거와는 무관하다",
    "노출": "image_raw/metadata 의 exposure_time_us 중앙값",
}


def settings(group):
    return dict(DEFAULTS, **((group or {}).get("sync_check") or {}))


def script_path(group):
    """검증 스크립트의 실제 경로. 레지스트리에 없거나 파일이 없으면 None."""
    raw = ((group or {}).get("sync_check") or {}).get("script")
    path = expand(raw) if raw else None
    return path if path and path.is_file() else None


def _level(value, warn, fail):
    if value is None:
        return None
    return OK if value < warn else (WARN if value < fail else FAIL)


def judge(report, cfg, modes, stamp=None):
    """(rows, summary) — rows: [[(글, 등급, 설명)]], summary: [(등급, 글)].

    modes: {네임스페이스: 동기 방식 표시} — 자유 실행으로 둔 카메라는 정렬이 안 맞는 게 정상이라 판정하지 않는다.
    stamp: {"mode": timestamp.mode, "grid_hz": timestamp.trigger_grid_hz} — 지금 설정 (격자 위상 해석용)
    """
    cams = report.get("cameras") or {}
    reference = report.get("reference")
    rates = [c["rate_hz"] for c in cams.values() if c.get("rate_hz")]
    rig_rate = sorted(rates)[len(rates) // 2] if rates else 0.0
    rounds = report.get("rounds") or {}
    matched = rounds.get("matched") or 0

    rows, silent, late, lossy = [], [], [], []
    for name, c in cams.items():
        mode = modes.get(name, "")
        free = mode == "자유 실행"
        cells = [(name + ("  (기준)" if name == reference else ""), None, ""), (mode, None, "")]
        if c.get("frames", 0) < 2:
            silent.append(name)
            cells.append(("0", FAIL, "이 카메라에서 메타데이터가 안 왔다"))
            rows.append(cells + [("", None, "")] * (len(COLUMNS) - len(cells)))
            continue
        cells.append((str(c["frames"]), None, ""))
        rate = c.get("rate_hz") or 0.0
        off = rig_rate and abs(rate - rig_rate) / rig_rate > cfg["rate_tolerance"]
        cells.append((f"{rate:.2f} Hz", WARN if off else OK,
                      f"리그 중앙값 {rig_rate:.2f} Hz 와 다름" if off else ""))
        lost = c.get("lost", 0)
        cells.append((str(lost), WARN if lost else OK, ""))
        missing = c.get("missing", 0)
        many = matched and missing > 0.05 * matched
        cells.append((str(missing), WARN if many else (OK if not missing else None), ""))
        if lost or many:
            lossy.append(name)
        if name == reference or c.get("jitter_us") is None:
            cells += [("—", None, "기준 카메라" if name == reference else "짝지은 프레임이 너무 적다")] * 3
        else:
            arrival = c["arrival_ms"]
            far = abs(arrival) >= cfg["arrival_warn_ms"]
            if far and not free:
                late.append(name)
            cells.append((f"{arrival:+.2f} ms", None if free else (WARN if far else OK), ""))
            jitter = c["jitter_us"]
            note = f"  (짝이 한 주기 어긋난 {c['mispaired']}개 제외)" if c.get("mispaired") else ""
            cells.append((f"{jitter:.2f} µs", None if free else _level(jitter, cfg["jitter_warn_us"],
                                                                         cfg["jitter_fail_us"]), note.strip()))
            cells.append((f"{c['drift_ppm']:+.1f} ppm", None, ""))
        exposure = c.get("exposure_us")
        cells.append((f"{exposure / 1000:.1f} ms" if exposure is not None else "", None, ""))
        rows.append(cells)

    summary = []
    if report.get("error"):
        summary.append((FAIL, report["error"]))
    if silent:
        summary.append((FAIL, f"프레임 0장: {', '.join(silent)} — HW 트리거면 입력 라인 · 트리거 엣지 · 배선부터 "
                              "확인 (트리거가 안 들어오면 카메라는 기다리기만 한다)"))
    jitters = [(c["jitter_us"], n) for n, c in cams.items()
               if c.get("jitter_us") is not None and modes.get(n) != "자유 실행"]
    if jitters:
        worst, who = max(jitters)
        grade = _level(worst, cfg["jitter_warn_us"], cfg["jitter_fail_us"])
        verdict = {OK: "정상", WARN: "느슨함", FAIL: "트리거가 안 맞음"}[grade]
        summary.append((grade, f"트리거 정렬: 최악 지터 {worst:.2f} µs ({who}) — {verdict} "
                               f"(경고 {cfg['jitter_warn_us']:g} µs · 실패 {cfg['jitter_fail_us']:g} µs)"))
    if late:
        summary.append((WARN, f"도착이 {cfg['arrival_warn_ms']:g} ms 넘게 어긋남: {', '.join(late)} — 트리거를 못 "
                              "받고 자유 실행 중일 수 있다 (동기 방식 · 배선 확인)"))
    if lossy:
        summary.append((WARN, f"프레임이 빠지는 카메라: {', '.join(lossy)}"))
    if report.get("period_ns"):
        summary.append((None, f"트리거 주기 {report['period_ns'] / 1e6:.3f} ms ({report['rate_hz']:.2f} Hz) · "
                              f"라운드 {matched} 중 모든 카메라 {rounds.get('full', 0)}"))
    if rounds.get("header_spread_median_ms") is not None:
        median, worst = rounds["header_spread_median_ms"], rounds["header_spread_worst_ms"]
        if worst < 0.1:
            summary.append((OK, f"같은 트리거의 header.stamp 가 카메라끼리 일치 (최악 {worst * 1000:.0f} µs) — "
                                "기록되는 시각이 노출 시각이다"))
        else:
            summary.append((WARN if median >= 1.0 else None,
                            f"같은 트리거의 header.stamp 차이: 중앙 {median:.2f} ms · 최악 {worst:.2f} ms — "
                            "노출은 위 지터만큼 맞아도 기록되는 시각은 이만큼 다르다. header.stamp 기준이 host(PC 도착 "
                            "시각)면 설정 → 동기 → 'header.stamp 기준' 을 camera_latched 로"))
    grid_line = _grid_verdict(report.get("grid"), stamp or {})
    if grid_line:
        summary.append(grid_line)
    return rows, summary


def _grid_verdict(grid, stamp):
    """header.stamp 가 1/N 초 격자의 어디에 있나 — '트리거 격자 스냅' 을 켜도 되는지 정하는 측정.

    header.stamp 가 PC 시계 위의 노출 시작(camera_latched, 스냅 끔)이고 PC 시계가 GNSS 에 맞춰져 있을 때만 뜻이
    있다. PPS 에 묶인 펄스면 위상이 고정되고 흐르지 않는다. 20초로는 드리프트 잡음이 ±2 ppm 쯤이라 10 ppm 을
    판정선으로 둔다 (따로 도는 발진기는 보통 10~100 ppm 틀린다).
    """
    if not grid:
        return None
    phase, jitter, drift = grid["phase_ms"], grid["jitter_ms"], grid["drift_ppm"]
    text = (f"header.stamp 의 {grid['hz']:g} Hz 격자 위상 {phase:+.2f} ms · 흔들림 {jitter:.2f} ms · "
            f"드리프트 {drift:+.1f} ppm")
    if stamp.get("mode") != "camera_latched":
        return None, text + " — header.stamp 기준이 camera_latched 일 때만 뜻이 있다"
    if stamp.get("grid_hz"):
        return None, text + " — 격자 스냅이 켜져 있어 0 이 당연하다. 스냅이 맞는지는 카메라 로그의 격자 잔차로 본다"
    note = " (PC 시계가 GNSS(PTP)에 맞춰져 있을 때만 믿을 수 있다)"
    if jitter > 1.0 or abs(drift) > 10.0:
        return WARN, (text + " — 트리거 펄스가 PC 시계의 격자에 묶여 있지 않다: 펄스가 PPS 에 동기되지 않았거나 PC 시계가 "
                      "GNSS 에 안 맞음. '트리거 격자 스냅' 을 켜지 마세요" + note)
    if abs(phase) <= 1.0:
        return OK, (text + f" — 펄스가 격자 위에 있다. '트리거 격자 스냅' 을 {grid['hz']:g} 으로 켜도 된다" + note)
    return OK, (text + f" — 펄스가 격자에서 일정하게 {phase:+.2f} ms 밀려 있다. timestamp.trigger_grid_offset_ns 를 "
                f"{round(phase * 1e6)} 로 두고 스냅을 켤 수 있다 (래치 측정 오차 1 ms 미만 포함)" + note)


class SyncCheckPanel(QWidget):
    """[동기 검증] — 실행 중인 카메라에서 N초 모아 판정한다."""

    def __init__(self, group, parent=None):
        super().__init__(parent)
        self.group = group
        self.cfg = settings(group)
        self.script = script_path(group)
        self.modes = {}              # {네임스페이스: 동기 방식 표시} — 지금 떠 있는 카메라
        self.stamp = {}              # {"mode", "grid_hz"} — header.stamp 설정 (격자 위상 해석용)
        self.running = False
        self._left = 0
        self._stdout = b""
        self._stderr = b""
        self.proc = None
        self._tick = QTimer(self, interval=1000, timeout=self._countdown)
        self._build()
        self._update_controls()

    def _build(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)
        row = QHBoxLayout()
        self.btn = QPushButton("동기 검증 시작")
        ui_theme.set_variant(self.btn, "primary")
        self.btn.clicked.connect(self._toggle)
        self.duration = QSpinBox()
        self.duration.setRange(5, 300)
        self.duration.setSuffix(" 초")
        self.duration.setValue(int(self.cfg["duration_s"]))
        self.duration.setToolTip("이만큼 모아서 판정합니다. 트리거가 1 Hz 면 20초로는 20라운드뿐이라 늘리세요.")
        self.status = QLabel("")
        self.status.setObjectName("Hint")
        self.status.setWordWrap(True)
        row.addWidget(self.btn)
        row.addWidget(QLabel("수집"))
        row.addWidget(self.duration)
        row.addWidget(self.status, 1)
        layout.addLayout(row)

        self.summary = QLabel("")
        self.summary.setWordWrap(True)
        self.summary.setTextFormat(Qt.RichText)
        self.summary.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.summary)

        self.table = QTableWidget(0, len(COLUMNS))
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        for index, title in enumerate(COLUMNS):
            item = QTableWidgetItem(title)
            item.setToolTip(COLUMN_TIPS.get(title, ""))
            self.table.setHorizontalHeaderItem(index, item)
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table, 1)

    # --- 대상 ---

    def set_targets(self, modes, running, stamp=None):
        """modes: {네임스페이스: 동기 방식 표시} — 이번 기동에 들어간 카메라. stamp: header.stamp 설정."""
        self.modes = dict(modes)
        self.running = running
        self.stamp = dict(stamp or {})
        self._update_controls()

    def _update_controls(self):
        busy = self.proc is not None
        self.duration.setEnabled(not busy)
        if busy:
            return
        self.btn.setText("동기 검증 시작")
        if not self.script:
            raw = (self.group.get("sync_check") or {}).get("script", "(sensors.yaml sync_check.script 없음)")
            self.btn.setEnabled(False)
            self.status.setText(f"검증 스크립트가 없습니다: {raw} — FLIR 리포가 이 스크립트가 있는 브랜치인지 확인하세요")
            return
        ready = self.running and len(self.modes) >= 2
        self.btn.setEnabled(ready)
        if not self.running:
            self.status.setText("카메라를 기동하면 검증할 수 있습니다.")
        elif len(self.modes) < 2:
            self.status.setText("이번 기동의 카메라가 2대 이상이어야 비교할 수 있습니다.")
        else:
            counts = {}
            for mode in self.modes.values():
                counts[mode] = counts.get(mode, 0) + 1
            self.status.setText(f"대상 {len(self.modes)}대 — " +
                                " · ".join(f"{mode} {n}" for mode, n in counts.items()))

    # --- 실행 ---

    def _toggle(self):
        if self.proc is not None:
            self.proc.kill()
            return
        seconds = self.duration.value()
        tokens = ["python3", str(self.script), "--json", "--duration", str(seconds),
                  "--jitter-warn-us", str(self.cfg["jitter_warn_us"]),
                  "--jitter-fail-us", str(self.cfg["jitter_fail_us"]),
                  "--arrival-warn-ms", str(self.cfg["arrival_warn_ms"]),
                  "--cameras", *sorted(self.modes)]
        self.proc = QProcess(self)
        self.proc.setProcessEnvironment(sensor_launcher.clean_environment())
        self.proc.setWorkingDirectory(str(sensor_launcher.work_dir(self.group)))
        self.proc.readyReadStandardOutput.connect(self._read_out)
        self.proc.readyReadStandardError.connect(self._read_err)
        self.proc.finished.connect(self._finished)
        self._stdout = self._stderr = b""
        self._left = seconds
        self.btn.setText("중지")
        self._update_controls()
        self.status.setText(f"수집 중… {self._left}초")
        self._tick.start()
        self.proc.start("bash", ["-c", sensor_launcher.sourced_script(self.group, tokens)])

    def _countdown(self):
        self._left -= 1
        self.status.setText(f"수집 중… {self._left}초" if self._left > 0 else "계산 중…")

    def _read_out(self):
        self._stdout += bytes(self.proc.readAllStandardOutput())

    def _read_err(self):
        self._stderr += bytes(self.proc.readAllStandardError())

    def _finished(self, code, _status):
        self._tick.stop()
        self.proc.deleteLater()
        self.proc = None
        self._update_controls()
        lines = [ln for ln in self._stdout.decode("utf-8", "replace").splitlines() if ln.startswith("{")]
        try:
            report = json.loads(lines[-1]) if lines else None
        except ValueError:
            report = None
        if report is None:
            tail = self._stderr.decode("utf-8", "replace").strip().splitlines()[-4:]
            self.status.setText(f"검증이 결과 없이 끝났습니다 (종료 코드 {code})")
            self.status.setToolTip("\n".join(tail))
            self._show_summary([(FAIL, "스크립트 출력: " + (" / ".join(tail) or "(없음)"))])
            return
        self.status.setToolTip("")
        rows, summary = judge(report, self.cfg, self.modes, self.stamp)
        self._show_summary(summary)
        self._show_rows(rows)
        self.status.setText(f"{int(report.get('duration_s', 0))}초 수집 결과")

    def _show_summary(self, items):
        html = []
        for grade, text in items:
            color = _COLOR.get(grade, ui_theme.MUTED)
            html.append(f"<span style='color:{color}'>{_MARK[grade]}</span>&nbsp;{text}")
        self.summary.setText("<br>".join(html))

    def _show_rows(self, rows):
        self.table.setRowCount(len(rows))
        for r, cells in enumerate(rows):
            for c, (text, grade, tip) in enumerate(cells):
                item = QTableWidgetItem(text)
                if grade in (WARN, FAIL):
                    item.setForeground(QColor(_COLOR[grade]))
                if tip:
                    item.setToolTip(tip)
                if c >= 2:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.table.setItem(r, c, item)
        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setStretchLastSection(True)


def main(argv):
    if len(argv) < 2:
        print(__doc__ or "usage: sync_check.py <report.json>")
        return 1
    report = json.loads(open(argv[1], encoding="utf-8").read())
    rows, summary = judge(report, dict(DEFAULTS), {})
    for grade, text in summary:
        print(_MARK[grade], text)
    for cells in rows:
        print("  " + " | ".join(f"{text}{'!' if grade in (WARN, FAIL) else ''}" for text, grade, _ in cells))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
