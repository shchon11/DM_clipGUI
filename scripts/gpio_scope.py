#!/usr/bin/env python3
# gpio_scope.py — Blackfly S GPIO 핀 신호를 로직 분석기처럼 실시간 파형으로 본다.
#
# gpio_watch.py 와 같은 방법(GVCP READREG 로 LineStatusAll 을 0.7 ms 간격쯤 계속 읽기, 읽기 전용)으로 모은 값을
# 카메라마다 · 라인마다 한 줄씩 흐르는 파형으로 그린다. 선을 옮기면서 펄스가 들어오는지 바로 보려는 것.
#
# 카메라가 알려 주는 건 **전압이 아니라 카메라 입력 회로의 판정(HIGH/LOW)** 이다 — GPIO 에 전압을 재는 회로
# (ADC)가 없다. 비절연 입력(Line3 = 핀1 초록, Line2 = 핀3 빨강)은 2.6 V 이상이면 HIGH · 1.4 V 이하면 LOW,
# 옵토 입력(Line0 = 핀2 검정)은 핀2→핀5 로 전류가 흐르면 HIGH 다. 핀 배치는 gpio_watch.py 머리말.
# 실제 전압 파형이 필요하면 멀티미터 · 오실로스코프로 잰다.
#
# 시각은 PC 가 응답을 받은 시각이라 ±1 ms 쯤 흔들린다 — 1 ms 펄스가 보이는지, 몇 Hz 인지, 카메라끼리 같은
# 순간에 오는지는 충분히 보이지만 µs 단위 정렬은 못 잰다 (그건 동기 검증 탭).
#
# 사용:
#   python3 gpio_scope.py                        카메라 NIC 의 Blackfly 전부 (입력 라인 Line0 · Line2 · Line3 만)
#   python3 gpio_scope.py --text [IP ...]        창 없이 터미널 표로 (= gpio_watch.py)
#   python3 gpio_scope.py 192.168.1.1            한 대 (네 라인 전부)
#   python3 gpio_scope.py --lines 0,1,2,3 ...    볼 라인 지정
#   python3 gpio_watch.py --scope ...            같은 것

import argparse
import multiprocessing
import select
import signal
import socket
import struct
import sys
import time
from multiprocessing import shared_memory
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QHBoxLayout, QLabel, QPushButton,
                             QScrollArea, QSizePolicy, QVBoxLayout, QWidget)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gpio_watch  # noqa: E402

RING = 1 << 15                 # 카메라마다 최근 샘플 수 (~0.8 ms 간격이면 약 25초)
WINDOWS = [("20 ms", 0.02), ("50 ms", 0.05), ("200 ms", 0.2), ("1 s", 1.0), ("5 s", 5.0)]
STATS_S = 2.0                  # 오른쪽 통계(HIGH 비율 · Hz · 폭)를 계산하는 최근 구간
STALE_S = 0.2                  # 이보다 오래 응답이 없으면 '응답 없음'
# 레벨마다 색을 나눈다 — 같은 색 선 하나로 위/아래만 다르면 HIGH · LOW 가 잘 안 갈린다
HIGH_C, LOW_C, EDGE_C = QColor("#22c55e"), QColor("#60a5fa"), QColor("#e2e8f0")


class Rings:
    """카메라 N대의 링 버퍼 (공유 메모리) — 읽기 프로세스가 쓰고 화면이 읽는다.

    읽기를 화면과 같은 프로세스의 스레드로 돌리면 14대 + 그리기가 GIL 을 나눠 써서 샘플 간격이 2 ms 넘게
    벌어진다 (1 ms 펄스를 놓친다). 그래서 읽기는 별도 프로세스 하나가 맡는다."""

    def __init__(self, n_cams, name=None):
        sizes = [n_cams * RING * 8, n_cams * RING * 4, n_cams * 8, n_cams * 8, n_cams * 8]
        self.shm = shared_memory.SharedMemory(name=name, create=name is None, size=sum(sizes))
        buf, off = self.shm.buf, 0
        self.t = np.ndarray((n_cams, RING), np.float64, buf, off)
        off += sizes[0]
        self.v = np.ndarray((n_cams, RING), np.uint32, buf, off)
        off += sizes[1]
        self.n = np.ndarray(n_cams, np.int64, buf, off)          # 지금까지 쌓은 개수 (링 위치 = n % RING)
        off += sizes[2]
        self.last = np.ndarray(n_cams, np.float64, buf, off)     # 마지막 샘플 시각
        off += sizes[3]
        self.err = np.ndarray(n_cams, np.int64, buf, off)        # 0 정상 · 1 응답 없음 · 그 밖 GVCP 상태
        if name is None:
            self.n[:] = 0
            self.last[:] = 0
            self.err[:] = 0

    def close(self, unlink=False):
        # 공유 메모리를 가리키는 배열을 먼저 놓아야 닫힌다
        self.t = self.v = self.n = self.last = self.err = None
        self.shm.close()
        if unlink:
            self.shm.unlink()


def _sample_loop(ips, shm_name, stop):
    """(별도 프로세스) 카메라마다 READREG 를 하나씩 띄워 두고, 응답이 오면 적고 바로 다음 것을 보낸다.
    50 ms 안에 응답이 없으면 다시 보낸다. 카메라들이 동시에 답하니 한 대를 읽는 것과 간격이 거의 같다."""
    rings = Rings(len(ips), shm_name)
    index = {ip: i for i, ip in enumerate(ips)}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    req, sent, count = [0] * len(ips), [0.0] * len(ips), [0] * len(ips)

    def send(i):
        req[i] = req[i] % 0xFFFF + 1
        try:
            sock.sendto(struct.pack(">BBHHHI", 0x42, 0x01, 0x0080, 4, req[i], gpio_watch.LINE_STATUS_ALL),
                        (ips[i], gpio_watch.net_tools.GVCP_PORT))
        except OSError:
            pass
        sent[i] = time.monotonic()

    for i in range(len(ips)):
        send(i)
    try:
        while not stop.is_set():
            ready, _, _ = select.select([sock], [], [], 0.01)
            while ready:
                try:
                    data, (src, _port) = sock.recvfrom(64)
                except BlockingIOError:
                    break
                i = index.get(src)
                if i is None or len(data) < 8:
                    continue
                status, _ack, _length, ack_id = struct.unpack(">HHHH", data[:8])
                if ack_id != req[i]:
                    continue
                now = time.monotonic()
                if status == 0 and len(data) >= 12:
                    k = count[i] % RING
                    rings.t[i, k] = now
                    rings.v[i, k] = struct.unpack(">I", data[8:12])[0]
                    count[i] += 1
                    rings.n[i] = count[i]              # 칸을 채운 뒤에 개수를 올린다 (읽는 쪽은 개수부터 본다)
                    rings.last[i] = now
                    rings.err[i] = 0
                else:
                    rings.err[i] = status or 2
                send(i)
            now = time.monotonic()
            for i in range(len(ips)):
                if now - sent[i] > 0.05:
                    rings.err[i] = 1
                    send(i)
    finally:
        sock.close()
        rings.close()


class CameraFeed:
    """링 버퍼에서 카메라 한 대 몫을 읽는 쪽."""

    def __init__(self, rings, i):
        self.rings, self.i = rings, i

    @property
    def n(self):
        return int(self.rings.n[self.i])

    @property
    def last_t(self):
        return float(self.rings.last[self.i])

    @property
    def error(self):
        code = int(self.rings.err[self.i])
        return None if code == 0 else "응답 없음" if code == 1 else f"GVCP 상태 0x{code:04x}"

    def window(self, t0, t1):
        """(시각들, 값들) — [t0, t1] 안의 샘플과 그 직전 샘플 하나 (창 왼쪽 끝 상태를 알려고).
        링 전체를 복사하지 않고 끝에서부터 필요한 만큼만 (모자라면 두 배씩 늘린다)."""
        n = self.n
        have = min(n, RING)
        if have == 0:
            return np.zeros(0), np.zeros(0, dtype=np.uint32)
        need = min(have, int(max(self.last_t - t0, 0) * 4000) + 16)      # 초당 4000회보다 빨리 읽지는 못한다
        while True:
            pos = np.arange(n - need, n) % RING
            t = self.rings.t[self.i, pos]
            if t[0] <= t0 or need >= have:
                break
            need = min(have, need * 2)
        v = self.rings.v[self.i, pos]
        lo = max(int(np.searchsorted(t, t0)) - 1, 0)
        hi = int(np.searchsorted(t, t1, side="right"))
        return t[lo:hi], v[lo:hi]


def lane_stats(t, bits):
    """한 라인의 최근 통계 — gpio_watch.analyse 그대로 (HIGH 비율 · 바뀐 횟수 · 펄스 Hz · 폭)."""
    if len(t) < 2:
        return None
    return gpio_watch.analyse(list(zip(t.tolist(), bits.tolist())), 0)


class ScopeView(QWidget):
    """카메라 × 라인 파형. 한 줄 = 한 라인 (왼쪽 이름 · 가운데 파형 · 오른쪽 통계)."""

    LABEL_W = 330
    STAT_W = 250
    LANE_H = 40
    CAM_GAP = 10

    def __init__(self, targets, names, lines, samplers):
        super().__init__()
        self.targets, self.names, self.lines, self.samplers = targets, names, lines, samplers
        self.span = 0.2
        self.t_end = time.monotonic()
        self.on_edge = False                     # True 면 가운데가 트리거 엣지 (눈금을 엣지 기준으로)
        self.stats = {}                          # (ip, bit) -> analyse 결과
        self.setMinimumHeight(self._height())
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setAttribute(Qt.WA_OpaquePaintEvent)

    def set_lines(self, lines):
        """보여 줄 라인을 바꾼다 (창의 'Line1 출력도 보기')."""
        self.lines = sorted(lines)
        self.setMinimumHeight(self._height())
        self.update_stats()
        self.update()

    def _height(self):
        return len(self.targets) * (len(self.lines) * self.LANE_H + self.CAM_GAP + 20) + 24

    def sizeHint(self):
        return self.minimumSizeHint().expandedTo(self.minimumSize())

    def update_stats(self):
        now = time.monotonic()
        for ip, _serial in self.targets:
            t, v = self.samplers[ip].window(now - STATS_S, now)
            for bit in self.lines:
                self.stats[(ip, bit)] = lane_stats(t, (v >> bit) & 1)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, False)
        bg, grid, text, muted = QColor("#0f172a"), QColor("#1e293b"), QColor("#e2e8f0"), QColor("#94a3b8")
        p.fillRect(self.rect(), bg)
        x0, x1 = self.LABEL_W, self.width() - self.STAT_W
        width = max(x1 - x0, 10)
        t1 = self.t_end
        t0 = t1 - self.span
        small = QFont(self.font())
        small.setPointSizeF(self.font().pointSizeF() * 0.9)

        # 시간 눈금 (위쪽) — 오른쪽 끝이 지금, 엣지 고정이면 가운데가 엣지(0)
        p.setPen(muted)
        p.setFont(small)
        for k in range(11):
            x = x0 + width * k / 10
            p.setPen(QPen(grid, 1))
            p.drawLine(int(x), 20, int(x), self.height())
            if k % 2 == 0 or (self.on_edge and k == 5):
                p.setPen(muted)
                offset = self.span * ((k - 5) if self.on_edge else (k - 10)) / 10
                text_ms = f"{offset * 1000:+.0f} ms" if self.span < 2 else f"{offset:+.1f} s"
                label = ("엣지" if k == 5 else text_ms) if self.on_edge else ("지금" if k == 10 else text_ms)
                p.drawText(QRectF(x - 40, 2, 80, 16), Qt.AlignCenter, label)

        y = 24
        for ip, serial in self.targets:
            sampler = self.samplers[ip]
            t, v = sampler.window(t0, t1)
            last = t[-1] if len(t) else None
            stale = sampler.error if (last is None or time.monotonic() - last > STALE_S) else None
            p.setPen(text)
            p.setFont(self.font())
            title = f"{self.names.get(serial, serial or ip)}   {ip}"
            p.drawText(QRectF(8, y, self.width() - 16, 18), Qt.AlignLeft | Qt.AlignVCenter,
                       title + (f"   ✗ {stale}" if stale else ""))
            y += 20
            for bit in self.lines:
                name, pin = gpio_watch.LINES[bit]
                is_input = bit in gpio_watch.INPUTS
                top, bottom = y + 7, y + self.LANE_H - 7
                p.setPen(QPen(grid, 1))
                p.drawLine(x0, y + self.LANE_H, x1, y + self.LANE_H)
                # HIGH · LOW 높이 안내선 (흐린 점선) + 왼쪽 H / L 글자
                for level, color in ((top, HIGH_C), (bottom, LOW_C)):
                    guide = QColor(color)
                    guide.setAlpha(55)
                    p.setPen(QPen(guide, 1, Qt.DotLine))
                    p.drawLine(x0, level, x1, level)
                p.setFont(small)
                p.setPen(HIGH_C)
                p.drawText(QRectF(x0 - 22, top - 8, 16, 16), Qt.AlignCenter, "H")
                p.setPen(LOW_C)
                p.drawText(QRectF(x0 - 22, bottom - 8, 16, 16), Qt.AlignCenter, "L")
                p.setPen(text if is_input else muted)
                p.drawText(QRectF(16, y, 230, self.LANE_H), Qt.AlignLeft | Qt.AlignVCenter, f"{name}  {pin}")
                bits = (v >> bit) & 1 if len(t) else None
                if bits is not None and not stale:
                    self._chip(p, QRectF(248, y + self.LANE_H / 2 - 9, 50, 18), int(bits[-1]), is_input, small)
                if bits is not None:
                    self._trace(p, t, bits, t0, t1, x0, width, top, bottom, is_input)
                self._stat(p, self.stats.get((ip, bit)), x1 + 10, y, is_input, small)
                y += self.LANE_H
            y += self.CAM_GAP
        p.end()

    @staticmethod
    def _chip(p, rect, level, is_input, font):
        """지금(가장 최근 샘플) 상태 — 초록 HIGH / 파랑 LOW 딱지."""
        color = QColor(HIGH_C if level else LOW_C)
        if not is_input:
            color.setAlpha(120)
        p.setPen(Qt.NoPen)
        p.setBrush(color)
        p.drawRoundedRect(rect, 4, 4)
        p.setBrush(Qt.NoBrush)
        p.setPen(QColor("#0f172a"))
        bold = QFont(font)
        bold.setBold(True)
        p.setFont(bold)
        p.drawText(rect, Qt.AlignCenter, "HIGH" if level else "LOW")
        p.setFont(font)

    def _trace(self, p, t, bits, t0, t1, x0, width, top, bottom, is_input):
        """샘플 -> 픽셀 칸마다 (LOW만 / HIGH만 / 둘 다) 로 줄여 계단 파형으로. HIGH 는 위 초록 선 + 칠함,
        LOW 는 아래 파랑 선, 바뀌는 순간은 흰 세로줄, 한 칸 안에서 바뀌었으면 흰 막대.
        샘플 간격(~0.8 ms)이 칸보다 넓으면 빈 칸은 직전 샘플 값으로 채운다. 마지막 샘플 뒤로는 그리지 않는다."""
        bits = bits.astype(int)
        cols = np.clip(((t - t0) / (t1 - t0) * width).astype(int), 0, width - 1)   # 시각순이라 오름차순
        count = np.bincount(cols, minlength=width)
        high = np.bincount(cols, weights=bits, minlength=width)
        has = count > 0
        state = np.where(high == 0, 0, np.where(high == count, 1, 2))            # 0 LOW · 1 HIGH · 2 둘 다
        last_val = np.zeros(width, dtype=int)                                    # 칸마다 마지막 샘플 값
        ends = np.r_[np.nonzero(np.diff(cols))[0], len(cols) - 1]
        last_val[cols[ends]] = bits[ends]
        idx = np.where(has, np.arange(width), 0)
        np.maximum.accumulate(idx, out=idx)
        filled = np.where(has, state, last_val[idx])
        filled[: cols[0]] = -1
        filled[cols[-1] + 1:] = -1

        change = np.nonzero(np.diff(filled))[0] + 1
        starts, stops = np.r_[0, change], np.r_[change, width]
        high_path, low_path, edge_path, fill, blocks = (QPainterPath(), QPainterPath(), QPainterPath(),
                                                        QPainterPath(), [])
        prev = None
        for a, b in zip(starts.tolist(), stops.tolist()):
            val = filled[a]
            if val < 0 or val == 2:
                if val == 2:
                    blocks.append(QRectF(x0 + a, top, b - a, bottom - top))
                prev = None
                continue
            if prev is not None and prev != val:
                edge_path.moveTo(QPointF(x0 + a, top))
                edge_path.lineTo(QPointF(x0 + a, bottom))
            y = top if val == 1 else bottom
            path = high_path if val == 1 else low_path
            path.moveTo(QPointF(x0 + a, y))
            path.lineTo(QPointF(x0 + b, y))
            if val == 1:
                fill.addRect(QRectF(x0 + a, top, b - a, bottom - top))
            prev = val

        alpha = 255 if is_input else 110                  # 출력 라인(Line1)은 흐리게
        hi, lo, edge = QColor(HIGH_C), QColor(LOW_C), QColor(EDGE_C)
        for c in (hi, lo, edge):
            c.setAlpha(alpha)
        shade = QColor(HIGH_C)
        shade.setAlpha(70 if is_input else 30)
        p.fillPath(fill, shade)
        for rect in blocks:
            p.fillRect(rect, edge)
        p.setPen(QPen(edge, 1))
        p.drawPath(edge_path)
        p.setPen(QPen(lo, 2))
        p.drawPath(low_path)
        p.setPen(QPen(hi, 2))
        p.drawPath(high_path)

    def _stat(self, p, a, x, y, is_input, font):
        p.setFont(font)
        if not a:
            p.setPen(QColor("#94a3b8"))
            p.drawText(QRectF(x, y, self.STAT_W - 14, self.LANE_H), Qt.AlignLeft | Qt.AlignVCenter, "—")
            return
        if a["flips"]:
            if a["high"] >= 0.5:
                width = f" · 폭 {a['low_ms']:.1f} ms" if a["low_ms"] else ""
                text = f"LOW 펄스 {a['fall_hz']:.1f} Hz{width}"
            else:
                width = f" · 폭 {a['high_ms']:.1f} ms" if a["high_ms"] else ""
                text = f"HIGH 펄스 {a['rise_hz']:.1f} Hz{width}"
            color = QColor("#4ade80") if is_input else QColor("#cbd5e1")
        else:
            text = f"늘 {'HIGH' if a['high'] >= 0.5 else 'LOW'} (변화 없음)"
            color = QColor("#fbbf24") if is_input else QColor("#94a3b8")
        p.setPen(color)
        p.drawText(QRectF(x, y, self.STAT_W - 14, self.LANE_H), Qt.AlignLeft | Qt.AlignVCenter, text)


class ScopeWindow(QWidget):
    def __init__(self, targets, names, lines):
        super().__init__()
        self.setWindowTitle("GPIO 스코프 — Blackfly S")
        ips = [ip for ip, _ in targets]
        self.rings = Rings(len(ips))
        # spawn — Qt 가 떠 있는 프로세스를 fork 하지 않는다
        ctx = multiprocessing.get_context("spawn")
        self.stop = ctx.Event()
        self.proc = ctx.Process(target=_sample_loop, args=(ips, self.rings.shm.name, self.stop), daemon=True)
        self.proc.start()
        self.samplers = {ip: CameraFeed(self.rings, i) for i, ip in enumerate(ips)}
        self.view = ScopeView(targets, names, lines, self.samplers)

        top = QHBoxLayout()
        top.addWidget(QLabel("시간 폭"))
        self.span = QComboBox()
        for label, seconds in WINDOWS:
            self.span.addItem(label, seconds)
        self.span.setCurrentIndex(2)
        self.span.currentIndexChanged.connect(lambda: setattr(self.view, "span", self.span.currentData()))
        top.addWidget(self.span)
        self.trig = QCheckBox("떨어지는 엣지에 고정")
        self.trig.setToolTip("가장 최근의 HIGH→LOW 엣지(입력 라인, 첫 번째로 펄스가 보이는 카메라)를 화면 가운데에\n"
                             "고정합니다 — 오실로스코프 트리거처럼 주기 펄스가 멈춰 보입니다. 모든 카메라가 같은 시간축이라\n"
                             "펄스가 카메라끼리 같은 순간에 오는지도 보입니다.")
        top.addWidget(self.trig)
        # 여러 대를 볼 때는 입력 라인만 기본으로 — Line1(핀4)은 카메라가 내보내는 출력이라 외부 신호가 안 보인다
        self.show_out = QCheckBox("Line1(출력)도 보기")
        self.show_out.setToolTip("Line1 = 핀4 흰색 옵토 출력. 카메라가 내보내는 핀이라 라이다 같은 외부 신호는 여기서\n"
                                 "안 보입니다 (master 카메라면 노출 신호가 나옵니다). 그래서 여러 대를 볼 때는 뺍니다.")
        self.show_out.setChecked(1 in lines)
        self.show_out.toggled.connect(
            lambda on: self.view.set_lines(set(self.view.lines) | {1} if on else set(self.view.lines) - {1}))
        top.addWidget(self.show_out)
        self.btn_pause = QPushButton("일시정지")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(lambda on: self.btn_pause.setText("다시 흐르게" if on else "일시정지"))
        top.addWidget(self.btn_pause)
        top.addStretch()
        note = QLabel(f"<span style='color:{HIGH_C.name()};font-weight:bold'>━ HIGH</span> &nbsp;"
                      f"<span style='color:{LOW_C.name()};font-weight:bold'>━ LOW</span> &nbsp;·&nbsp; "
                      "카메라 입력의 판정 (전압계 아님) · 읽기 전용 · 시각 ±1 ms")
        note.setObjectName("Hint")
        top.addWidget(note)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.view)
        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(scroll, 1)

        self.rate = QLabel("")
        self.rate.setObjectName("Hint")
        layout.addWidget(self.rate)
        self._counts = {ip: 0 for ip in self.samplers}
        self._count_t = time.monotonic()

        self.timer = QTimer(self, interval=33, timeout=self._tick)
        self.timer.start()
        self.stats_timer = QTimer(self, interval=500, timeout=self._tick_stats)
        self.stats_timer.start()
        rows = len(targets) * (len(lines) * ScopeView.LANE_H + ScopeView.CAM_GAP + 20) + 110
        self.resize(1400, min(rows, 950))

    def _trigger_time(self, now):
        """가운데에 둘 엣지 시각 — 화면 오른쪽 절반을 채울 만큼 지난 엣지 중 가장 최근 것."""
        half = self.view.span / 2
        for ip, _serial in self.view.targets:
            t, v = self.samplers[ip].window(now - 1.0, now - half)
            if len(t) < 2:
                continue
            for bit in (b for b in self.view.lines if b in gpio_watch.INPUTS):
                bits = (v >> bit) & 1
                falls = np.nonzero((bits[:-1] == 1) & (bits[1:] == 0))[0]
                if len(falls):
                    return t[falls[-1] + 1]
        return None

    def _tick(self):
        if self.btn_pause.isChecked():
            return
        now = time.monotonic()
        edge = self._trigger_time(now) if self.trig.isChecked() else None
        # 오른쪽 끝 = 가장 최근 샘플 (지금으로 두면 아직 안 온 응답만큼 오른쪽이 비어 보인다)
        newest = max((s.last_t for s in self.samplers.values() if s.n), default=now)
        self.view.t_end = edge + self.view.span / 2 if edge is not None else newest
        self.view.on_edge = edge is not None
        self.view.update()

    def _tick_stats(self):
        if not self.btn_pause.isChecked():
            self.view.update_stats()
        now = time.monotonic()
        rates = []
        for ip, s in self.samplers.items():
            rates.append((s.n - self._counts[ip]) / max(now - self._count_t, 1e-3))
            self._counts[ip] = s.n
        self._count_t = now
        if rates:
            gap = 1000 / max(min(rates), 1)
            text = f"{len(rates)}대 · 카메라당 초당 {min(rates):.0f}–{max(rates):.0f}회 읽음 (간격 약 {gap:.1f} ms)"
            # 카메라는 읽기 요청을 한 번에 하나씩 처리한다 — 다른 gpio 창 등이 같은 카메라를 읽으면 서로 반씩 나눈다
            slow = gap > 1.0
            if slow:
                text += ("  ⚠ 1 ms 보다 벌어져 1 ms 펄스를 놓칠 수 있습니다 — 다른 gpio 창 · 프로그램이 같은 카메라를 "
                         "읽고 있는지 확인하세요")
            self.rate.setText(text)
            self.rate.setStyleSheet("color: #d97706;" if slow else "")

    def shutdown(self):
        """읽기 프로세스를 멈추고 공유 메모리를 지운다. 두 번 불려도 된다."""
        if self.rings is None:
            return
        self.timer.stop()
        self.stats_timer.stop()
        self.stop.set()
        self.proc.join(1.0)
        if self.proc.is_alive():
            self.proc.terminate()
        self.samplers = {}
        self.rings.close(unlink=True)
        self.rings = None

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)


def resolve_targets(ips, iface):
    """[(ip, serial)] — gpio_watch 와 같은 규칙 (IP 가 없으면 NIC 에서 Blackfly 를 찾는다)."""
    net_tools = gpio_watch.net_tools
    if ips:
        nic = iface or net_tools.route_dev(ips[0]) or net_tools.auto_nic()
        serial_of = {d["ip"]: d["serial"] for d in net_tools.gvcp_discover_all(nic, 0.5)} if nic else {}
        return [(ip, serial_of.get(ip, "")) for ip in ips]
    nic = iface or net_tools.auto_nic()
    return [(d["ip"], d["serial"]) for d in gpio_watch.find_cameras(nic)] if nic else []


def run(targets, lines=None, names=None):
    if lines is None:
        lines = [0, 1, 2, 3] if len(targets) <= 2 else list(gpio_watch.INPUTS)
    app = QApplication.instance() or QApplication(sys.argv)
    try:
        import ui_theme
        ui_theme.apply(app)
    except Exception:             # 테마는 보기 좋으라고 — 없어도 된다
        pass
    window = ScopeWindow(targets, names if names is not None else gpio_watch.names_by_serial(), lines)
    app.aboutToQuit.connect(window.shutdown)
    # 터미널의 Ctrl+C 로도 닫히게 (Qt 이벤트 루프는 파이썬 시그널을 안 돌려서 타이머로 깨운다)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    wake = QTimer(interval=200, timeout=lambda: None)
    wake.start()
    window.show()
    return app.exec_()


def main():
    parser = argparse.ArgumentParser(description="Blackfly S GPIO 핀 신호 실시간 파형 (GVCP 읽기 전용)")
    parser.add_argument("ips", nargs="*", help="카메라 IP (없으면 NIC 에서 Blackfly 를 찾는다)")
    parser.add_argument("-i", "--iface", default=None, help="카메라 NIC (기본: 가장 빠른 NIC)")
    parser.add_argument("--lines", default=None,
                        help="볼 라인 (예: 0,3). 기본: 1~2대면 0,1,2,3 · 그보다 많으면 입력 라인 0,2,3")
    parser.add_argument("--text", action="store_true",
                        help="창 없이 터미널 표로 (gpio_watch.py 와 같음 — -t 초 · -w 반복도 그대로)")
    if "--text" in sys.argv[1:]:          # 나머지 인자는 gpio_watch 가 해석한다
        sys.argv.remove("--text")
        return gpio_watch.main()
    args = parser.parse_args()
    targets = resolve_targets(args.ips, args.iface)
    if not targets:
        print("Blackfly 카메라를 못 찾았습니다 (-i 로 NIC 지정, 또는 IP 를 직접)")
        return 1
    lines = [int(x) for x in args.lines.split(",")] if args.lines else None
    return run(targets, lines)


if __name__ == "__main__":
    sys.exit(main())
