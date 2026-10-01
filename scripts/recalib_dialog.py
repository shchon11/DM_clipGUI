# recalib_dialog.py — [이전 녹화에도 적용…] 창: 지금 차량 캘리브레이션을 이미 녹화한 bag 들에 넣는다.
#
# 목록(현황표와 같은 곳: 저장 위치 + /mnt/data 등) → 녹화마다 '지금 캘값과 같음 / 옛 값' 표시 → 골라서 적용.
# 진행: 녹화마다 막대 + 전체 막대(몇 번째 · 남은 시간) + 로그. 중단하면 하던 bag 은 원래대로 돌아간다
# (bag 하나 = 트랜잭션 하나). 되돌리기도 같은 창에서. 실제 일은 bag_recalib.py (Qt 없음).

import time
from pathlib import Path

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView, QDialog, QHBoxLayout, QHeaderView, QLabel, QMessageBox, QPlainTextEdit, QProgressBar,
    QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout,
)

import bag_recalib as br

OK_C, WARN_C, ERR_C, MUTED_C = "#16a34a", "#d97706", "#dc2626", "#6b7280"
COLS = ["", "녹화", "지역", "날짜 · 시각", "길이", "지금 들어 있는 캘리브레이션", "진행"]


class Worker(QThread):
    sig_bag = pyqtSignal(int, float, str)        # 행, 0~1, 문구
    sig_done = pyqtSignal(int, bool, str)        # 행, 성공?, 요약
    sig_log = pyqtSignal(str)

    def __init__(self, jobs, calib, mode):
        super().__init__()
        self.jobs, self.calib, self.mode = jobs, calib, mode      # jobs: [(행, bag 경로)]
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        for row, bag in self.jobs:
            if self._stop:
                self.sig_done.emit(row, False, "중단됨 (손대지 않음)")
                continue
            name = Path(bag).name
            self.sig_log.emit(f"▶ {name} {'적용' if self.mode == 'apply' else '되돌리기'} 시작")
            t0 = time.time()
            try:
                prog = lambda f, text, row=row: self.sig_bag.emit(row, f, text)   # noqa: E731
                if self.mode == "apply":
                    r = br.apply(bag, self.calib, prog, cancel=lambda: self._stop, log=self.sig_log.emit)
                    msg = (f"camera_info {len(r['camera_info_topics'])}대 · TF {len(r['tf_frames'])}개 "
                           f"({r['tf_added']}개 새로) · {time.time() - t0:.0f}초" +
                           ("" if r["verified"] else f" · ✖ 확인 실패 {r.get('verify_failed')}"))
                    ok = r["verified"]
                    for p in r["skipped"]:
                        self.sig_log.emit(f"   ⚠ {p}")
                else:
                    br.revert(bag, prog, cancel=lambda: self._stop, log=self.sig_log.emit)
                    msg, ok = f"되돌림 · {time.time() - t0:.0f}초", True
                self.sig_done.emit(row, ok, msg)
                self.sig_log.emit(f"{'✔' if ok else '✖'} {name}: {msg}")
            except br.Cancelled:
                self.sig_done.emit(row, False, "중단 — 원래대로 둠")
                self.sig_log.emit(f"■ {name}: 중단 — 이 녹화는 원래대로 돌아갔습니다")
            except Exception as e:
                self.sig_done.emit(row, False, f"실패: {e}")
                self.sig_log.emit(f"✖ {name}: {e}")


class RecalibDialog(QDialog):
    def __init__(self, parent, ci_path, ex_path, bag_rows, acquisition=None):
        """bag_rows: 현황표 줄들 (folder · region_name · date · start_time · duration_hms)."""
        super().__init__(parent)
        self.setWindowTitle("이전 녹화에도 캘리브레이션 적용")
        self.resize(1080, 720)
        self.acquisition = acquisition or (lambda: (None, None))
        self.worker = None
        self.calib = br.vehicle_calib(ci_path, ex_path)
        src = self.calib["source"]
        v = QVBoxLayout(self)
        head = QLabel(
            f"<b>지금 차량에 들어가 있는 캘리브레이션</b>을 이미 녹화한 bag 의 <b>camera_info</b> 와 <b>/tf_static</b> 에 넣습니다.<br>"
            f"camera_info: {src['camera_info']} <span style='color:{MUTED_C}'>({src['camera_info_mtime']})</span><br>"
            f"TF: {src['extrinsics']} <span style='color:{MUTED_C}'>({src['extrinsics_mtime']})</span><br>"
            f"<span style='color:{MUTED_C}'>영상은 건드리지 않고 그 행만 바꿉니다 (녹화 하나에 수 초~수 분). 바꾸기 전 값은 녹화 폴더 "
            f"calib_backup_*.json 에 남아 [되돌리기] 로 돌아갈 수 있습니다. 중단하면 하던 녹화는 원래대로 남습니다.</span>")
        head.setWordWrap(True)
        head.setTextFormat(Qt.RichText)
        v.addWidget(head)

        self.table = QTableWidget(0, len(COLS))
        self.table.setHorizontalHeaderLabels(COLS)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        hh = self.table.horizontalHeader()
        for i, mode in enumerate([QHeaderView.ResizeToContents, QHeaderView.Stretch, QHeaderView.ResizeToContents,
                                  QHeaderView.ResizeToContents, QHeaderView.ResizeToContents, QHeaderView.Stretch,
                                  QHeaderView.Fixed]):
            hh.setSectionResizeMode(i, mode)
        self.table.setColumnWidth(6, 260)
        v.addWidget(self.table, 1)
        self.bags = []
        for r in bag_rows:
            self._add_row(r)

        pick = QHBoxLayout()
        for text, fn in (("전체 선택", lambda: self._select(lambda b: True)),
                         ("옛 값인 녹화만 선택", lambda: self._select(lambda b: not b["current"])),
                         ("선택 해제", lambda: self._select(lambda b: False))):
            b = QPushButton(text)
            b.clicked.connect(fn)
            pick.addWidget(b)
        pick.addStretch(1)
        v.addLayout(pick)

        self.overall = QProgressBar()
        self.overall.setFormat("대기")
        self.lbl_overall = QLabel("")
        v.addWidget(self.overall)
        v.addWidget(self.lbl_overall)
        self.logbox = QPlainTextEdit(readOnly=True)
        self.logbox.setMaximumHeight(150)
        v.addWidget(self.logbox)

        btns = QHBoxLayout()
        self.btn_apply = QPushButton("선택한 녹화에 적용")
        self.btn_apply.setStyleSheet("QPushButton{background:#2563eb;color:white;font-weight:bold;padding:6px 16px;border-radius:8px}"
                                     "QPushButton:disabled{background:#cbd5e1}")
        self.btn_apply.clicked.connect(lambda: self._start("apply"))
        self.btn_revert = QPushButton("선택한 녹화 되돌리기")
        self.btn_revert.clicked.connect(lambda: self._start("revert"))
        self.btn_stop = QPushButton("중단")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._stop)
        self.btn_close = QPushButton("닫기")
        self.btn_close.clicked.connect(self.close)
        for b in (self.btn_apply, self.btn_revert, self.btn_stop):
            btns.addWidget(b)
        btns.addStretch(1)
        btns.addWidget(self.btn_close)
        v.addLayout(btns)

    # --- 목록 ---
    def _status(self, folder):
        """(지금 차량 값과 같음?, 표시 문구, 색)."""
        cal = br.bag_info(folder).get("calibration") or {}
        src = cal.get("source") or {}
        cur = self.calib["source"]
        if not cal.get("applied_at"):
            return False, "녹화 당시 값 (한 번도 안 바꿈)", MUTED_C
        if src.get("camera_info_sha") == cur["camera_info_sha"] and src.get("extrinsics_sha") == cur["extrinsics_sha"]:
            return True, f"✔ 지금 차량 값과 같음 ({cal['applied_at'][5:16].replace('T', ' ')} 적용)", OK_C
        return False, f"옛 캘리브레이션 ({src.get('camera_info_mtime', '?')[:16].replace('T', ' ')} 값, {cal['applied_at'][5:16].replace('T', ' ')} 적용)", WARN_C

    def _add_row(self, r):
        folder = Path(r["folder"])
        if not folder.is_dir():
            return
        i = self.table.rowCount()
        self.table.insertRow(i)
        chk = QTableWidgetItem()
        chk.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
        current, text, color = self._status(folder)
        chk.setCheckState(Qt.Unchecked if current else Qt.Checked)
        self.table.setItem(i, 0, chk)
        name = QTableWidgetItem(f"{folder.parent.name}/{folder.name}")
        name.setToolTip(str(folder))
        self.table.setItem(i, 1, name)
        self.table.setItem(i, 2, QTableWidgetItem(r.get("region_name") or r.get("region", "")))
        self.table.setItem(i, 3, QTableWidgetItem(f"{r.get('date', '')} {r.get('start_time', '')}"))
        self.table.setItem(i, 4, QTableWidgetItem(r.get("duration_hms") or ""))
        st = QTableWidgetItem(text)
        st.setForeground(QColor(color))
        self.table.setItem(i, 5, st)
        bar = QProgressBar()
        bar.setRange(0, 1000)
        bar.setValue(0)
        bar.setFormat("")
        bar.setTextVisible(True)
        self.table.setCellWidget(i, 6, bar)
        self.bags.append({"row": i, "folder": str(folder), "current": current, "bar": bar})

    def _select(self, pred):
        for b in self.bags:
            self.table.item(b["row"], 0).setCheckState(Qt.Checked if pred(b) else Qt.Unchecked)

    # --- 실행 ---
    def _start(self, mode):
        block, _ = self.acquisition()
        if block:
            QMessageBox.warning(self, "적용", f"{block} — 녹화 중에는 하지 않습니다 (디스크를 같이 써서 녹화가 끊길 수 있음).")
            return
        jobs = [(b["row"], b["folder"]) for b in self.bags if self.table.item(b["row"], 0).checkState() == Qt.Checked]
        if not jobs:
            QMessageBox.information(self, "적용", "녹화를 하나 이상 고르세요.")
            return
        what = "적용" if mode == "apply" else "되돌리기"
        if QMessageBox.question(self, what, f"녹화 {len(jobs)}개에 {what}할까요?\n\n"
                                "녹화 폴더의 bag 파일을 직접 바꿉니다 (바꾸기 전 값은 백업).") != QMessageBox.Yes:
            return
        self.mode, self.jobs, self.done_n, self.t_start = mode, jobs, 0, time.time()
        for row, _ in jobs:
            bar = self.table.cellWidget(row, 6)
            bar.setValue(0)
            bar.setFormat("대기")
        self.overall.setRange(0, len(jobs) * 1000)
        self.overall.setValue(0)
        self._busy(True)
        self.worker = Worker(jobs, self.calib, mode)
        self.worker.sig_bag.connect(self._on_bag)
        self.worker.sig_done.connect(self._on_done)
        self.worker.sig_log.connect(self._log)
        self.worker.finished.connect(self._finished)
        self.worker.start()

    def _busy(self, on):
        for b in (self.btn_apply, self.btn_revert, self.btn_close):
            b.setEnabled(not on)
        self.btn_stop.setEnabled(on)

    def _on_bag(self, row, frac, text):
        bar = self.table.cellWidget(row, 6)
        bar.setValue(int(frac * 1000))
        bar.setFormat(f"{frac * 100:.0f}% · {text}")
        idx = next(k for k, (r, _) in enumerate(self.jobs) if r == row)
        total = (idx + frac) / len(self.jobs)
        self.overall.setValue(int(total * self.overall.maximum()))
        el = time.time() - self.t_start
        eta = f" · 약 {el / total * (1 - total) / 60:.0f}분 남음" if total > 0.02 else ""
        self.overall.setFormat(f"{idx + 1}/{len(self.jobs)} 번째 · 전체 {total * 100:.0f}%")
        self.lbl_overall.setText(f"경과 {el / 60:.1f}분{eta}")

    def _on_done(self, row, ok, msg):
        bar = self.table.cellWidget(row, 6)
        bar.setValue(1000 if ok else bar.value())
        bar.setFormat("✔ 완료" if ok else "✖ " + msg.split(" — ")[0][:24])
        bar.setToolTip(msg)
        bar.setStyleSheet("" if ok else "QProgressBar::chunk{background:#dc2626}")
        folder = next(b["folder"] for b in self.bags if b["row"] == row)
        current, text, color = self._status(folder)
        st = self.table.item(row, 5)
        st.setText(text)
        st.setForeground(QColor(color))
        if ok:
            self.table.item(row, 0).setCheckState(Qt.Unchecked)

    def _log(self, text):
        self.logbox.appendPlainText(time.strftime("%H:%M:%S ") + text)

    def _stop(self):
        if self.worker:
            self.worker.stop()
            self.btn_stop.setEnabled(False)
            self._log("중단 요청 — 하던 녹화는 원래대로 돌리고 멈춥니다")

    def _finished(self):
        self._busy(False)
        self.overall.setValue(self.overall.maximum())
        self.overall.setFormat("끝")
        self._log(f"끝 — {(time.time() - self.t_start) / 60:.1f}분")

    def closeEvent(self, e):
        if self.worker and self.worker.isRunning():
            if QMessageBox.question(self, "진행 중", "적용이 진행 중입니다. 중단하고 닫을까요?\n(하던 녹화는 원래대로 돌아갑니다)") != QMessageBox.Yes:
                e.ignore()
                return
            self.worker.stop()
            self.worker.wait(60000)
        super().closeEvent(e)
