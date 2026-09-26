#!/usr/bin/env python3
# calib_tab.py — [온라인 캘리브레이션] 탭: 주행 bag 여러 개로 타깃 없는 캘리브레이션(nontarget_cal)을 돌리고,
# 결과를 보고, 차량의 camera_info · TF 에 적용/되돌리기.
#
# 흐름:  bag 고르기 → zero-shot / warm-start → 저장 위치 · 공간 추정 → [시작]
#        (공간이 모자라면 '대기' 로 남김) → 백그라운드 실행 · 진행 · 로그 · 중단 · 이어서 실행
#        → 결과(카메라별 1σ · 판정) → [차량에 적용…] (시리얼 키, 백업 · 검증) → [적용 기록 · 되돌리기…]
#
# 데이터 수집과 같이 돌리지 않는다: 녹화(수동 · 클립) 중이면 시작 버튼이 잠기고, 돌던 중에 녹화가 시작되면
# 캘리브레이션을 중단한다(끝난 단계는 남아 이어서 실행할 수 있다). CPU 를 대부분 쓰고 수십 분~수 시간 걸린다.
# 로직은 online_calib.py · calib_apply.py (Qt 없음, test/test_online_calib.py 로 시험).

import json
import os
import signal
import subprocess
import time
from pathlib import Path

from PyQt5.QtCore import QProcess, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView, QButtonGroup, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFileDialog,
    QFormLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox, QPlainTextEdit,
    QProgressBar, QPushButton, QRadioButton, QSplitter, QTableWidget, QTableWidgetItem, QTextBrowser,
    QVBoxLayout, QWidget,
)

import calib_apply as ca
import online_calib as oc

try:
    import ui_theme
except Exception:        # 단독 시험
    ui_theme = None

OK_C, WARN_C, ERR_C, MUTED_C = "#16a34a", "#d97706", "#dc2626", "#6b7280"
SERIAL_MAP_CANDIDATES = [
    Path(__file__).resolve().parent.parent / "config" / "camera_serial_map.yaml",
    # 설치본: install/clip_recorder/lib/clip_recorder/calib_tab.py → install/clip_recorder/share/clip_recorder/config
    Path(__file__).resolve().parents[2] / "share" / "clip_recorder" / "config" / "camera_serial_map.yaml",
    Path.home() / "DM_clipGUI" / "config" / "camera_serial_map.yaml",
]
FLIR_GROUP = "flir_cameras"
LOG_TAIL_BYTES = 64 * 1024

NOTICE = ("⚠ 데이터 수집(녹화) 중에는 실행하지 마세요. 캘리브레이션은 CPU 를 대부분 쓰고 수십 분에서 수 시간이 걸립니다 "
          "(15분 주행 bag 하나 ≈ 3–3.5시간, 작업 폴더 100 GB 이상). 녹화가 시작되면 캘리브레이션은 자동으로 중단되고, "
          "끝난 단계는 남아 나중에 [이어서 실행] 할 수 있습니다. GUI 를 닫아도 계속 돕니다.")


def _variant(btn, v):
    if ui_theme:
        ui_theme.set_variant(btn, v)


def _open(path):
    try:
        subprocess.Popen(["xdg-open", str(path)])
    except OSError:
        pass


def _item(text, color=None, align=None, tip=None):
    it = QTableWidgetItem(str(text))
    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
    if color:
        it.setForeground(QColor(color))
    if align:
        it.setTextAlignment(align)
    if tip:
        it.setToolTip(tip)
    return it


def _f(x, nd):
    try:
        return f"{float(x):.{nd}f}"
    except (TypeError, ValueError):
        return "–"


def used_gb_from_events(workdir):
    """작업 폴더의 events.jsonl 에서 단계별 disk_gb 최신값 합 (이어서 실행할 때 이미 쓴 양, du 없이)."""
    by = {}
    try:
        with open(Path(workdir) / "events.jsonl", encoding="utf-8") as f:
            for line in f:
                try:
                    e = json.loads(line)
                except ValueError:
                    continue
                if e.get("ev") == "stage_end" and e.get("disk_gb") is not None:
                    by[e.get("stage")] = float(e["disk_gb"])
    except OSError:
        pass
    return sum(by.values())


class CalibTab(QWidget):
    sig_log = pyqtSignal(str, str)          # level, text — 녹화 탭 로그에도 남긴다

    def __init__(self, cfg, probe=None, stage=None, parent=None):
        """probe() -> {"recording", "clip_busy", "sensors", "recorder"} (bool). stage: SensorStageWidget(선택)."""
        super().__init__(parent)
        self.cfg = cfg
        self.probe = probe or (lambda: {})
        self.stage = stage
        self.c = cfg.setdefault("ui", {}).setdefault("calib", {})
        self.store = oc.JobStore()
        self.progress = {}         # job id -> (attempt n, Progress)
        self.check_proc = None
        self._check_buf = b""
        self._bag_cache = {}
        self._build()
        for j in self.store.jobs:
            oc.settle_state(j)
        self.store.save()
        self.refresh_bags()
        self.refresh_jobs()
        self.timer = QTimer(self, interval=1000, timeout=self._tick)
        self.timer.start()
        self._tick()

    # ------------------------------------------------------------ UI
    def _build(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        note = QLabel(NOTICE)
        note.setWordWrap(True)
        note.setStyleSheet("background:#fef3c7; color:#92400e; padding:8px; border-radius:6px;")
        outer.addWidget(note)
        self.lbl_acq = QLabel()
        self.lbl_acq.setWordWrap(True)
        outer.addWidget(self.lbl_acq)

        split = QSplitter(Qt.Horizontal)
        split.setChildrenCollapsible(False)
        outer.addWidget(split, 1)

        # ---------------- 왼쪽: 새 작업
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 6, 0)

        g1 = QGroupBox("1. 주행 bag (여러 개 고를 수 있음)")
        v1 = QVBoxLayout(g1)
        row = QHBoxLayout()
        self.ed_bag_root = QLineEdit(self.c.get("bag_root") or self.cfg.get("recorder", {}).get("output_dir", ""))
        self.ed_bag_root.setToolTip("이 폴더의 녹화(rec_* · clip_*)를 목록에 보인다")
        b = QPushButton("폴더…")
        b.clicked.connect(self._pick_bag_root)
        r = QPushButton("새로고침")
        r.clicked.connect(self.refresh_bags)
        add = QPushButton("다른 bag 추가…")
        add.clicked.connect(self._add_bag)
        row.addWidget(QLabel("위치"))
        row.addWidget(self.ed_bag_root, 1)
        row.addWidget(b)
        row.addWidget(r)
        v1.addLayout(row)
        self.tbl_bags = QTableWidget(0, 6)
        self.tbl_bags.setHorizontalHeaderLabels(["bag", "길이", "크기", "RGB/열", "상태", "경로"])
        self.tbl_bags.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.tbl_bags.setColumnHidden(5, True)
        self.tbl_bags.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl_bags.verticalHeader().setVisible(False)
        self.tbl_bags.itemChanged.connect(lambda *_: self._update_estimate())
        v1.addWidget(self.tbl_bags, 1)
        v1.addWidget(add, 0, Qt.AlignLeft)
        lv.addWidget(g1, 3)

        g2 = QGroupBox("2. 시작 값")
        v2 = QVBoxLayout(g2)
        self.rb_zero = QRadioButton("zero-shot — 설계값에서 시작 (처음 · 렌즈나 카메라를 다시 조였을 때)")
        self.rb_warm = QRadioButton("warm-start — 지금 캘리브레이션에서 시작")
        grp = QButtonGroup(self)
        grp.addButton(self.rb_zero)
        grp.addButton(self.rb_warm)
        (self.rb_warm if self.c.get("mode") == "warm" else self.rb_zero).setChecked(True)
        v2.addWidget(self.rb_zero)
        v2.addWidget(self.rb_warm)
        wr = QHBoxLayout()
        self.cb_init = QComboBox()
        self.cb_init.addItem("차량에 적용된 값 (camera_info + TF 파일)", "vehicle")
        self.cb_init.addItem("마지막으로 적용한 결과 (적용 기록 보관본)", "applied")
        self.cb_init.addItem("폴더 지정 (도구 결과 · deliverable 폴더)", "dir")
        self.ed_init = QLineEdit(self.c.get("init_dir", ""))
        self.ed_init.setPlaceholderText("extrinsic/ + intrinsic/ 가 있는 폴더")
        bi = QPushButton("…")
        bi.clicked.connect(lambda: self._pick_dir(self.ed_init, "이전 캘리브레이션 폴더"))
        wr.addSpacing(24)
        wr.addWidget(self.cb_init)
        wr.addWidget(self.ed_init, 1)
        wr.addWidget(bi)
        v2.addLayout(wr)
        for w in (self.rb_zero, self.rb_warm):
            w.toggled.connect(self._mode_changed)
        self.cb_init.currentIndexChanged.connect(self._mode_changed)
        lv.addWidget(g2)

        g3 = QGroupBox("3. 저장 위치 · 공간")
        f3 = QFormLayout(g3)
        default_root = str(Path(os.path.expanduser(self.cfg.get("recorder", {}).get("output_dir", "~/DM_clipGUI/clips"))).parent
                           / "online_calib")
        self.ed_work = QLineEdit(self.c.get("workdir_root") or default_root)
        self.ed_work.setToolTip("큰 중간 파일(100 GB 이상)이 쌓이는 곳 — 여유가 큰 디스크 (NVMe 면 더 빠름). "
                                "작업마다 <여기>/<작업 id>/work")
        self.ed_out = QLineEdit(self.c.get("out_root") or default_root)
        self.ed_out.setToolTip("결과(YAML · 보고서 · 이미지, 수십 MB) — 작업마다 <여기>/<작업 id>/out")
        for ed, title in ((self.ed_work, "작업 폴더 위치"), (self.ed_out, "결과 폴더 위치")):
            h = QHBoxLayout()
            h.addWidget(ed, 1)
            bb = QPushButton("…")
            bb.clicked.connect(lambda _, e=ed, t=title: (self._pick_dir(e, t), self._update_estimate()))
            h.addWidget(bb)
            ed.editingFinished.connect(self._update_estimate)
            f3.addRow(title, h)
        sens = QHBoxLayout()
        self.chk_rgb = QCheckBox("RGB 14대")
        self.chk_th = QCheckBox("열화상 2대")
        self.chk_rgb.setChecked(True)
        self.chk_th.setChecked(True)
        for w in (self.chk_rgb, self.chk_th):
            w.toggled.connect(self._update_estimate)
            sens.addWidget(w)
        sens.addStretch(1)
        f3.addRow("센서", sens)
        self.lbl_est = QLabel("bag 을 고르세요")
        self.lbl_est.setWordWrap(True)
        f3.addRow("추정", self.lbl_est)
        adv = QHBoxLayout()
        self.chk_force = QCheckBox("--force (데이터 양 · 동기 거절 무시)")
        self.chk_force.setToolTip("회전 부족 · 동기 · 노출 거절과 bag 간 불일치를 무시하고 진행. 디스크 부족은 무시 못 함. "
                                  "결과를 믿기 어려울 수 있으니 판정을 꼭 확인")
        adv.addWidget(self.chk_force)
        self.ed_namemap = QLineEdit(self.c.get("name_map", ""))
        self.ed_namemap.setPlaceholderText("이름 대응표 (비우면 도구가 자동 확인)")
        adv.addWidget(self.ed_namemap, 1)
        f3.addRow("고급", adv)
        lv.addWidget(g3)

        br = QHBoxLayout()
        self.btn_check = QPushButton("사전 점검 (약 30초)")
        self.btn_check.setToolTip("nontarget_cal check — 이름 대응 · 움직임 · 회전 · 동기 · 노출 · 디스크를 bag 에서 가볍게 확인")
        self.btn_check.clicked.connect(self.run_check)
        self.btn_start = QPushButton("캘리브레이션 시작")
        _variant(self.btn_start, "primary")
        self.btn_start.clicked.connect(self.start_new)
        br.addWidget(self.btn_check)
        br.addStretch(1)
        br.addWidget(self.btn_start)
        lv.addLayout(br)
        self.lbl_tool = QLabel()
        self.lbl_tool.setWordWrap(True)
        self.lbl_tool.setStyleSheet(f"color:{MUTED_C};")
        lv.addWidget(self.lbl_tool)
        split.addWidget(left)

        # ---------------- 오른쪽: 작업 · 진행 · 결과
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(6, 0, 0, 0)
        self.tbl_jobs = QTableWidget(0, 5)
        self.tbl_jobs.setHorizontalHeaderLabels(["작업", "bag", "시작 값", "상태", "진행"])
        self.tbl_jobs.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.tbl_jobs.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl_jobs.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl_jobs.verticalHeader().setVisible(False)
        self.tbl_jobs.itemSelectionChanged.connect(self._show_job)
        self.tbl_jobs.setMaximumHeight(170)
        rv.addWidget(self.tbl_jobs)

        self.lbl_job = QLabel("작업을 고르세요")
        self.lbl_job.setWordWrap(True)
        self.lbl_job.setTextInteractionFlags(Qt.TextSelectableByMouse)
        rv.addWidget(self.lbl_job)
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        self.bar.setFormat("%p%")
        rv.addWidget(self.bar)
        self.lbl_stage = QLabel()
        self.lbl_stage.setWordWrap(True)
        rv.addWidget(self.lbl_stage)
        jb = QHBoxLayout()
        self.btn_cancel = QPushButton("중단")
        _variant(self.btn_cancel, "danger")
        self.btn_cancel.clicked.connect(lambda: self.cancel_job(reason="사용자가 중단"))
        self.btn_resume = QPushButton("이어서 실행")
        self.btn_resume.clicked.connect(self.resume_job)
        self.btn_recheck = QPushButton("공간 다시 확인")
        self.btn_recheck.clicked.connect(self.recheck_job)
        self.btn_move = QPushButton("작업 폴더 바꾸기…")
        self.btn_move.clicked.connect(self.move_workdir)
        self.btn_logs = QPushButton("로그 폴더")
        self.btn_logs.clicked.connect(lambda: self._sel() and _open(oc.job_dir(self._sel())))
        self.btn_del = QPushButton("삭제…")
        self.btn_del.clicked.connect(self.delete_job)
        for w in (self.btn_cancel, self.btn_resume, self.btn_recheck, self.btn_move, self.btn_logs, self.btn_del):
            jb.addWidget(w)
        jb.addStretch(1)
        rv.addLayout(jb)

        vs = QSplitter(Qt.Vertical)
        vs.setChildrenCollapsible(False)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(3000)
        self.log_view.setStyleSheet("background:#0f172a; color:#e2e8f0; font-family:monospace; font-size:11px;")
        vs.addWidget(self.log_view)

        res = QWidget()
        resv = QVBoxLayout(res)
        resv.setContentsMargins(0, 0, 0, 0)
        self.lbl_result = QLabel()
        self.lbl_result.setWordWrap(True)
        resv.addWidget(self.lbl_result)
        self.tbl_res = QTableWidget(0, 9)
        self.tbl_res.setHorizontalHeaderLabels(["카메라", "판정", "회전 1σ [°]", "위치 1σ [mm]", "광축 1σ [mm]",
                                                "초점 1σ [px]", "재투영 [px]", "held-out [px]", "vote"])
        self.tbl_res.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.tbl_res.verticalHeader().setVisible(False)
        resv.addWidget(self.tbl_res, 1)
        rb = QHBoxLayout()
        self.btn_report = QPushButton("보고서 보기")
        self.btn_report.clicked.connect(self.show_report)
        self.btn_outdir = QPushButton("결과 폴더")
        self.btn_outdir.clicked.connect(lambda: self._sel() and _open(self._sel()["out"]))
        self.btn_apply = QPushButton("차량에 적용…")
        _variant(self.btn_apply, "primary")
        self.btn_apply.clicked.connect(self.apply_dialog)
        self.btn_hist = QPushButton("적용 기록 · 되돌리기…")
        self.btn_hist.clicked.connect(self.history_dialog)
        for w in (self.btn_report, self.btn_outdir, self.btn_apply):
            rb.addWidget(w)
        rb.addStretch(1)
        rb.addWidget(self.btn_hist)
        resv.addLayout(rb)
        vs.addWidget(res)
        vs.setSizes([220, 320])
        rv.addWidget(vs, 1)
        split.addWidget(right)
        split.setSizes([520, 640])
        self._mode_changed()
        self._update_tool_label()

    # ------------------------------------------------------------ 공용
    def _save_settings(self):
        self.c.update({"bag_root": self.ed_bag_root.text().strip(), "workdir_root": self.ed_work.text().strip(),
                       "out_root": self.ed_out.text().strip(), "init_dir": self.ed_init.text().strip(),
                       "mode": "warm" if self.rb_warm.isChecked() else "zeroshot",
                       "name_map": self.ed_namemap.text().strip()})

    def _pick_dir(self, edit, title):
        d = QFileDialog.getExistingDirectory(self, title, edit.text() or str(Path.home()))
        if d:
            edit.setText(d)

    def _pick_bag_root(self):
        self._pick_dir(self.ed_bag_root, "녹화 폴더 위치")
        self.refresh_bags()

    def _add_bag(self):
        d = QFileDialog.getExistingDirectory(self, "bag 폴더 (metadata.yaml 이 있는 곳)", self.ed_bag_root.text())
        if not d:
            return
        extra = self.c.setdefault("extra_bags", [])
        if d not in extra:
            extra.append(d)
        self.refresh_bags(check=[d])

    def _log(self, level, text):
        self.sig_log.emit(level, f"[캘리브레이션] {text}")

    def acquisition(self):
        """(시작을 막는 이유 또는 None, 경고 목록)."""
        try:
            s = self.probe() or {}
        except Exception:
            s = {}
        block, warn = None, []
        if s.get("recording"):
            block = "수동 녹화 중입니다"
        elif s.get("clip_busy"):
            block = "클립을 저장하는 중입니다"
        if s.get("sensors"):
            warn.append("센서가 켜져 있습니다")
        if s.get("recorder"):
            warn.append("레코더(링 버퍼)가 돌고 있습니다")
        return block, warn

    def _tool(self):
        return oc.find_tool(self.c.get("executable") or None)

    def _update_tool_label(self):
        t = self._tool()
        v = oc.vendored_info()
        if t["ok"]:
            ver = t["version"]
            txt = f"도구: {t['exe']}"
            if ver.get("commit"):
                txt += f" · 커밋 {str(ver['commit'])[:8]}"
            if v.get("commit") and ver.get("commit") and str(v["commit"]) != str(ver["commit"]):
                txt += f" — 리포 사본({str(v['commit'])[:8]})과 다름: tools/setup_online_calib.sh 로 다시 설치"
        else:
            txt = t["msg"]
        self.lbl_tool.setText(txt)
        self.lbl_tool.setStyleSheet(f"color:{MUTED_C if t['ok'] else ERR_C};")

    def _mode_changed(self):
        warm = self.rb_warm.isChecked()
        self.cb_init.setEnabled(warm)
        self.ed_init.setEnabled(warm and self.cb_init.currentData() == "dir")

    # ------------------------------------------------------------ bag 목록
    def refresh_bags(self, check=None):
        checked = set(check or []) | set(self._checked_bags())
        root = Path(os.path.expanduser(self.ed_bag_root.text().strip() or "."))
        paths = []
        if root.is_dir():
            for d in sorted(root.iterdir(), reverse=True):
                if d.is_dir() and (d / "metadata.yaml").is_file() and d.name.startswith(("rec_", "clip_")):
                    paths.append(str(d))
        for d in self.c.get("extra_bags") or []:
            if d not in paths:
                paths.insert(0, d)
        self.tbl_bags.blockSignals(True)
        self.tbl_bags.setRowCount(0)
        for p in paths:
            b = self._bag_cache.get(p)
            if b is None or not b.get("ok"):
                b = oc.inspect_bag(p)
                self._bag_cache[p] = b
            r = self.tbl_bags.rowCount()
            self.tbl_bags.insertRow(r)
            it = _item(Path(p).name, tip=p)
            it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
            it.setCheckState(Qt.Checked if p in checked else Qt.Unchecked)
            self.tbl_bags.setItem(r, 0, it)
            self.tbl_bags.setItem(r, 1, _item(oc.fmt_duration(b["duration_s"]) if b["duration_s"] else "–"))
            self.tbl_bags.setItem(r, 2, _item(f"{b['size_gb']:.0f} GB"))
            self.tbl_bags.setItem(r, 3, _item(f"{b['n_rgb']}/{b['n_thermal']}"))
            self.tbl_bags.setItem(r, 4, _item("OK" if b["ok"] else b["error"], OK_C if b["ok"] else ERR_C,
                                              tip=b["error"]))
            self.tbl_bags.setItem(r, 5, _item(p))
        self.tbl_bags.blockSignals(False)
        self.tbl_bags.resizeColumnsToContents()
        self.tbl_bags.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self._update_estimate()

    def _checked_bags(self):
        out = []
        for r in range(self.tbl_bags.rowCount()):
            if self.tbl_bags.item(r, 0).checkState() == Qt.Checked:
                out.append(self.tbl_bags.item(r, 5).text())
        return out

    def _sensors(self):
        return tuple(s for s, w in (("rgb", self.chk_rgb), ("thermal", self.chk_th)) if w.isChecked())

    def _update_estimate(self):
        bags = [self._bag_cache.get(p) or oc.inspect_bag(p) for p in self._checked_bags()]
        if not bags:
            self.lbl_est.setText("bag 을 고르세요")
            return None
        est = oc.estimate_storage(bags, self._sensors() or ("rgb",))
        work, out = self.ed_work.text().strip(), self.ed_out.text().strip()
        ok, msg, _ = oc.check_space(est, work, out)
        lo, hi = est["runtime_h"]
        bad = [Path(b["path"]).name for b in bags if not b["ok"]]
        txt = (f"{est['basis']}<br><b style='color:{OK_C if ok else ERR_C}'>{msg}</b>"
               f"<br>예상 시간 약 {lo:.1f}–{hi:.1f}시간 (차량 PC 코어 수에 따라)")
        if bad:
            txt += f"<br><span style='color:{ERR_C}'>문제 있는 bag: {', '.join(bad)}</span>"
        self.lbl_est.setText(txt)
        return est

    # ------------------------------------------------------------ 작업 만들기 · 시작
    def _running(self):
        return [j for j in self.store.jobs if j["state"] == oc.RUNNING]

    def running_job(self):
        r = self._running()
        return r[0] if r else None

    def _confirm_acquisition(self):
        block, warn = self.acquisition()
        if block:
            QMessageBox.warning(self, "캘리브레이션", f"{block} — 데이터 수집 중에는 캘리브레이션을 시작할 수 없습니다.")
            return False
        if warn:
            box = QMessageBox(QMessageBox.Warning, "캘리브레이션",
                              "\n".join(warn) + ".\n\n캘리브레이션은 CPU 와 디스크를 대부분 써서 데이터 수집을 방해합니다. "
                              "지금 데이터를 모으는 중이 아니라면 계속할 수 있습니다.\n(녹화를 시작하면 캘리브레이션은 자동으로 중단됩니다)",
                              QMessageBox.Yes | QMessageBox.Cancel, self)
            box.button(QMessageBox.Yes).setText("수집 중 아님 — 시작")
            box.button(QMessageBox.Cancel).setText("취소")
            box.setDefaultButton(QMessageBox.Cancel)
            return box.exec_() == QMessageBox.Yes
        return True

    def _resolve_init(self, job):
        """warm-start 의 --init 폴더. 실패하면 (None, 이유)."""
        kind = self.cb_init.currentData()
        if kind == "dir":
            d = self.ed_init.text().strip()
            if not d or not (Path(d) / "extrinsic").is_dir():
                return None, "extrinsic/ + intrinsic/ 가 있는 폴더를 고르세요"
            return d, ""
        if kind == "applied":
            for m in ca.list_applied(oc.APPLIED_ROOT):
                if not m.get("rolled_back") and (Path(m["archive"]) / "result" / "extrinsic").is_dir():
                    return str(Path(m["archive"]) / "result"), ""
            return None, "적용 기록이 없습니다 (이 GUI 로 적용한 적이 없음)"
        ci, ex = self.vehicle_files()
        smap = self._serial_map()
        if smap is None:
            return None, "카메라 시리얼 표(config/camera_serial_map.yaml)를 찾을 수 없습니다"
        d = oc.job_dir(job) / "init_from_vehicle"
        try:
            r = ca.export_vehicle_init(ci, ex, smap, d)
        except Exception as e:
            return None, f"차량 파일을 읽을 수 없습니다: {e}"
        if not r["cameras"]:
            return None, ("차량 파일에 os_lidar 기준 캘리브레이션이 없습니다 (아직 적용한 적 없음 · 자리표시 값뿐). "
                          f"zero-shot 으로 하세요.\n{ci}\n{ex}")
        return d, f"차량 파일에서 {len(r['cameras'])}대: {', '.join(r['cameras'])}"

    def start_new(self):
        self._save_settings()
        bags = self._checked_bags()
        if not bags:
            QMessageBox.information(self, "캘리브레이션", "주행 bag 을 하나 이상 고르세요.")
            return
        infos = [self._bag_cache.get(p) or oc.inspect_bag(p) for p in bags]
        bad = [i for i in infos if not i["ok"]]
        if bad and not self.chk_force.isChecked():
            QMessageBox.warning(self, "캘리브레이션", "문제 있는 bag:\n" + "\n".join(
                f"{Path(i['path']).name}: {i['error']}" for i in bad))
            return
        if not self._sensors():
            QMessageBox.information(self, "캘리브레이션", "센서(RGB · 열화상)를 하나 이상 고르세요.")
            return
        if self.running_job():
            QMessageBox.information(self, "캘리브레이션", "이미 돌고 있는 캘리브레이션이 있습니다 — 끝나거나 중단한 뒤에 시작하세요.")
            return
        tool = self._tool()
        if not tool["ok"]:
            QMessageBox.warning(self, "캘리브레이션", tool["msg"])
            return
        if not self._confirm_acquisition():
            return
        mode = "warm" if self.rb_warm.isChecked() else "zeroshot"
        job = oc.new_job(bags, "zeroshot", self.ed_work.text().strip(), self.ed_out.text().strip(),
                         sensors=self._sensors(), name_map=self.ed_namemap.text().strip() or None,
                         force=self.chk_force.isChecked())
        if mode == "warm":
            init, why = self._resolve_init(job)
            if not init:
                QMessageBox.warning(self, "warm-start", why)
                return
            job.update(mode="warm", init=str(init), init_note=why)
        est = oc.estimate_storage(infos, job["sensors"])
        job["estimate"] = est
        self.store.add(job)
        self._start_or_pend(job, est)
        self.refresh_jobs(select=job["id"])

    def _start_or_pend(self, job, est, already_gb=0.0):
        ok, msg, det = oc.check_space(est, job["workdir"], job["out"], already_gb=already_gb)
        job["space"] = {"ok": ok, "msg": msg, **det}
        if not ok:
            job["state"] = oc.PENDING
            job["state_msg"] = "공간 부족 — " + msg
            self.store.save()
            self._log("WARN", f"{job['id']} 대기: {msg}")
            QMessageBox.warning(self, "공간 부족 — 대기", f"{msg}\n\n작업을 '대기' 로 두었습니다. 공간을 비우거나 "
                                "[작업 폴더 바꾸기…] 로 다른 디스크를 고른 뒤 [공간 다시 확인] 을 누르세요.")
            return False
        tool = self._tool()
        if not tool["ok"]:
            job["state"], job["state_msg"] = oc.PENDING, tool["msg"]
            self.store.save()
            return False
        try:
            Path(job["workdir"]).mkdir(parents=True, exist_ok=True)
            Path(job["out"]).parent.mkdir(parents=True, exist_ok=True)
            att = oc.launch_detached(job, tool["exe"])
        except Exception as e:
            job["state"], job["state_msg"] = oc.FAILED, f"시작 실패: {e}"
            self.store.save()
            QMessageBox.critical(self, "캘리브레이션", f"시작 실패: {e}")
            return False
        job["tool"] = {"exe": tool["exe"], "commit": (tool["version"] or {}).get("commit")}
        self.store.save()
        self._log("GUI", f"{job['id']} 시작 (시도 {att['n']}, bag {len(job['bags'])}개, {job['mode']}) — {msg}")
        return True

    # ------------------------------------------------------------ 작업 조작
    def _sel(self):
        rows = self.tbl_jobs.selectionModel().selectedRows() if self.tbl_jobs.selectionModel() else []
        if not rows:
            return None
        jid = self.tbl_jobs.item(rows[0].row(), 0).data(Qt.UserRole)
        return self.store.get(jid)

    def cancel_job(self, job=None, reason="사용자가 중단", ask=True):
        job = job or self._sel()
        if not job or job["state"] != oc.RUNNING:
            return
        if ask:
            if QMessageBox.question(self, "캘리브레이션 중단",
                                    "캘리브레이션을 중단할까요?\n끝난 단계 · 창은 작업 폴더에 남아 [이어서 실행] 하면 거기서부터 "
                                    "다시 합니다.") != QMessageBox.Yes:
                return
        job["attempts"][-1]["stop_reason"] = reason
        oc.signal_job(job, signal.SIGTERM)
        jid = job["id"]
        QTimer.singleShot(8000, lambda: self._kill_if_alive(jid))
        self.store.save()
        self._log("WARN", f"{jid} 중단: {reason}")

    def _kill_if_alive(self, jid):
        job = self.store.get(jid)
        if job and job["state"] == oc.RUNNING and oc.signal_job(job, signal.SIGKILL):
            self._log("WARN", f"{jid}: 8초 안에 안 끝나 강제 종료")

    def resume_job(self):
        job = self._sel()
        if not job or job["state"] not in oc.RESUMABLE:
            return
        if self.running_job():
            QMessageBox.information(self, "캘리브레이션", "이미 돌고 있는 캘리브레이션이 있습니다.")
            return
        missing = [b for b in job["bags"] if not Path(b).is_dir()]
        if missing:
            QMessageBox.warning(self, "캘리브레이션", "bag 이 안 보입니다 (디스크 연결 확인):\n" + "\n".join(missing))
            return
        if not self._confirm_acquisition():
            return
        est = job.get("estimate") or oc.estimate_storage([oc.inspect_bag(b) for b in job["bags"]], job["sensors"])
        self._start_or_pend(job, est, already_gb=used_gb_from_events(job["workdir"]))
        self.refresh_jobs(select=job["id"])

    def recheck_job(self):
        job = self._sel()
        if not job or job["state"] != oc.PENDING:
            return
        est = job.get("estimate") or oc.estimate_storage([oc.inspect_bag(b) for b in job["bags"]], job["sensors"])
        ok, msg, _ = oc.check_space(est, job["workdir"], job["out"], already_gb=used_gb_from_events(job["workdir"]))
        if not ok:
            job["state_msg"] = "공간 부족 — " + msg
            self.store.save()
            QMessageBox.information(self, "공간", msg)
        elif QMessageBox.question(self, "공간", f"{msg}\n\n이제 시작할까요?") == QMessageBox.Yes:
            self.resume_job()
        self.refresh_jobs(select=job["id"])

    def move_workdir(self):
        job = self._sel()
        if not job or job["state"] == oc.RUNNING or job["state"] == oc.DONE:
            return
        d = QFileDialog.getExistingDirectory(self, "새 작업 폴더 위치 (여유가 큰 디스크)", str(Path(job["workdir"]).parent.parent))
        if not d:
            return
        old = Path(job["workdir"])
        if old.exists() and any(old.iterdir()):
            if QMessageBox.question(self, "작업 폴더", f"기존 작업 폴더 {old} 의 중간 결과는 쓰지 않고 처음부터 합니다 "
                                    "(지우지는 않음). 계속?") != QMessageBox.Yes:
                return
        job["workdir"] = str(Path(d) / job["id"] / "work")
        self.store.save()
        self.refresh_jobs(select=job["id"])

    def delete_job(self):
        job = self._sel()
        if not job:
            return
        if job["state"] == oc.RUNNING:
            QMessageBox.information(self, "삭제", "먼저 중단하세요.")
            return
        box = QMessageBox(QMessageBox.Question, "작업 삭제",
                          f"{job['id']} 을 목록에서 지웁니다.\n작업 폴더(중간 파일, 큼)도 지울까요?\n{job['workdir']}\n"
                          f"(결과 폴더 {job['out']} 는 남깁니다)", QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel, self)
        box.button(QMessageBox.Yes).setText("작업 폴더도 지우기")
        box.button(QMessageBox.No).setText("목록에서만")
        box.button(QMessageBox.Cancel).setText("취소")
        r = box.exec_()
        if r == QMessageBox.Cancel:
            return
        if r == QMessageBox.Yes:
            wd = Path(job["workdir"])
            # 안전: 우리가 만든 <root>/<작업 id>/work 형태일 때만 지운다
            if wd.name == "work" and wd.parent.name == job["id"] and wd.is_dir():
                import shutil
                shutil.rmtree(wd, ignore_errors=True)
        self.store.remove(job["id"])
        self.progress.pop(job["id"], None)
        self.refresh_jobs()

    # ------------------------------------------------------------ 주기 갱신
    def _tick(self):
        block, warn = self.acquisition()
        running = self.running_job()
        if block:
            self.lbl_acq.setText(f"<b style='color:{ERR_C}'>● {block} — 캘리브레이션을 시작할 수 없습니다</b>")
        elif warn:
            self.lbl_acq.setText(f"<span style='color:{WARN_C}'>● {' · '.join(warn)} — 데이터 수집 중이면 시작하지 마세요</span>")
        else:
            self.lbl_acq.setText(f"<span style='color:{OK_C}'>● 녹화 중 아님</span>")
        self.btn_start.setEnabled(not block and not running)
        if block and running:
            self.cancel_job(running, reason=f"{block} — 데이터 수집을 위해 자동 중단", ask=False)
        changed = False
        for j in self.store.jobs:
            if j["state"] == oc.RUNNING:
                self._feed(j)
                changed |= oc.settle_state(j)
                if j["state"] != oc.RUNNING:
                    self._feed(j)
                    self._log("OK" if j["state"] == oc.DONE else "WARN",
                              f"{j['id']}: {oc.STATE_LABEL[j['state']]} {j.get('state_msg', '')}")
        if changed:
            self.store.save()
            self.refresh_jobs(select=(self._sel() or {}).get("id"))
        else:
            self._update_job_rows()
            self._show_progress()

    def _feed(self, job):
        if not job["attempts"]:
            return None
        att = job["attempts"][-1]
        key = job["id"]
        cur = self.progress.get(key)
        if not cur or cur[0] != att["n"]:
            p = oc.Progress(job["sensors"])      # 이어서 실행하면 끝난 단계는 stage_skip 으로 온다
            cur = (att["n"], p)
            self.progress[key] = cur
        cur[1].feed_file(att["stdout"])
        return cur[1]

    # ------------------------------------------------------------ 표시
    def refresh_jobs(self, select=None):
        self.btn_start.setEnabled(not self.acquisition()[0] and not self.running_job())
        self.tbl_jobs.blockSignals(True)
        self.tbl_jobs.setRowCount(0)
        for j in sorted(self.store.jobs, key=lambda j: -j["created"]):
            r = self.tbl_jobs.rowCount()
            self.tbl_jobs.insertRow(r)
            it = _item(time.strftime("%m-%d %H:%M", time.localtime(j["created"])), tip=j["id"])
            it.setData(Qt.UserRole, j["id"])
            self.tbl_jobs.setItem(r, 0, it)
            self.tbl_jobs.setItem(r, 1, _item(", ".join(Path(b).name for b in j["bags"]), tip="\n".join(j["bags"])))
            self.tbl_jobs.setItem(r, 2, _item("warm-start" if j["mode"] == "warm" else "zero-shot"))
            self.tbl_jobs.setItem(r, 3, _item(""))
            self.tbl_jobs.setItem(r, 4, _item(""))
        self.tbl_jobs.blockSignals(False)
        self._update_job_rows()
        if select:
            for r in range(self.tbl_jobs.rowCount()):
                if self.tbl_jobs.item(r, 0).data(Qt.UserRole) == select:
                    self.tbl_jobs.selectRow(r)
                    break
        elif self.tbl_jobs.rowCount() and not self._sel():
            self.tbl_jobs.selectRow(0)
        self._show_job()

    def _update_job_rows(self):
        colors = {oc.DONE: OK_C, oc.RUNNING: "#2563eb", oc.PENDING: WARN_C, oc.REFUSED: ERR_C, oc.FAILED: ERR_C,
                  oc.INTERRUPTED: WARN_C, oc.CANCELLED: MUTED_C}
        for r in range(self.tbl_jobs.rowCount()):
            j = self.store.get(self.tbl_jobs.item(r, 0).data(Qt.UserRole))
            if not j:
                continue
            st = oc.STATE_LABEL.get(j["state"], j["state"])
            if j.get("state_msg"):
                st += f" — {j['state_msg']}"
            it = self.tbl_jobs.item(r, 3)
            it.setText(st)
            it.setToolTip(st)
            it.setForeground(QColor(colors.get(j["state"], MUTED_C)))
            p = self.progress.get(j["id"])
            self.tbl_jobs.item(r, 4).setText(f"{100 * p[1].fraction():.0f}%" if p else ("100%" if j["state"] == oc.DONE else ""))

    def _show_job(self):
        job = self._sel()
        for w in (self.btn_cancel, self.btn_resume, self.btn_recheck, self.btn_move, self.btn_logs, self.btn_del,
                  self.btn_report, self.btn_outdir, self.btn_apply):
            w.setEnabled(False)
        self.btn_hist.setEnabled(True)
        self.log_view.clear()
        self.tbl_res.setRowCount(0)
        self.lbl_result.setText("")
        if not job:
            self.lbl_job.setText("작업을 고르세요")
            self.bar.setValue(0)
            self.lbl_stage.setText("")
            return
        st = job["state"]
        self.btn_cancel.setEnabled(st == oc.RUNNING)
        self.btn_resume.setEnabled(st in oc.RESUMABLE)
        self.btn_recheck.setEnabled(st == oc.PENDING)
        self.btn_move.setEnabled(st in (oc.PENDING, oc.INTERRUPTED, oc.CANCELLED, oc.FAILED))
        self.btn_logs.setEnabled(True)
        self.btn_del.setEnabled(st != oc.RUNNING)
        est = job.get("estimate") or {}
        txt = (f"<b>{job['id']}</b> · {'warm-start' if job['mode'] == 'warm' else 'zero-shot'} · "
               f"센서 {', '.join(job['sensors'])}<br>작업 폴더 {job['workdir']}<br>결과 {job['out']}")
        if job.get("init"):
            txt += f"<br>시작 값 {job['init']}" + (f" ({job['init_note']})" if job.get("init_note") else "")
        if est:
            txt += f"<br>추정: 작업 폴더 {est.get('work_gb', 0):.0f} GB (여유 포함 {est.get('work_need_gb', 0):.0f} GB 필요)"
        if job.get("state_msg"):
            c = ERR_C if st in (oc.FAILED, oc.REFUSED) else WARN_C if st != oc.DONE else OK_C
            txt += f"<br><b style='color:{c}'>{oc.STATE_LABEL.get(st, st)} — {job['state_msg']}</b>"
        self.lbl_job.setText(txt)
        self._feed(job)
        self._show_progress()
        self._load_log(job)
        self._show_result(job)

    def _show_progress(self):
        job = self._sel()
        if not job:
            return
        p = (self.progress.get(job["id"]) or (None, None))[1]
        if not p:
            self.bar.setValue(1000 if job["state"] == oc.DONE else 0)
            self.lbl_stage.setText("")
            return
        self.bar.setValue(int(1000 * p.fraction()))
        parts = []
        for k, lbl, _ in p.expected():
            s = p.stages[k]
            if s["state"] == "run":
                parts.append(f"<b>{lbl}</b>" + (f" {s['done']}/{s['total']}" if s["total"] else ""))
            elif s["state"] in ("ok", "skip"):
                parts.append(f"<span style='color:{OK_C}'>{lbl} ✓</span>")
            elif s["state"] == "fail":
                parts.append(f"<span style='color:{ERR_C}'>{lbl} ✗</span>")
            else:
                parts.append(f"<span style='color:{MUTED_C}'>{lbl}</span>")
        extra = ""
        if job["state"] == oc.RUNNING and job["attempts"]:
            extra = f"<br>경과 {oc.fmt_duration(time.time() - job['attempts'][-1]['t0'])}"
        if p.warnings:
            extra += "<br>" + "<br>".join(f"<span style='color:{WARN_C}'>경고: {w.get('msg_ko') or w.get('msg')}</span>"
                                          for w in p.warnings[-5:])
        if p.task_failed:
            extra += f"<br><span style='color:{ERR_C}'>실패한 작업 {len(p.task_failed)}개 (로그 폴더)</span>"
        if p.refusal:
            extra += f"<br><b style='color:{ERR_C}'>거절 [{p.refusal.get('code')}] {p.refusal.get('msg_ko') or p.refusal.get('msg')}</b>"
        self.lbl_stage.setText(" → ".join(parts) + extra)
        if job["state"] == oc.RUNNING:
            self._load_log(job, append=True)

    def _load_log(self, job, append=False):
        if not job["attempts"]:
            return
        f = Path(job["attempts"][-1]["stderr"])
        try:
            size = f.stat().st_size
        except OSError:
            return
        key = (job["id"], str(f))
        if not append or getattr(self, "_log_key", None) != key:
            self.log_view.clear()
            self._log_key, self._log_pos = key, max(0, size - LOG_TAIL_BYTES)
        if size < self._log_pos:
            self._log_pos = 0
        if size == self._log_pos:
            return
        with open(f, "rb") as fh:
            fh.seek(self._log_pos)
            data = fh.read()
        self._log_pos += len(data)
        self.log_view.appendPlainText(data.decode("utf-8", errors="replace").rstrip("\n"))

    def _show_result(self, job):
        res = oc.load_result(job["out"]) if job["state"] == oc.DONE or Path(job["out"], "summary.json").is_file() else None
        job["result"] = {"gate_pass": res["gate_pass"]} if res else None
        if not res:
            return
        self.btn_report.setEnabled(bool(res["report"]))
        self.btn_outdir.setEnabled(True)
        self.btn_apply.setEnabled(job["state"] == oc.DONE)
        n_ok = sum(1 for c in res["cameras"] if c["pass"])
        head = (f"<b style='color:{OK_C if res['gate_pass'] else WARN_C}'>판정: {'통과' if res['gate_pass'] else '확인 필요'}</b>"
                f" · 카메라 {n_ok}/{len(res['cameras'])} 합격 · 구간 {res['windows']}개")
        if res["failures"]:
            head += "<br>" + "<br>".join(f"<span style='color:{ERR_C}'>{f}</span>" for f in res["failures"][:6])
        bad_layout = [r["name"] for r in res["layout"] if not r["pass"]]
        if bad_layout:
            head += f"<br><span style='color:{ERR_C}'>배치 규칙 실패: {', '.join(bad_layout)}</span>"
        self.lbl_result.setText(head)
        self.tbl_res.setRowCount(0)
        for c in res["cameras"]:
            r = self.tbl_res.rowCount()
            self.tbl_res.insertRow(r)
            vt = c["vote"] or {}
            vote = "–" if vt.get("pass") is None else (("pass" if vt["pass"] else "FAIL") if vt.get("gate", True) else "참고")
            cells = [c["camera"], "합격" if c["pass"] else "불합격", _f(c["rot_deg"], 3), _f(c["pos_mm"], 1),
                     _f(c["along_axis_mm"], 1), _f(c["focal_px"], 2), _f(c["track_reproj_px"], 2),
                     _f(c["heldout_px"], 2), vote]
            for k, v in enumerate(cells):
                it = _item(v, (OK_C if c["pass"] else ERR_C) if k == 1 else None,
                           tip="; ".join(c["reasons"]) if k == 1 and c["reasons"] else None)
                self.tbl_res.setItem(r, k, it)

    def show_report(self):
        job = self._sel()
        if not job:
            return
        f = Path(job["out"]) / "report.md"
        if not f.is_file():
            return
        dlg = QDialog(self)
        dlg.setWindowTitle(f"보고서 — {job['id']}")
        v = QVBoxLayout(dlg)
        tb = QTextBrowser()
        text = f.read_text(encoding="utf-8")
        if hasattr(tb, "setMarkdown"):
            tb.setMarkdown(text)
        else:
            tb.setPlainText(text)
        v.addWidget(tb)
        bb = QDialogButtonBox(QDialogButtonBox.Close)
        bb.rejected.connect(dlg.reject)
        v.addWidget(bb)
        dlg.resize(1000, 800)
        dlg.exec_()

    # ------------------------------------------------------------ 사전 점검 (nontarget_cal check)
    def run_check(self):
        if self.check_proc and self.check_proc.state() != QProcess.NotRunning:
            return
        bags = self._checked_bags()
        if not bags:
            QMessageBox.information(self, "사전 점검", "bag 을 고르세요.")
            return
        tool = self._tool()
        if not tool["ok"]:
            QMessageBox.warning(self, "사전 점검", tool["msg"])
            return
        block, _ = self.acquisition()
        if block:
            QMessageBox.warning(self, "사전 점검", f"{block} — 녹화 중에는 bag 을 읽지 않습니다.")
            return
        job = oc.new_job(bags, "zeroshot", self.ed_work.text().strip(), self.ed_out.text().strip(),
                         sensors=self._sensors() or ("rgb",), name_map=self.ed_namemap.text().strip() or None)
        self.check_proc = QProcess(self)
        self._check_buf = b""
        self._check_prog = oc.Progress(job["sensors"])
        self.check_proc.readyReadStandardOutput.connect(self._check_out)
        self.check_proc.finished.connect(lambda code, _s: self._check_done(code))
        self.check_proc.start("nice", ["-n", "10", tool["exe"], *oc.check_args(job)])
        self.btn_check.setEnabled(False)
        self.btn_check.setText("점검 중…")
        self._log("GUI", f"사전 점검 시작: {', '.join(Path(b).name for b in bags)}")

    def _check_out(self):
        self._check_buf += bytes(self.check_proc.readAllStandardOutput())
        *lines, self._check_buf = self._check_buf.split(b"\n")
        for ln in lines:
            self._check_prog.feed_line(ln.decode("utf-8", errors="replace"))

    def _check_done(self, code):
        self._check_out()
        if self._check_buf:
            self._check_prog.feed_line(self._check_buf.decode("utf-8", errors="replace"))
        self.btn_check.setEnabled(True)
        self.btn_check.setText("사전 점검 (약 30초)")
        p = self._check_prog
        if p.refusal:
            msg = f"거절 [{p.refusal.get('code')}]\n{p.refusal.get('msg_ko')}\n\n{p.refusal.get('msg')}"
            QMessageBox.warning(self, "사전 점검 — 거절", msg)
        elif p.check:
            c = p.check
            e, d = c.get("estimate") or {}, c.get("disk") or {}
            lines = [f"결과: {'통과' if c.get('ok') else '문제 있음'}",
                     f"도구 추정: 40 s 창 {e.get('windows_40s_equiv')}개 · 디스크 {e.get('disk_gb')} GB · "
                     f"시간 {e.get('runtime_min')}분 (도구 자체 추정, 보수적)",
                     f"점검 위치 여유 {d.get('free_gb')} GB (도구 기준 필요 {d.get('needed_gb')} GB)",
                     "카메라 이름: " + "; ".join(f"{b}: {s}" for b, s in (c.get("names") or {}).items())]
            for w in c.get("warnings") or []:
                lines.append(f"경고: {w.get('msg_ko') or w.get('msg')}")
            QMessageBox.information(self, "사전 점검", "\n".join(lines))
        else:
            err = (p.error or {}).get("msg") or f"exit {code}"
            QMessageBox.warning(self, "사전 점검", f"점검 실패: {err}")
        self._log("GUI", f"사전 점검 끝 (exit {code})")

    # ------------------------------------------------------------ 차량 파일 · 적용
    def _flir_group(self):
        if self.stage is not None and getattr(self.stage, "groups", None):
            return self.stage.groups.get(FLIR_GROUP)
        try:
            import sensor_discovery
            reg = sensor_discovery.load_registry()
            return next((g for g in reg.get("groups", []) if g.get("key") == FLIR_GROUP), None)
        except Exception:
            return None

    def vehicle_files(self):
        """(camera_info 경로, extrinsics 경로). 설정에서 직접 정했으면 그것."""
        g = self._flir_group() or {"workdir": "~/FLIR_control"}
        ov = (self.cfg.get("sensors") or {}).get(FLIR_GROUP) or {}
        defaults = {}
        try:
            import sensor_config
            defaults = {k: v for k, v in sensor_config.launch_defaults(g).items()
                        if k in ("camera_info_yaml_path", "extrinsics_yaml_path")}
        except Exception:
            pass
        ci, ex, _ = ca.vehicle_paths(g, ov, defaults)
        ci = Path(os.path.expanduser(self.c.get("camera_info_path") or ci))
        ex = Path(os.path.expanduser(self.c.get("extrinsics_path") or ex))
        return ci, ex

    def inventory(self):
        """{시리얼: {"name", "frame_id"}} — 차량 인벤토리 + GUI 의 이름 설정 (카메라 노드가 쓰는 frame_id)."""
        g = self._flir_group()
        if not g:
            return {}
        out = {}
        try:
            import sensor_config
            ov = (self.cfg.get("sensors") or {}).get(FLIR_GROUP) or {}
            for sub in sensor_config.subsets_of(g):
                repo = sensor_config.repo_inventory(sub)
                for serial in set(repo) | set(sensor_config.camera_overrides(dict(ov))):
                    e = sensor_config.camera_entry(sub, serial, dict(ov), repo)
                    out[str(serial)] = {"name": e.get("name"), "frame_id": e.get("frame_id")}
        except Exception:
            pass
        return out

    def _serial_map(self):
        cands = [self.c.get("serial_map")] if self.c.get("serial_map") else []
        for p in cands + [str(c) for c in SERIAL_MAP_CANDIDATES]:
            if p and Path(os.path.expanduser(p)).is_file():
                try:
                    return ca.load_serial_map(os.path.expanduser(p))
                except Exception:
                    continue
        return None

    def apply_dialog(self):
        job = self._sel()
        if not job or job["state"] != oc.DONE:
            return
        block, _ = self.acquisition()
        if block:
            QMessageBox.warning(self, "적용", f"{block} — 녹화 중에는 적용하지 않습니다.")
            return
        smap = self._serial_map()
        if smap is None:
            QMessageBox.warning(self, "적용", "카메라 시리얼 표(config/camera_serial_map.yaml)를 찾을 수 없습니다.")
            return
        dlg = ApplyDialog(self, job, smap, self.inventory(), *self.vehicle_files())
        if dlg.exec_() == QDialog.Accepted and dlg.manifest:
            m = dlg.manifest
            self._log("OK", f"차량에 적용: {len(m['cameras'])}대 → {', '.join(f['path'] for f in m['files'])} "
                            f"(보관 {m['archive']})")
            QMessageBox.information(self, "적용 완료",
                                    f"{len(m['cameras'])}대를 적용했습니다.\n\n" +
                                    "\n".join(f"{f['path']}\n  백업: {f.get('backup_sidecar') or '(새 파일)'}" for f in m["files"]) +
                                    "\n\nFLIR 카메라 센서군을 다시 기동해야 반영됩니다 (camera_info 와 /tf_static 은 "
                                    "노드가 켜질 때 읽음).\n되돌리기: [적용 기록 · 되돌리기…]")

    def history_dialog(self):
        HistoryDialog(self).exec_()


class ApplyDialog(QDialog):
    """카메라별로 시리얼 · 대상 frame 을 보여 주고 고른 것만 적용."""

    def __init__(self, tab, job, smap, inventory, ci_path, ex_path):
        super().__init__(tab)
        self.tab, self.job, self.smap, self.inv = tab, job, smap, inventory
        self.manifest = None
        self.setWindowTitle("차량 camera_info · TF 에 적용")
        self.result = ca.load_result_cameras(job["out"])
        summ = oc.load_result(job["out"]) or {"cameras": [], "gate_pass": False}
        self.gate = {c["camera"]: c["pass"] for c in summ["cameras"]}
        self.reasons = {c["camera"]: c["reasons"] for c in summ["cameras"]}
        v = QVBoxLayout(self)
        info = QLabel(
            "결과를 카메라 <b>시리얼</b>로 적어 넣습니다 (토픽 이름이 바뀌어도 다른 카메라에 붙지 않게). "
            "TF 는 <code>os_lidar → &lt;카메라&gt;_optical_frame</code> 으로 바뀝니다. 적용 전 두 파일은 "
            "<code>파일.bak_시각</code> 과 적용 기록 보관함에 백업되고, 쓴 뒤 카메라 노드와 같은 규칙으로 다시 읽어 "
            "확인합니다 (실패하면 자동으로 되돌림). 시리얼을 모르거나 표와 다르게 넣은 카메라, 판정 불합격 카메라는 "
            "직접 체크해야 적용됩니다.")
        info.setWordWrap(True)
        v.addWidget(info)
        form = QFormLayout()
        self.ed_ci = QLineEdit(str(ci_path))
        self.ed_ex = QLineEdit(str(ex_path))
        form.addRow("camera_info 파일", self.ed_ci)
        form.addRow("TF(extrinsics) 파일", self.ed_ex)
        v.addLayout(form)
        for ed in (self.ed_ci, self.ed_ex):
            if not Path(ed.text()).is_file():
                ed.setStyleSheet(f"color:{WARN_C};")
                ed.setToolTip("이 파일이 없습니다 — 적용하면 새로 만듭니다. 경로가 맞는지 확인")
        self.tbl = QTableWidget(0, 7)
        self.tbl.setHorizontalHeaderLabels(["적용", "카메라", "판정", "시리얼", "시리얼 근거", "차량 이름 / frame", "확인할 것"])
        self.tbl.horizontalHeader().setSectionResizeMode(6, QHeaderView.Stretch)
        self.tbl.verticalHeader().setVisible(False)
        v.addWidget(self.tbl, 1)
        self.tbl.itemChanged.connect(self._changed)
        self.overrides = {}
        self.confirmed = set()
        self._fill()
        bb = QDialogButtonBox()
        self.btn_ok = bb.addButton("적용", QDialogButtonBox.AcceptRole)
        _variant(self.btn_ok, "primary")
        bb.addButton("취소", QDialogButtonBox.RejectRole)
        bb.accepted.connect(self._apply)
        bb.rejected.connect(self.reject)
        v.addWidget(bb)
        self.resize(1150, 700)

    def _fill(self):
        self.rows = ca.plan_apply(self.result, self.smap, self.inv, self.overrides, gate=self.gate)
        self.tbl.blockSignals(True)
        self.tbl.setRowCount(0)
        for r in self.rows:
            i = self.tbl.rowCount()
            self.tbl.insertRow(i)
            chk = QTableWidgetItem()
            chk.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled if r["ok"] else Qt.ItemIsUserCheckable)
            on = r["selected"] or (r["ok"] and r["camera"] in self.confirmed)
            chk.setCheckState(Qt.Checked if on else Qt.Unchecked)
            r["selected"] = on
            self.tbl.setItem(i, 0, chk)
            self.tbl.setItem(i, 1, _item(r["camera"]))
            g = self.gate.get(r["camera"], True)
            self.tbl.setItem(i, 2, _item("합격" if g else "불합격", OK_C if g else ERR_C,
                                         tip="; ".join(self.reasons.get(r["camera"], []))))
            s = QTableWidgetItem(r["serial"] or "")
            s.setToolTip("고치려면 더블클릭 — 시리얼 표와 다르면 확인이 필요합니다")
            self.tbl.setItem(i, 3, s)
            self.tbl.setItem(i, 4, _item(r["serial_source"], tip=(self.smap.get(r["camera"]) or {}).get("evidence")))
            self.tbl.setItem(i, 5, _item(f"{r['name']} / {r['frame_id']}", tip=f"bag 토픽: {r['topic_in_bag']}"))
            notes = r["issues"] + r["needs_confirm"] + ([] if g else ["판정 불합격: " + "; ".join(self.reasons.get(r["camera"], []))])
            self.tbl.setItem(i, 6, _item("; ".join(notes) or "–", ERR_C if r["issues"] else (WARN_C if notes else None)))
        self.tbl.blockSignals(False)
        self.tbl.resizeColumnsToContents()
        self.tbl.horizontalHeader().setSectionResizeMode(6, QHeaderView.Stretch)

    def _changed(self, item):
        r = self.rows[item.row()]
        if item.column() == 3:
            self.overrides[r["camera"]] = item.text().strip()
            self.confirmed.discard(r["camera"])
            self._fill()
            return
        if item.column() != 0:
            return
        if item.checkState() == Qt.Checked:
            need = list(r["needs_confirm"]) + ([] if r.get("gate_pass", True) else ["판정 불합격"])
            if need and r["camera"] not in self.confirmed:
                if QMessageBox.question(self, "확인", f"{r['camera']} → 시리얼 {r['serial']}\n\n" + "\n".join(need) +
                                        "\n\n이대로 적용할까요?") != QMessageBox.Yes:
                    self.tbl.blockSignals(True)
                    item.setCheckState(Qt.Unchecked)
                    self.tbl.blockSignals(False)
                    return
                self.confirmed.add(r["camera"])
            r["selected"] = True
        else:
            r["selected"] = False
            self.confirmed.discard(r["camera"])

    def _apply(self):
        sel = [r for r in self.rows if r["selected"]]
        if not sel:
            QMessageBox.information(self, "적용", "적용할 카메라를 고르세요.")
            return
        _, warn = self.tab.acquisition()
        extra = ("\n\n주의: " + " · ".join(warn) + " — 반영은 FLIR 카메라 센서군을 다시 기동해야 됩니다.") if warn else ""
        if QMessageBox.question(self, "적용", f"{len(sel)}대를 적용합니다:\n{self.ed_ci.text()}\n{self.ed_ex.text()}\n"
                                "(둘 다 백업한 뒤 씀)" + extra) != QMessageBox.Yes:
            return
        try:
            self.manifest = ca.apply_result(self.job["out"], self.rows, self.ed_ci.text().strip(),
                                            self.ed_ex.text().strip(), oc.APPLIED_ROOT,
                                            source_label=self.job["id"],
                                            tool_commit=str((self.job.get("tool") or {}).get("commit") or ""))
        except Exception as e:
            QMessageBox.critical(self, "적용 실패", f"{e}\n\n파일은 바뀌지 않았습니다 (또는 백업으로 되돌렸습니다).")
            return
        self.accept()


class HistoryDialog(QDialog):
    def __init__(self, tab):
        super().__init__(tab)
        self.tab = tab
        self.setWindowTitle("적용 기록 · 되돌리기")
        v = QVBoxLayout(self)
        self.tbl = QTableWidget(0, 5)
        self.tbl.setHorizontalHeaderLabels(["적용 시각", "카메라", "결과", "파일", "상태"])
        self.tbl.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.tbl.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl.verticalHeader().setVisible(False)
        v.addWidget(self.tbl, 1)
        h = QHBoxLayout()
        b = QPushButton("선택한 적용 되돌리기")
        _variant(b, "danger")
        b.clicked.connect(self._rollback)
        o = QPushButton("보관함 열기")
        o.clicked.connect(lambda: _open(oc.APPLIED_ROOT))
        h.addWidget(b)
        h.addWidget(o)
        h.addStretch(1)
        c = QPushButton("닫기")
        c.clicked.connect(self.reject)
        h.addWidget(c)
        v.addLayout(h)
        self._fill()
        self.resize(1000, 420)

    def _fill(self):
        self.items = ca.list_applied(oc.APPLIED_ROOT)
        self.tbl.setRowCount(0)
        for m in self.items:
            r = self.tbl.rowCount()
            self.tbl.insertRow(r)
            self.tbl.setItem(r, 0, _item(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(m["applied_at"]))))
            self.tbl.setItem(r, 1, _item(f"{len(m['cameras'])}대", tip="\n".join(
                f"{c['camera']} → {c['serial']} ({c['frame_id']})" for c in m["cameras"])))
            self.tbl.setItem(r, 2, _item(Path(m["result_dir"]).parent.name, tip=m["result_dir"]))
            self.tbl.setItem(r, 3, _item(" · ".join(f["path"] for f in m["files"])))
            changed = ca.rollback_check(m)
            st = "되돌림" if m.get("rolled_back") else ("이후 파일이 바뀜" if changed else "적용 중 (현재 파일)")
            self.tbl.setItem(r, 4, _item(st, MUTED_C if m.get("rolled_back") else (WARN_C if changed else OK_C)))

    def _rollback(self):
        rows = self.tbl.selectionModel().selectedRows()
        if not rows:
            return
        m = self.items[rows[0].row()]
        if m.get("rolled_back"):
            QMessageBox.information(self, "되돌리기", "이미 되돌린 적용입니다.")
            return
        changed = ca.rollback_check(m)
        msg = "이 적용 직전의 파일로 되돌립니다:\n" + "\n".join(f["path"] for f in m["files"])
        if changed:
            msg += ("\n\n주의: 이 적용 뒤에 파일이 또 바뀌었습니다 (다른 적용 · 손으로 고침). 되돌리면 그 변경도 사라집니다 "
                    "(지금 파일은 보관함 before_rollback/ 에 남김):\n" + "\n".join(changed))
        if QMessageBox.question(self, "되돌리기", msg) != QMessageBox.Yes:
            return
        try:
            ca.rollback(m, force=bool(changed))
        except Exception as e:
            QMessageBox.critical(self, "되돌리기 실패", str(e))
            return
        self.tab._log("WARN", f"적용 되돌림: {m['id']}")
        QMessageBox.information(self, "되돌리기", "되돌렸습니다. FLIR 카메라 센서군을 다시 기동해야 반영됩니다.")
        self._fill()
