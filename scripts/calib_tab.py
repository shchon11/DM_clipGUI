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
from PyQt5.QtGui import QColor, QPixmap
from PyQt5.QtWidgets import (
    QAbstractItemView, QButtonGroup, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFileDialog,
    QFormLayout, QFrame, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox, QPlainTextEdit,
    QProgressBar, QPushButton, QRadioButton, QSizePolicy, QSplitter, QTableWidget, QTableWidgetItem, QTextBrowser,
    QVBoxLayout, QWidget, QTabWidget,
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


def _heading(text):
    lb = QLabel(text)
    lb.setStyleSheet("font-size:17px; font-weight:700; padding:2px 0 4px 0;")
    return lb


def _step(num, title, hint=""):
    lb = QLabel(f"<span style='font-size:15px; font-weight:700;'>{num} {title}</span>"
                + (f"<br><span style='color:{MUTED_C};'>{hint}</span>" if hint else ""))
    lb.setWordWrap(True)
    lb.setTextFormat(Qt.RichText)
    lb.setStyleSheet("padding-top:8px;")
    return lb


def _hm(sec):
    """초 → '1시간 5분' · '12분' · '1분 미만'."""
    m = int(round(max(0.0, sec) / 60))
    if m < 1:
        return "1분 미만"
    return f"{m // 60}시간 {m % 60}분" if m >= 60 else f"{m}분"


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
    sig_title = pyqtSignal(str)             # 메인 창 탭 이름 (도는 중이면 진행률 · 남은 시간)

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
        self.ros_worker = None          # clip_gui 가 넣는다 (실시간 projection 보기)
        self.release = None             # clip_gui 가 넣는다: 센서 · 레코더를 끄고 끈 것 이름 목록을 돌려준다
        self._proj_dlg = None
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

        # ---------------- 맨 위: 지금 차량에 들어가 있는 캘값 (확인 · 되돌리기)
        top = QFrame()
        top.setStyleSheet("QFrame#calibTop{background:#eef2ff; border-radius:8px;}")
        top.setObjectName("calibTop")
        th = QHBoxLayout(top)
        th.setContentsMargins(12, 8, 12, 8)
        self.lbl_vehicle = QLabel()
        self.lbl_vehicle.setWordWrap(True)
        self.lbl_vehicle.setTextFormat(Qt.RichText)
        th.addWidget(self.lbl_vehicle, 1)
        self.btn_proj = QPushButton("실시간 projection 보기")
        _variant(self.btn_proj, "primary")
        self.btn_proj.setToolTip("센서가 켜져 있을 때: 카메라 노드가 지금 쓰는 camera_info · TF 로 라이다를 영상 위에 그린다")
        self.btn_proj.clicked.connect(self.projection_dialog)
        self.btn_hist = QPushButton("적용 기록 · 되돌리기")
        self.btn_hist.clicked.connect(self.history_dialog)
        # 지금 차량 값을 이미 녹화한 bag 의 camera_info · /tf_static 에도 (recalib_dialog · bag_recalib)
        self.btn_recalib = QPushButton("이전 녹화에도 적용…")
        self.btn_recalib.setToolTip("지금 차량에 들어가 있는 캘리브레이션을 이미 녹화한 bag 의 camera_info 와 /tf_static 에 넣습니다 "
                                    "(녹화마다 진행 막대 · 되돌리기 가능)")
        self.btn_recalib.clicked.connect(self.recalib_dialog)
        th.addWidget(self.btn_proj)
        th.addWidget(self.btn_hist)
        th.addWidget(self.btn_recalib)
        outer.addWidget(top)

        split = QSplitter(Qt.Horizontal)
        split.setChildrenCollapsible(False)
        outer.addWidget(split, 1)

        # ---------------- 왼쪽: 새 캘리브레이션 (① 데이터 → ② 시작)
        left = QWidget()
        self.setup_panel = left
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 6, 8, 0)
        lv.addWidget(_heading("새 캘리브레이션"))

        lv.addWidget(_step("①", "캘리브레이션에 쓸 주행 데이터를 고르세요",
                           "회전이 많은 주행일수록 좋습니다. 합쳐서 최소 80초, 권장 5분 이상 · 같은 날 같은 카메라 이름으로 녹화한 것끼리"))
        self.tbl_bags = QTableWidget(0, 6)
        self.tbl_bags.setHorizontalHeaderLabels(["녹화", "길이", "크기", "카메라", "상태", "경로"])
        self.tbl_bags.setColumnHidden(5, True)
        self.tbl_bags.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl_bags.verticalHeader().setVisible(False)
        self.tbl_bags.itemChanged.connect(lambda *_: self._update_estimate())
        lv.addWidget(self.tbl_bags, 1)
        row = QHBoxLayout()
        self.ed_bag_root = QLineEdit(self.c.get("bag_root") or self.cfg.get("recorder", {}).get("output_dir", ""))
        self.ed_bag_root.setToolTip("이 폴더와 그 안 route_* 폴더의 녹화(rec_* · clip_*)를 목록에 보인다")
        b = QPushButton("폴더…")
        b.clicked.connect(self._pick_bag_root)
        r = QPushButton("새로고침")
        r.clicked.connect(self.refresh_bags)
        add = QPushButton("다른 bag 추가…")
        add.clicked.connect(self._add_bag)
        row.addWidget(QLabel("녹화 폴더"))
        row.addWidget(self.ed_bag_root, 1)
        for w in (b, r, add):
            row.addWidget(w)
        lv.addLayout(row)
        self.lbl_est = QLabel("위 목록에서 녹화를 체크하세요")
        self.lbl_est.setWordWrap(True)
        self.lbl_est.setStyleSheet("padding:4px 0;")
        lv.addWidget(self.lbl_est)

        lv.addWidget(_step("②", "시작",
                           "켜져 있는 센서와 레코더는 자동으로 끕니다 (녹화 중이면 시작하지 않음). "
                           "시간은 위의 예상 시간만큼 걸리고, GUI 를 닫아도 계속 돕니다."))
        self.btn_start = QPushButton("센서 · 레코더 끄고 캘리브레이션 시작")
        _variant(self.btn_start, "primary")
        self.btn_start.setMinimumHeight(40)
        self.btn_start.clicked.connect(self.start_new)
        lv.addWidget(self.btn_start)
        self.lbl_acq = QLabel()
        self.lbl_acq.setWordWrap(True)
        lv.addWidget(self.lbl_acq)

        # 고급 설정 (접힘): 시작 값 · 저장 위치 · 센서 · --force · 이름 대응표 · 사전 점검
        self.btn_adv = QPushButton("▸ 고급 설정")
        self.btn_adv.setCheckable(True)
        self.btn_adv.setFlat(True)
        self.btn_adv.setStyleSheet("text-align:left;")
        lv.addWidget(self.btn_adv)
        adv = QWidget()
        self.adv_panel = adv
        av = QVBoxLayout(adv)
        av.setContentsMargins(12, 0, 0, 0)
        self.btn_adv.toggled.connect(lambda on: (adv.setVisible(on),
                                                 self.btn_adv.setText(("▾" if on else "▸") + " 고급 설정")))
        adv.setVisible(False)
        self.rb_zero = QRadioButton("처음부터 (zero-shot) — 설계값에서 시작. 렌즈 · 카메라를 만졌을 때도 이것")
        self.rb_warm = QRadioButton("지금 캘값에서 시작 (warm-start) — 빨라짐. 필요한 데이터 양은 같음")
        grp = QButtonGroup(self)
        grp.addButton(self.rb_zero)
        grp.addButton(self.rb_warm)
        (self.rb_warm if self.c.get("mode") == "warm" else self.rb_zero).setChecked(True)
        av.addWidget(self.rb_zero)
        av.addWidget(self.rb_warm)
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
        av.addLayout(wr)
        for w in (self.rb_zero, self.rb_warm):
            w.toggled.connect(self._mode_changed)
        self.cb_init.currentIndexChanged.connect(self._mode_changed)
        f3 = QFormLayout()
        default_root = str(Path(os.path.expanduser(self.cfg.get("recorder", {}).get("output_dir", "~/DM_clipGUI/clips"))).parent
                           / "online_calib")
        self.ed_work = QLineEdit(self.c.get("workdir_root") or default_root)
        self.ed_work.setToolTip("큰 중간 파일(약 80 GB)이 쌓이는 곳 — 여유가 큰 디스크. 작업마다 <여기>/<작업 id>/work")
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
        self.chk_force = QCheckBox("--force (데이터 양 · 동기 거절 무시 — 결과를 믿기 어려울 수 있음)")
        f3.addRow("", self.chk_force)
        self.ed_namemap = QLineEdit(self.c.get("name_map", ""))
        self.ed_namemap.setPlaceholderText("비우면 시리얼로 자동 생성 (도구가 영상 기하로 다시 확인)")
        f3.addRow("이름 대응표", self.ed_namemap)
        av.addLayout(f3)
        ck = QHBoxLayout()
        self.btn_check = QPushButton("사전 점검만 (약 30초)")
        self.btn_check.setToolTip("nontarget_cal check — 이름 대응 · 움직임 · 회전 · 동기 · 노출 · 디스크를 bag 에서 가볍게 확인")
        self.btn_check.clicked.connect(self.run_check)
        self.btn_import = QPushButton("다른 PC 결과 불러오기…")
        self.btn_import.setToolTip("다른 PC 에서 계산한 결과 폴더(summary.json · extrinsic/ · intrinsic/)를 목록에 올려 "
                                   "[차량에 적용] 할 수 있게 한다")
        self.btn_import.clicked.connect(self.import_result)
        ck.addWidget(self.btn_check)
        ck.addWidget(self.btn_import)
        ck.addStretch(1)
        av.addLayout(ck)
        self.lbl_tool = QLabel()
        self.lbl_tool.setWordWrap(True)
        self.lbl_tool.setStyleSheet(f"color:{MUTED_C};")
        av.addWidget(self.lbl_tool)
        lv.addWidget(adv)
        split.addWidget(left)

        # ---------------- 오른쪽: ③ 진행 · 결과
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(8, 6, 0, 0)
        rv.addWidget(_heading("③ 진행 · 결과"))
        self.tbl_jobs = QTableWidget(0, 5)
        self.tbl_jobs.setHorizontalHeaderLabels(["시작", "녹화", "방식", "상태", "진행"])
        for k in (0, 2, 4):
            self.tbl_jobs.horizontalHeader().setSectionResizeMode(k, QHeaderView.ResizeToContents)
        self.tbl_jobs.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.tbl_jobs.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl_jobs.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl_jobs.verticalHeader().setVisible(False)
        self.tbl_jobs.itemSelectionChanged.connect(self._show_job)
        self.tbl_jobs.setMaximumHeight(110)
        rv.addWidget(self.tbl_jobs)
        self.lbl_job = QLabel("아직 캘리브레이션 작업이 없습니다 — 왼쪽 ① ② 부터")
        self.lbl_job.setWordWrap(True)
        self.lbl_job.setTextFormat(Qt.RichText)
        self.lbl_job.setStyleSheet("font-size:14px; padding:4px 0;")
        rv.addWidget(self.lbl_job)
        # 진행 배너 — 지금 단계 · 얼마나 됐나 · 얼마나 남았나 (도는 중 · 끝난 작업)
        self.banner = QFrame()
        self.banner.setObjectName("calibBanner")
        self.banner.setStyleSheet("QFrame#calibBanner{background:#eff6ff; border:1px solid #bfdbfe; border-radius:8px;}")
        bv = QVBoxLayout(self.banner)
        bv.setContentsMargins(12, 8, 12, 10)
        self.lbl_now = QLabel()
        self.lbl_now.setTextFormat(Qt.RichText)
        self.lbl_now.setWordWrap(True)
        self.lbl_now.setStyleSheet("font-size:19px; font-weight:700;")
        bv.addWidget(self.lbl_now)
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        self.bar.setFormat("%p%")
        self.bar.setMinimumHeight(34)
        self.bar.setStyleSheet("QProgressBar{font-size:17px; font-weight:700;}")
        bv.addWidget(self.bar)
        self.lbl_eta = QLabel()
        self.lbl_eta.setTextFormat(Qt.RichText)
        self.lbl_eta.setStyleSheet("font-size:16px;")
        bv.addWidget(self.lbl_eta)
        self.banner.hide()
        rv.addWidget(self.banner)
        jb = QHBoxLayout()
        self.btn_apply = QPushButton("이 결과를 차량에 적용…")
        _variant(self.btn_apply, "primary")
        self.btn_apply.clicked.connect(self.apply_dialog)
        self.btn_cancel = QPushButton("중단")
        _variant(self.btn_cancel, "danger")
        self.btn_cancel.clicked.connect(lambda: self.cancel_job(reason="사용자가 중단"))
        self.btn_resume = QPushButton("이어서 실행")
        self.btn_resume.clicked.connect(self.resume_job)
        self.btn_recheck = QPushButton("공간 다시 확인")
        self.btn_recheck.clicked.connect(self.recheck_job)
        self.btn_move = QPushButton("작업 폴더 바꾸기…")
        self.btn_move.clicked.connect(self.move_workdir)
        self.btn_report = QPushButton("보고서")
        self.btn_report.clicked.connect(self.show_report)
        self.btn_outdir = QPushButton("결과 폴더")
        self.btn_outdir.clicked.connect(lambda: self._sel() and _open(self._sel()["out"]))
        self.btn_logs = QPushButton("로그 폴더")
        self.btn_logs.clicked.connect(lambda: self._sel() and _open(oc.job_dir(self._sel())))
        self.btn_del = QPushButton("삭제…")
        self.btn_del.clicked.connect(self.delete_job)
        self._job_buttons = (self.btn_apply, self.btn_cancel, self.btn_resume, self.btn_recheck, self.btn_move,
                             self.btn_report, self.btn_outdir, self.btn_logs, self.btn_del)
        for w in self._job_buttons[:5]:
            jb.addWidget(w)
        jb.addStretch(1)
        for w in self._job_buttons[5:]:
            jb.addWidget(w)
        rv.addLayout(jb)

        self.detail_tabs = QTabWidget()
        # 진행: 단계 목록 + 전체 막대 + 경과 · 남은 시간
        prog = QWidget()
        pv = QVBoxLayout(prog)
        self.lbl_stage = QLabel()
        self.lbl_stage.setWordWrap(True)
        self.lbl_stage.setTextFormat(Qt.RichText)
        self.lbl_stage.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.lbl_stage.setStyleSheet("font-size:14px; line-height:150%;")
        pv.addWidget(self.lbl_stage, 1)
        self.detail_tabs.addTab(prog, "단계별 진행")
        # 결과: 기존 캘 vs 새 결과 판정 (숫자) · 전/후 투영 비교 (눈)
        self.cmp = CompareView(self)
        self.detail_tabs.addTab(self.cmp, "판정 — 기존 vs 새 결과")
        self.ba = BeforeAfterView(self)
        self.detail_tabs.addTab(self.ba, "전/후 투영 비교")
        # 카메라별 수치
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
        self.detail_tabs.addTab(res, "카메라별 수치")
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(3000)
        self.log_view.setStyleSheet("background:#0f172a; color:#e2e8f0; font-family:monospace; font-size:11px;")
        self.detail_tabs.addTab(self.log_view, "작업 로그")
        rv.addWidget(self.detail_tabs, 1)
        split.addWidget(right)
        split.setSizes([560, 760])
        self._mode_changed()
        self._update_tool_label()
        self._update_vehicle_label()

    # ------------------------------------------------------------ 공용
    def current_title(self):
        return getattr(self, "_title", "온라인 캘리브레이션")

    def _update_vehicle_label(self):
        """맨 위: 지금 차량 파일에 들어가 있는 캘값 (마지막 적용 기록 + 파일에서 os_lidar 에 붙은 카메라 수)."""
        try:
            ci, ex = self.vehicle_files()
            items = ca.parse_extrinsics_like_node(Path(ex).read_text(encoding="utf-8"))
            n = sum(1 for it in items if it["parent_frame"] == ca.PARENT_FRAME)
        except Exception as e:                                  # noqa: BLE001
            self.lbl_vehicle.setText(f"<b>지금 차량 캘값</b>: 차량 파일을 못 읽음 — {e}")
            return
        last = next((m for m in ca.list_applied(oc.APPLIED_ROOT) if not m.get("rolled_back")), None)
        if not n:
            txt = (f"<b>지금 차량 캘값: <span style='color:{ERR_C}'>없음</span></b> — 자리표시 값뿐입니다. "
                   "아래에서 캘리브레이션을 돌리거나 결과를 적용하세요.")
        else:
            src = ""
            if last:
                rd = Path(last.get("result_dir", ""))
                name = rd.parent.name if rd.name == "out" else rd.name      # <작업 id>/out → 작업 id
                src = (f" · {name or '?'} 결과, "
                       f"{time.strftime('%m-%d %H:%M', time.localtime(last['applied_at']))} 적용")
            txt = f"<b>지금 차량 캘값</b>: 카메라 {n}대 라이다 기준으로 들어가 있음{src}"
        self.lbl_vehicle.setText(txt)

    def shutdown(self):
        self.timer.stop()
        self.ba.stop()
        self.cmp.stop("GUI 종료")

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)

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
            warn.append("센서가 켜져 있음")
        if s.get("recorder"):
            warn.append("레코더가 돌고 있음")
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
            # 녹화는 output_dir 바로 아래 또는 route_NNN_<지역>/ 안 (dataset_catalog) — 둘 다, 최신 순
            found = []
            for parent in [root] + sorted(p for p in root.iterdir() if p.name.startswith("route_") and p.is_dir()):
                try:
                    children = list(parent.iterdir())
                except OSError:                  # 읽을 수 없는 폴더 — 건너뛴다
                    continue
                for d in children:
                    # 이름을 먼저 본다: /mnt/data/lost+found 처럼 root 만 읽는 폴더 안을 stat 하면 PermissionError
                    if not d.name.startswith(("rec_", "clip_")):
                        continue
                    try:
                        if d.is_dir() and (d / "metadata.yaml").is_file():
                            found.append(d)
                    except OSError:
                        continue
            found.sort(key=lambda d: (d.name.split("_", 1)[-1], d.name), reverse=True)
            paths = [str(d) for d in found]
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
            short = [f"{k} {n}/{ref}대" for k, n, ref in (("RGB", b["n_rgb"], 14), ("열화상", b["n_thermal"], 2))
                     if 0 < n < ref]
            if b["ok"] and short:       # 빠진 카메라 — 그 센서는 도구가 거절한다 (이름 확인 단계)
                self.tbl_bags.setItem(r, 4, _item("카메라 빠짐", WARN_C,
                                                  tip=", ".join(short) + " — 빠진 카메라가 있어 그 센서는 캘리브레이션할 수 없습니다"))
            else:
                short_err = b["error"].split(" (")[0].split(",")[0] if b["error"] else ""
                self.tbl_bags.setItem(r, 4, _item("OK" if b["ok"] else short_err, OK_C if b["ok"] else ERR_C,
                                                  tip=b["error"]))
            self.tbl_bags.setItem(r, 5, _item(p))
        self.tbl_bags.blockSignals(False)
        hh = self.tbl_bags.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.Stretch)
        for k in range(1, 5):
            hh.setSectionResizeMode(k, QHeaderView.ResizeToContents)
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
            self.lbl_est.setText("위 목록에서 녹화를 체크하세요")
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
        """녹화 중이면 막는다. 센서 · 레코더가 켜져 있으면 끈다 (clip_gui 가 넣은 release). False = 시작 안 함."""
        block, warn = self.acquisition()
        if block:
            QMessageBox.warning(self, "캘리브레이션", f"{block} — 녹화를 끝낸 뒤 시작하세요.")
            return False
        if warn:
            if self.release is None:
                QMessageBox.warning(self, "캘리브레이션", " · ".join(warn) + " — 끈 뒤 시작하세요.")
                return False
            try:
                stopped = self.release()
            except Exception as e:                              # noqa: BLE001
                QMessageBox.warning(self, "캘리브레이션", f"센서 · 레코더를 끄지 못했습니다: {e}")
                return False
            self._log("GUI", f"캘리브레이션을 위해 끔: {', '.join(stopped) or '없음'}")
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
        total = sum(i["duration_s"] for i in infos)
        if total < oc.MIN_TOTAL_MOVING_S and not self.chk_force.isChecked():
            QMessageBox.warning(self, "캘리브레이션",
                                f"고른 bag 을 모두 합쳐 {total:.0f} s — 최소 {oc.MIN_TOTAL_MOVING_S:.0f} s 주행이 필요합니다 "
                                "(멈춰 있던 시간은 빠지므로 실제로는 더). 짧은 클립은 여러 개 같이 고르세요.")
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
        if not self._attach_name_map(job, infos, "캘리브레이션"):
            return
        if mode == "warm":
            init, why = self._resolve_init(job)
            if not init:
                QMessageBox.warning(self, "warm-start", why)
                return
            job.update(mode="warm", init=str(init), init_note=why)
        self._snapshot_before(job)
        est = oc.estimate_storage(infos, job["sensors"])
        job["estimate"] = est
        self.store.add(job)
        self._start_or_pend(job, est)
        self.refresh_jobs(select=job["id"])

    def _snapshot_before(self, job):
        """시작할 때 차량에 들어가 있던 캘값을 작업 폴더에 남긴다 — 끝난 뒤 전/후 투영 비교의 '전'."""
        smap = self._serial_map()
        if smap is None:
            return
        try:
            ci, ex = self.vehicle_files()
            r = ca.export_vehicle_init(ci, ex, smap, oc.job_dir(job) / "before_vehicle")
            job["before"] = {"dir": r["dir"], "cameras": len(r["cameras"]),
                             "label": f"시작할 때 차량 값 ({time.strftime('%m-%d %H:%M')})"}
        except Exception as e:                                  # noqa: BLE001
            self._log("WARN", f"시작 때 차량 캘값을 남기지 못함 (전/후 비교는 그때의 차량 값으로): {e}")

    def _before_dir(self, job):
        """전/후 비교의 '전': (폴더, 이름) 또는 (None, 이유)."""
        b = job.get("before") or {}
        if b.get("dir") and b.get("cameras") and (Path(b["dir"]) / "extrinsic").is_dir():
            return b["dir"], b.get("label", "시작할 때 차량 값")
        if job.get("mode") == "warm" and job.get("init") and (Path(job["init"]) / "extrinsic").is_dir():
            return job["init"], "시작 값 (warm-start)"
        # 시작 때 기록이 없는 (예전) 작업: 적용 기록에서 작업을 시작한 시각에 들어가 있던 것
        # (지금 차량 값은 이 결과를 이미 적용했으면 '후' 와 같아진다)
        t0 = job.get("created") or 0
        for m in ca.list_applied(oc.APPLIED_ROOT):             # 최신부터
            if m.get("applied_at", 0) >= t0:
                continue
            arch = Path(m.get("archive", "")) / "result"
            if (arch / "extrinsic").is_dir() and Path(m.get("result_dir", "")).resolve() != Path(job["out"]).resolve():
                when = time.strftime("%m-%d %H:%M", time.localtime(m["applied_at"]))
                return str(arch), f"시작 때 차량 값 ({Path(m.get('result_dir', '')).name} 결과, {when} 적용)"
            break
        smap = self._serial_map()
        if smap is None:
            return None, "카메라 시리얼 표가 없음"
        try:
            ci, ex = self.vehicle_files()
            r = ca.export_vehicle_init(ci, ex, smap, oc.job_dir(job) / "before_now")
        except Exception as e:                                  # noqa: BLE001
            return None, f"차량 파일을 못 읽음: {e}"
        if not r["cameras"]:
            return None, "차량에 적용된 캘값이 없음 (자리표시 값뿐) — 비교할 '전' 이 없습니다"
        last = next((m for m in ca.list_applied(oc.APPLIED_ROOT) if not m.get("rolled_back")), None)
        if last and Path(last.get("result_dir", "")).resolve() == Path(job["out"]).resolve():
            return None, "지금 차량 값이 이미 이 결과라 비교할 '전' 이 없습니다"
        return r["dir"], "지금 차량 값"

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

    def import_result(self):
        d = QFileDialog.getExistingDirectory(self, "nontarget_cal 결과 폴더 (summary.json 이 있는 out 폴더)",
                                             str(Path(__file__).resolve().parent.parent / "calib_results"))
        if not d:
            return
        try:
            job = oc.imported_job(d)
        except Exception as e:
            QMessageBox.warning(self, "결과 불러오기", str(e))
            return
        self.store.add(job)
        self._log("OK", f"결과 폴더를 불러왔습니다: {job['out']}")
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
        if running:
            self.lbl_acq.setText(f"<span style='color:#2563eb'>● 캘리브레이션이 도는 중입니다 — 오른쪽 ③ 에서 진행을 보세요</span>")
        elif block:
            self.lbl_acq.setText(f"<b style='color:{ERR_C}'>● {block} — 녹화를 끝낸 뒤 시작할 수 있습니다</b>")
        elif warn:
            self.lbl_acq.setText(f"<span style='color:{MUTED_C}'>● 지금 {' · '.join(warn)} — 시작하면 자동으로 끕니다</span>")
        else:
            self.lbl_acq.setText(f"<span style='color:{OK_C}'>● 시작할 수 있습니다</span>")
        self.btn_start.setEnabled(not block and not running)
        if block and self.cmp.running():
            self.cmp.stop(f"{block} — 비교 중단 (나중에 [비교 실행])")
        if block and running and not running["attempts"][-1].get("stop_reason"):
            self.cancel_job(running, reason=f"{block} — 데이터 수집을 위해 자동 중단", ask=False)
        title = "온라인 캘리브레이션"
        if running:
            p = self._feed(running)
            if p:
                el = time.time() - running["attempts"][-1]["t0"]
                rem = p.remaining_s(el)
                title += f" ▶ {100 * p.fraction():.0f}%" + (f" · 남은 {_hm(rem)}" if rem is not None else "")
        if title != getattr(self, "_title", None):
            self._title = title
            self.sig_title.emit(title)
        changed = False
        for j in self.store.jobs:
            if j["state"] == oc.RUNNING:
                self._feed(j)
                changed |= oc.settle_state(j)
                if j["state"] != oc.RUNNING:
                    self._feed(j)
                    self._log("OK" if j["state"] == oc.DONE else "WARN",
                              f"{j['id']}: {oc.STATE_LABEL[j['state']]} {j.get('state_msg', '')}")
                    if j["state"] == oc.DONE:
                        self.cmp.start(j, self._before_dir(j))      # 끝나면 바로 기존 캘과 비교
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
            # 이어서 실행하면 도구가 사전 점검을 건너뛰어 시간 추정이 안 온다 — 앞 시도의 것을 쓴다
            for prev in reversed(job["attempts"][:-1]):
                q = oc.Progress(job["sensors"])
                q.feed_file(prev["stdout"])
                if q.tool_estimate:
                    p.tool_estimate = q.tool_estimate
                    break
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
        for w in self._job_buttons:
            w.setVisible(False)
        self.log_view.clear()
        self.tbl_res.setRowCount(0)
        self.lbl_result.setText("")
        if not job:
            self.lbl_job.setText("아직 캘리브레이션 작업이 없습니다 — 왼쪽 ① 에서 녹화를 고르고 ② 시작")
            self.banner.hide()
            self.lbl_stage.setText("")
            self.ba.set_job(None)
            return
        st = job["state"]
        show = {self.btn_cancel: st == oc.RUNNING, self.btn_resume: st in oc.RESUMABLE,
                self.btn_recheck: st == oc.PENDING,
                self.btn_move: st in (oc.PENDING, oc.INTERRUPTED, oc.CANCELLED, oc.FAILED),
                self.btn_logs: True, self.btn_del: st != oc.RUNNING}
        for w, on in show.items():
            w.setVisible(on)
        color = {oc.DONE: OK_C, oc.RUNNING: "#2563eb", oc.REFUSED: ERR_C, oc.FAILED: ERR_C}.get(st, WARN_C)
        what = {oc.RUNNING: "도는 중", oc.DONE: "끝남", oc.PENDING: "대기 (공간 부족 등)", oc.REFUSED: "거절됨 (데이터 문제)",
                oc.FAILED: "오류로 멈춤", oc.INTERRUPTED: "끊김", oc.CANCELLED: "중단함"}.get(st, oc.STATE_LABEL.get(st, st))
        bags = ", ".join(Path(b).name for b in job["bags"][:3]) + (f" 외 {len(job['bags']) - 3}개" if len(job["bags"]) > 3 else "")
        txt = (f"<b style='color:{color}'>● {what}</b> · 녹화 {len(job['bags'])}개 ({bags}) · "
               f"{'지금 캘값에서 시작' if job['mode'] == 'warm' else '처음부터'}")
        if job.get("state_msg") and st != oc.DONE:
            txt += f"<br><span style='color:{color}'>{job['state_msg']}</span>"
        if st in oc.RESUMABLE and st != oc.PENDING:
            txt += "<br>끝난 단계는 남아 있습니다 — [이어서 실행] 하면 거기서부터 합니다."
        self.lbl_job.setText(txt)
        self.lbl_job.setToolTip(f"{job['id']}\n작업 폴더 {job['workdir']}\n결과 {job['out']}")
        self._feed(job)
        self._show_progress()
        self._load_log(job)
        self._show_result(job)
        # 도는 중이면 진행, 끝났으면 판정을 먼저
        want = 1 if st == oc.DONE else 0
        if getattr(self, "_shown_job", None) != (job["id"], st):
            self._shown_job = (job["id"], st)
            self.detail_tabs.setCurrentIndex(want)

    def _show_progress(self):
        job = self._sel()
        if not job:
            return
        p = self._feed(job)
        st = job["state"]
        self.banner.setVisible(st in (oc.RUNNING, oc.DONE) or bool(p))
        if not p:
            self.bar.setValue(1000 if st == oc.DONE else 0)
            self.lbl_stage.setText("")
            self.lbl_now.setText("✓ 끝남" if st == oc.DONE else "아직 시작 안 함")
            self.lbl_eta.setText("")
            return
        f = p.fraction()
        self.bar.setValue(int(1000 * (1.0 if st == oc.DONE else f)))
        # 단계 목록 (탭)
        rows = []
        done = st == oc.DONE
        for k, lbl, _ in p.expected():
            s_ = p.stages[k]
            if done and s_["state"] != "fail":           # 이어서 실행한 시도는 캐시된 단계를 다 알리지 않는다
                rows.append(f"<span style='color:{OK_C}'>✓ {lbl}</span>")
            elif s_["state"] == "run":
                n = f" — {s_['done']}/{s_['total']}" if s_["total"] else ""
                rows.append(f"<b style='color:#2563eb'>▶ {lbl}{n}</b>")
            elif s_["state"] in ("ok", "skip"):
                rows.append(f"<span style='color:{OK_C}'>✓ {lbl}</span>")
            elif s_["state"] == "fail":
                rows.append(f"<b style='color:{ERR_C}'>✗ {lbl}</b>")
            else:
                rows.append(f"<span style='color:{MUTED_C}'>○ {lbl}</span>")
        extra = ""
        if p.refusal:
            extra += f"<br><b style='color:{ERR_C}'>거절: {oc.name_bags(p.refusal.get('msg_ko') or p.refusal.get('msg'), job['bags'])}</b>"
        if p.task_failed:
            extra += f"<br><span style='color:{ERR_C}'>실패한 세부 작업 {len(p.task_failed)}개 — [작업 로그] 탭</span>"
        if p.warnings:
            extra += "<br>" + "<br>".join(f"<span style='color:{WARN_C}'>⚠ {oc.name_bags(w.get('msg_ko') or w.get('msg'), job['bags'])}</span>"
                                          for w in p.warnings[-4:])
        self.lbl_stage.setText("<br>".join(rows) + extra)
        # 배너
        att = job["attempts"][-1] if job["attempts"] else {}
        el = time.time() - att.get("t0", time.time())
        if st == oc.RUNNING:
            step, what = p.now_text()
            self.lbl_now.setText(f"▶ {step}: {what or '준비 중'}")
            rem = p.remaining_s(el)
            if rem is None:
                eta = "남은 시간: 계산 중 (사전 점검이 끝나면 나옵니다)"
            else:
                end = time.strftime("%H:%M", time.localtime(time.time() + rem))
                eta = f"남은 시간 약 <b>{_hm(rem)}</b> · 끝나는 시각 약 <b>{end}</b>"
            self.lbl_eta.setText(f"경과 {_hm(el)} · {eta}")
            self._load_log(job, append=True)
        elif st == oc.DONE:
            took = ""
            try:
                took = f" — 마지막 실행 {_hm(Path(att['exitcode']).stat().st_mtime - att['t0'])}"
            except (OSError, KeyError):
                pass
            self.lbl_now.setText(f"<span style='color:{OK_C}'>✓ 계산 끝남{took}</span>")
            self.lbl_eta.setText(self.cmp.banner_text(job))
            if self.cmp.running_for(job):
                self.bar.setValue(int(1000 * self.cmp.fraction()))
        else:
            step, what = p.now_text()
            self.lbl_now.setText(f"<span style='color:{ERR_C if st in (oc.FAILED, oc.REFUSED) else WARN_C}'>"
                                 f"■ {oc.STATE_LABEL.get(st, st)} — {step}{': ' + what if what else ''}</span>")
            self.lbl_eta.setText("끝난 단계는 남아 있습니다 — [이어서 실행]" if st in oc.RESUMABLE else "")

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
        res = oc.load_result(job["out"]) if job["state"] == oc.DONE else None       # 결과는 끝난 작업만
        job["result"] = {"gate_pass": res["gate_pass"]} if res else None
        if not res:
            self.ba.set_job(None)
            self.cmp.set_job(None)
            return
        self.btn_report.setVisible(bool(res["report"]))
        self.btn_outdir.setVisible(True)
        self.btn_apply.setVisible(job["state"] == oc.DONE)
        n_ok = sum(1 for c in res["cameras"] if c["pass"])
        verdict = (f"<b style='color:{OK_C if res['gate_pass'] else WARN_C}'>결과: "
                   f"{'통과' if res['gate_pass'] else '확인 필요'}</b> · 카메라 {n_ok}/{len(res['cameras'])} 합격")
        self.lbl_job.setText(self.lbl_job.text() + "<br>" + verdict)
        if job["state"] == oc.DONE:
            before = self._before_dir(job)
            self.ba.set_job(job, before)
            self.cmp.set_job(job, before)
            c = self.cmp.summary_html()
            if c:
                self.lbl_job.setText(self.lbl_job.text() + "<br>" + c)
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
            cells = [c["camera"], "합격" if c["pass"] is True else ("불합격" if c["pass"] is False else "미판정"), _f(c["rot_deg"], 3), _f(c["pos_mm"], 1),
                     _f(c["along_axis_mm"], 1), _f(c["focal_px"], 2), _f(c["track_reproj_px"], 2),
                     _f(c["heldout_px"], 2), vote]
            for k, v in enumerate(cells):
                it = _item(v, (OK_C if c["pass"] is True else ERR_C if c["pass"] is False else MUTED_C) if k == 1 else None,
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
        infos = [self._bag_cache.get(p) or oc.inspect_bag(p) for p in bags]
        if not self._attach_name_map(job, infos, "사전 점검"):
            return
        self._check_bags = list(job["bags"])
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
            msg = (f"거절 [{p.refusal.get('code')}]\n{oc.name_bags(p.refusal.get('msg_ko'), self._check_bags)}\n\n"
                   f"{oc.name_bags(p.refusal.get('msg'), self._check_bags)}")
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
                lines.append(f"경고: {oc.name_bags(w.get('msg_ko') or w.get('msg'), self._check_bags)}")
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
            raise RuntimeError(f"센서 레지스트리에 '{FLIR_GROUP}' 센서군이 없음")
        import sensor_config
        out = {}
        ov = (self.cfg.get("sensors") or {}).get(FLIR_GROUP) or {}
        for sub in sensor_config.subsets_of(g):
            repo = sensor_config.repo_inventory(sub)
            for serial in set(repo) | set(sensor_config.camera_overrides(dict(ov))):
                e = sensor_config.camera_entry(sub, serial, dict(ov), repo)
                out[str(serial)] = {"name": e.get("name"), "frame_id": e.get("frame_id")}
        if not out:
            raise RuntimeError("인벤토리가 비어 있음")
        return out

    def projection_dialog(self):
        worker = getattr(self, "ros_worker", None)
        if worker is None or getattr(worker, "node", None) is None:
            QMessageBox.warning(self, "실시간 projection 보기", "ROS 연결이 없습니다 (GUI 의 ROS 노드가 아직 안 떴음).")
            return
        if self._proj_dlg is None:
            import projection_check
            labels = self.stage.namespace_labels() if self.stage is not None else {}
            self._proj_dlg = projection_check.ProjectionDialog(worker, self, labels=labels, stage=self.stage,
                                                               busy=self._sensor_busy)
            self._proj_dlg.finished.connect(self._proj_closed)
        self._proj_dlg.show()
        self._proj_dlg.raise_()

    def _sensor_busy(self):
        """센서를 켜면 안 되는 이유 — 캘리브레이션이 도는 중(센서가 CPU · 디스크를 같이 쓰면 느려짐)."""
        if self.running_job():
            return "캘리브레이션이 도는 중이라 센서를 켜지 않습니다 (끝난 뒤에)"
        return None

    def _proj_closed(self, *_):
        dlg, self._proj_dlg = self._proj_dlg, None
        if dlg is not None:
            dlg.deleteLater()

    def _attach_name_map(self, job, infos, title):
        """이름 대응표 칸이 비었으면 시리얼로 만들어 job["name_map"] 에 건다. 못 만들면 이유를 보이고 계속할지 묻는다.
        False = 사용자가 멈춤."""
        if job.get("name_map"):
            self._log("GUI", f"이름 대응표 (직접 지정): {job['name_map']}")
            return True
        smap = self._serial_map()
        why = None
        if smap is None:
            why = "카메라 시리얼 표(config/camera_serial_map.yaml)를 찾을 수 없음"
        else:
            try:
                settings = {s: e["name"] for s, e in self.inventory().items() if e.get("name")}
            except Exception as e:
                settings = {}
                self._log("WARN", f"지금 GUI 설정의 카메라 인벤토리를 못 읽음 ({e}) — 녹화 기록 · 기동 사본 · 옛 이름으로만 맞춤")
            try:
                import sensor_config
                launched = oc.launched_inventory(sensor_config.GENERATED_DIR)
            except Exception:
                launched = {}
            r = oc.auto_name_map(infos, smap, lambda i: oc.naming_candidates(
                smap, oc.bag_camera_snapshot(i["path"]), launched, settings))
            if r["ok"]:
                path = oc.write_name_map(r["map"], oc.job_dir(job) / "name_map.yaml")
                job["name_map"] = str(path)
                job["name_map_note"] = r["why"]
                self._log("GUI", f"이름 대응표 (시리얼로 만듦, 근거 — {r['why']}): "
                                 + ", ".join(f"{k}={v}" for k, v in sorted(r["map"].items())))
                return True
            why = r["why"]
        self._log("WARN", f"이름 대응표를 만들지 못함: {why}")
        ans = QMessageBox.question(
            self, title,
            f"카메라 이름 대응표를 시리얼로 만들지 못했습니다:\n{why}\n\n"
            "계속하면 도구가 영상 기하만으로 카메라를 식별합니다 (확정 못 하면 거절).\n계속할까요?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        return ans == QMessageBox.Yes

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
        try:
            inv = self.inventory()
        except Exception as e:
            QMessageBox.warning(self, "적용", f"차량 카메라 인벤토리를 읽지 못해 적용할 수 없습니다: {e}\n\n"
                                "카메라 노드가 쓰는 frame 이름을 모르면 TF 가 엉뚱한 frame 에 붙습니다.")
            return
        dlg = ApplyDialog(self, job, smap, inv, *self.vehicle_files(), compare=self.cmp.data_for(job))
        if dlg.exec_() == QDialog.Accepted and dlg.manifest:
            self._update_vehicle_label()
            m = dlg.manifest
            self._log("OK", f"차량에 적용: {len(m['cameras'])}대 → {', '.join(f['path'] for f in m['files'])} "
                            f"(보관 {m['archive']})")
            QMessageBox.information(self, "적용 완료",
                                    f"{len(m['cameras'])}대를 적용했습니다.\n\n" +
                                    "\n".join(f"{f['path']}\n  백업: {f.get('backup_sidecar') or '(새 파일)'}" for f in m["files"]) +
                                    "\n\nFLIR 카메라 센서군을 다시 기동해야 반영됩니다 (camera_info 와 /tf_static 은 "
                                    "노드가 켜질 때 읽음). 다시 기동한 뒤 [실시간 projection 보기] 로 확인하세요.\n"
                                    "되돌리기: [적용 기록 · 되돌리기…]")

    def history_dialog(self):
        HistoryDialog(self).exec_()
        self._update_vehicle_label()

    def recalib_dialog(self):
        """이전 녹화에도 적용 — 현황표와 같은 목록(저장 위치 + 옮겨 둔 곳)의 녹화들."""
        try:
            import dataset_catalog
            import recalib_dialog
            base = (self.cfg.get("recorder") or {}).get("output_dir") or str(Path.home() / "DM_clipGUI" / "clips")
            rows = dataset_catalog.scan(base)
            ci, ex = self.vehicle_files()
            dlg = recalib_dialog.RecalibDialog(self, ci, ex, rows, acquisition=self.acquisition)
        except Exception as e:
            QMessageBox.warning(self, "이전 녹화에 적용", f"열 수 없습니다: {e}")
            return
        if not dlg.bags:
            QMessageBox.information(self, "이전 녹화에 적용", "지금 연결된 디스크에 녹화가 없습니다.")
            return
        dlg.exec_()


class ApplyDialog(QDialog):
    """카메라별로 시리얼 · 대상 frame 을 보여 주고 고른 것만 적용."""

    def __init__(self, tab, job, smap, inventory, ci_path, ex_path, compare=None):
        super().__init__(tab)
        self.tab, self.job, self.smap, self.inv = tab, job, smap, inventory
        cmp = compare or {}
        self.cmp_rows = {**(cmp.get("rgb") or {}), **(cmp.get("thermal") or {})}
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
        for r in self.rows:            # 기존 캘이 더 나은 카메라는 기본으로 빼 둔다 (직접 체크하면 적용)
            if (self.cmp_rows.get(r["camera"]) or {}).get("verdict") == "old" and r["camera"] not in self.confirmed:
                r["selected"] = False
                r["needs_confirm"] = list(r["needs_confirm"]) + ["비교: 기존 캘이 더 나음"]
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
        # 이번에도 os_lidar 에 안 붙는 카메라 — TF 노드는 그 카메라를 reference_frame(flir_rig_frame) 아래 자리표시
        # (항등 자세)로 낸다. 라이다 트리와 끊겨 있어 lookup 이 실패한다 (가짜 자세로 붙는 것보다는 낫다)
        applied = {r["serial"] for r in sel}
        try:
            cur = {it["serial"]: it["parent_frame"]
                   for it in ca.parse_extrinsics_like_node(Path(self.ed_ex.text().strip()).read_text(encoding="utf-8"))}
        except (OSError, ValueError):
            cur = {}
        left = sorted(e.get("name") or s for s, e in self.inv.items()
                      if s not in applied and cur.get(s) != ca.PARENT_FRAME)
        if left:
            extra += (f"\n\n이번에 적용하지 않는 {len(left)}대 ({', '.join(left)})는 TF 가 os_lidar 에 붙지 않고 "
                      "flir_rig_frame 아래 자리표시(항등)로 남습니다 — 라이다 기준 좌표가 없습니다.")
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


class BeforeAfterView(QWidget):
    """끝난 작업의 결과를 눈으로: 같은 장면(캘리브레이션에 쓴 데이터)에 라이다를 '전'(이전 캘값) · '후'(새 결과)로 그린 영상.

    '후' 는 도구가 결과 폴더에 남긴 out/images, '전' 은 calib_before_after.py 가 도구 venv 에서 같은 장면 · 같은
    그리기로 만든다 (작업 폴더의 추출 데이터가 필요 — 작업 폴더를 지웠으면 못 만든다). 한 번 만들면 작업 폴더 옆에 남는다.
    """
    SCENES = [("parked", "정차 중"), ("fastest", "가장 빠를 때"), ("turning", "회전할 때"), ("moderate", "보통 속도")]

    def __init__(self, tab):
        super().__init__(tab)
        self.tab = tab
        self.job = None
        self.before = (None, "")
        self.dest = None
        self.proc = None
        self._flip = False
        v = QVBoxLayout(self)
        row = QHBoxLayout()
        self.cb_cam = QComboBox()
        self.cb_scene = QComboBox()
        for k, lbl in self.SCENES:
            self.cb_scene.addItem(lbl, k)
        self.cb_view = QComboBox()
        self.cb_view.addItem("나란히", "side")
        self.cb_view.addItem("번갈아 (어긋남 찾기)", "flip")
        for w in (self.cb_cam, self.cb_scene, self.cb_view):
            w.currentIndexChanged.connect(self._render)
        for w in (QLabel("카메라"), self.cb_cam, QLabel("장면"), self.cb_scene, QLabel("보기"), self.cb_view):
            row.addWidget(w)
        row.addStretch(1)
        v.addLayout(row)
        self.lbl_status = QLabel("끝난 작업을 고르면 여기에 결과 영상이 보입니다")
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setStyleSheet(f"color:{MUTED_C};")
        v.addWidget(self.lbl_status)
        pics = QHBoxLayout()
        self.cap_l, self.cap_r = QLabel(), QLabel()
        self.img_l, self.img_r = QLabel(), QLabel()
        self.pane_l, self.pane_r = QWidget(), QWidget()
        for cap, img, pane in ((self.cap_l, self.img_l, self.pane_l), (self.cap_r, self.img_r, self.pane_r)):
            box = QVBoxLayout(pane)
            box.setContentsMargins(0, 0, 0, 0)
            cap.setStyleSheet("font-weight:700; font-size:14px;")
            cap.setWordWrap(False)
            cap.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)      # 제목 길이가 칸을 안 바꾸게
            img.setAlignment(Qt.AlignCenter)
            img.setMinimumSize(200, 150)
            img.setStyleSheet("background:#111; color:#aaa;")
            # 올린 그림 크기가 칸 크기를 바꾸지 않게 (안 그러면 칸 → 그림 → 칸 이 번갈아 볼 때마다 몇 px 씩 커진다)
            img.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            box.addWidget(cap)
            box.addWidget(img, 1)
            pics.addWidget(pane, 1)
        v.addLayout(pics, 1)
        hint = QLabel("라이다 점 색 = 거리 (빨강 가까움 → 파랑 멀리). 기둥 · 차 · 연석 같은 윤곽이 영상과 겹치면 맞는 것. "
                      "몇 px 어긋남은 [번갈아] 로 보면 점이 움찔거려서 잘 보입니다.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color:{MUTED_C};")
        v.addWidget(hint)
        self.flip_timer = QTimer(self, interval=700, timeout=self._flip_tick)

    # --- 작업 ---
    def set_job(self, job, before=(None, "")):
        key = (job or {}).get("id")
        if job is not None and key == (self.job or {}).get("id") and before == self.before:
            return
        self.stop()
        self.job, self.before = job, before
        self.cb_cam.blockSignals(True)
        self.cb_cam.clear()
        self.cb_cam.blockSignals(False)
        if not job:
            self.lbl_status.setText("끝난 작업을 고르면 여기에 결과 영상이 보입니다")
            self._clear()
            return
        after = Path(job["out"]) / "images"
        cams = sorted(f.stem for f in (after / "parked").glob("*.jpg"))
        if not cams:
            cams = sorted({f.stem.split("_", 1)[1] for f in (after / "driving").glob("*_*.jpg")
                           if not f.stem.endswith("_all")})
        cams = [c for c in cams if not c.endswith("_all")]
        names = self._vehicle_names()
        self.cb_cam.blockSignals(True)
        for c in sorted(cams, key=lambda c: (c.startswith("thermal"), c)):
            self.cb_cam.addItem(f"{c}  ({names[c]})" if names.get(c) else c, c)
        self.cb_cam.blockSignals(False)
        if not cams:
            self.lbl_status.setText("결과 폴더에 투영 이미지가 없습니다 (도구가 이미지를 만들지 못함 — 보고서 확인)")
            self._clear()
            return
        bdir, blabel = before
        if not bdir:
            self.dest = None
            self.lbl_status.setText(f"'전' 없음: {blabel}. '후'(새 결과)만 보입니다.")
        else:
            import hashlib
            h = hashlib.sha1(str(Path(bdir).resolve()).encode()).hexdigest()[:8]
            self.dest = oc.job_dir(job) / "before_after" / h
            if (self.dest / "done.json").is_file():
                self.lbl_status.setText(f"전: {blabel}  ·  후: 이번 결과")
            else:
                self._make_before(job, bdir, blabel)
        self._render()

    def _vehicle_names(self):
        """캘리브레이션 이름 → 지금 차량 토픽 이름 (시리얼로)."""
        try:
            smap = self.tab._serial_map() or {}
            inv = self.tab.inventory()
            return {c: (inv.get(v.get("serial") or "") or {}).get("name") for c, v in smap.items()}
        except Exception:                                       # noqa: BLE001
            return {}

    def _make_before(self, job, bdir, blabel):
        work = Path(job.get("workdir") or "")
        if not (work / "config_snapshot.json").is_file():
            self.lbl_status.setText("작업 폴더(중간 데이터)가 없어 '전' 영상을 만들 수 없습니다 — '후' 만 보입니다.")
            self.dest = None
            return
        tool = self.tab._tool()
        if not tool["ok"]:
            self.lbl_status.setText(f"도구가 없어 '전' 영상을 못 만듦: {tool['msg']}")
            self.dest = None
            return
        py = Path(tool["exe"]).parent / "python"
        script = Path(__file__).resolve().parent / "calib_before_after.py"
        self.dest.mkdir(parents=True, exist_ok=True)
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.setStandardOutputFile(str(self.dest / "log.txt"))
        self.proc.finished.connect(lambda code, _s, lbl=blabel: self._made(code, lbl))
        self.proc.start("nice", ["-n", "10", str(py), str(script), "--work", str(work), "--out", job["out"],
                                 "--before", str(bdir), "--dest", str(self.dest)])
        self.lbl_status.setText(f"'전'({blabel}) 영상을 만드는 중… (20–40초)")

    def _made(self, code, blabel):
        self.proc = None
        if code == 0 and self.dest and (self.dest / "done.json").is_file():
            self.lbl_status.setText(f"전: {blabel}  ·  후: 이번 결과")
        else:
            self.lbl_status.setText(f"<span style='color:{ERR_C}'>'전' 영상 만들기 실패 (exit {code}) — "
                                    f"{self.dest / 'log.txt' if self.dest else ''}</span>")
            self.dest = None
        self._render()

    # --- 그리기 ---
    def _paths(self):
        c, scene = self.cb_cam.currentData(), self.cb_scene.currentData()
        if not self.job or not c:
            return None, None
        rel = Path("images") / ("parked" if scene == "parked" else "driving") / \
            (f"{c}.jpg" if scene == "parked" else f"{scene}_{c}.jpg")
        after = Path(self.job["out"]) / rel
        before = (self.dest / rel) if self.dest else None
        return before, after

    def _set(self, label, path, empty):
        if path and Path(path).is_file():
            pm = QPixmap(str(path))
            label.setPixmap(pm.scaled(label.contentsRect().size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            label.setPixmap(QPixmap())
            label.setText(empty)

    def _clear(self):
        for lb in (self.img_l, self.img_r):
            lb.setPixmap(QPixmap())
            lb.setText("")
        self.cap_l.setText("")
        self.cap_r.setText("")

    def _render(self):
        self.flip_timer.stop()
        before, after = self._paths()
        if after is None:
            self._clear()
            return
        blabel = self.before[1] if self.dest else ""
        if self.cb_view.currentData() == "flip" and before is not None:
            self.pane_r.hide()
            self._flip = False
            QTimer.singleShot(0, self._flip_tick)     # 오른쪽 칸을 숨긴 뒤 레이아웃이 넓어진 크기로
            self.flip_timer.start()
            return
        self.pane_r.show()
        self.cap_l.setStyleSheet("font-weight:700; font-size:14px;")
        self.cap_l.setText(f"전 — {blabel}" if before is not None else "전 — 없음")
        self.cap_r.setText("후 — 이번 결과")
        self._set(self.img_l, before, "'전' 영상 없음" if before is None else "만드는 중…" if self.proc else "이 장면 없음")
        self._set(self.img_r, after, "이 장면 없음")

    def _flip_tick(self):
        before, after = self._paths()
        if after is None:
            return
        self._flip = not self._flip
        path, cap = (after, "후 — 이번 결과") if self._flip else (before, f"전 — {self.before[1]}")
        self.cap_l.setText(cap)
        self.cap_l.setStyleSheet(f"font-weight:700; font-size:14px; color:{OK_C if self._flip else WARN_C};")
        self._set(self.img_l, path, "만드는 중…" if self.proc else "이 장면 없음")

    def resizeEvent(self, e):
        super().resizeEvent(e)
        if self.cb_view.currentData() != "flip":
            self._render()

    def stop(self):
        self.flip_timer.stop()
        if self.proc is not None:
            self.proc.kill()
            self.proc.waitForFinished(2000)
            self.proc = None


class CompareView(QWidget):
    """끝난 작업: 기존 캘(작업을 시작할 때 차량 값 등) vs 새 결과, 어느 쪽이 나은가 — calib_compare.py (도구 venv, 약 10분).

    잣대는 도구 검증과 같다: held-out 재투영(새 쪽은 절반 데이터로 푼 값을 나머지 절반에서, 기존 쪽도 같은 절반들에서
    고정) + LiDAR 에지 정렬 (+ 열화상 투표). 결과는 jobs/<id>/compare/<before 해시>/compare.json 에 남는다.
    """
    EXPECT_S = 9 * 60          # 2026-09-27 실측: 창 30개 · 20코어 약 9분 (창 수에 비례)
    VERDICT = {"new": ("새 결과가 나음", OK_C), "old": ("기존 캘이 나음", ERR_C), "same": ("비슷", MUTED_C),
               "mixed": ("지표가 엇갈림", WARN_C), "no_old": ("기존 값 없음", MUTED_C)}

    def __init__(self, tab):
        super().__init__(tab)
        self.tab = tab
        self.job, self.before, self.dest = None, (None, ""), None
        self.proc, self.proc_job, self.t0, self.msg = None, None, 0.0, ""
        v = QVBoxLayout(self)
        self.lbl = QLabel("끝난 작업을 고르면 여기에 판정이 보입니다")
        self.lbl.setWordWrap(True)
        self.lbl.setTextFormat(Qt.RichText)
        self.lbl.setStyleSheet("font-size:16px; padding:4px 0;")
        v.addWidget(self.lbl)
        row = QHBoxLayout()
        self.btn = QPushButton("기존 캘과 비교 실행 (약 10분)")
        self.btn.clicked.connect(lambda: self.start(self.job, self.before))
        row.addWidget(self.btn)
        row.addStretch(1)
        v.addLayout(row)
        self.tbl = QTableWidget(0, 5)
        self.tbl.setHorizontalHeaderLabels(["카메라", "판정", "held-out 재투영 [px]\n기존 → 새", "LiDAR 에지 [px]\n기존 → 새",
                                            "근거"])
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.tbl.horizontalHeader().setStretchLastSection(True)
        self.tbl.verticalHeader().setVisible(False)
        v.addWidget(self.tbl, 1)
        hint = QLabel("공정하게: 새 결과는 이 데이터로 맞춘 값이라, 도구 검증처럼 데이터를 절반으로 나눠 한쪽으로 푼 값을 다른 쪽에서 "
                      "잽니다 (새 쪽이 오히려 불리). 기존 캘도 같은 절반들에서 캘값을 고정하고 잽니다. 에지는 풀이에 안 쓰는 독립 지표. "
                      f"held-out 차이가 {0.03:.2f} px 또는 4% 를 넘어야 '낫다'.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color:{MUTED_C};")
        v.addWidget(hint)

    # --- 상태 ---
    def _dest(self, job, before):
        import hashlib
        if not job or not before or not before[0]:
            return None
        h = hashlib.sha1(str(Path(before[0]).resolve()).encode()).hexdigest()[:8]
        return oc.job_dir(job) / "compare" / h

    def data_for(self, job):
        d = self._dest(job, self.tab._before_dir(job)) if job else None
        try:
            return json.loads((d / "compare.json").read_text(encoding="utf-8")) if d else None
        except (OSError, ValueError):
            return None

    def running(self):
        return self.proc is not None

    def running_for(self, job):
        return self.proc is not None and job and self.proc_job == job["id"]

    def fraction(self):
        return min(0.99, (time.time() - self.t0) / self._expect())

    def _expect(self):
        job = self.tab.store.get(self.proc_job) if self.proc_job else None
        n = float(((job or {}).get("estimate") or {}).get("windows") or 30)
        return self.EXPECT_S * max(0.3, n / 30)

    def banner_text(self, job):
        if self.running_for(job):
            el = time.time() - self.t0
            return (f"<b>기존 캘과 비교하는 중</b> · 경과 {_hm(el)} · 남은 시간 약 {_hm(max(60, self._expect() - el))} "
                    "— 끝나면 [판정] 탭에 권고가 나옵니다")
        d = self.data_for(job)
        if d:
            return self.summary_html(d)
        if self.msg and self.job and job and self.job["id"] == job["id"]:
            return f"<span style='color:{WARN_C}'>{self.msg}</span>"
        return "기존 캘과 아직 비교 안 함 — [판정] 탭의 [기존 캘과 비교 실행]"

    def summary_html(self, d=None):
        d = d or (self.data_for(self.job) if self.job else None)
        if not d:
            return ""
        n = d.get("counts", {})
        adv = d.get("advice")
        if adv == "part":
            olds = [c for k in ("rgb", "thermal") for c, r in (d.get(k) or {}).items() if r["verdict"] == "old"]
            return (f"<b style='color:{WARN_C}'>판정: 기존 캘이 나은 카메라 {len(olds)}대 ({', '.join(olds)}) — "
                    "그 카메라는 빼고 적용 권장</b>")
        if adv == "apply":
            return (f"<b style='color:{OK_C}'>판정: 새 결과 적용 권장</b> — 새 결과가 나음 {n.get('new', 0)}대, "
                    f"비슷 {n.get('same', 0) + n.get('mixed', 0)}대, 기존이 나음 0대")
        return f"<b>판정: 기존과 차이 없음</b> — 적용해도 되고 안 해도 됩니다 ({n.get('same', 0)}대 비슷)"

    # --- 작업 ---
    def set_job(self, job, before=(None, "")):
        self.job, self.before = job, before
        self.dest = self._dest(job, before)
        self._show()

    def start(self, job, before):
        if not job or self.proc is not None:
            return
        self.job, self.before = job, before
        self.dest = self._dest(job, before)
        if not self.dest:
            self.msg = f"비교할 기존 캘이 없음: {before[1] if before else ''}"
            self._show()
            return
        work = Path(job.get("workdir") or "")
        if not (work / "validation" / "validation.json").is_file():
            self.msg = "작업 폴더(중간 데이터)가 없어 비교할 수 없습니다"
            self._show()
            return
        block, _ = self.tab.acquisition()
        if block:
            self.msg = f"{block} — 비교는 녹화가 끝난 뒤 [기존 캘과 비교 실행]"
            self._show()
            return
        tool = self.tab._tool()
        if not tool["ok"]:
            self.msg = tool["msg"]
            self._show()
            return
        py = Path(tool["exe"]).parent / "python"
        script = Path(__file__).resolve().parent / "calib_compare.py"
        self.dest.mkdir(parents=True, exist_ok=True)
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.setStandardOutputFile(str(self.dest / "log.txt"))
        self.proc_job, self.t0, self.msg = job["id"], time.time(), ""
        self.proc.finished.connect(self._done)
        self.proc.start("nice", ["-n", "10", str(py), str(script), "--work", str(work), "--before", str(before[0]),
                                 "--dest", str(self.dest)])
        self.tab._log("GUI", f"{job['id']}: 기존 캘({before[1]})과 비교 시작 — 약 10분")
        self._show()

    def _done(self, code, _status=None):
        jid, self.proc, self.proc_job = self.proc_job, None, None
        d = self.data_for(self.tab.store.get(jid)) if jid else None
        if code == 0 and d:
            self.tab._log("OK", f"{jid}: 비교 끝 — {self.summary_html(d).replace('<b>', '').replace('</b>', '')}")
            self.msg = ""
        elif not self.msg:
            self.msg = f"비교 실패 (exit {code}) — {self.dest / 'log.txt' if self.dest else ''}"
            self.tab._log("WARN", f"{jid}: {self.msg}")
        self._show()
        self.tab._show_job()

    def stop(self, why=""):
        if self.proc is not None:
            self.msg = why
            self.proc.kill()
            self.proc.waitForFinished(3000)
            self.proc, self.proc_job = None, None

    # --- 표시 ---
    def _show(self):
        self.btn.setVisible(bool(self.job) and self.proc is None)
        self.tbl.setRowCount(0)
        if not self.job:
            self.lbl.setText("끝난 작업을 고르면 여기에 판정이 보입니다")
            return
        if self.running_for(self.job):
            self.lbl.setText(f"기존 캘({self.before[1]})과 비교하는 중… (약 10분, 진행은 위 배너)")
            return
        d = self.data_for(self.job)
        if not d:
            base = f"기존 캘: {self.before[1]}" if self.before and self.before[0] else f"비교할 기존 캘 없음 — {self.before[1]}"
            self.lbl.setText(base + (f"<br><span style='color:{WARN_C}'>{self.msg}</span>" if self.msg else ""))
            self.btn.setVisible(bool(self.dest))
            return
        self.btn.setText("다시 비교")
        self.lbl.setText(self.summary_html(d) + f"<br><span style='color:{MUTED_C}; font-size:13px;'>기존 캘: {self.before[1]}</span>")
        rows = [(c, r) for k in ("rgb", "thermal") for c, r in (d.get(k) or {}).items()]
        order = {"old": 0, "mixed": 1, "new": 2, "same": 3, "no_old": 4}
        for c, r in sorted(rows, key=lambda x: (order.get(x[1]["verdict"], 9), x[0])):
            i = self.tbl.rowCount()
            self.tbl.insertRow(i)
            txt, color = self.VERDICT.get(r["verdict"], (r["verdict"], None))
            names = self.tab.ba._vehicle_names() if hasattr(self.tab, "ba") else {}
            self.tbl.setItem(i, 0, _item(f"{c}  ({names[c]})" if names.get(c) else c))
            self.tbl.setItem(i, 1, _item(txt, color))
            self.tbl.setItem(i, 2, _item(f"{_f(r.get('heldout_old'), 3)} → {_f(r.get('heldout_new'), 3)}"))
            e = f"{_f(r.get('edge_old'), 2)} → {_f(r.get('edge_new'), 2)}"
            if r.get("vote_new") is not None:
                e += f"  (투표 {'통과' if r.get('vote_old') else '실패'} → {'통과' if r.get('vote_new') else '실패'})"
            self.tbl.setItem(i, 3, _item(e))
            self.tbl.setItem(i, 4, _item(", ".join(r.get("why") or []) or "–"))
