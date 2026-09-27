#!/usr/bin/env python3
"""[온라인 캘리브레이션] 탭을 화면 없이(offscreen) 돌려 보는 시험. 가짜 nontarget_cal 을 쓴다.

    QT_QPA_PLATFORM=offscreen python3 -m pytest -q test/test_calib_tab_gui.py
"""
import os
import shutil
import sys
import time
from pathlib import Path

import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
REF_OUT = Path("/hdd/DM_calib/nt_regress/full/out")
FLIR_CAL = Path.home() / "projects" / "DM" / "FLIR_control_master" / "calibration"

pytest.importorskip("PyQt5")
pytestmark = pytest.mark.skipif(not (REF_OUT / "summary.json").is_file(), reason="참조 결과 없음")

FAKE = r'''#!/bin/bash
# 가짜 nontarget_cal run: 진행 줄을 내고 --out 에 참조 결과를 복사
if [ "$1" = check ]; then
  echo '{"ev": "check_done", "ok": true, "total": {}, "estimate": {"windows_40s_equiv": 3, "disk_gb": 25}, "disk": {"free_gb": 999, "needed_gb": 52}, "warnings": [], "names": {"b0": "known map"}}'
  exit 0
fi
out=""; while [ $# -gt 0 ]; do [ "$1" = --out ] && out=$2; shift; done
echo '{"ev": "run_start"}'
for s in names preflight windows extract lo rgb_tracks thermal_tracks rgb_solve thermal_solve validation outputs; do
  echo "{\"ev\": \"stage_start\", \"stage\": \"$s\"}"; echo "working on $s" >&2
  echo "{\"ev\": \"stage_progress\", \"stage\": \"$s\", \"done\": 1, \"total\": 2}"
  sleep ${FAKE_SLEEP:-0.02}
  echo "{\"ev\": \"stage_end\", \"stage\": \"$s\", \"ok\": true, \"disk_gb\": 0.1}"
done
mkdir -p "$out"; cp -r REF/. "$out/"
echo '{"ev": "run_end", "ok": true, "gate_pass": true}'
'''


@pytest.fixture()
def env(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / "xdg"))
    import importlib
    import online_calib
    importlib.reload(online_calib)
    import calib_tab
    importlib.reload(calib_tab)
    from PyQt5.QtWidgets import QApplication, QMessageBox
    app = QApplication.instance() or QApplication([])
    # 대화상자는 전부 '예'
    monkeypatch.setattr(QMessageBox, "question", staticmethod(lambda *a, **k: QMessageBox.Yes))
    monkeypatch.setattr(QMessageBox, "information", staticmethod(lambda *a, **k: QMessageBox.Ok))
    monkeypatch.setattr(QMessageBox, "warning", staticmethod(lambda *a, **k: QMessageBox.Ok))
    monkeypatch.setattr(QMessageBox, "critical", staticmethod(lambda *a, **k: QMessageBox.Ok))
    monkeypatch.setattr(QMessageBox, "exec_", lambda self: QMessageBox.Yes)
    clips = tmp_path / "clips"
    for name, dur in (("rec_20260924_204902", 999.6), ("rec_20260925_100000", 300.0)):
        d = clips / name
        d.mkdir(parents=True)
        topics = [{"topic_metadata": {"name": n, "type": "t"}, "message_count": 1} for n in
                  ["/ouster/points", "/gps/fix"] + [f"/camera{i}/image_rgb/compressed" for i in range(14)] +
                  ["/thermal0/image_raw", "/thermal1/image_raw"]]
        (d / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
            "duration": {"nanoseconds": int(dur * 1e9)}, "storage_identifier": "sqlite3",
            "topics_with_message_count": topics}}))
        (d / f"{name}_0.db3").write_bytes(b"\0" * 100)
    exe = tmp_path / "nontarget_cal"
    exe.write_text(FAKE.replace("REF", str(REF_OUT)))
    exe.chmod(0o755)
    state = {"recording": False, "clip_busy": False, "sensors": False, "recorder": False}
    cfg = {"recorder": {"output_dir": str(clips)},
           "ui": {"calib": {"executable": str(exe), "workdir_root": str(tmp_path / "work"),
                            "out_root": str(tmp_path / "out")}}, "sensors": {}}
    tab = calib_tab.CalibTab(cfg, probe=lambda: dict(state))
    yield {"tab": tab, "state": state, "app": app, "tmp": tmp_path, "oc": online_calib, "ct": calib_tab,
           "monkeypatch": monkeypatch}
    tab.shutdown()
    tab.close()


def _pump(app, tab, until, timeout=15):
    t0 = time.time()
    while not until():
        app.processEvents()
        tab._tick()
        assert time.time() - t0 < timeout, "시간 초과"
        time.sleep(0.05)


def test_run_result_apply_rollback(env):
    tab, app, oc = env["tab"], env["app"], env["oc"]
    assert tab.tbl_bags.rowCount() == 2
    tab.tbl_bags.item(0, 0).setCheckState(2)          # Qt.Checked
    assert "GB" in tab.lbl_est.text()
    env["monkeypatch"].setattr(oc, "free_gb", lambda p: 10_000.0)
    tab.start_new()
    job = tab.running_job()
    assert job and job["mode"] == "zeroshot"
    assert not tab.btn_start.isEnabled() or tab.running_job() is None
    _pump(app, tab, lambda: job["state"] != oc.RUNNING)
    assert job["state"] == oc.DONE, job
    tab.refresh_jobs(select=job["id"])
    assert tab.tbl_res.rowCount() == 16 and not tab.btn_apply.isHidden()
    assert "working on outputs" in tab.log_view.toPlainText()
    assert tab.bar.value() == 1000
    assert tab.detail_tabs.widget(1) is tab.cmp                 # 끝나면 판정 (기존 vs 새)
    assert tab.detail_tabs.widget(2) is tab.ba                  # 전/후 투영 비교
    assert tab.detail_tabs.currentIndex() == 1
    assert tab.detail_tabs.widget(4) is tab.log_view
    assert tab.btn_cancel.isHidden()                            # 끝난 작업엔 중단 버튼 없음

    # 적용 (임시 차량 파일) → 되돌리기
    if not (FLIR_CAL / "flir_camera_info.yaml").is_file():
        pytest.skip("FLIR_control 파일 없음")
    veh = env["tmp"] / "FLIR_control" / "calibration"
    veh.mkdir(parents=True)
    for f in ("flir_camera_info.yaml", "flir_camera_extrinsics.yaml"):
        shutil.copy2(FLIR_CAL / f, veh / f)
    old = (veh / "flir_camera_info.yaml").read_text()
    dlg = env["ct"].ApplyDialog(tab, job, tab._serial_map(), {}, veh / "flir_camera_info.yaml",
                                veh / "flir_camera_extrinsics.yaml")
    rl = [i for i, r in enumerate(dlg.rows) if r["camera"] == "camera_rear_left"][0]
    assert not dlg.rows[rl]["selected"]
    dlg.tbl.item(rl, 3).setText("25415251")           # 사용자가 시리얼을 넣음
    assert dlg.rows[rl]["ok"] and not dlg.rows[rl]["selected"]
    dlg.tbl.item(rl, 0).setCheckState(2)              # 확인 대화상자 → 예
    assert dlg.rows[rl]["selected"]
    dlg._apply()
    m = dlg.manifest
    assert m and len(m["cameras"]) == 16
    assert (veh / "flir_camera_info.yaml").read_text() != old
    hist = env["ct"].HistoryDialog(tab)
    assert hist.tbl.rowCount() == 1
    hist.tbl.selectRow(0)
    hist._rollback()
    assert (veh / "flir_camera_info.yaml").read_text() == old

    # warm-start: '마지막 적용 결과' 는 되돌렸으니 없음, '차량 파일' 은 원래 파일이라 os_lidar 기준 값 없음 → 시작 안 함
    tab.rb_warm.setChecked(True)
    tab.c["camera_info_path"] = str(veh / "flir_camera_info.yaml")
    tab.c["extrinsics_path"] = str(veh / "flir_camera_extrinsics.yaml")
    n = len(tab.store.jobs)
    tab.start_new()
    assert len(tab.store.jobs) == n


def test_pending_when_no_space_and_block_when_recording(env):
    tab, app, oc, state = env["tab"], env["app"], env["oc"], env["state"]
    tab.tbl_bags.item(0, 0).setCheckState(2)
    env["monkeypatch"].setattr(oc, "free_gb", lambda p: 50.0)
    tab.start_new()
    job = tab.store.jobs[-1]
    assert job["state"] == oc.PENDING and "공간" in job["state_msg"]
    assert not job["attempts"]
    # 공간이 생기면 [공간 다시 확인] → 시작
    env["monkeypatch"].setattr(oc, "free_gb", lambda p: 10_000.0)
    env["monkeypatch"].setenv("FAKE_SLEEP", "1")
    tab.refresh_jobs(select=job["id"])
    tab.recheck_job()
    assert job["state"] == oc.RUNNING
    # 녹화가 시작되면 자동 중단, 시작 버튼 잠김
    state["recording"] = True
    tab._tick()
    assert not tab.btn_start.isEnabled()
    _pump(app, tab, lambda: job["state"] != oc.RUNNING)
    assert job["state"] == oc.CANCELLED and "녹화" in job["state_msg"]
    # 녹화 중에는 새로 시작도 안 됨
    n = len(tab.store.jobs)
    tab.start_new()
    assert len(tab.store.jobs) == n
    # 녹화가 끝나면 이어서 실행
    state["recording"] = False
    env["monkeypatch"].setenv("FAKE_SLEEP", "0.01")
    tab.refresh_jobs(select=job["id"])
    tab.resume_job()
    assert job["state"] == oc.RUNNING and len(job["attempts"]) == 2
    _pump(app, tab, lambda: job["state"] != oc.RUNNING)
    assert job["state"] == oc.DONE


def test_preflight_check_button(env):
    tab, app = env["tab"], env["app"]
    tab.tbl_bags.item(0, 0).setCheckState(2)
    tab.run_check()
    _pump(app, tab, lambda: tab.btn_check.isEnabled())
    assert tab._check_prog.check and tab._check_prog.check["ok"]
