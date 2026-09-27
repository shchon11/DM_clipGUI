#!/usr/bin/env python3
"""온라인 캘리브레이션 GUI 로직 시험 (Qt · ROS 없이).

    python3 -m pytest -q test/test_online_calib.py

참조 데이터(있으면 쓰고, 없으면 그 시험만 건너뜀 — 읽기만 한다):
  /hdd/DM_calib/nt_regress/full/out       nontarget_cal 전체 실행 결과 (2026-09-24 야간 bag, 창 25개)
  /hdd/DM_calib/nt_regress/full/work/events.jsonl   그 실행의 진행 이벤트
  ~/projects/DM/FLIR_control_master/calibration     차량 스택의 camera_info / extrinsics 파일 (시리얼 키)
  ~/FLIR_control/calibration                        (차량 PC 에서는 이것)
"""
import json
import os
import shutil
import signal
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import calib_apply as ca       # noqa: E402
import online_calib as oc      # noqa: E402

REF_OUT = Path("/hdd/DM_calib/nt_regress/full/out")
REF_EVENTS = Path("/hdd/DM_calib/nt_regress/full/work/events.jsonl")
FLIR_CAL = next((p for p in (Path.home() / "projects" / "DM" / "FLIR_control_master" / "calibration",
                              Path.home() / "FLIR_control" / "calibration")        # 차량 PC
                 if (p / "flir_camera_info.yaml").is_file()), Path("/없음"))
SERIAL_MAP = ROOT / "config" / "camera_serial_map.yaml"
need_ref = pytest.mark.skipif(not (REF_OUT / "summary.json").is_file(), reason="참조 결과 없음")
need_flir = pytest.mark.skipif(not (FLIR_CAL / "flir_camera_info.yaml").is_file(), reason="FLIR_control 파일 없음")


# ------------------------------------------------------------------ 좌표 변환
def _rand_rot(rng):
    q = rng.normal(size=4)
    return ca.quat_xyzw_to_rot(q / np.linalg.norm(q))


def test_quaternion_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(500):
        R = _rand_rot(rng)
        q = ca.rot_to_quat_xyzw(R)
        assert abs(np.linalg.norm(q) - 1) < 1e-12 and q[3] >= 0
        assert np.allclose(ca.quat_xyzw_to_rot(q), R, atol=1e-12)
    # 180° 회전 (trace = -1) 같은 경계
    for R in (np.diag([1, -1, -1]), np.diag([-1, 1, -1]), np.diag([-1, -1, 1])):
        assert np.allclose(ca.quat_xyzw_to_rot(ca.rot_to_quat_xyzw(R)), R, atol=1e-12)


def test_tf_convention_synthetic():
    """TF(parent os_lidar → child camera): x_lidar = R_pc x_cam + t_pc. 점 하나로 직접 확인."""
    rng = np.random.default_rng(1)
    for _ in range(100):
        R = _rand_rot(rng)
        t = rng.normal(size=3)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        tpc, q = ca.tf_from_T_cam_lidar(T)
        x_l = rng.normal(size=3) * 10
        x_c = R @ x_l + t                      # 도구 규약
        back = ca.quat_xyzw_to_rot(q) @ x_c + tpc   # TF 규약
        assert np.allclose(back, x_l, atol=1e-9)
        assert np.allclose(ca.T_cam_lidar_from_tf(tpc, q), T, atol=1e-12)


@need_ref
def test_tf_matches_reference_yaml():
    """참조 결과의 camera_position_in_os_lidar_m · quaternion_xyzw 과 맞는가 + 물리적으로 맞는 방향인가."""
    cams = ca.load_result_cameras(REF_OUT)
    assert len(cams) == 16
    for c, cam in cams.items():
        e = yaml.safe_load((REF_OUT / "extrinsic" / f"{c}.yaml").read_text())
        tpc, q = ca.tf_from_T_cam_lidar(cam["T_cam_lidar"])
        assert np.allclose(tpc, e["camera_position_in_os_lidar_m"], atol=1e-9), c
        q_tool = np.array(e["quaternion_xyzw"])           # 도구 YAML 의 쿼터니언은 R (lidar→cam)
        q_conj = np.array([-q_tool[0], -q_tool[1], -q_tool[2], q_tool[3]])
        assert min(np.abs(q - q_conj).max(), np.abs(q + q_conj).max()) < 1e-6, c
    # os_lidar 의 x 는 차량 뒤쪽 → 전방 카메라(front5) 광축(z_optical)은 os_lidar 의 −x, 카메라는 라이다보다 앞(−x)
    t5, q5 = ca.tf_from_T_cam_lidar(cams["camera_front5"]["T_cam_lidar"])
    z_axis = ca.quat_xyzw_to_rot(q5)[:, 2]
    assert z_axis[0] < -0.99
    assert t5[0] < -0.5
    # 차량 좌표(앞/왼/위) 위치 ≈ (−x, −y, z)_os_lidar. 도구의 차량 축은 라이다 장착 기울기(~1°)를 보정한
    # 축이라 cm 단위로만 같다 (front5 는 약 0.8 m 앞 → 1.5 cm)
    e5 = yaml.safe_load((REF_OUT / "extrinsic" / "camera_front5.yaml").read_text())
    assert np.allclose([-t5[0], -t5[1], t5[2]], e5["camera_position_vehicle_m"]["value"], atol=0.03)


# ------------------------------------------------------------------ 디스크 추정 · 대기
def _bag(dur, gb, n_rgb=14, n_th=2):
    return {"duration_s": dur, "size_gb": gb, "n_rgb": n_rgb, "n_thermal": n_th, "ok": True}


def test_storage_estimate_is_conservative():
    d = oc.TOOL_DEFAULTS
    # 2026-09-24 야간 bag: 1000 s · 182 GB · 실측 작업 폴더 최대 78 GB (2026-09-27 통합본, 처음부터)
    est = oc.estimate_storage([_bag(1000, 182)], defaults=d)
    assert est["work_gb"] >= 78
    tool_need = est["work_gb"] * 1.3 + 20
    assert est["work_need_gb"] >= tool_need            # 도구가 디스크로 거절할 일이 없다
    assert est["work_need_gb"] >= 1.5 * 78              # 실측의 1.5배 이상
    # 15분 14대 주행 ≈ 75 GB 기준도
    est15 = oc.estimate_storage([_bag(900, 165)], defaults=d)
    assert est15["work_gb"] >= 75 and est15["work_need_gb"] >= 1.5 * 75
    # 창 수 상한 (30 × 40 s): 1시간 bag 이라도 끝없이 커지지 않지만 크기 기준은 비례
    est60 = oc.estimate_storage([_bag(3600, 650)], defaults=d)
    assert est60["windows"] == 30
    assert est60["work_gb"] >= oc.SIZE_FRACTION * 650 * 1200 / 3600 - 1e-6
    # 센서 선택: 열화상만이면 훨씬 작다
    est_th = oc.estimate_storage([_bag(1000, 182)], sensors=("thermal",), defaults=d)
    assert est_th["work_gb"] < est["work_gb"]
    # 여러 bag 은 합산
    est2 = oc.estimate_storage([_bag(500, 91), _bag(500, 91)], defaults=d)
    assert abs(est2["work_gb"] - est["work_gb"]) < 1e-6


def test_check_space_pending_logic():
    est = oc.estimate_storage([_bag(1000, 182)], defaults=oc.TOOL_DEFAULTS)
    need = est["work_need_gb"]
    free = {"/w": need + 10, "/o": 100}
    ok, msg, det = oc.check_space(est, "/w", "/o", free_fn=lambda p: free[p], same_fs_fn=lambda a, b: False)
    assert ok, msg
    # 같은 디스크면 결과 폴더 몫을 더한다
    ok, msg, det = oc.check_space(est, "/w", "/o", free_fn=lambda p: need + 1, same_fs_fn=lambda a, b: True)
    assert not ok and "모자람" in msg
    # 공간 부족 → 대기
    ok, msg, _ = oc.check_space(est, "/w", "/o", free_fn=lambda p: 50, same_fs_fn=lambda a, b: False)
    assert not ok
    # 이어서 실행: 이미 쓴 만큼 덜 필요
    ok, msg, det = oc.check_space(est, "/w", "/o", already_gb=need - 30,
                                  free_fn=lambda p: 60, same_fs_fn=lambda a, b: False)
    assert ok and det["work_need_gb"] <= 30 + 1e-6


def test_inspect_bag_metadata(tmp_path):
    d = tmp_path / "rec_x"
    d.mkdir()
    topics = [{"topic_metadata": {"name": n, "type": "t"}, "message_count": 1} for n in
              ["/ouster/points", "/gps/fix"] + [f"/camera{i}/image_rgb/compressed" for i in range(14)] +
              ["/thermal0/image_raw", "/thermal1/image_raw", "/thermal0/image_raw/metadata"]]
    (d / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "duration": {"nanoseconds": 999583354226}, "starting_time": {"nanoseconds_since_epoch": 1},
        "storage_identifier": "sqlite3", "topics_with_message_count": topics}}))
    (d / "rec_x_0.db3").write_bytes(b"\0" * 1000)
    b = oc.inspect_bag(d / "rec_x_0.db3")        # 파일을 줘도 폴더로
    assert b["ok"], b["error"]
    assert b["n_rgb"] == 14 and b["n_thermal"] == 2 and abs(b["duration_s"] - 999.58) < 0.01
    (d / "metadata.yaml").unlink()
    assert not oc.inspect_bag(d)["ok"]
    assert not oc.inspect_bag(tmp_path / "없음")["ok"]


# ------------------------------------------------------------------ 진행 이벤트
def _mock_stream():
    return [
        {"ev": "run_start"}, {"ev": "stage_start", "stage": "names"}, {"ev": "stage_end", "stage": "names", "ok": True},
        {"ev": "stage_skip", "stage": "preflight"}, {"ev": "stage_start", "stage": "extract"},
        {"ev": "stage_progress", "stage": "extract_scan", "done": 1, "total": 1},
        {"ev": "stage_progress", "stage": "extract", "done": 5, "total": 10},
    ]


def test_progress_mock_stream():
    p = oc.Progress()
    for ev in _mock_stream():
        p.feed_line(json.dumps(ev))
    assert p.stages["names"]["state"] == "ok" and p.stages["preflight"]["state"] == "skip"
    assert p.stages["extract"]["done"] == 5 and p.stages["extract"]["total"] == 10
    f = p.fraction()
    assert 0.05 < f < 0.2
    assert p.current()[0][0] == "extract"
    p.feed_line("이건 JSON 이 아님")
    assert p.bad_lines == 1
    p.feed({"ev": "refusal", "code": "not_enough_rotation", "msg": "x", "msg_ko": "회전 부족"})
    assert p.refusal["code"] == "not_enough_rotation"


def test_progress_partial_lines(tmp_path):
    f = tmp_path / "s.jsonl"
    p = oc.Progress()
    lines = [json.dumps(e) for e in _mock_stream()]
    f.write_text(lines[0] + "\n" + lines[1][:10])        # 둘째 줄은 쓰다 만 상태
    assert len(p.feed_file(f)) == 1
    with open(f, "a") as fh:
        fh.write(lines[1][10:] + "\n" + "\n".join(lines[2:]) + "\n")
    assert len(p.feed_file(f)) == len(lines) - 1
    assert p.bad_lines == 0


def test_progress_latest_attempt_clears_old_errors_without_rewinding_tail(tmp_path):
    path = tmp_path / 'events.jsonl'
    old = [
        {'ev': 'run_start', 'run_id': 'failed'},
        {'ev': 'warning', 'msg': 'old warning'},
        {'ev': 'task_failed', 'stage': 'thermal_lens', 'msg': 'old failure'},
        {'ev': 'error', 'msg': 'windows must be given in chronological order'},
        {'ev': 'refusal', 'code': 'old'},
        {'ev': 'stage_end', 'stage': 'thermal_solve', 'ok': False},
        {'ev': 'run_end', 'ok': False},
    ]
    path.write_text(''.join(json.dumps(ev) + '\n' for ev in old))
    progress = oc.Progress()
    assert len(progress.feed_file(path)) == len(old)
    cursor = progress._offset
    assert progress.error and progress.task_failed and progress.warnings
    resumed = [
        {'ev': 'run_start', 'run_id': 'resumed', 'total_windows': 3},
        {'ev': 'stage_skip', 'stage': 'lo'},
        {'ev': 'warning', 'msg': 'current warning'},
    ]
    payload = ''.join(json.dumps(ev) + '\n' for ev in resumed).encode()
    with path.open('ab') as handle:
        handle.write(payload[:-5])
    assert len(progress.feed_file(path)) == 2
    assert progress._offset > cursor
    assert progress.error is None and progress.refusal is None and progress.run_end is None
    assert progress.task_failed == [] and progress.warnings == []
    assert progress.stages['thermal_solve']['state'] == 'wait'
    assert progress.stages['lo']['state'] == 'skip'
    with path.open('ab') as handle:
        handle.write(payload[-5:])
    assert len(progress.feed_file(path)) == 1
    assert [ev['msg'] for ev in progress.warnings] == ['current warning']
    assert progress.feed_file(path) == []
    assert progress.run_start['run_id'] == 'resumed'
    assert progress.viz_progress == {'total_windows': 3}


@pytest.mark.skipif(not REF_EVENTS.is_file(), reason="참조 이벤트 없음")
def test_progress_real_stream():
    p = oc.Progress()
    n = 0
    fracs = []
    for line in REF_EVENTS.read_text().splitlines():
        if p.feed_line(line):
            n += 1
            fracs.append(p.fraction())
    assert n > 100 and p.bad_lines == 0
    assert p.run_end and p.run_end.get("gate_pass") is True
    assert fracs[-1] == 1.0
    assert all(p.stages[k]["state"] in ("ok", "skip") for k, _, _ in oc.STAGES), p.stages


# ------------------------------------------------------------------ 백그라운드 실행 · 중단 · 이어서
FAKE_TOOL = r'''#!/bin/bash
# 가짜 nontarget_cal: 진행 줄을 내보내고, 모드에 따라 끝난다
mode=${FAKE_MODE:-ok}
echo '{"ev": "run_start", "t": 0}'
echo '{"ev": "stage_start", "stage": "extract", "t": 0}'
for i in 1 2 3; do echo "{\"ev\": \"stage_progress\", \"stage\": \"extract\", \"done\": $i, \"total\": 3}"; sleep ${FAKE_SLEEP:-0.05}; done
echo "log line" >&2
if [ "$mode" = refuse_disk ]; then
  echo '{"ev": "refusal", "code": "insufficient_disk", "msg": "disk", "msg_ko": "디스크 부족"}'; exit 2; fi
if [ "$mode" = refuse ]; then
  echo '{"ev": "refusal", "code": "not_enough_rotation", "msg": "rot", "msg_ko": "회전 부족"}'; exit 2; fi
if [ "$mode" = error ]; then
  echo '{"ev": "error", "msg": "boom", "msg_ko": "처리 중 오류"}'; exit 1; fi
echo '{"ev": "stage_end", "stage": "extract", "ok": true}'
echo '{"ev": "run_end", "ok": true, "gate_pass": true}'
'''


def _fake(tmp_path, mode="ok", sleep="0.05"):
    exe = tmp_path / "fake_tool"
    exe.write_text(FAKE_TOOL)
    exe.chmod(0o755)
    os.environ["FAKE_MODE"] = mode
    os.environ["FAKE_SLEEP"] = sleep
    return exe


def _wait_done(job, timeout=10):
    t0 = time.time()
    while oc.pid_is_job(job["pid"], job["id"]):
        assert time.time() - t0 < timeout
        time.sleep(0.05)
    time.sleep(0.05)


@pytest.mark.parametrize("mode,state", [("ok", oc.DONE), ("refuse", oc.REFUSED), ("error", oc.FAILED),
                                        ("refuse_disk", oc.PENDING)])
def test_launch_and_settle(tmp_path, mode, state):
    exe = _fake(tmp_path, mode)
    job = oc.new_job(["/nonexistent/bag"], "zeroshot", tmp_path / "w", tmp_path / "o")
    att = oc.launch_detached(job, exe, root=tmp_path / "jobs", nice=0)
    assert job["state"] == oc.RUNNING and oc.pid_is_job(att["pid"], job["id"])
    _wait_done(job)
    assert oc.settle_state(job)
    assert job["state"] == state, job
    assert Path(att["stderr"]).read_text().strip() == "log line"
    p = oc.Progress()
    p.feed_file(att["stdout"])
    assert p.stages["extract"]["done"] == 3


def test_cancel_and_resume(tmp_path):
    exe = _fake(tmp_path, "ok", sleep="5")
    job = oc.new_job(["/b"], "zeroshot", tmp_path / "w", tmp_path / "o")
    oc.launch_detached(job, exe, root=tmp_path / "jobs", nice=0)
    time.sleep(0.3)
    job["attempts"][-1]["stop_reason"] = "사용자가 중단"
    assert oc.signal_job(job, signal.SIGTERM)
    _wait_done(job)
    oc.settle_state(job)
    assert job["state"] == oc.CANCELLED and job["state"] in oc.RESUMABLE
    # 이어서 실행 = 같은 명령 다시 (새 시도 파일)
    os.environ["FAKE_SLEEP"] = "0.01"
    att2 = oc.launch_detached(job, exe, root=tmp_path / "jobs", nice=0)
    assert att2["n"] == 2 and att2["cmd"] == job["attempts"][0]["cmd"]
    _wait_done(job)
    oc.settle_state(job)
    assert job["state"] == oc.DONE


def test_settle_interrupted_without_exitcode(tmp_path):
    job = oc.new_job(["/b"], "zeroshot", tmp_path / "w", tmp_path / "o")
    job["state"] = oc.RUNNING
    job["pid"] = 999999
    job["attempts"] = [{"n": 1, "stdout": str(tmp_path / "none"), "exitcode": str(tmp_path / "none.rc")}]
    oc.settle_state(job, alive=False)
    assert job["state"] == oc.INTERRUPTED


def test_tool_args_and_store(tmp_path):
    job = oc.new_job(["/a", "/b/x_0.db3"], "warm", tmp_path, tmp_path, init="/prev", sensors=("rgb",))
    a = oc.tool_args(job)
    assert a[:4] == ["run", "--bags", "/a", "/b/x_0.db3"] or a[:4] == ["run", "--bags", "/a", "/b"]
    assert "--init" in a and a[a.index("--mode") + 1] == "warm" and a[a.index("--sensors") + 1] == "rgb"
    with pytest.raises(ValueError):
        oc.new_job(["/a"], "warm", tmp_path, tmp_path)
    s = oc.JobStore(tmp_path / "jobs.json")
    s.add(job)
    assert oc.JobStore(tmp_path / "jobs.json").get(job["id"])["mode"] == "warm"


# ------------------------------------------------------------------ 결과 요약
@need_ref
def test_load_result_summary():
    r = oc.load_result(REF_OUT)
    assert r["gate_pass"] is True and len(r["cameras"]) == 16
    by = {c["camera"]: c for c in r["cameras"]}
    assert by["thermal_left"]["pass"] and by["camera_front5"]["pass"]
    # 게이트를 좁히면 카메라별로 떨어진다
    d = oc.tool_defaults()
    d["gate"]["rgb_rot_deg"] = 0.1
    r2 = oc.load_result(REF_OUT, defaults=d)
    assert not {c["camera"]: c for c in r2["cameras"]}["camera_rear_right"]["pass"]   # 0.407°


# ------------------------------------------------------------------ 적용 · 되돌리기
def _vehicle_copy(tmp_path):
    d = tmp_path / "FLIR_control" / "calibration"
    d.mkdir(parents=True)
    shutil.copy2(FLIR_CAL / "flir_camera_info.yaml", d)
    shutil.copy2(FLIR_CAL / "flir_camera_extrinsics.yaml", d)
    return d / "flir_camera_info.yaml", d / "flir_camera_extrinsics.yaml"


@need_flir
def test_node_like_parser_reads_existing_files():
    # 차량 파일은 적용할 때마다 바뀐다 — 값이 아니라 "노드 규칙으로 모든 항목을 읽을 수 있나" 만 본다
    text = (FLIR_CAL / "flir_camera_info.yaml").read_text()
    serials = list(yaml.safe_load(text)["camera_info_by_serial"])
    assert "25415248" in serials
    for s in serials:
        got = ca.parse_camera_info_like_node(text, s)
        assert got["distortion_model"] in ("plumb_bob", "equidistant") and len(got["k"]) == 9 and got["k"][0] > 0
    ex_text = (FLIR_CAL / "flir_camera_extrinsics.yaml").read_text()
    items = ca.parse_extrinsics_like_node(ex_text)
    assert len(items) == len(yaml.safe_load(ex_text)["extrinsics_by_serial"])
    assert all(it["parent_frame"] in ("flir_rig_frame", "os_lidar") for it in items)
    with pytest.raises(ValueError):
        ca.parse_camera_info_like_node(text, "00000000")
    # 블록 목록(여러 줄)은 노드가 못 읽는다 → 검증이 잡아야 한다
    with pytest.raises(ValueError):
        ca.parse_extrinsics_like_node("extrinsics_by_serial:\n  \"1\":\n    parent_frame: a\n    child_frame: b\n"
                                      "    translation_xyz_m:\n    - 0\n    rotation_xyzw: [0, 0, 0, 1]\n")


def _inventory():
    sm = ca.load_serial_map(SERIAL_MAP)
    inv = {}
    for dm, v in sm.items():
        if v["serial"]:
            n = v["topic_20260924"]
            inv[v["serial"]] = {"name": n, "frame_id": f"{n}_optical_frame"}
    return sm, inv


@need_ref
def test_plan_apply_serials():
    sm, inv = _inventory()
    res = ca.load_result_cameras(REF_OUT)
    rows = {r["camera"]: r for r in ca.plan_apply(res, sm, inv)}
    assert len(rows) == 16
    assert rows["camera_rear_left"]["serial"] == "25415251" and rows["camera_rear_left"]["ok"]
    f5 = rows["camera_front5"]
    assert f5["serial"] == "26076474" and f5["frame_id"] == "camera_26076474_optical_frame" and f5["selected"]
    assert rows["thermal_left"]["serial"] == "89905157" and rows["thermal_left"]["name"] == "thermal1"
    # 표에 시리얼이 없으면 적용 못 함
    unk = dict(sm, camera_rear_left=dict(sm["camera_rear_left"], serial=None))
    rl = {r["camera"]: r for r in ca.plan_apply(res, unk, inv)}["camera_rear_left"]
    assert not rl["ok"] and not rl["selected"] and "시리얼" in rl["issues"][0]
    # 사용자가 시리얼을 넣으면 적용 가능 — 단 확인 필요
    rows2 = {r["camera"]: r for r in ca.plan_apply(res, unk, inv, serial_overrides={"camera_rear_left": "25415251"})}
    rl2 = rows2["camera_rear_left"]
    assert rl2["ok"] and rl2["needs_confirm"] and not rl2["selected"]
    # 인벤토리를 못 읽었으면 (옛 이름으로 frame 을 지으면 엉뚱한 frame) 적용 못 함
    assert not any(r["ok"] for r in ca.plan_apply(res, sm, {}))
    # 토픽 이름의 시리얼과 모순되면 막는다
    bad = {r["camera"]: r for r in ca.plan_apply(res, sm, inv, serial_overrides={"camera_front5": "26075999"})}
    assert not bad["camera_front5"]["ok"]
    # 이미 쓴 시리얼과 겹치면 막는다
    dup = {r["camera"]: r for r in ca.plan_apply(res, sm, inv, serial_overrides={"camera_rear_left": "26076474"})}
    assert not (dup["camera_rear_left"]["ok"] and dup["camera_front5"]["ok"])


@need_ref
@need_flir
def test_apply_validate_rollback(tmp_path):
    ci, ex = _vehicle_copy(tmp_path)
    ci_old, ex_old = ci.read_text(), ex.read_text()
    sm, inv = _inventory()
    res = ca.load_result_cameras(REF_OUT)
    rows = ca.plan_apply(res, sm, inv, serial_overrides={"camera_rear_left": "25415251"})
    for r in rows:
        if r["camera"] == "camera_rear_left":
            r["selected"] = True        # 사용자가 확인함
    man = ca.apply_result(REF_OUT, rows, ci, ex, tmp_path / "applied", tool_commit="18e19b9")
    assert len(man["cameras"]) == 16
    ci_new, ex_new = ci.read_text(), ex.read_text()
    # 노드 규칙으로 다시 읽기: 모든 시리얼
    for r in rows:
        cam = res[r["camera"]]
        got = ca.parse_camera_info_like_node(ci_new, r["serial"])
        assert got["distortion_model"] == cam["model"]
        assert np.isclose(got["k"][0], cam["K"][0, 0], rtol=1e-15) and np.isclose(got["k"][5], cam["K"][1, 2])
        assert np.allclose(got["d"], cam["D"], rtol=1e-15, atol=0)
    items = {it["serial"]: it for it in ca.parse_extrinsics_like_node(ex_new)}
    f5 = items["26076474"]
    assert f5["parent_frame"] == "os_lidar" and f5["child_frame"] == "camera_26076474_optical_frame"
    T = ca.T_cam_lidar_from_tf(f5["t"], f5["q"])
    assert np.allclose(T, res["camera_front5"]["T_cam_lidar"], atol=1e-9)
    # 결과에 없는 원래 항목(자리표시)은 남는다: 26076473 은 결과에 있음, 파일의 8개 + 결과의 새 시리얼
    y = yaml.safe_load(ex_new)["extrinsics_by_serial"]
    assert set(yaml.safe_load(ex_old)["extrinsics_by_serial"]) <= set(y)
    assert len(y) == len(set(yaml.safe_load(ex_old)["extrinsics_by_serial"]) | {r["serial"] for r in rows})
    # 머리 주석은 유지
    assert ci_new.startswith(ci_old.splitlines()[0])
    # 백업 두 벌 (옆 파일 + 보관함)
    for f in man["files"]:
        assert Path(f["backup"]).read_text() == Path(f["backup_sidecar"]).read_text()
    assert (Path(man["archive"]) / "result" / "extrinsic" / "camera_front5.yaml").is_file()
    assert ca.list_applied(tmp_path / "applied")[0]["id"] == man["id"]
    # 되돌리기
    ca.rollback(man)
    assert ci.read_text() == ci_old and ex.read_text() == ex_old
    assert ca.list_applied(tmp_path / "applied")[0]["rolled_back"]


@need_ref
@need_flir
def test_rollback_refuses_after_external_edit(tmp_path):
    ci, ex = _vehicle_copy(tmp_path)
    sm, inv = _inventory()
    res = ca.load_result_cameras(REF_OUT)
    man = ca.apply_result(REF_OUT, ca.plan_apply(res, sm, inv), ci, ex, tmp_path / "applied")
    with open(ci, "a") as f:
        f.write("# 누가 손으로 고침\n")
    with pytest.raises(RuntimeError):
        ca.rollback(man)
    ca.rollback(man, force=True)
    assert "누가 손으로" not in ci.read_text()


@need_ref
def test_apply_to_missing_files_and_warm_export(tmp_path):
    """파일이 없던 차량(새 설치)에도 쓸 수 있고, 적용한 값을 warm-start 용 init 폴더로 되돌려 뽑으면
    도구(init_calib.load_init)가 같은 값을 읽는다."""
    ci = tmp_path / "c" / "flir_camera_info.yaml"
    ex = tmp_path / "c" / "flir_camera_extrinsics.yaml"
    sm, inv = _inventory()
    res = ca.load_result_cameras(REF_OUT)
    ca.apply_result(REF_OUT, ca.plan_apply(res, sm, inv), ci, ex, tmp_path / "applied")
    got = ca.export_vehicle_init(ci, ex, sm, tmp_path / "init")
    assert "camera_rear_left" not in got["cameras"] and len(got["cameras"]) == 15
    sys.path.insert(0, str(ROOT / "third_party" / "nontarget_cal"))
    from nontarget_cal.init_calib import load_init
    ini = load_init(str(tmp_path / "init"))
    assert set(ini["rgb"]) == {c for c in got["cameras"] if not c.startswith("thermal")}
    assert set(ini["thermal"]) == {"thermal_left", "thermal_right"}
    assert np.allclose(ini["rgb"]["camera_front5"]["T_cam_lidar"], res["camera_front5"]["T_cam_lidar"], atol=1e-9)
    K = res["camera_front5"]["K"]
    assert np.allclose(ini["rgb"]["camera_front5"]["intr"][:4], [K[0, 0], K[1, 1], K[0, 2], K[1, 2]])
    assert abs(ini["thermal"]["thermal_left"]["dt_s"] - res["thermal_left"]["time_offset_s"]) < 1e-12


def test_vehicle_paths():
    g = {"workdir": "~/FLIR_control"}
    ci, ex, _ = ca.vehicle_paths(g)
    assert ci == Path.home() / "FLIR_control" / "calibration" / "flir_camera_info.yaml"
    ci, ex, _ = ca.vehicle_paths(g, {"launch_args": {"extrinsics_yaml_path": "/abs/e.yaml"}})
    assert ex == Path("/abs/e.yaml")


def test_imported_job_from_result_folder(tmp_path):
    import json as _json
    out = tmp_path / "res"
    (out / "extrinsic").mkdir(parents=True)
    (out / "intrinsic").mkdir()
    (out / "summary.json").write_text(_json.dumps({"mode": "zeroshot", "sensors": ["rgb"], "bags": {"b0": "/x/rec"}}))
    j = oc.imported_job(out)
    assert j["state"] == oc.DONE and j["imported"] and j["out"] == str(out.resolve())
    assert j["workdir"] == "" and j["bags"] == ["/x/rec"] and j["sensors"] == ["rgb"]
    import pytest as _pt
    with _pt.raises(ValueError):
        oc.imported_job(tmp_path)          # summary.json 없음


def test_bundled_result_folder_loads():
    root = Path(__file__).resolve().parent.parent / "calib_results" / "20260924_night_zeroshot"
    if not root.is_dir():
        return
    cams = ca.load_result_cameras(root)
    assert len(cams) == 16
    r = oc.load_result(root)
    assert r["gate_pass"] and len(r["cameras"]) == 16


# ------------------------------------------------------------------ 차량에서 찾은 문제 (2026-09-27)
def test_num_exponent_keeps_dot():
    """소수점 없는 지수(1e-05)는 PyYAML 이 문자열로 읽어, 다음 적용 때 따옴표가 붙고 카메라 노드가 죽는다."""
    for v in (1e-05, -4e-05, 2.5e-06, 1e16, 0.001, 123.0, -0.3278882638):
        t = ca._num(v)
        back = yaml.safe_load(t)
        assert isinstance(back, float) and back == v, (v, t)
        float(t)        # std::stod 처럼


def test_validate_checks_untouched_entries():
    text = ("camera_info_by_serial:\n  \"1\":\n    camera_info:\n      distortion_model: \"equidistant\"\n"
            "      d: [0.01, \"-4e-05\", 0.0, 0.0]\n      k: [1, 0, 1, 0, 1, 1, 0, 0, 1]\n")
    probs = ca.validate_written(text, "extrinsics_by_serial: {}\n", {}, {})
    assert any("1 (기존 항목)" in p for p in probs)


def _fake_bag(tmp_path, name, rgb, thermal=("thermal0", "thermal1"), storage="sqlite3", cameras=None, dur_s=300):
    d = tmp_path / name
    d.mkdir(parents=True)
    names = ["/ouster/points", "/gps/fix"] + [f"/{n}/image_rgb/compressed" for n in rgb] + \
            [f"/{n}/image_raw" for n in thermal]
    (d / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "duration": {"nanoseconds": int(dur_s * 10**9)}, "starting_time": {"nanoseconds_since_epoch": 1},
        "storage_identifier": storage,
        "topics_with_message_count": [{"topic_metadata": {"name": n, "type": "t"}, "message_count": 1} for n in names]}}))
    if cameras:
        (d / "dataset_info.json").write_text(json.dumps({"cameras": cameras}))
    return oc.inspect_bag(d)


def test_short_clips_are_usable(tmp_path):
    """80 s 는 고른 bag 전체의 합 — 30 s 클립 하나는 창 하나 (도구 windows.min_length_s 15 s)."""
    assert _fake_bag(tmp_path, "clip_30", ["c1"], dur_s=30)["ok"]
    b = _fake_bag(tmp_path, "clip_10", ["c1"], dur_s=10)
    assert not b["ok"] and "너무 짧음" in b["error"]


def test_inspect_bag_rejects_mcap(tmp_path):
    b = _fake_bag(tmp_path, "rec_m", ["c1"], storage="mcap")
    assert not b["ok"] and "sqlite3" in b["error"]


def test_auto_name_map_by_serial(tmp_path):
    sm = ca.load_serial_map(SERIAL_MAP)
    serial = {k: v["serial"] for k, v in sm.items()}
    # 09-27 GUI 이름: 시리얼 → camera_N 등 (26076474 = camera_6, 25415255 = camera_5)
    now = {serial["camera_front1"]: "camera_1", serial["camera_front2"]: "camera_2", serial["camera_front3"]: "camera_3",
           serial["camera_front4"]: "camera_4", serial["camera_front5"]: "camera_6", serial["camera_front6"]: "camera_5",
           serial["camera_front7"]: "camera_7", serial["camera_front8"]: "camera_8", serial["camera_front9"]: "camera_9",
           serial["camera_top"]: "camera_top", serial["camera_side_left"]: "camera_side_left",
           serial["camera_side_right"]: "camera_side_right", serial["camera_rear_left"]: "camera_rear_left",
           serial["camera_rear_right"]: "camera_rear_right", "89905156": "thermal0", "89905157": "thermal1"}
    new_bag = _fake_bag(tmp_path, "route_001_x/rec_2", [v for v in now.values() if not v.startswith("thermal")])
    old_names = [v["topic_20260924"] for v in sm.values() if v["topic_20260924"] and not v["topic_20260924"].startswith("thermal")]
    old_bag = _fake_bag(tmp_path, "rec_1", old_names)
    cands = lambda launched: (lambda i: oc.naming_candidates(sm, oc.bag_camera_snapshot(i["path"]), launched))  # noqa: E731
    r = oc.auto_name_map([new_bag], sm, cands(now))
    assert r["ok"] and r["map"]["camera_front5"] == "camera_6" and r["map"]["thermal_left"] == "thermal1"
    assert len(r["map"]) == 16
    r = oc.auto_name_map([old_bag], sm, cands(now))
    assert r["ok"] and r["map"]["camera_front5"] == "camera_26076474" and "09-24" in r["why"]
    # 이름 체계가 다른 bag 은 대응표 하나로 못 돌린다
    assert not oc.auto_name_map([new_bag, old_bag], sm, cands(now))["ok"]
    # 기동 사본이 없어도 녹화 당시 기록이 있으면 그것으로
    snap_bag = _fake_bag(tmp_path, "rec_3", [v for v in now.values() if not v.startswith("thermal")], cameras=now)
    r = oc.auto_name_map([snap_bag], sm, cands({}))
    assert r["ok"] and "녹화 당시" in r["why"]
    # 아무 체계로도 설명 못 하면 이유와 함께 실패
    assert not oc.auto_name_map([_fake_bag(tmp_path, "rec_4", ["mystery"])], sm, cands(now))["ok"]
    p = oc.write_name_map(r["map"], tmp_path / "nm.yaml")
    assert yaml.safe_load(p.read_text())["camera_front6"] == "camera_5"


def test_progress_fraction_and_eta():
    """단계가 겹쳐 도는 실행: 진행률은 단조 증가, 남은 시간은 도구 추정(× 이 PC 보정)과 실제 속도를 섞는다."""
    p = oc.Progress()
    t0 = 1_000_000.0
    evs = [(0, {"ev": "run_start"}), (0, {"ev": "stage_start", "stage": "names"}),
           (300, {"ev": "stage_end", "stage": "names"}), (300, {"ev": "stage_start", "stage": "preflight"}),
           (306, {"ev": "stage_end", "stage": "preflight", "estimate": {"runtime_min": 62}}),
           (310, {"ev": "stage_start", "stage": "extract"}), (310, {"ev": "stage_start", "stage": "lo"}),
           (400, {"ev": "stage_progress", "stage": "rgb_tracks", "done": 1, "total": 210}),   # 1/210 에 뛰면 안 된다
           (1500, {"ev": "stage_end", "stage": "extract"}),
           (2300, {"ev": "stage_end", "stage": "lo"}),
           (3000, {"ev": "stage_progress", "stage": "rgb_tracks", "done": 150, "total": 210}),
           (3700, {"ev": "stage_progress", "stage": "validation", "done": 1, "total": 3})]   # 열화상 검증이 먼저 돈다
    last = 0.0
    for dt, e in evs:
        p.feed(dict(e, t=t0 + dt))
        f = p.fraction(t0 + dt)
        assert f >= last - 1e-9, (e, f, last)
        last = f
        if e.get("stage") == "rgb_tracks" and e["done"] == 1:
            assert f < 0.15
    assert last < 0.9                                   # 검증 1/3 로 끝 근처까지 뛰지 않는다
    step, what = p.now_text()
    assert step.endswith("단계") and "RGB 특징점 추적" in what
    rem = p.remaining_s(3700, t0 + 3700)
    assert 10 * 60 < rem < 60 * 60
    p.feed({"ev": "run_end", "t": t0 + 5000})
    assert p.fraction() == 1.0 and p.remaining_s(5000) == 0.0


def test_compare_verdict_rules():
    """기존 캘 vs 새 결과 판정: 문턱을 넘는 차이만 '낫다', 엇갈리면 강한 지표 쪽, 둘 다 강하면 '엇갈림'."""
    import calib_compare as cc
    assert cc._better(0.78, 0.83, cc.HELDOUT_ABS_PX, cc.HELDOUT_REL) == 1        # 새 쪽이 확실히 낮음
    assert cc._better(0.80, 0.81, cc.HELDOUT_ABS_PX, cc.HELDOUT_REL) == 0        # 문턱 안
    assert cc._better(0.90, 0.80, cc.HELDOUT_ABS_PX, cc.HELDOUT_REL) == -1
    assert cc._better(None, 0.8, 0.03) == 0
    assert cc.verdict([("h", 1, True), ("e", 0, False)])[0] == "new"
    assert cc.verdict([("h", 0, True), ("e", -1, False)])[0] == "old"
    assert cc.verdict([("h", 1, True), ("e", -1, False)])[0] == "new"            # 강한 지표 쪽
    assert cc.verdict([("h", 1, True), ("e", -1, True)])[0] == "mixed"
    assert cc.verdict([("h", 0, True)])[0] == "same"
