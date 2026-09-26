"""Real-job visualization uses source values, never synthetic convergence."""
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import online_calib as oc
from calib_viz.job import JobSnapshot
from calib_viz.stream import validate_manifest, validate_snapshot


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) if path.suffix == ".json" else yaml.safe_dump(value))


@pytest.fixture
def setup(tmp_path, monkeypatch):
    package = tmp_path / "package/nontarget_cal"
    config = {"paths": {"rig_design": "@pkg/data/rig_design.yaml"},
              "cameras": {"rgb": ["camera_front5"], "thermal": ["thermal_left"],
                          "rgb_size": [1920, 1200], "thermal_size": [640, 480]}}
    design = {"rgb": {"camera_front5": {"design_R_lidar_cam_euler_ZYX_deg": [90, 0, -90],
                                        "seed_intr_kb": [800, 820, 960, 600, 0, 0, 0, 0],
                                        "reference_T_lidar_cam": np.eye(4).tolist()}},
              "thermal": {"thermal_left": {"design_R_lidar_cam_euler_ZYX_deg": [90, 0, -90],
                                            "design_centre_lidar_m": [-.7, -.2, -.4],
                                            "nominal_intr_pinhole": [680, 680, 320, 240, 0, 0]}}}
    write(package / "config/default.yaml", config)
    write(package / "data/rig_design.yaml", design)
    monkeypatch.setattr(oc, "DEFAULT_VENV", tmp_path / "missing-venv")
    monkeypatch.setattr(oc, "VENDORED_CANDIDATES", [package.parent])
    job = {"id": "unit", "state": oc.RUNNING, "mode": "zeroshot", "sensors": ["rgb", "thermal"],
           "workdir": str(tmp_path / "work"), "out": str(tmp_path / "out")}
    return job, package, tmp_path


def result_files(root, names=("camera_front5", "thermal_left")):
    for index, name in enumerate(names):
        pose = np.eye(4)
        pose[:3, 3] = [index + .1, .2, .3]
        thermal = name.startswith("thermal")
        write(root / "extrinsic" / (name + ".yaml"), {"T_cam_lidar": pose.tolist(), "parent_frame": "os_lidar"})
        write(root / "intrinsic" / (name + ".yaml"),
              {"camera_matrix": [[800, 0, 960], [0, 800, 600], [0, 0, 1]],
               "distortion_coefficients": [0] * (5 if thermal else 4),
               "model": "plumb_bob" if thermal else "equidistant", "image_width": 1920, "image_height": 1200})
    return pose


def finished_result(root):
    result_files(root)
    metrics = {name: {"sensor": sensor, "rot_deg": .1, "pos_mm": 3., "along_axis_mm": 2., "track_reproj_px": .8,
                      "vote": {"pass": False, "gate": False}}
               for name, sensor in (("camera_front5", "rgb"), ("thermal_left", "thermal"))}
    write(root / "metrics.json", metrics)
    write(root / "summary.json", {"metrics": metrics, "windows": {"kept": ["S01", "S02"]},
                                  "validation": {"gate": {"pass": True, "failures": []}}})
    write(root / "rig.yaml", {"R_lidar_V": np.eye(3).tolist()})


def test_zero_shot_matches_actual_design_not_board_reference(setup):
    job, _, _ = setup
    manifest, event, decoded = JobSnapshot(job).snapshot()
    validate_manifest(manifest)
    validate_snapshot(event)
    assert manifest["mode"] == "job"
    assert manifest["cameras"]["camera_front5"]["K"][0][0] == 810
    rgb = np.asarray(event["cameras"]["camera_front5"]["T_cam_lidar"])
    assert np.allclose(rgb[:3, 3], 0)  # actual zero-shot solver starts at lidar centre
    assert np.allclose(rgb[:3, :3], [[0, 1, 0], [0, 0, -1], [-1, 0, 0]])
    thermal = np.asarray(event["cameras"]["thermal_left"]["T_cam_lidar"])
    assert np.allclose(-thermal[:3, :3].T @ thermal[:3, 3], [-.7, -.2, -.4])
    assert all(cam[key] is None for cam in event["cameras"].values()
               for key in ("sigma_rot_deg", "sigma_pos_mm", "reprojection_px", "gate_pass"))
    assert event["interpolate"] is False and event["provenance"]["synthetic"] is False
    assert decoded == {"matching": {}}


def test_progress_never_changes_initial_pose_or_metrics(setup):
    job, _, _ = setup
    adapter, progress = JobSnapshot(job), oc.Progress()
    _, initial, _ = adapter.snapshot(progress)
    for done in (1, 120, 350):
        progress.feed({"ev": "stage_progress", "stage": "rgb_tracks", "done": done, "total": 350})
        _, event, _ = adapter.snapshot(progress)
        assert event["cameras"] == initial["cameras"]
        assert "window" not in event and "total_windows" not in event
        assert event["eta_s"] is None
        assert event["stage"] == "rgb_tracks"
    assert event["progress"] > initial["progress"]


def test_true_windows_eta_warnings_and_errors(setup):
    job, _, _ = setup
    adapter, progress = JobSnapshot(job), oc.Progress()
    progress.feed({"ev": "stage_progress", "stage": "lo", "done": 3, "total": 25, "eta_s": 41})
    _, event, _ = adapter.snapshot(progress)
    assert (event["window"], event["total_windows"], event["eta_s"]) == (3, 25, 41)
    progress.feed({"ev": "warning", "msg_ko": "야간 영상"})
    progress.feed({"ev": "error", "msg_ko": "계산 오류"})
    _, event, _ = adapter.snapshot(progress, job={**job, "state": oc.FAILED, "state_msg": "프로세스 종료"})
    assert "야간 영상" in event["warnings"] and "계산 오류" in event["warnings"]
    assert "프로세스 종료" in event["status_text"]
    assert event["gate"]["pass"] is None
    assert event["window"] == 3


def test_extract_counts_windows_but_extract_scan_counts_bags(setup):
    job, _, _ = setup
    adapter, progress = JobSnapshot(job), oc.Progress()
    progress.feed({"ev": "stage_progress", "stage": "extract_scan", "done": 1, "total": 2})
    _, event, _ = adapter.snapshot(progress)
    assert "window" not in event
    assert event["stage"] == "extract"
    assert "extract_scan" in event["stage_label"]
    progress.feed({"ev": "stage_progress", "stage": "extract", "done": 3, "total": 26})
    _, event, _ = adapter.snapshot(progress)
    assert (event["window"], event["total_windows"]) == (3, 26)


def test_substage_normalizes_display_stage_and_keeps_real_label(setup):
    job, _, _ = setup
    progress = oc.Progress()
    progress.feed({"ev": "stage_progress", "stage": "thermal_final", "done": 2, "total": 3})
    _, event, _ = JobSnapshot(job).snapshot(progress)
    assert event["stage"] == "thermal_solve"
    assert "thermal_final" in event["stage_label"]
    assert "window" not in event


def test_warm_team_yaml_keeps_actual_values_and_nominal_metric_null(setup):
    job, _, tmp = setup
    result_files(tmp / "init")
    job.update(mode="warm", init=str(tmp / "init"))
    manifest, event, _ = JobSnapshot(job).snapshot()
    pose = np.asarray(event["cameras"]["camera_front5"]["T_cam_lidar"])
    assert np.allclose(pose[:3, 3], [.1, .2, .3])
    assert event["provenance"]["pose_source"] == "warm-start"
    assert event["cameras"]["camera_front5"]["reprojection_px"] is None
    validate_manifest(manifest)


def test_missing_warm_init_is_not_replaced_by_fake_design(setup):
    job, _, tmp = setup
    job.update(mode="warm", init=str(tmp / "missing"))
    _, event, _ = JobSnapshot(job).snapshot()
    assert event["cameras"] == {}
    assert "초기값 표시 불가" in event["warnings"][0]


def test_warm_missing_thermal_uses_actual_tool_fallback(setup):
    job, _, tmp = setup
    result_files(tmp / "init", ["camera_front5"])
    job.update(mode="warm", init=str(tmp / "init"))
    _, event, _ = JobSnapshot(job).snapshot()
    thermal = np.asarray(event["cameras"]["thermal_left"]["T_cam_lidar"])
    assert np.allclose(-thermal[:3, :3].T @ thermal[:3, 3], [-.7, -.2, -.4])
    assert "열화상 설계값 사용" in event["warnings"][0]


@pytest.mark.parametrize("form", ["result", "json_dir", "thermal_yaml"])
def test_warm_additional_tool_formats(setup, form):
    job, _, tmp = setup
    pose = np.eye(4).tolist()
    if form == "result":
        path = tmp / "result.json"
        write(path, {"cameras": {"camera_front5": {"T_cam_lidar": pose, "intr_kb": [800, 800, 960, 600, 0, 0, 0, 0]}}})
    elif form == "json_dir":
        path = tmp / "json_dir"
        write(path / "camera_front5.json", {"T_cam_lidar": pose, "intr": [800, 800, 960, 600, 0, 0, 0, 0]})
    else:
        path = tmp / "thermal.yaml"
        write(path, {"cameras": {"thermal_left": {"T_cam_lidar": pose,
                    "intrinsics": {"camera_matrix": [[680, 0, 320], [0, 680, 240], [0, 0, 1]],
                                   "distortion_coefficients": [0, 0, 0, 0, 0]}}}})
        job["sensors"] = ["thermal"]
    job.update(mode="warm", init=str(path))
    manifest, event, _ = JobSnapshot(job).snapshot()
    name = "thermal_left" if form == "thermal_yaml" else "camera_front5"
    assert event["cameras"][name]["T_cam_lidar"] == pose
    validate_manifest(manifest)


def test_final_only_after_done_uses_real_gate_and_no_interpolation(setup):
    job, _, tmp = setup
    finished_result(tmp / "out")
    adapter = JobSnapshot(job)
    _, before, _ = adapter.snapshot()
    assert before["provenance"]["pose_source"] == "initial-design"
    manifest, event, _ = adapter.snapshot(job={**job, "state": oc.DONE})
    validate_manifest(manifest)
    validate_snapshot(event)
    assert event["provenance"]["pose_source"] == "final-result"
    assert event["interpolate"] is False
    assert event["gate"]["pass"] is True
    assert event["cameras"]["camera_front5"]["gate_pass"] is True
    assert event["cameras"]["camera_front5"]["validation_vote"] is False
    assert event["cameras"]["camera_front5"]["informational_checks"]
    assert event["progress"] == 1.0
    assert event["eta_s"] == 0.0
    assert event["cameras"]["camera_front5"]["sigma_rot_deg"] == .1
    assert event["window"] == event["total_windows"] == 2
    assert manifest["R_lidar_V"] == np.eye(3).tolist()


@pytest.mark.parametrize("state", [oc.FAILED, oc.CANCELLED, oc.INTERRUPTED, oc.REFUSED])
def test_unsuccessful_attempt_never_shows_existing_output_as_success(setup, state):
    job, _, tmp = setup
    finished_result(tmp / "out")
    _, event, _ = JobSnapshot({**job, "state": state}).snapshot()
    assert event["provenance"]["pose_source"] == "initial-design"
    assert event["gate"]["pass"] is None


def test_incomplete_finished_result_retries_without_losing_initial(setup):
    job, _, tmp = setup
    finished_result(tmp / "out")
    missing = tmp / "out/intrinsic/thermal_left.yaml"
    saved = missing.read_text()
    missing.unlink()
    job["state"] = oc.DONE
    adapter = JobSnapshot(job)
    _, event, _ = adapter.snapshot()
    assert event["provenance"]["pose_source"] == "initial-design"
    assert "최종 결과 표시 대기" in event["warnings"][-1]
    missing.write_text(saved)
    _, event, _ = adapter.snapshot()
    assert event["provenance"]["pose_source"] == "final-result"
    # Successful metadata is cached: later deletion cannot trigger more I/O.
    missing.unlink()
    _, event, _ = adapter.snapshot()
    assert event["provenance"]["pose_source"] == "final-result"


def test_overridden_design_and_sensor_subset(setup):
    job, _, tmp = setup
    design = {"rgb": {"camera_custom": {"design_R_lidar_cam_euler_ZYX_deg": [0, 0, 0],
                                         "seed_intr_kb": [90, 110, 100, 50, 0, 0, 0, 0]}}}
    write(tmp / "custom_design.yaml", design)
    write(tmp / "custom_config.yaml", {"paths": {"rig_design": str(tmp / "custom_design.yaml")},
                                        "cameras": {"rgb": ["camera_custom"], "rgb_size": [200, 100]}})
    job.update(config=str(tmp / "custom_config.yaml"), sensors=["rgb"])
    manifest, event, _ = JobSnapshot(job).snapshot()
    assert set(event["cameras"]) == {"camera_custom"}
    assert manifest["cameras"]["camera_custom"]["K"][0][0] == 100


def test_recorded_custom_executable_uses_its_own_venv_config(setup):
    job, package, tmp = setup
    custom = tmp / "custom-venv"
    custom_package = custom / "lib/python3.12/site-packages/nontarget_cal"
    shutil.copytree(package, custom_package)
    design_path = custom_package / "data/rig_design.yaml"
    design = yaml.safe_load(design_path.read_text())
    design["rgb"]["camera_front5"]["seed_intr_kb"][:2] = [1100, 1100]
    write(design_path, design)
    job["attempts"] = [{"cmd": [str(custom / "bin/nontarget_cal"), "run"]}]
    manifest, event, _ = JobSnapshot(job).snapshot()
    assert manifest["cameras"]["camera_front5"]["K"][0][0] == 1100
    assert not event["warnings"]


def test_unknown_custom_executable_never_uses_unrelated_bundled_design(setup):
    job, _, tmp = setup
    job["attempts"] = [{"cmd": [str(tmp / "custom-tool"), "run"]}]
    _, event, _ = JobSnapshot(job).snapshot()
    assert event["cameras"] == {}
    assert "실행 도구의 초기 설계 설정" in event["warnings"][0]
    finished_result(tmp / "out")
    _, final, _ = JobSnapshot({**job, "state": oc.DONE}).snapshot()
    assert final["provenance"]["pose_source"] == "final-result"
    assert final["gate"]["pass"] is True


def test_unknown_custom_executable_allows_explicit_rig_design(setup):
    job, package, tmp = setup
    job["attempts"] = [{"cmd": [str(tmp / "custom-tool"), "run"]}]
    write(tmp / "custom.yaml", {"paths": {"rig_design": str(package / "data/rig_design.yaml")}})
    job["config"] = str(tmp / "custom.yaml")
    manifest, event, _ = JobSnapshot(job).snapshot()
    assert set(event["cameras"]) == {"camera_front5", "thermal_left"}
    validate_manifest(manifest)


def test_first_recorded_executable_invalidates_prelaunch_design_cache(setup):
    job, _, tmp = setup
    adapter = JobSnapshot(job)
    _, before, _ = adapter.snapshot()
    assert before["cameras"]
    launched = {**job, "attempts": [{"cmd": [str(tmp / "custom-tool"), "run"]}]}
    _, after, _ = adapter.snapshot(job=launched)
    assert after["cameras"] == {}
    assert after["seq"] > before["seq"]
    assert "실행 도구의 초기 설계 설정" in after["warnings"][0]
