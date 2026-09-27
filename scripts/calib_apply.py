#!/usr/bin/env python3
# calib_apply.py — 온라인 캘리브레이션 결과를 차량의 camera_info 와 TF 에 반영 (백업 · 검증 · 되돌리기).
#
# 차량의 카메라 스택 (FLIR_control, sensors.yaml 의 flir_cameras 센서군, 작업 디렉터리 ~/FLIR_control):
#   - camera_info : 런치 인자 camera_info_yaml_path (기본 calibration/flir_camera_info.yaml, 리포 루트 기준)
#                   camera_info_by_serial.<시리얼>.camera_info.{distortion_model,d,k,r,p,…}
#                   가시광 · 열화상 노드 모두 같은 파일을 읽는다 (flir_spinnaker_camera_node, 기동할 때 한 번)
#   - TF          : 런치 인자 extrinsics_yaml_path (기본 calibration/flir_camera_extrinsics.yaml)
#                   extrinsics_by_serial.<시리얼>.{parent_frame, child_frame, translation_xyz_m, rotation_xyzw}
#                   flir_camera_extrinsics_tf_node 가 기동할 때 /tf_static 으로 (parent → child) 발행
# 두 파일 모두 **시리얼**이 키다 — 토픽 이름이 바뀌어도 (2026-09-24 차량 이름 대응 변경) 다른 카메라에
# 붙지 않는다. 그래서 결과(캘리브레이션 이름 camera_front1 …)를 config/camera_serial_map.yaml 로 시리얼에
# 옮겨 적는다. 시리얼을 모르는 카메라(camera_rear_left)는 사용자가 시리얼을 넣고 확인해야 적용된다.
#
# 노드의 YAML 읽기는 PyYAML 이 아니라 줄 단위 파서다 (두 칸 들여쓰기, 목록은 한 줄 [a, b, c], 키 이름으로
# 찾음). 그래서 쓰는 형식을 고정하고, 쓴 뒤 같은 규칙의 파서(아래 parse_*_like_node)로 다시 읽어 값을 확인한다.
#
# 좌표 규약:
#   도구 결과  x_cam = R · x_lidar + t   (T_cam_lidar, parent os_lidar = Ouster 라이다 프레임, x 가 차량 뒤쪽)
#              카메라 좌표는 OpenCV 광학 좌표 (x 오른쪽, y 아래, z 앞) = ROS 의 *_optical_frame 과 같다
#   ROS TF     parent → child 변환 = 부모 프레임에서 본 자식의 자세:  x_parent = R_pc · x_child + t_pc
#              → R_pc = Rᵀ,  t_pc = −Rᵀ t  (= 카메라 중심의 os_lidar 좌표; 결과 YAML 의 camera_position_in_os_lidar_m)
#   os_lidar 는 ouster_ros 가 os_sensor 아래에 발행하는 프레임과 같은 이름 (lidar_driver_params.yaml lidar_frame)
#   이라 카메라 TF 가 라이다 TF 트리에 바로 붙는다.

import hashlib
import json
import math
import os
import re
import shutil
import time
from pathlib import Path

import numpy as np
import yaml

PARENT_FRAME = "os_lidar"
CAMERA_INFO_DEFAULT = "calibration/flir_camera_info.yaml"
EXTRINSICS_DEFAULT = "calibration/flir_camera_extrinsics.yaml"
APPLY_NOTE = "# DM_clipGUI 온라인 캘리브레이션 적용"


# ---------------------------------------------------------------- 회전 · 변환
def rot_to_quat_xyzw(R):
    """회전 행렬 → 단위 쿼터니언 (x, y, z, w), w ≥ 0."""
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return q


def quat_xyzw_to_rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_from_T_cam_lidar(T_cam_lidar):
    """도구의 T_cam_lidar (lidar → camera) → ROS TF os_lidar → camera_optical (translation, quat xyzw)."""
    T = np.asarray(T_cam_lidar, float)
    R, t = T[:3, :3], T[:3, 3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6) or np.linalg.det(R) < 0.999:
        raise ValueError("T_cam_lidar 의 회전이 직교 행렬이 아닙니다")
    Rpc = R.T
    tpc = -R.T @ t
    return tpc, rot_to_quat_xyzw(Rpc)


def T_cam_lidar_from_tf(translation, quat_xyzw):
    """tf_from_T_cam_lidar 의 역."""
    Rpc = quat_xyzw_to_rot(quat_xyzw)
    T = np.eye(4)
    T[:3, :3] = Rpc.T
    T[:3, 3] = -Rpc.T @ np.asarray(translation, float)
    return T


# ---------------------------------------------------------------- 결과 폴더 읽기
def load_result_cameras(out):
    """nontarget_cal 결과 폴더 → {캘리브레이션 이름: {...}}.

    extrinsic/<cam>.yaml (T_cam_lidar, topic, time_offset_s …) + intrinsic/<cam>.yaml (K, D, model, 크기).
    """
    out = Path(out)
    cams = {}
    for f in sorted((out / "extrinsic").glob("*.yaml")):
        c = f.stem
        e = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
        ip = out / "intrinsic" / f"{c}.yaml"
        if not ip.is_file():
            continue
        i = yaml.safe_load(ip.read_text(encoding="utf-8")) or {}
        if e.get("parent_frame", PARENT_FRAME) != PARENT_FRAME:
            raise ValueError(f"{f}: parent_frame {e.get('parent_frame')} (기대: {PARENT_FRAME})")
        K = np.asarray(i["camera_matrix"], float)
        model = i.get("model") or ("equidistant" if len(i.get("distortion_coefficients", [])) == 4 else "plumb_bob")
        D = [float(x) for x in i["distortion_coefficients"]]
        if model == "equidistant":
            D = (D + [0.0] * 4)[:4]
        else:
            D = (D + [0.0] * 5)[:5]
        cams[c] = {
            "camera": c, "sensor": "thermal" if c.startswith("thermal") else "rgb",
            "T_cam_lidar": np.asarray(e["T_cam_lidar"], float),
            "K": K, "D": D, "model": model,
            "width": int(i.get("image_width", 0)), "height": int(i.get("image_height", 0)),
            "topic": e.get("topic"), "time_offset_s": e.get("time_offset_s"),
            "row_readout_s": e.get("row_readout_s"),
            "sigma": e.get("uncertainty_empirical_1sigma") or {},
            "data": e.get("data"), "generated_by": e.get("generated_by"),
            "position_in_lidar": e.get("camera_position_in_os_lidar_m"),
        }
    return cams


# ---------------------------------------------------------------- 시리얼 · 인벤토리
SERIAL_RX = re.compile(r"^\d{6,10}$")
SERIAL_IN_TOPIC = re.compile(r"_(\d{8})$")


def load_serial_map(path):
    """config/camera_serial_map.yaml → {캘리브레이션 이름: {"serial", "confidence", "topic_20260924", "evidence"}}."""
    y = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    out = {}
    for e in (y.get("cameras") or []) + (y.get("thermal") or []):
        s = e.get("serial")
        out[e["dm_name"]] = {"serial": str(s) if s not in (None, "") else None,
                             "confidence": e.get("confidence", ""), "topic_20260924": e.get("topic_20260924"),
                             "evidence": e.get("evidence", "")}
    return out


def plan_apply(result, serial_map, inventory=None, serial_overrides=None, selected=None, gate=None):
    """카메라마다 무엇을 어디에 쓸지. 반환: 행 목록.

    result: load_result_cameras. inventory: {serial: {"name", "frame_id"}} (차량 인벤토리 · GUI 설정 반영).
    serial_overrides: {캘리브레이션 이름: 시리얼} (사용자가 넣은 값 — 모르는 시리얼 · 고친 시리얼).
    selected: 적용할 캘리브레이션 이름 집합 (None = 전부). gate: {이름: 합격?} (load_result 의 카메라별 판정).
    행: {"camera", "serial", "serial_source", "frame_id", "name", "ok", "issues", "needs_confirm", "selected"}
    ok=False 면 적용하지 않는다. needs_confirm 은 사용자 확인이 있어야 적용 (모르는 시리얼 등).
    """
    inventory = inventory or {}
    serial_overrides = serial_overrides or {}
    rows = []
    seen = {}
    for c in sorted(result):
        r = result[c]
        sm = serial_map.get(c) or {}
        issues, confirm = [], []
        serial, source = sm.get("serial"), f"표 ({sm.get('confidence', '?')})"
        if c in serial_overrides and serial_overrides[c]:
            serial, source = str(serial_overrides[c]).strip(), "직접 입력"
            if sm.get("serial") and serial != sm.get("serial"):
                confirm.append(f"표의 시리얼 {sm['serial']} 과 다름")
            elif not sm.get("serial"):
                confirm.append("시리얼 표에 없는 카메라 — 직접 넣은 시리얼이 맞는지 확인")
        if not serial:
            issues.append("시리얼을 모름 — 시리얼을 넣어야 적용됩니다")
        elif not SERIAL_RX.match(serial):
            issues.append(f"시리얼 형식이 아님: {serial}")
        # 결과의 토픽 이름에 시리얼이 들어 있으면(camera_26076474) 그 시리얼과 맞아야 한다
        m = SERIAL_IN_TOPIC.search(str(r.get("topic") or ""))
        if serial and m and m.group(1) != serial:
            issues.append(f"결과의 토픽 {r['topic']} 의 시리얼 {m.group(1)} 과 다름")
        if sm.get("confidence") == "set" and source.startswith("표"):
            pass    # 그룹 안 배정은 FLIR_control 표 기준 — 표의 설명 참고 (README)
        inv = inventory.get(serial) if serial else None
        if not inventory:
            # 인벤토리 없이 옛 이름(topic_20260924)으로 frame 을 지으면 실제 노드의 frame_id 와 달라
            # TF 가 아무 카메라에도 안 붙는다 — 적용하지 않는다
            issues.append("차량 인벤토리를 읽지 못해 카메라 frame 이름을 알 수 없음")
        elif serial and inv is None:
            confirm.append("차량 인벤토리에 없는 시리얼 (지금 안 꽂힌 카메라?)")
        name = (inv or {}).get("name") or sm.get("topic_20260924") or c
        frame = (inv or {}).get("frame_id") or f"{name}_optical_frame"
        if serial in seen:
            issues.append(f"시리얼 {serial} 이 {seen[serial]} 과 겹침")
        elif serial:
            seen[serial] = c
        passed = True if gate is None else gate.get(c, True)
        rows.append({"camera": c, "sensor": r["sensor"], "serial": serial, "serial_source": source,
                     "name": name, "frame_id": frame, "topic_in_bag": r.get("topic"),
                     "ok": not issues, "issues": issues, "needs_confirm": confirm,
                     "gate_pass": passed,
                     "selected": (selected is None or c in selected) and not issues and not confirm and passed})
    return rows


# ---------------------------------------------------------------- YAML 쓰기 (노드 파서가 읽는 형식)
def _num(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    f = float(v)
    if not math.isfinite(f):
        raise ValueError(f"유한하지 않은 값: {v}")
    r = repr(f)
    # YAML 1.1(PyYAML)은 소수점 없는 지수(1e-05)를 문자열로 읽는다 — 다음 적용 때 "1e-05" 로 따옴표가 붙어
    # 카메라 노드의 stod 가 기동 중에 죽는다. 항상 1.0e-05 꼴로.
    if "e" in r and "." not in r.split("e")[0]:
        m, e = r.split("e")
        r = f"{m}.0e{e}"
    return r


def _scalar(v):
    if v is None:
        return "null"
    if isinstance(v, (bool, int, float, np.floating, np.integer)):
        return _num(v.item() if hasattr(v, "item") else v)
    return json.dumps(str(v), ensure_ascii=False)


def _is_scalar(v):
    return v is None or isinstance(v, (str, bool, int, float, np.floating, np.integer))


def emit(obj, indent=0):
    """dict/list → YAML 줄들. 스칼라 목록은 한 줄 [a, b], dict 키는 따옴표 (시리얼 "25415248")."""
    pad = " " * indent
    lines = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            key = json.dumps(str(k)) if (SERIAL_RX.match(str(k)) or not re.match(r"^[A-Za-z_][\w.\-]*$", str(k))) else str(k)
            if isinstance(v, dict):
                if not v:
                    lines.append(f"{pad}{key}: {{}}")
                else:
                    lines.append(f"{pad}{key}:")
                    lines += emit(v, indent + 2)
            elif isinstance(v, (list, tuple, np.ndarray)):
                v = list(np.asarray(v).tolist()) if isinstance(v, np.ndarray) else list(v)
                if all(_is_scalar(x) for x in v):
                    lines.append(f"{pad}{key}: [" + ", ".join(_scalar(x) for x in v) + "]")
                else:
                    lines.append(f"{pad}{key}:")
                    for x in v:
                        sub = emit(x, indent + 4)
                        lines.append(f"{pad}  - " + sub[0].lstrip())
                        lines += sub[1:]
            else:
                lines.append(f"{pad}{key}: {_scalar(v)}")
    else:
        lines.append(pad + _scalar(obj))
    return lines


def header_comments(text):
    """파일 머리의 주석 줄들 (값 앞)."""
    out = []
    for line in text.splitlines():
        if line.strip() == "" or line.lstrip().startswith("#"):
            out.append(line)
        else:
            break
    # 예전 적용 표시 줄은 한 줄만 남게 (적용할 때마다 쌓이지 않게)
    out = [l for l in out if not l.startswith(APPLY_NOTE)]
    while out and not out[-1].strip():
        out.pop()
    return out


def _stamp():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def camera_info_entry(cam, row, meta):
    K = np.asarray(cam["K"], float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return {
        "camera_name": row["name"],
        "frame_id": row["frame_id"],
        "camera_info": {
            "distortion_model": cam["model"],
            "d": [float(x) for x in cam["D"]],
            "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
        },
        "calibration": {
            "method": "nontarget_cal (targetless, online)",
            "calib_name": cam["camera"],
            "generated_at_utc": meta["applied_at_utc"],
            "source": meta["source"],
            "image_width": cam["width"], "image_height": cam["height"],
            "focal_1sigma_px": cam["sigma"].get("focal_px"),
        },
    }


def extrinsic_entry(cam, row, meta):
    t, q = tf_from_T_cam_lidar(cam["T_cam_lidar"])
    calib = {
        "method": "nontarget_cal (targetless, online)",
        "calib_name": cam["camera"],
        "generated_at_utc": meta["applied_at_utc"],
        "source": meta["source"],
        "convention": "parent->child: x_os_lidar = R(q) x_optical + t",
        "rot_1sigma_deg": cam["sigma"].get("rot_deg"),
        "pos_1sigma_mm": cam["sigma"].get("pos_mm"),
    }
    # 열화상 시각 모델 — warm-start 로 다시 읽을 수 있게 남긴다 (노드는 안 읽음)
    if cam.get("time_offset_s") is not None:
        calib["time_offset_s"] = float(cam["time_offset_s"])
    if cam.get("row_readout_s") is not None:
        calib["row_readout_s"] = float(cam["row_readout_s"])
    return {
        "camera_name": row["name"],
        "parent_frame": PARENT_FRAME,
        "child_frame": row["frame_id"],
        "translation_xyz_m": [float(x) for x in t],
        "rotation_xyzw": [float(x) for x in q],
        "calibration": calib,
    }


def _load_yaml_file(path):
    p = Path(path)
    if not p.is_file():
        return "", {}
    text = p.read_text(encoding="utf-8")
    return text, (yaml.safe_load(text) or {})


def render_camera_info(old_text, old, entries, note):
    reg = {str(k): v for k, v in ((old or {}).get("camera_info_by_serial") or {}).items()}
    reg.update(entries)
    doc = {"version": (old or {}).get("version", 2), "camera_info_by_serial": reg}
    for k, v in (old or {}).items():
        if k not in doc:
            doc[k] = v
    head = header_comments(old_text) + [f"# {note}"]
    return "\n".join(head + emit(doc)) + "\n"


def render_extrinsics(old_text, old, entries, note):
    reg = {str(k): v for k, v in ((old or {}).get("extrinsics_by_serial") or {}).items()}
    reg.update(entries)
    doc = {"version": (old or {}).get("version", 1),
           "reference_frame": (old or {}).get("reference_frame", PARENT_FRAME),
           "extrinsics_by_serial": reg}
    for k, v in (old or {}).items():
        if k not in doc:
            doc[k] = v
    head = header_comments(old_text) + [f"# {note}"]
    return "\n".join(head + emit(doc)) + "\n"


# ---------------------------------------------------------------- 노드와 같은 규칙으로 읽기 (검증용)
# flir_spinnaker_camera_node.cpp ApplySerialIndexedCameraInfoYamlOverrides /
# flir_camera_extrinsics_tf_node.cpp LoadExtrinsicsYaml 을 옮긴 것.
def _remove_comment(v):
    sq = dq = False
    for i, ch in enumerate(v):
        if ch == "'" and not dq:
            sq = not sq
        elif ch == '"' and not sq:
            dq = not dq
        elif ch == "#" and not sq and not dq:
            return v[:i]
    return v


def _strip_quotes(v):
    if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
        return v[1:-1]
    return v


def _scalar_value(line, key):
    t = _remove_comment(line).strip()
    if not t or not t.startswith(key + ":"):
        return None
    return t[len(key) + 1:].strip()


def _map_key(line):
    t = _remove_comment(line).strip()
    if not t or not t.endswith(":"):
        return None
    return _strip_quotes(t[:-1].strip())


def _indent(line):
    return len(line) - len(line.lstrip(" "))


def _double_list(raw, field):
    t = raw.strip()
    if len(t) < 2 or t[0] != "[" or t[-1] != "]":
        raise ValueError(f"'{field}' 가 한 줄 목록 [..] 이 아님")
    body = t[1:-1].strip()
    if not body:
        return []
    out = []
    for tok in body.split(","):
        tok = tok.strip()
        if not tok:
            raise ValueError(f"'{field}' 에 빈 값")
        out.append(float(tok))      # std::stod 와 같이 전부 숫자여야 한다
    return out


def parse_camera_info_like_node(text, serial):
    """노드가 이 시리얼로 읽을 camera_info 값. 못 읽으면 ValueError."""
    got = {}
    in_reg = in_target = in_ci = in_roi = False
    reg_i = ser_i = ci_i = roi_i = 0
    saw = False
    for line in text.splitlines():
        t = line.strip()
        if not t or t.startswith("#"):
            continue
        ind = _indent(line)
        if not in_reg:
            if _map_key(line) == "camera_info_by_serial":
                in_reg, reg_i = True, ind
            continue
        if ind <= reg_i and _map_key(line) != "camera_info_by_serial":
            break
        if ind == reg_i + 2:
            k = _map_key(line)
            if k is not None:
                in_target = k == serial
                saw = saw or in_target
                ser_i, in_ci, in_roi = ind, False, False
                continue
        if not in_target or ind <= ser_i:
            continue
        if _map_key(line) == "camera_info":
            in_ci, ci_i, in_roi = True, ind, False
            continue
        if not in_ci:
            continue
        if ind <= ci_i:
            in_ci = in_roi = False
            continue
        if in_roi and ind <= roi_i:
            in_roi = False
        if _map_key(line) == "roi":
            in_roi, roi_i = True, ind
            continue
        if in_roi:
            continue
        for key, n in (("distortion_model", None), ("d", 0), ("k", 9), ("r", 9), ("p", 12)):
            v = _scalar_value(line, key)
            if v is None:
                continue
            if key == "distortion_model":
                got[key] = _strip_quotes(v)
            else:
                vals = _double_list(v, "camera_info." + key)
                if n and len(vals) != n:
                    raise ValueError(f"camera_info.{key} 는 {n}개여야 함 ({len(vals)})")
                got[key] = vals
            break
    if not saw:
        raise ValueError(f"시리얼 {serial} 항목이 없음")
    if not got:
        raise ValueError(f"시리얼 {serial} 에 camera_info 값이 없음")
    return got


def parse_extrinsics_like_node(text):
    """노드가 발행할 변환 목록 [{serial, camera_name, parent_frame, child_frame, t, q}]. 못 읽으면 ValueError."""
    ref = ""
    items, cur = [], None
    in_ex = False
    ex_i = ser_i = 0
    for raw in text.splitlines():
        wc = _remove_comment(raw)
        t = wc.strip()
        if not t:
            continue
        v = _scalar_value(raw, "reference_frame")
        if v is not None:
            ref = _strip_quotes(v)
            continue
        ind = _indent(wc)
        if not in_ex:
            if _map_key(raw) == "extrinsics_by_serial":
                in_ex, ex_i = True, ind
            continue
        if ind <= ex_i and _map_key(raw) != "extrinsics_by_serial":
            break
        if ind == ex_i + 2:
            k = _map_key(raw)
            if k is not None:
                if cur:
                    items.append(cur)
                cur, ser_i = {"serial": k}, ind
                continue
        if cur is None or ind <= ser_i:
            continue
        for key in ("camera_name", "parent_frame", "child_frame"):
            v = _scalar_value(raw, key)
            if v is not None:
                cur[key] = _strip_quotes(v)
                break
        else:
            v = _scalar_value(raw, "translation_xyz_m")
            if v is not None:
                vals = _double_list(v, "translation_xyz_m")
                if len(vals) != 3:
                    raise ValueError("translation_xyz_m 는 3개")
                cur["t"] = vals
                continue
            v = _scalar_value(raw, "rotation_xyzw")
            if v is not None:
                vals = _double_list(v, "rotation_xyzw")
                if len(vals) != 4:
                    raise ValueError("rotation_xyzw 는 4개")
                cur["q"] = vals
    if cur:
        items.append(cur)
    for it in items:
        it.setdefault("parent_frame", ref)
        for k in ("parent_frame", "child_frame", "t", "q"):
            if not it.get(k):
                raise ValueError(f"extrinsic {it['serial']}: {k} 없음")
        if np.linalg.norm(it["q"]) < 1e-12:
            raise ValueError(f"extrinsic {it['serial']}: 쿼터니언 길이 0")
    if not items:
        raise ValueError("extrinsics_by_serial 항목이 없음")
    return items


def validate_written(ci_text, ex_text, ci_entries, ex_entries):
    """쓴 파일을 노드 규칙 + PyYAML 로 다시 읽어 넣은 값과 같은지. 문제 목록 (비면 통과)."""
    problems = []
    for name, text in (("camera_info", ci_text), ("extrinsics", ex_text)):
        try:
            yaml.safe_load(text)
        except yaml.YAMLError as e:
            problems.append(f"{name}: YAML 오류 {e}")
    # 이번에 쓴 항목만이 아니라 다시 쓴 파일의 모든 시리얼 — 하나라도 노드가 못 읽으면 그 카메라 노드가 기동 중에 죽는다
    try:
        every = [str(k) for k in ((yaml.safe_load(ci_text) or {}).get("camera_info_by_serial") or {})]
    except yaml.YAMLError:
        every = []
    for serial in every:
        if serial in ci_entries:
            continue
        try:
            parse_camera_info_like_node(ci_text, serial)
        except ValueError as ex:
            problems.append(f"camera_info {serial} (기존 항목): {ex}")
    for serial, e in ci_entries.items():
        try:
            got = parse_camera_info_like_node(ci_text, serial)
        except ValueError as ex:
            problems.append(f"camera_info {serial}: {ex}")
            continue
        want = e["camera_info"]
        if got.get("distortion_model") != want["distortion_model"]:
            problems.append(f"camera_info {serial}: distortion_model {got.get('distortion_model')}")
        for k in ("d", "k", "p"):
            if not np.allclose(got.get(k, []), want[k], rtol=1e-12, atol=1e-15):
                problems.append(f"camera_info {serial}: {k} 가 다르게 읽힘")
    try:
        items = {it["serial"]: it for it in parse_extrinsics_like_node(ex_text)}
    except ValueError as ex:
        return problems + [f"extrinsics: {ex}"]
    for serial, e in ex_entries.items():
        it = items.get(serial)
        if not it:
            problems.append(f"extrinsics {serial}: 항목 없음")
            continue
        if it["parent_frame"] != e["parent_frame"] or it["child_frame"] != e["child_frame"]:
            problems.append(f"extrinsics {serial}: 프레임 {it['parent_frame']}→{it['child_frame']}")
        if not np.allclose(it["t"], e["translation_xyz_m"], atol=1e-12) or \
                not np.allclose(it["q"], e["rotation_xyzw"], atol=1e-12):
            problems.append(f"extrinsics {serial}: 값이 다르게 읽힘")
    return problems


# ---------------------------------------------------------------- 적용 · 되돌리기
def sha256(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def _atomic_write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp_dmcalib")
    tmp.write_text(text, encoding="utf-8")
    try:
        os.chmod(tmp, os.stat(path).st_mode & 0o777)
    except OSError:
        pass
    os.replace(tmp, path)


def apply_result(result_dir, rows, camera_info_path, extrinsics_path, archive_root, source_label="",
                 tool_commit=""):
    """선택된 행을 두 파일에 쓴다. 백업 → 쓰기 → 검증(실패하면 자동 되돌리기) → 기록.

    반환 manifest (archive_root/<시각>/manifest.json 에도 저장).
    """
    result = load_result_cameras(result_dir)
    sel = [r for r in rows if r.get("selected")]
    if not sel:
        raise ValueError("적용할 카메라가 없습니다")
    for r in sel:
        if not r.get("ok"):
            raise ValueError(f"{r['camera']}: {', '.join(r['issues'])}")
    ts = time.strftime("%Y%m%d_%H%M%S")
    arch = Path(archive_root) / ts
    n = 1
    while arch.exists():
        n += 1
        arch = Path(archive_root) / f"{ts}_{n}"
    (arch / "backup").mkdir(parents=True)
    meta = {"applied_at_utc": _stamp(),
            "source": f"{result_dir}" + (f" ({source_label})" if source_label else "")}
    ci_text_old, ci_old = _load_yaml_file(camera_info_path)
    ex_text_old, ex_old = _load_yaml_file(extrinsics_path)
    ci_entries, ex_entries = {}, {}
    for r in sel:
        cam = result[r["camera"]]
        ci_entries[r["serial"]] = camera_info_entry(cam, r, meta)
        ex_entries[r["serial"]] = extrinsic_entry(cam, r, meta)
    note = f"{APPLY_NOTE[2:]} {meta['applied_at_utc']} — 시리얼 {', '.join(sorted(ci_entries))}"
    ci_text = render_camera_info(ci_text_old, ci_old, ci_entries, note)
    ex_text = render_extrinsics(ex_text_old, ex_old, ex_entries, note)
    problems = validate_written(ci_text, ex_text, ci_entries, ex_entries)
    if problems:
        raise ValueError("쓰기 전 검증 실패: " + "; ".join(problems))

    files = []
    for path, text in ((camera_info_path, ci_text), (extrinsics_path, ex_text)):
        path = Path(path)
        rec = {"path": str(path.resolve() if path.exists() else path.absolute()), "existed": path.is_file(),
               "old_sha256": sha256(path), "backup": None, "backup_sidecar": None}
        if path.is_file():
            b = arch / "backup" / path.name
            shutil.copy2(path, b)
            side = path.with_name(f"{path.name}.bak_{ts}")
            shutil.copy2(path, side)
            rec.update(backup=str(b), backup_sidecar=str(side))
        files.append(rec)
    try:
        for rec, text in zip(files, (ci_text, ex_text)):
            _atomic_write(rec["path"], text)
            rec["new_sha256"] = sha256(rec["path"])
        problems = validate_written(Path(files[0]["path"]).read_text(encoding="utf-8"),
                                    Path(files[1]["path"]).read_text(encoding="utf-8"), ci_entries, ex_entries)
        if problems:
            raise ValueError("쓴 뒤 검증 실패: " + "; ".join(problems))
    except Exception:
        _restore(files)
        raise

    # warm-start 용 결과 사본 (도구의 --init 이 읽는 extrinsic/ + intrinsic/ 형식) + 보고서
    rd = arch / "result"
    for sub in ("extrinsic", "intrinsic", "camera_info"):
        src = Path(result_dir) / sub
        if src.is_dir():
            shutil.copytree(src, rd / sub)
    for f in ("report.md", "metrics.md", "summary.json", "rig.yaml"):
        if (Path(result_dir) / f).is_file():
            shutil.copy2(Path(result_dir) / f, rd / f)
    manifest = {
        "id": arch.name, "applied_at": time.time(), "applied_at_utc": meta["applied_at_utc"],
        "result_dir": str(result_dir), "tool_commit": tool_commit, "archive": str(arch),
        "files": files,
        "cameras": [{k: r[k] for k in ("camera", "serial", "serial_source", "name", "frame_id", "needs_confirm")}
                    for r in sel],
        "rolled_back": False,
    }
    (arch / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8")
    return manifest


def _restore(files):
    for rec in files:
        p = Path(rec["path"])
        if rec.get("backup") and Path(rec["backup"]).is_file():
            shutil.copy2(rec["backup"], p)
        elif not rec.get("existed") and p.exists():
            p.unlink()


def list_applied(archive_root):
    out = []
    for m in sorted(Path(archive_root).glob("*/manifest.json"), reverse=True):
        try:
            out.append(json.loads(m.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            pass
    return out


def rollback_check(manifest):
    """되돌리기 전에: 적용 뒤 파일이 또 바뀌었는지. 바뀐 파일 목록."""
    return [f["path"] for f in manifest["files"] if sha256(f["path"]) != f.get("new_sha256")]


def rollback(manifest, force=False):
    """manifest 의 백업으로 두 파일을 되돌린다. 적용 뒤 다른 사람이 고쳤으면 force 없이는 거부.

    되돌리기 직전 상태도 archive/<id>/before_rollback/ 에 남긴다.
    """
    changed = rollback_check(manifest)
    if changed and not force:
        raise RuntimeError("적용 뒤에 파일이 또 바뀌었습니다: " + ", ".join(changed))
    arch = Path(manifest["archive"])
    keep = arch / "before_rollback"
    keep.mkdir(parents=True, exist_ok=True)
    for f in manifest["files"]:
        if Path(f["path"]).is_file():
            shutil.copy2(f["path"], keep / Path(f["path"]).name)
    _restore(manifest["files"])
    for f in manifest["files"]:
        if f.get("old_sha256") and sha256(f["path"]) != f["old_sha256"]:
            raise RuntimeError(f"되돌린 파일이 백업과 다릅니다: {f['path']}")
    manifest["rolled_back"] = True
    manifest["rolled_back_at"] = time.time()
    (arch / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8")
    return manifest


# ---------------------------------------------------------------- warm-start 용: 차량 파일 → 도구 init 폴더
def export_vehicle_init(camera_info_path, extrinsics_path, serial_map, out_dir):
    """차량의 두 파일(시리얼 키) → nontarget_cal --init 이 읽는 폴더 (extrinsic/<이름>.yaml + intrinsic/<이름>.yaml).

    parent_frame 이 os_lidar 이고 camera_info 가 있는 시리얼만 (자리표시 · 리그 기준 항목은 뺀다).
    반환: {"cameras": [이름...], "skipped": {시리얼: 이유}}
    """
    _, ci = _load_yaml_file(camera_info_path)
    _, ex = _load_yaml_file(extrinsics_path)
    by_serial = {v["serial"]: k for k, v in serial_map.items() if v.get("serial")}
    reg_ci = {str(k): v for k, v in (ci.get("camera_info_by_serial") or {}).items()}
    out_dir = Path(out_dir)
    (out_dir / "extrinsic").mkdir(parents=True, exist_ok=True)
    (out_dir / "intrinsic").mkdir(parents=True, exist_ok=True)
    done, skipped = [], {}
    for serial, e in ((str(k), v) for k, v in (ex.get("extrinsics_by_serial") or {}).items()):
        name = by_serial.get(serial)
        if not name:
            skipped[serial] = "시리얼 표에 없음"
            continue
        if e.get("parent_frame", ex.get("reference_frame")) != PARENT_FRAME:
            skipped[serial] = f"parent_frame {e.get('parent_frame')} (os_lidar 기준 아님)"
            continue
        c = (reg_ci.get(serial) or {}).get("camera_info")
        if not c:
            skipped[serial] = "camera_info 없음"
            continue
        model = c.get("distortion_model")
        is_thermal = name.startswith("thermal")
        if not is_thermal and model != "equidistant":
            skipped[serial] = f"RGB 인데 {model} (도구는 equidistant 만 이어 받음)"
            continue
        T = T_cam_lidar_from_tf(e["translation_xyz_m"], e["rotation_xyzw"])
        k = c["k"]
        cal = e.get("calibration") or {}
        ext = {"camera": name, "parent_frame": PARENT_FRAME, "child_frame": name,
               "T_cam_lidar": T.tolist(), "source": f"vehicle serial {serial}"}
        if is_thermal:
            ext["time_offset_s"] = float(cal.get("time_offset_s", -0.1))
            ext["row_readout_s"] = float(cal.get("row_readout_s", -0.025))
        cinfo = (reg_ci.get(serial) or {}).get("calibration") or {}
        intr = {"camera": name, "model": model,
                "image_width": int(cinfo.get("image_width", 640 if is_thermal else 1920)),
                "image_height": int(cinfo.get("image_height", 480 if is_thermal else 1200)),
                "camera_matrix": [[k[0], 0.0, k[2]], [0.0, k[4], k[5]], [0.0, 0.0, 1.0]],
                "distortion_coefficients": [float(x) for x in c.get("d", [])]}
        (out_dir / "extrinsic" / f"{name}.yaml").write_text(yaml.safe_dump(ext, sort_keys=False), encoding="utf-8")
        (out_dir / "intrinsic" / f"{name}.yaml").write_text(yaml.safe_dump(intr, sort_keys=False), encoding="utf-8")
        done.append(name)
    return {"cameras": sorted(done), "skipped": skipped, "dir": str(out_dir)}


# ---------------------------------------------------------------- 차량 경로 찾기
def vehicle_paths(group, overrides=None, launch_defaults=None):
    """sensors.yaml 의 flir_cameras 센서군 → (camera_info 경로, extrinsics 경로, 설명).

    런치 인자(GUI 설정 overrides["launch_args"] > 런치 파일 기본값 > 알려진 기본값)를 작업 디렉터리(workdir)
    기준으로 푼다 — 노드가 CWD=workdir 에서 상대 경로로 여는 것과 같다.
    """
    overrides = overrides or {}
    la = dict(launch_defaults or {})
    la.update((overrides.get("launch_args") or {}))
    wd = Path(os.path.expanduser(str(group.get("workdir") or "~/FLIR_control")))
    ci = Path(os.path.expanduser(str(la.get("camera_info_yaml_path") or CAMERA_INFO_DEFAULT)))
    ex = Path(os.path.expanduser(str(la.get("extrinsics_yaml_path") or EXTRINSICS_DEFAULT)))
    ci = ci if ci.is_absolute() else wd / ci
    ex = ex if ex.is_absolute() else wd / ex
    return ci, ex, f"작업 디렉터리 {wd}"
