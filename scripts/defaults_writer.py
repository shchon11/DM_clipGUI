#!/usr/bin/env python3
# defaults_writer.py — GUI 에서 바꾼 값을 리포의 "원본" 에 써서 아예 기본값으로 만든다.
#
# 평소 GUI 는 원본을 건드리지 않고 사본(~/.config/dm_clip_gui/sensors/*.yaml)을 만들어 넘긴다.
# [기본값으로 저장] 을 누르면 그 차이를 원본에 옮긴다 — GUI 없이 ros2 launch 로 띄워도 같은 값이 되고,
# GUI 의 '원본과 다른 값' 표시도 0 이 된다.
#
#   params YAML   : 그 키의 줄만 고친다 (주석 · 순서 · 따옴표 모양 유지). 주석 처리된 키는 주석을 풀고,
#                   '지정 안 함' 으로 바꾼 키는 주석 처리하고, 원본에 없던 키는 끝에 모아 붙인다.
#   런치 파일      : DeclareLaunchArgument 의 default_value 문자열만 바꾼다 (ast 로 위치를 찾는다).
#   sensors.yaml  : 레지스트리에 default 가 있는 런치 옵션은 그 default 를 바꾼다 (GUI 는 그 값을 원본으로 본다).
#   인벤토리       : 카메라 이름 · 동기 역할 · IP 를 인벤토리 형식 그대로 다시 쓴다 (주석 없는 기계 생성 파일).
#
# 모든 쓰기는 (1) ~/.config/dm_clip_gui/backups 에 원본을 복사하고 (2) 새 내용을 다시 파싱해 값이 맞는지
# 확인한 뒤에만 파일을 바꾼다. 확인이 틀리면 아무것도 쓰지 않는다.

import ast
import datetime
import json
import math
import re
import shutil
from pathlib import Path

import yaml

from param_doc import split_inline_comment

BACKUP_DIR = Path.home() / ".config" / "dm_clip_gui" / "backups"


def yaml_scalar(value):
    """파이썬 값 -> YAML 표기. PyYAML(1.1)과 ROS(yaml-cpp, 1.2)가 둘 다 같은 타입으로 읽게."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if math.isnan(value):
            return ".nan"
        if math.isinf(value):
            return ".inf" if value > 0 else "-.inf"
        text = repr(value)
        if "e" in text and "." not in text:        # 1e-05 는 PyYAML 에게 문자열이다
            mantissa, exponent = text.split("e")
            text = f"{mantissa}.0e{exponent}"
        return text
    if isinstance(value, list):
        return "[" + ", ".join(yaml_scalar(v) for v in value) + "]"
    return json.dumps(str(value), ensure_ascii=False)


def launch_text(value):
    """런치 인자 기본값 문자열 (런치 인자는 전부 글자다)."""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _same(a, b):
    if isinstance(a, float) or isinstance(b, float):
        try:
            return math.isclose(float(a), float(b), rel_tol=1e-9, abs_tol=1e-12)
        except (TypeError, ValueError):
            return False
    return a == b


def _backup(path):
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    target = BACKUP_DIR / f"{path.name}.{stamp}"
    shutil.copy2(path, target)
    return target


# ---------- params YAML ----------

def write_params(path, changes):
    """changes = {키: 값 | None(=주석 처리)} → 원본 params YAML 을 줄 단위로 고친다. 백업 경로를 돌려준다."""
    path = Path(path).expanduser()
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    start = next(i for i, l in enumerate(lines) if l.strip() == "ros__parameters:")
    indent = next((l[:len(l) - len(l.lstrip())] for l in lines[start + 1:]
                   if l.strip() and not l.lstrip().startswith("#")), "    ")
    added = []
    for key, value in changes.items():
        active = re.compile(rf"^{re.escape(indent)}{re.escape(key)}\s*:(.*)$")
        commented = re.compile(rf"^\s*#\s*{re.escape(key)}\s*:(.*)$")
        index = next((i for i in range(start + 1, len(lines)) if active.match(lines[i].rstrip("\n"))), None)
        if index is not None:
            rest = active.match(lines[index].rstrip("\n")).group(1).strip()
            old_text, comment = split_inline_comment(rest)
            following = lines[index + 1] if index + 1 < len(lines) else ""
            if following.strip() and not following.lstrip().startswith("#") and \
                    len(following) - len(following.lstrip()) > len(indent):
                raise ValueError(f"{key}: 여러 줄에 걸친 값이라 자동으로 고치지 않습니다 — 직접 고쳐 주세요")
            tail = f"  # {comment}" if comment else ""
            lines[index] = (f"{indent}# {key}: {old_text}{tail}\n" if value is None
                            else f"{indent}{key}: {yaml_scalar(value)}{tail}\n")
            continue
        index = next((i for i in range(start + 1, len(lines)) if commented.match(lines[i].rstrip("\n"))), None)
        if index is not None:
            if value is None:
                continue
            _old, comment = split_inline_comment(commented.match(lines[index].rstrip("\n")).group(1).strip())
            lines[index] = f"{indent}{key}: {yaml_scalar(value)}" + (f"  # {comment}" if comment else "") + "\n"
            continue
        if value is not None:
            added.append((key, value))
    if added:
        if lines and not lines[-1].endswith("\n"):
            lines[-1] += "\n"
        lines.append(f"\n{indent}# DM_clipGUI [기본값으로 저장] 으로 추가 ({datetime.date.today()})\n")
        lines += [f"{indent}{key}: {yaml_scalar(value)}\n" for key, value in added]
    new_text = "".join(lines)

    doc = yaml.safe_load(new_text) or {}
    params = next((body["ros__parameters"] for body in doc.values()
                   if isinstance(body, dict) and "ros__parameters" in body), {})
    for key, value in changes.items():
        if value is None and key in params:
            raise ValueError(f"{key}: 주석 처리가 안 됐습니다")
        if value is not None and not _same(params.get(key), value):
            raise ValueError(f"{key}: 다시 읽은 값 {params.get(key)!r} 이 {value!r} 과 다릅니다")
    backup = _backup(path)
    path.write_text(new_text, encoding="utf-8")
    return backup


# ---------- 런치 파일 ----------

def write_launch_defaults(path, changes):
    """changes = {인자: 값} → DeclareLaunchArgument(default_value="...") 문자열만 바꾼다."""
    path = Path(path).expanduser()
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    spots = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = node.func.id if isinstance(node.func, ast.Name) else getattr(node.func, "attr", "")
        if name != "DeclareLaunchArgument" or not node.args:
            continue
        key = node.args[0].value if isinstance(node.args[0], ast.Constant) else None
        default = next((k.value for k in node.keywords if k.arg == "default_value"), None)
        if key in changes:
            if not (isinstance(default, ast.Constant) and isinstance(default.value, str)):
                raise ValueError(f"{key}: 기본값이 글자 상수가 아니라(경로 계산 등) 자동으로 못 바꿉니다")
            spots[key] = default
    missing = set(changes) - set(spots)
    if missing:
        raise ValueError(f"런치 파일에 없는 인자: {', '.join(sorted(missing))}")
    # ast 의 열 위치는 UTF-8 바이트 기준 — 줄을 바이트로 다룬다. 뒤에서부터 바꿔야 앞 위치가 안 밀린다.
    lines = [l.encode("utf-8") for l in source.splitlines(keepends=True)]
    for key, node in sorted(spots.items(), key=lambda kv: (kv[1].lineno, kv[1].col_offset), reverse=True):
        if node.lineno != node.end_lineno:
            raise ValueError(f"{key}: 기본값 문자열이 여러 줄이라 자동으로 못 바꿉니다")
        row = lines[node.lineno - 1]
        literal = json.dumps(launch_text(changes[key]), ensure_ascii=False).encode("utf-8")
        lines[node.lineno - 1] = row[:node.col_offset] + literal + row[node.end_col_offset:]
    new_source = b"".join(lines).decode("utf-8")

    import param_doc
    tmp = path.with_name(f".{path.name}.check")
    tmp.write_text(new_source, encoding="utf-8")
    try:
        found = {d["key"]: d["text"] for d in param_doc.parse_launch_args(tmp)}
    finally:
        tmp.unlink(missing_ok=True)
    for key, value in changes.items():
        if found.get(key) != launch_text(value):
            raise ValueError(f"{key}: 다시 읽은 기본값 {found.get(key)!r} 이 {launch_text(value)!r} 과 다릅니다")
    backup = _backup(path)
    path.write_text(new_source, encoding="utf-8")
    return backup


# ---------- 레지스트리 (sensors.yaml) ----------

def write_registry_defaults(path, group_key, changes):
    """sensors.yaml 에서 그 센서군의 런치 옵션 필드 default 를 바꾼다 (없으면 key 줄 다음에 넣는다)."""
    path = Path(path).expanduser()
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    group_at = next(i for i, l in enumerate(lines) if re.match(rf"^\s*- key:\s*{re.escape(group_key)}\s*$", l))
    group_end = next((i for i in range(group_at + 1, len(lines))
                      if re.match(r"^\s*- key:\s*\S+\s*$", lines[i])
                      and len(lines[i]) - len(lines[i].lstrip()) <= len(lines[group_at]) - len(lines[group_at].lstrip())),
                     len(lines))
    for key, value in changes.items():
        key_at = next(i for i in range(group_at, group_end)
                      if re.match(rf"^\s*key:\s*[\"']?{re.escape(key)}[\"']?\s*$", lines[i]))
        indent = lines[key_at][:len(lines[key_at]) - len(lines[key_at].lstrip())]
        block_end = next((i for i in range(key_at + 1, group_end)
                          if lines[i].strip() and len(lines[i]) - len(lines[i].lstrip()) < len(indent)), group_end)
        default_at = next((i for i in range(key_at + 1, block_end)
                           if re.match(rf"^{re.escape(indent)}default:", lines[i])), None)
        text = f"{indent}default: {yaml_scalar(value)}\n"
        if default_at is not None:
            lines[default_at] = text
        else:
            lines.insert(key_at + 1, text)
            group_end += 1
    new_text = "".join(lines)
    registry = yaml.safe_load(new_text)
    group = next(g for g in registry["groups"] if g["key"] == group_key)
    for key, value in changes.items():
        field = next(f for f in group.get("fields") or [] if f["key"] == key and f.get("target") == "launch_arg")
        if not _same(field.get("default"), value):
            raise ValueError(f"{key}: 다시 읽은 default {field.get('default')!r} 이 {value!r} 과 다릅니다")
    backup = _backup(path)
    path.write_text(new_text, encoding="utf-8")
    return backup


# ---------- 인벤토리 ----------

INVENTORY_ORDER = ("name", "serial", "namespace", "frame_id", "hardware_trigger_role", "ptp_action_role",
                   "force_ip_address", "force_ip_subnet_mask", "force_ip_gateway", "mac_address")


def write_inventory(path, node_key, entries):
    """entries = [{name, serial, ...}] → 인벤토리 형식 그대로 다시 쓴다 (C++ 인벤토리 툴과 같은 모양)."""
    path = Path(path).expanduser()
    out = [f"{node_key or 'flir_multicam'}:\n", "  ros__parameters:\n", "    cameras:\n"]
    for entry in entries:
        keys = [k for k in INVENTORY_ORDER if k in entry] + [k for k in entry if k not in INVENTORY_ORDER]
        for index, key in enumerate(keys):
            lead = "      - " if index == 0 else "        "
            out.append(f"{lead}{key}: {json.dumps(str(entry[key]), ensure_ascii=False)}\n")
    new_text = "".join(out)
    doc = yaml.safe_load(new_text)
    cams = doc[node_key or "flir_multicam"]["ros__parameters"]["cameras"]
    if [str(c.get("serial")) for c in cams] != [str(e.get("serial")) for e in entries]:
        raise ValueError("다시 읽은 인벤토리가 다릅니다")
    backup = _backup(path) if path.exists() else None
    path.write_text(new_text, encoding="utf-8")
    return backup
