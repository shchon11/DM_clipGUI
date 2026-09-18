#!/usr/bin/env python3
# sensor_config.py — 센서군 설정 폼의 값 <-> 실제 런치 인자/params 파일 변환.
#
# 원칙: 리포의 원본 params YAML은 절대 건드리지 않는다.
#   flir_camera.yaml 의 color_processing / buffer_handling_mode 주석처럼, 왜 그 값인지가
#   파일 안에 기록되어 있는 것들이 있다. 그래서 편집이 아니라 "사본 생성"으로 간다:
#
#   원본 YAML 전체  +  폼에서 바꾼 키만 덮어쓰기
#       -> ~/.config/dm_clip_gui/sensors/<group>[__<subset>].yaml
#       -> ros2 launch ... <params_arg>:=<생성 파일>
#
# 안 건드린 키는 원본 값 그대로 사본에 실린다. 그래서 GUI로 프레임레이트 하나만 바꿔도
# 나머지 200개 설정은 리포가 정한 그대로 간다.
#
# 단독 실행:  python3 sensor_config.py [group_key]   — 원본 대비 무엇이 바뀌는지 출력

import copy
import fnmatch
import re
import sys
from pathlib import Path

import yaml

import param_doc
from sensor_discovery import OK, expand, load_params, load_registry

# 생성 파일이 모이는 곳. clip_gui의 CONFIG_DIR 와 같은 뿌리를 쓴다.
GENERATED_DIR = Path.home() / ".config" / "dm_clip_gui" / "sensors"

# subset이 없는 센서군(라이다·GNSS)의 params를 담는 키
NO_SUBSET = "_"


# ---------- 레지스트리 조회 ----------

def group_by_key(registry, key):
    for group in registry.get("groups", []):
        if group.get("key") == key:
            return group
    return None


def subsets_of(group):
    return group.get("subsets") or []


def param_targets(group):
    """이 센서군이 만들어야 할 params 파일 목록.

    [{subset, label, source, params_arg, node_key}] — 카메라군은 가시광/열화상 2개,
    나머지는 1개.
    """
    targets = []
    for sub in subsets_of(group):
        if sub.get("params"):
            node_key, _ = load_params(sub["params"])
            targets.append({"subset": sub["key"], "label": sub["label"],
                            "source": sub["params"], "params_arg": sub.get("params_arg"),
                            "node_key": node_key})
    if group.get("params"):
        node_key, _ = load_params(group["params"])
        targets.append({"subset": NO_SUBSET, "label": group.get("label", ""),
                        "source": group["params"], "params_arg": group.get("params_arg"),
                        "node_key": node_key})
    return targets


def fields_for(group, subset):
    """그 subset(또는 launch_arg)에 속한 폼 필드만."""
    out = []
    for field in group.get("fields") or []:
        if field.get("target") == "launch_arg":
            if subset == "launch":
                out.append(field)
            continue
        if (field.get("subset") or NO_SUBSET) == subset:
            out.append(field)
    return out


# ---------- 원본 파일에서 읽는 '전체 설정' ----------
#
# 레지스트리의 fields 는 맨 위 '주요 설정' 뿐이다. 나머지는 원본 params YAML 과 런치 파일을
# param_doc 으로 직접 읽어 전부 보여준다 — 리포 파일에 키가 늘면 GUI 도 그대로 늘어난다.

ROS_UNDERLAY_SHARE = Path("/opt/ros/humble/share")


def launch_source(group):
    """이 센서군이 띄우는 런치 파일의 실제 경로 (기본값·설명을 읽으려고). 못 찾으면 None.

    GUI 프로세스는 그 워크스페이스를 소싱하지 않았을 수 있어서 ament 인덱스만 믿지 않고,
    레지스트리의 workspaces(…/install/setup.bash) 옆 install 트리에서 먼저 찾는다.
    """
    package, name = group.get("launch_package"), group.get("launch_file")
    if not package or not name:
        return None
    roots = []
    for workspace in group.get("workspaces") or []:
        path = expand(workspace)
        if path:
            roots.append(path.parent)
    candidates = []
    for root in roots:
        candidates += [root / package / "share" / package / "launch" / name,
                       root / "share" / package / "launch" / name]
    candidates.append(ROS_UNDERLAY_SHARE / package / "launch" / name)
    for cand in candidates:
        if cand.is_file():
            return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = Path(get_package_share_directory(package)) / "launch" / name
        return cand if cand.is_file() else None
    except Exception:                                             # noqa: BLE001
        return None


_LAUNCH_DOCS = {}          # 경로 -> (mtime, 결과) — 설정 칸을 바꿀 때마다 다시 파싱하지 않게


def launch_arg_docs(group):
    """[{key, value, text, help, type, choices, computed}] — 런치 파일의 모든 인자."""
    source = launch_source(group)
    if not source:
        return []
    try:
        mtime = source.stat().st_mtime
    except OSError:
        return []
    cached = _LAUNCH_DOCS.get(str(source))
    if not cached or cached[0] != mtime:
        cached = (mtime, param_doc.parse_launch_args(source))
        _LAUNCH_DOCS[str(source)] = cached
    return cached[1]


def launch_defaults(group):
    """{인자: 런치 파일 기본값(편집용 타입)}"""
    return {doc["key"]: doc["value"] for doc in launch_arg_docs(group)}


def _gui_launch_args(group):
    """GUI 가 기동할 때마다 직접 넘기는 런치 인자 — 사용자가 따로 정할 수 없다."""
    out = {}
    for sub in subsets_of(group):
        if sub.get("enable_arg"):
            out[sub["enable_arg"]] = f"'{sub['label']}' 카드의 체크로 정합니다"
        if sub.get("params_arg"):
            out[sub["params_arg"]] = "GUI 가 만든 params 사본이 들어갑니다 (이 탭의 설정)"
        if sub.get("inventory_arg"):
            out[sub["inventory_arg"]] = "GUI 가 지금 보이는 카메라만 담은 인벤토리 사본을 넘깁니다"
    if group.get("params_arg"):
        out[group["params_arg"]] = "GUI 가 만든 params 사본이 들어갑니다 (이 탭의 설정)"
    for key in group.get("inventory_launch_args") or {}:
        out[key] = "GUI 가 인벤토리 사본을 넘길 때 끕니다 (감지·이름·동기 역할은 GUI 가 정함)"
    return out


def managed_reason(group, scope, key):
    """이 키를 GUI 설정으로 바꿀 수 없는 이유. 바꿀 수 있으면 None."""
    if scope == "launch":
        table = dict(group.get("managed_launch_args") or {})
        fixed = _gui_launch_args(group)
        if key in fixed:
            return fixed[key]
    else:
        table = dict(group.get("managed_params") or {})
        sub = next((x for x in subsets_of(group) if x["key"] == scope), None)
        table.update((sub or {}).get("managed_params") or {})
    for pattern, reason in table.items():
        if fnmatch.fnmatchcase(key, pattern):
            return reason
    return None


def _editor_type(value_type, choices):
    if value_type == "bool":
        return "bool"
    if choices:
        return "enum"
    return {"int": "int_text", "float": "float_text", "list": "list"}.get(value_type, "string")


def full_fields(group, scope):
    """'전체 설정' 에 올릴 필드들 — [(섹션 제목, [field])]. 주요 설정에 이미 있는 키는 뺀다.

    scope: subset 키 / NO_SUBSET / "launch"
    """
    curated = {f["key"] for f in fields_for(group, scope)}
    if scope == "launch":
        fields = []
        for doc in launch_arg_docs(group):
            if doc["key"] in curated:
                continue
            fields.append({
                "key": doc["key"], "label": doc["key"], "target": "launch_arg", "generic": True,
                "type": _editor_type(doc["type"], doc["choices"]), "value_type": doc["type"],
                "choices": doc["choices"], "editable_choices": False,
                "default": doc["value"], "help": doc["help"],
                "managed": managed_reason(group, "launch", doc["key"]),
            })
        source = launch_source(group)
        return [(f"런치 인자 — {source.name if source else group.get('launch_file', '')}", fields)] \
            if fields else []

    target = next((t for t in param_targets(group) if t["subset"] == scope), None)
    doc = param_doc.parse_params(expand(target["source"])) if target else None
    if not doc:
        return []
    by_key = {e["key"]: e for e in doc["entries"]}
    out = []
    for title, keys in doc["sections"]:
        fields = []
        for key in keys:
            if key in curated:
                continue
            entry = by_key[key]
            fields.append({
                "key": key, "label": key, "subset": scope, "generic": True,
                "type": _editor_type(entry["type"], entry["choices"]), "value_type": entry["type"],
                "choices": entry["choices"], "editable_choices": True,
                "optional": entry["commented"],
                "example": entry["value"] if entry["commented"] else None,
                "help": entry["help"], "managed": managed_reason(group, scope, key),
            })
        if fields:
            out.append((title, fields))
    return out


def known_keys(group, scope):
    """주요 설정 + 원본 파일에 (주석으로라도) 있는 키 — 이 밖의 키는 '직접 추가' 한 것."""
    keys = {f["key"] for f in fields_for(group, scope)}
    for _title, fields in full_fields(group, scope):
        keys.update(f["key"] for f in fields)
    return keys


def custom_field(scope, key, value):
    """'직접 추가' 한 키의 필드 (값 타입은 넣은 값에서)."""
    kind = param_doc.value_type(value)
    return {"key": key, "label": key, "subset": scope, "generic": True, "custom": True,
            "type": _editor_type(kind, None), "value_type": kind, "choices": None,
            "help": "직접 추가한 키 — 원본 YAML 에 없는 키입니다. 카메라 노드는 camera.* / stream.* / "
                    "tl_device.* 를 GenICam 노드 이름으로 그대로 씁니다."}


# ---------- 값 해석 ----------

def base_values(group):
    """{subset: {키: 원본 YAML 값}} — 폼의 '기본값'이자 '안 바꾼 것'의 기준."""
    out = {}
    for target in param_targets(group):
        _, params = load_params(target["source"])
        out[target["subset"]] = params
    return out


def base_value(group, field, base=None):
    """필드 하나의 원본 값. YAML에 없으면 레지스트리의 default, 그것도 없으면 None."""
    base = base if base is not None else base_values(group)
    subset = field.get("subset") or NO_SUBSET
    if field.get("target") == "launch_arg":
        # 레지스트리에 default 가 없으면 런치 파일의 기본값 (= 인자를 안 넘겼을 때 실제로 쓰이는 값)
        if "default" in field:
            return field["default"]
        return launch_defaults(group).get(field["key"])
    value = (base.get(subset) or {}).get(field["key"])
    return field.get("default") if value is None else value


def effective_value(group, field, overrides, base=None):
    """폼이 보여줄 현재 값 = 사용자가 바꾼 값이 있으면 그것, 없으면 원본 값."""
    subset = "launch" if field.get("target") == "launch_arg" else (field.get("subset") or NO_SUBSET)
    store = _store_for(overrides, subset)
    if field["key"] in store:
        return store[field["key"]]
    return base_value(group, field, base)


def _store_for(overrides, subset):
    if subset == "launch":
        return (overrides or {}).get("launch_args") or {}
    return ((overrides or {}).get("params") or {}).get(subset) or {}


def changed_keys(group, overrides, base=None):
    """원본과 실제로 다른 키만. 폼에서 굵게 표시하는 데 쓴다."""
    base = base if base is not None else base_values(group)
    out = []
    for field in group.get("fields") or []:
        current = effective_value(group, field, overrides, base)
        if current != base_value(group, field, base):
            out.append(field["key"])
    return out


# ---------- 생성 ----------

def _merged_params(group, target, overrides):
    """원본 params + 그 subset의 오버라이드 (+ implies). 원본을 못 읽으면 None."""
    _, params = load_params(target["source"])
    if not params:
        return None

    merged = dict(params)
    store = _store_for(overrides, target["subset"])
    by_key = {f["key"]: f for f in fields_for(group, target["subset"])}
    for key, value in store.items():
        if managed_reason(group, target["subset"], key):
            continue                   # 런치가 카메라마다 덮어쓰는 키 — 적어 봐야 소용없다
        field = by_key.get(key, {})
        if value is None:
            # optional 필드를 '지정 안 함'으로 되돌린 경우 — 키 자체를 뺀다.
            merged.pop(key, None)
            continue
        merged[key] = value
        # 값만 써서는 안 먹는 키가 있다 (AcquisitionFrameRate 는 Enable 이 켜져야 한다).
        for implied_key, implied_value in (field.get("implies") or {}).items():
            merged[implied_key] = implied_value

    # 조건이 안 맞는 수동 값은 빼야 한다. 폼에서 회색 처리만 하고 파일에 남겨 두면, 예를 들어
    # ExposureAuto=Once/Continuous 인데 ExposureTime 이 원본 YAML 에서 딸려 와 카메라 노드가
    # "node is not writable in the current camera state" 로 죽는다 (카메라 12대가 실제로 그랬다).
    for field in fields_for(group, target["subset"]):
        condition = field.get("enabled_when") or {}
        if condition and any(merged.get(k) != v for k, v in condition.items()):
            merged.pop(field["key"], None)
    return merged


def build_params_files(group, overrides):
    """target마다 생성 파일을 쓰고 {params_arg: 경로} 반환.

    아무것도 안 바꿨어도 만든다. 런치의 기본 params_file 이 우리가 원본으로 삼은 파일과
    같다는 보장이 없기 때문이다 — ouster_ros driver.launch.py 의 기본값은 벤더 패키지의
    params 이지 리그의 lidar_driver_params.yaml 이 아니다. 인자를 생략하면 GUI에 표시된
    값과 실제로 적용되는 값이 조용히 달라진다.

    매 기동마다 원본을 다시 읽어 만들므로, 리포의 원본이 바뀌면 그대로 따라간다.
    """
    out = {}
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    for target in param_targets(group):
        merged = _merged_params(group, target, overrides)
        if merged is None or not target.get("params_arg"):
            continue
        name = group["key"] if target["subset"] == NO_SUBSET else f"{group['key']}__{target['subset']}"
        path = GENERATED_DIR / f"{name}.yaml"
        node_key = target["node_key"] or "/**"
        header = (f"# {path.name} — DM_clipGUI가 생성한 파일. 직접 고치지 마세요.\n"
                  f"# 원본: {target['source']}\n"
                  f"# GUI에서 바꾼 키만 덮어쓴 사본입니다.\n")
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(header)
            yaml.safe_dump({node_key: {"ros__parameters": merged}}, stream,
                           allow_unicode=True, sort_keys=False, default_flow_style=False)
        out[target["params_arg"]] = str(path)
    return out


def _arg_text(value):
    """ros2 launch 인자 값 표기. bool은 반드시 소문자 true/false여야 한다."""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


# ---------- 카메라별 설정 (이름 · 동기 역할) + 이번 기동용 인벤토리 ----------
#
# 저장된 인벤토리(리포 YAML)를 그대로 런치에 주면:
#   - 꺼져 있는 카메라까지 노드가 떠서 "camera_serial was not found" 로 죽고
#     (camera_start_stagger 만큼씩 기동도 늦어진다)
#   - 안 보이는 카메라가 PTP sender 면, 보이는 receiver 는 오지 않을 트리거를 기다리며
#     NEW_BUFFER_DATA 타임아웃만 반복한다 — 프레임 0장.
# 그래서 기동할 때마다 "지금 보이는 카메라만" 담은 사본을 만들어 넘긴다. 이름/역할의 우선순위는
#   GUI 에서 정한 값 > 리포 인벤토리 > 자동 이름(<prefix>_<serial>).
# 리포 인벤토리는 읽기만 한다. 런치의 auto_update 도 사본에 쓰게 된다.

ROS_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
SENDER_ROLES = {"sender", "send", "master"}      # 인벤토리 툴의 EnsurePtpSender 와 같은 기준
PTP_ROLES = ["sender", "receiver", "none"]


def _role(entry, key):
    return str(entry.get(key) or "none").strip().lower()


def camera_overrides(overrides):
    """cfg["sensors"][group]["cameras"] — {serial: {name?, ptp_action_role?}}"""
    return (overrides or {}).setdefault("cameras", {})


def repo_inventory(sub):
    """리포 인벤토리 → {serial: entry}"""
    _, params = load_params(sub.get("inventory"))
    out = {}
    for cam in params.get("cameras") or []:
        if isinstance(cam, dict) and cam.get("serial"):
            out[str(cam["serial"])] = {k: str(v) for k, v in cam.items()}
    return out


def camera_entry(sub, serial, overrides, repo=None):
    """한 카메라의 실효 인벤토리 항목 (리포 + GUI 설정 반영)."""
    repo = repo if repo is not None else repo_inventory(sub)
    entry = dict(repo.get(serial) or {})
    if not entry:
        name = f"{sub.get('name_prefix', 'camera')}_{serial}"
        entry = {"name": name, "serial": serial, "namespace": name,
                 "frame_id": f"{name}_optical_frame"}
        if sub.get("sync_roles"):
            entry.update({"hardware_trigger_role": "none", "ptp_action_role": "receiver"})
    mine = camera_overrides(overrides).get(serial) or {}
    if mine.get("name"):
        entry["name"] = entry["namespace"] = mine["name"]
        entry["frame_id"] = f"{mine['name']}_optical_frame"
    if sub.get("sync_roles") and mine.get("ptp_action_role"):
        entry["ptp_action_role"] = mine["ptp_action_role"]
    if mine.get("force_ip_address"):
        # GUI 에서 [IP 할당] 으로 준 주소 — 인벤토리 사본에도 실어서 다음부터 같은 주소를 쓴다
        entry["force_ip_address"] = mine["force_ip_address"]
        entry.setdefault("force_ip_subnet_mask", mine.get("force_ip_subnet_mask", "255.255.255.0"))
        entry.setdefault("force_ip_gateway", "0.0.0.0")
    entry.setdefault("namespace", entry.get("name", ""))
    return entry


def default_camera_entry(sub, serial):
    """GUI 설정을 빼고 본 항목 — '원본' 비교/되돌리기용."""
    return camera_entry(sub, serial, {})


def validate_camera_name(group, overrides, serial, name):
    """ROS 네임스페이스로 쓸 수 있고, 같은 센서군 안에서 겹치지 않는가. 문제 없으면 None."""
    if not ROS_NAME.match(name or ""):
        return "영문자로 시작하고 영문·숫자·밑줄만 쓸 수 있습니다"
    for sub in subsets_of(group):
        repo = repo_inventory(sub)
        serials = set(repo) | set(camera_overrides(overrides))
        for other in serials:
            if other == serial:
                continue
            if camera_entry(sub, other, overrides, repo).get("namespace") == name:
                return f"이미 {other} 가 쓰고 있는 이름입니다"
    return None


def _ip_int(ip):
    import socket
    import struct
    return struct.unpack(">I", socket.inet_aton(ip))[0]


def _ip_str(value):
    import socket
    import struct
    return socket.inet_ntoa(struct.pack(">I", value))


def pick_force_ip(group, overrides, sub, serial, devices, host_ip, prefix_len):
    """이 카메라에 줄 IP.

    인벤토리(또는 GUI)에 이미 정해 둔 주소가 있고 다른 장비가 안 쓰고 있으면 그 주소.
    아니면 subset 의 force_ip_base 부터 빈 주소 — 이때 두 인벤토리에 예약된 주소는 지금 안 꽂힌
    카메라 것이라도 피한다 (안 그러면 나중에 그 카메라를 꽂았을 때 충돌한다).
    """
    mask = (0xFFFFFFFF << (32 - prefix_len)) & 0xFFFFFFFF
    network = _ip_int(host_ip) & mask
    broadcast = network | (~mask & 0xFFFFFFFF)
    in_subnet = lambda ip: (_ip_int(ip) & mask) == network                 # noqa: E731

    others = {d["ip"] for d in devices or [] if d.get("identity") != serial}
    reserved = set()
    for other_sub in subsets_of(group):
        for other_serial, entry in repo_inventory(other_sub).items():
            if other_serial != serial and entry.get("force_ip_address"):
                reserved.add(entry["force_ip_address"])
    for other_serial, mine in camera_overrides(overrides).items():
        if other_serial != serial and mine.get("force_ip_address"):
            reserved.add(mine["force_ip_address"])

    wanted = camera_entry(sub, serial, overrides).get("force_ip_address")
    if wanted and in_subnet(wanted) and wanted not in others and wanted not in reserved:
        return wanted

    used = others | reserved | {host_ip}
    base = sub.get("force_ip_base")
    start = _ip_int(base) if base and in_subnet(base) else network + 1
    for value in list(range(start, broadcast)) + list(range(network + 1, start)):
        candidate = _ip_str(value)
        if candidate not in used:
            return candidate
    return None


def temperature_scale(group, overrides):
    """A70 IRFormat → mono16 한 칸이 몇 켈빈인가 (10mK → 0.01). 온도 포맷이 아니면 None."""
    for field in group.get("fields") or []:
        if field.get("key") == "camera.IRFormat":
            value = effective_value(group, field, overrides)
            return {"TemperatureLinear10mK": 0.01, "TemperatureLinear100mK": 0.1}.get(value)
    return None


def included_devices(group, overrides, devices):
    """{subset: [device]} — 이번 기동에 실제로 넣을 카메라 = 지금 열 수 있는 카메라만.

    IP 가 안 맞는 카메라는 기동 전에 GUI 가 맞춘다 (툴바의 IP 자동 맞춤). 업데이터 모드 카메라는
    전원을 다시 넣기 전에는 못 연다.
    """
    return {sub["key"]: [d for d in devices or [] if d.get("subset") == sub["key"] and d["state"] == "ok"]
            for sub in subsets_of(group)}


def _fix_sync_roles(entries, repo, label, notes):
    """보내는 쪽이 안 보이면 받는 쪽은 프레임이 0장이다 — 이번 기동에서만 바로잡는다."""
    ptp = [e for e in entries if _role(e, "ptp_action_role") != "none"]
    if ptp and not any(_role(e, "ptp_action_role") in SENDER_ROLES for e in ptp):
        gone = [e.get("namespace") or e.get("name") for e in repo.values()
                if _role(e, "ptp_action_role") in SENDER_ROLES]
        ptp[0]["ptp_action_role"] = "sender"
        notes.append(f"{label}: PTP sender" + (f"({', '.join(gone)})" if gone else "") +
                     f" 가 안 보여 {ptp[0]['namespace']} 를 이번 기동의 sender 로 씁니다")
    senders = [e for e in ptp if _role(e, "ptp_action_role") in SENDER_ROLES]
    for e in senders[1:]:
        e["ptp_action_role"] = "receiver"
        notes.append(f"{label}: sender 는 하나여야 해서 {e['namespace']} 는 receiver 로 띄웁니다")

    # GPIO 트리거는 배선이 필요한 역할이라 master 로 올리지 않는다 — slave 를 자유 실행으로.
    slaves = [e for e in entries if _role(e, "hardware_trigger_role") == "slave"]
    if slaves and not any(_role(e, "hardware_trigger_role") == "master" for e in entries):
        for e in slaves:
            e["hardware_trigger_role"] = "none"
        notes.append(f"{label}: GPIO 트리거 master 가 안 보여 slave {len(slaves)}대를 "
                     "자유 실행으로 띄웁니다 (프레임 동기 안 맞음)")


def _yaml_quote(value):
    return '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'


def _write_inventory(path, node_key, source, entries):
    """리포 인벤토리와 같은 모양으로 쓴다 — 런치와 C++ 인벤토리 툴이 둘 다 읽는다."""
    lines = [f"# {path.name} — DM_clipGUI 가 기동할 때마다 새로 만드는 사본. 직접 고치지 마세요.",
             f"# 원본: {source}",
             "# 지금 보이는 카메라만 담았고, GUI 에서 정한 이름/동기 역할이 반영돼 있습니다.",
             f"{node_key or 'flir_multicam'}:", "  ros__parameters:", "    cameras:"]
    for entry in entries:
        for index, (key, value) in enumerate(entry.items()):
            lead = "      - " if index == 0 else "        "
            lines.append(f"{lead}{key}: {_yaml_quote(value)}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_inventory_files(group, overrides, devices, notes):
    """보이는 카메라만 담은 인벤토리 사본을 쓰고 ({런치 인자: 경로}, {subset: [항목]}) 반환."""
    out, entries_by_subset = {}, {}
    enabled = (overrides or {}).get("subsets") or {}
    included = included_devices(group, overrides, devices)
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    for sub in subsets_of(group):
        if not sub.get("inventory") or not sub.get("inventory_arg"):
            continue
        if not enabled.get(sub["key"], True) or not included.get(sub["key"]):
            continue
        repo = repo_inventory(sub)
        order = {serial: i for i, serial in enumerate(repo)}
        serials = sorted({d["identity"] for d in included[sub["key"]]},
                         key=lambda sn: (order.get(sn, len(order)), sn))
        entries = [camera_entry(sub, sn, overrides, repo) for sn in serials]

        new = [e["namespace"] for e in entries if e["serial"] not in repo]
        if new:
            notes.append(f"{sub['label']}: 인벤토리에 없는 카메라 {len(new)}대 → {', '.join(new)}")
        seen = {d["identity"]: d for d in devices or [] if d.get("subset") == sub["key"]}
        name = lambda sn, e: e.get("namespace") or e.get("name") or sn          # noqa: E731
        gone = [name(sn, e) for sn, e in repo.items() if sn not in serials and sn not in seen]
        broken = [f"{name(sn, repo.get(sn, {}))}({seen[sn]['note'].split(' — ')[0]})"
                  for sn in seen if sn not in serials]
        if gone:
            notes.append(f"{sub['label']}: 안 보이는 {len(gone)}대는 이번 기동에서 뺍니다 ({', '.join(gone)})")
        if broken:
            notes.append(f"{sub['label']}: 보이지만 못 여는 {len(broken)}대도 뺍니다 ({', '.join(broken)})")
        if sub.get("sync_roles"):
            _fix_sync_roles(entries, repo, sub["label"], notes)

        node_key, _ = load_params(sub["inventory"])
        path = GENERATED_DIR / f"{group['key']}__{sub['key']}__inventory.yaml"
        _write_inventory(path, node_key, sub["inventory"], entries)
        out[sub["inventory_arg"]] = str(path)
        entries_by_subset[sub["key"]] = entries
    return out, entries_by_subset


def _auto_sensor_host(group, overrides, devices, notes):
    """라이다를 찾은 주소가 설정(sensor_hostname)과 다르면 이번 기동에서만 찾은 주소로 붙는다.

    라이다를 다른 NIC 로 옮기거나 센서 IP 가 바뀌어도 GUI 에서 주소를 고치지 않고 뜨게 하려는 것.
    GUI 설정에는 저장하지 않는다 (다음 감지에서 다시 정한다).
    """
    if (group.get("discovery") or {}).get("kind") != "ouster_probe" or not devices:
        return overrides
    found = [d["ip"] for d in devices if d.get("state") == OK and d.get("ip")]
    current = _store_for(overrides, NO_SUBSET).get("sensor_hostname")
    if current in (None, ""):
        current = (base_values(group).get(NO_SUBSET) or {}).get("sensor_hostname")
    if not found or current in found:
        return overrides
    if len(found) > 1:
        notes.append(f"라이다가 여러 대 보이는데({', '.join(found)}) 설정 주소 {current} 는 없어 그대로 둡니다")
        return overrides
    patched = copy.deepcopy(overrides or {})
    patched.setdefault("params", {}).setdefault(NO_SUBSET, {})["sensor_hostname"] = found[0]
    notes.append(f"라이다를 {found[0]} 에서 찾아 이번 기동은 그 주소로 붙습니다 (설정값 {current})")
    return patched


def plan_launch(group, overrides, devices=None, notes=None):
    """이번 기동 계획 -> (런치 인자 목록, 기대 네임스페이스 목록).

    devices 를 주면 보이는 카메라만 띄운다 (인벤토리 사본). 안 주면 저장된 설정 그대로 (CLI 확인용).
    """
    notes = notes if notes is not None else []
    overrides = _auto_sensor_host(group, overrides, devices, notes)
    enabled = dict((overrides or {}).get("subsets") or {})
    inventory_args, entries = {}, {}
    if devices is not None and any(s.get("inventory") for s in subsets_of(group)):
        included = included_devices(group, overrides, devices)
        for sub in subsets_of(group):
            if sub.get("inventory") and enabled.get(sub["key"], True) and not included.get(sub["key"]):
                enabled[sub["key"]] = False
                notes.append(f"{sub['label']}: 감지된 장비가 없어 이번 기동에서 뺍니다")
        inventory_args, entries = build_inventory_files(
            group, dict(overrides or {}, subsets=enabled), devices, notes)

    args = []
    for sub in subsets_of(group):
        if sub.get("enable_arg"):
            args.append(f"{sub['enable_arg']}:={_arg_text(enabled.get(sub['key'], True))}")

    curated = fields_for(group, "launch")
    values = {f["key"]: effective_value(group, f, overrides) for f in curated}
    defaults = launch_defaults(group)

    def add_arg(key, value, field=None):
        # ros2 launch 는 빈 값("key:=")을 malformed 로 거부한다. 레지스트리의 empty_value(예: "none")가
        # 있으면 그걸로, 런치 기본값도 비어 있으면 안 넘겨도 같고, 그 밖엔 넘길 방법이 없다.
        if value == "":
            if field and field.get("empty_value"):
                value = field["empty_value"]
            else:
                if defaults.get(key) not in (None, ""):
                    notes.append(f"런치 인자 {key} 를 비울 수 없어 런치 기본값({defaults[key]})이 쓰입니다")
                return
        args.append(f"{key}:={_arg_text(value)}")

    for field in curated:
        value = values[field["key"]]
        # 조건이 안 맞는 인자는 넘기지 않는다 (순차 기동이면 고정 간격은 의미가 없다)
        condition = field.get("enabled_when") or {}
        if value is None or any(values.get(k) != v for k, v in condition.items()):
            continue
        add_arg(field["key"], value, field)
    # '전체 설정' 에서 바꾼 나머지 런치 인자
    for key, value in ((overrides or {}).get("launch_args") or {}).items():
        if key in values or value is None or managed_reason(group, "launch", key):
            continue
        add_arg(key, value)

    for params_arg, path in build_params_files(group, overrides).items():
        args.append(f"{params_arg}:={path}")

    for inventory_arg, path in inventory_args.items():
        args.append(f"{inventory_arg}:={path}")
    if inventory_args:
        for key, value in (group.get("inventory_launch_args") or {}).items():
            args.append(f"{key}:={_arg_text(value)}")

    namespaces = [e["namespace"] for subset_entries in entries.values() for e in subset_entries]
    return args, namespaces


def launch_args(group, overrides, devices=None, notes=None):
    """['arg:=value', ...] — plan_launch 의 인자 부분만."""
    return plan_launch(group, overrides, devices, notes)[0]


def enabled_subsets(group, overrides):
    """실제로 띄우기로 한 subset 키들 (기동 완료 판정에 쓸 topic_regex를 고르는 데 필요)."""
    enabled = (overrides or {}).get("subsets") or {}
    subs = subsets_of(group)
    if not subs:
        return [NO_SUBSET]
    return [s["key"] for s in subs if enabled.get(s["key"], True)]


def topic_regexes(group, overrides):
    """이 센서군이 기동되면 떠야 하는 토픽 패턴들."""
    subs = subsets_of(group)
    if not subs:
        return [group["topic_regex"]] if group.get("topic_regex") else []
    active = set(enabled_subsets(group, overrides))
    return [s["topic_regex"] for s in subs
            if s.get("topic_regex") and s["key"] in active]


def missing_paths(group):
    """존재하지 않는 워크스페이스/params 경로 — 기동 버튼을 잠그는 근거."""
    missing = []
    for workspace in group.get("workspaces") or []:
        path = expand(workspace)
        if not path or not path.is_file():
            missing.append(str(workspace))
    for target in param_targets(group):
        path = expand(target["source"])
        if not path or not path.is_file():
            missing.append(str(target["source"]))
    return missing


# ---------- [기본값으로 저장] — GUI 의 차이를 원본에 옮긴다 (defaults_writer) ----------

def promote_plan(group, overrides, scopes):
    """[{"kind", "file", "items": [(키, 원본값, 새 값)]}] — 원본에 쓸 것.

    params 는 GUI 가 실제로 넘기는 사본(_merged_params)과 원본의 차이다. 그래서 implies(프레임레이트 →
    Enable)와 enabled_when(노출 auto 면 노출 시간 뺌)까지 원본에 그대로 반영된다 — GUI 없이 ros2 launch 로
    띄워도 같은 값이고 노드가 "not writable" 로 죽지 않는다.
    """
    plan = []
    for scope in scopes:
        if scope == "launch":
            curated = {f["key"]: f for f in fields_for(group, "launch")}
            defaults = launch_defaults(group)
            registry, launch = [], []
            for key, value in ((overrides or {}).get("launch_args") or {}).items():
                if value is None or managed_reason(group, "launch", key):
                    continue
                field = curated.get(key)
                if field and "default" in field:
                    registry.append((key, field["default"], value))
                else:
                    launch.append((key, defaults.get(key), value))
            if registry:
                plan.append({"kind": "registry", "file": str(_registry_file()), "items": registry})
            if launch and launch_source(group):
                plan.append({"kind": "launch", "file": str(launch_source(group)), "items": launch})
            continue
        target = next((t for t in param_targets(group) if t["subset"] == scope), None)
        if target is None:
            continue
        _, original = load_params(target["source"])
        merged = _merged_params(group, target, overrides) or {}
        items = [(k, original.get(k), v) for k, v in merged.items() if k not in original or original[k] != v]
        items += [(k, original[k], None) for k in original if k not in merged]
        if items:
            plan.append({"kind": "params", "file": str(expand(target["source"])), "scope": scope, "items": items})
    return plan


def _registry_file():
    from sensor_discovery import registry_path
    path = registry_path()
    return path.resolve() if path else None


def promote_apply(group, overrides, plan):
    """plan 을 파일에 쓰고, 옮긴 값은 GUI 오버라이드에서 뺀다. [(항목, 백업 경로 | 오류 글)] 반환."""
    import defaults_writer
    results = []
    for step in plan:
        changes = {key: new for key, _old, new in step["items"]}
        try:
            if step["kind"] == "params":
                backup = defaults_writer.write_params(step["file"], changes)
                store = _store_for(overrides, step["scope"])
                for key in list(store):
                    store.pop(key, None)
            elif step["kind"] == "launch":
                backup = defaults_writer.write_launch_defaults(step["file"], changes)
                for key in changes:
                    ((overrides or {}).get("launch_args") or {}).pop(key, None)
            else:
                backup = defaults_writer.write_registry_defaults(step["file"], group["key"], changes)
                for field in fields_for(group, "launch"):      # 메모리의 레지스트리도 맞춘다
                    if field["key"] in changes:
                        field["default"] = changes[field["key"]]
                for key in changes:
                    ((overrides or {}).get("launch_args") or {}).pop(key, None)
            results.append((step, backup, None))
        except Exception as exc:                                 # noqa: BLE001
            results.append((step, None, str(exc)))
    return results


def promote_inventory_plan(group, overrides, subset_key, devices):
    """(인벤토리 경로, 노드 키, 새 항목 목록, [(시리얼, 무엇이 바뀌나)]) — 카메라 이름 · 역할 · IP 를 인벤토리에."""
    sub = next(s for s in subsets_of(group) if s["key"] == subset_key)
    repo = repo_inventory(sub)
    mine = camera_overrides(overrides)
    seen = {d["identity"]: d for d in devices or [] if d.get("subset") == subset_key}
    entries, changes = [], []
    for serial in list(repo) + [sn for sn in mine if sn not in repo and sn in seen]:
        entry = camera_entry(sub, serial, overrides, repo)
        if serial not in repo:
            mac = (seen[serial].get("mac") or "").replace(":", "").upper()
            if mac:
                entry["mac_address"] = mac
            changes.append((serial, f"새 카메라 → {entry['namespace']}"))
        else:
            diff = [f"{k} {repo[serial].get(k, '-')} → {entry[k]}" for k in
                    ("namespace", "ptp_action_role", "force_ip_address")
                    if k in entry and str(repo[serial].get(k, "")) != str(entry[k])]
            if diff:
                changes.append((serial, " · ".join(diff)))
        entries.append(entry)
    node_key, _ = load_params(sub["inventory"])
    return str(expand(sub["inventory"])), node_key, entries, changes


def promote_inventory_apply(group, overrides, subset_key, devices):
    import defaults_writer
    path, node_key, entries, changes = promote_inventory_plan(group, overrides, subset_key, devices)
    backup = defaults_writer.write_inventory(path, node_key, entries)
    mine = camera_overrides(overrides)
    for serial, _ in changes:
        for key in ("name", "ptp_action_role", "force_ip_address", "force_ip_subnet_mask"):
            (mine.get(serial) or {}).pop(key, None)
        if not mine.get(serial):
            mine.pop(serial, None)
    return path, backup, changes


def main(argv):
    registry = load_registry()
    wanted = set(argv[1:])
    for group in registry.get("groups", []):
        if wanted and group["key"] not in wanted:
            continue
        print(f"\n=== {group['label']} ({group['key']})")
        gaps = missing_paths(group)
        if gaps:
            print("  없는 경로:", ", ".join(gaps))
        for target in param_targets(group):
            _, params = load_params(target["source"])
            print(f"  params[{target['subset']}] {target['params_arg']} "
                  f"<- {target['source']} (node={target['node_key']}, {len(params)}키)")
        print("  런치 인자:", " ".join(launch_args(group, {})) or "(없음)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
