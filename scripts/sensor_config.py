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

import genicam_doc
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


def matches(value, wanted):
    """enabled_when 의 값 하나. 목록이면 그중 하나면 된다 ({"pixel_format": ["RGB8Packed", "BGR8"]})."""
    return value in wanted if isinstance(wanted, list) else value == wanted


def condition_met(condition, value_of):
    """enabled_when / relevant_when 판정. value_of(키) -> 그 키의 현재 값.

    {키: 값, …} 는 키끼리 AND. [{…}, {…}] 처럼 목록이면 그중 하나만 맞으면 된다 (OR).
    """
    if isinstance(condition, list):
        return any(condition_met(c, value_of) for c in condition)
    return all(matches(value_of(key), wanted) for key, wanted in (condition or {}).items())


# 파라미터 접두사 -> 노드맵. 이 접두사 키는 노드가 기동 때 그 GenICam 노드에 그대로 쓴다.
GENICAM_PREFIXES = ("camera.", "stream.", "tl_device.")


def field_route(field):
    """값이 카메라에 닿는 경로 -> (종류, 설명). 종류: 'genicam' · 'node' · 'pc'.

    genicam : camera.X — 노드가 GenICam 노드 X 에 그대로 쓴다 (기동 때 한꺼번에, 순서는 타입별)
    node    : 노드 파라미터인데 노드가 자기 로직으로 센서에 쓴다 (레지스트리 writes, 대상은 writes_to —
              카메라는 GenICam 노드, 라이다는 드라이버가 센서 HTTP 설정으로 넣는 값)
    pc      : 센서에 가지 않는다 — PC 의 드라이버 노드가 쓰는 설정
    """
    key = field["key"]
    if field.get("target") == "launch_arg":
        return "pc", "런치 인자 — 노드를 어떻게 띄울지 정합니다 (센서에 가지 않습니다)"
    if key.startswith(GENICAM_PREFIXES):
        node = key.split(".", 1)[1]
        if field.get("alias_of"):
            return "genicam", f"GenICam {field['alias_of']} 의 레지스터 별칭 {node} 에 노드가 그대로 씁니다"
        return "genicam", f"GenICam 노드 {node} 에 노드가 그대로 씁니다"
    if field.get("writes"):
        target = field.get("writes_to", "GenICam")
        text = f"노드가 자기 로직으로 {target} " + " · ".join(field["writes"]) + " 를 씁니다"
        if target == "GenICam":
            text += " (camera.* 를 적용한 뒤라 같은 노드를 camera.* 로 줘도 이쪽이 이깁니다)"
        return "node", text
    return "pc", "센서에 가지 않습니다 — PC 의 드라이버 노드가 쓰는 설정"


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

    원본 params YAML 의 키 뒤에, 레지스트리에 genicam 이 있는 subset 은 카메라 XML 의 노드가 붙는다.
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
    return out + genicam_fields(group, scope, curated | set(by_key))


def _genicam_conf(group, scope):
    sub = next((x for x in subsets_of(group) if x["key"] == scope), None)
    return (sub or {}).get("genicam") or {}


def genicam_source(group, scope):
    """이 subset 의 카메라 GenICam XML 경로 (레지스트리 genicam.xml, glob 가능). 없으면 None."""
    conf = _genicam_conf(group, scope)
    return genicam_doc.resolve(conf["xml"]) if conf.get("xml") else None


def genicam_fields(group, scope, exclude):
    """카메라 XML 의 카테고리별 노드 — [(섹션 제목, [field])]. exclude(주요 설정 · 원본 YAML 에 있는 키)는 뺀다.

    원본 YAML 에 없는 키라 전부 '지정' 해야 실리는 선택 항목이다. 예시값이 없는 이유: 기본값은 XML 이 아니라
    카메라 레지스터에 있다 — 숫자 칸은 비워 두고 사용자가 적게 한다 (0 같은 가짜 값이 실리지 않게).
    """
    conf = _genicam_conf(group, scope)
    path = genicam_source(group, scope)
    doc = genicam_doc.load(path) if path else None
    if not doc:
        return []
    prefix = conf.get("prefix", "camera")
    out = []
    for title, _name, features in genicam_doc.category_sections(doc, conf.get("categories") or []):
        fields = []
        for feature in features:
            key = f"{prefix}.{feature['name']}"
            if key in exclude:
                continue
            rule = (conf.get("rules") or {}).get(feature["name"]) or {}
            fields.append({
                "key": key, "label": key, "subset": scope, "generic": True, "genicam": True,
                "type": _editor_type(feature["type"], feature["choices"]), "value_type": feature["type"],
                "choices": feature["choices"], "editable_choices": False,
                "optional": True, "example": None, "section": feature["section"] or None,
                "help": _genicam_help(feature, path, rule), "managed": managed_reason(group, scope, key),
                "enabled_when": rule.get("enabled_when"), "relevant_when": rule.get("relevant_when"),
            })
        if fields:
            out.append((f"GenICam · {title}", fields))
    return out


def _genicam_help(feature, path, rule=None):
    parts = [feature["help"]] if feature["help"] else []
    if (rule or {}).get("why"):
        parts.append(rule["why"])
    meta = f"GenICam 노드 {feature['name']}"
    if feature["display"] and feature["display"] != feature["name"]:
        meta += f" ({feature['display']})"
    meta += f" · {feature['visibility']}"
    if feature["unit"]:
        meta += f" · 단위 {feature['unit']}"
    parts.append(meta)
    if feature["selectors"]:
        parts.append(f"선택자 {', '.join(feature['selectors'])} 로 고른 한 칸에만 적용됩니다. 파라미터 파일에는 "
                     "한 값만 넣을 수 있고, 노드는 열거형(선택자)을 먼저 씁니다.")
    if feature.get("depends"):
        parts.append(f"잠김이 바뀌는 조건: {', '.join(feature['depends'])} — 이 노드들 값에 따라 쓸 수 있게 되거나 "
                     "잠깁니다 (XML 의 잠김 레지스터). 기동 때 잠겨 있으면 값이 무시되고, 적용 도중 잠기면 노드가 죽습니다.")
    parts.append(f"카메라 XML: {path.name}")
    return "\n".join(parts)


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
    for field in conditional_fields(group, target["subset"]):
        if not condition_met(field.get("enabled_when"), merged.get):
            merged.pop(field["key"], None)
    # 패킷 간 지연(GevSCPD)과 링크 대역폭 제한(DeviceLinkThroughputLimit)은 카메라 안에서 같은 값의 짝이다 — 하나를
    # 쓰면 카메라가 다른 하나를 다시 계산한다. 둘 다 넘기면 노드가 쓰는 순서에 따라 나중 것이 이긴다. 원본 YAML 의
    # GevSCPD: 0 이 75 MB/s 제한을 125 MB/s 로 되돌려(14대 중 13대) 동시 트리거에서 스위치가 패킷을 버렸다 (2026-09-22).
    if merged.get("camera.DeviceLinkThroughputLimit"):
        merged.pop("camera.GevSCPD", None)
    return merged


def conditional_fields(group, subset):
    """enabled_when 이 있는 필드 — 주요 설정 + 레지스트리 genicam.rules (전체 설정의 GenICam 칸).

    rules 는 XML 을 읽지 않고 레지스트리만으로 만든다 — 생성 파일을 만들 때마다 XML 을 파싱하지 않게.
    """
    out = [f for f in fields_for(group, subset) if f.get("enabled_when")]
    conf = _genicam_conf(group, subset)
    prefix = conf.get("prefix", "camera")
    for name, rule in (conf.get("rules") or {}).items():
        if rule.get("enabled_when"):
            out.append({"key": f"{prefix}.{name}", "enabled_when": rule["enabled_when"]})
    return out


def restart_only(group, scope, field):
    """실행 중에는 못 바꾸고 다시 기동해야 들어가는 camera.* 필드인가 (카메라 XML 의 잠김 조건, 또는 레지스트리
    restart_only). ISP 튜닝 탭이 이런 칸을 잠근다."""
    if field.get("restart_only") is not None:
        return bool(field["restart_only"])
    key = field["key"]
    if not key.startswith("camera."):
        return False
    path = genicam_source(group, scope)
    doc = genicam_doc.load(path) if path else None
    if not doc:
        return False
    name = (field.get("alias_of") or key.split(".", 1)[1]).split(" (")[0]
    return genicam_doc.locked_while_streaming(doc, name)


def genicam_dependencies(group, scope, field):
    """camera.* 필드를 잠그거나 풀 수 있는 공개 GenICam 기능들 (XML 이 없거나 camera.* 가 아니면 [])."""
    key = field["key"]
    if not key.startswith("camera."):
        return []
    path = genicam_source(group, scope)
    doc = genicam_doc.load(path) if path else None
    if not doc:
        return []
    name = (field.get("alias_of") or key.split(".", 1)[1]).split(" (")[0]
    return genicam_doc.dependencies(doc, name)


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


# ---------- 설정 점검 (기동 전) ----------
#
# 한 칸만 보면 멀쩡한데 다른 칸과 엮여서 문제가 되는 조합을 노드가 뜨기 전에 찾는다. 카드 머리와 런치 로그에 뜬다.
#   - 픽셀 포맷 × 해상도 × fps 가 카메라 링크 제한(DeviceLinkThroughputLimit)을 넘으면 카메라가 fps 를 깎는다.
#     자유 실행 프레임레이트를 그 위로 지정해 두면 카메라가 값을 거부해 노드가 기동 중에 죽는다 — RGB8Packed
#     1920×1200 에 30 fps 를 지정했더니 14대가 전부 "must be smaller than or equal 14.387029" 로 죽었다.
#   - 노출이 프레임 주기보다 길면 트리거를 건너뛴다. auto 노출 한계의 하한 > 상한이면 카메라가 거부한다.
#   - 노출 auto 는 카메라마다 따로 노출을 정해서 밝기와 노출 끝 타임스탬프가 카메라마다 달라진다.
#   - 비닝 뒤 Width/Height 가 최대를 넘으면 카메라가 거부하고, 해상도가 바뀌면 캘리브레이션이 안 맞는다.
#   - 카메라 NIC 의 MTU 가 GigE 패킷 크기보다 작으면 NIC 가 영상 패킷을 전부 버려 프레임이 한 장도 완성되지
#     않는다. `ip link set mtu 9000` 은 재부팅하면 풀린다 — 2026-09-21 재부팅 뒤 MTU 1500 인 채로 띄웠더니 14대가
#     전부 'Incomplete image status=3' 이었고 NIC 의 rx_long_length_errors 가 8700만이었다.

def bytes_per_pixel(pixel_format):
    """링크에 실리는 화소당 바이트. 모르는 포맷이면 None."""
    name = str(pixel_format or "").replace("Spinnaker::", "").replace("PixelFormat_", "")
    if name in ("RGB8", "RGB8Packed", "BGR8", "BGR8Packed", "YUV444Packed", "YCbCr8"):
        return 3.0
    if name in ("BGRa8", "RGBa8"):
        return 4.0
    if name in ("YUV422Packed", "YUV422_8", "YCbCr422_8"):
        return 2.0
    if name in ("YUV411Packed", "YCbCr411_8"):
        return 1.5
    if re.fullmatch(r"(Mono|Bayer[A-Z]{2}|Polarized)(8|10p|10Packed|12p|12Packed|16)", name):
        bits = re.search(r"(8|10p|10Packed|12p|12Packed|16)$", name).group(1)
        return {"8": 1.0, "10p": 1.25, "10Packed": 1.5, "12p": 1.5, "12Packed": 1.5, "16": 2.0}[bits]
    return None


def nic_mtus(devices):
    """{NIC: MTU} — 감지된 장비가 달린 NIC 들. 못 읽는 NIC 는 뺀다."""
    out = {}
    for nic in {d.get("nic") for d in devices or [] if d.get("nic")}:
        try:
            out[nic] = int((Path("/sys/class/net") / nic / "mtu").read_text())
        except (OSError, ValueError):
            pass
    return out


def check_settings(group, subset, overrides, n_cameras=0, mtus=None, synced=0, ptp_cams=0):
    """[(level, 글)] — level 은 'error'(노드가 기동 중에 죽거나 프레임이 안 나온다) · 'warn'. 점검할 수 없으면 [].

    mtus: {NIC: MTU} — 이 종류의 카메라가 달린 NIC (nic_mtus). 주면 GigE 패킷 크기와 비교한다.
    synced: 같은 트리거로 동시에 찍는 카메라 수 (HW 트리거 · PTP 액션). 동시에 프레임을 쏟아내서 순간 합을 본다.
    ptp_cams: 동기 방식이 PTP 액션(보내기 · 받기)인 카메라 수. 있으면 PC 가 PTP grandmaster 인지(ptp4l) 본다.

    생성 파일과 같은 값(_merged_params)으로 계산한다 — enabled_when 으로 빠지는 키까지 반영된다.
    레지스트리 subset 의 checks: {sensor: [폭, 높이], uplink_MBps} 가 있어야 점검한다.
    """
    sub = next((x for x in subsets_of(group) if x["key"] == subset), None)
    conf = (sub or {}).get("checks")
    target = next((t for t in param_targets(group) if t["subset"] == subset), None)
    if not conf or not target:
        return []
    merged = _merged_params(group, target, overrides)
    if not merged:
        return []
    errors, warns = [], []

    # --- 해상도 · 비닝 ---
    width, height = merged.get("camera.Width"), merged.get("camera.Height")
    sensor = conf.get("sensor") or []
    shrunk = False
    for axis, size, full in (("Horizontal", width, sensor[0] if sensor else None),
                             ("Vertical", height, sensor[1] if len(sensor) > 1 else None)):
        shrink = int(merged.get(f"camera.Binning{axis}") or 1) * int(merged.get(f"camera.Decimation{axis}") or 1)
        shrunk = shrunk or shrink > 1 or bool(full and size and size != full)
        if full and size and size * shrink > full:
            name = "Width" if axis == "Horizontal" else "Height"
            errors.append(f"camera.{name} {size} 이 비닝 · 데시메이션 {shrink} 배 뒤 최대({full // shrink})를 넘습니다 — "
                          f"카메라가 거부해 노드가 기동 중에 죽습니다. {name} 값을 {full // shrink} 이하로.")
    if shrunk and sensor:
        warns.append(f"해상도가 센서 전체({sensor[0]}×{sensor[1]})가 아닙니다 — camera_info 캘리브레이션은 전체 해상도 "
                     "기준이라 내부 파라미터(초점거리 · 주점)가 안 맞습니다.")

    # --- 프레임레이트 · 노출 ---
    fps = None
    if merged.get("camera.AcquisitionFrameRateEnable") and merged.get("camera.AcquisitionFrameRate"):
        fps = float(merged["camera.AcquisitionFrameRate"])
    grid = merged.get("timestamp.trigger_grid_hz") or 0
    rate, source = (grid, "트리거 격자") if grid > 0 else (merged.get("ptp_action.rate_hz"), "PTP 동기 프레임레이트")
    period_hz = fps or rate
    exposure_auto = str(merged.get("camera.ExposureAuto") or "")
    if period_hz:
        period = 1e6 / period_hz
        exposure, what = None, ""
        if exposure_auto == "Off" and merged.get("camera.ExposureTime"):
            exposure, what = float(merged["camera.ExposureTime"]), "노출 시간"
        elif exposure_auto in ("Once", "Continuous") and merged.get("camera.AutoExposureExposureTimeUpperLimit"):
            exposure, what = float(merged["camera.AutoExposureExposureTimeUpperLimit"]), "auto 노출 상한"
        if exposure and exposure >= period:
            warns.append(f"{what} {exposure:g} µs 가 프레임 주기 {period:.0f} µs ({period_hz:g} Hz) 이상입니다 — 노출이 "
                         "끝나기 전에 다음 트리거가 와서 트리거를 건너뜁니다 (fps 가 절반으로).")
    # 트리거로 찍는 카메라에 '자유 실행 프레임레이트' 상한이 켜져 있다. BFS 는 프레임 주기를 센서 줄 단위로 올려
    # 잡아서 30 Hz 를 주면 29.9952 Hz (33.3386 ms) 로 돈다 — 30 Hz 트리거보다 매 프레임 5.3 µs 느려 노출 시작이
    # 조금씩 밀리고, 밀린 게 여유(주기 − 노출)를 넘으면 트리거가 노출 중에 와서 한 번 버려진다. 2026-09-22 PTP 액션:
    # 노출 30 ms 면 20초마다, 25 ms 면 50초마다 카메라마다 한 장씩 안 찍었다 (frame_id 연속). 카메라가 전부 트리거로
    # 찍으면 기동할 때 상한을 끈다 (_drop_frame_rate_cap). 섞여 있으면 파일이 하나라 못 끈다 — 알린다.
    if synced and n_cameras > synced and merged.get("camera.AcquisitionFrameRateEnable") \
            and merged.get("camera.AcquisitionFrameRate"):
        warns.append(f"트리거로 찍는 카메라 {synced}대에도 '자유 실행 프레임레이트' 상한 "
                     f"{float(merged['camera.AcquisitionFrameRate']):g} Hz 가 걸립니다 — 카메라가 이 값을 센서 줄 단위로 "
                     "올려 잡아 (30 Hz → 29.995 Hz) 트리거보다 조금 느리고, 몇십 초마다 트리거를 한 번씩 놓칩니다. "
                     "자유 실행 카메라를 빼거나 동기 방식을 통일하면 기동할 때 상한을 알아서 끕니다. 아니면 상한을 "
                     "트리거보다 높게 (31 Hz 이상).")
    for name, unit in (("ExposureTime", "µs"), ("Gain", "dB")):
        low = merged.get(f"camera.AutoExposure{name}LowerLimit")
        high = merged.get(f"camera.AutoExposure{name}UpperLimit")
        if low is not None and high is not None and low > high:
            errors.append(f"auto {'노출' if name == 'ExposureTime' else '게인'} 하한 {low:g} {unit} 가 상한 {high:g} {unit} "
                          "보다 큽니다 — 카메라가 거부해 노드가 기동 중에 죽습니다.")
    if exposure_auto in ("Once", "Continuous") and n_cameras > 1:
        warns.append(f"노출 auto({exposure_auto})는 카메라마다 따로 노출을 정합니다 — 밝기가 카메라마다 다르고, 노출 "
                     "끝에 찍히는 카메라 타임스탬프도 노출 차이만큼 어긋납니다 (2026-09-21: 3대 1 ms · 나머지 18 ms). "
                     "동기 데이터면 Off 에 같은 노출 시간을.")
    # WB auto 도 카메라마다 자기 화면으로 정한다. Once 는 한 번 맞추면 Off 로 돌아가야 하는데, 어두운 장면에서는
    # 끝내지 못하고 Once 에 머문 채 비율이 치우친다 — 2026-09-21 밤 14대 중 4대가 녹색 · 보라로 틀어졌다
    # (앞쪽은 흰 가로등을 보고 끝남). 메타데이터의 balance_white_auto 가 계속 Once 면 그 상태다.
    wb_auto = str(merged.get("camera.BalanceWhiteAuto") or "")
    if wb_auto in ("Once", "Continuous") and n_cameras > 1:
        warns.append(f"화이트밸런스 auto({wb_auto})는 카메라마다 자기 화면으로 색을 맞춥니다 — 카메라끼리 색이 달라지고, "
                     "어두운 장면에서는 끝내지 못해 한쪽으로 치우칩니다 (2026-09-21 밤: Once 가 4대에서 안 끝나 녹색 · "
                     "보라 색조). 여러 카메라를 이어 붙일 데이터면 Off 에 R · B 비율을 모두 같게.")

    # --- PTP 액션인데 grandmaster(ptp4l)를 끄라고 적어 뒀다 ---
    # 카메라는 SlaveOnly 라 저희끼리 기준 시계를 못 뽑는다. 'PTP grandmaster NIC' 를 비우면 기동할 때 자동으로
    # 정한다 (ptp_master_nic). none 같은 값을 직접 적었으면 ptp4l 이 안 떠서, 따로 띄운 게 없으면 카메라가 PTP
    # Slave 가 못 되고 노드가 ptp.sync_timeout_ms 동안 기다리다 죽는다 (ptp.require_sync).
    if ptp_cams:
        field = next((f for f in fields_for(group, "launch") if f["key"] == "ptp_master_interface"), None)
        nic = str(effective_value(group, field, overrides) or "").strip() if field else ""
        if nic.lower() in PTP_MASTER_OFF:
            wait = float(merged.get("ptp.sync_timeout_ms") or 60000) / 1000
            fate = (f"노드가 {wait:g}초 기다리다 죽습니다" if merged.get("ptp.require_sync", True) else
                    "카메라끼리 시계가 안 맞은 채 찍습니다")
            warns.append(f"동기 방식이 PTP 액션인 카메라가 {ptp_cams}대인데 'PTP grandmaster NIC'(카메라 공통 옵션)가 "
                         f"'{nic}' 입니다 — ptp4l 을 따로 띄우지 않았다면 카메라가 PTP Slave 가 못 되고 {fate}. "
                         "비워 두면 기동할 때 카메라 NIC 로 알아서 정합니다.")

    # --- NIC MTU 대 GigE 패킷 크기 ---
    packet_size = merged.get("camera.GevSCPSPacketSize")
    for nic, mtu in sorted((mtus or {}).items()):
        if packet_size and mtu < int(packet_size):
            errors.append(f"카메라 NIC {nic} 의 MTU {mtu} 가 GigE 패킷 크기 {packet_size} 보다 작습니다 — NIC 가 영상 패킷을 "
                          "전부 버려 프레임이 한 장도 완성되지 않습니다 ('Incomplete image status=3'). ip link 로 올린 MTU 는 "
                          f"재부팅하면 풀립니다 — NIC 의 NetworkManager 프로파일에 MTU {packet_size} 을 저장해 올리거나, "
                          "GigE 패킷 크기를 1400 으로.")

    # --- 링크 대역폭 ---
    bpp = bytes_per_pixel(merged.get("pixel_format"))
    limit = merged.get("camera.DeviceLinkThroughputLimit")
    if bpp and width and height and limit:
        # GVSP 패킷마다 IP · UDP · GVSP 헤더(36 B)가 붙고 링크 제한은 이더넷 헤더(18 B)까지 센다.
        # 9000 B 패킷이면 0.6% — 카메라가 계산한 14.387 fps 와 맞는다 (단순 나눗셈은 14.47).
        packet = int(merged.get("camera.GevSCPSPacketSize") or 1400)
        frame = width * height * bpp * (packet + 18) / max(packet - 36, 1)
        max_fps = limit / frame
        what = f"{merged.get('pixel_format')} {width}×{height} · 링크 {limit / 1e6:g} MB/s 에서 카메라당 최대 약 {max_fps:.1f} fps"
        fix = "픽셀 포맷을 BayerRG8(화소당 1 B)로 · 비닝 2 (Width/Height 도 절반으로) · fps 를 낮추기 중 하나."
        if fps and fps > max_fps:
            errors.append(f"자유 실행 프레임레이트 {fps:g} fps 가 {what} 를 넘습니다 — 카메라가 값을 거부해 노드가 "
                          f"기동 중에 죽습니다. {fix}")
        elif rate and rate > max_fps:
            warns.append(f"리그 프레임레이트 {rate:g} Hz ({source})를 못 따라갑니다 — {what}. 트리거를 건너뛰어 "
                         f"프레임이 빠집니다. {fix}")
        uplink = conf.get("uplink_MBps")
        # 동시 트리거: 모든 카메라가 같은 순간 한 프레임을 링크 제한 속도로 보낸다 → 순간 합 = 대수 × 링크 제한.
        # 평균(대수 × fps × 프레임)이 업링크 안이어도 이 순간 합이 넘으면 스위치가 패킷을 버린다 — 영상이 깨지거나
        # 시작 패킷을 잃어 frame_id · 카메라 시각이 이전 프레임 것으로 실린다 (2026-09-21). DeviceLinkThroughputLimit
        # 설명의 '13~14대면 70~75 MB/s' 가 이 계산이다.
        if uplink and synced > 1 and synced * limit > uplink * 1e6:
            fair = uplink * 1e6 / synced
            send_ms = frame / fair * 1000
            period = f", 한 프레임 전송 {send_ms:.1f} ms" + (" — 프레임 간격보다 길어 fps 도 낮춰야 합니다"
                                                            if period_hz and send_ms > 1000 / period_hz else "")
            warns.append(f"동시 트리거 {synced}대가 같은 순간 링크 {limit / 1e6:g} MB/s 로 보내 순간 합 "
                         f"{synced * limit / 1e6:.0f} MB/s 가 스위치 업링크 {uplink:g} MB/s 를 넘습니다 — 스위치가 패킷을 "
                         f"버려 프레임이 깨지거나 메타데이터가 틀어집니다. 카메라당 링크 대역폭을 {fair / 1e6 * 0.95:.0f} "
                         f"MB/s 이하로{period}.")
        elif uplink and n_cameras:
            per = min(limit, min(period_hz or max_fps, max_fps) * frame)
            total = n_cameras * per
            if total > uplink * 1e6:
                warns.append(f"카메라 {n_cameras}대 × 약 {per / 1e6:.0f} MB/s = {total / 1e6:.0f} MB/s 가 스위치 업링크 "
                             f"{uplink:g} MB/s 를 넘습니다 — 스위치가 패킷을 버려 'Incomplete image' 로 프레임이 "
                             "깨집니다 (동기 트리거면 순간 합이 더 크다).")
    return [("error", text) for text in errors] + [("warn", text) for text in warns]


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

# '감지된 장비' 표의 동기 방식 칸. 카메라 노드의 두 역할(hardware_trigger.role · ptp_action.role)을
# 한 쌍으로 고른다 — 둘이 같이 켜지면 노드가 예외로 죽는다 (flir_spinnaker_camera_node 의 배타 검사).
# GPIO master(카메라가 다른 카메라를 때리는 옛 배선)는 고를 수 없다. 리포 인벤토리에 있으면 '기타' 로 보인다.
#   (키, 표시, hardware_trigger_role, ptp_action_role)
SYNC_MODES = [
    ("hw_trigger", "HW 트리거 (GPIO)", "slave", "none"),
    ("ptp_sender", "PTP 액션 (보내기)", "none", "sender"),
    ("ptp_receiver", "PTP 액션 (받기)", "none", "receiver"),
    ("free", "자유 실행", "none", "none"),
]
SYNC_ROLE_KEYS = ("hardware_trigger_role", "ptp_action_role")
SYNCED_MODES = ("hw_trigger", "ptp_sender", "ptp_receiver")   # 같은 트리거로 동시에 찍는 동기 방식


def _role(entry, key):
    return str(entry.get(key) or "none").strip().lower()


def sync_mode(entry):
    """이 항목의 동기 방식 키. 표에 없는 조합(GPIO master 등)이면 None."""
    hw, ptp = _role(entry, "hardware_trigger_role"), _role(entry, "ptp_action_role")
    ptp = "sender" if ptp in SENDER_ROLES else ptp
    for key, _label, mode_hw, mode_ptp in SYNC_MODES:
        if (hw, ptp) == (mode_hw, mode_ptp):
            return key
    return None


def sync_mode_label(entry):
    key = sync_mode(entry)
    if key:
        return next(label for k, label, *_ in SYNC_MODES if k == key)
    return (f"기타 (GPIO {_role(entry, 'hardware_trigger_role')} · "
            f"PTP {_role(entry, 'ptp_action_role')})")


def set_sync_mode(sub, overrides, serial, mode):
    """카메라 하나의 동기 방식을 GUI 설정에 적는다. 원본(인벤토리 · 기본값)과 같으면 지운다."""
    _key, _label, hw, ptp = next(m for m in SYNC_MODES if m[0] == mode)
    store = camera_overrides(overrides).setdefault(serial, {})
    default = default_camera_entry(sub, serial)
    for role in SYNC_ROLE_KEYS:
        store.pop(role, None)
    if sync_mode(default) != mode:
        # 두 역할을 늘 한 쌍으로 적는다 — 하나만 적으면 camera_entry 가 나머지를 옛 규칙으로 채운다
        store.update({"hardware_trigger_role": hw, "ptp_action_role": ptp})
    if not store:
        camera_overrides(overrides).pop(serial, None)


def camera_overrides(overrides):
    """cfg["sensors"][group]["cameras"] — {serial: {name?, hardware_trigger_role?, ptp_action_role?, force_ip_*?}}"""
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
            # 트리거가 외부(GNSS PPS 등)면 새 카메라도 그걸 받는다. 아니면 PTP action 을 받는 쪽.
            entry.update({"hardware_trigger_role": "slave", "ptp_action_role": "none"}
                         if sub.get("external_trigger") else
                         {"hardware_trigger_role": "none", "ptp_action_role": "receiver"})
    mine = camera_overrides(overrides).get(serial) or {}
    if mine.get("name"):
        entry["name"] = entry["namespace"] = mine["name"]
        entry["frame_id"] = f"{mine['name']}_optical_frame"
    if sub.get("sync_roles") and any(mine.get(role) for role in SYNC_ROLE_KEYS):
        # 예전 GUI 는 ptp_action_role 만 적었고, 그때 GPIO 역할은 인벤토리 값(새 카메라면 none)이었다.
        # 그 뜻 그대로 읽어야 'none'(자유 실행)으로 저장해 둔 새 카메라가 HW 트리거로 바뀌지 않는다.
        entry["hardware_trigger_role"] = mine.get(
            "hardware_trigger_role", (repo.get(serial) or {}).get("hardware_trigger_role", "none"))
        entry["ptp_action_role"] = mine.get("ptp_action_role", entry.get("ptp_action_role", "none"))
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


def _fix_sync_roles(entries, repo, sub, notes):
    """보내는 쪽이 안 보이면 받는 쪽은 프레임이 0장이다 — 이번 기동에서만 바로잡는다."""
    label = sub["label"]
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

    slaves = [e for e in entries if _role(e, "hardware_trigger_role") == "slave"]
    masters = [e for e in entries if _role(e, "hardware_trigger_role") == "master"]
    if sub.get("external_trigger"):
        # 트리거 발생원은 외부(GNSS PPS 등)라 master 카메라 없이 slave 만 있는 게 정상이다. 오히려 master 가
        # 있으면 그 카메라만 외부 트리거를 안 받고 자유 실행한다 — slave 로 돌린다.
        for e in masters:
            e["hardware_trigger_role"] = "slave"
            notes.append(f"{label}: 트리거는 외부 발생원이라 GPIO master 인 {e['namespace']} 도 "
                         "slave 로 띄웁니다")
        return
    # GPIO 트리거는 배선이 필요한 역할이라 master 로 올리지 않는다 — slave 를 자유 실행으로.
    if slaves and not masters:
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
            _fix_sync_roles(entries, repo, sub, notes)

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


PTP_MASTER_OFF = ("none", "off", "false", "-")     # 'PTP grandmaster NIC' 에 적으면 ptp4l 을 안 띄운다 (빈 값 = 자동)


def _drop_frame_rate_cap(group, overrides, entries, notes):
    """카메라군의 이번 기동 카메라가 전부 트리거(HW 트리거 · PTP 액션)로 찍으면 '자유 실행 프레임레이트' 상한을 끈다.

    BFS 는 AcquisitionFrameRate 를 센서 줄 단위로 올려 잡는다 — 30 Hz 가 29.9952 Hz (33.3386 ms). 트리거가 정확히
    30 Hz 면 카메라가 매 프레임 5.3 µs 씩 늦게 찍다가 한 번씩 트리거를 버린다 (check_settings 설명). 트리거로 찍을 땐
    fps 를 트리거가 정하니 상한이 필요 없다. 노드는 AcquisitionFrameRate 값이 있으면 Enable 을 도로 켜므로
    (NormalizeFrameRateStartupOverrides) 값까지 뺀다. GUI 설정에는 저장하지 않는다 (이번 기동만)."""
    patched = None
    for sub in subsets_of(group):
        cams = entries.get(sub["key"]) or []
        if not sub.get("sync_roles") or not cams or any(sync_mode(e) not in SYNCED_MODES for e in cams):
            continue
        target = next((t for t in param_targets(group) if t["subset"] == sub["key"]), None)
        merged = _merged_params(group, target, patched or overrides) if target else None
        if not merged or not merged.get("camera.AcquisitionFrameRateEnable"):
            continue
        rate = merged.get("camera.AcquisitionFrameRate")
        patched = patched or copy.deepcopy(overrides or {})
        store = patched.setdefault("params", {}).setdefault(sub["key"], {})
        for key in ("camera.AcquisitionFrameRate", "camera.FrameRateHz_Val"):
            store[key] = None
        store["camera.AcquisitionFrameRateEnable"] = False
        notes.append(f"{sub['label']}: {len(cams)}대 모두 트리거로 찍어서 이번 기동은 '자유 실행 프레임레이트' 상한"
                     + (f"({float(rate):g} Hz)" if rate else "") + "을 끕니다 — 카메라가 상한을 센서 줄 단위로 올려 잡아 "
                     "(30 Hz → 29.995 Hz) 트리거보다 느려지고, 몇십 초마다 트리거를 한 번씩 놓치기 때문 (fps 는 트리거가 정함)")
    return patched or overrides


def running_process(name):
    """이름이 name 인 프로세스가 돌고 있나 (/proc/*/cmdline 의 실행 파일 이름)."""
    proc = Path("/proc")
    for pid_dir in proc.iterdir() if proc.is_dir() else []:
        if not pid_dir.name.isdigit():
            continue
        try:
            argv0 = (pid_dir / "cmdline").read_bytes().split(b"\0", 1)[0].decode(errors="replace")
        except OSError:
            continue
        if Path(argv0).name == name:
            return True
    return False


def external_ptp4l_interfaces():
    """{NIC: 설명} — 지금 돌고 있는 ptp4l 이 맡은 NIC (-i 인자, -f 설정 파일의 [NIC] 절). 기동 전에 부르므로
    여기 잡히는 ptp4l 은 전부 GUI 밖에서 띄운 것이다 (예: sudo ptp4l -f /etc/linuxptp/ptp4l-bc.conf)."""
    out = {}
    proc = Path("/proc")
    for pid_dir in proc.iterdir() if proc.is_dir() else []:
        if not pid_dir.name.isdigit():
            continue
        try:
            argv = [a.decode(errors="replace") for a in (pid_dir / "cmdline").read_bytes().split(b"\0") if a]
        except OSError:
            continue
        if not argv or Path(argv[0]).name != "ptp4l":
            continue
        label = f"{' '.join(argv)} · pid {pid_dir.name}"
        for i, arg in enumerate(argv[:-1]):
            if arg == "-i":
                out.setdefault(argv[i + 1], label)
            elif arg == "-f":
                try:
                    for line in Path(argv[i + 1]).read_text(errors="replace").splitlines():
                        m = re.match(r"\s*\[([^\]]+)\]", line)
                        if m and m.group(1) not in ("global", "unicast_master_table"):
                            out.setdefault(m.group(1).strip(), label)
                except OSError:
                    pass
    return out


def ptp_master_nic(group, overrides, entries, included):
    """(NIC, 대수) — PC 가 PTP grandmaster 여야 하는 카메라(동기 방식 PTP 액션, 또는 'PTP 시각 동기' 가 켜진
    카메라군)가 달린 NIC. 그런 카메라가 없으면 (None, 0). 여러 NIC 에 나뉘면 가장 많은 쪽.

    entries: {subset: [인벤토리 항목]} (이번 기동에 들어간 카메라), included: {subset: [감지된 장비]}."""
    nics, need = [], 0
    for sub in subsets_of(group):
        if not sub.get("sync_roles"):            # 열화상 A70 은 PTP Slave 가 안 된다 (자유 실행)
            continue
        target = next((t for t in param_targets(group) if t["subset"] == sub["key"]), None)
        ptp_all = bool((_merged_params(group, target, overrides) or {}).get("ptp.enable")) if target else False
        nic_of = {d["identity"]: d.get("nic") for d in included.get(sub["key"]) or []}
        for entry in entries.get(sub["key"]) or []:
            if ptp_all or sync_mode(entry) in ("ptp_sender", "ptp_receiver"):
                need += 1
                if nic_of.get(str(entry.get("serial"))):
                    nics.append(nic_of[str(entry.get("serial"))])
    if not nics:
        return None, need
    return max(sorted(set(nics)), key=nics.count), need


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
    included = included_devices(group, overrides, devices) if devices is not None else {}
    overrides = _drop_frame_rate_cap(group, overrides, entries, notes)
    for sub in subsets_of(group):
        if enabled.get(sub["key"], True):
            mtus = nic_mtus(included.get(sub["key"]))
            modes = [sync_mode(e) for e in entries.get(sub["key"]) or []]
            synced = sum(1 for m in modes if m in SYNCED_MODES)
            ptp_cams = sum(1 for m in modes if m in ("ptp_sender", "ptp_receiver"))
            for level, text in check_settings(group, sub["key"], overrides, len(entries.get(sub["key"]) or []),
                                              mtus, synced, ptp_cams):
                notes.append(f"{'⛔' if level == 'error' else '⚠'} {sub['label']}: {text}")

    args = []
    for sub in subsets_of(group):
        if sub.get("enable_arg"):
            args.append(f"{sub['enable_arg']}:={_arg_text(enabled.get(sub['key'], True))}")

    curated = fields_for(group, "launch")
    values = {f["key"]: effective_value(group, f, overrides) for f in curated}
    # 'PTP grandmaster NIC' — 비워 두면 알아서 (PTP 를 쓰는 카메라가 있으면 그 NIC 에서 ptp4l, 없으면 안 띄운다).
    # 이미 다른 ptp4l 이 맡은 NIC 에는 띄우지 않는다 — 외부 grandmaster(Orin GNSS) + boundary clock 을 따로 돌릴 때
    # 같은 NIC 에 둘이면 PTP 식별자(NIC MAC)가 같아 응답을 서로 가로채고, 소프트웨어 모드 ptp4l 은 slave 가 되면
    # PC 시스템 시계를 끌고 가서 phc2sys 와 싸운다 (2026-09-22: 'eno1' 을 적어 두어 boundary clock 의 eno1 에 한 개 더 떴다).
    if "ptp_master_interface" in values and devices is not None:
        requested = str(values["ptp_master_interface"] or "").strip()
        nic, need = ptp_master_nic(group, overrides, entries, included)
        external = external_ptp4l_interfaces()
        if not requested:
            if nic and nic in external:
                notes.append(f"PTP grandmaster: {nic} 은 이미 다른 ptp4l 이 맡고 있어 GUI 는 ptp4l 을 띄우지 않습니다 "
                             f"({external[nic]}) — 카메라는 그 PTP 를 받습니다")
            elif nic:
                values["ptp_master_interface"] = nic
                notes.append(f"PTP grandmaster: PTP 를 쓰는 카메라 {need}대가 있어 {nic} 에서 ptp4l 을 띄웁니다 "
                             "(PC = 기준 시계, 자동)")
                # 카메라는 PC 시계를 받는다 — 그 PC 시계가 GNSS(Orin)를 따라가는지는 phc2sys 가 도는지로 본다.
                # 안 돌면 카메라끼리는 맞아도 GNSS 시각이 아니다 (config/systemd/ptp-orin · phc2sys-orin).
                if not running_process("phc2sys"):
                    notes.append("⚠ phc2sys 가 안 돌고 있어 PC 시계가 GNSS(Orin) 시각을 따라가지 않습니다 — 카메라끼리는 "
                                 "맞지만 GNSS 시각이 아닙니다. 부팅 서비스 ptp-orin · phc2sys-orin 을 켜세요 "
                                 "(config/systemd, README 'PTP 처음부터 세팅')")
            elif need:
                notes.append(f"⛔ PTP 를 쓰는 카메라가 {need}대인데 달린 NIC 를 몰라 ptp4l 을 못 띄웁니다 — "
                             "'PTP grandmaster NIC' 에 카메라 NIC 를 직접 적으세요")
        elif requested.lower() not in PTP_MASTER_OFF:
            if requested in external:
                values["ptp_master_interface"] = ""
                notes.append(f"⛔ 'PTP grandmaster NIC' 가 {requested} 인데 거기엔 이미 다른 ptp4l 이 돌고 있어 띄우지 "
                             f"않습니다 ({external[requested]}). 같은 NIC 에 둘이면 PTP 식별자가 겹치고 둘 다 PC 시계를 "
                             "건드립니다. 외부 PTP 를 쓸 거면 이 칸을 비워 두세요 (자동)")
            elif nic and requested != nic:
                notes.append(f"⚠ 'PTP grandmaster NIC' 가 {requested} 인데 PTP 를 쓰는 카메라는 {nic} 에 있습니다 — "
                             f"{requested} 의 ptp4l 시각은 카메라에 닿지 않습니다. 이 칸은 PC 의 카메라 NIC 이름입니다")
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
        if value is None or not condition_met(field.get("enabled_when"), values.get):
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
                    ("namespace", "hardware_trigger_role", "ptp_action_role", "force_ip_address")
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
        for key in ("name", *SYNC_ROLE_KEYS, "force_ip_address", "force_ip_subnet_mask"):
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
