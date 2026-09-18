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

import sys
from pathlib import Path

import yaml

from sensor_discovery import expand, load_params, load_registry

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
        return field.get("default")
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
        field = by_key.get(key, {})
        if value is None:
            # optional 필드를 '지정 안 함'으로 되돌린 경우 — 키 자체를 뺀다.
            merged.pop(key, None)
            continue
        merged[key] = value
        # 값만 써서는 안 먹는 키가 있다 (AcquisitionFrameRate 는 Enable 이 켜져야 한다).
        for implied_key, implied_value in (field.get("implies") or {}).items():
            merged[implied_key] = implied_value
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


def launch_args(group, overrides):
    """['arg:=value', ...] — subset 활성화 플래그 + 폼의 launch_arg + 생성 params 파일."""
    args = []

    enabled = (overrides or {}).get("subsets") or {}
    for sub in subsets_of(group):
        if not sub.get("enable_arg"):
            continue
        args.append(f"{sub['enable_arg']}:={_arg_text(enabled.get(sub['key'], True))}")

    for field in fields_for(group, "launch"):
        value = effective_value(group, field, overrides)
        if value is None:
            continue
        args.append(f"{field['key']}:={_arg_text(value)}")

    for params_arg, path in build_params_files(group, overrides).items():
        args.append(f"{params_arg}:={path}")

    return args


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
