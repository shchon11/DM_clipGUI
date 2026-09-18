#!/usr/bin/env python3
# param_doc.py — 원본 params YAML · 런치 파일에서 "설정할 수 있는 것 전부"를 뽑는다.
#
# GUI 설정 탭의 '전체 설정' 이 읽는다. 폼을 sensors.yaml 에 손으로 다 적지 않고 파일에서 뽑는
# 이유: 리포의 YAML 에 키가 늘거나 주석이 바뀌면 GUI 가 그대로 따라가야 한다.
#
# params YAML 에서 읽는 것:
#   - 키 순서 그대로, 섹션은 "# ----- / # 제목 / # -----" 머리와 빈 줄 뒤의 짧은 제목 줄
#   - 키 바로 위 주석 + 줄 끝 주석 → 도움말 (툴팁)
#   - 주석 처리된 키 (# camera.Gamma_FloatVal: 1.0) → '지정' 해야 들어가는 선택 항목, 값은 예시
#   - 주석의 "Off | Once | Continuous", "{NORMAL, EXTENDED}", "- LEGACY" 목록 → 고르기 칸
# 런치 파일에서 읽는 것: DeclareLaunchArgument 의 이름 · 기본값 · 설명 (ast, 실행하지 않는다)
#
# 단독 실행:  python3 param_doc.py <params.yaml | *.launch.py>

import ast
import re
import sys
from pathlib import Path

import yaml

_DASH = re.compile(r"^-{8,}")
_KEY = re.compile(r"^([A-Za-z_][A-Za-z0-9_.]*)\s*:(?:\s+(.*))?$")
_PIPE_CHOICES = re.compile(r"(?:^|\s)([\w.+-]+(?:[ \t]*\|[ \t]*[\w.+-]+)+)")
_BRACE_CHOICES = re.compile(r"\{([^{}]+)\}")
_DASH_ITEM = re.compile(r'^-\s+"?([A-Za-z][A-Za-z0-9_]*)"?\s*(?::|\(|$)')


def split_inline_comment(text):
    """'"Red"   # Red | Blue' -> ('"Red"', 'Red | Blue'). 따옴표 안의 # 은 주석이 아니다."""
    quote = None
    for index, char in enumerate(text):
        if quote:
            if char == quote:
                quote = None
        elif char in "\"'":
            quote = char
        elif char == "#" and (index == 0 or text[index - 1] in " \t"):
            return text[:index].rstrip(), text[index + 1:].strip()
    return text.rstrip(), ""


def _choices_from(lines, value):
    """도움말에서 고를 수 있는 값 목록을 찾는다. 못 찾으면 None.

    숫자 목록({1, 2, 4, 8})은 고르기로 만들지 않는다 — 콤보는 글자를 돌려주는데 ROS 파라미터는
    타입이 엄격해서, 원래 타입으로 되돌리는 데서 틀리기 쉽다. 그런 건 도움말로만 둔다.
    """
    if isinstance(value, bool) or (value is not None and not isinstance(value, str)):
        return None
    found = []
    items = [m.group(1) for m in (_DASH_ITEM.match(l) for l in lines) if m]
    if len(items) >= 2 and all(i.isupper() or "_" in i or i[:1].isupper() for i in items):
        found = items
    for line in reversed(lines):             # 줄 끝 주석이 마지막 줄이라 먼저 본다
        if found:
            break
        m = _PIPE_CHOICES.search(line)
        if m:
            found = [t.strip() for t in m.group(1).split("|")]
    if not found:
        # "{ 512x10,\n 512x20, ... }" 처럼 줄을 넘기는 목록이 있어서 이어 붙인 글에서 찾는다
        m = _BRACE_CHOICES.search(" ".join(lines))
        if m:
            found = [t.strip().strip("'\"") for t in m.group(1).split(",") if t.strip()]
    found = [c for c in found if c and c != "..."]
    if len(found) < 2 or any(re.fullmatch(r"-?[0-9.]+", c) for c in found):
        return None
    if value is not None and value not in found:
        found.insert(0, value)
    return found


def value_type(value):
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, list):
        return "list"
    return "string"


def _is_title(text):
    """빈 줄 뒤의 '# Pixel format / output setup' 같은 소제목인가."""
    return (0 < len(text) <= 40 and text[-1] not in ".:,;)" and "|" not in text
            and "=" not in text and not _KEY.match(text) and not text.startswith("-"))


def _entry(key, value, commented, help_lines, section, inline=""):
    lines = [l for l in help_lines if l is not None]
    if inline:
        lines.append(inline)
    return {"key": key, "value": value, "commented": commented, "section": section,
            "help": "\n".join(lines).strip(), "type": value_type(value),
            "choices": _choices_from(lines, value)}


def parse_params(path):
    """params YAML -> {"node_key", "entries": [...], "sections": [(제목, [키])]}. 못 읽으면 None.

    entry = {key, value, commented, section, help, type, choices}
    """
    path = Path(path).expanduser()
    try:
        text = path.read_text(encoding="utf-8")
        doc = yaml.safe_load(text) or {}
    except (OSError, yaml.YAMLError):
        return None
    node_key, params = None, {}
    for key, body in doc.items():
        if isinstance(body, dict) and "ros__parameters" in body:
            node_key, params = key, dict(body["ros__parameters"] or {})
            break
    if node_key is None:
        return None

    lines = text.splitlines()
    start = next((i for i, l in enumerate(lines) if l.strip() == "ros__parameters:"), None)
    if start is None:
        return None

    default_section = path.name
    section = default_section
    entries, seen = [], set()
    pending = []             # 다음 키의 도움말이 될 주석 줄들
    # 주석 없이 바로 붙어 있는 다음 키가 같이 쓰는 도움말 (Width/Height, OffsetX/OffsetY).
    # 주석 처리된 키와 실제 키 사이에는 나누지 않는다 — A70 의 IRWindowing 설명이 Width 에 붙는다.
    shared = None            # (도움말, 주석 처리된 키였나)
    after_blank = True
    in_header = False
    header_title = None
    indent = None

    for raw in lines[start + 1:]:
        stripped = raw.strip()
        if not stripped:
            pending, shared, after_blank = [], None, True
            continue
        lead = len(raw) - len(raw.lstrip())
        if stripped.startswith("#"):
            body = stripped[1:].strip()
            if _DASH.match(body):
                if in_header:
                    in_header = False
                    section = header_title or section
                else:
                    in_header, header_title = True, None
                pending, shared, after_blank = [], None, False
                continue
            if in_header:
                if header_title is None and body:
                    header_title = body
                continue
            match = _KEY.match(body)
            if match and match.group(2):
                value_text, inline = split_inline_comment(match.group(2))
                try:
                    value = yaml.safe_load(value_text) if value_text else None
                except yaml.YAMLError:
                    value = None
                # "# receiver: Action0 FrameStart trigger를 arm함" 같은 설명 줄은 키가 아니다 —
                # 주석 처리된 진짜 키는 숫자/불/목록이거나 따옴표 친 문자열이다.
                looks_real = value_text and (isinstance(value, (bool, int, float, list))
                                             or value_text[:1] in "\"'")
                if looks_real and match.group(1) not in params and match.group(1) not in seen:
                    help_lines = pending if pending else ([shared[0]] if shared and shared[1] else [])
                    entries.append(_entry(match.group(1), value, True, help_lines, section, inline))
                    seen.add(match.group(1))
                    shared = ("\n".join(help_lines), True)
                    pending, after_blank = [], False
                    continue
            if after_blank and not pending and _is_title(body):
                section = body
                after_blank = False
                continue
            pending.append(body)
            after_blank = False
            continue

        after_blank = False
        if indent is None:
            indent = lead
        if lead != indent:
            continue                           # 여러 줄에 걸친 값의 뒷부분
        match = _KEY.match(stripped)
        if not match or match.group(1) not in params or match.group(1) in seen:
            continue
        key = match.group(1)
        _, inline = split_inline_comment(match.group(2) or "")
        help_lines = pending if pending else ([shared[0]] if shared and not shared[1] else [])
        entries.append(_entry(key, params[key], False, help_lines, section, inline))
        seen.add(key)
        shared = ("\n".join(help_lines), False)
        pending = []

    # 파일 줄에서 못 찾은 키 (흐름 매핑 등) 도 빠뜨리지 않는다
    for key, value in params.items():
        if key not in seen:
            entries.append(_entry(key, value, False, [], default_section))

    sections, order = {}, []
    for entry in entries:
        if entry["section"] not in sections:
            sections[entry["section"]] = []
            order.append(entry["section"])
        sections[entry["section"]].append(entry["key"])
    return {"node_key": node_key, "entries": entries,
            "sections": [(title, sections[title]) for title in order]}


# ---------- 런치 인자 ----------

def _literal(node):
    try:
        return ast.literal_eval(node)
    except (ValueError, TypeError, SyntaxError, MemoryError, RecursionError):
        return None


def launch_value(text):
    """런치 인자 기본값 글자 -> 편집용 값 ('true' -> True, '3.0' -> 3.0)."""
    if text is None:
        return None
    low = text.strip().lower()
    if low in ("true", "false"):
        return low == "true"
    if re.fullmatch(r"-?\d+", text.strip()):
        return int(text)
    if re.fullmatch(r"-?(\d+\.\d*|\.\d+)([eE]-?\d+)?", text.strip()):
        return float(text)
    return text


def parse_launch_args(path):
    """*.launch.py 의 DeclareLaunchArgument 들 -> [{key, value, text, help, type, choices}].

    실행하지 않고 ast 로만 읽는다 (런치 파일을 import 하면 인벤토리 툴 같은 부수 효과가 돈다).
    기본값이 식(경로 계산 등)이면 value=None.
    """
    try:
        tree = ast.parse(Path(path).expanduser().read_text(encoding="utf-8"))
    except (OSError, SyntaxError):
        return []
    out, seen = [], set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", "")
        if name != "DeclareLaunchArgument":
            continue
        kw = {k.arg: k.value for k in node.keywords if k.arg}
        key = _literal(node.args[0]) if node.args else _literal(kw.get("name"))
        if not isinstance(key, str) or key in seen:
            continue
        seen.add(key)
        text = _literal(kw["default_value"]) if "default_value" in kw else None
        text = text if isinstance(text, str) else None
        value = launch_value(text)
        description = _literal(kw["description"]) if "description" in kw else ""
        choices = _literal(kw["choices"]) if "choices" in kw else None
        out.append({"key": key, "value": value, "text": text,
                    "help": description if isinstance(description, str) else "",
                    "type": value_type(value) if value is not None else "string",
                    "choices": [str(c) for c in choices] if isinstance(choices, list) else None,
                    "computed": "default_value" in kw and text is None})
    # ast.walk 는 너비 우선이라 파일 순서가 아니다 — 줄 번호로 되돌린다
    lines = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", getattr(node.func, "attr", "")) \
                == "DeclareLaunchArgument":
            key = _literal(node.args[0]) if node.args else None
            if isinstance(key, str):
                lines.setdefault(key, node.lineno)
    out.sort(key=lambda e: lines.get(e["key"], 0))
    return out


def main(argv):
    for arg in argv[1:]:
        if arg.endswith(".py"):
            for entry in parse_launch_args(arg):
                print(f"  {entry['key']:<40} {entry['type']:<6} {entry['text']!r:<24} "
                      f"{(entry['help'] or '')[:60]}")
            continue
        doc = parse_params(arg)
        if not doc:
            print(f"{arg}: 못 읽음")
            continue
        print(f"== {arg} (node={doc['node_key']}, {len(doc['entries'])}키)")
        by_key = {e["key"]: e for e in doc["entries"]}
        for title, keys in doc["sections"]:
            print(f"\n  [{title}]")
            for key in keys:
                e = by_key[key]
                mark = "#" if e["commented"] else " "
                choices = f" {{{', '.join(e['choices'])}}}" if e["choices"] else ""
                head = e["help"].splitlines()[0][:50] if e["help"] else ""
                print(f"   {mark} {key:<44} {e['type']:<6} {e['value']!r:<22}{choices}  | {head}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
