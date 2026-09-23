#!/usr/bin/env python3
# genicam_doc.py — 카메라의 GenICam XML 에서 "카메라 노드로 설정할 수 있는 것 전부"를 뽑는다.
#
# GUI 설정 탭 '전체 설정' 의 'GenICam' 상자들이 읽는다. params YAML 에는 누군가 적어 둔 키만 있어서
# 감마 · 채도 · 샤프닝 · 색변환 · LUT · 자동 노출 튜닝 같은 ISP 노드는 거의 빠져 있다. 카메라 노드
# (flir_spinnaker_camera_node)는 쓰기 가능한 GenICam 노드를 전부 camera.<노드 이름> 파라미터로 받으니,
# 노드 목록의 원본인 카메라 XML 을 읽으면 빠짐없이 올릴 수 있다.
#
# XML 은 Spinnaker 가 카메라에 처음 붙을 때 받아 ~/.config/spinnaker/xml/<모델>_<해시>_GENICAM.zip 에
# 캐시한다 — 카메라가 꺼져 있어도 읽힌다.
#
# XML 로 알 수 없는 것: 값의 범위(카메라 레지스터가 정한다)와 지금 쓸 수 있는지(픽셀 포맷 · Enable ·
# Auto 상태에 따라 잠긴다). 노드는 기동할 때 쓸 수 있는 노드만 파라미터로 등록해서, 잠긴 노드를
# 지정하면 값이 조용히 무시된다.
#
# 단독 실행:  python3 genicam_doc.py <xml|zip> [카테고리 ...]

import glob
import re
import sys
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

# 파라미터 파일로 넣을 수 있는 노드 종류 -> param_doc 과 같은 타입 이름
_KINDS = {"Boolean": "bool", "Float": "float", "Integer": "int", "Enumeration": "enum",
          "String": "string", "StringReg": "string"}
_VISIBILITY = ("Beginner", "Expert", "Guru")

_CACHE = {}            # 경로 -> (mtime, 결과)


def resolve(pattern):
    """'~/…/BFS-PGE-23S3C_*_GENICAM.zip' -> 가장 최근 파일 경로. 없으면 None."""
    matches = glob.glob(str(Path(pattern).expanduser()))
    if not matches:
        return None
    return Path(max(matches, key=lambda p: Path(p).stat().st_mtime))


def _read_xml(path):
    if path.suffix.lower() == ".zip":
        with zipfile.ZipFile(path) as archive:
            name = next((n for n in archive.namelist() if n.lower().endswith(".xml")), None)
            if name is None:
                raise ValueError("zip 안에 XML 이 없습니다")
            return archive.read(name)
    return path.read_bytes()


def _parse(path):
    root = ET.fromstring(_read_xml(path))
    match = re.match(r"\{.*\}", root.tag)
    ns = match.group(0) if match else ""

    def tag(element):
        return element.tag[len(ns):] if ns and element.tag.startswith(ns) else element.tag

    def text(element, child):
        found = element.find(ns + child)
        return (found.text or "").strip() if found is not None and found.text else ""

    # 같은 이름이 열거 항목(EnumEntry)으로도 나온다 (SequencerFeatureSelector 의 'Gamma' 등) — 노드만 모은다
    nodes = {}
    for element in root.iter():
        name = element.get("Name")
        if name and tag(element) not in ("EnumEntry", "pFeature"):
            nodes.setdefault(name, element)

    # 선택자: BalanceRatioSelector 의 <pSelected>BalanceRatio</pSelected> -> BalanceRatio 는 선택자로 고른 한 칸
    selectors = {}
    for name, element in nodes.items():
        for child in element.findall(ns + "pSelected"):
            if child.text:
                selectors.setdefault(child.text.strip(), []).append(name)

    return {"root": root, "ns": ns, "tag": tag, "text": text, "nodes": nodes, "selectors": selectors,
            "model": root.get("ModelName", ""), "vendor": root.get("VendorName", "")}


def load(path):
    """XML(또는 zip) -> 파싱 결과. 못 읽으면 None. mtime 이 같으면 다시 파싱하지 않는다."""
    path = Path(path).expanduser()
    try:
        mtime = path.stat().st_mtime
    except OSError:
        return None
    cached = _CACHE.get(str(path))
    if cached and cached[0] == mtime:
        return cached[1]
    try:
        doc = _parse(path)
    except (OSError, ValueError, zipfile.BadZipFile, ET.ParseError):
        doc = None
    _CACHE[str(path)] = (mtime, doc)
    return doc


def _feature(doc, name):
    """노드 하나 -> {name, type, choices, unit, help, visibility, selectors}. 설정할 수 없는 노드면 None."""
    element = doc["nodes"].get(name)
    if element is None:
        return None
    kind = _KINDS.get(doc["tag"](element))
    if kind is None:                            # Command · Register · SwissKnife …
        return None
    visibility = doc["text"](element, "Visibility") or "Beginner"
    if visibility not in _VISIBILITY:           # Invisible — 노드도 파라미터로 안 올린다
        return None
    if doc["text"](element, "ImposedAccessMode") == "RO":
        return None
    choices = None
    if kind == "enum":
        choices = []
        for entry in element.findall(doc["ns"] + "EnumEntry"):
            if (doc["text"](entry, "Visibility") or "Beginner") in _VISIBILITY and entry.get("Name"):
                choices.append(entry.get("Name"))
        if not choices:
            return None
    tip = doc["text"](element, "ToolTip") or doc["text"](element, "Description")
    display = doc["text"](element, "DisplayName")
    return {"name": name, "display": display, "type": kind, "choices": choices,
            "unit": doc["text"](element, "Unit"), "help": " ".join(tip.split()),
            "visibility": visibility, "selectors": doc["selectors"].get(name, []),
            "depends": dependencies(doc, name)}


def _public_features(doc):
    """Root 카테고리 트리에 걸린 공개 기능 이름들 (SpinView 가 보여주는 것)."""
    out, stack = [], ["Root"]
    while stack:
        element = doc["nodes"].get(stack.pop())
        if element is None:
            continue
        for child in element.findall(doc["ns"] + "pFeature"):
            name = (child.text or "").strip()
            node = doc["nodes"].get(name)
            if node is None:
                continue
            if doc["tag"](node) == "Category":
                stack.append(name)
            else:
                out.append(name)
    return out


def _owners(doc):
    """내부 노드 -> 그 노드를 값으로 쓰는 공개 기능 (Gamma_FloatVal · Gamma_Val -> Gamma). 한 번만 만든다."""
    if "_owners" not in doc:
        owners = {}
        for public in _public_features(doc):
            stack, seen = [public], set()
            while stack:
                name = stack.pop()
                if name in seen or name not in doc["nodes"]:
                    continue
                seen.add(name)
                owners.setdefault(name, public)
                stack += [c.text.strip() for c in doc["nodes"][name].findall(doc["ns"] + "pValue") if c.text]
        doc["_owners"] = owners
    return doc["_owners"]


def dependencies(doc, name):
    """이 노드를 잠그거나 풀 수 있는 공개 기능들 — 사용 가능 · 잠김 레지스터를 다시 읽게 만드는(pInvalidator)
    노드를 공개 이름으로 되돌린 것. 예: Gamma -> [GammaEnable], Saturation -> [SaturationEnable].

    값의 방향(켜야 풀리나 꺼야 풀리나)은 XML 에 없다 — 카메라 펌웨어가 정한다. 시퀀서 · 사용자 설정 세트는
    모든 노드에 걸려 있어서 뺀다.
    """
    element = doc["nodes"].get(name)
    if element is None:
        return []
    owners, found = _owners(doc), set()
    stack = [doc["text"](element, t) for t in ("pIsAvailable", "pIsLocked")]
    seen = set()
    while stack:
        ref = stack.pop()
        if not ref or ref in seen or ref not in doc["nodes"]:
            continue
        seen.add(ref)
        for child in doc["nodes"][ref]:
            kind = doc["tag"](child)
            if kind == "pInvalidator" and child.text:
                found.add(child.text.strip())
            elif kind == "pVariable" and child.text:
                stack.append(child.text.strip())
    out = set()
    for register in found:
        base = register.rsplit("_", 1)[0]
        owner = owners.get(register) or (base if base in owners else None)
        if owner and owner != name and not owner.startswith(("Sequencer", "UserSet")):
            out.add(owner)
    return sorted(out)


def locked_while_streaming(doc, name):
    """영상을 받는 동안(취득 중) 카메라가 잠그는 노드인가 — 잠김 조건에 TLParamsLocked 가 들어 있으면.

    이런 노드는 실행 중에 바꿀 수 없고 다시 기동해야 들어간다 (BlackLevelClampingEnable · IspEnable 등).
    """
    element = doc["nodes"].get(name)
    if element is None:
        return False
    stack, seen = [doc["text"](element, "pIsLocked")], set()
    while stack:
        ref = stack.pop()
        if not ref or ref in seen or ref not in doc["nodes"]:
            continue
        if ref == "TLParamsLocked":
            return True
        seen.add(ref)
        stack += [c.text.strip() for c in doc["nodes"][ref]
                  if doc["tag"](c) in ("pVariable", "pValue", "pIsLocked") and c.text]
    return False


def category_sections(doc, categories):
    """[(카테고리 표시 이름, 카테고리 이름, [feature])] — 하위 카테고리는 펼쳐서 같은 상자에 넣는다."""
    out = []
    for category in categories:
        element = doc["nodes"].get(category)
        if element is None or doc["tag"](element) != "Category":
            continue
        features, seen = [], set()

        def walk(cat, sub):
            for child in cat.findall(doc["ns"] + "pFeature"):
                name = (child.text or "").strip()
                if not name or name in seen:
                    continue
                seen.add(name)
                node = doc["nodes"].get(name)
                if node is not None and doc["tag"](node) == "Category":
                    walk(node, doc["text"](node, "DisplayName") or name)
                    continue
                feature = _feature(doc, name)
                if feature:
                    feature["section"] = sub
                    features.append(feature)

        walk(element, "")
        if features:
            out.append((doc["text"](element, "DisplayName") or category, category, features))
    return out


def main(argv):
    if len(argv) < 2:
        print(__doc__ or "usage: genicam_doc.py <xml|zip> [category ...]")
        return 1
    path = resolve(argv[1]) or Path(argv[1])
    doc = load(path)
    if not doc:
        print(f"{argv[1]}: 못 읽음")
        return 1
    print(f"== {path.name} ({doc['vendor']} {doc['model']})")
    if len(argv) == 2:
        for name, element in doc["nodes"].items():
            if doc["tag"](element) == "Category":
                print(f"  {name}")
        return 0
    for title, name, features in category_sections(doc, argv[2:]):
        print(f"\n  [{title}] ({name})")
        for f in features:
            choices = f" {{{', '.join(f['choices'])}}}" if f["choices"] else ""
            sel = f" <- {', '.join(f['selectors'])}" if f["selectors"] else ""
            print(f"    {f['name']:<40} {f['type']:<6} {f['visibility']:<8}{choices}{sel}  | {f['help'][:60]}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
