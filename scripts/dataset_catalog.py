#!/usr/bin/env python3
# dataset_catalog.py — 지역별 데이터셋 목록(CSV) + 브라우저 취득 현황표.
#
# 기준은 폴더다: <저장 위치>/route_NNN_<지역>/{clip_*,rec_*}/ 를 훑어
#   - metadata.yaml (rosbag2)  → 날짜 · 시작/끝 시각 · bag 길이 · 메시지 수
#   - dataset_info.json (GUI)  → 운전자 · 동승자 · 라벨 · 비고
# 를 모아 <저장 위치>/datasets.csv 를 새로 쓴다. 한 번 들어온 줄은 datasets_archive.json 에 보관해서
# bag 폴더를 지우거나 옮겨도 목록에 남는다 ('bag 폴더' 칸 = 없음).
#
#   python3 dataset_catalog.py rebuild            # CSV 만 다시 쓰기
#   python3 dataset_catalog.py serve [--port N]   # http://localhost:8765 현황표 (인터넷 없이 동작)
#
# GUI(clip_gui.py)는 녹화가 끝날 때마다 dataset_info.json 을 쓰고 CSV 를 다시 만든다.

import argparse
import csv
import datetime as dt
import io
import json
import math
import re
import secrets
import sys
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

import hangul_roman

try:
    YamlLoader = yaml.CSafeLoader
except AttributeError:
    YamlLoader = yaml.SafeLoader

INFO_FILE = "dataset_info.json"
CSV_NAME = "datasets.csv"
DEFAULT_PORT = 8765
DEFAULT_GOAL_HOURS = 4.0
DEFAULT_SCALE_HOURS = 6.0     # 게이지 전체 길이 — 목표(4h)를 넘겨 더 모아도 계속 보이게
ROUTE_RE = re.compile(r"route_(\d+)_(.+)$")
EDITABLE = ("driver", "passenger", "note", "label")
# 도로 형상 — 녹화 GUI(clip_gui.ROAD_SHAPES)와 같은 목록. 현황표에서 더블클릭해 고친다.
ROAD_SHAPES = ["좁은 골목", "일반도로", "대로", "교차로", "경사로", "터널/지하"]
KIND_REC, KIND_CLIP = "수동녹화(Loop)", "Clip(30s)"

# 시간대 — 녹화 구간의 태양 고도로 나눈다 (30초 간격으로 쪼개 합산, 녹화 하나가 걸쳐 있으면 나눠서 센다)
#   day: 고도 ≥ 6°   dusk: -6° ~ 6° (해 질 녘·해 뜰 녘 박명 포함)   night: < -6°
# 위치는 bag 의 GNSS 궤적(gnss_track.json, 측위된 점만) → 없으면 서울 시청
TOD = ("day", "dusk", "night")
TOD_DAY_DEG, TOD_NIGHT_DEG = 6.0, -6.0
DEFAULT_LATLON = (37.5665, 126.9780)
TOD_STEP_SEC = 30.0

# (CSV 열, 현황표 제목)
COLUMNS = [
    ("region", "지역(영문)"), ("region_name", "지역"), ("route", "route"), ("date", "날짜"),
    ("start_time", "시작"), ("end_time", "끝"),
    ("time_of_day", "시간대"), ("day_sec", "day(초)"), ("dusk_sec", "dusk(초)"), ("night_sec", "night(초)"),
    ("duration_sec", "bag 길이(초)"), ("duration_hms", "bag 길이"),
    ("kind", "종류"), ("road_shapes", "도로 형상"), ("driver", "운전자"), ("passenger", "동승자"),
    ("label", "라벨"), ("map_route", "지도 루트"), ("map_coverage", "재현율(%)"), ("map_run", "지도 run"),
    ("messages", "메시지 수"), ("size_gb", "용량(GB)"),
    ("note", "비고"), ("folder", "폴더"), ("on_disk", "bag 폴더"),
]
_lock = threading.Lock()

# 계획표를 불러올 구글 시트 (링크 공유 '보기 가능' 이어야 한다). 첫 탭을 CSV 로 받는다.
GSHEET_ID = "18Idhfcgy4xb68a5XEqkxtbppPgnUABZk94sRKkaEGpc"
GSHEET_EDIT_URL = f"https://docs.google.com/spreadsheets/d/{GSHEET_ID}/edit"
GSHEET_CSV_URL = f"https://docs.google.com/spreadsheets/d/{GSHEET_ID}/export?format=csv"

# 현황을 올릴 구글 시트 — 시트에 붙인 Apps Script 웹 앱이 받아서 gid=0 탭에 쓴다 (구글 쓰기는 인증이 필요해서).
# 웹 앱 URL · 토큰은 ~/.config/dm_clip_gui/gsheet_upload.json (토큰이 들어가므로 저장소에 두지 않는다)
UPLOAD_SHEET_ID = "1PMSNS9krULhPD0G6QuZO1az43fwIgoCMVbMobe-zgLI"
UPLOAD_SHEET_GID = 0
UPLOAD_SHEET_URL = f"https://docs.google.com/spreadsheets/d/{UPLOAD_SHEET_ID}/edit#gid={UPLOAD_SHEET_GID}"
UPLOAD_CFG = Path.home() / ".config" / "dm_clip_gui" / "gsheet_upload.json"
NUMERIC = {"duration_sec", "day_sec", "dusk_sec", "night_sec", "messages", "size_gb"}

APPS_SCRIPT = r"""// DM 유효한 데이터셋 리스트 → 구글 시트 (gid=__GID__ 탭을 통째로 새로 쓴다)
// 확장 프로그램 ▸ Apps Script 에 이 코드를 붙여 넣고  배포 ▸ 새 배포 ▸ 웹 앱
//   (실행 계정: 나 / 액세스 권한: 모든 사용자)  → 나오는 웹 앱 URL 을 현황표에 넣는다.
const TOKEN = "__TOKEN__";
const SHEET_ID = "__SHEET_ID__";
const SHEET_GID = __GID__;

function doPost(e) {
  try {
    const d = JSON.parse(e.postData.contents);
    if (d.token !== TOKEN) return out({ok: false, error: "토큰이 맞지 않습니다 — 현황표에 표시된 코드로 다시 배포하세요"});
    const ss = SpreadsheetApp.openById(SHEET_ID);
    const sh = ss.getSheets().find(s => s.getSheetId() === SHEET_GID) || ss.getSheets()[0];
    sh.clearContents();
    const values = [d.header].concat(d.rows);
    // 날짜 · 시각 · 길이는 글자 그대로 (시트가 시각으로 바꿔 초를 숨기지 않게 — 10:00:00 → '오전 10:00')
    (d.text_cols || []).forEach(c => sh.getRange(1, c + 1, values.length, 1).setNumberFormat("@"));
    sh.getRange(1, 1, values.length, d.header.length).setValues(values);
    sh.setFrozenRows(1);
    return out({ok: true, n: d.rows.length});
  } catch (err) {
    return out({ok: false, error: String(err)});
  }
}

function out(o) {
  return ContentService.createTextOutput(JSON.stringify(o)).setMimeType(ContentService.MimeType.JSON);
}
"""


def upload_cfg():
    """{url, token} — 토큰이 없으면 만들어 저장한다 (Apps Script 코드에 같이 들어간다)."""
    try:
        cfg = json.loads(UPLOAD_CFG.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        cfg = {}
    if not cfg.get("token"):
        cfg["token"] = secrets.token_urlsafe(24)
        save_upload_cfg(cfg)
    return cfg


def save_upload_cfg(cfg):
    UPLOAD_CFG.parent.mkdir(parents=True, exist_ok=True)
    UPLOAD_CFG.write_text(json.dumps(cfg, indent=1), encoding="utf-8")
    UPLOAD_CFG.chmod(0o600)


def apps_script_code():
    return (APPS_SCRIPT.replace("__TOKEN__", upload_cfg()["token"])
            .replace("__SHEET_ID__", UPLOAD_SHEET_ID).replace("__GID__", str(UPLOAD_SHEET_GID)))


def upload_to_gsheet(base):
    """유효한 데이터셋 리스트(리스트에서 뺀 녹화 제외)를 Apps Script 웹 앱으로 보낸다. 반환: 줄 수."""
    cfg = upload_cfg()
    if not cfg.get("url"):
        raise LookupError("웹 앱 URL 이 아직 없습니다")
    rows = _read_csv(rebuild(base)[0])                 # 폴더를 지운 녹화도 포함한 유효한 리스트
    keys = [k for k, _ in COLUMNS]
    body = json.dumps({
        "token": cfg["token"],
        "header": [t for _, t in COLUMNS],
        "text_cols": [keys.index(k) for k in ("date", "start_time", "end_time", "duration_hms")],
        "rows": [[(float(r[k]) if k in NUMERIC and r[k] not in ("", None) else r[k]) for k in keys]
                 for r in rows],
    }, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(cfg["url"], data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30) as r:        # 302 → 결과 페이지(GET)는 urllib 이 따라간다
        text = r.read().decode("utf-8", "replace")
    try:
        res = json.loads(text)
    except ValueError:
        raise RuntimeError("웹 앱이 JSON 대신 로그인 페이지 등을 돌려줬습니다 — 배포의 '액세스 권한: 모든 사용자' 를 확인하세요")
    if not res.get("ok"):
        raise RuntimeError(res.get("error") or "알 수 없는 오류")
    return res.get("n", len(rows))

# 계획표 — 원본은 구글 시트(GSHEET_ID). 서버가 PLAN_SYNC_SEC 마다 읽어 <저장 위치>/plans.csv 에 캐시한다.
# 현황표에서는 고치지 않는다 (양쪽에서 고치다 덮어쓰지 않게) — 인터넷이 없으면 마지막으로 가져온 계획을 쓴다.
PLAN_SYNC_SEC = 60
PLAN_CSV = "plans.csv"
PLAN_COLUMNS = [
    ("region", "지역"), ("scene", "장면 이름"), ("region_en", "지역(영문)"), ("route_desc", "경로 (출발 → 경유 → 도착)"),
    ("planned_hours", "계획 시간(h)"), ("note", "비고"),
]


def default_base():
    try:
        cfg = yaml.safe_load((Path.home() / ".config/dm_clip_gui/last_session.yaml")
                             .read_text(encoding="utf-8")) or {}
        return Path(cfg["recorder"]["output_dir"]).expanduser()
    except Exception:
        return Path.home() / "DM_clipGUI" / "clips"


def hms(sec):
    sec = int(round(sec))
    return f"{sec // 3600}:{sec % 3600 // 60:02d}:{sec % 60:02d}"


def read_info(bag):
    try:
        return json.loads((Path(bag) / INFO_FILE).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def write_info(bag, **fields):
    """dataset_info.json 에 필드를 덧쓴다 (없던 파일이면 만든다)."""
    info = read_info(bag)
    info.update({k: v for k, v in fields.items() if v is not None})
    (Path(bag) / INFO_FILE).write_text(
        json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")
    return info


def sun_elevation(ts, lat, lon):
    """태양 고도(도) — 간이 천문 공식, 오차 0.1° 수준 (시간대 나누기에 충분)."""
    d = ts / 86400.0 + 2440587.5 - 2451545.0
    g = math.radians(357.529 + 0.98560028 * d)
    L = math.radians(280.459 + 0.98564736 * d + 1.915 * math.sin(g) + 0.020 * math.sin(2 * g))
    e = math.radians(23.439 - 0.00000036 * d)
    ra = math.degrees(math.atan2(math.cos(e) * math.sin(L), math.cos(L)))
    dec = math.asin(math.sin(e) * math.sin(L))
    ha = math.radians((18.697374558 + 24.06570982441908 * d) * 15.0 + lon - ra)
    la = math.radians(lat)
    return math.degrees(math.asin(math.sin(la) * math.sin(dec) + math.cos(la) * math.cos(dec) * math.cos(ha)))


def tod_of(elev):
    return "day" if elev >= TOD_DAY_DEG else "night" if elev < TOD_NIGHT_DEG else "dusk"


def bag_latlon(bag):
    """GNSS 궤적의 측위된 점(status ≥ 0) 평균. 없으면 None."""
    try:
        track = json.loads((Path(bag) / "gnss_track.json").read_text(encoding="utf-8"))
        pts = [(f["lat"], f["lon"]) for fixes in track.values() for f in fixes
               if f.get("status", -1) >= 0 and -90 <= f["lat"] <= 90 and (f["lat"], f["lon"]) != (0, 0)]
        if pts:
            return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        pass
    return None


def split_tod(t0, dur, lat, lon):
    """[t0, t0+dur] 를 시간대별 초로 나눈다."""
    secs = dict.fromkeys(TOD, 0.0)
    t, end = t0, t0 + dur
    while t < end:
        step = min(TOD_STEP_SEC, end - t)
        secs[tod_of(sun_elevation(t + step / 2, lat, lon))] += step
        t += step
    return secs


def tod_label(secs):
    """가장 긴 시간대, 두 번째가 10% 넘게 섞였으면 'dusk+night' 처럼."""
    total = sum(secs.values())
    if total <= 0:
        return ""
    order = sorted(TOD, key=lambda k: -secs[k])
    return "+".join(k for k in order[:2] if secs[k] > 0 and (k == order[0] or secs[k] / total > 0.1))


def bag_row(bag, region, route):
    """bag 폴더 하나 → CSV 한 줄. metadata.yaml 이 없으면 (쓰는 중·깨짐) 그 사실을 비고에 남긴다."""
    info = read_info(bag)
    row = {k: "" for k, _ in COLUMNS}
    row.update(region=region, route=route, folder=str(bag),
               kind=KIND_CLIP if bag.name.startswith("clip_") else KIND_REC,
               region_name=info.get("region_name", "") or region,
               road_shapes=" · ".join(info.get("road_shapes") or []),
               driver=info.get("driver", ""), passenger=info.get("passenger", ""),
               label=info.get("label", ""), note=info.get("note", ""),
               # 지도·내비(Data Machine)의 루트를 따라 달리며 녹화한 bag — GUI 의 지도 연동 녹화가 적는다
               map_route=info.get("map_route", ""), map_run=info.get("map_run", ""),
               map_coverage=("" if info.get("map_coverage") is None else f"{float(info['map_coverage']) * 100:.0f}"))
    row["_excluded"] = bool(info.get("excluded"))     # 현황표에서 '유효한 리스트에서 지우기' 한 녹화
    size = sum(f.stat().st_size for f in bag.iterdir() if f.is_file())
    row["size_gb"] = f"{size / 1e9:.2f}"
    try:
        meta = yaml.load((bag / "metadata.yaml").read_text(encoding="utf-8"),
                         Loader=YamlLoader)["rosbag2_bagfile_information"]
        t0 = meta["starting_time"]["nanoseconds_since_epoch"] / 1e9
        dur = meta["duration"]["nanoseconds"] / 1e9
        start = dt.datetime.fromtimestamp(t0)
        end = dt.datetime.fromtimestamp(t0 + dur)
        row.update(date=start.strftime("%Y-%m-%d"), start_time=start.strftime("%H:%M:%S"),
                   end_time=end.strftime("%H:%M:%S"), duration_sec=f"{dur:.1f}",
                   duration_hms=hms(dur), messages=str(meta.get("message_count", "")))
        secs = split_tod(t0, dur, *(bag_latlon(bag) or DEFAULT_LATLON))
        row.update(time_of_day=tod_label(secs), **{f"{k}_sec": f"{v:.1f}" for k, v in secs.items()})
    except Exception as e:
        m = re.search(r"(\d{8})_(\d{6})", bag.name)     # 폴더 이름의 시각이라도
        if m:
            t = dt.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
            row.update(date=t.strftime("%Y-%m-%d"), start_time=t.strftime("%H:%M:%S"))
        row["duration_sec"] = "0"
        row["note"] = (row["note"] + " " if row["note"] else "") + \
            f"[metadata.yaml 읽기 실패 — 녹화 중이거나 깨진 bag: {type(e).__name__}]"
    return row


# 저장 위치 말고도 bag 을 옮겨 두는 곳 (다른 디스크 등) — 여기의 route_*/clip_*|rec_* 도 목록에 넣는다.
# 2026-09-29: 녹화 뒤 /mnt/data 로 옮긴 bag 이 '녹화 중' 모습(길이 0)으로 목록에 굳어 합계에서 빠졌다.
CATALOG_CFG = Path.home() / ".config" / "dm_clip_gui" / "catalog.yaml"
DEFAULT_EXTRA_ROOTS = ["/mnt/data"]


_BAG_KEY = re.compile(r"^((?:clip|rec)_\d{8}_\d{6})")


def bag_key(folder):
    """녹화 하나를 가리키는 이름 — 폴더 이름 앞의 clip_/rec_날짜_시각 (뒤에 붙인 메모는 무시)."""
    name = Path(folder).name
    m = _BAG_KEY.match(name)
    return m.group(1) if m else name


def scan_roots(base):
    try:
        extra = (yaml.safe_load(CATALOG_CFG.read_text(encoding="utf-8")) or {}).get("extra_roots")
    except FileNotFoundError:
        extra = None
    if extra is None:
        extra = DEFAULT_EXTRA_ROOTS
    roots, seen = [], set()
    for r in [base] + list(extra):
        p = Path(r).expanduser()
        if p.is_dir() and p.resolve() not in seen:
            seen.add(p.resolve())
            roots.append(p)
    return roots


def scan(base, excluded=False):
    """route_* 폴더 안의 clip_*/rec_* 만 모은다 (지역 없이 clips/ 바로 아래 있는 옛 클립은 뺀다).

    저장 위치(base)와 옮겨 두는 곳(catalog.yaml 의 extra_roots, 기본 /mnt/data)을 같이 본다.
    현황표에서 리스트에서 뺀 녹화(dataset_info.json 의 excluded)는 빠진다 — bag 파일은 그대로.
    excluded=True 면 거꾸로 뺀 것만 돌려준다 (되돌리기 목록).
    """
    rows, seen = [], set()
    for root in scan_roots(Path(base)):
        for route in sorted(root.iterdir()):
            m = ROUTE_RE.match(route.name)
            if not (route.is_dir() and m):
                continue
            try:
                bags = sorted(route.iterdir())
            except OSError:
                continue
            for bag in bags:
                if bag.is_dir() and bag.name.startswith(("clip_", "rec_")):
                    if bag_key(bag) in seen:          # 같은 녹화의 복사본이 다른 곳에도 있으면 한 번만 센다
                        continue
                    seen.add(bag_key(bag))
                    row = bag_row(bag, m.group(2), route.name)
                    if row.pop("_excluded") == excluded:
                        rows.append(row)
    rows.sort(key=lambda r: (r["date"], r["start_time"]))
    return rows


def to_csv(rows):
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=[k for k, _ in COLUMNS])
    w.writeheader()
    w.writerows(rows)
    return buf.getvalue()


# 한 번 목록에 들어온 녹화는 bag 폴더를 지우거나 옮겨도 목록에 남는다 — 모든 줄을 여기 보관한다.
# {폴더: {"row": {CSV 한 줄}, "excluded": bool, "excluded_at": ""}}. datasets.csv 는 (지금 있는 폴더) + (보관본) 이다.
ARCHIVE = "datasets_archive.json"
ON_DISK, OFF_DISK = "있음", "없음 (지움·옮김)"


def load_archive(base):
    try:
        return json.loads((Path(base) / ARCHIVE).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}


def save_archive(base, arch):
    path = Path(base) / ARCHIVE
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(arch, ensure_ascii=False, indent=1), encoding="utf-8")
    tmp.replace(path)


def _clean(row):
    return {k: ("" if row.get(k) is None else str(row.get(k))) for k, _ in COLUMNS}


def _read_csv(path):
    try:
        with open(path, encoding="utf-8-sig", newline="") as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []


def rebuild(base):
    """datasets.csv 를 다시 쓴다 (엑셀에서 한글이 깨지지 않게 UTF-8 BOM). 반환: (경로, 줄 수).

    지금 있는 bag 폴더를 훑은 줄 + 보관본(ARCHIVE)의 폴더 없는 줄. 폴더가 사라져도 줄은 남는다.
    """
    base = Path(base)
    with _lock:
        valid, excl = scan(base), scan(base, excluded=True)
        on_disk = {r["folder"] for r in valid + excl}
        arch = load_archive(base)
        # 옮겨진 bag: 보관본의 옛 경로와 이름(rec_…)이 같은 bag 이 다른 곳에 있으면 옛 기록을 버린다
        # (옛 기록은 녹화 중에 찍힌 모습일 수 있다 — 길이 0). 리스트에서 뺀 표시는 이어받는다.
        # 같은 녹화 = 폴더 이름 앞의 clip_/rec_날짜_시각 이 같음 (뒤에 '-카메라2죽음' 처럼 붙여 이름을 바꿔도 같은 것)
        here = {bag_key(f): f for f in on_disk}
        inherited = False
        for old in [f for f in arch if f not in on_disk and bag_key(f) in here]:
            if arch[old].get("excluded"):
                new = Path(here[bag_key(old)])
                if new.is_dir() and not read_info(new).get("excluded"):
                    write_info(new, excluded=True, excluded_at=arch[old].get("excluded_at", ""))
                    inherited = True
            del arch[old]
        if inherited:
            valid, excl = scan(base), scan(base, excluded=True)
        for r in _read_csv(base / CSV_NAME):          # 보관본이 생기기 전 목록에 있던 줄도 옮겨 둔다
            f = r.get("folder")
            if f and f not in on_disk and f not in arch and bag_key(f) not in here:   # 옮겨진 bag 은 새 위치로
                arch[f] = {"row": _clean(r), "excluded": False}
        for r in valid:
            arch[r["folder"]] = {"row": _clean(dict(r, on_disk=ON_DISK)), "excluded": False}
        for r in excl:
            arch[r["folder"]] = {"row": _clean(dict(r, on_disk=ON_DISK)), "excluded": True}
        rows = [dict(r, on_disk=ON_DISK) for r in valid]
        rows += [dict(e["row"], on_disk=OFF_DISK) for f, e in arch.items()
                 if f not in on_disk and not e.get("excluded")]
        rows.sort(key=lambda r: (r["date"], r["start_time"]))
        base.mkdir(parents=True, exist_ok=True)
        save_archive(base, arch)
        path = base / CSV_NAME
        tmp = path.with_suffix(".csv.tmp")
        tmp.write_text(to_csv(rows), encoding="utf-8-sig")
        tmp.replace(path)
    return path, len(rows)


def excluded_rows(base):
    """리스트에서 뺀 녹화 — 폴더가 있는 것 + 보관본에만 남은 것."""
    rows = scan(base, excluded=True)
    have = {r["folder"] for r in rows}
    rows += [dict(e["row"], on_disk=OFF_DISK) for f, e in load_archive(base).items()
             if e.get("excluded") and f not in have and not Path(f).is_dir()]
    return rows


def import_rows(base, rows):
    """다른 PC · 엑셀에서 가져온 줄을 보관본에 넣는다 (같은 폴더가 이미 있으면 건너뜀). 반환: 넣은 수."""
    with _lock:
        arch = load_archive(base)
        n = 0
        for r in rows:
            f = r.get("folder")
            if f and f not in arch:
                arch[f] = {"row": _clean(r), "excluded": False}
                n += 1
        save_archive(base, arch)
    rebuild(base)
    return n


def read_plans(base):
    path = Path(base) / PLAN_CSV
    if not path.is_file():
        return []
    with path.open(encoding="utf-8-sig", newline="") as f:
        rows = [{k: (r.get(k) or "") for k, _ in PLAN_COLUMNS} for r in csv.DictReader(f)]
    for r in rows:
        r["region_en"] = hangul_roman.region_to_english(r["region"])
    return rows


def write_plans(base, plans):
    keys = [k for k, _ in PLAN_COLUMNS]
    rows = [{k: str(p.get(k, "")).strip() for k in keys} for p in plans]
    for r in rows:
        r["region_en"] = hangul_roman.region_to_english(r["region"])   # 현황과 맞출 때 쓰는 이름
    for r in rows:
        if r["planned_hours"]:
            float(r["planned_hours"])                      # 숫자가 아니면 ValueError
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=keys)
    w.writeheader()
    w.writerows(rows)
    path = Path(base) / PLAN_CSV
    with _lock:
        Path(base).mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".csv.tmp")
        tmp.write_text(buf.getvalue(), encoding="utf-8-sig")
        tmp.replace(path)
    return rows


# ---------- 구글 시트 → 계획표 동기화 ----------
PLAN_HEADERS = {   # 시트 제목 줄 → 계획 열 (영문 키도 받는다, 날짜 등 나머지 열은 무시)
    "region": ("지역", "region"),
    "scene": ("장면 이름", "장면", "장면이름", "scene", "scene_name"),
    "route_desc": ("경로", "route_desc", "route"),
    "planned_hours": ("계획 시간", "planned_hours", "hours"),
    "note": ("비고", "note"),
}
_sync_status = {"ok_at": "", "err": "", "err_at": "", "rows": 0, "warn": []}
_sync_lock = threading.Lock()


def _plan_key(header):
    h = header.strip().lstrip("\ufeff")
    for key, names in PLAN_HEADERS.items():
        if any(h == n or h.startswith(n + "(") or h.startswith(n + " (") for n in names):
            return key
    return None


def parse_plan_sheet(text):
    """시트 CSV → (계획 목록, 경고). 지역 열이 없으면 ValueError (그때는 캐시를 그대로 둔다)."""
    reader = csv.DictReader(io.StringIO(text.lstrip("\ufeff")))
    cols = {h: _plan_key(h) for h in (reader.fieldnames or []) if h and _plan_key(h)}
    if "region" not in cols.values():
        raise ValueError("시트 첫 줄에서 '지역' 열을 찾지 못했습니다 — 제목 줄: 지역 | 장면 이름 | 경로 | 계획 시간(h) | 비고")
    plans, warn = [], []
    for n, r in enumerate(reader, start=2):
        p = {k: "" for k, _ in PLAN_COLUMNS}
        for h, k in cols.items():
            p[k] = (r.get(h) or "").strip()
        if not any(p.values()):
            continue                                             # 빈 줄
        if p["planned_hours"]:
            try:
                float(p["planned_hours"])
            except ValueError:
                warn.append(f"{n}번째 줄 계획 시간 '{p['planned_hours']}' 는 숫자가 아니라 비워 둠")
                p["planned_hours"] = ""
        plans.append(p)
    return plans, warn


def sync_plans(base):
    """구글 시트를 읽어 plans.csv 를 바꾼다. 실패하면 캐시는 그대로 두고 이유를 _sync_status 에 남긴다."""
    with _sync_lock:
        now = dt.datetime.now().isoformat(timespec="seconds")
        try:
            with urllib.request.urlopen(GSHEET_CSV_URL, timeout=10) as r:
                body = r.read().decode("utf-8", "replace")
            if body.lstrip().startswith("<"):
                raise ValueError("CSV 대신 웹 페이지가 왔습니다 — 시트 공유가 '링크가 있는 모든 사용자 · 보기' 인지 확인하세요")
            plans, warn = parse_plan_sheet(body)
            write_plans(base, plans)
            _sync_status.update(ok_at=now, err="", err_at="", rows=len(plans), warn=warn)
        except urllib.error.HTTPError as e:
            _sync_status.update(err=f"구글이 거절했습니다 (HTTP {e.code}) — 시트 공유 설정을 확인하세요", err_at=now)
        except (urllib.error.URLError, OSError) as e:
            _sync_status.update(err=f"오프라인 — 구글 시트에 연결할 수 없습니다 ({getattr(e, 'reason', e)})", err_at=now)
        except Exception as e:
            _sync_status.update(err=f"시트를 읽지 못했습니다: {e}", err_at=now)
        return dict(_sync_status)


def _sync_loop(base):
    while True:
        sync_plans(base)
        threading.Event().wait(PLAN_SYNC_SEC)


# ---------------- 브라우저 현황표 ----------------

class Handler(BaseHTTPRequestHandler):
    base = None
    goal_hours = DEFAULT_GOAL_HOURS
    scale_hours = DEFAULT_SCALE_HOURS

    def log_message(self, *_):
        pass

    def _send(self, code, body, ctype):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            page = PAGE.replace("__GOAL__", json.dumps(self.goal_hours)) \
                       .replace("__SCALE__", json.dumps(max(self.scale_hours, self.goal_hours))) \
                       .replace("__COLUMNS__", json.dumps(COLUMNS, ensure_ascii=False)) \
                       .replace("__PLAN_COLUMNS__", json.dumps(PLAN_COLUMNS, ensure_ascii=False)) \
                       .replace("__BASE__", json.dumps(str(self.base), ensure_ascii=False)) \
                       .replace("__GSHEET_EDIT__", json.dumps(GSHEET_EDIT_URL)) \
                       .replace("__ROAD_SHAPES__", json.dumps(ROAD_SHAPES, ensure_ascii=False))
            self._send(200, page, "text/html; charset=utf-8")
        elif path == "/" + CSV_NAME:
            try:
                p, _ = rebuild(self.base)          # 열 때마다 폴더 기준으로 새로
                self._send(200, p.read_bytes(), "text/csv; charset=utf-8")
            except Exception as e:
                self._send(500, f"CSV 생성 실패: {e}", "text/plain; charset=utf-8")
        elif path == "/api/plans":
            try:
                self._send(200, json.dumps({"plans": read_plans(self.base), "sync": dict(_sync_status),
                                            "every": PLAN_SYNC_SEC}, ensure_ascii=False),
                           "application/json; charset=utf-8")
            except Exception as e:
                self._send(500, f"계획표 읽기 실패: {e}", "text/plain; charset=utf-8")
        elif path == "/api/upload-script":
            self._send(200, json.dumps({"code": apps_script_code(), "configured": bool(upload_cfg().get("url")),
                                        "sheet": UPLOAD_SHEET_URL}, ensure_ascii=False),
                       "application/json; charset=utf-8")
        elif path == "/api/excluded":
            try:
                self._send(200, json.dumps(excluded_rows(self.base), ensure_ascii=False),
                           "application/json; charset=utf-8")
            except Exception as e:
                self._send(500, f"제외 목록 읽기 실패: {e}", "text/plain; charset=utf-8")
        elif path == "/" + PLAN_CSV:
            p = Path(self.base) / PLAN_CSV
            if not p.is_file():
                write_plans(self.base, [])
            self._send(200, p.read_bytes(), "text/csv; charset=utf-8")
        else:
            self._send(404, "not found", "text/plain")

    def _json(self, code, obj):
        self._send(code, json.dumps(obj, ensure_ascii=False), "application/json; charset=utf-8")

    def do_POST(self):
        if self.path == "/api/upload-config":
            try:
                req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                url = str(req.get("url", "")).strip()
                if not re.fullmatch(r"https://script\.google\.com/macros/s/[\w-]+/exec", url):
                    raise ValueError("https://script.google.com/macros/s/…/exec 형태의 웹 앱 URL 이어야 합니다")
                cfg = upload_cfg()
                cfg["url"] = url
                save_upload_cfg(cfg)
                return self._json(200, {"ok": True})
            except Exception as e:
                return self._json(400, {"ok": False, "error": str(e)})
        if self.path == "/api/upload":
            try:
                n = upload_to_gsheet(self.base)
                return self._json(200, {"ok": True, "n": n, "sheet": UPLOAD_SHEET_URL})
            except LookupError as e:
                return self._json(200, {"ok": False, "setup": True, "error": str(e)})
            except urllib.error.HTTPError as e:                    # 연결은 됐는데 구글이 거절
                why = ("웹 앱 '액세스 권한이 있는 사용자' 가 '모든 사용자' 가 아닙니다 — Apps Script 의 배포 ▸ 배포 관리 ▸ ✏️ 에서 "
                       "'모든 사용자' 로 바꾸고 버전 '새 버전' 으로 배포하세요 (회사·학교 계정은 이 옵션이 막혀 있을 수 있음)"
                       if e.code in (401, 403) else "웹 앱 URL 이 맞는지 확인하세요" if e.code == 404 else "")
                return self._json(502, {"ok": False, "error": f"구글이 거절했습니다 (HTTP {e.code} {e.reason}) — {why}"})
            except (urllib.error.URLError, OSError) as e:
                return self._json(502, {"ok": False, "error": f"구글에 연결할 수 없습니다 (인터넷 · PC 시계 확인): {getattr(e, 'reason', e)}"})
            except Exception as e:
                return self._json(502, {"ok": False, "error": f"업로드 실패: {e}"})
        if self.path == "/api/plans-sync":                       # [지금 가져오기]
            st = sync_plans(self.base)
            return self._json(200, {"plans": read_plans(self.base), "sync": st, "every": PLAN_SYNC_SEC})
        if self.path != "/api/edit":
            return self._send(404, "not found", "text/plain")
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            fields = {k: str(req[k]).strip() for k in EDITABLE if k in req}
            shapes = None
            if "road_shapes" in req:                               # 목록으로 받는다 — 모르는 이름은 거절
                shapes = [x for x in ROAD_SHAPES if x in list(req["road_shapes"] or [])]
                unknown = [x for x in (req["road_shapes"] or []) if x not in ROAD_SHAPES]
                if unknown:
                    raise ValueError(f"모르는 도로 형상: {unknown}")
            if "excluded" in req:                                  # 리스트에서 빼기 / 되돌리기
                fields["excluded"] = bool(req["excluded"])
                fields["excluded_at"] = dt.datetime.now().isoformat(timespec="seconds") if req["excluded"] else ""
            bag = Path(req["folder"]).resolve()
            with _lock:
                arch = load_archive(self.base)
            if bag.is_dir():
                # 저장 위치 안의 route_*/clip_*|rec_* 만 고칠 수 있다
                roots = {r.resolve() for r in scan_roots(Path(self.base))}   # 저장 위치 + 옮겨 두는 곳
                if bag.parent.parent not in roots or not ROUTE_RE.match(bag.parent.name):
                    raise ValueError("저장 위치 안의 녹화 폴더가 아닙니다")
                write_info(bag, **fields, **({"road_shapes": shapes} if shapes is not None else {}))
            elif req["folder"] in arch:                              # 폴더는 없고 목록에만 남은 줄
                with _lock:
                    arch = load_archive(self.base)
                    e = arch[req["folder"]]
                    e["row"].update({k: v for k, v in fields.items() if k in EDITABLE})
                    if shapes is not None:
                        e["row"]["road_shapes"] = " · ".join(shapes)
                    if "excluded" in fields:
                        e["excluded"], e["excluded_at"] = fields["excluded"], fields["excluded_at"]
                    save_archive(self.base, arch)
            else:
                raise ValueError("목록에 없는 녹화입니다")
            rebuild(self.base)
            self._send(200, json.dumps({"ok": True}), "application/json")
        except Exception as e:
            self._send(400, json.dumps({"ok": False, "error": str(e)}, ensure_ascii=False),
                       "application/json; charset=utf-8")


def make_server(base, port=DEFAULT_PORT, goal_hours=DEFAULT_GOAL_HOURS,
                scale_hours=DEFAULT_SCALE_HOURS):
    """localhost 전용 (차량 PC 밖으로 열지 않는다). 포트가 이미 쓰이면 OSError."""
    handler = type("H", (Handler,), {"base": Path(base), "goal_hours": goal_hours,
                                         "scale_hours": scale_hours})
    srv = ThreadingHTTPServer(("127.0.0.1", port), handler)
    threading.Thread(target=_sync_loop, args=(Path(base),), daemon=True).start()   # 계획표 ← 구글 시트
    return srv


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>데이터셋 취득 현황</title>
<style>
/* 파스텔 · 둥근 카드 · 이모지 — 외부 폰트/라이브러리 없이 (지하 오프라인에서도 같게 보인다) */
:root{--bg:#fff7ef;--dot:#f6e3d3;--card:#ffffff;--fg:#3b3350;--mute:#8a82a0;--line:#efe6f5;--acc:#7c8cff;--acc2:#ff8fb1;
  --ok:#34c38f;--warn:#ff9f43;--bad:#ff6b81;--bar:#f1ecf8;--day:#ffd166;--dusk:#ff9f7a;--night:#8e8cff;--shadow:0 6px 18px rgba(124,108,170,.12)}
@media (prefers-color-scheme:dark){:root{--bg:#1c1a26;--dot:#26233a;--card:#26233a;--fg:#ece8ff;--mute:#a39cc0;--line:#35314c;--acc:#9aa6ff;--acc2:#ff9fbf;
  --ok:#5ee0ae;--warn:#ffb86b;--bad:#ff8a9e;--bar:#35314c;--day:#ffd97a;--dusk:#ffae8f;--night:#a5a3ff;--shadow:0 6px 18px rgba(0,0,0,.35)}}
*{box-sizing:border-box}
body{margin:0;color:var(--fg);font:14px/1.5 "Pretendard","Noto Sans KR","Apple SD Gothic Neo",system-ui,sans-serif;
  background:var(--bg) radial-gradient(var(--dot) 1.2px,transparent 1.3px) 0 0/22px 22px}
main{max-width:1400px;margin:0 auto;padding:24px 16px 60px}
h1{font-size:26px;font-weight:800;margin:0 0 2px;letter-spacing:-.5px}
h1 .wave{display:inline-block;animation:drive 2.4s ease-in-out infinite}
@keyframes drive{0%,100%{transform:translateX(0)}50%{transform:translateX(6px) rotate(-3deg)}}
.sub{color:var(--mute);font-size:12px;margin-bottom:16px;word-break:break-all}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:14px;margin-bottom:18px}
.card{background:var(--card);border:2px solid var(--line);border-radius:20px;padding:16px 18px;box-shadow:var(--shadow);transition:transform .15s}
.card:hover{transform:translateY(-2px)}
.card .k{color:var(--mute);font-size:13px;font-weight:600}.card .k .e{font-size:17px;margin-right:4px}
.card .v{font-size:23px;font-weight:800;font-variant-numeric:tabular-nums;white-space:nowrap}.goal .v{font-size:19px;white-space:normal}
.goal{grid-column:1/-1;background:linear-gradient(135deg,color-mix(in srgb,var(--acc) 10%,var(--card)),color-mix(in srgb,var(--acc2) 10%,var(--card)))}
.bar{height:14px;background:var(--bar);border-radius:999px;overflow:hidden;margin-top:8px}
.bar>div{height:100%;background:linear-gradient(90deg,var(--acc),var(--acc2));border-radius:999px}
.gauge{position:relative;margin-top:34px}.gauge .bar{height:26px;margin:0;box-shadow:inset 0 2px 4px rgba(0,0,0,.06)}
.stack{display:flex}.stack>div{height:100%;border-radius:0}
.stack>div:first-child{border-radius:999px 0 0 999px}
.stack .day{background:repeating-linear-gradient(45deg,var(--day) 0 10px,color-mix(in srgb,var(--day) 80%,#fff) 10px 20px)}
.stack .dusk{background:repeating-linear-gradient(45deg,var(--dusk) 0 10px,color-mix(in srgb,var(--dusk) 80%,#fff) 10px 20px)}
.stack .night{background:repeating-linear-gradient(45deg,var(--night) 0 10px,color-mix(in srgb,var(--night) 80%,#fff) 10px 20px)}
.gauge .car{position:absolute;top:-30px;font-size:24px;transform:translateX(-50%) scaleX(-1);transition:left .6s cubic-bezier(.3,1.4,.5,1);filter:drop-shadow(0 2px 2px rgba(0,0,0,.15))}
.legend{display:flex;flex-wrap:wrap;gap:8px 12px;margin-top:10px;font-size:13px;font-variant-numeric:tabular-nums}
.legend span{background:var(--card);border:2px solid var(--line);border-radius:999px;padding:3px 12px}
.legend .pct{border-color:color-mix(in srgb,var(--warn) 55%,var(--line));background:color-mix(in srgb,var(--warn) 12%,var(--card))}
.legend i{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
.tod{display:inline-block;padding:2px 10px;border-radius:999px;font-size:12px;font-weight:700;color:#4a3b1f}
.tod.day{background:var(--day)}.tod.dusk{background:var(--dusk)}.tod.night{background:var(--night);color:#fff}
.gauge .mark{position:absolute;top:-6px;height:38px;border-left:3px dashed var(--warn)}
.gauge .mark span{position:absolute;top:-24px;left:4px;white-space:nowrap;font-size:12px;font-weight:800;color:var(--warn)}
.ticks{position:relative;height:16px;margin-top:6px;font-size:11px;color:var(--mute)}
.ticks span{position:absolute;transform:translateX(-50%)}.ticks span:first-child{transform:none}.ticks span:last-child{transform:translateX(-100%)}
.sec{background:var(--card);border:2px solid var(--line);border-radius:20px;padding:16px 18px;margin-bottom:18px;box-shadow:var(--shadow)}
.sec h2{font-size:16px;font-weight:800;margin:0 0 12px}
/* 지역 도넛 — 범주형 팔레트(검증된 기본 8색, 고정 순서). 9번째부터는 '기타' 로 묶는다 */
:root{--s1:#2a78d6;--s2:#eb6834;--s3:#1baf7a;--s4:#eda100;--s5:#e87ba4;--s6:#008300;--s7:#4a3aa7;--s8:#e34948;--sother:#9b98a8}
@media (prefers-color-scheme:dark){:root{--s1:#3987e5;--s2:#d95926;--s3:#199e70;--s4:#c98500;--s5:#d55181;--s6:#008300;--s7:#9085e9;--s8:#e66767;--sother:#7d7a8c}}
details.fold>summary{list-style:none;cursor:pointer;display:flex;align-items:center;gap:10px;user-select:none}
details.fold>summary::-webkit-details-marker{display:none}
details.fold>summary h2{margin:0}
details.fold>summary::before{content:"▶";font-size:11px;color:var(--acc);transition:transform .2s}
details.fold[open]>summary::before{transform:rotate(90deg)}
details.fold>summary .hint{font-size:12px;color:var(--mute)}details.fold[open]>summary .hint{display:none}
details.fold[open]>summary{margin-bottom:12px}
.setup{border:2px dashed color-mix(in srgb,#0f9d58 55%,var(--line));border-radius:16px;padding:12px 16px;margin-bottom:12px;background:color-mix(in srgb,#0f9d58 6%,var(--card))}
.setup[hidden]{display:none}.setup ol{margin:8px 0;padding-left:20px}.setup li{margin:4px 0}.setup a{color:var(--acc)}
.setup textarea{width:100%;height:150px;font:12px/1.4 ui-monospace,monospace;color:var(--fg);background:var(--bg);border:2px solid var(--line);border-radius:10px;padding:8px;margin:6px 0}
/* 필터 묶음과 버튼 묶음 — 창이 좁아도 버튼이 하나씩 떨어지지 않고 한 묶음으로 움직인다 */
.bar2{justify-content:space-between}.bar2 .grp{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.bar2 .acts{flex-wrap:nowrap;margin-left:auto}
@media (max-width:1100px){.bar2 .acts .lbl{display:none}.bar2 .acts button{padding:5px 10px}.bar2 .acts .gsb{padding:5px 12px}}
.exfold{margin-top:12px;border-top:2px dashed var(--line);padding-top:10px}
.exfold>summary{font-size:13px;color:var(--mute);font-weight:700}.exfold td{color:var(--mute)}
button.del{border:none;background:none;padding:2px 6px;color:var(--mute);font-size:14px}button.del:hover{color:var(--bad);transform:scale(1.2)}
button.undo{padding:2px 10px;font-size:12px;border-radius:999px}
.regwrap{display:grid;grid-template-columns:minmax(220px,300px) 1fr;gap:18px;align-items:start}
@media (max-width:760px){.regwrap{grid-template-columns:1fr}}
.donut-box{position:relative;margin:0;text-align:center}
#donut{width:100%;max-width:230px;display:block;margin:0 auto}
#donut path{stroke:var(--card);stroke-width:2;cursor:pointer;transition:opacity .12s,transform .15s;transform-origin:100px 100px}
#donut.hov path{opacity:.35}#donut.hov path.on{opacity:1;transform:scale(1.04)}
#donut .c1{font-size:22px;font-weight:800;fill:var(--fg)}#donut .c2{font-size:11px;fill:var(--mute)}
.donut-tip{position:absolute;pointer-events:none;background:var(--card);border:2px solid var(--line);border-radius:12px;padding:6px 10px;font-size:12px;box-shadow:var(--shadow);white-space:nowrap;text-align:left;transform:translate(-50%,-110%)}
.donut-tip[hidden]{display:none}
.dlegend{display:flex;flex-direction:column;gap:4px;margin-top:10px;font-size:13px;text-align:left;font-variant-numeric:tabular-nums}
.dlegend div{display:flex;align-items:center;gap:8px;padding:2px 8px;border-radius:10px;cursor:default}
.dlegend div.on{background:color-mix(in srgb,var(--acc) 10%,transparent)}
.dlegend i{width:11px;height:11px;border-radius:50%;flex:none}.dlegend .p{margin-left:auto;font-weight:800}
.regions{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px 22px}
.rg{background:color-mix(in srgb,var(--acc) 5%,var(--card));border-radius:16px;padding:10px 12px}
.rg .n{display:flex;flex-wrap:wrap;justify-content:space-between;gap:0 8px;font-variant-numeric:tabular-nums}
.rg .nm{white-space:nowrap}.rg .tt{margin-left:auto;white-space:nowrap}
.rg .sw{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}.rg .bar{height:10px;margin-top:6px}
.tools{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:12px}
select,input,button{font:inherit;color:var(--fg);background:var(--card);border:2px solid var(--line);border-radius:12px;padding:5px 11px}
select:focus,input:focus{outline:none;border-color:var(--acc)}
button{cursor:pointer;font-weight:600;transition:transform .12s,border-color .12s}button:hover{border-color:var(--acc);transform:translateY(-1px)}button:active{transform:scale(.97)}
button.primary{background:linear-gradient(135deg,var(--acc),var(--acc2));border:none;color:#fff;font-weight:800;padding:7px 18px;border-radius:999px;box-shadow:0 4px 10px color-mix(in srgb,var(--acc) 35%,transparent)}
button.primary:hover{filter:brightness(1.1)}
button.gsb{display:inline-flex;align-items:center;gap:7px;border-radius:999px;padding:5px 14px 5px 10px;
  background:color-mix(in srgb,#0f9d58 12%,var(--card));border-color:color-mix(in srgb,#0f9d58 45%,var(--line));font-weight:600}
button.gsb:hover{background:color-mix(in srgb,#0f9d58 22%,var(--card));border-color:#0f9d58;transform:translateY(-1px)}
button.gsb .gs{width:14px;height:18px;flex:none;transition:transform .15s}button.gsb:hover .gs{transform:rotate(-8deg) scale(1.1)}
.wrap{overflow-x:auto}table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}
th,td{border-bottom:1px dashed var(--line);padding:8px 9px;text-align:left;white-space:nowrap}
th{position:sticky;top:0;background:var(--card);cursor:pointer;font-size:12px;color:var(--mute);font-weight:700;border-bottom:2px solid var(--line)}
tbody tr{transition:background .12s}tbody tr:hover{background:color-mix(in srgb,var(--acc) 7%,transparent)}
td.num{text-align:right}td.ed{cursor:text;min-width:70px}td.ed:empty::after{content:"입력";color:var(--mute);opacity:.5}
td.path.gone{text-decoration:line-through;opacity:.7}
td.dbl{cursor:pointer}td.dbl:hover{background:color-mix(in srgb,var(--acc) 8%,transparent)}
td.shapes .chips{display:flex;flex-wrap:wrap;gap:4px;max-width:360px;white-space:normal}
td.shapes .chip{padding:2px 9px;font-size:12px;border-radius:999px}
td.shapes .chip.on{background:var(--acc);border-color:var(--acc);color:#fff}
td.shapes .chip.ok{border-color:var(--ok);color:var(--ok)}
td.ed:focus{outline:2px solid var(--acc)}td.path{color:var(--mute);font-size:11px}
tr.miss td.ed:not([data-k=note]):empty{background:color-mix(in srgb,var(--warn) 12%,transparent)}
.err{color:var(--bad);font-weight:600}.msg{color:var(--mute);font-size:12px}
.tabs{display:inline-flex;gap:4px;background:var(--card);border:2px solid var(--line);border-radius:999px;padding:4px;margin-bottom:18px;box-shadow:var(--shadow)}
.tabs button{border:none;border-radius:999px;background:none;padding:7px 18px;font-weight:700;color:var(--mute)}
.tabs button:hover{transform:none;color:var(--fg)}
.tabs button.on{color:#fff;background:linear-gradient(135deg,var(--acc),var(--acc2))}
.pane[hidden]{display:none}
#ptable td.route{white-space:normal;min-width:260px;max-width:460px}#ptable td.pnote{white-space:normal;min-width:200px;max-width:340px;font-size:13px;color:var(--mute)}
#ptable input{width:100%;min-width:80px;padding:4px 6px}#ptable input.wide{min-width:260px}#ptable input.reg{min-width:0;width:90px}#ptable input.scene{min-width:130px}#ptable input.hrs{min-width:70px;text-align:right}
#ptable input.bad{border-color:var(--bad)}
.st{display:inline-block;padding:2px 10px;border-radius:999px;font-size:12px;font-weight:700}
.st.plan{background:color-mix(in srgb,var(--mute) 18%,transparent)}.st.prog{background:color-mix(in srgb,var(--acc) 22%,transparent)}
.st.done{background:color-mix(in srgb,var(--ok) 22%,transparent)}.st.late{background:color-mix(in srgb,var(--bad) 20%,transparent)}
.mini{width:90px;height:8px;margin:0;display:inline-block;vertical-align:middle}
.x{border:none;background:none;color:var(--mute);padding:2px 6px}.x:hover{color:var(--bad);transform:rotate(90deg)}
</style></head><body><main>
<h1><span class="wave">🚗</span> 데이터셋 취득 현황</h1>
<div class="sub" id="src"></div>
<div class="tabs"><button data-tab="status" class="on">📊 현황</button><button data-tab="plan">🗓️ 계획표</button></div>
<div class="pane" id="pane-status">
<div class="cards">
  <div class="card goal"><div class="k" id="goalk"></div><div class="v" id="goalv"></div>
    <div class="gauge"><div class="car" id="goalcar">🚗</div><div class="bar stack" id="goalbar"></div><div class="mark" id="goalmark"><span></span></div></div>
    <div class="ticks" id="ticks"></div><div class="legend" id="todlegend"></div></div>
  <div class="card"><div class="k"><span class="e">🎥</span>수동녹화(Loop)</div><div class="v" id="c_rec"></div></div>
  <div class="card"><div class="k"><span class="e">🎬</span>Clip(30s)</div><div class="v" id="c_clip"></div></div>
  <div class="card"><div class="k"><span class="e">🗺️</span>지역 수</div><div class="v" id="c_reg"></div></div>
</div>
<details class="sec fold" id="regfold">
  <summary><h2>📍 지역별 취득 현황</h2><span class="hint">눌러서 열기</span></summary>
  <div class="regwrap">
    <figure class="donut-box" aria-label="지역별 취득 시간 비율">
      <svg id="donut" viewBox="0 0 200 200" role="img"></svg>
      <div class="donut-tip" id="donuttip" hidden></div>
      <figcaption id="donutlegend" class="dlegend"></figcaption>
    </figure>
    <div class="regions" id="regions"></div>
  </div>
</details>
<div class="sec">
  <div class="tools bar2">
    <div class="grp">
    <select id="f_region"><option value="">모든 지역</option></select>
    <select id="f_kind"><option value="">모든 종류</option><option>수동녹화(Loop)</option><option>Clip(30s)</option></select>
    <select id="f_tod"><option value="">모든 시간대</option><option>day</option><option>dusk</option><option>night</option></select>
    </div>
    <div class="grp acts">
    <button id="reload" title="새로 고침">🔄<span class="lbl"> 새로 고침</span></button>
    <a href="/datasets.csv" download="valid_dataset_list.csv"><button title="CSV 내려받기">⬇️<span class="lbl"> CSV 내려받기</span></button></a>
    <button type="button" id="gup" class="gsb" title="구글 시트에 업로드"><svg class="gs" viewBox="0 0 24 32" aria-hidden="true"><path d="M3 0h13l8 8v21a3 3 0 0 1-3 3H3a3 3 0 0 1-3-3V3a3 3 0 0 1 3-3z" fill="#0f9d58"/><path d="M16 0l8 8h-5a3 3 0 0 1-3-3z" fill="#87ceac"/><rect x="5" y="14" width="14" height="11" rx="1" fill="#fff"/><path d="M5 17.7h14M5 21.3h14M10.5 14v11" stroke="#0f9d58" stroke-width="1.4"/></svg><span class="lbl">구글 시트에 </span>업로드</button>
    </div>
  </div>
  <div class="msg" id="msg" style="margin:-4px 0 8px"></div>
  <div id="gsetup" class="setup" hidden>
    <b>구글 시트 자동 업로드 설정 (처음 한 번)</b>
    <ol>
      <li><a id="gsheetlink" target="_blank" rel="noopener">업로드할 구글 시트</a>에서 <b>확장 프로그램 ▸ Apps Script</b> → 원래 코드를 지우고 아래 코드를 붙여 넣고 💾 저장. <button type="button" id="gcopy">📋 코드 복사</button></li>
      <li>오른쪽 위 <b>배포 ▸ 새 배포</b> → 유형 선택 ⚙️ 에서 <b>웹 앱</b> → 다음 사용자 인증 정보로 실행: <b>나</b>, 액세스 권한이 있는 사용자: <b>모든 사용자</b> → <b>배포</b></li>
      <li><b>액세스 승인</b> → 내 구글 계정 선택 → "Google에서 확인하지 않은 앱" 화면이 나오면 왼쪽 아래 <b>고급</b> → <b>(프로젝트 이름)(으)로 이동(안전하지 않음)</b> → <b>허용</b>
        <span class="msg">— 내가 만든 스크립트라 구글 심사를 안 받았다는 뜻입니다. 이 시트에만 씁니다.</span></li>
      <li>나온 <b>웹 앱 URL</b> (…/exec) 을 복사해서 아래에 넣고 저장.</li>
    </ol>
    <textarea id="gcode" readonly spellcheck="false"></textarea>
    <div class="tools"><input id="gurl" placeholder="https://script.google.com/macros/s/…/exec" style="flex:1;min-width:280px">
      <button type="button" id="gsave" class="primary">저장하고 업로드</button><button type="button" id="gclose">닫기</button></div>
  </div>
  <div class="wrap"><table><thead id="thead"></thead><tbody id="tbody"></tbody></table></div>
  <details class="fold exfold" id="exfold" hidden>
    <summary><span id="exsum"></span></summary>
    <div class="wrap"><table><tbody id="exbody"></tbody></table></div>
  </details>
</div>
</div>
<div class="pane" id="pane-plan" hidden>
<div class="cards">
  <div class="card"><div class="k"><span class="e">📝</span>계획 합계 / 목표</div><div class="v" id="p_sum"></div></div>
  <div class="card"><div class="k"><span class="e">🎯</span>계획 대비 실제 취득</div><div class="v" id="p_act"></div></div>
  <div class="card"><div class="k"><span class="e">✅</span>완료 / 전체 계획</div><div class="v" id="p_done"></div></div>
</div>
<div class="sec">
  <div class="tools">
    <a id="pgslink" target="_blank" rel="noopener"><button type="button" class="gsb"><svg class="gs" viewBox="0 0 24 32" aria-hidden="true"><path d="M3 0h13l8 8v21a3 3 0 0 1-3 3H3a3 3 0 0 1-3-3V3a3 3 0 0 1 3-3z" fill="#0f9d58"/><path d="M16 0l8 8h-5a3 3 0 0 1-3-3z" fill="#87ceac"/><rect x="5" y="14" width="14" height="11" rx="1" fill="#fff"/><path d="M5 17.7h14M5 21.3h14M10.5 14v11" stroke="#0f9d58" stroke-width="1.4"/></svg>시트에서 고치기 ↗</button></a>
    <button type="button" id="psync">🔄 지금 가져오기</button>
    <a href="/plans.csv" download><button>⬇️ CSV 내려받기</button></a>
    <span class="msg" id="pmsg"></span>
  </div>
  <div class="wrap"><table id="ptable"><thead id="phead"></thead><tbody id="pbody"></tbody></table></div>
  <p class="msg">실제 취득 = 같은 지역에 저장된 녹화 시간 (같은 지역 계획이 여러 줄이면 위에서부터 채움). 지역은 한글로 써도 되고, 영문으로 바꾼 이름(폴더 이름)으로 GUI 녹화와 맞춥니다.
    계획은 구글 시트에서만 고칩니다 — 현황표가 1분마다 시트를 읽어 맞춥니다 (인터넷이 없으면 마지막으로 가져온 계획).</p>
</div>
</div>
</main>
<script>
const GOAL_H = __GOAL__, SCALE_H = __SCALE__, COLS = __COLUMNS__, PCOLS = __PLAN_COLUMNS__, BASE = __BASE__;
const SHOW = ["region_name","route","date","start_time","end_time","time_of_day","duration_hms","kind","road_shapes","map_route","map_coverage","driver","passenger","label","size_gb","note","folder"];
const EDIT = ["driver","passenger","note"];
let rows = [], live = true, sortKey = "date", sortDir = 1;
const $ = id => document.getElementById(id);
const TOD_EMOJI = {day: "☀️", dusk: "🌇", night: "🌙"};
const title = k => (COLS.find(c => c[0] === k) || [k, k])[1];
const hms = s => { s = Math.round(s); return `${Math.floor(s/3600)}:${String(Math.floor(s%3600/60)).padStart(2,"0")}:${String(s%60).padStart(2,"0")}`; };

function parseCSV(text) {
  text = text.replace(/^﻿/, "");
  const out = []; let row = [], f = "", q = false;
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (q) { if (c === '"') { if (text[i+1] === '"') { f += '"'; i++; } else q = false; } else f += c; }
    else if (c === '"') q = true;
    else if (c === ",") { row.push(f); f = ""; }
    else if (c === "\n" || c === "\r") { if (c === "\r" && text[i+1] === "\n") i++; row.push(f); out.push(row); row = []; f = ""; }
    else f += c;
  }
  if (f || row.length) { row.push(f); out.push(row); }
  const head = out.shift() || [];
  return out.filter(r => r.length > 1).map(r => Object.fromEntries(head.map((h, i) => [h, r[i] ?? ""])));
}

async function load() {
  try {
    const r = await fetch("/datasets.csv", {cache: "no-store"});
    if (!r.ok) throw new Error(await r.text());
    rows = parseCSV(await r.text()); live = true;
    loadExcluded();
    $("src").textContent = `${BASE}/datasets.csv — 폴더 기준으로 방금 다시 만든 목록 · ${new Date().toLocaleTimeString()}`;
    render();
  } catch (e) { $("msg").innerHTML = `<span class="err">불러오기 실패: ${e.message}</span>`; }
}

function render() {
  const fr = $("f_region").value, fk = $("f_kind").value, ft = $("f_tod").value;
  const sum = rs => rs.reduce((a, r) => a + (parseFloat(r.duration_sec) || 0), 0);
  // 목표는 GOAL_H(4h), 게이지 길이는 SCALE_H(6h) — 목표 지점은 주황 세로선
  const total = sum(rows), pct = total / (GOAL_H * 3600) * 100, left = GOAL_H * 3600 - total;
  $("goalk").textContent = `총 취득 시간 / 목표 ${GOAL_H}시간`;
  $("goalv").textContent = `${hms(total)} — ` +
    (left > 0 ? `남은 시간 ${hms(left)}` : `🎉 목표 달성! ${hms(-left)} 초과`);
  // 게이지는 시간대별로 쌓는다 (day · dusk · night)
  const TODS = ["day", "dusk", "night"], todSum = (rs, k) => rs.reduce((a, r) => a + (parseFloat(r[k + "_sec"]) || 0), 0);
  const stack = rs => TODS.map(k => `<div class="${k}" style="width:${todSum(rs, k) / (SCALE_H * 3600) * 100}%" title="${k} ${hms(todSum(rs, k))}"></div>`).join("");
  $("goalbar").innerHTML = stack(rows);
  $("goalcar").style.left = Math.max(1.5, Math.min(98.5, total / (SCALE_H * 3600) * 100)) + "%";   // 🚗 가 달린 만큼
  $("goalcar").textContent = left > 0 ? "🚗" : "🏎️";
  $("todlegend").innerHTML = TODS.map(k => { const v = todSum(rows, k);
    return `<span>${TOD_EMOJI[k]} <b>${k}</b> ${hms(v)} (${total ? (v / total * 100).toFixed(0) : 0}%)</span>`; }).join("") +
    `<span class="pct">🏁 목표 대비 <b>${pct.toFixed(1)}%</b></span>`;
  $("goalmark").style.left = (GOAL_H / SCALE_H * 100) + "%";
  $("goalmark").firstChild.textContent = `🏁 목표 ${GOAL_H}h`;
  $("ticks").innerHTML = Array.from({length: Math.floor(SCALE_H) + 1}, (_, h) =>
    `<span style="left:${h / SCALE_H * 100}%">${h}h</span>`).join("");
  const byKind = k => rows.filter(r => r.kind === k);
  $("c_rec").textContent = `${hms(sum(byKind("수동녹화(Loop)")))} · ${byKind("수동녹화(Loop)").length}개`;
  $("c_clip").textContent = `${hms(sum(byKind("Clip(30s)")))} · ${byKind("Clip(30s)").length}개`;
  const regions = [...new Set(rows.map(r => r.region))].sort();
  const nameOf = g => (rows.find(r => r.region === g && r.region_name) || {}).region_name || g;
  $("c_reg").textContent = regions.length;
  const colorOf = renderDonut(regions, nameOf, g => sum(rows.filter(r => r.region === g)));
  const maxR = Math.max(1, ...regions.map(g => sum(rows.filter(r => r.region === g))));
  $("regions").innerHTML = regions.map((g, gi) => {
    const rs = rows.filter(r => r.region === g), s = sum(rs), routes = new Set(rs.map(r => r.route)).size;
    const part = TODS.filter(k => todSum(rs, k) > 0).map(k => `${k} ${hms(todSum(rs, k))}`).join(" · ");
    return `<div class="rg"><div class="n"><span class="nm"><i class="sw" style="background:${colorOf[g] || "var(--sother)"}"></i><b title="폴더 이름: ${esc(g)}">${esc(nameOf(g))}</b></span><span class="tt">${hms(s)} · route ${routes}개 · ${rs.length}개</span></div>
            <div class="bar stack">${TODS.map(k => `<div class="${k}" style="width:${todSum(rs, k) / maxR * 100}%"></div>`).join("")}</div>
            <div class="msg">${part}</div></div>`;
  }).join("") || '<span class="msg">아직 route 폴더에 저장된 데이터가 없습니다</span>';
  const sel = $("f_region"), cur = sel.value;
  sel.innerHTML = '<option value="">모든 지역</option>' + regions.map(g => `<option value="${esc(g)}">${esc(nameOf(g))}</option>`).join("");
  sel.value = regions.includes(cur) ? cur : "";

  $("thead").innerHTML = "<tr><th></th>" + SHOW.map(k => `<th data-k="${k}">${title(k)}${k === sortKey ? (sortDir > 0 ? " ▲" : " ▼") : ""}</th>`).join("") + "</tr>";
  let vis = rows.filter(r => (!fr || r.region === fr) && (!fk || r.kind === fk) && (!ft || (r.time_of_day || "").split("+").includes(ft)));
  const num = k => ["duration_hms","size_gb"].includes(k);
  vis.sort((a, b) => {
    const x = sortKey === "duration_hms" ? +a.duration_sec : num(sortKey) ? +a[sortKey] : a[sortKey] + a.start_time;
    const y = sortKey === "duration_hms" ? +b.duration_sec : num(sortKey) ? +b[sortKey] : b[sortKey] + b.start_time;
    return (x > y ? 1 : x < y ? -1 : 0) * sortDir;
  });
  $("tbody").innerHTML = vis.map(r => `<tr class="${!r.driver || !r.passenger ? "miss" : ""}" data-f="${esc(r.folder)}"><td><button class="del" data-ex="1" title="유효한 리스트에서 지우기 (bag 파일은 그대로)">🗑️</button></td>` + SHOW.map(k =>
    EDIT.includes(k) && live ? `<td class="ed" contenteditable="plaintext-only" data-k="${k}">${esc(r[k])}</td>`
    : k === "label" && live ? `<td class="dbl" data-k="label" title="더블클릭해서 수정">${esc(r.label)}</td>`
    : k === "road_shapes" && live ? `<td class="dbl shapes" data-k="road_shapes" title="더블클릭해서 수정">${esc(r.road_shapes)}</td>`
    : k === "region_name" ? `<td title="${esc(r.region)}">${esc(r.region_name || r.region)}</td>`
    : k === "folder" ? `<td class="path${r.on_disk && r.on_disk !== "있음" ? " gone" : ""}" title="${r.on_disk && r.on_disk !== "있음" ? "bag 폴더 없음 (지움·옮김) — 목록에는 남겨 둡니다" : esc(r.folder)}">${esc(r.folder.split("/").slice(-2).join("/"))}</td>`
    : k === "time_of_day" ? `<td>${(r.time_of_day || "").split("+").filter(Boolean).map(t => `<span class="tod ${t}">${TOD_EMOJI[t]} ${t}</span>`).join(" ")}</td>`
    : `<td class="${num(k) ? "num" : ""}">${esc(r[k])}</td>`).join("") + "</tr>").join("");
  $("msg").textContent = `${vis.length}개 표시` + (live ? " · 운전자/동승자/비고는 눌러서, 라벨·도로 형상은 더블클릭해서 고칩니다" : " · 불러온 CSV (읽기 전용)");
  if (!$("pbody").contains(document.activeElement)) renderPlan(); else renderPlanStats();
}

// ---------- 계획표 ----------
let plans = [];
// 실제 취득: 같은 지역에서 찍은 시간을 위 계획부터 차례로 채운다 (같은 지역 계획이 여러 줄이면 나눠 갖는다)
function planActuals() {
  const left = {};
  rows.forEach(r => { left[r.region] = (left[r.region] || 0) + (parseFloat(r.duration_sec) || 0); });
  return plans.map(p => {
    const have = left[p.region_en] || 0, goal = (parseFloat(p.planned_hours) || 0) * 3600;
    const take = goal ? Math.min(have, goal) : have;
    left[p.region_en] = have - take;
    return take;
  });
}
function planStatus(p, act) {
  const goal = (parseFloat(p.planned_hours) || 0) * 3600;
  if (goal && act >= goal) return ["done", "완료"];
  return act > 0 ? ["prog", "진행 중"] : ["plan", "예정"];
}
// 계획은 구글 시트가 원본 — 서버가 1분마다 가져온다. 여기서는 보기만 (추가·수정·삭제 없음).
let planSync = {}, planEvery = 60;
function showSync() {
  const t = iso => iso ? iso.slice(11, 19) : "";
  const st = planSync;
  let h = st.err
    ? `<span class="err">🔴 ${esc(st.err)}</span> · ${st.ok_at ? `${t(st.ok_at)} 에 가져온 계획을 보여 줍니다` : "아직 한 번도 못 가져왔습니다"}`
    : st.ok_at ? `🟢 ${t(st.ok_at)} 구글 시트에서 가져옴 · ${planEvery}초마다 자동` : "구글 시트 가져오는 중…";
  if (st.warn && st.warn.length) h += ` <span style="color:var(--warn)">· ⚠️ ${esc(st.warn.slice(0, 3).join(", "))}</span>`;
  $("pmsg").innerHTML = h;
}
function takePlans(j) {
  plans = j.plans; planSync = j.sync || {}; planEvery = j.every || 60;
  renderPlan(); showSync();
}
async function loadPlans() {
  try {
    const r = await fetch("/api/plans", {cache: "no-store"});
    if (!r.ok) throw new Error(await r.text());
    takePlans(await r.json());
  } catch (e) { $("pmsg").innerHTML = `<span class="err">계획표 불러오기 실패: ${esc(e.message)}</span>`; }
}
function renderPlan() {
  const SHOWN = PCOLS.filter(c => c[0] !== "region_en");   // 지역(영문)은 숨김 — 지역에 마우스를 올리면 보인다
  $("phead").innerHTML = "<tr>" + SHOWN.map(c => `<th>${c[1]}</th>`).join("") + "<th>실제 취득</th><th>달성</th><th>상태</th></tr>";
  $("pbody").innerHTML = plans.map((p, i) => "<tr>" + SHOWN.map(c =>
      c[0] === "region" ? `<td title="${p.region_en ? "폴더 이름: " + esc(p.region_en) : ""}"><b>${esc(p.region)}</b></td>`
      : c[0] === "planned_hours" ? `<td class="num">${esc(p.planned_hours)}</td>`
      : c[0] === "route_desc" ? `<td class="route">${esc(p.route_desc)}</td>`
      : c[0] === "note" ? `<td class="pnote">${esc(p.note)}</td>`
      : `<td>${esc(p[c[0]])}</td>`).join("") +
    `<td class="num" id="pa${i}"></td><td id="pb${i}"></td><td id="ps${i}"></td></tr>`).join("")
    || `<tr><td colspan="${SHOWN.length + 3}" class="msg">계획이 없습니다 — [시트에서 고치기] 로 구글 시트에 적어 주세요</td></tr>`;
  renderPlanStats();
}
function renderPlanStats() {
  let planned = 0, actual = 0, done = 0;
  const acts = planActuals();
  plans.forEach((p, i) => {
    const act = acts[i], goal = (parseFloat(p.planned_hours) || 0) * 3600, [cls, txt] = planStatus(p, act);
    planned += goal; actual += Math.min(act, goal || act); if (cls === "done") done++;
    if (!$("pa" + i)) return;
    $("pa" + i).textContent = hms(act);
    $("pb" + i).innerHTML = goal ? `<div class="bar mini"><div style="width:${Math.min(100, act / goal * 100)}%"></div></div> ${Math.round(act / goal * 100)}%` : "";
    $("ps" + i).innerHTML = `<span class="st ${cls}">${txt}</span>`;
  });
  $("p_sum").innerHTML = `${hms(planned)} / ${GOAL_H}h` + (planned < GOAL_H * 3600 ? ` <span style="color:var(--warn);font-size:13px">(${hms(GOAL_H * 3600 - planned)} 부족)</span>` : "");
  $("p_act").textContent = planned ? `${hms(actual)} (${(actual / planned * 100).toFixed(0)}%)` : "-";
  $("p_done").textContent = `${done} / ${plans.length}`;
}
$("pgslink").href = __GSHEET_EDIT__;
$("psync").onclick = async () => {
  $("pmsg").textContent = "구글 시트 가져오는 중…";
  try { takePlans(await (await fetch("/api/plans-sync", {method: "POST"})).json()); }
  catch (e) { $("pmsg").innerHTML = `<span class="err">${esc(e.message)}</span>`; }
};
setInterval(() => { if (!$("pane-plan").hidden) loadPlans(); }, 30000);
document.querySelectorAll(".tabs button").forEach(b => b.onclick = () => {
  document.querySelectorAll(".tabs button").forEach(x => x.classList.toggle("on", x === b));
  $("pane-status").hidden = b.dataset.tab !== "status"; $("pane-plan").hidden = b.dataset.tab !== "plan";
  if (b.dataset.tab === "plan") loadPlans();
  try { localStorage.setItem("tab", b.dataset.tab); } catch (e) {}
});
// 지역별 취득 현황 — 처음엔 닫혀 있고, 열고 닫은 상태는 이 브라우저에 기억한다
try { if (localStorage.getItem("regfold") === "open") $("regfold").open = true; } catch (e) {}
$("regfold").addEventListener("toggle", () => { try { localStorage.setItem("regfold", $("regfold").open ? "open" : "closed"); } catch (e) {} });
try { const t = location.hash.slice(1) || localStorage.getItem("tab"); if (t) document.querySelector(`.tabs button[data-tab="${t}"]`)?.click(); } catch (e) {}

// ---------- 지역별 도넛 ----------
// 색은 지역(영문 이름) 순서로 고정 — 필터를 바꿔도 지역 색이 바뀌지 않는다. 8개 넘으면 작은 것부터 '기타'.
function renderDonut(regions, nameOf, secOf) {
  let items = regions.map((g, i) => ({g, name: nameOf(g), v: secOf(g), color: `var(--s${i + 1})`}));
  if (items.length > 8) {
    const keep = [...items].sort((a, b) => b.v - a.v).slice(0, 7).map(x => x.g);
    const rest = items.filter(x => !keep.includes(x.g));
    items = items.filter(x => keep.includes(x.g))
      .concat([{g: "__other", name: `기타 (${rest.length}곳)`, v: rest.reduce((a, x) => a + x.v, 0), color: "var(--sother)"}]);
  }
  const total = items.reduce((a, x) => a + x.v, 0), svg = $("donut");
  const colorOf = Object.fromEntries(items.map(x => [x.g, x.color]));   // 지역 카드도 같은 색
  if (!total) { svg.innerHTML = `<circle cx="100" cy="100" r="70" fill="none" stroke="var(--bar)" stroke-width="30"/><text x="100" y="105" text-anchor="middle" class="c2">데이터 없음</text>`; $("donutlegend").innerHTML = ""; return colorOf; }
  const R = 85, r = 55, pt = (a, rad) => [100 + rad * Math.sin(a), 100 - rad * Math.cos(a)];
  let a0 = 0;
  const paths = items.filter(x => x.v > 0).map(x => {
    const frac = x.v / total, a1 = a0 + frac * 2 * Math.PI;
    let d;
    if (frac >= 0.9999) {           // 한 지역뿐 — 꽉 찬 고리
      d = `M100 ${100 - R}A${R} ${R} 0 1 1 99.99 ${100 - R}ZM100 ${100 - r}A${r} ${r} 0 1 0 100.01 ${100 - r}Z`;
    } else {
      const big = a1 - a0 > Math.PI ? 1 : 0, [x0, y0] = pt(a0, R), [x1, y1] = pt(a1, R), [x2, y2] = pt(a1, r), [x3, y3] = pt(a0, r);
      d = `M${x0} ${y0}A${R} ${R} 0 ${big} 1 ${x1} ${y1}L${x2} ${y2}A${r} ${r} 0 ${big} 0 ${x3} ${y3}Z`;
    }
    a0 = a1;
    return `<path d="${d}" fill="${x.color}" fill-rule="evenodd" data-g="${esc(x.g)}"></path>`;
  }).join("");
  svg.innerHTML = paths + `<text x="100" y="100" text-anchor="middle" class="c1">${hms(total)}</text>` +
                  `<text x="100" y="118" text-anchor="middle" class="c2">지역 ${regions.length}곳</text>`;
  $("donutlegend").innerHTML = items.map(x =>
    `<div data-g="${esc(x.g)}"><i style="background:${x.color}"></i>${esc(x.name)}<span class="msg">${hms(x.v)}</span><span class="p">${(x.v / total * 100).toFixed(1)}%</span></div>`).join("");
  const byG = Object.fromEntries(items.map(x => [x.g, x]));
  const hi = g => {
    svg.classList.toggle("hov", !!g);
    svg.querySelectorAll("path").forEach(p => p.classList.toggle("on", p.dataset.g === g));
    $("donutlegend").querySelectorAll("div").forEach(d => d.classList.toggle("on", d.dataset.g === g));
  };
  svg.onmousemove = e => {
    const g = e.target.dataset && e.target.dataset.g, tip = $("donuttip");
    if (!g) { hi(null); tip.hidden = true; return; }
    const x = byG[g], box = svg.parentElement.getBoundingClientRect();
    hi(g); tip.hidden = false;
    tip.innerHTML = `<b>${esc(x.name)}</b><br>${hms(x.v)} · ${(x.v / total * 100).toFixed(1)}%`;
    tip.style.left = (e.clientX - box.left) + "px"; tip.style.top = (e.clientY - box.top) + "px";
  };
  svg.onmouseleave = () => { hi(null); $("donuttip").hidden = true; };
  $("donutlegend").onmouseover = e => { const d = e.target.closest("[data-g]"); hi(d ? d.dataset.g : null); };
  $("donutlegend").onmouseleave = () => hi(null);
  return colorOf;
}

function esc(s) { return String(s ?? "").replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c])); }

$("thead").addEventListener("click", e => {
  const k = e.target.dataset.k; if (!k) return;
  sortDir = k === sortKey ? -sortDir : 1; sortKey = k; render();
});
$("tbody").addEventListener("keydown", e => { if (e.key === "Enter") { e.preventDefault(); e.target.blur(); } });
// 라벨 · 도로 형상: 잘못 눌러 바뀌지 않게 더블클릭으로만 연다
const ROAD_SHAPES = __ROAD_SHAPES__;
$("tbody").addEventListener("dblclick", e => {
  const td = e.target.closest("td.dbl"); if (!td || td.classList.contains("editing")) return;
  const folder = td.parentElement.dataset.f, row = rows.find(r => r.folder === folder); if (!row) return;
  if (td.dataset.k === "label") {           // 글자 수정 → 기존 칸 저장(focusout)으로
    td.classList.add("ed"); td.contentEditable = "plaintext-only"; td.focus();
    const sel = getSelection(); sel.selectAllChildren(td);
    return;
  }
  // 도로 형상: 칸 안에 버튼 6개 — 누르면 켜고 끄고, 칸 밖을 누르거나 Enter 면 저장
  const cur = new Set((row.road_shapes || "").split(" · ").filter(Boolean));
  td.classList.add("editing");
  td.innerHTML = `<div class="chips">${ROAD_SHAPES.map(s => `<button type="button" class="chip${cur.has(s) ? " on" : ""}">${esc(s)}</button>`).join("")}
    <button type="button" class="chip ok">✔ 저장</button></div>`;
  const save = async () => {
    document.removeEventListener("mousedown", outside, true);
    const pick = [...td.querySelectorAll(".chip.on")].map(b => b.textContent);
    const joined = pick.join(" · ");
    if (joined === (row.road_shapes || "")) { render(); return; }
    const j = await (await fetch("/api/edit", {method: "POST", headers: {"Content-Type": "application/json"},
                                          body: JSON.stringify({folder, road_shapes: pick})})).json();
    if (j.ok) { row.road_shapes = joined; $("msg").textContent = `저장됨: ${folder.split("/").pop()} 도로 형상 = ${joined || "(없음)"}`; }
    else $("msg").innerHTML = `<span class="err">저장 실패: ${esc(j.error)}</span>`;
    render();
  };
  const outside = ev => { if (!td.contains(ev.target)) save(); };
  td.querySelectorAll(".chip").forEach(b => b.onclick = ev => {
    ev.stopPropagation();
    if (b.classList.contains("ok")) save(); else b.classList.toggle("on");
  });
  setTimeout(() => document.addEventListener("mousedown", outside, true), 0);
});
$("tbody").addEventListener("focusout", async e => {
  const td = e.target; if (!td.classList.contains("ed")) return;
  const folder = td.parentElement.dataset.f, k = td.dataset.k, v = td.textContent.trim();
  const row = rows.find(r => r.folder === folder); if (!row || row[k] === v) return;
  const r = await fetch("/api/edit", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({folder, [k]: v})});
  const j = await r.json();
  if (j.ok) { row[k] = v; $("msg").textContent = `저장됨: ${folder.split("/").pop()} ${title(k)} = ${v || "(빈 칸)"}`; render(); }
  else { $("msg").innerHTML = `<span class="err">저장 실패: ${esc(j.error)}</span>`; td.textContent = row[k]; }
});
// ---------- 구글 시트에 업로드 (유효한 리스트 = 리스트에서 뺀 녹화 제외) ----------
async function showSetup() {
  const j = await (await fetch("/api/upload-script", {cache: "no-store"})).json();
  $("gcode").value = j.code; $("gsheetlink").href = j.sheet; $("gsetup").hidden = false;
}
async function upload() {
  $("msg").textContent = "구글 시트에 올리는 중…";
  try {
    const j = await (await fetch("/api/upload", {method: "POST"})).json();
    if (j.setup) { $("msg").innerHTML = `<span class="err">${esc(j.error)} — 아래 설정을 한 번 해 주세요</span>`; showSetup(); return; }
    if (!j.ok) throw new Error(j.error);
    $("gsetup").hidden = true;
    $("msg").innerHTML = `✅ 구글 시트에 ${j.n}개 올림 · ${new Date().toLocaleTimeString()} · <a href="${esc(j.sheet)}" target="_blank" rel="noopener">시트 열기 ↗</a>`;
  } catch (e) { $("msg").innerHTML = `<span class="err">${esc(e.message)}</span>`; showSetup(); }
}
$("gup").onclick = upload;
$("gclose").onclick = () => { $("gsetup").hidden = true; };
$("gcopy").onclick = async () => {
  try { await navigator.clipboard.writeText($("gcode").value); }
  catch (e) { $("gcode").select(); document.execCommand("copy"); }
  $("gcopy").textContent = "복사됨 ✓";
};
$("gsave").onclick = async () => {
  const j = await (await fetch("/api/upload-config", {method: "POST", headers: {"Content-Type": "application/json"},
                                                   body: JSON.stringify({url: $("gurl").value})})).json();
  if (!j.ok) { $("msg").innerHTML = `<span class="err">${esc(j.error)}</span>`; return; }
  upload();
};

// ---------- 유효한 리스트에서 지우기 / 되돌리기 (bag 파일은 지우지 않는다) ----------
async function setExcluded(folder, ex) {
  const r = await fetch("/api/edit", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({folder, excluded: ex})});
  const j = await r.json();
  if (!j.ok) { $("msg").innerHTML = `<span class="err">${ex ? "지우기" : "되돌리기"} 실패: ${esc(j.error)}</span>`; return; }
  await load();
  $("msg").textContent = `${folder.split("/").pop()} — ${ex ? "유효한 리스트에서 지웠습니다 (bag 파일은 그대로)" : "리스트로 되돌렸습니다"}`;
}
$("tbody").addEventListener("click", e => {
  const b = e.target.closest("button.del"); if (!b) return;
  const folder = b.closest("tr").dataset.f, r = rows.find(x => x.folder === folder) || {};
  if (!confirm(`유효한 리스트에서 지울까요?\n\n${r.region_name || r.region} · ${r.date} ${r.start_time} · ${r.kind} · ${r.duration_hms}\n${folder}\n\nbag 파일은 지우지 않고, 목록과 합계에서만 빠집니다. (아래 '리스트에서 뺀 항목' 에서 되돌릴 수 있음)`)) return;
  setExcluded(folder, true);
});
async function loadExcluded() {
  try {
    const ex = await (await fetch("/api/excluded", {cache: "no-store"})).json();
    $("exfold").hidden = !ex.length;
    $("exsum").textContent = `🚫 리스트에서 뺀 항목 ${ex.length}개`;
    $("exbody").innerHTML = ex.map(r => `<tr data-f="${esc(r.folder)}"><td><button class="undo">↩️ 되돌리기</button></td><td>${esc(r.region_name || r.region)}</td><td>${esc(r.date)}</td><td>${esc(r.start_time)}</td><td>${esc(r.kind)}</td><td class="num">${esc(r.duration_hms)}</td><td class="path">${esc(r.folder.split("/").slice(-2).join("/"))}</td></tr>`).join("");
  } catch (e) { $("msg").innerHTML = `<span class="err">제외 목록 읽기 실패: ${esc(e.message)}</span>`; }
}
$("exbody").addEventListener("click", e => {
  const b = e.target.closest("button.undo"); if (b) setExcluded(b.closest("tr").dataset.f, false);
});

$("f_region").onchange = $("f_kind").onchange = $("f_tod").onchange = render;
$("reload").onclick = load;
loadPlans(); load(); setInterval(() => { if (live && !document.activeElement.classList.contains("ed")) load(); }, 30000);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description="지역별 데이터셋 CSV · 취득 현황표")
    ap.add_argument("cmd", choices=["rebuild", "serve"])
    ap.add_argument("--base", default=None, help="저장 위치 (기본: GUI 설정의 output_dir)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--goal-hours", type=float, default=DEFAULT_GOAL_HOURS)
    ap.add_argument("--scale-hours", type=float, default=DEFAULT_SCALE_HOURS, help="게이지 전체 길이")
    a = ap.parse_args()
    base = Path(a.base).expanduser() if a.base else default_base()
    path, n = rebuild(base)
    print(f"{path} — {n}개")
    if a.cmd == "serve":
        try:
            srv = make_server(base, a.port, a.goal_hours, a.scale_hours)
        except OSError as e:
            sys.exit(f"포트 {a.port} 를 열 수 없음 ({e}) — 이미 현황표 서버가 떠 있으면 "
                     f"http://localhost:{a.port} 를 여세요")
        print(f"취득 현황표: http://localhost:{a.port}  (Ctrl+C 로 종료)")
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
