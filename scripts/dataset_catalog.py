#!/usr/bin/env python3
# dataset_catalog.py — 지역별 데이터셋 목록(CSV) + 브라우저 취득 현황표.
#
# 기준은 폴더다: <저장 위치>/route_NNN_<지역>/{clip_*,rec_*}/ 를 훑어
#   - metadata.yaml (rosbag2)  → 날짜 · 시작/끝 시각 · bag 길이 · 메시지 수
#   - dataset_info.json (GUI)  → 운전자 · 동승자 · 라벨 · 비고
# 를 모아 <저장 위치>/datasets.csv 를 새로 쓴다. 폴더를 지우거나 옮겨도 다시 만들면 맞는다.
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
import sys
import threading
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
EDITABLE = ("driver", "passenger", "note")
KIND_REC, KIND_CLIP = "수동녹화(GS)", "클립(TH)"

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
    ("kind", "종류"), ("driver", "운전자"), ("passenger", "동승자"),
    ("label", "라벨"), ("messages", "메시지 수"), ("size_gb", "용량(GB)"),
    ("note", "비고"), ("folder", "폴더"),
]
_lock = threading.Lock()

# 계획표 — <저장 위치>/plans.csv. 브라우저의 [계획표] 탭에서 쓰고 고친다.
PLAN_CSV = "plans.csv"
PLAN_COLUMNS = [
    ("date", "날짜"), ("region", "지역"), ("region_en", "지역(영문)"), ("route_desc", "경로 (출발 → 경유 → 도착)"),
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
               driver=info.get("driver", ""), passenger=info.get("passenger", ""),
               label=info.get("label", ""), note=info.get("note", ""))
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


def scan(base):
    """route_* 폴더 안의 clip_*/rec_* 만 모은다 (지역 없이 clips/ 바로 아래 있는 옛 클립은 뺀다)."""
    base = Path(base)
    rows = []
    if not base.is_dir():
        return rows
    for route in sorted(base.iterdir()):
        m = ROUTE_RE.match(route.name)
        if not (route.is_dir() and m):
            continue
        for bag in sorted(route.iterdir()):
            if bag.is_dir() and bag.name.startswith(("clip_", "rec_")):
                rows.append(bag_row(bag, m.group(2), route.name))
    rows.sort(key=lambda r: (r["date"], r["start_time"]))
    return rows


def to_csv(rows):
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=[k for k, _ in COLUMNS])
    w.writeheader()
    w.writerows(rows)
    return buf.getvalue()


def rebuild(base):
    """datasets.csv 를 다시 쓴다 (엑셀에서 한글이 깨지지 않게 UTF-8 BOM). 반환: (경로, 줄 수)."""
    base = Path(base)
    with _lock:
        rows = scan(base)
        base.mkdir(parents=True, exist_ok=True)
        path = base / CSV_NAME
        tmp = path.with_suffix(".csv.tmp")
        tmp.write_text(to_csv(rows), encoding="utf-8-sig")
        tmp.replace(path)
    return path, len(rows)


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
                       .replace("__BASE__", json.dumps(str(self.base), ensure_ascii=False))
            self._send(200, page, "text/html; charset=utf-8")
        elif path == "/" + CSV_NAME:
            try:
                p, _ = rebuild(self.base)          # 열 때마다 폴더 기준으로 새로
                self._send(200, p.read_bytes(), "text/csv; charset=utf-8")
            except Exception as e:
                self._send(500, f"CSV 생성 실패: {e}", "text/plain; charset=utf-8")
        elif path == "/api/plans":
            try:
                self._send(200, json.dumps(read_plans(self.base), ensure_ascii=False),
                           "application/json; charset=utf-8")
            except Exception as e:
                self._send(500, f"계획표 읽기 실패: {e}", "text/plain; charset=utf-8")
        elif path == "/" + PLAN_CSV:
            p = Path(self.base) / PLAN_CSV
            if not p.is_file():
                write_plans(self.base, [])
            self._send(200, p.read_bytes(), "text/csv; charset=utf-8")
        else:
            self._send(404, "not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/plans":
            try:
                req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                rows = write_plans(self.base, req["plans"])
                return self._send(200, json.dumps({"ok": True, "n": len(rows),
                                                   "region_en": [r["region_en"] for r in rows]}),
                                  "application/json")
            except Exception as e:
                return self._send(400, json.dumps({"ok": False, "error": f"계획표 저장 실패: {e}"},
                                                  ensure_ascii=False), "application/json; charset=utf-8")
        if self.path != "/api/edit":
            return self._send(404, "not found", "text/plain")
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            bag = Path(req["folder"]).resolve()
            # 저장 위치 안의 route_*/clip_*|rec_* 만 고칠 수 있다
            if bag.parent.parent != Path(self.base).resolve() or not ROUTE_RE.match(bag.parent.name) \
                    or not bag.is_dir():
                raise ValueError("저장 위치 안의 녹화 폴더가 아닙니다")
            write_info(bag, **{k: str(req[k]).strip() for k in EDITABLE if k in req})
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
    return ThreadingHTTPServer(("127.0.0.1", port), handler)


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>데이터셋 취득 현황</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--fg:#111827;--mute:#6b7280;--line:#e5e7eb;--acc:#2563eb;--ok:#16a34a;--warn:#d97706;--bad:#dc2626;--bar:#e5e7eb;--day:#eab308;--dusk:#f97316;--night:#4f46e5}
@media (prefers-color-scheme:dark){:root{--bg:#0f1115;--card:#171a21;--fg:#e5e7eb;--mute:#9ca3af;--line:#2a2f3a;--acc:#60a5fa;--ok:#4ade80;--warn:#fbbf24;--bad:#f87171;--bar:#2a2f3a;--day:#facc15;--dusk:#fb923c;--night:#818cf8}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,"Noto Sans KR",sans-serif}
main{max-width:1400px;margin:0 auto;padding:20px 16px 60px}
h1{font-size:22px;margin:0 0 4px}.sub{color:var(--mute);font-size:12px;margin-bottom:16px;word-break:break-all}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px;margin-bottom:16px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px}
.card .k{color:var(--mute);font-size:12px}.card .v{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums;white-space:nowrap}.goal .v{font-size:26px;white-space:normal}
.goal{grid-column:1/-1}.bar{height:14px;background:var(--bar);border-radius:7px;overflow:hidden;margin-top:8px}
.bar>div{height:100%;background:var(--acc)}
.gauge{position:relative;margin-top:10px}.gauge .bar{height:22px;border-radius:6px;margin:0}
.stack{display:flex}.stack>div{height:100%}.stack .day{background:var(--day)}.stack .dusk{background:var(--dusk)}.stack .night{background:var(--night)}
.legend{display:flex;flex-wrap:wrap;gap:6px 18px;margin-top:8px;font-size:13px;font-variant-numeric:tabular-nums}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px}
.tod{display:inline-block;padding:1px 8px;border-radius:10px;font-size:12px;font-weight:600;color:#111}
.tod.day{background:var(--day)}.tod.dusk{background:var(--dusk)}.tod.night{background:var(--night);color:#fff}
.gauge .mark{position:absolute;top:-4px;height:30px;border-left:3px solid var(--warn)}
.gauge .mark span{position:absolute;top:-18px;left:-4px;white-space:nowrap;font-size:11px;font-weight:700;color:var(--warn)}
.ticks{position:relative;height:16px;margin-top:4px;font-size:11px;color:var(--mute)}
.ticks span{position:absolute;transform:translateX(-50%)}.ticks span:first-child{transform:none}.ticks span:last-child{transform:translateX(-100%)}
.sec{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:16px}
.sec h2{font-size:15px;margin:0 0 10px}
.regions{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:8px 20px}
.rg .n{display:flex;justify-content:space-between;font-variant-numeric:tabular-nums}.rg .bar{height:8px;margin-top:4px}
.tools{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:10px}
select,input,button{font:inherit;color:var(--fg);background:var(--card);border:1px solid var(--line);border-radius:6px;padding:5px 9px}
button{cursor:pointer}button:hover{border-color:var(--acc)}
button.primary{background:var(--acc);border-color:var(--acc);color:#fff;font-weight:700;padding:6px 16px}
button.primary:hover{filter:brightness(1.1)}
.wrap{overflow-x:auto}table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}
th,td{border-bottom:1px solid var(--line);padding:6px 8px;text-align:left;white-space:nowrap}
th{position:sticky;top:0;background:var(--card);cursor:pointer;font-size:12px;color:var(--mute)}
td.num{text-align:right}td.ed{cursor:text;min-width:70px}td.ed:empty::after{content:"입력";color:var(--mute);opacity:.5}
td.ed:focus{outline:2px solid var(--acc)}td.path{color:var(--mute);font-size:11px}
tr.miss td.ed:not([data-k=note]):empty{background:color-mix(in srgb,var(--warn) 12%,transparent)}
.err{color:var(--bad);font-weight:600}.msg{color:var(--mute);font-size:12px}
.tabs{display:flex;gap:4px;border-bottom:1px solid var(--line);margin-bottom:16px}
.tabs button{border:none;border-bottom:3px solid transparent;border-radius:0;background:none;padding:8px 16px;font-weight:600;color:var(--mute)}
.tabs button.on{color:var(--fg);border-bottom-color:var(--acc)}
.pane[hidden]{display:none}
#ptable input{width:100%;min-width:80px;padding:4px 6px}#ptable input.wide{min-width:260px}#ptable input.reg{min-width:150px}#ptable input.hrs{min-width:70px;text-align:right}
#ptable input.bad{border-color:var(--bad)}
.st{display:inline-block;padding:1px 8px;border-radius:10px;font-size:12px;font-weight:600}
.st.plan{background:color-mix(in srgb,var(--mute) 18%,transparent)}.st.prog{background:color-mix(in srgb,var(--acc) 22%,transparent)}
.st.done{background:color-mix(in srgb,var(--ok) 22%,transparent)}.st.late{background:color-mix(in srgb,var(--bad) 20%,transparent)}
.mini{width:90px;height:8px;margin:0;display:inline-block;vertical-align:middle}
.x{border:none;background:none;color:var(--mute);padding:2px 6px}.x:hover{color:var(--bad)}
</style></head><body><main>
<h1>데이터셋 취득 현황</h1>
<div class="sub" id="src"></div>
<div class="tabs"><button data-tab="status" class="on">현황</button><button data-tab="plan">계획표</button></div>
<div class="pane" id="pane-status">
<div class="cards">
  <div class="card goal"><div class="k" id="goalk"></div><div class="v" id="goalv"></div>
    <div class="gauge"><div class="bar stack" id="goalbar"></div><div class="mark" id="goalmark"><span></span></div></div>
    <div class="ticks" id="ticks"></div><div class="legend" id="todlegend"></div></div>
  <div class="card"><div class="k">수동녹화(GS)</div><div class="v" id="c_rec"></div></div>
  <div class="card"><div class="k">클립(TH)</div><div class="v" id="c_clip"></div></div>
  <div class="card"><div class="k">지역 수</div><div class="v" id="c_reg"></div></div>
  <div class="card"><div class="k">운전자·동승자 빈 칸</div><div class="v" id="c_miss"></div></div>
</div>
<div class="sec"><h2>지역별</h2><div class="regions" id="regions"></div></div>
<div class="sec">
  <div class="tools">
    <select id="f_region"><option value="">모든 지역</option></select>
    <select id="f_kind"><option value="">모든 종류</option><option>수동녹화(GS)</option><option>클립(TH)</option></select>
    <select id="f_tod"><option value="">모든 시간대</option><option>day</option><option>dusk</option><option>night</option></select>
    <button id="reload">새로 고침</button>
    <a href="/datasets.csv" download><button>CSV 내려받기</button></a>
    <label><button type="button" onclick="document.getElementById('file').click()">다른 CSV 불러오기…</button>
      <input type="file" id="file" accept=".csv" hidden></label>
    <span class="msg" id="msg"></span>
  </div>
  <div class="wrap"><table><thead id="thead"></thead><tbody id="tbody"></tbody></table></div>
</div>
</div>
<div class="pane" id="pane-plan" hidden>
<div class="cards">
  <div class="card"><div class="k">계획 합계 / 목표</div><div class="v" id="p_sum"></div></div>
  <div class="card"><div class="k">계획 대비 실제 취득</div><div class="v" id="p_act"></div></div>
  <div class="card"><div class="k">완료 / 전체 계획</div><div class="v" id="p_done"></div></div>
</div>
<div class="sec">
  <div class="tools">
    <button id="padd" class="primary">+ 계획 추가</button>
    <a href="/plans.csv" download><button>계획표 CSV 내려받기</button></a>
    <span class="msg" id="pmsg"></span>
  </div>
  <div class="wrap"><table id="ptable"><thead id="phead"></thead><tbody id="pbody"></tbody></table></div>
  <p class="msg">실제 취득 = 같은 날짜 · 같은 지역에 저장된 녹화 시간의 합 (현황 탭과 같은 목록). 지역은 한글로 써도 되고, 영문으로 바꾼 이름(폴더 이름)으로 GUI 녹화와 맞춥니다.
    칸을 고치면 바로 clips/plans.csv 에 저장됩니다.</p>
</div>
</div>
</main>
<script>
const GOAL_H = __GOAL__, SCALE_H = __SCALE__, COLS = __COLUMNS__, PCOLS = __PLAN_COLUMNS__, BASE = __BASE__;
const SHOW = ["region_name","route","date","start_time","end_time","time_of_day","duration_hms","kind","driver","passenger","label","size_gb","note","folder"];
const EDIT = ["driver","passenger","note"];
let rows = [], live = true, sortKey = "date", sortDir = 1;
const $ = id => document.getElementById(id);
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
  $("goalv").textContent = `${hms(total)}  (목표의 ${pct.toFixed(1)}%) — ` +
    (left > 0 ? `남은 시간 ${hms(left)}` : `목표 달성, ${hms(-left)} 초과`);
  // 게이지는 시간대별로 쌓는다 (day · dusk · night)
  const TODS = ["day", "dusk", "night"], todSum = (rs, k) => rs.reduce((a, r) => a + (parseFloat(r[k + "_sec"]) || 0), 0);
  const stack = rs => TODS.map(k => `<div class="${k}" style="width:${todSum(rs, k) / (SCALE_H * 3600) * 100}%" title="${k} ${hms(todSum(rs, k))}"></div>`).join("");
  $("goalbar").innerHTML = stack(rows);
  $("todlegend").innerHTML = TODS.map(k => { const v = todSum(rows, k);
    return `<span><i style="background:var(--${k})"></i><b>${k}</b> ${hms(v)} (${total ? (v / total * 100).toFixed(0) : 0}%)</span>`; }).join("");
  $("goalmark").style.left = (GOAL_H / SCALE_H * 100) + "%";
  $("goalmark").firstChild.textContent = `목표 ${GOAL_H}h`;
  $("ticks").innerHTML = Array.from({length: Math.floor(SCALE_H) + 1}, (_, h) =>
    `<span style="left:${h / SCALE_H * 100}%">${h}h</span>`).join("");
  const byKind = k => rows.filter(r => r.kind === k);
  $("c_rec").textContent = `${hms(sum(byKind("수동녹화(GS)")))} · ${byKind("수동녹화(GS)").length}개`;
  $("c_clip").textContent = `${hms(sum(byKind("클립(TH)")))} · ${byKind("클립(TH)").length}개`;
  const regions = [...new Set(rows.map(r => r.region))].sort();
  const nameOf = g => (rows.find(r => r.region === g && r.region_name) || {}).region_name || g;
  $("c_reg").textContent = regions.length;
  const miss = rows.filter(r => !r.driver || !r.passenger).length;
  $("c_miss").innerHTML = miss ? `<span style="color:var(--warn)">${miss}개</span>` : "없음";
  const maxR = Math.max(1, ...regions.map(g => sum(rows.filter(r => r.region === g))));
  $("regions").innerHTML = regions.map(g => {
    const rs = rows.filter(r => r.region === g), s = sum(rs), routes = new Set(rs.map(r => r.route)).size;
    const part = TODS.filter(k => todSum(rs, k) > 0).map(k => `${k} ${hms(todSum(rs, k))}`).join(" · ");
    return `<div class="rg"><div class="n"><span><b>${esc(nameOf(g))}</b>${nameOf(g) !== g ? ` <span class="msg">${esc(g)}</span>` : ""}</span><span>${hms(s)} · route ${routes}개 · ${rs.length}개</span></div>
            <div class="bar stack">${TODS.map(k => `<div class="${k}" style="width:${todSum(rs, k) / maxR * 100}%"></div>`).join("")}</div>
            <div class="msg">${part}</div></div>`;
  }).join("") || '<span class="msg">아직 route 폴더에 저장된 데이터가 없습니다</span>';
  const sel = $("f_region"), cur = sel.value;
  sel.innerHTML = '<option value="">모든 지역</option>' + regions.map(g => `<option value="${esc(g)}">${esc(nameOf(g))}</option>`).join("");
  sel.value = regions.includes(cur) ? cur : "";

  $("thead").innerHTML = "<tr>" + SHOW.map(k => `<th data-k="${k}">${title(k)}${k === sortKey ? (sortDir > 0 ? " ▲" : " ▼") : ""}</th>`).join("") + "</tr>";
  let vis = rows.filter(r => (!fr || r.region === fr) && (!fk || r.kind === fk) && (!ft || (r.time_of_day || "").split("+").includes(ft)));
  const num = k => ["duration_hms","size_gb"].includes(k);
  vis.sort((a, b) => {
    const x = sortKey === "duration_hms" ? +a.duration_sec : num(sortKey) ? +a[sortKey] : a[sortKey] + a.start_time;
    const y = sortKey === "duration_hms" ? +b.duration_sec : num(sortKey) ? +b[sortKey] : b[sortKey] + b.start_time;
    return (x > y ? 1 : x < y ? -1 : 0) * sortDir;
  });
  $("tbody").innerHTML = vis.map(r => `<tr class="${!r.driver || !r.passenger ? "miss" : ""}" data-f="${esc(r.folder)}">` + SHOW.map(k =>
    EDIT.includes(k) && live ? `<td class="ed" contenteditable="plaintext-only" data-k="${k}">${esc(r[k])}</td>`
    : k === "region_name" ? `<td title="${esc(r.region)}">${esc(r.region_name || r.region)}</td>`
    : k === "folder" ? `<td class="path">${esc(r.folder.split("/").slice(-2).join("/"))}</td>`
    : k === "time_of_day" ? `<td>${(r.time_of_day || "").split("+").filter(Boolean).map(t => `<span class="tod ${t}">${t}</span>`).join(" ")}</td>`
    : `<td class="${num(k) ? "num" : ""}">${esc(r[k])}</td>`).join("") + "</tr>").join("");
  $("msg").textContent = `${vis.length}개 표시` + (live ? " · 운전자/동승자/비고 칸은 눌러서 고칠 수 있습니다" : " · 불러온 CSV (읽기 전용)");
  if (!$("pbody").contains(document.activeElement)) renderPlan(); else renderPlanStats();
}

// ---------- 계획표 ----------
let plans = [];
const today = () => new Date().toLocaleDateString("sv");        // YYYY-MM-DD (로컬)
const actualFor = p => rows.filter(r => r.date === p.date && r.region === p.region_en)
                           .reduce((a, r) => a + (parseFloat(r.duration_sec) || 0), 0);
function planStatus(p) {
  const act = actualFor(p), goal = (parseFloat(p.planned_hours) || 0) * 3600;
  if (goal && act >= goal) return ["done", "완료"];
  if (act > 0) return p.date < today() ? ["late", "미달"] : ["prog", "진행 중"];
  return p.date && p.date < today() ? ["late", "미취득"] : ["plan", "예정"];
}
async function loadPlans() {
  try {
    const r = await fetch("/api/plans", {cache: "no-store"});
    if (!r.ok) throw new Error(await r.text());
    plans = await r.json(); renderPlan();
  } catch (e) { $("pmsg").innerHTML = `<span class="err">계획표 불러오기 실패: ${esc(e.message)}</span>`; }
}
let saveT = null;
function savePlans() {
  clearTimeout(saveT);
  saveT = setTimeout(async () => {
    const r = await fetch("/api/plans", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({plans})});
    const j = await r.json();
    if (j.ok) j.region_en.forEach((en, i) => { if (plans[i]) plans[i].region_en = en; });
    renderPlanStats();
    $("pmsg").innerHTML = j.ok ? `저장됨 (${j.n}개) · ${new Date().toLocaleTimeString()}` : `<span class="err">${esc(j.error)}</span>`;
  }, 400);
}
function renderPlan() {
  const inp = (i, k, v) => {
    const cls = k === "route_desc" ? "wide" : k === "planned_hours" ? "hrs" : k === "region" ? "reg" : "";
    const type = k === "date" ? "date" : k === "planned_hours" ? "number" : "text";
    const extra = k === "planned_hours" ? ' step="0.5" min="0"' : k === "region" ? ' placeholder="강남역 4번출구"' : "";
    return `<td><input class="${cls}" type="${type}" data-i="${i}" data-k="${k}" value="${esc(v)}"${extra}></td>`;
  };
  $("phead").innerHTML = "<tr><th></th>" + PCOLS.map(c => `<th>${c[1]}</th>`).join("") + "<th>실제 취득</th><th>달성</th><th>상태</th></tr>";
  $("pbody").innerHTML = plans.map((p, i) => `<tr><td><button class="x" data-del="${i}" title="이 계획 지우기">✕</button></td>` + PCOLS.map(c => c[0] === "region_en"
      ? `<td class="path" id="pr${i}"></td>` : inp(i, c[0], p[c[0]])).join("") +
    `<td class="num" id="pa${i}"></td><td id="pb${i}"></td><td id="ps${i}"></td></tr>`).join("")
    || `<tr><td colspan="${PCOLS.length + 4}" class="msg">아직 계획이 없습니다 — [+ 계획 추가] 를 누르세요</td></tr>`;
  renderPlanStats();
}
function renderPlanStats() {
  let planned = 0, actual = 0, done = 0;
  plans.forEach((p, i) => {
    const act = actualFor(p), goal = (parseFloat(p.planned_hours) || 0) * 3600, [cls, txt] = planStatus(p);
    planned += goal; actual += Math.min(act, goal || act); if (cls === "done") done++;
    if (!$("pa" + i)) return;
    $("pr" + i).textContent = p.region_en || "";
    $("pa" + i).textContent = hms(act);
    $("pb" + i).innerHTML = goal ? `<div class="bar mini"><div style="width:${Math.min(100, act / goal * 100)}%"></div></div> ${Math.round(act / goal * 100)}%` : "";
    $("ps" + i).innerHTML = `<span class="st ${cls}">${txt}</span>`;
  });
  $("p_sum").innerHTML = `${hms(planned)} / ${GOAL_H}h` + (planned < GOAL_H * 3600 ? ` <span style="color:var(--warn);font-size:13px">(${hms(GOAL_H * 3600 - planned)} 부족)</span>` : "");
  $("p_act").textContent = planned ? `${hms(actual)} (${(actual / planned * 100).toFixed(0)}%)` : "-";
  $("p_done").textContent = `${done} / ${plans.length}`;
}
$("pbody").addEventListener("input", e => {
  const i = e.target.dataset.i, k = e.target.dataset.k; if (i === undefined) return;
  let v = e.target.value;
  plans[i][k] = v; renderPlanStats(); savePlans();
});
$("pbody").addEventListener("click", e => {
  const i = e.target.dataset.del; if (i === undefined) return;
  const p = plans[i];
  if (!confirm(`계획을 지울까요?\n${p.date} ${p.region} ${p.route_desc}`)) return;
  plans.splice(i, 1); renderPlan(); savePlans();
});
$("padd").onclick = () => {
  plans.push({date: today(), region: "", region_en: "", route_desc: "", planned_hours: "1", note: ""});
  renderPlan(); savePlans();
  $("pbody").querySelector(`input[data-i="${plans.length - 1}"][data-k="region"]`).focus();
};
document.querySelectorAll(".tabs button").forEach(b => b.onclick = () => {
  document.querySelectorAll(".tabs button").forEach(x => x.classList.toggle("on", x === b));
  $("pane-status").hidden = b.dataset.tab !== "status"; $("pane-plan").hidden = b.dataset.tab !== "plan";
  try { localStorage.setItem("tab", b.dataset.tab); } catch (e) {}
});
try { const t = location.hash.slice(1) || localStorage.getItem("tab"); if (t) document.querySelector(`.tabs button[data-tab="${t}"]`)?.click(); } catch (e) {}

function esc(s) { return String(s ?? "").replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c])); }

$("thead").addEventListener("click", e => {
  const k = e.target.dataset.k; if (!k) return;
  sortDir = k === sortKey ? -sortDir : 1; sortKey = k; render();
});
$("tbody").addEventListener("keydown", e => { if (e.key === "Enter") { e.preventDefault(); e.target.blur(); } });
$("tbody").addEventListener("focusout", async e => {
  const td = e.target; if (!td.classList.contains("ed")) return;
  const folder = td.parentElement.dataset.f, k = td.dataset.k, v = td.textContent.trim();
  const row = rows.find(r => r.folder === folder); if (!row || row[k] === v) return;
  const r = await fetch("/api/edit", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({folder, [k]: v})});
  const j = await r.json();
  if (j.ok) { row[k] = v; $("msg").textContent = `저장됨: ${folder.split("/").pop()} ${title(k)} = ${v || "(빈 칸)"}`; render(); }
  else { $("msg").innerHTML = `<span class="err">저장 실패: ${esc(j.error)}</span>`; td.textContent = row[k]; }
});
$("f_region").onchange = $("f_kind").onchange = $("f_tod").onchange = render;
$("reload").onclick = load;
$("file").onchange = async e => {
  const f = e.target.files[0]; if (!f) return;
  rows = parseCSV(await f.text()); live = false;
  $("src").textContent = `불러온 파일: ${f.name} (읽기 전용 — [새로 고침]을 누르면 이 PC 의 목록으로 돌아갑니다)`;
  render(); e.target.value = "";
};
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
