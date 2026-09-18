#!/usr/bin/env python3
# gnss_tools.py — 클립에 기록된 GNSS(NavSatFix) 품질 분석 + OpenStreetMap 궤적 렌더링.
#
#   read_navsat(bag_dir)      → {topic: [fix dict, ...]}   (sensor_msgs/NavSatFix 전부)
#   read_strings(bag_dir, t)  → [(t_ns, str)]              (pos_type / nav_status 등)
#   quality(fixes, ...)       → 품질 보고서 dict (fix 비율, 정확도, 점프, 공백, 판정)
#   render_map(fixes, png)    → OSM 타일 위에 궤적을 그려 PNG 저장 (오프라인이면 타일 없이)
#
# 단독 실행: python3 gnss_tools.py <clip_dir> [--map out.png]

import argparse
import json
import math
import sqlite3
import statistics
import sys
import time
import urllib.request
from pathlib import Path

NS = 1e9
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
TILE_CACHE = Path.home() / ".cache" / "dm_clip_gui" / "tiles"
USER_AGENT = "dm_clip_gui/0.1 (ROS2 bag GNSS viewer)"
STATUS_NAMES = {-1: "NO_FIX", 0: "FIX", 1: "SBAS_FIX", 2: "GBAS_FIX"}
TRACK_FILE = "gnss_track.json"     # 클립 폴더에 캐시되는 궤적 (bag 재독 방지)
# 클립 궤적 색 — 흑백으로 흐린 바탕 지도 위에서 서로 잘 갈리는 선명한 색만 (갈색 · 회색 · 올리브는 묻혔다)
TRACK_COLORS = ["#2563eb", "#e11d48", "#16a34a", "#9333ea", "#ea580c",
                "#0891b2", "#db2777", "#ca8a04", "#4f46e5", "#0d9488"]
OK, WARN, FAIL = "OK", "WARN", "FAIL"


# ---------- 읽기 ----------

def _sqlite_rows(bag_dir, type_filter=None, topic_filter=None):
    db3 = sorted(Path(bag_dir).glob("*.db3"))
    if not db3:
        return None
    con = sqlite3.connect(f"file:{db3[0]}?mode=ro", uri=True)
    out = {}
    for tid, name, typ in con.execute("SELECT id, name, type FROM topics"):
        if type_filter and typ != type_filter:
            continue
        if topic_filter and name != topic_filter:
            continue
        out[name] = (typ, [(t, bytes(d)) for t, d in con.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? "
            "ORDER BY timestamp", (tid,))])
    con.close()
    return out


def _rosbag2py_rows(bag_dir, type_filter=None, topic_filter=None):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {n for n, t in types.items()
            if (not type_filter or t == type_filter)
            and (not topic_filter or n == topic_filter)}
    out = {n: (types[n], []) for n in want}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in want:
            out[topic][1].append((t, bytes(data)))
    return out


def _rows(bag_dir, **kw):
    r = _sqlite_rows(bag_dir, **kw)
    return r if r is not None else _rosbag2py_rows(bag_dir, **kw)


def read_navsat(bag_dir):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import NavSatFix
    out = {}
    for topic, (_, rows) in _rows(bag_dir,
                                  type_filter="sensor_msgs/msg/NavSatFix").items():
        fixes = []
        for t, data in rows:
            try:
                m = deserialize_message(data, NavSatFix)
            except Exception:
                fixes.append({"t": t, "corrupt": True})
                continue
            hs = m.header.stamp
            fixes.append({
                "t": t, "stamp": hs.sec * 10**9 + hs.nanosec,
                "lat": m.latitude, "lon": m.longitude, "alt": m.altitude,
                "status": int(m.status.status), "service": int(m.status.service),
                "cov": list(m.position_covariance),
                "cov_type": int(m.position_covariance_type)})
        out[topic] = fixes
    return out


def read_strings(bag_dir, topic):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    r = _rows(bag_dir, type_filter="std_msgs/msg/String", topic_filter=topic)
    if topic not in r:
        return []
    return [(t, deserialize_message(d, String).data) for t, d in r[topic][1]]


def save_track(bag_dir, fixes_by_topic):
    p = Path(bag_dir) / TRACK_FILE
    try:
        p.write_text(json.dumps(fixes_by_topic), encoding="utf-8")
    except OSError:
        pass


def load_track(bag_dir):
    p = Path(bag_dir) / TRACK_FILE
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def get_track(bag_dir):
    """클립의 첫 NavSatFix 토픽 궤적. 캐시가 있으면 즉시, 없으면 bag에서 추출 후 캐시.
    NavSatFix 토픽이 없으면 []."""
    cached = load_track(bag_dir)
    if cached is None:
        cached = read_navsat(bag_dir)
        save_track(bag_dir, cached)
    for _, fixes in cached.items():
        return fixes
    return []


# ---------- 품질 ----------

def _valid(f):
    return (not f.get("corrupt") and f["status"] >= 0
            and math.isfinite(f["lat"]) and math.isfinite(f["lon"])
            and not (abs(f["lat"]) < 1e-9 and abs(f["lon"]) < 1e-9)
            and abs(f["lat"]) <= 90 and abs(f["lon"]) <= 180)


def _haversine_m(a, b):
    R = 6371000.0
    p1, p2 = math.radians(a["lat"]), math.radians(b["lat"])
    dp = p2 - p1
    dl = math.radians(b["lon"] - a["lon"])
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))


def quality(fixes, pos_types=None, nav_statuses=None, expected_hz=None):
    q = {"count": len(fixes), "level": OK, "notes": [], "status_hist": {},
         "valid": 0, "invalid": 0, "corrupt": 0, "jumps": 0, "gaps": 0}
    if not fixes:
        q["level"] = WARN
        q["notes"].append("NavSatFix 메시지 없음 (fix 없으면 드라이버가 발행하지 않음)")
    else:
        for f in fixes:
            if f.get("corrupt"):
                q["corrupt"] += 1
                continue
            name = STATUS_NAMES.get(f["status"], str(f["status"]))
            q["status_hist"][name] = q["status_hist"].get(name, 0) + 1
            if _valid(f):
                q["valid"] += 1
            else:
                q["invalid"] += 1
        if q["corrupt"]:
            q["level"] = FAIL
            q["notes"].append(f"역직렬화 실패 {q['corrupt']}개")
        q["fix_ratio"] = round(q["valid"] / len(fixes), 3)
        if q["valid"] == 0:
            q["level"] = _worse(q["level"], WARN)
            q["notes"].append("유효한 fix 0개 (실내/차폐?)")
        elif q["fix_ratio"] < 0.9:
            q["level"] = _worse(q["level"], WARN)
            q["notes"].append(f"fix 비율 {q['fix_ratio']*100:.0f}%")
        bad_val = sum(1 for f in fixes if not f.get("corrupt")
                      and f["status"] >= 0 and not _valid(f))
        if bad_val:
            q["level"] = FAIL
            q["notes"].append(f"status=FIX인데 좌표가 비정상(NaN/0/범위 밖) {bad_val}개")

        valid = [f for f in fixes if _valid(f)]
        if valid:
            # 수평 정확도: 공분산 대각의 sqrt (cov_type 0 = UNKNOWN이면 생략)
            hacc = [math.sqrt(max(f["cov"][0], f["cov"][4])) for f in valid
                    if f["cov_type"] > 0 and f["cov"][0] >= 0 and f["cov"][4] >= 0]
            if hacc:
                hacc.sort()
                q["hacc_med_m"] = round(hacc[len(hacc) // 2], 2)
                q["hacc_p95_m"] = round(hacc[int(0.95 * (len(hacc) - 1))], 2)
                if q["hacc_p95_m"] > 5.0:
                    q["level"] = _worse(q["level"], WARN)
                    q["notes"].append(f"수평 정확도 p95 {q['hacc_p95_m']:.1f} m")
            # 점프 (비현실적 속도) / 공백
            ts = [f["t"] for f in valid]
            dts = [(b - a) / NS for a, b in zip(ts, ts[1:])]
            if len(dts) >= 5:
                med = statistics.median(dts)
                q["rate_hz"] = round(1.0 / med, 2) if med > 0 else None
                q["gaps"] = sum(1 for d in dts if med > 0 and d > 3 * med)
                for a, b, dt in zip(valid, valid[1:], dts):
                    if dt > 0 and _haversine_m(a, b) / dt > 100.0:   # 360 km/h
                        q["jumps"] += 1
                if q["jumps"]:
                    q["level"] = _worse(q["level"], WARN)
                    q["notes"].append(f"위치 점프 {q['jumps']}회 (>100 m/s)")
                if q["gaps"]:
                    q["level"] = _worse(q["level"], WARN)
                    q["notes"].append(f"fix 공백 {q['gaps']}회 (>3주기)")
            lat = [f["lat"] for f in valid]
            lon = [f["lon"] for f in valid]
            q["bbox"] = [min(lat), min(lon), max(lat), max(lon)]
            q["track_m"] = round(sum(_haversine_m(a, b)
                                     for a, b in zip(valid, valid[1:])), 1)

    def hist(items):
        h = {}
        for _, s in items or []:
            h[s] = h.get(s, 0) + 1
        return h
    if pos_types:
        q["pos_type_hist"] = hist(pos_types)
        if set(q["pos_type_hist"]) <= {"NONE", ""}:
            q["notes"].append("pos_type 전 구간 NONE")
    if nav_statuses:
        q["nav_status_hist"] = hist(nav_statuses)
    return q


def _worse(a, b):
    rank = {OK: 0, WARN: 1, FAIL: 2}
    return a if rank[a] >= rank[b] else b


# ---------- 지도 ----------

def _deg2num(lat, lon, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(math.radians(lat)) +
                        1 / math.cos(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def _fetch_tile(z, x, y):
    p = TILE_CACHE / str(z) / str(x) / f"{y}.png"
    if p.exists():
        return p
    p.parent.mkdir(parents=True, exist_ok=True)
    req = urllib.request.Request(TILE_URL.format(z=z, x=x, y=y),
                                 headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=6) as r:
        p.write_bytes(r.read())
    return p


def render_map(fixes, out_png, title="", max_tiles=5):
    """유효 fix 궤적 하나를 OSM 위에 그려 out_png로 저장 (정확도 색상)."""
    return render_map_multi([(title or "궤적", fixes)], out_png, title=title,
                            max_tiles=max_tiles, color_by_accuracy=True)


def render_map_multi(tracks, out_png, title="", max_tiles=5, color_by_accuracy=False):
    """여러 클립 궤적을 한 지도에 겹쳐 그린다. tracks = [(라벨, fixes), ...].
    반환: 저장 경로, 유효 fix가 하나도 없으면 None."""
    tracks = [(lbl, [f for f in fx if _valid(f)]) for lbl, fx in tracks]
    tracks = [(lbl, v) for lbl, v in tracks if v]
    if not tracks:
        return None
    valid = [f for _, v in tracks for f in v]
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.colors import Normalize
    # 한글 라벨용 폰트 (있는 것 중 첫 번째)
    have = {f.name for f in font_manager.fontManager.ttflist}
    for fam in ("NanumGothic", "NanumBarunGothic", "Noto Sans CJK KR",
                "Noto Sans CJK JP", "UnDotum"):
        if fam in have:
            plt.rcParams["font.family"] = fam
            break
    plt.rcParams["axes.unicode_minus"] = False

    lat = [f["lat"] for f in valid]
    lon = [f["lon"] for f in valid]
    lat0, lat1, lon0, lon1 = min(lat), max(lat), min(lon), max(lon)
    pad_lat = max((lat1 - lat0) * 0.15, 0.0005)
    pad_lon = max((lon1 - lon0) * 0.15, 0.0005)
    lat0, lat1, lon0, lon1 = lat0 - pad_lat, lat1 + pad_lat, lon0 - pad_lon, lon1 + pad_lon

    # 타일이 max_tiles×max_tiles 이내가 되는 최대 줌
    z = 18
    while z > 2:
        x0, y1 = _deg2num(lat0, lon0, z)
        x1, y0 = _deg2num(lat1, lon1, z)
        if (int(x1) - int(x0) + 1) <= max_tiles and (int(y1) - int(y0) + 1) <= max_tiles:
            break
        z -= 1
    x0, y1 = _deg2num(lat0, lon0, z)
    x1, y0 = _deg2num(lat1, lon1, z)
    tx0, tx1, ty0, ty1 = int(x0), int(x1), int(y0), int(y1)

    fig, ax = plt.subplots(figsize=(9, 8), dpi=110)
    have_tiles = False
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            try:
                img = plt.imread(str(_fetch_tile(z, tx, ty)))
            except Exception:
                continue
            ax.imshow(img, extent=[tx, tx + 1, ty + 1, ty], zorder=0)
            have_tiles = True

    for i, (lbl, v) in enumerate(tracks):
        xs, ys = zip(*[_deg2num(f["lat"], f["lon"], z) for f in v])
        color = TRACK_COLORS[i % len(TRACK_COLORS)]
        if color_by_accuracy and len(tracks) == 1:
            hacc = [math.sqrt(max(f["cov"][0], f["cov"][4]))
                    if f["cov_type"] > 0 and f["cov"][0] >= 0 else float("nan")
                    for f in v]
            ax.plot(xs, ys, "-", color=color, lw=1.2, alpha=0.7, zorder=2)
            if any(math.isfinite(h) for h in hacc):
                sc = ax.scatter(
                    xs, ys, c=hacc, cmap="RdYlGn_r", s=10, zorder=3,
                    norm=Normalize(0, max(2.0, max(h for h in hacc if math.isfinite(h)))))
                fig.colorbar(sc, ax=ax, label="수평 정확도 (m)", shrink=0.7)
            ax.plot(xs[0], ys[0], "o", color="#2e7d32", ms=9, zorder=4, label="시작")
            ax.plot(xs[-1], ys[-1], "s", color="#c62828", ms=9, zorder=4, label="끝")
        else:
            dist = sum(_haversine_m(a, b) for a, b in zip(v, v[1:]))
            ax.plot(xs, ys, "-", color=color, lw=1.6, alpha=0.85, zorder=2,
                    label=f"{lbl} ({len(v)} fix, {dist/1000:.2f} km)")
            ax.plot(xs[0], ys[0], "o", color=color, ms=7, mec="white", zorder=4)
            ax.plot(xs[-1], ys[-1], "s", color=color, ms=7, mec="white", zorder=4)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(title or (f"GNSS 궤적 {len(tracks)}개 클립, {len(valid)} fix (z={z})"))
    if have_tiles:
        ax.text(0.01, 0.01, "© OpenStreetMap contributors", transform=ax.transAxes,
                fontsize=7, color="#444", bbox=dict(fc="white", alpha=0.7, lw=0))
    else:
        ax.text(0.5, 0.5, "지도 타일 없음 (오프라인)", transform=ax.transAxes,
                ha="center", color="#888")
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)
    return str(out_png)


# ---------- CLI ----------

def analyze_bag(bag_dir):
    """클립의 모든 NavSatFix 토픽 품질 + 상태 문자열 히스토그램."""
    fixes_by_topic = read_navsat(bag_dir)
    save_track(bag_dir, fixes_by_topic)          # 누적 지도용 캐시
    pos_types = read_strings(bag_dir, "/gps/pos_type")
    nav_statuses = read_strings(bag_dir, "/gps/nav_status")
    out = {}
    for topic, fixes in fixes_by_topic.items():
        out[topic] = quality(fixes, pos_types, nav_statuses)
    if not fixes_by_topic and (pos_types or nav_statuses):
        out["(fix 없음)"] = quality([], pos_types, nav_statuses)
    return out, fixes_by_topic


def main():
    ap = argparse.ArgumentParser(description="클립 GNSS 품질/지도")
    ap.add_argument("bag_dir")
    ap.add_argument("--map", help="궤적 PNG 저장 경로")
    args = ap.parse_args()
    rep, fixes = analyze_bag(args.bag_dir)
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    if args.map:
        for topic, fx in fixes.items():
            p = render_map(fx, args.map, title=f"{topic} — {Path(args.bag_dir).name}")
            print("지도:", p or "유효 fix 없음")
            break


if __name__ == "__main__":
    sys.exit(main())
