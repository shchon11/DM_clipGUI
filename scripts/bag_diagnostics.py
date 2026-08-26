#!/usr/bin/env python3
# bag_diagnostics.py — rosbag2 클립 무결성 진단.
#
# rosbag 수신 시각이 아니라 각 메시지의 header.stamp(CDR 앞부분 고정 오프셋에서
# 직접 파싱 — 메시지 정의가 설치돼 있지 않은 타입도 검사 가능)를 기준으로:
#   - 프레임 중간 유실 (진짜 드랍) vs 클립 경계 잘림 (정상적인 창 효과) 구분
#   - 수신 지연 (bag 기록 시각 - header.stamp) 통계 → 전송 백로그 탐지
#   - 중복/비단조 stamp, 0바이트 페이로드, 크기 이상치
#   - 샘플 역직렬화 + 타입별 페이로드 검증 (Image 크기, JPEG/PNG 매직)
#
# 단독 실행:  python3 bag_diagnostics.py <clip_dir> [--json out.json]
# 모듈 사용:  report = analyze(clip_dir); print(render_text(report))

import argparse
import json
import math
import sqlite3
import statistics
import struct
import sys
from pathlib import Path

NS = 1e9
SAMPLES_PER_TOPIC = 25      # 역직렬화/페이로드 검사 샘플 수
MIN_MSGS_FOR_GAPS = 10      # 이보다 적으면 간격 분석 생략 (저빈도 토픽)
GAP_FACTOR = 1.6            # dt > median*GAP_FACTOR 이면 유실 후보
LAT_WARN_MS = 200.0         # 수신 지연 p95 경고 문턱

OK, WARN, FAIL = "OK", "WARN", "FAIL"
_RANK = {OK: 0, WARN: 1, FAIL: 2}


def _worse(a, b):
    return a if _RANK[a] >= _RANK[b] else b


def _parse_header_ns(head: bytes):
    """CDR 인코딩 앞 12바이트에서 header.stamp를 뽑는다. 실패 시 None."""
    if head is None or len(head) < 12:
        return None
    endian = "<" if head[1] in (1, 3) else ">"
    try:
        sec, nsec = struct.unpack(endian + "iI", head[4:12])
    except struct.error:
        return None
    if sec <= 0 or nsec >= 1_000_000_000:
        return None
    return sec * 10**9 + nsec


# ---------- 스토리지 리더 ----------

def _read_sqlite(db_path, progress):
    """{topic: {'type':.., 'rows':[(bag_ns, hdr_ns|None, size)], 'ids':[rowid]}}"""
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    topics = {tid: (name, typ) for tid, name, typ in
              con.execute("SELECT id, name, type FROM topics")}
    out = {}
    for tid, (name, typ) in sorted(topics.items(), key=lambda kv: kv[1][0]):
        rows, ids = [], []
        for rid, bag_ns, head, size in con.execute(
                "SELECT id, timestamp, substr(data,1,12), length(data) "
                "FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
            rows.append((bag_ns, _parse_header_ns(head), size or 0))
            ids.append(rid)
        out[name] = {"type": typ, "rows": rows, "_ids": ids, "_db": str(db_path)}
        if progress:
            progress(f"읽는 중: {name} ({len(rows)}개)")
    con.close()
    return out


def _fetch_samples_sqlite(info, indices):
    con = sqlite3.connect(f"file:{info['_db']}?mode=ro", uri=True)
    out = []
    for i in indices:
        row = con.execute("SELECT data FROM messages WHERE id=?",
                          (info["_ids"][i],)).fetchone()
        out.append((i, bytes(row[0]) if row and row[0] is not None else b""))
    con.close()
    return out


def _read_rosbag2py(bag_dir, progress):
    """mcap 등 sqlite3가 아닌 스토리지 폴백. 샘플은 균등 간격으로 본문 보관."""
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {n: {"type": t, "rows": [], "_samples": []} for n, t in types.items()}
    n_read = 0
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        info = out[topic]
        info["rows"].append((bag_ns, _parse_header_ns(bytes(data[:12])), len(data)))
        # 대략 SAMPLES_PER_TOPIC개가 남도록 성긴 간격으로 본문 보관
        if len(info["_samples"]) < SAMPLES_PER_TOPIC * 4 and \
                len(info["rows"]) % max(1, len(info["rows"]) // SAMPLES_PER_TOPIC + 1) == 0:
            info["_samples"].append((len(info["rows"]) - 1, bytes(data)))
        n_read += 1
        if progress and n_read % 2000 == 0:
            progress(f"읽는 중: {n_read}개…")
    for info in out.values():
        info["rows"].sort(key=lambda r: r[0])
    return out


# ---------- 페이로드 검사 ----------

def _get_msg_class(type_name):
    try:
        from rosidl_runtime_py.utilities import get_message
        return get_message(type_name)
    except Exception:
        return None


def _check_payload(type_name, msg):
    """역직렬화된 메시지의 내용 검증. 문제 문자열 리스트를 돌려준다."""
    issues = []
    if type_name == "sensor_msgs/msg/Image":
        expect = msg.step * msg.height
        if len(msg.data) != expect:
            issues.append(
                f"Image data {len(msg.data)}B != step*height {expect}B")
    elif type_name == "sensor_msgs/msg/CompressedImage":
        d = bytes(msg.data[:8])
        fmt = (msg.format or "").lower()
        if "jpeg" in fmt or "jpg" in fmt:
            if d[:2] != b"\xff\xd8":
                issues.append("JPEG SOI 매직(FFD8) 없음")
            elif bytes(msg.data[-2:]) != b"\xff\xd9":
                issues.append("JPEG EOI(FFD9)로 끝나지 않음 (잘린 프레임 의심)")
        elif "png" in fmt and d[:4] != b"\x89PNG":
            issues.append("PNG 매직 없음")
    return issues


# ---------- 토픽 하나 분석 ----------

def _analyze_topic(name, info, bag_t0, bag_t1, fetch_samples, progress):
    rows = info["rows"]
    r = {
        "type": info["type"], "count": len(rows),
        "bytes": sum(sz for _, _, sz in rows),
        "level": OK, "notes": [], "gaps": [],
        "lost_mid": 0, "head_trunc": 0, "tail_trunc": 0,
        "dup_stamps": 0, "nonmonotonic": 0, "zero_size": 0,
        "deser_checked": 0, "deser_failed": 0, "payload_issues": [],
    }
    if not rows:
        r["level"] = FAIL
        r["notes"].append("메시지 0개")
        return r

    # 0바이트 페이로드
    r["zero_size"] = sum(1 for _, _, sz in rows if sz == 0)
    if r["zero_size"]:
        r["level"] = FAIL
        r["notes"].append(f"0바이트 페이로드 {r['zero_size']}개")
    sizes = sorted(sz for _, _, sz in rows)
    r["size_min"], r["size_med"], r["size_max"] = \
        sizes[0], sizes[len(sizes) // 2], sizes[-1]

    # stamp 기준 선택: header가 95% 이상 유효하고 bag 시각과 5분 내로 일치할 때
    hdr_valid = [h for b, h, _ in rows
                 if h is not None and abs(h - b) < 300 * NS]
    use_header = len(hdr_valid) >= 0.95 * len(rows)
    r["stamp_mode"] = "header" if use_header else "bag_time"
    stamps = sorted(h for _, h, _ in rows) if use_header \
        else [b for b, _, _ in rows]

    # 수신 지연 (header 모드에서만 의미 있음)
    if use_header:
        lat = sorted((b - h) / 1e6 for b, h, _ in rows if h is not None)
        r["lat_med_ms"] = round(lat[len(lat) // 2], 1)
        r["lat_p95_ms"] = round(lat[min(len(lat) - 1, int(0.95 * len(lat)))], 1)
        r["lat_max_ms"] = round(lat[-1], 1)
        if r["lat_p95_ms"] > LAT_WARN_MS:
            r["level"] = _worse(r["level"], WARN)
            r["notes"].append(
                f"수신 지연 p95 {r['lat_p95_ms']:.0f} ms — 전송 백로그 의심")

        raw_h = [h for _, h, _ in rows if h is not None]
        r["nonmonotonic"] = sum(1 for a, b2 in zip(raw_h, raw_h[1:]) if b2 < a)
        r["dup_stamps"] = len(raw_h) - len(set(raw_h))
        if r["dup_stamps"]:
            r["level"] = _worse(r["level"], WARN)
            r["notes"].append(f"중복 stamp {r['dup_stamps']}개")
        if r["nonmonotonic"]:
            r["level"] = _worse(r["level"], WARN)
            r["notes"].append(f"stamp 역행 {r['nonmonotonic']}회")

    # 간격 분석 (충분히 빠른 스트림만)
    if len(stamps) >= MIN_MSGS_FOR_GAPS:
        dts = [(b - a) for a, b in zip(stamps, stamps[1:])]
        med = statistics.median(dts)
        if med > 0:
            r["dt_median_ms"] = round(med / 1e6, 2)
            r["rate_hz"] = round(NS / med, 1)
            for i, dt in enumerate(dts):
                if dt <= med * GAP_FACTOR:
                    continue
                # stamp 흔들림 보정: 다음 간격과 합이 2주기 이내면 지터, 유실 아님
                if i + 1 < len(dts) and dts[i] + dts[i + 1] < 2.5 * med:
                    continue
                est = int(round(dt / med)) - 1
                if est < 1:
                    continue
                r["lost_mid"] += est
                if len(r["gaps"]) < 20:
                    r["gaps"].append({
                        "t": round((stamps[i] - bag_t0) / NS, 3),
                        "dt_ms": round(dt / 1e6, 1), "est_lost": est})
            if r["lost_mid"]:
                r["level"] = FAIL
                r["notes"].append(f"스트림 중간 유실 {r['lost_mid']}프레임")

            # 클립 경계 잘림 (유실 아님 — 창이 닫힐 때 아직 도착 전이던 프레임)
            r["head_trunc"] = max(0, int((stamps[0] - bag_t0) / med - 0.5))
            r["tail_trunc"] = max(0, int((bag_t1 - stamps[-1]) / med - 0.5))
            if r["head_trunc"] or r["tail_trunc"]:
                # 1~2프레임은 수신 지연만큼 창이 어긋난 정상 범위 → 참고만
                if max(r["head_trunc"], r["tail_trunc"]) > 2:
                    r["level"] = _worse(r["level"], WARN)
                r["notes"].append(
                    f"클립 경계 잘림 앞 {r['head_trunc']} / 뒤 {r['tail_trunc']}"
                    "프레임 (창 효과, 중간 유실 아님)")
            r["expected"] = r["count"] + r["lost_mid"]
    else:
        r["notes"].append("저빈도 토픽 — 간격 분석 생략")

    # 샘플 역직렬화 + 페이로드 검증
    cls = _get_msg_class(info["type"])
    if cls is None:
        r["notes"].append("메시지 타입 미설치 — 역직렬화 검사 생략")
    else:
        from rclpy.serialization import deserialize_message
        if "_samples" in info:                      # rosbag2_py 경로
            samples = info["_samples"][:SAMPLES_PER_TOPIC]
        else:                                       # sqlite 경로
            n = len(rows)
            idx = sorted({int(i * (n - 1) / max(1, SAMPLES_PER_TOPIC - 1))
                          for i in range(min(SAMPLES_PER_TOPIC, n))})
            samples = fetch_samples(info, idx)
        for i, data in samples:
            r["deser_checked"] += 1
            try:
                msg = deserialize_message(data, cls)
            except Exception as e:
                r["deser_failed"] += 1
                if len(r["payload_issues"]) < 10:
                    r["payload_issues"].append(f"#{i} 역직렬화 실패: {e}")
                continue
            for issue in _check_payload(info["type"], msg):
                if len(r["payload_issues"]) < 10:
                    r["payload_issues"].append(f"#{i} {issue}")
        if r["deser_failed"]:
            r["level"] = FAIL
            r["notes"].append(
                f"역직렬화 실패 {r['deser_failed']}/{r['deser_checked']} — 데이터 손상")
        elif r["payload_issues"]:
            r["level"] = _worse(r["level"], WARN)
            r["notes"].append(f"페이로드 이상 {len(r['payload_issues'])}건")
    if progress:
        progress(f"검사 완료: {name} [{r['level']}]")
    return r


# ---------- 공개 API ----------

def analyze(bag_dir, progress=None):
    bag_dir = Path(bag_dir)
    if not bag_dir.is_dir():
        raise FileNotFoundError(f"클립 디렉터리가 아님: {bag_dir}")
    db3 = sorted(bag_dir.glob("*.db3"))
    if db3:
        data = _read_sqlite(db3[0], progress)
        fetch = _fetch_samples_sqlite
        storage = "sqlite3"
    else:
        data = _read_rosbag2py(bag_dir, progress)
        fetch = None
        storage = "mcap/other"

    all_bag = [b for info in data.values() for b, _, _ in info["rows"]]
    if not all_bag:
        raise RuntimeError("클립에 메시지가 하나도 없음")
    bag_t0, bag_t1 = min(all_bag), max(all_bag)

    topics = {}
    for name, info in sorted(data.items()):
        topics[name] = _analyze_topic(name, info, bag_t0, bag_t1, fetch, progress)

    # GNSS 품질 (NavSatFix 또는 상태 문자열 토픽이 있을 때만)
    gnss = _gnss_section(bag_dir, topics, progress)
    if gnss:
        for tname, q in gnss.items():
            if tname in topics:
                topics[tname]["level"] = _worse(topics[tname]["level"], q["level"])
                topics[tname]["notes"].extend("GNSS: " + n for n in q["notes"])
                topics[tname]["gnss"] = q

    level = OK
    for t in topics.values():
        level = _worse(level, t["level"])
    for q in (gnss or {}).values():
        level = _worse(level, q["level"])
    return {
        "gnss": gnss,
        "bag": str(bag_dir), "storage": storage,
        "start": round(bag_t0 / NS, 6),
        "duration": round((bag_t1 - bag_t0) / NS, 3),
        "total_msgs": sum(t["count"] for t in topics.values()),
        "total_mb": round(sum(t["bytes"] for t in topics.values()) / 1e6, 1),
        "level": level,
        "fail_topics": [n for n, t in topics.items() if t["level"] == FAIL],
        "warn_topics": [n for n, t in topics.items() if t["level"] == WARN],
        "topics": topics,
    }


def _gnss_section(bag_dir, topics, progress):
    has_fix = any(t["type"] == "sensor_msgs/msg/NavSatFix" for t in topics.values())
    has_status = any(n.endswith(("/pos_type", "/nav_status")) for n in topics)
    if not (has_fix or has_status):
        return None
    try:
        import gnss_tools
    except ImportError:
        return None
    if progress:
        progress("GNSS 품질 분석…")
    try:
        rep, _ = gnss_tools.analyze_bag(bag_dir)
        return rep
    except Exception as e:
        return {"(오류)": {"level": WARN, "count": 0,
                          "notes": [f"GNSS 분석 실패: {e}"]}}


def render_text(rep):
    L = []
    L.append(f"클립 진단: {rep['bag']}")
    L.append(f"  {rep['duration']:.1f}s, {rep['total_msgs']}개 메시지, "
             f"{rep['total_mb']:.1f} MB ({rep.get('storage', '?')})")
    L.append(f"  종합 판정: {rep['level']}"
             + (f"   FAIL={len(rep['fail_topics'])} WARN={len(rep['warn_topics'])}"
                if rep["level"] != OK else " — 이상 없음"))
    L.append("")
    order = {FAIL: 0, WARN: 1, OK: 2}
    for name, t in sorted(rep["topics"].items(),
                          key=lambda kv: (order[kv[1]["level"]], kv[0])):
        line = f"[{t['level']:4s}] {name}  ({t['type'].split('/')[-1]}, {t['count']}개"
        if "rate_hz" in t:
            line += f", {t['rate_hz']:.1f} Hz"
        if "lat_p95_ms" in t:
            line += f", 지연 p95 {t['lat_p95_ms']:.0f} ms"
        line += f", stamp={t.get('stamp_mode', '-')})"
        L.append(line)
        for note in t["notes"]:
            L.append(f"         - {note}")
        for g in t["gaps"]:
            L.append(f"           유실 지점 t0+{g['t']:.3f}s: "
                     f"간격 {g['dt_ms']:.0f} ms ≈ {g['est_lost']}프레임")
        for pi in t["payload_issues"]:
            L.append(f"           {pi}")
    if rep.get("gnss"):
        L.append("")
        L.append("GNSS 품질:")
        for tname, q in rep["gnss"].items():
            line = f"  [{q['level']:4s}] {tname}: {q.get('count', 0)}개"
            if "fix_ratio" in q:
                line += f", 유효 fix {q['fix_ratio']*100:.0f}%"
            if "hacc_med_m" in q:
                line += f", 수평정확도 중앙값 {q['hacc_med_m']} m / p95 {q['hacc_p95_m']} m"
            if "track_m" in q:
                line += f", 이동 {q['track_m']} m"
            L.append(line)
            for k in ("status_hist", "pos_type_hist", "nav_status_hist"):
                if q.get(k):
                    L.append(f"         {k}: {q[k]}")
            for n in q.get("notes", []):
                L.append(f"         - {n}")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description="rosbag2 클립 무결성 진단")
    ap.add_argument("bag_dir")
    ap.add_argument("--json", help="보고서를 JSON으로도 저장")
    args = ap.parse_args()
    rep = analyze(args.bag_dir, progress=lambda s: print("  ..", s, end="\r"))
    print(" " * 79, end="\r")
    print(render_text(rep))
    if args.json:
        Path(args.json).write_text(
            json.dumps(rep, ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"\nJSON 저장: {args.json}")
    return 0 if rep["level"] != FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
