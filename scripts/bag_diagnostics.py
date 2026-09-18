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
# 도착 시각만으로는 "멈췄다가 몰려 온 것"과 "빠진 것"을 못 가른다 — 드라이버나 DDS 가 잠깐 멈추면
# 긴 간격이 생긴다. 그래서 센서가 매긴 번호/시각이 있으면 그걸로 센다:
#   - Ouster lidar_packets : 패킷 안의 frame_id · measurement_id (Ouster 패킷엔 header 가 없다)
#   - Ouster imu_packets   : 패킷 안의 센서 시각
#   - FLIR 카메라           : image_raw/metadata 의 camera_frame_id — 같은 stamp 를 쓰는 image_raw ·
#                            camera_info · image_rgb/compressed 에도 적용
# 그 밖의 토픽은 도착 시각 간격으로 세되, 긴 간격 뒤에 메시지가 몰려 와서 시간선을 따라잡으면
# 지터(늦게 왔을 뿐)로 본다. 빠진 거라면 시간선이 영영 한 주기 밀린다.
#
# 단독 실행:  python3 bag_diagnostics.py <clip_dir> [--json out.json]
# 모듈 사용:  report = analyze(clip_dir); print(render_text(report))

import argparse
import collections
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
LAT_GROWTH_MS = 100.0       # 클립 앞 1/3 → 뒤 1/3 지연 중앙값이 이만큼 늘면 백로그 (쌓이는 중)
JITTER_LOOKAHEAD = 64       # 긴 간격 뒤 이 개수 안에서 시간선을 따라잡으면 지터
JITTER_WINDOW_S = 0.5       # … 단 이 시간 안에서
MAX_SEQ_JUMP = 100_000      # 센서 번호가 이보다 크게 뛰면 재시작으로 보고 유실로 세지 않는다
CADENCE_MIN_SKIPS = 20      # 한 칸 빈 자리가 이만큼은 있어야 장비 출력 패턴인지 본다
CADENCE_REGULAR = 0.8       # 빈 자리 사이 간격이 최빈값 ±1 안에 드는 비율이 이 이상이면 규칙적
CADENCE_SPREAD = 0.5        # 클립을 10구간으로 나눠 구간마다 빈 비율이 전체의 0.5 ~ 2배 안 = 고르게 퍼짐

PACKET_TYPE = "ouster_sensor_msgs/msg/PacketMsg"
FLIR_META_TYPE = "flir_spinnaker_camera/msg/FlirMetadata"
# 라이다 한 바퀴를 모아 내는 토픽 — stamp 가 스캔 시작이라 한 바퀴(1/rate)만큼은 원래 늦다
SCAN_TYPES = {"sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Image", "sensor_msgs/msg/LaserScan"}

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

def _head_len(type_name):
    """메시지 앞 몇 바이트를 읽을지. 센서 번호를 꺼낼 타입만 더 읽는다 (나머지는 header 12바이트)."""
    if type_name == PACKET_TYPE:
        return 8 + 64           # CDR(캡슐 4 + 배열 길이 4) + 패킷 앞 64바이트
    if type_name == FLIR_META_TYPE:
        return 512              # 작은 메시지 — camera_frame_id 까지
    return 12


def _db3_files(bag_dir):
    """<이름>_0.db3, _1.db3, … 를 번호 순으로 — 나뉜 bag(record_split_sec, ros2 bag record
    --max-bag-*)도 이어서 읽는다. 문자열 정렬이면 _10 이 _2 앞에 온다."""
    def index(path):
        tail = path.stem.rsplit("_", 1)[-1]
        return (int(tail) if tail.isdigit() else -1, path.name)
    return sorted(Path(bag_dir).glob("*.db3"), key=index)


def _read_sqlite(db_paths, progress):
    """{topic: {'type':.., 'rows':[(bag_ns, hdr_ns|None, size)], '_ids':[(db, rowid)], '_heads'?:[bytes]}}

    파일이 여러 개면 시간 순으로 이어 붙인다 (나뉜 파일은 시간 구간이 겹치지 않는다).
    """
    out = {}
    for db_path in db_paths:
        con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
        topics = {tid: (name, typ) for tid, name, typ in
                  con.execute("SELECT id, name, type FROM topics")}
        for tid, (name, typ) in sorted(topics.items(), key=lambda kv: kv[1][0]):
            info = out.setdefault(name, {"type": typ, "rows": [], "_ids": []})
            n = _head_len(typ)
            heads = info.setdefault("_heads", []) if n > 12 else None
            for rid, bag_ns, head, size in con.execute(
                    f"SELECT id, timestamp, substr(data,1,{n}), length(data) "
                    "FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
                info["rows"].append((bag_ns, _parse_header_ns(head), size or 0))
                info["_ids"].append((str(db_path), rid))
                if heads is not None:
                    heads.append(bytes(head or b""))
            if progress:
                progress(f"읽는 중: {name} ({len(info['rows'])}개)")
        con.close()
    return out


def _fetch_samples_sqlite(info, indices):
    out, cons = [], {}
    try:
        for i in indices:
            db, rid = info["_ids"][i]
            if db not in cons:
                cons[db] = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
            row = cons[db].execute("SELECT data FROM messages WHERE id=?", (rid,)).fetchone()
            out.append((i, bytes(row[0]) if row and row[0] is not None else b""))
    finally:
        for con in cons.values():
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
        n = _head_len(info["type"])
        if n > 12:
            info.setdefault("_heads", []).append(bytes(data[:n]))
        # 대략 SAMPLES_PER_TOPIC개가 남도록 성긴 간격으로 본문 보관
        if len(info["_samples"]) < SAMPLES_PER_TOPIC * 4 and \
                len(info["rows"]) % max(1, len(info["rows"]) // SAMPLES_PER_TOPIC + 1) == 0:
            info["_samples"].append((len(info["rows"]) - 1, bytes(data)))
        n_read += 1
        if progress and n_read % 2000 == 0:
            progress(f"읽는 중: {n_read}개…")
    for info in out.values():
        order = sorted(range(len(info["rows"])), key=lambda i: info["rows"][i][0])
        info["rows"] = [info["rows"][i] for i in order]
        if "_heads" in info:                     # 센서 번호와 행이 어긋나지 않게 같이 정렬
            info["_heads"] = [info["_heads"][i] for i in order]
    return out


# ---------- 센서가 매긴 번호 / 시각 ----------

def _packet_bytes(head):
    """PacketMsg CDR → 패킷 앞부분 (uint8[] buf)."""
    if not head or len(head) < 8:
        return b""
    n = struct.unpack_from("<I", head, 4)[0]
    return bytes(head[8:8 + n])


def _ouster_lidar_ids(bufs):
    """lidar 패킷마다 연속 번호 → (번호들, 한 바퀴 도는 값, 설명). 모르는 형식이면 None.

    새 프로파일(RNG19_… 등): 패킷 머리 32B 의 frame_id(u16 @2), 첫 열 머리의 measurement_id(u16 @40).
    LEGACY: 패킷 머리 없이 첫 열이 timestamp(u64 @0) · measurement_id(u16 @8) · frame_id(u16 @10).
    패킷당 열 수는 metadata 없이 데이터에서 — 같은 프레임 안 연속 패킷의 measurement_id 차이.
    (/ouster/metadata 는 기동 때 한 번만 나와서 클립에 없는 게 보통이다.)
    """
    def modern(b):
        return struct.unpack_from("<H", b, 2)[0], struct.unpack_from("<H", b, 40)[0]

    def legacy(b):
        mid, fid = struct.unpack_from("<HH", b, 8)
        return fid, mid

    for layout in (modern, legacy):
        try:
            vals = [layout(b) for b in bufs]
        except struct.error:
            continue
        steps = [m2 - m1 for (f1, m1), (f2, m2) in zip(vals, vals[1:]) if f1 == f2 and m2 > m1]
        if not vals or len(steps) < len(vals) // 2:
            continue
        step = int(statistics.median(steps))
        mids = [m for _, m in vals]
        if step <= 0 or sum(1 for m in mids if m % step == 0) < 0.99 * len(mids):
            continue
        cols = next((c for c in (512, 1024, 2048, 4096) if c >= max(mids) + step), None)
        if cols is None:
            continue
        per_frame = cols // step
        return ([f * per_frame + m // step for f, m in vals], 65536 * per_frame,
                f"센서 패킷 번호 (한 바퀴 {per_frame}패킷)")
    return None


def _ouster_imu_stamps(bufs):
    """LEGACY IMU 패킷(48B): 진단·가속·자이로 시각(u64 ns) + 값. 가속 시각을 쓴다. 다른 형식이면 None."""
    if not bufs or any(len(b) != 48 for b in bufs):
        return None
    return [struct.unpack_from("<Q", b, 8)[0] for b in bufs]


def _cdr_skip_string(buf, off):
    off = (off + 3) & ~3
    n = struct.unpack_from("<I", buf, off)[0]
    return off + 4 + n


def _flir_frame_id(head):
    """FlirMetadata CDR → (header stamp ns, camera_frame_id). 실패하면 None."""
    try:
        b = head[4:]                                  # CDR 정렬은 캡슐 헤더 뒤부터 센다
        sec, nsec = struct.unpack_from("<iI", b, 0)
        off = _cdr_skip_string(b, 8)                  # header.frame_id
        off = ((off + 3) & ~3) + 12                   # width, height, step
        off = _cdr_skip_string(b, off)                # encoding
        off = _cdr_skip_string(b, off)                # pixel_format
        off = (off + 7) & ~7
        return sec * 10**9 + nsec, struct.unpack_from("<Q", b, off)[0]
    except (struct.error, TypeError):
        return None


def _sensor_sequences(data):
    """토픽별 센서 기준 판정 재료.

    {name: {"times", "ids", "wrap", "basis"}}  — 번호로 세는 토픽
    {name: {"stamps", "basis"}}                 — 센서 시각 간격으로 세는 토픽
    """
    out = {}
    frames = {}                                      # 카메라 네임스페이스 -> {stamp: frame_id}
    for name, info in data.items():
        if info["type"] == FLIR_META_TYPE and info.get("_heads"):
            ns = name[:-len("/image_raw/metadata")] if name.endswith("/image_raw/metadata") \
                else name.rsplit("/", 1)[0]
            table = {}
            for head in info["_heads"]:
                got = _flir_frame_id(head)
                if got:
                    table[got[0]] = got[1]
            if table:
                frames[ns] = table
    for name, info in data.items():
        for ns, table in frames.items():
            if not name.startswith(ns + "/"):
                continue
            pairs = sorted((h, table[h]) for _, h, _ in info["rows"] if h in table)
            if len(pairs) >= max(MIN_MSGS_FOR_GAPS, 0.95 * len(info["rows"])):
                out[name] = {"times": [p[0] for p in pairs], "ids": [p[1] for p in pairs],
                             "wrap": None, "basis": "카메라 frame_id"}
            break

    for name, info in data.items():
        if info["type"] != PACKET_TYPE or not info.get("_heads"):
            continue
        bufs = [_packet_bytes(h) for h in info["_heads"]]
        stamps = _ouster_imu_stamps(bufs)
        if stamps:
            out[name] = {"stamps": stamps, "basis": "센서 IMU 시각"}
            continue
        got = _ouster_lidar_ids(bufs)
        if got:
            ids, wrap, basis = got
            out[name] = {"times": [b for b, _, _ in info["rows"]], "ids": ids,
                         "wrap": wrap, "basis": basis}
    return out


def _scan_topics(data, sensors):
    """라이다 패킷과 같은 네임스페이스에서 한 바퀴를 모아 내는 토픽 (points · *_image · scan)."""
    lidar_ns = {n.rsplit("/", 1)[0] for n, s in sensors.items()
                if "ids" in s and data[n]["type"] == PACKET_TYPE}
    return {n for n, info in data.items()
            if info["type"] in SCAN_TYPES and n.rsplit("/", 1)[0] in lidar_ns}


def _seq_loss(ids, wrap=None):
    """센서 번호 열 → (유실, 중복, 재시작, [(위치, 빠진 개수)])"""
    lost = dup = restart = 0
    where = []
    for i, (a, b) in enumerate(zip(ids, ids[1:])):
        d = (b - a) % wrap if wrap else b - a
        if d == 1:
            continue
        if d == 0:
            dup += 1
        elif d < 0 or d > MAX_SEQ_JUMP or (wrap and d > wrap // 2):
            restart += 1
        else:
            lost += d - 1
            where.append((i, d - 1))
    return lost, dup, restart, where


def _caught_up(stamps, i, med):
    """i→i+1 의 긴 간격을 뒤 메시지들이 몰려 와서 메우는가 (늦게 왔을 뿐 빠진 게 없다).

    k 개 뒤까지의 시간이 (k+1) 주기 + 반 주기 안이면 시간선을 따라잡은 것. 빠진 거라면 시간선이
    영영 한 주기 밀려서 따라잡지 못한다. k=1 이 예전의 "다음 간격과 합이 2.5주기 이내" 규칙이다.
    """
    base = stamps[i]
    limit = base + JITTER_WINDOW_S * NS
    for k in range(1, JITTER_LOOKAHEAD + 1):
        j = i + 1 + k
        if j >= len(stamps) or stamps[j] > limit:
            break
        if stamps[j] - base <= (k + 1.5) * med:
            return True
    return False


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

def _loss_by_sequence(r, sensor, bag_t0):
    """센서 번호로 유실 · 중복 · 재시작을 센다."""
    lost, dup, restart, where = _seq_loss(sensor["ids"], sensor.get("wrap"))
    times = sensor["times"]
    r["loss_basis"] = sensor["basis"]
    r["lost_mid"] = lost
    for i, n in where[:20]:
        r["gaps"].append({"t": round((times[i] - bag_t0) / NS, 3),
                          "dt_ms": round((times[i + 1] - times[i]) / 1e6, 1), "est_lost": n})
    if dup:
        r["level"] = _worse(r["level"], WARN)
        r["notes"].append(f"같은 {sensor['basis']}가 {dup}번 (중복)")
    if restart:
        r["level"] = _worse(r["level"], WARN)
        r["notes"].append(f"{sensor['basis']}가 {restart}번 크게 뛰거나 되돌아감 (센서/노드 재시작?) — 유실로 세지 않음")


def _sender_cadence(skips, n_intervals):
    """한 칸씩 빈 자리(간격 인덱스)들이 장비 출력 주기처럼 규칙적이고 클립 내내 고른가.

    OxTS RT2000 은 12 ms 격자에서 4칸마다 한 칸을 비우고 보낸다 (12·12·12·24 ms → 실제 67 Hz).
    2026-09-19 실측: GNSS NIC 에 도착하는 패킷부터 67개/초이고 NIC·소켓 드롭 0 — 녹화에서 빠진 게
    아니다. 전송·녹화 유실은 부하 따라 몰리거나 무작위로 흩어져서, 같은 비율의 무작위 유실을 흉내 내면
    빈 자리 사이 간격이 최빈값 ±1 안에 드는 비율이 40% 남짓이다 (RT2000 은 98%).
    """
    if len(skips) < CADENCE_MIN_SKIPS:
        return None
    spacing = [b - a for a, b in zip(skips, skips[1:])]
    mode = collections.Counter(spacing).most_common(1)[0][0]
    if sum(1 for x in spacing if abs(x - mode) <= 1) < CADENCE_REGULAR * len(spacing):
        return None
    frac = len(skips) / n_intervals
    bins = [0] * 10
    for i in skips:
        bins[min(9, i * 10 // n_intervals)] += 1
    per_bin = n_intervals / 10
    if not all(CADENCE_SPREAD * frac <= b / per_bin <= frac / CADENCE_SPREAD for b in bins):
        return None
    return mode


def _loss_by_gaps(r, stamps, times, bag_t0):
    """간격으로 유실을 센다. stamps 가 센서 시각이면 times(같은 순서의 bag 시각)로 위치를 적는다."""
    dts = [(b - a) for a, b in zip(stamps, stamps[1:])]
    med = statistics.median(dts)
    if med <= 0:
        return
    gaps = []
    for i, dt in enumerate(dts):
        if dt <= med * GAP_FACTOR or _caught_up(stamps, i, med):
            continue
        est = int(round(dt / med)) - 1
        if est >= 1:
            gaps.append((i, dt, est))
    # 거의 다 한 칸짜리이고 규칙적이면 보내는 쪽의 출력 주기 — 유실로 세지 않는다 (두 칸 이상 빈 건 그대로 유실)
    singles = [i for i, _dt, est in gaps if est == 1]
    mode = _sender_cadence(singles, len(dts)) if len(singles) >= 0.95 * len(gaps) else None
    if mode:
        period = (stamps[-1] - stamps[0]) / (len(stamps) - 1)          # 빈칸까지 친 실제 평균 주기
        r["cadence"] = {"skips": len(singles), "every": mode, "grid_ms": round(med / 1e6, 1),
                        "rate_hz": round(NS / period, 1)}
        # 남은 긴 간격은 격자 주기가 아니라 실제 평균 주기로 따라잡았는지 다시 본다. 격자(med) 기준이면
        # 몇 칸마다 한 칸씩 비는 스트림은 시간선을 영영 못 따라잡아서, 전달이 잠깐 멈췄다 몰려 온 것
        # (46 ms 멈춤 → 2 ms 안에 3개)까지 유실로 셌다.
        # 리듬을 깨는 한 칸(주기가 4칸인데 1~2칸 만에 또 빔)은 출력 패턴이 아니라 유실이다.
        off_beat = {b for a, b in zip(singles, singles[1:]) if b - a < mode - 1}
        gaps = [g for g in gaps
                if g[0] in off_beat or (g[2] != 1 and not _caught_up(stamps, g[0], period))]
    for i, dt, est in gaps:
        r["lost_mid"] += est
        if len(r["gaps"]) < 20:
            at = times[i] if i < len(times) else stamps[i]
            r["gaps"].append({"t": round((at - bag_t0) / NS, 3),
                              "dt_ms": round(dt / 1e6, 1), "est_lost": est})


def _judge_latency(r, lat, scan):
    """지연 판정. 백로그 = 클립 동안 지연이 계속 늘어나는 것. 일정하게 늦은 건 처리 지연이다."""
    p95 = r["lat_p95_ms"]
    third = len(lat) // 3
    early = late = None
    if third >= 3:
        early, late = statistics.median(lat[:third]), statistics.median(lat[-third:])
    # 라이다 스캔 토픽은 stamp 가 한 바퀴의 시작이라 한 바퀴(1/rate)는 원래 늦다
    inherent = r.get("dt_median_ms", 0.0) if scan else 0.0
    if early is not None and late - early > LAT_GROWTH_MS:
        r["level"] = _worse(r["level"], WARN)
        r["notes"].append(f"수신 지연이 클립 동안 계속 늘어남 ({early:.0f} → {late:.0f} ms) — 전송 백로그")
    elif p95 - inherent > LAT_WARN_MS:
        r["level"] = _worse(r["level"], WARN)
        extra = f", 스캔 한 바퀴 {inherent:.0f} ms 를 빼고도 {p95 - inherent:.0f} ms" if scan else ""
        r["notes"].append(f"수신 지연 p95 {p95:.0f} ms{extra} — 클립 내내 일정하게 늦음 "
                          "(처리 지연, 쌓이지는 않음)")
    elif scan and p95 > LAT_WARN_MS:
        r["notes"].append(f"지연 p95 {p95:.0f} ms = 스캔 한 바퀴 {inherent:.0f} ms (stamp 가 스캔 시작) "
                          f"+ 처리·전송 {p95 - inherent:.0f} ms — 정상")


def _analyze_topic(name, info, bag_t0, bag_t1, fetch_samples, progress, sensor=None, scan=False):
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

    # 수신 지연 (header 모드에서만 의미 있음). 판정은 주기를 안 뒤에 (라이다 스캔은 한 바퀴만큼 원래 늦다)
    lat = []
    if use_header:
        lat = [(b - h) / 1e6 for b, h, _ in rows if h is not None]     # 도착 순서 — 추세를 본다
        ordered = sorted(lat)
        r["lat_med_ms"] = round(ordered[len(ordered) // 2], 1)
        r["lat_p95_ms"] = round(ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))], 1)
        r["lat_max_ms"] = round(ordered[-1], 1)

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
            if sensor and "ids" in sensor:
                _loss_by_sequence(r, sensor, bag_t0)
                worst = max(dts)
                if not r["lost_mid"] and worst > med * GAP_FACTOR * 2:
                    r["notes"].append(
                        f"도착은 최대 {worst / 1e6:.0f} ms 멈췄다가 몰려 옴 — {sensor['basis']}로는 "
                        "빠진 것 없음 (드라이버·전송 지터)")
            elif sensor and "stamps" in sensor:
                r["loss_basis"] = sensor["basis"]
                _loss_by_gaps(r, sorted(sensor["stamps"]), [b for b, _, _ in rows], bag_t0)
            else:
                _loss_by_gaps(r, stamps, stamps, bag_t0)
            if r["lost_mid"]:
                r["level"] = FAIL
                basis = f" ({r['loss_basis']} 기준)" if r.get("loss_basis") else ""
                r["notes"].insert(0, f"스트림 중간 유실 {r['lost_mid']}프레임{basis}")
            cad = r.get("cadence")
            if cad:
                r["rate_hz"] = cad["rate_hz"]
                r["level"] = _worse(r["level"], WARN)
                r["notes"].append(
                    f"보내는 간격이 고르지 않음 — {cad['grid_ms']:g} ms 격자에서 {cad['every']}칸마다 한 칸씩 비어 "
                    f"실제 {cad['rate_hz']:g} Hz ({cad['skips']}칸, 클립 내내 같은 비율). 녹화·전송 유실은 이렇게 "
                    "규칙적이지 않다 — 장비가 이렇게 보내는 것 (장비의 출력 주기 설정 확인)")

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

    if lat:
        _judge_latency(r, lat, scan)

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
    db3 = _db3_files(bag_dir)
    if db3:
        data = _read_sqlite(db3, progress)
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

    sensors = _sensor_sequences(data)
    scans = _scan_topics(data, sensors)
    topics = {}
    for name, info in sorted(data.items()):
        topics[name] = _analyze_topic(name, info, bag_t0, bag_t1, fetch, progress,
                                      sensors.get(name), name in scans)

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
        line += f", stamp={t.get('stamp_mode', '-')}"
        if t.get("loss_basis"):
            line += f", 유실 기준={t['loss_basis']}"
        line += ")"
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
