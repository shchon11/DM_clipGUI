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
import bisect
import collections
import json
import math
import sqlite3
import statistics
import struct
import sys
import unicodedata
from pathlib import Path

NS = 1e9
SAMPLES_PER_TOPIC = 25      # 역직렬화/페이로드 검사 샘플 수
MIN_MSGS_FOR_GAPS = 10      # 이보다 적으면 간격 분석 생략 (저빈도 토픽)
GAP_FACTOR = 1.6            # dt > median*GAP_FACTOR 이면 유실 후보
LAT_WARN_MS = 200.0         # 수신 지연 p95 가 이보다 크면 '참고' 로 적는다 (일정하면 기록된 시각엔 영향 없음 — 판정은 안 올림)
# 얼마나 잃어야 '문제' 인가 — 900장에 1장 빠진 걸 FAIL 로 올리면 진짜 문제가 묻힌다 (2026-09-23 지적: "너무 빡세다").
LOSS_INFO_MAX = 2           # 이 개수 이하이면서 아래 비율 이하면 참고로만 (판정 안 올림)
LOSS_INFO_RATIO = 0.002     # 0.2%
LOSS_FAIL_MIN = 10          # 이만큼 넘게 잃거나
LOSS_FAIL_RATIO = 0.01      # 1% 넘게 잃으면 '문제'. 그 사이는 '확인'
FROZEN_INFO_S = 2.0         # 셔터 보정(NUC) 멈춤이 이보다 짧고 한 번뿐이면 참고로만 — 열화상의 정상 동작
CLOCK_AHEAD_MS = 20.0       # 기록 시각이 받은 시각보다 늘(중앙값) 이만큼 넘게 미래면 센서 시계가 PC 보다 앞선 것
STAMP_OFFSET_MS = 1000.0    # 지연이 이보다 크게 늘 일정하면 처리가 밀린 게 아니라 기록 시각의 기준이 다른 것 (UTC↔TAI 37초 등)
TRUNC_WARN_S = 0.5          # 녹화 앞/뒤로 이보다 길게(그리고 3주기 넘게) 비면 센서가 늦게 들어왔거나 먼저 끊긴 것
LAT_GROWTH_MS = 100.0       # 클립 앞 1/3 → 뒤 1/3 지연 중앙값이 이만큼 늘면 백로그 (쌓이는 중)
JITTER_LOOKAHEAD = 64       # 긴 간격 뒤 이 개수 안에서 시간선을 따라잡으면 지터
JITTER_WINDOW_S = 0.5       # … 단 이 시간 안에서
MAX_SEQ_JUMP = 100_000      # 센서 번호가 이보다 크게 뛰면 재시작으로 보고 유실로 세지 않는다
STALE_WINDOW = 256          # 최근 이만큼의 번호 안으로 되돌아가면 '이전 프레임 번호가 다시 실린' 것 (재시작 아님)
CADENCE_MIN_SKIPS = 20      # 한 칸 빈 자리가 이만큼은 있어야 장비 출력 패턴인지 본다
CADENCE_REGULAR = 0.8       # 빈 자리 사이 간격이 최빈값 ±1 안에 드는 비율이 이 이상이면 규칙적
CADENCE_SPREAD = 0.5        # 클립을 10구간으로 나눠 구간마다 빈 비율이 전체의 0.5 ~ 2배 안 = 고르게 퍼짐

PACKET_TYPE = "ouster_sensor_msgs/msg/PacketMsg"
FLIR_META_TYPE = "flir_spinnaker_camera/msg/FlirMetadata"
# 라이다 한 바퀴를 모아 내는 토픽 — stamp 가 스캔 시작이라 한 바퀴(1/rate)만큼은 원래 늦다
SCAN_TYPES = {"sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Image", "sensor_msgs/msg/LaserScan"}
IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
FUTURE_STAMP_MS = 200      # header.stamp 가 수신(bag) 시각보다 이만큼 넘게 미래면 시계가 튄 것
FROZEN_MID_BYTES = 256      # 멈춘 화면 판정: 메시지 한가운데서 읽는 바이트 수 (센서 잡음 때문에 새 프레임은 늘 다르다)
LIDAR_PHASE_MAX_OFFSET_MS = 50.0   # 라이다 센서 시각이 PC(수신) 시각과 이보다 더 다르면 같은 시계가 아니라 촬영 각도를 못 잰다
LIDAR_PHASE_LOCKED_DEG = 15.0      # 촬영 때 라이다 각도의 흔들림(p95)이 이 안이면 '맞물림' — OS-2 위상 고정 정확도 ±5°
FROZEN_MIN_REPEATS = 3      # 같은 화면이 이보다 많이 연달아 나오면 '멈춤' (한두 장은 우연 · 압축 동일 가능)

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


def header_usable(bag_ns, hdr_ns):
    """이 메시지의 header.stamp 를 믿을 만한가 — bag 시각과 5분 안 (센서 내부 시계·0 stamp 는 탈락)."""
    return hdr_ns is not None and abs(hdr_ns - bag_ns) < 300 * NS


def use_header_stamps(rows):
    """[(bag_ns, header_ns|None, …)] → 토픽의 시간 기준을 header.stamp 로 할까: 95% 이상 믿을 만할 때.
    rqt_bag_hdr 의 타임라인도 같은 규칙을 쓴다."""
    return sum(1 for b, h, *_ in rows if header_usable(b, h)) >= 0.95 * len(rows)


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


def _read_sqlite(db_paths, progress, hold=None):
    """{topic: {'type':.., 'rows':[(bag_ns, hdr_ns|None, size)], '_ids':[(db, rowid)], '_heads'?:[bytes]}}

    파일이 여러 개면 시간 순으로 이어 붙인다 (나뉜 파일은 시간 구간이 겹치지 않는다).
    hold: 몇백 행마다 부른다 — 부른 쪽이 여기서 기다리게 할 수 있다 (GUI: 녹화 중이면 진단을 멈춤).
    """
    out = {}
    n_read = 0
    for db_path in db_paths:
        con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
        topics = {tid: (name, typ) for tid, name, typ in
                  con.execute("SELECT id, name, type FROM topics")}
        for tid, (name, typ) in sorted(topics.items(), key=lambda kv: kv[1][0]):
            info = out.setdefault(name, {"type": typ, "rows": [], "_ids": []})
            n = _head_len(typ)
            heads = info.setdefault("_heads", []) if n > 12 else None
            # 영상은 한가운데 몇백 바이트도 읽는다 — 같은 화면이 연달아 오는지 (열화상 셔터 보정 NUC 동안 멈춤)
            mids = info.setdefault("_mid", []) if typ in IMAGE_TYPES else None
            mid_sql = (f"substr(data, max(1, length(data) / 2), {FROZEN_MID_BYTES})" if mids is not None
                       else "NULL")
            for rid, bag_ns, head, size, mid in con.execute(
                    f"SELECT id, timestamp, substr(data,1,{n}), length(data), {mid_sql} "
                    "FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
                info["rows"].append((bag_ns, _parse_header_ns(head), size or 0))
                info["_ids"].append((str(db_path), rid))
                if heads is not None:
                    heads.append(bytes(head or b""))
                if mids is not None:
                    mids.append(bytes(mid or b""))
                n_read += 1
                if hold and n_read % 500 == 0:
                    hold()
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


def _read_rosbag2py(bag_dir, progress, hold=None):
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
        if info["type"] in IMAGE_TYPES:
            half = max(0, len(data) // 2 - 1)
            info.setdefault("_mid", []).append(bytes(data[half:half + FROZEN_MID_BYTES]))
        # 대략 SAMPLES_PER_TOPIC개가 남도록 성긴 간격으로 본문 보관
        if len(info["_samples"]) < SAMPLES_PER_TOPIC * 4 and \
                len(info["rows"]) % max(1, len(info["rows"]) // SAMPLES_PER_TOPIC + 1) == 0:
            info["_samples"].append((len(info["rows"]) - 1, bytes(data)))
        n_read += 1
        if hold and n_read % 500 == 0:
            hold()
        if progress and n_read % 2000 == 0:
            progress(f"읽는 중: {n_read}개…")
    for info in out.values():
        order = sorted(range(len(info["rows"])), key=lambda i: info["rows"][i][0])
        info["rows"] = [info["rows"][i] for i in order]
        for key in ("_heads", "_mid"):            # 센서 번호 · 화면과 행이 어긋나지 않게 같이 정렬
            if key in info:
                info[key] = [info[key][i] for i in order]
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


def _ouster_columns(bufs):
    """lidar 패킷마다 첫 열의 (센서 시각 ns, measurement_id) + 한 바퀴 열 수. 모르는 형식이면 None.
    새 프로파일: 열 머리가 패킷 머리 32B 뒤 — timestamp(u64 @32) · measurement_id(u16 @40). LEGACY: @0 · @8."""
    for ts_at, mid_at, fid_at in ((32, 40, 2), (0, 8, 10)):
        try:
            vals = [(struct.unpack_from("<Q", b, ts_at)[0], struct.unpack_from("<H", b, mid_at)[0],
                     struct.unpack_from("<H", b, fid_at)[0]) for b in bufs]
        except struct.error:
            continue
        steps = [m2 - m1 for (_, m1, f1), (_, m2, f2) in zip(vals, vals[1:]) if f1 == f2 and m2 > m1]
        if not vals or len(steps) < len(vals) // 2:
            continue
        step = int(statistics.median(steps))
        mids = [m for _, m, _ in vals]
        if step <= 0 or sum(1 for m in mids if m % step == 0) < 0.99 * len(mids):
            continue
        cols = next((c for c in (512, 1024, 2048, 4096) if c >= max(mids) + step), None)
        if cols:
            return [(t, m) for t, m, _ in vals], cols
    return None


def _lidar_phase(data, topics):
    """카메라가 찍을 때(header.stamp = 노출 시작) 라이다 빔이 어느 각도였나 — 라이다 위상 고정 확인.

    각도는 Ouster 라이다 좌표계(0° = 커넥터 쪽, 위에서 볼 때 반시계 +, θ = 360·(1 − measurement_id/열 수)) —
    드라이버 phase_lock_offset 과 같은 기준이라 어긋난 만큼 그 값에서 빼면 된다. 라이다 패킷의 열 시각(센서 PTP
    시계)을 보간해 잰다 — 라이다와 PC 가 같은 시계(PTP)여야 한다. 카메라 30 Hz · 라이다 10 Hz 면 한 바퀴에 세 번
    찍으므로 120° 격자로 접어서 본다 (0/120/240° 에서 몇 도).
    """
    lidar = next((n for n, info in sorted(data.items())
                  if info["type"] == PACKET_TYPE and n.endswith("lidar_packets") and info.get("_heads")), None)
    if lidar is None:
        return None
    got = _ouster_columns([_packet_bytes(h) for h in data[lidar]["_heads"]])
    if not got:
        return None
    cols, width = got
    rows = data[lidar]["rows"]
    gap = statistics.median((c[0] - r[0]) / 1e6 for c, r in zip(cols, rows))
    out = {"lidar": lidar, "cams": {}}
    if abs(gap) > LIDAR_PHASE_MAX_OFFSET_MS:
        out["error"] = (f"라이다 센서 시각이 PC 시각과 {gap:+.0f} ms 달라 촬영 때 라이다 각도를 못 잽니다 "
                        "(라이다 PTP 가 아직 덜 맞음)" if abs(gap) < 10_000 else
                        "라이다 센서 시각의 기준이 PC 와 달라(PTP 시각이 아님 — 부팅 후 경과 시각 등) 촬영 때 라이다 각도를 "
                        "못 잽니다")
        return out
    cols = sorted(cols)
    stamps = [t for t, _ in cols]
    rates = [((m1 - m0) % width) / (t1 - t0) for (t0, m0), (t1, m1) in zip(cols, cols[1:]) if 0 < t1 - t0 < 5e6]
    if not rates:
        return None
    rev_hz = statistics.median(rates) * NS / width
    out["rev_hz"] = round(rev_hz, 2)
    lidar_ns = _sensor_of(lidar)
    for name, t in sorted(topics.items()):
        ns = _sensor_of(name)
        if t["type"] not in IMAGE_TYPES or t.get("stamp_mode") != "header" or ns == lidar_ns or ns in out["cams"]:
            continue
        if not t.get("rate_hz"):
            continue
        per_rev = t["rate_hz"] / rev_hz
        shots = round(per_rev)
        if shots < 1 or abs(per_rev - shots) > 0.02 * per_rev:
            out["cams"][ns] = {"note": f"카메라 {t['rate_hz']:g} Hz · 라이다 {rev_hz:.1f} Hz — 정수배가 아니라 찍을 때 "
                                       "각도가 계속 바뀜"}
            continue
        grid = 360.0 / shots
        residuals = []
        for _, h, _ in data[name]["rows"]:
            i = bisect.bisect_right(stamps, h or 0)
            if not h or i == 0 or i >= len(stamps):
                continue
            (t0, m0), (t1, m1) = cols[i - 1], cols[i]
            if not 0 < t1 - t0 < 5e6:                         # 패킷이 빠진 자리 — 보간하지 않는다
                continue
            mid = (m0 + ((m1 - m0) % width) * (h - t0) / (t1 - t0)) % width
            theta = (360.0 * (1 - mid / width)) % 360
            residuals.append((theta + grid / 2) % grid - grid / 2)
        if len(residuals) < MIN_MSGS_FOR_GAPS:
            continue
        # 격자 경계(±grid/2) 근처에서 갈라지지 않게, 중앙값 기준으로 다시 접는다
        center = statistics.median(residuals)
        folded = sorted((r - center + grid / 2) % grid - grid / 2 for r in residuals)
        center = (center + statistics.median(folded) + grid / 2) % grid - grid / 2
        spread = sorted(abs(x - statistics.median(folded)) for x in folded)
        out["cams"][ns] = {"shots_per_rev": shots, "grid_deg": round(grid, 1), "offset_deg": round(center, 1),
                           "spread_deg": round(spread[min(len(spread) - 1, int(0.95 * len(spread)))], 1),
                           "n": len(residuals)}
    return out


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
    """FlirMetadata CDR → (header stamp ns, camera_frame_id, camera_timestamp_ns). 실패하면 None."""
    try:
        b = head[4:]                                  # CDR 정렬은 캡슐 헤더 뒤부터 센다
        sec, nsec = struct.unpack_from("<iI", b, 0)
        off = _cdr_skip_string(b, 8)                  # header.frame_id
        off = ((off + 3) & ~3) + 12                   # width, height, step
        off = _cdr_skip_string(b, off)                # encoding
        off = _cdr_skip_string(b, off)                # pixel_format
        off = (off + 7) & ~7
        frame_id, camera_ns = struct.unpack_from("<QQ", b, off)
        return sec * 10**9 + nsec, frame_id, camera_ns
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
                    table[got[0]] = got[1:]
            if table:
                frames[ns] = table
    for name, info in data.items():
        for ns, table in frames.items():
            if not name.startswith(ns + "/"):
                continue
            pairs = sorted((h, *table[h]) for _, h, _ in info["rows"] if h in table)
            if len(pairs) >= max(MIN_MSGS_FOR_GAPS, 0.95 * len(info["rows"])):
                # 유실 지점의 간격은 카메라 시각으로 적는다 — 열화상은 header.stamp 가 PC 수신 시각이라 지터가 크다
                out[name] = {"times": [p[0] for p in pairs], "ids": [p[1] for p in pairs],
                             "gap_times": [p[2] for p in pairs], "gap_basis": "카메라 시각",
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
    """센서 번호 열 → (유실, 중복, 재시작, [(앞 정상 위치, 뒤 위치, 빠진 개수, 사이의 이전 번호 메시지 수)], [(위치, 몇 장 전 번호)])

    마지막 결과는 '이전 프레임 번호가 다시 실린' 메시지 — 번호가 앞서 이미 본 값으로 잠깐 되돌아갔다가 곧바로
    원래 흐름으로 이어지는 경우. 2026-09-21 FLIR 클립: 영상은 새 프레임인데 frame_id 와 카메라 시각만 정확히
    32장 전 값이었다 (노드 수신 버퍼가 32개). 이런 메시지는 그 자리의 프레임이 온 것이므로 유실로도,
    재시작으로도 세지 않는다 — 예전엔 '되돌아감 1번 + 32장 유실' 로 세어 901/901 개 온 스트림이 FAIL 이었다.
    """
    lost = dup = restart = 0
    where, stale = [], []
    if wrap:
        for i, (a, b) in enumerate(zip(ids, ids[1:])):
            d = (b - a) % wrap
            if d == 1:
                continue
            if d == 0:
                dup += 1
            elif d > MAX_SEQ_JUMP or d > wrap // 2:
                restart += 1
            else:
                lost += d - 1
                where.append((i, i + 1, d - 1, 0))
        return lost, dup, restart, where, stale

    last, last_i = (ids[0] if ids else None), 0
    recent = collections.deque([last], maxlen=STALE_WINDOW) if ids else collections.deque()
    seen = set(recent)
    pending = 0                        # 마지막 정상 번호 뒤에 온 '이전 번호' 메시지 수 — 그만큼 자리를 채웠다
    for i in range(1, len(ids)):
        b = ids[i]
        d = b - last
        if d == 0:
            dup += 1
            continue
        if d < 0 and b in seen:
            stale.append((i, last + 1 - b))
            pending += 1
            continue
        if d < 0 or d > MAX_SEQ_JUMP:
            restart += 1
        elif d > 1 + pending:
            lost += d - 1 - pending
            where.append((last_i, i, d - 1 - pending, pending))
        pending = 0
        last, last_i = b, i
        if len(recent) == recent.maxlen:
            seen.discard(recent[0])
        recent.append(b)
        seen.add(b)
    return lost, dup, restart, where, stale


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
    lost, dup, restart, where, stale = _seq_loss(sensor["ids"], sensor.get("wrap"))
    times = sensor["times"]
    r["loss_basis"] = sensor["basis"]
    r["lost_mid"] = lost
    if stale:
        # 그 메시지의 stamp 가 앞뒤 평균에서 얼마나 벗어났나 — header.stamp 도 잘못 찍혔는지
        off = [abs(times[i] - (times[i - 1] + times[i + 1]) / 2) / 1e6
               for i, _ in stale if 0 < i < len(times) - 1]
        back = collections.Counter(n for _, n in stale).most_common(1)[0][0]
        r["stale_ids"] = len(stale)
        r["level"] = _worse(r["level"], WARN)
        r["notes"].append(
            f"{sensor['basis']}가 {len(stale)}번 이전 프레임 값으로 실림 (대개 {back}장 전) — 메시지는 그 자리의 새 "
            "프레임이라 유실로 세지 않음. 카메라 메타데이터(frame_id · 카메라 시각)가 이전 프레임 것이라는 뜻"
            + (f", header.stamp 도 앞뒤 평균에서 최대 {max(off):.0f} ms 벗어남" if off and max(off) >= 1 else ""))
    gap_times = sensor.get("gap_times") or times
    for a, b, n, filled in where[:20]:
        r["gaps"].append({"t": round((times[a] - bag_t0) / NS, 3),
                          "dt_ms": round((gap_times[b] - gap_times[a]) / 1e6, 1), "est_lost": n,
                          "filled": filled})
    if sensor.get("gap_basis"):
        r["gap_basis"] = sensor["gap_basis"]
    _missed_captures(r, sensor, stale, bag_t0)
    if dup:
        r["seq_dup"] = dup
        r["level"] = _worse(r["level"], WARN)
        r["notes"].append(f"같은 {sensor['basis']}가 {dup}번 (중복)")
    if restart:
        r["seq_restart"] = restart
        r["level"] = _worse(r["level"], WARN)
        r["notes"].append(f"{sensor['basis']}가 {restart}번 크게 뛰거나 되돌아감 (센서/노드 재시작?) — 유실로 세지 않음")


def _missed_captures(r, sensor, stale, bag_t0):
    """frame_id 는 연속인데 카메라 시각이 두 주기 이상 빈 자리 — 카메라가 그 순간 안 찍은 것 (트리거를 놓침).

    frame_id 는 카메라가 '찍은' 장수라, 트리거가 안 왔거나 놓쳐서 안 찍은 자리는 번호가 건너뛰지 않는다.
    번호만 보면 유실 0 이 나온다. 2026-09-22 PTP 액션 클립: 14대가 각자 한 번씩(서로 다른 순간) 한 장을
    안 찍었는데 진단은 전부 OK 였다. 그 순간 카메라 시각 간격이 63 ms (주기 33 ms)였고, 카메라 PTP 시계가
    3.3 ms 씩 튄 자리와 겹쳤다.
    """
    cam, ids = sensor.get("gap_times"), sensor["ids"]
    if not cam or sensor.get("gap_basis") != "카메라 시각" or len(cam) < MIN_MSGS_FOR_GAPS:
        return
    skip = {i for i, _ in stale} | {i - 1 for i, _ in stale}      # 이전 프레임 값이 실린 자리는 시각도 틀리다
    dts = [b - a for a, b in zip(cam, cam[1:])]
    med = statistics.median([d for d in dts if d > 0] or [0])
    if med <= 0:
        return
    missed = [(i, d, int(round(d / med)) - 1) for i, d in enumerate(dts)
              if i not in skip and ids[i + 1] - ids[i] == 1 and d > med * GAP_FACTOR and round(d / med) >= 2]
    if not missed:
        return
    total = sum(n for _, _, n in missed)
    r["missed_captures"] = total
    r["lost_mid"] += total
    times = sensor["times"]
    for i, d, n in missed:
        if len(r["gaps"]) < 20:
            r["gaps"].append({"t": round((times[i] - bag_t0) / NS, 3), "dt_ms": round(d / 1e6, 1),
                              "est_lost": n, "no_capture": True})
    # 주기의 정수배에서 5% 넘게 벗어나면 그 사이 카메라 시계가 튄 것 (A70 자유 실행은 ±1 ms 쯤 흔들린다)
    odd = [d / 1e6 for _, d, n in missed if abs(d - (n + 1) * med) > 0.05 * med]
    r["notes"].append(
        f"카메라가 {len(missed)}번 그 자리 프레임을 안 찍음 ({total}프레임) — frame_id 는 찍은 장수라 안 건너뛰고, "
        f"카메라 시각 간격만 주기({med / 1e6:.1f} ms)의 {max(round(d / med) for _, d, _ in missed)}배쯤 벌어짐. "
        "트리거로 찍는 카메라면 트리거를 놓친 것 (HW 트리거면 펄스, PTP 액션이면 액션 명령 · 카메라 PTP 시계), "
        "자유 실행이면 카메라가 한 장을 건너뛴 것"
        + (f". 간격이 주기의 정수배가 아님({', '.join(f'{x:.1f}' for x in odd[:3])} ms) — 그 순간 카메라 시계가 "
           "튀었다는 뜻 (PTP 보정)" if odd else ""))


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


def _frozen_frames(r, info, bag_t0):
    """같은 화면이 연달아 온 구간 — 메시지는 왔지만 새 데이터가 아니다. 유실로는 안 잡힌다.

    2026-09-22 열화상 A70: 셔터 보정(NUC, NUCMode=Automatic)마다 1.4초 동안 같은 화면이 42장 연달아 오고,
    그 앞뒤로 한 장씩 건너뛰었다. 센서 잡음 때문에 진짜 새 프레임은 한가운데 몇백 바이트도 늘 다르다.
    """
    mids = info.get("_mid")
    if not mids or len(mids) != len(info["rows"]):
        return
    runs, start = [], 0
    for i in range(1, len(mids) + 1):
        if i < len(mids) and mids[i] and mids[i] == mids[i - 1]:
            continue
        if i - start > FROZEN_MIN_REPEATS:
            runs.append((start, i - 1))
        start = i
    if not runs:
        return
    rows = info["rows"]
    frozen = sum(b - a for a, b in runs)                 # 첫 장은 진짜 → 뒤의 반복만 센다
    longest = max((rows[b][0] - rows[a][0]) / NS for a, b in runs)
    r["frozen_frames"] = frozen
    r["frozen_runs"] = len(runs)
    r["frozen_spans"] = [{"t": round((rows[a][0] - bag_t0) / NS, 3), "sec": round((rows[b][0] - rows[a][0]) / NS, 2),
                          "frames": b - a} for a, b in runs[:10]]
    r["level"] = _worse(r["level"], WARN)
    r["notes"].append(
        f"화면이 {len(runs)}번 멈춤 — 같은 화면이 연달아 {frozen}장 (최장 {longest:.1f}초): "
        + ", ".join(f"t0+{x['t']:.1f}s {x['sec']:.1f}초" for x in r["frozen_spans"][:4])
        + ". 메시지는 왔지만 새 데이터가 아님. 열화상이면 셔터 보정(NUC) — NUCMode=Automatic 이 주기적으로 한다")


def _explain_nuc_skips(r, spans):
    """열화상 셔터 보정(NUC) 중에 건너뛴 프레임 — 카메라가 보정하는 동안 안 찍은 것이라 녹화 · 전송 유실이 아니다.
    멈춤 구간(영상 토픽에서 찾은 frozen_spans) 앞뒤 0.5초 안의 '안 찍음' 자리를 NUC 로 보고, 다른 유실이 없으면
    FAIL 대신 WARN 으로 이유를 붙인다. 2026-09-22: A70 NUC 마다 보정 시작 · 끝에서 한 장씩 건너뛰어, 멀쩡한 열화상이
    클립마다 '트리거를 놓침 · 스트림 중간 유실' FAIL 로 나왔다."""
    skips = [g for g in r["gaps"] if g.get("no_capture")]
    nuc = [g for g in skips if any(sp["t"] - 0.5 <= g["t"] <= sp["t"] + sp["sec"] + 0.5 for sp in spans)]
    if not nuc:
        return
    for g in nuc:
        g["nuc"] = True
    n = sum(g["est_lost"] for g in nuc)
    r["nuc_skipped"] = n
    only_nuc = r["lost_mid"] == n and len(nuc) == len(skips)
    notes = []
    for note in r["notes"]:
        if note.startswith("스트림 중간 유실") and only_nuc:
            notes.append(f"셔터 보정(NUC) 중 {n}프레임 건너뜀 — 카메라가 보정하는 동안 안 찍은 것 (녹화 · 전송 유실 아님)")
        elif note.startswith("카메라가 ") and "그 자리 프레임을 안 찍음" in note and only_nuc:
            continue
        else:
            notes.append(note)
    if not only_nuc:
        notes.append(f"그중 {n}프레임은 셔터 보정(NUC) 중에 건너뛴 것")
    r["notes"] = notes
    if only_nuc and r["level"] == FAIL and not (r.get("future_stamps") or r.get("zero_size") or r.get("deser_failed")):
        r["level"] = WARN


def _judge_latency(r, lat, scan):
    """지연 판정. 백로그 = 클립 동안 지연이 계속 늘어나는 것. 일정하게 늦은 건 처리 지연이다."""
    p95 = r["lat_p95_ms"]
    third = len(lat) // 3
    early = late = None
    if third >= 3:
        early, late = statistics.median(lat[:third]), statistics.median(lat[-third:])
    # 라이다 스캔 토픽은 stamp 가 한 바퀴의 시작이라 한 바퀴(1/rate)는 원래 늦다
    inherent = r.get("dt_median_ms", 0.0) if scan else 0.0
    med = r["lat_med_ms"]
    if early is not None and late - early > LAT_GROWTH_MS:
        r["level"] = _worse(r["level"], WARN)
        r["lat_growth"] = [round(early), round(late)]
        r["notes"].append(f"수신 지연이 클립 동안 계속 늘어남 ({early:.0f} → {late:.0f} ms) — 전송 백로그")
    elif med < -CLOCK_AHEAD_MS:
        # 받기 전에 찍힐 수는 없다 — 늘 미래면 센서 시계가 PC 보다 앞서 있다. 2026-09-22 23:25 라이다: 기준 시계가
        # 없던 동안 떠내려간 시계를 PTP 가 초당 0.5 ms 씩 당기는 중이라 stamp 가 0.2초 미래였다 (FUTURE_STAMP_MS 에 안 걸림).
        r["clock_ahead_ms"] = round(-med)
        r["level"] = _worse(r["level"], WARN)
        r["notes"].append(f"기록 시각이 받은 시각보다 늘 {-med:.0f} ms 미래 — 센서 시계가 PC 보다 앞서 있음 "
                          "(PTP 가 아직 안 맞았거나 시계 기준이 다름)")
    elif med - inherent > STAMP_OFFSET_MS:
        # 1초 넘게 늘 같은 만큼 늦다 = 처리가 밀린 게 아니라 기록 시각의 기준이 다르다. 2026-09-22 라이다:
        # grandmaster(GUI ptp4l)는 UTC 인데 ptp_utc_tai_offset=-37 이라 stamp 가 전부 37초 과거로 찍혔다.
        off = (med - inherent) / 1000
        tai = abs(off - 37) < 1.5
        r["stamp_offset_s"] = round(off, 1)
        r["level"] = FAIL if tai else _worse(r["level"], WARN)
        r["notes"].append(
            f"기록 시각이 받은 시각보다 늘 {off:.1f}초 과거 — "
            + ("UTC 와 TAI 기준이 섞임 (37초). PTP grandmaster 가 UTC(GUI ptp4l)면 라이다 ptp_utc_tai_offset=0"
               if tai else "처리가 밀린 게 아니라 센서 시각의 기준이 PC 와 다름 (센서 시계 · PTP 설정 확인)"))
    elif p95 - inherent > LAT_WARN_MS:
        # 일정하게 늦은 건 기록된 시각(header.stamp)에 영향이 없다 — 판정은 올리지 않고 참고로만 적는다.
        # (2026-09-22 지적: 열화상 지연 275 ms 로 WARN 이 8개씩 떠서 진짜 문제가 안 보였다)
        r["lat_const_ms"] = round(p95 - inherent)
        extra = f", 스캔 한 바퀴 {inherent:.0f} ms 를 빼고도 {p95 - inherent:.0f} ms" if scan else ""
        r["notes"].append(f"참고: 수신 지연 p95 {p95:.0f} ms{extra} — 내내 일정하게 늦음 "
                          "(처리 지연, 쌓이지는 않음 · 기록된 시각에는 영향 없음)")
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
    r["first_t"] = round((rows[0][0] - bag_t0) / NS, 3)
    r["last_t"] = round((rows[-1][0] - bag_t0) / NS, 3)
    sizes = sorted(sz for _, _, sz in rows)
    r["size_min"], r["size_med"], r["size_max"] = \
        sizes[0], sizes[len(sizes) // 2], sizes[-1]

    # stamp 기준 선택: header가 95% 이상 유효하고 bag 시각과 5분 내로 일치할 때
    use_header = use_header_stamps(rows)
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

        # 받은 시각보다 미래로 찍힌 stamp — 있을 수 없는 값이라 시계가 튄 자리다. 2026-09-22 Orin GNSS grandmaster +
        # boundary clock: 카메라 PTP 시계가 N × 0.5 s 씩 앞으로 튀었다 돌아와, 튄 뒤 최대 2초(시계 매핑 재측정 주기)
        # 동안 프레임이 7–8 s 미래로 찍혔다. 레이트도 엉뚱하게 나온다 (107 Hz).
        timed = [(b, (h - b) / 1e6) for b, h, _ in rows if h is not None]
        ahead = [(b, x) for b, x in timed if x > FUTURE_STAMP_MS]
        if ahead:
            spans, cur = [], [ahead[0]]
            for item in ahead[1:]:
                if item[0] - cur[-1][0] > 2 * NS:
                    spans.append(cur)
                    cur = [item]
                else:
                    cur.append(item)
            spans.append(cur)
            r["future_stamps"] = len(ahead)
            r["future_spans"] = [{"t": round((sp[0][0] - bag_t0) / NS, 3), "sec": round((sp[-1][0] - sp[0][0]) / NS, 2),
                                  "n": len(sp), "max_s": round(max(x for _, x in sp) / 1000, 1)} for sp in spans[:10]]
            r["level"] = FAIL
            r["notes"].append(
                f"header.stamp 가 받은 시각보다 미래인 메시지 {len(ahead)}개 (최대 +{max(x for _, x in ahead) / 1000:.1f} s): "
                + ", ".join(f"t0+{(sp[0][0] - bag_t0) / NS:.1f}s {len(sp)}개 +{max(x for _, x in sp) / 1000:.1f}s"
                            for sp in spans[:4])
                + " — 센서 시계가 튄 자리 (카메라면 PTP 시계). 그 구간은 시각을 믿을 수 없음")

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
            # 레이트 = (받은 개수 + 중간 유실) / 전체 시간. 간격 중앙값으로 내면 호스트 시각으로 찍는 토픽은
            # 도착 지터에 끌려 틀린다 — A70 은 카메라 시각 30.0 Hz 인데 header 간격 중앙값 29.9 ms 라 33.4 Hz 로 보였다.
            if stamps[-1] > stamps[0]:
                r["rate_hz"] = round((len(stamps) - 1 + r["lost_mid"]) * NS / (stamps[-1] - stamps[0]), 1)
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
            # 비어 있는 시간은 도착 시각으로 잰다 (header 는 수신 지연만큼 앞서 있어 뒤쪽이 길게 나온다)
            empty_head, empty_tail = (rows[0][0] - bag_t0) / NS, (bag_t1 - rows[-1][0]) / NS
            limit = max(TRUNC_WARN_S, 3 * med / NS)
            if empty_head > limit:
                r["absent_head_s"] = round(empty_head, 1)
                r["level"] = _worse(r["level"], WARN)
                r["notes"].append(f"녹화 시작 후 {empty_head:.1f}초 동안 이 토픽이 없음 — 센서가 늦게 들어옴")
            if empty_tail > limit:
                r["absent_tail_s"] = round(empty_tail, 1)
                r["level"] = _worse(r["level"], WARN)
                r["notes"].append(f"녹화가 끝나기 {empty_tail:.1f}초 전부터 이 토픽이 없음 — 센서가 먼저 끊김")
            if (r["head_trunc"] or r["tail_trunc"]) and not (r.get("absent_head_s") or r.get("absent_tail_s")):
                # 몇 프레임은 수신 지연만큼 창이 어긋난 정상 범위
                r["notes"].append(
                    f"참고: 앞 {r['head_trunc']} / 뒤 {r['tail_trunc']}프레임 잘림 "
                    "(녹화 시작 · 끝 경계, 중간 유실 아님)")
            r["expected"] = r["count"] + r["lost_mid"]
    else:
        r["notes"].append("저빈도 토픽 — 간격 분석 생략")

    if lat:
        _judge_latency(r, lat, scan)
    _frozen_frames(r, info, bag_t0)

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

def analyze(bag_dir, progress=None, hold=None):
    """hold: 읽는 동안 몇백 메시지마다 부른다 — 부른 쪽이 거기서 기다리게 할 수 있다 (녹화 중 일시정지)."""
    bag_dir = Path(bag_dir)
    if not bag_dir.is_dir():
        raise FileNotFoundError(f"클립 디렉터리가 아님: {bag_dir}")
    db3 = _db3_files(bag_dir)
    if db3:
        data = _read_sqlite(db3, progress, hold)
        fetch = _fetch_samples_sqlite
        storage = "sqlite3"
    else:
        data = _read_rosbag2py(bag_dir, progress, hold)
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

    # 멈춤(셔터 보정)은 영상 토픽에서만 보인다 — 같은 카메라의 camera_info · metadata 에도 그 구간을 적용한다
    frozen_by_ns = {}
    for name, t in topics.items():
        if t.get("frozen_spans"):
            frozen_by_ns.setdefault("/" + name.split("/")[1], []).extend(t["frozen_spans"])
    for name, t in topics.items():
        spans = frozen_by_ns.get("/" + name.split("/")[1]) if name.count("/") >= 2 else None
        if spans:
            _explain_nuc_skips(t, spans)

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
    rep = {
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
    try:
        rep["lidar_phase"] = _lidar_phase(data, topics)
    except Exception as e:                       # 보조 측정 — 실패해도 진단 전체를 막지 않되 이유는 남긴다
        rep["lidar_phase"] = {"error": f"촬영 때 라이다 각도 측정 실패: {type(e).__name__}: {e}", "cams": {}}
    rep["summary"] = summarize(rep)
    return rep


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


# ---------- 한눈에 보는 요약 — 진단 창 · diagnostics.txt 맨 위 ----------
# 토픽 84개를 줄줄이 늘어놓으면 뭐가 문제인지 안 보인다 (2026-09-22 지적: "진단기가 직관적이지 못하다").
# 센서(토픽의 첫 이름 = 네임스페이스) 단위로 묶고, 같은 사건이 camera_info · metadata · 영상에 세 번 잡힌 건 한 번만,
# 녹화된 데이터에 영향이 있는 것만 문제로 올리고, 쉬운 말 + 무엇을 하면 되는지로 적는다. 토픽별 원래 결과는 그대로 남긴다.

WORDS = {OK: "정상", WARN: "확인", FAIL: "문제"}
AUX_TOPICS = {"diagnostics", "tf", "tf_static", "rosout", "parameter_events", "clock"}
_KINDS = (("camera", "camera", "RGB 카메라"), ("thermal", "thermal", "열화상"), ("ouster", "lidar", "라이다"),
          ("lidar", "lidar", "라이다"), ("gps", "gnss", "GNSS"), ("gnss", "gnss", "GNSS"), ("imu", "imu", "IMU"))
_KIND_ORDER = ["camera", "thermal", "lidar", "gnss", "imu", "other", "aux"]
_LIDAR_PRODUCTS = ("range_image", "signal_image", "reflec_image", "nearir_image")


def _sensor_of(topic):
    """토픽 → 센서 키. /camera_front1/image_raw → camera_front1, /diagnostics · /tf_static → _aux."""
    parts = topic.strip("/").split("/")
    if parts[0] in AUX_TOPICS:
        return "_aux"
    return parts[0]


def _sensor_kind(key):
    if key == "_aux":
        return "aux", "기타 (diagnostics · tf)"
    for prefix, kind, label in _KINDS:
        if key.startswith(prefix):
            rest = key[len(prefix):].strip("_")
            return kind, f"{label} {rest}" if rest else label
    return "other", key


def _main_rank(name, typ):
    """센서를 대표하는 토픽 — 개수 · 주기를 이걸로 보여 준다."""
    if typ in ("sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/NavSatFix"):
        return 0
    if typ in IMAGE_TYPES and not any(k in name for k in _LIDAR_PRODUCTS):
        return 1 if any(k in name for k in ("image_rgb", "image_raw", "image_color")) else 2
    if typ == "sensor_msgs/msg/Imu":
        return 3
    return 4


def _qty(n, typ):
    if typ == PACKET_TYPE:
        return f"패킷 {n}개"
    if typ in IMAGE_TYPES or typ in SCAN_TYPES:
        return f"{n}프레임"
    return f"{n}개"


def _summarize_sensor(key, names, topics):
    kind, label = _sensor_kind(key)
    main = min(names, key=lambda n: (_main_rank(n, topics[n]["type"]), -topics[n]["count"], n))
    mt = topics[main]
    issues, info, events = [], [], []
    handled = [OK]        # 알아본 사건의 판정 (참고로 내린 것 포함) — 아래 '못 옮긴 이유' 가 다시 올리지 않게

    def add(level, text, hint="", kind=None, seconds=None):
        issues.append({"level": level, "text": text, "hint": hint, "kind": kind, "seconds": seconds})
        handled[0] = _worse(handled[0], level)

    def light(level, text):
        """판정은 안 올리고 참고로만 (아주 적은 유실 · 짧은 셔터 보정)."""
        info.append(text)
        handled[0] = _worse(handled[0], level)

    def loss_level(count, total):
        if count <= LOSS_INFO_MAX and count <= LOSS_INFO_RATIO * max(1, total):
            return None
        return FAIL if (count >= LOSS_FAIL_MIN or count >= LOSS_FAIL_RATIO * max(1, total)) else WARN

    def short(n):
        return n if key == "_aux" else n[len(key) + 2:] or n

    def where(n):
        return "" if n == main else f" ({short(n)})"

    def worst(field):
        """이 센서 토픽 중 field 가 가장 큰 것 — 같은 사건이 camera_info · metadata · 영상에 다 잡힌다."""
        best = max(names, key=lambda n: topics[n].get(field) or 0)
        return best, topics[best].get(field) or 0

    def event(t, sec, kind_, text):
        for e in events:
            if e["kind"] == kind_ and abs(e["t"] - t) < 0.1:
                return
        events.append({"t": t, "sec": max(0.0, sec), "kind": kind_, "text": text})

    empty = [n for n in names if topics[n]["count"] == 0]
    if empty:
        add(FAIL, f"메시지가 하나도 안 옴: {', '.join(short(n) for n in empty)}", "센서 · 드라이버가 이 토픽을 안 냈음 — 센서 상태 확인")

    broken = sum(topics[n].get("zero_size", 0) + topics[n].get("deser_failed", 0) for n in names)
    if broken:
        add(FAIL, f"깨진 메시지 {broken}개 (0바이트 · 읽기 실패)", "녹화 · 디스크 오류 — 레코더 로그 확인")

    n, fut = worst("future_stamps")
    if fut:
        spans = topics[n].get("future_spans") or []
        top = max((sp["max_s"] for sp in spans), default=0)
        add(FAIL, f"기록 시각이 {len(spans)}군데서 튐 — 받은 시각보다 최대 {top:.1f}초 미래로 찍힌 메시지 {fut}개",
            "센서 시계(카메라면 PTP)가 튄 것. 그 구간의 시각은 믿을 수 없음 — 터미널에서 ptp status 로 확인")
        for sp in spans:
            event(sp["t"], sp["sec"], "clock", f"시각 튐 +{sp['max_s']:.1f}초 ({sp['n']}개)")

    n, ahead = worst("clock_ahead_ms")
    if ahead:
        add(WARN, f"기록 시각이 늘 {ahead} ms 미래로 찍힘 — 센서 시계가 PC 보다 앞섬{where(n)}",
            "PTP 가 아직 안 맞은 채 녹화했을 수 있음 (라이다는 기준 시계를 다시 잡으면 초당 0.5 ms 씩 맞춤) — "
            "센서 화면의 동기 상태 줄이 ✓ 인지 보고 녹화하세요")
    n, off = worst("stamp_offset_s")
    if off:
        tai = abs(off - 37) < 1.5
        add(FAIL if tai else WARN, f"기록 시각이 늘 {off:.1f}초 과거로 찍힘{where(n)}",
            "UTC 와 TAI 기준이 섞임 — 라이다면 ptp_utc_tai_offset 을 grandmaster 에 맞추세요 (GUI ptp4l = UTC → 0)"
            if tai else "센서 시각의 기준이 PC 와 다름 — 센서 시계 · PTP 설정 확인")

    # 빠진 것: 녹화 · 전송 중에 잃음 vs 카메라가 그 순간 안 찍음 (셔터 보정 중 건너뛴 건 아래에서 따로)
    def split(t):
        missed = t.get("missed_captures", 0)
        return t["lost_mid"] - missed, missed - t.get("nuc_skipped", 0)
    n = max(names, key=lambda x: split(topics[x])[0])
    lost = split(topics[n])[0]
    if lost > 0:
        t = topics[n]
        places = sum(1 for g in t["gaps"] if not g.get("no_capture"))
        total = t["count"] + t["lost_mid"]
        pct = 100 * lost / max(1, total)
        detail = [f"{places}{'+' if len(t['gaps']) >= 20 else ''}곳", f"{pct:.1f}%" if pct >= 0.1 else "0.1% 미만"]
        if n != main:
            detail.append(short(n))
        text = f"{_qty(lost, t['type'])} 빠짐 — 녹화 · 전송 중에 잃음 ({' · '.join(detail)})"
        level = loss_level(lost, total)
        if level is None:
            light(t["level"], text + " — 아주 적어서 참고만")
        else:
            add(level, text, "네트워크 · 디스크 부하나 케이블 문제 — 빠진 시점은 타임라인의 빨간 표시")
    n = max(names, key=lambda x: split(topics[x])[1])
    nocap = split(topics[n])[1]
    if nocap > 0:
        t = topics[n]
        places = sum(1 for g in t["gaps"] if g.get("no_capture") and not g.get("nuc"))
        text = f"카메라가 {places}번 안 찍음 ({nocap}프레임)"
        level = loss_level(nocap, t["count"] + t["lost_mid"])
        if level is None:
            light(t["level"], text + " — 아주 적어서 참고만")
        else:
            add(level, text,
                "저장 문제가 아니라 카메라가 그 순간 찍지 않은 것 — 트리거를 놓쳤거나 카메라 시계(PTP)가 튐")
    for x in names:
        for g in topics[x]["gaps"]:
            k = "nuc" if g.get("nuc") else "nocap" if g.get("no_capture") else "lost"
            event(g["t"], g["dt_ms"] / 1000, k,
                  {"nuc": f"셔터 보정 중 {g['est_lost']}프레임 건너뜀",
                   "nocap": f"카메라가 {g['est_lost']}프레임 안 찍음",
                   "lost": f"{g['est_lost']}개 빠짐 (간격 {g['dt_ms']:.0f} ms)"}[k])

    n, frozen = worst("frozen_frames")
    nuc = max(topics[x].get("nuc_skipped", 0) for x in names)
    if frozen or nuc:
        spans = topics[n].get("frozen_spans") or [] if frozen else []
        runs = topics[n].get("frozen_runs", len(spans)) if frozen else 0
        longest = max((sp["sec"] for sp in spans), default=0)
        if kind == "thermal" or nuc:
            text = f"셔터 보정으로 화면이 {runs}번 멈춤 (최장 {longest:.1f}초)" if runs else "셔터 보정 중 프레임 건너뜀"
            if nuc:
                text += f" · 앞뒤로 {nuc}프레임 건너뜀"
            if runs <= 1 and longest <= FROZEN_INFO_S:
                # 열화상이 몇 분마다 스스로 하는 정상 동작 — 한 번, 짧게면 참고만 (분석에서 그 구간만 빼면 된다)
                light(topics[n]["level"], text + " — 열화상의 정상 동작이라 참고만 (그 구간은 분석에서 빼세요)")
            else:
                add(WARN, text, "열화상 카메라가 몇 분마다 스스로 하는 자동 보정(NUC) — 고장 · 유실은 아님. "
                                "그동안은 같은 열화상이 반복되니 분석에서 그 구간은 빼세요")
        else:
            add(WARN, f"화면이 {runs}번 멈춤 — 같은 영상이 연달아 {frozen}장 (최장 {longest:.1f}초)",
                "메시지는 왔지만 새 영상이 아님 — 카메라 · 드라이버 확인")
        for sp in spans:
            event(sp["t"], sp["sec"], "frozen", f"화면 {sp['sec']:.1f}초 멈춤 (같은 영상 {sp['frames']}장)")

    n = next((x for x in names if topics[x].get("lat_growth")), None)
    if n:
        a, b = topics[n]["lat_growth"]
        add(WARN, f"전송이 점점 밀림 — 지연 {a} → {b} ms{where(n)}",
            "이대로 길게 찍으면 유실로 번질 수 있음 — 네트워크 · CPU 부하 확인")

    n, dup = worst("dup_stamps")
    m, back = worst("nonmonotonic")
    if dup or back:
        parts = ([f"겹친 시각 {dup}개"] if dup else []) + ([f"거꾸로 간 시각 {back}번"] if back else [])
        add(WARN, f"기록 시각 이상 — {', '.join(parts)}", "센서 시계 · 드라이버 확인")

    n, stale = worst("stale_ids")
    if stale:
        add(WARN, f"카메라 정보(frame_id · 카메라 시각)가 이전 프레임 값으로 {stale}번 실림 — 영상은 새 프레임",
            "그 프레임들의 카메라 시각은 믿지 마세요 (드라이버 수신 버퍼)")
    n, dup_id = worst("seq_dup")
    if dup_id:
        add(WARN, f"같은 번호의 메시지가 {dup_id}번 겹침{where(n)}", "드라이버가 같은 데이터를 두 번 냈음")
    n, restart = worst("seq_restart")
    if restart:
        add(WARN, f"센서 번호가 {restart}번 크게 뛰거나 되돌아감{where(n)} — 센서 · 드라이버가 재시작했을 수 있음",
            "그 자리 앞뒤로 데이터가 끊겼는지 확인")

    for x in names:
        cad = topics[x].get("cadence")
        if cad:
            add(WARN, f"장비가 {cad['rate_hz']:g} Hz 로 보냄 — {cad['every']}칸마다 한 칸씩 규칙적으로 빔{where(x)}",
                "녹화 유실이 아니라 장비의 출력 주기 — 필요하면 장비 설정 확인")
            break

    payload = [pi for x in names for pi in topics[x].get("payload_issues", [])]
    if payload and not broken:
        add(WARN, f"내용이 이상한 메시지 {len(payload)}건 (샘플 검사)", payload[0])

    n, head = worst("absent_head_s")
    if head:
        add(WARN, f"녹화 시작 후 {head:.1f}초 동안 데이터 없음 — 늦게 들어옴",
            "센서가 녹화 시작보다 늦게 나왔음 — 센서를 켜고 데이터가 들어온 뒤 녹화하세요",
            kind="absent_head", seconds=head)
        event(0.0, head, "absent", f"처음 {head:.1f}초 데이터 없음")
    n, tail = worst("absent_tail_s")
    if tail:
        add(WARN, f"녹화 끝나기 {tail:.1f}초 전부터 데이터 없음 — 먼저 끊김",
            "센서 · 드라이버가 녹화 중에 멈췄음 — 센서 로그 확인")
        event(topics[n].get("last_t", 0.0), tail, "absent", f"마지막 {tail:.1f}초 데이터 없음")

    for x in names:
        q = topics[x].get("gnss")
        if not q:
            continue
        fix = q.get("fix_ratio")
        if fix is not None and fix < 0.01:
            add(q["level"] if q["level"] != OK else WARN, "위치를 못 잡음 (유효 fix 0%)",
                "실내이거나 하늘이 가려짐 — 야외에서 다시 확인")
        elif q["level"] != OK:
            for note in q.get("notes", []):
                add(q["level"], note)
        bits = []
        if fix:
            bits.append(f"유효 fix {fix * 100:.0f}%")
        if "hacc_p95_m" in q:
            bits.append(f"수평 정확도 p95 {q['hacc_p95_m']} m")
        if "track_m" in q:
            bits.append(f"이동 {q['track_m']} m")
        if bits:
            info.append(" · ".join(bits))

    # 위에서 못 옮긴 이유로 판정이 올라간 토픽이 있으면 원래 문장 그대로라도 올린다 — 요약에서 문제가 사라지면 안 된다
    top = max([_RANK[handled[0]]] + [_RANK[i["level"]] for i in issues])
    for x in names:
        t = topics[x]
        if _RANK[t["level"]] > top:
            note = next((z for z in t["notes"] if not z.startswith("참고")), "이유 미상 — 원본 보고서 확인")
            add(t["level"], f"{note}{where(x)}")

    lat = mt.get("lat_const_ms") or max(topics[x].get("lat_const_ms") or 0 for x in names)
    if lat:
        info.append(f"받는 데 약 {lat} ms 걸림 — 일정하게 늦은 것이라 기록된 시각에는 영향 없음")

    issues.sort(key=lambda i: -_RANK[i["level"]])
    # 센서 판정은 '확인할 것' 기준 — 참고로 내린 것(아주 적은 유실 등)은 올리지 않는다
    level = OK
    for i in issues:
        level = _worse(level, i["level"])
    if issues:
        headline = issues[0]["text"] + (f"  (외 {len(issues) - 1}건)" if len(issues) > 1 else "")
    elif kind == "aux":
        headline = "정상"
    else:
        bits = ["정상"]
        if mt.get("rate_hz"):
            bits.append(f"{mt['rate_hz']:g} Hz")
        bits.append(_qty(mt["count"], mt["type"]))
        if "expected" in mt:
            bits.append("빠짐 없음")
        headline = " · ".join(bits)
    events.sort(key=lambda e: e["t"])
    return {"key": key, "label": label, "kind": kind, "level": level, "headline": headline,
            "main": main, "topics": names, "count": mt["count"], "expected": mt.get("expected", mt["count"]),
            "rate_hz": mt.get("rate_hz"), "first_t": mt.get("first_t", 0.0), "last_t": mt.get("last_t", 0.0),
            "issues": issues, "info": info, "events": events[:200]}


def _merge_same(problems, sensors):
    """여러 센서의 같은 문제는 한 줄로 — 'RGB 카메라 14대 전부 — 카메라가 5번 안 찍음'. 카메라 13대가 같은 줄을
    13번 늘어놓으면 공통 원인(트리거 · PTP)이라는 게 오히려 안 보인다."""
    by_label = {s["label"]: s for s in sensors}
    groups = {}
    for p in problems:
        groups.setdefault((p["level"], p["text"], p.get("hint", "")), []).append(p)
    out = []
    for (level, text, hint), same in groups.items():
        if len(same) == 1:
            out.append(same[0])
            continue
        labels = [p["sensor"] for p in same]
        kinds = {by_label[x]["kind"] for x in labels if x in by_label}
        kind_label = _sensor_kind(same[0]["key"])[1].rsplit(" ", 1)[0] if same[0]["key"] else ""
        if len(kinds) == 1 and kind_label and all(x.startswith(kind_label + " ") for x in labels):
            total = sum(1 for s in sensors if s["kind"] in kinds)
            rests = [x[len(kind_label) + 1:] for x in labels]
            name = (f"{kind_label} {len(labels)}대 전부" if len(labels) == total
                    else f"{kind_label} {len(labels)}대 ({', '.join(rests)})")
        else:
            name = ", ".join(labels)
        out.append({"sensor": name, "key": same[0]["key"], "keys": [p["key"] for p in same],
                    "level": level, "text": text, "hint": hint})
    return out


def _add_lidar_phase(sensors, phase):
    """라이다 줄의 '참고' 에 카메라가 찍을 때의 라이다 각도를 붙인다 (위상 고정이 맞았는지)."""
    if not phase:
        return
    lidar = next((s for s in sensors if phase.get("lidar") in s["topics"]), None)
    if lidar is None:
        return
    if phase.get("error"):
        lidar["info"].append(phase["error"])
        return
    locked, loose = {}, {}
    for ns, c in sorted(phase["cams"].items()):
        kind_label = _sensor_kind(ns)[1].rsplit(" ", 1)[0]
        if "note" in c:
            lidar["info"].append(f"{_sensor_kind(ns)[1]}: {c['note']}")
        elif c["spread_deg"] <= LIDAR_PHASE_LOCKED_DEG:
            locked.setdefault((kind_label, c["grid_deg"]), []).append(c)
        else:
            loose.setdefault(kind_label, []).append(c)
    for (label, grid), cams in locked.items():
        off = statistics.median(c["offset_deg"] for c in cams)
        angles = " · ".join(f"{(off + k * grid) % 360:.0f}°" for k in range(int(round(360 / grid))))
        base = " / ".join(f"{k * grid:g}" for k in range(int(round(360 / grid))))
        lidar["info"].append(
            f"{label} {len(cams)}대가 찍을 때 라이다 각도: {angles} — {base}° 에서 {off:+.1f}° "
            f"(흔들림 ±{max(c['spread_deg'] for c in cams):.1f}°, Ouster 좌표계 0° = 커넥터 쪽)"
            + (f". {base}° 에 맞추려면: 라이다 위상 고정이 꺼져 있었으면 켜고(정각 때 라이다 각도 0°), 켜져 있었으면 "
               f"'정각 때 라이다 각도' 를 {-off:+.1f}° 고치세요" if abs(off) > 5 else ""))
    for label, cams in loose.items():
        lidar["info"].append(f"{label} {len(cams)}대: 찍을 때 라이다 각도가 매번 다름 (±{min(c['spread_deg'] for c in cams):.0f}° 이상) "
                             "— 카메라가 자유 실행이거나 라이다 위상 고정이 안 잠김")


def summarize(rep):
    """보고서 → 센서별 요약 + 한 줄 판정. GUI 진단 창과 diagnostics.txt 머리가 이걸 쓴다."""
    topics = rep["topics"]
    groups = {}
    for name in topics:
        groups.setdefault(_sensor_of(name), []).append(name)
    sensors = [_summarize_sensor(key, sorted(names), topics) for key, names in groups.items()]
    _add_lidar_phase(sensors, rep.get("lidar_phase"))
    sensors.sort(key=lambda s: (-_RANK[s["level"]], _KIND_ORDER.index(s["kind"]), s["label"]))
    problems = [{"sensor": s["label"], "key": s["key"], **i} for s in sensors for i in s["issues"]]
    # GNSS 분석 자체가 실패하면 토픽에 안 붙는다 — 따로 올린다
    for tname, q in (rep.get("gnss") or {}).items():
        if tname not in topics and q.get("level", OK) != OK:
            problems.append({"sensor": "GNSS", "key": "", "level": q["level"],
                             "text": "; ".join(q.get("notes") or ["GNSS 분석 실패"]), "hint": ""})
    problems.sort(key=lambda p: -_RANK[p["level"]])
    problems = _merge_same(problems, sensors)
    counted = [s for s in sensors if s["kind"] != "aux" or s["level"] != OK]
    level = rep["level"]
    return {
        "level": level,
        "title": {OK: "이상 없음 — 그대로 쓰면 됩니다",
                  WARN: "쓸 수 있지만 확인할 것이 있습니다",
                  FAIL: "데이터에 문제가 있습니다"}[level],
        "sub": {OK: "모든 센서가 빠짐없이, 제 시각으로 기록됐습니다",
                WARN: "녹화 자체는 됐습니다 — 노란 항목이 쓰려는 데이터에 괜찮은지 확인하세요",
                FAIL: "빠진 데이터나 시각이 틀린 데이터가 있습니다 — 빨간 항목을 보세요"}[level],
        "counts": {lvl: sum(1 for s in counted if s["level"] == lvl) for lvl in (OK, WARN, FAIL)},
        "n_sensors": len(counted),
        "sensors": sensors,
        "problems": problems,
    }


def _pad(text, width):
    """터미널에서 한글은 두 칸 — 칸을 맞춰 채운다."""
    used = sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text)
    return text + " " * max(0, width - used)


def render_summary(rep):
    sm = rep.get("summary") or summarize(rep)
    c = sm["counts"]
    mark = {OK: "✔", WARN: "⚠", FAIL: "✖"}
    L = [f"녹화 진단: {rep['bag']}",
         f"  {rep['duration']:.1f}초 · 메시지 {rep['total_msgs']}개 · {rep['total_mb'] / 1000:.1f} GB",
         "",
         f"  {mark[sm['level']]} {sm['title']}",
         f"    {sm['sub']}",
         f"    센서 {sm['n_sensors']}개: 정상 {c[OK]} · 확인 {c[WARN]} · 문제 {c[FAIL]}"]
    if sm["problems"]:
        L += ["", "  확인할 것:"]
        for p in sm["problems"]:
            L.append(f"   {mark[p['level']]} [{WORDS[p['level']]}] {p['sensor']} — {p['text']}")
            if p.get("hint"):
                L.append(f"        → {p['hint']}")
    L += ["", "  센서별:"]
    width = max((len(_pad(s["label"], 0)) + sum(1 for ch in s["label"] if unicodedata.east_asian_width(ch) in "WF")
                 for s in sm["sensors"]), default=0)
    for s in sm["sensors"]:
        L.append(f"   {mark[s['level']]} {_pad(s['label'], width)}  {s['headline']}")
        for x in s["info"]:
            L.append(f"     {' ' * width}  참고: {x}")
    return "\n".join(L)


def render_text(rep):
    L = [render_summary(rep), "", "", "======== 토픽별 자세한 결과 (전문가용) ========", ""]
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
            basis = f"{t['gap_basis']} " if t.get("gap_basis") else ""
            L.append(f"           유실 지점 t0+{g['t']:.3f}s: "
                     f"{basis}간격 {g['dt_ms']:.0f} ms · {g['est_lost']}프레임 빠짐"
                     + (f" (사이에 이전 번호로 온 프레임 {g['filled']}장)" if g.get("filled") else "")
                     + (" — 셔터 보정(NUC) 중" if g.get("nuc") else
                        " — 카메라가 안 찍음 (frame_id 연속)" if g.get("no_capture") else ""))
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
    tty = sys.stderr.isatty()
    rep = analyze(args.bag_dir, progress=lambda s: tty and print(f"  .. {s[:70]:<70}", end="\r", file=sys.stderr))
    if tty:
        print(" " * 79, end="\r", file=sys.stderr)
    print(render_text(rep))
    if args.json:
        Path(args.json).write_text(
            json.dumps(rep, ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"\nJSON 저장: {args.json}")
    return 0 if rep["level"] != FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
