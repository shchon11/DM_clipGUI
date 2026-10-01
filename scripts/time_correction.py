#!/usr/bin/env python3
# time_correction.py — 녹화의 센서 시각이 틀린 만큼을 재서 보정 파일(time_correction.json)을 남긴다.
#
# bag 원본은 고치지 않는다. 녹화 안의 기준 시각과 비교해 '얼마나 틀렸나' 만 잰다:
#   - 라이다: /ouster/imu 의 header.stamp − PC 가 받은 시각. 라이다 시계가 PTP 에서 어긋나 있으면 이 값이
#     수십 초가 되고, Ouster 는 초당 0.5 ms 씩만 따라가므로 녹화 안에서 직선으로 변한다 → 직선 맞춤.
#   - 다른 센서(카메라 · 열화상 · IMU · GNSS): 같은 방식으로 재서 정상인지 확인만 한다.
#   - GNSS 진짜 시각(/gps/time_ref) − PC 받은 시각: PC 시계 자체가 맞았는지 확인.
#
# bag 전체를 읽으면 수십 GB 라 오래 걸리므로, 녹화를 따라 고르게 짧은 구간(기본 5초마다 1초)만 읽는다.
#
#   python3 time_correction.py measure <녹화폴더> [...]          # 재서 time_correction.json 쓰기
#   python3 time_correction.py measure --dry-run <녹화폴더>      # 쓰지 않고 결과만
#
# 2026-09-28 하남: Orin chrony 가 15:38 에 GPS → 인터넷 NTP 로 갈아타며 시계를 37 s 옮겼고, PC 는 곧 따라
# 돌아왔지만 라이다는 +37 s 에서 초당 0.5 ms 씩만 돌아와 그 뒤 녹화의 라이다 시각이 35~37 s 미래로 찍혔다.

import argparse
import datetime as dt
import glob
import json
import sqlite3
import statistics as st
import sys
from pathlib import Path

CORRECTION_FILE = "time_correction.json"
LIDAR_TOPIC = "/ouster/imu"                      # 100 Hz, 전송 지연이 작고 일정 — 라이다 시계를 재기 좋다
LIDAR_APPLIES = "/ouster/*"
CHECK_TOPICS = {                                 # 정상인지 확인만 하는 센서 (없는 토픽은 건너뜀)
    "RGB 카메라": "/camera_1/camera_info",
    "열화상": "/thermal0/camera_info",
    "IMU": "/imu/data",
    "GNSS": "/gps/fix",
}
REF_TOPIC = "/gps/time_ref"
OK_S = 0.2                                       # 이보다 크면 '정상 아님'
FIT_OK_MS = 2.0                                  # 직선에서 벗어난 정도(p95)가 이 안이면 보정 신뢰


def _ts(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _fit_line(xs, ys):
    """최소제곱 직선 → 벗어난 값(중앙값에서 5×MAD 밖) 빼고 한 번 더. (a, b, 잔차 목록)."""
    def ls(x, y):
        n = len(x)
        mx, my = sum(x) / n, sum(y) / n
        sxx = sum((xi - mx) ** 2 for xi in x)
        b = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y)) / sxx if sxx else 0.0
        return my - b * mx, b
    a, b = ls(xs, ys)
    res = [y - (a + b * x) for x, y in zip(xs, ys)]
    med = st.median(res)
    mad = st.median(abs(r - med) for r in res) or 1e-6
    keep = [i for i, r in enumerate(res) if abs(r - med) <= 5 * mad]
    a, b = ls([xs[i] for i in keep], [ys[i] for i in keep])
    return a, b, [ys[i] - (a + b * xs[i]) for i in keep]


def _p95(values):
    v = sorted(abs(x) for x in values)
    return v[min(len(v) - 1, int(0.95 * len(v)))] if v else 0.0


def measure(bag_dir, every_s=5.0, window_s=1.0):
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    bag_dir = Path(bag_dir)
    dbs = sorted(glob.glob(str(bag_dir / "*.db3")))
    if not dbs:
        raise FileNotFoundError(f"{bag_dir}: .db3 가 없습니다 (sqlite3 녹화만 지원)")
    samples = {"lidar": [], "ref": []}
    checks = {k: [] for k in CHECK_TOPICS}
    t_start = None
    for db in dbs:
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        topics = {n: (i, t) for n, i, t in con.execute("select name, id, type from topics")}
        if LIDAR_TOPIC not in topics:
            raise LookupError(f"{bag_dir.name}: {LIDAR_TOPIC} 가 없습니다")
        want = {LIDAR_TOPIC: "lidar"}
        if REF_TOPIC in topics:
            want[REF_TOPIC] = "ref"
        for label, name in CHECK_TOPICS.items():
            if name in topics:
                want[name] = label
        by_id = {topics[n][0]: (n, role) for n, role in want.items()}
        cls = {n: get_message(topics[n][1]) for n in want}
        t0, t1 = con.execute("select min(timestamp), max(timestamp) from messages").fetchone()
        if t_start is None:
            t_start = t0 / 1e9
        ids = list(by_id)
        q = (f"select topic_id, timestamp, data from messages where timestamp between ? and ? "
             f"and topic_id in ({','.join('?' * len(ids))})")
        t = t0
        while t < t1:
            for tid, recv, data in con.execute(q, (t, t + int(window_s * 1e9), *ids)):
                name, role = by_id[tid]
                msg = deserialize_message(data, cls[name])
                rt = recv / 1e9
                if role == "lidar":
                    samples["lidar"].append((rt - t_start, _ts(msg.header.stamp) - rt))
                elif role == "ref":
                    samples["ref"].append(_ts(msg.time_ref) - rt)
                else:
                    checks[role].append(_ts(msg.header.stamp) - rt)
            t += int(every_s * 1e9)
        con.close()

    lid = samples["lidar"]
    if len(lid) < 50:
        raise RuntimeError(f"{bag_dir.name}: 라이다 샘플이 너무 적습니다 ({len(lid)}개)")
    a, b, res = _fit_line([x for x, _ in lid], [y for _, y in lid])
    fit_p95_ms = _p95(res) * 1000
    out = {
        "bag": bag_dir.name,
        "measured_at": dt.datetime.now().isoformat(timespec="seconds"),
        "method": f"{LIDAR_TOPIC} header.stamp − 받은 시각을 {every_s:g}초마다 {window_s:g}초씩 모아 직선 맞춤",
        "reference": {
            "gnss_minus_pc_s": round(st.median(samples["ref"]), 4) if samples["ref"] else None,
            "note": "GNSS 진짜 시각(/gps/time_ref) − PC 가 받은 시각. 0 근처면 PC 시계가 맞았다",
        },
        "sensors": {k: {"stamp_minus_recv_s": round(st.median(v), 4), "ok": abs(st.median(v)) < OK_S}
                    for k, v in checks.items() if v},
        "lidar": {
            "stamp_minus_recv_s": {"start": round(a, 4), "end": round(a + b * lid[-1][0], 4)},
            "ok": abs(a) < OK_S and abs(a + b * lid[-1][0]) < OK_S,
        },
        "corrections": [],
    }
    if not out["lidar"]["ok"]:
        out["corrections"].append({
            "applies_to": [LIDAR_APPLIES],
            "stamp": "header.stamp (라이다 원시 패킷 안의 시각 필드도 같은 값)",
            "model": "linear",
            # 보정된 시각 = 기록된 시각 − (a + b × (받은 시각 − t0))
            "t0_unix": round(t_start, 6),
            "a_s": round(a, 6),
            "b_s_per_s": round(b, 9),
            "fit_residual_p95_ms": round(fit_p95_ms, 2),
            "trusted": fit_p95_ms <= FIT_OK_MS,
            "samples": len(lid),
        })
    return out


# ---------------- 보정된 bag 내보내기 ----------------
# 원본을 처음부터 끝까지 한 번 읽으며 라이다 메시지의 시각 바이트만 고쳐 새 sqlite 에 쓴다 (역직렬화 없음):
#   - header 가 첫 필드인 메시지(Imu · PointCloud2 · Image · LaserScan · Telemetry): CDR 4바이트 뒤 sec(int32) · nanosec(uint32)
#   - /ouster/lidar_packets (PacketMsg, RNG19_RFL8_SIG16_NIR16): buf 32바이트 머리 뒤 열 16개, 열마다 첫 8바이트가 시각(ns)
#   - /ouster/imu_packets (LEGACY 48바이트): 앞 24바이트가 시각 3개(ns)
# 점군의 점별 t(스캔 시작 기준 상대값)와 bag 기록 시각(PC 가 받은 시각, 원래 맞음)은 그대로 둔다.
import os
import shutil
import struct
import time

HEADER_TYPES = {"sensor_msgs/msg/Imu", "sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Image",
                "sensor_msgs/msg/LaserScan", "ouster_sensor_msgs/msg/Telemetry"}
LIDAR_PACKET_LEN = 24832                        # 32 + 16 × (12 + 128 × 12) + 32
LIDAR_COL_STRIDE, LIDAR_COLS = 1548, 16
IMU_PACKET_LEN = 48
SIDE_FILES = ("metadata.yaml", "dataset_info.json", "gnss_track.json")


def _patch_header(data, off_ns):
    sec, nsec = struct.unpack_from("<iI", data, 4)
    t = sec * 1_000_000_000 + nsec - off_ns
    b = bytearray(data)
    struct.pack_into("<iI", b, 4, t // 1_000_000_000, t % 1_000_000_000)
    return bytes(b)


def _patch_packet(data, off_ns, kind):
    b = bytearray(data)
    n = struct.unpack_from("<I", b, 4)[0]           # uint8[] 길이
    base = 8
    if kind == "lidar":
        if n != LIDAR_PACKET_LEN:
            raise ValueError(f"라이다 패킷 길이 {n} ≠ {LIDAR_PACKET_LEN} (UDP 프로필이 다름)")
        for i in range(LIDAR_COLS):
            p = base + 32 + i * LIDAR_COL_STRIDE
            t = struct.unpack_from("<Q", b, p)[0]
            if t:
                struct.pack_into("<Q", b, p, t - off_ns)
    else:
        if n != IMU_PACKET_LEN:
            raise ValueError(f"IMU 패킷 길이 {n} ≠ {IMU_PACKET_LEN}")
        for k in range(3):
            p = base + 8 * k
            t = struct.unpack_from("<Q", b, p)[0]
            if t:
                struct.pack_into("<Q", b, p, t - off_ns)
    return bytes(b)


def export(bag_dir, out_root, log=print):
    """보정 파일대로 라이다 시각을 고친 bag 을 out_root/<route>/<녹화>/ 에 쓴다. 반환: 새 폴더."""
    bag_dir = Path(bag_dir).resolve()
    corr = json.loads((bag_dir / CORRECTION_FILE).read_text(encoding="utf-8"))
    lin = next((c for c in corr["corrections"] if c.get("model") == "linear"), None)
    if not lin:
        raise LookupError(f"{bag_dir.name}: 보정할 것이 없습니다 (라이다 정상)")
    if not lin.get("trusted"):
        raise RuntimeError(f"{bag_dir.name}: 보정값이 불확실(직선 오차 {lin['fit_residual_p95_ms']} ms) — 내보내지 않음")
    a, b_, t0 = lin["a_s"], lin["b_s_per_s"], lin["t0_unix"]
    dest = Path(out_root) / bag_dir.parent.name / bag_dir.name
    if dest.exists():
        raise FileExistsError(f"{dest} 가 이미 있습니다")
    part = dest.with_name(dest.name + ".partial")
    if part.exists():
        shutil.rmtree(part)
    part.mkdir(parents=True)
    dbs = sorted(bag_dir.glob("*.db3"))
    total = sum(p.stat().st_size for p in dbs)
    done, t_begin, last = 0, time.time(), 0.0
    counts = {"header": 0, "lidar": 0, "imu": 0, "copied": 0}
    for db in dbs:
        src = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        out = sqlite3.connect(part / db.name)
        out.execute("PRAGMA journal_mode=OFF")
        out.execute("PRAGMA synchronous=OFF")
        for (sql,) in src.execute("select sql from sqlite_master where type='table'"):
            out.execute(sql)
        for table in ("schema", "metadata", "topics"):
            cols = [r[1] for r in src.execute(f"pragma table_info({table})")]
            if cols:
                out.executemany(f"insert into {table} values ({','.join('?' * len(cols))})",
                                src.execute(f"select * from {table}"))
        kind = {}
        for i, name, typ in src.execute("select id, name, type from topics"):
            if not name.startswith("/ouster/"):
                continue
            if typ in HEADER_TYPES:
                kind[i] = "header"
            elif name == "/ouster/lidar_packets":
                kind[i] = "lidar"
            elif name == "/ouster/imu_packets":
                kind[i] = "imu"
        batch = []
        for mid, tid, recv, data in src.execute("select id, topic_id, timestamp, data from messages order by id"):
            k = kind.get(tid)
            if k:
                off_ns = round((a + b_ * (recv / 1e9 - t0)) * 1e9)
                data = _patch_header(data, off_ns) if k == "header" else _patch_packet(data, off_ns, k)
                counts[k] += 1
            else:
                counts["copied"] += 1
            batch.append((mid, tid, recv, data))
            done += len(data)
            if len(batch) >= 2000:
                out.executemany("insert into messages values (?,?,?,?)", batch)
                batch.clear()
                if time.time() - last > 30:
                    last = time.time()
                    rate = done / max(1e-3, last - t_begin) / 1e6
                    log(f"   {bag_dir.name}: {done / total * 100:5.1f}%  {rate:.0f} MB/s  "
                        f"남은 시간 약 {(total - done) / 1e6 / max(rate, 1) / 60:.0f}분")
        if batch:
            out.executemany("insert into messages values (?,?,?,?)", batch)
        out.commit()
        log(f"   {bag_dir.name}: 시각 색인 만드는 중…")
        for (sql,) in src.execute("select sql from sqlite_master where type='index' and sql is not null"):
            out.execute(sql)
        out.commit()
        out.close()
        src.close()
    for f in SIDE_FILES:
        if (bag_dir / f).exists():
            shutil.copy2(bag_dir / f, part / f)
    corr["applied"] = {"at": dt.datetime.now().isoformat(timespec="seconds"), "source": str(bag_dir), **counts}
    (part / CORRECTION_FILE).write_text(json.dumps(corr, ensure_ascii=False, indent=1), encoding="utf-8")
    os.replace(part, dest)
    return dest, counts


def main():
    ap = argparse.ArgumentParser(description="녹화 센서 시각 보정값 재기 · 보정된 bag 내보내기")
    ap.add_argument("cmd", choices=["measure", "export"])
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--dry-run", action="store_true", help="보정 파일을 쓰지 않고 결과만")
    ap.add_argument("--every", type=float, default=5.0, help="몇 초마다 읽을지 (기본 5)")
    ap.add_argument("--out", default=None, help="export: 내보낼 곳 (기본 <clips>/time_corrected)")
    a = ap.parse_args()
    if a.cmd == "export":
        failed = 0
        for bag in a.bags:
            bag = Path(bag).resolve()
            out_root = Path(a.out) if a.out else bag.parent.parent / "time_corrected"
            t = time.time()
            try:
                print(f"▶ {bag.name} 내보내기 → {out_root / bag.parent.name / bag.name}", flush=True)
                dest, counts = export(bag, out_root, log=lambda m: print(m, flush=True))
                chk = measure(dest)                       # 새 bag 을 다시 재서 확인
                lid = chk["lidar"]["stamp_minus_recv_s"]
                ok = chk["lidar"]["ok"]
                print(f"{'✔' if ok else '✖'} {bag.name}: {(time.time() - t) / 60:.1f}분 · 고친 메시지 "
                      f"헤더 {counts['header']} · 라이다 패킷 {counts['lidar']} · IMU 패킷 {counts['imu']} · "
                      f"그대로 {counts['copied']} → 새 bag 라이다 오차 {lid['start'] * 1000:+.1f} ~ {lid['end'] * 1000:+.1f} ms",
                      flush=True)
                if not ok:
                    failed += 1
            except Exception as e:
                failed += 1
                print(f"✖ {bag.name}: {e}", flush=True)
        sys.exit(1 if failed else 0)
    failed = 0
    for bag in a.bags:
        try:
            r = measure(bag, every_s=a.every)
        except Exception as e:
            failed += 1
            print(f"✖ {Path(bag).name}: {e}")
            continue
        lid = r["lidar"]["stamp_minus_recv_s"]
        c = r["corrections"][0] if r["corrections"] else None
        print(f"■ {r['bag']}  GNSS−PC {r['reference']['gnss_minus_pc_s']:+.3f}s  "
              + "  ".join(f"{k} {v['stamp_minus_recv_s']:+.3f}s{'' if v['ok'] else ' ✖'}" for k, v in r["sensors"].items()))
        if c:
            print(f"   라이다 {lid['start']:+.3f}s → {lid['end']:+.3f}s  (기울기 {c['b_s_per_s'] * 1000:+.3f} ms/s, "
                  f"직선 오차 p95 {c['fit_residual_p95_ms']:.2f} ms, 샘플 {c['samples']}) "
                  f"→ {'보정 가능' if c['trusted'] else '⚠ 보정 불확실'}")
        else:
            print(f"   라이다 정상 ({lid['start']:+.3f}s) — 보정 없음")
        if not a.dry_run:
            (Path(bag) / CORRECTION_FILE).write_text(json.dumps(r, ensure_ascii=False, indent=1), encoding="utf-8")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
