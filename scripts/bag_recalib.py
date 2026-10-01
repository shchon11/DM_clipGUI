#!/usr/bin/env python3
# bag_recalib.py — 지금 차량에 들어가 있는 캘리브레이션(camera_info · TF)을 이미 녹화한 bag 에도 넣는다.
#
# 캘리브레이션 탭의 [차량에 적용] 은 FLIR_control 의 두 파일(시리얼 키)을 바꾸고, 카메라 노드를 다시 켜야
# 다음 녹화부터 반영된다. 그 전에 녹화한 bag 은 옛 값이라, 이 모듈이 bag 안의
#   - /<카메라>/camera_info : distortion_model · d · k · r · p 를 새 값으로 (header · 크기 · binning · roi 는 그대로)
#   - /tf_static           : 부모(os_lidar) → 각 카메라 광학 프레임 변환을 새 값으로 (없던 것은 채워 넣는다 —
#                            레코더가 /tf_static 을 토픽당 마지막 메시지 하나만 남겨 카메라 TF 가 빠진 bag 이 있다)
# 을 바꾼다.
#
# 어떻게:
#   - bag 의 dataset_info.json "cameras" = {시리얼: 토픽 이름} (녹화 당시 인벤토리)로 토픽 → 시리얼을 찾는다.
#     카메라 이름이 나중에 바뀌어도 시리얼로 정확히 맞는다.
#   - sqlite 를 새로 쓰지 않고 그 행만 UPDATE 한다 (영상은 안 건드려 빠르고, 디스크도 더 안 든다).
#     id 범위를 잘라 읽어 진행률(%)을 내고, bag 하나를 트랜잭션 하나로 — 중단 · 전원 꺼짐에도 반쯤 바뀐 bag 이 없다.
#   - 바꾸기 전 값을 bag 폴더 calib_backup_<시각>.json 에 남겨 되돌릴 수 있다 (revert).
#   - 끝나면 몇 개를 다시 읽어 값이 맞는지 확인하고 dataset_info.json "calibration" 에 기록한다.
#
#   python3 bag_recalib.py plan   <bag> [...]        # 무엇을 바꿀지만
#   python3 bag_recalib.py apply  <bag> [...]        # 바꾸기 (진행률 표시)
#   python3 bag_recalib.py revert <bag> [...]        # 마지막 적용 되돌리기

import base64
import datetime as dt
import glob
import hashlib
import json
import os
import sqlite3
import sys
import time
from pathlib import Path

import calib_apply as ca

CI_TYPE = "sensor_msgs/msg/CameraInfo"
TF_TYPE = "tf2_msgs/msg/TFMessage"
CHUNK_IDS = 20000                 # 한 번에 읽는 행 id 범위 (진행률 · 중단 확인 간격)
BACKUP_GLOB = "calib_backup_*.json"


class Cancelled(Exception):
    pass


def _msgs():
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message, serialize_message
    return get_message, deserialize_message, serialize_message


def _sha(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12]
    except OSError:
        return ""


# ---------------------------------------------------------------- 차량 캘리브레이션 읽기
def vehicle_calib(ci_path, ex_path):
    """지금 차량 파일 → {"ci_text", "ex": {시리얼: {parent_frame, t, q}}, "source": {...}}."""
    ci_path, ex_path = Path(ci_path), Path(ex_path)
    ci_text = ci_path.read_text(encoding="utf-8")
    ex_items = ca.parse_extrinsics_like_node(ex_path.read_text(encoding="utf-8"))
    return {
        "ci_text": ci_text,
        "ex": {str(it["serial"]): it for it in ex_items},
        "source": {
            "camera_info": str(ci_path), "camera_info_sha": _sha(ci_path),
            "camera_info_mtime": dt.datetime.fromtimestamp(ci_path.stat().st_mtime).isoformat(timespec="seconds"),
            "extrinsics": str(ex_path), "extrinsics_sha": _sha(ex_path),
            "extrinsics_mtime": dt.datetime.fromtimestamp(ex_path.stat().st_mtime).isoformat(timespec="seconds"),
        },
    }


def _ci_for(calib, serial):
    try:
        return ca.parse_camera_info_like_node(calib["ci_text"], str(serial))
    except ValueError:
        return None


# ---------------------------------------------------------------- bag 살펴보기
def _db(bag):
    dbs = sorted(glob.glob(str(Path(bag) / "*.db3")))
    if not dbs:
        raise FileNotFoundError(f"{Path(bag).name}: .db3 가 없습니다 (sqlite3 녹화만 지원)")
    if len(dbs) > 1:
        raise RuntimeError(f"{Path(bag).name}: db3 가 {len(dbs)}개 — 나뉜 bag 은 아직 지원하지 않습니다")
    return dbs[0]


def bag_info(bag):
    try:
        return json.loads((Path(bag) / "dataset_info.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def plan(bag, calib):
    """무엇을 바꿀지: {"cameras": [{topic, ns, serial, ci, ex}], "tf_rows": n, "problems": [...], "ok": bool}."""
    bag = Path(bag)
    problems = []
    if not (bag / "metadata.yaml").exists():
        return {"cameras": [], "tf_rows": 0, "problems": ["녹화가 아직 안 끝났거나 깨진 bag (metadata.yaml 없음)"], "ok": False}
    db = _db(bag)
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    topics = {n: (i, t) for n, i, t in con.execute("select name, id, type from topics")}
    tf_rows = 0
    if "/tf_static" in topics:
        tf_rows = con.execute("select count(*) from messages where topic_id=?", (topics["/tf_static"][0],)).fetchone()[0]
    con.close()
    serial_of = {str(ns): str(s) for s, ns in (bag_info(bag).get("cameras") or {}).items()}
    if not serial_of:
        problems.append("녹화 당시 카메라 시리얼 기록(dataset_info.json cameras)이 없어 카메라를 확실히 알 수 없습니다")
    cams = []
    for name, (_, typ) in sorted(topics.items()):
        if typ != CI_TYPE or not name.endswith("/camera_info"):
            continue
        ns = name.strip("/").split("/")[0]
        serial = serial_of.get(ns)
        ci = _ci_for(calib, serial) if serial else None
        ex = calib["ex"].get(serial) if serial else None
        cams.append({"topic": name, "ns": ns, "serial": serial, "ci": ci is not None, "ex": ex is not None})
        if not serial:
            problems.append(f"{ns}: 시리얼 모름 — 건너뜀")
        elif ci is None and ex is None:
            problems.append(f"{ns} ({serial}): 차량 캘리브레이션에 이 카메라가 없음 — 건너뜀")
    if tf_rows == 0 and any(c["ex"] for c in cams):
        problems.append("/tf_static 이 bag 에 없어 TF 는 못 넣습니다 (camera_info 만 바꿈)")
    ok = any(c["ci"] or c["ex"] for c in cams)
    return {"cameras": cams, "tf_rows": tf_rows, "problems": problems, "ok": ok}


def _recorder_open(db):
    """레코더가 지금 이 db 를 열고 있는지 (녹화 중)."""
    target = os.path.realpath(db)
    for p in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.path.realpath(p) == target:
                return True
        except OSError:
            continue
    return False


# ---------------------------------------------------------------- 적용
def _ci_template(first_bytes, fields, deser, ser, cls):
    m = deser(first_bytes, cls)
    m.distortion_model = fields.get("distortion_model", m.distortion_model)
    for key in ("d", "k", "r", "p"):
        if key in fields:
            setattr(m, key, [float(x) for x in fields[key]])
    return ser(m)


def _patch_tf(data, frames_new, deser, ser, cls):
    """TFMessage 에서 카메라 프레임 변환을 새 값으로 바꾸거나 더한다. (새 bytes, 바꾼 수, 더한 수)."""
    from geometry_msgs.msg import TransformStamped
    m = deser(data, cls)
    stamp = m.transforms[0].header.stamp if m.transforms else None
    changed = added = 0
    by_child = {t.child_frame_id: t for t in m.transforms}
    for child, (parent, t, q) in frames_new.items():
        tr = by_child.get(child)
        if tr is None:
            tr = TransformStamped()
            if stamp is not None:
                tr.header.stamp = stamp
            m.transforms.append(tr)
            added += 1
        else:
            changed += 1
        tr.header.frame_id = parent
        tr.child_frame_id = child
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = map(float, t)
        (tr.transform.rotation.x, tr.transform.rotation.y,
         tr.transform.rotation.z, tr.transform.rotation.w) = map(float, q)
    return ser(m), changed, added


def apply(bag, calib, progress=None, cancel=None, log=print):
    """bag 에 새 캘리브레이션을 넣는다. progress(0~1, 문구), cancel() → True 면 중단(Cancelled, 원래대로).

    반환: 요약 dict (dataset_info.json "calibration" 에도 기록).
    """
    get_message, deser, ser = _msgs()
    bag = Path(bag)
    pl = plan(bag, calib)
    if not pl["ok"]:
        raise RuntimeError("; ".join(pl["problems"]) or "바꿀 카메라가 없습니다")
    db = _db(bag)
    if _recorder_open(db):
        raise RuntimeError("지금 녹화 중인 bag 입니다 — 녹화가 끝난 뒤에 하세요")
    ci_cls, tf_cls = get_message(CI_TYPE), get_message(TF_TYPE)
    con = sqlite3.connect(db, isolation_level=None, timeout=5)
    topics = {n: i for n, i in con.execute("select name, id from topics")}
    targets = {}                                            # topic_id → (ns, 새 필드)
    frames_new = {}                                         # 카메라 광학 프레임 → (parent, t, q)
    backup = {"bag": bag.name, "created_at": dt.datetime.now().isoformat(timespec="seconds"),
              "camera_info": {}, "tf_static": [], "calibration_before": bag_info(bag).get("calibration")}
    for c in pl["cameras"]:
        if not c["serial"]:
            continue
        tid = topics[c["topic"]]
        first = con.execute("select data from messages where topic_id=? order by id limit 1", (tid,)).fetchone()
        if not first:
            continue
        old = deser(first[0], ci_cls)
        backup["camera_info"][c["topic"]] = {"distortion_model": old.distortion_model, "d": list(old.d),
                                             "k": list(old.k), "r": list(old.r), "p": list(old.p)}
        fields = _ci_for(calib, c["serial"]) if c["ci"] else None
        if fields:
            targets[tid] = (c["ns"], _ci_template(first[0], fields, deser, ser, ci_cls), bytes(first[0][12:]))
        ex = calib["ex"].get(c["serial"]) if c["ex"] else None
        if ex and old.header.frame_id:
            frames_new[old.header.frame_id] = (ex["parent_frame"], ex["t"], ex["q"])
    tf_tid = topics.get("/tf_static")
    if tf_tid is not None:
        for rid, data in con.execute("select id, data from messages where topic_id=?", (tf_tid,)):
            backup["tf_static"].append({"id": rid, "data_b64": base64.b64encode(bytes(data)).decode()})

    lo, hi = con.execute("select min(id), max(id) from messages").fetchone()
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = bag / f"calib_backup_{stamp}.json"
    backup_path.write_text(json.dumps(backup, ensure_ascii=False), encoding="utf-8")

    ids = list(targets)
    n_ci = n_slow = 0
    t0 = time.time()
    con.execute("BEGIN IMMEDIATE")
    try:
        if ids:
            q = (f"select id, topic_id, data from messages where id between ? and ? "
                 f"and topic_id in ({','.join('?' * len(ids))})")
            a = lo
            while a <= hi:
                if cancel and cancel():
                    raise Cancelled()
                b = a + CHUNK_IDS - 1
                updates = []
                for rid, tid, data in con.execute(q, (a, b, *ids)):
                    ns, tmpl, first_tail = targets[tid]
                    data = bytes(data)
                    if data[12:] == first_tail:              # header 시각만 다름 → 틀에 시각만 끼운다
                        new = tmpl[:4] + data[4:12] + tmpl[12:]
                    else:                                   # 드물게 다른 필드가 다르면 하나씩
                        m = deser(data, ci_cls)
                        f = _ci_for(calib, next(c["serial"] for c in pl["cameras"] if c["ns"] == ns))
                        m.distortion_model = f.get("distortion_model", m.distortion_model)
                        for key in ("d", "k", "r", "p"):
                            if key in f:
                                setattr(m, key, [float(x) for x in f[key]])
                        new = ser(m)
                        n_slow += 1
                    updates.append((new, rid))
                if updates:
                    con.executemany("update messages set data=? where id=?", updates)
                    n_ci += len(updates)
                if progress:
                    frac = (min(b, hi) - lo + 1) / max(1, hi - lo + 1)
                    el = time.time() - t0
                    eta = el / frac * (1 - frac) if frac > 0.02 else None
                    progress(frac * 0.97, f"camera_info {n_ci:,}개 바꿈" + (f" · 약 {eta / 60:.0f}분 남음" if eta else ""))
                a = b + 1
        tf_changed = tf_added = 0
        if tf_tid is not None and frames_new:
            for rec in backup["tf_static"]:
                new, ch, ad = _patch_tf(base64.b64decode(rec["data_b64"]), frames_new, deser, ser, tf_cls)
                con.execute("update messages set data=? where id=?", (new, rec["id"]))
                tf_changed += ch
                tf_added += ad
        if cancel and cancel():
            raise Cancelled()
        con.execute("COMMIT")
    except BaseException:
        con.execute("ROLLBACK")
        con.close()
        backup_path.unlink(missing_ok=True)
        raise
    # 확인: 카메라마다 처음 · 가운데 · 끝 하나씩 다시 읽어 새 값인지
    bad = []
    for tid, (ns, tmpl, _) in targets.items():
        want = deser(tmpl, ci_cls)
        rows = con.execute("select data from messages where topic_id=? order by id", (tid,)).fetchall()
        for data, in (rows[0], rows[len(rows) // 2], rows[-1]):
            got = deser(data, ci_cls)
            if list(got.k) != list(want.k) or list(got.d) != list(want.d) or list(got.p) != list(want.p):
                bad.append(ns)
                break
    con.close()
    summary = {
        "applied_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source": calib["source"],
        "camera_info_topics": sorted(targets[t][0] for t in targets),
        "camera_info_messages": n_ci,
        "tf_frames": sorted(frames_new),
        "tf_changed": tf_changed, "tf_added": tf_added,
        "skipped": pl["problems"],
        "backup": backup_path.name,
        "verified": not bad,
        "seconds": round(time.time() - t0, 1),
    }
    if bad:
        summary["verify_failed"] = bad
    import dataset_catalog
    dataset_catalog.write_info(bag, calibration=summary)
    if progress:
        progress(1.0, "완료" + ("" if not bad else f" · ✖ 확인 실패 {bad}"))
    if n_slow:
        log(f"   ({n_slow}개는 다른 필드가 달라 하나씩 바꿈)")
    return summary


def revert(bag, progress=None, cancel=None, log=print):
    """마지막 calib_backup_*.json 으로 되돌린다."""
    get_message, deser, ser = _msgs()
    bag = Path(bag)
    backups = sorted(bag.glob(BACKUP_GLOB))
    if not backups:
        raise FileNotFoundError(f"{bag.name}: 되돌릴 백업이 없습니다")
    bk = json.loads(backups[-1].read_text(encoding="utf-8"))
    fake = {"ci_text": "", "ex": {}, "source": {"revert_of": backups[-1].name}}
    db = _db(bag)
    if _recorder_open(db):
        raise RuntimeError("지금 녹화 중인 bag 입니다")
    ci_cls = get_message(CI_TYPE)
    con = sqlite3.connect(db, isolation_level=None, timeout=5)
    topics = {n: i for n, i in con.execute("select name, id from topics")}
    targets = {}
    for topic, old in bk["camera_info"].items():
        tid = topics.get(topic)
        first = con.execute("select data from messages where topic_id=? order by id limit 1", (tid,)).fetchone()
        if first:
            targets[tid] = (_ci_template(first[0], old, deser, ser, ci_cls), bytes(first[0][12:]), old)
    lo, hi = con.execute("select min(id), max(id) from messages").fetchone()
    ids = list(targets)
    con.execute("BEGIN IMMEDIATE")
    try:
        q = f"select id, topic_id, data from messages where id between ? and ? and topic_id in ({','.join('?' * len(ids))})"
        a = lo
        while ids and a <= hi:
            if cancel and cancel():
                raise Cancelled()
            b = a + CHUNK_IDS - 1
            ups = []
            for rid, tid, data in con.execute(q, (a, b, *ids)):
                tmpl, tail, old = targets[tid]
                data = bytes(data)
                if data[12:] == tail:
                    ups.append((tmpl[:4] + data[4:12] + tmpl[12:], rid))
                else:
                    ups.append((_ci_template(data, old, deser, ser, ci_cls), rid))
            con.executemany("update messages set data=? where id=?", ups)
            if progress:
                progress((min(b, hi) - lo + 1) / max(1, hi - lo + 1) * 0.97, "되돌리는 중")
            a = b + 1
        for rec in bk["tf_static"]:
            con.execute("update messages set data=? where id=?", (base64.b64decode(rec["data_b64"]), rec["id"]))
        con.execute("COMMIT")
    except BaseException:
        con.execute("ROLLBACK")
        con.close()
        raise
    con.close()
    import dataset_catalog
    dataset_catalog.write_info(bag, calibration=bk.get("calibration_before") or {"reverted_at": dt.datetime.now().isoformat(timespec="seconds")})
    backups[-1].rename(backups[-1].with_suffix(".reverted.json"))
    if progress:
        progress(1.0, "되돌림")
    return {"reverted": backups[-1].name}


# ---------------------------------------------------------------- 명령줄
def _vehicle_default():
    import sensor_discovery
    import sensor_config
    reg = sensor_discovery.load_registry()
    g = next(g for g in reg.get("groups", []) if g.get("key") == "flir_cameras")
    defaults = {k: v for k, v in sensor_config.launch_defaults(g).items()
                if k in ("camera_info_yaml_path", "extrinsics_yaml_path")}
    ci, ex, _ = ca.vehicle_paths(g, {}, defaults)
    return ci, ex


def main():
    import argparse
    ap = argparse.ArgumentParser(description="차량 캘리브레이션을 녹화한 bag 에 넣기")
    ap.add_argument("cmd", choices=["plan", "apply", "revert"])
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--camera-info", default=None)
    ap.add_argument("--extrinsics", default=None)
    a = ap.parse_args()
    ci, ex = (a.camera_info, a.extrinsics) if a.camera_info and a.extrinsics else _vehicle_default()
    calib = vehicle_calib(ci, ex)
    print(f"기준: {ci} ({calib['source']['camera_info_mtime']}) · {ex} ({calib['source']['extrinsics_mtime']})")
    failed = 0
    for i, bag in enumerate(a.bags, 1):
        name = Path(bag).name
        try:
            if a.cmd == "plan":
                p = plan(bag, calib)
                print(f"■ {name}: camera_info {sum(c['ci'] for c in p['cameras'])}대 · TF {sum(c['ex'] for c in p['cameras'])}대 "
                      f"· /tf_static {p['tf_rows']}개" + ("" if p["ok"] else " · ✖ 바꿀 것 없음"))
                for pr in p["problems"]:
                    print("   ⚠", pr)
                continue
            last = [0.0]

            def prog(f, text, name=name, i=i):
                if f - last[0] >= 0.05 or f >= 1.0:
                    last[0] = f
                    print(f"   [{i}/{len(a.bags)}] {name} {f * 100:5.1f}% · {text}", flush=True)
            r = apply(bag, calib, prog) if a.cmd == "apply" else revert(bag, prog)
            print(f"✔ {name}: {json.dumps(r, ensure_ascii=False)[:300]}")
        except Exception as e:
            failed += 1
            print(f"✖ {name}: {e}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
