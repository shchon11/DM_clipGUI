#!/usr/bin/env python3
# calib_compare.py — 끝난 캘리브레이션 작업에서 "이전 캘값(before)" 과 "새 결과" 중 어느 쪽이 나은지 잰다.
#
# 공정하게: 새 결과는 이 데이터로 맞춘 값이라 같은 데이터에서 재면 당연히 유리하다. 그래서 도구의 검증과 같은 잣대를 쓴다.
#   · held-out 재투영 — 도구는 창을 두 절반(A · B)으로 나눠 A 로 푼 캘값을 B 에서 (B 로 푼 것을 A 에서) 캘값을 고정하고
#     잰다 (validation.json 의 heldout). 이전 캘값은 이 데이터와 무관하므로 같은 두 절반에서 똑같이 고정하고 잰다.
#     → 새 쪽은 데이터 절반으로만 푼 값이라 오히려 불리한 조건.
#   · LiDAR 에지 정렬 — 풀이에 안 쓰는 독립 지표. 같은 프레임에서 두 캘값으로 잰다 (RGB: 정차 장면, 열화상: 점-선 거리 · 투표).
# 시간 모델(카메라 시간 오프셋 · 행 읽기)은 새 결과의 것을 두 쪽에 똑같이 쓴다 — 기하(자세 · 렌즈)만 비교.
# 도구의 venv 파이썬으로 돈다:
#   <venv>/bin/python calib_compare.py --work W --before DIR --dest D
# 결과: D/compare.json (카메라별 전/후 수치와 판정, 전체 권고). 중간 풀이는 W/rgb|thermal/cmp_<id>_* 에 남는다.
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

# 판정 문턱: held-out 재투영 차이가 이보다 커야 "낫다" (절반 데이터 풀이의 산포 · 반올림을 넘는 차이)
HELDOUT_ABS_PX = 0.03
HELDOUT_REL = 0.04
EDGE_RGB_PX = 1.0          # 정차 장면 에지 거리 중앙값 (RGB, 수 px 단위 잡음)
EDGE_TH_PX = 0.25          # 열화상 점-선 거리 중앙값


def _better(new, old, abs_thr, rel_thr=0.0):
    """낮을수록 좋은 지표: +1 새 쪽이 확실히 낮음, -1 이전 쪽이 확실히 낮음, 0 비슷/모름."""
    if new is None or old is None or not np.isfinite(new) or not np.isfinite(old):
        return 0
    thr = max(abs_thr, rel_thr * min(new, old))
    return 1 if old - new > thr else (-1 if new - old > thr else 0)


def verdict(signals):
    """signals: [(이름, +1/0/-1, 강함?)] → ("new"|"old"|"same"|"mixed", 이유 목록)."""
    pos = [n for n, s, _ in signals if s > 0]
    neg = [n for n, s, _ in signals if s < 0]
    if pos and not neg:
        return "new", pos
    if neg and not pos:
        return "old", neg
    if pos and neg:
        strong_pos = [n for n, s, st in signals if s > 0 and st]
        strong_neg = [n for n, s, st in signals if s < 0 and st]
        if strong_pos and not strong_neg:
            return "new", strong_pos
        if strong_neg and not strong_pos:
            return "old", strong_neg
        return "mixed", pos + neg
    return "same", []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--work", required=True)
    ap.add_argument("--before", required=True)
    ap.add_argument("--dest", required=True)
    a = ap.parse_args()

    from nontarget_cal import tasks
    from nontarget_cal.init_calib import load_init
    from nontarget_cal.pipeline import Pipeline
    from nontarget_cal.workspace import Workspace, read_json

    log = lambda *x: print(*x, file=sys.stderr, flush=True)      # noqa: E731
    work, dest = Path(a.work), Path(a.dest)
    dest.mkdir(parents=True, exist_ok=True)
    cfg = json.loads((work / "config_snapshot.json").read_text())
    ws = Workspace(work)
    me = SimpleNamespace(cfg=cfg)                     # Pipeline._rgb_opts / _thermal_opts 는 cfg 만 쓴다
    tag = "cmp_" + hashlib.sha1(str(Path(a.before).resolve()).encode()).hexdigest()[:8]
    ini = load_init(a.before)
    val = read_json(work / "validation" / "validation.json")
    out = {"before": str(a.before), "rgb": {}, "thermal": {}}

    # ---------------- RGB
    rs_path = work / "rgb" / "summary.json"
    if rs_path.exists() and ini["rgb"]:
        rs = read_json(rs_path)
        fin = rs["final"]
        new = read_json(fin)
        old = copy.deepcopy(new)
        have = []
        for c, v in ini["rgb"].items():
            if c in old["cameras"]:
                T = np.array(v["T_cam_lidar"], float)
                old["cameras"][c].update(T_cam_lidar=T.tolist(), T_lidar_cam=np.linalg.inv(T).tolist(),
                                         intr_kb=list(map(float, v["intr"]))[:8])
                have.append(c)
        old_path = dest / "old_rgb.json"
        old_path.write_text(json.dumps(old))
        cams = list(cfg["cameras"]["rgb"])
        wins = rs["windows"]
        A, B = wins[0::2], wins[1::2]
        ho = {}
        for h, segs in (("A", A), ("B", B)):
            if not segs:
                continue
            opts = dict(Pipeline._rgb_opts(me, len(segs)), ties=False, seg_span_s=None, frame_corr=None, obs_weight=None)
            log(f"RGB held-out: 이전 캘값을 절반 {h} ({len(segs)}창)에서 고정")
            r = tasks.t_rgb_solve(ws, cfg, log, f"{tag}_on{h}", segs, cams, {"kind": "result", "path": fin}, opts,
                                  held=str(old_path))
            ho[h] = read_json(r["path"])
        parked = sorted(d.name for d in (work / "extract").glob("*_parked") if (d / "source.json").exists())
        edge_wins = wins[:: max(1, len(wins) // 6)][:6]
        log("RGB 에지: 이전 캘값")
        old_edges = tasks.t_rgb_edges(ws, cfg, log, str(old_path), edge_wins, parked, cams,
                                      str(dest / "old_rgb_edges.json"), vote=cfg["validation"]["vote"])
        vr = val.get("rgb") or {}
        for c in cams:
            o_ho = [ho[h]["cameras"][c]["reproj_median_px"] for h in ho if ho[h]["cameras"].get(c, {}).get("reproj_median_px") is not None]
            row = {"has_old": c in have,
                   "heldout_new": (vr.get("heldout") or {}).get(c),
                   "heldout_old": float(np.mean(o_ho)) if len(o_ho) == len(ho) and o_ho else None}
            for kind, e in (("new", (vr.get("edges") or {}).get(c)), ("old", old_edges.get(c))):
                e = e or {}
                pk = e.get("parked") or {}
                row[f"edge_{kind}"] = pk.get("median_px")
            sig = [("held-out 재투영", _better(row["heldout_new"], row["heldout_old"], HELDOUT_ABS_PX, HELDOUT_REL), True),
                   ("정차 장면 에지", _better(row["edge_new"], row["edge_old"], EDGE_RGB_PX), False)]
            row["verdict"], row["why"] = verdict(sig) if row["has_old"] else ("no_old", [])
            out["rgb"][c] = row

    # ---------------- 열화상
    ts_path = work / "thermal" / "summary.json"
    if ts_path.exists() and ini["thermal"]:
        ts = read_json(ts_path)
        tfin = ts["final"]
        new = read_json(tfin)
        old = copy.deepcopy(new)
        have = []
        for c, v in ini["thermal"].items():
            if c in old["cameras"]:
                T = np.array(v["T_cam_lidar"], float)
                old["cameras"][c].update(T_cam_lidar=T.tolist(), T_lidar_cam=np.linalg.inv(T).tolist(),
                                         intr=list(map(float, v["intr"]))[:6], focal_px=float(v["intr"][0]))
                have.append(c)
        old_path = dest / "old_thermal.json"
        old_path.write_text(json.dumps(old))
        twins = ts["windows"]
        TA, TB = twins[0::2], twins[1::2]
        rgb_res = str(work / "rgb" / "final" / "result.json") if (work / "rgb" / "final" / "result.json").exists() else None
        ho = {}
        for h, segs in (("A", TA), ("B", TB)):
            if not segs:
                continue
            o = dict(Pipeline._thermal_opts(me, "final"), free=["dt"], edges=False, ties=False, lidar_report=False)
            log(f"열화상 held-out: 이전 캘값을 절반 {h} ({len(segs)}창)에서 고정")
            r = tasks.t_thermal_solve(ws, cfg, log, f"{tag}_on{h}", segs, {"path": str(old_path)}, o,
                                      rgb_result=rgb_res, held=True)
            ho[h] = read_json(r["path"])
        log("열화상 에지: 이전 캘값")
        old_edges = tasks.t_thermal_edge_eval(ws, cfg, log, str(old_path), twins, str(dest / "old_thermal_edges.json"))
        vt = val.get("thermal") or {}
        for c in new["cams"] if "cams" in new else new["cameras"]:
            o_ho = [ho[h]["cameras"][c]["reproj_median_px"] for h in ho if c in ho[h]["cameras"]]
            en, eo = (vt.get("edges") or {}).get(c) or {}, old_edges.get(c) or {}
            row = {"has_old": c in have,
                   "heldout_new": (vt.get("heldout") or {}).get(c),
                   "heldout_old": float(np.mean(o_ho)) if len(o_ho) == len(ho) and o_ho else None,
                   "edge_new": en.get("assoc_point_to_line_median_px"), "edge_old": eo.get("assoc_point_to_line_median_px"),
                   "vote_new": (en.get("vote") or {}).get("pass"), "vote_old": (eo.get("vote") or {}).get("pass")}
            vote = 0
            if row["vote_new"] is True and row["vote_old"] is False:
                vote = 1
            elif row["vote_new"] is False and row["vote_old"] is True:
                vote = -1
            sig = [("held-out 재투영", _better(row["heldout_new"], row["heldout_old"], HELDOUT_ABS_PX, HELDOUT_REL), True),
                   ("LiDAR 에지 거리", _better(row["edge_new"], row["edge_old"], EDGE_TH_PX), True),
                   ("에지 투표 게이트", vote, True)]
            row["verdict"], row["why"] = verdict(sig) if row["has_old"] else ("no_old", [])
            out["thermal"][c] = row

    rows = {**out["rgb"], **out["thermal"]}
    n = {k: sum(1 for r in rows.values() if r["verdict"] == k) for k in ("new", "old", "same", "mixed", "no_old")}
    out["counts"] = n
    if n["old"]:
        out["advice"] = "part"          # 기존이 나은 카메라는 빼고 적용
    elif n["new"]:
        out["advice"] = "apply"
    else:
        out["advice"] = "either"
    (dest / "compare.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print(json.dumps({"ok": True, "counts": n, "advice": out["advice"]}))


if __name__ == "__main__":
    main()
