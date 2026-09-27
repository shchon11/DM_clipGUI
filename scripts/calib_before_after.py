#!/usr/bin/env python3
# calib_before_after.py — 끝난 캘리브레이션 작업의 "전(before)" 투영 이미지를 만든다.
#
# 도구(nontarget_cal)는 결과 폴더에 새 캘값(after)으로 라이다를 영상에 그린 이미지를 남긴다
# (out/images/parked/<cam>.jpg · out/images/driving/<장면>_<cam>.jpg). 여기서는 같은 장면 · 같은 그리기로
# 이전 캘값(before — 작업을 시작할 때 차량에 적용돼 있던 값 등)을 그려 나란히 볼 수 있게 한다.
# 도구의 venv 파이썬으로 돈다 (nontarget_cal 을 import):
#
#   <venv>/bin/python calib_before_after.py --work W --out O --before DIR --dest D
#
# DIR = extrinsic/ + intrinsic/ (도구 결과 · calib_apply.export_vehicle_init 형식). before 에 없는 카메라는 그리지 않는다.
# 열화상 주행 장면은 시간 모델(시간 오프셋 · 행 읽기)을 after 것으로 쓴다 — 기하(자세 · 렌즈)만 비교.
import argparse
import json
import sys
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--work", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--before", required=True)
    ap.add_argument("--dest", required=True)
    a = ap.parse_args()

    from nontarget_cal.init_calib import load_init
    from nontarget_cal.outputs.images import make_images
    from nontarget_cal.workspace import Workspace

    work, out, dest = Path(a.work), Path(a.out), Path(a.dest)
    cfg = json.loads((work / "config_snapshot.json").read_text())
    summary = json.loads((out / "summary.json").read_text())
    ini = load_init(a.before)
    dest.mkdir(parents=True, exist_ok=True)

    rgb_json = th_json = None
    if ini["rgb"]:
        rgb_json = dest / "before_rgb.json"
        rgb_json.write_text(json.dumps({"cameras": {c: {"T_cam_lidar": v["T_cam_lidar"], "intr_kb": list(v["intr"])}
                                                    for c, v in ini["rgb"].items()}}))
    after_th = work / "thermal" / "final" / "result.json"
    if ini["thermal"] and after_th.is_file():
        th = json.loads(after_th.read_text())
        th["cameras"] = {c: dict(th["cameras"][c], T_cam_lidar=v["T_cam_lidar"], intr=list(v["intr"]))
                         for c, v in ini["thermal"].items() if c in th.get("cameras", {})}
        th_json = dest / "before_thermal.json"
        th_json.write_text(json.dumps(th))

    ws = Workspace(work)
    parked = sorted(d.name for d in (work / "extract").glob("*_parked") if (d / "source.json").exists())
    moving = (summary.get("windows") or {}).get("kept") or []
    r = make_images(ws, cfg, rgb_json, th_json, parked, moving, dest, Path(cfg["paths"]["masks_dir"]),
                    log=lambda *x: print(*x, file=sys.stderr))
    (dest / "done.json").write_text(json.dumps({"before": str(a.before), "images": len(r.get("images", [])),
                                                "rgb": sorted(ini["rgb"]), "thermal": sorted(ini["thermal"])}))
    print(json.dumps({"ok": True, "images": len(r.get("images", []))}))


if __name__ == "__main__":
    main()
