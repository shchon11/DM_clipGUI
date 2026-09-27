"""Metrics table (markdown + json) and the concise run report (Korean)."""
from __future__ import annotations

import json
from pathlib import Path

COLS = [("rot_deg", "rot 1σ [deg]", 3), ("pos_mm", "pos 1σ [mm]", 1), ("along_axis_mm", "along-axis 1σ [mm]", 1),
        ("focal_px", "focal 1σ [px]", 2), ("track_reproj_px", "track reproj [px]", 2),
        ("heldout_track_reproj_px", "held-out reproj [px]", 2), ("lidar_edge_px", "LiDAR edge [px]", 2)]


def metrics_table(v: dict, cfg) -> dict:
    out = {}
    for sensor in ("rgb", "thermal"):
        s = v.get(sensor) or {}
        if not s:
            continue
        sig = s.get("halves", {}).get("sigma", {})
        for c, tr in s.get("track_reproj", {}).items():
            row = {"sensor": sensor, "track_reproj_px": tr}
            row.update(sig.get(c, {}))
            row["heldout_track_reproj_px"] = s.get("heldout", {}).get(c)
            e = s.get("edges", {}).get(c, {})
            if sensor == "rgb":
                # parked (no trajectory involved) is the meaningful one; while driving at night the metric is
                # 12-16 px for every calibration (lidar_odo README 3.5c)
                row["lidar_edge_moving_px"] = (e.get("moving") or {}).get("median_px")
                row["lidar_edge_parked_px"] = (e.get("parked") or {}).get("median_px")
                row["lidar_edge_px"] = row["lidar_edge_parked_px"] if row["lidar_edge_parked_px"] is not None else row["lidar_edge_moving_px"]
                vt = (e.get("parked") or {}).get("vote") or (e.get("moving") or {}).get("vote")
                if vt:
                    vt = dict(vt, pass_=vt.get("pass"), gate=False)
            else:
                row["lidar_edge_px"] = e.get("assoc_point_to_line_median_px")
                row["lidar_edge_dist_px"] = e.get("dist_to_edge_median_px")
                vt = e.get("vote")
            if vt:
                row["vote"] = {k: vt.get(k) for k in ("pass", "du", "dv", "contrast", "cells_at_origin")}
                row["vote"]["gate"] = vt.get("gate", True)
            out[c] = row
    return out


def _fmt(x, nd):
    if x is None:
        return "–"
    try:
        return f"{x:.{nd}f}"
    except (TypeError, ValueError):
        return str(x)


def metrics_md(metrics: dict) -> str:
    lines = ["| camera | " + " | ".join(h for _, h, _ in COLS) + " | vote |",
             "|---|" + "---|" * (len(COLS) + 1)]
    for c, r in metrics.items():
        vt = r.get("vote") or {}
        if not vt.get("gate", True):
            vs = f"info {_fmt(vt.get('du'), 1)}/{_fmt(vt.get('dv'), 1)} px, x{_fmt(vt.get('contrast'), 2)}"
        else:
            vs = "–" if vt.get("pass") is None else ("pass" if vt["pass"] else "FAIL") + \
                f" ({_fmt(vt.get('du'), 1)}/{_fmt(vt.get('dv'), 1)} px, x{_fmt(vt.get('contrast'), 2)})"
        lines.append(f"| {c} | " + " | ".join(_fmt(r.get(k), nd) for k, _, nd in COLS) + f" | {vs} |")
    return "\n".join(lines) + "\n"


def write_all(out: Path, s: dict, cfg):
    m = s["metrics"]
    (out / "metrics.json").write_text(json.dumps(m, indent=1, ensure_ascii=False))
    note = ("\n1σ = 서로 겹치지 않는 두 절반 데이터로 각각 푼 값의 차이 / √2 (절반 데이터 풀이의 산포, 보수적; 전체 데이터 결과는 약 √2배 더 좋음).\n"
            "track reproj = 최종 풀이의 KLT 트랙 재투영 오차 중앙값; held-out = 한쪽 절반으로 얻은 보정을 고정하고 다른 절반에서 잰 값.\n"
            "LiDAR edge: RGB = 정차 장면에서 LiDAR 깊이 에지와 영상(Canny) 에지 거리 중앙값(정차 장면이 없으면 주행 중 값; 야간 주행 중 값은 "
            "어떤 보정이든 12-16 px라 판별력 없음), 열화상 = LO로 누적한 근거리 LiDAR 에지와 열화상 에지 사이 점-선 거리 중앙값.\n"
            "vote: 열화상 = 투표 게이트(영상을 ±8 px 옮기며 에지 일치를 셈: 최고점이 {off} px 안, 대비 ≥ {con}, 3x3 영역의 "
            "{frac:.0f}% 이상이 원점); RGB = 참고용(야간에는 판별력 없음, 게이트 아님).\n")
    g = cfg["validation"]["gate"]
    note = note.format(off=g["vote_max_offset_px"], con=g["vote_min_contrast"], frac=100 * g.get("vote_min_cell_frac", 0))
    (out / "metrics.md").write_text("# 카메라별 지표\n\n" + metrics_md(m) + note)
    v = s["validation"]
    L = [f"# nontarget_cal 결과 보고서\n",
         f"- 버전 {s['version']}, 모드 `{s['mode']}`, 센서 {', '.join(s['sensors'])}",
         f"- bag: " + ", ".join(f"{b} = `{p}`" for b, p in s["bags"].items()),
         f"- 이름 대응: " + ", ".join(f"{b}: {n['source']}" for b, n in s["names"].items()),
         f"- 사용 구간 {len(s['windows']['kept'])}개" + (f", LO 품질 불량으로 제외 {s['windows']['dropped']}" if s['windows']['dropped'] else "") +
         (f", 열화상 구간 {len(s['windows']['thermal'] or [])}개" if "thermal" in s["sensors"] else ""),
         f"- 판정: **{'통과' if v['gate']['pass'] else '확인 필요'}**" + ("" if v["gate"]["pass"] else " — " + "; ".join(v["gate"]["failures"][:6])),
         ""]
    pf = s["preflight"]
    L.append("## 사전 점검")
    t = pf["total"]
    L.append(f"- 움직인 시간 {t['moving_s']:.0f} s, 누적 회전 {t['rotation_deg']:.0f}°, 회전(≥45°) {t['turns']}회, 열화상용 회전 구간 {t['turning_windows']}개")
    for w in pf["warnings"]:
        L.append(f"- 경고: {w.get('msg_ko') or w['msg']}")
    L.append("")
    L.append("## 카메라별 지표")
    L.append(metrics_md(m) + note)
    if s.get("rgb"):
        r = s["rgb"]
        L.append("## RGB")
        L.append(f"- 최종 풀이: 구간 {len(r['windows'])}개, 관측 {r['n_obs']:,}개, 재투영 중앙값 {r['reproj_median_px']:.3f} px, 시작 = {r['start']['kind']}")
        h = v.get("rgb", {}).get("halves", {}).get("summary")
        if h:
            L.append(f"- 절반 vs 절반: 위치 차 중앙값 {h['median_dpos_mm']:.1f} mm (최대 {h['max_dpos_mm']:.1f}), 광축 방향 {h['median_abs_daxis_mm']:.1f} mm, "
                     f"회전 {h['median_drot_deg']:.3f}°, 초점 {h['median_abs_df_px']:.2f} px")
        ag = r.get("bag_agreement")
        if ag and ag.get("bags"):
            L.append(f"- bag 간 일치: " + ", ".join(f"{b}: 벗어난 카메라 {x['cameras_outside']}대" for b, x in ag["bags"].items()) +
                     (f" → 불일치 {ag['disagreeing']}" if ag["disagreeing"] else ""))
        lc = v.get("rgb", {}).get("lidar_check", {})
        if lc:
            L.append(f"- 랜드마크-LiDAR 평면 거리 중앙값 {100 * lc['plane_abs_median_m']:.1f} cm; 깊이별 광선 오차 " +
                     ", ".join(f"{k} {100 * x['rel_depth_err_median']:+.2f}%" for k, x in lc.get("by_depth", {}).items()))
        L.append("")
    if s.get("thermal"):
        tt = s["thermal"]
        L.append("## 열화상")
        L.append(f"- 구간 {len(tt['windows'])}개, 기준선(좌-우) {tt.get('baseline_mm') or 0:.1f} mm, 재투영 중앙값 {tt['reproj_median_px']:.3f} px")
        L.append(f"- 시간 오프셋(bag별 평균, 가운데 행 기준): " + json.dumps(tt["dt_per_bag_s"]))
        h = v.get("thermal", {}).get("halves", {})
        if h:
            L.append(f"- 절반 vs 절반 기준선: {h.get('baseline_mm')}")
        L.append("")
    L.append("## 물리 배치 검사 (config/layout_rules.yaml)")
    for r in v.get("layout", {}).get("rules", []):
        kv = ", ".join(f"{k} {x}" for k, x in r.items() if k not in ("name", "type", "pass", "limits", "lo", "hi"))
        L.append(f"- {r['name']}: {'통과' if r['pass'] else '**실패**'} ({kv})")
    if v.get("layout", {}).get("skipped"):
        L.append(f"- 건너뜀(카메라 없음): {v['layout']['skipped']}")
    L.append("")
    L.append("## 단계별 시간·디스크")
    L.append("| 단계 | 시간 [min] | 디스크 [GB] |\n|---|---|---|")
    for st in s["stages"]:
        L.append(f"| {st['stage']} | {(st.get('wall_s') or 0) / 60:.1f} | {_fmt(st.get('disk_gb'), 2)} |")
    L.append("")
    L.append("## 파일")
    L.append("- `extrinsic/`, `intrinsic/`, `camera_info/`: 카메라별 YAML (팀 형식) — `nontarget_cal.zip`에 YAML만 묶음")
    L.append("- `rig.yaml`: 기준 카메라(front5) 광학 좌표계 기준 리그")
    L.append("- `images/parked/`, `images/driving/`: LiDAR 투영 (색 = 거리)")
    L.append("- `metrics.md`, `metrics.json`, `summary.json`(전체 수치)")
    (out / "report.md").write_text("\n".join(L) + "\n")
