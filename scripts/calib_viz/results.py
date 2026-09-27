"""Per-camera decisions from solver results, independent of Qt and the solver.

A failed informational RGB vote is not a failed calibration. The solver's
recorded camera decisions/failures take precedence over legacy threshold
reconstruction. Layout failures belong only to the overall result.
"""
import math


DEFAULT_GATES = {"rgb_rot_deg": 0.5, "rgb_axis_mm": 60.0, "thermal_axis_mm": 60.0}


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def camera_gate(summary, metrics, name, defaults=None):
    """Return ``pass`` (bool/None), ``reasons``, ``informational`` and ``source``.

    ``metrics`` is the complete name-to-metrics mapping. ``defaults`` optionally
    supplies legacy gate thresholds (or a mapping containing ``gate``), useful
    for deliberately reviewing a legacy result under different thresholds.
    Explicit per-camera solver decisions always win. Unknown is never a pass.
    """
    summary, metrics = summary or {}, metrics or {}
    metric = metrics.get(name) or {}
    validation = summary.get("validation") or {}
    gate = validation.get("gate") or {}
    sensor = metric.get("sensor", "thermal" if name.startswith("thermal") else "rgb")
    vote = metric.get("vote") or {}
    informational = []
    if vote and (vote.get("gate") is False or sensor == "rgb"):
        informational.append("RGB 에지/투표 참고용 · 야간 판별력 약함" if sensor == "rgb"
                             else "에지/투표 참고용 · 합격 판정에 미포함")

    def result(passed, reasons, source):
        return {"pass": passed, "reasons": reasons, "informational": informational, "source": source}

    # Newer producers may publish the decision directly; do not second-guess it
    # using local defaults or a diagnostic vote.
    for records in (gate.get("cameras"), gate.get("per_camera")):
        entry = (records or {}).get(name)
        if isinstance(entry, bool):
            return result(entry, [] if entry else ["카메라 게이트 실패"], "tool_camera_gate")
        if isinstance(entry, dict) and isinstance(entry.get("pass"), bool):
            reasons = entry.get("reasons") or entry.get("failures") or []
            return result(entry["pass"], list(reasons), "tool_camera_gate")
    explicit = metric.get("gate")
    if isinstance(explicit, dict) and isinstance(explicit.get("pass"), bool):
        return result(explicit["pass"], list(explicit.get("reasons") or explicit.get("failures") or []),
                      "tool_camera_gate")
    if isinstance(metric.get("gate_pass"), bool):
        return result(metric["gate_pass"], list(metric.get("gate_reasons") or []), "tool_camera_gate")

    sigma = ((validation.get(sensor) or {}).get("halves") or {}).get("sigma") or {}
    repeatability = sigma.get(name) or metric
    rot, axis = repeatability.get("rot_deg"), repeatability.get("along_axis_mm")
    has_repeatability = _finite(axis) and (sensor != "rgb" or _finite(rot))
    # nontarget_cal 1.0 records an exhaustive failure list, with each camera
    # failure prefixed by its exact name. A layout rule is not a camera verdict.
    failures = gate.get("failures")
    if defaults is None and isinstance(gate.get("pass"), bool) and isinstance(failures, list):
        own = [failure for failure in failures if isinstance(failure, str)
               and failure.startswith(name + ":")]
        if own:
            return result(False, own, "tool_gate_failures")
        if has_repeatability:
            return result(True, [], "tool_gate_failures")
        return result(None, ["카메라 게이트 검증 자료 없음"], "unknown")

    # Legacy summaries without an explicit gate use the same thresholds and
    # checks as pipeline._gate, not reprojection or RGB edge/vote diagnostics.
    limits = dict(DEFAULT_GATES)
    config = ((summary.get("config") or {}).get("validation") or {}).get("gate") or {}
    limits.update({key: config[key] for key in limits if _finite(config.get(key))})
    if defaults is not None:
        overrides = defaults.get("gate", defaults)
        limits.update({key: overrides[key] for key in limits if _finite(overrides.get(key))})
    reasons = []
    if sensor == "rgb" and _finite(rot) and float(rot) > float(limits["rgb_rot_deg"]):
        reasons.append(f"회전 1σ {float(rot):.2f}° > {float(limits['rgb_rot_deg']):g}")
    limit = float(limits["thermal_axis_mm" if sensor == "thermal" else "rgb_axis_mm"])
    if _finite(axis) and float(axis) > limit:
        reasons.append(f"광축 1σ {float(axis):.0f} mm > {limit:.0f}")
    if sensor == "thermal" and vote.get("gate", True) and vote.get("pass") is False:
        reasons.append("투표 게이트 실패")
    if reasons:
        return result(False, reasons, "legacy_thresholds")
    if has_repeatability:
        return result(True, [], "legacy_thresholds")
    return result(None, ["반복성(절반 풀이) 검증 자료 없음"], "unknown")
