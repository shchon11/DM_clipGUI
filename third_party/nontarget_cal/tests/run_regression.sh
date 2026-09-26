#!/bin/bash
# Regression on the 2026-09-24 night bag (SSD must be mounted).
#   tests/run_regression.sh reduced [ROOT]   4 windows (S01-S04, 160 s), every stage, RGB + thermal, zero-shot
#   tests/run_regression.sh full [ROOT]      the 25 windows of the validated work (S01-S10 + W00-W14, 894 s)
# Then compares with /hdd/DM_calib/lidar_odo/final and thermal_lo/thermal_lo_calib.yaml (tests/regression.py).
set -e
HERE=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-$HERE/.venv/bin/python}
BAG="/media/shchon11/Seonghyun/Data Machine/Calibration/rec_20260924_204902"
KIND=${1:-reduced}
ROOT=${2:-/hdd/DM_calib/nt_regress/$KIND}
S="S01:120:160,S02:180:220,S03:300:340,S04:380:420,S05:560:600,S06:660:700,S07:700:740,S08:760:800,S09:840:880,S10:955:995"
W="W00:0:18,W01:72:120,W02:160:180,W03:220:260,W04:260:300,W05:340:380,W06:420:460,W07:460:500,W08:500:530,W09:540:560,W10:600:623,W11:740:760,W12:800:840,W13:880:920,W14:920:955"
if [ "$KIND" = reduced ]; then WIN="S01:120:160,S02:180:220,S03:300:340,S04:380:420"; else WIN="$S,$W"; fi
mkdir -p "$ROOT"
"$PY" -m nontarget_cal run --bags "$BAG" --out "$ROOT/out" --workdir "$ROOT/work" --mode zeroshot \
    --sensors rgb,thermal --windows "$WIN" > "$ROOT/events.jsonl" 2> "$ROOT/stderr.log"
"$PY" "$HERE/tests/regression.py" --out "$ROOT/out" --work "$ROOT/work" --components \
    --json "$ROOT/regression.json" --md "$ROOT/regression.md"
