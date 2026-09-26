#!/bin/bash
# usage: run_eval.sh PREFIX MERGES   (same protocol as ../crosstime/run_all.sh)
PY=/hdd/DM_calib/nontarget_cal/.venv/bin/python
R=/hdd/DM_calib/nt_regress/crosstime/run_solve.py
W=/hdd/DM_calib/nt_regress/full/work
P=$1; MG=$2
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 NONTARGET_TIE_THREADS=2
SEGS=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows']))")
A=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows'][0::2]))")
B=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows'][1::2]))")
O='{"cross": "'$MG'", "free": ["rot","pos","f","k","pp"], "ties": true, "tie_sigma": 0.03, "pos_prior": 10.0, "f_prior": 0.05, "pp_prior": 30.0, "k_prior": 0.05, "huber": 1.5, "iters": 25, "min_len": 12, "min_parallax": 2.0, "max_dist": 60.0, "max_tracks": 2000, "step": 4, "init": "from"}'
H='{"free": ["rot","pos","f","k","pp"], "ties": false, "huber": 1.5, "iters": 25, "min_len": 12, "min_parallax": 2.0, "max_dist": 60.0, "max_tracks": 2000, "step": 4}'
S2='{"kind": "result", "path": "'$W'/rgb/zeroshot_pass2/result.json"}'
SF='{"kind": "result", "path": "'$W'/rgb/final/result.json"}'
cd /hdd/DM_calib/nt_regress/crosstime_learned
date
$PY $R ${P}final "$SEGS" "$S2" "$O" > ${P}final.out 2>&1 &
$PY $R ${P}halfA "$A" "$SF" "$O" > ${P}halfA.out 2>&1 &
$PY $R ${P}halfB "$B" "$SF" "$O" > ${P}halfB.out 2>&1 &
wait; date
$PY $R ${P}heldout_AonB "$B" "$SF" "$H" $W/rgb/${P}halfA/result.json > ${P}ho1.out 2>&1 &
$PY $R ${P}heldout_BonA "$A" "$SF" "$H" $W/rgb/${P}halfB/result.json > ${P}ho2.out 2>&1 &
wait; date; echo ALLDONE
