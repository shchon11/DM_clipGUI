#!/bin/bash
# usage: run_eval.sh PREFIX EXTRA_FRAGMENT [HELD_EXTRA_FRAGMENT]   (env DTW = work dir, default resid_study/work)
# Run inside the shared memory-capped slice: systemd-run --user --slice=calib.slice --scope -q bash resid_run_eval.sh ...
# (final ~9 GB peak RSS, then two half solves / two held-out solves in parallel).
# Protocol of tests/time_run_eval.sh: final (25 windows from zeroshot_pass2), halves A/B from this final,
# held-out = each half's calibration held on the other half. Fragments are '"key": value, ' JSON pieces.
PY=/hdd/DM_calib/nontarget_cal/.venv/bin/python
R=/hdd/DM_calib/nontarget_cal_resid/tests/resid_run_solve.py
export DTW=${DTW:-/hdd/DM_calib/nt_regress/resid_study/work}
W=$DTW
P=$1; EX=$2; HX=${3:-}
export OMP_NUM_THREADS=${NTH:-4} OPENBLAS_NUM_THREADS=${NTH:-4} NONTARGET_TIE_THREADS=2
SEGS=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows']))")
A=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows'][0::2]))")
B=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows'][1::2]))")
FR='["rot", "pos", "f", "k", "pp"]'
O='{'$EX'"free": '$FR', "ties": true, "tie_sigma": 0.03, "pos_prior": 10.0, "f_prior": 0.05, "pp_prior": 30.0, "k_prior": 0.05, "huber": 1.5, "iters": 25, "min_len": 12, "min_parallax": 2.0, "max_dist": 60.0, "max_tracks": 2000, "step": 4, "init": "from"}'
H='{'$HX'"free": '$FR', "ties": false, "huber": 1.5, "iters": 25, "min_len": 12, "min_parallax": 2.0, "max_dist": 60.0, "max_tracks": 2000, "step": 4}'
S2='{"kind": "result", "path": "/hdd/DM_calib/nt_regress/full/work/rgb/zeroshot_pass2/result.json"}'
SF='{"kind": "result", "path": "'$W'/rgb/'${P}'final/result.json"}'
cd /hdd/DM_calib/nt_regress/resid_study
date
[ -z "$SKIPFINAL" ] && /usr/bin/time -v $PY $R ${P}final "$SEGS" "$S2" "$O" > ${P}final.out 2>&1
date
$PY $R ${P}halfA "$A" "$SF" "$O" > ${P}halfA.out 2>&1 &
$PY $R ${P}halfB "$B" "$SF" "$O" > ${P}halfB.out 2>&1 &
wait; date
$PY $R ${P}heldout_AonB "$B" "$SF" "$H" $W/rgb/${P}halfA/result.json > ${P}ho1.out 2>&1 &
$PY $R ${P}heldout_BonA "$A" "$SF" "$H" $W/rgb/${P}halfB/result.json > ${P}ho2.out 2>&1 &
wait; date; echo ALLDONE
