#!/bin/bash
# usage: run_eval.sh PREFIX FREE_JSON [MERGES]   same protocol as tests/crosstime_run_eval.sh, in dt_study/work
PY=/hdd/DM_calib/nontarget_cal/.venv/bin/python
R=/hdd/DM_calib/nt_regress/dt_study/run_solve.py
W=/hdd/DM_calib/nt_regress/dt_study/work
P=$1; FR=$2; MG=${3:-}
export OMP_NUM_THREADS=${NTH:-4} OPENBLAS_NUM_THREADS=${NTH:-4} NONTARGET_TIE_THREADS=2
SEGS=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows']))")
A=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows'][0::2]))")
B=$(python3 -c "import json;print(json.dumps(json.load(open('$W/rgb/summary.json'))['windows'][1::2]))")
CR=""; [ -n "$MG" ] && CR='"cross": "'$MG'", '
O='{'$CR'"free": '$FR', "ties": true, "tie_sigma": 0.03, "pos_prior": 10.0, "f_prior": 0.05, "pp_prior": 30.0, "k_prior": 0.05, "huber": 1.5, "iters": 25, "min_len": 12, "min_parallax": 2.0, "max_dist": 60.0, "max_tracks": 2000, "step": 4, "init": "from"}'
OH='{"free": '$FR', "ties": true, "tie_sigma": 0.03, "pos_prior": 10.0, "f_prior": 0.05, "pp_prior": 30.0, "k_prior": 0.05, "huber": 1.5, "iters": 25, "min_len": 12, "min_parallax": 2.0, "max_dist": 60.0, "max_tracks": 2000, "step": 4, "init": "from"}'
H='{"free": '$FR', "ties": false, "huber": 1.5, "iters": 25, "min_len": 12, "min_parallax": 2.0, "max_dist": 60.0, "max_tracks": 2000, "step": 4}'
S2='{"kind": "result", "path": "'${S2PATH:-$W/rgb/base_zeroshot_pass2/result.json}'"}'
SF='{"kind": "result", "path": "'$W'/rgb/'${P}'final/result.json"}'
cd /hdd/DM_calib/nt_regress/dt_study
date
[ -z "$SKIPFINAL" ] && /usr/bin/time -v $PY $R ${P}final "$SEGS" "$S2" "$O" > ${P}final.out 2>&1
date
# halves: the pipeline protocol (start = this variant's final; halves never use links, as validation does)
[ -n "$HALFCROSS" ] && OH="$O"
$PY $R ${P}halfA "$A" "$SF" "$OH" > ${P}halfA.out 2>&1 &
$PY $R ${P}halfB "$B" "$SF" "$OH" > ${P}halfB.out 2>&1 &
wait; date
$PY $R ${P}heldout_AonB "$B" "$SF" "$H" $W/rgb/${P}halfA/result.json > ${P}ho1.out 2>&1 &
$PY $R ${P}heldout_BonA "$A" "$SF" "$H" $W/rgb/${P}halfB/result.json > ${P}ho2.out 2>&1 &
wait; date; echo ALLDONE
