#!/bin/bash
# usage: quick.sh NAME EXTRA_JSON_FRAGMENT   (2 windows S03 W05, start = base final)
PY=/hdd/DM_calib/nontarget_cal/.venv/bin/python
export OMP_NUM_THREADS=${NTH:-3} OPENBLAS_NUM_THREADS=${NTH:-3} NONTARGET_TIE_THREADS=2
W=${DTW:-/hdd/DM_calib/nt_regress/resid_study/work}
SEGS=${SEGS:-'["S03", "W05"]'}
O='{'$2'"free": ["rot", "pos", "f", "k", "pp"], "ties": true, "tie_sigma": 0.03, "pos_prior": 10.0, "f_prior": 0.05, "pp_prior": 30.0, "k_prior": 0.05, "huber": 1.5, "iters": 25, "min_len": 12, "min_parallax": 2.0, "max_dist": 60.0, "max_tracks": 3000, "step": 3, "init": "from"}'
S='{"kind": "result", "path": "/hdd/DM_calib/nt_regress/full/work/rgb/final/result.json"}'
cd /hdd/DM_calib/nt_regress/resid_study
/usr/bin/time -v $PY /hdd/DM_calib/nontarget_cal_resid/tests/resid_run_solve.py q_$1 "$SEGS" "$S" "$O" > q_$1.out 2>&1
