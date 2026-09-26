import numpy as np, glob, json, sys
from pathlib import Path
import os
D = Path(os.environ.get('BAKEOFF_OUT', '/hdd/DM_calib/nt_regress/crosstime_learned/bakeoff'))
tol = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
mats = ['ncc_only', 'sift_crops', 'superpoint_lightglue', 'aliked_lightglue', 'disk_lightglue', 'xfeat_lighterglue', 'xfeat_mnn']
print(f'tol {tol} px | pairs | matches med | RANSAC inl med | epi<=2px ratio | verified all / tight / FS / rear | median err tight | false-acc shift2 / shift4 | ms/pair extract+match | GPU MB')
for m in mats:
    fs = sorted(glob.glob(str(D / f'res_*_{m}.npz')))
    if not fs: continue
    A = {}
    for f in fs:
        win = Path(f).name.split('_')[1]
        P = np.load(D / f'pairs_{win}.npz'); R = np.load(f)
        for k in ('grp',): A.setdefault(k, []).append(P[k])
        A.setdefault('tight', []).append((P['med'].max(1) <= 1) & (P['p90'].max(1) <= 2))
        for k in R.files:
            if R[k].ndim: A.setdefault(k, []).append(R[k])
        A.setdefault('gpu', []).append(float(R['gpu_mb'])); A.setdefault('t', []).append((float(R['0_t_extract']) + float(R['0_t_match'])) / int(R['n']))
    A = {k: (np.concatenate(v) if k not in ('gpu', 't') else np.array(v)) for k, v in A.items()}
    rule = lambda p: ((A[p + '_err_px'] <= tol) | (m == 'ncc_only')) & (A[p + '_ncc'] >= 0.8) & (A[p + '_ncc_px'] <= 1.0) if (p + '_ncc') in A else A[p + '_err_px'] <= tol
    v = rule('0')
    tight = A['tight']; rear = np.char.find(A['grp'].astype(str), 'R') >= 0; fsm = A['grp'] == 'FS'
    epi = A['0_epi_ok2'].sum() / max(A['0_epi_n'].sum(), 1)
    print(f"{m:21s} | {len(v)} | {np.median(A['0_n_match']):.0f} | {np.median(A['0_n_inl']):.0f} | {epi:.2f} | "
          f"{v.mean():.2f} / {v[tight].mean():.2f} / {v[fsm].mean():.2f} / {v[rear].mean():.2f} ({rear.sum()}) | "
          f"{np.median(A['0_err_px'][tight & np.isfinite(A['0_err_px'])]):.2f} | {rule('s2').mean():.3f} / {rule('s4').mean():.3f} | "
          f"{1e3 * np.mean(A['t']):.1f} | {max(A['gpu']):.0f}")
