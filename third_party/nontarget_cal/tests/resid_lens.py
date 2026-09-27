"""Lens-model adequacy: per camera, mean residual vector in 12x8 image cells (a systematic field = model error)
vs the noise of the mean; residual vs radius per camera. usage: diag_lens.py SOLVE_DIR"""
import sys, json, numpy as np, torch
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal')
from pathlib import Path
from nontarget_cal.rgb.rigba import kb_project
from nontarget_cal.lo.lotraj import LOTraj
d = Path(sys.argv[1]); res = json.load(open(d / 'result.json')); z = np.load(d / 'state.npz')
LO = Path('/hdd/DM_calib/nt_regress/full/work/lo'); cams = res['cams']; segs = res['segs']
X = z['X']; lm_seg = z['lm_seg']; oc = z['obs_cam']; ol = z['obs_lm']; uv = z['obs_uv']; ot = z['obs_t'].astype(np.float64)
dt = np.array([res['cameras'][c].get('time_dt_s', 0.0) for c in cams])
Tcl = np.array([res['cameras'][c]['T_cam_lidar'] for c in cams]); intr = np.array([res['cameras'][c]['intr_kb'] for c in cams])
R = np.zeros((len(oc), 2)); seg_o = lm_seg[ol]
for si, s in enumerate(segs):
    idx = np.flatnonzero(seg_o == si); tr = LOTraj.load(LO / f'ref_{s}.npz'); t = ot[idx] + 1e9 * dt[oc[idx]]
    XL = np.einsum('nji,nj->ni', tr.R(t), X[ol[idx]] - tr.p(t))
    Xc = np.einsum('nij,nj->ni', Tcl[oc[idx], :3, :3], XL) + Tcl[oc[idx], :3, 3]
    I = intr[oc[idx]]; T = lambda a: torch.as_tensor(a, dtype=torch.float64)
    R[idx] = kb_project(T(Xc), *[T(I[:, k]) for k in range(7)], jac=False).numpy() - uv[idx]
e = np.linalg.norm(R, axis=1); ok = e < 3
print(f'{d.name}: cell field (12x8 cells, |r|<3): per camera  median |cell mean| px, max |cell mean| px, median noise-of-mean px, '
      'share of residual variance explained by the cell means;  med|r| by radius 0-300/300-600/600-900/>900 px')
tot_ex = []
for i, c in enumerate(cams):
    m = (oc == i) & ok
    cx = np.clip((uv[m, 0] / 1920 * 12).astype(int), 0, 11); cy = np.clip((uv[m, 1] / 1200 * 8).astype(int), 0, 7)
    k = cy * 12 + cx; n = np.bincount(k, minlength=96)
    mu = np.stack([np.bincount(k, R[m, j], 96) for j in range(2)], 1) / np.maximum(n, 1)[:, None]
    var = np.stack([np.bincount(k, R[m, j] ** 2, 96) for j in range(2)], 1) / np.maximum(n, 1)[:, None] - mu ** 2
    good = n > 200
    mag = np.linalg.norm(mu[good], axis=1); noise = np.sqrt(var[good].sum(1) / n[good])
    ex = (n[good] * (mu[good] ** 2).sum(1)).sum() / (R[m] ** 2).sum()
    tot_ex.append(ex)
    rad = np.hypot(uv[oc == i, 0] - intr[i, 2], uv[oc == i, 1] - intr[i, 3]); ei = e[oc == i]
    rb = ' / '.join(f'{np.median(ei[(rad >= a) & (rad < b)]):.2f}' if ((rad >= a) & (rad < b)).sum() > 200 else '  - ' for a, b in [(0, 300), (300, 600), (600, 900), (900, 3000)])
    print(f'  {c:18s} {np.median(mag):.3f} {mag.max():.3f} {np.median(noise):.3f}  {100 * ex:4.1f}%  | {rb}')
print(f'  median share of variance explained by a per-cell (lens/position) field: {100 * np.median(tot_ex):.1f}%')
np.save('/hdd/DM_calib/nt_regress/dt_study/' + d.name + '_resid.npy', R.astype(np.float32))
