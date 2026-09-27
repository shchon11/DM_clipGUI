"""Half-split comparison for the residual study (tests/resid_run_eval.sh runs):
    python resid_compare.py [--bins] ROOT:PREFIX [ROOT:PREFIX ...]        (first = reference, normally the validated base)
ROOT = a work dir's rgb/ folder (default /hdd/DM_calib/nt_regress/resid_study/work/rgb), PREFIX e.g. base_, fc6_.
1-sigma = |A-B|/sqrt(2) per camera, group medians (front+top / side / rear); held-out = mean of the two cross-half
medians (and '<prefix>ho_base' = the halves' calibration held on the BASE tracks, if present); plane = |landmark -
LiDAR plane| median of the final; shift = final vs the reference final in units of the reference empirical 1-sigma
(floors 5 mm / 0.02 deg), > 3 sigma flagged. --bins: residual of the final vs vehicle speed and vs |t - track centre|
(state.npz obs_r if saved, else re-projected with the LO trajectory)."""
import json, sys
import numpy as np
from pathlib import Path

DEF = Path('/hdd/DM_calib/nt_regress/resid_study/work/rgb')
LO = Path('/hdd/DM_calib/nt_regress/full/work/lo')


def load(root, p):
    f = root / p / 'result.json'
    return json.load(open(f)) if f.exists() else None


def ang(Ta, Tb):
    dR = Ta[:3, :3].T @ Tb[:3, :3]
    return np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))


def stats(root, pre):
    A, B, F = load(root, pre + 'halfA'), load(root, pre + 'halfB'), load(root, pre + 'final')
    h1, h2 = load(root, pre + 'heldout_AonB'), load(root, pre + 'heldout_BonA')
    g1, g2 = load(DEF, pre + 'hobase_AonB'), load(DEF, pre + 'hobase_BonA')
    out = {}
    for c in F['cams']:
        d = dict(trk=F['cameras'][c]['reproj_median_px'],
                 plane=F['lidar_check']['per_cam'].get(c, {}).get('abs_plane_median_m', np.nan) * 1e3)
        if A and B:
            Ta, Tb = np.array(A['cameras'][c]['T_lidar_cam']), np.array(B['cameras'][c]['T_lidar_cam'])
            dp = Tb[:3, 3] - Ta[:3, 3]
            s = np.sqrt(2)
            d.update(rot=ang(Ta, Tb) / s, pos=np.linalg.norm(dp) * 1e3 / s,
                     axis=abs(dp @ (Ta[:3, 2] + Tb[:3, 2]) / 2) * 1e3 / s,
                     f=abs(A['cameras'][c]['intr_kb'][0] - B['cameras'][c]['intr_kb'][0]) / s)
        if h1 and h2:
            d['ho'] = (h1['cameras'][c]['reproj_median_px'] + h2['cameras'][c]['reproj_median_px']) / 2
        if g1 and g2:
            d['hob'] = (g1['cameras'][c]['reproj_median_px'] + g2['cameras'][c]['reproj_median_px']) / 2
        out[c] = d
    return out, F, [h for h in (h1, h2) if h], [g for g in (g1, g2) if g]


def bins(root, pre):
    import torch
    sys.path.insert(0, '/hdd/DM_calib/nontarget_cal_resid')
    from nontarget_cal.lo.lotraj import LOTraj
    from nontarget_cal.rgb.rigba import kb_project
    d = root / (pre + 'final')
    res = json.load(open(d / 'result.json'))
    z = np.load(d / 'state.npz')
    cams, segs = res['cams'], res['segs']
    X, lm_seg, oc, ol, uv, ot = z['X'], z['lm_seg'], z['obs_cam'], z['obs_lm'], z['obs_uv'], z['obs_t'].astype(np.float64)
    seg_o = lm_seg[ol]
    N = len(oc)
    R = z['obs_r'].astype(np.float64) if 'obs_r' in z.files else None
    Tcl = np.array([res['cameras'][c]['T_cam_lidar'] for c in cams])
    intr = np.array([res['cameras'][c]['intr_kb'] for c in cams])
    SPD = np.zeros(N)
    if R is None:
        R = np.zeros((N, 2))
    for si, s in enumerate(segs):
        idx = np.flatnonzero(seg_o == si)
        if not len(idx):
            continue
        tr = LOTraj.load(LO / f'ref_{s}.npz')
        t = ot[idx]
        SPD[idx] = np.linalg.norm(tr.p(t + 5e7) - tr.p(t - 5e7), axis=1) / 0.1
        if 'obs_r' not in z.files:
            Rw, pw = tr.R(t), tr.p(t)
            XL = np.einsum('nji,nj->ni', Rw, X[ol[idx]] - pw)
            Xc = np.einsum('nij,nj->ni', Tcl[oc[idx], :3, :3], XL) + Tcl[oc[idx], :3, 3]
            I = intr[oc[idx]]
            tt = lambda a: torch.as_tensor(a, dtype=torch.float64)
            R[idx] = kb_project(tt(Xc), *[tt(I[:, k]) for k in range(7)], jac=False).numpy() - uv[idx]
    e = np.linalg.norm(R, axis=1)
    M = len(X)
    grp = np.arange(M)
    if res.get('seg_span_s') and 'lm_key' in z.files:
        # pieces of one KLT track (id * 64 + piece): measure |t - centre| from the WHOLE track's centre so the
        # bins are comparable with an unsplit solve
        kk = [k.rsplit(',', 1) for k in z['lm_key']]
        _, grp = np.unique([f'{a},{int(b.strip(" )")) // 64}' for a, b in kk], return_inverse=True)
    G = grp.max() + 1
    tmid = np.bincount(grp[ol], weights=ot, minlength=G) / np.maximum(np.bincount(grp[ol], minlength=G), 1)
    age = np.abs(ot - tmid[grp[ol]]) * 1e-9
    rear = np.isin(oc, [i for i, c in enumerate(cams) if 'rear' in c])
    sb = [(0, 3), (3, 6), (6, 9), (9, 12), (12, 1e9)]
    ab = [(0, .2), (.2, .5), (.5, 1), (1, 2), (2, 1e9)]
    f = lambda m: f'{np.median(e[m]):.3f}' if m.sum() > 100 else '  -  '
    return (' speed m/s ' + ' '.join(f'{a:g}-{b if b < 1e8 else "":<3}:{f((SPD >= a) & (SPD < b))}' for a, b in sb) +
            '\n     |t-centre| s ' + ' '.join(f'{a:g}-{b if b < 1e8 else "":<3}:{f((age >= a) & (age < b))}' for a, b in ab) +
            '  (rear only: ' + ' '.join(f'{f(rear & (age >= a) & (age < b))}' for a, b in ab) + ')')


args = [a for a in sys.argv[1:] if not a.startswith('--')]
specs = []
for a in args:
    r, p = a.split(':') if ':' in a else (str(DEF), a)
    specs.append((Path(r), p))
S = [stats(r, p) for r, p in specs]
names = [p for _, p in specs]
cams = list(S[0][0])
keys = [('rot', '.3f'), ('pos', '.1f'), ('axis', '.1f'), ('f', '.2f'), ('ho', '.3f'), ('hob', '.3f'), ('trk', '.3f'), ('plane', '.1f')]
print('per camera: rot1s deg | pos1s mm | axis1s mm | f1s px | heldout px | heldout on base tracks | track px | plane mm   ('
      + ' / '.join(names) + ')')
for c in cams:
    row = ['/'.join(f'{s[0][c][k]:{fm}}' if k in s[0][c] else '-' for s in S) for k, fm in keys]
    print(f'{c:18s} | ' + ' | '.join(row))
groups = {'front+top': [c for c in cams if 'front' in c or 'top' in c], 'side': [c for c in cams if 'side' in c],
          'rear': [c for c in cams if 'rear' in c]}
print()
for g, cs in groups.items():
    print(f'{g:10s}', {k: '/'.join(str(round(float(np.median([s[0][c][k] for c in cs])), 3)) if k in s[0][cs[0]] else '-'
                                  for s in S) for k, _ in keys})
base = S[0]
for (root, p), s in zip(specs, S):
    F = s[1]
    ho = np.mean([h['reproj_median_px'] for h in s[2]]) if s[2] else np.nan
    hob = np.mean([h['reproj_median_px'] for h in s[3]]) if s[3] else np.nan
    print(f"{p:10s} final track reproj {F['reproj_median_px']:.3f} px (rms<5 {F['reproj_rms_px']:.3f}) | held-out {ho:.3f} "
          f"| held-out on base tracks {hob:.3f} | |plane| median {1e3 * F['lidar_check']['plane_abs_median_m']:.1f} mm "
          f"| n_obs {F['n_obs']} n_lm {F['n_lm']} | wall {F['wall_s']:.0f} s")
    worst = []
    ref = base[1]
    for c in cams:
        T0, T1 = np.array(ref['cameras'][c]['T_lidar_cam']), np.array(F['cameras'][c]['T_lidar_cam'])
        s_pos = max(base[0][c]['pos'] * 1e-3, 5e-3)
        s_rot = max(base[0][c]['rot'], 0.02)
        pos, rot = np.linalg.norm(T1[:3, 3] - T0[:3, 3]), ang(T0, T1)
        worst.append((max(pos / s_pos, rot / s_rot), c, pos * 1e3, rot))
    worst.sort(reverse=True)
    print('           shift vs reference final: max %.2f sigma (%s: %.1f mm %.3f deg)' % worst[0], '| >3 sigma:',
          [w[1] for w in worst if w[0] > 3], '| median %.1f mm' % np.median([w[2] for w in worst]))
    if '--bins' in sys.argv:
        print('    ' + bins(root, p))
