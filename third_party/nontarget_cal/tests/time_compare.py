"""Half-split comparison for the dt study: python compare.py base_ dt_ dtrs_ ...
1-sigma = |A-B|/sqrt(2); held-out = mean of the two cross-half medians; plane = |landmark-LiDAR plane| median
of the final; shift = final vs base_final in units of the base empirical 1-sigma (floors 5 mm / 0.02 deg)."""
import json, sys
import numpy as np
from pathlib import Path
R = Path('/hdd/DM_calib/nt_regress/dt_study/work/rgb')
def load(p): return json.load(open(R / p / 'result.json'))
def ang(Ta, Tb):
    dR = Ta[:3, :3].T @ Tb[:3, :3]
    return np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
def stats(pre):
    A, B, F = load(pre + 'halfA'), load(pre + 'halfB'), load(pre + 'final')
    h1, h2 = load(pre + 'heldout_AonB'), load(pre + 'heldout_BonA')
    out = {}
    for c in A['cams']:
        Ta, Tb = np.array(A['cameras'][c]['T_lidar_cam']), np.array(B['cameras'][c]['T_lidar_cam'])
        dp = Tb[:3, 3] - Ta[:3, 3]
        ax = abs(dp @ (Ta[:3, 2] + Tb[:3, 2]) / 2)
        s = np.sqrt(2)
        out[c] = dict(rot=ang(Ta, Tb) / s, pos=np.linalg.norm(dp) * 1e3 / s, axis=ax * 1e3 / s,
                      f=abs(A['cameras'][c]['intr_kb'][0] - B['cameras'][c]['intr_kb'][0]) / s,
                      ho=(h1['cameras'][c]['reproj_median_px'] + h2['cameras'][c]['reproj_median_px']) / 2,
                      trk=F['cameras'][c]['reproj_median_px'],
                      dtF=1e3 * F['cameras'][c].get('time_dt_s', 0), dtA=1e3 * A['cameras'][c].get('time_dt_s', 0),
                      dtB=1e3 * B['cameras'][c].get('time_dt_s', 0), rsF=1e3 * F['cameras'][c].get('time_rs_s', 0),
                      rsA=1e3 * A['cameras'][c].get('time_rs_s', 0), rsB=1e3 * B['cameras'][c].get('time_rs_s', 0),
                      sd_dt=1e3 * (F['cameras'][c]['sd'].get('dt') or np.nan),
                      plane=F['lidar_check']['per_cam'].get(c, {}).get('abs_plane_median_m', np.nan) * 1e3)
    return out, F, (h1, h2)
pres = sys.argv[1:]
S = [stats(p) for p in pres]
cams = list(S[0][0])
print('per camera: rot1s deg | pos1s mm | axis1s mm | f1s px | heldout px | track px | plane mm   (' + ' / '.join(pres) + ')')
for c in cams:
    row = ['/'.join(f'{s[0][c][k]:{fm}}' for s in S) for k, fm in [('rot', '.3f'), ('pos', '.1f'), ('axis', '.1f'), ('f', '.2f'), ('ho', '.3f'), ('trk', '.3f'), ('plane', '.1f')]]
    print(f'{c:18s} | ' + ' | '.join(row))
for p, s in zip(pres, S):
    if any(abs(s[0][c]['dtF']) > 0 for c in cams):
        print(f'\n{p} time offsets [ms]: final (formal sd) | half A | half B | A-B   (rs final/A/B if solved)')
        for c in cams:
            q = s[0][c]
            rs = f"   rs {q['rsF']:+.2f}/{q['rsA']:+.2f}/{q['rsB']:+.2f}" if abs(q['rsF']) > 0 else ''
            print(f"  {c:18s} {q['dtF']:+6.2f} ({q['sd_dt']:.2f}) | {q['dtA']:+6.2f} | {q['dtB']:+6.2f} | {q['dtA'] - q['dtB']:+5.2f}{rs}")
        v = np.array([s[0][c]['dtF'] for c in cams]); d = np.array([s[0][c]['dtA'] - s[0][c]['dtB'] for c in cams])
        print(f'  mean {v.mean():+.2f} ms, spread (sd over cameras) {v.std():.2f} ms, |A-B| median {np.median(np.abs(d)):.2f} max {np.abs(d).max():.2f} ms')
groups = {'front+top': [c for c in cams if 'front' in c or 'top' in c], 'side': [c for c in cams if 'side' in c],
          'rear': [c for c in cams if 'rear' in c]}
print()
for g, cs in groups.items():
    print(f'{g:10s}', {k: '/'.join(str(round(float(np.median([s[0][c][k] for c in cs])), 3)) for s in S) for k in ('rot', 'pos', 'axis', 'f', 'ho', 'trk')})
base = S[0]
for p, s in zip(pres, S):
    F = s[1]
    ho = np.mean([h['reproj_median_px'] for h in s[2]])
    print(f"{p:8s} final track reproj {F['reproj_median_px']:.3f} px (rms<5 {F['reproj_rms_px']:.3f}) | held-out {ho:.3f} | "
          f"|plane| median {1e3 * F['lidar_check']['plane_abs_median_m']:.1f} mm | n_obs {F['n_obs']} | wall {F['wall_s']:.0f} s")
    worst = []
    ref = base[1]
    for c in cams:
        T0, T1 = np.array(ref['cameras'][c]['T_lidar_cam']), np.array(F['cameras'][c]['T_lidar_cam'])
        s_pos = max(base[0][c]['pos'] * 1e-3, 5e-3); s_rot = max(base[0][c]['rot'], 0.02)
        pos, rot = np.linalg.norm(T1[:3, 3] - T0[:3, 3]), ang(T0, T1)
        worst.append((max(pos / s_pos, rot / s_rot), c, pos * 1e3, rot))
    worst.sort(reverse=True)
    print('         shift vs base final: max %.2f sigma (%s: %.1f mm %.3f deg)' % worst[0], '| >3 sigma:', [w[1] for w in worst if w[0] > 3],
          '| median %.1f mm' % np.median([w[2] for w in worst]))
