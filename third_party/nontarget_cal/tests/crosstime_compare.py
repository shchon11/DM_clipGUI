"""Half-split 1-sigma comparison: python compare.py prefix1 prefix2 ...  ('' = no links, 'cross_', 'strict_')"""
import json, sys
import numpy as np
from pathlib import Path
R = Path('/hdd/DM_calib/nt_regress/full/work/rgb')
def load(p): return json.load(open(R / p / 'result.json'))
def stats(pre):
    A, B = load(pre + 'halfA'), load(pre + 'halfB')
    F = load(pre + 'final')
    h1, h2 = load(pre + 'heldout_AonB'), load(pre + 'heldout_BonA')
    out = {}
    for c in A['cams']:
        Ta, Tb = np.array(A['cameras'][c]['T_lidar_cam']), np.array(B['cameras'][c]['T_lidar_cam'])
        dR = Ta[:3, :3].T @ Tb[:3, :3]
        rot = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
        dp = Tb[:3, 3] - Ta[:3, 3]
        ax = abs(dp @ (Ta[:3, 2] + Tb[:3, 2]) / 2)
        f = abs(A['cameras'][c]['intr_kb'][0] - B['cameras'][c]['intr_kb'][0])
        ho = (h1['cameras'][c]['reproj_median_px'] + h2['cameras'][c]['reproj_median_px']) / 2
        s = np.sqrt(2)
        out[c] = dict(rot=rot / s, pos=np.linalg.norm(dp) * 1e3 / s, axis=ax * 1e3 / s, f=f / s, ho=ho,
                      trk=F['cameras'][c]['reproj_median_px'])
    return out, F
pres = sys.argv[1:]
S = [stats(p) for p in pres]
cams = list(S[0][0])
print('camera             | rot1s deg | pos1s mm | axis1s mm | f1s px | heldout px | track px   (' + ' / '.join(p or 'none' for p in pres) + ')')
for c in cams:
    row = [ '/'.join(f'{s[0][c][k]:{fm}}' for s in S) for k, fm in [('rot','.3f'),('pos','.1f'),('axis','.1f'),('f','.2f'),('ho','.3f'),('trk','.2f')]]
    print(f'{c:18s} | ' + ' | '.join(row))
groups = {'front+top': [c for c in cams if 'front' in c or 'top' in c], 'side': [c for c in cams if 'side' in c], 'rear': [c for c in cams if 'rear' in c]}
for g, cs in groups.items():
    print(g, {k: '/'.join(str(round(float(np.median([s[0][c][k] for c in cs])), 3)) for s in S) for k in ('rot', 'pos', 'axis', 'f')})
for p, s in zip(pres, S):
    print(f"{p or 'none'} final reproj {s[1]['reproj_median_px']:.3f} n_obs {s[1]['n_obs']} wall {s[1]['wall_s']:.0f}")
