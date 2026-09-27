import sys, json, numpy as np
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal')
from pathlib import Path
from nontarget_cal.lo.lotraj import LOTraj
from nontarget_cal.rgb.solve import traj_rates
W0 = Path('/hdd/DM_calib/nt_regress/full/work')
ins = np.load(W0 / 'extract/b0_ins.npz'); ti = ins['imu_t'].astype(np.int64); wi = ins['imu_rate']
segs = json.load(open(W0 / 'rgb/summary.json'))['windows']
A, B = [], []
for s in segs:
    tr = LOTraj.load(W0 / f'lo/ref_{s}.npz'); t0, t1 = tr.span(); m = (ti > t0 + 5e8) & (ti < t1 - 5e8)
    Wl, _ = traj_rates(tr, ti[m], h_ns=5e7)
    k = np.ones(11) / 11; Ws = np.stack([np.convolve(wi[m][:, j], k, 'same') for j in range(3)], 1)
    A.append(Ws[20:-20]); B.append(Wl[20:-20])
A, B = np.concatenate(A), np.concatenate(B)
print('rms LO rate per LiDAR axis deg/s', np.degrees(np.sqrt((B**2).mean(0))).round(2), ' IMU rms per IMU axis', np.degrees(np.sqrt((A**2).mean(0))).round(2))
print('corr matrix LO(rows) vs IMU(cols):'); print(np.round(np.corrcoef(B.T, A.T)[:3, 3:], 3))
