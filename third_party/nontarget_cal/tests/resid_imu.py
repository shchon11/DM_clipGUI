"""Is the per-frame pose correction (diag_pose.py) real high-frequency motion that the 10 Hz LO trajectory misses?
IMU gyro (100 Hz, extract/b0_ins.npz imu_rate) -> LiDAR axes (Kabsch fit on the LO body rate), integrate the
difference to the LO rate, and keep only the part between LO knots (deviation from the knot-to-knot SLERP).
Compare with the per-frame rotation fitted from the image residuals.  usage: diag_imu.py SOLVE_DIR"""
import sys, json, numpy as np
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal')
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
from nontarget_cal.lo.lotraj import LOTraj
from nontarget_cal.rgb.solve import traj_rates
d = Path(sys.argv[1]); res = json.load(open(d / 'result.json')); segs = res['segs']
W0 = Path('/hdd/DM_calib/nt_regress/full/work')
ins = np.load(W0 / 'extract/b0_ins.npz'); ti = ins['imu_t'].astype(np.int64); wi = ins['imu_rate']
pf = np.load(f'/hdd/DM_calib/nt_regress/dt_study/{d.name}_posefit.npz'); x, uk = pf['x'], pf['uk']
fseg, fms = uk // 10**7, uk % 10**7
z = np.load(d / 'state.npz'); ot = z['obs_t']
trs = {s: LOTraj.load(W0 / f'lo/ref_{s}.npz') for s in segs}
# ---- IMU -> LiDAR rotation and lag from the rates
A, B = [], []
for s in segs:
    tr = trs[s]; t0, t1 = tr.span(); m = (ti > t0 + 5e8) & (ti < t1 - 5e8)
    Wl, _ = traj_rates(tr, ti[m], h_ns=5e7)          # LO rate smoothed over 0.1 s
    A.append(wi[m]); B.append(Wl)
A, B = np.concatenate(A), np.concatenate(B)
def kabsch(P, Q):
    U, S, Vt = np.linalg.svd(Q.T @ P); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); return U @ D @ Vt
# smooth IMU to the same 0.1 s band before the fit
R_LI = np.diag([-1.0, 1.0, 1.0])     # IMU x-rate has the opposite sign (corr -0.95; y +0.99, z +1.00): not a rotation
r = B - A @ R_LI.T
print(f'IMU->LiDAR axes diag(-1,1,1); |w_LO - R w_IMU| rms '
      f'{np.degrees(np.sqrt((r**2).sum(1).mean())):.2f} deg/s (|w| rms {np.degrees(np.sqrt((B**2).sum(1).mean())):.2f})')
# lag: shift IMU times, pick the best
best = None
for lag in range(-60, 61, 5):
    rr = []
    for s in segs[:8]:
        tr = trs[s]; t0, t1 = tr.span(); m = (ti > t0 + 5e8) & (ti < t1 - 5e8)
        Wl, _ = traj_rates(tr, ti[m] + lag * 1e6, h_ns=5e7); rr.append(Wl - wi[m] @ R_LI.T)
    v = np.sqrt((np.concatenate(rr) ** 2).sum(1).mean())
    best = min(best or (9, 0), (v, lag))
print(f'IMU vs LO best lag {best[1]} ms (rms {np.degrees(best[0]):.2f} deg/s)')
lag = best[1] * 1e6
# ---- deviation between LO knots at the frame times
dev = np.full((len(uk), 3), np.nan)
for si, s in enumerate(segs):
    tr = trs[s]; sel = np.flatnonzero(fseg == si)
    if not len(sel):
        continue
    tf = np.unique(ot[(z['lm_seg'][z['obs_lm']] == si)])
    tf = tf[np.isin(np.round(tf / 1e6).astype(np.int64) % 10**7, fms[sel])]
    m = (ti > tr.tau[0] - 2e8) & (ti < tr.tau[-1] + 2e8)
    tI = ti[m].astype(np.float64) - lag
    Wl, _ = traj_rates(tr, tI, h_ns=2e6)             # LO body rate (piecewise constant between knots)
    dw = wi[m] @ R_LI.T - Wl
    dw -= np.median(dw, 0)                           # gyro bias
    th = np.concatenate([[np.zeros(3)], np.cumsum(0.5 * (dw[1:] + dw[:-1]) * np.diff(tI)[:, None] * 1e-9, 0)])
    q = lambda t: np.stack([np.interp(t, tI, th[:, k]) for k in range(3)], -1)
    k = np.clip(np.searchsorted(tr.tau, tf) - 1, 0, len(tr.tau) - 2)
    a = (tf - tr.tau[k]) / (tr.tau[k + 1] - tr.tau[k])
    between = q(tf) - (q(tr.tau[k].astype(np.float64)) * (1 - a)[:, None] + q(tr.tau[k + 1].astype(np.float64)) * a[:, None])
    key = np.round(tf / 1e6).astype(np.int64) % 10**7
    pos = {kk: j for j, kk in zip(sel, fms[sel])}
    for j, kk in enumerate(key):
        if kk in pos:
            dev[pos[kk]] = between[j]
ok = np.isfinite(dev).all(1)
fit = x[ok, :3]; dv = dev[ok]
print(f'frames {ok.sum()}: IMU between-knot rotation rms per axis [mdeg] {np.round(np.degrees(np.sqrt((dv**2).mean(0))) * 1e3, 1)}; '
      f'image-fitted rotation rms [mdeg] {np.round(np.degrees(np.sqrt((fit**2).mean(0))) * 1e3, 1)}')
for kx in range(3):
    c = np.corrcoef(fit[:, kx], dv[:, kx])[0, 1]
    sl = (fit[:, kx] @ dv[:, kx]) / (dv[:, kx] @ dv[:, kx])
    print(f'  axis {"xyz"[kx]} (LiDAR): corr(image fit, IMU between-knot) {c:+.2f}, slope {sl:+.2f}')
np.savez(f'/hdd/DM_calib/nt_regress/dt_study/{d.name}_imudev.npz', dev=dev, uk=uk, R_LI=R_LI, lag_ms=best[1])
# ---- full deviation (knot errors included), high-passed: theta = int (R w_imu - w_LO) dt minus its running mean
print('full IMU-vs-LO rotation deviation, high-passed (running mean removed), vs image fit:')
okey = z['lm_seg'][z['obs_lm']].astype(np.int64) * 10**7 + np.round(ot / 1e6).astype(np.int64) % 10**7
_, first = np.unique(okey, return_index=True)
tkey = dict(zip(okey[first].tolist(), ot[first].astype(np.float64).tolist()))
for hw in (0.25, 0.5, 1.0):
    F, D = [], []
    for si, s in enumerate(segs):
        tr = trs[s]; sel = np.flatnonzero(fseg == si)
        if len(sel) < 50:
            continue
        m = (ti > tr.tau[0] - 2e8) & (ti < tr.tau[-1] + 2e8)
        tI = ti[m].astype(np.float64) - lag
        Wl, _ = traj_rates(tr, tI, h_ns=2e6)
        dw = wi[m] @ R_LI.T - Wl; dw -= np.median(dw, 0)
        th = np.concatenate([[np.zeros(3)], np.cumsum(0.5 * (dw[1:] + dw[:-1]) * np.diff(tI)[:, None] * 1e-9, 0)])
        n = int(round(hw * 100)); k = np.ones(2 * n + 1) / (2 * n + 1)
        hp = th - np.stack([np.convolve(th[:, j], k, 'same') for j in range(3)], 1)
        o = np.argsort(fms[sel]); sel = sel[o]
        # frame times of this window (ms key -> ns, frames are within one window so the ms key is unambiguous)
        tf = np.array([tkey.get(int(kk), np.nan) for kk in uk[sel]])
        ok = np.isfinite(tf) & (tf > tI[n]) & (tf < tI[-n - 1])
        tf, sel = tf[ok], sel[ok]
        D.append(np.stack([np.interp(tf, tI, hp[:, j]) for j in range(3)], 1))
        xf = x[sel, :3]
        # the same running-mean removal on the image fit (frames ~30 Hz)
        nf = int(round(hw * 30)); kf = np.ones(2 * nf + 1) / (2 * nf + 1)
        F.append(xf - np.stack([np.convolve(xf[:, j], kf, 'same') for j in range(3)], 1))
    F, D = np.concatenate(F), np.concatenate(D)
    s_ = ', '.join(f'{"xyz"[j]}: corr {np.corrcoef(F[:, j], D[:, j])[0, 1]:+.2f} slope {(F[:, j] @ D[:, j]) / (D[:, j] @ D[:, j]):+.2f} '
                   f'rms img/imu {np.degrees(F[:, j].std()) * 1e3:.0f}/{np.degrees(D[:, j].std()) * 1e3:.0f} mdeg' for j in range(3))
    print(f'  high-pass {hw:.2f} s: {s_}')
