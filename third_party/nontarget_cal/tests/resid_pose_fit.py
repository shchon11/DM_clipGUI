"""How much of the RGB track residual is a per-frame LiDAR-pose (trajectory) error?
For a finished solve (landmarks, calibration fixed): per synchronous frame (all 14 cameras share the stamp) fit a
6-DoF correction of the LiDAR pose T_WL (body rotation dth, body translation dp) to the residuals of all
cameras (|r| < 3 px), and compare with a null fit on random groups of the same size. Also reports residual
growth with |t - track centre| before/after, the lag-1 correlation of residuals along a track, and residual vs
yaw rate / speed / time in window.  usage: diag_pose.py SOLVE_DIR"""
import sys, json, numpy as np, torch
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal')
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
from nontarget_cal.rgb.rigba import kb_project
from nontarget_cal.lo.lotraj import LOTraj
from nontarget_cal.rgb.solve import traj_rates
torch.set_num_threads(4)
d = Path(sys.argv[1])
res = json.load(open(d / 'result.json')); z = np.load(d / 'state.npz')
LO = Path('/hdd/DM_calib/nt_regress/full/work/lo')
cams = res['cams']; segs = res['segs']
X = z['X']; lm_seg = z['lm_seg']; oc = z['obs_cam']; ol = z['obs_lm']; uv = z['obs_uv']; ot = z['obs_t']
dt = np.array([res['cameras'][c].get('time_dt_s', 0.0) for c in cams])
Tcl = np.array([res['cameras'][c]['T_cam_lidar'] for c in cams]); intr = np.array([res['cameras'][c]['intr_kb'] for c in cams])
N = len(oc); seg_o = lm_seg[ol]
R = np.zeros((N, 2)); J = np.zeros((N, 2, 6)); YAW = np.zeros(N); SPD = np.zeros(N); TW = np.zeros(N)
def proj(XL, idx):
    Xc = np.einsum('nij,nj->ni', Tcl[oc[idx], :3, :3], XL) + Tcl[oc[idx], :3, 3]
    I = intr[oc[idx]]; t = lambda a: torch.as_tensor(a, dtype=torch.float64)
    return kb_project(t(Xc), *[t(I[:, k]) for k in range(7)], jac=False).numpy()
eps = 1e-6
for si, s in enumerate(segs):
    idx = np.flatnonzero(seg_o == si)
    tr = LOTraj.load(LO / f'ref_{s}.npz')
    t = ot[idx].astype(np.float64) + 1e9 * dt[oc[idx]]
    Rw, pw = tr.R(t), tr.p(t)
    XL = np.einsum('nji,nj->ni', Rw, X[ol[idx]] - pw)
    u0 = proj(XL, idx); R[idx] = u0 - uv[idx]
    # T_WL <- T_WL * exp([dth, dp]) (body frame): XL' = exp(-dth) (XL - dp)
    for k in range(3):
        e = np.zeros(3); e[k] = eps
        J[idx, :, k] = (proj(XL - np.cross(e, XL), idx) - u0) / eps
        J[idx, :, 3 + k] = (proj(XL - e, idx) - u0) / eps
    W, V = traj_rates(tr, t)
    YAW[idx] = np.degrees(np.linalg.norm(W, axis=1)); SPD[idx] = np.linalg.norm(V, axis=1)
    t0, t1 = tr.span(); TW[idx] = np.minimum(t - t0, t1 - t) * 1e-9
e0 = np.linalg.norm(R, axis=1)
key = seg_o.astype(np.int64) * 10**7 + np.round(ot / 1e6).astype(np.int64) % 10**7    # (window, frame ms)
uk, g = np.unique(key, return_inverse=True)
cnt = np.bincount(g)
ncam = np.array([len(np.unique(k)) for k in np.split(oc[np.argsort(g, kind='stable')], np.cumsum(cnt)[:-1])])
print(f'{d.name}: {N} obs, {len(uk)} synchronous frames, obs/frame median {np.median(cnt):.0f}, cameras/frame median {np.median(ncam):.0f}')
def fit(groups, label, lam=1e-3):
    ok = e0 < 3
    A = np.zeros((groups.max() + 1, 6, 6)); b = np.zeros((groups.max() + 1, 6))
    Jo, ro, go = J[ok], R[ok], groups[ok]
    np.add.at(A, go, np.einsum('nki,nkj->nij', Jo, Jo)); np.add.at(b, go, np.einsum('nki,nk->ni', Jo, ro))
    sc = np.maximum(np.einsum('gii->gi', A).mean(1), 1e-9)
    x = -np.linalg.solve(A + lam * sc[:, None, None] * np.eye(6), b[..., None])[..., 0]
    Rn = R + np.einsum('nki,ni->nk', J, x[groups])
    e1 = np.linalg.norm(Rn, axis=1)
    print(f'  {label:28s} median |r| {np.median(e0):.3f} -> {np.median(e1):.3f} px; rms(<3) {np.sqrt((e0[ok]**2).mean()):.3f} -> {np.sqrt((e1[ok]**2).mean()):.3f}; '
          f'|dth| median {np.degrees(np.median(np.linalg.norm(x[:, :3], axis=1))) * 1e3:.1f} mdeg, |dp| median {1e3 * np.median(np.linalg.norm(x[:, 3:], axis=1)):.1f} mm')
    return x, Rn, e1
x, Rn, e1 = fit(g, 'per-frame 6-DoF LiDAR pose')
rng = np.random.default_rng(0)
fit(rng.permutation(g), 'null: random groups')
# smoothness of the correction in time (a real trajectory error is smooth; noise is white)
fs = uk // 10**7
dth = np.degrees(x[:, :3]) * 1e3
same = fs[1:] == fs[:-1]
cc = [np.corrcoef(dth[:-1][same, k], dth[1:][same, k])[0, 1] for k in range(3)]
ccp = [np.corrcoef(x[:-1][same, 3 + k], x[1:][same, 3 + k])[0, 1] for k in range(3)]
print(f'  consecutive-frame correlation of the fitted correction: rot {np.round(cc, 2)}, pos {np.round(ccp, 2)}  (white noise ~0)')
print(f'  rms of fitted rotation per axis [mdeg] {np.round(np.sqrt((dth ** 2).mean(0)), 1)}, translation [mm] {np.round(1e3 * np.sqrt((x[:, 3:] ** 2).mean(0)), 1)}')
# along-track structure
M = len(X)
tmid = np.bincount(ol, weights=ot.astype(np.float64), minlength=M) / np.maximum(np.bincount(ol, minlength=M), 1)
age = np.abs(ot - tmid[ol]) * 1e-9
print(' |t - track centre| s : med|r| before / after per-frame pose fit')
for a, b in [(0, .2), (.2, .5), (.5, 1), (1, 2), (2, 1e9)]:
    m = (age >= a) & (age < b)
    print(f'   {a:.1f}-{b if b < 1e8 else "inf":<4} {m.sum() // 1000:6d}k  {np.median(e0[m]):.3f} / {np.median(e1[m]):.3f}')
o = np.lexsort((ot, ol)); lo_, Ro = ol[o], R[o]
nx = lo_[1:] == lo_[:-1]
ok = (e0[o][1:] < 3) & (e0[o][:-1] < 3) & nx
cor = [np.corrcoef(Ro[:-1][ok, k], Ro[1:][ok, k])[0, 1] for k in range(2)]
print(f'  lag-1 (4 frames = 133 ms) correlation of the residual along a track: u {cor[0]:.2f} v {cor[1]:.2f} (white KLT noise ~0; drift -> near 1)')
# per-track mean removal: how much is a constant offset per track (e.g. feature centre definition)
mu = np.zeros((M, 2)); np.add.at(mu, ol, R); mu /= np.maximum(np.bincount(ol, minlength=M), 1)[:, None]
print(f'  residual minus its track mean: median {np.median(np.linalg.norm(R - mu[ol], axis=1)):.3f} px')
print(' yaw rate deg/s : med|r|      speed m/s : med|r|      time from window edge s : med|r|')
for (a, b), (c, dd), (p, q) in zip([(0, 2), (2, 5), (5, 10), (10, 20), (20, 1e9)], [(0, 3), (3, 6), (6, 9), (9, 12), (12, 1e9)], [(0, 1), (1, 3), (3, 8), (8, 15), (15, 1e9)]):
    m1, m2, m3 = (YAW >= a) & (YAW < b), (SPD >= c) & (SPD < dd), (TW >= p) & (TW < q)
    f = lambda m: f'{np.median(e0[m]):.3f} ({m.sum() // 1000}k)' if m.sum() > 100 else '-'
    print(f'   {a:>3}-{b if b < 1e8 else "inf":<4}: {f(m1):16s}  {c:>3}-{dd if dd < 1e8 else "inf":<4}: {f(m2):16s}  {p:>3}-{q if q < 1e8 else "inf":<4}: {f(m3)}')
np.savez('/hdd/DM_calib/nt_regress/dt_study/' + Path(sys.argv[1]).name + '_posefit.npz', x=x, uk=uk)
keys = z['lm_key']
lk = np.array([k.startswith("('g'") for k in keys])
if lk.any():
    L = lk[ol]
    print(f'linked landmarks: {lk.sum()} ({L.sum()} obs): med|r| linked {np.median(e0[L]):.3f} -> {np.median(e1[L]):.3f} after per-frame pose fit; '
          f'unlinked {np.median(e0[~L]):.3f} -> {np.median(e1[~L]):.3f}')
    span = np.zeros(M); np.maximum.at(span, ol, ot / 1e9); mn = np.full(M, np.inf); np.minimum.at(mn, ol, ot / 1e9); span -= mn
    for a, b in [(0, 1), (1, 3), (3, 6), (6, 1e9)]:
        m = L & (span[ol] >= a) & (span[ol] < b)
        if m.sum() > 100:
            print(f'   linked, landmark time span {a}-{b if b < 1e8 else "inf"} s: n {m.sum()} med|r| {np.median(e0[m]):.3f} / {np.median(e1[m]):.3f}')
    m = ~L
    for a, b in [(0, 1), (1, 3), (3, 6), (6, 1e9)]:
        mm = m & (span[ol] >= a) & (span[ol] < b)
        if mm.sum() > 100:
            print(f'   unlinked, time span {a}-{b if b < 1e8 else "inf"} s: n {mm.sum()} med|r| {np.median(e0[mm]):.3f} / {np.median(e1[mm]):.3f}')
