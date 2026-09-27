"""Jacobian check of the RGB time model (dt, rs, landmark, rotation) by central differences, and the
first-order pose vs the exact LO pose at t + d."""
import sys, numpy as np, torch
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal')
from nontarget_cal.rgb.rigba import Cameras, Obs, project, NPT as NP, DT
from nontarget_cal.rgb.solve import traj_rates, retime
from nontarget_cal.lo.lotraj import LOTraj
torch.manual_seed(0); rng = np.random.default_rng(0)
tr = LOTraj.load('/hdd/DM_calib/nt_regress/full/work/lo/ref_S03.npz')
t0, t1 = tr.span()
C, K, N = 3, 40, 400
T = np.tile(np.eye(4), (C, 1, 1))
from scipy.spatial.transform import Rotation as Rot
for c in range(C):
    T[c, :3, :3] = Rot.from_euler('ZYX', [90 * c + 10, 5, -100]).as_matrix(); T[c, :3, 3] = rng.normal(0, .5, 3)
I = np.tile([1000., 1000., 960., 600., 0.02, -0.01, 0.003, 0.], (C, 1))
cams = Cameras(['a', 'b', 'c'], T, I); cams.enable_time()
cams.set_time([0.012, -0.020, 0.005], [0.003, -0.002, 0.0])
tp = np.sort(rng.uniform(t0 + 1e9, t1 - 1e9, K)).astype(np.int64)
pc = rng.integers(0, C, K)
cam = pc[(pidx := rng.integers(0, K, N))]
# landmarks in front of each camera at the pose time
M = N
Xc = np.c_[rng.uniform(-3, 3, (N, 2)), rng.uniform(4, 20, N)]
R, p = tr.R(tp[pidx] + 1e9 * cams.dt.numpy()[cam]), tr.p(tp[pidx] + 1e9 * cams.dt.numpy()[cam])
XL = np.einsum('nij,nj->ni', T[cam, :3, :3], Xc) + T[cam, :3, 3]
X = torch.tensor(np.einsum('nij,nj->ni', R, XL) + p)
uv = rng.uniform(0, 1200, (N, 2))
obs = Obs(cam, np.arange(N), uv, pidx, np.zeros((K, 3, 3)), np.zeros((K, 3)))
obs.poses = {'t': tp, 'seg': np.zeros(K, int), 'cam': pc, 'segs': ['S03']}
retime(obs, cams, {'S03': tr}, True, 1200.)
# move dt away from the linearisation point
cams.dt = cams.dt + torch.tensor([0.004, -0.003, 0.002], dtype=DT)
r, Jc, Jl, _ = project(cams, obs, X)
def num_c(k, h):
    out = []
    for sgn in (1, -1):
        c2 = cams.copy(); d = torch.zeros(C, NP, dtype=DT); d[:, k] = sgn * h; c2.apply(d)
        out.append(project(c2, obs, X, jac=False)[0])
    return (out[0] - out[1]) / (2 * h)
for k, h, nm in [(0, 1e-6, 'wx'), (4, 1e-6, 'cy_L'), (6, 1e-6, 'logF'), (13, 1e-6, 'dt'), (14, 1e-6, 'rs')]:
    n = num_c(k, h); a = Jc[:, :, k]
    print(f'{nm:5s} max|ana-num| {float((a - n).abs().max()):.2e}  (max|J| {float(a.abs().max()):.2e})')
for j in range(3):
    out = []
    for sgn in (1, -1):
        X2 = X.clone(); X2[:, j] += sgn * 1e-6; out.append(project(cams, obs, X2, jac=False)[0])
    n = (out[0] - out[1]) / 2e-6
    print(f'X{j}    max|ana-num| {float((Jl[:, :, j] - n).abs().max()):.2e}  (max|J| {float(Jl[:, :, j].abs().max()):.2e})')
# first-order vs exact pose at t + d
rowf = obs.uv[:, 1] / 1200 - 0.5
d = (cams.dt[obs.cam] - obs.tm['T0'][obs.pidx] + cams.rs[obs.cam] * rowf).numpy()
te = tp[pidx] + 1e9 * (obs.tm['T0'][obs.pidx].numpy() + d)
Re, pe = tr.R(te), tr.p(te)
XLe = np.einsum('nji,nj->ni', Re, X.numpy() - pe)
Xce = np.einsum('nji,nj->ni', T[cam, :3, :3], XLe - T[cam, :3, 3])
from nontarget_cal.rgb.rigba import kb_project
fx, fy, cx, cy, k1, k2, k3 = [a_[obs.cam] for a_ in cams.intr()]
uve = kb_project(torch.tensor(Xce), fx, fy, cx, cy, k1, k2, k3, jac=False)
print('first-order vs exact pose: |d| max %.1f ms, uv diff median %.4f max %.4f px' % (
    1e3 * np.abs(d).max(), float((uve - (r + obs.uv)).norm(dim=1).median()), float((uve - (r + obs.uv)).norm(dim=1).max())))
W, V = traj_rates(tr, tp)
print('rates: |w| median %.1f deg/s max %.1f, |v| median %.1f m/s' % (np.degrees(np.median(np.linalg.norm(W, axis=1))),
      np.degrees(np.linalg.norm(W, axis=1).max()), np.median(np.linalg.norm(V, axis=1))))
