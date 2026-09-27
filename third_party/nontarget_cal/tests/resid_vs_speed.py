"""Residual vs image-motion speed of a finished RGB solve (result.json + state.npz).
Per observation: residual r = model - measured (pose at t + dt_c), image velocity g = d uv / d t from the
LO trajectory (+-5 ms), speed = |g| * 33.3 ms (px/frame). A time error delta shows up as r = -g delta:
per camera delta_hat = -sum(r.g)/sum(|g|^2) over |r| < 3 px (the offset the residuals still ask for).
usage: diag.py SOLVE_DIR [--json out.json]"""
import sys, json, numpy as np, torch
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal')
from pathlib import Path
from nontarget_cal.rgb.rigba import kb_project
from nontarget_cal.lo.lotraj import LOTraj
torch.set_num_threads(4)
d = Path(sys.argv[1])
res = json.load(open(d / 'result.json')); z = np.load(d / 'state.npz')
LO = Path('/hdd/DM_calib/nt_regress/full/work/lo')
cams = res['cams']; segs = res['segs']
X = z['X']; lm_seg = z['lm_seg']; oc = z['obs_cam']; ol = z['obs_lm']; uv = z['obs_uv']; ot = z['obs_t'].astype(np.float64)
H = 1200.0
dt = np.array([res['cameras'][c].get('time_dt_s', 0.0) for c in cams]) + 1e-3 * res.get('dt_ms', 0.0)
rs = np.array([res['cameras'][c].get('time_rs_s', 0.0) for c in cams])
Tcl = np.array([res['cameras'][c]['T_cam_lidar'] for c in cams]); intr = np.array([res['cameras'][c]['intr_kb'] for c in cams])
seg_o = lm_seg[ol]
N = len(oc); R = np.zeros((N, 2)); G = np.zeros((N, 2)); DEP = np.zeros(N)
def proj(Rw, pw, idx):
    XL = np.einsum('nji,nj->ni', Rw, X[ol[idx]] - pw)
    Xc = np.einsum('nij,nj->ni', Tcl[oc[idx], :3, :3], XL) + Tcl[oc[idx], :3, 3]
    I = intr[oc[idx]]
    t = lambda a: torch.as_tensor(a, dtype=torch.float64)
    DEP[idx] = np.linalg.norm(Xc, axis=1)
    return kb_project(t(Xc), *[t(I[:, k]) for k in range(7)], jac=False).numpy()
h = 5e6
for si, s in enumerate(segs):
    idx = np.flatnonzero(seg_o == si)
    if not len(idx):
        continue
    tr = LOTraj.load(LO / f'ref_{s}.npz')
    t = ot[idx] + 1e9 * (dt[oc[idx]] + rs[oc[idx]] * (uv[idx, 1] / H - 0.5))
    u0 = proj(tr.R(t), tr.p(t), idx)
    up = proj(tr.R(t + h), tr.p(t + h), idx); um = proj(tr.R(t - h), tr.p(t - h), idx)
    u0 = proj(tr.R(t), tr.p(t), idx)
    R[idx] = u0 - uv[idx]; G[idx] = (up - um) / (2 * h * 1e-9)
e = np.linalg.norm(R, axis=1); gs = np.linalg.norm(G, axis=1); spd = gs * 1 / 30.0
ok = e < 3
rpar = np.einsum('ni,ni->n', R, G) / np.maximum(gs, 1e-9)       # residual along the motion (px)
rperp = (R[:, 0] * G[:, 1] - R[:, 1] * G[:, 0]) / np.maximum(gs, 1e-9)
out = {'solve': str(d), 'median_px': float(np.median(e)), 'bins': [], 'cams': {}}
edges = [0, 1, 2, 4, 8, 16, 32, 1e9]
print(f'{d.name}: median |r| {np.median(e):.3f} px, n {N}')
print(' speed px/frame |   n     | med|r| | med|r_par| | med|r_perp| | mean r_par (signed) | rms r_par(<3px)')
for a, b in zip(edges[:-1], edges[1:]):
    m = (spd >= a) & (spd < b)
    if m.sum() < 100:
        continue
    mm = m & ok
    row = dict(lo=a, hi=b, n=int(m.sum()), med=float(np.median(e[m])), med_par=float(np.median(np.abs(rpar[m]))),
               med_perp=float(np.median(np.abs(rperp[m]))), mean_par=float(rpar[mm].mean()), rms_par=float(np.sqrt((rpar[mm] ** 2).mean())))
    out['bins'].append(row)
    print(f' {a:5.0f}-{b if b < 1e8 else np.inf:<6} | {row["n"]:8d} | {row["med"]:.3f}  | {row["med_par"]:.3f}      | {row["med_perp"]:.3f}       | {row["mean_par"]:+.3f}              | {row["rms_par"]:.3f}')
print(' camera            | delta_hat ms (resid asks) | rs_hat ms | med|r| slow(<4) fast(>=8 px/fr) | par/perp fast')
for i, c in enumerate(cams):
    m = (oc == i) & ok
    g = G[m]; r = R[m]
    dh = -float((r * g).sum() / (g * g).sum())
    # joint dt + rs fit
    rowf = (uv[m, 1] / H - 0.5)
    A = np.stack([g.reshape(-1), (g * rowf[:, None]).reshape(-1)], 1)
    sol = np.linalg.lstsq(A, -r.reshape(-1), rcond=None)[0]
    ms, mf = (oc == i) & (spd < 4), (oc == i) & (spd >= 8)
    out['cams'][c] = dict(delta_hat_ms=1e3 * dh, dt_rs_fit_ms=[1e3 * sol[0], 1e3 * sol[1]], dt_model_ms=1e3 * dt[i], rs_model_ms=1e3 * rs[i],
                          med_slow=float(np.median(e[ms])) if ms.sum() else None, med_fast=float(np.median(e[mf])) if mf.sum() else None,
                          par_fast=float(np.median(np.abs(rpar[mf]))) if mf.sum() else None, perp_fast=float(np.median(np.abs(rperp[mf]))) if mf.sum() else None,
                          frac_fast=float(mf.sum() / max((oc == i).sum(), 1)))
    q = out['cams'][c]
    print(f' {c:18s}| {q["delta_hat_ms"]:+7.2f} (dt model {q["dt_model_ms"]:+6.2f}) | {q["dt_rs_fit_ms"][1]:+6.2f} | {q["med_slow"]:.3f} {q["med_fast"] if q["med_fast"] is None else round(q["med_fast"], 3)} ({100*q["frac_fast"]:.0f}% fast) | {q["par_fast"] if q["par_fast"] is None else round(q["par_fast"],3)}/{q["perp_fast"] if q["perp_fast"] is None else round(q["perp_fast"],3)}')
if '--json' in sys.argv:
    json.dump(out, open(sys.argv[sys.argv.index('--json') + 1], 'w'), indent=1)
# ---- other structure: residual vs time from the track centre (KLT drift) and vs radius (lens)
M = len(X)
tmid = np.bincount(ol, weights=ot, minlength=M) / np.maximum(np.bincount(ol, minlength=M), 1)
age = np.abs(ot - tmid[ol]) * 1e-9
print(' |t - track centre| s | n | med|r|')
for a, b in [(0, .2), (.2, .5), (.5, 1), (1, 2), (2, 1e9)]:
    m = (age >= a) & (age < b)
    if m.sum() > 100:
        print(f'  {a:.1f}-{b:<5} | {m.sum():8d} | {np.median(e[m]):.3f}')
rad = np.hypot(uv[:, 0] - intr[oc, 2], uv[:, 1] - intr[oc, 3])
print(' radius px | n | med|r|')
for a, b in [(0, 200), (200, 400), (400, 600), (600, 800), (800, 2000)]:
    m = (rad >= a) & (rad < b)
    if m.sum() > 100:
        print(f'  {a}-{b} | {m.sum():8d} | {np.median(e[m]):.3f}')
print(' med|r| by |t - track centre| (rows, s) x depth (cols, m)')
db = [(0, 6), (6, 12), (12, 25), (25, 1e9)]
print('   age      | ' + ' | '.join(f'{a}-{b if b < 1e8 else "inf"} m' for a, b in db))
for a, b in [(0, .2), (.2, .5), (.5, 1), (1, 2), (2, 1e9)]:
    m = (age >= a) & (age < b)
    print(f'  {a:.1f}-{b if b < 1e8 else "inf":<4}  | ' + ' | '.join(f'{np.median(e[m & (DEP >= c) & (DEP < d)]):.3f} ({(m & (DEP >= c) & (DEP < d)).sum() // 1000}k)' for c, d in db))
