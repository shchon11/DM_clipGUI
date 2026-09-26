"""Matcher bake-off on real candidate links of the night bag (rgb/crosstime.py geometry, wide gate 4/8 px).
usage: bakeoff.py WIN[,WIN..] matcher[,matcher..] [max_pairs_per_window]"""
import json, sys, time, os
import numpy as np, cv2
from pathlib import Path
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal')
cv2.setNumThreads(1)
from nontarget_cal.workspace import Workspace
from nontarget_cal.lo.lotraj import LOTraj
from nontarget_cal.rgb import crosstime as CT
from nontarget_cal.rgb.lmatch import Matcher
import torch

W = Path('/hdd/DM_calib/nt_regress/full/work'); ws = Workspace(W)
OUT = Path(os.environ.get('BAKEOFF_OUT', '/hdd/DM_calib/nt_regress/crosstime_learned/bakeoff')); OUT.mkdir(exist_ok=True, parents=True)
wins, mats = sys.argv[1].split(','), sys.argv[2].split(',')
maxp = int(sys.argv[3]) if len(sys.argv) > 3 else 1500
result = json.load(open(W / 'rgb/final_keys/result.json'))
state = W / 'rgb/final_keys/state.npz'
segs, cams = result['segs'], result['cams']


class NoMatch:
    """NCC only (no learned gross check)."""
    t_extract = t_match = 0.0
    def extract(self, crops): return [None] * len(crops)
    def match(self, fa, fb, S): return [(np.zeros((0, 2)), np.zeros((0, 2)))] * len(fa)


class SiftCrops:
    """Baseline: SIFT on the same rectified crops, ratio test 0.8."""
    def __init__(self):
        self.s = cv2.SIFT_create(nfeatures=256); self.bf = cv2.BFMatcher(); self.t_extract = self.t_match = 0.0
    def extract(self, crops):
        t0 = time.time(); r = []
        for c in crops:
            k, d = self.s.detectAndCompute(c, None)
            r.append((np.array([p.pt for p in k], np.float32).reshape(-1, 2), d))
        self.t_extract += time.time() - t0; return r
    def match(self, fa, fb, S):
        t0 = time.time(); res = []
        for (ka, da), (kb, db) in zip(fa, fb):
            if da is None or db is None or len(da) < 2 or len(db) < 2:
                res.append((np.zeros((0, 2)), np.zeros((0, 2)))); continue
            mm = [m for m, n in self.bf.knnMatch(da, db, k=2) if m.distance < 0.8 * n.distance]
            res.append((ka[[m.queryIdx for m in mm]], kb[[m.trainIdx for m in mm]]))
        self.t_match += time.time() - t0; return res


z = np.load(state, allow_pickle=True)
ot, ouv, olm, ocam = z['obs_t'], z['obs_uv'], z['obs_lm'], z['obs_cam']
M_ = len(z['X'])
order = np.argsort(olm, kind='stable'); starts = np.searchsorted(olm[order], np.arange(M_ + 1))
lm_cam = np.full(M_, -1); lm_cam[olm] = ocam
T_LC = {c: np.array(result['cameras'][c]['T_lidar_cam']) for c in cams}
intr = {c: np.array(result['cameras'][c]['intr_kb']) for c in cams}
cache = {}
def img(seg, cam, t):
    k = (seg, cam, t)
    if k not in cache:
        g = cv2.imread(str(ws.seg_dir(seg) / 'cam' / cam / f'{int(t)}.jpg'), cv2.IMREAD_GRAYSCALE)
        if g is not None and g.mean() < 60:
            g = cv2.createCLAHE(3.0, (8, 8)).apply(g)
        cache[k] = g
        if len(cache) > 300: cache.pop(next(iter(cache)))
    return cache[k]

def grp(c):
    return 'R' if 'rear' in c else ('S' if 'side' in c else 'F')

rng = np.random.default_rng(0)
for win in wins:
    si = segs.index(win)
    f = OUT / f'pairs_{win}.npz'
    if not f.exists():
        t0 = time.time()
        pairs, st = CT.associate(ws, result, state, gate_a=0.1, gate_b=0.01, max_px_med=4.0, max_px_p90=8.0, only={si}, matcher='none', log=lambda *a: None)
        la = np.array([p[1] for p in pairs]); lb = np.array([p[2] for p in pairs])
        g = np.array([''.join(sorted(grp(cams[lm_cam[a]]) + grp(cams[lm_cam[b]]))) for a, b in zip(la, lb)])
        # stratified sample: all pairs with rear, up to maxp/4 of each other group
        sel = []
        for gg in np.unique(g):
            ii = np.flatnonzero(g == gg)
            k = len(ii) if 'R' in gg else min(len(ii), maxp // 4)
            sel += list(rng.choice(ii, k, replace=False))
        sel = np.sort(np.array(sel))
        np.savez(f, la=la[sel], lb=lb[sel], Xm=np.stack([pairs[i][3] for i in sel]), dts=np.array([pairs[i][4] for i in sel]),
                 med=np.array([pairs[i][5] for i in sel]), p90=np.array([pairs[i][6] for i in sel]), grp=g[sel],
                 n_all=len(pairs), n_grp=json.dumps({gg: int((g == gg).sum()) for gg in np.unique(g)}), cand=st['candidates'])
        print(win, 'geometry', time.time() - t0, 's pairs', len(pairs), json.dumps({gg: int((g == gg).sum()) for gg in np.unique(g)}), flush=True)
    P = np.load(f)
    tr = LOTraj.load(ws.lo('ref', win))
    for mname in mats:
        of = OUT / f'res_{win}_{mname}.npz'
        if of.exists(): continue
        if torch.cuda.is_available(): torch.cuda.reset_peak_memory_stats()
        t0 = time.time()
        M = SiftCrops() if mname == 'sift_crops' else (NoMatch() if mname == 'ncc_only' else Matcher(mname))
        tl = time.time() - t0
        n = len(P['la'])
        rs = {}
        for tag, sh in [('0', None), ('s2', np.full(n, 2.0)), ('s4', np.full(n, 4.0))]:
            t1 = time.time(); e0, m0 = M.t_extract, M.t_match
            r = CT.verify_pairs(win, tr, cams, T_LC, intr, ot, ouv, order, starts, lm_cam, P['la'], P['lb'], P['Xm'], M, img,
                                shift_px=sh, epi=(sh is None))
            for k, v in r.items(): rs[f'{tag}_{k}'] = v
            rs[f'{tag}_wall'] = time.time() - t1; rs[f'{tag}_t_extract'] = M.t_extract - e0; rs[f'{tag}_t_match'] = M.t_match - m0
        gpu = torch.cuda.max_memory_allocated() / 2**20 if torch.cuda.is_available() else 0
        np.savez(of, gpu_mb=gpu, load_s=tl, n=n, **rs)
        print(win, mname, f"n {n} wall {rs['0_wall']:.1f}s extract {rs['0_t_extract']:.1f} match {rs['0_t_match']:.1f} gpu {gpu:.0f} MB", flush=True)
        del M; torch.cuda.empty_cache()
