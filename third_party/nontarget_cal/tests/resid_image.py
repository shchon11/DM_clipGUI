"""Night feature localisation: residual vs local brightness / corner strength at the observation, and the KLT
forward-backward closure over whole tracks (drift).  usage: diag_img.py (sample: windows S03 W05 W11, all cameras)"""
import sys, json, numpy as np, cv2
from pathlib import Path
from multiprocessing import Pool
W0 = Path('/hdd/DM_calib/nt_regress/full/work'); D = Path('/hdd/DM_calib/nt_regress/dt_study')
res = json.load(open(W0 / 'rgb/final/result.json')); cams = res['cams']; segs = res['segs']
LK = dict(winSize=(21, 21), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
WINS = ['S03', 'W05', 'W11']
def job(args):
    cv2.setNumThreads(1)
    win, ci = args
    cam = cams[ci]; si = segs.index(win)
    z = np.load(W0 / 'rgb/final/state.npz'); R = np.load(D / 'base_final_resid.npy')
    m = (z['obs_cam'] == ci) & (z['lm_seg'][z['obs_lm']] == si)
    idx = np.flatnonzero(m); t = z['obs_t'][idx]; uv = z['obs_uv'][idx]
    out = {'bright': np.full(len(idx), np.nan), 'minEig': np.full(len(idx), np.nan), 'e': np.linalg.norm(R[idx], axis=1)}
    frames = np.unique(t)[::3]                      # every 3rd observed frame
    for f in frames:
        img = cv2.imread(str(W0 / 'extract' / win / 'cam' / cam / f'{f}.jpg'), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        k = np.flatnonzero(t == f)
        box = cv2.blur(img.astype(np.float32), (21, 21))
        eig = cv2.cornerMinEigenVal(img, 7, 3)
        x = np.clip(uv[k, 0].round().astype(int), 0, img.shape[1] - 1); y = np.clip(uv[k, 1].round().astype(int), 0, img.shape[0] - 1)
        out['bright'][k] = box[y, x]; out['minEig'][k] = eig[y, x]
    # forward-backward closure of long tracks from the tracks file (every frame)
    tz = np.load(W0 / 'tracks' / f'tracks_{win}_{cam}.npz')
    fns, of, otk, xy = tz['frame_ns'], tz['obs_frame'], tz['obs_track'], tz['obs_xy']
    ids, cnt = np.unique(otk, return_counts=True)
    long_ = ids[cnt >= 60]
    rng = np.random.default_rng(ci); long_ = rng.choice(long_, min(300, len(long_)), replace=False) if len(long_) else long_
    sel = np.isin(otk, long_)
    of, otk, xy = of[sel], otk[sel], xy[sel]
    last = {tid: of[otk == tid].max() for tid in long_}; first = {tid: of[otk == tid].min() for tid in long_}
    fwd = {(a, b): p for a, b, p in zip(otk, of, xy)}
    cur = {}; clos = []
    prev = None
    for fi in range(max(last.values()) if last else -1, -1, -1):
        img = cv2.imread(str(W0 / 'extract' / win / 'cam' / cam / f'{fns[fi]}.jpg'), cv2.IMREAD_GRAYSCALE)
        if prev is not None and cur:
            tids = list(cur); p0 = np.array([cur[k] for k in tids], np.float32).reshape(-1, 1, 2)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, img, p0, None, **LK)
            for k, p, s in zip(tids, p1.reshape(-1, 2), st.ravel()):
                if s != 1:
                    del cur[k]; continue
                cur[k] = p
                if (k, fi) in fwd:
                    clos.append((last[k] - fi, np.linalg.norm(p - fwd[(k, fi)])))
                if fi <= first[k]:
                    del cur[k]
        for k in long_:
            if last[k] == fi:
                cur[k] = fwd[(k, fi)]
        prev = img
    out['closure'] = np.array(clos).reshape(-1, 2)
    return win, cam, out
if __name__ == '__main__':
    jobs = [(w, c) for w in WINS for c in range(len(cams))]
    with Pool(4) as p:
        rs = p.map(job, jobs, chunksize=1)
    np.save(D / 'diag_img_raw.npy', np.array(rs, dtype=object), allow_pickle=True)
    B = np.concatenate([r[2]['bright'] for r in rs]); E = np.concatenate([r[2]['minEig'] for r in rs]); e = np.concatenate([r[2]['e'] for r in rs])
    ok = np.isfinite(B)
    print(f'obs with image samples: {ok.sum()} (windows {WINS})')
    print(' local brightness (21x21 mean gray) : n, med|r|')
    for a, b in [(0, 20), (20, 40), (40, 80), (80, 140), (140, 256)]:
        m = ok & (B >= a) & (B < b)
        if m.sum() > 100: print(f'   {a:3d}-{b:<3d}: {m.sum():7d}  {np.median(e[m]):.3f}')
    q = np.nanpercentile(E[ok], [0, 20, 40, 60, 80, 100])
    print(' corner strength (min eigenvalue, quintiles): n, med|r|')
    for a, b in zip(q[:-1], q[1:]):
        m = ok & (E >= a) & (E <= b)
        print(f'   {a:9.5f}-{b:9.5f}: {m.sum():7d}  {np.median(e[m]):.3f}')
    C = np.concatenate([r[2]['closure'] for r in rs])
    print(' KLT forward-backward closure over whole tracks (per camera group): frames travelled back -> median |closure| px')
    for g, sel in [('front+top', lambda c: 'front' in c or 'top' in c), ('side', lambda c: 'side' in c), ('rear', lambda c: 'rear' in c)]:
        Cg = np.concatenate([r[2]['closure'] for r in rs if sel(r[1])])
        print(f'   {g:9s} ' + '  '.join(f'{a}-{b}: {np.median(Cg[(Cg[:, 0] >= a) & (Cg[:, 0] < b), 1]):.2f}' for a, b in [(1, 5), (5, 15), (15, 30), (30, 60), (60, 120), (120, 1200)] if ((Cg[:, 0] >= a) & (Cg[:, 0] < b)).sum() > 50))
