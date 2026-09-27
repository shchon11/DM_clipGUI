"""KLT drift: forward-backward closure over whole tracks, for the frame-to-frame tracker (tracks.py) and the
anchored tracker (tracks_anchor.py), each closed with ITS OWN method. From a tracks file, long tracks (>= 60 frames)
are followed backwards from their last observation with the same tracker and compared with the forward position
at every earlier frame: closure vs frames travelled.
usage: resid_closure.py TRACKS_DIR_BASE TRACKS_DIR_ANCHORED WIN CAM [CAM ...]"""
import sys
import numpy as np
import cv2
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal_resid')
from pathlib import Path
from nontarget_cal.rgb.tracks import LK
from nontarget_cal.rgb.tracks_anchor import HALF, Templates, _measure

W0 = Path('/hdd/DM_calib/nt_regress/full/work')
cv2.setNumThreads(1)


def long_tracks(path, n=300, min_len=60, seed=0):
    z = np.load(path)
    fns, of, ot, xy = z['frame_ns'], z['obs_frame'], z['obs_track'], z['obs_xy']
    ids, cnt = np.unique(ot, return_counts=True)
    L = ids[cnt >= min_len]
    L = np.random.default_rng(seed).choice(L, min(n, len(L)), replace=False) if len(L) else L
    sel = np.isin(ot, L)
    return fns, of[sel], ot[sel], xy[sel]


def closure(win, cam, path, anchored, max_age=30, fb=0.3, jump=2.0):
    fns, of, ot, xy = long_tracks(path)
    first = {t: of[ot == t].min() for t in np.unique(ot)}
    last = {t: of[ot == t].max() for t in np.unique(ot)}
    fwd = {(t, f): p for t, f, p in zip(ot, of, xy)}
    cur, age, T = {}, {}, {}
    out = []
    prev = prev_p = None
    for fi in range(max(last.values()), -1, -1):
        img = cv2.imread(str(W0 / 'extract' / win / 'cam' / cam / f'{fns[fi]}.jpg'), cv2.IMREAD_GRAYSCALE)
        img_p = cv2.copyMakeBorder(img, HALF, HALF, HALF, HALF, cv2.BORDER_REPLICATE)
        if prev is not None and cur:
            tids = list(cur)
            p0 = np.array([cur[k] for k in tids], np.float32)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, img, p0.reshape(-1, 1, 2), None, **LK)
            p1 = p1.reshape(-1, 2)
            ok = st.ravel() == 1
            if anchored:
                Tm = Templates()
                Tm.P = np.stack([T[k][0] for k in tids]); Tm.q = np.stack([T[k][1] for k in tids])
                Tm.f = np.array([T[k][2] for k in tids])
                old = ok & (Tm.f - fi > max_age)                  # backwards: anchor frame is later
                for j in np.flatnonzero(old):
                    P_, q_ = Templates.cut(prev_p, p0[j:j + 1]); T[tids[j]] = (P_[0], q_[0], fi + 1)
                Tm.P = np.stack([T[k][0] for k in tids]); Tm.q = np.stack([T[k][1] for k in tids])
                k_ = np.flatnonzero(ok)
                pos, g, _ = _measure(Tm, k_, img_p, p1[k_], fb, jump)
                p1[k_[g]] = pos[g]
                bad = k_[~g]
                for j in bad:          # re-anchor at the previous frame and retry
                    P_, q_ = Templates.cut(prev_p, p0[j:j + 1]); T[tids[j]] = (P_[0], q_[0], fi + 1)
                if len(bad):
                    Tm.P = np.stack([T[k][0] for k in tids]); Tm.q = np.stack([T[k][1] for k in tids])
                    pos, g2, _ = _measure(Tm, bad, img_p, p1[bad], fb, jump)
                    p1[bad[g2]] = pos[g2]
                    for j in bad[~g2]:
                        P_, q_ = Templates.cut(img_p, p1[j:j + 1]); T[tids[j]] = (P_[0], q_[0], fi)
            for j, k in enumerate(tids):
                if not ok[j]:
                    del cur[k]; continue
                cur[k] = p1[j]
                if (k, fi) in fwd:
                    out.append((last[k] - fi, np.linalg.norm(p1[j] - fwd[(k, fi)])))
                if fi <= first[k]:
                    del cur[k]
        for k in last:
            if last[k] == fi:
                cur[k] = fwd[(k, fi)]
                P_, q_ = Templates.cut(img_p, cur[k][None].astype(np.float32)); T[k] = (P_[0], q_[0], fi)
        prev, prev_p = img, img_p
    return np.array(out).reshape(-1, 2)


if __name__ == '__main__':
    base, anch, win = sys.argv[1], sys.argv[2], sys.argv[3]
    for cam in sys.argv[4:]:
        for name, d, a in (('frame-to-frame', base, False), ('anchored', anch, True)):
            C = closure(win, cam, Path(d) / f'tracks_{win}_{cam}.npz', a)
            print(f'{cam:18s} {name:15s} ' + '  '.join(
                f'{lo}-{hi}: {np.median(C[(C[:, 0] >= lo) & (C[:, 0] < hi), 1]):.2f}'
                for lo, hi in [(1, 5), (5, 15), (15, 30), (30, 60), (60, 120), (120, 1200)]
                if ((C[:, 0] >= lo) & (C[:, 0] < hi)).sum() > 50), flush=True)
