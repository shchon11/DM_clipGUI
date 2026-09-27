"""Re-track windows with the anchored tracker into WORKDIR/tracks.  usage: retrack.py WORKDIR WINS(comma|all) ANCHOR_JSON [NPROC]"""
import sys, os, json, time
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal_resid')
from pathlib import Path
from multiprocessing import Pool
W0 = Path('/hdd/DM_calib/nt_regress/full/work')
MASKS = Path('/hdd/DM_calib/nontarget_cal_resid/nontarget_cal/data/masks')
out_root = Path(sys.argv[1]); anchor = json.loads(sys.argv[3]); nproc = int(sys.argv[4]) if len(sys.argv) > 4 else 4
cams = json.load(open(W0 / 'config_snapshot.json'))['cameras']['rgb']
wins = json.load(open(W0 / 'rgb/summary.json'))['windows'] if sys.argv[2] == 'all' else sys.argv[2].split(',')
def job(a):
    import cv2
    cv2.setNumThreads(1)
    from nontarget_cal.rgb.tracks import run_tracks
    w, c = a
    out = out_root / 'tracks' / f'tracks_{w}_{c}.npz'
    if out.exists():
        return None
    lg = []
    r = run_tracks(W0 / 'extract' / w, c, MASKS / f'{c}.png', W0 / 'extract' / 'b0_ins.npz', out, grid=[16, 9], per_cell=8,
                   fb_max=0.5, min_len=5, log=lg.append, anchor=anchor)
    with open(out_root / 'tracks' / 'retrack.log', 'a') as f:
        f.write(lg[0] + '\n')
    return r
if __name__ == '__main__':
    (out_root / 'tracks').mkdir(parents=True, exist_ok=True)
    jobs = [(w, c) for w in wins for c in cams]
    # longest windows first
    t0 = time.time()
    with Pool(nproc) as p:
        rs = [r for r in p.imap_unordered(job, jobs) if r]
    print(f'{len(rs)} jobs, wall {time.time() - t0:.0f} s, cpu {sum(r["cpu_s"] for r in rs):.0f} s')
