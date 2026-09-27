"""Full linking run with a learned matcher (wide candidate gate, resumable cache) + merges for the gating variants."""
import json, sys, time, numpy as np, cv2
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal'); cv2.setNumThreads(1)
from pathlib import Path
from nontarget_cal.workspace import Workspace
from nontarget_cal.rgb.crosstime import associate_resumable, associate
W = Path('/hdd/DM_calib/nt_regress/full/work'); ws = Workspace(W)
O = Path('/hdd/DM_calib/nt_regress/crosstime_learned')
m = sys.argv[1]
result = json.load(open(W / 'rgb/final_keys/result.json')); state = W / 'rgb/final_keys/state.npz'
VAR = {'loose': dict(accept_gate_a_m=0.03, accept_gate_b=0.004, accept_px_med=2.0, accept_px_p90=4.0),
       'strict': dict(accept_gate_a_m=0.03, accept_gate_b=0.004, accept_px_med=1.0, accept_px_p90=2.0),
       'wide': dict()}
log = open(O / f'links_{m}.log', 'a')
L = lambda *a: (log.write(' '.join(map(str, a)) + '\n'), log.flush())
t0 = time.time()
for v in (sys.argv[2].split(',') if len(sys.argv) > 2 else VAR):
    out, st = associate_resumable(ws, result, state, O / f'cache_{m}', log=L, gate_a=0.1, gate_b=0.01, max_px_med=4.0,
                                  max_px_p90=8.0, matcher=m, learned=VAR[v])
    np.savez(O / f'merges_{m}_{v}.npz', **{k: np.array(x) for k, x in out.items()})
    json.dump(st, open(O / f'merges_{m}_{v}.json', 'w'), indent=1)
    print(v, 'wall', round(time.time() - t0), {k: st[k] for k in ('desc_ok', 'groups', 'landmarks_linked', 'linked_per_camera')}, flush=True)
