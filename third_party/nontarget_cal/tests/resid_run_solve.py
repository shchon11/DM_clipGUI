import json, sys, os
sys.path.insert(0, '/hdd/DM_calib/nontarget_cal_resid')
from pathlib import Path
from nontarget_cal.config import Config
from nontarget_cal.workspace import Workspace
from nontarget_cal import tasks
W = Path(os.environ.get('DTW', '/hdd/DM_calib/nt_regress/resid_study/work'))
cfg = Config(json.loads((W / 'config_snapshot.json').read_text()))
ws = Workspace(W)
name, segs, start, opts, held = sys.argv[1], json.loads(sys.argv[2]), json.loads(sys.argv[3]), json.loads(sys.argv[4]), (sys.argv[5] if len(sys.argv) > 5 else None)
log = open(W / 'rgb' / f'{name}.log', 'a')
r = tasks.t_rgb_solve(ws, cfg, lambda *a, **k: (log.write(' '.join(map(str, a)) + '\n'), log.flush()), name, segs, list(cfg['cameras']['rgb']), start, opts, held=held)
print(r)
