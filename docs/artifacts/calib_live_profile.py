"""One default round-robin solve with observation-only timing; no repository edits."""
from importlib.util import module_from_spec, spec_from_file_location
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=Path('/hdd/DM_calib/nontarget_cal'))
parser.add_argument('--scratch', type=Path, required=True, help='new output directory')
parser.add_argument('--baseline', type=Path, required=True, help='off measurement.json to compare')
args = parser.parse_args()
if args.scratch.exists():
    parser.error('--scratch must not already exist')
ROOT = args.source.resolve()
sys.path.insert(0, str(ROOT))
spec = spec_from_file_location('viz_equivalence', ROOT / 'tests/viz_equivalence.py')
harness = module_from_spec(spec)
spec.loader.exec_module(harness)

from nontarget_cal.viz import LiveViz

metrics = {name: dict(calls=0, cpu_s=0.0, wall_s=0.0, max_wall_s=0.0)
           for name in ('solver_observer', 'io_processing')}


def timed(callback, name):
    def call(*args, **kwargs):
        wall, cpu = time.perf_counter(), time.thread_time()
        try:
            return callback(*args, **kwargs)
        finally:
            elapsed = time.perf_counter() - wall
            record = metrics[name]
            record['calls'] += 1
            record['cpu_s'] += time.thread_time() - cpu
            record['wall_s'] += elapsed
            record['max_wall_s'] = max(record['max_wall_s'], elapsed)
    return call


original_configure = harness.configure_viz


def configure_and_instrument(*args, **kwargs):
    emitter = original_configure(*args, **kwargs)
    # Keep torch/solver import inside the measured workload, as in the harness.
    from nontarget_cal.rgb import solve as rgb_solve
    original_factory = rgb_solve._viz_observer

    def observer_factory(*args, **kwargs):
        observer = original_factory(*args, **kwargs)
        return timed(observer, 'solver_observer') if observer is not None else None

    rgb_solve._viz_observer = observer_factory
    return emitter


harness.configure_viz = configure_and_instrument
LiveViz._process = timed(LiveViz._process, 'io_processing')
scratch = args.scratch.resolve()
harness.worker(SimpleNamespace(scratch=scratch, enabled=True, case='rgb', iters=5, camera=None))
result = json.loads((scratch / 'measurement.json').read_text())
baseline = json.loads(args.baseline.read_text())
result['byte_identical_to_round_robin_off'] = result['numerical_sha256'] == baseline['numerical_sha256']
result['timing_scope'] = 'instrumented default round-robin on-only; do not pool with uninstrumented ABBA cohort'
result['observer_profile'] = metrics
result['observed_cpu_fraction_of_worker_cpu_pct'] = (
    100 * sum(item['cpu_s'] for item in metrics.values()) / result['cpu_s'])
result['observer_profile_note'] = (
    'thread CPU counts observed callback and I/O processing work, excluding waits; '
    'I/O wall time includes waits and overlaps the solve. This is not causal end-to-end wall overhead.')
(scratch / 'measurement.json').write_text(json.dumps(result, indent=2) + '\n')
assert result['byte_identical_to_round_robin_off'], 'timed observer altered numerical results'
print(json.dumps(result, indent=2))
