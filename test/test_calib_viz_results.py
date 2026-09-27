"""The viewer must preserve solver gate semantics, including nighttime RGB."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from calib_viz.results import camera_gate
import online_calib as oc


REF_OUT = Path('/hdd/DM_calib/nt_regress/full/out')


def night_result():
    names = ['camera_front' + str(i) for i in range(1, 10)] + [
        'camera_top', 'camera_side_left', 'camera_side_right', 'camera_rear_left', 'camera_rear_right']
    metrics = {name: {'sensor': 'rgb', 'rot_deg': .2, 'along_axis_mm': 20,
                      'vote': {'pass': False, 'gate': False}} for name in names}
    metrics.update({name: {'sensor': 'thermal', 'rot_deg': .01, 'along_axis_mm': 5,
                          'vote': {'pass': True, 'gate': True}}
                    for name in ['thermal_left', 'thermal_right']})
    return {'validation': {'gate': {'pass': True, 'failures': []}}, 'metrics': metrics}


def test_night_rgb_informational_votes_do_not_fail_16_camera_gate():
    summary = night_result()
    for name in summary['metrics']:
        gate = camera_gate(summary, summary['metrics'], name)
        assert gate['pass'] is True
        assert bool(gate['informational']) == name.startswith('camera_')
        assert gate['source'] == 'tool_gate_failures'


@pytest.mark.skipif(not (REF_OUT / 'summary.json').exists(), reason='reference run unavailable')
def test_actual_night_result_matches_tool_report():
    summary = json.loads((REF_OUT / 'summary.json').read_text())
    metrics = json.loads((REF_OUT / 'metrics.json').read_text())
    assert len(metrics) == 16 and summary['validation']['gate']['pass'] is True
    assert all(camera_gate(summary, metrics, name)['pass'] is True for name in metrics)
    assert sum(bool(camera_gate(summary, metrics, name)['informational']) for name in metrics) == 14


def test_recorded_thermal_failure_is_local_and_authoritative():
    summary = night_result()
    summary['validation']['gate'] = {'pass': False, 'failures': ['thermal_left: voting gate failed']}
    assert camera_gate(summary, summary['metrics'], 'thermal_left')['pass'] is False
    assert camera_gate(summary, summary['metrics'], 'thermal_right')['pass'] is True
    assert camera_gate(summary, summary['metrics'], 'camera_front1')['pass'] is True


def test_layout_failure_does_not_recolour_individual_cameras():
    summary = night_result()
    summary['validation']['gate'] = {'pass': False, 'failures': ['layout rule front_row_straight failed']}
    assert all(camera_gate(summary, summary['metrics'], name)['pass'] is True for name in summary['metrics'])


def test_explicit_camera_gate_wins_over_metrics_and_default_thresholds():
    summary = night_result()
    summary['validation']['gate']['cameras'] = {'camera_front1': {'pass': False, 'reasons': ['heldout gate']}}
    decision = camera_gate(summary, summary['metrics'], 'camera_front1')
    assert decision['pass'] is False and decision['reasons'] == ['heldout gate']
    summary['validation']['gate']['cameras']['camera_front1'] = {'pass': True}
    assert camera_gate(summary, summary['metrics'], 'camera_front1', defaults={'rgb_rot_deg': .001})['pass'] is True


@pytest.mark.parametrize('summary,metrics', [({}, {}), ({'validation': {'gate': {'pass': True, 'failures': []}}}, {}),
                                            ({}, {'camera_front1': {'rot_deg': float('nan'), 'along_axis_mm': 10}})])
def test_missing_validation_is_unknown(summary, metrics):
    assert camera_gate(summary, metrics, 'camera_front1')['pass'] is None


def test_legacy_fallback_uses_gate_thresholds_and_thermal_vote_only():
    summary = night_result()
    del summary['validation']['gate']
    metrics = summary['metrics']
    assert camera_gate(summary, metrics, 'camera_front1')['pass'] is True
    assert camera_gate(summary, metrics, 'camera_front1', defaults={'rgb_rot_deg': .1})['pass'] is False
    metrics['thermal_left']['vote']['pass'] = False
    assert camera_gate(summary, metrics, 'thermal_left')['pass'] is False
    assert camera_gate(summary, metrics, 'thermal_right')['pass'] is True


def test_recorded_gate_does_not_use_local_thresholds_but_explicit_review_can(tmp_path):
    summary = night_result()
    summary['metrics']['camera_front1']['rot_deg'] = .8
    (tmp_path / 'summary.json').write_text(json.dumps(summary))
    by_name = {r['camera']: r for r in oc.load_result(tmp_path)['cameras']}
    assert by_name['camera_front1']['pass'] is True
    by_name = {r['camera']: r for r in oc.load_result(tmp_path, defaults=copy.deepcopy(oc.TOOL_DEFAULTS))['cameras']}
    assert by_name['camera_front1']['pass'] is False
    assert by_name['camera_front1']['informational']


def test_metrics_file_fallback(tmp_path):
    summary = night_result()
    metrics = summary.pop('metrics')
    (tmp_path / 'summary.json').write_text(json.dumps(summary))
    (tmp_path / 'metrics.json').write_text(json.dumps(metrics))
    assert len(oc.load_result(tmp_path)['cameras']) == 16


def test_scene_colours_gate_not_vote_or_uncertainty():
    pytest.importorskip('PyQt5')
    pytest.importorskip('pyqtgraph.opengl')
    from calib_viz.scene import CYAN, state_color
    assert state_color({'gate_pass': True, 'validation_vote': False, 'sigma_rot_deg': 5}) == CYAN
    failed = state_color({'gate_pass': False})
    assert failed[0] > failed[1]
    unknown = state_color({'gate_pass': None, 'validation_vote': False})
    assert unknown != failed and unknown != CYAN
