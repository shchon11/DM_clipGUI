"""Small replay fixtures; no solver, ROS, live sensors, or reference bag required."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from calib_viz import replay
from calib_viz.stream import validate_manifest, validate_snapshot


@pytest.fixture
def replay_run(tmp_path):
    work, out = tmp_path / 'work', tmp_path / 'out'
    work.mkdir()
    out.mkdir()
    (work / 'lo').mkdir()
    (out / 'intrinsic').mkdir()
    (out / 'extrinsic').mkdir()
    windows = ['S01', 'S02', 'W00', 'W01']
    (out / 'rig.yaml').write_text(yaml.safe_dump({'R_lidar_V': np.eye(3).tolist()}))
    metrics = {}
    for name in replay.CAMERA_NAMES:
        thermal = name.startswith('thermal')
        intr = {'model': 'plumb_bob' if thermal else 'equidistant',
                'image_width': 64, 'image_height': 48,
                'camera_matrix': [[30, 0, 32], [0, 30, 24], [0, 0, 1]],
                'distortion_coefficients': [0] * (5 if thermal else 4)}
        (out / 'intrinsic' / (name + '.yaml')).write_text(yaml.safe_dump(intr))
        (out / 'extrinsic' / (name + '.yaml')).write_text(yaml.safe_dump({'T_cam_lidar': np.eye(4).tolist()}))
        metrics[name] = {'rot_deg': .1, 'pos_mm': 5., 'track_reproj_px': 1.2,
                         'vote': {'pass': False}, 'gate': {'pass': True}}
    (out / 'metrics.json').write_text(json.dumps(metrics))
    summary = {'windows': {'kept': windows}, 'validation': {'gate': {'pass': True}}}
    (out / 'summary.json').write_text(json.dumps(summary))
    events = [{'ev': 'stage_start', 'stage': stage, 't': when}
              for stage, when in [('extract', 0), ('lo', 10), ('rgb_tracks', 20),
                                   ('thermal_solve', 30), ('validation', 40)]]
    events.append({'ev': 'stage_end', 'stage': 'validation', 'ok': True, 't': 50})
    (work / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
    for index, window in enumerate(windows):
        stamps = np.array([1_000_000_000, 1_100_000_000, 1_200_000_000], np.int64)
        poses = np.repeat(np.eye(4)[None], 3, axis=0)
        poses[:, 0, 3] = index * 100 + np.arange(3) * .1
        np.savez(work / 'lo' / ('ref_' + window + '.npz'), tau=stamps, T_w_L=poses)
        lidar = work / 'extract' / window / 'lidar'
        lidar.mkdir(parents=True)
        for stamp in stamps:
            np.save(lidar / (str(stamp) + '.npy'), np.array([[4, 0, 1], [5, 1, 2]], np.float32))
        for name in replay.CAMERA_NAMES:
            thermal = name.startswith('thermal')
            folder = work / 'thermal16' / window / name if thermal else work / 'extract' / window / 'cam' / name
            folder.mkdir(parents=True)
            for stamp in stamps:
                image = np.arange(48 * 64, dtype=np.uint16).reshape(48, 64) if thermal else np.full((48, 64, 3), 100, np.uint8)
                assert cv2.imwrite(str(folder / (str(stamp) + ('.png' if thermal else '.jpg'))), image)
    return work, out, tmp_path / 'viz', windows


def make_producer(replay_run, **kwargs):
    work, out, viz, _ = replay_run
    return replay.ReplayProducer(work, viz, out, **kwargs)


def test_prepare_is_metadata_only_and_exposes_every_window_and_camera(replay_run, monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError('prepare must not decode frames or scan arrays')
    monkeypatch.setattr(replay.cv2, 'imread', forbidden)
    monkeypatch.setattr(replay, '_sample_scan', forbidden)
    producer = make_producer(replay_run)
    manifest = validate_manifest(producer.prepare())
    assert manifest['windows'] == replay_run[3]
    assert manifest['preview_cameras'] == replay.CAMERA_NAMES
    assert not list((replay_run[2] / 'assets').rglob('*.npz'))


def test_windows_have_separate_frames_and_all_camera_previews(replay_run):
    producer = make_producer(replay_run)
    for index, window in enumerate(replay_run[3]):
        snapshot = validate_snapshot(producer.snapshot_at((index + .5) / 4))
        assert snapshot['map_window'] == window
        assert snapshot['map_frame'] == 'vehicle_aligned_window:' + window
        assert set(snapshot['assets']['matching']) == set(replay.CAMERA_NAMES)
        assert all(p['source_window'] == window for p in snapshot['assets']['matching'].values())
        with np.load(replay_run[2] / snapshot['assets']['map']) as arrays:
            assert len(arrays['points']) <= replay.ASSET_STEPS * replay.POINTS_PER_SWEEP
            assert arrays['points'][:, 0].min() >= index * 100
            assert arrays['points'][:, 0].max() < index * 100 + 10


def test_seek_is_deterministic_and_disk_cache_is_bounded(replay_run):
    producer = make_producer(replay_run)
    first = producer.snapshot_at(.12)
    with np.load(replay_run[2] / first['assets']['map']) as arrays:
        first_points = arrays['points'].copy()
    for fraction in np.linspace(0, 1, 14):
        producer.snapshot_at(fraction)
        assert len(producer.assets) <= replay.MAX_ASSET_SETS
        assert len(list((replay_run[2] / 'assets').glob('replay_*'))) <= replay.MAX_ASSET_SETS
    again = producer.snapshot_at(.12)
    with np.load(replay_run[2] / again['assets']['map']) as arrays:
        assert np.array_equal(first_points, arrays['points'])
    final = producer.snapshot_at(1, window='S01')
    assert final['map_window'] == 'S01'
    assert all(np.allclose(c['T_cam_lidar'], np.eye(4)) for c in final['cameras'].values())
    assert final['provenance']['synthetic_intermediate'] is True


def test_extreme_speed_still_publishes_each_window_at_capped_rate(replay_run, monkeypatch):
    now = [0.]
    waits = []
    monkeypatch.setattr(replay.time, 'monotonic', lambda: now[0])
    class Stop:
        def is_set(self):
            return False
        def wait(self, seconds):
            waits.append(seconds)
            now[0] += seconds
            return False
    producer = make_producer(replay_run, speed=1e9)
    producer.run(Stop())
    snapshots = [json.loads(line) for line in (replay_run[2] / 'events.jsonl').read_text().splitlines()]
    assert {s['map_window'] for s in snapshots} == set(replay_run[3])
    assert all(any(s['map_window'] == w and s['map_progress'] > .99 for s in snapshots)
               for w in replay_run[3])
    assert len(snapshots) <= replay.MAX_SNAPSHOTS
    assert all(seconds >= .25 for seconds in waits)
    assert snapshots[-1]['progress'] == 1
    assert [s['seq'] for s in snapshots] == list(range(len(snapshots)))


def test_missing_camera_frames_do_not_reuse_another_windows_image(replay_run):
    producer = make_producer(replay_run)
    before = producer.snapshot_at(.1)
    assert 'thermal_right' in before['assets']['matching']
    folder = replay_run[0] / 'thermal16' / 'W01' / 'thermal_right'
    for path in folder.iterdir():
        path.unlink()
    after = producer.snapshot_at(1)
    assert 'thermal_right' not in after['assets']['matching']
    assert all(asset['source_window'] == 'W01' for asset in after['assets']['matching'].values())


def test_reopening_replay_keeps_retention_bounded(replay_run):
    producer = make_producer(replay_run)
    for fraction in (.1, .3, .6):
        producer.snapshot_at(fraction)
    reopened = make_producer(replay_run)
    reopened.prepare()
    snapshot = reopened.snapshot_at(1)
    assert len(list((replay_run[2] / 'assets').glob('replay_*'))) == replay.MAX_ASSET_SETS
    assert (replay_run[2] / snapshot['assets']['map']).is_file()
