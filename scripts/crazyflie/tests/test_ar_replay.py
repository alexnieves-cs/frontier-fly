"""ar_replay.py: the camera from the floor markers, and a whole replay of a SYNTHETIC video of a sim flight.

The synthetic video is rendered from the sim log's truth (arsynth.py), so every number the replay
measures from the pixels can be checked against what really happened. Needs OpenCV
(requirements-ar.txt); skipped without it.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import ar_replay  # noqa: E402
import fly  # noqa: E402
from arcam import Camera, detect, detector, fit_focal, intrinsics, load_setup, look_at, marker_sheet, solve_pose  # noqa: E402
from arsynth import default_camera, render_room  # noqa: E402
from flightlog import read_log  # noqa: E402

ARENAS = HERE / 'arenas'
ROOM = ARENAS / 'example-room.json'
SETUP = ARENAS / 'example-room.ar.json'


def test_camera_round_trip():
    K = intrinsics(1280, 720, hfov_deg=70)
    R, t = look_at((0.2, -0.4, 1.8), (2.0, 1.5, 0.0))
    cam = Camera(K, R, t, 1280, 720)
    pts = np.array([[1.0, 1.0, 0.0], [2.5, 2.0, 0.5], [3.1, 0.4, 0.3]])
    uv, depth = cam.project(pts)
    assert (depth > 0).all()
    for p, q in zip(pts, uv):
        assert cam.to_plane(q[None], p[2])[0] == pytest.approx(p[:2], abs=1e-6)
    assert np.allclose(cam.center, (0.2, -0.4, 1.8))
    assert Camera.from_json(cam.to_json()).project(pts)[0] == pytest.approx(uv, abs=1e-4)
    # A ray going down never meets a plane above the camera.
    assert np.isnan(cam.to_plane(uv[:1], 3.0)).all()


def test_focal_and_pose_from_the_markers():
    setup = load_setup(SETUP)
    truth = default_camera(setup, 960, 540, hfov=78)
    img, *_ = render_room(truth, setup, seed=3)
    gray = cv2.cvtColor(np.clip(img * 255, 0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY)   # render_room is 0..1
    world = setup.corners()
    dets = detect(detector(setup), gray, set(world))
    assert len(dets) >= 4
    focal, rms = fit_focal([dets], world, 960, 540)
    assert focal == pytest.approx(truth.focal, rel=0.01)
    R, t, rms = solve_pose(dets, world, intrinsics(960, 540, focal_px=focal))
    assert rms < 1.0
    assert np.linalg.norm(-R.T @ t - truth.center) < 0.02


def test_marker_sheet(tmp_path):
    pages = marker_sheet(load_setup(SETUP), tmp_path / 'markers.png', dpi=100)
    assert pages and all(p.exists() for p in pages)
    # 0.18 m markers at 100 dpi are about 709 px wide: every marker is on a page at its true size.
    setup = load_setup(SETUP)
    ids = {int(m['id']) for m in setup.markers}
    found = set()
    for p in pages:
        g = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        found |= set(detect(detector(setup), g, ids))
    assert found == ids


@pytest.fixture(scope='module')
def replay(tmp_path_factory):
    """A square flight whose flow deck loses its position at 9 s (the guard lands it), filmed by a
    SYNTHETIC phone video, and replayed."""
    d = tmp_path_factory.mktemp('ar')
    log = d / 'square.jsonl'
    fly.main(['--arena', str(ROOM), '--mode', 'square', '--sim', '--yes', '--fault', 'flow_jump_at=9', '--log', str(log)])
    video = d / 'square.mp4'
    assert ar_replay.main(['synth', str(log), '--setup', str(SETUP), '--out', str(video), '--size', '640x360', '--quiet']) == 0
    out = d / 'out'
    assert ar_replay.main(['run', str(video), str(log), '--setup', str(SETUP), '--out', str(out),
                           '--no-video', '--stills', '6.0', '--quiet']) == 0
    side = json.loads((video.parent / (video.name + '.truth.json')).read_text())
    return read_log(log), side, json.loads((out / 'ar.json').read_text()), out


def test_replay_finds_the_camera_and_the_time(replay):
    _, side, ar, _ = replay
    assert ar['synthetic_video'] is True
    chk = ar['self_check']
    assert abs(chk['offset_error_s']) < 1 / side['fps']          # within a frame
    assert abs(chk['focal_error_rel']) < 0.02
    assert chk['camera_position_error_m'] < 0.05


def test_replay_follows_the_drone_not_its_shadow(replay):
    _, _, ar, _ = replay
    chk = ar['self_check']
    assert ar['tracked_share_in_flight'] > 0.95
    assert chk['camera_vs_truth_median_m'] < 0.03
    assert chk['camera_vs_truth_p95_m'] < 0.08


def test_replay_sees_the_jump_the_camera_did_not(replay):
    _, _, ar, _ = replay
    [j] = ar['estimate_jumps']
    assert j['log_s'] == pytest.approx(9.0, abs=0.2)
    assert j['estimate_moved_m'] == pytest.approx(0.79, abs=0.05)
    assert j['camera_moved_m'] < 0.1


def test_replay_measures_where_it_landed(replay):
    log, side, ar, out = replay
    last = [s for s in log.steps if s.get('truth')][-1]['truth']
    L = ar['landed']
    assert math.dist(L['camera_xy'], (last['x'], last['y'])) < 0.05
    assert L['offset_m'] > 0.6                                    # the estimate is still off by the jump
    assert '--landed' in ar['compare_cmd']
    assert len(ar['frames']) == side['frames']
    assert (out / 'track.csv').exists() and (out / 'still-006.00s.png').exists()
