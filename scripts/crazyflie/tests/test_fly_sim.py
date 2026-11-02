"""End to end: fly.py + the guard + planner.ts against the simulated Crazyflie."""
import json
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import fly  # noqa: E402
import links  # noqa: E402

ARENAS = HERE / 'arenas'


def run(tmp_path, *args):
    report = tmp_path / 'report.json'
    code = fly.main(['--sim', '--yes', '--log', str(tmp_path / 'log.jsonl'), '--report', str(report), *args])
    return code, json.loads(report.read_text())


@pytest.mark.parametrize('arena', ['example-room.json', 'two-rooms.json'])
def test_explore_maps_the_arena_without_touching_anything(tmp_path, arena):
    code, r = run(tmp_path, '--arena', str(ARENAS / arena), '--mode', 'explore')
    assert code == 0, r
    assert r['outcome'] == 'complete' and not r['crashed']
    # Narrow gaps (a box 0.4 m off a wall) can bring the prop disc within a few cm; docs/CRAZYFLIE.md says to close them.
    assert r['min_clearance'] > 0.03
    assert r['known'] > 0.8
    # Every logged step went through the guard.
    lines = [json.loads(x) for x in (tmp_path / 'log.jsonl').read_text().splitlines()]
    assert all('verdict' in x or 'event' in x for x in lines)


@pytest.mark.parametrize('mode', ['check', 'hover', 'yaw', 'square'])
def test_ladder_modes(tmp_path, mode):
    code, r = run(tmp_path, '--arena', str(ARENAS / 'example-room.json'), '--mode', mode)
    assert code == 0, r


@pytest.mark.parametrize('fault,reason', [
    ('vbat_drop_at=20', 'battery'),
    ('flow_jump_at=25', 'jumped'),
    ('telemetry_stop_at=15', 'telemetry'),
])
def test_faults_land(tmp_path, fault, reason):
    code, r = run(tmp_path, '--arena', str(ARENAS / 'example-room.json'), '--mode', 'explore', '--fault', fault)
    assert code == 1
    assert r['outcome'] == 'guard-landed' and reason in r['reason'], r
    assert not r['crashed']


def test_missing_deck_never_takes_off(tmp_path):
    code, r = run(tmp_path, '--arena', str(ARENAS / 'example-room.json'), '--mode', 'hover', '--fault', 'decks_missing')
    assert code == 3 and r['outcome'] == 'preflight-failed'


def test_yaw_check_catches_a_flipped_sign(tmp_path, monkeypatch):
    orig = links.SimLink.send_hover
    monkeypatch.setattr(links.SimLink, 'send_hover', lambda self, vx, vy, w, z: orig(self, vx, vy, -w, z))
    code, r = run(tmp_path, '--arena', str(ARENAS / 'example-room.json'), '--mode', 'yaw')
    assert code == 1 and r['yaw_ok'] is False


def test_bad_arena_is_refused(tmp_path):
    bad = tmp_path / 'bad.json'
    bad.write_text(json.dumps({'w': 3, 'd': 3, 'walls': [], 'boxes': [], 'start': {'x': 0.1, 'y': 0.1, 'yaw': 0}}))
    assert fly.main(['--sim', '--yes', '--arena', str(bad), '--mode', 'hover']) == 2
