"""analysis.py and compare.py: the measures behind the sim-versus-real report, checked on sim flights."""
import json
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
import compare  # noqa: E402
import fly  # noqa: E402
from analysis import coverage, fit_response, flight_metrics, ranger_noise, reachable_cells  # noqa: E402
from flightlog import read_log  # noqa: E402

ARENAS = HERE / 'arenas'
ROOM = ARENAS / 'example-room.json'


def fly_log(tmp_path, name, *args):
    path = tmp_path / f'{name}.jsonl'
    fly.main(['--arena', str(ROOM), '--sim', '--yes', '--log', str(path), *args])
    return path


def test_response_fit_recovers_the_sim_settings(tmp_path):
    fit = fit_response(read_log(fly_log(tmp_path, 'slow', '--mode', 'square', '--sim-set', 'tau=0.6,latency=0.2')))
    assert fit is not None
    assert fit['tau_s'] == pytest.approx(0.6, abs=0.03)
    assert fit['latency_s'] == pytest.approx(0.2, abs=0.02)
    # A hover never moves, so there is nothing to fit.
    assert fit_response(read_log(fly_log(tmp_path, 'hover', '--mode', 'hover'))) is None


def test_ranger_noise_recovers_sigma(tmp_path):
    noise = ranger_noise(read_log(fly_log(tmp_path, 'noisy', '--mode', 'hover', '--sim-set', 'sigma=0.05')))
    assert noise['all'] == pytest.approx(0.05, abs=0.01)


def test_metrics_of_a_sim_flight(tmp_path):
    m = flight_metrics(read_log(fly_log(tmp_path, 'sq', '--mode', 'square')))
    assert m['outcome'] == 'complete' and m['source'] == 'sim'
    assert m['path_m'] == pytest.approx(2.4, abs=0.3)          # a 0.6 m square
    assert m['landing_offset_m'] < 0.1
    # The ideal sim has no flow drift, so estimate and truth agree.
    assert m['coverage'] == m['coverage_truth'] and m['closest_pass_m'] == m['closest_pass_truth_m']
    assert m['verdicts']['fly'] > 100 and m['guard_stops'] == {}


@pytest.mark.parametrize('arena', ['example-room.json', 'two-rooms.json'])
def test_coverage_matches_the_typescript_sim(arena):
    # With no drift the estimate is the true track, so analysis.py must score it exactly as sim.ts does.
    r = subprocess.run(['npx', 'vite-node', 'scripts/crazyflie/replay.ts', '--', '--arena', str(ARENAS / arena),
                        '--seeds', '2', '--drift', '0', '--max-sec', '30', '--steps'], cwd=REPO, capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout.strip().splitlines()[-1])
    a = json.loads((ARENAS / arena).read_text())
    reach = reachable_cells(a)
    assert reach.count == out['reachable']
    for f in out['flights']:
        assert 0.3 < f['coverage'] < 1.0
        assert coverage(a, [tuple(p) for p in f['steps']], reach) == pytest.approx(f['coverage'], abs=0.005)


def test_report_agrees_with_itself(tmp_path):
    # Flown on the same bench profile compare.py replays with, but another seed: nothing should stand out.
    log = fly_log(tmp_path, 'square', '--mode', 'square', '--sim-profile', 'bench', '--seed', '7')
    assert compare.main([str(log), '--seeds', '2']) == 0
    out = tmp_path / 'square-report'
    data = json.loads((out / 'report.json').read_text())
    assert data['disagreements'] == [] and len(data['sim_flights']) == 2
    md = (out / 'report.md').read_text()
    assert 'dry run' in md and 'Nothing falls outside' in md
    assert (out / 'track.svg').read_text().startswith('<svg')


def test_report_flags_a_slow_drone_and_measured_drift(tmp_path):
    log = fly_log(tmp_path, 'slow', '--mode', 'square', '--sim-set', 'tau=0.7,latency=0.25')
    assert compare.main([str(log), '--seeds', '2', '--landed', '1.3,1.3', '--out', str(tmp_path / 'rep')]) == 0
    flags = json.loads((tmp_path / 'rep' / 'report.json').read_text())['disagreements']
    assert any('velocity lag fits 0.7' in f for f in flags)
    assert any('command latency' in f for f in flags)
    assert any('Flow drift' in f for f in flags)
    # Told about the slow drone, the sim agrees with it again.
    assert compare.main([str(log), '--seeds', '2', '--sim-set', 'tau=0.7,latency=0.25', '--out', str(tmp_path / 'rep2')]) == 0
    flags = json.loads((tmp_path / 'rep2' / 'report.json').read_text())['disagreements']
    assert not any(f.startswith('Response') for f in flags), flags


def test_report_flags_a_lossy_radio(tmp_path):
    log = fly_log(tmp_path, 'lossy', '--mode', 'hover', '--mock-radio', 'drop=0.5')
    assert compare.main([str(log), '--seeds', '1']) == 0
    flags = json.loads((tmp_path / 'lossy-report' / 'report.json').read_text())['disagreements']
    assert any(f.startswith('Radio:') for f in flags)


def test_refuses_logs_it_cannot_compare(tmp_path):
    check = fly_log(tmp_path, 'check', '--mode', 'check')
    assert compare.main([str(check)]) == 2
    old = tmp_path / 'old.jsonl'
    old.write_text(json.dumps({'t': 0.0, 'verdict': 'land'}) + '\n')
    assert compare.main([str(old)]) == 2
