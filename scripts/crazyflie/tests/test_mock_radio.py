"""MockRadioLink: the radio between fly.py and the simulated Crazyflie, so link trouble can be rehearsed without one."""
import json
import math
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import fly  # noqa: E402
from flightlog import read_log, validate  # noqa: E402
from links import MockRadioLink, SimLink  # noqa: E402

ARENA_FILE = HERE / 'arenas' / 'example-room.json'
ARENA = json.loads(ARENA_FILE.read_text())


def fly_sim(tmp_path, *args):
    log, report = tmp_path / 'log.jsonl', tmp_path / 'report.json'
    code = fly.main(['--arena', str(ARENA_FILE), '--sim', '--yes', '--log', str(log), '--report', str(report), *args])
    return code, json.loads(report.read_text()), log


def test_clean_radio_flies_the_square(tmp_path):
    code, r, log = fly_sim(tmp_path, '--mode', 'square', '--mock-radio', 'drop=0.02,latency=0.01')
    assert code == 0, r
    assert r['outcome'] == 'complete' and not r['crashed']
    assert 0 < r['link']['tel_loss'] < 0.1 and r['link']['cmd_delivered'] > 0
    assert validate(log) == []
    assert read_log(log).source == 'mock-radio'


def test_outage_lands_on_stale_telemetry(tmp_path):
    # A 2 s blackout: the guard must land on telemetry age, and the drone must not hit anything.
    code, r, log = fly_sim(tmp_path, '--mode', 'explore', '--mock-radio', 'outage_at=20,outage_sec=2')
    assert code == 1
    assert r['outcome'] == 'guard-landed' and 'telemetry' in r['reason'], r
    assert not r['crashed']


def test_landing_holds_through_an_outage(tmp_path):
    # The radio drops out for 1.2 s in the middle of the landing. The setpoint holds while the drone
    # can't be heard, and the motors stop only once a fresh reading shows touchdown.
    code, r, log = fly_sim(tmp_path, '--mode', 'hover', '--mock-radio', 'outage_at=12.3,outage_sec=1.2')
    assert code == 0 and r['outcome'] == 'complete', r
    lines = [json.loads(x) for x in log.read_text().splitlines()]
    landing = [x for x in lines if x.get('phase') == 'land']
    assert max(x['age'] or 0 for x in landing) > 1.0, 'the outage should fall inside the landing'
    landed = next(x for x in lines if x.get('event') == 'landed')
    assert landed['confirmed'] is True
    assert landing[-1]['truth']['z'] <= fly.TOUCHDOWN_Z + 0.005
    assert not r['crashed']


def test_unknown_setting_is_refused():
    with pytest.raises(ValueError):
        MockRadioLink(SimLink(ARENA), packet_loss=0.1)


def test_watchdog_levels_then_stops():
    radio = MockRadioLink(SimLink(ARENA), seed=1)
    radio.arm()
    radio.send_hover(0.3, 0.0, 0.0, 0.4)
    radio.sleep(0.4)
    assert radio.inner.motors and radio.inner.cmd[0] == 0.3
    radio.sleep(0.3)   # 0.7 s without a setpoint: level out
    assert radio.watchdog == 'level' and radio.inner.cmd[:3] == (0.0, 0.0, 0.0)
    radio.sleep(1.5)   # past 2 s: motors off
    assert radio.watchdog == 'stop' and not radio.inner.motors


def test_a_delivered_stop_disarms_the_watchdog():
    radio = MockRadioLink(SimLink(ARENA), seed=1)
    radio.send_hover(0.0, 0.0, 0.0, 0.4)
    radio.sleep(0.1)
    radio.stop_motors()
    radio.sleep(3.0)
    assert radio.watchdog == '' and not radio.inner.motors


def test_age_counts_from_arrival():
    assert MockRadioLink(SimLink(ARENA), latency=0.02).read() == (None, math.inf)   # nothing has arrived yet
    radio = MockRadioLink(SimLink(ARENA), seed=1, outage_at=1.0, outage_sec=1.0)
    radio.sleep(0.5)
    _, age = radio.read()
    assert age < 0.06
    radio.sleep(0.9)   # 0.4 s into the blackout
    _, age = radio.read()
    assert 0.35 < age < 0.5
    assert radio.stats()['max_gap_s'] == 0.05
