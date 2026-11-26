"""A failure inside the flight loop lands the drone; it never leaves it in the air.

Before this, an exception between take-off and landing (the planner dying, a reply that is not
JSON, a reply with a missing field, a bug in a mode's step) went straight out of fly.main. The
finally block closed the link without landing, so on the real drone the motors ran on the last
setpoint until the firmware's 2 s watchdog cut them, and it fell.

These tests need no Node: a small Python stand-in plays the planner, and the arena check is
skipped. The sim has no firmware watchdog, so "left in the air" shows here as motors still on and
the true height still near the hover height when fly.main gives up.
"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import fly  # noqa: E402
import links  # noqa: E402
from guard import Command, Limits  # noqa: E402

ARENA = HERE / 'arenas' / 'example-room.json'

# Answers GOOD readings with a hover command, then misbehaves as argv[1] says.
FAKE_PLANNER = r'''
import json, os, sys
mode = sys.argv[1]
GOOD = 20
print(json.dumps({'ready': True}), flush=True)
n = 0
for line in sys.stdin:
    n += 1
    json.loads(line)
    if n <= GOOD or mode == 'fine':
        print(json.dumps({'vx': 0.0, 'vy': 0.0, 'yawRate': 0.0, 'z': 0.5, 'phase': 'explore', 'known': 0.1}), flush=True)
    elif mode == 'dies':
        sys.exit(3)
    elif mode == 'not-json':
        print('TypeError: cannot read properties of undefined', flush=True)
    elif mode == 'missing-field':
        print(json.dumps({'vx': 0.1, 'vy': 0.0, 'phase': 'explore'}), flush=True)
    elif mode == 'wrong-type':
        print(json.dumps({'vx': 'fast', 'vy': 0.0, 'yawRate': 0.0, 'z': 0.5, 'phase': 'explore'}), flush=True)
    elif mode == 'not-an-object':
        print('[0.1, 0.0, 0.0, 0.5]', flush=True)
    elif mode == 'closes-output':
        os.close(1)
        for _ in sys.stdin:
            pass
        sys.exit(0)
    elif mode == 'hangs':
        pass
    elif mode == 'null':
        # JSON.stringify writes NaN as null.
        print(json.dumps({'vx': None, 'vy': 0.0, 'yawRate': 0.0, 'z': 0.5, 'phase': 'explore'}), flush=True)
    elif mode == 'nan':
        print('{"vx": NaN, "vy": 0.0, "yawRate": 0.0, "z": 0.5, "phase": "explore"}', flush=True)
'''


@pytest.fixture
def no_node(tmp_path, monkeypatch):
    """Run fly.main with the Python stand-in planner and no arena check; hand back the sim link."""
    fake = tmp_path / 'fake_planner.py'
    fake.write_text(FAKE_PLANNER)
    monkeypatch.setattr(fly, 'CHECK_ARENA_CMD', [sys.executable, '-I', '-c', 'pass'])
    made = []
    real_make_link = fly.make_link

    def capture(a, arena):
        link, source = real_make_link(a, arena)
        made.append(link)
        return link, source
    monkeypatch.setattr(fly, 'make_link', capture)

    def run(mode, behaviour='fine', timeout=2.0):
        monkeypatch.setattr(fly, 'PLANNER_CMD', [sys.executable, '-I', str(fake), behaviour])
        monkeypatch.setattr(fly, 'PLANNER_TIMEOUT', {'sim': timeout, 'radio': 0.04})
        log, report = tmp_path / 'log.jsonl', tmp_path / 'report.json'
        args = ['--sim', '--yes', '--arena', str(ARENA), '--mode', mode, '--log', str(log), '--report', str(report)]
        try:
            code = fly.main(args)
        finally:
            run.link = made[-1] if made else None
            run.lines = [json.loads(x) for x in log.read_text().splitlines()] if log.exists() else []
            run.report = json.loads(report.read_text()) if report.exists() else {}
        return code
    return run


def assert_landed(run):
    """Motors off, touchdown confirmed by the drone's own height, and truly on the floor."""
    assert run.link.motors is False
    landed = [x for x in run.lines if x.get('event') == 'landed']
    assert landed and landed[-1]['confirmed'] is True, landed
    last = [x for x in run.lines if 'phase' in x][-1]
    assert last['truth']['z'] <= fly.TOUCHDOWN_Z + 0.005
    assert not run.link.crashed


def test_the_stand_in_planner_flies_like_the_real_one(no_node):
    # Sanity check for the stand-in: a planner that keeps answering flies until the time limit.
    code = no_node('hover')
    assert code == 0
    assert_landed(no_node)


@pytest.mark.parametrize('behaviour', ['dies', 'not-json', 'missing-field', 'wrong-type', 'not-an-object', 'closes-output'])
def test_a_broken_planner_lands_the_drone_at_once(no_node, behaviour):
    code = no_node('explore', behaviour)
    assert code == 1
    r = no_node.report
    assert r['outcome'] == 'error-landed', r
    assert 'planner' in r['reason'], r
    assert_landed(no_node)
    error = next(x for x in no_node.lines if x.get('event') == 'error')
    assert 'planner' in error['reason']
    # It lands from where it was, without first hovering out the 2 s stale-command watchdog.
    fly_steps = [x for x in no_node.lines if x.get('phase') == 'fly']
    land_steps = [x for x in no_node.lines if x.get('phase') == 'land']
    assert fly_steps and land_steps
    assert land_steps[0]['t'] - fly_steps[-1]['t'] < 0.2


@pytest.mark.parametrize('behaviour,timeout', [('hangs', 0.02), ('null', 2.0), ('nan', 2.0)])
def test_a_silent_planner_hovers_then_the_guard_lands(no_node, behaviour, timeout):
    # No reply, or one with a non-finite number, is a missed step, not a broken planner: the guard's
    # watchdog hovers after 0.5 s and lands after 2 s, as it always has.
    code = no_node('explore', behaviour, timeout)
    assert code == 1
    r = no_node.report
    assert r['outcome'] == 'guard-landed' and 'planner silent' in r['reason'], r
    assert_landed(no_node)


def test_an_error_in_a_modes_step_lands(no_node, monkeypatch):
    def broken_hold(_seconds):
        def step(tel, t):
            if t > 1.0:
                raise ZeroDivisionError('division by zero')
            return Command(0, 0, 0, None), False
        return step
    monkeypatch.setattr(fly, 'hold', broken_hold)
    code = no_node('hover')
    assert code == 1
    r = no_node.report
    assert r['outcome'] == 'error-landed' and 'ZeroDivisionError' in r['reason'], r
    assert_landed(no_node)


def test_anything_that_ends_the_script_in_the_air_still_lands(no_node, monkeypatch):
    # KeyboardInterrupt stands for anything that is not an Exception (fly.py's Ctrl-C handler means a
    # real Ctrl-C never raises it). The finally block lands before it closes the link.
    def interrupted_hold(_seconds):
        def step(tel, t):
            if t > 1.0:
                raise KeyboardInterrupt
            return Command(0, 0, 0, None), False
        return step
    monkeypatch.setattr(fly, 'hold', interrupted_hold)
    with pytest.raises(KeyboardInterrupt):
        no_node('hover')
    r = no_node.report
    assert r['outcome'] == 'error-landed' and 'KeyboardInterrupt' in r['reason'], r
    assert_landed(no_node)


# ---------------------------------------------------------------- the landing itself fails

def arena():
    return json.loads(ARENA.read_text())


class RadioThatFails(links.SimLink):
    """The sim behind a radio that stops taking hover setpoints once told to. No onboard landing."""

    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self.failing = False

    def send_hover(self, *args):
        if self.failing:
            raise OSError('radio link lost')
        super().send_hover(*args)


class RadioThatFailsWithOnboardLanding(RadioThatFails):
    """... and a firmware that can fly its own descent, as CflibLink.land_onboard asks it to."""

    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self.onboard_calls: list[float] = []

    def land_onboard(self, duration_s):
        self.onboard_calls.append(duration_s)
        self._send((0.0, 0.0, 0.0, 0.0))


def fly_until_it_fails(link):
    flight = fly.Flight(link, arena(), Limits(), None, 0.5)

    def step(tel, t):
        if t > 1.0:
            link.failing = True
            raise RuntimeError('planner went away')
        return Command(0, 0, 0, 0.5), False
    with pytest.raises(RuntimeError):
        flight.run(step, 30)
    assert flight.armed
    return flight


def test_when_the_landing_loop_fails_the_firmware_lands_it():
    link = RadioThatFailsWithOnboardLanding(arena())
    flight = fly_until_it_fails(link)
    assert link.z > 0.4
    assert flight.abort('planner went away') == 'error-landed'
    assert len(link.onboard_calls) == 1 and link.onboard_calls[0] >= 1.0
    assert link.motors is False and not flight.armed
    assert link.truth()['z'] <= fly.TOUCHDOWN_Z


def test_abort_never_raises_even_with_no_way_to_land():
    # A link with no onboard landing and a dead uplink: abort still returns and stops the motors.
    link = RadioThatFails(arena())
    flight = fly_until_it_fails(link)
    assert flight.abort('planner went away') == 'error-landed'
    assert link.motors is False and not flight.armed


def test_cflib_onboard_landing_releases_the_streamed_setpoints_first():
    # The firmware ignores the high-level commander while streamed (hover) setpoints are active, so
    # land_onboard must send notify_setpoint_stop before land, and must not stop the motors itself.
    calls = []
    link = object.__new__(links.CflibLink)
    link.cf = SimpleNamespace(
        commander=SimpleNamespace(
            send_notify_setpoint_stop=lambda *a, **k: calls.append(('notify_setpoint_stop', a, k)),
            send_stop_setpoint=lambda *a, **k: calls.append(('stop_setpoint', a, k)),
        ),
        high_level_commander=SimpleNamespace(land=lambda *a, **k: calls.append(('land', a, k))),
    )
    link.land_onboard(2.5)
    assert calls == [('notify_setpoint_stop', (), {}), ('land', (0.0, 2.5), {'yaw': None})]


def test_a_flight_log_that_cannot_be_written_does_not_stop_the_flight(capsys):
    class FullDisk:
        def step(self, *a, **k):
            raise OSError(28, 'No space left on device')

        def event(self, *a, **k):
            raise OSError(28, 'No space left on device')
    link = links.SimLink(arena())
    flight = fly.Flight(link, arena(), Limits(), FullDisk(), 0.5)
    assert flight.run(fly.hold(2), 30) == 'complete'
    assert link.motors is False and not flight.armed
    assert link.truth()['z'] <= fly.TOUCHDOWN_Z + 0.005
    assert capsys.readouterr().err.count('flight log') == 1
