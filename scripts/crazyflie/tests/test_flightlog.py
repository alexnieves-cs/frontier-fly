"""The flight log format (flightlog.py): what fly.py and bringup.py write and compare.py reads."""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flightlog import SCHEMA, VERSION, FlightLog, read_log, validate  # noqa: E402
from guard import Command, Telemetry  # noqa: E402

ARENA = {'name': 'box', 'w': 3.0, 'd': 3.0, 'walls': [], 'boxes': [], 'start': {'x': 1.0, 'y': 1.0, 'yaw': 0.0}}


def write(path, steps=3, close=True):
    log = FlightLog(path, {'source': 'sim', 'mode': 'hover', 'arena': ARENA}, t0=0.0)
    for k in range(steps):
        tel = Telemetry(t=0.05 * k, x=1.0, y=1.0, z=0.4, yaw_deg=0.0, front=math.nan, back=None, left=1.23456789)
        log.step(0.05 * k, 'fly', tel, 0.01, Command(0.1, 0, 0, 0.4), Command(0.1, 0, 0, 0.4), 'fly', '', {'x': 1.0, 'y': 1.0, 'z': 0.4, 'yaw_deg': 0.0})
    log.event(0.2, 'landed', 'mission complete')
    if close:
        log.close(0.25, {'outcome': 'complete', 'reason': 'mission complete'})
    return path


def test_round_trip(tmp_path):
    p = write(tmp_path / 'a.jsonl')
    assert validate(p) == []
    first = json.loads(p.read_text().splitlines()[0])
    assert first['schema'] == SCHEMA and first['version'] == VERSION
    log = read_log(p)
    assert log.source == 'sim' and log.mode == 'hover' and log.arena['name'] == 'box'
    assert len(log.steps) == 3 and [e['event'] for e in log.events] == ['landed']
    assert log.outcome() == ('complete', 'mission complete')
    tel = log.steps[0]['tel']
    # Invalid readings (NaN) and no-return (None) both come back as null; floats keep 4 decimals.
    assert tel['front'] is None and tel['back'] is None and tel['left'] == 1.2346
    assert log.steps[0]['sent']['vx'] == 0.1 and log.steps[0]['truth']['z'] == 0.4


def test_close_is_idempotent_and_survives_a_cut_run(tmp_path):
    p = write(tmp_path / 'b.jsonl', close=False)
    assert 'no end record (the run was cut off)' in validate(p)
    # Still readable: every line was flushed as it was written.
    assert len(read_log(p).steps) == 3
    log = FlightLog(tmp_path / 'c.jsonl', {'source': 'real', 'mode': 'hover', 'arena': ARENA})
    log.close(1.0, {'outcome': 'complete'})
    log.close(2.0)
    assert sum(1 for x in (tmp_path / 'c.jsonl').read_text().splitlines() if '"end"' in x) == 1


def test_validate_catches_bad_logs(tmp_path):
    p = write(tmp_path / 'd.jsonl')
    lines = p.read_text().splitlines()
    step = json.loads(lines[2])
    step['t'] = -1.0
    step['phase'] = 'cruise'
    lines[2] = json.dumps(step)
    p.write_text('\n'.join(lines) + '\n')
    problems = validate(p)
    assert any('time went backwards' in x for x in problems)
    assert any("phase 'cruise'" in x for x in problems)
    bad = tmp_path / 'e.jsonl'
    bad.write_text(json.dumps({'event': 'start', 'schema': SCHEMA, 'version': VERSION, 't': 0, 'meta': {'source': 'drone', 'mode': 'hover'}}) + '\n')
    problems = validate(bad)
    assert any('meta.source' in x for x in problems) and any('meta.arena' in x for x in problems)


def test_reads_logs_from_before_version_1(tmp_path):
    p = tmp_path / 'old.jsonl'
    p.write_text(json.dumps({'t': 0.0, 'tel': None, 'cmd': None, 'verdict': 'land', 'reason': 'no telemetry'}) + '\n'
                 + json.dumps({'event': 'guard-landed', 't': 0.1, 'reason': 'no telemetry'}) + '\n')
    log = read_log(p)
    assert log.meta == {} and log.source == 'unknown'
    assert log.outcome() == ('guard-landed', 'no telemetry')
    assert any('start record' in x for x in validate(p))
