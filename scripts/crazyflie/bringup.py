#!/usr/bin/env python3
"""Bring a Crazyflie up with the props off: find the radio, then measure the link before anything flies.

    python scripts/crazyflie/bringup.py scan
    python scripts/crazyflie/bringup.py link --uri radio://0/80/2M/E7E7E7E7E7
    python scripts/crazyflie/bringup.py link --sim --mock-radio drop=0.1,latency=0.02   # rehearse it

scan   lists the Crazyradio dongles and the Crazyflies that answer on them, with the URI to use.
link   connects and listens for --seconds (20 by default) without ever starting the motors. It checks
       the decks, the resting battery and the rangers, and measures how much telemetry arrives, how old
       it gets and, on cflib 0.1.34, the round-trip latency and link quality. It passes when the link is
       good enough for the guard: the guard lands when telemetry is 0.5 s old, so the worst gap here
       must stay under half that.

Both write nothing to the drone and send no setpoint. link writes a flight log (flightlog.py, mode
"link", every step "ground"/"idle") and, with --report, a JSON summary.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from flightlog import FlightLog  # noqa: E402
from guard import Limits, Telemetry  # noqa: E402
from links import CflibLink, MockRadioLink, SimLink, mm_to_m  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
PERIOD = 0.05
# The bench arena the sim rehearsal sits in: link needs no arena, but the simulated drone needs a room.
REHEARSAL_ARENA = REPO / 'scripts' / 'crazyflie' / 'arenas' / 'example-room.json'


@dataclass
class LinkRules:
    """What a link must show before the first flight. Derived from guard.Limits where they overlap."""
    fresh_age_s: float = 0.1        # telemetry this new or newer counts as fresh (two 50 ms periods)
    min_fresh_share: float = 0.98   # of the samples taken
    max_age_s: float = Limits().telemetry_land_sec / 2   # 0.25 s: half the guard's landing threshold
    max_latency_ms: float = 50.0    # cflib's p95 round trip, when it reports one
    min_link_quality: float = 90.0  # cflib's 0-100 link quality, when it reports one
    rest_voltage: float = Limits().land_voltage + 0.5    # 3.9 V, the same rule as fly.py's preflight


@dataclass
class LinkResult:
    ok: bool = False
    problems: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    samples: int = 0
    fresh_share: float = 0.0
    age_p50_s: float | None = None
    age_p95_s: float | None = None
    age_max_s: float | None = None
    decks: dict = field(default_factory=dict)
    vbat: float | None = None
    rangers_m: dict = field(default_factory=dict)
    radio: dict = field(default_factory=dict)


def percentile(xs: list[float], p: float) -> float | None:
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(p * len(s)))]


def link_check(link, seconds: float = 20.0, rules: LinkRules | None = None, log: FlightLog | None = None) -> LinkResult:
    """Listen to the link for `seconds`. Never arms, never sends a setpoint, never stops the motors."""
    rules = rules or LinkRules()
    res = LinkResult()
    res.decks = link.decks()
    ages: list[float] = []
    last: Telemetry | None = None
    t0 = link.now()
    while link.now() - t0 < seconds:
        raw, age = link.read()
        tel = None
        if raw is not None:
            tel = Telemetry(
                t=raw.t, x=raw.x, y=raw.y, z=raw.z, yaw_deg=raw.yaw_deg, vx=raw.vx, vy=raw.vy,
                roll_deg=raw.roll_deg, pitch_deg=raw.pitch_deg, vbat=raw.vbat,
                front=mm_to_m(raw.front_mm), back=mm_to_m(raw.back_mm), left=mm_to_m(raw.left_mm),
                right=mm_to_m(raw.right_mm), up=mm_to_m(raw.up_mm),
            )
            last = tel
        # Ages count from the first packet on: the wait for the first one is connection set-up, not a gap.
        if last is not None:
            ages.append(age if math.isfinite(age) else seconds)
        if log:
            log.step(link.now(), 'ground', tel, age, None, None, 'idle')
        link.sleep(PERIOD)
    res.samples = len(ages)
    res.fresh_share = round(sum(a <= rules.fresh_age_s for a in ages) / max(1, len(ages)), 4)
    res.age_p50_s, res.age_p95_s = percentile(ages, 0.5), percentile(ages, 0.95)
    res.age_max_s = max(ages) if ages else None
    res.radio = link.stats() if hasattr(link, 'stats') else {}

    if not res.decks.get('flow2'):
        res.problems.append('Flow deck v2 not detected')
    if not res.decks.get('multiranger'):
        res.problems.append('Multi-ranger deck not detected')
    if last is None:
        res.problems.append('no telemetry arrived: check the URI, the channel and that the Crazyflie is on')
    else:
        res.vbat = round(last.vbat, 2)
        res.rangers_m = {k: getattr(last, k) for k in ('front', 'back', 'left', 'right', 'up')}
        if last.vbat < rules.rest_voltage:
            res.warnings.append(f'battery {last.vbat:.2f} V at rest: charge to {rules.rest_voltage:.1f} V before a flight')
        if all(res.rangers_m[k] is None for k in ('front', 'back', 'left', 'right')):
            res.warnings.append('no horizontal ranger sees anything within 4 m: is the Multi-ranger deck working?')
        if last.z > 0.15:
            res.warnings.append(f'height reads {last.z:.2f} m: it should be on the floor (is the Flow deck facing down?)')
    if res.fresh_share < rules.min_fresh_share:
        res.problems.append(f'telemetry fresh on {100 * res.fresh_share:.1f}% of samples, needs {100 * rules.min_fresh_share:.0f}%')
    if res.age_max_s is not None and res.age_max_s > rules.max_age_s:
        res.problems.append(f'telemetry went {res.age_max_s:.2f} s stale, the limit is {rules.max_age_s:.2f} s (the guard lands at {Limits().telemetry_land_sec} s)')
    lat = res.radio.get('latency_ms')
    if lat is not None and lat > rules.max_latency_ms:
        res.problems.append(f'round trip {lat:.0f} ms (p95), the limit is {rules.max_latency_ms:.0f} ms')
    lq = res.radio.get('link_quality')
    if lq is not None and lq < rules.min_link_quality:
        res.problems.append(f'link quality {lq:.0f}, needs {rules.min_link_quality:.0f}: move closer, or try another channel')
    if lat is None:
        res.warnings.append('no round-trip latency reported: only telemetry age was checked')
    res.ok = not res.problems
    return res


def scan(address: str | None = None) -> int:
    try:
        import cflib.crtp
    except ImportError:
        print('cflib is not installed: pip install -r scripts/crazyflie/requirements.txt', file=sys.stderr)
        return 2
    cflib.crtp.init_drivers()
    try:
        status = cflib.crtp.get_interfaces_status()
    except Exception as e:
        status = {'error': str(e)}
    for name, st in status.items():
        print(f'interface {name}: {st}')
    addr = int(address, 16) if address else None
    try:
        found = cflib.crtp.scan_interfaces(addr)
    except Exception as e:
        print(f'scan failed: {e}. On Linux this is usually the Crazyradio udev rules (docs/CRAZYFLIE.md, "Flight ladder").', file=sys.stderr)
        return 1
    if not found:
        print('No Crazyflie answered. Check that it is switched on, the Crazyradio is plugged in, and on Linux '
              'that the udev rules are installed (docs/CRAZYFLIE.md, "Flight ladder").', file=sys.stderr)
        return 1
    for uri, comment in found:
        print(f'found {uri}{f"  ({comment})" if comment else ""}')
    print('Use the radio:// URI with --uri, or export CF_URI=<uri>.')
    return 0


def print_result(r: LinkResult, seconds: float) -> None:
    print(f'link check, {seconds:.0f} s, {r.samples} samples:')
    print(f'  decks: Flow deck v2 {"yes" if r.decks.get("flow2") else "NO"}, Multi-ranger {"yes" if r.decks.get("multiranger") else "NO"}')
    if r.vbat is not None:
        print(f'  battery at rest: {r.vbat:.2f} V')
        print('  rangers (m): ' + ', '.join(f'{k} {"-" if v is None else f"{v:.2f}"}' for k, v in r.rangers_m.items()))
    if r.age_p50_s is not None:
        print(f'  telemetry: fresh {100 * r.fresh_share:.1f}%, age p50 {1000 * r.age_p50_s:.0f} ms, p95 {1000 * r.age_p95_s:.0f} ms, worst {1000 * r.age_max_s:.0f} ms')
    if r.radio:
        print('  radio: ' + ', '.join(f'{k} {v}' for k, v in r.radio.items()))
    for w in r.warnings:
        print(f'  note: {w}')
    if r.ok:
        print('LINK OK. Next: fly.py --mode check (props still off).')
    else:
        print('LINK NOT OK:\n  - ' + '\n  - '.join(r.problems), file=sys.stderr)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sc = sub.add_parser('scan', help='list Crazyradios and the Crazyflies that answer')
    sc.add_argument('--address', default=None, help='radio address in hex, e.g. E7E7E7E7E7 (default: cflib scans the default one)')
    lk = sub.add_parser('link', help='measure the link with the props off; the motors never start')
    lk.add_argument('--uri', default=os.environ.get('CF_URI', 'radio://0/80/2M/E7E7E7E7E7'))
    lk.add_argument('--seconds', type=float, default=20.0)
    lk.add_argument('--sim', action='store_true', help='rehearse against the simulated Crazyflie')
    lk.add_argument('--mock-radio', default=None, metavar='SETTINGS', help='sim only: e.g. drop=0.1,latency=0.02 (see links.MockRadioLink)')
    lk.add_argument('--seed', type=int, default=1)
    lk.add_argument('--log', default=None, help='flight log path (default flights/<time>-link.jsonl)')
    lk.add_argument('--report', default=None, help='write the result as JSON here')
    a = ap.parse_args(argv)

    if a.cmd == 'scan':
        return scan(a.address)

    from fly import git_commit, parse_pairs  # same helpers as the flight script
    if a.mock_radio is not None and not a.sim:
        print('--mock-radio needs --sim', file=sys.stderr)
        return 2
    if a.sim:
        sim = SimLink(json.loads(REHEARSAL_ARENA.read_text()), seed=a.seed)
        link = MockRadioLink(sim, seed=a.seed, **parse_pairs(a.mock_radio, '--mock-radio')) if a.mock_radio is not None else sim
        source = 'mock-radio' if a.mock_radio is not None else 'sim'
    else:
        try:
            link, source = CflibLink(a.uri), 'real'
        except ImportError:
            print('cflib is not installed: pip install -r scripts/crazyflie/requirements.txt', file=sys.stderr)
            return 2
        except Exception as e:  # cflib raises plain Exceptions for a missing radio or a drone that does not answer
            print(f'could not open {a.uri}: {e}\nRun bringup.py scan to find the URI, and check the drone is on.', file=sys.stderr)
            return 1
    log_path = a.log or str(REPO / 'flights' / f"{time.strftime('%Y%m%d-%H%M%S')}-link{'-sim' if a.sim else ''}.jsonl")
    meta = {'source': source, 'mode': 'link', 'seconds': a.seconds, 'started_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'commit': git_commit(), 'rules': LinkRules().__dict__}
    try:
        meta['link'] = link.describe() if hasattr(link, 'describe') else {}
    except Exception as e:
        meta['link'] = {'error': str(e)}
    log = FlightLog(log_path, meta, t0=link.now())
    try:
        r = link_check(link, a.seconds, log=log)
        print_result(r, a.seconds)
        out = {'outcome': 'link-ok' if r.ok else 'link-failed', **r.__dict__}
        if a.report:
            Path(a.report).write_text(json.dumps(out, indent=1) + '\n')
        log.close(link.now(), out)
        return 0 if r.ok else 1
    finally:
        log.close(link.now())
        link.close()


if __name__ == '__main__':
    sys.exit(main())
