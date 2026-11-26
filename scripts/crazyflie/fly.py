#!/usr/bin/env python3
"""Fly frontier-fly's explorer on a Crazyflie, or rehearse it in simulation.

    python scripts/crazyflie/fly.py --arena my-room.json --mode check   --uri radio://0/80/2M/E7E7E7E7E7
    python scripts/crazyflie/fly.py --arena my-room.json --mode explore --sim

Modes, in the order docs/CRAZYFLIE.md flies them:
  check    connect, check decks, battery and estimator, print telemetry; motors never start
  hover    take off to 0.4 m, hold 10 s, land
  yaw      hover, turn left 90 deg at 30 deg/s, check the estimate turned left, land
  square   hover, fly a 0.6 m square at 0.2 m/s, land
  explore  the frontier-fly explorer (scripts/crazyflie/planner.ts) maps the arena, then lands

Every command passes through guard.Guard. Ctrl-C lands; a second Ctrl-C cuts
the motors. An error between take-off and touchdown (the planner dying or sending a
reply that is not a command, a bug in a mode) lands the drone too (Flight.abort). A flight log (JSON lines, flightlog.py) is written to --log, in the
same format for the real drone and the sim, so compare.py can put them side by side.

The ladder modes (hover, yaw, square) fly under tighter limits than explore:
0.3 m/s, a 0.6 m height setpoint at most, a 0.9 m ceiling, a 1.5 m fly-away cut
and under a minute in the air (first_flight_limits below).

Before the first real flight, check the radio link with bringup.py, and read
docs/CRAZYFLIE_FIRST_FLIGHT.md.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from flightlog import FlightLog  # noqa: E402
from guard import Command, Guard, Limits, Telemetry, arena_to_body, estimate_to_arena  # noqa: E402
from links import SIM_PROFILES, CflibLink, MockRadioLink, SimLink, mm_to_m  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
DT = 0.05
# Landing (Flight.land): ramp rate, how far the setpoint may lead the measured height, the lowest
# setpoint, the height that counts as touchdown, how fresh a reading must be to count, and the
# time allowed past the ramp before the motors stop without a confirmed touchdown.
LAND_RATE = 0.2       # m/s
LAND_LEAD = 0.15      # m
LAND_FLOOR = 0.05     # m
TOUCHDOWN_Z = 0.08    # m, measured
FRESH_TEL_SEC = 0.3   # s
LAND_GRACE = 3.0      # s

# The planner and the arena check are TypeScript, run through vite-node. Tests swap these for
# stand-ins. How long a step waits for the planner's reply: the sim runs slower than real time.
PLANNER_CMD = ['npx', 'vite-node', 'scripts/crazyflie/planner.ts']
CHECK_ARENA_CMD = ['npx', 'vite-node', 'scripts/crazyflie/check-arena.ts']
PLANNER_TIMEOUT = {'sim': 5.0, 'radio': 0.04}


class PlannerError(RuntimeError):
    """The planner can no longer answer: it exited, closed its pipes, or sent something that is not a reply."""


class Planner:
    """scripts/crazyflie/planner.ts as a child process, one JSON line each way."""

    def __init__(self, arena_path: str):
        cmd = [*PLANNER_CMD, '--', '--arena', arena_path]
        self.p = subprocess.Popen(cmd, cwd=REPO, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.lines: list[str] = []
        self.eof = False
        self.cv = threading.Condition()
        threading.Thread(target=self._pump, daemon=True).start()
        self.sent = 0
        self.taken = 0
        first = self._next(60.0)
        try:
            ready = bool(first) and json.loads(first).get('ready')
        except (ValueError, AttributeError):
            ready = False
        if not ready:
            raise PlannerError(f'planner did not start: {first!r}')

    def _pump(self):
        try:
            for line in self.p.stdout:
                with self.cv:
                    self.lines.append(line)
                    self.cv.notify()
        except (OSError, ValueError):
            pass
        finally:
            with self.cv:
                self.eof = True
                self.cv.notify_all()

    def _next(self, timeout: float) -> str | None:
        """The next line, or None once timeout passes or the planner's output has closed."""
        deadline = time.monotonic() + timeout
        with self.cv:
            while not self.lines and not self.eof:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self.cv.wait(left)
            return self.lines.pop(0) if self.lines else None

    def ask(self, reading: dict, timeout: float) -> dict | None:
        """The reply to this reading, or None if it is not back in time.

        The planner answers every reading in order, so replies are counted: one that arrives after
        its timeout is dropped when the next reading is asked, never handed back as that reading's
        answer. Without the count a single slow step left every later command one step stale, and
        the caller stamped each as fresh, which hid them from the guard's stale-command watchdog.

        Raises PlannerError when no answer can come any more (the process exited, a pipe closed)
        or the answer is not a JSON object. A late answer is only a missed step; a broken planner
        lands the drone (Flight.abort) instead of hovering out the 2 s watchdog first.
        """
        if self.p.poll() is not None:
            raise PlannerError(f'planner exited (code {self.p.returncode})')
        try:
            self.p.stdin.write(json.dumps(reading) + '\n')
            self.p.stdin.flush()
        except (OSError, ValueError) as e:
            raise PlannerError(f'planner input closed ({e})') from e
        self.sent += 1
        deadline = time.monotonic() + timeout
        while True:
            line = self._next(max(0.0, deadline - time.monotonic()))
            if line is None:
                if self.eof:
                    code = self.p.poll()
                    raise PlannerError('planner closed its output' + (f' (exited, code {code})' if code is not None else ''))
                return None
            self.taken += 1
            if self.taken == self.sent:
                try:
                    out = json.loads(line)
                except ValueError as e:
                    raise PlannerError(f'planner reply is not JSON: {line.strip()[:80]!r}') from e
                if not isinstance(out, dict):
                    raise PlannerError(f'planner reply is not an object: {line.strip()[:80]!r}')
                return out

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(5)
        except Exception:
            self.p.kill()


def planner_command(out: dict) -> tuple[Command | None, bool]:
    """A planner reply as (Command, done), the way Flight.run's step_fn returns it.

    An {"error": ...} reply, or a number that is null or not finite (JSON.stringify writes NaN as
    null), is a missed step: (None, False), and the guard hovers, then lands if it goes on. A reply
    with a field missing or of the wrong type means the planner and fly.py disagree on the protocol,
    so it raises PlannerError and the drone lands now.
    """
    if 'error' in out:
        return None, False
    vals = {}
    for key in ('vx', 'vy', 'yawRate', 'z'):
        if key not in out:
            raise PlannerError(f'planner reply has no {key!r}: {json.dumps(out)[:80]}')
        v = out[key]
        if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))):
            raise PlannerError(f'planner reply {key!r} is not a number: {v!r}')
        vals[key] = v
    phase = out.get('phase')
    if not isinstance(phase, str):
        raise PlannerError(f'planner reply has no phase: {json.dumps(out)[:80]}')
    if any(v is None or not math.isfinite(v) for v in vals.values()):
        return None, False
    return Command(vals['vx'], vals['vy'], math.degrees(vals['yawRate']), vals['z']), phase == 'done'


# Seconds in the air for each first-flight mode before the guard lands it (take-off included).
FIRST_FLIGHT_SEC = {'hover': 40.0, 'yaw': 40.0, 'square': 60.0}


def first_flight_limits(mode: str, max_time: float) -> Limits:
    """The guard's limits for this mode. explore keeps the defaults; the ladder modes fly tighter.

    hover, yaw and square never need more than 0.2 m/s or 0.4 m of height, so a fault that sends the
    drone faster or higher is caught sooner than it would be in explore.
    """
    if mode not in FIRST_FLIGHT_SEC:
        return Limits(max_flight_sec=max_time + 30)
    return Limits(max_speed=0.3, max_height=0.6, ceiling=0.9, flyaway=1.5, max_flight_sec=FIRST_FLIGHT_SEC[mode])


class Flight:
    def __init__(self, link, arena: dict, limits: Limits, log: FlightLog | None, height: float):
        self.link = link
        self.arena = arena
        self.guard = Guard(arena['w'], arena['d'], limits)
        self.log = log
        self.height = height
        self.stop_requested = 0
        self.outcome = 'aborted'
        self.reason = ''
        self.last_tel: Telemetry | None = None
        self.last_age = math.inf
        # True from the arming request until the motors are stopped or cut: main lands any flight
        # that ends with this still set.
        self.armed = False
        self.log_failed = False

    def telemetry(self) -> tuple[Telemetry | None, float]:
        raw, age = self.link.read()
        self.last_age = age
        if raw is None:
            return None, age
        x, y, yaw = estimate_to_arena(raw.x, raw.y, raw.yaw_deg, self.arena['start'])
        c, s = math.cos(self.arena['start']['yaw']), math.sin(self.arena['start']['yaw'])
        tel = Telemetry(
            t=raw.t, x=x, y=y, z=raw.z, yaw_deg=yaw, vx=c * raw.vx - s * raw.vy, vy=s * raw.vx + c * raw.vy,
            roll_deg=raw.roll_deg, pitch_deg=raw.pitch_deg, vbat=raw.vbat,
            front=mm_to_m(raw.front_mm), back=mm_to_m(raw.back_mm), left=mm_to_m(raw.left_mm),
            right=mm_to_m(raw.right_mm), up=mm_to_m(raw.up_mm),
        )
        self.last_tel = tel
        return tel, age

    def write(self, event: str, reason: str = '', **extra):
        if self.log:
            try:
                self.log.event(self.link.now(), event, reason, **extra)
            except Exception as e:
                self._log_error(e)

    def record(self, phase: str, tel: Telemetry | None, cmd: Command | None, sent: Command | None, verdict: str, reason: str = ''):
        """One control step in the flight log: what was measured, asked for, let through and why."""
        if self.log:
            try:
                truth = self.link.truth() if hasattr(self.link, 'truth') else None
                self.log.step(self.link.now(), phase, tel, self.last_age, cmd, sent, verdict, reason, truth)
            except Exception as e:
                self._log_error(e)

    def _log_error(self, e: Exception):
        # A full disk must not end a flight: losing the log is better than losing the landing.
        if not self.log_failed:
            self.log_failed = True
            print(f'WARNING: cannot write the flight log ({e}); flying on without it', file=sys.stderr)

    def stop_motors(self):
        self.link.stop_motors()
        self.armed = False

    def kill(self):
        try:
            self.link.kill()
        finally:
            self.armed = False

    def preflight(self) -> list[str]:
        problems = []
        decks = self.link.decks()
        if not decks.get('flow2'):
            problems.append('Flow deck v2 not detected')
        if not decks.get('multiranger'):
            problems.append('Multi-ranger deck not detected')
        self.link.reset_estimator()
        for _ in range(40):
            tel, _age = self.telemetry()
            if tel:
                break
            self.link.sleep(DT)
        tel, _age = self.telemetry()
        if tel is None:
            return problems + ['no telemetry']
        if tel.vbat < self.guard.limits.land_voltage + 0.5:
            problems.append(f'battery {tel.vbat:.2f} V: charge to at least {self.guard.limits.land_voltage + 0.5:.1f} V')
        if tel.z > 0.15:
            problems.append(f'height reads {tel.z:.2f} m on the ground (is it on the floor? flow deck ok?)')
        for name in ('front', 'back', 'left', 'right'):
            r = getattr(tel, name)
            if r is not None and r < 0.35:
                problems.append(f'{name} ranger reads {r:.2f} m: take off at least 0.4 m from everything')
        if math.hypot(tel.x - self.arena['start']['x'], tel.y - self.arena['start']['y']) > 0.05:
            problems.append('estimator did not start at the take-off point')
        return problems

    def send(self, cmd: Command, tel: Telemetry):
        bx, by = arena_to_body(cmd.vx, cmd.vy, tel.yaw_deg)
        self.link.send_hover(bx, by, cmd.yaw_rate, cmd.z)

    def land(self, reason: str):
        """Bring the height setpoint down at 0.2 m/s and cut the motors once the drone reads touchdown.

        The setpoint never runs more than LAND_LEAD below the last fresh height reading, and it holds
        while telemetry is stale, so a drone that lags its setpoint, or a radio that drops the
        setpoints, is not cut off in the air when the ramp runs out. If no fresh reading under
        TOUCHDOWN_Z arrives by the ramp time plus LAND_GRACE, the motors stop anyway and the
        landed event says so (confirmed: false).
        """
        self.reason = self.reason or reason
        tel = self.last_tel
        z = tel.z if tel else 0.3
        measured = tel.z if tel and self.last_age <= FRESH_TEL_SEC else None
        deadline = self.link.now() + max(z, 0.0) / LAND_RATE + LAND_GRACE
        confirmed = False
        while True:
            if measured is not None and measured <= TOUCHDOWN_Z:
                confirmed = True
                break
            if self.link.now() >= deadline:
                break
            if measured is not None:
                z = max(z - LAND_RATE * DT, measured - LAND_LEAD, LAND_FLOOR)
            if self.last_tel:
                self.link.send_hover(0.0, 0.0, 0.0, z)
            self.link.sleep(DT)
            fresh, age = self.telemetry()
            self.record('land', fresh, None, Command(0.0, 0.0, 0.0, z), 'landing', self.reason)
            measured = fresh.z if fresh is not None and age <= FRESH_TEL_SEC else None
        self.stop_motors()
        self.write('landed', self.reason, confirmed=confirmed, height=measured)

    def abort(self, reason: str) -> str:
        """Land after an error in the flight loop, whatever failed. Never raises.

        First the ordinary landing (land). If that fails too, the link's onboard landing where it
        has one (CflibLink.land_onboard: the firmware's high-level commander flies the descent),
        then the motors stop. With no way to land at all the motors stop where it is, which is
        still better than leaving them on the last setpoint until the firmware's watchdog cuts them.
        """
        self.outcome, self.reason = 'error-landed', reason
        self.write('error', reason)
        if self.armed:
            try:
                self.land(reason)
            except Exception as e:
                self.write('error', f'landing failed: {type(e).__name__}: {e}')
                print(f'ERROR: landing failed ({e})', file=sys.stderr)
                self._land_without_the_loop()
        self.write(self.outcome, self.reason)
        return self.outcome

    def _land_without_the_loop(self):
        onboard = getattr(self.link, 'land_onboard', None)
        if onboard:
            z = self.last_tel.z if self.last_tel and math.isfinite(self.last_tel.z) else 0.5
            duration = max(1.0, min(z, 1.5) / LAND_RATE)
            try:
                onboard(duration)
                self.write('onboard-landing', self.reason, duration=duration)
                self.link.sleep(duration + 0.5)
            except Exception as e:
                self.write('error', f'onboard landing failed: {type(e).__name__}: {e}')
        try:
            self.stop_motors()
        except Exception as e:
            self.write('error', f'could not stop the motors: {type(e).__name__}: {e}')
            print(f'ERROR: could not stop the motors ({e}); the firmware cuts them 2 s after the last setpoint', file=sys.stderr)

    def run(self, step_fn, max_sec: float) -> str:
        """step_fn(tel, t_air) -> (Command | None, done: bool). Takes off, flies, lands.

        An exception from step_fn or the link goes out to the caller with the drone still in the
        air (self.armed): the caller lands it with abort, as main does.
        """
        self.armed = True
        self.link.arm()
        t0 = self.link.now()
        # Take-off: ramp the height setpoint so the flow deck sees the floor the whole way.
        while True:
            tel, age = self.telemetry()
            t_air = self.link.now() - t0
            target = min(self.height, 0.1 + 0.3 * t_air)
            v = self.guard.check(tel or self.last_tel, Command(0, 0, 0, target), 0.0, age) if (tel or self.last_tel) else None
            if v is not None:
                self.record('takeoff', tel, Command(0, 0, 0, target), v.command, v.action, v.reason)
            if v is None or v.action in ('land', 'kill'):
                return self.finish(v)
            self.send(v.command, tel or self.last_tel)
            self.link.sleep(DT)
            if target >= self.height and tel and tel.z > self.height * 0.8:
                break
            if t_air > 8:
                self.land('take-off did not reach height')
                return 'aborted'
        t1 = self.link.now()
        last_cmd, last_cmd_t = None, -math.inf
        while True:
            if self.stop_requested:
                self.land('operator')
                self.outcome = 'operator'
                return self.outcome
            tel, age = self.telemetry()
            now = self.link.now()
            if tel is not None:
                cmd, done = step_fn(tel, now - t1)
                if done:
                    self.land('mission complete')
                    self.outcome = 'complete'
                    return self.outcome
                if cmd is not None:
                    last_cmd, last_cmd_t = cmd, now
            v = self.guard.check(tel or self.last_tel, last_cmd, now - last_cmd_t, age)
            self.record('fly', tel, last_cmd, v.command, v.action, v.reason)
            if v.action in ('land', 'kill'):
                return self.finish(v)
            self.send(v.command, tel or self.last_tel)
            if now - t1 > max_sec:
                self.land('time limit')
                self.outcome = 'complete'
                return self.outcome
            self.link.sleep(DT)

    def finish(self, v) -> str:
        if v is None:
            self.stop_motors()
            self.outcome, self.reason = 'aborted', 'no telemetry at take-off'
        elif v.action == 'kill':
            self.kill()
            self.outcome, self.reason = 'killed', v.reason
        else:
            self.outcome = 'guard-landed'
            self.land(v.reason)
        self.write(self.outcome, self.reason)
        return self.outcome


def hold(seconds):
    def step(tel, t):
        return Command(0, 0, 0, None), t > seconds
    return step


def parse_pairs(text: str | None, what: str) -> dict[str, float]:
    """'drop=0.1,latency=0.03' -> {'drop': 0.1, 'latency': 0.03}."""
    out: dict[str, float] = {}
    for part in (text or '').split(','):
        if not part.strip():
            continue
        k, sep, v = part.partition('=')
        try:
            out[k.strip()] = float(v)
        except ValueError:
            raise SystemExit(f'{what}: expected name=number, got {part!r}')
        if not sep:
            raise SystemExit(f'{what}: expected name=number, got {part!r}')
    return out


def git_commit() -> str | None:
    try:
        r = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=REPO, capture_output=True, text=True, timeout=5)
        return r.stdout.strip() or None
    except Exception:
        return None


def make_link(a, arena: dict):
    """The real radio, the simulated Crazyflie, or the simulated one behind a mock radio."""
    if not a.sim:
        return CflibLink(a.uri), 'real'
    faults = {}
    for f in a.fault:
        k, _, v = f.partition('=')
        faults[k] = True if v in ('', 'true') else float(v)
    sim = SimLink(arena, seed=a.seed, faults=faults, profile=a.sim_profile, **parse_pairs(a.sim_set, '--sim-set'))
    if a.mock_radio is None:
        return sim, 'sim'
    return MockRadioLink(sim, seed=a.seed, **parse_pairs(a.mock_radio, '--mock-radio')), 'mock-radio'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arena', required=True)
    ap.add_argument('--mode', choices=['check', 'hover', 'yaw', 'square', 'explore'], required=True)
    ap.add_argument('--uri', default=os.environ.get('CF_URI', 'radio://0/80/2M/E7E7E7E7E7'))
    ap.add_argument('--sim', action='store_true', help='fly the simulated Crazyflie instead of the radio')
    ap.add_argument('--sim-profile', choices=sorted(SIM_PROFILES), default='ideal',
                    help='sim only: ideal (default) or bench, the model behind npm run crazyflie:bench')
    ap.add_argument('--sim-set', default=None, help='sim only: override profile values, e.g. latency=0.2,tau=0.5')
    ap.add_argument('--mock-radio', default=None, metavar='SETTINGS',
                    help='sim only: put a lossy radio in between, e.g. drop=0.1,latency=0.02,outage_at=20,outage_sec=1')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--fault', action='append', default=[], help='sim only: name=seconds, e.g. vbat_drop_at=20')
    ap.add_argument('--height', type=float, default=0.5)
    ap.add_argument('--max-time', type=float, default=180.0)
    ap.add_argument('--log', default=None, help='flight log path (default flights/<time>-<mode>.jsonl)')
    ap.add_argument('--report', default=None, help='write a JSON summary here')
    ap.add_argument('--yes', action='store_true', help='skip the take-off prompt (sim and CI)')
    a = ap.parse_args(argv)

    arena = json.loads(Path(a.arena).read_text())
    errs = subprocess.run(
        [*CHECK_ARENA_CMD, '--', str(Path(a.arena).resolve())],
        cwd=REPO, capture_output=True, text=True,
    )
    if errs.returncode != 0:
        print(errs.stdout + errs.stderr, file=sys.stderr)
        return 2

    if not a.sim and (a.mock_radio is not None or a.sim_set or a.fault or a.sim_profile != 'ideal'):
        print('--mock-radio, --sim-profile, --sim-set and --fault need --sim', file=sys.stderr)
        return 2
    link, source = make_link(a, arena)
    log_path = a.log or str(REPO / 'flights' / f"{time.strftime('%Y%m%d-%H%M%S')}-{a.mode}{'-sim' if a.sim else ''}.jsonl")
    limits = first_flight_limits(a.mode, a.max_time)
    height = a.height if a.mode == 'explore' else 0.4
    meta = {
        'source': source, 'mode': a.mode, 'arena': arena, 'arena_file': Path(a.arena).name,
        'height': height, 'max_time': a.max_time, 'limits': asdict(limits), 'dt': DT,
        'started_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), 'commit': git_commit(),
    }
    if a.sim:
        meta.update(seed=a.seed, faults=a.fault)
    try:
        meta['link'] = link.describe() if hasattr(link, 'describe') else {}
    except Exception as e:  # a describe() problem must never stop a flight
        meta['link'] = {'error': str(e)}
    flight = Flight(link, arena, limits, FlightLog(log_path, meta, t0=link.now()), height)

    def on_sigint(_s, _f):
        flight.stop_requested += 1
        if flight.stop_requested >= 2:
            link.kill()
            print('\nMOTORS CUT', file=sys.stderr)
            os._exit(1)
        print('\nlanding (Ctrl-C again cuts the motors)', file=sys.stderr)
    signal.signal(signal.SIGINT, on_sigint)

    planner = None
    result = {'mode': a.mode, 'sim': a.sim}
    try:
        problems = flight.preflight()
        tel = flight.last_tel
        print(f'preflight: battery {tel.vbat:.2f} V, height {tel.z:.2f} m, rangers F{tel.front} B{tel.back} L{tel.left} R{tel.right} U{tel.up}' if tel else 'preflight: no telemetry')
        if problems:
            print('NOT FLYING:\n  - ' + '\n  - '.join(problems), file=sys.stderr)
            result.update(outcome='preflight-failed', problems=problems)
            return 3
        if a.mode == 'check':
            for _ in range(int(5 / DT)):
                t, _ = flight.telemetry()
                flight.record('ground', t, None, None, 'idle')
                if t and abs(t.t * 2 - round(t.t * 2)) < DT / 2 + 1e-9:
                    print(f'  x {t.x:5.2f} y {t.y:5.2f} z {t.z:4.2f} yaw {t.yaw_deg:6.1f}  F {t.front} B {t.back} L {t.left} R {t.right}  {t.vbat:.2f} V')
                link.sleep(DT)
            result.update(outcome='checked')
            return 0
        if not a.sim and not a.yes:
            input(f'Ready to fly "{a.mode}" in {arena.get("name", a.arena)}. Props on, area clear, hand on Ctrl-C. Press Enter to take off. ')

        if a.mode == 'hover':
            step = hold(10)
        elif a.mode == 'yaw':
            yaw0 = {}

            def step(tel, t):
                yaw0.setdefault('v', tel.yaw_deg)
                turned = (tel.yaw_deg - yaw0['v'] + 180.0) % 360.0 - 180.0
                result['yaw_turned_deg'] = round(turned, 1)
                if t > 2 and turned < -10:
                    # It turned right when asked to turn left: the yaw sign is wrong. Stop now.
                    flight.reason = 'yaw sign check failed: asked left, turned right'
                    return Command(0, 0, 0, None), True
                return Command(0, 0, 30.0 if t < 3 else 0.0, None), t > 5
        elif a.mode == 'square':
            s0 = {}
            corners = [(0.6, 0), (0.6, 0.6), (0, 0.6), (0, 0)]

            def step(tel, t):
                s0.setdefault('p', (tel.x, tel.y, math.radians(tel.yaw_deg)))
                x0, y0, h = s0['p']
                i = s0.get('i', 0)
                if i >= len(corners):
                    return Command(0, 0, 0, None), True
                cx, cy = corners[i]
                gx, gy = x0 + math.cos(h) * cx - math.sin(h) * cy, y0 + math.sin(h) * cx + math.cos(h) * cy
                dx, dy = gx - tel.x, gy - tel.y
                d = math.hypot(dx, dy)
                if d < 0.08:
                    s0['i'] = i + 1
                sp = min(0.2, d)
                return Command(dx / max(d, 1e-6) * sp, dy / max(d, 1e-6) * sp, 0, None), False
        else:
            planner = Planner(str(Path(a.arena).resolve()))
            timeout = PLANNER_TIMEOUT['sim' if a.sim else 'radio']

            def step(tel, t):
                out = planner.ask({
                    't': t, 'x': tel.x, 'y': tel.y, 'z': tel.z, 'yaw': math.radians(tel.yaw_deg),
                    'vx': tel.vx, 'vy': tel.vy, 'front': tel.front, 'back': tel.back,
                    'left': tel.left, 'right': tel.right, 'up': tel.up,
                }, timeout)
                if out is None or 'error' in out:
                    return None, False
                result['known'] = out.get('known')
                return planner_command(out)

        def wrapped(tel, t):
            cmd, done = step(tel, t)
            if cmd is not None and cmd.z is None:
                cmd.z = flight.height
            return cmd, done

        try:
            outcome = flight.run(wrapped, a.max_time)
        except Exception as e:
            why = str(e) if isinstance(e, PlannerError) else f'{type(e).__name__} in the flight loop: {e}'
            print(f'ERROR: {why}: landing', file=sys.stderr)
            outcome = flight.abort(why)
        result.update(outcome=outcome, reason=flight.reason, events=flight.guard.events[-5:])
        if a.sim:
            result.update(crashed=link.crashed, min_clearance=round(link.min_clearance, 3), seconds=round(link.t, 1))
        if hasattr(link, 'stats'):
            result['link'] = link.stats()
        print(json.dumps(result))
        if a.mode == 'yaw':
            # Asked to turn 90 deg left (counter-clockwise): the estimate must agree, or the yaw sign is wrong.
            result['yaw_ok'] = result.get('yaw_turned_deg', 0) >= 60
        ok = outcome in ('complete', 'checked') and not result.get('crashed') and result.get('yaw_ok', True)
        return 0 if ok else 1
    finally:
        # Whatever ended the script (an exception that is not an Exception, a bug above), it does
        # not close the link with the motors on: a closed link means no setpoints, and the
        # firmware cuts the motors 2 s later wherever the drone is.
        if flight.armed:
            exc = sys.exc_info()[1]
            why = f'{type(exc).__name__} with the motors on' if exc else 'the flight ended with the motors on'
            print(f'ERROR: {why}: landing', file=sys.stderr)
            flight.abort(why)
            result.update(outcome=flight.outcome, reason=flight.reason)
        if planner:
            planner.close()
        if a.report:
            Path(a.report).write_text(json.dumps(result, indent=1) + '\n')
        if flight.log:
            flight.log.close(link.now(), result)
        link.close()


if __name__ == '__main__':
    sys.exit(main())
