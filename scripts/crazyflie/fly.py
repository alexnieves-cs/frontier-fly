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
the motors. A flight log (JSON lines) is written to --log.
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
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from guard import Command, Guard, Limits, Telemetry, arena_to_body, estimate_to_arena  # noqa: E402
from links import CflibLink, SimLink, mm_to_m  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
DT = 0.05


class Planner:
    """scripts/crazyflie/planner.ts as a child process, one JSON line each way."""

    def __init__(self, arena_path: str):
        cmd = ['npx', 'vite-node', 'scripts/crazyflie/planner.ts', '--', '--arena', arena_path]
        self.p = subprocess.Popen(cmd, cwd=REPO, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.lines: list[str] = []
        self.cv = threading.Condition()
        threading.Thread(target=self._pump, daemon=True).start()
        self.sent = 0
        self.taken = 0
        first = self._next(60.0)
        if not first or not json.loads(first).get('ready'):
            raise RuntimeError(f'planner did not start: {first!r}')

    def _pump(self):
        for line in self.p.stdout:
            with self.cv:
                self.lines.append(line)
                self.cv.notify()

    def _next(self, timeout: float) -> str | None:
        with self.cv:
            if not self.lines:
                self.cv.wait(timeout)
            return self.lines.pop(0) if self.lines else None

    def ask(self, reading: dict, timeout: float) -> dict | None:
        """The reply to this reading, or None if it is not back in time.

        The planner answers every reading in order, so replies are counted: one that arrives after
        its timeout is dropped when the next reading is asked, never handed back as that reading's
        answer. Without the count a single slow step left every later command one step stale, and
        the caller stamped each as fresh, which hid them from the guard's stale-command watchdog.
        """
        self.p.stdin.write(json.dumps(reading) + '\n')
        self.p.stdin.flush()
        self.sent += 1
        deadline = time.monotonic() + timeout
        while True:
            line = self._next(max(0.0, deadline - time.monotonic()))
            if line is None:
                return None
            self.taken += 1
            if self.taken == self.sent:
                return json.loads(line)

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(5)
        except Exception:
            self.p.kill()


class Flight:
    def __init__(self, link, arena: dict, limits: Limits, log_path: str | None, height: float):
        self.link = link
        self.arena = arena
        self.guard = Guard(arena['w'], arena['d'], limits)
        self.log = open(log_path, 'w') if log_path else None
        self.height = height
        self.stop_requested = 0
        self.outcome = 'aborted'
        self.reason = ''
        self.last_tel: Telemetry | None = None

    def telemetry(self) -> tuple[Telemetry | None, float]:
        raw, age = self.link.read()
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

    def write(self, **rec):
        if self.log:
            self.log.write(json.dumps(rec, default=lambda o: o.__dict__) + '\n')

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
        self.reason = self.reason or reason
        tel = self.last_tel
        z = tel.z if tel else 0.3
        while z > 0.08:
            z -= 0.2 * DT
            if tel:
                self.link.send_hover(0.0, 0.0, 0.0, max(z, 0.05))
            self.link.sleep(DT)
            tel, _ = self.telemetry()
            tel = tel or self.last_tel
        self.link.stop_motors()
        self.write(t=self.link.now(), event='landed', reason=self.reason)

    def run(self, step_fn, max_sec: float) -> str:
        """step_fn(tel, t_air) -> (Command | None, done: bool). Takes off, flies, lands."""
        self.link.arm()
        t0 = self.link.now()
        # Take-off: ramp the height setpoint so the flow deck sees the floor the whole way.
        while True:
            tel, age = self.telemetry()
            t_air = self.link.now() - t0
            target = min(self.height, 0.1 + 0.3 * t_air)
            v = self.guard.check(tel or self.last_tel, Command(0, 0, 0, target), 0.0, age) if (tel or self.last_tel) else None
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
            self.write(t=now, tel=tel, cmd=last_cmd, verdict=v.action, reason=v.reason)
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
            self.link.stop_motors()
            self.outcome, self.reason = 'aborted', 'no telemetry at take-off'
        elif v.action == 'kill':
            self.link.kill()
            self.outcome, self.reason = 'killed', v.reason
        else:
            self.outcome = 'guard-landed'
            self.land(v.reason)
        self.write(t=self.link.now(), event=self.outcome, reason=self.reason)
        return self.outcome


def hold(seconds):
    def step(tel, t):
        return Command(0, 0, 0, None), t > seconds
    return step


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arena', required=True)
    ap.add_argument('--mode', choices=['check', 'hover', 'yaw', 'square', 'explore'], required=True)
    ap.add_argument('--uri', default=os.environ.get('CF_URI', 'radio://0/80/2M/E7E7E7E7E7'))
    ap.add_argument('--sim', action='store_true', help='fly the simulated Crazyflie instead of the radio')
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
        ['npx', 'vite-node', 'scripts/crazyflie/check-arena.ts', '--', str(Path(a.arena).resolve())],
        cwd=REPO, capture_output=True, text=True,
    )
    if errs.returncode != 0:
        print(errs.stdout + errs.stderr, file=sys.stderr)
        return 2

    faults = {}
    for f in a.fault:
        k, _, v = f.partition('=')
        faults[k] = True if v in ('', 'true') else float(v)
    link = SimLink(arena, seed=a.seed, faults=faults) if a.sim else CflibLink(a.uri)
    log_path = a.log or str(REPO / 'flights' / f"{time.strftime('%Y%m%d-%H%M%S')}-{a.mode}{'-sim' if a.sim else ''}.jsonl")
    Path(log_path).parent.mkdir(parents=True, exist_ok=True)
    flight = Flight(link, arena, Limits(max_flight_sec=a.max_time + 30), log_path, a.height if a.mode == 'explore' else 0.4)

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
            timeout = 5.0 if a.sim else 0.04

            def step(tel, t):
                out = planner.ask({
                    't': t, 'x': tel.x, 'y': tel.y, 'z': tel.z, 'yaw': math.radians(tel.yaw_deg),
                    'vx': tel.vx, 'vy': tel.vy, 'front': tel.front, 'back': tel.back,
                    'left': tel.left, 'right': tel.right, 'up': tel.up,
                }, timeout)
                if not out or 'error' in out:
                    return None, False
                result['known'] = out.get('known')
                return Command(out['vx'], out['vy'], math.degrees(out['yawRate']), out['z']), out['phase'] == 'done'

        def wrapped(tel, t):
            cmd, done = step(tel, t)
            if cmd is not None and cmd.z is None:
                cmd.z = flight.height
            return cmd, done

        outcome = flight.run(wrapped, a.max_time)
        result.update(outcome=outcome, reason=flight.reason, events=flight.guard.events[-5:])
        if isinstance(link, SimLink):
            result.update(crashed=link.crashed, min_clearance=round(link.min_clearance, 3), seconds=round(link.t, 1))
        print(json.dumps(result))
        if a.mode == 'yaw':
            # Asked to turn 90 deg left (counter-clockwise): the estimate must agree, or the yaw sign is wrong.
            result['yaw_ok'] = result.get('yaw_turned_deg', 0) >= 60
        ok = outcome in ('complete', 'checked') and not result.get('crashed') and result.get('yaw_ok', True)
        return 0 if ok else 1
    finally:
        if planner:
            planner.close()
        if a.report:
            Path(a.report).write_text(json.dumps(result, indent=1) + '\n')
        link.close()
        if flight.log:
            flight.log.close()


if __name__ == '__main__':
    sys.exit(main())
