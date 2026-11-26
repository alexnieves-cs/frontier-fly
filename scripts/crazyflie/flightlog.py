"""The Crazyflie flight log: one JSON object per line, the same for a real flight and the sim.

fly.py and bringup.py write it; compare.py reads it. A log has three kinds of line:

  {"event": "start", "schema": "frontier-fly/crazyflie-log", "version": 1, "t": s, "meta": {...}}
      First line. meta says what flew where: source ("real", "sim" or "mock-radio"), mode, the
      arena itself, the guard limits, the cruise height, and for the sim its seed, profile and faults.
  {"t": s, "phase": "takeoff"|"fly"|"land"|"ground", "tel": {...}|null, "age": s,
   "cmd": {...}|null, "sent": {...}|null, "verdict": "fly"|"hover"|"land"|"kill"|"landing"|"idle",
   "reason": str, "truth": {...}}
      One per control step (20 Hz). tel is the drone's own telemetry in the arena frame
      (guard.Telemetry), age how old it was, cmd what the planner asked for, sent what the guard
      let through (arena frame). truth is the simulator's real pose; a real flight has none.
  {"event": "<name>", "t": s, "reason": str, ...}
      Anything else that happened: "landed", "guard-landed", "killed", "error" (what failed in the
      flight script), "error-landed" (it landed because of that error), "onboard-landing" (the
      firmware flew the descent because the script's own landing failed), and "end" as the last
      line, which carries the run's summary.

Logs written before version 1 have no start line. read_log still reads them; meta is then empty.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Iterable

SCHEMA = 'frontier-fly/crazyflie-log'
VERSION = 1
PHASES = ('takeoff', 'fly', 'land', 'ground')
VERDICTS = ('fly', 'hover', 'land', 'kill', 'landing', 'idle')
SOURCES = ('real', 'sim', 'mock-radio')


def _plain(o: Any) -> Any:
    if is_dataclass(o):
        return asdict(o)
    if isinstance(o, float) and not math.isfinite(o):
        return None
    return o.__dict__ if hasattr(o, '__dict__') else str(o)


def _round(v: Any) -> Any:
    """Floats to 4 decimals (0.1 mm, 0.1 ms): the log stays small and diffs stay readable."""
    if isinstance(v, float):
        return round(v, 4) if math.isfinite(v) else None
    if isinstance(v, dict):
        return {k: _round(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_round(x) for x in v]
    if is_dataclass(v):
        return _round(asdict(v))
    return v


class FlightLog:
    """Writes one log. Every write is flushed, so a crash or a Ctrl-C keeps what was flown."""

    def __init__(self, path: str | Path, meta: dict, t0: float = 0.0):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, 'w')
        self.closed = False
        self._line({'event': 'start', 'schema': SCHEMA, 'version': VERSION, 't': t0, 'meta': meta})

    def _line(self, rec: dict) -> None:
        if self.closed:
            return
        self.f.write(json.dumps(_round(rec), default=_plain) + '\n')
        self.f.flush()

    def step(self, t: float, phase: str, tel: Any, age: float | None, cmd: Any, sent: Any, verdict: str,
             reason: str = '', truth: dict | None = None) -> None:
        rec = {'t': t, 'phase': phase, 'tel': tel, 'age': age if age is None or math.isfinite(age) else None,
               'cmd': cmd, 'sent': sent, 'verdict': verdict, 'reason': reason}
        if truth is not None:
            rec['truth'] = truth
        self._line(rec)

    def event(self, t: float, name: str, reason: str = '', **extra: Any) -> None:
        self._line({'event': name, 't': t, 'reason': reason, **extra})

    def close(self, t: float, summary: dict | None = None) -> None:
        if self.closed:
            return
        self._line({'event': 'end', 't': t, 'summary': summary or {}})
        self.closed = True
        self.f.close()


class Log:
    """A log read back: meta from the start line, the control steps, the events, the end summary."""

    def __init__(self, path: Path, header: dict | None, steps: list[dict], events: list[dict], summary: dict | None):
        self.path = path
        self.header = header
        self.meta: dict = (header or {}).get('meta', {})
        self.steps = steps
        self.events = events
        self.summary = summary

    @property
    def source(self) -> str:
        return self.meta.get('source', 'unknown')

    @property
    def mode(self) -> str | None:
        return self.meta.get('mode')

    @property
    def arena(self) -> dict | None:
        return self.meta.get('arena')

    def outcome(self) -> tuple[str, str]:
        """(outcome, reason) from the end summary, or the last flight event for older logs."""
        if self.summary and 'outcome' in self.summary:
            return self.summary['outcome'], self.summary.get('reason', '')
        for e in reversed(self.events):
            if e['event'] in ('complete', 'guard-landed', 'error-landed', 'killed', 'aborted', 'operator'):
                return e['event'], e.get('reason', '')
        return 'unknown', ''


def read_log(path: str | Path) -> Log:
    path = Path(path)
    header, steps, events, summary = None, [], [], None
    for n, line in enumerate(path.read_text().splitlines()):
        if not line.strip():
            continue
        rec = json.loads(line)
        ev = rec.get('event')
        if ev == 'start' and n == 0:
            header = rec
        elif ev == 'end':
            summary = rec.get('summary', {})
        elif ev is not None:
            events.append(rec)
        else:
            steps.append(rec)
    return Log(path, header, steps, events, summary)


def validate(path: str | Path) -> list[str]:
    """Problems with a log, empty when it follows version 1. compare.py runs this on every log it reads."""
    problems: list[str] = []
    lines = [x for x in Path(path).read_text().splitlines() if x.strip()]
    if not lines:
        return ['empty log']
    try:
        recs = [json.loads(x) for x in lines]
    except json.JSONDecodeError as e:
        return [f'not JSON lines: {e}']
    h = recs[0]
    if h.get('event') != 'start' or h.get('schema') != SCHEMA:
        problems.append('first line is not a start record (a log from before version 1?)')
    elif h.get('version') != VERSION:
        problems.append(f'version {h.get("version")}, expected {VERSION}')
    else:
        meta = h.get('meta', {})
        if meta.get('source') not in SOURCES:
            problems.append(f'meta.source must be one of {", ".join(SOURCES)}')
        if not meta.get('mode'):
            problems.append('meta.mode is missing')
        if meta.get('mode') != 'link' and not isinstance(meta.get('arena'), dict):
            problems.append('meta.arena is missing')
    if recs[-1].get('event') != 'end':
        problems.append('no end record (the run was cut off)')
    last_t = -math.inf
    for i, r in enumerate(recs[1:], start=2):
        if 'event' in r:
            continue
        if not isinstance(r.get('t'), (int, float)):
            problems.append(f'line {i}: step without a time')
            continue
        if r['t'] < last_t - 1e-9:
            problems.append(f'line {i}: time went backwards')
        last_t = r['t']
        if r.get('phase') not in PHASES:
            problems.append(f'line {i}: phase {r.get("phase")!r}')
        if r.get('verdict') not in VERDICTS:
            problems.append(f'line {i}: verdict {r.get("verdict")!r}')
        tel = r.get('tel')
        if tel is not None and not all(k in tel for k in ('x', 'y', 'z', 'yaw_deg')):
            problems.append(f'line {i}: telemetry without a pose')
        if len(problems) > 20:
            problems.append('... more')
            break
    return problems


def steps_with_tel(steps: Iterable[dict], phases: tuple[str, ...] = ('takeoff', 'fly', 'land')) -> list[dict]:
    return [s for s in steps if s.get('tel') and s.get('phase') in phases]
