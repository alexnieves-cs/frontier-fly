#!/usr/bin/env python3
"""Sim versus real: replay a flight's arena and mode in the sim and report where they disagree.

    python3 scripts/crazyflie/compare.py flights/20261201-101500-hover.jsonl [--seeds 5] [--landed 1.32,1.05]

For each log it writes a folder next to it (or --out) with report.md, report.json and track.svg:
  1. reads the log (version 1, see flightlog.py) and measures it with analysis.py;
  2. flies the same arena, mode, height and time limit through fly.py --sim --sim-profile bench for
     --seeds seeds: the same guard, planner and limits as the real flight, on the simulated drone;
  3. for explore, also flies the bench model behind npm run crazyflie:bench (scripts/crazyflie/replay.ts),
     the numbers docs/CRAZYFLIE.md publishes;
  4. lists every measure where the real flight falls outside the sim's spread.

The real track is the drone's own position estimate: nothing measures where it really was. --landed X,Y
(the landing spot you measured with a tape, arena frame, meters) gives one true point, and the report
turns it into the flow deck's real drift. Any log works, a sim or mock-radio log included, so the tool
can be tried before there is a drone; the report says when the "real" side came from the sim.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import signal
import subprocess
import sys
import tempfile
from pathlib import Path
from statistics import mean

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

from analysis import PROP_RADIUS, flight_metrics, reachable_cells  # noqa: E402
from flightlog import read_log, validate  # noqa: E402
from links import SIM_PROFILES  # noqa: E402

FLYING_MODES = ('hover', 'yaw', 'square', 'explore')
BENCH = SIM_PROFILES['bench']

# How far outside the sim's spread a measure has to be before the report flags it.
TOL = {
    'coverage': 0.05,          # share of reachable floor
    'closest_pass_m': 0.03,
    'landing_offset_m': 0.10,
    'rel': 0.15,               # time in the air and path, relative to the sim's mean
    'age_p95_s': 0.10,         # Limits().telemetry_land_sec is 0.5; the sim's link has none
    'tau_rel': 0.30,
    'latency_s': 0.05,
    'noise_x': 2.0,            # ranger noise as a multiple of the sim's
    'drift_x': 2.0,            # flow drift as a multiple of the bench model's
}


def parse_xy(text: str | None) -> tuple[float, float] | None:
    if not text:
        return None
    try:
        x, y = (float(v) for v in text.split(','))
    except ValueError:
        raise SystemExit(f'--landed: expected X,Y in meters, got {text!r}')
    return x, y


def sim_flights(log, seeds: int, base: int, sim_set: str | None, workdir: Path) -> list[dict]:
    """Fly the log's arena and mode in the sim, through fly.py itself, one run per seed."""
    import fly  # imported here: fly.py installs a Ctrl-C handler while it flies

    arena_file = workdir / 'arena.json'
    arena_file.write_text(json.dumps(log.arena))
    meta = log.meta
    out = []
    for seed in range(base, base + seeds):
        path = workdir / f'sim-{seed}.jsonl'
        argv = ['--arena', str(arena_file), '--mode', log.mode, '--sim', '--yes', '--sim-profile', 'bench',
                '--seed', str(seed), '--height', str(meta.get('height', 0.5)),
                '--max-time', str(meta.get('max_time', 180.0)), '--log', str(path)]
        if sim_set:
            argv += ['--sim-set', sim_set]
        handler = signal.getsignal(signal.SIGINT)
        try:
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()) as err:
                code = fly.main(argv)
        finally:
            signal.signal(signal.SIGINT, handler)
        if not path.exists():
            raise SystemExit(f'sim replay seed {seed} wrote no log (exit {code}):\n{err.getvalue()}')
        m = flight_metrics(read_log(path), log.arena)
        m['seed'] = seed
        out.append(m)
    return out


def bench_flights(log, seeds: int, base: int) -> dict | None:
    """The bench model (src/crazyflie/sim) on the same arena, for explore. None if it cannot run."""
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
        json.dump(log.arena, f)
    cmd = ['npx', 'vite-node', 'scripts/crazyflie/replay.ts', '--', '--arena', f.name, '--seeds', str(seeds),
           '--base', str(base), '--max-sec', str(log.meta.get('max_time', 180.0))]
    try:
        r = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=600)
    finally:
        Path(f.name).unlink(missing_ok=True)
    if r.returncode != 0:
        print(f'bench model skipped: {r.stderr.strip()[:300]}', file=sys.stderr)
        return None
    return json.loads(r.stdout.strip().splitlines()[-1])


# ------------------------------------------------------------------ comparing

def spread(sims: list[dict], key: str, sub: str | None = None) -> list[float]:
    vals = []
    for m in sims:
        v = m.get(key)
        if sub is not None:
            v = v.get(sub) if isinstance(v, dict) else None
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            vals.append(float(v))
    return vals


def disagreements(real: dict, sims: list[dict], mode: str, drift: dict | None, model: dict | None = None) -> list[str]:
    """Plain sentences, one per measure where the real flight sits outside the sim's spread.

    model is the sim's settings (the bench profile with any --sim-set applied).
    """
    model = model or BENCH
    flags: list[str] = []

    outcomes = [m['outcome'] for m in sims]
    if outcomes.count(real['outcome']) == 0:
        flags.append(f"Outcome: the real flight ended {real['outcome']} ({real['reason'] or 'no reason given'}); "
                     f"no sim run did ({', '.join(sorted(set(outcomes)))}).")

    sim_stops = {k for m in sims for k in m.get('guard_stops', {})}
    for k, c in real.get('guard_stops', {}).items():
        if k not in sim_stops:
            flags.append(f'Guard: "{k}" happened {c} times in the real flight and never in the sim.')

    def outside(key: str, tol: float, label: str, unit: str, lower_only: bool = False):
        r, s = real.get(key), spread(sims, key)
        if not isinstance(r, (int, float)) or not s:
            return
        lo, hi = min(s), max(s)
        if r < lo - tol:
            flags.append(f'{label}: {r:g}{unit} real, {lo:g} to {hi:g}{unit} in the sim (lower).')
        elif not lower_only and r > hi + tol:
            flags.append(f'{label}: {r:g}{unit} real, {lo:g} to {hi:g}{unit} in the sim (higher).')

    outside('coverage', TOL['coverage'], 'Coverage by the estimate', '')
    outside('closest_pass_m', TOL['closest_pass_m'], 'Closest pass by the map', ' m', lower_only=True)
    if mode != 'explore':
        r, s = real.get('landing_offset_m'), spread(sims, 'landing_offset_m')
        if isinstance(r, (int, float)) and s and r > max(s) + TOL['landing_offset_m']:
            flags.append(f'Landing: {r:g} m from the take-off point by its own estimate, the sim landed within {max(s):g} m.')
    for key, label in (('airborne_s', 'Time in the air'), ('path_m', 'Path flown')):
        r, s = real.get(key), spread(sims, key)
        if isinstance(r, (int, float)) and s and mean(s) > 0:
            if abs(r - mean(s)) > TOL['rel'] * mean(s) and not min(s) <= r <= max(s):
                unit = ' s' if key == 'airborne_s' else ' m'
                flags.append(f'{label}: {r:g}{unit} real, {mean(s):.2f}{unit} on average in the sim.')

    age = real.get('telemetry_age_s', {}).get('p95')
    if isinstance(age, (int, float)) and age > TOL['age_p95_s']:
        flags.append(f'Radio: 95% of telemetry was under {age:g} s old; the sim radio delivers it at once. '
                     'Try the comparison again with --sim-set, or fly the sim behind --mock-radio.')
    if real.get('no_telemetry_steps'):
        flags.append(f"Radio: {real['no_telemetry_steps']} control steps had no telemetry at all.")

    fit = real.get('response')
    if fit:
        if abs(fit['tau_s'] - model['tau']) > TOL['tau_rel'] * model['tau']:
            flags.append(f"Response: the drone's velocity lag fits {fit['tau_s']:g} s, the sim uses {model['tau']:g} s.")
        if abs(fit['latency_s'] - model['latency']) > TOL['latency_s']:
            flags.append(f"Response: its command latency fits {fit['latency_s']:g} s, the sim uses {model['latency']:g} s.")
    noise = real.get('ranger_noise_m', {}).get('all')
    if isinstance(noise, (int, float)) and noise > TOL['noise_x'] * model['sigma']:
        flags.append(f"Rangers: {noise:g} m of noise while holding still, the sim uses {model['sigma']:g} m.")
    if drift and drift['fraction'] > TOL['drift_x'] * model['drift']:
        flags.append(f"Flow drift: the estimate was {drift['error_m']:g} m off at landing after {drift['path_m']:g} m flown "
                     f"({100 * drift['fraction']:.1f}%), the sim assumes about {100 * model['drift']:.0f}%.")
    return flags


def measured_drift(real: dict, log, landed: tuple[float, float] | None) -> dict | None:
    if not landed or not real.get('track'):
        return None
    steps = [s for s in log.steps if s.get('tel') and s['tel']['z'] > 0.1]
    last = steps[-1]['tel'] if steps else None
    if not last:
        return None
    err = math.hypot(last['x'] - landed[0], last['y'] - landed[1])
    path = real.get('path_m') or 0.0
    return {'landed': landed, 'estimate': (round(last['x'], 3), round(last['y'], 3)), 'error_m': round(err, 3),
            'path_m': path, 'fraction': round(err / path, 4) if path > 0.2 else 0.0}


# ------------------------------------------------------------------ writing

def fmt(v, nd: int = 3) -> str:
    if v is None:
        return 'n/a'
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, float):
        return f'{round(v, nd):g}'
    return str(v)


def cell(sims: list[dict], key: str, sub: str | None = None, nd: int = 3) -> str:
    s = spread(sims, key, sub)
    if not s:
        return 'n/a'
    if len(s) == 1 or min(s) == max(s):
        return fmt(s[0], nd)
    return f'{fmt(mean(s), nd)} ({fmt(min(s), nd)} to {fmt(max(s), nd)})'


def counts(d: dict | None) -> str:
    return ', '.join(f'{k} {v}' for k, v in sorted((d or {}).items(), key=lambda kv: -kv[1])) or 'none'


def track_svg(arena: dict, real: dict, sims: list[dict], landed: tuple[float, float] | None) -> str:
    s, pad = 100.0, 20.0
    W, H = arena['w'] * s + 2 * pad, arena['d'] * s + 2 * pad

    def px(x: float, y: float) -> tuple[float, float]:
        return pad + x * s, pad + (arena['d'] - y) * s

    def poly(pts, colour: str, width: float, dash: str = '') -> str:
        if len(pts) < 2:
            return ''
        d = ' '.join(f'{px(x, y)[0]:.1f},{px(x, y)[1]:.1f}' for x, y in pts)
        da = f' stroke-dasharray="{dash}"' if dash else ''
        return f'<polyline points="{d}" fill="none" stroke="{colour}" stroke-width="{width}"{da}/>'

    W = max(W, 560.0)  # room for the legend under a small arena
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H + 30:.0f}" viewBox="0 0 {W:.0f} {H + 30:.0f}" font-family="sans-serif" font-size="12">',
           f'<rect width="100%" height="100%" fill="#ffffff"/>']
    x0, y0 = px(0, arena['d'])
    out.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{arena["w"] * s:.1f}" height="{arena["d"] * s:.1f}" fill="none" stroke="#222" stroke-width="3"/>')
    for w in arena.get('walls', []):
        a, b = px(w['ax'], w['ay']), px(w['bx'], w['by'])
        out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="#222" stroke-width="3"/>')
    for b in arena.get('boxes', []):
        p = px(b['x'], b['y'] + b['d'])
        out.append(f'<rect x="{p[0]:.1f}" y="{p[1]:.1f}" width="{b["w"] * s:.1f}" height="{b["d"] * s:.1f}" fill="#bbb" stroke="#555"/>')
    for m in sims:
        out.append(poly(m.get('track', []), '#e08a2c', 1.2, '5,4'))
    if sims and sims[0].get('track_truth'):
        out.append(poly(sims[0]['track_truth'], '#999999', 1.0, '2,3'))
    out.append(poly(real.get('track', []), '#1f5fbf', 2.2))
    st = px(arena['start']['x'], arena['start']['y'])
    out.append(f'<circle cx="{st[0]:.1f}" cy="{st[1]:.1f}" r="6" fill="#2a9d3a"/>')
    if landed:
        lp = px(*landed)
        out.append(f'<path d="M{lp[0] - 6:.1f},{lp[1] - 6:.1f} l12,12 m0,-12 l-12,12" stroke="#c0392b" stroke-width="2.5"/>')
    ly = H + 18
    out.append(f'<line x1="{pad}" y1="{ly - 4}" x2="{pad + 24}" y2="{ly - 4}" stroke="#1f5fbf" stroke-width="2.2"/>'
               f'<text x="{pad + 30}" y="{ly}">{real["source"]} (its own estimate)</text>')
    out.append(f'<line x1="{pad + 210}" y1="{ly - 4}" x2="{pad + 234}" y2="{ly - 4}" stroke="#e08a2c" stroke-width="1.2" stroke-dasharray="5,4"/>'
               f'<text x="{pad + 240}" y="{ly}">sim (estimate)</text>')
    out.append(f'<circle cx="{pad + 350}" cy="{ly - 4}" r="5" fill="#2a9d3a"/><text x="{pad + 360}" y="{ly}">take-off</text>')
    if landed:
        out.append(f'<text x="{pad + 430}" y="{ly}" fill="#c0392b">x measured landing</text>')
    out.append('</svg>')
    return '\n'.join(o for o in out if o) + '\n'


def report_md(log, real: dict, sims: list[dict], bench: dict | None, flags: list[str], drift: dict | None,
              seeds: tuple[int, int], sim_set: str | None, model: dict | None = None) -> str:
    model = model or BENCH
    meta = log.meta
    name = log.arena.get('name', meta.get('arena_file', 'arena'))
    n = len(sims)
    lines = [f'# Sim versus real: {name}, {log.mode}', '']
    where = f"`{log.path.name}`, started {meta.get('started_at', 'n/a')}, commit {meta.get('commit') or 'n/a'}"
    if log.source == 'real':
        lines.append(f'Real flight: {where}.')
    else:
        lines.append(f'Left column: {where}. This log came from the {log.source} link, not a drone, so this report is a dry run of the tool.')
    tweak = f' with `--sim-set {sim_set}`' if sim_set else ''
    lines.append(f'Sim: {n} flights of the same arena, mode, height and time limit through `fly.py --sim --sim-profile bench`{tweak} '
                 f'(seeds {seeds[0]} to {seeds[0] + seeds[1] - 1}), so the same guard, planner and limits on the simulated drone.')
    if bench:
        lines.append(f"Bench model: {len(bench['flights'])} flights of src/crazyflie/sim on the same arena (scripts/crazyflie/replay.ts), the model behind the numbers in docs/CRAZYFLIE.md.")
    lines += ['', '## Where they disagree', '']
    lines += [f'- {f}' for f in flags] if flags else ['Nothing falls outside the sim\'s spread by more than the report\'s tolerances.']
    left = 'Real flight' if log.source == 'real' else f'This log ({log.source})'
    lines += ['', '## Side by side', '', f'| Measure | {left} | Sim, mean (range) |' + (' Bench model |' if bench else ''),
              '|---|---|---|' + ('---|' if bench else '')]

    bf = bench['flights'] if bench else []

    def bench_cell(key: str, nd: int = 3) -> str:
        vals = [f[key] for f in bf if isinstance(f.get(key), (int, float))]
        if not vals:
            return 'n/a'
        return f'{fmt(mean(vals), nd)} ({fmt(min(vals), nd)} to {fmt(max(vals), nd)})' if min(vals) != max(vals) else fmt(vals[0], nd)

    def row(label: str, rv: str, sv: str, bv: str = 'n/a'):
        lines.append(f'| {label} | {rv} | {sv} |' + (f' {bv} |' if bench else ''))

    outcomes = [m['outcome'] for m in sims]
    sim_out = ', '.join(f'{outcomes.count(o)} {o}' for o in sorted(set(outcomes)))
    b_out = ', '.join(f"{sum(1 for f in bf if f['outcome'] == o)} {o}" for o in sorted({f['outcome'] for f in bf})) if bf else 'n/a'
    row('Outcome', f"{real['outcome']}" + (f" ({real['reason']})" if real['reason'] else ''), sim_out, b_out)
    row('Time in the air, s', fmt(real.get('airborne_s'), 1), cell(sims, 'airborne_s', nd=1), bench_cell('seconds', 1))
    row('Path, m', fmt(real.get('path_m'), 2), cell(sims, 'path_m', nd=2), bench_cell('pathM', 2))
    row('Coverage by the estimate', fmt(real.get('coverage')), cell(sims, 'coverage'))
    row('Coverage, true', fmt(real.get('coverage_truth')), cell(sims, 'coverage_truth'), bench_cell('coverage'))
    row('Closest pass by the map, m', fmt(real.get('closest_pass_m')), cell(sims, 'closest_pass_m'))
    row('Closest pass, true, m', fmt(real.get('closest_pass_truth_m')), cell(sims, 'closest_pass_truth_m'), bench_cell('minClearance'))
    row('Closest ranger reading, m', fmt(real.get('closest_beam_m')), cell(sims, 'closest_beam_m'))
    row('Landing, from take-off, m', fmt(real.get('landing_offset_m')), cell(sims, 'landing_offset_m'))
    row('Estimate error at the end, m', f"{fmt(drift['error_m'])} (you measured)" if drift else fmt(real.get('estimate_error_end_m')), cell(sims, 'estimate_error_end_m'), bench_cell('finalEstErr'))
    row('Height error, m', fmt(real.get('height_error_m')), cell(sims, 'height_error_m'))
    if log.mode == 'hover':
        row('Hover drift by the estimate, m', fmt(real.get('hover_drift_m')), cell(sims, 'hover_drift_m'))
    if log.mode == 'yaw':
        row('Yaw turned, deg', fmt(real.get('yaw_turned_deg'), 1), cell(sims, 'yaw_turned_deg', nd=1))
    if log.mode == 'explore':
        row("Explorer's own map, known share", fmt(real.get('known')), cell(sims, 'known'), bench_cell('mapped'))
    age = real.get('telemetry_age_s', {})
    row('Telemetry age, p95 / max, s', f"{fmt(age.get('p95'))} / {fmt(age.get('max'))}", f"{cell(sims, 'telemetry_age_s', 'p95')} / {cell(sims, 'telemetry_age_s', 'max')}")
    row('Steps without telemetry', fmt(real.get('no_telemetry_steps')), cell(sims, 'no_telemetry_steps', nd=0))
    bat = real.get('battery', {})
    row('Battery, first / lowest, V', f"{fmt(bat.get('first'), 2)} / {fmt(bat.get('min'), 2)}", 'modelled, not measured')
    fit = real.get('response') or {}
    row('Velocity lag (tau), s', fmt(fit.get('tau_s')), cell(sims, 'response', 'tau_s'))
    row('Command latency, s', fmt(fit.get('latency_s')), cell(sims, 'response', 'latency_s'))
    noise = real.get('ranger_noise_m') or {}
    row('Ranger noise holding still, m', fmt(noise.get('all'), 4), cell(sims, 'ranger_noise_m', 'all', nd=4))
    row('Guard clamps (steps)', counts(real.get('clamps')), counts(sims[0].get('clamps')) + (' (seed 1)' if n > 1 else ''))
    row('Guard stops', counts(real.get('guard_stops')), counts({k: sum(m.get('guard_stops', {}).get(k, 0) for m in sims) for m in sims for k in m.get('guard_stops', {})}))

    lines += ['', '## Track', '', '![The real track, the sim tracks and the arena](track.svg)', '',
              'Blue: the drone\'s own position estimate. Orange, dashed: each sim run\'s estimate. Grey, dotted: the first sim run\'s true track. Green: take-off.' + (' Red cross: the landing spot you measured.' if drift else ''),
              '', '## How to read this', '']
    lines.append('- The real track is the flow deck\'s estimate, so the real coverage and closest pass are by the estimate. The sim has both, '
                 'and the gap between its estimate and true rows is how much flow drift alone moves those numbers.')
    if drift:
        lines.append(f"- You measured the landing at {drift['landed'][0]:g}, {drift['landed'][1]:g}; the drone thought it was at "
                     f"{drift['estimate'][0]:g}, {drift['estimate'][1]:g}: {drift['error_m']:g} m of drift after {drift['path_m']:g} m "
                     f"({100 * drift['fraction']:.1f}%). The sim assumes about {100 * model['drift']:.0f}%.")
    else:
        lines.append('- Measure where it landed with a tape (arena frame) and pass `--landed X,Y` to turn the end of the track into the real flow drift.')
    lines.append(f"- Closest pass by the map is the prop disc's distance to the nearest wall or box in the arena file ({PROP_RADIUS:g} m prop radius), "
                 'so a tape-measure error in the arena moves it too. The closest ranger reading is what the drone saw itself.')
    lines.append(f"- The sim's velocity lag is {model['tau']:g} s and its latency {model['latency']:g} s. If the real fit differs, run this again with "
                 f"`--sim-set tau={fit.get('tau_s', model['tau']):g},latency={fit.get('latency_s', model['latency']):g}` and see whether the other rows close up.")
    if log.mode != 'explore':
        lines.append('- Battery sag, wind from the props and floor texture are not in the sim at all.')
    return '\n'.join(lines) + '\n'


def compare_one(path: Path, a) -> int:
    problems = validate(path)
    if problems:
        print(f'{path}: not a version 1 flight log:\n  - ' + '\n  - '.join(problems), file=sys.stderr)
        return 2
    log = read_log(path)
    if log.mode not in FLYING_MODES:
        print(f'{path}: a {log.mode} log never left the ground, so there is nothing to compare', file=sys.stderr)
        return 2
    real = flight_metrics(log, log.arena, reachable_cells(log.arena))
    if not real.get('airborne_s'):
        print(f'{path}: the drone never got above 0.1 m, so there is nothing to compare', file=sys.stderr)
        return 2
    landed = parse_xy(a.landed)
    drift = measured_drift(real, log, landed)
    from fly import parse_pairs
    model = {**BENCH, **parse_pairs(a.sim_set, '--sim-set')}
    out = Path(a.out) if a.out else path.with_name(path.stem + '-report')
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        sims = sim_flights(log, a.seeds, a.base, a.sim_set, Path(tmp))
    bench = bench_flights(log, a.seeds, a.base) if log.mode == 'explore' and not a.no_bench else None
    flags = disagreements(real, sims, log.mode, drift, model)
    (out / 'report.md').write_text(report_md(log, real, sims, bench, flags, drift, (a.base, a.seeds), a.sim_set, model))
    (out / 'track.svg').write_text(track_svg(log.arena, real, sims, landed))
    data = {'log': str(path), 'source': log.source, 'mode': log.mode, 'arena': log.arena.get('name'),
            'sim': {'profile': 'bench', 'settings': model, 'sim_set': a.sim_set, 'seeds': [a.base, a.base + a.seeds - 1]},
            'disagreements': flags, 'real': real, 'sim_flights': sims, 'measured_drift': drift,
            'bench_model': None if bench is None else {k: v for k, v in bench.items() if k != 'flights'} | {
                'flights': [{k: v for k, v in f.items() if k not in ('track', 'steps')} for f in bench['flights']]}}
    (out / 'report.json').write_text(json.dumps(data, indent=1) + '\n')
    print(f'{path.name}: {len(flags)} disagreement{"s" if len(flags) != 1 else ""}; wrote {out}/report.md')
    for f in flags:
        print(f'  - {f}')
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+', help='flight logs from fly.py')
    ap.add_argument('--seeds', type=int, default=5, help='sim runs per log (default 5)')
    ap.add_argument('--base', type=int, default=1, help='first sim seed (default 1)')
    ap.add_argument('--sim-set', default=None, help='override the bench profile, e.g. tau=0.5,latency=0.15')
    ap.add_argument('--landed', default=None, metavar='X,Y', help='where it landed, measured with a tape (arena frame, m)')
    ap.add_argument('--out', default=None, help='report folder (one log only; default <log>-report next to the log)')
    ap.add_argument('--no-bench', action='store_true', help='explore: skip the bench model flights')
    a = ap.parse_args(argv)
    if a.out and len(a.logs) > 1:
        ap.error('--out takes one log')
    if a.landed and len(a.logs) > 1:
        ap.error('--landed takes one log')
    if a.seeds < 1:
        ap.error('--seeds must be at least 1')
    worst = 0
    for p in a.logs:
        worst = max(worst, compare_one(Path(p), a))
    return worst


if __name__ == '__main__':
    sys.exit(main())
