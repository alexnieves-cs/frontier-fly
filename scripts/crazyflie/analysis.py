"""Measures for one flight log, computed the same way for a real flight and the sim.

compare.py puts these side by side. Everything works from what a real flight logs: the drone's own
telemetry, the planner's command and what the guard sent. A sim log also has the true pose, and the
measures that can use it say so (the *_truth keys). Pure Python, no numpy: CI installs only cflib.

coverage() is a port of src/crazyflie/sim.ts's coverage (reachable floor within 1 m of the track, in
line of sight, on src/crazyflie/arena.ts's reachableCells grid), so a number here means the same as
the bench table in docs/CRAZYFLIE.md. tests/test_compare.py checks the port against the TypeScript.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from statistics import median, pstdev

from flightlog import Log, steps_with_tel
from links import arena_segments, cast_ray, clearance

PROP_RADIUS = 0.07       # src/crazyflie/profile.ts CRAZYFLIE.propRadius
AIRBORNE_Z = 0.1         # below this the drone is on the floor (sim.ts counts clearance only above it)
FOOTPRINT_M = 1.0        # coverage radius, sim.ts markFootprint


# ------------------------------------------------------------------ coverage (port of arena.ts / sim.ts)

@dataclass
class Reach:
    nx: int
    ny: int
    cell: float
    mask: list[int]
    count: int


def _inside_box(a: dict, x: float, y: float) -> bool:
    return any(b['x'] <= x <= b['x'] + b['w'] and b['y'] <= y <= b['y'] + b['d'] for b in a.get('boxes', []))


def _sign(v: float) -> int:
    return (v > 0) - (v < 0)


def _segments_cross(p, q) -> bool:
    def o(ax, ay, bx, by, cx, cy):
        return _sign((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    return (o(p[0], p[1], p[2], p[3], q[0], q[1]) != o(p[0], p[1], p[2], p[3], q[2], q[3])
            and o(q[0], q[1], q[2], q[3], p[0], p[1]) != o(q[0], q[1], q[2], q[3], p[2], p[3]))


def reachable_cells(a: dict, cell: float = 0.25, radius: float = 0.12) -> Reach:
    """arena.ts reachableCells: grid centres a Crazyflie fits at, flood-filled from the start."""
    segs = arena_segments(a)
    nx, ny = math.ceil(a['w'] / cell), math.ceil(a['d'] / cell)
    fits = [0] * (nx * ny)
    for j in range(ny):
        for i in range(nx):
            x, y = (i + 0.5) * cell, (j + 0.5) * cell
            if x < a['w'] and y < a['d'] and not _inside_box(a, x, y) and clearance(segs, x, y) >= radius:
                fits[j * nx + i] = 1
    mask = [0] * (nx * ny)
    best, best_d = -1, math.inf
    for k, f in enumerate(fits):
        if not f:
            continue
        d = math.hypot((k % nx + 0.5) * cell - a['start']['x'], (k // nx + 0.5) * cell - a['start']['y'])
        if d < best_d:
            best, best_d = k, d
    if best < 0:
        return Reach(nx, ny, cell, mask, 0)
    stack, mask[best], count = [best], 1, 1
    while stack:
        k = stack.pop()
        i, j = k % nx, k // nx
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if ni < 0 or nj < 0 or ni >= nx or nj >= ny:
                continue
            nk = nj * nx + ni
            if mask[nk] or not fits[nk]:
                continue
            hop = ((i + 0.5) * cell, (j + 0.5) * cell, (ni + 0.5) * cell, (nj + 0.5) * cell)
            if any(_segments_cross(hop, s) for s in segs):
                continue
            mask[nk] = 1
            count += 1
            stack.append(nk)
    return Reach(nx, ny, cell, mask, count)


def coverage(a: dict, track: list[tuple[float, float]], reach: Reach | None = None) -> float:
    """Share of reachable floor that came within 1 m of the track in line of sight (sim.ts markFootprint)."""
    reach = reach or reachable_cells(a)
    if not reach.count:
        return 0.0
    segs = arena_segments(a)
    seen = [0] * len(reach.mask)
    nx, ny, cell = reach.nx, reach.ny, reach.cell
    for x, y in track:
        i0, i1 = max(0, math.floor((x - FOOTPRINT_M) / cell)), min(nx - 1, math.floor((x + FOOTPRINT_M) / cell))
        j0, j1 = max(0, math.floor((y - FOOTPRINT_M) / cell)), min(ny - 1, math.floor((y + FOOTPRINT_M) / cell))
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                k = j * nx + i
                if not reach.mask[k] or seen[k]:
                    continue
                cx, cy = (i + 0.5) * cell, (j + 0.5) * cell
                d = math.hypot(cx - x, cy - y)
                if d > FOOTPRINT_M:
                    continue
                if cast_ray(segs, x, y, math.atan2(cy - y, cx - x)) < d:
                    continue
                seen[k] = 1
    return sum(seen) / reach.count


# ------------------------------------------------------------------ response fit

def _response_error(seq: list[list[tuple[float, float, float]]], tau: float, latency: float) -> tuple[float, int, float]:
    """Squared error of a first-order lag with a delay, run open loop over each segment.

    Each segment is a list of (t, commanded velocity, measured velocity) for one axis. The model is
    the stock velocity loop as sim.ts and SimLink fly it: dv/dt = (u(t - latency) - v) / tau, with u
    the command the guard sent, held from one step to the next.
    """
    sse, n, lag = 0.0, 0, 0.0
    for seg in seq:
        v = seg[0][2]
        for k in range(len(seg) - 1):
            t, t1 = seg[k][0], seg[k + 1][0]
            # The command in force at t - latency: the last one sent at or before then.
            target = t - latency + 1e-9
            j = k
            while j > 0 and seg[j][0] > target:
                j -= 1
            u = seg[j][1] if seg[j][0] <= target else 0.0
            lag += t - seg[j][0]
            v += (t1 - t) / tau * (u - v)
            e = seg[k + 1][2] - v
            sse += e * e
            n += 1
    return sse, n, lag / max(1, n)


def fit_response(log: Log, max_latency: float = 0.4, latency_step: float = 0.025) -> dict | None:
    """Fit the velocity lag (tau) and the command latency to how the drone answered its commands.

    Uses the fly phase only, both horizontal axes, in the arena frame. Returns None when the flight
    did not move enough to say anything (a hover). The latency is the measured time from a step's
    command to the command the best fit says was acting, so it counts the radio, the telemetry age
    and the drone together and comes out in whole control steps (50 ms) on a steady link.
    """
    segs: list[list[tuple[float, float, float]]] = []
    cur_x: list[tuple[float, float, float]] = []
    cur_y: list[tuple[float, float, float]] = []
    moved = 0.0
    for s in log.steps:
        tel, sent = s.get('tel'), s.get('sent')
        ok = s.get('phase') == 'fly' and tel and sent and tel.get('z', 0) > AIRBORNE_Z
        if not ok:
            if len(cur_x) > 10:
                segs += [cur_x, cur_y]
            cur_x, cur_y = [], []
            continue
        cur_x.append((s['t'], sent['vx'], tel['vx']))
        cur_y.append((s['t'], sent['vy'], tel['vy']))
        moved += math.hypot(sent['vx'], sent['vy']) * 0.05
    if len(cur_x) > 10:
        segs += [cur_x, cur_y]
    if not segs or moved < 0.5:
        return None

    def golden(f, lo, hi, iters=28):
        g = (math.sqrt(5) - 1) / 2
        a, b = lo, hi
        c, d = b - g * (b - a), a + g * (b - a)
        fc, fd = f(c), f(d)
        for _ in range(iters):
            if fc < fd:
                b, d, fd = d, c, fc
                c = b - g * (b - a)
                fc = f(c)
            else:
                a, c, fc = c, d, fd
                d = a + g * (b - a)
                fd = f(d)
        return (c, fc) if fc < fd else (d, fd)

    best = None
    steps = int(round(max_latency / latency_step))
    for i in range(steps + 1):
        lat = i * latency_step
        tau, sse = golden(lambda tau: _response_error(segs, tau, lat)[0], 0.03, 2.0)
        if best is None or sse < best[2] - 1e-12:
            best = (lat, tau, sse)
    lat, tau, sse = best
    _, n, lag = _response_error(segs, tau, lat)
    return {'tau_s': round(tau, 3), 'latency_s': round(lag, 3), 'rmse_mps': round(math.sqrt(sse / max(1, n)), 4), 'samples': n}


# ------------------------------------------------------------------ ranger noise

def ranger_noise(log: Log, window: int = 20) -> dict | None:
    """Range noise (1 sigma, m) from stretches where the drone held still in the air.

    A still stretch is `window` steps (1 s) with the measured speed under 3 cm/s and the heading
    steady to half a degree a step. Each beam's spread in each stretch, the median over stretches.
    """
    beams = ('front', 'back', 'left', 'right')
    per: dict[str, list[float]] = {b: [] for b in beams}
    run: list[dict] = []

    def flush():
        for k in range(0, len(run) - window + 1, window):
            chunk = run[k:k + window]
            for b in beams:
                vals = [c[b] for c in chunk if isinstance(c.get(b), (int, float))]
                if len(vals) >= window * 0.8:
                    per[b].append(pstdev(vals))

    prev = None
    for s in log.steps:
        tel = s.get('tel')
        still = (tel and s.get('phase') == 'fly' and tel['z'] > AIRBORNE_Z and math.hypot(tel['vx'], tel['vy']) < 0.03
                 and prev is not None and abs((tel['yaw_deg'] - prev['yaw_deg'] + 180) % 360 - 180) < 0.5)
        if still:
            run.append(tel)
        else:
            flush()
            run = []
        prev = tel or prev
    flush()
    out = {b: round(median(v), 4) for b, v in per.items() if v}
    if not out:
        return None
    out['all'] = round(median([x for v in per.values() for x in v]), 4)
    out['stretches'] = max(len(v) for v in per.values())
    return out


# ------------------------------------------------------------------ one flight's measures

def _reason_kind(reason: str) -> str:
    """'battery 3.31 V' -> 'battery V': numbers out, so the same rule counts once across flights."""
    return re.sub(r'\s+', ' ', re.sub(r'-?\d+(\.\d+)?', '', reason)).strip()


def flight_metrics(log: Log, arena: dict | None = None, reach: Reach | None = None) -> dict:
    """Every measure compare.py reports, for one log."""
    arena = arena or log.arena
    if arena is None:
        raise ValueError(f'{log.path}: no arena in the log; pass one')
    segs = arena_segments(arena)
    reach = reach or reachable_cells(arena)
    outcome, reason = log.outcome()
    air = [s for s in steps_with_tel(log.steps) if s['tel']['z'] > AIRBORNE_Z]
    fly = [s for s in air if s['phase'] == 'fly']
    m: dict = {'source': log.source, 'mode': log.mode, 'outcome': outcome, 'reason': reason, 'steps': len(log.steps)}

    if air:
        m['airborne_s'] = round(air[-1]['t'] - air[0]['t'], 2)
        track = [(s['tel']['x'], s['tel']['y']) for s in air]
        m['coverage'] = round(coverage(arena, track, reach), 4)
        m['closest_pass_m'] = round(min(clearance(segs, x, y) for x, y in track) - PROP_RADIUS, 3)
        m['path_m'] = round(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(track, track[1:])), 2)
        rng = [s['tel'][b] for s in air for b in ('front', 'back', 'left', 'right') if isinstance(s['tel'].get(b), (int, float))]
        m['closest_beam_m'] = round(min(rng), 3) if rng else None
        st = arena['start']
        last = air[-1]['tel']
        m['landing_offset_m'] = round(math.hypot(last['x'] - st['x'], last['y'] - st['y']), 3)
        zs = [s['tel']['z'] for s in fly]
        want = [s['sent']['z'] for s in fly if s.get('sent')]
        if zs and want:
            m['height_mean_m'] = round(sum(zs) / len(zs), 3)
            m['height_error_m'] = round(sum(abs(s['tel']['z'] - s['sent']['z']) for s in fly if s.get('sent')) / len(want), 3)
        if log.mode == 'hover' and fly:
            x0, y0 = fly[0]['tel']['x'], fly[0]['tel']['y']
            m['hover_drift_m'] = round(max(math.hypot(s['tel']['x'] - x0, s['tel']['y'] - y0) for s in fly), 3)
        if air[0].get('truth'):
            truth = [(s['truth']['x'], s['truth']['y']) for s in air if s.get('truth')]
            m['coverage_truth'] = round(coverage(arena, truth, reach), 4)
            m['closest_pass_truth_m'] = round(min(clearance(segs, x, y) for x, y in truth) - PROP_RADIUS, 3)
            tl = air[-1]['truth']
            m['estimate_error_end_m'] = round(math.hypot(last['x'] - tl['x'], last['y'] - tl['y']), 3)
            m['track_truth'] = [(round(x, 2), round(y, 2)) for x, y in truth[::10]]
        m['track'] = [(round(x, 2), round(y, 2)) for x, y in track[::10]]
    else:
        m['airborne_s'] = 0.0

    tels = [s['tel'] for s in log.steps if s.get('tel')]
    if tels:
        v = [t['vbat'] for t in tels]
        m['battery'] = {'first': round(v[0], 2), 'min': round(min(v), 2), 'last': round(v[-1], 2)}
    ages = [s['age'] for s in log.steps if isinstance(s.get('age'), (int, float)) and s.get('phase') != 'ground']
    if ages:
        sa = sorted(ages)
        m['telemetry_age_s'] = {'p50': round(sa[len(sa) // 2], 3), 'p95': round(sa[min(len(sa) - 1, int(0.95 * len(sa)))], 3), 'max': round(sa[-1], 3)}
    m['no_telemetry_steps'] = sum(1 for s in log.steps if s.get('tel') is None and s.get('phase') != 'ground')

    verdicts: dict[str, int] = {}
    clamps: dict[str, int] = {}
    stops: dict[str, int] = {}
    for s in log.steps:
        verdict = s.get('verdict', '')
        verdicts[verdict] = verdicts.get(verdict, 0) + 1
        r = s.get('reason') or ''
        if verdict == 'fly' and r.startswith('clamp: '):
            for c in r[len('clamp: '):].split(', '):
                clamps[c] = clamps.get(c, 0) + 1
        elif verdict in ('hover', 'land', 'kill') and r:
            k = f'{verdict}: {_reason_kind(r)}'
            stops[k] = stops.get(k, 0) + 1
    m['verdicts'] = verdicts
    m['clamps'] = clamps
    m['guard_stops'] = stops
    if log.summary:
        for k in ('yaw_turned_deg', 'yaw_ok', 'known', 'crashed'):
            if k in log.summary:
                m[k] = log.summary[k]
    fit = fit_response(log)
    if fit:
        m['response'] = fit
    noise = ranger_noise(log)
    if noise:
        m['ranger_noise_m'] = noise
    return m
