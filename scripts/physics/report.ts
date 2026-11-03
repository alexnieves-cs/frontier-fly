/**
 * Score the physics validation's flights and write the results.
 *
 *   npx vite-node scripts/physics/report.ts -- --in test-results/physics [--write] [--summary FILE]
 *
 * Reads every <id>.<sim>.json that scripts/physics/run.sh wrote (PX4 on SIH,
 * PX4 on frontier-fly's plant, frontier-fly's own controller), puts them on
 * one time base, and measures each maneuver. It also flies the
 * near-surface and battery maneuvers with the shipped physics (no aero
 * model, no sag) for comparison, and runs the analyses that need no
 * simulator: the ground and ceiling effect curves against the published
 * measurements, and PX4's battery gauge on a sagging pack.
 *
 * --write puts the numbers in docs/physics-validation.json, the plots in
 * docs/media/physics/ and the tables in docs/PHYSICS_VALIDATION.md.
 */
import { existsSync, mkdirSync, readFileSync, writeFileSync, appendFileSync } from 'node:fs';
import { join } from 'node:path';
import { ceilingEffect, groundEffect, legacyGroundEffect } from '../../src/sim/dynamics/aero';
import { hoverPowerW, DEFAULT_BATTERY } from '../../src/mission/autonomy/battery';
import { cellOcv, cellVoltage, MISSION_PACK, motorSpeedScale, packCurrent, Px4BatteryEstimator, px4VoltageSoc } from '../../src/mission/autonomy/pack';
import { airframe, MANEUVERS, type Maneuver, type Sim } from './maneuvers';
import { flyNative } from './native';

const args = process.argv.slice(2);
const opt = (name: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const inDir = opt('in') ?? 'test-results/physics';
const write = args.includes('--write');
const summaryPath = opt('summary');

/** One flight on a common time base: seconds from the maneuver start, NED, degrees. */
interface Series {
  sim: Sim | 'shipped';
  t: number[];
  n: number[];
  e: number[];
  d: number[];
  vn: number[];
  ve: number[];
  vd: number[];
  roll: number[];
  pitch: number[];
  /** Highest motor output as a fraction of what the pack allows now. */
  sat: number[];
  starts: number[];
  source: string;
}

function fromRows(sim: Series['sim'], rows: number[][], toT: (r: number[]) => number, starts: number[], source: string): Series {
  const s: Series = { sim, t: [], n: [], e: [], d: [], vn: [], ve: [], vd: [], roll: [], pitch: [], sat: [], starts, source };
  for (const r of rows) {
    s.t.push(toT(r));
    s.n.push(r[1]!);
    s.e.push(r[2]!);
    s.d.push(r[3]!);
    s.vn.push(r[4]!);
    s.ve.push(r[5]!);
    s.vd.push(r[6]!);
    s.roll.push(r[7]!);
    s.pitch.push(r[8]!);
    const scale = r[17] ?? 1;
    s.sat.push(Math.max(r[10]!, r[11]!, r[12]!, r[13]!) / Math.max(1e-6, scale));
  }
  return s;
}

interface Px4Log {
  ok: boolean;
  t0Ms: number;
  segmentStartsMs: number[];
  local: number[][];
  att: number[][];
  truth: number[][];
  truthOrigin: number[] | null;
  failsafe: string[];
}

function loadSih(path: string): Series | null {
  if (!existsSync(path)) return null;
  const log = JSON.parse(readFileSync(path, 'utf8')) as Px4Log;
  const t0 = log.t0Ms;
  const starts = log.segmentStartsMs.map((ms) => (ms - t0) / 1000);
  const o = log.truthOrigin;
  if (log.truth.length > 10 && o) {
    const lat0 = o[1]! / 1e7;
    const lon0 = o[2]! / 1e7;
    const alt0 = o[3]! / 1000;
    const mPerDegLat = 111_132.92;
    const mPerDegLon = 111_412.84 * Math.cos((lat0 * Math.PI) / 180);
    const rows = log.truth.map((r) => {
      const [qw, qx, qy, qz] = [r[7]!, r[8]!, r[9]!, r[10]!];
      const roll = Math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
      const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (qw * qy - qz * qx))));
      return [r[0]!, (r[1]! / 1e7 - lat0) * mPerDegLat, (r[2]! / 1e7 - lon0) * mPerDegLon, -(r[3]! / 1000 - alt0), r[4]! / 100, r[5]! / 100, r[6]! / 100, (roll * 180) / Math.PI, (pitch * 180) / Math.PI, 0, 0, 0, 0, 0];
    });
    return fromRows('sih', rows, (r) => (r[0]! - t0) / 1000, starts, "PX4's ground truth (HIL_STATE_QUATERNION)");
  }
  const att = new Map(log.att.map((r) => [r[0]!, r]));
  const rows = log.local.map((r) => {
    const a = att.get(r[0]!) ?? [0, 0, 0];
    return [r[0]!, r[1]!, r[2]!, r[3]!, r[4]!, r[5]!, r[6]!, a[1]!, a[2]!, 0, 0, 0, 0, 0];
  });
  return fromRows('sih', rows, (r) => (r[0]! - t0) / 1000, starts, "PX4's estimate");
}

/** PX4's own estimate of a flight (LOCAL_POSITION_NED and ATTITUDE). */
function loadEstimate(path: string): Series | null {
  if (!existsSync(path)) return null;
  const log = JSON.parse(readFileSync(path, 'utf8')) as Px4Log;
  const t0 = log.t0Ms;
  const att = new Map(log.att.map((r) => [r[0]!, r]));
  const rows = log.local.map((r) => {
    const a = att.get(r[0]!) ?? [0, 0, 0];
    return [r[0]!, r[1]!, r[2]!, r[3]!, r[4]!, r[5]!, r[6]!, a[1]!, a[2]!, 0, 0, 0, 0, 0];
  });
  return fromRows('bridge', rows, (r) => (r[0]! - t0) / 1000, log.segmentStartsMs.map((ms) => (ms - t0) / 1000), "PX4's estimate");
}

function loadBridge(dir: string, id: string): Series | null {
  const path = join(dir, `${id}.bridge.json`);
  const plant = join(dir, `${id}.bridge.plant.json`);
  if (!existsSync(path) || !existsSync(plant)) return null;
  const log = JSON.parse(readFileSync(path, 'utf8')) as Px4Log;
  const p = JSON.parse(readFileSync(plant, 'utf8')) as { rows: number[][] };
  const t0 = log.t0Ms;
  const starts = log.segmentStartsMs.map((ms) => (ms - t0) / 1000);
  return fromRows('bridge', p.rows, (r) => (r[0]! / 1000 - t0) / 1000, starts, 'simulator truth');
}

function loadNative(path: string): Series | null {
  if (!existsSync(path)) return null;
  const log = JSON.parse(readFileSync(path, 'utf8')) as { rows: number[][]; segmentStartsUs: number[] };
  return fromRows('native', log.rows, (r) => r[0]! / 1e6, log.segmentStartsUs.map((u) => u / 1e6), 'simulator truth');
}

function shipped(m: Maneuver): Series {
  const log = flyNative({ ...m, aero: undefined, soc: undefined });
  return fromRows('shipped', log.rows, (r) => r[0]! / 1e6, log.segmentStartsUs.map((u) => u / 1e6), 'simulator truth');
}

// ---------------------------------------------------------------- metrics

const round = (v: number, k = 2): number => (Number.isFinite(v) ? Math.round(v * 10 ** k) / 10 ** k : NaN);

function window(s: Series, a: number, b: number): number[] {
  const idx: number[] = [];
  for (let i = 0; i < s.t.length; i++) if (s.t[i]! >= a && s.t[i]! < b) idx.push(i);
  return idx;
}

/** Step response of one signal from segment `seg` start, toward `target` from the value at the step. */
function step(s: Series, sig: number[], seg: number, target: number, band: number): Record<string, number> {
  const a = s.starts[seg]!;
  const b = s.starts[seg + 1] ?? s.t[s.t.length - 1]!;
  const idx = window(s, a, b);
  const v0 = sig[idx[0]!]!;
  const span = target - v0;
  let t10 = NaN;
  let t90 = NaN;
  let peak = -Infinity;
  for (const i of idx) {
    const f = (sig[i]! - v0) / span;
    if (Number.isNaN(t10) && f >= 0.1) t10 = s.t[i]! - a;
    if (Number.isNaN(t90) && f >= 0.9) t90 = s.t[i]! - a;
    peak = Math.max(peak, f);
  }
  let settle = 0;
  for (const i of idx) if (Math.abs(sig[i]! - target) > band) settle = s.t[i]! - a;
  const tail = idx.filter((i) => s.t[i]! >= b - 1);
  const end = tail.reduce((acc, i) => acc + sig[i]!, 0) / Math.max(1, tail.length);
  let tilt = 0;
  for (const i of idx) tilt = Math.max(tilt, tiltDeg(s, i));
  return { rise: round(t90 - t10), t90: round(t90), overshootPct: round(Math.max(0, peak - 1) * 100, 1), settle: round(settle), end: round(end), peakTiltDeg: round(tilt, 1) };
}

function tiltDeg(s: Series, i: number): number {
  const r = (s.roll[i]! * Math.PI) / 180;
  const p = (s.pitch[i]! * Math.PI) / 180;
  return (Math.acos(Math.max(-1, Math.min(1, Math.cos(r) * Math.cos(p)))) * 180) / Math.PI;
}

function hold(s: Series, a: number, b: number, at: [number, number, number]): Record<string, number> {
  const idx = window(s, a, b);
  // About the mean position: a steady offset is the estimator's (PX4 holds where it thinks the point is), the wander is the controller's.
  const mn = idx.reduce((acc, i) => acc + s.n[i]!, 0) / Math.max(1, idx.length);
  const me = idx.reduce((acc, i) => acc + s.e[i]!, 0) / Math.max(1, idx.length);
  const h = idx.map((i) => Math.hypot(s.n[i]! - mn, s.e[i]! - me));
  const v = idx.map((i) => Math.abs(s.d[i]! - at[2]));
  const sorted = [...h].sort((x, y) => x - y);
  const rms = (xs: number[]): number => Math.sqrt(xs.reduce((acc, x) => acc + x * x, 0) / Math.max(1, xs.length));
  let tilt = 0;
  for (const i of idx) tilt = Math.max(tilt, tiltDeg(s, i));
  return {
    offsetM: round(Math.hypot(mn - at[0], me - at[1])),
    horizRms: round(rms(h)),
    horizP95: round(sorted[Math.floor(sorted.length * 0.95)] ?? NaN),
    horizMax: round(sorted[sorted.length - 1] ?? NaN),
    vertRms: round(rms(v)),
    peakTiltDeg: round(tilt, 1),
  };
}

function measure(m: Maneuver, s: Series): Record<string, number> {
  const end = m.segments.reduce((acc, g) => acc + g.dur, 0);
  if (m.id === 'step-forward') {
    const go = step(s, s.vn, 1, 4, 0.2);
    const stop = step(s, s.vn, 2, 0, 0.2);
    const i0 = window(s, s.starts[2]!, end)[0]!;
    const idx = window(s, s.starts[2]!, end);
    const nEnd = s.n[idx[idx.length - 1]!]!;
    let reverse = 0;
    for (const i of idx) reverse = Math.min(reverse, s.vn[i]!);
    let alt = 0;
    for (const i of window(s, s.starts[1]!, end)) alt = Math.max(alt, Math.abs(s.d[i]! - m.start[2]));
    return {
      riseSec: go.rise,
      t90Sec: go.t90,
      overshootPct: go.overshootPct,
      settleSec: go.settle,
      speedAfter6sMps: go.end,
      peakTiltDeg: Math.max(go.peakTiltDeg, stop.peakTiltDeg),
      stopSec: stop.settle,
      stopDistM: round(nEnd - s.n[i0]!),
      reverseMps: round(-reverse),
      heightDevM: round(alt),
    };
  }
  if (m.id === 'step-climb') {
    const up = s.vd.map((v) => -v);
    const go = step(s, up, 1, 1.5, 0.15);
    const stop = step(s, up, 2, 0, 0.15);
    let lateral = 0;
    const j0 = window(s, s.starts[1]!, end)[0]!;
    for (const i of window(s, s.starts[1]!, end)) lateral = Math.max(lateral, Math.hypot(s.n[i]! - s.n[j0]!, s.e[i]! - s.e[j0]!));
    return { riseSec: go.rise, t90Sec: go.t90, overshootPct: go.overshootPct, settleSec: go.settle, climbMps: go.end, stopSec: stop.settle, lateralDevM: round(lateral) };
  }
  if (m.id.startsWith('gust')) return hold(s, 5, end, m.start);
  if (m.id.startsWith('sag')) {
    const idx = window(s, s.starts[1]!, s.starts[2]!);
    const last = idx.filter((i) => s.t[i]! >= s.starts[2]! - 1.5);
    const mean = (xs: number[]): number => xs.reduce((a, x) => a + x, 0) / Math.max(1, xs.length);
    const satFrac = idx.filter((i) => s.sat[i]! >= 0.99).length / Math.max(1, idx.length);
    const climbed = -(s.d[idx[idx.length - 1]!]! - s.d[idx[0]!]!);
    return {
      speedMps: round(mean(last.map((i) => s.vn[i]!))),
      climbMps: round(mean(last.map((i) => -s.vd[i]!))),
      climbedM: round(climbed),
      saturatedPct: round(satFrac * 100, 0),
      peakTiltDeg: round(Math.max(...idx.map((i) => tiltDeg(s, i))), 1),
    };
  }
  if (m.id === 'ceiling') {
    const ceiling = m.aero?.ceilingM ?? Infinity;
    const idx = window(s, s.starts[1]!, s.starts[2]!);
    const top = Math.max(...idx.map((i) => -s.d[i]!));
    const settled = idx.filter((i) => s.t[i]! >= s.starts[2]! - 3);
    const mean = settled.reduce((a, i) => a + -s.d[i]!, 0) / Math.max(1, settled.length);
    const target = -(m.segments[1]!.pos![2] as number);
    // The model holds no ceiling: a rotor plane at or past it is contact, and what follows is not flight.
    const hit = idx.find((i) => -s.d[i]! >= ceiling);
    const contact = hit !== undefined;
    return {
      minClearanceM: round(Math.max(0, ceiling - top), 3),
      contactSec: contact ? round(s.t[hit]! - s.starts[1]!, 2) : NaN,
      overshootM: contact ? NaN : round(top - target, 3),
      heightErrM: contact ? NaN : round(mean - target, 3),
    };
  }
  if (m.id === 'ground') {
    const idx = window(s, s.starts[1]!, s.starts[2]!);
    const low = Math.min(...idx.map((i) => -s.d[i]!));
    const settled = idx.filter((i) => s.t[i]! >= s.starts[2]! - 4);
    const mean = settled.reduce((a, i) => a + -s.d[i]!, 0) / Math.max(1, settled.length);
    const back = window(s, s.starts[2]!, end);
    const top = Math.max(...back.map((i) => -s.d[i]!));
    const target = -(m.segments[1]!.pos![2] as number);
    return { lowestM: round(low, 3), heightErrM: round(mean - target, 3), climbOutOvershootM: round(top - 2, 3) };
  }
  return {};
}

// ------------------------------------------------------------- analyses

function surfaceCurves(): { zr: number; legacy: number; ground: number; ceiling: number }[] {
  const r = airframe().rotorRadiusM;
  return [0.25, 0.5, 0.75, 1, 1.5, 2, 3, 4].map((zr) => ({
    zr,
    legacy: round(legacyGroundEffect(zr * r), 3),
    ground: round(groundEffect(zr * r, r), 3),
    ceiling: round(ceilingEffect(zr * r, r), 3),
  }));
}

/** Hover a full pack to empty at the mission's hover power and read PX4's gauge along the way. */
function batteryDischarge(): Record<string, unknown> {
  const pack = MISSION_PACK;
  const p = hoverPowerW(DEFAULT_BATTERY);
  const capacityMah = pack.capacityAh * 1000;
  const voltOnly = new Px4BatteryEstimator(pack.cells, -1);
  const withCap = new Px4BatteryEstimator(pack.cells, capacityMah);
  const dt = 1;
  let soc = 1;
  const rows: { t: number; soc: number; v: number; i: number; px4V: number; px4C: number }[] = [];
  let t = 0;
  while (soc > 0.005) {
    const i = packCurrent(pack, soc, p);
    if (!Number.isFinite(i)) break;
    const v = pack.cells * cellVoltage(pack, soc, i);
    // PX4 with no capacity set reports the voltage estimate alone.
    const pv = px4VoltageSoc(pack.cells, v, i);
    voltOnly.update(v, i, dt);
    const pc = withCap.update(v, i, dt);
    rows.push({ t, soc, v, i, px4V: pv, px4C: pc });
    soc -= (i * dt) / 3600 / pack.capacityAh;
    t += dt;
  }
  const at = (target: number): (typeof rows)[number] => rows.reduce((best, r) => (Math.abs(r.soc - target) < Math.abs(best.soc - target) ? r : best), rows[0]!);
  const first = (pred: (r: (typeof rows)[number]) => boolean): (typeof rows)[number] | undefined => rows.find(pred);
  const lowTrue = first((r) => r.soc <= 0.15);
  const lowV = first((r) => r.px4V <= 0.15);
  const lowC = first((r) => r.px4C <= 0.15);
  const hoverA = packCurrent(pack, 1, p);
  return {
    hoverPowerW: round(p, 1),
    hoverCurrentA: round(hoverA, 1),
    hoverMinutes: round(rows[rows.length - 1]!.t / 60, 1),
    gauge: [0.9, 0.5, 0.3, 0.15, 0.07].map((s) => {
      const r = at(s);
      return { trueSoc: s, cellV: round(r.v / pack.cells, 3), px4VoltageOnly: round(r.px4V, 3), px4WithCapacity: round(r.px4C, 3) };
    }),
    lowWarningMinBeforeTrue15: {
      voltageOnly: lowTrue && lowV ? round((lowTrue.t - lowV.t) / 60, 1) : null,
      withCapacity: lowTrue && lowC ? round((lowTrue.t - lowC.t) / 60, 1) : null,
    },
    thrustCeiling: [1, 0.5, 0.3, 0.15, 0.08].map((s) => {
      // Full-throttle current: the four rotors at the pack's top speed (fixed point, a few iterations).
      const { dyn, rotorRadiusM } = airframe();
      const area = Math.PI * rotorRadiusM * rotorRadiusM;
      let scale = 1;
      let i = 0;
      for (let k = 0; k < 20; k++) {
        const tRotor = dyn.kf * (1100 * scale) ** 2;
        const pw = (4 * tRotor ** 1.5) / Math.sqrt(2 * DEFAULT_BATTERY.airDensity * area) / DEFAULT_BATTERY.figureOfMerit / DEFAULT_BATTERY.electricalEfficiency + DEFAULT_BATTERY.avionicsW;
        i = packCurrent(pack, s, pw);
        if (!Number.isFinite(i)) break;
        scale = motorSpeedScale(pack, s, i);
      }
      const tw = (4 * airframe().dyn.kf * (1100 * scale) ** 2) / (airframe().dyn.mass * 9.81);
      return { soc: s, ocvCell: round(cellOcv(s), 3), fullThrottleA: round(i, 1), thrustToWeight: round(tw, 2) };
    }),
    shippedThrustToWeight: round((4 * airframe().dyn.kf * 1100 ** 2) / (airframe().dyn.mass * 9.81), 2),
  };
}

// ---------------------------------------------------------------- plots

const COLORS: Record<string, string> = { sih: '#d97706', bridge: '#2563eb', native: '#059669', shipped: '#9ca3af' };
const LABELS: Record<string, string> = {
  sih: 'PX4 on SIH',
  bridge: 'PX4 on frontier-fly physics',
  native: 'frontier-fly controller',
  shipped: 'frontier-fly, shipped physics',
};

function svgPlot(title: string, yLabel: string, series: { key: string; t: number[]; y: number[] }[], t0: number, t1: number, ref?: { y: number; label: string }): string {
  const W = 640;
  const Hh = 260;
  const L = 52;
  const R = 12;
  const T = 30;
  const B = 58;
  let lo = Infinity;
  let hi = -Infinity;
  for (const s of series)
    for (let i = 0; i < s.t.length; i++)
      if (s.t[i]! >= t0 && s.t[i]! <= t1) {
        lo = Math.min(lo, s.y[i]!);
        hi = Math.max(hi, s.y[i]!);
      }
  if (!Number.isFinite(lo)) return '';
  const pad = (hi - lo) * 0.08 || 0.5;
  lo -= pad;
  hi += pad;
  const x = (t: number): number => L + ((t - t0) / (t1 - t0)) * (W - L - R);
  const y = (v: number): number => T + (1 - (v - lo) / (hi - lo)) * (Hh - T - B);
  const ticks = (a: number, b: number, n: number): number[] => {
    const raw = (b - a) / n;
    const mag = 10 ** Math.floor(Math.log10(raw));
    const st = [1, 2, 5, 10].map((k) => k * mag).find((k) => k >= raw)!;
    const out: number[] = [];
    for (let v = Math.ceil(a / st) * st; v <= b + 1e-9; v += st) out.push(+v.toFixed(6));
    return out;
  };
  const parts: string[] = [];
  parts.push(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${Hh}" font-family="system-ui, sans-serif" font-size="11">`);
  parts.push(`<rect width="${W}" height="${Hh}" fill="#ffffff"/>`);
  parts.push(`<text x="${L}" y="18" font-size="13" font-weight="600" fill="#111827">${title}</text>`);
  for (const v of ticks(lo, hi, 5)) {
    parts.push(`<line x1="${L}" x2="${W - R}" y1="${y(v).toFixed(1)}" y2="${y(v).toFixed(1)}" stroke="#e5e7eb"/>`);
    parts.push(`<text x="${L - 6}" y="${(y(v) + 4).toFixed(1)}" text-anchor="end" fill="#6b7280">${v}</text>`);
  }
  for (const v of ticks(t0, t1, 8)) parts.push(`<text x="${x(v).toFixed(1)}" y="${Hh - B + 15}" text-anchor="middle" fill="#6b7280">${v}</text>`);
  parts.push(`<text x="${(L + W - R) / 2}" y="${Hh - B + 30}" text-anchor="middle" fill="#374151">seconds</text>`);
  parts.push(`<text transform="translate(14 ${(T + Hh - B) / 2}) rotate(-90)" text-anchor="middle" fill="#374151">${yLabel}</text>`);
  if (ref && ref.y >= lo && ref.y <= hi) {
    parts.push(`<line x1="${L}" x2="${W - R}" y1="${y(ref.y).toFixed(1)}" y2="${y(ref.y).toFixed(1)}" stroke="#111827" stroke-width="1.2"/>`);
    parts.push(`<text x="${W - R - 4}" y="${(y(ref.y) - 4).toFixed(1)}" text-anchor="end" fill="#111827">${ref.label}</text>`);
  }
  let lx = L;
  for (const s of series) {
    const pts: string[] = [];
    for (let i = 0; i < s.t.length; i++) if (s.t[i]! >= t0 && s.t[i]! <= t1) pts.push(`${x(s.t[i]!).toFixed(1)},${y(s.y[i]!).toFixed(1)}`);
    const dash = s.key === 'shipped' ? ' stroke-dasharray="5 4"' : '';
    parts.push(`<polyline fill="none" stroke="${COLORS[s.key]}" stroke-width="1.8"${dash} points="${pts.join(' ')}"/>`);
    parts.push(`<line x1="${lx}" x2="${lx + 18}" y1="${Hh - 10}" y2="${Hh - 10}" stroke="${COLORS[s.key]}" stroke-width="2.5"${dash}/>`);
    parts.push(`<text x="${lx + 22}" y="${Hh - 6}" fill="#374151">${LABELS[s.key]}</text>`);
    lx += 40 + LABELS[s.key]!.length * 5.8;
  }
  parts.push('</svg>');
  return `${parts.join('\n')}\n`;
}

// ---------------------------------------------------------------- main

interface Result {
  id: string;
  title: string;
  sims: Record<string, { source: string; metrics: Record<string, number> } | null>;
}

const results: Result[] = [];
const plots: Record<string, string> = {};
for (const m of MANEUVERS) {
  const all: Series[] = [];
  const sih = m.sims.includes('sih') ? loadSih(join(inDir, `${m.id}.sih.json`)) : null;
  const bridge = loadBridge(inDir, m.id);
  const native = loadNative(join(inDir, `${m.id}.native.json`));
  const nat = native ?? (() => {
    const log = flyNative(m);
    return fromRows('native', log.rows, (r) => r[0]! / 1e6, log.segmentStartsUs.map((u) => u / 1e6), 'simulator truth');
  })();
  if (sih) all.push(sih);
  if (bridge) all.push(bridge);
  all.push(nat);
  const near = m.aero !== undefined || m.soc !== undefined;
  if (near) all.push(shipped(m));
  const r: Result = { id: m.id, title: m.title, sims: {} };
  for (const s of all) r.sims[s.sim] = { source: s.source, metrics: measure(m, s) };
  if (m.wind && bridge && r.sims.bridge) {
    // How well PX4 holds the point it believes it is at: the same hold measured on its own estimate.
    const est = loadEstimate(join(inDir, `${m.id}.bridge.json`));
    if (est) r.sims.bridge.metrics.estHorizRms = measure(m, est).horizRms!;
  }
  results.push(r);
  const end = m.segments.reduce((a, g) => a + g.dur, 0);
  const pick = (f: (s: Series) => number[], scale = 1): { key: string; t: number[]; y: number[] }[] => all.map((s) => ({ key: s.sim, t: s.t, y: f(s).map((v) => v * scale) }));
  if (m.id === 'step-forward') plots[m.id] = svgPlot('Forward speed step: 4 m/s north, then stop', 'north speed, m/s', pick((s) => s.vn), -1, end);
  if (m.id === 'step-climb') plots[m.id] = svgPlot('Climb step: 1.5 m/s up, then hold', 'climb rate, m/s', pick((s) => s.vd, -1), -1, end);
  if (m.id === 'gust-moderate')
    plots[m.id] = svgPlot('Position hold, 4 m/s wind with 1.5 m/s gusts', 'north position about its mean, m', all.map((s) => {
      const mean = s.n.reduce((a, x) => a + x, 0) / Math.max(1, s.n.length);
      return { key: s.sim, t: s.t, y: s.n.map((n) => n - mean) };
    }), 0, end);
  if (m.id === 'sag-8') plots[m.id] = svgPlot('Dash and climb on an 8% pack', 'height above pad, m', pick((s) => s.d, -1), -1, end);
  if (m.id === 'ceiling') plots[m.id] = svgPlot('Creeping up under a ceiling at 6 m, then holding 0.12 m below it', 'height above pad, m', pick((s) => s.d, -1), -1, end, { y: m.aero?.ceilingM ?? 6, label: 'ceiling (no contact model: above it is a hit)' });
  if (m.id === 'ground') plots[m.id] = svgPlot('Low hover at 0.12 m', 'height above pad, m', pick((s) => s.d, -1), -1, end);
}

const out = {
  generatedFrom: 'scripts/physics/report.ts',
  px4Version: 'v1.15.4',
  airframe: (() => {
    const { dyn, rotorRadiusM } = airframe();
    return { massKg: dyn.mass, inertia: dyn.inertia, armM: dyn.arm, rotorRadiusM, thrustToWeight: round((4 * dyn.kf * 1100 ** 2) / (dyn.mass * 9.81), 2) };
  })(),
  maneuvers: results,
  surfaces: surfaceCurves(),
  battery: batteryDischarge(),
};

// ---------------------------------------------------------------- tables

const fmt = (v: number | undefined, unit = ''): string => (v === undefined || Number.isNaN(v) ? '–' : `${v}${unit}`);
const SIM_ORDER = ['sih', 'bridge', 'native', 'shipped'] as const;
const ROW_LABEL: Record<string, string> = {
  sih: 'PX4 on SIH (PX4\'s own physics, matched airframe)',
  bridge: 'PX4 on frontier-fly physics',
  native: 'frontier-fly controller on frontier-fly physics',
  shipped: 'frontier-fly controller, shipped physics (no sag, linear ground effect)',
};

function table(id: string, cols: [string, string, string?][]): string {
  const r = results.find((x) => x.id === id)!;
  const head = `| Flown by | ${cols.map((c) => c[1]).join(' | ')} |\n| --- | ${cols.map(() => '---').join(' | ')} |`;
  const lines = SIM_ORDER.filter((s) => r.sims[s]).map((s) => `| ${ROW_LABEL[s]} | ${cols.map(([k, , u]) => fmt(r.sims[s]!.metrics[k], u ?? '')).join(' | ')} |`);
  return [head, ...lines].join('\n');
}

const md: string[] = [];
md.push('### Step response\n');
md.push(table('step-forward', [['t90Sec', 'Time to 3.6 m/s (s)'], ['overshootPct', 'Overshoot (%)'], ['speedAfter6sMps', 'Speed after 6 s (m/s)'], ['peakTiltDeg', 'Peak tilt (°)'], ['stopSec', 'Stop within 0.2 m/s (s)'], ['stopDistM', 'Stop distance (m)'], ['reverseMps', 'Reverse speed (m/s)']]));
md.push('\n![Forward step](media/physics/step-forward.svg)\n');
md.push(table('step-climb', [['t90Sec', 'Time to 1.35 m/s (s)'], ['overshootPct', 'Overshoot (%)'], ['climbMps', 'Climb rate (m/s)'], ['stopSec', 'Stop within 0.15 m/s (s)'], ['lateralDevM', 'Drift (m)']]));
md.push('\n![Climb step](media/physics/step-climb.svg)\n');
md.push('### Hover in gusts\n');
md.push('Light wind (2 m/s, 0.75 m/s gusts):\n');
md.push(table('gust-light', [['offsetM', 'Offset of the mean (m)'], ['horizRms', 'Wander RMS (m)'], ['estHorizRms', 'Wander RMS in PX4\'s own estimate (m)'], ['horizMax', 'Max wander (m)'], ['peakTiltDeg', 'Peak tilt (°)']]));
md.push('\nModerate wind (4 m/s, 1.5 m/s gusts):\n');
md.push(table('gust-moderate', [['offsetM', 'Offset of the mean (m)'], ['horizRms', 'Wander RMS (m)'], ['estHorizRms', 'Wander RMS in PX4\'s own estimate (m)'], ['horizMax', 'Max wander (m)'], ['peakTiltDeg', 'Peak tilt (°)']]));
md.push('\n![Gust hold](media/physics/gust-moderate.svg)\n');
md.push('### Battery sag\n');
for (const id of ['sag-100', 'sag-30', 'sag-8']) {
  md.push(`${results.find((x) => x.id === id)!.title}:\n`);
  md.push(table(id, [['speedMps', 'Speed (m/s)'], ['climbMps', 'Climb (m/s)'], ['climbedM', 'Climbed in 4 s (m)'], ['saturatedPct', 'Motors saturated (% of dash)'], ['peakTiltDeg', 'Peak tilt (°)']]));
  md.push('');
}
md.push('![Sag dash](media/physics/sag-8.svg)\n');
const bat = out.battery as ReturnType<typeof batteryDischarge> & { gauge: { trueSoc: number; cellV: number; px4VoltageOnly: number; px4WithCapacity: number }[]; thrustCeiling: { soc: number; ocvCell: number; fullThrottleA: number; thrustToWeight: number }[]; lowWarningMinBeforeTrue15: { voltageOnly: number | null; withCapacity: number | null } };
md.push('Thrust ceiling as the pack drains (full throttle, loaded voltage):\n');
md.push(`| Charge | Cell voltage, no load (V) | Full-throttle current (A) | Thrust-to-weight |\n| --- | --- | --- | --- |\n${bat.thrustCeiling.map((r) => `| ${Math.round(r.soc * 100)}% | ${r.ocvCell} | ${r.fullThrottleA} | ${r.thrustToWeight} |`).join('\n')}\n| shipped model, any charge | – | – | ${bat.shippedThrustToWeight} |\n`);
md.push(`What PX4's battery gauge would read on this pack in a hover (${bat.hoverPowerW} W, ${bat.hoverCurrentA} A, ${bat.hoverMinutes} min to empty), with PX4's default cell voltages (empty 3.6 V, full 4.05 V):\n`);
md.push(`| True charge | Cell voltage under load (V) | PX4, voltage only (default, no capacity set) | PX4, capacity set to 5000 mAh |\n| --- | --- | --- | --- |\n${bat.gauge.map((g) => `| ${Math.round(g.trueSoc * 100)}% | ${g.cellV} | ${Math.round(g.px4VoltageOnly * 100)}% | ${Math.round(g.px4WithCapacity * 100)}% |`).join('\n')}\n`);
md.push('### Ground and ceiling effect\n');
md.push(`Thrust multiplier against the distance from the rotor to the surface, in rotor radii (R = ${airframe().rotorRadiusM} m):\n`);
md.push(`| z / R | Shipped (linear, world height) | Ground, Cheeseman–Bennett | Ceiling, Sanchez-Cuevas fit |\n| --- | --- | --- | --- |\n${out.surfaces.map((s) => `| ${s.zr} | ${s.legacy} | ${s.ground} | ${s.ceiling} |`).join('\n')}\n`);
md.push(table('ceiling', [['minClearanceM', 'Closest to ceiling (m)'], ['contactSec', 'Hit the ceiling after (s)'], ['overshootM', 'Overshoot toward it (m)'], ['heightErrM', 'Settled height error (m)']]));
md.push('\n![Ceiling](media/physics/ceiling.svg)\n');
md.push(table('ground', [['lowestM', 'Lowest (m)'], ['heightErrM', 'Settled height error (m)'], ['climbOutOvershootM', 'Climb-out overshoot (m)']]));
md.push('\n![Ground](media/physics/ground.svg)\n');
const tables = md.join('\n');

console.log(tables);
if (summaryPath) appendFileSync(summaryPath, `## Physics validation\n\n${tables}\n`);
if (write) {
  writeFileSync('docs/physics-validation.json', `${JSON.stringify(out, null, 1)}\n`);
  mkdirSync('docs/media/physics', { recursive: true });
  for (const [id, svg] of Object.entries(plots)) if (svg) writeFileSync(`docs/media/physics/${id}.svg`, svg);
  const doc = 'docs/PHYSICS_VALIDATION.md';
  if (existsSync(doc)) {
    const text = readFileSync(doc, 'utf8');
    const a = '<!-- physics-results:start -->';
    const b = '<!-- physics-results:end -->';
    const i = text.indexOf(a);
    const j = text.indexOf(b);
    if (i >= 0 && j > i) writeFileSync(doc, `${text.slice(0, i + a.length)}\n${tables}\n${text.slice(j)}`);
  }
  console.log('wrote docs/physics-validation.json, docs/media/physics/ and docs/PHYSICS_VALIDATION.md');
}
