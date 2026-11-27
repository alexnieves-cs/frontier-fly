/**
 * The outdoor leg of the joint mission: a DSS footage-lab run exported as a dss-leg@1 bundle
 * (denied-signal-stack tools/footage/export_leg.py). Frames rendered from one year's aerial
 * imagery are matched against another year's, and a GNSS spoof is replayed through DSS's gate.
 * Positions are local metres of the bundle's AOI: x east, y north.
 */

export type GateState = 'NO_FIX' | 'TRUSTED' | 'SUSPECT' | 'REJECTED';

export interface LegSolution {
  pub: [number, number] | null;
  /** Protection level, m. */
  pl: number | null;
  state: GateState;
}

export interface LegEpoch {
  t: number;
  frame: number;
  truth: [number, number];
  gps: [number, number];
  spoofed: [number, number];
  agl: number;
  heading: number | null;
  fix: {
    xy: [number, number] | null;
    used: boolean;
    ok: boolean;
    peak: number | null;
    ratio: number | null;
    reason: string;
    sprite: number | null;
    searchM: number | null;
  };
  /** GPS fused through the gate, spoof on. */
  gated: LegSolution;
  /** GPS always fused, spoof on: what a receiver-trusting autopilot believes. */
  blind: LegSolution;
  /** GPS never fused: the GNSS-denied solution. */
  vision: LegSolution;
  /** Gated, no spoof. */
  nominal: LegSolution;
}

export interface Extent {
  xmin: number;
  xmax: number;
  ymin: number;
  ymax: number;
}

export interface LegBundle {
  schema: 'dss-leg@1';
  title: string;
  note: string;
  frameNote: string;
  kHpl: number;
  alertLimitM: number;
  spoof: { startS: number; rateMps: number; bearingDeg: number };
  firstRejectS: Record<string, number | null>;
  ground: { file: string; w: number; h: number; extent: Extent; title: string; attribution: string };
  map: { file: string; w: number; h: number; extent: Extent; title: string; attribution: string };
  patches: { file: string; px: number; cols: number } | null;
  camera: { hfovDeg: number; patchM: number; framesDir: string; frameW: number };
  summary: Record<string, number | string | null>;
  track: { t: number[]; x: number[]; y: number[]; heading: number[]; agl: number[] };
  epochs: LegEpoch[];
}

export async function fetchLeg(url: string): Promise<LegBundle> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Could not load the leg (${res.status} ${url})`);
  const leg = (await res.json()) as LegBundle;
  if (leg.schema !== 'dss-leg@1') throw new Error(`Not a DSS leg bundle (schema ${String(leg.schema)})`);
  return leg;
}

/** Index of the last epoch at or before t (0 before the first). */
export function epochAt(leg: LegBundle, t: number): number {
  const E = leg.epochs;
  let lo = 0;
  let hi = E.length - 1;
  if (t <= E[0]!.t) return 0;
  if (t >= E[hi]!.t) return hi;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (E[mid]!.t <= t) lo = mid;
    else hi = mid;
  }
  return lo;
}

function lerp(a: number, b: number, u: number): number {
  return a + (b - a) * u;
}

function lerpAngle(a: number, b: number, u: number): number {
  const d = ((((b - a) % 360) + 540) % 360) - 180;
  return a + d * u;
}

/** The true pose at time t, from the 10 Hz track. Heading in degrees clockwise from north. */
export function truthAt(leg: LegBundle, t: number): { x: number; y: number; heading: number; agl: number; vx: number; vy: number } {
  const T = leg.track.t;
  const n = T.length;
  const tt = Math.max(T[0]!, Math.min(T[n - 1]!, t));
  let i = Math.min(n - 2, Math.max(0, Math.floor(((tt - T[0]!) / (T[n - 1]! - T[0]!)) * (n - 1))));
  while (i > 0 && T[i]! > tt) i--;
  while (i < n - 2 && T[i + 1]! < tt) i++;
  const u = (tt - T[i]!) / Math.max(1e-6, T[i + 1]! - T[i]!);
  const { x, y, heading, agl } = leg.track;
  const dt = Math.max(1e-6, T[i + 1]! - T[i]!);
  return {
    x: lerp(x[i]!, x[i + 1]!, u),
    y: lerp(y[i]!, y[i + 1]!, u),
    heading: lerpAngle(heading[i]!, heading[i + 1]!, u),
    agl: lerp(agl[i]!, agl[i + 1]!, u),
    vx: (x[i + 1]! - x[i]!) / dt,
    vy: (y[i + 1]! - y[i]!) / dt,
  };
}

/** A solution's published position at t, interpolated between epochs. */
export function solutionAt(leg: LegBundle, t: number, mode: 'gated' | 'blind' | 'vision' | 'nominal'): [number, number] | null {
  const i = epochAt(leg, t);
  const a = leg.epochs[i]![mode].pub;
  const nb = leg.epochs[Math.min(leg.epochs.length - 1, i + 1)]!;
  const b = nb[mode].pub;
  if (!a) return b;
  if (!b || nb.t <= leg.epochs[i]!.t) return a;
  const u = Math.max(0, Math.min(1, (t - leg.epochs[i]!.t) / (nb.t - leg.epochs[i]!.t)));
  return [lerp(a[0], b[0], u), lerp(a[1], b[1], u)];
}

export function spoofedAt(leg: LegBundle, t: number): [number, number] {
  const i = epochAt(leg, t);
  const a = leg.epochs[i]!;
  const b = leg.epochs[Math.min(leg.epochs.length - 1, i + 1)]!;
  const u = b.t > a.t ? Math.max(0, Math.min(1, (t - a.t) / (b.t - a.t))) : 0;
  return [lerp(a.spoofed[0], b.spoofed[0], u), lerp(a.spoofed[1], b.spoofed[1], u)];
}

export function dist(a: [number, number] | null, b: [number, number] | null): number | null {
  return a && b ? Math.hypot(a[0] - b[0], a[1] - b[1]) : null;
}
