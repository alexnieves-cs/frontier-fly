import type { LocFrame, ScanSample } from './types';

/**
 * Real robot logs for the pose estimator: CARMEN laser logs (the format of
 * the Radish datasets and of the Freiburg SLAM benchmark) and the benchmark's
 * relation-based error (Kümmerle et al., "On measuring the accuracy of SLAM
 * algorithms", Autonomous Robots 27(4), 2009).
 *
 * A wheeled robot has no IMU or flow sensor; its wheel odometry stands in for
 * both. The heading change between sweeps is the gyro, the odometry step in
 * the body frame is the flow velocity. The estimator itself is unchanged.
 */

/** FLASER messages of a CARMEN log: one front-laser sweep each, with the odometry pose at that time. */
export interface LaserLog {
  /** Sweeps in the log. */
  n: number;
  /** Beams per sweep. */
  beams: number;
  /** Angle of beam 0 and the step between beams, rad, counter-clockwise from the robot's forward axis. */
  angleMin: number;
  angleStep: number;
  /**
   * Per sweep: ipc timestamp, s, the stamp benchmark relations name. In file
   * order, which is the order the sweeps happened in; the stamps themselves
   * jitter (in the Intel log 631 of them step back by up to 0.9 s).
   */
  t: Float64Array;
  /** Per sweep: seconds since the previous sweep, from the logger clock, at least 10 ms. */
  dt: Float64Array;
  /** Per sweep: odometry x, y, theta. */
  odom: Float64Array;
  /** n * beams ranges, m. */
  ranges: Float32Array;
}

/**
 * Parses the FLASER lines of a CARMEN log:
 * `FLASER num_readings r_1 .. r_n x y theta odom_x odom_y odom_theta ipc_timestamp ipc_hostname logger_timestamp`.
 * Other lines are ignored. The beams of a SICK LMS in this format span 180° from the robot's right to its left.
 */
export function parseFlaser(text: string, fov = Math.PI): LaserLog {
  const t: number[] = [];
  const logger: number[] = [];
  const odom: number[] = [];
  const ranges: number[] = [];
  let beams = -1;
  for (const line of text.split('\n')) {
    if (!line.startsWith('FLASER ')) continue;
    const f = line.trim().split(/\s+/);
    const k = Number(f[1]);
    if (beams < 0) beams = k;
    if (k !== beams || f.length < 2 + k + 9) throw new Error(`FLASER line with ${k} beams (expected ${beams}) or too few fields`);
    for (let i = 0; i < k; i++) ranges.push(Number(f[2 + i]));
    const o = 2 + k;
    odom.push(Number(f[o + 3]), Number(f[o + 4]), Number(f[o + 5]));
    t.push(Number(f[o + 6]));
    logger.push(Number(f[o + 8]));
  }
  if (beams < 0) throw new Error('no FLASER lines');
  // N beams over the field of view: the SICK LMS in 1° mode reports 180 beams, -90° .. +89°.
  const angleStep = fov / beams;
  return {
    n: t.length,
    beams,
    angleMin: -fov / 2,
    angleStep,
    t: Float64Array.from(t),
    dt: Float64Array.from(logger, (l, i) => (i ? Math.max(0.01, Math.abs(l - logger[i - 1]!)) : 0)),
    odom: Float64Array.from(odom),
    ranges: Float32Array.from(ranges),
  };
}

/** Beams of the estimator's 360° ring: the estimator assumes beam i points at yaw + i * 2π / beams. */
export const RING_BEAMS = 360;

/**
 * Sweep i of the log as the estimator's ring: each real beam goes to the ring
 * slot of its angle, every other slot (behind the robot) is a miss. Ranges at
 * or above `maxRange` are misses too (the SICK reports 81.83 m for no return).
 */
export function ringScan(log: LaserLog, i: number, maxRange: number, ringBeams = RING_BEAMS): ScanSample {
  const ring = new Float64Array(ringBeams).fill(maxRange);
  const step = (2 * Math.PI) / ringBeams;
  const o = i * log.beams;
  for (let b = 0; b < log.beams; b++) {
    const r = log.ranges[o + b]!;
    if (!(r > 0.05) || r >= maxRange) continue;
    const a = log.angleMin + b * log.angleStep;
    const slot = ((Math.round(a / step) % ringBeams) + ringBeams) % ringBeams;
    ring[slot] = r;
  }
  return { ranges: ring, maxRange };
}

function wrap(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

/** Odometry step from sweep i-1 to i: heading change, and the step in the body frame after the turn. */
function odomStep(log: LaserLog, i: number): { dth: number; bx: number; by: number } {
  const o = log.odom;
  const dx = o[i * 3]! - o[(i - 1) * 3]!;
  const dy = o[i * 3 + 1]! - o[(i - 1) * 3 + 1]!;
  const th0 = o[(i - 1) * 3 + 2]!;
  const dth = wrap(o[i * 3 + 2]! - th0);
  // The estimator turns first, then steps along the new heading: express the step in that frame.
  const c = Math.cos(th0 + dth);
  const s = Math.sin(th0 + dth);
  return { dth, bx: c * dx + s * dy, by: -s * dx + c * dy };
}

/**
 * Sensor frame for sweep i (> 0) from wheel odometry: the heading change is
 * the gyro, the step is the flow velocity, and the change of that velocity is
 * the accelerometer (what an ideal IMU riding on the odometry would read).
 * Height sensors read a flat floor.
 */
export function odomFrame(log: LaserLog, i: number, maxRange: number): LocFrame {
  const dt = log.dt[i]!;
  const { dth, bx, by } = odomStep(log, i);
  const vx = bx / dt;
  const vy = by / dt;
  // The estimator starts at rest, rotates last tick's velocity into this tick's body frame, then
  // adds accel * dt: with this accel its prediction is the odometry velocity exactly.
  let pvx = 0;
  let pvy = 0;
  if (i > 1) {
    const p = odomStep(log, i - 1);
    pvx = p.bx / log.dt[i - 1]!;
    pvy = p.by / log.dt[i - 1]!;
  }
  const cr = Math.cos(dth);
  const sr = Math.sin(dth);
  const ax = (vx - (cr * pvx + sr * pvy)) / dt;
  const ay = (vy - (-sr * pvx + cr * pvy)) / dt;
  return {
    imu: { dt, gyro: [0, 0, dth / dt], accel: [ax, ay, 0] },
    flow: { vel: [vx, vy], quality: 1 },
    scan: ringScan(log, i, maxRange),
    alt: { down: null, baro: 0 },
  };
}

/** One benchmark relation: the pose at t2 in the frame of the pose at t1. */
export interface Relation {
  t1: number;
  t2: number;
  dx: number;
  dy: number;
  dyaw: number;
}

/** Relations file of the Freiburg benchmark: `t1 t2 x y z roll pitch yaw` per line. */
export function parseRelations(text: string): Relation[] {
  const out: Relation[] = [];
  for (const line of text.split('\n')) {
    const f = line.trim().split(/\s+/);
    if (f.length < 8 || line.trim().startsWith('#')) continue;
    out.push({ t1: Number(f[0]), t2: Number(f[1]), dx: Number(f[2]), dy: Number(f[3]), dyaw: Number(f[7]) });
  }
  return out;
}

export interface ErrorStats {
  mean: number;
  std: number;
  max: number;
}

export interface RelationError {
  /** Relations scored (both stamps matched a pose). */
  n: number;
  /** Relations skipped because a stamp had no pose within the tolerance. */
  missing: number;
  /** Translational error, m: |trans(δ ⊖ δ*)| and its square. */
  transAbs: ErrorStats;
  transSq: ErrorStats;
  /** Rotational error, deg: |rot(δ ⊖ δ*)| and its square (deg²). */
  rotAbs: ErrorStats;
  rotSq: ErrorStats;
  /** Per relation, in file order (NaN where missing): translational m, rotational deg. */
  perTrans: Float64Array;
  perRot: Float64Array;
}

function stats(xs: number[]): ErrorStats {
  if (!xs.length) return { mean: 0, std: 0, max: 0 };
  const mean = xs.reduce((a, b) => a + b, 0) / xs.length;
  const v = xs.reduce((a, b) => a + (b - mean) ** 2, 0) / xs.length;
  return { mean, std: Math.sqrt(v), max: Math.max(...xs) };
}

/**
 * Relation-based error of a trajectory (poses x, y, yaw at stamps t, sorted).
 * For each relation, δ = x(t1)⁻¹ x(t2) is the estimate's displacement and the
 * error is δ ⊖ δ* = δ*⁻¹ δ, split into its translation and rotation, as in
 * the benchmark's metricEvaluator. A stamp matches the sweep with exactly that stamp.
 */
export function relationError(t: Float64Array, pose: Float64Array, rels: Relation[]): RelationError {
  // Stamps name sweeps exactly (relations files copy them from the log); they are not sorted.
  const index = new Map<number, number>();
  t.forEach((ts, i) => index.set(ts, i));
  const find = (ts: number): number => index.get(ts) ?? -1;
  const trans: number[] = [];
  const rot: number[] = [];
  const perTrans = new Float64Array(rels.length).fill(NaN);
  const perRot = new Float64Array(rels.length).fill(NaN);
  let missing = 0;
  rels.forEach((r, k) => {
    const i = find(r.t1);
    const j = find(r.t2);
    if (i < 0 || j < 0) {
      missing += 1;
      return;
    }
    const [xi, yi, ai] = [pose[i * 3]!, pose[i * 3 + 1]!, pose[i * 3 + 2]!];
    const [xj, yj, aj] = [pose[j * 3]!, pose[j * 3 + 1]!, pose[j * 3 + 2]!];
    // δ = x_i⁻¹ x_j
    const ci = Math.cos(ai);
    const si = Math.sin(ai);
    const dx = ci * (xj - xi) + si * (yj - yi);
    const dy = -si * (xj - xi) + ci * (yj - yi);
    const da = wrap(aj - ai);
    // e = δ*⁻¹ δ
    const cr = Math.cos(r.dyaw);
    const sr = Math.sin(r.dyaw);
    const ex = cr * (dx - r.dx) + sr * (dy - r.dy);
    const ey = -sr * (dx - r.dx) + cr * (dy - r.dy);
    const ea = Math.abs(wrap(da - r.dyaw)) * (180 / Math.PI);
    const et = Math.hypot(ex, ey);
    trans.push(et);
    rot.push(ea);
    perTrans[k] = et;
    perRot[k] = ea;
  });
  return {
    n: trans.length,
    missing,
    transAbs: stats(trans),
    transSq: stats(trans.map((x) => x * x)),
    rotAbs: stats(rot),
    rotSq: stats(rot.map((x) => x * x)),
    perTrans,
    perRot,
  };
}
