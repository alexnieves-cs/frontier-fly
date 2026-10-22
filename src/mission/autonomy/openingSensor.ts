import { RNG } from '../../shared/rng';
import type { Opening } from '../../world/outdoor/types';
import type { DronePose, V3 } from './contracts';
import { add, cross, dot, scale, sub, unit } from './planner/geometry';

/*
 * Sensor models for openings. Like the lidar and the detector, these are the
 * only code here that reads the true opening: each turns it into what a camera
 * with a depth channel would report, with errors that grow with range. The
 * mission plans entries and flies through on their output alone.
 */

/** An opening as the drone measured it, placed with its own pose. */
export type MeasuredOpening = Opening & {
  /** Camera range of the measurement, m. Closer looks are kept. */
  rangeM: number;
};

/**
 * 1-sigma errors of one opening measurement at range r, a + b r, as a depth camera fitting the
 * aperture's edges would make them: position in the face plane, depth along the normal, width and
 * height, rotation of the face about the vertical (rad), and the door leaf angle (rad). Each opening
 * also carries a fixed bias of BIAS (same units) that no number of views averages away.
 */
export const OPENING_ERR = {
  plane: [0.005, 0.004],
  depth: [0.005, 0.003],
  size: [0.005, 0.004],
  yaw: [0.003, 0.002],
  leaf: [0.01, 0.004],
  bias: { plane: 0.01, depth: 0.01, size: 0.01, yaw: 0.003, leaf: 0.01 },
} as const;

/** Width, height and normal axes of an opening's face. */
function basis(o: Pick<Opening, 'normal' | 'tangent'>): { t: V3; b: V3; n: V3 } {
  const n = unit(o.normal as V3);
  const t = unit(o.tangent as V3);
  const vertical = Math.abs(n[2]) > 0.7;
  return { t, b: vertical ? unit(cross(n, t)) : [0, 0, 1], n };
}

function rotZ(v: V3, a: number): V3 {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
}

/** Errors of one view, in the order [u, v, n, width, height, yaw, leaf]. */
const N_ERR = 7;

/**
 * Measures openings the camera sees during inspection, and fuses the views of each one by inverse
 * variance, as a tracker would. `sensed` is where the camera really is, `own` where the drone
 * believes it is: each view is relative to the camera, so it lands offset by the drone's own pose
 * error, which the fusion averages like any other error.
 */
export class OpeningSensor {
  private readonly rng: RNG;
  private n = 0;
  private readonly fused = new Map<number, { w: number[]; sum: number[]; bias: number[]; rangeM: number }>();

  constructor(seed: number) {
    this.rng = new RNG(seed >>> 0, 'opening-sensor');
  }

  measure(o: Opening, sensed: DronePose, own: DronePose): MeasuredOpening {
    const r = Math.hypot(o.center[0] - sensed.pos[0], o.center[1] - sensed.pos[1], o.center[2] - sensed.pos[2]);
    const e = OPENING_ERR;
    let f = this.fused.get(o.id);
    if (!f) {
      const g = this.rng.fork(`bias${o.id}`);
      const b = e.bias;
      f = {
        w: new Array<number>(N_ERR).fill(0),
        sum: new Array<number>(N_ERR).fill(0),
        bias: [b.plane, b.plane, b.depth, b.size, b.size, b.yaw, b.leaf].map((s) => s * g.gaussian()),
        rangeM: Infinity,
      };
      this.fused.set(o.id, f);
    }
    const g = this.rng.fork(`m${this.n++}`);
    const sig = [e.plane, e.plane, e.depth, e.size, e.size, e.yaw, e.leaf].map(([a, b]) => a + b * r);
    // This view's error on each quantity, relative to the camera, plus where the drone thinks the camera is.
    const { t, b, n } = basis(o);
    const dp = sub(own.pos, sensed.pos);
    const dyaw = own.yaw - sensed.yaw;
    const pose = [dot(dp, t), dot(dp, b), dot(dp, n), 0, 0, dyaw, 0];
    for (let i = 0; i < N_ERR; i++) {
      const w = 1 / (sig[i]! * sig[i]!);
      f.w[i]! += w;
      f.sum[i]! += w * (sig[i]! * g.gaussian() + pose[i]!);
    }
    f.rangeM = Math.min(f.rangeM, r);
    const err = f.sum.map((s, i) => s / f.w[i]! + f.bias[i]!);
    const center = add(add(add(o.center as V3, scale(t, err[0]!)), scale(b, err[1]!)), scale(n, err[2]!));
    const size = (v: number, d: number) => Math.max(0, v + d);
    const width = size(o.width, err[3]!);
    const height = size(o.height, err[4]!);
    return {
      ...o,
      center: center as [number, number, number],
      normal: rotZ(o.normal as V3, err[5]!) as [number, number, number],
      tangent: rotZ(o.tangent as V3, err[5]!) as [number, number, number],
      width,
      height,
      angle: o.angle > 0 ? Math.max(0, o.angle + err[6]!) : 0,
      clearWidth: o.clearWidth > 0 ? size(o.clearWidth, err[3]!) : 0,
      clearHeight: o.clearHeight > 0 ? height : 0,
      // What the camera can't tell, and the building's cell bookkeeping, are not part of a measurement.
      passable: true,
      decoy: false,
      apertureCells: [],
      carvedCells: [],
      rangeM: f.rangeM,
    };
  }
}

/** 1-sigma of the aperture tracker's position reading at range r from the aperture centre, m: a + b r. */
export const APERTURE_ERR = { a: 0.005, b: 0.006, velSigma: 0.02, tauSec: 0.3, yawSigma: 0.005 } as const;

/**
 * Relative position to an opening during a fly-through, as a visual tracker
 * locked on the aperture frame reports it: the drone's offset from the real
 * aperture, in the opening's own axes, with range-dependent error filtered
 * over a fraction of a second. The reading is expressed against the measured
 * opening the plan was made on, so the drone flies the plan relative to the
 * aperture it actually sees. `sense` is called once a tick after the airframe
 * moves; the controller reads the last reading.
 */
export class ApertureSensor {
  private readonly rng: RNG;
  private readonly truth: { c: V3; t: V3; b: V3; n: V3 };
  private readonly meas: { c: V3; t: V3; b: V3; n: V3 };
  private readonly err: [number, number, number] = [0, 0, 0];
  private ready = false;
  /** Last reading, world frame of the measured opening. */
  pos: V3 = [0, 0, 0];
  vel: V3 = [0, 0, 0];
  /** Heading relative to the measured opening, from the same reading. */
  yaw = 0;
  /** Range of the last reading, m. */
  range = 0;

  constructor(real: Opening, measured: Opening, seed: number) {
    this.rng = new RNG(seed >>> 0, 'aperture');
    const bt = basis(real);
    const bm = basis(measured);
    this.truth = { c: real.center as V3, ...bt };
    this.meas = { c: measured.center as V3, ...bm };
  }

  get hasReading(): boolean {
    return this.ready;
  }

  /** One tracker frame from the vehicle's true state. */
  sense(pos: V3, vel: V3, yaw: number, dt: number): void {
    const T = this.truth;
    const M = this.meas;
    const rel = sub(pos, T.c);
    const r = Math.hypot(rel[0], rel[1], rel[2]);
    const sigma = APERTURE_ERR.a + APERTURE_ERR.b * r;
    const k = Math.exp(-dt / APERTURE_ERR.tauSec);
    const w = Math.sqrt(1 - k * k);
    for (let i = 0; i < 3; i++) this.err[i] = this.ready ? this.err[i]! * k + sigma * w * this.rng.gaussian() : sigma * this.rng.gaussian();
    const u = dot(rel, T.t) + this.err[0];
    const v = dot(rel, T.b) + this.err[1];
    const s = dot(rel, T.n) + this.err[2];
    this.pos = add(add(add(M.c, scale(M.t, u)), scale(M.b, v)), scale(M.n, s));
    const vn = APERTURE_ERR.velSigma;
    const vu = dot(vel, T.t) + vn * this.rng.gaussian();
    const vv = dot(vel, T.b) + vn * this.rng.gaussian();
    const vs = dot(vel, T.n) + vn * this.rng.gaussian();
    this.vel = add(add(scale(M.t, vu), scale(M.b, vv)), scale(M.n, vs));
    const trueFace = Math.atan2(T.n[1], T.n[0]);
    const measFace = Math.atan2(M.n[1], M.n[0]);
    this.yaw = measFace + (yaw - trueFace) + APERTURE_ERR.yawSigma * this.rng.gaussian();
    this.range = r;
    this.ready = true;
  }

  /** 1-sigma of the position reading at the current range, m. */
  get sigma(): number {
    return APERTURE_ERR.a + APERTURE_ERR.b * this.range;
  }
}
