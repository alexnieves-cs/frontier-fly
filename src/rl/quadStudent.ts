/**
 * The quadrotor student: a feed-forward policy that flies the randomized quadrotor alone, from the
 * same sensor observation as every other pilot plus a short history of its own commands
 * (docs/QUAD_STUDENT.md).
 *
 * The quad cannot stop on the spot. Its velocity follows the command through a tilt the attitude
 * loop needs about a tenth of a second to reach, so a pilot has to know what it asked for a moment
 * ago as well as what the airframe is doing now: the observation carries the measured velocity,
 * and the extra slots below carry the last two commands. That is all the state the student has; it
 * has no memory and no map of its own beyond the observation's egocentric maps.
 *
 * Input layout: the OBS_DIM observation (src/sim/OBS_LAYOUT.md), then STUDENT_EXT values:
 *   +0..3  the command executed last step, [vx, vy, vz, yawRate] in action units
 *   +4..7  the command executed the step before
 *   +8..15 braking clearance in 8 body-frame sectors (forward, then every 45 degrees to the left):
 *          the nearest horizontal lidar return within 22.5 degrees of the sector, minus the distance
 *          the drone needs to stop at its measured velocity toward it, divided by 2 m and clipped to
 *          [-1, 1]. The same quantity the planner's braking filter thresholds; the policy only sees it.
 */
import { ACT_DIM, LIDAR_H, LIDAR_MAX, OBS, OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import { InferPolicy } from './infer';
import type { Checkpoint } from './checkpoint';

/** Body-frame sectors of the braking-clearance features. */
export const SECTORS = 8;
/** Extra input values after the observation. */
export const STUDENT_EXT = 2 * ACT_DIM + SECTORS;
/** Stopping model of the clearance features: reaction time (s) and deceleration (m/s²). */
const CLEAR_REACTION = 0.25;
const CLEAR_DECEL = 4;
/** Meters of clearance that read as 1 (the margin matters within a couple of meters). */
const CLEAR_SCALE = 2;

/** Write the SECTORS braking-clearance features of `obs` into out[at..]. */
export function clearanceFeatures(obs: Float32Array, out: Float32Array, at: number): void {
  const vf = obs[OBS.vel]! * 2;
  const vl = obs[OBS.vel + 1]! * 2;
  const per = LIDAR_H / SECTORS;
  for (let k = 0; k < SECTORS; k++) {
    const c = k * per;
    let r = Infinity;
    for (let d = -per / 2; d <= per / 2; d++) {
      const j = (((c + d) % LIDAR_H) + LIDAR_H) % LIDAR_H;
      r = Math.min(r, obs[OBS.lidar + j]! * LIDAR_MAX);
    }
    const th = (k * 2 * Math.PI) / SECTORS;
    const v = Math.max(0, vf * Math.cos(th) + vl * Math.sin(th));
    const stop = v * CLEAR_REACTION + (v * v) / (2 * CLEAR_DECEL);
    out[at + k] = Math.max(-1, Math.min(1, (r - stop) / CLEAR_SCALE));
  }
}
/** Input width of the quad student. */
export const STUDENT_DIM = OBS_DIM + STUDENT_EXT;
/** Default hidden layers of the quad student (wider than the kinematic policy's 256, 128). */
export const STUDENT_HIDDEN = [512, 256] as const;

/** Rolling command history and the student's input row. */
export class StudentInput {
  readonly row = new Float32Array(STUDENT_DIM);
  private readonly prev = new Float32Array(ACT_DIM);
  private readonly prev2 = new Float32Array(ACT_DIM);

  reset(): void {
    this.prev.fill(0);
    this.prev2.fill(0);
  }

  /** The input row for this observation (history as of the last executed command). */
  build(obs: Float32Array): Float32Array {
    this.row.set(obs.subarray(0, OBS_DIM), 0);
    this.row.set(this.prev, OBS_DIM);
    this.row.set(this.prev2, OBS_DIM + ACT_DIM);
    clearanceFeatures(obs, this.row, OBS_DIM + 2 * ACT_DIM);
    return this.row;
  }

  /** Record the command the airframe was actually given this step. */
  executed(action: ArrayLike<number>): void {
    this.prev2.set(this.prev);
    for (let i = 0; i < ACT_DIM; i++) this.prev[i] = Math.max(-1, Math.min(1, action[i] ?? 0));
  }
}

/** A quad student as a coverage-env pilot: deterministic actions, its own command history. */
export class QuadStudentPilot {
  readonly policy: InferPolicy;
  private readonly input = new StudentInput();
  private readonly rng = new RNG(1, 'quad-student');

  /**
   * @param gain output scale on the horizontal command. Imitation by squared error shrinks a
   *   student's speed toward the average of the teacher's (it hedges between the teacher's
   *   options); a fixed gain, picked on practice flights and kept during RL, restores the pace.
   */
  constructor(source: Checkpoint | InferPolicy, readonly gain = 1) {
    if (source instanceof InferPolicy) this.policy = source;
    else {
      if (source.obsDim !== STUDENT_DIM) throw new Error(`quad student expects obsDim ${STUDENT_DIM}, checkpoint has ${source.obsDim}`);
      this.policy = new InferPolicy(source.obsDim, source.actDim, source.hidden);
      this.policy.setWeights(paddedWeights(source));
    }
  }

  reset(): void {
    this.input.reset();
  }

  /** The next command; `filter` (a safety shield, say) may change it in place before it is recorded as executed. */
  act(obs: Float32Array, filter?: (a: Float32Array) => void): Float32Array {
    const a = this.policy.actWith(this.input.build(obs), 1, this.rng, true).action;
    if (this.gain !== 1) {
      a[0] = Math.max(-1, Math.min(1, a[0]! * this.gain));
      a[1] = Math.max(-1, Math.min(1, a[1]! * this.gain));
    }
    filter?.(a);
    this.input.executed(a);
    return a;
  }
}

/** Checkpoint weights as InferPolicy buffers (zeros for a missing value head). */
export function paddedWeights(c: Checkpoint): ArrayBuffer[] {
  const out = c.weights.map((w) => Float32Array.from(w).buffer as ArrayBuffer);
  const h0 = c.hidden[0]!;
  const h1 = c.hidden[1]!;
  const sizes = [c.obsDim * h0, h0, h0 * h1, h1, h1, 1];
  for (let i = out.length; i < 13; i++) out.push(new Float32Array(sizes[i - 7]!).buffer);
  return out;
}
