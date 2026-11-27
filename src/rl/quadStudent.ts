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
 */
import { ACT_DIM, OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import { InferPolicy } from './infer';
import type { Checkpoint } from './checkpoint';

/** Extra input values after the observation. */
export const STUDENT_EXT = 2 * ACT_DIM;
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

  constructor(source: Checkpoint | InferPolicy) {
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

  act(obs: Float32Array): Float32Array {
    const a = this.policy.actWith(this.input.build(obs), 1, this.rng, true).action;
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
