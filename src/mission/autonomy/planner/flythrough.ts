import type { V3 } from '../contracts';
import { add, clamp, dot, len, scale, sub } from './geometry';
import { ALIGN_OUT, PASS_IN, type PassageAxis } from './entry';

/**
 * Fly-through-opening controller. Aligns on the opening axis outside the
 * aperture, then crosses slowly with a PD loop holding the centerline on
 * both the width and height axes. If the error ever exceeds the clearance
 * margin near the aperture it backs out to the alignment point and reports
 * an abort. The same controller exits by running the axis in reverse.
 */

export type FlyThroughState = 'align' | 'cross' | 'backout' | 'done' | 'aborted';

export interface FlyThroughCommand {
  vel: V3;
  /** Heading to hold. Null keeps the current heading. */
  yaw: number | null;
}

export interface FlyThroughOptions {
  /** Crossing speed along the axis, m/s. */
  crossSpeed?: number;
  /** Alignment approach speed cap, m/s. */
  alignSpeed?: number;
  kp?: number;
  kd?: number;
}

export interface FlyThroughStep {
  cmd: FlyThroughCommand;
  state: FlyThroughState;
  /** Signed errors from the centerline, meters. */
  errU: number;
  errV: number;
  /** Distance past the aperture plane along the travel direction. */
  along: number;
}

export class FlyThrough {
  state: FlyThroughState = 'align';
  readonly start: V3;
  readonly end: V3;
  readonly dir: V3;
  /** Largest absolute errors seen while crossing. */
  maxErrU = 0;
  maxErrV = 0;
  abortReason = '';
  private readonly crossSpeed: number;
  private readonly alignSpeed: number;
  private readonly kp: number;
  private readonly kd: number;
  private readonly yaw: number | null;

  constructor(
    readonly axis: PassageAxis,
    readonly direction: 'in' | 'out',
    options: FlyThroughOptions = {},
  ) {
    const n = axis.normal;
    this.dir = direction === 'in' ? scale(n, -1) : n;
    this.start = direction === 'in' ? add(axis.center, scale(n, ALIGN_OUT)) : add(axis.center, scale(n, -PASS_IN));
    this.end = direction === 'in' ? add(axis.center, scale(n, -PASS_IN)) : add(axis.center, scale(n, ALIGN_OUT));
    this.crossSpeed = options.crossSpeed ?? 0.35;
    this.alignSpeed = options.alignSpeed ?? 0.7;
    this.kp = options.kp ?? 2.4;
    this.kd = options.kd ?? 0.25;
    this.yaw = axis.vertical ? null : Math.atan2(this.dir[1], this.dir[0]);
  }

  private seek(pos: V3, target: V3, cap: number): V3 {
    const d = sub(target, pos);
    const l = len(d);
    if (l < 1e-6) return [0, 0, 0];
    const speed = Math.min(cap, l * 1.6);
    return scale(d, speed / l);
  }

  step(pos: V3, vel: V3, collided = false): FlyThroughStep {
    const rel = sub(pos, this.axis.center);
    const errU = dot(rel, this.axis.tangent);
    const errV = dot(rel, this.axis.bitangent);
    const along = dot(rel, this.dir);
    const endAlong = dot(sub(this.end, this.axis.center), this.dir);
    let vel3: V3 = [0, 0, 0];
    if (this.state === 'align') {
      vel3 = this.seek(pos, this.start, this.alignSpeed);
      if (len(sub(pos, this.start)) < 0.06 && len(vel) < 0.12) this.state = 'cross';
    }
    if (this.state === 'cross') {
      const du = dot(vel, this.axis.tangent);
      const dv = dot(vel, this.axis.bitangent);
      const vu = clamp(-this.kp * errU - this.kd * du, -0.3, 0.3);
      const vv = clamp(-this.kp * errV - this.kd * dv, -0.3, 0.3);
      const remaining = endAlong - along;
      const speed = clamp(remaining * 1.5, 0.08, this.crossSpeed);
      vel3 = add(add(scale(this.dir, speed), scale(this.axis.tangent, vu)), scale(this.axis.bitangent, vv));
      const throat = along > -1.0 && along < 1.0;
      if (throat) {
        this.maxErrU = Math.max(this.maxErrU, Math.abs(errU));
        this.maxErrV = Math.max(this.maxErrV, Math.abs(errV));
      }
      if (throat && (Math.abs(errU) > this.axis.marginU || Math.abs(errV) > this.axis.marginV || collided)) {
        this.state = 'backout';
        this.abortReason = collided
          ? 'contact in the aperture'
          : `centerline error ${Math.abs(errU).toFixed(2)}/${Math.abs(errV).toFixed(2)} m over margin ${this.axis.marginU.toFixed(2)}/${this.axis.marginV.toFixed(2)} m`;
      } else if (remaining <= 0.04) {
        this.state = 'done';
        vel3 = [0, 0, 0];
      }
    }
    if (this.state === 'backout') {
      // Retreat along the axis first, then settle at the alignment point.
      const startAlong = dot(sub(this.start, this.axis.center), this.dir);
      if (along > startAlong + 0.05) {
        const lateral = add(scale(this.axis.tangent, -errU), scale(this.axis.bitangent, -errV));
        vel3 = add(scale(this.dir, -0.3), scale(lateral, 1.5));
      } else {
        vel3 = this.seek(pos, this.start, 0.3);
      }
      if (len(sub(pos, this.start)) < 0.12) {
        this.state = 'aborted';
        vel3 = [0, 0, 0];
      }
    }
    return { cmd: { vel: vel3, yaw: this.yaw }, state: this.state, errU, errV, along };
  }
}
