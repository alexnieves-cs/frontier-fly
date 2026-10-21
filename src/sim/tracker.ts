/**
 * Quad-aware trajectory tracking for the sensor planner (opt-in via
 * SensorPlannerOptions.tracker).
 *
 * The planner's pure pursuit asks for full cruise speed toward a point at most
 * 2.5 m ahead, and the command swings by 90° or more whenever the pursuit
 * point rounds a corner or the goal changes. A kinematic airframe follows
 * that almost exactly. A quadrotor has to tilt to change velocity (about
 * 4 m/s² at the angle controller's 0.4 rad limit, after a tenth of a second of
 * attitude lag), so a step in the command overshoots the corner and the
 * velocity loop rings. The tracker shapes the command into one the airframe
 * can fly:
 *
 * 1. A speed profile from the path ahead. The path is string-pulled into
 *    straight legs (line of sight on the planner's own map). At each corner
 *    of angle θ the drone may arrive at the speed from which it can cancel
 *    its sideways velocity within `corner` meters of overshoot:
 *    v·sin θ ≤ √(2·accel·corner) (all of it, for θ ≥ 90°). Before the corner
 *    the cap rises with the airframe's braking distance (reaction time, then
 *    constant deceleration), as in the planner's braking filter. The same
 *    braking curve slows the drone for the end of its sweep lane (the first
 *    flown or blocked voxel straight ahead), where it U-turns into the next
 *    lane one voxel over: v = √(accel · 0.25 m).
 * 2. An acceleration limit on the command itself: the commanded velocity
 *    moves toward the desired one by at most `accel`·dt per step, so the
 *    controller sees a ramp it can follow instead of a step. The lidar safety
 *    filter runs after the tracker and can still cut speed at once.
 * 3. A yaw rate cap while facing the direction of travel. The env's command
 *    is body-frame velocity, so a drone yawing at full rate through a U-turn
 *    asks for a world-frame velocity that keeps rotating under it; with the
 *    quad's lag that broke the sweep lanes into loops.
 *
 * The tracker uses only what the planner already has: its path on its own
 * map, the drone's pose and its measured velocity. It never reads the floor plan.
 */
export interface TrackerOptions {
  /** Horizontal acceleration the command may ask for, m/s². */
  accel?: number;
  /** Overshoot allowed at a corner, meters (about the half-width of a doorway). */
  corner?: number;
  /** Seconds before deceleration starts (latency plus tilting back). */
  reaction?: number;
  /** Lowest speed the profile asks for at a sharp corner, m/s. */
  minSpeed?: number;
  /** Meters of path the profile looks ahead. */
  lookahead?: number;
  /** Speed cap at the end of the path (the frontier goal), as a share of cruise. 1 = no cap. */
  endShare?: number;
  /** Yaw rate cap while facing the direction of travel, rad/s. */
  yawRate?: number;
  /** Brake for the end of the sweep lane ahead (the first flown or blocked voxel straight ahead). */
  lane?: boolean;
  /** Radius of the U-turn between sweep lanes, meters (half the 0.5 m voxel). */
  turnRadius?: number;
}

export interface ResolvedTracker {
  accel: number;
  corner: number;
  reaction: number;
  minSpeed: number;
  lookahead: number;
  endShare: number;
  yawRate: number;
  lane: boolean;
  turnRadius: number;
}

/** Defaults picked on the planner-tuning buildings 9101-9108 only (scripts/study/track-diag.ts). */
export const TRACKER_DEFAULTS: ResolvedTracker = {
  accel: 4,
  corner: 0.4,
  reaction: 0.2,
  minSpeed: 0.5,
  lookahead: 6,
  endShare: 1,
  yawRate: 0.3,
  lane: true,
  turnRadius: 0.25,
};

export function resolveTracker(o: TrackerOptions): ResolvedTracker {
  return { ...TRACKER_DEFAULTS, ...o };
}

/** Fastest speed at which the airframe can take a corner of `theta` radians (0 = straight on). */
export function cornerSpeed(theta: number, t: ResolvedTracker, cruise: number): number {
  const a = Math.abs(theta);
  const s = a >= Math.PI / 2 ? 1 : Math.sin(a);
  const v = s > 1e-6 ? Math.sqrt(2 * t.accel * t.corner) / s : Infinity;
  return Math.max(t.minSpeed, Math.min(cruise, v));
}

/**
 * Speed for the U-turn at the end of a sweep lane: the next lane is one voxel
 * over, so the turn radius is half a voxel and v = √(accel · VOXEL / 2).
 */
export function uTurnSpeed(t: ResolvedTracker, cruise: number): number {
  return Math.max(t.minSpeed, Math.min(cruise, Math.sqrt(t.accel * t.turnRadius)));
}

/** Fastest speed now from which the airframe slows to `vEnd` within `d` meters. */
export function approachSpeed(vEnd: number, d: number, t: ResolvedTracker): number {
  if (!(d > 0)) return vEnd;
  // v·reaction + (v² − vEnd²) / (2a) = d, solved for v ≥ vEnd.
  const a = t.accel;
  const r = t.reaction;
  const v = -a * r + Math.sqrt(a * a * r * r + vEnd * vEnd + 2 * a * d);
  return Math.max(vEnd, v);
}

/**
 * Speed cap from a string-pulled path: `pts` are the corner points after the
 * drone's position (x0, y0), in order. The last point is the path's end when
 * `endIsGoal`, otherwise where the lookahead cut it.
 */
export function profileSpeed(
  x0: number,
  y0: number,
  pts: readonly (readonly [number, number])[],
  cruise: number,
  t: ResolvedTracker,
  endIsGoal = true,
): number {
  let cap = cruise;
  let px = x0;
  let py = y0;
  let along = 0;
  for (let i = 0; i < pts.length; i++) {
    const [qx, qy] = pts[i]!;
    const lx = qx - px;
    const ly = qy - py;
    const leg = Math.hypot(lx, ly);
    along += leg;
    if (along > t.lookahead) break;
    let vc: number;
    if (i + 1 < pts.length) {
      const [rx, ry] = pts[i + 1]!;
      const nx = rx - qx;
      const ny = ry - qy;
      const nl = Math.hypot(nx, ny);
      if (leg < 1e-6 || nl < 1e-6) vc = cruise;
      else {
        const c = (lx * nx + ly * ny) / (leg * nl);
        vc = cornerSpeed(Math.acos(Math.max(-1, Math.min(1, c))), t, cruise);
      }
    } else vc = endIsGoal ? Math.max(t.minSpeed, cruise * t.endShare) : cruise;
    const v = approachSpeed(vc, along, t);
    if (v < cap) cap = v;
    px = qx;
    py = qy;
  }
  return cap;
}

/** Acceleration limit on the commanded horizontal velocity. */
export class CommandShaper {
  private vx = 0;
  private vy = 0;
  private primed = false;

  reset(): void {
    this.vx = 0;
    this.vy = 0;
    this.primed = false;
  }

  /**
   * Move the held command toward (vx, vy) by at most accel·dt. The first call
   * after a reset starts from the measured velocity.
   */
  shape(vx: number, vy: number, accel: number, dt: number, measured?: readonly [number, number]): [number, number] {
    if (!this.primed) {
      this.vx = measured ? measured[0] : 0;
      this.vy = measured ? measured[1] : 0;
      this.primed = true;
    }
    const dx = vx - this.vx;
    const dy = vy - this.vy;
    const dl = Math.hypot(dx, dy);
    const step = accel * dt;
    if (dl <= step) {
      this.vx = vx;
      this.vy = vy;
    } else {
      this.vx += (dx / dl) * step;
      this.vy += (dy / dl) * step;
    }
    return [this.vx, this.vy];
  }

  /** After the safety filter cut the command, hold the cut value so the ramp starts from it. */
  hold(vx: number, vy: number): void {
    this.vx = vx;
    this.vy = vy;
  }
}
