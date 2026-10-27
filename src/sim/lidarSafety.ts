import * as C from '../shared/constants';

// Module-local copies (imported bindings are getter calls under vite-node).
const LIDAR_H = C.LIDAR_H;
const LIDAR_MAX = C.LIDAR_MAX;
const MAX_SPEED_H = C.MAX_SPEED_H;
const DRONE_RADIUS = C.DRONE_RADIUS;

/**
 * Lidar safety layer shared by every pilot: the airframe stopping model, the
 * braking filter (a control barrier on stopping distance), the plain velocity
 * filter, and the guard that wraps the learned policy's action. It reads only
 * the drone's own lidar ring, height, yaw and measured velocity.
 */

/** Free air the safety filter keeps between the airframe edge and a lidar return, meters. */
export const SAFETY_MARGIN = 0.2;
/** Seconds of travel the filter allows toward a return: v·u ≤ (r − radius − margin) / horizon. */
export const SAFETY_HORIZON = 0.3;

/**
 * How an airframe stops: after a command to stop it keeps going for about
 * `reaction` seconds (latency, then tilting back), then decelerates at
 * `decel` m/s². Stopping distance from speed v is v·reaction + v²/(2·decel).
 */
export interface BrakingModel {
  reaction: number;
  decel: number;
}

/**
 * Stopping model of the simulator's quadrotor (sim/dynamics/quad.ts), fitted by
 * scripts/study/quad-sysid.ts to the 97th percentile of stopping distance over
 * domain-randomized airframes with two control ticks of latency, the most the
 * noise model draws. It is the airframe's own spec, not knowledge of the
 * building.
 */
export const QUAD_BRAKING: BrakingModel = { reaction: 0.21, decel: 5 };

/**
 * Air the braking filter keeps past the airframe edge, meters: for the planner,
 * and for the policy guard. Picked on the planner-tuning buildings
 * (9101-9108) as the smallest margins that bring quad collisions under about
 * one per episode; larger margins cost coverage in doorways.
 */
export const BRAKING_MARGIN = 0.25;
export const GUARD_BRAKING_MARGIN = 0.15;

/** Fastest approach speed from which `model` stops within `d` meters. */
export function safeSpeed(d: number, model: BrakingModel): number {
  if (!(d > 0)) return 0;
  const a = model.decel;
  const t = model.reaction;
  return a * (Math.sqrt(t * t + (2 * d) / a) - t);
}

/**
 * Model-aware lidar filter, a discrete control barrier on stopping distance.
 * For each horizontal ray with a return at range r, with d = r − radius −
 * margin, the barrier is h = d − S(v), where S is the airframe's stopping
 * distance at the speed v toward the return. The commanded speed toward the
 * return is capped at safeSpeed(d). When the measured speed toward it is
 * already past that cap (the drone is still carrying momentum from a turn),
 * the cap drops below it by the excess, so the controller brakes hard.
 *
 * A NaN range is a ray whose reading is missing (a failed sector): it is
 * treated as a return at the margin, so the drone never moves toward a sector
 * it cannot see. A range at LIDAR_MAX or more, or Infinity, is open air.
 */
export function brakingFilter(
  vx: number,
  vy: number,
  yaw: number,
  lidar: ArrayLike<number>,
  vel: readonly [number, number] | null | undefined,
  model: BrakingModel,
  margin = BRAKING_MARGIN,
): [number, number] {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const mx = vel ? vel[0] : vx;
  const my = vel ? vel[1] : vy;
  for (let pass = 0; pass < 3; pass++) {
    let changed = false;
    for (let i = 0; i < LIDAR_H; i++) {
      const raw = lidar[i]!;
      const r = Number.isNaN(raw) ? DRONE_RADIUS + margin : raw;
      if (r >= LIDAR_MAX - 1e-3) continue;
      const ux = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const uy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      const cap = safeSpeed(r - DRONE_RADIUS - margin, model);
      const measured = mx * ux + my * uy;
      const allowed = measured > cap ? cap - (measured - cap) : cap;
      const along = vx * ux + vy * uy;
      if (along > allowed) {
        const cut = along - allowed;
        vx -= cut * ux;
        vy -= cut * uy;
        changed = true;
      }
    }
    if (!changed) break;
  }
  return [vx, vy];
}

const RAY_COS = new Float64Array(LIDAR_H);
const RAY_SIN = new Float64Array(LIDAR_H);
for (let i = 0; i < LIDAR_H; i++) {
  RAY_COS[i] = Math.cos((i / LIDAR_H) * Math.PI * 2);
  RAY_SIN[i] = Math.sin((i / LIDAR_H) * Math.PI * 2);
}

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

/**
 * Lidar velocity filter. Projects a world-frame horizontal velocity so that,
 * along every horizontal ray with a return, the drone closes at most
 * (range − radius − margin) / horizon m/s. Rays without a return (misses,
 * glass or night dropouts) constrain nothing. Uses only the lidar.
 */
export function filterVelocity(
  vx: number,
  vy: number,
  yaw: number,
  lidar: ArrayLike<number>,
  margin = SAFETY_MARGIN,
  horizon = SAFETY_HORIZON,
): [number, number] {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  for (let pass = 0; pass < 3; pass++) {
    let changed = false;
    for (let i = 0; i < LIDAR_H; i++) {
      const r = lidar[i]!;
      if (r >= LIDAR_MAX - 1e-3) continue;
      const ux = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const uy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      const allowed = Math.max(0, (r - DRONE_RADIUS - margin) / horizon);
      const along = vx * ux + vy * uy;
      if (along > allowed) {
        const cut = along - allowed;
        vx -= cut * ux;
        vy -= cut * uy;
        changed = true;
      }
    }
    if (!changed) break;
  }
  return [vx, vy];
}

/** Height above the floor the policy guard holds the policy near, meters. */
export const GUARD_HEIGHT = 1.15;
/** Lidar filter settings for the policy guard: only imminent contacts are cut. */
export const GUARD_MARGIN = 0.05;
export const GUARD_HORIZON = 0.2;

/**
 * Horizontal distance to the nearest wall the lidar ring sees below which
 * the drone does not descend: a window sill or a ledge just under the ring is
 * invisible to it (a drone that came in through a window would sink onto the
 * sill otherwise).
 */
export const DESCENT_CLEARANCE = 0.6;

export function descentClear(lidar: ArrayLike<number>): boolean {
  for (let i = 0; i < lidar.length; i++) if (lidar[i]! < DESCENT_CLEARANCE) return false;
  return true;
}

/**
 * Sensor-only safety guard for the learned policy, applied to its body-frame
 * tanh action. It nudges altitude toward GUARD_HEIGHT above the current floor
 * (over the 0.9 m furniture tops, under the 1.4 m ones, so the lidar ring sees
 * those) and cuts horizontal velocity only when a lidar return is about to be
 * touched. Inputs are the drone's own height, yaw and horizontal lidar.
 */
export function guardAction(
  action: ArrayLike<number>,
  heightAboveFloor: number,
  yaw: number,
  lidar: ArrayLike<number>,
  out = new Float32Array(4),
  horizon = GUARD_HORIZON,
  braking?: { model: BrakingModel; vel: readonly [number, number] | null },
  speedH = MAX_SPEED_H,
): Float32Array {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const f = clamp(action[0] ?? 0, -1, 1) * speedH;
  const l = clamp(action[1] ?? 0, -1, 1) * speedH;
  const [vx, vy] = braking
    ? brakingFilter(cy * f - sy * l, sy * f + cy * l, yaw, lidar, braking.vel, braking.model, GUARD_BRAKING_MARGIN)
    : filterVelocity(cy * f - sy * l, sy * f + cy * l, yaw, lidar, GUARD_MARGIN, horizon);
  out[0] = (cy * vx + sy * vy) / speedH;
  out[1] = (-sy * vx + cy * vy) / speedH;
  out[2] = clamp((action[2] ?? 0) + (GUARD_HEIGHT - heightAboveFloor) * 2, -1, 1);
  if (out[2] < 0 && heightAboveFloor > 0.55 && !descentClear(lidar)) out[2] = 0;
  out[3] = clamp(action[3] ?? 0, -1, 1);
  return out;
}
