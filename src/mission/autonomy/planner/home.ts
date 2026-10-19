import { DRONE_RADIUS } from '../../../shared/constants';
import type { OutdoorPhysics } from '../../../world/outdoor/physics';
import { sampleHeight } from '../../../world/outdoor/terrain';
import type { V3 } from '../contracts';
import { type Airspace, columnClear, CRUISE_AGL, cruisePath, cruiseZ, dist2d } from './geometry';

/**
 * Return to home. Climb to cruise through an open column (or from a known
 * open standoff), fly the operable cruise route home, then descend over the
 * pad. Landing slows near the ground and settles just above it.
 */

/** Height above the pad where the final slow descent starts, meters. */
export const FLARE_AGL = 1.2;
/** Touchdown height of the airframe center above ground, meters. */
export const TOUCHDOWN_AGL = DRONE_RADIUS + 0.06;
/** Landed within this horizontal distance of home counts as home. */
export const HOME_RADIUS = 1.5;

export interface ReturnPlan {
  /** Climb, cruise, and descent waypoints, ending over the pad at flare height. */
  waypoints: V3[];
  lengthM: number;
}

/** Waypoints to climb out from `from`; via `refuge` (an open column) when the column overhead is blocked. */
export function climbOut(physics: OutdoorPhysics, air: Airspace, from: V3, refuge: V3 | null, agl = CRUISE_AGL): V3[] {
  const top = cruiseZ(air, from[0], from[1], agl);
  if (from[2] >= top - 0.5) return [];
  if (columnClear(physics, from[0], from[1], from[2], top, 0.45)) return [[from[0], from[1], top]];
  if (refuge) return [refuge, [refuge[0], refuge[1], cruiseZ(air, refuge[0], refuge[1], agl)]];
  return [[from[0], from[1], top]];
}

export function planReturn(
  physics: OutdoorPhysics,
  air: Airspace,
  from: V3,
  home: V3,
  refuge: V3 | null,
  agl = CRUISE_AGL,
): ReturnPlan {
  const waypoints = climbOut(physics, air, from, refuge, agl);
  const start = waypoints[waypoints.length - 1] ?? from;
  const route = cruisePath(air, start, { x: home[0], y: home[1] }, agl);
  waypoints.push(...route);
  const ground = sampleHeight(air.terrain, home[0], home[1]);
  const top = waypoints[waypoints.length - 1] ?? start;
  const flare = ground + FLARE_AGL;
  const steps = Math.max(1, Math.ceil((top[2] - flare) / 3));
  for (let k = 1; k <= steps; k++) waypoints.push([home[0], home[1], top[2] + ((flare - top[2]) * k) / steps]);
  let lengthM = 0;
  let prev = from;
  for (const p of waypoints) {
    lengthM += Math.hypot(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]);
    prev = p;
  }
  return { waypoints, lengthM };
}

/** Landing setpoint: vertical speed by height above ground, with xy hold. */
export function landingCommand(pos: V3, target: V3, groundZ: number): { vel: V3; touched: boolean } {
  const agl = pos[2] - groundZ;
  const touched = agl <= TOUCHDOWN_AGL + 0.03;
  const vz = touched ? 0 : -Math.max(0.15, Math.min(0.8, (agl - TOUCHDOWN_AGL) * 0.8));
  const kx = 1.2;
  return { vel: [(target[0] - pos[0]) * kx, (target[1] - pos[1]) * kx, vz], touched };
}

export function landedAtHome(pos: V3, home: V3): boolean {
  return dist2d(pos, home) <= HOME_RADIUS;
}
