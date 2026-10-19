import type { OutdoorPhysics } from '../../../world/outdoor/physics';
import { sampleHeight } from '../../../world/outdoor/terrain';
import type { Mission, Point } from '../../types';
import type { MissionBuilding, MissionWorld, V3 } from '../contracts';
import { cameraFootprintRadius } from '../report';
import {
  type Airspace,
  airspaceFor,
  buildingBox,
  CRUISE_AGL,
  densify,
  fenceBounds,
  operable,
  routeXY,
} from './geometry';

/**
 * Lawnmower area coverage at cruise height, plus building discovery.
 * Lanes run east-west; spacing comes from the camera footprint so adjacent
 * swaths overlap. Connectors route around no-fly zones inside the fence.
 */

export interface CoverageOptions {
  /** Cruise height above ground. Default CRUISE_AGL. */
  agl?: number;
  /** Fraction of the swath shared by neighbouring lanes. Default 0.2. */
  overlap?: number;
  /** Lane inset from the fence and no-fly edges. Default 3.5 m. */
  margin?: number;
  /** Waypoint spacing along a lane. Default 5 m. */
  step?: number;
}

export interface CoverageLane {
  y: number;
  x0: number;
  x1: number;
}

export interface CoveragePlan {
  agl: number;
  /** Ground footprint radius at cruise. */
  footprintM: number;
  laneSpacing: number;
  lanes: CoverageLane[];
  /** Ordered cruise waypoints, world frame. Lanes and connectors. */
  waypoints: V3[];
  lengthM: number;
}

/** Ground radius the camera resolves at `agl`. Shared with the report's coverage model. */
export function cameraFootprint(agl: number): number {
  return cameraFootprintRadius(agl);
}

/** Cruise height for a mission: above the canopy, inside the altitude band. */
export function cruiseAglFor(mission: Mission): number {
  return Math.max(mission.altitude.minM + 0.5, Math.min(mission.altitude.maxM - 1, CRUISE_AGL));
}

function laneRuns(air: Airspace, y: number, minX: number, maxX: number, margin: number): CoverageLane[] {
  const runs: CoverageLane[] = [];
  let start: number | null = null;
  let last = minX;
  for (let x = minX; x <= maxX + 1e-6; x += 0.5) {
    const ok = operable(air, x, y, margin);
    if (ok && start === null) start = x;
    if (!ok && start !== null) {
      if (last - start >= 1) runs.push({ y, x0: start, x1: last });
      start = null;
    }
    if (ok) last = x;
  }
  if (start !== null && last - start >= 1) runs.push({ y, x0: start, x1: last });
  return runs;
}

export function planCoverage(mission: Mission, world: MissionWorld, options: CoverageOptions = {}): CoveragePlan {
  const air = airspaceFor(mission, world);
  const agl = options.agl ?? cruiseAglFor(mission);
  const overlap = options.overlap ?? 0.2;
  const margin = options.margin ?? 3.5;
  const step = options.step ?? 5;
  const footprintM = cameraFootprint(agl);
  const maxSpacing = Math.max(2, 2 * footprintM * (1 - overlap));
  const box = fenceBounds(air);
  const height = box.maxY - box.minY - 2 * margin;
  const count = Math.max(1, Math.ceil(height / maxSpacing));
  const laneSpacing = height / count;
  const lanes: CoverageLane[] = [];
  const route: Point[] = [];
  const waypoints: V3[] = [];
  const push = (pts: readonly Point[]) => {
    const dense = densify(air, pts, agl, step);
    for (const p of waypoints.length ? dense.slice(1) : dense) waypoints.push(p);
  };
  for (let k = 0; k < count; k++) {
    const y = box.minY + margin + (k + 0.5) * laneSpacing;
    const runs = laneRuns(air, y, box.minX, box.maxX, margin);
    const forward = k % 2 === 0;
    if (!forward) runs.reverse();
    for (const run of runs) {
      lanes.push(run);
      const a: Point = forward ? { x: run.x0, y } : { x: run.x1, y };
      const b: Point = forward ? { x: run.x1, y } : { x: run.x0, y };
      const prev = route[route.length - 1];
      if (prev) {
        const link = routeXY(air, prev, a, margin) ?? routeXY(air, prev, a, 1) ?? [prev, a];
        push(link);
      }
      push([a, b]);
      route.push(b);
    }
  }
  let lengthM = 0;
  for (let i = 1; i < waypoints.length; i++) {
    const a = waypoints[i - 1]!;
    const b = waypoints[i]!;
    lengthM += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  }
  return { agl, footprintM, laneSpacing, lanes, waypoints, lengthM };
}

/** Footprint sample points of a building at roof height. */
function roofSamples(building: MissionBuilding): V3[] {
  const box = buildingBox(building);
  const z = box.roofZ + 0.3;
  const xs = [box.minX + 0.3, box.cx, box.maxX - 0.3];
  const ys = [box.minY + 0.3, box.cy, box.maxY - 0.3];
  const out: V3[] = [];
  for (const x of xs) for (const y of ys) out.push([x, y, z]);
  return out;
}

/**
 * Buildings sensed from a pose: some part of the roof is inside the camera
 * footprint and a ray reaches it (or hits the shell itself) unblocked.
 */
export function senseBuildings(
  pos: V3,
  world: MissionWorld,
  physics: OutdoorPhysics,
  known: ReadonlySet<string>,
): MissionBuilding[] {
  const agl = pos[2] - sampleHeight(world.outdoor.terrain, pos[0], pos[1]);
  const radius = cameraFootprint(Math.max(agl, 4));
  const found: MissionBuilding[] = [];
  for (const building of world.buildings) {
    if (known.has(building.id)) continue;
    for (const p of roofSamples(building)) {
      const dx = p[0] - pos[0];
      const dy = p[1] - pos[1];
      if (dx * dx + dy * dy > radius * radius) continue;
      const dz = p[2] - pos[2];
      const d = Math.hypot(dx, dy, dz);
      const hit = physics.cast(pos[0], pos[1], pos[2], dx, dy, dz, d);
      if (!hit || hit.dist >= d - 0.6 || hit.tag === 'roof' || hit.tag === 'wall') {
        found.push(building);
        break;
      }
    }
  }
  return found;
}
