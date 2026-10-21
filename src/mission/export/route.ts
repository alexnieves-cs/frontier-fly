import type { MissionWorld, V3 } from '../autonomy/contracts';
import { cruiseAglFor, planCoverage, type CoveragePlan } from '../autonomy/planner/coverage';
import { sampleHeight } from '../../world/outdoor/terrain';
import { isOperable, planPath } from '../path';
import { distToSegment } from '../polygon';
import type { Mission, Point } from '../types';

/**
 * The flight an export describes, before any file format: ordered legs of
 * world-frame points, each with the ground height under it so the writer can
 * pick a relative-to-home or above-terrain altitude reference.
 *
 * This is the interface between the planners and the exporters. A planner
 * hands over ordered V3 waypoints (x east, y north, z absolute, meters) and
 * `routeFromWaypoints` labels and simplifies them. Only the pre-planned part of
 * a mission can be exported: the adaptive confirmation looks, the building
 * entry and the GPS-denied indoor search are decided in flight.
 */

export type LegKind = 'transit' | 'sweep' | 'search';

export interface RoutePoint {
  x: number;
  y: number;
  /** Absolute height in the sim frame, meters. */
  z: number;
  /** Ground height under the point in the sim frame, meters. */
  groundZ: number;
}

export interface RouteLeg {
  kind: LegKind;
  points: RoutePoint[];
}

export type RouteSource = 'coverage' | 'preview';

export interface MissionRoute {
  source: RouteSource;
  /** Takeoff and landing pad. */
  home: RoutePoint;
  /** Cruise height above ground the planner used. */
  cruiseAglM: number;
  /** Horizontal cruise speed the sim flies, m/s. */
  speedMps: number;
  legs: RouteLeg[];
}

/** The mission vehicle's outdoor speed cap (OUTDOOR_LIMITS.speedH). */
export const SIM_CRUISE_SPEED_MPS = 5.5;
/** The editor preview sweeps at this speed (see the time-budget warning in validate.ts). */
export const PREVIEW_SPEED_MPS = 3;

export interface SimplifyOptions {
  /** Points closer than this to the line through their neighbours are dropped. Default 0.5 m. */
  toleranceM?: number;
}

function offLine(p: RoutePoint, a: RoutePoint, b: RoutePoint): number {
  const ab = [b.x - a.x, b.y - a.y, b.z - a.z] as const;
  const ap = [p.x - a.x, p.y - a.y, p.z - a.z] as const;
  const len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
  if (len2 < 1e-12) return Math.hypot(ap[0], ap[1], ap[2]);
  const t = Math.max(0, Math.min(1, (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2));
  return Math.hypot(ap[0] - ab[0] * t, ap[1] - ab[1] * t, ap[2] - ab[2] * t);
}

/** Douglas-Peucker in 3D. Keeps both ends; every dropped point is within `tol` of the kept polyline. */
export function simplify(points: readonly RoutePoint[], tol: number): RoutePoint[] {
  if (points.length <= 2 || tol <= 0) return points.map((p) => ({ ...p }));
  const keep = new Uint8Array(points.length);
  keep[0] = 1;
  keep[points.length - 1] = 1;
  const stack: [number, number][] = [[0, points.length - 1]];
  while (stack.length) {
    const [i, j] = stack.pop()!;
    let worst = -1;
    let at = -1;
    for (let k = i + 1; k < j; k++) {
      const d = offLine(points[k]!, points[i]!, points[j]!);
      if (d > worst) {
        worst = d;
        at = k;
      }
    }
    if (at >= 0 && worst > tol) {
      keep[at] = 1;
      stack.push([i, at], [at, j]);
    }
  }
  return points.filter((_, k) => keep[k]).map((p) => ({ ...p }));
}

export function routeLength(route: MissionRoute, kind?: LegKind): number {
  let total = 0;
  let prev: RoutePoint | null = null;
  for (const leg of route.legs) {
    for (const p of leg.points) {
      if (prev && (!kind || leg.kind === kind)) total += Math.hypot(p.x - prev.x, p.y - prev.y, p.z - prev.z);
      prev = p;
    }
  }
  return total;
}

export function routePointCount(route: MissionRoute): number {
  return route.legs.reduce((sum, leg) => sum + leg.points.length, 0);
}

/**
 * Split ordered waypoints into legs. A segment is `sweep` when both ends sit
 * on the same lane (same y, inside the lane's x range); everything else is a
 * `transit` connector. Without lanes, every point gets `fallback`.
 */
export function labelWaypoints(
  waypoints: readonly V3[],
  lanes: readonly { y: number; x0: number; x1: number }[],
  fallback: LegKind,
): { kind: LegKind; points: V3[] }[] {
  const onLane = (a: V3, b: V3): boolean =>
    lanes.some(
      (lane) =>
        Math.abs(a[1] - lane.y) < 1e-6 &&
        Math.abs(b[1] - lane.y) < 1e-6 &&
        Math.min(a[0], b[0]) >= lane.x0 - 1e-6 &&
        Math.max(a[0], b[0]) <= lane.x1 + 1e-6,
    );
  const legs: { kind: LegKind; points: V3[] }[] = [];
  for (let i = 0; i < waypoints.length; i++) {
    const p = waypoints[i]!;
    if (i === 0) {
      legs.push({ kind: lanes.length ? 'transit' : fallback, points: [p] });
      continue;
    }
    const kind: LegKind = lanes.length ? (onLane(waypoints[i - 1]!, p) ? 'sweep' : 'transit') : fallback;
    const last = legs[legs.length - 1]!;
    if (last.kind === kind) {
      last.points.push(p);
    } else {
      // The shared corner belongs to both legs; the writer emits it once.
      legs.push({ kind, points: [last.points[last.points.length - 1]!, p] });
    }
  }
  if (legs[0] && legs[0].points.length === 1 && legs.length > 1) legs.shift();
  return legs;
}

export interface WaypointRouteOptions extends SimplifyOptions {
  source: RouteSource;
  home: RoutePoint;
  cruiseAglM: number;
  speedMps: number;
  /** Ground height lookup in the sim frame. */
  ground: (x: number, y: number) => number;
  /** Lanes, to tell sweep segments from connectors. */
  lanes?: readonly { y: number; x0: number; x1: number }[];
  /** Leg kind when no lanes are given. Default 'search'. */
  leg?: LegKind;
}

/** Ordered world-frame waypoints to a labelled, simplified route. Any planner can hand its plan over through this. */
export function routeFromWaypoints(waypoints: readonly V3[], options: WaypointRouteOptions): MissionRoute {
  const tol = options.toleranceM ?? 0.5;
  const labelled = labelWaypoints(waypoints, options.lanes ?? [], options.leg ?? 'search');
  const legs: RouteLeg[] = labelled.map((leg) => ({
    kind: leg.kind,
    points: simplify(
      leg.points.map(([x, y, z]) => ({ x, y, z, groundZ: options.ground(x, y) })),
      tol,
    ),
  }));
  return {
    source: options.source,
    home: { ...options.home },
    cruiseAglM: options.cruiseAglM,
    speedMps: options.speedMps,
    legs: legs.filter((leg) => leg.points.length > 0),
  };
}

/** The autonomy stack's own lawnmower plan over a generated world: terrain-following, routed around no-fly zones. */
export function routeFromCoverage(
  mission: Mission,
  world: MissionWorld,
  options: SimplifyOptions & { plan?: CoveragePlan } = {},
): MissionRoute {
  const agl = cruiseAglFor(mission);
  const plan = options.plan ?? planCoverage(mission, world, { agl });
  const terrain = world.outdoor.terrain;
  const ground = (x: number, y: number): number => sampleHeight(terrain, x, y);
  // Home is the mission's start pose, which is what the editor shows and what a .plan import anchors on.
  const hx = mission.startPose.x;
  const hy = mission.startPose.y;
  return routeFromWaypoints(plan.waypoints, {
    source: 'coverage',
    home: { x: hx, y: hy, z: ground(hx, hy), groundZ: ground(hx, hy) },
    cruiseAglM: plan.agl,
    speedMps: SIM_CRUISE_SPEED_MPS,
    ground,
    lanes: plan.lanes,
    toleranceM: options.toleranceM,
  });
}

/**
 * How far an exported waypoint stays from a fence edge, meters. The editor's
 * preview runs its lanes edge to edge, but an autopilot checks every waypoint
 * against the uploaded fence before it accepts a mission (PX4 refuses one on
 * the line), and a multicopter rounds a corner a meter or two wide.
 */
export const FENCE_CLEARANCE_M = 2;

function fenceRings(mission: Mission): (readonly Point[])[] {
  return [mission.geofence.points, ...mission.noFlyZones.map((z) => z.polygon.points)];
}

function fenceDistance(p: Point, mission: Mission): number {
  let best = Infinity;
  for (const ring of fenceRings(mission)) {
    for (let i = 0; i < ring.length; i++) best = Math.min(best, distToSegment(p, ring[i]!, ring[(i + 1) % ring.length]!));
  }
  return best;
}

/** Closest point on any fence edge, and that edge's unit normal. */
function nearestFenceEdge(p: Point, mission: Mission): { q: Point; n: Point; d: number } {
  let best = { q: p, n: { x: 0, y: 1 }, d: Infinity };
  for (const ring of fenceRings(mission)) {
    for (let i = 0; i < ring.length; i++) {
      const a = ring[i]!;
      const b = ring[(i + 1) % ring.length]!;
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const ll = dx * dx + dy * dy;
      if (ll === 0) continue;
      const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / ll));
      const q = { x: a.x + t * dx, y: a.y + t * dy };
      const d = Math.hypot(p.x - q.x, p.y - q.y);
      if (d < best.d) best = { q, n: { x: -dy / Math.sqrt(ll), y: dx / Math.sqrt(ll) }, d };
    }
  }
  return best;
}

/**
 * Moves any waypoint closer than `clearance` to a fence edge straight away
 * from that edge, onto the operable side (the editor's detours run along
 * no-fly zone edges). A few passes settle points near two edges, such as a
 * zone's corner.
 */
export function clearFences(waypoints: V3[], mission: Mission, clearance: number): V3[] {
  return waypoints.map((w) => {
    let p: Point = { x: w[0], y: w[1] };
    for (let pass = 0; pass < 8; pass++) {
      const { q, n, d } = nearestFenceEdge(p, mission);
      if (d >= clearance - 1e-9) break;
      let away = d > 1e-6 ? { x: (p.x - q.x) / d, y: (p.y - q.y) / d } : n;
      if (!isOperable({ x: p.x + away.x * 0.05, y: p.y + away.y * 0.05 }, mission)) away = { x: -away.x, y: -away.y };
      // A centimeter extra survives the rounding below.
      p = { x: q.x + away.x * (clearance + 0.01), y: q.y + away.y * (clearance + 0.01) };
    }
    return [Math.round(p.x * 100) / 100, Math.round(p.y * 100) / 100, w[2]];
  });
}

/**
 * Pulls the ends of each east-west lane in along the lane until they clear
 * every fence edge by `clearance`, never past the lane's middle. The hops
 * between lanes move with the ends they join.
 */
export function insetLaneEnds(waypoints: V3[], mission: Mission, clearance: number): V3[] {
  const out = waypoints.map((w) => [...w] as V3);
  for (let i = 1; i < out.length; i++) {
    const a = out[i - 1]!;
    const b = out[i]!;
    if (Math.abs(a[1] - b[1]) > 1e-6) continue;
    const len = Math.abs(b[0] - a[0]);
    if (len < 1e-6) continue;
    const dir = Math.sign(b[0] - a[0]);
    for (const [end, sign] of [
      [a, 1],
      [b, -1],
    ] as const) {
      const x0 = end[0];
      for (let moved = 0; moved <= len / 2; moved += 0.1) {
        const x = x0 + sign * dir * moved;
        if (fenceDistance({ x, y: end[1] }, mission) >= clearance) {
          end[0] = Math.round(x * 100) / 100;
          break;
        }
      }
    }
  }
  return out;
}

/** The editor's flat-site sweep preview (planPath) at cruise height. Used for the mock practice site, which has no terrain. */
export function routeFromPreview(mission: Mission, options: SimplifyOptions = {}): MissionRoute {
  const agl = cruiseAglFor(mission);
  const path = planPath(mission);
  // planPath starts at the start pose; the takeoff item covers that point.
  const waypoints: V3[] = clearFences(
    insetLaneEnds(
      path.slice(1).map((p) => [p.x, p.y, agl]),
      mission,
      FENCE_CLEARANCE_M,
    ),
    mission,
    FENCE_CLEARANCE_M,
  );
  const home: RoutePoint = { x: mission.startPose.x, y: mission.startPose.y, z: 0, groundZ: 0 };
  // Preview lanes are the east-west runs; the hops between them are connectors.
  const lanes: { y: number; x0: number; x1: number }[] = [];
  for (let i = 1; i < waypoints.length; i++) {
    const a = waypoints[i - 1]!;
    const b = waypoints[i]!;
    if (Math.abs(a[1] - b[1]) < 1e-6 && Math.abs(a[0] - b[0]) >= 4) {
      lanes.push({ y: a[1], x0: Math.min(a[0], b[0]), x1: Math.max(a[0], b[0]) });
    }
  }
  return routeFromWaypoints(waypoints, {
    source: 'preview',
    home,
    cruiseAglM: agl,
    speedMps: PREVIEW_SPEED_MPS,
    ground: () => 0,
    lanes,
    toleranceM: options.toleranceM,
  });
}
