import { DRONE_RADIUS } from '../../../shared/constants';
import type { OutdoorPhysics } from '../../../world/outdoor/physics';
import { sampleHeight } from '../../../world/outdoor/terrain';
import type { Terrain } from '../../../world/outdoor/types';
import type { Mission, Point } from '../../types';
import type { MissionBuilding, MissionWorld, V3 } from '../contracts';

/** Cruise height above ground. Canopy tops reach about 10 m. */
export const CRUISE_AGL = 13;
/** Keep this far inside the fence and outside no-fly polygons. */
export const AIRSPACE_MARGIN = 2.5;
/** Swept radius used for obstacle casts: airframe plus air. */
export const SWEEP_RADIUS = DRONE_RADIUS + 0.15;

// ------------------------------------------------------------- vectors

export function sub(a: V3, b: V3): V3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function add(a: V3, b: V3): V3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function scale(a: V3, s: number): V3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function dot(a: V3, b: V3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function cross(a: V3, b: V3): V3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

export function len(a: V3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

export function dist(a: V3, b: V3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

export function dist2d(a: V3 | Point, b: V3 | Point): number {
  const ax = Array.isArray(a) ? a[0] : a.x;
  const ay = Array.isArray(a) ? a[1] : a.y;
  const bx = Array.isArray(b) ? b[0] : b.x;
  const by = Array.isArray(b) ? b[1] : b.y;
  return Math.hypot(ax - bx, ay - by);
}

export function unit(a: V3): V3 {
  const l = len(a);
  return l < 1e-9 ? [0, 0, 0] : [a[0] / l, a[1] / l, a[2] / l];
}

export function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

/** Wrap to (-pi, pi]. */
export function wrapAngle(a: number): number {
  const tau = Math.PI * 2;
  let r = ((a + Math.PI) % tau + tau) % tau - Math.PI;
  if (r <= -Math.PI) r += tau;
  return r;
}

export function headingTo(from: V3, to: V3): number {
  return Math.atan2(to[1] - from[1], to[0] - from[0]);
}

// ------------------------------------------------------------- airspace

export interface Airspace {
  fence: Point[];
  /** Mission no-fly polygons plus the site plan's zones. Avoided at every altitude. */
  noFly: Point[][];
  terrain: Terrain;
  minAgl: number;
  maxAgl: number;
}

export function airspaceFor(mission: Mission, world: MissionWorld): Airspace {
  const noFly = mission.noFlyZones.map((zone) => zone.polygon.points.map((p) => ({ x: p.x, y: p.y })));
  for (const zone of world.outdoor.noFly) noFly.push(zone.polygon.map(([x, y]) => ({ x, y })));
  return {
    fence: mission.geofence.points.map((p) => ({ x: p.x, y: p.y })),
    noFly,
    terrain: world.outdoor.terrain,
    minAgl: mission.altitude.minM,
    maxAgl: mission.altitude.maxM,
  };
}

export function inRing(x: number, y: number, ring: readonly Point[]): boolean {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const pi = ring[i]!;
    const pj = ring[j]!;
    if (pi.y > y !== pj.y > y && x < ((pj.x - pi.x) * (y - pi.y)) / (pj.y - pi.y) + pi.x) inside = !inside;
  }
  return inside;
}

function segDist(px: number, py: number, a: Point, b: Point): number {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const l2 = abx * abx + aby * aby;
  let t = l2 < 1e-12 ? 0 : ((px - a.x) * abx + (py - a.y) * aby) / l2;
  t = clamp(t, 0, 1);
  return Math.hypot(px - (a.x + abx * t), py - (a.y + aby * t));
}

export function ringEdgeDist(x: number, y: number, ring: readonly Point[]): number {
  let best = Infinity;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) best = Math.min(best, segDist(x, y, ring[j]!, ring[i]!));
  return best;
}

/** Inside the fence by `margin` and at least `margin` outside every no-fly polygon. */
export function operable(air: Airspace, x: number, y: number, margin = AIRSPACE_MARGIN): boolean {
  if (!inRing(x, y, air.fence)) return false;
  if (margin > 0 && ringEdgeDist(x, y, air.fence) < margin) return false;
  for (const zone of air.noFly) {
    if (inRing(x, y, zone)) return false;
    if (margin > 0 && ringEdgeDist(x, y, zone) < margin) return false;
  }
  return true;
}

export function segmentOperable(air: Airspace, a: Point, b: Point, margin = AIRSPACE_MARGIN): boolean {
  const n = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.5));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    if (!operable(air, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, margin)) return false;
  }
  return true;
}

function ringBounds(ring: readonly Point[]): { minX: number; minY: number; maxX: number; maxY: number } {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const p of ring) {
    minX = Math.min(minX, p.x);
    minY = Math.min(minY, p.y);
    maxX = Math.max(maxX, p.x);
    maxY = Math.max(maxY, p.y);
  }
  return { minX, minY, maxX, maxY };
}

export function fenceBounds(air: Airspace): { minX: number; minY: number; maxX: number; maxY: number } {
  return ringBounds(air.fence);
}

/**
 * Horizontal route that stays operable. Straight when possible, otherwise
 * A* on a 1 m grid with string pulling. Null when no route exists.
 */
export function routeXY(air: Airspace, from: Point, to: Point, margin = AIRSPACE_MARGIN): Point[] | null {
  if (segmentOperable(air, from, to, margin)) return [from, to];
  const box = ringBounds(air.fence);
  const cell = 1;
  const w = Math.ceil((box.maxX - box.minX) / cell) + 1;
  const h = Math.ceil((box.maxY - box.minY) / cell) + 1;
  const free = new Uint8Array(w * h);
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) free[j * w + i] = operable(air, box.minX + i * cell, box.minY + j * cell, margin) ? 1 : 0;
  }
  const idxOf = (p: Point): number => {
    let best = -1;
    let bestD = Infinity;
    const ci = Math.round((p.x - box.minX) / cell);
    const cj = Math.round((p.y - box.minY) / cell);
    for (let r = 0; r < 6 && best < 0; r++) {
      for (let j = cj - r; j <= cj + r; j++) {
        for (let i = ci - r; i <= ci + r; i++) {
          if (i < 0 || j < 0 || i >= w || j >= h || !free[j * w + i]) continue;
          const d = Math.hypot(box.minX + i * cell - p.x, box.minY + j * cell - p.y);
          if (d < bestD) {
            bestD = d;
            best = j * w + i;
          }
        }
      }
    }
    return best;
  };
  const start = idxOf(from);
  const goal = idxOf(to);
  if (start < 0 || goal < 0) return null;
  const g = new Float64Array(w * h).fill(Infinity);
  const prev = new Int32Array(w * h).fill(-1);
  const closed = new Uint8Array(w * h);
  const gx = goal % w;
  const gy = Math.floor(goal / w);
  const heur = (k: number) => Math.hypot((k % w) - gx, Math.floor(k / w) - gy);
  const open: number[] = [start];
  g[start] = 0;
  const fOf = (k: number) => g[k]! + heur(k);
  while (open.length) {
    let bi = 0;
    for (let k = 1; k < open.length; k++) if (fOf(open[k]!) < fOf(open[bi]!)) bi = k;
    const cur = open[bi]!;
    open.splice(bi, 1);
    if (cur === goal) break;
    if (closed[cur]) continue;
    closed[cur] = 1;
    const cx = cur % w;
    const cy = Math.floor(cur / w);
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        if (!dx && !dy) continue;
        const nx = cx + dx;
        const ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const nk = ny * w + nx;
        if (!free[nk] || closed[nk]) continue;
        if (dx && dy && (!free[cy * w + nx] || !free[ny * w + cx])) continue;
        const cost = g[cur]! + (dx && dy ? Math.SQRT2 : 1);
        if (cost < g[nk]!) {
          g[nk] = cost;
          prev[nk] = cur;
          open.push(nk);
        }
      }
    }
  }
  if (!Number.isFinite(g[goal]!)) return null;
  const cells: Point[] = [];
  for (let k = goal; k >= 0; k = prev[k]!) {
    cells.push({ x: box.minX + (k % w) * cell, y: box.minY + Math.floor(k / w) * cell });
    if (k === start) break;
  }
  cells.reverse();
  const raw = [from, ...cells, to];
  const out: Point[] = [raw[0]!];
  let anchor = 0;
  while (anchor < raw.length - 1) {
    let next = anchor + 1;
    for (let k = raw.length - 1; k > anchor + 1; k--) {
      if (segmentOperable(air, raw[anchor]!, raw[k]!, margin * 0.8)) {
        next = k;
        break;
      }
    }
    out.push(raw[next]!);
    anchor = next;
  }
  return out;
}

/** Highest ground within `radius` of a point. */
export function groundMax(terrain: Terrain, x: number, y: number, radius = 3): number {
  let best = sampleHeight(terrain, x, y);
  for (let k = 0; k < 8; k++) {
    const a = (k / 8) * Math.PI * 2;
    best = Math.max(best, sampleHeight(terrain, x + Math.cos(a) * radius, y + Math.sin(a) * radius));
  }
  return best;
}

/** Cruise altitude for a point, clamped into the mission band. */
export function cruiseZ(air: Airspace, x: number, y: number, agl = CRUISE_AGL): number {
  const ground = sampleHeight(air.terrain, x, y);
  const want = groundMax(air.terrain, x, y) + agl;
  return clamp(want, ground + air.minAgl + 0.2, ground + air.maxAgl - 0.2);
}

/** Terrain-following cruise path between two points, densified every `step` meters. */
export function cruisePath(air: Airspace, from: V3, to: Point, agl = CRUISE_AGL, step = 5): V3[] {
  const route = routeXY(air, { x: from[0], y: from[1] }, to) ?? routeXY(air, { x: from[0], y: from[1] }, to, 0.5) ?? [
    { x: from[0], y: from[1] },
    to,
  ];
  return densify(air, route, agl, step).slice(1);
}

export function densify(air: Airspace, route: readonly Point[], agl = CRUISE_AGL, step = 5): V3[] {
  const out: V3[] = [];
  for (let i = 0; i < route.length; i++) {
    const a = route[i]!;
    if (i === 0) {
      out.push([a.x, a.y, cruiseZ(air, a.x, a.y, agl)]);
      continue;
    }
    const p = route[i - 1]!;
    const n = Math.max(1, Math.ceil(Math.hypot(a.x - p.x, a.y - p.y) / step));
    for (let k = 1; k <= n; k++) {
      const x = p.x + ((a.x - p.x) * k) / n;
      const y = p.y + ((a.y - p.y) * k) / n;
      out.push([x, y, cruiseZ(air, x, y, agl)]);
    }
  }
  return out;
}

// ------------------------------------------------------------- obstacles

const SOLID_TAGS = new Set(['terrain', 'trunk', 'canopy', 'undergrowth', 'wall', 'floor', 'jamb', 'door', 'roof']);

/** True when a sphere of `radius` can travel a -> b. Seven parallel casts. */
export function segmentClear(physics: OutdoorPhysics, a: V3, b: V3, radius = SWEEP_RADIUS): boolean {
  const d = sub(b, a);
  const length = len(d);
  if (length < 1e-6) return pointFree(physics, a, radius);
  const dir = scale(d, 1 / length);
  let side = cross(dir, [0, 0, 1]);
  if (len(side) < 1e-3) side = [1, 0, 0];
  side = unit(side);
  const up = unit(cross(side, dir));
  const offsets: [number, number][] = [
    [0, 0],
    [1, 0],
    [-1, 0],
    [0.5, 0],
    [-0.5, 0],
    [0, 1],
    [0, -1],
  ];
  for (const [s, u] of offsets) {
    const o = add(a, add(scale(side, s * radius), scale(up, u * radius)));
    const start = sub(o, scale(dir, radius * 0.5));
    const hit = physics.cast(start[0], start[1], start[2], dir[0], dir[1], dir[2], length + radius);
    if (hit && SOLID_TAGS.has(hit.tag)) return false;
  }
  return true;
}

/** No solid within `radius` along 10 directions. */
export function pointFree(physics: OutdoorPhysics, p: V3, radius = 0.45): boolean {
  const dirs: V3[] = [
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, -1, 0],
    [0, 0, 1],
    [0, 0, -1],
    [0.707, 0.707, 0],
    [-0.707, 0.707, 0],
    [0.707, -0.707, 0],
    [-0.707, -0.707, 0],
  ];
  for (const d of dirs) {
    if (physics.cast(p[0], p[1], p[2], d[0], d[1], d[2], radius)) return false;
  }
  return true;
}

/** Vertical column from `zLow` to `zHigh` free of canopy and other solids. */
export function columnClear(physics: OutdoorPhysics, x: number, y: number, zLow: number, zHigh: number, radius = 0.6): boolean {
  if (zHigh <= zLow) return true;
  const span = zHigh - zLow;
  const pts: [number, number][] = [[0, 0]];
  for (let k = 0; k < 8; k++) {
    const a = (k / 8) * Math.PI * 2;
    pts.push([Math.cos(a) * radius, Math.sin(a) * radius]);
  }
  for (const [ox, oy] of pts) {
    if (physics.cast(x + ox, y + oy, zHigh, 0, 0, -1, span)) return false;
    if (physics.cast(x + ox, y + oy, zLow, 0, 0, 1, span)) return false;
  }
  return true;
}

// ------------------------------------------------------------- buildings

export interface BuildingBox {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
  /** Pad height (world z of the ground floor). */
  baseZ: number;
  roofZ: number;
  cx: number;
  cy: number;
}

export function buildingBox(building: MissionBuilding): BuildingBox {
  const shell = building.shell;
  const floor = shell.spec.floors[0]!;
  const w = floor.w * shell.spec.cellSize;
  const d = floor.d * shell.spec.cellSize;
  const baseZ = shell.origin[2];
  return {
    minX: shell.origin[0],
    minY: shell.origin[1],
    maxX: shell.origin[0] + w,
    maxY: shell.origin[1] + d,
    baseZ,
    roofZ: baseZ + shell.spec.floors.length * shell.spec.floorHeight,
    cx: shell.origin[0] + w / 2,
    cy: shell.origin[1] + d / 2,
  };
}

/** Inside the shell volume (with an optional margin, negative shrinks). */
export function insideBox(box: BuildingBox, p: V3, margin = 0): boolean {
  return (
    p[0] > box.minX - margin &&
    p[0] < box.maxX + margin &&
    p[1] > box.minY - margin &&
    p[1] < box.maxY + margin &&
    p[2] > box.baseZ - 0.5 &&
    p[2] < box.roofZ - margin
  );
}

/** A local route between two points near a building: direct, or a hop over the roof line. */
export function legBetween(physics: OutdoorPhysics, a: V3, b: V3, hopZ: number, margin = 0): V3[] | null {
  // Every leg, straight or hopping over, must clear solids by `margin` more than the usual sweep: the
  // climb and descent columns pass canopy and eaves as closely as a straight leg passes a corner.
  const r = SWEEP_RADIUS + margin;
  if (segmentClear(physics, a, b, r)) return [b];
  for (const z of [hopZ, hopZ + 3]) {
    const aUp: V3 = [a[0], a[1], Math.max(a[2], z)];
    const bUp: V3 = [b[0], b[1], Math.max(b[2], z)];
    if (segmentClear(physics, a, aUp, r) && segmentClear(physics, aUp, bUp, r) && segmentClear(physics, bUp, b, r)) {
      return [aUp, bUp, b];
    }
  }
  return null;
}
