import { isWalkable } from '../../../gen/validate';
import { indoorCells, type GridCell } from '../../../perception/seenEmpty';
import type { InferPolicy } from '../../../rl/infer';

/** What the explorer needs from a policy: a deterministic mean action. */
export type ActingPolicy = Pick<InferPolicy, 'actWith'>;
import { CELL, DRONE_RADIUS, FLOOR_HEIGHT, LIDAR_H, LIDAR_MAX, MAX_SPEED_H, OBS, OBS_DIM, VOXEL } from '../../../shared/constants';
import { RNG } from '../../../shared/rng';
import type { BuildingSpec, LightingState } from '../../../shared/types';
import { makeDroneState, scaleAction } from '../../../sim/drone';
import { zeroNoise } from '../../../sim/dynamics/dr';
import { SensorNoise } from '../../../sim/dynamics/noise';
import { FrontierNav } from '../../../sim/frontierFeatures';
import { EgoMaps, SeenMap } from '../../../sim/seenMap';
import { SensorPlanner, type SensorFrame } from '../../../sim/sensorPlanner';
import { BRAKING_MARGIN, brakingFilter, type BrakingModel, descentClear, filterVelocity, guardAction, SAFETY_MARGIN } from '../../../sim/lidarSafety';
import { OccupancyMap, sensorClearance, sensorLos, sensorPath, type SensorGrid } from '../../../sim/sensorMap';
import { sense } from '../../../sim/sensors';
import type { PhysicsWorld } from '../../../sim/world';
import type { MissionBuilding, V3 } from '../contracts';
import type { OutdoorPhysics } from '../../../world/outdoor/physics';
import { clamp, wrapAngle } from './geometry';

/**
 * Indoor exploration on the building grid, in the building-local frame
 * (world minus shell.origin). The frontier explorer keeps its own camera
 * visibility estimate over the same cells the seen-empty maps use, picks
 * the reachable cell that would reveal the most unseen cells per meter of
 * travel, flies there on a clearance-checked grid path, and turns a full
 * circle. Shaft cells link floors. That explorer plans on the true floor
 * plan, so it is kept only as the 'oracle' reference. Missions fly the
 * trained policy and the sensor-only explorer, which read the drone's own
 * maps (IndoorSensing below).
 */

/** Flight height above each floor, meters. */
export const INDOOR_FLY_Z = 1.2;
/** Range the visibility estimate trusts, meters. */
export const SEE_RANGE = 6;
/** Range the sensor-map look estimate trusts, meters (shorter: the map is coarser than the floor plan). */
export const LOOK_RANGE = 5;
/** Half of the horizontal angle the estimate trusts, radians. */
export const SEE_HALF_FOV = (55 * Math.PI) / 180;
/** Free air kept around the airframe on planned paths, meters. */
export const PATH_CLEARANCE = DRONE_RADIUS + 0.17;

export interface IndoorGrid {
  building: MissionBuilding;
  spec: BuildingSpec;
  origin: V3;
  w: number;
  d: number;
  floors: number;
  cs: number;
  H: number;
  /** Per floor: 1 where the planner may fly. The outer wall ring is excluded. */
  nav: Uint8Array[];
}

export interface IndoorCell {
  f: number;
  x: number;
  z: number;
}

export function indoorGrid(building: MissionBuilding): IndoorGrid {
  const spec = building.shell.spec;
  const w = spec.floors[0]!.w;
  const d = spec.floors[0]!.d;
  const nav = spec.floors.map((floor) => {
    const mask = new Uint8Array(w * d);
    for (let z = 1; z < d - 1; z++) {
      for (let x = 1; x < w - 1; x++) if (isWalkable(floor.cells[z * w + x]!)) mask[z * w + x] = 1;
    }
    return mask;
  });
  return {
    building,
    spec,
    origin: [building.shell.origin[0], building.shell.origin[1], building.shell.origin[2]],
    w,
    d,
    floors: spec.floors.length,
    cs: spec.cellSize,
    H: spec.floorHeight,
    nav,
  };
}

export function toLocal(grid: IndoorGrid, p: V3): V3 {
  return [p[0] - grid.origin[0], p[1] - grid.origin[1], p[2] - grid.origin[2]];
}

export function toWorld(grid: IndoorGrid, p: V3): V3 {
  return [p[0] + grid.origin[0], p[1] + grid.origin[1], p[2] + grid.origin[2]];
}

export function floorOf(grid: IndoorGrid, z: number): number {
  return clamp(Math.floor(z / grid.H), 0, grid.floors - 1);
}

export function cellAt(grid: IndoorGrid, local: V3): IndoorCell {
  return {
    f: floorOf(grid, local[2]),
    x: clamp(Math.floor(local[0] / grid.cs), 0, grid.w - 1),
    z: clamp(Math.floor(local[1] / grid.cs), 0, grid.d - 1),
  };
}

export function cellCenter(grid: IndoorGrid, c: IndoorCell): V3 {
  return [(c.x + 0.5) * grid.cs, (c.z + 0.5) * grid.cs, c.f * grid.H + INDOOR_FLY_Z];
}

function isNav(grid: IndoorGrid, f: number, x: number, z: number): boolean {
  if (f < 0 || f >= grid.floors || x < 0 || z < 0 || x >= grid.w || z >= grid.d) return false;
  return grid.nav[f]![z * grid.w + x] === 1;
}

function isOpen(grid: IndoorGrid, f: number, x: number, z: number): boolean {
  if (f < 0 || f >= grid.floors || x < 0 || z < 0 || x >= grid.w || z >= grid.d) return false;
  return isWalkable(grid.spec.floors[f]!.cells[z * grid.w + x]!);
}

function key(grid: IndoorGrid, c: IndoorCell): number {
  return (c.f * grid.d + c.z) * grid.w + c.x;
}

function unkey(grid: IndoorGrid, k: number): IndoorCell {
  const x = k % grid.w;
  const z = Math.floor(k / grid.w) % grid.d;
  const f = Math.floor(k / (grid.w * grid.d));
  return { f, x, z };
}

/** Nearest navigable cell on the same floor, searching outward. */
export function nearestNav(grid: IndoorGrid, local: V3): IndoorCell | null {
  const c = cellAt(grid, local);
  if (isNav(grid, c.f, c.x, c.z)) return c;
  let best: IndoorCell | null = null;
  let bestD = Infinity;
  for (let z = 0; z < grid.d; z++) {
    for (let x = 0; x < grid.w; x++) {
      if (!isNav(grid, c.f, x, z)) continue;
      const dd = Math.hypot((x + 0.5) * grid.cs - local[0], (z + 0.5) * grid.cs - local[1]);
      if (dd < bestD) {
        bestD = dd;
        best = { f: c.f, x, z };
      }
    }
  }
  return best;
}

function neighbours(grid: IndoorGrid, c: IndoorCell): { c: IndoorCell; cost: number }[] {
  const out: { c: IndoorCell; cost: number }[] = [];
  for (const [dx, dz] of [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ] as const) {
    if (isNav(grid, c.f, c.x + dx, c.z + dz)) out.push({ c: { f: c.f, x: c.x + dx, z: c.z + dz }, cost: 1 });
  }
  const here = grid.spec.floors[c.f]!.cells[c.z * grid.w + c.x];
  if (here === CELL.SHAFT) {
    for (const nf of [c.f + 1, c.f - 1]) {
      if (nf < 0 || nf >= grid.floors) continue;
      if (grid.spec.floors[nf]!.cells[c.z * grid.w + c.x] === CELL.SHAFT && isNav(grid, nf, c.x, c.z)) {
        out.push({ c: { f: nf, x: c.x, z: c.z }, cost: 3 });
      }
    }
  }
  return out;
}

/** Dijkstra over nav cells with shaft links. Returns distances and parents by cell key. */
export function gridDistances(grid: IndoorGrid, from: IndoorCell): { dist: Float64Array; prev: Int32Array } {
  const n = grid.floors * grid.w * grid.d;
  const dist = new Float64Array(n).fill(Infinity);
  const prev = new Int32Array(n).fill(-1);
  const start = key(grid, from);
  dist[start] = 0;
  const open: number[] = [start];
  while (open.length) {
    let bi = 0;
    for (let k = 1; k < open.length; k++) if (dist[open[k]!]! < dist[open[bi]!]!) bi = k;
    const cur = open[bi]!;
    open.splice(bi, 1);
    for (const { c, cost } of neighbours(grid, unkey(grid, cur))) {
      const nk = key(grid, c);
      const nd = dist[cur]! + cost;
      if (nd < dist[nk]!) {
        if (!Number.isFinite(dist[nk]!)) open.push(nk);
        dist[nk] = nd;
        prev[nk] = cur;
      }
    }
  }
  return { dist, prev };
}

/** Distance from a point to the nearest blocked cell edge on a floor, capped at 1 m. */
export function wallClearance(grid: IndoorGrid, f: number, x: number, y: number): number {
  let best = 1;
  const gx = Math.floor(x / grid.cs);
  const gz = Math.floor(y / grid.cs);
  for (let dz = -1; dz <= 1; dz++) {
    for (let dx = -1; dx <= 1; dx++) {
      const cx = gx + dx;
      const cz = gz + dz;
      if (isNav(grid, f, cx, cz)) continue;
      const nx = clamp(x, cx * grid.cs, (cx + 1) * grid.cs);
      const ny = clamp(y, cz * grid.cs, (cz + 1) * grid.cs);
      best = Math.min(best, Math.hypot(x - nx, y - ny));
    }
  }
  return best;
}

function segmentSafe(grid: IndoorGrid, f: number, a: V3, b: V3): boolean {
  const l = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const n = Math.max(1, Math.ceil(l / 0.08));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    if (wallClearance(grid, f, a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t) < PATH_CLEARANCE) return false;
  }
  return true;
}

/** Cells to local waypoints, string-pulled per floor while keeping PATH_CLEARANCE. */
export function cellsToPath(grid: IndoorGrid, cells: readonly IndoorCell[]): V3[] {
  const pts = cells.map((c) => cellCenter(grid, c));
  if (pts.length <= 2) return pts.slice(1);
  const out: V3[] = [];
  let anchor = 0;
  while (anchor < pts.length - 1) {
    let next = anchor + 1;
    const f = cells[anchor]!.f;
    if (cells[next]!.f === f) {
      for (let k = pts.length - 1; k > anchor + 1; k--) {
        let sameFloor = true;
        for (let j = anchor; j <= k; j++) if (cells[j]!.f !== f) sameFloor = false;
        if (sameFloor && segmentSafe(grid, f, pts[anchor]!, pts[k]!)) {
          next = k;
          break;
        }
      }
    }
    out.push(pts[next]!);
    anchor = next;
  }
  return out;
}

/** Local waypoint path between two local points, or null when unreachable. */
export function planIndoorPath(grid: IndoorGrid, from: V3, to: V3): V3[] | null {
  const a = nearestNav(grid, from);
  const b = nearestNav(grid, to);
  if (!a || !b) return null;
  const { dist, prev } = gridDistances(grid, a);
  const goal = key(grid, b);
  if (!Number.isFinite(dist[goal]!)) return null;
  const cells: IndoorCell[] = [];
  for (let k = goal; k >= 0; k = prev[k]!) cells.push(unkey(grid, k));
  cells.reverse();
  const first = cellCenter(grid, a);
  const path = cellsToPath(grid, cells);
  const out: V3[] = [];
  if (Math.hypot(first[0] - from[0], first[1] - from[1]) > 0.05 || Math.abs(first[2] - from[2]) > 0.2) out.push(first);
  out.push(...path);
  return out;
}

// ------------------------------------------------------------- visibility

/** Line of sight on a floor grid between two local xy points. Walls block. */
export function gridLos(grid: IndoorGrid, f: number, ax: number, ay: number, bx: number, by: number): boolean {
  const l = Math.hypot(bx - ax, by - ay);
  const n = Math.max(1, Math.ceil(l / 0.1));
  for (let i = 1; i < n; i++) {
    const t = i / n;
    const x = Math.floor((ax + (bx - ax) * t) / grid.cs);
    const z = Math.floor((ay + (by - ay) * t) / grid.cs);
    if (!isOpen(grid, f, x, z)) return false;
  }
  return true;
}

/** The explorer's own estimate of which seen-empty cells the camera has covered. */
export class IndoorVisibility {
  readonly cells: GridCell[];
  readonly seen: Uint8Array;
  private readonly cellXY: Int32Array;
  private count = 0;

  constructor(
    readonly grid: IndoorGrid,
    readonly range = SEE_RANGE,
    readonly halfFov = SEE_HALF_FOV,
  ) {
    this.cells = indoorCells(grid.spec);
    this.seen = new Uint8Array(this.cells.length);
    this.cellXY = new Int32Array(this.cells.length * 2);
    this.cells.forEach((cell, i) => {
      this.cellXY[i * 2] = Math.floor(cell.center[0] / grid.cs);
      this.cellXY[i * 2 + 1] = Math.floor(cell.center[1] / grid.cs);
    });
  }

  /** Mark cells seen from a local pose. Returns how many were new. */
  observe(local: V3, yaw: number): number {
    const f = floorOf(this.grid, local[2]);
    let fresh = 0;
    for (let i = 0; i < this.cells.length; i++) {
      if (this.seen[i]) continue;
      const cell = this.cells[i]!;
      if (cell.floor !== f) continue;
      const dx = cell.center[0] - local[0];
      const dy = cell.center[1] - local[1];
      const d = Math.hypot(dx, dy);
      if (d > this.range) continue;
      if (d > 0.7 && Math.abs(wrapAngle(Math.atan2(dy, dx) - yaw)) > this.halfFov) continue;
      if (!gridLos(this.grid, f, local[0], local[1], cell.center[0], cell.center[1])) continue;
      this.seen[i] = 1;
      fresh += 1;
    }
    this.count += fresh;
    return fresh;
  }

  coverage(): number {
    return this.cells.length ? this.count / this.cells.length : 1;
  }

  reset(): void {
    this.seen.fill(0);
    this.count = 0;
  }

  unseen(): number[] {
    const out: number[] = [];
    for (let i = 0; i < this.cells.length; i++) if (!this.seen[i]) out.push(i);
    return out;
  }
}

// ------------------------------------------------------------- frontier

export interface IndoorGoal {
  /** Local waypoints to the goal cell. */
  path: V3[];
  /** Turn a full circle at the goal. */
  spin: boolean;
  /** Unseen cells the goal should reveal. */
  gain: number;
}

export class FrontierExplorer {
  private readonly tried = new Set<number>();

  constructor(
    readonly grid: IndoorGrid,
    readonly vis: IndoorVisibility,
    /** Radius a goal is credited with revealing, inside SEE_RANGE so a spin delivers it. */
    readonly gainRange = SEE_RANGE - 1,
  ) {}

  /** Next goal from a local position, or null when nothing reachable is left unseen. */
  next(local: V3): IndoorGoal | null {
    const start = nearestNav(this.grid, local);
    if (!start) return null;
    const unseen = this.vis.unseen();
    if (!unseen.length) return null;
    const { dist, prev } = gridDistances(this.grid, start);
    let bestK = -1;
    let bestScore = 0;
    let bestGain = 0;
    for (let k = 0; k < dist.length; k++) {
      const d = dist[k]!;
      if (!Number.isFinite(d) || this.tried.has(k)) continue;
      const c = unkey(this.grid, k);
      const p = cellCenter(this.grid, c);
      let gain = 0;
      for (const i of unseen) {
        const cell = this.vis.cells[i]!;
        if (cell.floor !== c.f) continue;
        const dx = cell.center[0] - p[0];
        const dy = cell.center[1] - p[1];
        if (dx * dx + dy * dy > this.gainRange * this.gainRange) continue;
        if (gridLos(this.grid, c.f, p[0], p[1], cell.center[0], cell.center[1])) gain += 1;
      }
      if (gain < 1) continue;
      const score = gain / (1 + 0.3 * d);
      if (score > bestScore + 1e-9) {
        bestScore = score;
        bestK = k;
        bestGain = gain;
      }
    }
    if (bestK < 0) return null;
    this.tried.add(bestK);
    const cells: IndoorCell[] = [];
    for (let k = bestK; k >= 0; k = prev[k]!) cells.push(unkey(this.grid, k));
    cells.reverse();
    const path: V3[] = [];
    const first = cellCenter(this.grid, start);
    if (Math.hypot(first[0] - local[0], first[1] - local[1]) > 0.05 || Math.abs(first[2] - local[2]) > 0.2) path.push(first);
    path.push(...cellsToPath(this.grid, cells));
    if (!path.length) path.push(first);
    return { path, spin: true, gain: bestGain };
  }
}

// ------------------------------------------------------------- sensing

/** The lidar's view of the world, in the building-local frame. */
export type LidarWorld = Pick<PhysicsWorld, 'cast'>;

/**
 * Cast the indoor lidar against the mission's real collision world (walls,
 * jambs, ajar doors, window holes), not the floor plan's idealized walls, so
 * the drone senses what it can actually hit. Glass comes from the building's
 * surface classes at the hit point.
 */
export function shellLidar(grid: IndoorGrid, cast: OutdoorPhysics['cast']): LidarWorld {
  const [ox, oy, oz] = grid.origin;
  const spec = grid.spec;
  return {
    cast(x, y, z, dx, dy, dz, maxDist) {
      const hit = cast(x + ox, y + oy, z + oz, dx, dy, dz, maxDist);
      if (!hit) return null;
      const len = Math.hypot(dx, dy, dz) || 1;
      const hx = x + (dx / len) * hit.dist;
      const hy = y + (dy / len) * hit.dist;
      const f = clamp(Math.floor((z + (dz / len) * hit.dist) / spec.floorHeight), 0, spec.floors.length - 1);
      const floor = spec.floors[f]!;
      const gx = clamp(Math.floor(hx / spec.cellSize), 0, floor.w - 1);
      const gy = clamp(Math.floor(hy / spec.cellSize), 0, floor.d - 1);
      return {
        dist: hit.dist,
        surface: floor.surfaces?.[gy * floor.w + gx] ?? 0,
      };
    },
  };
}

/**
 * The drone's own view of a building it is inside: the 1088-float
 * observation built by sense() on a PhysicsWorld of the building in the local
 * frame, the seen-free map its lidar paints, and the voxels it has flown.
 * One instance lives for a whole visit, so a fallback explorer inherits the
 * map the policy built. Nothing in here reads the floor plan for control:
 * spec is handed to sense() only as the world the simulated lidar hits.
 */
/** Height above the floor the dark look estimate checks: a small object lying there. */
const LOOK_POINT_Z = 0.25;

/** Angle between the camera axis (perception/camera.ts fpvCameraFromDrone) and a point. */
function coneAngle(local: V3, yaw: number, px: number, py: number, pz: number): number {
  const nx = Math.cos(yaw);
  const ny = Math.sin(yaw);
  const ex = local[0] + nx * 0.1;
  const ey = local[1] + ny * 0.1;
  const ez = local[2] + 0.02;
  const fx = nx * 2.9;
  const fy = ny * 2.9;
  const fz = -0.57;
  const dx = px - ex;
  const dy = py - ey;
  const dz = pz - ez;
  const c = (fx * dx + fy * dy + fz * dz) / (Math.hypot(fx, fy, fz) * Math.hypot(dx, dy, dz));
  return Math.acos(clamp(c, -1, 1));
}

export class IndoorSensing implements SensorGrid {
  readonly visited: Uint8Array;
  private readonly occ: OccupancyMap;
  /** Voxels the camera has looked at, by the drone's own estimate (see look()). */
  readonly looked: Uint8Array;
  /** Voxels where the up (down) range finder saw past the ceiling (floor): a shaft to the next floor. */
  readonly shaftUp: Uint8Array;
  readonly shaftDown: Uint8Array;
  /**
   * Per shaft voxel: summed x, y and count of the positions the range finder
   * saw through from. Their mean is inside the hole whatever the voxel grid's
   * alignment to the building, which a voxel centre is not.
   */
  private readonly shaftObsUp: Float64Array;
  private readonly shaftObsDown: Float64Array;
  /** Shaft voxels the drone tried and could not fly through; never marked again. */
  private readonly shaftRejected: Uint8Array;
  /** Camera half-angle and range the look estimate trusts; set from the lighting the drone knows. */
  lookHalf = SEE_HALF_FOV;
  lookRange = LOOK_RANGE;
  /**
   * In the dark: half-angle of the flashlight cone around the camera axis
   * (nose, tilted down as in perception/camera.ts). A voxel counts as looked
   * at only when a point just above its floor falls inside the cone, so the
   * floor right under the drone is not counted. Null in daylight.
   */
  lookCone: number | null = null;
  /** Closest distance at which a viewpoint can light a voxel on the floor (see lookCone). */
  lookMin = 0;
  readonly vw: number;
  readonly vd: number;
  visitedCount = 0;
  lastNewVoxelSec = 0;
  readonly obs = new Float32Array(OBS_DIM);
  readonly lidar = new Float64Array(LIDAR_H);
  private readonly noise: SensorNoise;
  readonly rng: RNG;
  private prevVel: [number, number, number] = [0, 0, 0];
  yawRate = 0;
  /** Stopping model of the airframe flying this building; when set, filter() is the braking filter. */
  braking: BrakingModel | null = null;
  /**
   * Called on the lidar ranges after every reading and before anything uses
   * them, so a fault model can rewrite them in place. A NaN range is a
   * missing reading; the filters treat it as blocked.
   */
  rangeHook: ((lidar: Float64Array, tSec: number) => void) | null = null;
  private readonly velWorld: [number, number] = [0, 0];
  private steps = 0;
  readonly maxSteps = 2400;
  private readonly seenMap = new SeenMap();
  private readonly nav = new FrontierNav();
  private readonly maps = new EgoMaps();
  private stall = 0;

  constructor(
    readonly grid: IndoorGrid,
    readonly physics: LidarWorld,
    readonly lighting: LightingState,
    seed: number,
  ) {
    const floor0 = grid.spec.floors[0]!;
    this.vw = floor0.w * 2;
    this.vd = floor0.d * 2;
    this.visited = new Uint8Array(grid.floors * this.vw * this.vd);
    this.occ = new OccupancyMap(this.vw, this.vd, grid.floors);
    // The outer wall ring: the envelope inspection measured the building from outside and the
    // fly-through crossed one cell of wall. Lidar through a window hole would paint it free.
    for (let f = 0; f < grid.floors; f++) {
      for (let y = 0; y < this.vd; y++) {
        for (let x = 0; x < this.vw; x++) {
          if (x < 2 || y < 2 || x >= this.vw - 2 || y >= this.vd - 2) this.occ.block(f * this.vw * this.vd + y * this.vw + x);
        }
      }
    }
    this.looked = new Uint8Array(grid.floors * this.vw * this.vd);
    this.shaftUp = new Uint8Array(grid.floors * this.vw * this.vd);
    this.shaftDown = new Uint8Array(grid.floors * this.vw * this.vd);
    this.shaftObsUp = new Float64Array(grid.floors * this.vw * this.vd * 3);
    this.shaftRejected = new Uint8Array(grid.floors * this.vw * this.vd);
    this.shaftObsDown = new Float64Array(grid.floors * this.vw * this.vd * 3);
    this.rng = new RNG(seed >>> 0, 'indoor-policy');
    this.noise = new SensorNoise(zeroNoise(), this.rng.fork('noise'));
    // The seen-free map may paint any voxel inside the outer wall ring, not just the floor plan's
    // reachable ones: the drone knows its footprint from outside, not which voxels it can fly.
    const inside = new Uint8Array(grid.floors * this.vw * this.vd);
    for (let i = 0; i < inside.length; i++) {
      const x = i % this.vw;
      const y = Math.floor(i / this.vw) % this.vd;
      inside[i] = x >= 2 && y >= 2 && x < this.vw - 2 && y < this.vd - 2 ? 1 : 0;
    }
    this.seenMap.reset(grid.spec, inside);
  }

  get seen(): Uint8Array {
    return this.seenMap.data;
  }

  /** Mark the voxel the drone is in, by its own pose, as flown. True when it is new. */
  private markFlown(local: V3): boolean {
    const vx = Math.floor(local[0] / VOXEL);
    const vy = Math.floor(local[1] / VOXEL);
    const f = Math.floor(local[2] / this.grid.H);
    if (vx < 0 || vy < 0 || vx >= this.vw || vy >= this.vd || f < 0 || f >= this.grid.floors) return false;
    const k = f * this.vw * this.vd + vy * this.vw + vx;
    if (this.visited[k]) return false;
    this.visited[k] = 1;
    return true;
  }

  get occupied(): Uint8Array {
    return this.occ.occupied;
  }

  /**
   * Ranges the drone's own map predicts for the horizontal lidar ring from a local pose: the first
   * wall each beam would meet among voxels the lidar has returned from at about this height, or
   * Infinity. The fault failsafe compares them with the ring to catch beams that read open wrongly.
   */
  expectedRanges(local: V3, yaw: number, out: Float64Array): Float64Array {
    return this.occ.expected(floorOf(this.grid, local[2]), local[0], local[1], local[2], yaw, out);
  }

  /** Sense from a local pose: marks flown and seen-free voxels and writes obs and lidar (meters). */
  sense(local: V3, vel: V3, yaw: number, tSec: number): Float32Array {
    if (this.markFlown(local)) {
      this.visitedCount += 1;
      this.lastNewVoxelSec = tSec;
      this.stall = 0;
    } else {
      this.stall += 1;
    }
    const state = makeDroneState([local[0], local[1], local[2]], yaw);
    state.vel = [vel[0], vel[1], vel[2]];
    sense(
      {
        world: this.physics,
        state,
        spec: this.grid.spec,
        visited: this.visited,
        yawRate: this.yawRate,
        floor: floorOf(this.grid, local[2]),
        stepsLeft: Math.max(0, this.maxSteps - this.steps),
        maxSteps: this.maxSteps,
        lighting: this.lighting,
        noise: this.noise,
        prevVel: this.prevVel,
        seen: this.seenMap.data,
        markSeen: this.seenMap,
        nav: this.nav,
        maps: this.maps,
        stall: this.stall,
      },
      this.obs,
    );
    this.prevVel = [vel[0], vel[1], vel[2]];
    this.steps += 1;
    for (let i = 0; i < LIDAR_H; i++) this.lidar[i] = this.obs[OBS.lidar + i]! * LIDAR_MAX;
    this.rangeHook?.(this.lidar, tSec);
    this.velWorld[0] = vel[0];
    this.velWorld[1] = vel[1];
    const f = floorOf(this.grid, local[2]);
    this.occ.mark(f, local[0], local[1], yaw, this.lidar, local[2]);
    this.look(local, yaw);
    // Range finders: open air well past the ceiling or the floor of this storey means a shaft.
    const zf = local[2] - f * this.grid.H;
    const vx = Math.floor(local[0] / VOXEL);
    const vy = Math.floor(local[1] / VOXEL);
    if (vx >= 0 && vy >= 0 && vx < this.vw && vy < this.vd && !this.shaftRejected[f * this.vw * this.vd + vy * this.vw + vx]) {
      const k = f * this.vw * this.vd + vy * this.vw + vx;
      const note = (obs: Float64Array) => {
        obs[k * 3] += local[0];
        obs[k * 3 + 1] += local[1];
        obs[k * 3 + 2] += 1;
      };
      if (f < this.grid.floors - 1 && this.seesThrough('up', zf)) {
        this.shaftUp[k] = 1;
        note(this.shaftObsUp);
      }
      if (f > 0 && this.seesThrough('down', zf)) {
        this.shaftDown[k] = 1;
        note(this.shaftObsDown);
      }
    }
    return this.obs;
  }

  /**
   * Mark known-free voxels inside the camera's trusted cone and range, with a
   * clear line of sight through the drone's own map, as looked at. This is
   * the sensor-map version of IndoorVisibility: it never reads the floor plan.
   */
  look(local: V3, yaw: number): number {
    const f = floorOf(this.grid, local[2]);
    const { vw, vd } = this;
    const base = f * vw * vd;
    const r = this.lookRange;
    const x0 = Math.max(0, Math.floor((local[0] - r) / VOXEL));
    const x1 = Math.min(vw - 1, Math.floor((local[0] + r) / VOXEL));
    const y0 = Math.max(0, Math.floor((local[1] - r) / VOXEL));
    const y1 = Math.min(vd - 1, Math.floor((local[1] + r) / VOXEL));
    let fresh = 0;
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const k = base + y * vw + x;
        if (this.looked[k]) continue;
        if (!(this.visited[k] || (this.seen[k] && !this.occupied[k]))) continue;
        const cx = (x + 0.5) * VOXEL;
        const cy = (y + 0.5) * VOXEL;
        const d = Math.hypot(cx - local[0], cy - local[1]);
        if (d > r) continue;
        if (this.lookCone !== null) {
          if (coneAngle(local, yaw, cx, cy, f * this.grid.H + LOOK_POINT_Z) > this.lookCone) continue;
        } else if (d > 0.7 && Math.abs(wrapAngle(Math.atan2(cy - local[1], cx - local[0]) - yaw)) > this.lookHalf) continue;
        if (!sensorLos(this, f, local[0], local[1], cx, cy)) continue;
        this.looked[k] = 1;
        fresh += 1;
      }
    }
    return fresh;
  }

  /** Drop the shaft voxels within a metre of `at` on `floor`: the drone tried them and could not get through. */
  forgetShaft(floor: number, dir: 'up' | 'down', at: [number, number]): void {
    const map = dir === 'up' ? this.shaftUp : this.shaftDown;
    const base = floor * this.vw * this.vd;
    for (let k = 0; k < this.vw * this.vd; k++) {
      if (Math.hypot(((k % this.vw) + 0.5) * VOXEL - at[0], (Math.floor(k / this.vw) + 0.5) * VOXEL - at[1]) >= 1) continue;
      map[base + k] = 0;
      this.shaftRejected[base + k] = 1;
    }
  }

  /**
   * True when the last reading of the up (down) range finder, taken `zf` above
   * the current floor, saw well past the ceiling (floor): open air to the next storey.
   */
  seesThrough(dir: 'up' | 'down', zf: number): boolean {
    return dir === 'up' ? this.obs[OBS.ceil]! * FLOOR_HEIGHT > this.grid.H - zf + 0.8 : this.obs[OBS.alt]! * FLOOR_HEIGHT > zf + 0.8;
  }

  /**
   * Centre of the shaft voxels (found by the range finders) nearest `from` on
   * `floor`, leading up or down, or null when none is known. The centre is
   * the mean of the positions the 8-connected group was seen through from, so
   * the climb stays inside the column.
   */
  shaftPoint(floor: number, dir: 'up' | 'down', from: V3): [number, number] | null {
    const map = dir === 'up' ? this.shaftUp : this.shaftDown;
    const obs = dir === 'up' ? this.shaftObsUp : this.shaftObsDown;
    const { vw, vd } = this;
    const base = floor * vw * vd;
    let best = -1;
    let bestD = Infinity;
    for (let k = 0; k < vw * vd; k++) {
      if (!map[base + k]) continue;
      const d = Math.hypot(((k % vw) + 0.5) * VOXEL - from[0], (Math.floor(k / vw) + 0.5) * VOXEL - from[1]);
      if (d < bestD) {
        bestD = d;
        best = k;
      }
    }
    if (best < 0) return null;
    const group = [best];
    const seen = new Set(group);
    let sx = 0;
    let sy = 0;
    let n = 0;
    for (let i = 0; i < group.length; i++) {
      const k = group[i]!;
      const x = k % vw;
      const y = (k - x) / vw;
      sx += obs[(base + k) * 3]!;
      sy += obs[(base + k) * 3 + 1]!;
      n += obs[(base + k) * 3 + 2]!;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
          const nk = ny * vw + nx;
          if (map[base + nk] && !seen.has(nk)) {
            seen.add(nk);
            group.push(nk);
          }
        }
      }
    }
    return [sx / n, sy / n];
  }

  /** Looked-at voxels over known-free voxels: the drone's own estimate of search coverage. */
  lookedCoverage(): number {
    let free = 0;
    let seen = 0;
    for (let i = 0; i < this.visited.length; i++) {
      if (!(this.visited[i] || (this.seen[i] && !this.occupied[i]))) continue;
      free += 1;
      if (this.looked[i]) seen += 1;
    }
    return free ? seen / free : 0;
  }

  /**
   * Per floor: the known-free voxels connected to where the drone flew, and how many of them the look
   * estimate covers. Confirm-clear reads it. Free voxels the drone can't get to through its own map
   * (slivers the lidar painted through a gap in a wall) are left out, like the space it never saw:
   * whether every reachable space was found is the explorer's job, which ends only with no frontier left.
   */
  lookStats(): { free: number; looked: number }[] {
    const { vw, vd } = this;
    const per = vw * vd;
    // A flown voxel the lidar later marked as wall (the pose estimate drifted) seeds nothing, or the fill would leak through that wall.
    const free = (k: number) => !this.occupied[k] && (this.visited[k] || this.seen[k]);
    const reach = new Uint8Array(this.visited.length);
    const queue: number[] = [];
    for (let k = 0; k < this.visited.length; k++) {
      if (this.visited[k] && !this.occupied[k]) {
        reach[k] = 1;
        queue.push(k);
      }
    }
    for (let i = 0; i < queue.length; i++) {
      const k = queue[i]!;
      const x = k % vw;
      const y = Math.floor(k / vw) % vd;
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]] as const) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
        const n = k + dy * vw + dx;
        if (reach[n] || !free(n)) continue;
        reach[n] = 1;
        queue.push(n);
      }
    }
    const out = Array.from({ length: this.grid.floors }, () => ({ free: 0, looked: 0 }));
    for (let k = 0; k < reach.length; k++) {
      if (!reach[k]) continue;
      const f = out[Math.floor(k / per)]!;
      f.free += 1;
      if (this.looked[k]) f.looked += 1;
    }
    return out;
  }

  /** True when the last observation's frontier search found nothing to fly to on this floor. */
  get frontierLeft(): boolean {
    return this.obs[OBS.hasFrontier]! > 0.5;
  }

  /** Flown voxels over the voxels this drone has mapped as free (its own estimate of how done it is). */
  mappedCoverage(): number {
    let free = 0;
    let flown = 0;
    for (let i = 0; i < this.visited.length; i++) {
      if (this.visited[i] || (this.seen[i] && !this.occupied[i])) free += 1;
      if (this.visited[i]) flown += 1;
    }
    return free ? flown / free : 0;
  }

  /** Visited voxels over reachable voxels (ground truth; for reports only, never for control). */
  coverage(): number {
    return this.grid.spec.reachableVoxels > 0 ? this.visitedCount / this.grid.spec.reachableVoxels : 0;
  }

  frame(local: V3, yaw: number): SensorFrame {
    return {
      seen: this.seen,
      visited: this.visited,
      occupied: this.occupied,
      vw: this.vw,
      vd: this.vd,
      floors: this.grid.floors,
      floorHeight: this.grid.H,
      pos: [local[0], local[1], local[2]],
      yaw,
      lidar: this.lidar,
      alt: this.obs[OBS.alt]! * FLOOR_HEIGHT,
      vel: this.velWorld,
    };
  }

  /**
   * The lidar velocity filter for this airframe on the latest reading: the
   * braking filter when the airframe has a stopping model (the quad), the
   * kinematic closing-speed rule otherwise.
   */
  filter(vx: number, vy: number, yaw: number, margin = SAFETY_MARGIN): [number, number] {
    return this.braking
      ? brakingFilter(vx, vy, yaw, this.lidar, this.velWorld, this.braking, Math.max(margin, BRAKING_MARGIN))
      : filterVelocity(vx, vy, yaw, this.lidar, margin, INDOOR_FILTER_HORIZON);
  }
}

// ------------------------------------------------------------- navigation

/** Path and viewpoint queries the runner makes inside a building. */
export interface IndoorNav {
  /** Local waypoints from one local point to another, or null when unreachable. */
  path(from: V3, to: V3): V3[] | null;
  /** A hover point is safe and sees the target point. */
  canView(p: V3, target: V3): boolean;
}

/** Floor-plan navigation: the reference oracle. It knows every wall before the drone sees it. */
export function oracleNav(grid: IndoorGrid): IndoorNav {
  return {
    path: (from, to) => planIndoorPath(grid, from, to),
    canView: (p, target) => {
      const f = floorOf(grid, p[2]);
      const c = nearestNav(grid, p);
      if (!c || Math.hypot((c.x + 0.5) * grid.cs - p[0], (c.z + 0.5) * grid.cs - p[1]) > 0.75) return false;
      if (wallClearance(grid, f, p[0], p[1]) < PATH_CLEARANCE) return false;
      return gridLos(grid, f, p[0], p[1], target[0], target[1]);
    },
  };
}

/** Navigation on the drone's own maps: only voxels it has seen free or flown. */
export function sensorNav(grid: IndoorGrid, sensing: IndoorSensing): IndoorNav {
  return {
    path: (from, to) => {
      const f = floorOf(grid, from[2]);
      if (floorOf(grid, to[2]) !== f) return null;
      const xy = sensorPath(sensing, f, [from[0], from[1]], [to[0], to[1]]);
      if (!xy) return null;
      const z = f * grid.H + INDOOR_FLY_Z;
      return xy.map(([x, y]) => [x, y, z] as V3);
    },
    canView: (p, target) => {
      const f = floorOf(grid, p[2]);
      if (sensorClearance(sensing, f, p[0], p[1]) < VOXEL) return false;
      // The target itself may sit in a voxel the lidar never painted free (a person on the floor).
      return sensorLos(sensing, f, p[0], p[1], target[0], target[1], VOXEL);
    },
  };
}

// ------------------------------------------------------------- explorers

export interface PolicyCommand {
  /** Local-frame velocity setpoint (axes match the world frame). */
  vel: V3;
  yawRate: number;
}

/**
 * Safety-filter horizon indoors, seconds. The mission vehicle brakes at
 * 1.5 m/s² with a jerk limit (INDOOR_LIMITS), so it needs longer to stop than
 * the coverage env's first-order lag.
 */
export const INDOOR_FILTER_HORIZON = 0.7;

/** Steps in a row without a frontier before an explorer reports the building done. */
const DONE_STEPS = 20;

/**
 * Trained-policy explorer. Takes the policy's mean action on the shared
 * sensing's observation, caps speed for indoor flight, and runs it through
 * the sensor-only guard (sim/sensorPlanner.ts guardAction): an altitude nudge
 * and a lidar filter that cuts velocity only at imminent contact.
 */
export class PolicyExplorer {
  private readonly scaled = new Float32Array(4);
  private readonly guarded = new Float32Array(4);
  private idle = 0;

  constructor(
    readonly grid: IndoorGrid,
    readonly policy: ActingPolicy,
    readonly sensing: IndoorSensing,
    readonly speedCap = 1,
  ) {}

  /** No frontier left in the drone's own map for DONE_STEPS steps. */
  get exhausted(): boolean {
    return this.idle >= DONE_STEPS;
  }

  /** One policy step from a local pose. */
  act(local: V3, vel: V3, yaw: number, tSec: number): PolicyCommand {
    const obs = this.sensing.sense(local, vel, yaw, tSec);
    this.idle = this.sensing.frontierLeft ? 0 : this.idle + 1;
    const action = this.policy.actWith(obs, 1, this.sensing.rng, true).action;
    const f = floorOf(this.grid, local[2]);
    const brake = this.sensing.braking ? { model: this.sensing.braking, vel: [vel[0], vel[1]] as const } : undefined;
    guardAction(action, local[2] - f * this.grid.H, yaw, this.sensing.lidar, this.guarded, INDOOR_FILTER_HORIZON, brake);
    scaleAction(this.guarded, this.scaled);
    const k = this.speedCap / MAX_SPEED_H;
    const bx = this.scaled[0]! * k;
    const by = this.scaled[1]! * k;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const vx = cy * bx - sy * by;
    const vy = sy * bx + cy * by;
    const vz = clamp(this.scaled[2]!, -0.6, 0.6);
    // Hold a sane height band on the current floor.
    const zLo = f * this.grid.H + 0.5;
    const zHi = (f + 1) * this.grid.H - 0.5;
    const vzSafe = local[2] < zLo ? Math.max(vz, 0.3) : local[2] > zHi ? Math.min(vz, -0.3) : vz;
    this.sensing.yawRate = this.scaled[3]!;
    return { vel: [vx, vy, vzSafe], yawRate: this.sensing.yawRate };
  }

  /** Visited voxels over reachable voxels. */
  coverage(): number {
    return this.sensing.coverage();
  }
}

/** A viewpoint is worth flying to when it would show at least this many unlooked voxels. */
const MIN_LOOK_GAIN = 4;

/**
 * Classical sensor-only explorer (sim/sensorPlanner.ts SensorPlanner) on the
 * shared sensing. It is the mission's indoor fallback and the baseline the
 * policy is measured against. Per floor: sweep (fly every known-free voxel,
 * facing the direction of travel), then look (fly to the viewpoint that would
 * show the most voxels the camera has not looked at yet, per meter of travel,
 * and turn a full circle there) until no viewpoint is worth it. Then it takes
 * a shaft its up and down range finders found to a floor it has not searched,
 * and finally returns to the floor it entered on.
 */
export class SensorExplorer {
  private readonly planner: SensorPlanner;
  private idle = 0;
  private phase: 'sweep' | 'look' | 'shaft' | 'done';
  private route: V3[] = [];
  private spin = 0;
  private entryFloor = -1;
  private readonly floorsDone = new Set<number>();
  /** Shaft leg: hover point (local xy) and the floor to reach. */
  private shaft: { x: number; y: number; to: number } | null = null;
  /** Centring on the shaft before climbing: the axis being probed, its direction, and the edges found. */
  private centring: { axis: 0 | 1; dir: 1 | -1; open: boolean; from: number; ends: number[]; ticks: number } | null = null;
  private centred = false;
  /** Progress watch on the climb: closest the drone has come to the target height, and ticks since. */
  private climbBest = Infinity;
  private followBest = Infinity;
  private followStall = 0;
  private climbStall = 0;

  constructor(
    readonly grid: IndoorGrid,
    readonly sensing: IndoorSensing,
    readonly speedCap = 1,
    start: 'sweep' | 'look' = 'sweep',
  ) {
    this.planner = new SensorPlanner({
      speed: speedCap,
      flyHeight: INDOOR_FLY_Z,
      horizon: INDOOR_FILTER_HORIZON,
      braking: sensing.braking ?? undefined,
    });
    this.phase = start;
  }

  get exhausted(): boolean {
    return this.phase === 'done';
  }

  get phaseName(): 'sweep' | 'look' | 'shaft' | 'done' {
    return this.phase;
  }

  /**
   * Out of time: stop searching and get back to the entry floor (the way out),
   * through the shaft if the drone is on another floor or climbing.
   */
  wrapUp(local: V3): void {
    if (this.phase === 'done') return;
    for (let f = 0; f < this.grid.floors; f++) this.floorsDone.add(f);
    const f = floorOf(this.grid, local[2]);
    if (this.phase === 'shaft' && this.shaft) {
      // Already in the column: climb or descend straight to the entry floor.
      if (!this.route.length) this.shaft.to = this.entryFloor;
      else if (f === this.entryFloor) this.phase = 'done';
      return;
    }
    if (f === this.entryFloor) {
      this.phase = 'done';
      return;
    }
    this.route = [];
    this.spin = 0;
    this.phase = 'look';
    this.floorDone(local, 0);
  }

  act(local: V3, vel: V3, yaw: number, tSec: number): PolicyCommand {
    this.sensing.sense(local, vel, yaw, tSec);
    if (this.entryFloor < 0) this.entryFloor = floorOf(this.grid, local[2]);
    if (this.phase === 'sweep') {
      const cmd = this.planner.act(this.sensing.frame(local, yaw));
      this.idle = cmd.exploring ? 0 : this.idle + 1;
      if (this.idle >= DONE_STEPS) {
        this.idle = 0;
        this.phase = 'look';
      } else {
        this.sensing.yawRate = cmd.yawRate;
        return { vel: [cmd.vx, cmd.vy, cmd.vz], yawRate: cmd.yawRate };
      }
    }
    if (this.phase === 'look') return this.lookStep(local, yaw);
    if (this.phase === 'shaft') return this.shaftStep(local, yaw);
    return { vel: [0, 0, 0], yawRate: 0 };
  }

  private hold(local: V3): number {
    const f = floorOf(this.grid, local[2]);
    const vz = clamp((f * this.grid.H + INDOOR_FLY_Z - local[2]) * 2, -0.6, 0.6);
    return vz < 0 && !descentClear(this.sensing.lidar) ? 0 : vz;
  }

  /** Follow this.route on the current floor; true once the last point is reached. */
  private follow(local: V3, yaw: number, out: { cmd: PolicyCommand | null }): boolean {
    const target = this.route[0]!;
    const dx = target[0] - local[0];
    const dy = target[1] - local[1];
    const d = Math.hypot(dx, dy);
    // A waypoint the shield will not let the drone reach (it hugs a wall the map put a hair too far)
    // is skipped after a while, as the runner's routed moves do.
    if (d < this.followBest - 0.03) {
      this.followBest = d;
      this.followStall = 0;
    } else if ((this.followStall += 1) > FOLLOW_STALL_TICKS && this.route.length > 1) {
      this.route.shift();
      this.followBest = Infinity;
      this.followStall = 0;
      out.cmd = { vel: [0, 0, this.hold(local)], yawRate: 0 };
      return false;
    }
    if (d < (this.route.length === 1 ? 0.12 : 0.2)) {
      this.route.shift();
      this.followBest = Infinity;
      this.followStall = 0;
      out.cmd = { vel: [0, 0, this.hold(local)], yawRate: 0 };
      return this.route.length === 0;
    }
    const sp = Math.min(this.speedCap, 0.3 + d);
    const [vx, vy] = this.sensing.filter((dx / d) * sp, (dy / d) * sp, yaw);
    const yawRate = clamp(wrapAngle(Math.atan2(dy, dx) - yaw) * 2, -MAX_YAW_RATE_INDOOR, MAX_YAW_RATE_INDOOR);
    this.sensing.yawRate = yawRate;
    out.cmd = { vel: [vx, vy, this.hold(local)], yawRate };
    return false;
  }

  private lookStep(local: V3, yaw: number): PolicyCommand {
    const f = floorOf(this.grid, local[2]);
    if (this.spin > 0) {
      this.spin -= 1;
      this.sensing.yawRate = MAX_YAW_RATE_INDOOR;
      return { vel: [0, 0, this.hold(local)], yawRate: MAX_YAW_RATE_INDOOR };
    }
    if (!this.route.length) {
      const goal = bestViewpoint(this.sensing, f, local);
      const path = goal ? sensorPath(this.sensing, f, [local[0], local[1]], [goal[0], goal[1]]) : null;
      if (!path) return this.floorDone(local, yaw);
      this.route = path.map(([x, y]) => [x, y, f * this.grid.H + INDOOR_FLY_Z] as V3);
    }
    const out: { cmd: PolicyCommand | null } = { cmd: null };
    if (this.follow(local, yaw, out)) this.spin = Math.ceil((Math.PI * 2) / MAX_YAW_RATE_INDOOR / 0.05) + 2;
    return out.cmd!;
  }

  /** This floor is searched: pick a shaft to an unsearched floor, or head back to the entry floor, or stop. */
  private floorDone(local: V3, yaw: number): PolicyCommand {
    const f = floorOf(this.grid, local[2]);
    this.floorsDone.add(f);
    const want: number[] = [];
    for (const g of [f + 1, f - 1]) if (g >= 0 && g < this.grid.floors && !this.floorsDone.has(g)) want.push(g);
    if (!want.length && f !== this.entryFloor) want.push(f > this.entryFloor ? f - 1 : f + 1);
    for (const g of want) {
      const at = this.sensing.shaftPoint(f, g > f ? 'up' : 'down', local);
      if (!at) continue;
      const path = sensorPath(this.sensing, f, [local[0], local[1]], at);
      if (!path) continue;
      this.route = path.map(([x, y]) => [x, y, f * this.grid.H + INDOOR_FLY_Z] as V3);
      this.shaft = { x: at[0], y: at[1], to: g };
      this.centred = false;
      this.centring = null;
      this.climbBest = Infinity;
      this.climbStall = 0;
      this.phase = 'shaft';
      return this.shaftStep(local, yaw);
    }
    this.phase = 'done';
    return { vel: [0, 0, 0], yawRate: 0 };
  }

  private shaftStep(local: V3, yaw: number): PolicyCommand {
    const shaft = this.shaft!;
    if (this.route.length) {
      const out: { cmd: PolicyCommand | null } = { cmd: null };
      this.follow(local, yaw, out);
      return out.cmd!;
    }
    if (!this.centred) return this.centreStep(local, yaw);
    // In the column: hold xy on the shaft centre and climb or descend to the next floor's flight height.
    const zTarget = shaft.to * this.grid.H + INDOOR_FLY_Z;
    const ez = zTarget - local[2];
    if (Math.abs(ez) < 0.08) {
      this.shaft = null;
      this.planner.reset();
      this.phase = this.floorsDone.has(shaft.to) ? 'look' : 'sweep';
      if (this.phase === 'look' && shaft.to === this.entryFloor) this.phase = 'done';
      return { vel: [0, 0, 0], yawRate: 0 };
    }
    // A climb that stops gaining height is not a shaft (open air over a doorway, say): forget it and
    // pick again. Only before the drone has left its floor; once between floors it keeps going.
    if (Math.abs(ez) < this.climbBest - 0.02) {
      this.climbBest = Math.abs(ez);
      this.climbStall = 0;
    } else if ((this.climbStall += 1) > CLIMB_STALL_TICKS && Math.abs(ez) > this.grid.H - 0.5) {
      const f = floorOf(this.grid, local[2]);
      this.sensing.forgetShaft(f, shaft.to > f ? 'up' : 'down', [shaft.x, shaft.y]);
      this.shaft = null;
      this.phase = 'look';
      return this.floorDone(local, yaw);
    }
    const vx = clamp((shaft.x - local[0]) * 2, -0.3, 0.3);
    const vy = clamp((shaft.y - local[1]) * 2, -0.3, 0.3);
    return { vel: [vx, vy, clamp(ez * 1.5, -0.5, 0.5)], yawRate: 0 };
  }

  /**
   * Before a climb or descent, find the hole's edges with the range finder:
   * fly slowly along each building axis until it stops seeing through, both
   * ways, and centre between the edges. The shaft point alone is only known
   * to be somewhere over the hole, possibly at its rim, and the airframe is
   * wider than a point.
   */
  private centreStep(local: V3, yaw: number): PolicyCommand {
    const shaft = this.shaft!;
    const f = floorOf(this.grid, local[2]);
    const open = this.sensing.seesThrough(shaft.to > f ? 'up' : 'down', local[2] - f * this.grid.H);
    const at = [shaft.x, shaft.y];
    const c = (this.centring ??= { axis: 0, dir: 1, open: false, from: at[0]!, ends: [], ticks: 0 });
    const p = local[c.axis]!;
    c.open ||= open;
    c.ticks += 1;
    // An edge: the reading closed after being open, or the probe ran out of room or time (a wall the
    // shield will not let it approach).
    if ((c.open && !open) || Math.abs(p - c.from) > CENTRE_PROBE_M || c.ticks > CENTRE_PROBE_TICKS) {
      c.ends.push(p);
      c.open = false;
      c.ticks = 0;
      if (c.dir === 1) c.dir = -1;
      else {
        at[c.axis] = (c.ends[0]! + c.ends[1]!) / 2;
        shaft.x = at[0]!;
        shaft.y = at[1]!;
        if (c.axis === 1) {
          this.centred = true;
          this.centring = null;
          return { vel: [0, 0, this.hold(local)], yawRate: 0 };
        }
        this.centring = { axis: 1, dir: 1, open: false, from: at[1]!, ends: [], ticks: 0 };
      }
    }
    const k = this.centring!;
    const v: [number, number] = [clamp((at[0]! - local[0]) * 2, -0.3, 0.3), clamp((at[1]! - local[1]) * 2, -0.3, 0.3)];
    v[k.axis] = k.dir * CENTRE_PROBE_SPEED;
    const [vx, vy] = this.sensing.filter(v[0], v[1], yaw);
    return { vel: [vx, vy, this.hold(local)], yawRate: 0 };
  }
}

/** Speed and reach of the range-finder probe that centres the drone on a shaft, m/s and m. */
const CENTRE_PROBE_SPEED = 0.3;
const CENTRE_PROBE_M = 0.9;
const CENTRE_PROBE_TICKS = 120;
/** A climb or descent that gains no height for this many ticks gives the shaft up. */
const CLIMB_STALL_TICKS = 60;
/** Ticks without progress toward a route waypoint before it is skipped (not the last one). */
const FOLLOW_STALL_TICKS = 60;

/** Yaw rate for turning in place indoors, rad/s (the vehicle's indoor limit). */
const MAX_YAW_RATE_INDOOR = Math.PI / 2;

/**
 * The reachable known-free voxel that would show the most unlooked voxels
 * per meter of travel (gain / (1 + 0.3 * path length)), judged on the
 * drone's own map. Null when no voxel would show MIN_LOOK_GAIN.
 */
export function bestViewpoint(sensing: IndoorSensing, floor: number, from: V3): [number, number] | null {
  const { vw, vd } = sensing;
  const base = floor * vw * vd;
  const open = (x: number, y: number) => {
    if (x < 0 || y < 0 || x >= vw || y >= vd) return false;
    const k = base + y * vw + x;
    return sensing.visited[k] === 1 || (sensing.seen[k] === 1 && sensing.occupied[k] !== 1);
  };
  const unlooked: number[] = [];
  for (let y = 0; y < vd; y++) for (let x = 0; x < vw; x++) if (open(x, y) && !sensing.looked[base + y * vw + x]) unlooked.push(y * vw + x);
  if (unlooked.length < MIN_LOOK_GAIN) return null;
  // BFS path lengths from the drone over known-free voxels.
  const dist = new Float32Array(vw * vd).fill(Infinity);
  const sx = clamp(Math.floor(from[0] / VOXEL), 0, vw - 1);
  const sy = clamp(Math.floor(from[1] / VOXEL), 0, vd - 1);
  const queue = [sy * vw + sx];
  dist[sy * vw + sx] = 0;
  for (let h = 0; h < queue.length; h++) {
    const k = queue[h]!;
    const x = k % vw;
    const y = (k - x) / vw;
    for (const [dx, dy] of [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
    ] as const) {
      const nx = x + dx;
      const ny = y + dy;
      if (!open(nx, ny)) continue;
      const nk = ny * vw + nx;
      if (dist[nk]! <= dist[k]! + 1) continue;
      dist[nk] = dist[k]! + 1;
      queue.push(nk);
    }
  }
  const range = sensing.lookRange - 0.5;
  let best: [number, number] | null = null;
  let bestScore = 0;
  for (const k of queue) {
    const x = k % vw;
    const y = (k - x) / vw;
    // Viewpoints keep a voxel of known free air around them.
    if (sensorClearance(sensing, floor, (x + 0.5) * VOXEL, (y + 0.5) * VOXEL) < 2 * VOXEL) continue;
    const px = (x + 0.5) * VOXEL;
    const py = (y + 0.5) * VOXEL;
    let gain = 0;
    for (const u of unlooked) {
      const ux = ((u % vw) + 0.5) * VOXEL;
      const uy = (Math.floor(u / vw) + 0.5) * VOXEL;
      const dd = (ux - px) ** 2 + (uy - py) ** 2;
      if (dd > range * range || dd < sensing.lookMin * sensing.lookMin) continue;
      if (sensorLos(sensing, floor, px, py, ux, uy)) gain += 1;
    }
    if (gain < MIN_LOOK_GAIN) continue;
    const score = gain / (1 + 0.3 * dist[k]! * VOXEL);
    if (score > bestScore) {
      bestScore = score;
      best = [px, py];
    }
  }
  return best;
}
