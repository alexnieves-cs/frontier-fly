import { isWalkable } from '../../../gen/validate';
import { indoorCells, type GridCell } from '../../../perception/seenEmpty';
import type { InferPolicy } from '../../../rl/infer';
import { CELL, DRONE_RADIUS, MAX_SPEED_H, OBS_DIM } from '../../../shared/constants';
import { RNG } from '../../../shared/rng';
import type { BuildingSpec, LightingState } from '../../../shared/types';
import { makeDroneState, scaleAction } from '../../../sim/drone';
import { zeroNoise } from '../../../sim/dynamics/dr';
import { SensorNoise } from '../../../sim/dynamics/noise';
import { markVisited, sense } from '../../../sim/sensors';
import type { PhysicsWorld } from '../../../sim/world';
import type { MissionBuilding, V3 } from '../contracts';
import { clamp, wrapAngle } from './geometry';

/**
 * Indoor exploration on the building grid, in the building-local frame
 * (world minus shell.origin). The frontier explorer keeps its own camera
 * visibility estimate over the same cells the seen-empty maps use, picks
 * the reachable cell that would reveal the most unseen cells per meter of
 * travel, flies there on a clearance-checked grid path, and turns a full
 * circle. Shaft cells link floors. The policy explorer drives the trained
 * network from its 306-float observation instead.
 */

/** Flight height above each floor, meters. */
export const INDOOR_FLY_Z = 1.2;
/** Range the visibility estimate trusts, meters. */
export const SEE_RANGE = 6;
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
  if (Math.hypot(first[0] - from[0], first[1] - from[1]) > 0.15 || Math.abs(first[2] - from[2]) > 0.2) out.push(first);
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
    if (Math.hypot(first[0] - local[0], first[1] - local[1]) > 0.15 || Math.abs(first[2] - local[2]) > 0.2) path.push(first);
    path.push(...cellsToPath(this.grid, cells));
    if (!path.length) path.push(first);
    return { path, spin: true, gain: bestGain };
  }
}

// ------------------------------------------------------------- policy

export interface PolicyCommand {
  /** Local-frame velocity setpoint (axes match the world frame). */
  vel: V3;
  yawRate: number;
}

/**
 * Trained-policy explorer. Builds the 306-float observation with sense()
 * on a PhysicsWorld of the building in the local frame and takes the
 * policy's mean action. Speeds are capped for indoor flight, and commands
 * toward the outer wall ring are dropped so the policy cannot leave.
 */
export class PolicyExplorer {
  readonly visited: Uint8Array;
  visitedCount = 0;
  lastNewVoxelSec = 0;
  private readonly obs = new Float32Array(OBS_DIM);
  private readonly scaled = new Float32Array(4);
  private readonly noise: SensorNoise;
  private readonly rng: RNG;
  private prevVel: [number, number, number] = [0, 0, 0];
  private yawRate = 0;
  private steps = 0;
  private readonly maxSteps = 2400;

  constructor(
    readonly grid: IndoorGrid,
    readonly policy: InferPolicy,
    readonly physics: PhysicsWorld,
    readonly lighting: LightingState,
    seed: number,
    readonly speedCap = 1,
  ) {
    const floor0 = grid.spec.floors[0]!;
    this.visited = new Uint8Array(grid.floors * floor0.w * 2 * floor0.d * 2);
    this.rng = new RNG(seed >>> 0, 'indoor-policy');
    this.noise = new SensorNoise(zeroNoise(), this.rng.fork('noise'));
  }

  /** One policy step from a local pose. */
  act(local: V3, vel: V3, yaw: number, tSec: number): PolicyCommand {
    if (markVisited(this.grid.spec, this.visited, local[0], local[1], local[2])) {
      this.visitedCount += 1;
      this.lastNewVoxelSec = tSec;
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
      },
      this.obs,
    );
    this.prevVel = [vel[0], vel[1], vel[2]];
    this.steps += 1;
    const action = this.policy.actWith(this.obs, 1, this.rng, true).action;
    scaleAction(action, this.scaled);
    const k = this.speedCap / MAX_SPEED_H;
    const bx = this.scaled[0]! * k;
    const by = this.scaled[1]! * k;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    let vx = cy * bx - sy * by;
    let vy = sy * bx + cy * by;
    const vz = clamp(this.scaled[2]!, -0.6, 0.6);
    // Keep the policy off the outer wall ring and away from the apertures.
    const ahead = cellAt(this.grid, [local[0] + vx * 0.6, local[1] + vy * 0.6, local[2]]);
    if (!isNav(this.grid, ahead.f, ahead.x, ahead.z)) {
      const ax = cellAt(this.grid, [local[0] + vx * 0.6, local[1], local[2]]);
      const ay = cellAt(this.grid, [local[0], local[1] + vy * 0.6, local[2]]);
      if (!isNav(this.grid, ax.f, ax.x, ax.z)) vx = 0;
      if (!isNav(this.grid, ay.f, ay.x, ay.z)) vy = 0;
    }
    // Hold a sane height band on the current floor.
    const f = floorOf(this.grid, local[2]);
    const zLo = f * this.grid.H + 0.5;
    const zHi = (f + 1) * this.grid.H - 0.5;
    const vzSafe = local[2] < zLo ? Math.max(vz, 0.3) : local[2] > zHi ? Math.min(vz, -0.3) : vz;
    this.yawRate = this.scaled[3]!;
    return { vel: [vx, vy, vzSafe], yawRate: this.yawRate };
  }

  /** Visited voxels over reachable voxels. */
  coverage(): number {
    return this.grid.spec.reachableVoxels > 0 ? this.visitedCount / this.grid.spec.reachableVoxels : 0;
  }
}

