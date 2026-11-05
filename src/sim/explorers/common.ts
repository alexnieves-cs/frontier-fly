/**
 * What the published exploration baselines share, so that they differ only in
 * how they choose where to go: the grid queries over the drone's own maps,
 * Dijkstra with the same cost near unknown space as the sensor-only planner,
 * pure pursuit, the lidar safety filter (the braking filter on the
 * quadrotor), and the planner's altitude hold. Nothing here reads the floor
 * plan; it reads only the sensor frame (sensorPlanner.ts SensorFrame).
 */
import * as C from '../../shared/constants';
import { brakingFilter, descentClear, filterVelocity, SAFETY_HORIZON, SAFETY_MARGIN, type BrakingModel } from '../lidarSafety';
import type { SensorCommand, SensorFrame } from '../sensorPlanner';

// Module-local copies (imported bindings are getter calls under vite-node).
const VOXEL = C.VOXEL;
const MAX_YAW_RATE = C.MAX_YAW_RATE;

/** An exploration strategy over the sensor frame: Climber (sim/climber.ts) can drive any of these. */
export interface Explorer {
  /** Flight height above the floor, meters. */
  readonly flyHeight: number;
  reset(): void;
  act(f: SensorFrame): SensorCommand;
}

export interface ExplorerOptions {
  /** Cruise speed, m/s. */
  speed?: number;
  /** Airframe stopping model: when set, the braking filter replaces the kinematic velocity filter. */
  braking?: BrakingModel;
}

export const DX = [1, -1, 0, 0, 1, 1, -1, -1];
export const DY = [0, 0, 1, -1, 1, -1, 1, -1];
const STEP = [1, 1, 1, 1, Math.SQRT2, Math.SQRT2, Math.SQRT2, Math.SQRT2];

export function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

function wrap(a: number): number {
  return a - Math.round(a / (Math.PI * 2)) * Math.PI * 2;
}

export const cellX = (k: number, vw: number) => ((k % vw) + 0.5) * VOXEL;
export const cellY = (k: number, vw: number) => (Math.floor(k / vw) + 0.5) * VOXEL;

/**
 * The drone's maps on one floor plane, and shortest paths over them. A voxel
 * is open when the drone has flown it, or seen it free and the lidar has not
 * marked it occupied; unknown when it is none of seen, flown or occupied.
 */
export class Grid {
  vw = 0;
  vd = 0;
  plane = 0;
  base = 0;
  floor = 0;
  /** Voxels (0..3) from each open voxel to the nearest one that is not open. */
  clear = new Uint8Array(0);
  dist = new Float64Array(0);
  prev = new Int32Array(0);
  private heap = new Int32Array(0);
  private done = new Uint8Array(0);
  private f!: SensorFrame;

  /** Point the grid at this frame's floor; recomputes clearance. Returns the floor. */
  load(f: SensorFrame): number {
    this.f = f;
    const plane = f.vw * f.vd;
    if (plane !== this.plane) {
      this.plane = plane;
      this.clear = new Uint8Array(plane);
      this.dist = new Float64Array(plane);
      this.prev = new Int32Array(plane);
      this.heap = new Int32Array(plane * 8 + 16);
      this.done = new Uint8Array(plane);
    }
    this.vw = f.vw;
    this.vd = f.vd;
    this.floor = clamp(Math.floor(f.pos[2] / f.floorHeight), 0, f.floors - 1);
    this.base = this.floor * plane;
    this.clearance();
    return this.floor;
  }

  open(k: number): boolean {
    const f = this.f;
    const i = this.base + k;
    return f.visited[i] === 1 || (f.seen[i] === 1 && !(f.occupied && f.occupied[i] === 1));
  }

  unknown(k: number): boolean {
    const f = this.f;
    const i = this.base + k;
    return f.seen[i] !== 1 && f.visited[i] !== 1 && !(f.occupied && f.occupied[i] === 1);
  }

  occupied(k: number): boolean {
    const o = this.f.occupied;
    return !!o && o[this.base + k] === 1;
  }

  flown(k: number): boolean {
    return this.f.visited[this.base + k] === 1;
  }

  /** The voxel under (x, y), clamped to the grid. */
  cellAt(x: number, y: number): number {
    return clamp(Math.floor(y / VOXEL), 0, this.vd - 1) * this.vw + clamp(Math.floor(x / VOXEL), 0, this.vw - 1);
  }

  /** The voxel under (x, y), or -1 outside the grid. */
  cellIn(x: number, y: number): number {
    const cx = Math.floor(x / VOXEL);
    const cy = Math.floor(y / VOXEL);
    return cx < 0 || cy < 0 || cx >= this.vw || cy >= this.vd ? -1 : cy * this.vw + cx;
  }

  private clearance(): void {
    const { vw, vd } = this;
    const c = this.clear;
    for (let k = 0; k < this.plane; k++) c[k] = this.open(k) ? 3 : 0;
    for (let level = 1; level <= 2; level++) {
      for (let y = 0; y < vd; y++) {
        for (let x = 0; x < vw; x++) {
          const k = y * vw + x;
          if (c[k]! <= level) continue;
          let near = x === 0 || y === 0 || x === vw - 1 || y === vd - 1;
          for (let d = 0; d < 8 && !near; d++) if (c[(y + DY[d]!) * vw + x + DX[d]!]! === level - 1) near = true;
          if (near) c[k] = level;
        }
      }
    }
  }

  /**
   * Dijkstra over open voxels from `start`, 8-connected without cutting
   * corners, with the sensor-only planner's extra cost next to unknown or
   * occupied space. Fills `dist` (voxel steps, Infinity when unreachable) and `prev`.
   */
  dijkstra(start: number): void {
    const { vw, vd, dist, prev, heap, done, clear } = this;
    dist.fill(Infinity);
    prev.fill(-1);
    done.fill(0);
    dist[start] = 0;
    let size = 0;
    const push = (k: number) => {
      let i = size++;
      heap[i] = k;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (dist[heap[p]!]! <= dist[heap[i]!]!) break;
        const t = heap[p]!;
        heap[p] = heap[i]!;
        heap[i] = t;
        i = p;
      }
    };
    const pop = (): number => {
      const top = heap[0]!;
      heap[0] = heap[--size]!;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < size && dist[heap[l]!]! < dist[heap[m]!]!) m = l;
        if (r < size && dist[heap[r]!]! < dist[heap[m]!]!) m = r;
        if (m === i) break;
        const t = heap[m]!;
        heap[m] = heap[i]!;
        heap[i] = t;
        i = m;
      }
      return top;
    };
    push(start);
    while (size > 0) {
      const k = pop();
      if (done[k]) continue;
      done[k] = 1;
      const x = k % vw;
      const y = (k - x) / vw;
      for (let d = 0; d < 8; d++) {
        const nx = x + DX[d]!;
        const ny = y + DY[d]!;
        if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
        const nk = ny * vw + nx;
        if (!this.open(nk)) continue;
        if (d >= 4 && (!this.open(y * vw + nx) || !this.open(ny * vw + x))) continue;
        const cl = clear[nk]!;
        const nd = dist[k]! + STEP[d]! * (1 + (cl <= 1 ? 2.5 : cl === 2 ? 0.6 : 0));
        if (nd < dist[nk]!) {
          dist[nk] = nd;
          prev[nk] = k;
          if (size < heap.length) push(nk);
        }
      }
    }
  }

  /** Voxels from the last Dijkstra start (excluded) to `goal`. */
  pathTo(goal: number): number[] {
    const path: number[] = [];
    for (let k = goal; k >= 0 && this.prev[k]! >= 0; k = this.prev[k]!) path.push(k);
    path.reverse();
    return path;
  }

  /** Every sample of the segment lies on open voxels; `margin` (0..3) also asks for that much clearance before the end. */
  lineClear(ax: number, ay: number, bx: number, by: number, margin = 2): boolean {
    const l = Math.hypot(bx - ax, by - ay);
    const steps = Math.max(1, Math.ceil(l / (VOXEL * 0.4)));
    for (let i = 1; i <= steps; i++) {
      const t = i / steps;
      const k = this.cellIn(ax + (bx - ax) * t, ay + (by - ay) * t);
      if (k < 0 || !this.open(k)) return false;
      if (i < steps && this.clear[k]! < margin) return false;
    }
    return true;
  }
}

/**
 * Low-level flight shared by the baselines: fly toward a point or along a
 * voxel path at cruise speed through the lidar filter, face the direction of
 * travel, and hold the planner's flight height.
 */
export class Follower {
  readonly speed: number;
  readonly braking: BrakingModel | null;
  readonly flyHeight = 1.2;

  constructor(opts: ExplorerOptions) {
    this.speed = opts.speed ?? 2;
    this.braking = opts.braking ?? null;
  }

  private vz(f: SensorFrame, floor: number): number {
    let vz = clamp((floor * f.floorHeight + this.flyHeight - f.pos[2]) * 2, -0.6, 0.6);
    // Something under the drone (a table, a bed): climb over it rather than settle onto it.
    if (vz < 0 && !descentClear(f.lidar)) vz = 0;
    if (f.alt < 0.55) vz = Math.max(vz, 0.4);
    return vz;
  }

  /** Turn in place: nothing left to fly to (exploring false once the strategy is done). */
  hover(f: SensorFrame, floor: number, exploring: boolean): SensorCommand {
    return { vx: 0, vy: 0, vz: this.vz(f, floor), yawRate: 0.6, exploring };
  }

  /**
   * Fly toward (tx, ty) at `speed` (default cruise), through the lidar filter.
   */
  toward(f: SensorFrame, floor: number, tx: number, ty: number, speed = this.speed): SensorCommand {
    const ex = tx - f.pos[0];
    const ey = ty - f.pos[1];
    const el = Math.hypot(ex, ey);
    let vx = el > 1e-6 ? (ex / el) * speed : 0;
    let vy = el > 1e-6 ? (ey / el) * speed : 0;
    [vx, vy] = this.braking ? brakingFilter(vx, vy, f.yaw, f.lidar, f.vel, this.braking) : filterVelocity(vx, vy, f.yaw, f.lidar, SAFETY_MARGIN, SAFETY_HORIZON);
    const yawRate = el > 0.2 ? clamp(wrap(Math.atan2(ey, ex) - f.yaw) * 2, -MAX_YAW_RATE, MAX_YAW_RATE) : 0;
    return { vx, vy, vz: this.vz(f, floor), yawRate, exploring: true };
  }

  /** Pure pursuit along a voxel path: the farthest path voxel within 2.5 m in clear line of sight. */
  along(f: SensorFrame, g: Grid, path: readonly number[], speed = this.speed): SensorCommand {
    const vw = g.vw;
    let tx = NaN;
    let ty = NaN;
    for (let i = Math.min(path.length - 1, 6); i >= 0; i--) {
      const px = cellX(path[i]!, vw);
      const py = cellY(path[i]!, vw);
      if (Math.hypot(px - f.pos[0], py - f.pos[1]) > 2.5) continue;
      if (g.lineClear(f.pos[0], f.pos[1], px, py)) {
        tx = px;
        ty = py;
        break;
      }
    }
    if (Number.isNaN(tx)) {
      if (!path.length) return this.hover(f, g.floor, true);
      tx = cellX(path[0]!, vw);
      ty = cellY(path[0]!, vw);
    }
    return this.toward(f, g.floor, tx, ty, speed);
  }
}

/** Gives up on a goal that has not gotten closer for `patience` steps. */
export class Watchdog {
  private best = Infinity;
  private still = 0;
  private goal = -1;

  constructor(readonly patience = 60) {}

  reset(): void {
    this.best = Infinity;
    this.still = 0;
    this.goal = -1;
  }

  /** True when the goal should be dropped. */
  stuck(goal: number, distance: number): boolean {
    if (goal !== this.goal) {
      this.goal = goal;
      this.best = Infinity;
      this.still = 0;
    }
    if (distance < this.best - 0.05) {
      this.best = distance;
      this.still = 0;
      return false;
    }
    this.still += 1;
    if (this.still <= this.patience) return false;
    this.reset();
    return true;
  }
}
