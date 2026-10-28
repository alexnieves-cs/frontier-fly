// A small frontier explorer for the explainer page, written for it: it flies a real generated
// floor plan (src/gen) with 32 horizontal lidar beams, keeps the same seen-free and visited maps
// the simulator's observation is built from (0.5 m voxels), and heads for the nearest frontier by
// breadth-first search. It ignores furniture, dynamics and noise, so it shows the idea, not the
// numbers; the shipped pilots and their benchmarks run in src/sim.
import { generateBuilding } from '../../gen/building';
import { isWalkable } from '../../gen/validate';
import type { BuildingSpec } from '../../shared/types';

const VOXEL = 0.5;
const BEAMS = 32;
const RANGE = 8;
const SPEED = 1.6;
/** How far along the BFS path the drone aims, like the observation's waypoint. */
const LOOKAHEAD = 1.25;

/** Building size per curriculum stage, as on the sim page (STAGE_SIZE in src/ui/episode.ts). */
export const STAGES: Record<number, { w: number; d: number; label: string }> = {
  0: { w: 8, d: 8, label: 'Stage 0: one room' },
  2: { w: 16, d: 16, label: 'Stage 2: rooms and doors' },
  3: { w: 22, d: 16, label: 'Stage 3: hallways and loops' },
};

const UNKNOWN = 0;
const FREE = 1;
const WALL = 2;

export class Explorer {
  readonly spec: BuildingSpec;
  readonly vw: number;
  readonly vd: number;
  /** True floor plan per voxel: 1 where the drone could fly. */
  readonly open: Uint8Array;
  /** What the lidar has shown: UNKNOWN, FREE or WALL. */
  readonly seen: Uint8Array;
  readonly visited: Uint8Array;
  /** Voxels reachable from the spawn: the coverage denominator. */
  readonly reachable: number;
  x: number;
  y: number;
  beams: [number, number][] = [];
  path: number[] = [];
  done = false;
  private visitedCount = 0;

  constructor(seed: number, stage: number) {
    const size = STAGES[stage] ?? STAGES[3]!;
    this.spec = generateBuilding({ seed, floors: 1, w: size.w, d: size.d, stage });
    const floor = this.spec.floors[0]!;
    const per = Math.round(this.spec.cellSize / VOXEL);
    this.vw = floor.w * per;
    this.vd = floor.d * per;
    const n = this.vw * this.vd;
    this.open = new Uint8Array(n);
    for (let vy = 0; vy < this.vd; vy++) {
      for (let vx = 0; vx < this.vw; vx++) {
        const cell = floor.cells[Math.floor(vy / per) * floor.w + Math.floor(vx / per)]!;
        this.open[vy * this.vw + vx] = isWalkable(cell) ? 1 : 0;
      }
    }
    this.seen = new Uint8Array(n);
    this.visited = new Uint8Array(n);
    this.x = this.spec.spawn[0];
    this.y = this.spec.spawn[1];
    this.reachable = this.flood(this.voxelAt(this.x, this.y));
    this.sense();
  }

  get coverage(): number {
    return this.visitedCount / Math.max(1, this.reachable);
  }

  voxelAt(x: number, y: number): number {
    const vx = Math.min(this.vw - 1, Math.max(0, Math.floor(x / VOXEL)));
    const vy = Math.min(this.vd - 1, Math.max(0, Math.floor(y / VOXEL)));
    return vy * this.vw + vx;
  }

  private flood(start: number): number {
    const mark = new Uint8Array(this.open.length);
    const queue = [start];
    mark[start] = 1;
    let count = 0;
    while (queue.length) {
      const v = queue.pop()!;
      count++;
      for (const u of this.neighbors(v)) {
        if (!mark[u] && this.open[u]) {
          mark[u] = 1;
          queue.push(u);
        }
      }
    }
    return count;
  }

  private *neighbors(v: number): Generator<number> {
    const vx = v % this.vw;
    const vy = (v - vx) / this.vw;
    if (vx > 0) yield v - 1;
    if (vx < this.vw - 1) yield v + 1;
    if (vy > 0) yield v - this.vw;
    if (vy < this.vd - 1) yield v + this.vw;
  }

  /** Cast the beams and mark what they cross as free and what they hit as wall. */
  private sense(): void {
    this.beams = [];
    for (let i = 0; i < BEAMS; i++) {
      const a = (i / BEAMS) * Math.PI * 2;
      const dx = Math.cos(a);
      const dy = Math.sin(a);
      let r = 0;
      for (; r < RANGE; r += VOXEL / 4) {
        const px = this.x + dx * r;
        const py = this.y + dy * r;
        if (px < 0 || py < 0 || px >= this.vw * VOXEL || py >= this.vd * VOXEL) break;
        const v = this.voxelAt(px, py);
        if (!this.open[v]) {
          this.seen[v] = WALL;
          break;
        }
        if (this.seen[v] === UNKNOWN) this.seen[v] = FREE;
      }
      this.beams.push([this.x + dx * r, this.y + dy * r]);
    }
    const here = this.voxelAt(this.x, this.y);
    for (const v of [here, ...this.neighbors(here)]) {
      if (this.open[v] && !this.visited[v]) {
        this.visited[v] = 1;
        this.visitedCount++;
      }
    }
  }

  /** Shortest path over seen-free voxels to the nearest one not yet visited (a frontier). */
  private plan(): number[] {
    const start = this.voxelAt(this.x, this.y);
    const prev = new Int32Array(this.open.length).fill(-1);
    prev[start] = start;
    const queue = [start];
    for (let head = 0; head < queue.length; head++) {
      const v = queue[head]!;
      if (v !== start && !this.visited[v]) {
        const path = [v];
        while (path[0] !== start) path.unshift(prev[path[0]!]!);
        return path;
      }
      for (const u of this.neighbors(v)) {
        if (prev[u] === -1 && this.seen[u] === FREE) {
          prev[u] = v;
          queue.push(u);
        }
      }
    }
    return [];
  }

  /** Advance dt seconds of flight. */
  step(dt: number): void {
    if (this.done) return;
    this.path = this.plan();
    if (!this.path.length) {
      this.done = true;
      return;
    }
    // Aim at the first path voxel LOOKAHEAD along, or the end of a shorter path.
    let target = this.path[this.path.length - 1]!;
    for (const v of this.path) {
      const [cx, cy] = this.center(v);
      if (Math.hypot(cx - this.x, cy - this.y) >= LOOKAHEAD) {
        target = v;
        break;
      }
    }
    // A straight line to the lookahead point can clip a wall corner; then step along the path instead.
    if (!this.fly(target, dt) && this.path.length > 1) this.fly(this.path[1]!, dt);
    this.sense();
  }

  private fly(target: number, dt: number): boolean {
    const [tx, ty] = this.center(target);
    const dist = Math.hypot(tx - this.x, ty - this.y);
    if (dist < 1e-6) return true;
    const move = Math.min(dist, SPEED * dt);
    const nx = this.x + ((tx - this.x) / dist) * move;
    const ny = this.y + ((ty - this.y) / dist) * move;
    if (!this.open[this.voxelAt(nx, ny)]) return false;
    this.x = nx;
    this.y = ny;
    return true;
  }

  center(v: number): [number, number] {
    const vx = v % this.vw;
    return [(vx + 0.5) * VOXEL, ((v - vx) / this.vw + 0.5) * VOXEL];
  }
}

export const SEEN = { UNKNOWN, FREE, WALL } as const;
export const VOXEL_SIZE = VOXEL;
