/**
 * Greedy information-gain exploration, the next-best-view strategy of
 * H. H. Gonzalez-Banos and J.-C. Latombe, "Navigation strategies for
 * exploring indoor environments", International Journal of Robotics
 * Research 21(10-11), 2002.
 *
 * Candidate viewpoints are sampled in known free space next to unexplored
 * space. Each candidate q is scored by g(q) = A(q) · exp(−λ · L(q)), where
 * A(q) is the unexplored area its sensor would see from q (rays stop at
 * obstacles) and L(q) is the length of the shortest known-free path to q.
 * The robot flies the whole path to the best candidate, senses, and chooses
 * again. Exploration ends when no candidate would see anything new.
 *
 * It differs from the nearest-frontier baseline (Yamauchi 1997) by trading
 * distance against how much a viewpoint would reveal, and from NBVP
 * (Bircher et al. 2016) by choosing among global frontier candidates and
 * committing to the full path instead of flying one edge of a random tree.
 *
 * Adaptations: the bench is 2D per floor, so candidates are the 0.5 m voxels
 * at flight height that border unknown space, thinned so no two candidates
 * lie within `stride - 1` voxels; A(q) counts unknown voxels hit by `rays` lidar-range
 * rays, the same gain NBVP uses here; L(q) is the shortest-path cost every
 * baseline here uses (voxel steps with extra cost next to walls, in meters);
 * λ was picked on the dev splits only.
 */
import * as C from '../../shared/constants';
import type { SensorCommand, SensorFrame } from '../sensorPlanner';
import { DX, DY, cellX, cellY, Follower, Grid, Watchdog, type Explorer, type ExplorerOptions } from './common';

const VOXEL = C.VOXEL;

export interface InformationGainOptions extends ExplorerOptions {
  /** Path-length discount λ, 1/m. */
  lambda?: number;
  /** Rays cast per candidate for the gain. */
  rays?: number;
  /** Sensor range for the gain, meters. */
  range?: number;
  /** Gain, in unknown voxels, below which a candidate counts as seeing nothing new. */
  minGain?: number;
  /** Candidates are thinned so that no two lie within stride - 1 voxels of each other. */
  stride?: number;
  /** Replan the path (not the goal) every this many steps, as the map grows. */
  repath?: number;
}

export class InformationGainExplorer implements Explorer {
  readonly flyHeight: number;
  readonly lambda: number;
  readonly rays: number;
  readonly range: number;
  readonly minGain: number;
  readonly stride: number;
  readonly repath: number;
  private readonly fly: Follower;
  private readonly g = new Grid();
  private readonly watch = new Watchdog(60);
  /** Destinations given up on or reached, per voxel of the whole building (all floors). */
  private skip = new Uint8Array(0);
  private stamp = new Uint32Array(0);
  private covered = new Uint8Array(0);
  private tick = 0;
  private goal = -1;
  private goalFloor = -1;
  private path: number[] = [];
  private sincePath = 0;
  private idle = 0;

  constructor(opts: InformationGainOptions = {}) {
    this.fly = new Follower(opts);
    this.flyHeight = this.fly.flyHeight;
    this.lambda = opts.lambda ?? 0.1;
    this.rays = opts.rays ?? 32;
    this.range = opts.range ?? C.LIDAR_MAX;
    this.minGain = opts.minGain ?? 2;
    this.stride = opts.stride ?? 2;
    this.repath = opts.repath ?? 5;
  }

  reset(): void {
    this.skip = new Uint8Array(0);
    this.goal = -1;
    this.goalFloor = -1;
    this.path = [];
    this.sincePath = 0;
    this.idle = 0;
    this.watch.reset();
  }

  /** The current destination in world xy, or null. */
  get goalPoint(): [number, number] | null {
    return this.goal < 0 ? null : [cellX(this.goal, this.g.vw), cellY(this.goal, this.g.vw)];
  }

  /** Unknown voxels in sensor view from (x, y): rays stop at occupied voxels and at the grid edge. */
  private gain(x: number, y: number): number {
    const g = this.g;
    if (this.stamp.length !== g.plane) this.stamp = new Uint32Array(g.plane);
    const tick = ++this.tick;
    const stamp = this.stamp;
    const step = VOXEL * 0.5;
    let n = 0;
    for (let i = 0; i < this.rays; i++) {
      const a = (i / this.rays) * Math.PI * 2;
      const ux = Math.cos(a);
      const uy = Math.sin(a);
      for (let d = step; d <= this.range; d += step) {
        const k = g.cellIn(x + ux * d, y + uy * d);
        if (k < 0 || g.occupied(k)) break;
        if (stamp[k] === tick) continue;
        stamp[k] = tick;
        if (g.unknown(k)) n += 1;
      }
    }
    return n;
  }

  private frontier(k: number): boolean {
    const g = this.g;
    const { vw, vd } = g;
    const x = k % vw;
    const y = (k - x) / vw;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d]!;
      const ny = y + DY[d]!;
      if (nx >= 0 && ny >= 0 && nx < vw && ny < vd && g.unknown(ny * vw + nx)) return true;
    }
    return false;
  }

  /** The candidate with the best discounted gain, or -1 when none would see anything new. */
  private choose(start: number): number {
    const g = this.g;
    const { vw, vd, plane, base } = g;
    g.dijkstra(start);
    if (this.covered.length !== plane) this.covered = new Uint8Array(plane);
    const covered = this.covered;
    covered.fill(0);
    const r = this.stride - 1;
    let best = -1;
    let bestU = 0;
    for (let k = 0; k < plane; k++) {
      if (k === start || covered[k] || this.skip[base + k] || !Number.isFinite(g.dist[k]!) || !this.frontier(k)) continue;
      // Thin the candidates: no other candidate within `stride - 1` voxels of this one.
      const x = k % vw;
      const y = (k - x) / vw;
      for (let dy = -r; dy <= r; dy++)
        for (let dx = -r; dx <= r; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < vw && ny < vd) covered[ny * vw + nx] = 1;
        }
      const a = this.gain(cellX(k, vw), cellY(k, vw));
      if (a < this.minGain) continue;
      const u = a * Math.exp(-this.lambda * g.dist[k]! * VOXEL);
      if (u > bestU) {
        bestU = u;
        best = k;
      }
    }
    return best;
  }

  private mark(k: number): void {
    const { vw, vd, base } = this.g;
    const x = k % vw;
    const y = (k - x) / vw;
    for (let dy = -1; dy <= 1; dy++)
      for (let dx = -1; dx <= 1; dx++) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < vw && ny < vd) this.skip[base + ny * vw + nx] = 1;
      }
  }

  act(f: SensorFrame): SensorCommand {
    const g = this.g;
    const floor = g.load(f);
    const n = f.floors * g.plane;
    if (this.skip.length !== n) this.skip = new Uint8Array(n);
    if (floor !== this.goalFloor) this.goal = -1;
    const here = g.cellAt(f.pos[0], f.pos[1]);
    if (this.goal >= 0) {
      const d = Math.hypot(cellX(this.goal, g.vw) - f.pos[0], cellY(this.goal, g.vw) - f.pos[1]);
      if (d < 0.5) {
        this.mark(this.goal);
        this.goal = -1;
      } else if (!g.open(this.goal) || this.watch.stuck(this.goal, d)) {
        this.mark(this.goal);
        this.goal = -1;
      }
    }
    if (this.goal < 0) {
      this.goal = this.choose(here);
      this.goalFloor = floor;
      this.watch.reset();
      this.path = this.goal >= 0 ? g.pathTo(this.goal) : [];
      this.sincePath = 0;
    } else if (++this.sincePath >= this.repath) {
      g.dijkstra(here);
      this.path = Number.isFinite(g.dist[this.goal]!) ? g.pathTo(this.goal) : [];
      this.sincePath = 0;
      if (!this.path.length) {
        this.mark(this.goal);
        this.goal = -1;
      }
    }
    if (this.goal < 0) {
      this.idle += 1;
      return this.fly.hover(f, floor, this.idle < 20);
    }
    this.idle = 0;
    while (this.path.length > 1 && this.path[0] === here) this.path.shift();
    return this.fly.along(f, g, this.path);
  }
}
