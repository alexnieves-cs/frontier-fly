/**
 * Frontier-based exploration as Yamauchi published it (B. Yamauchi, "A
 * frontier-based approach for autonomous exploration", IEEE CIRA 1997).
 *
 * A frontier cell is a known-free cell next to unknown space. Adjacent
 * frontier cells form a frontier region, and regions smaller than the robot
 * are ignored. The robot picks the nearest accessible region, by path length
 * over known free space, and flies to it. On arrival the location joins a
 * list of visited destinations; a destination it cannot make progress toward
 * joins a list of inaccessible ones. Frontiers near either are skipped. The
 * goal is kept until it is reached or given up on, as in the paper, and then
 * frontiers are detected again. Exploration ends when no region is left.
 *
 * Adaptations, all forced by the airframe rather than chosen for score: the
 * grid is the bench's 0.5 m voxel plane at flight height; "unknown" means
 * neither seen free, flown nor marked occupied by the drone's own lidar; and
 * the paper's 360° sensor sweep on arrival is dropped, because the lidar
 * ring already covers 360° on every step.
 */
import type { SensorCommand, SensorFrame } from '../sensorPlanner';
import { DX, DY, cellX, cellY, Follower, Grid, Watchdog, type Explorer, type ExplorerOptions } from './common';

export interface NearestFrontierOptions extends ExplorerOptions {
  /** Smallest frontier region kept, in voxels (about the airframe's width plus margin). */
  minRegion?: number;
  /** Frontier cells within this many voxels of a visited or inaccessible destination are skipped. */
  skipRadius?: number;
  /** Replan the path (not the goal) every this many steps, as the map grows. */
  repath?: number;
}

export class NearestFrontierExplorer implements Explorer {
  readonly flyHeight: number;
  readonly minRegion: number;
  readonly skipRadius: number;
  readonly repath: number;
  private readonly fly: Follower;
  private readonly g = new Grid();
  private readonly watch = new Watchdog(60);
  /** Visited or inaccessible destinations, per voxel of the whole building (all floors). */
  private skip = new Uint8Array(0);
  private frontier = new Uint8Array(0);
  private region = new Int32Array(0);
  private goal = -1;
  private goalFloor = -1;
  private path: number[] = [];
  private sincePath = 0;
  private idle = 0;

  constructor(opts: NearestFrontierOptions = {}) {
    this.fly = new Follower(opts);
    this.flyHeight = this.fly.flyHeight;
    this.minRegion = opts.minRegion ?? 3;
    this.skipRadius = opts.skipRadius ?? 1;
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

  private mark(k: number): void {
    const { vw, vd, base } = this.g;
    const x = k % vw;
    const y = (k - x) / vw;
    const r = this.skipRadius;
    for (let dy = -r; dy <= r; dy++)
      for (let dx = -r; dx <= r; dx++) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < vw && ny < vd) this.skip[base + ny * vw + nx] = 1;
      }
  }

  /** Detect frontier regions and return the nearest reachable cell of a region big enough, or -1. */
  private choose(start: number): number {
    const g = this.g;
    const { vw, vd, plane, base } = g;
    if (this.frontier.length !== plane) {
      this.frontier = new Uint8Array(plane);
      this.region = new Int32Array(plane);
    }
    const fr = this.frontier;
    for (let k = 0; k < plane; k++) {
      fr[k] = 0;
      if (!g.open(k) || this.skip[base + k]) continue;
      const x = k % vw;
      const y = (k - x) / vw;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d]!;
        const ny = y + DY[d]!;
        if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
        if (g.unknown(ny * vw + nx)) {
          fr[k] = 1;
          break;
        }
      }
    }
    // Frontier regions: 8-connected groups of frontier cells, labelled by size.
    const region = this.region;
    region.fill(0);
    const stack: number[] = [];
    const members: number[] = [];
    for (let k = 0; k < plane; k++) {
      if (!fr[k] || region[k]) continue;
      members.length = 0;
      stack.push(k);
      region[k] = -1;
      while (stack.length) {
        const c = stack.pop()!;
        members.push(c);
        const x = c % vw;
        const y = (c - x) / vw;
        for (let d = 0; d < 8; d++) {
          const nx = x + DX[d]!;
          const ny = y + DY[d]!;
          if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
          const nk = ny * vw + nx;
          if (fr[nk] && !region[nk]) {
            region[nk] = -1;
            stack.push(nk);
          }
        }
      }
      for (const c of members) region[c] = members.length;
    }
    g.dijkstra(start);
    let best = -1;
    let bestD = Infinity;
    for (let k = 0; k < plane; k++) {
      if (k === start || region[k]! < this.minRegion) continue;
      const d = g.dist[k]!;
      if (d < bestD) {
        bestD = d;
        best = k;
      }
    }
    return best;
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
        this.mark(this.goal); // reached: a visited destination
        this.goal = -1;
      } else if (!g.open(this.goal) || this.watch.stuck(this.goal, d)) {
        this.mark(this.goal); // inaccessible
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
