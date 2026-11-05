/**
 * Online boustrophedon coverage: back-and-forth lanes, with A*-style
 * backtracking when a lane system is used up. This is BA* (H. H. Viet,
 * V.-H. Dang, M. N. U. Laskar, T. Chung, "BA*: an online complete coverage
 * algorithm for cleaning robots", Applied Intelligence 2013), the
 * sensor-based form of boustrophedon coverage (H. Choset, P. Pignon,
 * "Coverage path planning: the boustrophedon cellular decomposition", Field
 * and Service Robotics 1997).
 *
 * Boustrophedon motion: from each cell the robot steps to the first
 * neighbour, in the order north, south, east, west, that is known free and
 * not yet covered. That runs lanes north and south and side-steps east. A
 * cell with no such neighbour is a critical point: the robot backtracks along
 * the shortest path over known free space to the nearest backtracking point
 * (an uncovered free cell next to a covered one) and starts a new
 * boustrophedon region there. Coverage ends when no backtracking point is
 * left.
 *
 * The cell is the bench's 0.5 m voxel, which is what flown coverage counts,
 * so lanes are one voxel apart. "Covered" is the drone's own flown map.
 */
import type { SensorCommand, SensorFrame } from '../sensorPlanner';
import { cellX, cellY, DX, DY, Follower, Grid, Watchdog, type Explorer, type ExplorerOptions } from './common';

/** Motion priority: north (+y), south, east (+x), west, as indices into DX/DY. */
const PRIORITY = [2, 3, 0, 1];

export interface BoustrophedonOptions extends ExplorerOptions {
  /** Farthest a lane target is set ahead, in cells. */
  laneCells?: number;
}

export class BoustrophedonExplorer implements Explorer {
  readonly flyHeight: number;
  readonly laneCells: number;
  private readonly fly: Follower;
  private readonly g = new Grid();
  private readonly watch = new Watchdog(40);
  /** Cells given up on (could not be reached), whole building. */
  private skip = new Uint8Array(0);
  private mode: 'sweep' | 'back' = 'sweep';
  private goal = -1;
  private path: number[] = [];
  private sincePath = 0;
  private idle = 0;
  private lastFloor = -1;
  /** Backtracks taken (diagnostics). */
  backtracks = 0;

  constructor(opts: BoustrophedonOptions = {}) {
    this.fly = new Follower(opts);
    this.flyHeight = this.fly.flyHeight;
    this.laneCells = opts.laneCells ?? 5;
  }

  reset(): void {
    this.skip = new Uint8Array(0);
    this.mode = 'sweep';
    this.goal = -1;
    this.path = [];
    this.sincePath = 0;
    this.idle = 0;
    this.lastFloor = -1;
    this.backtracks = 0;
    this.watch.reset();
  }

  get goalPoint(): [number, number] | null {
    return this.goal < 0 ? null : [cellX(this.goal, this.g.vw), cellY(this.goal, this.g.vw)];
  }

  /** Free and not yet covered (nor given up on). */
  private target(k: number): boolean {
    return this.g.open(k) && !this.g.flown(k) && !this.skip[this.g.base + k];
  }

  private neighbour(k: number, d: number): number {
    const { vw, vd } = this.g;
    const x = (k % vw) + DX[d]!;
    const y = Math.floor(k / vw) + DY[d]!;
    return x < 0 || y < 0 || x >= vw || y >= vd ? -1 : y * vw + x;
  }

  /** Boustrophedon motion from cell k: the lane end in the first free uncovered direction, or -1 at a critical point. */
  private laneFrom(k: number): number {
    for (const d of PRIORITY) {
      let c = this.neighbour(k, d);
      if (c < 0 || !this.target(c)) continue;
      for (let i = 1; i < this.laneCells; i++) {
        const n = this.neighbour(c, d);
        if (n < 0 || !this.target(n)) break;
        c = n;
      }
      return c;
    }
    return -1;
  }

  /** Nearest backtracking point by path over known free space, or -1. */
  private backtrackPoint(here: number): number {
    const g = this.g;
    g.dijkstra(here);
    let best = -1;
    let bestD = Infinity;
    let any = -1;
    let anyD = Infinity;
    for (let k = 0; k < g.plane; k++) {
      if (k === here || !this.target(k)) continue;
      const d = g.dist[k]!;
      if (!Number.isFinite(d)) continue;
      if (d < anyD) {
        anyD = d;
        any = k;
      }
      if (d >= bestD) continue;
      let nextToCovered = false;
      for (let i = 0; i < 8 && !nextToCovered; i++) {
        const n = this.neighbour(k, i);
        if (n >= 0 && g.flown(n)) nextToCovered = true;
      }
      if (nextToCovered) {
        bestD = d;
        best = k;
      }
    }
    // A region seen but bordering nothing covered yet still has to be reached.
    return best >= 0 ? best : any;
  }

  act(f: SensorFrame): SensorCommand {
    const g = this.g;
    const floor = g.load(f);
    const n = f.floors * g.plane;
    if (this.skip.length !== n) this.skip = new Uint8Array(n);
    if (floor !== this.lastFloor) {
      this.lastFloor = floor;
      this.mode = 'sweep';
      this.goal = -1;
    }
    const here = g.cellAt(f.pos[0], f.pos[1]);
    if (this.goal >= 0) {
      const d = Math.hypot(cellX(this.goal, g.vw) - f.pos[0], cellY(this.goal, g.vw) - f.pos[1]);
      if (this.watch.stuck(this.goal, d)) {
        this.skip[g.base + this.goal] = 1;
        this.goal = -1;
        this.mode = 'sweep';
      } else if (this.mode === 'back' && (here === this.goal || !this.target(this.goal))) {
        this.mode = 'sweep';
        this.goal = -1;
      }
    }
    if (this.mode === 'sweep') {
      const lane = this.laneFrom(here);
      if (lane >= 0) {
        this.goal = lane;
        this.idle = 0;
        const tx = cellX(lane, g.vw);
        const ty = cellY(lane, g.vw);
        // Arrive at the lane end slowly enough to turn into the next lane on its cell.
        const d = Math.hypot(tx - f.pos[0], ty - f.pos[1]);
        return this.fly.toward(f, floor, tx, ty, Math.min(this.fly.speed, Math.max(0.4, 2 * d)));
      }
      // Critical point: backtrack.
      this.goal = this.backtrackPoint(here);
      if (this.goal < 0) {
        this.idle += 1;
        return this.fly.hover(f, floor, this.idle < 20);
      }
      this.mode = 'back';
      this.backtracks += 1;
      this.path = g.pathTo(this.goal);
      this.sincePath = 0;
      this.watch.reset();
    } else if (++this.sincePath >= 5) {
      g.dijkstra(here);
      this.path = Number.isFinite(g.dist[this.goal]!) ? g.pathTo(this.goal) : [];
      this.sincePath = 0;
    }
    this.idle = 0;
    while (this.path.length > 1 && this.path[0] === here) this.path.shift();
    if (!this.path.length) return this.fly.toward(f, floor, cellX(this.goal, g.vw), cellY(this.goal, g.vw));
    return this.fly.along(f, g, this.path);
  }
}

