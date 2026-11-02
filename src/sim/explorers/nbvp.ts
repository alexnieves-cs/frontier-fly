/**
 * Receding-horizon next-best-view planning (A. Bircher, M. Kamel, K. Alexis,
 * H. Oleynikova, R. Siegwart, "Receding horizon 'next-best-view' planner
 * for 3D exploration", IEEE ICRA 2016), the NBVP algorithm.
 *
 * Every iteration grows a rapidly-exploring random tree (RRT) through known
 * free space from the drone. Each node's gain is the unknown space its sensor
 * would see from there, rays stopping at occupied voxels; a node's score is
 * its parent's plus its gain discounted by the path length to it,
 * gain · exp(−λ · cost). The tree grows until it has at least `minNodes`
 * nodes and a node with gain above `gZero`, or `maxNodes` nodes. The drone
 * flies only the first edge of the best branch, then replans, seeding the new
 * tree with the rest of that branch. Exploration ends when a full tree finds
 * no node with gain above `gZero`.
 *
 * Adaptations: the tree lives on the 0.5 m voxel plane at flight height
 * (the bench is 2D per floor), and the sampled yaw is dropped because the
 * lidar ring sees 360°, so gain does not depend on heading.
 */
import * as C from '../../shared/constants';
import { RNG } from '../../shared/rng';
import type { SensorCommand, SensorFrame } from '../sensorPlanner';
import { cellX, cellY, Follower, Grid, Watchdog, type Explorer, type ExplorerOptions } from './common';

const VOXEL = C.VOXEL;

export interface NbvpOptions extends ExplorerOptions {
  /** Path-length discount λ, 1/m (the paper's value is 0.5). */
  lambda?: number;
  /** RRT extension range, meters. */
  extension?: number;
  /** Nodes the tree grows to before it may stop. */
  minNodes?: number;
  /** Nodes the tree may grow to. */
  maxNodes?: number;
  /** Gain, in unknown voxels, below which a node counts as seeing nothing new. */
  gZero?: number;
  /** Sensor range for the gain, meters. */
  range?: number;
  /** Rays cast per node for the gain. */
  rays?: number;
  /** RNG seed; each reset restarts the stream, so flights are deterministic. */
  seed?: number;
}

interface Node {
  x: number;
  y: number;
  parent: number;
  cost: number;
  gain: number;
  score: number;
}

export class NbvpExplorer implements Explorer {
  readonly flyHeight: number;
  readonly lambda: number;
  readonly extension: number;
  readonly minNodes: number;
  readonly maxNodes: number;
  readonly gZero: number;
  readonly range: number;
  readonly rays: number;
  readonly seed: number;
  private readonly fly: Follower;
  private readonly g = new Grid();
  private readonly watch = new Watchdog(40);
  private rng: RNG;
  private nodes: Node[] = [];
  /** The best branch of the last tree, root excluded: [x, y] points. */
  private branch: [number, number][] = [];
  private target: [number, number] | null = null;
  private stamp = new Uint32Array(0);
  private tick = 0;
  private free: number[] = [];
  private idle = 0;
  private lastFloor = -1;
  /** Trees grown (diagnostics). */
  iterations = 0;

  constructor(opts: NbvpOptions = {}) {
    this.fly = new Follower(opts);
    this.flyHeight = this.fly.flyHeight;
    this.lambda = opts.lambda ?? 0.5;
    this.extension = opts.extension ?? 1;
    this.minNodes = opts.minNodes ?? 15;
    this.maxNodes = opts.maxNodes ?? 150;
    this.gZero = opts.gZero ?? 2;
    this.range = opts.range ?? C.LIDAR_MAX;
    this.rays = opts.rays ?? 64;
    this.seed = opts.seed ?? 2016;
    this.rng = new RNG(this.seed, 'nbvp');
  }

  reset(): void {
    this.rng = new RNG(this.seed, 'nbvp');
    this.nodes = [];
    this.branch = [];
    this.target = null;
    this.idle = 0;
    this.lastFloor = -1;
    this.iterations = 0;
  }

  /** The next viewpoint in world xy, or null. */
  get goalPoint(): [number, number] | null {
    return this.target;
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

  private add(x: number, y: number, parent: number): Node {
    const p = this.nodes[parent]!;
    const cost = p.cost + Math.hypot(x - p.x, y - p.y);
    const gain = this.gain(x, y);
    const node = { x, y, parent, cost, gain, score: p.score + gain * Math.exp(-this.lambda * cost) };
    this.nodes.push(node);
    return node;
  }

  /** Edge between two points stays on open voxels. */
  private edgeClear(ax: number, ay: number, bx: number, by: number): boolean {
    return this.g.lineClear(ax, ay, bx, by, 1);
  }

  /** Grow one tree from the drone; returns the best branch (root excluded), empty when nothing is left to see. */
  private grow(f: SensorFrame): [number, number][] {
    const g = this.g;
    this.iterations += 1;
    this.nodes = [{ x: f.pos[0], y: f.pos[1], parent: -1, cost: 0, gain: 0, score: 0 }];
    let best = 0;
    let bestGain = 0;
    const consider = (i: number) => {
      const nd = this.nodes[i]!;
      if (nd.score > this.nodes[best]!.score) best = i;
      if (nd.gain > bestGain) bestGain = nd.gain;
    };
    // Seed with the rest of the last best branch while its edges are still free.
    let parent = 0;
    for (const [x, y] of this.branch) {
      const p = this.nodes[parent]!;
      if (!this.edgeClear(p.x, p.y, x, y)) break;
      this.add(x, y, parent);
      parent = this.nodes.length - 1;
      consider(parent);
    }
    // Known free voxels to sample from, on this floor.
    const free = this.free;
    free.length = 0;
    for (let k = 0; k < g.plane; k++) if (g.open(k)) free.push(k);
    if (!free.length) return [];
    let tries = 0;
    while (this.nodes.length < this.maxNodes && (this.nodes.length < this.minNodes || bestGain <= this.gZero) && tries < this.maxNodes * 20) {
      tries += 1;
      const k = free[this.rng.int(free.length)]!;
      const sx = cellX(k, g.vw) + (this.rng.next() - 0.5) * VOXEL;
      const sy = cellY(k, g.vw) + (this.rng.next() - 0.5) * VOXEL;
      let near = 0;
      let nd = Infinity;
      for (let i = 0; i < this.nodes.length; i++) {
        const d = Math.hypot(this.nodes[i]!.x - sx, this.nodes[i]!.y - sy);
        if (d < nd) {
          nd = d;
          near = i;
        }
      }
      if (nd < 0.05) continue;
      const p = this.nodes[near]!;
      const t = Math.min(1, this.extension / nd);
      const x = p.x + (sx - p.x) * t;
      const y = p.y + (sy - p.y) * t;
      if (!this.edgeClear(p.x, p.y, x, y)) continue;
      this.add(x, y, near);
      consider(this.nodes.length - 1);
    }
    if (bestGain <= this.gZero || best === 0) return [];
    const out: [number, number][] = [];
    for (let i = best; i > 0; i = this.nodes[i]!.parent) out.push([this.nodes[i]!.x, this.nodes[i]!.y]);
    return out.reverse();
  }

  act(f: SensorFrame): SensorCommand {
    const g = this.g;
    const floor = g.load(f);
    if (floor !== this.lastFloor) {
      this.lastFloor = floor;
      this.branch = [];
      this.target = null;
    }
    if (this.target) {
      const d = Math.hypot(this.target[0] - f.pos[0], this.target[1] - f.pos[1]);
      const k = g.cellIn(this.target[0], this.target[1]);
      if (d < 0.35 || k < 0 || !g.open(k) || this.watch.stuck(k, d)) {
        this.target = null;
        if (d >= 0.35) this.branch = [];
      }
    }
    if (!this.target) {
      const branch = this.grow(f);
      this.watch.reset();
      if (!branch.length) {
        this.branch = [];
        this.idle += 1;
        return this.fly.hover(f, floor, this.idle < 20);
      }
      this.target = branch[0]!;
      this.branch = branch.slice(1);
    }
    this.idle = 0;
    return this.fly.toward(f, floor, this.target[0], this.target[1]);
  }
}
