import { operable, type Airspace } from '../planner/geometry';
import type { SearchBelief } from './belief';
import { cellCenter, sectorOf, skyVisible } from './grid';
import { framePod, PERSON_CENTER_M, type SensorConditions } from './pod';

/*
 * Receding-horizon search planner on the belief.
 *
 * At each replan a beam search grows paths of fixed-length legs in a fan of
 * headings from the drone. A leg is scored by the probability mass its
 * camera frames would remove from the belief (the expected gain in POS),
 * with frames along one path discounting each other's cells, and the sum is
 * discounted by flight time so that mass found sooner counts more. A
 * terminal term pulls toward the densest unsearched block beyond the
 * horizon, which keeps the planner from circling once its neighbourhood is
 * searched. The drone flies the first legs of the best path and replans.
 *
 * Buildings are search segments too. Each holds the prior mass that the
 * person sheltered inside it. A building is visited when its rate (mass
 * times indoor POD per second of travel and search) beats the best outdoor
 * rate, the classic rule of allocating effort where POD x POA per unit of
 * effort is highest (Stone 1975; the IAMSAR manual's effort allocation).
 */

export interface PlannerConfig {
  /** Planned cruise height above ground, meters. */
  agl: number;
  /** Cruise speed, m/s. */
  speed: number;
  /** Seconds between camera frames. */
  framePeriod: number;
  /** Leg length, meters. */
  legM: number;
  /** Headings fanned per expansion. */
  headings: number;
  /** Legs per path. 1 makes a greedy planner. */
  depth: number;
  /** Paths kept per depth. */
  beam: number;
  /** Legs flown before replanning. */
  commit: number;
  /** Time constant of the reward discount, seconds. */
  discountSec: number;
  /** Pull toward unsearched mass beyond the horizon. */
  terminal: boolean;
  /** Seconds a building visit is expected to take, door to door. */
  visitSec: number;
  /** Chance an indoor search finds a person who is inside. */
  structurePod: number;
  /** Yaw rate used for the turn cost, rad/s. */
  yawRate: number;
  /** Stop when the area and the buildings hold less than this fraction of the starting mass. */
  doneFraction: number;
}

export const DEFAULT_PLANNER: PlannerConfig = {
  agl: 13,
  speed: 5.5,
  framePeriod: 0.2,
  legM: 6,
  headings: 12,
  depth: 5,
  beam: 12,
  commit: 2,
  discountSec: 90,
  terminal: true,
  visitSec: 200,
  structurePod: 0.85,
  yawRate: Math.PI / 2,
  doneFraction: 0.01,
};

export interface BuildingSegment {
  id: string;
  center: [number, number];
}

export type SearchDecision =
  | { kind: 'fly'; path: [number, number][]; gain: number; rate: number }
  | { kind: 'building'; id: string; rate: number }
  | { kind: 'done'; reason: string };

interface KernelCell {
  di: number;
  dj: number;
  p: number;
  /** Sky-view sector and tan(elevation) of the sightline from the cell up to the camera. */
  sector: number;
  tanE: number;
}

interface Node {
  x: number;
  y: number;
  heading: number;
  t: number;
  score: number;
  gain: number;
  /** Remaining-mass fraction of cells this node's own leg touched. */
  local: Map<number, number>;
  parent: Node | null;
  value: number;
}

const BLOCK = 4;

export class ProbabilityPlanner {
  readonly cfg: PlannerConfig;
  private readonly kernels: KernelCell[][];

  constructor(
    readonly belief: SearchBelief,
    readonly air: Airspace,
    sensor: SensorConditions,
    cfg: Partial<PlannerConfig> = {},
  ) {
    this.cfg = { ...DEFAULT_PLANNER, ...cfg };
    const g = belief.grid;
    // Frame kernels per heading for a camera over a cell center. Occlusion is looked up per cell.
    const h = this.cfg.agl - PERSON_CENTER_M;
    const reach = Math.ceil((h * 1.6) / g.cell) + 1;
    this.kernels = [];
    for (let k = 0; k < this.cfg.headings; k++) {
      const yaw = (k / this.cfg.headings) * Math.PI * 2;
      const c = Math.cos(yaw);
      const s = Math.sin(yaw);
      const out: KernelCell[] = [];
      for (let dj = -reach; dj <= reach; dj++) {
        for (let di = -reach; di <= reach; di++) {
          const dx = di * g.cell;
          const dy = dj * g.cell;
          const p = framePod(h, dx * c + dy * s, -dx * s + dy * c, this.cfg.speed, 0, sensor);
          if (p > 1e-4) out.push({ di, dj, p, sector: sectorOf(Math.atan2(-dy, -dx)), tanE: h / Math.max(Math.hypot(dx, dy), 0.01) });
        }
      }
      this.kernels.push(out);
    }
  }

  private remainingFraction(node: Node | null, k: number): number {
    for (let n = node; n; n = n.parent) {
      const r = n.local.get(k);
      if (r !== undefined) return r;
    }
    return 1;
  }

  /** Child node for a leg from `parent` along heading index `hi`, or null when the leg leaves the airspace. */
  private leg(parent: Node, hi: number): Node | null {
    const cfg = this.cfg;
    const g = this.belief.grid;
    const yaw = (hi / cfg.headings) * Math.PI * 2;
    const x1 = parent.x + Math.cos(yaw) * cfg.legM;
    const y1 = parent.y + Math.sin(yaw) * cfg.legM;
    const xm = (parent.x + x1) / 2;
    const ym = (parent.y + y1) / 2;
    if (!operable(this.air, x1, y1) || !operable(this.air, xm, ym)) return null;
    const turn = Math.abs(Math.atan2(Math.sin(yaw - parent.heading), Math.cos(yaw - parent.heading)));
    const dt = cfg.legM / cfg.speed + (turn / cfg.yawRate) * 0.6;
    const frames = Math.max(1, Math.round(cfg.legM / (cfg.speed * cfg.framePeriod)));
    const kernel = this.kernels[hi]!;
    const local = new Map<number, number>();
    let gain = 0;
    for (let f = 0; f < frames; f++) {
      const u = (f + 0.5) / frames;
      const fx = parent.x + (x1 - parent.x) * u;
      const fy = parent.y + (y1 - parent.y) * u;
      const ci = Math.floor((fx - g.x0) / g.cell);
      const cj = Math.floor((fy - g.y0) / g.cell);
      for (const kc of kernel) {
        const i = ci + kc.di;
        const j = cj + kc.dj;
        if (i < 0 || j < 0 || i >= g.nx || j >= g.ny) continue;
        const k = j * g.nx + i;
        const m = this.belief.mass[k]!;
        if (m <= 0) continue;
        if (!skyVisible(g, k, kc.sector, kc.tanE)) continue;
        const p = kc.p;
        const r = local.get(k) ?? this.remainingFraction(parent, k);
        gain += m * r * p;
        local.set(k, r * (1 - p));
      }
    }
    const t = parent.t + dt;
    const score = parent.score + gain * Math.exp(-t / cfg.discountSec);
    return { x: x1, y: y1, heading: yaw, t, score, gain: parent.gain + gain, local, parent, value: score };
  }

  /** Unsearched mass per BLOCK x BLOCK block of cells, with block centers. */
  private blocks(): { x: number; y: number; m: number }[] {
    const g = this.belief.grid;
    const bx = Math.ceil(g.nx / BLOCK);
    const by = Math.ceil(g.ny / BLOCK);
    const sums = new Float64Array(bx * by);
    const wx = new Float64Array(bx * by);
    const wy = new Float64Array(bx * by);
    for (const k of g.cells) {
      const m = this.belief.mass[k]!;
      if (m <= 0) continue;
      const i = k % g.nx;
      const j = Math.floor(k / g.nx);
      const b = Math.floor(j / BLOCK) * bx + Math.floor(i / BLOCK);
      const [cx, cy] = cellCenter(g, k);
      sums[b] += m;
      wx[b] += m * cx;
      wy[b] += m * cy;
    }
    const out: { x: number; y: number; m: number }[] = [];
    for (let b = 0; b < sums.length; b++) {
      if (sums[b]! <= 0) continue;
      const x = wx[b]! / sums[b]!;
      const y = wy[b]! / sums[b]!;
      if (!operable(this.air, x, y, 0.5)) continue;
      out.push({ x, y, m: sums[b]! });
    }
    return out;
  }

  /** Discounted value of the best block from a leaf, minus what the leaf's path already took there. */
  private terminal(node: Node, blocks: readonly { x: number; y: number; m: number }[]): number {
    const cfg = this.cfg;
    let best = 0;
    for (const b of blocks) {
      const d = Math.hypot(b.x - node.x, b.y - node.y);
      const v = b.m * 0.7 * Math.exp(-(node.t + d / cfg.speed) / cfg.discountSec);
      if (v > best) best = v;
    }
    return best;
  }

  plan(pos: [number, number], heading: number, buildings: readonly BuildingSegment[] = [], visited: ReadonlySet<string> = new Set()): SearchDecision {
    const cfg = this.cfg;
    const belief = this.belief;
    const area = belief.areaMass();
    let left = area;
    for (const b of buildings) if (!visited.has(b.id)) left += belief.structure.get(b.id) ?? 0;
    if (left < cfg.doneFraction * belief.initial) return { kind: 'done', reason: 'probability left in the search area is below the stop threshold' };

    const blocks = cfg.terminal ? this.blocks() : [];
    const root: Node = { x: pos[0], y: pos[1], heading, t: 0, score: 0, gain: 0, local: new Map(), parent: null, value: 0 };
    let frontier: Node[] = [root];
    let best: Node | null = null;
    for (let depth = 0; depth < cfg.depth; depth++) {
      const children: Node[] = [];
      for (const node of frontier) {
        for (let hi = 0; hi < cfg.headings; hi++) {
          const child = this.leg(node, hi);
          if (!child) continue;
          child.value = child.score + (cfg.terminal ? this.terminal(child, blocks) : 0);
          children.push(child);
        }
      }
      if (!children.length) break;
      children.sort((a, b) => b.value - a.value);
      frontier = children.slice(0, cfg.beam);
      if (!best || frontier[0]!.value > best.value || depth === cfg.depth - 1) best = frontier[0]!;
    }

    // Outdoor rate: the best path's gain per second, or the best block's when the neighbourhood is spent.
    let outdoorRate = best ? best.gain / Math.max(1, best.t) : 0;
    for (const b of blocks) {
      const r = (b.m * 0.7) / (Math.hypot(b.x - pos[0], b.y - pos[1]) / cfg.speed + 4);
      if (r > outdoorRate) outdoorRate = r;
    }
    let bestBuilding: { id: string; rate: number } | null = null;
    for (const b of buildings) {
      if (visited.has(b.id)) continue;
      const m = belief.structure.get(b.id) ?? 0;
      const rate = (m * cfg.structurePod) / (Math.hypot(b.center[0] - pos[0], b.center[1] - pos[1]) / cfg.speed + cfg.visitSec);
      if (rate > 0 && (!bestBuilding || rate > bestBuilding.rate)) bestBuilding = { id: b.id, rate };
    }
    if (bestBuilding && (bestBuilding.rate > outdoorRate || area < cfg.doneFraction * belief.initial)) {
      return { kind: 'building', id: bestBuilding.id, rate: bestBuilding.rate };
    }
    if (area < cfg.doneFraction * belief.initial) return { kind: 'done', reason: 'outdoor area searched' };

    if (!best || best.gain <= 1e-9) {
      // Nothing to gain within the horizon: head for the richest block.
      let target: { x: number; y: number; m: number } | null = null;
      let tv = 0;
      for (const b of blocks) {
        const v = b.m / (Math.hypot(b.x - pos[0], b.y - pos[1]) + 8);
        if (v > tv) {
          tv = v;
          target = b;
        }
      }
      if (!target) return { kind: 'done', reason: 'no reachable unsearched ground' };
      return { kind: 'fly', path: [[target.x, target.y]], gain: 0, rate: outdoorRate };
    }
    const chain: Node[] = [];
    for (let n: Node | null = best; n && n.parent; n = n.parent) chain.push(n);
    chain.reverse();
    const path = chain.slice(0, cfg.commit).map((n) => [n.x, n.y] as [number, number]);
    return { kind: 'fly', path, gain: best.gain, rate: outdoorRate };
  }
}
