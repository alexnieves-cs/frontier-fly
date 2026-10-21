import { RNG } from '../../../shared/rng';
import { PERSON_SIZE } from '../../../targets/catalog';
import { bodyAabb } from '../../../targets/motion';
import type { PersonState, PlacedTarget } from '../../../targets/types';
import { containsPoint } from '../../../world/outdoor/geofence';
import { distToSegment } from '../../../world/outdoor/math';
import { sampleHeight } from '../../../world/outdoor/terrain';
import type { MissionBuilding, MissionWorld } from '../contracts';
import { plantingObstacles, type GroundBox } from '../world';
import { cellCenter, type SearchGrid } from './grid';

/*
 * Lost-person behavior, two ways.
 *
 * Search planning starts from where the person was last seen (the initial
 * planning point, IPP) and from how lost people tend to move. The SAR
 * literature sorts that into reorientation strategies: staying put, following
 * a trail, wandering, climbing for a view, heading for a structure (Syrotuck,
 * "Analysis of Lost Person Behavior", 1977; Hill, "Lost Person Behavior",
 * 1998; Koester, "Lost Person Behavior", 2008). Distances from the IPP are
 * heavily right-skewed: most people are found near it and a few very far.
 *
 * `lostPersonPrior` is what the searcher believes: an analytic probability-of-
 * area (POA) map, a lognormal distance ring around the IPP per strategy,
 * shaped by map features. `simulateLostPerson` is how the benchmark decides
 * where the person really is: an agent that walks from the IPP under one of
 * the same strategies, with its own mechanics (correlated random walks,
 * trail following, hill climbing, heading for a building). The two share
 * strategy names and weights but not their geometry, so the prior is a model
 * of the truth rather than a copy of it.
 *
 * No published data tables are copied. The distances are scaled to the 80 m
 * site: the prior's median is about 12 m from the IPP and its tail runs past
 * the geofence, which is where the rest-of-world mass comes from.
 */

export const STRATEGIES = ['stay_put', 'trail', 'wander', 'view', 'shelter'] as const;
export type Strategy = (typeof STRATEGIES)[number];

/** Planning weights of the strategies. The simulator draws with the same weights. */
export const STRATEGY_WEIGHT: Record<Strategy, number> = {
  stay_put: 0.2,
  trail: 0.3,
  wander: 0.25,
  view: 0.1,
  shelter: 0.15,
};

/** Lognormal distance-from-IPP ring of each strategy in the prior: median meters and log-sd. */
const RING: Record<Strategy, { median: number; sigma: number }> = {
  stay_put: { median: 3, sigma: 0.6 },
  trail: { median: 13, sigma: 0.75 },
  wander: { median: 12, sigma: 0.9 },
  view: { median: 12, sigma: 0.8 },
  shelter: { median: 12, sigma: 0.8 },
};

/** Share of the shelter strategy that ends inside a structure in the prior. */
const SHELTER_INSIDE = 0.6;

export interface LostPersonPrior {
  ipp: [number, number];
  /** Prior mass per grid cell. Zero off the mask. */
  cell: Float64Array;
  /** Prior mass inside each building, by building id. */
  structure: Map<string, number>;
  /**
   * Mass the aerial search cannot reach: outside the geofence, or on ground
   * hidden under a crown core or a no-fly zone. The rest of world.
   */
  rest: number;
  /** Per-strategy share of the in-area mass, for the report. */
  byStrategy: Record<Strategy, number>;
}

export interface PriorOptions {
  /** Strategy weights. Default STRATEGY_WEIGHT. */
  weights?: Partial<Record<Strategy, number>>;
  /**
   * 'behavior' (default): rings shaped by trails, terrain and structures.
   * 'ring': the distance ring alone, with no map features.
   * 'uniform': flat over the search area: the no-information prior.
   */
  shape?: 'behavior' | 'ring' | 'uniform';
}

function lognormalPdf(d: number, median: number, sigma: number): number {
  if (d <= 0) return 0;
  const z = (Math.log(d) - Math.log(median)) / sigma;
  return Math.exp(-0.5 * z * z) / (d * sigma * Math.sqrt(2 * Math.PI));
}

/** Planar density of a distance ring at distance d: pdf(d) / (2 pi d). */
function ringDensity(d: number, median: number, sigma: number): number {
  const r = Math.max(d, 0.75);
  return lognormalPdf(r, median, sigma) / (2 * Math.PI * r);
}

function buildingBoxOf(b: MissionBuilding): GroundBox {
  const xs = b.footprint.map((p) => p.x);
  const ys = b.footprint.map((p) => p.y);
  return { minX: Math.min(...xs), minY: Math.min(...ys), maxX: Math.max(...xs), maxY: Math.max(...ys) };
}

function boxCenter(box: GroundBox): [number, number] {
  return [(box.minX + box.maxX) / 2, (box.minY + box.maxY) / 2];
}

/** Feature multiplier of a strategy at a cell. Unnormalized; 1 means no preference. */
function feature(strategy: Strategy, grid: SearchGrid, k: number, ippGround: number): number {
  switch (strategy) {
    case 'trail': {
      const d = grid.trailDist[k]!;
      return 0.15 + 6 * Math.exp(-(d * d) / (2 * 3 * 3));
    }
    case 'view':
      return Math.exp(0.5 * Math.max(-6, Math.min(6, grid.ground[k]! - ippGround)));
    case 'shelter': {
      const d = grid.structureDist[k]!;
      return 0.05 + 8 * Math.exp(-(d * d) / (2 * 3 * 3));
    }
    default:
      return 1;
  }
}

/**
 * The searcher's POA map over the grid. Each strategy's ring is normalized
 * over the whole plane, treating ground beyond the fence as featureless, so
 * the share of each ring that falls outside the fence lands in `rest`.
 */
export function lostPersonPrior(world: MissionWorld, grid: SearchGrid, ipp: [number, number], options: PriorOptions = {}): LostPersonPrior {
  const shape = options.shape ?? 'behavior';
  const weights = { ...STRATEGY_WEIGHT, ...options.weights };
  const total = STRATEGIES.reduce((s, k) => s + weights[k], 0);
  const n = grid.nx * grid.ny;
  const cell = new Float64Array(n);
  const area = grid.cell * grid.cell;
  const ippGround = sampleHeight(world.outdoor.terrain, ipp[0], ipp[1]);
  const fence = world.outdoor.geofence;
  const inFence = new Uint8Array(n);
  for (let k = 0; k < n; k++) {
    const [x, y] = cellCenter(grid, k);
    if (containsPoint(fence, x, y)) inFence[k] = 1;
  }
  // Share of each cell that is building floor, and whose. A person placed there is inside it.
  const boxes = world.buildings.map(buildingBoxOf);
  const floor = new Float64Array(n);
  const floorOf = new Int16Array(n).fill(-1);
  for (let k = 0; k < n; k++) {
    if (!inFence[k]) continue;
    const [x, y] = cellCenter(grid, k);
    const i = boxes.findIndex((b) => x + grid.cell > b.minX && x - grid.cell < b.maxX && y + grid.cell > b.minY && y - grid.cell < b.maxY);
    if (i < 0) continue;
    const b = boxes[i]!;
    let hit = 0;
    for (let sy = 0; sy < 4; sy++) {
      for (let sx = 0; sx < 4; sx++) {
        const px = x + ((sx + 0.5) / 4 - 0.5) * grid.cell;
        const py = y + ((sy + 0.5) / 4 - 0.5) * grid.cell;
        if (px >= b.minX && px < b.maxX && py >= b.minY && py < b.maxY) hit += 1;
      }
    }
    floor[k] = hit / 16;
    floorOf[k] = i;
  }
  const structure = new Map<string, number>();
  const addStructure = (k: number, m: number): void => {
    const id = world.buildings[floorOf[k]!]!.id;
    structure.set(id, (structure.get(id) ?? 0) + m);
  };
  const byStrategy = Object.fromEntries(STRATEGIES.map((s) => [s, 0])) as Record<Strategy, number>;
  let rest = 0;

  if (shape === 'uniform') {
    let fenceCells = 0;
    for (let k = 0; k < n; k++) fenceCells += inFence[k]!;
    let placed = 0;
    for (const k of grid.cells) {
      cell[k] = grid.free[k]! / fenceCells;
      placed += cell[k]!;
    }
    for (let k = 0; k < n; k++) {
      if (!floor[k]) continue;
      addStructure(k, floor[k]! / fenceCells);
      placed += floor[k]! / fenceCells;
    }
    rest = 1 - placed;
    for (const s of STRATEGIES) byStrategy[s] = weights[s] / total;
    return { ipp, cell, structure, rest, byStrategy };
  }

  const dense = new Float64Array(n);
  for (const strategy of STRATEGIES) {
    const w = weights[strategy] / total;
    if (w <= 0) continue;
    const ring = RING[strategy];
    // Mass of the bare ring inside the fence; the remainder is beyond it.
    let ringIn = 0;
    let shaped = 0;
    for (let k = 0; k < n; k++) {
      dense[k] = 0;
      if (!inFence[k]) continue;
      const [x, y] = cellCenter(grid, k);
      const rho = ringDensity(Math.hypot(x - ipp[0], y - ipp[1]), ring.median, ring.sigma) * area;
      ringIn += rho;
      const f = shape === 'behavior' ? feature(strategy, grid, k, ippGround) : 1;
      dense[k] = rho * f;
      shaped += dense[k]!;
    }
    const beyond = Math.max(0, 1 - ringIn);
    let inside = 0;
    if (shape === 'behavior' && strategy === 'shelter' && boxes.length) {
      // Part of the shelter mass goes into the buildings, nearer ones first.
      const near = boxes.map((box) => {
        const [cx, cy] = boxCenter(box);
        return ringDensity(Math.hypot(cx - ipp[0], cy - ipp[1]), ring.median, ring.sigma);
      });
      const sum = near.reduce((a, b) => a + b, 0) || 1;
      world.buildings.forEach((b, i) => {
        const m = w * SHELTER_INSIDE * (1 - beyond) * (near[i]! / sum);
        structure.set(b.id, (structure.get(b.id) ?? 0) + m);
        inside += m;
      });
    }
    const outsideShare = shape === 'behavior' && strategy === 'shelter' && boxes.length ? 1 - SHELTER_INSIDE : 1;
    const scale = (w * outsideShare * (1 - beyond)) / (shaped || 1);
    let placed = inside;
    for (let k = 0; k < n; k++) {
      if (!dense[k]) continue;
      const m = dense[k]! * scale;
      const f = grid.free[k]!;
      const g = floor[k]!;
      cell[k] += m * f;
      if (g > 0) addStructure(k, m * g);
      placed += m * (f + g);
      rest += m * Math.max(0, 1 - f - g);
    }
    rest += w * beyond;
    byStrategy[strategy] = placed;
  }
  return { ipp, cell, structure, rest, byStrategy };
}

// ------------------------------------------------------------ simulator

export interface LostPersonOutcome {
  strategy: Strategy;
  /** Where the walk ended. Inside a building when `building` is set. */
  xy: [number, number];
  building: number | null;
  state: PersonState;
  /** Meters walked. */
  walked: number;
}

interface WalkWorld {
  world: MissionWorld;
  blocked: GroundBox[];
  buildings: GroundBox[];
}

function blockedAt(w: WalkWorld, x: number, y: number): boolean {
  if (!containsPoint(w.world.outdoor.geofence, x, y)) return true;
  for (const box of w.blocked) if (x >= box.minX && x < box.maxX && y >= box.minY && y < box.maxY) return true;
  return false;
}

/** Terrain gradient by central differences. */
function gradient(world: MissionWorld, x: number, y: number): [number, number] {
  const t = world.outdoor.terrain;
  const h = 1;
  return [(sampleHeight(t, x + h, y) - sampleHeight(t, x - h, y)) / (2 * h), (sampleHeight(t, x, y + h) - sampleHeight(t, x, y - h)) / (2 * h)];
}

function lognormal(rng: RNG, median: number, sigma: number): number {
  return median * Math.exp(sigma * rng.gaussian());
}

/**
 * One 1 m step along `heading`, turning away from anything in the way. Returns
 * the new heading, or null when every direction is blocked.
 */
function stepFrom(w: WalkWorld, pos: [number, number], heading: number, rng: RNG): number | null {
  for (let k = 0; k < 12; k++) {
    const turn = k === 0 ? 0 : (Math.ceil(k / 2) * (k % 2 ? 1 : -1) * Math.PI) / 6 + rng.range(-0.2, 0.2);
    const h = heading + turn;
    const x = pos[0] + Math.cos(h);
    const y = pos[1] + Math.sin(h);
    if (blockedAt(w, x, y)) continue;
    pos[0] = x;
    pos[1] = y;
    return h;
  }
  return null;
}

function nearestTrailPoint(world: MissionWorld, x: number, y: number): { p: [number, number]; dir: [number, number]; d: number } | null {
  let best: { p: [number, number]; dir: [number, number]; d: number } | null = null;
  for (const c of world.outdoor.corridors) {
    const dx = c.bx - c.ax;
    const dy = c.by - c.ay;
    const l2 = dx * dx + dy * dy || 1;
    const t = Math.max(0, Math.min(1, ((x - c.ax) * dx + (y - c.ay) * dy) / l2));
    const p: [number, number] = [c.ax + dx * t, c.ay + dy * t];
    const d = distToSegment(x, y, c.ax, c.ay, c.bx, c.by);
    if (!best || d < best.d) {
      const l = Math.sqrt(l2);
      best = { p, dir: [dx / l, dy / l], d };
    }
  }
  return best;
}

function stateFor(strategy: Strategy, rng: RNG): PersonState {
  if (strategy === 'stay_put') return rng.bool(0.6) ? 'lying' : 'sitting';
  const u = rng.next();
  return u < 0.4 ? 'sitting' : u < 0.7 ? 'lying' : 'standing';
}

/**
 * Walk a lost person from the IPP. Deterministic for a seed. The person
 * stays inside the geofence: the benchmark measures searches that can
 * succeed, while the prior still keeps the mass it believes lies beyond.
 */
export function simulateLostPerson(world: MissionWorld, ipp: [number, number], seed: number, forced?: Strategy): LostPersonOutcome {
  const rng = new RNG(seed >>> 0, 'lost-person');
  const w: WalkWorld = { world, blocked: plantingObstacles(world.outdoor), buildings: world.buildings.map(buildingBoxOf) };
  let strategy: Strategy = forced ?? 'wander';
  if (!forced) {
    let u = rng.next();
    for (const s of STRATEGIES) {
      u -= STRATEGY_WEIGHT[s];
      if (u <= 0) {
        strategy = s;
        break;
      }
    }
  }
  const pos: [number, number] = [ipp[0], ipp[1]];
  let heading = rng.range(-Math.PI, Math.PI);
  let walked = 0;
  let building: number | null = null;
  const walk = (meters: number, steer: () => number | null): void => {
    for (let i = 0; i < meters; i++) {
      const want = steer();
      if (want !== null) {
        const diff = Math.atan2(Math.sin(want - heading), Math.cos(want - heading));
        heading += diff * 0.5;
      }
      heading += rng.gaussian() * 0.35;
      const h = stepFrom(w, pos, heading, rng);
      if (h === null) return;
      heading = h;
      walked += 1;
    }
  };

  switch (strategy) {
    case 'stay_put':
      walk(Math.floor(rng.range(0, 6)), () => null);
      break;
    case 'wander': {
      // Correlated random walk with a slight downhill pull.
      walk(Math.round(Math.min(60, lognormal(rng, 15, 0.6))), () => {
        const [gx, gy] = gradient(world, pos[0], pos[1]);
        return Math.hypot(gx, gy) > 0.08 && rng.bool(0.3) ? Math.atan2(-gy, -gx) : null;
      });
      break;
    }
    case 'view': {
      const meters = Math.round(Math.min(50, lognormal(rng, 14, 0.5)));
      walk(meters, () => {
        const [gx, gy] = gradient(world, pos[0], pos[1]);
        return Math.hypot(gx, gy) > 0.02 ? Math.atan2(gy, gx) : null;
      });
      break;
    }
    case 'trail': {
      const near = nearestTrailPoint(world, pos[0], pos[1]);
      if (!near || near.d > 18) {
        walk(Math.round(lognormal(rng, 12, 0.6)), () => null);
        break;
      }
      const to = near.p;
      walk(Math.ceil(near.d), () => Math.atan2(to[1] - pos[1], to[0] - pos[0]));
      const sign = rng.bool() ? 1 : -1;
      const along = Math.round(Math.min(45, lognormal(rng, 12, 0.6)));
      walk(along, () => {
        const t = nearestTrailPoint(world, pos[0], pos[1]);
        if (!t) return null;
        // Follow the trail direction, pulled back onto the centerline.
        const fx = t.dir[0] * sign * 2 + (t.p[0] - pos[0]);
        const fy = t.dir[1] * sign * 2 + (t.p[1] - pos[1]);
        return Math.atan2(fy, fx);
      });
      if (rng.bool(0.5)) walk(Math.floor(rng.range(2, 9)), () => null);
      break;
    }
    case 'shelter': {
      if (!w.buildings.length) {
        walk(Math.round(lognormal(rng, 12, 0.6)), () => null);
        break;
      }
      const order = w.buildings
        .map((box, i) => ({ i, d: Math.hypot(boxCenter(box)[0] - pos[0], boxCenter(box)[1] - pos[1]) }))
        .sort((a, b) => a.d - b.d);
      // Usually the nearest building, sometimes the one they happened to see.
      const pick = order[rng.bool(0.75) || order.length === 1 ? 0 : 1]!;
      const box = w.buildings[pick.i]!;
      const goal = boxCenter(box);
      for (let i = 0; i < 90; i++) {
        const dx = Math.max(box.minX - pos[0], 0, pos[0] - box.maxX);
        const dy = Math.max(box.minY - pos[1], 0, pos[1] - box.maxY);
        if (Math.hypot(dx, dy) < 2.6) break;
        walk(1, () => Math.atan2(goal[1] - pos[1], goal[0] - pos[0]));
      }
      if (rng.bool(0.65)) building = pick.i;
      break;
    }
  }
  return { strategy, xy: [pos[0], pos[1]], building, state: stateFor(strategy, rng), walked };
}

// ------------------------------------------------------------ planting

/**
 * A lying, sitting or standing person at a ground point, as the target
 * placement would make one. Nudged outward in a spiral when the body would
 * touch a trunk, bush, crown core, building margin or no-fly zone. Null when
 * no spot within 5 m fits.
 */
export function plantPersonAt(world: MissionWorld, xy: [number, number], state: PersonState, seed: number): PlacedTarget | null {
  const rng = new RNG(seed >>> 0, 'lost-person-plant');
  const size = PERSON_SIZE[state];
  const blocked = plantingObstacles(world.outdoor);
  const yaw = state === 'lying' ? (rng.bool() ? 0 : Math.PI / 2) : rng.range(-Math.PI, Math.PI);
  const fits = (x: number, y: number): boolean => {
    if (!containsPoint(world.outdoor.geofence, x, y)) return false;
    const box = bodyAabb([x, y, 0], yaw, size.hx, size.hy, size.height);
    return !blocked.some((o) => box.min[0] < o.maxX && box.max[0] > o.minX && box.min[1] < o.maxY && box.max[1] > o.minY);
  };
  for (let r = 0; r <= 5; r += 0.5) {
    const steps = r === 0 ? 1 : Math.ceil((2 * Math.PI * r) / 0.5);
    const a0 = rng.range(0, Math.PI * 2);
    for (let s = 0; s < steps; s++) {
      const a = a0 + (s / steps) * Math.PI * 2;
      const x = xy[0] + Math.cos(a) * r;
      const y = xy[1] + Math.sin(a) * r;
      if (!fits(x, y)) continue;
      return {
        id: `outdoor/person-${state}-lost`,
        classId: 'person',
        category: 'person',
        state,
        feet: [x, y, sampleHeight(world.outdoor.terrain, x, y)],
        yaw,
        hx: size.hx,
        hy: size.hy,
        height: size.height,
        speed: 0,
        floor: 0,
        roomIndex: -1,
        occluders: [],
      };
    }
  }
  return null;
}
