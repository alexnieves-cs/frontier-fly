/**
 * Sensor-footprint coverage: the share of the building's reachable volume
 * that came within a sensor's view, the way exploration papers score
 * coverage. The env's own coverage counts the voxels the drone *center* flew
 * through, which rewards sweeping lanes 0.5 m apart; this counts what a
 * sensor of radius R on the drone could see, so a drone that inspects a room
 * from its doorway gets credit for the room.
 *
 * The sensor is a horizontal 360° ring of ground-truth rays at the drone's
 * altitude (the airframe's lidar plane), cast against the true geometry with
 * no noise, so lighting and sensor dropouts never change the score. Walls and
 * furniture block the view. Only voxels that count toward the flown-coverage
 * denominator (BuildingSpec.reachableVoxels) can be marked, so both metrics
 * share a denominator and 100% means the same building.
 *
 * Several radii are tracked from one set of rays. 3 m is the headline: it is
 * about where the detector's recall falls to half for the smallest search
 * items (phone, radio: d50 3.2-3.6 m, perception/model.ts). 1.5 m and 8 m
 * (the lidar's range) bracket it.
 *
 * This is an evaluation metric only. It does not feed any pilot, observation
 * or reward, so checkpoints and published flown-coverage numbers are unchanged.
 */
import { VOXEL } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';
import { flightMap } from '../gen/validate';
import type { PhysicsWorld } from '../sim/world';

export const FOOTPRINT_RADII = [1.5, 3, 8] as const;
/** The headline radius, meters. */
export const FOOTPRINT_RADIUS = 3;
/** Rays in the ring: 4° apart, 0.2 m between neighbours at 3 m, under half a voxel. */
export const FOOTPRINT_RAYS = 90;
/** Points on a coverage-over-time curve: coverage at k / CURVE_POINTS of the step budget, k = 1..CURVE_POINTS. */
export const CURVE_POINTS = 20;

/** Written into each study's JSON so the numbers say how they were measured. */
export const FOOTPRINT_META = {
  radii: [...FOOTPRINT_RADII],
  headline: FOOTPRINT_RADIUS,
  rays: FOOTPRINT_RAYS,
  curvePoints: CURVE_POINTS,
  sensor: 'horizontal 360-degree ground-truth ring at the drone altitude; walls and furniture block it; reachable voxels only',
};

const RAY_COS = new Float64Array(FOOTPRINT_RAYS);
const RAY_SIN = new Float64Array(FOOTPRINT_RAYS);
for (let i = 0; i < FOOTPRINT_RAYS; i++) {
  RAY_COS[i] = Math.cos((i / FOOTPRINT_RAYS) * Math.PI * 2);
  RAY_SIN[i] = Math.sin((i / FOOTPRINT_RAYS) * Math.PI * 2);
}

export class FootprintCoverage {
  readonly radii: readonly number[];
  /** Per radius: 1 where a reachable voxel has been inside the footprint. Layout of the env's visited grid. */
  private seen: Uint8Array[] = [];
  private counts: number[] = [];
  private reachable: Uint8Array = new Uint8Array(0);
  private total = 0;
  private vw = 0;
  private vd = 0;
  private floors = 0;
  private floorHeight = 1;

  constructor(radii: readonly number[] = FOOTPRINT_RADII) {
    this.radii = [...radii].sort((a, b) => a - b);
  }

  reset(spec: BuildingSpec): void {
    const map = flightMap(spec);
    this.reachable = map.reachable;
    this.total = spec.reachableVoxels;
    const floor0 = spec.floors[0]!;
    this.vw = floor0.w * 2;
    this.vd = floor0.d * 2;
    this.floors = spec.floors.length;
    this.floorHeight = spec.floorHeight;
    const n = this.floors * this.vw * this.vd;
    this.seen = this.radii.map(() => new Uint8Array(n));
    this.counts = this.radii.map(() => 0);
  }

  /** Coverage at radius index `r` (default: the headline radius). */
  coverage(r = this.radii.indexOf(FOOTPRINT_RADIUS)): number {
    return this.total > 0 ? this.counts[r]! / this.total : 0;
  }

  /** Coverage at every radius, in `radii` order. */
  all(): number[] {
    return this.counts.map((c) => (this.total > 0 ? c / this.total : 0));
  }

  /** Mark what the ring sees from (x, y, z). */
  observe(world: PhysicsWorld, x: number, y: number, z: number): void {
    const floor = Math.floor(z / this.floorHeight);
    if (floor < 0 || floor >= this.floors || this.vw === 0) return;
    const vw = this.vw;
    const vd = this.vd;
    const base = floor * vw * vd;
    const reachable = this.reachable;
    const radii = this.radii;
    const nr = radii.length;
    const maxR = radii[nr - 1]!;
    const ox = x / VOXEL;
    const oy = y / VOXEL;
    for (let i = 0; i < FOOTPRINT_RAYS; i++) {
      const dx = RAY_COS[i]!;
      const dy = RAY_SIN[i]!;
      const hit = world.cast(x, y, z, dx, dy, 0, maxR);
      const end = (hit ? hit.dist : maxR) / VOXEL;
      // Voxel DDA from the drone to the return; t is the entry distance into the current voxel, in voxels.
      let vx = Math.floor(ox);
      let vy = Math.floor(oy);
      const stepX = dx > 0 ? 1 : -1;
      const stepY = dy > 0 ? 1 : -1;
      const tdx = Math.abs(dx) > 1e-12 ? Math.abs(1 / dx) : Infinity;
      const tdy = Math.abs(dy) > 1e-12 ? Math.abs(1 / dy) : Infinity;
      let tmx = dx > 0 ? (vx + 1 - ox) * tdx : dx < 0 ? (ox - vx) * tdx : Infinity;
      let tmy = dy > 0 ? (vy + 1 - oy) * tdy : dy < 0 ? (oy - vy) * tdy : Infinity;
      let t = 0;
      for (;;) {
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) break;
        const k = base + vy * vw + vx;
        if (reachable[k] === 1) {
          const meters = t * VOXEL;
          for (let r = nr - 1; r >= 0; r--) {
            if (meters > radii[r]!) break;
            const grid = this.seen[r]!;
            if (grid[k] === 0) {
              grid[k] = 1;
              this.counts[r]! += 1;
            }
          }
        }
        if (tmx < tmy) {
          if (tmx > end) break;
          t = tmx;
          vx += stepX;
          tmx += tdx;
        } else {
          if (tmy > end) break;
          t = tmy;
          vy += stepY;
          tmy += tdy;
        }
      }
    }
  }
}

/**
 * Samples coverage at fixed fractions of the step budget, so episodes that
 * stop early (success, stall) hold their last value and every curve has the
 * same x axis. Call `step(stepIndex, values)` after each env step and
 * `finish(values)` once the episode ends.
 */
export class CoverageCurve {
  readonly points: number[][] = [];
  private next = 1;

  constructor(
    private readonly maxSteps: number,
    private readonly n = CURVE_POINTS,
  ) {}

  /** `steps` is the number of steps taken so far (1-based). */
  step(steps: number, values: number[]): void {
    while (this.next <= this.n && steps >= Math.round((this.next * this.maxSteps) / this.n)) {
      this.points.push(values.slice());
      this.next += 1;
    }
  }

  finish(values: number[]): void {
    while (this.next <= this.n) {
      this.points.push(values.slice());
      this.next += 1;
    }
  }

  /** Column `c` of the curve (one series). */
  series(c: number): number[] {
    return this.points.map((p) => p[c]!);
  }
}

/** Normalized area under a coverage-time curve sampled at k/n of the budget: 1 means full coverage from the first instant. */
export function curveArea(series: number[]): number {
  return series.length ? series.reduce((a, b) => a + b, 0) / series.length : 0;
}

/** Seconds until a series first reaches `level`, interpolating between samples; null if never. */
export function timeTo(series: number[], level: number, budgetS: number): number | null {
  const n = series.length;
  let prev = 0;
  for (let k = 0; k < n; k++) {
    const v = series[k]!;
    if (v >= level) {
      const t0 = (k / n) * budgetS;
      const t1 = ((k + 1) / n) * budgetS;
      const f = v > prev ? (level - prev) / (v - prev) : 1;
      return t0 + f * (t1 - t0);
    }
    prev = v;
  }
  return null;
}

/** What a study stores per episode, beside the flown coverage it already had. */
export interface FootprintRecord {
  /** Footprint coverage at the end, per radius: { '1.5': .., '3': .., '8': .. }. */
  footprint: Record<string, number>;
  /** Coverage-over-time, CURVE_POINTS samples at k / CURVE_POINTS of the step budget: flown, and footprint at FOOTPRINT_RADIUS. */
  curve: { flown: number[]; footprint: number[] };
}

/** The env surface the tracker needs: the true geometry and the drone's position. */
export interface FootprintEnv {
  readonly world: PhysicsWorld | null;
  readonly position: ArrayLike<number>;
}

const round4 = (v: number) => Math.round(v * 1e4) / 1e4;

/**
 * Footprint coverage plus both coverage-over-time curves for one episode.
 * `start` right after env.reset, `step` after every env.step with the
 * flown coverage the env reported, `finish` once at the end.
 */
export class EpisodeFootprint {
  readonly fp = new FootprintCoverage();
  private curve = new CoverageCurve(1);
  private head = 0;

  start(env: FootprintEnv, spec: BuildingSpec, maxSteps: number): void {
    this.fp.reset(spec);
    this.curve = new CoverageCurve(maxSteps);
    this.head = this.fp.radii.indexOf(FOOTPRINT_RADIUS);
    this.observe(env);
  }

  step(env: FootprintEnv, steps: number, flown: number): void {
    this.observe(env);
    this.curve.step(steps, [flown, this.fp.coverage(this.head)]);
  }

  finish(flown: number): FootprintRecord {
    this.curve.finish([flown, this.fp.coverage(this.head)]);
    const all = this.fp.all();
    const footprint: Record<string, number> = {};
    this.fp.radii.forEach((r, i) => (footprint[String(r)] = round4(all[i]!)));
    return { footprint, curve: { flown: this.curve.series(0).map(round4), footprint: this.curve.series(1).map(round4) } };
  }

  private observe(env: FootprintEnv): void {
    const world = env.world;
    if (!world) return;
    const p = env.position;
    this.fp.observe(world, p[0]!, p[1]!, p[2]!);
  }
}

/**
 * Per-cell footprint summary for a study: mean footprint coverage per radius,
 * mean coverage-over-time curves, the normalized area under each curve, and
 * the mean time to cover half the building (over the episodes that did).
 * `ci` computes a confidence interval (the studies pass their bootstrap).
 */
export function summarizeFootprint(rows: FootprintRecord[], budgetS: number, ci: (xs: number[]) => [number, number]) {
  const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
  const radii = rows.length ? Object.keys(rows[0]!.footprint) : [];
  const footprint: Record<string, number> = {};
  for (const r of radii) footprint[r] = round4(mean(rows.map((e) => e.footprint[r]!)));
  const head = rows.map((e) => e.footprint[String(FOOTPRINT_RADIUS)]!);
  const meanCurve = (pick: (e: FootprintRecord) => number[]) =>
    Array.from({ length: CURVE_POINTS }, (_, k) => round4(mean(rows.map((e) => pick(e)[k] ?? 0))));
  const half = (pick: (e: FootprintRecord) => number[]) => {
    const ts = rows.map((e) => timeTo(pick(e), 0.5, budgetS)).filter((t): t is number => t !== null);
    return { reached: ts.length, s: ts.length ? Math.round(mean(ts) * 10) / 10 : null };
  };
  return {
    footprint,
    footprintCi: ci(head).map(round4) as [number, number],
    curve: { budgetS, flown: meanCurve((e) => e.curve.flown), footprint: meanCurve((e) => e.curve.footprint) },
    area: { flown: round4(mean(rows.map((e) => curveArea(e.curve.flown)))), footprint: round4(mean(rows.map((e) => curveArea(e.curve.footprint)))) },
    half: { flown: half((e) => e.curve.flown), footprint: half((e) => e.curve.footprint) },
  };
}

/** An episode row without its curves (kept in shard files, dropped from the merged JSON to keep it small). */
export function withoutCurve<T extends Partial<FootprintRecord>>(row: T): Omit<T, 'curve'> {
  const rest = { ...row };
  delete rest.curve;
  return rest;
}
