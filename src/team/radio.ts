import { CELL, SURFACE } from '../shared/constants';
import type { RNG } from '../shared/rng';
import type { BuildingSpec } from '../shared/types';

/**
 * Indoor radio link model for a drone team: the multi-wall model (COST 231),
 * free-space loss plus a fixed loss for every wall and floor slab the straight
 * line between two radios crosses, with log-normal shadowing per packet.
 *
 * It reads the true floor plan, because the radio waves do. No drone ever
 * reads it: a drone only learns whether a packet arrived.
 */
export interface RadioParams {
  /** Transmit power, dBm. */
  txPower: number;
  /** Receiver sensitivity, dBm. A packet arrives when the received power, after shadowing, is above it. */
  sensitivity: number;
  /** Path loss at 1 m, dB (40 dB is free space at 2.4 GHz). */
  pl0: number;
  /** Path-loss exponent between walls (2 is free space; the walls are counted separately). */
  exponent: number;
  /** Shadowing standard deviation per packet, dB. */
  shadowing: number;
  /** Loss per floor slab crossed, dB. */
  floorLoss: number;
  /** Loss per wall crossed by surface class (SURFACE index), dB. */
  wallLoss: readonly number[];
}

/**
 * Low-power 2.4 GHz mesh radio. Wall losses are typical 2.4 GHz values from
 * ITU-R P.2040 and indoor measurement tables (drywall about 3 dB, concrete
 * 10 to 15 dB, glass 2 to 3 dB, metal more than 20 dB, a concrete floor slab
 * about 15 dB). The 0 dBm transmitter and -82 dBm sensitivity are a
 * deliberately weak radio, so that walls and range matter inside a 20 m
 * building the way they do for a small drone deep in a large one.
 */
export const DEFAULT_RADIO: RadioParams = {
  txPower: 0,
  sensitivity: -82,
  pl0: 40,
  exponent: 2,
  shadowing: 4,
  floorLoss: 15,
  wallLoss: [3, 12, 5, 2, 26],
};

export type Point3 = readonly [number, number, number];

export class RadioModel {
  readonly params: RadioParams;
  private readonly spec: BuildingSpec;

  constructor(spec: BuildingSpec, params: Partial<RadioParams> = {}) {
    this.spec = spec;
    this.params = { ...DEFAULT_RADIO, ...params };
  }

  /** Floor index of a height, clamped to the building; points outside count as the nearest floor. */
  private floorOf(z: number): number {
    const n = this.spec.floors.length;
    return Math.max(0, Math.min(n - 1, Math.floor(z / this.spec.floorHeight)));
  }

  /** Wall loss along the straight line from a to b on one floor, dB. A wall is counted once per entry. */
  wallLossDb(a: Point3, b: Point3, floor = this.floorOf(a[2])): number {
    const grid = this.spec.floors[floor]!;
    const cs = this.spec.cellSize;
    const ax = a[0] / cs;
    const ay = a[1] / cs;
    const dx = b[0] / cs - ax;
    const dy = b[1] / cs - ay;
    const len = Math.hypot(dx, dy);
    if (len < 1e-9) return 0;
    let cx = Math.floor(ax);
    let cy = Math.floor(ay);
    const ex = Math.floor(b[0] / cs);
    const ey = Math.floor(b[1] / cs);
    const sx = dx > 0 ? 1 : -1;
    const sy = dy > 0 ? 1 : -1;
    const tdx = dx !== 0 ? Math.abs(1 / dx) : Infinity;
    const tdy = dy !== 0 ? Math.abs(1 / dy) : Infinity;
    let tmx = dx > 0 ? (cx + 1 - ax) * tdx : dx < 0 ? (ax - cx) * tdx : Infinity;
    let tmy = dy > 0 ? (cy + 1 - ay) * tdy : dy < 0 ? (ay - cy) * tdy : Infinity;
    const loss = this.params.wallLoss;
    let total = 0;
    let inWall = this.wallAt(grid, cx, cy) >= 0;
    // A bounded walk: at most one step per cell boundary crossed.
    for (let guard = 0; guard < 4 * (grid.w + grid.d) + 8; guard++) {
      if (cx === ex && cy === ey) break;
      if (tmx < tmy) {
        if (tmx > 1) break;
        cx += sx;
        tmx += tdx;
      } else {
        if (tmy > 1) break;
        cy += sy;
        tmy += tdy;
      }
      const s = this.wallAt(grid, cx, cy);
      if (s >= 0 && !inWall) total += loss[s] ?? loss[SURFACE.DRYWALL]!;
      inWall = s >= 0;
    }
    return total;
  }

  /** Surface class of a wall or shaft cell, or -1 for open space (rooms, doors, outside the footprint). */
  private wallAt(grid: BuildingSpec['floors'][number], x: number, y: number): number {
    if (x < 0 || y < 0 || x >= grid.w || y >= grid.d) return -1;
    const i = y * grid.w + x;
    const c = grid.cells[i]!;
    if (c === CELL.WALL) return grid.surfaces?.[i] ?? SURFACE.DRYWALL;
    if (c === CELL.SHAFT) return SURFACE.METAL;
    return -1;
  }

  /** Mean link margin from a to b, dB: received power minus sensitivity, before shadowing. */
  margin(a: Point3, b: Point3): number {
    const p = this.params;
    const d = Math.max(1, Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]));
    const fa = this.floorOf(a[2]);
    const fb = this.floorOf(b[2]);
    // Walls on the transmitter's floor; slabs between the two floors.
    const walls = this.wallLossDb(a, b, fa);
    const slabs = Math.abs(fa - fb) * p.floorLoss;
    return p.txPower - (p.pl0 + 10 * p.exponent * Math.log10(d)) - walls - slabs - p.sensitivity;
  }

  /** One packet from a to b: true when it arrives. */
  deliver(a: Point3, b: Point3, rng: RNG): boolean {
    return this.margin(a, b) + this.params.shadowing * rng.gaussian() > 0;
  }

  /** Probability a packet arrives, given the mean margin (shadowing is Gaussian in dB). */
  deliveryProbability(a: Point3, b: Point3): number {
    const s = this.params.shadowing;
    const m = this.margin(a, b);
    return s > 0 ? normalCdf(m / s) : m > 0 ? 1 : 0;
  }
}

/** Standard normal CDF (Abramowitz and Stegun 7.1.26, error below 1.5e-7). */
export function normalCdf(x: number): number {
  const t = 1 / (1 + 0.3275911 * Math.abs(x) / Math.SQRT2);
  const y = 1 - ((((1.061405429 * t - 1.453152027) * t + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-(x * x) / 2);
  return x >= 0 ? 0.5 * (1 + y) : 0.5 * (1 - y);
}

/**
 * Where the team's base radio stands: outside the building, `offset` meters
 * past the outer wall nearest the entry point, at 1 m height. That is where an
 * operator would set up before sending the drones in.
 */
export function basePosition(spec: BuildingSpec, offset = 3): [number, number, number] {
  const f = spec.floors[0]!;
  const w = f.w * spec.cellSize;
  const d = f.d * spec.cellSize;
  const [x, y] = spec.spawn;
  const gaps = [x, w - x, y, d - y];
  const side = gaps.indexOf(Math.min(...gaps));
  if (side === 0) return [-offset, y, 1];
  if (side === 1) return [w + offset, y, 1];
  if (side === 2) return [x, -offset, 1];
  return [x, d + offset, 1];
}
