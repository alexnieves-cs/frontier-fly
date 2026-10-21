import { cellIndex, sectorOf, skyVisible, type SearchGrid } from './grid';
import type { LostPersonPrior } from './lostPerson';
import { framePod, frameHalfExtents, PERSON_CENTER_M, type SensorConditions } from './pod';

/**
 * Bayesian search belief. Holds, for every grid cell, building and the rest
 * of world, the probability mass that the person is there and has not been
 * detected yet. A frame with no contact near a cell multiplies its mass by
 * (1 - POD); normalizing gives the posterior POA. The mass that has left the
 * map is the cumulative probability of success (POS) the search has earned
 * so far, under the model (Koopman's search theory; Stone, "Theory of Optimal
 * Search", 1975).
 */
export class SearchBelief {
  readonly mass: Float64Array;
  readonly structure: Map<string, number>;
  rest: number;
  /** Total mass at the start, 1 for a normalized prior. */
  readonly initial: number;
  frames = 0;

  constructor(
    readonly grid: SearchGrid,
    prior: LostPersonPrior,
    readonly sensor: SensorConditions,
  ) {
    this.mass = Float64Array.from(prior.cell);
    this.structure = new Map(prior.structure);
    this.rest = prior.rest;
    this.initial = this.remaining();
  }

  /** Mass not yet searched away: in the area, in buildings, and beyond reach. */
  remaining(): number {
    let s = this.rest;
    for (const k of this.grid.cells) s += this.mass[k]!;
    for (const m of this.structure.values()) s += m;
    return s;
  }

  /** Mass in the outdoor search area alone. */
  areaMass(): number {
    let s = 0;
    for (const k of this.grid.cells) s += this.mass[k]!;
    return s;
  }

  /** Cumulative probability of success under the model, 0..1. */
  pos(): number {
    return Math.max(0, 1 - this.remaining() / this.initial);
  }

  /** Posterior probability of area of a cell. */
  poa(k: number): number {
    return this.mass[k]! / Math.max(1e-12, this.remaining());
  }

  /**
   * Apply one gimbal frame with no person contact, except near the given
   * contact estimates, whose cells are left alone so a contact the tracker
   * later rejects does not erase the evidence there. Returns the mass removed.
   */
  observeFrame(x: number, y: number, z: number, yaw: number, speed: number, contacts: readonly [number, number][] = []): number {
    const g = this.grid;
    const { along: maxA, across: maxC } = frameHalfExtents(Math.max(0.5, z - minGround(g)));
    const reach = Math.hypot(maxA, maxC) + g.cell;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const i0 = Math.max(0, Math.floor((x - reach - g.x0) / g.cell));
    const i1 = Math.min(g.nx - 1, Math.floor((x + reach - g.x0) / g.cell));
    const j0 = Math.max(0, Math.floor((y - reach - g.y0) / g.cell));
    const j1 = Math.min(g.ny - 1, Math.floor((y + reach - g.y0) / g.cell));
    let removed = 0;
    for (let j = j0; j <= j1; j++) {
      for (let i = i0; i <= i1; i++) {
        const k = j * g.nx + i;
        if (!g.mask[k]) continue;
        const m = this.mass[k]!;
        if (m <= 0) continue;
        const cx = g.x0 + (i + 0.5) * g.cell;
        const cy = g.y0 + (j + 0.5) * g.cell;
        if (contacts.some(([px, py]) => Math.hypot(px - cx, py - cy) < 3)) continue;
        const dx = cx - x;
        const dy = cy - y;
        const along = dx * c + dy * s;
        const across = -dx * s + dy * c;
        const h = z - (g.ground[k]! + PERSON_CENTER_M);
        const flat = Math.hypot(dx, dy);
        const seen = skyVisible(g, k, sectorOf(Math.atan2(-dy, -dx)), h / Math.max(flat, 0.01));
        const p = framePod(h, along, across, speed, seen ? 0 : 1, this.sensor);
        if (p <= 0) continue;
        const d = m * p;
        this.mass[k] = m - d;
        removed += d;
      }
    }
    this.frames += 1;
    return removed;
  }

  /** A building searched with probability of detection `pod`. */
  observeStructure(id: string, pod: number): void {
    const m = this.structure.get(id);
    if (m !== undefined) this.structure.set(id, m * (1 - pod));
  }

  /** Index of the cell under a point, or -1. */
  cellAt(x: number, y: number): number {
    return cellIndex(this.grid, x, y);
  }
}

const groundCache = new WeakMap<SearchGrid, number>();

function minGround(g: SearchGrid): number {
  let v = groundCache.get(g);
  if (v === undefined) {
    v = Infinity;
    for (const k of g.cells) v = Math.min(v, g.ground[k]!);
    if (!Number.isFinite(v)) v = 0;
    groundCache.set(g, v);
  }
  return v;
}
