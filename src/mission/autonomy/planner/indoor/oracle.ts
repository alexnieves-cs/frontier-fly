/**
 * The 'oracle' reference: a camera-visibility estimate over the true floor-plan cells and a frontier
 * explorer that plans on them. Kept only as a reference arm; missions fly ./explorers.ts.
 */
import { type GridCell, indoorCells } from '../../../../perception/seenEmpty';
import { type V3 } from '../../contracts';
import { wrapAngle } from '../geometry';
import { type IndoorCell, type IndoorGrid, SEE_HALF_FOV, SEE_RANGE, cellCenter, cellsToPath, floorOf, gridDistances, gridLos, nearestNav, unkey } from './grid';

/** The explorer's own estimate of which seen-empty cells the camera has covered. */
export class IndoorVisibility {
  readonly cells: GridCell[];
  readonly seen: Uint8Array;
  private readonly cellXY: Int32Array;
  private count = 0;

  constructor(
    readonly grid: IndoorGrid,
    readonly range = SEE_RANGE,
    readonly halfFov = SEE_HALF_FOV,
  ) {
    this.cells = indoorCells(grid.spec);
    this.seen = new Uint8Array(this.cells.length);
    this.cellXY = new Int32Array(this.cells.length * 2);
    this.cells.forEach((cell, i) => {
      this.cellXY[i * 2] = Math.floor(cell.center[0] / grid.cs);
      this.cellXY[i * 2 + 1] = Math.floor(cell.center[1] / grid.cs);
    });
  }

  /** Mark cells seen from a local pose. Returns how many were new. */
  observe(local: V3, yaw: number): number {
    const f = floorOf(this.grid, local[2]);
    let fresh = 0;
    for (let i = 0; i < this.cells.length; i++) {
      if (this.seen[i]) continue;
      const cell = this.cells[i]!;
      if (cell.floor !== f) continue;
      const dx = cell.center[0] - local[0];
      const dy = cell.center[1] - local[1];
      const d = Math.hypot(dx, dy);
      if (d > this.range) continue;
      if (d > 0.7 && Math.abs(wrapAngle(Math.atan2(dy, dx) - yaw)) > this.halfFov) continue;
      if (!gridLos(this.grid, f, local[0], local[1], cell.center[0], cell.center[1])) continue;
      this.seen[i] = 1;
      fresh += 1;
    }
    this.count += fresh;
    return fresh;
  }

  coverage(): number {
    return this.cells.length ? this.count / this.cells.length : 1;
  }

  reset(): void {
    this.seen.fill(0);
    this.count = 0;
  }

  unseen(): number[] {
    const out: number[] = [];
    for (let i = 0; i < this.cells.length; i++) if (!this.seen[i]) out.push(i);
    return out;
  }
}

export interface IndoorGoal {
  /** Local waypoints to the goal cell. */
  path: V3[];
  /** Turn a full circle at the goal. */
  spin: boolean;
  /** Unseen cells the goal should reveal. */
  gain: number;
}

export class FrontierExplorer {
  private readonly tried = new Set<number>();

  constructor(
    readonly grid: IndoorGrid,
    readonly vis: IndoorVisibility,
    /** Radius a goal is credited with revealing, inside SEE_RANGE so a spin delivers it. */
    readonly gainRange = SEE_RANGE - 1,
  ) {}

  /** Next goal from a local position, or null when nothing reachable is left unseen. */
  next(local: V3): IndoorGoal | null {
    const start = nearestNav(this.grid, local);
    if (!start) return null;
    const unseen = this.vis.unseen();
    if (!unseen.length) return null;
    const { dist, prev } = gridDistances(this.grid, start);
    let bestK = -1;
    let bestScore = 0;
    let bestGain = 0;
    for (let k = 0; k < dist.length; k++) {
      const d = dist[k]!;
      if (!Number.isFinite(d) || this.tried.has(k)) continue;
      const c = unkey(this.grid, k);
      const p = cellCenter(this.grid, c);
      let gain = 0;
      for (const i of unseen) {
        const cell = this.vis.cells[i]!;
        if (cell.floor !== c.f) continue;
        const dx = cell.center[0] - p[0];
        const dy = cell.center[1] - p[1];
        if (dx * dx + dy * dy > this.gainRange * this.gainRange) continue;
        if (gridLos(this.grid, c.f, p[0], p[1], cell.center[0], cell.center[1])) gain += 1;
      }
      if (gain < 1) continue;
      const score = gain / (1 + 0.3 * d);
      if (score > bestScore + 1e-9) {
        bestScore = score;
        bestK = k;
        bestGain = gain;
      }
    }
    if (bestK < 0) return null;
    this.tried.add(bestK);
    const cells: IndoorCell[] = [];
    for (let k = bestK; k >= 0; k = prev[k]!) cells.push(unkey(this.grid, k));
    cells.reverse();
    const path: V3[] = [];
    const first = cellCenter(this.grid, start);
    if (Math.hypot(first[0] - local[0], first[1] - local[1]) > 0.05 || Math.abs(first[2] - local[2]) > 0.2) path.push(first);
    path.push(...cellsToPath(this.grid, cells));
    if (!path.length) path.push(first);
    return { path, spin: true, gain: bestGain };
  }
}
