/**
 * Ground-truth hidden gain, for training the map predictor (sim/mapPredict.ts)
 * and as an oracle reference. Privileged: it reads the building's true flight
 * map, so nothing that flies may use it except as a labelled upper bound.
 */
import { flightMap } from '../gen/validate';
import type { BuildingSpec } from '../shared/types';
import { classifyFloor, GAIN_RADIUS, type GainMaps, type GainSource } from '../sim/mapPredict';

const DX = [1, -1, 0, 0, 1, 1, -1, -1];
const DY = [0, 0, 1, -1, 1, -1, 1, -1];

/** BFS scratch reused across labels. */
export class GainLabeler {
  private stamp = new Int32Array(0);
  private depth = new Int16Array(0);
  private queue = new Int32Array(0);
  private gen = 0;

  /**
   * Flyable voxels (true flight map) the drone has neither seen nor flown,
   * within GAIN_RADIUS steps of (x, y) through flyable voxels, 8-connected
   * without corner cutting. `reach` and `cls` are one floor plane.
   */
  label(reach: Uint8Array, cls: Uint8Array, vw: number, vd: number, x: number, y: number, radius = GAIN_RADIUS): number {
    const n = vw * vd;
    if (this.stamp.length !== n) {
      this.stamp = new Int32Array(n);
      this.depth = new Int16Array(n);
      this.queue = new Int32Array(n);
      this.gen = 0;
    }
    this.gen += 1;
    const g = this.gen;
    const start = y * vw + x;
    if (!reach[start]) return 0;
    let head = 0;
    let tail = 0;
    this.queue[tail++] = start;
    this.stamp[start] = g;
    this.depth[start] = 0;
    let count = 0;
    while (head < tail) {
      const k = this.queue[head++]!;
      if (cls[k] === 3) count += 1;
      const d = this.depth[k]!;
      if (d >= radius) continue;
      const kx = k % vw;
      const ky = (k - kx) / vw;
      for (let i = 0; i < 8; i++) {
        const nx = kx + DX[i]!;
        const ny = ky + DY[i]!;
        if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
        const nk = ny * vw + nx;
        if (!reach[nk] || this.stamp[nk] === g) continue;
        if (i >= 4 && (!reach[ky * vw + nx] || !reach[ny * vw + kx])) continue;
        this.stamp[nk] = g;
        this.depth[nk] = d + 1;
        this.queue[tail++] = nk;
      }
    }
    return count;
  }
}

/** One floor plane of the true flight map. */
export function reachablePlane(spec: BuildingSpec, floor: number): { reach: Uint8Array; vw: number; vd: number } {
  const map = flightMap(spec);
  const plane = map.vw * map.vd;
  return { reach: map.reachable.subarray(floor * plane, (floor + 1) * plane), vw: map.vw, vd: map.vd };
}

/** The true hidden gain as a gain source: the upper bound a perfect predictor would reach. */
export class OracleGain implements GainSource {
  readonly max = 160;
  private readonly labeler = new GainLabeler();
  private cls = new Uint8Array(0);
  private cache = new Float32Array(0);
  private reach = new Uint8Array(0);
  private vw = 1;
  private vd = 1;

  constructor(private readonly spec: BuildingSpec) {}

  prepare(maps: GainMaps): void {
    const n = maps.vw * maps.vd;
    if (this.cls.length !== n) {
      this.cls = new Uint8Array(n);
      this.cache = new Float32Array(n);
    }
    this.vw = maps.vw;
    this.vd = maps.vd;
    classifyFloor(maps, this.cls);
    this.reach = reachablePlane(this.spec, Math.round(maps.base / n)).reach;
    this.cache.fill(-1);
  }

  gain(k: number): number {
    const c = this.cache[k]!;
    if (c >= 0) return c;
    const x = k % this.vw;
    const g = Math.min(this.max, this.labeler.label(this.reach, this.cls, this.vw, this.vd, x, (k - x) / this.vw));
    this.cache[k] = g;
    return g;
  }
}
