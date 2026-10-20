/**
 * Learned map prediction for frontier selection.
 *
 * The sensor-only planner (sensorPlanner.ts) picks the cheapest frontier by
 * path length. It cannot tell a frontier that opens into a room from one that
 * grazes a closet, because what lies behind both is unknown. This module
 * predicts, from the drone's own maps around a candidate voxel, how many
 * flyable voxels it has not seen yet lie within a short flight of it (its
 * hidden gain). The planner can subtract that gain from the path cost.
 *
 * The predictor is a small MLP on a two-scale patch of the drone's maps. It is
 * trained offline on buildings generated from seeds no evaluation uses
 * (scripts/study/mappred.ts), with labels from the building's true flight map;
 * at run time it reads only the drone's maps, so the planner stays sensor-only.
 */
/** Voxels per patch side at each scale. */
export const GAIN_PATCH = 11;
/** Scale s pools 2^s × 2^s voxels per patch cell (0.5 m, 1 m, 2 m). */
export const GAIN_SCALES = 3;
/** Per patch cell: flyable-known, flown, unknown, inside the building footprint. */
export const GAIN_CHANNELS = 4;
export const GAIN_INPUT = GAIN_PATCH * GAIN_PATCH * GAIN_SCALES * GAIN_CHANNELS;
/** Geodesic radius of the gain label, voxels (4 m). */
export const GAIN_RADIUS = 8;
/** Labels are divided by this so targets sit near 0..1. */
export const GAIN_SCALE = 60;

/** The drone's own maps on one floor plane, as the sensor planner sees them. */
export interface GainMaps {
  vw: number;
  vd: number;
  /** Floor base index into seen/visited/occupied (floor * vw * vd). */
  base: number;
  seen: Uint8Array;
  visited: Uint8Array;
  occupied?: Uint8Array;
}

/**
 * Per-voxel class on one floor: 0 out of bounds or blocked, 1 known flyable,
 * 2 flown, 3 unknown (neither seen nor flown, no occupancy evidence).
 */
export function classifyFloor(m: GainMaps, out: Uint8Array): Uint8Array {
  const n = m.vw * m.vd;
  for (let k = 0; k < n; k++) {
    const i = m.base + k;
    if (m.visited[i] === 1) out[k] = 2;
    else if (m.occupied && m.occupied[i] === 1) out[k] = 0;
    else if (m.seen[i] === 1) out[k] = 1;
    else out[k] = 3;
  }
  return out;
}

/**
 * Summed-area tables of each class, so a pooled patch cell is four reads.
 * Layout: (vw + 1) × (vd + 1) per channel, channels 0..2 = known, flown, unknown.
 */
export class ClassIntegral {
  vw = 0;
  vd = 0;
  sums = new Float32Array(0);

  build(cls: Uint8Array, vw: number, vd: number): void {
    this.vw = vw;
    this.vd = vd;
    const W = vw + 1;
    const plane = W * (vd + 1);
    if (this.sums.length !== plane * 3) this.sums = new Float32Array(plane * 3);
    const s = this.sums;
    s.fill(0);
    for (let y = 0; y < vd; y++) {
      for (let x = 0; x < vw; x++) {
        const c = cls[y * vw + x]!;
        for (let ch = 0; ch < 3; ch++) {
          const v = c === ch + 1 ? 1 : 0;
          const o = ch * plane;
          s[o + (y + 1) * W + x + 1] = v + s[o + y * W + x + 1]! + s[o + (y + 1) * W + x]! - s[o + y * W + x]!;
        }
      }
    }
  }

  /** Count of class ch+1 voxels in [x0, x1) × [y0, y1), clipped to the grid. */
  count(ch: number, x0: number, y0: number, x1: number, y1: number): number {
    const ax = Math.max(0, x0);
    const ay = Math.max(0, y0);
    const bx = Math.min(this.vw, x1);
    const by = Math.min(this.vd, y1);
    if (bx <= ax || by <= ay) return 0;
    const W = this.vw + 1;
    const o = ch * W * (this.vd + 1);
    const s = this.sums;
    return s[o + by * W + bx]! - s[o + ay * W + bx]! - s[o + by * W + ax]! + s[o + ay * W + ax]!;
  }
}

/**
 * Patch features around voxel (cx, cy). Each scale is a GAIN_PATCH² window of
 * pooled cells centred on the voxel; a cell holds the fraction of its voxels
 * in each class and the fraction inside the grid. With `sym` (0..7) the patch
 * is read through one of the eight grid symmetries, for training augmentation.
 */
export function gainFeatures(ii: ClassIntegral, cx: number, cy: number, out: Float32Array, sym = 0): Float32Array {
  const P = GAIN_PATCH;
  const h = (P - 1) / 2;
  let o = 0;
  for (let s = 0; s < GAIN_SCALES; s++) {
    const c = 1 << s;
    const area = c * c;
    for (let py = 0; py < P; py++) {
      for (let px = 0; px < P; px++) {
        // Patch cell (px, py) → offset from centre, through the symmetry.
        let u = px - h;
        let v = py - h;
        if (sym & 1) u = -u;
        if (sym & 2) v = -v;
        if (sym & 4) {
          const t = u;
          u = v;
          v = t;
        }
        // Cell u covers voxels [cx + u*c - c/2, ...) so the centre voxel sits in cell 0 at every scale.
        const x0 = cx + u * c - (c >> 1);
        const y0 = cy + v * c - (c >> 1);
        const x1 = x0 + c;
        const y1 = y0 + c;
        const inside = (Math.min(ii.vw, x1) - Math.max(0, x0)) * (Math.min(ii.vd, y1) - Math.max(0, y0));
        out[o++] = ii.count(0, x0, y0, x1, y1) / area;
        out[o++] = ii.count(1, x0, y0, x1, y1) / area;
        out[o++] = ii.count(2, x0, y0, x1, y1) / area;
        out[o++] = Math.max(0, inside) / area;
      }
    }
  }
  return out;
}

/** Trained predictor weights: dense layers, row-major [in][out], ReLU between. */
export interface GainModelData {
  version: 1;
  input: number;
  hidden: number[];
  /** Per layer: weights (in × out) then biases (out). */
  layers: { w: Float32Array; b: Float32Array }[];
  /** Validation metrics recorded at training time. */
  meta?: Record<string, number | string>;
}

/** Pure-TS forward pass of the gain MLP. */
export class GainModel {
  private readonly acts: Float32Array[];

  constructor(readonly data: GainModelData) {
    this.acts = [...data.hidden, 1].map((n) => new Float32Array(n));
  }

  /** Predicted hidden gain in voxels (≥ 0). */
  predict(x: Float32Array): number {
    let input: Float32Array = x;
    const L = this.data.layers.length;
    for (let l = 0; l < L; l++) {
      const { w, b } = this.data.layers[l]!;
      const out = this.acts[l]!;
      const nOut = out.length;
      const nIn = input.length;
      out.set(b);
      for (let i = 0; i < nIn; i++) {
        const xi = input[i]!;
        if (xi === 0) continue;
        const row = i * nOut;
        for (let j = 0; j < nOut; j++) out[j] += xi * w[row + j]!;
      }
      if (l < L - 1) for (let j = 0; j < nOut; j++) if (out[j]! < 0) out[j] = 0;
      input = out;
    }
    return Math.max(0, input[0]!) * GAIN_SCALE;
  }
}

/** Serialized form: JSON header with base64 float32 per layer. */
export function serializeGainModel(m: GainModelData): string {
  const b64 = (a: Float32Array) => Buffer.from(a.buffer, a.byteOffset, a.byteLength).toString('base64');
  return JSON.stringify({ ...m, layers: m.layers.map((l) => ({ w: b64(l.w), b: b64(l.b) })) });
}

export function parseGainModel(text: string): GainModelData {
  const raw = JSON.parse(text) as Omit<GainModelData, 'layers'> & { layers: { w: string; b: string }[] };
  const f32 = (s: string) => {
    const bin = typeof atob === 'function' ? atob(s) : Buffer.from(s, 'base64').toString('binary');
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    return new Float32Array(bytes.buffer);
  };
  return { ...raw, layers: raw.layers.map((l) => ({ w: f32(l.w), b: f32(l.b) })) };
}

/**
 * Non-learned baseline: unknown voxels within the label radius (as the crow
 * flies), the classic information-gain proxy. Same units as the model.
 */
export function unknownGain(ii: ClassIntegral, cx: number, cy: number): number {
  const r = GAIN_RADIUS;
  return ii.count(2, cx - r, cy - r, cx + r + 1, cy + r + 1) * (Math.PI / 4);
}

/**
 * Hidden-gain estimate for frontier candidates on one floor, refreshed once
 * per replan. `gain(k)` is in voxels; `max` bounds it, so the planner knows
 * how far past its cheapest candidate a better one could lie.
 */
export interface GainSource {
  prepare(maps: GainMaps): void;
  gain(k: number): number;
  readonly max: number;
}

/** Shared per-replan state: voxel classes, their integrals, and a per-voxel cache. */
abstract class GridGain implements GainSource {
  abstract readonly max: number;
  protected cls = new Uint8Array(0);
  protected ii = new ClassIntegral();
  protected cache = new Float32Array(0);
  protected vw = 1;

  prepare(maps: GainMaps): void {
    const n = maps.vw * maps.vd;
    if (this.cls.length !== n) {
      this.cls = new Uint8Array(n);
      this.cache = new Float32Array(n);
    }
    this.vw = maps.vw;
    classifyFloor(maps, this.cls);
    this.ii.build(this.cls, maps.vw, maps.vd);
    this.cache.fill(-1);
  }

  gain(k: number): number {
    const c = this.cache[k]!;
    if (c >= 0) return c;
    const x = k % this.vw;
    const y = (k - x) / this.vw;
    // Nothing unknown within reach: nothing hidden to gain.
    const r = GAIN_RADIUS;
    const g = this.ii.count(2, x - r, y - r, x + r + 1, y + r + 1) > 0 ? Math.min(this.max, this.estimate(x, y)) : 0;
    this.cache[k] = g;
    return g;
  }

  protected abstract estimate(x: number, y: number): number;
}

/** The learned predictor as a gain source. */
export class LearnedGain extends GridGain {
  readonly max: number;
  private readonly x = new Float32Array(GAIN_INPUT);
  private readonly model: GainModel;

  constructor(data: GainModelData, max = 120) {
    super();
    this.model = new GainModel(data);
    this.max = max;
  }

  protected estimate(x: number, y: number): number {
    return this.model.predict(gainFeatures(this.ii, x, y, this.x));
  }
}

/** The unknown-area baseline as a gain source. */
export class UnknownGain extends GridGain {
  readonly max = Math.PI * (GAIN_RADIUS + 0.5) ** 2;

  protected estimate(x: number, y: number): number {
    return unknownGain(this.ii, x, y);
  }
}
