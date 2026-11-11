/**
 * Learned goal choice for the sensor planner (opt-in via SensorPlannerOptions.picker).
 *
 * The planner chooses its next goal by a hand-written rule: the frontier voxel
 * with the lowest path cost, minus a small bonus for frontiers next to flown
 * or blocked space, with hysteresis toward the current goal. A GoalPicker
 * replaces only that choice. The planner hands it one candidate per frontier
 * cluster (the cluster's voxel the rule likes best) plus the current goal,
 * each with a feature vector, and flies to whichever candidate it returns:
 * Dijkstra path on the drone's own map, braking filter, trajectory tracker and
 * watchdog are unchanged. Every feature comes from what the planner already
 * reads (its own maps, pose and measured velocity), never the floor plan.
 *
 * docs/QUAD_PICKER.md has the study this is for.
 */

/** Feature slots, in order. Values are scaled to roughly [0, 1] or [-1, 1]. */
export const PICK_FEATURES = [
  'pathCost', // Dijkstra cost to the candidate (voxel steps incl. wall penalties) / 40
  'straight', // straight-line distance, m / 10
  'ruleScore', // the planner's own score (cost - edge bonus [+ momentum]) / 40
  'edges', // flown or blocked 8-neighbours / 8
  'clusterSize', // log(1 + frontier voxels in the candidate's cluster) / 5
  'unknownNear', // share of voxels within 2 m that are neither seen nor flown
  'headingCos', // cos(angle between measured velocity and the first 1.5 m of the path), times speed / 2
  'momentumCost', // (1 - cos) * speed / 2: what reversing toward it costs
  'turns', // sum of corner angles of the string-pulled path / π, clamped to 4
  'trackTime', // estimated flight time under the tracker's speed profile, s / 20
  'laneOnArrival', // straight unflown traversable air past the candidate along the last leg, m / 6
  'isCurrent', // 1 for the planner's current goal
  'pathMeters', // string-pulled path length, m / 10
  'frontierNear', // share of frontier voxels among the 8 neighbours
  'timeLeft', // context: fraction of the step budget left (1 when unknown)
  'speed', // context: measured speed / 2 m/s
  'nCand', // context: candidates / MAX_CANDIDATES
  'bias', // 1
] as const;
export const PICK_DIM = PICK_FEATURES.length;
export const MAX_CANDIDATES = 13;

export interface PickCandidates {
  /** Number of candidates (≥ 1 when the picker is called). */
  n: number;
  /** n × PICK_DIM, row-major. */
  feats: Float32Array;
  /** The planner rule's raw score of each candidate (lower is better). */
  ruleScore: Float64Array;
  /** Index of the planner's current goal among the candidates, or -1. */
  current: number;
}

export interface GoalPicker {
  /** Called on every planner reset (a new episode). */
  reset?(): void;
  /** Index of the chosen candidate. */
  pick(c: PickCandidates): number;
}

/** The planner's own rule over the candidates: lowest score, with its 1.5-step hysteresis toward the current goal. */
export class RulePicker implements GoalPicker {
  pick(c: PickCandidates): number {
    let best = 0;
    for (let i = 1; i < c.n; i++) if (c.ruleScore[i]! < c.ruleScore[best]!) best = i;
    if (c.current >= 0 && c.ruleScore[c.current]! <= c.ruleScore[best]! + 1.5) return c.current;
    return best;
  }
}

/** Score = w · features; the highest score wins (deterministic), or a softmax sample at `temperature` > 0. */
export class LinearPicker implements GoalPicker {
  readonly w: Float64Array;
  temperature: number;
  private rand: () => number;
  constructor(w: ArrayLike<number>, temperature = 0, rand: () => number = Math.random) {
    if (w.length !== PICK_DIM) throw new Error(`LinearPicker needs ${PICK_DIM} weights, got ${w.length}`);
    this.w = Float64Array.from(w);
    this.temperature = temperature;
    this.rand = rand;
  }
  scores(c: PickCandidates, out = new Float64Array(c.n)): Float64Array {
    for (let i = 0; i < c.n; i++) {
      let s = 0;
      for (let j = 0; j < PICK_DIM; j++) s += this.w[j]! * c.feats[i * PICK_DIM + j]!;
      out[i] = s;
    }
    return out;
  }
  pick(c: PickCandidates): number {
    const s = this.scores(c);
    return this.temperature > 0 ? sampleSoftmax(s, c.n, this.temperature, this.rand) : argmax(s, c.n);
  }
}

export function argmax(s: ArrayLike<number>, n: number): number {
  let b = 0;
  for (let i = 1; i < n; i++) if (s[i]! > s[b]!) b = i;
  return b;
}

export function sampleSoftmax(s: ArrayLike<number>, n: number, temperature: number, rand: () => number): number {
  let m = -Infinity;
  for (let i = 0; i < n; i++) m = Math.max(m, s[i]!);
  let z = 0;
  const p = new Float64Array(n);
  for (let i = 0; i < n; i++) z += p[i] = Math.exp((s[i]! - m) / temperature);
  let u = rand() * z;
  for (let i = 0; i < n; i++) if ((u -= p[i]!) <= 0) return i;
  return n - 1;
}

/**
 * Linear weights that reproduce the planner rule up to its hysteresis: score = −ruleScore, with
 * the current goal credited 1.5 steps. A starting point for search (ties aside, it picks like RulePicker).
 */
export function ruleWeights(): Float64Array {
  const w = new Float64Array(PICK_DIM);
  w[PICK_FEATURES.indexOf('ruleScore')] = -40;
  w[PICK_FEATURES.indexOf('isCurrent')] = 1.5 + 1e-6;
  return w;
}

/** One hidden layer scorer: s_i = w2 · tanh(W1 x_i + b1). Row-major W1 (hidden × PICK_DIM). */
export interface MlpWeights {
  hidden: number;
  w1: number[];
  b1: number[];
  w2: number[];
}

export function initMlp(hidden: number, rand: () => number, scale = 0.3): MlpWeights {
  const g = () => (rand() * 2 - 1) * scale;
  return {
    hidden,
    w1: Array.from({ length: hidden * PICK_DIM }, g),
    b1: Array.from({ length: hidden }, g),
    w2: Array.from({ length: hidden }, () => g() / Math.sqrt(hidden)),
  };
}

/** Forward pass for one candidate; `h` receives the hidden activations. */
export function mlpScore(m: MlpWeights, x: ArrayLike<number>, xo: number, h: Float64Array): number {
  let s = 0;
  for (let j = 0; j < m.hidden; j++) {
    let a = m.b1[j]!;
    const r = j * PICK_DIM;
    for (let d = 0; d < PICK_DIM; d++) a += m.w1[r + d]! * x[xo + d]!;
    const t = Math.tanh(a);
    h[j] = t;
    s += m.w2[j]! * t;
  }
  return s;
}

/** Per-decision log of an episode, for learning from it. */
export interface PickRecorder {
  /** Tell the recorder the env step and coverage before the pilot acts. */
  now(step: number, coverage: number): void;
  take(): { steps: Int32Array; coverage: Float32Array; n: Int32Array; feats: Float32Array; chosen: Int32Array; logp: Float32Array };
}

export class MlpPicker implements GoalPicker {
  readonly m: MlpWeights;
  temperature: number;
  private rand: () => number;
  private h: Float64Array;
  private s = new Float64Array(MAX_CANDIDATES);
  private rec: { step: number; cov: number; steps: number[]; coverage: number[]; n: number[]; feats: number[]; chosen: number[]; logp: number[] } | null = null;
  constructor(m: MlpWeights, temperature = 0, rand: () => number = Math.random) {
    if (m.w1.length !== m.hidden * PICK_DIM) throw new Error('MlpPicker: w1 size does not match PICK_DIM');
    this.m = m;
    this.temperature = temperature;
    this.rand = rand;
    this.h = new Float64Array(m.hidden);
  }
  recorder(): PickRecorder {
    this.rec = { step: 0, cov: 0, steps: [], coverage: [], n: [], feats: [], chosen: [], logp: [] };
    return {
      now: (step, coverage) => {
        this.rec!.step = step;
        this.rec!.cov = coverage;
      },
      take: () => {
        const r = this.rec!;
        return {
          steps: Int32Array.from(r.steps),
          coverage: Float32Array.from(r.coverage),
          n: Int32Array.from(r.n),
          feats: Float32Array.from(r.feats),
          chosen: Int32Array.from(r.chosen),
          logp: Float32Array.from(r.logp),
        };
      },
    };
  }
  pick(c: PickCandidates): number {
    const s = this.s;
    for (let i = 0; i < c.n; i++) s[i] = mlpScore(this.m, c.feats, i * PICK_DIM, this.h);
    const T = this.temperature;
    const k = T > 0 ? sampleSoftmax(s, c.n, T, this.rand) : argmax(s, c.n);
    if (this.rec) {
      const r = this.rec;
      // A single candidate is no decision: nothing to learn from it.
      if (c.n > 1) {
        let m = -Infinity;
        for (let i = 0; i < c.n; i++) m = Math.max(m, s[i]! / (T || 1));
        let z = 0;
        for (let i = 0; i < c.n; i++) z += Math.exp(s[i]! / (T || 1) - m);
        r.steps.push(r.step);
        r.coverage.push(r.cov);
        r.n.push(c.n);
        for (let i = 0; i < MAX_CANDIDATES * PICK_DIM; i++) r.feats.push(i < c.n * PICK_DIM ? c.feats[i]! : 0);
        r.chosen.push(k);
        r.logp.push(s[k]! / (T || 1) - m - Math.log(z));
      }
    }
    return k;
  }
}
