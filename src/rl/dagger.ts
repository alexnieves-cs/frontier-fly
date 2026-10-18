/**
 * DAgger for the indoor explorer.
 *
 * The privileged frontier teacher (sim/frontier.ts) labels every state the
 * rollout visits, while a beta-mixture decides who actually flies: the teacher
 * with probability beta, otherwise the student (deterministic, optionally with
 * light action noise). States where the student disagrees with the teacher or
 * is stalled are up-weighted, so the retrained student learns to recover from
 * its own mistakes (corners, dead ends) instead of only the teacher's clean
 * trajectories.
 *
 * collectDagger() takes only plain inputs and returns typed arrays that own
 * their buffers, so it can run unchanged inside a worker_thread and post its
 * result with daggerTransferables(result) as the transfer list.
 */
import { generateBuilding } from '../gen/building';
import { STAGE0_SEEDS, STAGE3_SEEDS } from '../eval/coverage';
import { ACT_DIM, HIDDEN, OBS_DIM, OBS_VERSION } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec, EpisodeConfig } from '../shared/types';
import { createEnv, defaultEpisodeConfig, type ExplorerEnv } from '../sim/env';
import { frontierAction } from '../sim/frontier';
import { lightingPreset, type LightingPreset } from '../sim/lighting';
import { checkpointMismatch, type Checkpoint } from './checkpoint';
import { Curriculum } from './curriculum';
import { InferPolicy } from './infer';
import { TfTrainer, type BcData } from './model';

/** Anything that can produce actions the way InferPolicy.actWith does. */
export interface StudentPolicy {
  actWith(obs: Float32Array, n: number, rng: RNG, deterministic?: boolean): { action: Float32Array };
}

export interface DaggerWeighting {
  /** Weight of an ordinary labelled state. */
  base: number;
  /** Student/teacher action L2 distance above which a state counts as divergent. */
  divergeThreshold: number;
  /** Added weight for a divergent state. */
  divergeBoost: number;
  /** Steps without a new voxel after which a state counts as stalled. */
  stallSteps: number;
  /** Window (steps) for the small-displacement stall check. */
  stallWindow: number;
  /** Net xy displacement (m) over stallWindow below which a non-covering state counts as stalled. */
  stallDist: number;
  /** Added weight for a stalled state. */
  stallBoost: number;
  /** Added weight for the states leading up to a new hard collision. */
  collisionBoost: number;
  /** How many states before an impact get collisionBoost. */
  collisionLookback: number;
  /** Upper bound on any sample weight. */
  maxWeight: number;
}

export const DEFAULT_WEIGHTING: DaggerWeighting = {
  base: 1,
  divergeThreshold: 0.35,
  divergeBoost: 1,
  stallSteps: 30,
  stallWindow: 20,
  stallDist: 0.3,
  stallBoost: 2,
  collisionBoost: 1.5,
  collisionLookback: 6,
  maxWeight: 4,
};

export interface DaggerCollectOpts {
  env: ExplorerEnv;
  spec: BuildingSpec;
  cfg: EpisodeConfig;
  policy: StudentPolicy;
  /** Probability per step of executing the teacher action instead of the student's. */
  beta: number;
  /** Max steps to collect (one episode; stops early on done). */
  steps: number;
  rng: RNG;
  /** Env reset seed; drawn from rng when absent. */
  seed?: number;
  /** Std of Gaussian noise added to executed student actions (action space, clamped). 0 = deterministic. */
  studentNoise?: number;
  weighting?: Partial<DaggerWeighting>;
  /** Stop collecting after this many consecutive steps without a new voxel. */
  maxStall?: number;
}

export interface DaggerRollout {
  /** [n * obsDim] observations of every visited state. */
  obs: Float32Array;
  /** [n * actDim] teacher labels. */
  actions: Float32Array;
  /** [n] sample weights. */
  weights: Float32Array;
  n: number;
  obsDim: number;
  actDim: number;
  /** Coverage reached at the end of the rollout. */
  coverage: number;
  /** Steps where the teacher's action was executed. */
  teacherSteps: number;
  /** States flagged divergent / stalled / leading into a collision. */
  divergent: number;
  stalled: number;
  preCollision: number;
  /** Mean student/teacher action L2 distance. */
  meanDivergence: number;
}

/** ArrayBuffers of a rollout, for postMessage transfer lists. */
export function daggerTransferables(r: DaggerRollout): ArrayBuffer[] {
  return [r.obs.buffer as ArrayBuffer, r.actions.buffer as ArrayBuffer, r.weights.buffer as ArrayBuffer];
}

export function collectDagger(opts: DaggerCollectOpts): DaggerRollout {
  const { env, spec, cfg, policy, rng } = opts;
  const w: DaggerWeighting = { ...DEFAULT_WEIGHTING, ...opts.weighting };
  const beta = Math.min(1, Math.max(0, opts.beta));
  const noise = opts.studentNoise ?? 0;
  const maxStall = opts.maxStall ?? Number.POSITIVE_INFINITY;
  const steps = Math.max(0, Math.floor(opts.steps));
  const seed = opts.seed ?? rng.int(1_000_000_000);
  let obs = env.reset(spec, seed, cfg);
  const obsDim = obs.length;
  const actDim = ACT_DIM;
  const obsBuf = new Float32Array(steps * obsDim);
  const actBuf = new Float32Array(steps * actDim);
  const wBuf = new Float32Array(steps);
  const histLen = Math.max(1, w.stallWindow);
  const hx = new Float64Array(histLen);
  const hy = new Float64Array(histLen);
  let hHead = 0;
  let hFill = 0;
  const pushPos = (x: number, y: number) => {
    hx[hHead] = x;
    hy[hHead] = y;
    hHead = (hHead + 1) % histLen;
    if (hFill < histLen) hFill += 1;
  };
  const start = env.snapshot();
  pushPos(start.pos[0], start.pos[1]);
  let sinceNew = 0;
  let lastCoverage = env.coverage;
  let n = 0;
  let teacherSteps = 0;
  let divergent = 0;
  let stalled = 0;
  let preCollision = 0;
  let collisions = 0;
  let divSum = 0;
  const exec = new Float32Array(actDim);
  for (let t = 0; t < steps; t++) {
    const pose = env.snapshot();
    const teacher = frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw);
    const student = policy.actWith(obs, 1, rng, true).action;
    let d2 = 0;
    for (let j = 0; j < actDim; j++) {
      const d = (student[j] ?? 0) - (teacher[j] ?? 0);
      d2 += d * d;
    }
    const div = Math.sqrt(d2);
    divSum += div;
    let weight = w.base;
    if (div > w.divergeThreshold) {
      weight += w.divergeBoost;
      divergent += 1;
    }
    let isStalled = sinceNew >= w.stallSteps;
    if (!isStalled && hFill >= histLen && sinceNew > 0) {
      const ox = hx[hHead]!;
      const oy = hy[hHead]!;
      isStalled = Math.hypot(pose.pos[0] - ox, pose.pos[1] - oy) < w.stallDist;
    }
    if (isStalled) {
      weight += w.stallBoost;
      stalled += 1;
    }
    obsBuf.set(obs, n * obsDim);
    actBuf.set(teacher.subarray(0, actDim), n * actDim);
    wBuf[n] = Math.min(w.maxWeight, weight);
    n += 1;

    const useTeacher = rng.next() < beta;
    if (useTeacher) {
      teacherSteps += 1;
      exec.set(teacher.subarray(0, actDim));
    } else {
      for (let j = 0; j < actDim; j++) {
        const a = (student[j] ?? 0) + (noise > 0 ? noise * rng.gaussian() : 0);
        exec[j] = a < -1 ? -1 : a > 1 ? 1 : a;
      }
    }
    const step = env.step(exec);
    obs = step.obs;
    if (step.info.collisions > collisions) {
      collisions = step.info.collisions;
      for (let k = Math.max(0, n - w.collisionLookback); k < n; k++) {
        if (w.collisionBoost > 0) {
          wBuf[k] = Math.min(w.maxWeight, wBuf[k]! + w.collisionBoost);
          preCollision += 1;
        }
      }
    }
    const cov = step.info.coverage;
    if (cov > lastCoverage) sinceNew = 0;
    else sinceNew += 1;
    lastCoverage = cov;
    const after = env.snapshot();
    pushPos(after.pos[0], after.pos[1]);
    if (step.done || sinceNew >= maxStall) break;
  }
  return {
    obs: obsBuf.slice(0, n * obsDim),
    actions: actBuf.slice(0, n * actDim),
    weights: wBuf.slice(0, n),
    n,
    obsDim,
    actDim,
    coverage: env.coverage,
    teacherSteps,
    divergent,
    stalled,
    preCollision,
    meanDivergence: n > 0 ? divSum / n : 0,
  };
}

export type DatasetPolicy = 'reservoir' | 'fifo';

/**
 * Aggregated DAgger dataset with a fixed capacity. When full, 'reservoir'
 * keeps a uniform sample of everything ever added and 'fifo' overwrites the
 * oldest samples.
 */
export class DaggerDataset {
  readonly obsDim: number;
  readonly actDim: number;
  readonly capacity: number;
  readonly policy: DatasetPolicy;
  readonly obs: Float32Array;
  readonly actions: Float32Array;
  readonly weights: Float32Array;
  private count = 0;
  private total = 0;
  private cursor = 0;
  private readonly rng: RNG;

  constructor(obsDim: number, actDim: number, capacity: number, opts?: { policy?: DatasetPolicy; seed?: number }) {
    if (obsDim <= 0 || actDim <= 0 || capacity <= 0) throw new Error('DaggerDataset dims must be positive');
    this.obsDim = obsDim;
    this.actDim = actDim;
    this.capacity = Math.floor(capacity);
    this.policy = opts?.policy ?? 'reservoir';
    this.obs = new Float32Array(this.capacity * obsDim);
    this.actions = new Float32Array(this.capacity * actDim);
    this.weights = new Float32Array(this.capacity);
    this.rng = new RNG(opts?.seed ?? 1, 'dagger-dataset');
  }

  /** Samples currently stored. */
  get size(): number {
    return this.count;
  }

  /** Samples ever offered to add(). */
  get seen(): number {
    return this.total;
  }

  add(obs: ArrayLike<number>, action: ArrayLike<number>, weight = 1, obsOffset = 0, actOffset = 0): void {
    this.total += 1;
    let slot: number;
    if (this.count < this.capacity) {
      slot = this.count;
      this.count += 1;
    } else if (this.policy === 'fifo') {
      slot = this.cursor;
      this.cursor = (this.cursor + 1) % this.capacity;
    } else {
      const j = this.rng.int(this.total);
      if (j >= this.capacity) return;
      slot = j;
    }
    const ob = slot * this.obsDim;
    for (let k = 0; k < this.obsDim; k++) this.obs[ob + k] = obs[obsOffset + k] ?? 0;
    const ab = slot * this.actDim;
    for (let k = 0; k < this.actDim; k++) this.actions[ab + k] = action[actOffset + k] ?? 0;
    this.weights[slot] = weight;
  }

  addRollout(r: { obs: Float32Array; actions: Float32Array; weights?: Float32Array; n: number; obsDim?: number }): void {
    if (r.obsDim != null && r.obsDim !== this.obsDim) {
      throw new Error(`DaggerDataset: rollout obsDim ${r.obsDim} != dataset obsDim ${this.obsDim}`);
    }
    if (r.obs.length < r.n * this.obsDim || r.actions.length < r.n * this.actDim) {
      throw new Error('DaggerDataset: rollout arrays are shorter than n');
    }
    for (let i = 0; i < r.n; i++) {
      this.add(r.obs, r.actions, r.weights ? r.weights[i]! : 1, i * this.obsDim, i * this.actDim);
    }
  }

  /** Live views of the stored samples (no copy). */
  view(): BcData & { weights: Float32Array } {
    return {
      obs: this.obs.subarray(0, this.count * this.obsDim),
      actions: this.actions.subarray(0, this.count * this.actDim),
      weights: this.weights.subarray(0, this.count),
      n: this.count,
    };
  }

  meanWeight(): number {
    let s = 0;
    for (let i = 0; i < this.count; i++) s += this.weights[i]!;
    return this.count > 0 ? s / this.count : 0;
  }
}

const EVAL_SEEDS = new Set<number>([...STAGE0_SEEDS, ...STAGE3_SEEDS]);

/** Episode config matching the eval protocol (src/eval/coverage.ts). */
export function evalLikeConfig(stage: number, spec: BuildingSpec, lighting: LightingPreset = 'day'): EpisodeConfig {
  const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
  const cfg = defaultEpisodeConfig(stage, nRooms);
  cfg.maxSteps = stage <= 1 ? 1000 : 2400;
  cfg.stallSteps = 500;
  cfg.maxCollisions = 16;
  cfg.dynamics = 'kinematic';
  cfg.lighting = lightingPreset(lighting, nRooms);
  return cfg;
}

/**
 * A training-distribution building. The seed goes through a label that the
 * eval protocol never uses, and the eval seeds themselves are rejected.
 */
export function trainingBuilding(stage: number, seed: number, label = 'dagger'): BuildingSpec {
  if (EVAL_SEEDS.has(seed)) throw new Error(`seed ${seed} is reserved for eval`);
  const params = new Curriculum().genParams(stage, new RNG(seed, `${label}-${stage}`));
  return generateBuilding(params);
}

export interface ValidationEpisode {
  stage: number;
  seed: number;
  lighting: LightingPreset;
}

/** Held-out validation set from the training distribution (never the eval seeds). */
export function defaultValidationSet(): ValidationEpisode[] {
  const out: ValidationEpisode[] = [];
  for (const seed of [7001, 7002, 7003, 7004]) out.push({ stage: 0, seed, lighting: 'day' });
  for (const seed of [7101, 7102, 7103]) out.push({ stage: 3, seed, lighting: 'day' });
  out.push({ stage: 0, seed: 7005, lighting: 'night-dark' });
  out.push({ stage: 3, seed: 7104, lighting: 'night-dark' });
  return out;
}

export interface ValidationResult {
  /** Mean of the stage-0-ish mean and the stage>=2 mean (balanced). */
  score: number;
  small: number;
  large: number;
  perEpisode: number[];
}

export async function validationCoverage(
  policy: StudentPolicy,
  set: ValidationEpisode[] = defaultValidationSet(),
  env?: ExplorerEnv,
): Promise<ValidationResult> {
  const own = env ?? (await createEnv());
  const rng = new RNG(1, 'validation');
  const perEpisode: number[] = [];
  const small: number[] = [];
  const large: number[] = [];
  for (const ep of set) {
    const spec = trainingBuilding(ep.stage, ep.seed, 'val');
    const cfg = evalLikeConfig(ep.stage, spec, ep.lighting);
    let obs = own.reset(spec, spec.seed, cfg);
    let coverage = 0;
    for (let t = 0; t < (cfg.maxSteps ?? 1000); t++) {
      const step = own.step(policy.actWith(obs, 1, rng, true).action);
      obs = step.obs;
      coverage = step.info.coverage;
      if (step.done) break;
    }
    perEpisode.push(coverage);
    (ep.stage <= 1 ? small : large).push(coverage);
  }
  if (!env) own.dispose();
  const avg = (xs: number[]) => (xs.length ? xs.reduce((s, v) => s + v, 0) / xs.length : 0);
  const s = avg(small);
  const l = avg(large);
  const score = small.length && large.length ? 0.5 * (s + l) : avg(perEpisode);
  return { score, small: s, large: l, perEpisode };
}

export function checkpointFromPolicy(policy: InferPolicy, stage = 3): Checkpoint {
  return {
    version: 1,
    obsVersion: OBS_VERSION,
    obsDim: policy.obsDim,
    actDim: policy.actDim,
    hidden: [...policy.hidden],
    weights: policy.getWeights().map((buf) => Array.from(new Float32Array(buf))),
    obsMean: new Array(policy.obsDim).fill(0),
    obsVar: new Array(policy.obsDim).fill(1),
    obsCount: 1,
    rewVar: 1,
    stage,
  };
}

/** Throws when the checkpoint was trained on a different observation layout. */
export function policyFromCheckpoint(c: Checkpoint, seed = 1): InferPolicy {
  const mismatch = checkpointMismatch(c);
  if (mismatch) throw new Error(mismatch);
  const policy = new InferPolicy(c.obsDim, c.actDim, c.hidden, seed);
  policy.setWeights(c.weights.map((row) => Float32Array.from(row).buffer));
  return policy;
}

export interface DaggerIterMetrics {
  /** 0 is the teacher-only BC round. */
  iter: number;
  beta: number;
  episodes: number;
  samples: number;
  datasetSize: number;
  /** Mean coverage of the collection rollouts (mixture policy). */
  rolloutCoverage: number;
  divergentFrac: number;
  stalledFrac: number;
  meanDivergence: number;
  bcLoss: number;
  validation?: ValidationResult;
  seconds: number;
}

export interface TrainDaggerOpts {
  seed?: number;
  /** DAgger iterations after the BC round. */
  iterations?: number;
  /** Teacher-only episodes for the initial BC round. */
  bcEpisodes?: number;
  episodesPerIter?: number;
  /** Stage mix for collection and its sampling weights. */
  stages?: number[];
  stageWeights?: number[];
  /** beta_k = beta0 * betaDecay^(k-1) for DAgger iteration k >= 1. */
  beta0?: number;
  betaDecay?: number;
  /** Fraction of collection episodes flown with the night-dark preset. */
  nightFraction?: number;
  bcEpochs?: number;
  retrainEpochs?: number;
  minibatch?: number;
  lr?: number;
  capacity?: number;
  datasetPolicy?: DatasetPolicy;
  studentNoise?: number;
  maxStall?: number;
  /** Cap on steps per collection episode (defaults to the eval horizon per stage). */
  maxEpisodeSteps?: number;
  weighting?: Partial<DaggerWeighting>;
  /** Validate after every round and return the best round's weights. false disables. */
  validation?: ValidationEpisode[] | false;
  /** Warm start instead of a fresh policy. */
  init?: Checkpoint;
  /** Existing dataset to keep aggregating into. */
  dataset?: DaggerDataset;
  onIter?: (m: DaggerIterMetrics) => void;
}

export interface TrainDaggerResult {
  checkpoint: Checkpoint;
  metrics: DaggerIterMetrics[];
  dataset: DaggerDataset;
  /** Round whose weights were returned. */
  bestIter: number;
  /** Weights after the final round, whatever validation said. */
  lastCheckpoint: Checkpoint;
}

function pickStage(stages: number[], weights: number[], rng: RNG): number {
  let total = 0;
  for (let i = 0; i < stages.length; i++) total += weights[i] ?? 1;
  let u = rng.next() * total;
  for (let i = 0; i < stages.length; i++) {
    u -= weights[i] ?? 1;
    if (u <= 0) return stages[i]!;
  }
  return stages[stages.length - 1]!;
}

/** BC on teacher rollouts, then K DAgger rounds of collect -> aggregate -> weighted, warm-started BC. */
export async function trainDagger(opts: TrainDaggerOpts = {}): Promise<TrainDaggerResult> {
  const seed = opts.seed ?? 11;
  const iterations = opts.iterations ?? 4;
  const stages = opts.stages ?? [0, 1, 2, 3];
  const stageWeights = opts.stageWeights ?? [0.3, 0.15, 0.2, 0.35];
  const beta0 = opts.beta0 ?? 0.5;
  const betaDecay = opts.betaDecay ?? 0.5;
  const nightFraction = opts.nightFraction ?? 0.25;
  const minibatch = opts.minibatch ?? 128;
  const policy = opts.init
    ? policyFromCheckpoint(opts.init, seed)
    : new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, seed);
  const trainer = new TfTrainer(policy, opts.lr ?? 1e-3);
  await trainer.ready();
  const env = await createEnv();
  const dataset =
    opts.dataset ??
    new DaggerDataset(policy.obsDim, policy.actDim, opts.capacity ?? 60_000, {
      policy: opts.datasetPolicy ?? 'reservoir',
      seed,
    });
  const rng = new RNG(seed, 'dagger');
  const validation = opts.validation === false ? null : (opts.validation ?? defaultValidationSet());
  const metrics: DaggerIterMetrics[] = [];
  let bestScore = -1;
  let bestWeights = policy.getWeights();
  let bestIter = 0;
  let episodeCounter = 0;

  for (let iter = 0; iter <= iterations; iter++) {
    const started = Date.now();
    const bcRound = iter === 0 && !opts.init;
    const beta = bcRound ? 1 : beta0 * Math.pow(betaDecay, Math.max(0, iter - 1));
    const episodes = bcRound ? (opts.bcEpisodes ?? 16) : (opts.episodesPerIter ?? 12);
    let samples = 0;
    let divergent = 0;
    let stalled = 0;
    let divSum = 0;
    let covSum = 0;
    for (let e = 0; e < episodes; e++) {
      const stage = pickStage(stages, stageWeights, rng);
      let bseed = 10_000 + seed * 100_003 + episodeCounter * 7919;
      while (EVAL_SEEDS.has(bseed)) bseed += 1;
      episodeCounter += 1;
      const spec = trainingBuilding(stage, bseed);
      const lighting: LightingPreset = rng.next() < nightFraction ? 'night-dark' : 'day';
      const cfg = evalLikeConfig(stage, spec, lighting);
      const horizon = Math.min(cfg.maxSteps ?? 1000, opts.maxEpisodeSteps ?? Number.POSITIVE_INFINITY);
      const r = collectDagger({
        env,
        spec,
        cfg,
        policy,
        beta,
        steps: horizon,
        rng,
        seed: spec.seed,
        studentNoise: opts.studentNoise ?? 0.05,
        maxStall: opts.maxStall ?? 300,
        // In the teacher-only round the student is untrained, so divergence carries no signal.
        weighting: bcRound ? { ...opts.weighting, divergeBoost: 0 } : opts.weighting,
      });
      dataset.addRollout(r);
      samples += r.n;
      divergent += r.divergent;
      stalled += r.stalled;
      divSum += r.meanDivergence * r.n;
      covSum += r.coverage;
    }
    const view = dataset.view();
    const bcLoss = await trainer.imitate(view.obs, view.actions, view.n, {
      epochs: bcRound ? (opts.bcEpochs ?? 6) : (opts.retrainEpochs ?? 3),
      minibatch,
      weights: view.weights,
    });
    const m: DaggerIterMetrics = {
      iter,
      beta,
      episodes,
      samples,
      datasetSize: dataset.size,
      rolloutCoverage: episodes > 0 ? covSum / episodes : 0,
      divergentFrac: samples > 0 ? divergent / samples : 0,
      stalledFrac: samples > 0 ? stalled / samples : 0,
      meanDivergence: samples > 0 ? divSum / samples : 0,
      bcLoss,
      seconds: 0,
    };
    if (validation) {
      m.validation = await validationCoverage(policy, validation, env);
      if (m.validation.score > bestScore) {
        bestScore = m.validation.score;
        bestWeights = policy.getWeights();
        bestIter = iter;
      }
    } else {
      bestWeights = policy.getWeights();
      bestIter = iter;
    }
    m.seconds = (Date.now() - started) / 1000;
    metrics.push(m);
    opts.onIter?.(m);
  }
  const lastCheckpoint = checkpointFromPolicy(policy);
  policy.setWeights(bestWeights);
  env.dispose();
  return { checkpoint: checkpointFromPolicy(policy), metrics, dataset, bestIter, lastCheckpoint };
}
