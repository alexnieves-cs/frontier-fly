/**
 * One rollout engine: several envs, one InferPolicy (batched inference across
 * the envs) and one WorldPool. Runs inside a worker_threads worker
 * (src/train/nodeWorker.ts) or in-process (benchmarks, tests). Has no TF.js
 * dependency.
 */
import { evalEpisodeSetup } from '../eval/coverage';
import { generateBuilding } from '../gen/building';
import { Curriculum } from '../rl/curriculum';
import { InferPolicy } from '../rl/infer';
import { RunningNorm, type RunningNormState } from '../rl/normalize';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec } from '../shared/types';
import { ExplorerEnv } from '../sim/env';
import { frontierAction } from '../sim/frontier';
import type { LightingPreset } from '../sim/lighting';
import { initPhysics } from '../sim/world';
import { WorldPool, type PooledWorld, type WorldPoolOptions } from '../sim/worldPool';
import { makeConfig } from '../workers/rollout';

/** Weights per stage: an array indexed by stage, or a {stage: weight} map. */
export type StageMix = readonly number[] | Readonly<Record<number, number>>;

export interface CollectPpoOptions {
  stepsPerEnv: number;
  /** Stage sampling weights for episode resets. Default: stage 0 only. */
  stageMix?: StageMix;
  /** Reseeds this worker's action/stage stream for the call (worker index is mixed in). */
  seed?: number;
  /** Mean actions instead of samples. */
  deterministic?: boolean;
  /** 'curriculum' (default) uses Curriculum.dynamicsFor(stage). */
  dynamics?: 'kinematic' | 'quad' | 'curriculum';
  /** Overrides on top of the training config from workers/rollout.ts makeConfig. */
  episode?: { maxSteps?: number; stallSteps?: number; maxCollisions?: number };
  /** Start every env on a fresh episode instead of continuing the previous one. */
  resetEnvs?: boolean;
  /** 'pool' (default) reuses pre-built worlds; 'rebuild' generates + builds physics every reset. */
  worlds?: 'pool' | 'rebuild';
}

export interface EpisodeStat {
  /** Global env index (worker-major) once merged by NodeRolloutPool, local index inside a core. */
  env: number;
  stage: number;
  coverage: number;
  reward: number;
  length: number;
  collisions: number;
  reachedNewFloor: boolean;
  success: boolean;
}

export interface ObsMoments {
  count: number;
  mean: Float64Array;
  m2: Float64Array;
}

/**
 * Env-major PPO arrays: env e owns rows [e * stepsPerEnv, (e + 1) * stepsPerEnv).
 * `obs` is what the policy saw (normalized when a norm was set with the weights).
 * `dones` marks true terminals, `truncated` stall cut-offs (same flags as
 * workers/rollout.ts). `bootstrap[e]` is V(obs after the last step) of env e.
 */
export interface PpoChunk {
  obs: Float32Array;
  actions: Float32Array;
  logp: Float32Array;
  values: Float32Array;
  rewards: Float32Array;
  dones: Uint8Array;
  truncated: Uint8Array;
  n: number;
  nEnvs: number;
  stepsPerEnv: number;
  obsDim: number;
  bootstrap: Float32Array;
  episodes: EpisodeStat[];
  /** Moments of the raw observations of this chunk, when a norm is active. Feed to RunningNorm.merge. */
  obsMoments: ObsMoments | null;
  ms: number;
  stepsPerSec: number;
}

export interface EvalJob {
  stage: number;
  seed: number;
  lighting: LightingPreset;
  /** 'policy' (default): deterministic actWith on the current weights. 'teacher': frontierAction. */
  actor?: 'policy' | 'teacher';
  /** Apply the norm that came with setWeights before acting. Default false (matches scripts/eval-coverage.ts). */
  normalize?: boolean;
}

export interface EvalResult {
  coverage: number;
  steps: number;
}

/** What a job function sees inside a worker. */
export interface JobContext {
  readonly workerIndex: number;
  readonly envs: ExplorerEnv[];
  readonly policy: InferPolicy;
  readonly norm: RunningNorm | null;
  readonly worlds: WorldPool;
  readonly obsDim: number;
  readonly actDim: number;
  readonly rng: RNG;
}

export type JobFn = (ctx: JobContext, payload: unknown) => unknown;

const jobRegistry = new Map<string, JobFn>();

/** Register named jobs for runJob(). Job modules export `jobs: Record<string, JobFn>`. */
export function registerJobs(jobs: Record<string, JobFn> | undefined | null): void {
  if (!jobs) return;
  for (const [name, fn] of Object.entries(jobs)) {
    if (typeof fn === 'function') jobRegistry.set(name, fn);
  }
}

export function registeredJobs(): string[] {
  return [...jobRegistry.keys()].sort();
}

registerJobs({
  ping: (ctx, payload) => ({ workerIndex: ctx.workerIndex, envs: ctx.envs.length, obsDim: ctx.obsDim, jobs: registeredJobs(), payload }),
});

function stageWeights(mix: StageMix | undefined): { stages: number[]; cdf: number[] } {
  const stages: number[] = [];
  const weights: number[] = [];
  if (!mix) return { stages: [0], cdf: [1] };
  const entries: [number, number][] = Array.isArray(mix)
    ? (mix as readonly number[]).map((w, s) => [s, w])
    : Object.entries(mix as Record<number, number>).map(([s, w]) => [Number(s), w]);
  for (const [s, w] of entries) {
    if (w > 0) {
      stages.push(s);
      weights.push(w);
    }
  }
  if (stages.length === 0) return { stages: [0], cdf: [1] };
  const total = weights.reduce((a, b) => a + b, 0);
  let acc = 0;
  return { stages, cdf: weights.map((w) => (acc += w / total)) };
}

function sampleStage(mix: { stages: number[]; cdf: number[] }, rng: RNG): number {
  const u = rng.next();
  for (let i = 0; i < mix.cdf.length; i++) if (u < mix.cdf[i]!) return mix.stages[i]!;
  return mix.stages[mix.stages.length - 1]!;
}

function roomsOf(spec: BuildingSpec): number {
  return spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
}

export interface RolloutCoreOptions {
  workerIndex?: number;
  nEnvs: number;
  seed?: number;
  obsDim?: number;
  hidden?: readonly number[];
  worldPool?: WorldPoolOptions;
}

export class RolloutCore {
  readonly workerIndex: number;
  readonly obsDim: number;
  readonly envs: ExplorerEnv[] = [];
  readonly policy: InferPolicy;
  readonly worlds: WorldPool;
  private norm: RunningNorm | null = null;
  private rng: RNG;
  private readonly seed: number;
  private readonly curriculum = new Curriculum();
  private readonly obs: (Float32Array | null)[] = [];
  private readonly held: (PooledWorld | null)[] = [];
  private readonly stage: number[] = [];
  private readonly ret: number[] = [];
  private readonly len: number[] = [];
  private evalEnvs: ExplorerEnv[] = [];

  private constructor(opts: RolloutCoreOptions, worlds: WorldPool) {
    this.workerIndex = opts.workerIndex ?? 0;
    this.seed = (opts.seed ?? 1) >>> 0;
    this.obsDim = opts.obsDim ?? OBS_DIM;
    if (this.obsDim !== OBS_DIM) throw new Error(`RolloutCore: obsDim ${this.obsDim} does not match the env OBS_DIM ${OBS_DIM}`);
    this.policy = new InferPolicy(this.obsDim, ACT_DIM, opts.hidden ?? HIDDEN, this.seed + this.workerIndex);
    this.worlds = worlds;
    this.rng = new RNG(this.seed, `core-${this.workerIndex}`);
    for (let i = 0; i < opts.nEnvs; i++) {
      this.envs.push(new ExplorerEnv());
      this.obs.push(null);
      this.held.push(null);
      this.stage.push(0);
      this.ret.push(0);
      this.len.push(0);
    }
  }

  static async create(opts: RolloutCoreOptions): Promise<RolloutCore> {
    await initPhysics();
    const index = opts.workerIndex ?? 0;
    const poolOpts = { ...opts.worldPool, seed: ((opts.worldPool?.seed ?? opts.seed ?? 1) + index * 7919) >>> 0 };
    const worlds = await WorldPool.create(poolOpts);
    return new RolloutCore(opts, worlds);
  }

  get context(): JobContext {
    return {
      workerIndex: this.workerIndex,
      envs: this.envs,
      policy: this.policy,
      norm: this.norm,
      worlds: this.worlds,
      obsDim: this.obsDim,
      actDim: ACT_DIM,
      rng: this.rng,
    };
  }

  /** Weight buffers in InferPolicy order; `norm` (optional) is applied to observations before acting. */
  setWeights(weights: ArrayBuffer[], norm?: RunningNormState | null): void {
    this.policy.setWeights(weights);
    if (norm) {
      this.norm ??= new RunningNorm(this.obsDim);
      this.norm.load(norm);
    } else {
      this.norm = null;
    }
  }

  async runJob(name: string, payload: unknown): Promise<unknown> {
    const fn = jobRegistry.get(name);
    if (!fn) throw new Error(`unknown job "${name}" (registered: ${registeredJobs().join(', ')})`);
    return await fn(this.context, payload);
  }

  private resetEnv(i: number, mix: { stages: number[]; cdf: number[] }, opts: CollectPpoOptions): void {
    const stage = sampleStage(mix, this.rng);
    const env = this.envs[i]!;
    let spec: BuildingSpec;
    let world: PooledWorld | null = null;
    if ((opts.worlds ?? 'pool') === 'pool') {
      world = this.worlds.acquire(stage, this.rng);
      spec = world.spec;
    } else {
      spec = generateBuilding(this.curriculum.genParams(stage, this.rng));
    }
    const seed = this.rng.int(1_000_000_000);
    const dynamics =
      opts.dynamics && opts.dynamics !== 'curriculum' ? opts.dynamics : this.curriculum.dynamicsFor(stage, new RNG(seed, 'dyn'));
    const cfg = makeConfig(stage, seed, roomsOf(spec), dynamics);
    if (opts.episode?.maxSteps != null) cfg.maxSteps = opts.episode.maxSteps;
    if (opts.episode?.stallSteps != null) cfg.stallSteps = opts.episode.stallSteps;
    if (opts.episode?.maxCollisions != null) cfg.maxCollisions = opts.episode.maxCollisions;
    this.obs[i] = world ? env.reset(spec, seed, cfg, world.physics) : env.reset(spec, seed, cfg);
    // Release after the reset so the old world is never freed while the env still points at it.
    this.worlds.release(this.held[i]);
    this.held[i] = world;
    this.stage[i] = stage;
    this.ret[i] = 0;
    this.len[i] = 0;
  }

  collectPpo(opts: CollectPpoOptions): PpoChunk {
    const T = Math.max(0, opts.stepsPerEnv | 0);
    const E = this.envs.length;
    const D = this.obsDim;
    if (opts.seed != null) this.rng = new RNG((opts.seed >>> 0) ^ Math.imul(this.workerIndex + 1, 0x9e3779b1), `collect-${this.workerIndex}`);
    const mix = stageWeights(opts.stageMix);
    const deterministic = opts.deterministic ?? false;
    for (let i = 0; i < E; i++) if (opts.resetEnvs || !this.obs[i]) this.resetEnv(i, mix, opts);

    const n = T * E;
    const out: PpoChunk = {
      obs: new Float32Array(n * D),
      actions: new Float32Array(n * ACT_DIM),
      logp: new Float32Array(n),
      values: new Float32Array(n),
      rewards: new Float32Array(n),
      dones: new Uint8Array(n),
      truncated: new Uint8Array(n),
      n,
      nEnvs: E,
      stepsPerEnv: T,
      obsDim: D,
      bootstrap: new Float32Array(E),
      episodes: [],
      obsMoments: null,
      ms: 0,
      stepsPerSec: 0,
    };
    const norm = this.norm;
    const moments: ObsMoments | null = norm ? { count: 0, mean: new Float64Array(D), m2: new Float64Array(D) } : null;
    const packed = new Float32Array(E * D);
    const t0 = performance.now();
    const pack = (count: boolean): Float32Array => {
      for (let i = 0; i < E; i++) packed.set(this.obs[i]!, i * D);
      if (!norm || !moments) return packed;
      if (!count) return norm.apply(packed, E);
      for (let i = 0; i < E; i++) {
        moments.count += 1;
        const base = i * D;
        const c = moments.count;
        for (let d = 0; d < D; d++) {
          const x = packed[base + d]!;
          const delta = x - moments.mean[d]!;
          moments.mean[d] = moments.mean[d]! + delta / c;
          moments.m2[d] = moments.m2[d]! + delta * (x - moments.mean[d]!);
        }
      }
      return norm.apply(packed, E);
    };

    for (let t = 0; t < T; t++) {
      const x = pack(true);
      const act = this.policy.actWith(x, E, this.rng, deterministic);
      for (let i = 0; i < E; i++) {
        const row = i * T + t;
        out.obs.set(x.subarray(i * D, (i + 1) * D), row * D);
        const action = act.action.subarray(i * ACT_DIM, (i + 1) * ACT_DIM);
        out.actions.set(action, row * ACT_DIM);
        out.logp[row] = act.logp[i]!;
        out.values[row] = act.value[i]!;
        const step = this.envs[i]!.step(action);
        out.rewards[row] = step.reward;
        out.dones[row] = step.done && !step.truncated ? 1 : 0;
        out.truncated[row] = step.truncated ? 1 : 0;
        this.ret[i]! += step.reward;
        this.len[i]! += 1;
        this.obs[i] = step.obs;
        if (step.done) {
          out.episodes.push({
            env: i,
            stage: this.stage[i]!,
            coverage: step.info.coverage,
            reward: this.ret[i]!,
            length: this.len[i]!,
            collisions: step.info.collisions,
            reachedNewFloor: step.info.floor > 0 || step.info.newFloor,
            success: step.info.success,
          });
          this.resetEnv(i, mix, opts);
        }
      }
    }
    if (E > 0) {
      // Not counted into the moments: these are the first observations of the next collect.
      const tail = this.policy.actWith(pack(false), E, this.rng, true);
      out.bootstrap.set(tail.value);
    }
    out.obsMoments = moments;
    out.ms = performance.now() - t0;
    out.stepsPerSec = n > 0 ? (n * 1000) / Math.max(1e-3, out.ms) : 0;
    return out;
  }

  /**
   * Coverage episodes with the src/eval/coverage.ts protocol, run in lockstep so
   * policy inference is batched. Per-row inference does not depend on the batch,
   * so results match the sequential evalCoverage bit for bit.
   */
  evaluate(jobs: EvalJob[]): EvalResult[] {
    while (this.evalEnvs.length < jobs.length) this.evalEnvs.push(new ExplorerEnv());
    const D = this.obsDim;
    const live = jobs.map((job, k) => {
      const setup = evalEpisodeSetup(job.stage, job.seed, job.lighting);
      const env = this.evalEnvs[k]!;
      const obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
      return { job, env, obs, maxSteps: setup.cfg.maxSteps, steps: 0, coverage: 0, done: false };
    });
    const results: EvalResult[] = jobs.map(() => ({ coverage: 0, steps: 0 }));
    const rng = new RNG(1);
    for (;;) {
      const active = live.filter((e) => !e.done && e.steps < e.maxSteps);
      if (active.length === 0) break;
      const learners = active.filter((e) => (e.job.actor ?? 'policy') === 'policy');
      const actions = new Map<(typeof live)[number], Float32Array>();
      for (const normalize of [false, true]) {
        const group = learners.filter((e) => Boolean(e.job.normalize) === normalize);
        if (group.length === 0) continue;
        const x = new Float32Array(group.length * D);
        group.forEach((e, k) => x.set(e.obs, k * D));
        const input = normalize && this.norm ? this.norm.apply(x, group.length) : x;
        const act = this.policy.actWith(input, group.length, rng, true).action;
        group.forEach((e, k) => actions.set(e, act.slice(k * ACT_DIM, (k + 1) * ACT_DIM)));
      }
      for (const e of active) {
        let action = actions.get(e);
        if (!action) {
          const pose = e.env.snapshot();
          action = frontierAction(e.env.building!, e.env.visitedGrid, pose.pos, pose.yaw);
        }
        const step = e.env.step(action);
        e.obs = step.obs;
        e.coverage = step.info.coverage;
        e.steps += 1;
        if (step.done) e.done = true;
      }
    }
    live.forEach((e, k) => {
      results[k] = { coverage: e.coverage, steps: e.steps };
      e.env.dispose();
    });
    return results;
  }

  dispose(): void {
    for (const env of this.envs) env.dispose();
    for (const env of this.evalEnvs) env.dispose();
    this.worlds.free();
  }
}
