/**
 * Worker jobs for PPO fine-tuning with a privileged critic and an optional GRU
 * memory (scripts/train-ppo.ts). Registered through NodeRolloutPool
 * `jobModules: ['src/train/ppoJob.ts']`.
 *
 * - seqRollout: on-policy steps in eval-like episodes on pooled training
 *   buildings. Besides the PPO arrays it records, per step, the frontier
 *   teacher's action (a DAgger label on the student's own states) and a small
 *   privileged vector that only the critic sees.
 * - seqEval: deterministic coverage on fixed (stage, seed, lighting) episodes
 *   with the src/eval/coverage.ts protocol, memory included.
 *
 * Every episode end is a terminal for GAE: the time and collision budgets are
 * part of what the critic sees, so a time-out is a real end, not a truncation.
 */
import { evalEpisodeSetup } from '../eval/coverage';
import { MemPolicy } from '../rl/memPolicy';
import { ACT_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec } from '../shared/types';
import { ExplorerEnv, policyEpisodeLimits } from '../sim/env';
import { frontierAction, frontierDistance } from '../sim/frontier';
import { lightingPreset } from '../sim/lighting';
import type { PooledWorld } from '../sim/worldPool';
import { makeConfig } from '../workers/rollout';
import type { EvalJob, JobContext, JobFn } from './rolloutCore';

/** Size of the critic-only privileged vector (see privileged()). */
export const PRIV_DIM = 12;

export interface PolicySpec {
  weights: ArrayBuffer[];
  hidden: number[];
  memDim: number;
}

export interface SeqRolloutPayload {
  policy: PolicySpec;
  /** Steps per env in this call; a multiple of seqLen. */
  stepsPerEnv: number;
  seqLen: number;
  /** Weight per stage, indexed by stage. */
  stageMix: number[];
  nightFraction: number;
  seed: number;
  /** Start every env on a fresh episode (e.g. after a policy change that makes the old memory stale). */
  resetEnvs?: boolean;
}

export interface SeqEpisode {
  stage: number;
  night: boolean;
  coverage: number;
  collisions: number;
  length: number;
  reward: number;
}

/**
 * Env-major arrays: env e owns rows [e * T, (e + 1) * T). `starts[row]` is 1
 * when an episode begins at that row (the memory is zeroed before it).
 * `state0` holds each sequence's memory before its first step, sequences being
 * the T / seqLen consecutive blocks of each env.
 */
export interface SeqRolloutResult {
  obs: Float32Array;
  priv: Float32Array;
  actions: Float32Array;
  teacher: Float32Array;
  logp: Float32Array;
  rewards: Float32Array;
  dones: Uint8Array;
  starts: Uint8Array;
  state0: Float32Array;
  /** Observation and privileged vector after the last step, for the bootstrap value. */
  lastObs: Float32Array;
  lastPriv: Float32Array;
  nEnvs: number;
  stepsPerEnv: number;
  episodes: SeqEpisode[];
  ms: number;
}

interface EnvSlot {
  obs: Float32Array | null;
  world: PooledWorld | null;
  spec: BuildingSpec | null;
  stage: number;
  night: boolean;
  maxSteps: number;
  steps: number;
  reward: number;
  started: boolean;
}

interface WorkerState {
  slots: EnvSlot[];
  state: Float32Array;
  policy: MemPolicy | null;
  key: string;
}

let worker: WorkerState | null = null;

function policyFor(spec: PolicySpec, obsDim: number): MemPolicy {
  const key = `${spec.hidden.join('x')}:${spec.memDim}`;
  if (!worker!.policy || worker!.key !== key) {
    worker!.policy = new MemPolicy(obsDim, ACT_DIM, spec.hidden, spec.memDim);
    worker!.key = key;
  }
  worker!.policy.setWeights(spec.weights);
  return worker!.policy;
}

function pickStage(mix: number[], rng: RNG): number {
  const total = mix.reduce((a, b) => a + Math.max(0, b), 0);
  let u = rng.next() * total;
  for (let s = 0; s < mix.length; s++) {
    u -= Math.max(0, mix[s]!);
    if (u <= 0 && mix[s]! > 0) return s;
  }
  return mix.length - 1;
}

/**
 * Critic-only inputs: coverage, time used, collision budget used, true BFS
 * distance to the nearest unvisited voxel, the teacher's action, stage, size,
 * voxels left and lighting. None of it reaches the actor.
 */
function privileged(env: ExplorerEnv, slot: EnvSlot, teacher: Float32Array, collisions: number, out: Float32Array, at: number): void {
  const spec = slot.spec!;
  const pose = env.snapshot();
  const cov = env.coverage;
  out[at] = cov;
  out[at + 1] = slot.steps / Math.max(1, slot.maxSteps);
  out[at + 2] = collisions / 16;
  out[at + 3] = Math.min(2, frontierDistance(spec, env.visitedGrid, pose.pos) / 20);
  for (let j = 0; j < 4; j++) out[at + 4 + j] = teacher[j] ?? 0;
  out[at + 8] = slot.stage / 5;
  out[at + 9] = Math.min(2, spec.reachableVoxels / 2000);
  out[at + 10] = Math.min(2, (spec.reachableVoxels * (1 - cov)) / 1000);
  out[at + 11] = slot.night ? 1 : 0;
}

function resetSlot(ctx: JobContext, env: ExplorerEnv, slot: EnvSlot, p: SeqRolloutPayload, rng: RNG): void {
  const stage = pickStage(p.stageMix, rng);
  const world = ctx.worlds.acquire(stage, rng);
  const spec = world.spec;
  const nRooms = spec.floors.reduce((n, f) => n + f.rooms.length, 0);
  const envSeed = rng.int(1_000_000_000);
  const night = rng.next() < p.nightFraction;
  // The DAgger training config (sampled noise per stage), with the eval lighting presets and episode limits.
  const cfg = makeConfig(stage, envSeed, nRooms, 'kinematic');
  cfg.lighting = lightingPreset(night ? 'night-dark' : 'day', nRooms);
  Object.assign(cfg, policyEpisodeLimits(stage));
  slot.obs = env.reset(spec, envSeed, cfg, world.physics);
  ctx.worlds.release(slot.world);
  slot.world = world;
  slot.spec = spec;
  slot.stage = stage;
  slot.night = night;
  slot.maxSteps = cfg.maxSteps!;
  slot.steps = 0;
  slot.reward = 0;
  slot.started = true;
}

const seqRollout: JobFn = (ctx, raw) => {
  const p = raw as SeqRolloutPayload;
  const E = ctx.envs.length;
  const D = ctx.obsDim;
  const T = p.stepsPerEnv;
  const L = Math.max(1, p.seqLen);
  if (T % L !== 0) throw new Error('seqRollout: stepsPerEnv must be a multiple of seqLen');
  worker ??= { slots: [], state: new Float32Array(0), policy: null, key: '' };
  if (worker.slots.length !== E) {
    worker.slots = ctx.envs.map(() => ({ obs: null, world: null, spec: null, stage: 0, night: false, maxSteps: 1, steps: 0, reward: 0, started: false }));
  }
  const policy = policyFor(p.policy, D);
  const m = policy.memDim;
  if (worker.state.length !== E * m) worker.state = new Float32Array(E * m);
  const rng = new RNG((p.seed >>> 0) ^ Math.imul(ctx.workerIndex + 1, 0x9e3779b1), `seq-${ctx.workerIndex}`);
  const slots = worker.slots;
  for (let i = 0; i < E; i++) {
    if (p.resetEnvs || !slots[i]!.obs) {
      resetSlot(ctx, ctx.envs[i]!, slots[i]!, p, rng);
      worker.state.fill(0, i * m, (i + 1) * m);
    }
  }
  const n = E * T;
  const out: SeqRolloutResult = {
    obs: new Float32Array(n * D),
    priv: new Float32Array(n * PRIV_DIM),
    actions: new Float32Array(n * ACT_DIM),
    teacher: new Float32Array(n * ACT_DIM),
    logp: new Float32Array(n),
    rewards: new Float32Array(n),
    dones: new Uint8Array(n),
    starts: new Uint8Array(n),
    state0: new Float32Array(E * (T / L) * m),
    lastObs: new Float32Array(E * D),
    lastPriv: new Float32Array(E * PRIV_DIM),
    nEnvs: E,
    stepsPerEnv: T,
    episodes: [],
    ms: 0,
  };
  const t0 = performance.now();
  const packed = new Float32Array(E * D);
  const collisions = new Array<number>(E).fill(0);
  for (let t = 0; t < T; t++) {
    for (let i = 0; i < E; i++) {
      const slot = slots[i]!;
      const row = i * T + t;
      packed.set(slot.obs!, i * D);
      out.obs.set(slot.obs!, row * D);
      if (slot.started) {
        // Episode starts zero the memory before the step that sees the first observation.
        out.starts[row] = 1;
        slot.started = false;
        collisions[i] = 0;
        if (m > 0) worker.state.fill(0, i * m, (i + 1) * m);
      }
      const env = ctx.envs[i]!;
      const pose = env.snapshot();
      const teacher = frontierAction(slot.spec!, env.visitedGrid, pose.pos, pose.yaw);
      out.teacher.set(teacher.subarray(0, ACT_DIM), row * ACT_DIM);
      privileged(env, slot, teacher, collisions[i]!, out.priv, row * PRIV_DIM);
      if (m > 0 && t % L === 0) out.state0.set(worker.state.subarray(i * m, (i + 1) * m), (i * (T / L) + t / L) * m);
    }
    const act = policy.act(packed, E, m > 0 ? worker.state : null, rng, false);
    for (let i = 0; i < E; i++) {
      const slot = slots[i]!;
      const env = ctx.envs[i]!;
      const row = i * T + t;
      const action = act.action.subarray(i * ACT_DIM, (i + 1) * ACT_DIM);
      out.actions.set(action, row * ACT_DIM);
      out.logp[row] = act.logp[i]!;
      const step = env.step(action);
      out.rewards[row] = step.reward;
      slot.reward += step.reward;
      slot.steps += 1;
      collisions[i] = step.info.collisions;
      slot.obs = step.obs;
      if (step.done) {
        out.dones[row] = 1;
        out.episodes.push({
          stage: slot.stage,
          night: slot.night,
          coverage: step.info.coverage,
          collisions: step.info.collisions,
          length: slot.steps,
          reward: slot.reward,
        });
        resetSlot(ctx, env, slot, p, rng);
      }
    }
  }
  for (let i = 0; i < E; i++) {
    const slot = slots[i]!;
    out.lastObs.set(slot.obs!, i * D);
    const env = ctx.envs[i]!;
    const pose = env.snapshot();
    const teacher = frontierAction(slot.spec!, env.visitedGrid, pose.pos, pose.yaw);
    privileged(env, slot, teacher, slot.started ? 0 : collisions[i]!, out.lastPriv, i * PRIV_DIM);
  }
  out.ms = performance.now() - t0;
  return out;
};

export interface SeqEvalPayload {
  policy: PolicySpec;
  jobs: EvalJob[];
}

export interface SeqEvalResult {
  coverage: number;
  collisions: number;
  steps: number;
}

let evalEnvs: ExplorerEnv[] = [];

/** Deterministic coverage episodes (eval protocol), run in lockstep so inference is batched. */
const seqEval: JobFn = (ctx, raw) => {
  const p = raw as SeqEvalPayload;
  worker ??= { slots: [], state: new Float32Array(0), policy: null, key: '' };
  const policy = policyFor(p.policy, ctx.obsDim);
  const m = policy.memDim;
  const D = ctx.obsDim;
  while (evalEnvs.length < p.jobs.length) evalEnvs.push(new ExplorerEnv());
  const live = p.jobs.map((job, k) => {
    const setup = evalEpisodeSetup(job.stage, job.seed, job.lighting);
    const env = evalEnvs[k]!;
    const obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
    return { env, obs, maxSteps: setup.cfg.maxSteps, steps: 0, coverage: 0, collisions: 0, done: false, state: new Float32Array(m) };
  });
  const rng = new RNG(1);
  for (;;) {
    const active = live.filter((e) => !e.done && e.steps < e.maxSteps);
    if (active.length === 0) break;
    const x = new Float32Array(active.length * D);
    const state = new Float32Array(active.length * m);
    active.forEach((e, k) => {
      x.set(e.obs, k * D);
      state.set(e.state, k * m);
    });
    const act = policy.act(x, active.length, m > 0 ? state : null, rng, true).action;
    active.forEach((e, k) => {
      if (m > 0) e.state.set(state.subarray(k * m, (k + 1) * m));
      const step = e.env.step(act.subarray(k * ACT_DIM, (k + 1) * ACT_DIM));
      e.obs = step.obs;
      e.coverage = step.info.coverage;
      e.collisions = step.info.collisions;
      e.steps += 1;
      if (step.done) e.done = true;
    });
  }
  const results: SeqEvalResult[] = live.map((e) => ({ coverage: e.coverage, collisions: e.collisions, steps: e.steps }));
  for (const e of live) e.env.dispose();
  return results;
};

export const jobs: Record<string, JobFn> = { seqRollout, seqEval };
