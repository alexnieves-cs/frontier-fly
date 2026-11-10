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
import { classicalPilot, guardedAction } from '../eval/pilots';
import { MemPolicy } from '../rl/memPolicy';
import { ACT_DIM, CRASH_SPEED, MAX_SPEED_H, MAX_YAW_RATE } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec } from '../shared/types';
import { defaultDynParams, sampleDynParams, sampleNoise } from '../sim/dynamics/dr';
import { ExplorerEnv, policyEpisodeLimits } from '../sim/env';
import { frontierAction, frontierDistance } from '../sim/frontier';
import { QUAD_BRAKING } from '../sim/lidarSafety';
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
  /**
   * Extra reward on a crash (CRASH_SPEED), on top of the env's REWARD.collision + REWARD.crash. The
   * crash already ends the episode, so the lost future reward is most of the signal; this sharpens it.
   */
  crashPenalty?: number;
  /** Share of episodes flown with quadrotor dynamics, domain-randomized at full scale (as the coverage study's quad condition). */
  quadFraction?: number;
  /**
   * What the BC labels (the `teacher` rows) are: the frontier teacher's action (default), the teacher's
   * action through the lidar guard (src/eval/pilots.ts guardedAction), or the policy's own action
   * through the guard, which teaches the actor to brake the way the guard does so it needs no guard.
   * 'brake-self' is 'guard-self' with, on quadrotor episodes, the guard's braking model of the airframe
   * (lidarSafety.ts QUAD_BRAKING: slow down while there is still room to stop) and the yaw rate capped
   * at `labelYawRate`; the plain guard only cuts a contact 0.2 s ahead, too late for an airframe that
   * has to tilt back to stop. 'planner' labels quadrotor episodes with the sensor-only planner that flies
   * the quad without crashing (the study's classicalBrakeTrack pilot: braking filter and trajectory
   * tracker), run beside the policy on the policy's own states; other episodes are labelled as 'guard-self'.
   */
  labels?: 'teacher' | 'guard-teacher' | 'guard-self' | 'brake-self' | 'planner';
  /**
   * Fly quadrotor episodes through the braking guard (the 'guardedBrake' pilot of the coverage study):
   * the env executes guardedAction(action, QUAD_BRAKING), PPO still learns on the policy's own action.
   * The guard is then part of what the policy flies, as it is when the guarded policy is deployed.
   */
  quadGuard?: boolean;
  /** With labels 'brake-self': yaw rate cap of the quadrotor labels, rad/s (0 = none). */
  labelYawRate?: number;
  /** With labels 'guard-self': the label moves this many times the guard's horizontal change (1 = the guard's action). */
  guardGain?: number;
  /** Penalty per contact, in reward per m/s of impact speed above half the crash speed (0 = off). */
  impactSpeedPenalty?: number;
}

export interface SeqEpisode {
  stage: number;
  night: boolean;
  coverage: number;
  collisions: number;
  /** The crash rule ended the episode. */
  crashed: boolean;
  quad: boolean;
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
  quad: boolean;
  maxSteps: number;
  steps: number;
  reward: number;
  started: boolean;
  /** Labels 'planner': the planner shadowing this env's quad episode, and the last command the env was given (world frame). */
  shadow?: ReturnType<typeof classicalPilot>;
  cmd: [number, number];
}

interface WorkerState {
  slots: EnvSlot[];
  state: Float32Array;
  policy: MemPolicy | null;
  key: string;
}

let worker: WorkerState | null = null;

const clamp1 = (v: number) => Math.max(-1, Math.min(1, v));

/** Contacts slower than this are free under --impact-speed-penalty: half the crash speed. */
const CONTACT_FREE_SPEED = CRASH_SPEED / 2;

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
  const quad = rng.next() < (p.quadFraction ?? 0);
  const cfg = makeConfig(stage, envSeed, nRooms, quad ? 'quad' : 'kinematic');
  if (quad) {
    const dr = new RNG(envSeed, 'ppo-dr');
    cfg.dyn = sampleDynParams(dr, defaultDynParams(), 1);
    cfg.noise = sampleNoise(dr, 1, night);
  }
  cfg.lighting = lightingPreset(night ? 'night-dark' : 'day', nRooms);
  Object.assign(cfg, policyEpisodeLimits(stage));
  slot.obs = env.reset(spec, envSeed, cfg, world.physics);
  ctx.worlds.release(slot.world);
  slot.world = world;
  slot.spec = spec;
  slot.stage = stage;
  slot.night = night;
  slot.quad = quad;
  slot.maxSteps = cfg.maxSteps!;
  slot.steps = 0;
  slot.reward = 0;
  slot.started = true;
  slot.cmd = [0, 0];
  if (p.labels === 'planner' && quad) {
    slot.shadow ??= classicalPilot({ tracker: {}, braking: QUAD_BRAKING });
    slot.shadow.reset();
  }
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
    worker.slots = ctx.envs.map(() => ({ obs: null, world: null, spec: null, stage: 0, night: false, quad: false, maxSteps: 1, steps: 0, reward: 0, started: false, cmd: [0, 0] }));
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
      if (p.labels === 'guard-teacher') {
        const g = guardedAction(teacher, slot.obs!, env);
        out.teacher[row * ACT_DIM] = g[0]!;
        out.teacher[row * ACT_DIM + 1] = g[1]!;
      }
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
      if (p.labels === 'planner' && slot.quad) {
        // The planner's command for this state, ramped from the command the airframe actually got last step.
        const shadow = slot.shadow!;
        if (slot.steps > 0) shadow.planner.holdCommand(slot.cmd[0], slot.cmd[1]);
        out.teacher.set(shadow.act(slot.obs!, env), row * ACT_DIM);
      } else if (p.labels === 'guard-self' || p.labels === 'brake-self' || p.labels === 'planner') {
        // Horizontal velocity only: the guard also holds a fixed height, which the policy should not copy.
        const braking = p.labels === 'brake-self' && slot.quad;
        const g = guardedAction(action, slot.obs!, env, braking ? QUAD_BRAKING : undefined);
        // guardGain > 1 pulls harder where the guard brakes (the label moves k times the guard's change).
        const k = p.guardGain ?? 1;
        out.teacher.set(action, row * ACT_DIM);
        out.teacher[row * ACT_DIM] = clamp1(action[0]! + k * (g[0]! - action[0]!));
        out.teacher[row * ACT_DIM + 1] = clamp1(action[1]! + k * (g[1]! - action[1]!));
        if (braking && p.labelYawRate) {
          const lim = p.labelYawRate / MAX_YAW_RATE;
          out.teacher[row * ACT_DIM + 3] = Math.max(-lim, Math.min(lim, action[3]!));
        }
      }
      out.logp[row] = act.logp[i]!;
      const exec = p.quadGuard && slot.quad ? guardedAction(action, slot.obs!, env, QUAD_BRAKING) : action;
      if (slot.shadow && slot.quad) {
        const yaw = env.snapshot().yaw;
        const f = clamp1(exec[0]!) * MAX_SPEED_H;
        const l = clamp1(exec[1]!) * MAX_SPEED_H;
        slot.cmd = [Math.cos(yaw) * f - Math.sin(yaw) * l, Math.sin(yaw) * f + Math.cos(yaw) * l];
      }
      const step = env.step(exec);
      let reward = step.reward;
      if (step.info.crashed && step.done) reward += p.crashPenalty ?? 0;
      if (p.impactSpeedPenalty && step.info.impactSpeed > CONTACT_FREE_SPEED) {
        reward -= p.impactSpeedPenalty * (step.info.impactSpeed - CONTACT_FREE_SPEED);
      }
      out.rewards[row] = reward;
      slot.reward += reward;
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
          crashed: step.info.crashed,
          quad: slot.quad,
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
  jobs: (EvalJob & { quad?: boolean; guard?: boolean })[];
}

export interface SeqEvalResult {
  coverage: number;
  collisions: number;
  crashed: boolean;
  steps: number;
}

const evalEnvs: ExplorerEnv[] = [];

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
    if (job.quad) {
      // As the coverage study's quad condition (scripts/study/coverage-study.ts startEpisode).
      const dr = new RNG(job.seed, 'study-dr');
      setup.cfg.dynamics = 'quad';
      setup.cfg.dyn = sampleDynParams(dr, defaultDynParams(), 1);
      setup.cfg.noise = sampleNoise(dr, 1, job.lighting !== 'day');
    }
    const env = evalEnvs[k]!;
    const obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
    return { env, obs, guard: !!job.guard, maxSteps: setup.cfg.maxSteps, steps: 0, coverage: 0, collisions: 0, crashed: false, done: false, state: new Float32Array(m) };
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
      const a = act.subarray(k * ACT_DIM, (k + 1) * ACT_DIM);
      // A guarded quad job flies the coverage study's 'guardedBrake' pilot.
      const step = e.env.step(e.guard ? guardedAction(a, e.obs, e.env, QUAD_BRAKING) : a);
      e.obs = step.obs;
      e.coverage = step.info.coverage;
      e.collisions = step.info.collisions;
      e.crashed = step.info.crashed;
      e.steps += 1;
      if (step.done) e.done = true;
    });
  }
  const results: SeqEvalResult[] = live.map((e) => ({ coverage: e.coverage, collisions: e.collisions, crashed: e.crashed, steps: e.steps }));
  for (const e of live) e.env.dispose();
  return results;
};

export const jobs: Record<string, JobFn> = { seqRollout, seqEval };
