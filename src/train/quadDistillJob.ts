/**
 * Worker jobs for the quad student (scripts/train-quad.ts, docs/QUAD_STUDENT.md). Registered through
 * NodeRolloutPool `jobModules: ['src/train/quadDistillJob.ts']`.
 *
 * quadDagger flies randomized quadrotor episodes with a beta mixture of the student and the expert
 * (the default quad pilot: planner, learned goal picker, braking filter, trajectory tracker) and
 * labels every state with the expert's command. The expert is told what the airframe was actually
 * commanded each step, so its acceleration ramp starts from the student's command, not its own.
 * The states are packed compactly: the 64 scalar observation values and the command history as
 * float32, the 1024 map cells (all in [0, 1]) as bytes.
 *
 * quadEval flies the student alone (or the expert) and reports coverage and crashes.
 */
import { ACT_DIM, MAX_SPEED_H, OBS, OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import { guardedAction, quadPilot } from '../eval/pilots';
import { gaussianLogp, InferPolicy } from '../rl/infer';
import { StudentInput, STUDENT_DIM } from '../rl/quadStudent';
import { ExplorerEnv } from '../sim/env';
import { QUAD_PICKER_BRAKING } from '../sim/quadPilot';
import type { SensorPlannerOptions } from '../sim/sensorPlanner';
import { episodeSetup, type PickerEpisode } from './pickerJob';
import type { JobFn } from './rolloutCore';

/** Observation values before the maps (stored as float32). */
export const HEAD = OBS.localVisited;
/** Map cells (stored as bytes). */
export const MAPS = OBS_DIM - HEAD;
/** Float32 values per stored row: the observation head and the command history. */
export const HEAD_ROW = HEAD + (STUDENT_DIM - OBS_DIM);

export interface StudentWeights {
  hidden: number[];
  weights: ArrayBuffer[];
}

export interface DaggerPayload {
  episodes: PickerEpisode[];
  /** Student weights; null = the expert flies every step. */
  student: StudentWeights | null;
  /** Probability per step that the expert's command is executed instead of the student's. */
  beta: number;
  /** Std of Gaussian noise on executed horizontal and yaw commands (DART), action units. */
  noise: number;
  /** Weight multiplier of the states leading into a crash. */
  crashBoost: number;
  /** How many states before a crash get crashBoost. */
  crashLookback: number;
  /** Weight multiplier where student and expert commands differ by more than 0.3 (L2). */
  divergeBoost: number;
  /** Std of a slow random vertical command (Ornstein-Uhlenbeck, action units) added to executed commands, so off-height states get labeled. */
  vzNoise: number;
  /** Planner options layered over the default quad pilot's (plain numbers only), e.g. a wider braking margin. */
  expert?: ExpertKnobs;
  seed: number;
}

export type ExpertKnobs = Pick<SensorPlannerOptions, 'speed' | 'brakingMargin' | 'flyHeight'> & { reaction?: number; decel?: number };

/** The expert: the default quad pilot with `k` layered on top. */
export function makeExpert(k: ExpertKnobs | undefined): ReturnType<typeof quadPilot> {
  if (!k) return quadPilot();
  const { reaction, decel, ...rest } = k;
  const extra: SensorPlannerOptions = { ...rest };
  if (reaction != null || decel != null) extra.braking = { reaction: reaction ?? QUAD_PICKER_BRAKING.reaction, decel: decel ?? QUAD_PICKER_BRAKING.decel };
  return quadPilot(extra);
}

export interface DaggerChunk {
  n: number;
  head: Float32Array;
  maps: Uint8Array;
  labels: Float32Array;
  weights: Float32Array;
  episodes: { seed: number; lighting: string; coverage: number; crashed: boolean; steps: number }[];
  expertSteps: number;
}

export interface EvalPayload {
  episodes: PickerEpisode[];
  /** null = fly the expert itself. */
  student: StudentWeights | null;
  expert?: ExpertKnobs;
  /** Fly the student behind the braking shield. */
  shield?: boolean;
}

export interface EvalEpisode extends PickerEpisode {
  coverage: number;
  crashed: boolean;
  collisions: number;
  steps: number;
  /** Steps where the shield changed the command by more than 0.02. */
  interventions: number;
}

let env: ExplorerEnv | null = null;

/**
 * The braking shield: the default quad pilot's braking filter applied to the horizontal command
 * only (vertical and yaw pass through). Returns how far it moved the command (L2, action units).
 */
function shieldInPlace(a: Float32Array, obs: Float32Array, e: ExplorerEnv): number {
  const g = guardedAction(a, obs, e, QUAD_PICKER_BRAKING);
  const d = Math.hypot(g[0]! - a[0]!, g[1]! - a[1]!);
  a[0] = g[0]!;
  a[1] = g[1]!;
  return d;
}

function makeStudent(w: StudentWeights | null): InferPolicy | null {
  if (!w) return null;
  const p = new InferPolicy(STUDENT_DIM, ACT_DIM, w.hidden);
  p.setWeights(w.weights);
  return p;
}

/** World-frame horizontal command (m/s) of a body-frame action. */
function worldCommand(a: ArrayLike<number>, yaw: number, speedH: number): [number, number] {
  const f = Math.max(-1, Math.min(1, a[0]!)) * speedH;
  const l = Math.max(-1, Math.min(1, a[1]!)) * speedH;
  return [Math.cos(yaw) * f - Math.sin(yaw) * l, Math.sin(yaw) * f + Math.cos(yaw) * l];
}

const quadDagger: JobFn = (_ctx, raw) => {
  const p = raw as DaggerPayload;
  env ??= new ExplorerEnv();
  const student = makeStudent(p.student);
  const rng = new RNG(p.seed, 'quad-dagger');
  const head: number[] = [];
  const mapsParts: Uint8Array[] = [];
  const labels: number[] = [];
  const weights: number[] = [];
  const episodes: DaggerChunk['episodes'] = [];
  let expertSteps = 0;
  const input = new StudentInput();
  const exec = new Float32Array(ACT_DIM);
  for (const e of p.episodes) {
    const setup = episodeSetup(e);
    const expert = makeExpert(p.expert);
    expert.reset();
    input.reset();
    let obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
    const speedH = env.speedLimit ?? MAX_SPEED_H;
    const start = weights.length;
    let coverage = 0;
    let crashed = false;
    let steps = 0;
    let studentCmd: [number, number] | null = null;
    let ou = 0;
    const maps = new Uint8Array(setup.cfg.maxSteps * MAPS);
    for (let t = 0; t < setup.cfg.maxSteps; t++) {
      if (studentCmd) expert.planner.holdCommand(studentCmd[0], studentCmd[1]);
      const label = expert.act(obs, env);
      const row = input.build(obs);
      for (let i = 0; i < HEAD; i++) head.push(row[i]!);
      for (let i = OBS_DIM; i < STUDENT_DIM; i++) head.push(row[i]!);
      for (let i = 0; i < MAPS; i++) maps[t * MAPS + i] = Math.round(Math.max(0, Math.min(1, row[HEAD + i]!)) * 255);
      for (let i = 0; i < ACT_DIM; i++) labels.push(label[i]!);
      let w = 1;
      const useExpert = !student || rng.next() < p.beta;
      if (useExpert) {
        exec.set(label);
        expertSteps++;
        studentCmd = null;
      } else {
        const a = student.actWith(row, 1, rng, true).action;
        let d = 0;
        for (let i = 0; i < ACT_DIM; i++) d += (a[i]! - label[i]!) ** 2;
        if (Math.sqrt(d) > 0.3) w *= p.divergeBoost;
        exec.set(a);
      }
      if (p.noise > 0) {
        exec[0] = Math.max(-1, Math.min(1, exec[0]! + rng.gaussian() * p.noise));
        exec[1] = Math.max(-1, Math.min(1, exec[1]! + rng.gaussian() * p.noise));
        exec[3] = Math.max(-1, Math.min(1, exec[3]! + rng.gaussian() * p.noise * 0.5));
      }
      if (p.vzNoise > 0) {
        // Stationary std p.vzNoise with a ~1.5 s correlation time.
        ou = 0.967 * ou + rng.gaussian() * p.vzNoise * 0.255;
        exec[2] = Math.max(-1, Math.min(1, exec[2]! + ou));
      }
      if (!useExpert || p.noise > 0) studentCmd = worldCommand(exec, env.snapshot().yaw, speedH);
      weights.push(w);
      input.executed(exec);
      const step = env.step(exec);
      obs = step.obs;
      coverage = step.info.coverage;
      steps = t + 1;
      if (step.done) {
        crashed = step.info.crashed;
        break;
      }
    }
    if (crashed) for (let k = Math.max(start, weights.length - p.crashLookback); k < weights.length; k++) weights[k]! *= p.crashBoost;
    mapsParts.push(maps.subarray(0, steps * MAPS));
    episodes.push({ seed: e.seed, lighting: e.lighting, coverage, crashed, steps });
  }
  const n = weights.length;
  const allMaps = new Uint8Array(n * MAPS);
  let off = 0;
  for (const m of mapsParts) {
    allMaps.set(m, off);
    off += m.length;
  }
  const out: DaggerChunk = {
    n,
    head: Float32Array.from(head),
    maps: allMaps,
    labels: Float32Array.from(labels),
    weights: Float32Array.from(weights),
    episodes,
    expertSteps,
  };
  return out;
};

const quadEval: JobFn = (_ctx, raw) => {
  const p = raw as EvalPayload;
  env ??= new ExplorerEnv();
  const student = makeStudent(p.student);
  const rng = new RNG(1, 'quad-eval');
  const input = new StudentInput();
  const out: EvalEpisode[] = [];
  for (const e of p.episodes) {
    const setup = episodeSetup(e);
    const expert = student ? null : makeExpert(p.expert);
    expert?.reset();
    input.reset();
    let obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
    let coverage = 0;
    let crashed = false;
    let collisions = 0;
    let steps = 0;
    let interventions = 0;
    for (let t = 0; t < setup.cfg.maxSteps; t++) {
      let a: Float32Array;
      if (student) {
        a = student.actWith(input.build(obs), 1, rng, true).action;
        if (p.shield && shieldInPlace(a, obs, env) > 0.02) interventions++;
        input.executed(a);
      } else a = expert!.act(obs, env);
      const step = env.step(a);
      obs = step.obs;
      coverage = step.info.coverage;
      collisions = step.info.collisions;
      steps = t + 1;
      if (step.done) {
        crashed = step.info.crashed;
        break;
      }
    }
    out.push({ ...e, coverage, crashed, collisions, steps, interventions });
  }
  return out;
};

export interface RolloutPayload {
  episodes: PickerEpisode[];
  student: StudentWeights;
  /** Added to the reward when a crash ends the episode (before scaling). */
  crashPenalty: number;
  /** Reward scale. */
  rewardScale: number;
  /** Label every state with the expert's command (for the imitation anchor). */
  labels: boolean;
  expert?: ExpertKnobs;
  /** Execute through the braking shield; the policy's own sample is what PPO trains on. */
  shield?: boolean;
  /** Reward (before scaling) subtracted per unit of shield correction (L2, action units). */
  shieldPenalty?: number;
  seed: number;
}

export interface RolloutChunk extends DaggerChunk {
  /** Executed (squashed) actions, log-probs and values of the sampling policy. */
  actions: Float32Array;
  logp: Float32Array;
  values: Float32Array;
  rewards: Float32Array;
  /** 1 on the last step of an episode. */
  dones: Uint8Array;
}

/** Largest |action| a rollout samples. */
const A_MAX = 0.995;

/** On-policy episodes of the stochastic student, for PPO (scripts/train-quad.ts --rl). */
const quadRollout: JobFn = (_ctx, raw) => {
  const p = raw as RolloutPayload;
  env ??= new ExplorerEnv();
  const student = makeStudent(p.student)!;
  const logStd = student.logStd();
  const rng = new RNG(p.seed, 'quad-rollout');
  const input = new StudentInput();
  const total = p.episodes.length * 2400;
  const head = new Float32Array(total * HEAD_ROW);
  const maps = new Uint8Array(total * MAPS);
  const labels = new Float32Array(total * ACT_DIM);
  const actions = new Float32Array(total * ACT_DIM);
  const logp = new Float32Array(total);
  const values = new Float32Array(total);
  const rewards = new Float32Array(total);
  const dones = new Uint8Array(total);
  const episodes: DaggerChunk['episodes'] = [];
  let n = 0;
  for (const e of p.episodes) {
    const setup = episodeSetup(e);
    const expert = p.labels ? makeExpert(p.expert) : null;
    expert?.reset();
    input.reset();
    let obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
    const speedH = env.speedLimit ?? MAX_SPEED_H;
    let coverage = 0;
    let crashed = false;
    let steps = 0;
    let lastExec = new Float32Array(ACT_DIM);
    for (let t = 0; t < setup.cfg.maxSteps && n < total; t++) {
      const row = input.build(obs);
      head.set(row.subarray(0, HEAD), n * HEAD_ROW);
      head.set(row.subarray(OBS_DIM, STUDENT_DIM), n * HEAD_ROW + HEAD);
      for (let i = 0; i < MAPS; i++) maps[n * MAPS + i] = Math.round(Math.max(0, Math.min(1, row[HEAD + i]!)) * 255);
      // Sample a squashed Gaussian action, kept within ±A_MAX, and score it exactly as the learner
      // will (src/rl/model.ts squashedLogp on the stored float32 action). Without the bound, a
      // saturated mean gives tanh(u) = ±1 in float32 and the learner's log-prob disagrees wildly.
      const mv = student.meanAndValue(row, 1);
      const a = new Float32Array(ACT_DIM);
      let lp = 0;
      for (let j = 0; j < ACT_DIM; j++) {
        const m = mv.mean[j]!;
        const ls = logStd[j]!;
        const sd = Math.exp(ls);
        a[j] = Math.max(-A_MAX, Math.min(A_MAX, Math.tanh(m + sd * rng.gaussian())));
        const u = Math.atanh(a[j]!);
        lp += gaussianLogp(u, m, sd, ls) - Math.log(1 - a[j]! * a[j]! + 1e-6);
      }
      const out = { logp: [lp], value: mv.value };
      if (expert) {
        if (t > 0) {
          const c = worldCommand(lastExec, env.snapshot().yaw, speedH);
          expert.planner.holdCommand(c[0], c[1]);
        }
        labels.set(expert.act(obs, env).subarray(0, ACT_DIM), n * ACT_DIM);
      }
      actions.set(a, n * ACT_DIM);
      logp[n] = out.logp[0]!;
      values[n] = out.value[0]!;
      const exec = a.slice();
      lastExec = exec;
      const corr = p.shield ? shieldInPlace(exec, obs, env) : 0;
      input.executed(exec);
      const step = env.step(exec);
      let r = step.reward - corr * (p.shieldPenalty ?? 0);
      if (step.done && step.info.crashed) r += p.crashPenalty;
      rewards[n] = r * p.rewardScale;
      obs = step.obs;
      coverage = step.info.coverage;
      steps = t + 1;
      n++;
      if (step.done) {
        crashed = step.info.crashed;
        break;
      }
    }
    dones[n - 1] = 1;
    episodes.push({ seed: e.seed, lighting: e.lighting, coverage, crashed, steps });
  }
  const out: RolloutChunk = {
    n,
    head: head.slice(0, n * HEAD_ROW),
    maps: maps.slice(0, n * MAPS),
    labels: labels.slice(0, n * ACT_DIM),
    weights: new Float32Array(n).fill(1),
    episodes,
    expertSteps: 0,
    actions: actions.slice(0, n * ACT_DIM),
    logp: logp.slice(0, n),
    values: values.slice(0, n),
    rewards: rewards.slice(0, n),
    dones: dones.slice(0, n),
  };
  return out;
};

export const jobs: Record<string, JobFn> = { quadDagger, quadEval, quadRollout };
