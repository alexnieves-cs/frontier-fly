import { generateBuilding } from '../gen/building';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { EpisodeConfig } from '../shared/types';
import { Curriculum } from './curriculum';
import { InferPolicy } from './infer';
import { TfTrainer } from './model';
import { RunningNorm } from './normalize';
import { computeGae } from './ppo';
import type { Checkpoint } from './checkpoint';
import { createEnv, defaultEpisodeConfig } from '../sim/env';
import { frontierAction } from '../sim/frontier';
import { lightingPreset } from '../sim/lighting';
import { collectRollout } from '../workers/rollout';

/** Headless PPO on the explorer. Used to ship a starting checkpoint. */
export async function trainExplorer(opts?: {
  updates?: number;
  envs?: number;
  steps?: number;
  seed?: number;
  onUpdate?: (info: { update: number; meanCoverage: number; stepsPerSec: number; episodes: number }) => void;
}): Promise<{ checkpoint: Checkpoint; meanCoverage: number; stepsPerSec: number }> {
  const updates = opts?.updates ?? 8;
  const nEnvs = opts?.envs ?? 2;
  const steps = opts?.steps ?? 64;
  const seed = opts?.seed ?? 11;
  const policy = new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, seed);
  const trainer = new TfTrainer(policy, 3e-4);
  await trainer.ready();
  const norm = new RunningNorm(OBS_DIM);
  const curriculum = new Curriculum({ window: 50 });
  const envs = [];
  const specs = [];
  const obs = [];
  for (let i = 0; i < nEnvs; i++) {
    const env = await createEnv();
    const params = curriculum.genParams(0, new RNG(seed + i, 'boot'));
    const spec = generateBuilding(params);
    const first = env.reset(spec, params.seed);
    envs.push(env);
    specs.push(spec);
    obs.push(first);
  }
  let meanCoverage = 0;
  let stepsPerSec = 0;
  for (let update = 0; update < updates; update++) {
    const outcome = await collectRollout({
      envs,
      specs,
      obs,
      policy,
      norm,
      curriculum,
      steps,
      baseSeed: seed + update * 1009,
      stageOverride: 0,
    });
    const arrays = outcome.buffer.toArrays();
    const last = arrays.n > 0 ? arrays.values[arrays.n - 1]! : 0;
    const gae = computeGae({
      rewards: arrays.rewards,
      values: arrays.values,
      dones: arrays.dones,
      truncated: arrays.truncated,
      nextValue: last,
    });
    await trainer.update(
      {
        obs: arrays.obs,
        actions: arrays.actions,
        logp: arrays.logp,
        advantages: gae.advantages,
        returns: gae.returns,
        n: arrays.n,
      },
      { epochs: 3, minibatch: 128 },
    );
    trainer.pullWeights();
    meanCoverage = outcome.episodes.length
      ? outcome.episodes.reduce((s, e) => s + e.coverage, 0) / outcome.episodes.length
      : meanCoverage;
    stepsPerSec = outcome.stepsPerSec;
    opts?.onUpdate?.({ update, meanCoverage, stepsPerSec, episodes: outcome.episodes.length });
  }
  const state = norm.toJSON();
  const checkpoint: Checkpoint = {
    version: 1,
    obsDim: OBS_DIM,
    actDim: ACT_DIM,
    hidden: [...HIDDEN],
    weights: policy.getWeights().map((buf) => Array.from(new Float32Array(buf))),
    obsMean: state.mean,
    obsVar: state.var,
    obsCount: state.count,
    rewVar: state.rewVar,
    stage: curriculum.stage,
  };
  return { checkpoint, meanCoverage, stepsPerSec };
}

function episodeConfig(stage: number, nRooms: number, lighting: EpisodeConfig['lighting']): EpisodeConfig {
  const cfg = defaultEpisodeConfig(stage, nRooms);
  cfg.maxSteps = stage <= 1 ? 1000 : 2400;
  cfg.stallSteps = 500;
  cfg.maxCollisions = 16;
  cfg.dynamics = 'kinematic';
  cfg.lighting = lighting;
  return cfg;
}

async function coverEpisode(
  policy: InferPolicy,
  stage: number,
  seed: number,
  lightingName: 'day' | 'night-dark',
): Promise<number> {
  const env = await createEnv();
  const curriculum = new Curriculum();
  const params = curriculum.genParams(stage, new RNG(seed, `eval-${stage}`));
  const spec = generateBuilding(params);
  const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
  const lighting = lightingPreset(lightingName, nRooms);
  let obs = env.reset(spec, params.seed, episodeConfig(stage, nRooms, lighting));
  let coverage = 0;
  const steps = stage <= 1 ? 1000 : 2400;
  for (let t = 0; t < steps; t++) {
    const action = policy.actWith(obs, 1, new RNG(1), true).action;
    const step = env.step(action);
    obs = step.obs;
    coverage = step.info.coverage;
    if (step.done) break;
  }
  return coverage;
}

/**
 * Fit the policy to the privileged frontier teacher, then keep that checkpoint.
 * The teacher uses the visited grid; the network only sees the standard observation.
 */
export async function distillExplorer(opts?: {
  epochs?: number;
  onEpoch?: (epoch: number, loss: number) => void;
}): Promise<{ checkpoint: Checkpoint; stage0: number; stage3: number; nightToDay: number }> {
  const epochs = opts?.epochs ?? 8;
  const policy = new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, 11);
  const trainer = new TfTrainer(policy, 3e-4);
  await trainer.ready();
  const env = await createEnv();
  const curriculum = new Curriculum();
  const cap = 12_000;
  const obsBuf = new Float32Array(cap * OBS_DIM);
  const actBuf = new Float32Array(cap * ACT_DIM);
  let n = 0;
  const jobs: { stage: number; seed: number }[] = [];
  for (let i = 0; i < 8; i++) jobs.push({ stage: 0, seed: 20 + i * 3 });
  for (let i = 0; i < 6; i++) jobs.push({ stage: 3, seed: 80 + i * 5 });
  for (const task of jobs) {
    if (n >= cap) break;
    const params = curriculum.genParams(task.stage, new RNG(task.seed, 'teach'));
    const spec = generateBuilding(params);
    const nRooms = spec.floors.reduce((rooms, floor) => rooms + floor.rooms.length, 0);
    let obs = env.reset(spec, params.seed, episodeConfig(task.stage, nRooms, lightingPreset('day', nRooms)));
    const limit = Math.min(task.stage === 0 ? 780 : 900, cap - n);
    for (let t = 0; t < limit; t++) {
      const pose = env.snapshot();
      const teacher = frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw);
      obsBuf.set(obs, n * OBS_DIM);
      actBuf.set(teacher, n * ACT_DIM);
      n += 1;
      const step = env.step(teacher);
      obs = step.obs;
      if (step.done) break;
    }
  }
  await trainer.imitate(obsBuf, actBuf, n, {
    epochs,
    minibatch: 512,
    onEpoch: opts?.onEpoch,
  });
  let mae = 0;
  const probe = Math.min(n, 128);
  for (let i = 0; i < probe; i++) {
    const pred = policy.actWith(obsBuf.subarray(i * OBS_DIM, (i + 1) * OBS_DIM), 1, new RNG(1), true).action;
    for (let j = 0; j < ACT_DIM; j++) mae += Math.abs((pred[j] ?? 0) - (actBuf[i * ACT_DIM + j] ?? 0));
  }
  if (probe > 0) console.info(`distill action mae ${(mae / (probe * ACT_DIM)).toFixed(3)} on ${n} samples`);
  const updates = 24;
  const horizon = 384;
  let bestStage0 = await coverEpisode(policy, 0, 101, 'day');
  let bestWeights = policy.getWeights();
  console.info(`ppo start stage0 ${bestStage0.toFixed(3)}`);
  for (let update = 0; update < updates; update++) {
    const stage = update % 3 === 2 ? 3 : 0;
    const params = curriculum.genParams(stage, new RNG(500 + update * 19, 'ppo'));
    const spec = generateBuilding(params);
    const nRooms = spec.floors.reduce((rooms, floor) => rooms + floor.rooms.length, 0);
    let obs = env.reset(spec, params.seed, episodeConfig(stage, nRooms, lightingPreset('day', nRooms)));
    const obsA = new Float32Array(horizon * OBS_DIM);
    const actA = new Float32Array(horizon * ACT_DIM);
    const logpA = new Float32Array(horizon);
    const valuesA = new Float32Array(horizon);
    const rewardsA = new Float32Array(horizon);
    const donesA = new Uint8Array(horizon);
    const truncA = new Uint8Array(horizon);
    for (let t = 0; t < horizon; t++) {
      const out = policy.act(obs, 1);
      obsA.set(obs, t * OBS_DIM);
      actA.set(out.action, t * ACT_DIM);
      logpA[t] = out.logp[0] ?? 0;
      valuesA[t] = out.value[0] ?? 0;
      const step = env.step(out.action);
      rewardsA[t] = step.reward;
      donesA[t] = step.done && !step.truncated ? 1 : 0;
      truncA[t] = step.truncated ? 1 : 0;
      obs = step.obs;
      if (step.done) {
        const next = curriculum.genParams(stage, new RNG(800 + update * 31 + t, 'ppo-reset'));
        const nextSpec = generateBuilding(next);
        const rooms = nextSpec.floors.reduce((count, floor) => count + floor.rooms.length, 0);
        obs = env.reset(nextSpec, next.seed, episodeConfig(stage, rooms, lightingPreset('day', rooms)));
      }
    }
    const gae = computeGae({
      rewards: rewardsA,
      values: valuesA,
      dones: donesA,
      truncated: truncA,
      nextValue: valuesA[horizon - 1] ?? 0,
    });
    const losses = await trainer.update(
      {
        obs: obsA,
        actions: actA,
        logp: logpA,
        advantages: gae.advantages,
        returns: gae.returns,
        n: horizon,
      },
      { epochs: 2, minibatch: 192, entropyCoef: 0.01, valueCoef: 0.15 },
    );
    if (update % 3 === 2) {
      const cov = await coverEpisode(policy, 0, 101, 'day');
      console.info(`ppo update ${update + 1} stage0 ${cov.toFixed(3)} pi ${losses.policyLoss.toFixed(3)} v ${losses.valueLoss.toFixed(3)}`);
      if (cov > bestStage0) {
        bestStage0 = cov;
        bestWeights = policy.getWeights();
      }
    }
  }
  policy.setWeights(bestWeights);
  const stage0 = await coverEpisode(policy, 0, 101, 'day');
  const stage0Night = await coverEpisode(policy, 0, 101, 'night-dark');
  const stage3 = await coverEpisode(policy, 3, 4242, 'day');
  const night = await coverEpisode(policy, 3, 4242, 'night-dark');
  console.info(`night/day stage0 ${stage0 > 0 ? (stage0Night / stage0).toFixed(3) : 'n/a'} stage3 ${stage3 > 0 ? (night / stage3).toFixed(3) : 'n/a'}`);
  const blank: Checkpoint = {
    version: 1,
    obsDim: OBS_DIM,
    actDim: ACT_DIM,
    hidden: [...HIDDEN],
    weights: policy.getWeights().map((buf) => Array.from(new Float32Array(buf))),
    obsMean: new Array(OBS_DIM).fill(0),
    obsVar: new Array(OBS_DIM).fill(1),
    obsCount: 1,
    rewVar: 1,
    stage: 3,
  };
  return { checkpoint: blank, stage0, stage3, nightToDay: stage0 > 0 ? stage0Night / stage0 : 0 };
}

/** More PPO updates from a checkpoint already on disk. */
export async function continueExplorer(
  checkpoint: Checkpoint,
  updates = 24,
): Promise<{ checkpoint: Checkpoint; stage0: number; stage3: number; nightToDay: number }> {
  const policy = new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, 11);
  policy.setWeights(checkpoint.weights.map((row) => Float32Array.from(row).buffer));
  const trainer = new TfTrainer(policy, 2e-4);
  await trainer.ready();
  const env = await createEnv();
  const curriculum = new Curriculum();
  let bestStage0 = await coverEpisode(policy, 0, 101, 'day');
  let bestWeights = policy.getWeights();
  console.info(`continue start stage0 ${bestStage0.toFixed(3)}`);
  const horizon = 384;
  for (let update = 0; update < updates; update++) {
    const stage = update % 5 === 4 ? 3 : 0;
    const params = curriculum.genParams(stage, new RNG(900 + update * 19, 'cont'));
    const spec = generateBuilding(params);
    const nRooms = spec.floors.reduce((rooms, floor) => rooms + floor.rooms.length, 0);
    let obs = env.reset(spec, params.seed, episodeConfig(stage, nRooms, lightingPreset('day', nRooms)));
    const obsA = new Float32Array(horizon * OBS_DIM);
    const actA = new Float32Array(horizon * ACT_DIM);
    const logpA = new Float32Array(horizon);
    const valuesA = new Float32Array(horizon);
    const rewardsA = new Float32Array(horizon);
    const donesA = new Uint8Array(horizon);
    const truncA = new Uint8Array(horizon);
    for (let t = 0; t < horizon; t++) {
      const out = policy.act(obs, 1);
      obsA.set(obs, t * OBS_DIM);
      actA.set(out.action, t * ACT_DIM);
      logpA[t] = out.logp[0] ?? 0;
      valuesA[t] = out.value[0] ?? 0;
      const step = env.step(out.action);
      rewardsA[t] = step.reward;
      donesA[t] = step.done && !step.truncated ? 1 : 0;
      truncA[t] = step.truncated ? 1 : 0;
      obs = step.obs;
      if (step.done) {
        const next = curriculum.genParams(stage, new RNG(1200 + update * 13 + t, 'cont-reset'));
        const nextSpec = generateBuilding(next);
        const rooms = nextSpec.floors.reduce((count, floor) => count + floor.rooms.length, 0);
        obs = env.reset(nextSpec, next.seed, episodeConfig(stage, rooms, lightingPreset('day', rooms)));
      }
    }
    const gae = computeGae({
      rewards: rewardsA,
      values: valuesA,
      dones: donesA,
      truncated: truncA,
      nextValue: valuesA[horizon - 1] ?? 0,
    });
    await trainer.update(
      { obs: obsA, actions: actA, logp: logpA, advantages: gae.advantages, returns: gae.returns, n: horizon },
      { epochs: 2, minibatch: 192, entropyCoef: 0.005, valueCoef: 0.1 },
    );
    if (update % 4 === 3) {
      const cov = await coverEpisode(policy, 0, 101, 'day');
      console.info(`continue update ${update + 1} stage0 ${cov.toFixed(3)}`);
      if (cov > bestStage0) {
        bestStage0 = cov;
        bestWeights = policy.getWeights();
      }
    }
  }
  policy.setWeights(bestWeights);
  const stage0 = await coverEpisode(policy, 0, 101, 'day');
  const stage0Night = await coverEpisode(policy, 0, 101, 'night-dark');
  const stage3 = await coverEpisode(policy, 3, 4242, 'day');
  const night = await coverEpisode(policy, 3, 4242, 'night-dark');
  console.info(`night/day stage0 ${stage0 > 0 ? (stage0Night / stage0).toFixed(3) : 'n/a'} stage3 ${stage3 > 0 ? (night / stage3).toFixed(3) : 'n/a'}`);
  return {
    checkpoint: { ...checkpoint, weights: policy.getWeights().map((buf) => Array.from(new Float32Array(buf))), stage: 0 },
    stage0,
    stage3,
    nightToDay: stage0 > 0 ? stage0Night / stage0 : 0,
  };
}
