import { generateBuilding } from '../gen/building';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import { Curriculum } from './curriculum';
import { InferPolicy } from './infer';
import { TfTrainer } from './model';
import { RunningNorm } from './normalize';
import { computeGae } from './ppo';
import type { Checkpoint } from './checkpoint';
import { createEnv } from '../sim/env';
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
