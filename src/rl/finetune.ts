/**
 * Anchored PPO fine-tune for a DAgger checkpoint.
 *
 * Plain PPO from a cloned policy tends to overwrite the cloned skill because
 * the critic starts random and early advantages are noise. This driver
 *   1. warms the value trunk up on on-policy returns (policy frozen),
 *   2. runs PPO with a KL(pi_ref || pi) anchor to the frozen start policy and a
 *      BC anchor on the DAgger dataset, both coefficients annealed linearly,
 *   3. validates on held-out training-distribution buildings (never the eval
 *      seeds) and returns the best weights seen, including the start, so the
 *      result is never worse than the input on that validation set.
 */
import { RNG } from '../shared/rng';
import { createEnv } from '../sim/env';
import type { LightingPreset } from '../sim/lighting';
import type { Checkpoint } from './checkpoint';
import {
  checkpointFromPolicy,
  defaultValidationSet,
  evalLikeConfig,
  policyFromCheckpoint,
  trainingBuilding,
  validationCoverage,
  type ValidationEpisode,
  type ValidationResult,
} from './dagger';
import { TfTrainer, type BcData, type PpoMetrics } from './model';
import { computeGae } from './ppo';

export type Schedule = number | { start: number; end: number };

function scheduled(s: Schedule, frac: number): number {
  if (typeof s === 'number') return s;
  return s.start + (s.end - s.start) * Math.min(1, Math.max(0, frac));
}

export interface PpoFinetuneOpts {
  checkpoint: Checkpoint;
  /** DAgger dataset view for the BC anchor. Without it the BC term is off. */
  dataset?: BcData;
  updates?: number;
  /** Env steps per update. */
  horizon?: number;
  stages?: number[];
  stageWeights?: number[];
  nightFraction?: number;
  seed?: number;
  lr?: number;
  epochs?: number;
  minibatch?: number;
  clipEps?: number;
  entropyCoef?: number;
  valueCoef?: number;
  /** KL(pi_ref || pi) coefficient, constant or linearly annealed over the updates. */
  klCoef?: Schedule;
  /** BC anchor coefficient, constant or linearly annealed. */
  bcCoef?: Schedule;
  bcMinibatch?: number;
  /** Leading updates that train only the value trunk. */
  valueWarmup?: number;
  /** Set the policy log std before fine-tuning (exploration noise). null keeps the checkpoint's. */
  initLogStd?: number | null;
  /** Rewards are multiplied by this before GAE. */
  rewardScale?: number;
  /** Validate every this many PPO updates (and at the end). */
  evalEvery?: number;
  validation?: ValidationEpisode[];
  onUpdate?: (m: FinetuneMetrics) => void;
}

export interface FinetuneMetrics {
  update: number;
  valueOnly: boolean;
  policyLoss: number;
  valueLoss: number;
  entropy: number;
  kl: number;
  anchorKl: number;
  bcLoss: number;
  klCoef: number;
  bcCoef: number;
  /** Mean coverage of episodes finished during this update's rollout (or the running one's). */
  rolloutCoverage: number;
  validation?: ValidationResult;
}

export interface PpoFinetuneResult {
  checkpoint: Checkpoint;
  startValidation: ValidationResult;
  bestValidation: ValidationResult;
  /** -1 when the start weights were kept. */
  bestUpdate: number;
  metrics: FinetuneMetrics[];
}

export async function ppoFinetune(opts: PpoFinetuneOpts): Promise<PpoFinetuneResult> {
  const seed = opts.seed ?? 11;
  const updates = opts.updates ?? 24;
  const horizon = opts.horizon ?? 1024;
  const stages = opts.stages ?? [0, 1, 2, 3];
  const stageWeights = opts.stageWeights ?? [0.3, 0.15, 0.2, 0.35];
  const nightFraction = opts.nightFraction ?? 0.25;
  const valueWarmup = opts.valueWarmup ?? 3;
  const evalEvery = Math.max(1, opts.evalEvery ?? 4);
  const rewardScale = opts.rewardScale ?? 0.1;
  const klSchedule = opts.klCoef ?? { start: 1, end: 0.2 };
  const bcSchedule = opts.bcCoef ?? { start: 1, end: 0.25 };
  const validation = opts.validation ?? defaultValidationSet();

  const policy = policyFromCheckpoint(opts.checkpoint, seed);
  const initLogStd = opts.initLogStd === undefined ? -1.2 : opts.initLogStd;
  if (initLogStd != null) {
    const w = policy.getWeights();
    new Float32Array(w[6]!).fill(initLogStd);
    policy.setWeights(w);
  }
  const ref = policyFromCheckpoint(checkpointFromPolicy(policy), seed);
  const trainer = new TfTrainer(policy, opts.lr ?? 1e-4);
  await trainer.ready();
  const env = await createEnv();
  const startValidation = await validationCoverage(policy, validation, env);
  let bestValidation = startValidation;
  let bestWeights = policy.getWeights();
  let bestUpdate = -1;
  const metrics: FinetuneMetrics[] = [];

  const rng = new RNG(seed, 'finetune');
  let episode = 0;
  const pickStage = () => {
    let total = 0;
    for (let i = 0; i < stages.length; i++) total += stageWeights[i] ?? 1;
    let u = rng.next() * total;
    for (let i = 0; i < stages.length; i++) {
      u -= stageWeights[i] ?? 1;
      if (u <= 0) return stages[i]!;
    }
    return stages[stages.length - 1]!;
  };
  const resetEnv = () => {
    const stage = pickStage();
    const spec = trainingBuilding(stage, 20_000 + seed * 100_003 + episode * 6007, 'ppo');
    episode += 1;
    const lighting: LightingPreset = rng.next() < nightFraction ? 'night-dark' : 'day';
    return env.reset(spec, spec.seed, evalLikeConfig(stage, spec, lighting));
  };

  let obs = resetEnv();
  const obsDim = policy.obsDim;
  const actDim = policy.actDim;
  const total = valueWarmup + updates;
  for (let update = 0; update < total; update++) {
    const valueOnly = update < valueWarmup;
    const obsA = new Float32Array(horizon * obsDim);
    const actA = new Float32Array(horizon * actDim);
    const logpA = new Float32Array(horizon);
    const valuesA = new Float32Array(horizon);
    const rewardsA = new Float32Array(horizon);
    const donesA = new Uint8Array(horizon);
    const truncA = new Uint8Array(horizon);
    const finished: number[] = [];
    for (let t = 0; t < horizon; t++) {
      const out = policy.act(obs, 1);
      obsA.set(obs, t * obsDim);
      actA.set(out.action, t * actDim);
      logpA[t] = out.logp[0] ?? 0;
      valuesA[t] = out.value[0] ?? 0;
      const step = env.step(out.action);
      rewardsA[t] = step.reward * rewardScale;
      obs = step.obs;
      if (step.done) {
        // Every episode end is treated as terminal so GAE never bootstraps across episodes.
        donesA[t] = 1;
        finished.push(step.info.coverage);
        obs = resetEnv();
      }
    }
    const nextValue = policy.meanAndValue(obs, 1).value[0] ?? 0;
    const gae = computeGae({ rewards: rewardsA, values: valuesA, dones: donesA, truncated: truncA, nextValue });
    const frac = valueOnly ? 0 : (update - valueWarmup) / Math.max(1, updates - 1);
    const klCoef = scheduled(klSchedule, frac);
    const bcCoef = opts.dataset ? scheduled(bcSchedule, frac) : 0;
    const losses: PpoMetrics = await trainer.update(
      { obs: obsA, actions: actA, logp: logpA, advantages: gae.advantages, returns: gae.returns, n: horizon },
      {
        epochs: opts.epochs ?? 3,
        minibatch: opts.minibatch ?? 256,
        clipEps: opts.clipEps ?? 0.15,
        entropyCoef: opts.entropyCoef ?? 0,
        valueCoef: opts.valueCoef ?? 0.5,
        valueOnly,
        anchor: valueOnly
          ? undefined
          : { ref, klCoef, bc: opts.dataset, bcCoef, bcMinibatch: opts.bcMinibatch ?? 256 },
      },
    );
    const m: FinetuneMetrics = {
      update,
      valueOnly,
      policyLoss: losses.policyLoss,
      valueLoss: losses.valueLoss,
      entropy: losses.entropy,
      kl: losses.kl,
      anchorKl: losses.anchorKl ?? 0,
      bcLoss: losses.bcLoss ?? 0,
      klCoef: valueOnly ? 0 : klCoef,
      bcCoef: valueOnly ? 0 : bcCoef,
      rolloutCoverage: finished.length ? finished.reduce((s, v) => s + v, 0) / finished.length : env.coverage,
    };
    const ppoIndex = update - valueWarmup;
    if (!valueOnly && ((ppoIndex + 1) % evalEvery === 0 || update === total - 1)) {
      m.validation = await validationCoverage(policy, validation, env);
      if (m.validation.score > bestValidation.score) {
        bestValidation = m.validation;
        bestWeights = policy.getWeights();
        bestUpdate = update;
      }
    }
    metrics.push(m);
    opts.onUpdate?.(m);
  }
  if (bestUpdate < 0) {
    // Keep the start policy but with the trained critic, which is harmless for deterministic eval.
    bestWeights = policy.getWeights().map((buf, i) => (i <= 6 ? ref.getWeights()[i]! : buf));
  }
  policy.setWeights(bestWeights);
  env.dispose();
  return {
    checkpoint: checkpointFromPolicy(policy),
    startValidation,
    bestValidation,
    bestUpdate,
    metrics,
  };
}
