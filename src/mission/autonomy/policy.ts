import { ACT_DIM, HIDDEN, OBS_DIM } from '../../shared/constants';
import { parseCheckpoint, type Checkpoint } from '../../rl/checkpoint';
import { InferPolicy, type ActResult } from '../../rl/infer';
import { RunningNorm } from '../../rl/normalize';
import type { RNG } from '../../shared/rng';
import type { ActingPolicy } from './planner/indoor';

/**
 * The shipped exploration policy wrapped with its observation normalizer,
 * so the indoor explorer feeds it the same inputs it was trained on.
 */
export class NormalizedPolicy implements ActingPolicy {
  constructor(
    private readonly policy: InferPolicy,
    private readonly norm: RunningNorm,
    readonly stage: number,
  ) {}

  actWith(obs: Float32Array, n: number, rng: RNG, deterministic = false): ActResult {
    return this.policy.actWith(this.norm.apply(obs, n), n, rng, deterministic);
  }
}

export function policyFromCheckpoint(checkpoint: Checkpoint): NormalizedPolicy {
  const hidden = checkpoint.hidden.length === 2 ? checkpoint.hidden : [...HIDDEN];
  const policy = new InferPolicy(checkpoint.obsDim || OBS_DIM, checkpoint.actDim || ACT_DIM, hidden, 1);
  policy.setWeights(checkpoint.weights.map((row) => Float32Array.from(row).buffer));
  const norm = new RunningNorm(policy.obsDim);
  if (checkpoint.obsMean.length === policy.obsDim) {
    norm.load({
      dim: policy.obsDim,
      count: Math.max(1, checkpoint.obsCount),
      mean: checkpoint.obsMean,
      var: checkpoint.obsVar,
      rewCount: 1,
      rewMean: 0,
      rewVar: checkpoint.rewVar || 1,
    });
  }
  return new NormalizedPolicy(policy, norm, checkpoint.stage);
}

export function policyFromJson(text: string): NormalizedPolicy {
  return policyFromCheckpoint(parseCheckpoint(text));
}
