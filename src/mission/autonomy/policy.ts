import { ACT_DIM, HIDDEN, OBS_DIM } from '../../shared/constants';
import { parseCheckpoint, type Checkpoint } from '../../rl/checkpoint';
import { InferPolicy, weightShapes, type ActResult } from '../../rl/infer';
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
  const weights = checkpoint.weights.map((row) => Float32Array.from(row).buffer);
  // An actor-only file leaves out the value head, which acting never reads: zeros stand in for it.
  if (checkpoint.actorOnly) {
    for (const shape of weightShapes(policy.obsDim, policy.actDim, hidden).slice(weights.length)) {
      weights.push(new Float32Array(shape.reduce((a, b) => a * b, 1)).buffer);
    }
  }
  policy.setWeights(weights);
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

/** From a checkpoint file's contents, JSON or binary (see parseCheckpoint). */
export function policyFromFile(data: string | ArrayBuffer | Uint8Array): NormalizedPolicy {
  return policyFromCheckpoint(parseCheckpoint(data));
}
