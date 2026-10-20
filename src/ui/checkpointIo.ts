import { ACT_DIM, HIDDEN, OBS_DIM, OBS_VERSION } from '../shared/constants';
import { serializeCheckpoint, type Checkpoint } from '../rl/checkpoint';
import { InferPolicy } from '../rl/infer';
import { RunningNorm } from '../rl/normalize';

/** Checkpoint <-> flyable policy for the sim page, and the file download. */

export interface LoadedPolicy {
  policy: InferPolicy;
  norm: RunningNorm;
  /** True when the checkpoint was trained on raw observations (identity stats; no ±5 clip). */
  rawObs: boolean;
}

/** Build the inference policy and observation norm a checkpoint describes. Check checkpointMismatch first. */
export function policyFromCheckpoint(checkpoint: Checkpoint): LoadedPolicy {
  const policy = new InferPolicy(checkpoint.obsDim || OBS_DIM, checkpoint.actDim || ACT_DIM, checkpoint.hidden.length === 2 ? checkpoint.hidden : HIDDEN, 1);
  const buffers = checkpoint.weights.map((row) => {
    const arr = Float32Array.from(row);
    return arr.buffer;
  });
  policy.setWeights(buffers);
  const identity =
    checkpoint.obsMean.every((v) => v === 0) && checkpoint.obsVar.every((v) => v === 1);
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
  return { policy, norm, rawObs: identity };
}

export function checkpointFrom(policy: InferPolicy, norm: RunningNorm, stage: number): Checkpoint {
  const state = norm.toJSON();
  return {
    version: 1,
    obsVersion: OBS_VERSION,
    obsDim: policy.obsDim,
    actDim: policy.actDim,
    hidden: policy.hidden,
    weights: policy.getWeights().map((buf) => Array.from(new Float32Array(buf))),
    obsMean: state.mean,
    obsVar: state.var,
    obsCount: state.count,
    rewVar: state.rewVar,
    stage,
  };
}

export function downloadCheckpoint(checkpoint: Checkpoint): void {
  const blob = new Blob([serializeCheckpoint(checkpoint)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'frontier-fly.json';
  a.click();
  URL.revokeObjectURL(url);
}
