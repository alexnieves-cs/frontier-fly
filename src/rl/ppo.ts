import { GAE_LAMBDA, GAMMA } from '../shared/constants';
import type { PpoBatch } from './buffer';
import { TfTrainer, type PpoMetrics, type TrainOpts } from './model';

export interface GaeArgs {
  rewards: Float32Array;
  values: Float32Array;
  dones: Uint8Array;
  truncated: Uint8Array;
  nextValue: number;
  gamma?: number;
  lambda?: number;
}

/**
 * GAE(λ). Callers set done=1 only for true terminals. Truncated episodes keep
 * done=0 so the value bootstraps. If truncated[t] is 1, the step still
 * bootstraps when a caller also set done. For a length-T rollout, V_{T} is
 * nextValue and V_{t+1} for t < T-1 is values[t+1].
 *
 * delta_t = r_t + gamma * V_{t+1} * (1 - terminal_t) - V_t
 */
export function computeGae(args: GaeArgs): { advantages: Float32Array; returns: Float32Array } {
  const gamma = args.gamma ?? GAMMA;
  const lambda = args.lambda ?? GAE_LAMBDA;
  const { rewards, values, dones, truncated, nextValue } = args;
  const tSteps = rewards.length;
  if (values.length !== tSteps || dones.length !== tSteps || truncated.length !== tSteps) {
    throw new Error('computeGae: array length mismatch');
  }
  const advantages = new Float32Array(tSteps);
  let gae = 0;
  for (let t = tSteps - 1; t >= 0; t--) {
    const nextV = t === tSteps - 1 ? nextValue : values[t + 1]!;
    const terminal = dones[t] === 1 && truncated[t] !== 1;
    const nonTerminal = terminal ? 0 : 1;
    const delta = rewards[t]! + gamma * nextV * nonTerminal - values[t]!;
    gae = delta + gamma * lambda * nonTerminal * gae;
    advantages[t] = gae;
  }
  const returns = new Float32Array(tSteps);
  for (let t = 0; t < tSteps; t++) returns[t] = advantages[t]! + values[t]!;
  return { advantages, returns };
}

export async function ppoUpdate(trainer: TfTrainer, batch: PpoBatch, opts?: TrainOpts): Promise<PpoMetrics> {
  return trainer.update(batch, opts);
}
