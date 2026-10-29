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
 *
 * Every caller resets the env right after a cut-off, so the row after a
 * truncated step (or nextValue, at the end) is the next episode's first state.
 * A truncated step therefore bootstraps from its own value, the closest stored
 * estimate of the state it was cut off in, and the trace does not carry
 * across the reset.
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
    const cut = truncated[t] === 1;
    const nextV = cut ? values[t]! : t === tSteps - 1 ? nextValue : values[t + 1]!;
    const terminal = dones[t] === 1 && !cut;
    const nonTerminal = terminal ? 0 : 1;
    const delta = rewards[t]! + gamma * nextV * nonTerminal - values[t]!;
    gae = delta + (cut ? 0 : gamma * lambda * nonTerminal * gae);
    advantages[t] = gae;
  }
  const returns = new Float32Array(tSteps);
  for (let t = 0; t < tSteps; t++) returns[t] = advantages[t]! + values[t]!;
  return { advantages, returns };
}

/**
 * GAE over a rollout buffer that interleaves several envs: within each block
 * of `block` rows, row b * block + t * envs + i belongs to env i at step t
 * (workers/rollout.ts collects that way, and TrainPool concatenates one block
 * per worker). Each env is its own time series; its last row bootstraps from
 * its own last value, like computeGae does for a single env.
 */
export function computeGaeInterleaved(
  args: Omit<GaeArgs, 'nextValue'>,
  envs: number,
  block = args.rewards.length,
): { advantages: Float32Array; returns: Float32Array } {
  const n = args.rewards.length;
  const advantages = new Float32Array(n);
  const returns = new Float32Array(n);
  const E = Math.max(1, envs | 0);
  const B = Math.max(E, block | 0);
  if (B % E !== 0 || n % B !== 0) throw new Error(`computeGaeInterleaved: ${n} rows do not split into blocks of ${B} over ${E} envs`);
  const T = B / E;
  const pick = <A extends Float32Array | Uint8Array>(src: A, out: A, base: number, i: number): A => {
    for (let t = 0; t < T; t++) out[t] = src[base + t * E + i]!;
    return out;
  };
  const rewards = new Float32Array(T);
  const values = new Float32Array(T);
  const dones = new Uint8Array(T);
  const truncated = new Uint8Array(T);
  for (let base = 0; base < n; base += B) {
    for (let i = 0; i < E; i++) {
      pick(args.values, values, base, i);
      const gae = computeGae({
        rewards: pick(args.rewards, rewards, base, i),
        values,
        dones: pick(args.dones, dones, base, i),
        truncated: pick(args.truncated, truncated, base, i),
        nextValue: values[T - 1] ?? 0,
        gamma: args.gamma,
        lambda: args.lambda,
      });
      for (let t = 0; t < T; t++) {
        advantages[base + t * E + i] = gae.advantages[t]!;
        returns[base + t * E + i] = gae.returns[t]!;
      }
    }
  }
  return { advantages, returns };
}

export async function ppoUpdate(trainer: TfTrainer, batch: PpoBatch, opts?: TrainOpts): Promise<PpoMetrics> {
  return trainer.update(batch, opts);
}
