import { ACT_DIM, OBS_DIM } from '../shared/constants';

/** Shared rollout slot. Header int32s, then f32 blocks, then two u8 flags. */
const HEADER = 8;

export interface RolloutArraysLike {
  obs: Float32Array;
  actions: Float32Array;
  logp: Float32Array;
  values: Float32Array;
  rewards: Float32Array;
  dones: Uint8Array;
  truncated: Uint8Array;
  n: number;
}

export interface RolloutViews {
  header: Int32Array;
  obs: Float32Array;
  actions: Float32Array;
  logp: Float32Array;
  values: Float32Array;
  rewards: Float32Array;
  dones: Uint8Array;
  truncated: Uint8Array;
  capacity: number;
}

export function rolloutBytes(capacity: number): number {
  const floats = capacity * (OBS_DIM + ACT_DIM + 3);
  return HEADER * 4 + floats * 4 + capacity * 2;
}

export function createRolloutSab(capacity: number): SharedArrayBuffer {
  return new SharedArrayBuffer(rolloutBytes(capacity));
}

export function rolloutViews(sab: SharedArrayBuffer, capacity: number): RolloutViews {
  if (sab.byteLength < rolloutBytes(capacity)) throw new Error('rollout buffer is short');
  const header = new Int32Array(sab, 0, HEADER);
  let offset = HEADER * 4;
  const obs = new Float32Array(sab, offset, capacity * OBS_DIM);
  offset += obs.byteLength;
  const actions = new Float32Array(sab, offset, capacity * ACT_DIM);
  offset += actions.byteLength;
  const logp = new Float32Array(sab, offset, capacity);
  offset += logp.byteLength;
  const values = new Float32Array(sab, offset, capacity);
  offset += values.byteLength;
  const rewards = new Float32Array(sab, offset, capacity);
  offset += rewards.byteLength;
  const dones = new Uint8Array(sab, offset, capacity);
  offset += dones.byteLength;
  const truncated = new Uint8Array(sab, offset, capacity);
  return { header, obs, actions, logp, values, rewards, dones, truncated, capacity };
}

export function writeRollout(view: RolloutViews, arrays: RolloutArraysLike): void {
  const n = Math.min(view.capacity, arrays.n);
  view.obs.set(arrays.obs.subarray(0, n * OBS_DIM));
  view.actions.set(arrays.actions.subarray(0, n * ACT_DIM));
  view.logp.set(arrays.logp.subarray(0, n));
  view.values.set(arrays.values.subarray(0, n));
  view.rewards.set(arrays.rewards.subarray(0, n));
  view.dones.set(arrays.dones.subarray(0, n));
  view.truncated.set(arrays.truncated.subarray(0, n));
  view.header[0] = n;
}

export function readRollout(view: RolloutViews): RolloutArraysLike {
  const n = view.header[0] ?? 0;
  return {
    obs: view.obs.slice(0, n * OBS_DIM),
    actions: view.actions.slice(0, n * ACT_DIM),
    logp: view.logp.slice(0, n),
    values: view.values.slice(0, n),
    rewards: view.rewards.slice(0, n),
    dones: view.dones.slice(0, n),
    truncated: view.truncated.slice(0, n),
    n,
  };
}

/** Concatenate per-worker slots into one learner batch. */
export function mergeRollouts(parts: RolloutArraysLike[]): RolloutArraysLike {
  const n = parts.reduce((sum, part) => sum + part.n, 0);
  const out: RolloutArraysLike = {
    obs: new Float32Array(n * OBS_DIM),
    actions: new Float32Array(n * ACT_DIM),
    logp: new Float32Array(n),
    values: new Float32Array(n),
    rewards: new Float32Array(n),
    dones: new Uint8Array(n),
    truncated: new Uint8Array(n),
    n,
  };
  let cursor = 0;
  for (const part of parts) {
    out.obs.set(part.obs.subarray(0, part.n * OBS_DIM), cursor * OBS_DIM);
    out.actions.set(part.actions.subarray(0, part.n * ACT_DIM), cursor * ACT_DIM);
    out.logp.set(part.logp.subarray(0, part.n), cursor);
    out.values.set(part.values.subarray(0, part.n), cursor);
    out.rewards.set(part.rewards.subarray(0, part.n), cursor);
    out.dones.set(part.dones.subarray(0, part.n), cursor);
    out.truncated.set(part.truncated.subarray(0, part.n), cursor);
    cursor += part.n;
  }
  return out;
}
