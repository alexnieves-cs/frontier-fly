/// <reference lib="webworker" />
import * as tf from '@tensorflow/tfjs';
import { OBS_DIM, ACT_DIM } from '../shared/constants';
import type { RolloutArrays } from '../rl/buffer';
import { InferPolicy } from '../rl/infer';
import { TfTrainer } from '../rl/model';
import type { RunningNormState } from '../rl/normalize';
import { computeGae } from '../rl/ppo';

interface RolloutMsg {
  type: 'rollout';
  arrays: RolloutArrays;
  norm: RunningNormState;
}
interface BootMsg {
  type: 'boot';
  weights: ArrayBuffer[];
}

let trainer: TfTrainer | null = null;
let policy: InferPolicy | null = null;
let updates = 0;

async function pickBackend(): Promise<string> {
  try {
    if (await tf.setBackend('webgl')) {
      await tf.ready();
      return tf.getBackend();
    }
  } catch {
    /* Node and some headless sessions have no WebGL. */
  }
  await tf.setBackend('cpu');
  await tf.ready();
  return tf.getBackend();
}

async function boot(weights: ArrayBuffer[]): Promise<void> {
  const backend = await pickBackend();
  policy = new InferPolicy(OBS_DIM, ACT_DIM, [256, 128], 1);
  policy.setWeights(weights);
  trainer = new TfTrainer(policy, 3e-4, false);
  await trainer.ready();
  self.postMessage({ type: 'learner-ready', backend });
}

self.onmessage = (event: MessageEvent<RolloutMsg | BootMsg>) => {
  const msg = event.data;
  if (msg.type === 'boot') {
    void boot(msg.weights);
    return;
  }
  if (!trainer || !policy) return;
  const { arrays } = msg;
  const n = arrays.n;
  const last = n > 0 ? arrays.values[n - 1]! : 0;
  const gae = computeGae({
    rewards: arrays.rewards,
    values: arrays.values,
    dones: arrays.dones,
    truncated: arrays.truncated,
    nextValue: last,
  });
  void trainer
    .update({
      obs: arrays.obs,
      actions: arrays.actions,
      logp: arrays.logp,
      advantages: gae.advantages,
      returns: gae.returns,
      n,
    })
    .then((losses) => {
      trainer?.pullWeights();
      updates += 1;
      const weights = policy!.getWeights();
      self.postMessage({ type: 'updated', weights, losses, update: updates, norm: msg.norm }, { transfer: weights });
    });
};
