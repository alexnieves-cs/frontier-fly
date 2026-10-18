/**
 * 1D point mass. State is pos, vel. Target is 1.
 * obs = [pos, vel, target - pos, 1], action in [-1, 1].
 * pos += vel * dt; vel = (vel + action * 2 * dt) * 0.98; dt = 0.05.
 * reward = -|target - pos|. Done when |target - pos| < 0.05 or steps >= 60.
 * No truncation.
 */
import { RNG } from '../shared/rng';
import { RolloutBuffer } from './buffer';
import { InferPolicy } from './infer';
import { TfTrainer } from './model';
import { computeGae, ppoUpdate } from './ppo';

const DT = 0.05;
const TARGET = 1;
const MAX_STEPS = 60;
const SUCCESS_DIST = 0.05;
const ROLLOUT = 512;
const EVAL_EPISODES = 16;
const SOLVED_DIST = 0.12;

export interface ToyResult {
  solved: boolean;
  updates: number;
  finalDist: number;
}

function stepPoint(pos: number, vel: number, action: number): { pos: number; vel: number } {
  const nextPos = pos + vel * DT;
  const nextVel = (vel + action * 2 * DT) * 0.98;
  return { pos: nextPos, vel: nextVel };
}

function writeObs(raw: Float32Array, pos: number, vel: number): void {
  raw[0] = pos;
  raw[1] = vel;
  raw[2] = TARGET - pos;
  raw[3] = 1;
}

/** Deterministic rollouts. Every episode starts at rest, so the average is one trajectory repeated. */
function evaluateToy(policy: InferPolicy): number {
  let total = 0;
  const raw = new Float32Array(4);
  const rng = new RNG(0);
  for (let ep = 0; ep < EVAL_EPISODES; ep++) {
    let pos = 0;
    let vel = 0;
    let steps = 0;
    let dist = Math.abs(TARGET - pos);
    while (steps < MAX_STEPS && dist >= SUCCESS_DIST) {
      writeObs(raw, pos, vel);
      const action = policy.actWith(raw, 1, rng, true).action[0]!;
      const next = stepPoint(pos, vel, action);
      pos = next.pos;
      vel = next.vel;
      steps += 1;
      dist = Math.abs(TARGET - pos);
    }
    total += dist;
  }
  return total / EVAL_EPISODES;
}

export async function trainToy(opts?: { updates?: number; seed?: number; lr?: number }): Promise<ToyResult> {
  const updatesCap = opts?.updates ?? 200;
  const seed = opts?.seed ?? 1;
  const lr = opts?.lr ?? 1e-3;
  const policy = new InferPolicy(4, 1, [32, 32], seed);
  const trainer = new TfTrainer(policy, lr);
  await trainer.ready();
  const buf = new RolloutBuffer(ROLLOUT, 4, 1);
  const raw = new Float32Array(4);
  let finalDist = Number.POSITIVE_INFINITY;
  let used = 0;

  for (let update = 0; update < updatesCap; update++) {
    buf.reset();
    let pos = 0;
    let vel = 0;
    let steps = 0;
    for (let t = 0; t < ROLLOUT; t++) {
      writeObs(raw, pos, vel);
      const out = policy.act(raw, 1);
      const action = out.action[0]!;
      const next = stepPoint(pos, vel, action);
      pos = next.pos;
      vel = next.vel;
      steps += 1;
      const dist = Math.abs(TARGET - pos);
      const done = dist < SUCCESS_DIST || steps >= MAX_STEPS;
      buf.add(raw, out.action, out.logp[0]!, out.value[0]!, -dist, done, false);
      if (done) {
        pos = 0;
        vel = 0;
        steps = 0;
      }
    }
    const rollout = buf.toArrays();
    let nextValue = 0;
    if (rollout.dones[rollout.n - 1] !== 1) {
      writeObs(raw, pos, vel);
      nextValue = policy.actWith(raw, 1, new RNG(0), true).value[0]!;
    }
    const { advantages, returns } = computeGae({
      rewards: rollout.rewards,
      values: rollout.values,
      dones: rollout.dones,
      truncated: rollout.truncated,
      nextValue,
    });
    const metrics = await ppoUpdate(trainer, {
      obs: rollout.obs,
      actions: rollout.actions,
      logp: rollout.logp,
      advantages,
      returns,
      n: rollout.n,
    });
    used = update + 1;
    finalDist = evaluateToy(policy);
    console.log(
      `toy update ${used} finalDist ${finalDist.toFixed(4)} policy ${metrics.policyLoss.toFixed(4)} value ${metrics.valueLoss.toFixed(3)} ent ${metrics.entropy.toFixed(3)} kl ${metrics.kl.toFixed(4)}`,
    );
    if (!Number.isFinite(finalDist) || !Number.isFinite(metrics.policyLoss)) {
      throw new Error(`non-finite toy training at update ${used}`);
    }
    if (finalDist < SOLVED_DIST) return { solved: true, updates: used, finalDist };
  }
  return { solved: finalDist < SOLVED_DIST, updates: used, finalDist };
}
