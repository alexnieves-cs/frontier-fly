// Helpers shared by the study scripts. Kept apart from the scripts themselves, which run a
// shard or a merge as soon as they are imported with those flags on the command line.
import { readFileSync } from 'node:fs';
import { parseCheckpoint } from '../../src/rl/checkpoint';
import { InferPolicy } from '../../src/rl/infer';
import { RNG } from '../../src/shared/rng';

export function loadPolicy(path: string): (obs: Float32Array) => Float32Array {
  const ck = parseCheckpoint(readFileSync(path));
  const policy = new InferPolicy(ck.obsDim, ck.actDim, ck.hidden, 1);
  policy.setWeights(ck.weights.map((row) => Float32Array.from(row).buffer));
  const rng = new RNG(1);
  return (obs) => policy.actWith(obs, 1, rng, true).action.slice();
}

/** Bootstrap 95% interval of the mean, deterministic. */
export function bootstrapCi(xs: number[], resamples = 2000): [number, number] {
  if (!xs.length) return [0, 0];
  const rng = new RNG(7, 'bootstrap');
  const means: number[] = [];
  for (let b = 0; b < resamples; b++) {
    let sum = 0;
    for (let i = 0; i < xs.length; i++) sum += xs[Math.floor(rng.next() * xs.length)]!;
    means.push(sum / xs.length);
  }
  means.sort((a, b) => a - b);
  return [means[Math.floor(resamples * 0.025)]!, means[Math.floor(resamples * 0.975)]!];
}
