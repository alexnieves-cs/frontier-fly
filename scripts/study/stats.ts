// Shared statistics for the study scripts. Kept apart from the study entry points, which run
// their shard or merge step on import when those flags are on the command line.
import { RNG } from '../../src/shared/rng';

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
