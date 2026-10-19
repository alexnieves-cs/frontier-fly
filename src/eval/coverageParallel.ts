import type { NodeRolloutPool } from '../train/nodePool';
import type { EvalJob } from '../train/rolloutCore';
import { STAGE0_SEEDS, STAGE3_SEEDS, type CoverageReport } from './coverage';

/**
 * evalCoverage (src/eval/coverage.ts) fanned out over NodeRolloutPool workers:
 * same 14 episodes, same per-episode setup, same deterministic actions, so
 * the report is identical to the sequential one. `actor: 'policy'` uses the
 * weights last given to pool.setWeights (raw observations, like
 * scripts/eval-coverage.ts); 'teacher' runs the privileged frontier teacher.
 */
export async function evalCoverageParallel(
  pool: NodeRolloutPool,
  opts: { actor?: 'policy' | 'teacher'; normalize?: boolean } = {},
): Promise<CoverageReport> {
  const actor = opts.actor ?? 'policy';
  const normalize = opts.normalize ?? false;
  const jobs: EvalJob[] = [];
  for (const seed of STAGE0_SEEDS) {
    jobs.push({ stage: 0, seed, lighting: 'day', actor, normalize });
    jobs.push({ stage: 0, seed, lighting: 'night-dark', actor, normalize });
  }
  for (const seed of STAGE3_SEEDS) {
    jobs.push({ stage: 3, seed, lighting: 'day', actor, normalize });
    jobs.push({ stage: 3, seed, lighting: 'night-dark', actor, normalize });
  }
  const results = await pool.evaluate(jobs);
  const report: CoverageReport = { stage0Day: [], stage0Night: [], stage3Day: [], stage3Night: [] };
  jobs.forEach((job, i) => {
    const cov = results[i]!.coverage;
    if (job.stage === 0) (job.lighting === 'day' ? report.stage0Day : report.stage0Night).push(cov);
    else (job.lighting === 'day' ? report.stage3Day : report.stage3Night).push(cov);
  });
  return report;
}
