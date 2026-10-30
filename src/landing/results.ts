// Headline benchmark numbers shared by the homepage and the guided tour's closing card.
import bench from '../../docs/benchmarks.json';

interface CoverageRow {
  stage: number;
  lighting: string;
  coverage: number;
}

const mean = (xs: number[]): number => (xs.length ? xs.reduce((s, v) => s + v, 0) / xs.length : NaN);
export const pct = (x: number): string => `${(x * 100).toFixed(1)}%`;

function dayMean(rows: readonly CoverageRow[], stage: number): number {
  return mean(rows.filter((r) => r.stage === stage && r.lighting === 'day').map((r) => r.coverage));
}

/** Headline numbers from docs/benchmarks.json, computed the way scripts/docs/generate.mjs computes them. */
export function results() {
  const { coverage, protocol } = bench;
  const hard = protocol.hard.stage;
  const easy = protocol.easy.stage;
  const of = (lighting: string) => coverage.policy.filter((r) => r.lighting === lighting).map((r) => r.coverage);
  return {
    hardStage: hard,
    hardSeeds: protocol.hard.seeds.length,
    easySeeds: protocol.easy.seeds.length,
    pilots: [
      { name: 'Learned policy (ships in the page)', value: dayMean(coverage.policy, hard), tone: 'policy' },
      { name: 'Oracle (the privileged teacher)', value: dayMean(coverage.teacher, hard), tone: 'teacher' },
      { name: 'Frontier follower (no learning)', value: dayMean(coverage.waypoint, hard), tone: 'base' },
      { name: 'Lidar heuristic', value: dayMean(coverage.heuristic, hard), tone: 'base' },
    ],
    policyLeads: dayMean(coverage.policy, hard) > dayMean(coverage.teacher, hard),
    easy: dayMean(coverage.policy, easy),
    nightOverDay: mean(of('night-dark')) / mean(of('day')),
    measured: bench.measuredAt.slice(0, 10),
    commit: bench.commit.slice(0, 7),
  };
}

export type Results = ReturnType<typeof results>;
