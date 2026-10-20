// Flies each mission scenario on fresh world seeds with each indoor explorer (src/eval/missions.ts)
// and writes docs/mission-bench.json. Run with `npm run bench`, or faster in parallel with
// `npm run study:missions`; `npm run docs` renders it into docs/BENCHMARKS.md.
// Nothing here is a pass/fail gate. It records what the explorers did.
import { execSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { flyMission, MISSION_SEED_BASE, missionBenchReport, missionJobs, type MissionFlight } from '../src/eval/missions';

const OUT = process.env.BENCH_MISSIONS_OUT ?? 'docs/mission-bench.json';
const SEEDS_PER_SCENARIO = Number(process.env.BENCH_MISSION_SEEDS ?? 10);

describe('mission bench', () => {
  it('flies every scenario on fresh seeds with each indoor explorer', async () => {
    const flights: MissionFlight[] = [];
    for (const job of missionJobs(SEEDS_PER_SCENARIO)) {
      const f = await flyMission(job);
      flights.push(f);
      console.log(
        `${f.scenario} seed ${f.seed} ${f.indoor}: ${f.outcome} in ${f.durationSec} s, explorer ${f.explorer}, handoffs ${f.handoffs.join(',') || 'none'}`,
      );
    }
    let commit = 'unknown';
    try {
      commit = execSync('git rev-parse --short HEAD').toString().trim();
    } catch {
      // not a git checkout
    }
    writeFileSync(OUT, `${JSON.stringify(missionBenchReport(flights, commit, SEEDS_PER_SCENARIO, MISSION_SEED_BASE), null, 2)}\n`);
    expect(flights.length).toBe(missionJobs(SEEDS_PER_SCENARIO).length);
  });
});
