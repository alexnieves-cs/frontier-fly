// Dev-seed check of the quad mission vehicle: each scenario on tuning seeds 9101+, both airframes.
//   vite-node scripts/study/quad-mission-diag.ts -- [--seeds 4] [--indoor frontier|auto]
import { readFileSync } from 'node:fs';
import { policyFromFile } from '../../src/mission/autonomy/policy';
import { buildReport } from '../../src/mission/autonomy/report';
import { runMission } from '../../src/mission/autonomy/runner';
import { SCENARIOS } from '../../src/mission/autonomy/scenarios';
import { createPerceptionSource } from '../../src/mission/autonomy/perception';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}
const seeds = Number(arg('seeds', '4'));
const indoor = arg('indoor', 'frontier') as 'frontier' | 'auto';
const vehicles = arg('vehicles', 'kinematic,quad').split(',') as ('kinematic' | 'quad')[];
const policy = indoor === 'auto' ? policyFromFile(readFileSync(SHIPPED_CHECKPOINT_PATH)) : undefined;
for (const scenario of SCENARIOS) {
  for (const vehicle of vehicles) {
    let ok = 0;
    let cov = 0;
    let col = 0;
    let dur = 0;
    const outs: string[] = [];
    for (let i = 0; i < seeds; i++) {
      const world = buildMissionWorld({ ...scenario.world, seed: 9101 + i });
      const mission = missionForWorld(world, scenario.mission);
      const source = createPerceptionSource(world);
      const run = await runMission(mission, world, source, { ...scenario.runner, operator: () => 'approve', indoor, policy, vehicle });
      const rep = buildReport(run, source, world);
      ok += run.outcome === scenario.expect.outcome ? 1 : 0;
      cov += rep.indoorCoverage;
      col += rep.collisions;
      dur += run.tSec;
      outs.push(run.outcome);
    }
    console.log(
      `${scenario.id.padEnd(14)} ${vehicle.padEnd(9)} expected ${ok}/${seeds} indoorCov ${(cov / seeds).toFixed(3)} collisions ${(col / seeds).toFixed(2)} dur ${Math.round(dur / seeds)} s  ${outs.join(',')}`,
    );
  }
}
