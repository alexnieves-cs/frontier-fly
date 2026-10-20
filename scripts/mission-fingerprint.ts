/**
 * Fingerprints whole mission runs, for refactors that must not change behaviour:
 * flies each scenario with each indoor explorer on a few dev seeds and prints a
 * hash of every full RunRecord (track, log, contacts, outcome).
 *
 *   npx vite-node scripts/mission-fingerprint.ts -- [--seeds 9101,9102] [--out file.json]
 *
 * Run it before and after the change and diff the output. Dev seeds only
 * (9101-9116); the bench seeds (9600+) stay untouched.
 */
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import { policyFromFile } from '../src/mission/autonomy/policy';
import { SHIPPED_CHECKPOINT_PATH } from '../src/rl/checkpoint';
import { createPerceptionSource } from '../src/mission/autonomy/perception';
import { runMission } from '../src/mission/autonomy/runner';
import { SCENARIOS } from '../src/mission/autonomy/scenarios';
import { buildMissionWorld, missionForWorld } from '../src/mission/autonomy/world';
import { MISSION_MODES, missionModeOptions } from '../src/eval/missions';

const args = process.argv.slice(2);
const opt = (name: string) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const seeds = (opt('seeds') ?? '9101,9102').split(',').map(Number);
const policy = policyFromFile(readFileSync(SHIPPED_CHECKPOINT_PATH));
const rows: Record<string, string> = {};
for (const scenario of SCENARIOS) {
  for (const seed of seeds) {
    for (const mode of MISSION_MODES) {
      const world = buildMissionWorld({ ...scenario.world, seed });
      const mission = missionForWorld(world, scenario.mission);
      const source = createPerceptionSource(world);
      const run = await runMission(mission, world, source, {
        ...scenario.runner,
        operator: () => 'approve' as const,
        ...missionModeOptions(mode, policy),
      });
      const hash = createHash('sha256').update(JSON.stringify(run)).digest('hex').slice(0, 16);
      const key = `${scenario.id}/${seed}/${mode}`;
      rows[key] = `${run.outcome} ${run.tSec.toFixed(2)}s ${hash}`;
      console.log(`${key}: ${rows[key]}`);
    }
  }
}
const out = opt('out');
if (out) writeFileSync(out, `${JSON.stringify(rows, null, 2)}\n`);
