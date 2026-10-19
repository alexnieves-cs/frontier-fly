import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { policyFromJson } from '../../src/mission/autonomy/policy';
import { buildReport } from '../../src/mission/autonomy/report';
import { runMission } from '../../src/mission/autonomy/runner';
import { loadScenario, SCENARIOS } from '../../src/mission/autonomy/scenarios';
import { OBS_DIM } from '../../src/shared/constants';

const shipped = policyFromJson(readFileSync('public/checkpoints/pretrained.json', 'utf8'));

describe('shipped policy indoors', () => {
  it('normalizes observations before acting', () => {
    const obs = new Float32Array(OBS_DIM).fill(0.5);
    const a = shipped.actWith(obs, 1, { next: () => 0.5, gaussian: () => 0 } as never, true).action;
    expect(a.length).toBe(4);
    expect([...a].every((v) => Number.isFinite(v) && Math.abs(v) <= 1)).toBe(true);
  });

  for (const scenario of SCENARIOS) {
    it(`${scenario.id} still meets its expected outcome with the policy flying first`, async () => {
      const { world, mission, source, options } = loadScenario(scenario.id);
      const run = await runMission(mission, world, source, { ...options, indoor: 'auto', policy: shipped });
      const report = buildReport(run, source, world);
      expect(run.outcome).toBe(scenario.expect.outcome);
      expect(run.indoorExplorer).not.toBeNull();
      expect(report.falseClears).toBe(0);
      expect(report.falseAlarms).toBe(0);
      expect(report.collisions).toBeLessThanOrEqual(1);
      expect(run.landedHome).toBe(true);
    });
  }
});
