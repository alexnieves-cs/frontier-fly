import { describe, expect, it } from 'vitest';
import type { MissionForWorldOptions, MissionWorld, MissionWorldOptions, RunRecord } from '../../src/mission/autonomy/contracts';
import { createMissionRunner, runMission } from '../../src/mission/autonomy/runner';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { InferPolicy } from '../../src/rl/infer';
import { ACT_DIM, OBS_DIM } from '../../src/shared/constants';
import { createFakeSource, type FakeSourceOptions } from '../helpers/fakeSource';
import { checkTiming } from './timing';

/*
 * Full missions with the test detection source. Worlds: seed 20 with one
 * building (person inside), seed 37 with two buildings and no targets.
 */

const PERSON_WORLD: MissionWorldOptions = {
  seed: 20,
  buildings: 1,
  timeOfDay: 13,
  interiorLights: 'on',
  targets: [{ classId: 'person', state: 'lying', where: 'indoor', building: 0 }],
  decoys: 1,
};

const FIND_PERSON: MissionForWorldOptions = {
  missionType: 'find_person',
  targets: [{ id: 'tgt-person', kind: 'person', label: 'Person', minCount: 1, minConfidence: null }],
  requiredConfidence: 0.8,
  entryPolicy: 'auto',
  budgets: { batteryPct: 80, timeSec: 900 },
};

async function run(worldOptions: MissionWorldOptions, missionOptions: MissionForWorldOptions, fake: FakeSourceOptions = {}, extra = {}): Promise<{ run: RunRecord; world: MissionWorld }> {
  const world = buildMissionWorld(worldOptions);
  const mission = missionForWorld(world, missionOptions);
  const source = await createFakeSource(world, fake);
  return { run: await runMission(mission, world, source, { dt: 0.05, ...extra }), world };
}

describe('mission runner', () => {
  it('finds the person, lands home, and repeats exactly', async () => {
    const t0 = performance.now();
    const first = await run(PERSON_WORLD, FIND_PERSON);
    const wall = performance.now() - t0;
    const r = first.run;
    expect(r.outcome).toBe('found');
    expect(r.landedHome).toBe(true);
    expect(r.collisions).toHaveLength(0);
    expect(r.confirmed.some((c) => c.classId === 'person' && c.truth.kind === 'tp')).toBe(true);
    expect(r.entries.some((e) => e.decision === 'auto')).toBe(true);
    expect(r.indoorExplorer).toBe('frontier');
    const seen = new Set(r.phases.map((p) => p.phase));
    for (const phase of ['coverage', 'approach', 'inspect', 'fly_through', 'indoor_explore', 'confirm', 'exit', 'return_home', 'land'] as const) {
      expect(seen.has(phase), phase).toBe(true);
    }
    expect(r.batteryEndPct).toBeLessThan(r.batteryStartPct);
    if (checkTiming) expect(wall).toBeLessThan(20_000);

    const second = await run(PERSON_WORLD, FIND_PERSON);
    expect(JSON.stringify(second.run)).toBe(JSON.stringify(r));
  });

  it('rejects a single high-confidence ghost instead of reporting it', async () => {
    const world = buildMissionWorld(PERSON_WORLD);
    const home = world.home;
    const ghost = { at: [home[0] + 12, home[1] + 12, home[2] + 0.5] as [number, number, number], classId: 'person' as const, confidence: 0.97, frames: 1 };
    const { run: r } = await run(PERSON_WORLD, FIND_PERSON, { ghosts: [ghost] });
    expect(r.outcome).toBe('found');
    expect(r.confirmed.every((c) => c.truth.kind === 'tp')).toBe(true);
  });

  it('waits for the operator on ask entries and sweeps two buildings clear', async () => {
    const world = buildMissionWorld({ seed: 37, buildings: 2, timeOfDay: 10.5, interiorLights: 'on', targets: [], decoys: 2 });
    const mission = missionForWorld(world, {
      missionType: 'safety_sweep',
      targets: [{ id: 'tgt-clear', kind: 'clearance', label: 'Clear', minCount: 1, minConfidence: null }],
      requiredConfidence: 0.8,
      entryPolicy: 'ask',
      budgets: { batteryPct: 85, timeSec: 1200 },
    });
    const source = await createFakeSource(world);
    const runner = await createMissionRunner(mission, world, source, { dt: 0.05 });
    let asks = 0;
    let waited = 0;
    for (let i = 0; i < 40_000 && !runner.done; i++) {
      if (runner.snapshot().pendingEntry) {
        // Pending requests hold the drone until answered.
        waited += 1;
        if (waited < 20) {
          runner.step();
          continue;
        }
        asks += 1;
        waited = 0;
        runner.approve();
      }
      runner.step();
    }
    const r = runner.record();
    expect(asks).toBeGreaterThanOrEqual(2);
    expect(r.entries.filter((e) => e.decision === 'approved').length).toBeGreaterThanOrEqual(2);
    expect(r.outcome).toBe('clear');
    for (const b of world.buildings) expect(r.declaredClear.some((d) => d.scope === b.id)).toBe(true);
    expect(r.landedHome).toBe(true);
    expect(r.collisions).toHaveLength(0);
  });

  it('falls back to frontier exploration when the policy explorer fails', async () => {
    const policy = new InferPolicy(OBS_DIM, ACT_DIM, [16, 16], 3);
    const { run: r } = await run(PERSON_WORLD, FIND_PERSON, {}, { indoor: 'policy', policy });
    expect(r.landedHome).toBe(true);
    expect(r.outcome).toBe('found');
    expect(r.indoorExplorer === 'policy' || r.indoorExplorer === 'frontier').toBe(true);
    if (r.indoorExplorer === 'frontier') expect(r.log.some((l) => l.message.includes('Falling back')), r.log.filter((l) => /olicy/.test(l.message)).map((l) => l.message).join(' | ')).toBe(true);
  }, 60_000);
});
