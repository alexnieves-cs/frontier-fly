import { describe, expect, it } from 'vitest';
import { buildReport } from '../../src/mission/autonomy/report';
import { runMission } from '../../src/mission/autonomy/runner';
import { loadScenario, SCENARIOS, scenarioById, type ScenarioId } from '../../src/mission/autonomy/scenarios';
import { canonicalMissionJson } from '../../src/mission/serialize';
import { validateMission } from '../../src/mission/validate';
import { fingerprint } from '../../src/world/outdoor/fingerprint';
import { validateOutdoor } from '../../src/world/outdoor/validate';

function sceneDigest(id: ScenarioId): string {
  const { world, mission } = loadScenario(id);
  return JSON.stringify({
    outdoor: fingerprint(world.outdoor),
    buildings: world.buildings.map((b) => ({ id: b.id, origin: b.shell.origin, targets: b.scene.targets, lights: Array.from(b.interiorLightsOn) })),
    outdoorTargets: world.outdoorScene.targets,
    home: world.home,
    mission: canonicalMissionJson(mission),
  });
}

describe('autonomy scenarios', () => {
  it('ships exactly the three demo scenarios', () => {
    expect(SCENARIOS.map((s) => s.id)).toEqual(['cabin-day', 'compound-clear', 'backpack-night']);
    expect(() => scenarioById('nope')).toThrow();
  });

  for (const scenario of SCENARIOS) {
    describe(scenario.id, () => {
      it('builds a valid world on the first generator attempt with a good entry per building', () => {
        const { world } = loadScenario(scenario.id);
        // generateOutdoor retries seed + 1 on a bad world; the chosen seed must not need that.
        expect(world.outdoor.seed).toBe(scenario.world.seed);
        expect(validateOutdoor(world.outdoor)).toEqual([]);
        expect(world.buildings.length).toBe(scenario.world.buildings);
        for (const building of world.buildings) {
          const entries = building.shell.openings.filter((o) => o.passable && o.kind !== 'roof_hatch' && o.floor === 0);
          expect(entries.length, building.id).toBeGreaterThan(0);
          expect(Math.min(...entries.map((o) => Math.min(o.clearWidth, o.clearHeight)))).toBeGreaterThan(0.6);
        }
        expect(world.timeOfDay).toBe(scenario.world.timeOfDay);
        expect(world.flashlightOn).toBe(scenario.world.flashlightOn);
      });

      it('builds a mission that validates against the world', () => {
        const { mission, world } = loadScenario(scenario.id);
        const v = validateMission(mission);
        expect(v.issues.filter((i) => i.severity === 'error').map((i) => i.message)).toEqual([]);
        expect(mission.seed).toBe(world.seed);
        expect(mission.missionType).toBe(scenario.mission.missionType);
        expect(mission.entryPolicy).toBe(scenario.mission.entryPolicy);
      });

      it('is deterministic', () => {
        expect(sceneDigest(scenario.id)).toBe(sceneDigest(scenario.id));
      });
    });
  }

  it('night scenario has every interior light off and the flashlight on', () => {
    const { world } = loadScenario('backpack-night');
    expect(world.timeOfDay).toBeGreaterThanOrEqual(22);
    expect(world.flashlightOn).toBe(true);
    for (const building of world.buildings) expect(Array.from(building.interiorLightsOn).every((on) => on === 0)).toBe(true);
  });

  it('compound sweep plants no person, so the truth is clear', () => {
    const { world } = loadScenario('compound-clear');
    const people = world.buildings.flatMap((b) => b.scene.targets).filter((t) => t.category === 'person');
    expect(people).toEqual([]);
    expect(world.outdoorScene.targets.filter((t) => t.category === 'person')).toEqual([]);
  });
});

// Full headless runs. These need the real runner, world, and perception; the
// starter stubs see nothing, so they fail until those workstreams land.
describe('scenario end to end', () => {
  it('cabin-day finds the injured person and lands home', async () => {
    const { world, mission, source, options } = loadScenario('cabin-day');
    const run = await runMission(mission, world, source, options);
    const report = buildReport(run, source, world);
    expect(run.outcome).toBe('found');
    expect(report.outcome).toBe('found');
    expect(report.timeToFindSec).not.toBeNull();
    const person = run.confirmed.find((c) => c.classId === 'person');
    expect(person).toBeDefined();
    expect(person!.truth.kind).toBe('tp');
    expect(person!.truth.trueClass).toBe('person');
    expect(report.falseAlarms).toBe(0);
    expect(run.landedHome).toBe(true);
    expect(report.landedHome).toBe(true);
  });

  it('compound-clear declares both buildings clear after an approved entry', async () => {
    const { world, mission, source, options } = loadScenario('compound-clear');
    const run = await runMission(mission, world, source, options);
    const report = buildReport(run, source, world);
    expect(run.outcome).toBe('clear');
    for (const building of world.buildings) {
      expect(run.declaredClear.some((d) => d.scope === building.id), building.id).toBe(true);
    }
    expect(report.falseClears).toBe(0);
    expect(run.entries.some((e) => e.decision === 'approved')).toBe(true);
    expect(run.landedHome).toBe(true);
  });

  it('backpack-night finds the backpack in the dark and lands home', async () => {
    const { world, mission, source, options } = loadScenario('backpack-night');
    const run = await runMission(mission, world, source, options);
    const report = buildReport(run, source, world);
    expect(run.outcome).toBe('found');
    const bag = run.confirmed.find((c) => c.classId === 'backpack');
    expect(bag).toBeDefined();
    expect(bag!.truth.trueClass).toBe('backpack');
    expect(report.timeToFindSec).not.toBeNull();
    expect(report.falseAlarms).toBe(0);
    expect(run.landedHome).toBe(true);
  });
});
