import { describe, expect, it } from 'vitest';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { validateMission } from '../../src/mission/validate';

describe('autonomy starter world', () => {
  it('builds a seeded world and a valid mission for it', () => {
    const world = buildMissionWorld({ seed: 7, buildings: 1, targets: [{ classId: 'person', where: 'indoor' }] });
    expect(world.buildings.length).toBe(1);
    const mission = missionForWorld(world, { missionType: 'find_person' });
    const v = validateMission(mission);
    expect(v.issues.filter((i) => i.severity === 'error').map((i) => i.message)).toEqual([]);
  });
});
