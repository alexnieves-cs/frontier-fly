import { describe, expect, it } from 'vitest';
import { buildMissionWorld, missionForWorld, missionNoFly, outdoorWorldProvider } from '../../src/mission/autonomy/world';
import type { MissionWorld, TargetWant } from '../../src/mission/autonomy/contracts';
import { pointInRing, ringInsideRing, signedArea } from '../../src/mission/polygon';
import { validateMission } from '../../src/mission/validate';
import { woodsSampleMission } from '../../src/mission/outdoorWorld';
import { MISSION_TYPES } from '../../src/mission/types';
import { placementIssues } from '../../src/targets/validate';

function allTargets(world: MissionWorld): { id: string; classId: string; state: string; building: number | null }[] {
  const indoor = world.buildings.flatMap((building, index) =>
    building.scene.targets.map((target) => ({ id: target.id, classId: target.classId, state: target.state, building: index })),
  );
  const outdoor = world.outdoorScene.targets.map((target) => ({ id: target.id, classId: target.classId, state: target.state, building: null }));
  return [...indoor, ...outdoor];
}

function placementOk(world: MissionWorld): string[] {
  const issues = world.buildings.flatMap((building) => placementIssues(building.scene, building.shell.spec));
  return [...issues, ...placementIssues(world.outdoorScene)];
}

describe('mission world', () => {
  it('is deterministic for a seed', () => {
    const options = { seed: 23, targets: [{ classId: 'person', state: 'lying', where: 'indoor' }, { classId: 'radio', where: 'outdoor' }] as TargetWant[], decoys: 2 };
    const a = buildMissionWorld(options);
    const b = buildMissionWorld(options);
    expect(a.buildings.map((item) => [item.id, item.label, item.footprint, item.heightM])).toEqual(b.buildings.map((item) => [item.id, item.label, item.footprint, item.heightM]));
    expect(JSON.stringify(a.buildings.map((item) => item.scene))).toBe(JSON.stringify(b.buildings.map((item) => item.scene)));
    expect(JSON.stringify(a.outdoorScene)).toBe(JSON.stringify(b.outdoorScene));
    expect(a.home).toEqual(b.home);
    const other = buildMissionWorld({ ...options, seed: 24 });
    expect(JSON.stringify(other.buildings.map((item) => item.scene))).not.toBe(JSON.stringify(a.buildings.map((item) => item.scene)));
  });

  it('plants exactly the requested targets with valid placement', () => {
    for (const seed of [3, 9, 17, 31, 44]) {
      const wants: TargetWant[] = [
        { classId: 'person', state: 'lying', where: 'indoor', building: 1 },
        { classId: 'person', state: 'standing', where: 'indoor' },
        { classId: 'medkit', where: 'indoor' },
        { classId: 'person', state: 'sitting', where: 'outdoor' },
        { classId: 'backpack', where: 'outdoor' },
      ];
      const world = buildMissionWorld({ seed, buildings: 2, targets: wants });
      expect(world.buildings).toHaveLength(2);
      const targets = allTargets(world);
      const pick = (building: number | null) =>
        targets.filter((target) => target.building === building).map((target) => `${target.classId}:${target.state}`).sort();
      expect(pick(0)).toEqual(['medkit:static', 'person:standing']);
      expect(pick(1)).toEqual(['person:lying']);
      expect(pick(null)).toEqual(['backpack:static', 'person:sitting']);
      expect(placementOk(world)).toEqual([]);
      expect(new Set(targets.map((target) => target.id)).size).toBe(targets.length);
      for (const target of world.outdoorScene.targets) {
        const ring = world.outdoor.geofence.polygon.map(([x, y]) => ({ x, y }));
        expect(pointInRing({ x: target.feet[0], y: target.feet[1] }, ring)).toBe(true);
        for (const building of world.buildings) expect(pointInRing({ x: target.feet[0], y: target.feet[1] }, building.footprint)).toBe(false);
      }
    }
  });

  it('grows the building count when a target names a later building', () => {
    const world = buildMissionWorld({ seed: 42, targets: [{ classId: 'person', state: 'hiding', where: 'indoor', building: 2 }] });
    expect(world.buildings.length).toBeGreaterThanOrEqual(3);
    expect(world.buildings[2]!.scene.targets.map((target) => target.state)).toEqual(['hiding']);
    expect(() => buildMissionWorld({ seed: 42, buildings: 1, targets: [{ classId: 'person', where: 'indoor', building: 1 }] })).toThrow(RangeError);
  });

  it('keeps a clear world clear, and decoys are look-alikes only', () => {
    for (const seed of [1, 2, 3]) {
      const clear = buildMissionWorld({ seed, targets: [] });
      expect(allTargets(clear)).toEqual([]);
      const decoyed = buildMissionWorld({ seed, targets: [], decoys: 3 });
      const decoys = decoyed.buildings.flatMap((building) => building.scene.targets);
      expect(decoys).toHaveLength(3);
      expect(decoys.every((target) => target.category === 'decoy')).toBe(true);
      expect(decoyed.outdoorScene.targets).toEqual([]);
      expect(placementOk(decoyed)).toEqual([]);
    }
  });

  it('seeds a default person and object when targets are omitted', () => {
    const world = buildMissionWorld({ seed: 5 });
    const targets = allTargets(world);
    expect(targets.filter((target) => target.classId === 'person' && target.building !== null)).toHaveLength(1);
    expect(targets.filter((target) => target.building === null)).toHaveLength(1);
  });

  it('applies interior lighting modes per room', () => {
    const on = buildMissionWorld({ seed: 8, interiorLights: 'on', targets: [] });
    const off = buildMissionWorld({ seed: 8, interiorLights: 'off', targets: [] });
    const rooms = on.buildings[0]!.shell.spec.floors.reduce((sum, floor) => sum + floor.rooms.length, 0);
    expect(on.buildings[0]!.interiorLightsOn).toHaveLength(rooms);
    expect([...on.buildings[0]!.interiorLightsOn].every((value) => value === 1)).toBe(true);
    expect([...off.buildings[0]!.interiorLightsOn].every((value) => value === 0)).toBe(true);
    const a = buildMissionWorld({ seed: 8, interiorLights: 'seeded', targets: [] });
    const b = buildMissionWorld({ seed: 8, interiorLights: 'seeded', targets: [] });
    expect([...a.buildings[0]!.interiorLightsOn]).toEqual([...b.buildings[0]!.interiorLightsOn]);
  });

  it('labels buildings and gives world-frame CCW footprints with heights', () => {
    const world = buildMissionWorld({ seed: 11, targets: [] });
    const labels = world.buildings.map((building) => building.label);
    expect(new Set(labels).size).toBe(labels.length);
    for (const building of world.buildings) {
      expect(building.id).toBe(`bldg-${building.shell.id}`);
      expect(signedArea(building.footprint)).toBeGreaterThan(0);
      expect(building.footprint[0]).toEqual({ x: building.shell.origin[0], y: building.shell.origin[1] });
      expect(building.heightM).toBe(building.shell.spec.floors.length * 3);
    }
  });
});

describe('mission for a world', () => {
  it('validates over many seeds, building counts, and mission types', () => {
    for (let seed = 0; seed < 24; seed++) {
      const world = buildMissionWorld({ seed, buildings: 1 + (seed % 4), targets: [] });
      const type = MISSION_TYPES[seed % MISSION_TYPES.length]!;
      const mission = missionForWorld(world, { missionType: type });
      const errors = validateMission(mission).issues.filter((issue) => issue.severity === 'error');
      expect(errors.map((issue) => `${seed}: ${issue.message}`)).toEqual([]);
      expect(mission.altitude.minM).toBeGreaterThanOrEqual(3);
      expect(mission.altitude.maxM).toBeGreaterThan(10);
      expect(mission.startPose.z).toBeGreaterThanOrEqual(mission.altitude.minM);
      expect(mission.startPose.z).toBeLessThanOrEqual(mission.altitude.maxM);
    }
  });

  it('keeps every site no-fly zone that reaches inside the geofence', () => {
    for (let seed = 0; seed < 20; seed++) {
      const world = buildMissionWorld({ seed, targets: [] });
      const zones = missionNoFly(world.outdoor);
      const fence = world.outdoor.geofence.polygon.map(([x, y]) => ({ x, y }));
      const kept = new Set(zones.flatMap((zone) => zone.id.replace('nf-', '').split('-').map(Number)));
      for (const zone of world.outdoor.noFly) {
        const inside = zone.polygon.some(([x, y]) => pointInRing({ x, y }, fence));
        if (inside) expect(kept.has(zone.id), `seed ${seed} zone ${zone.name}`).toBe(true);
      }
      for (const zone of zones) expect(ringInsideRing(zone.polygon.points, fence)).toBe(true);
    }
  });

  it('the page sample mission is valid', () => {
    const mission = woodsSampleMission(11);
    expect(validateMission(mission).ok).toBe(true);
    expect(mission.entryPolicy).toBe('ask');
  });
});

describe('outdoor world provider', () => {
  it('returns real building footprints, trails, and bounded canopy', () => {
    const snapshot = outdoorWorldProvider.load(11);
    const world = buildMissionWorld({ seed: 11 });
    const buildings = snapshot.footprints.filter((footprint) => footprint.kind === 'building');
    expect(buildings.map((footprint) => footprint.id)).toEqual(world.buildings.map((building) => building.id));
    for (const building of world.buildings) {
      const footprint = buildings.find((item) => item.id === building.id)!;
      expect(footprint.polygon).toEqual(building.footprint);
      expect(footprint.heightM).toBe(building.heightM);
      expect(footprint.label).toBe(building.label);
    }
    const canopy = snapshot.footprints.filter((footprint) => footprint.kind === 'canopy');
    expect(canopy.length).toBeGreaterThan(3);
    expect(canopy.length).toBeLessThanOrEqual(25);
    for (const footprint of canopy) {
      expect(footprint.polygon.length).toBeGreaterThanOrEqual(3);
      expect(signedArea(footprint.polygon)).toBeGreaterThan(0);
      expect(footprint.heightM).toBeLessThan(10);
    }
    expect(snapshot.footprints.filter((footprint) => footprint.kind === 'road')).toHaveLength(world.buildings.length);
    expect(outdoorWorldProvider.load(11)).toEqual(snapshot);
    expect(snapshot.seed).toBe(11);
  });
});
