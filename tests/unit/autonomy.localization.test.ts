import { describe, expect, it } from 'vitest';
import type { MissionForWorldOptions, MissionWorldOptions } from '../../src/mission/autonomy/contracts';
import type { LocFrame } from '../../src/loc/types';
import { Localizer } from '../../src/mission/autonomy/localizer';
import { indoorGrid, type LidarWorld } from '../../src/mission/autonomy/planner/indoor';
import { runMission } from '../../src/mission/autonomy/runner';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { createFakeSource } from '../helpers/fakeSource';
import { SegmentWorld } from '../helpers/segmentWorld';

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

describe('localizer', () => {
  it('casts lidar from the estimated frame into the real building', () => {
    const world = buildMissionWorld(PERSON_WORLD);
    const grid = indoorGrid(world.buildings[0]!);
    const room = new SegmentWorld().box(0, 0, 8, 6);
    const start = { pos: [3, 2, 1] as [number, number, number], yaw: 0.3, vel: [0, 0, 0] as [number, number, number], accel: [0, 0, 0] as [number, number, number], yawRate: 0, dt: 0.05 };
    const loc = new Localizer(grid, room, 5, false, start);
    loc.step(start, 0.05);
    const e = loc.estimate;
    // The estimate starts off the truth by the entry error.
    expect(Math.hypot(e.pos[0] - 3, e.pos[1] - 2) + Math.abs(e.yaw - 0.3)).toBeGreaterThan(1e-3);
    const framed: LidarWorld = loc.frameLidar(room);
    for (const a of [0, 1, 2.5, 4]) {
      // A beam at body angle a, cast from the estimate, measures what the real beam measures.
      const est = framed.cast(e.pos[0], e.pos[1], 1, Math.cos(e.yaw + a), Math.sin(e.yaw + a), 0, 20)!;
      const real = room.cast(3, 2, 1, Math.cos(0.3 + a), Math.sin(0.3 + a), 0, 20)!;
      expect(est.dist).toBeCloseTo(real.dist, 9);
    }
  });
});

describe('missions on the estimated pose', () => {
  it('finds the person indoors without the true pose, and records its own error', async () => {
    const world = buildMissionWorld(PERSON_WORLD);
    const mission = missionForWorld(world, FIND_PERSON);
    const fly = async () => runMission(mission, world, await createFakeSource(world), { dt: 0.05, localization: 'estimated' });
    const r = await fly();
    expect(r.outcome).toBe('found');
    expect(r.landedHome).toBe(true);
    expect(r.collisions).toHaveLength(0);
    expect(r.localization).toHaveLength(1);
    const m = r.localization[0]!.metrics;
    expect(m.ticks).toBeGreaterThan(200);
    // Error against the building stays near the entry error; drift against its own map is a few cm.
    expect(m.ateRmse).toBeLessThan(0.3);
    expect(m.ateMapRmse).toBeLessThan(0.1);
    expect(r.log.some((l) => /estimated pose/.test(l.message))).toBe(true);
    const again = await fly();
    expect(JSON.stringify(again)).toBe(JSON.stringify(r));
  });

  it('hovers while the estimate is lost and carries on once it re-acquires', async () => {
    const world = buildMissionWorld(PERSON_WORLD);
    let first = -1;
    // Five seconds into the visit, two seconds of lidar sweeps turned by 80 degrees: scans that contradict the map.
    const locFault = (frame: LocFrame, t: number) => {
      if (first < 0) first = t;
      if (!frame.scan || t < first + 5 || t > first + 7) return;
      const r = frame.scan.ranges;
      const turned = Float64Array.from(r, (_, i) => r[(i + 20) % r.length]!);
      r.set(turned);
    };
    const r = await runMission(missionForWorld(world, FIND_PERSON), world, await createFakeSource(world), { dt: 0.05, localization: 'estimated', locFault });
    const lost = r.log.findIndex((l) => /Pose estimate lost/.test(l.message));
    expect(lost).toBeGreaterThan(-1);
    expect(r.log.slice(lost).some((l) => /re-acquired/.test(l.message))).toBe(true);
    expect(r.localization[0]!.lostEvents).toBeGreaterThan(0);
    expect(r.outcome).toBe('found');
    expect(r.collisions).toHaveLength(0);
  });

  it('hands a pose lost past the re-acquire hover to the failsafe, which lands in place', async () => {
    const world = buildMissionWorld(PERSON_WORLD);
    let first = -1;
    // From five seconds into the visit on, every sweep is turned by 80 degrees: the estimate cannot re-acquire.
    const locFault = (frame: LocFrame, t: number) => {
      if (first < 0) first = t;
      if (!frame.scan || t < first + 5) return;
      const r = frame.scan.ranges;
      r.set(Float64Array.from(r, (_, i) => r[(i + 20) % r.length]!));
    };
    const r = await runMission(missionForWorld(world, FIND_PERSON), world, await createFakeSource(world), { dt: 0.05, localization: 'estimated', locFault });
    const lost = r.log.find((l) => /Pose estimate lost/.test(l.message));
    const fault = r.safetyEvents.find((e) => e.kind === 'estimator_fault' && e.action === 'land');
    expect(lost).toBeDefined();
    expect(fault).toBeDefined();
    expect(fault!.tSec - lost!.tSec).toBeGreaterThanOrEqual(20);
    expect(r.collisions).toHaveLength(0);
  });

  it('leaves truth-pose runs without a localization record', async () => {
    const world = buildMissionWorld(PERSON_WORLD);
    const r = await runMission(missionForWorld(world, FIND_PERSON), world, await createFakeSource(world), { dt: 0.05 });
    expect(r.localization).toEqual([]);
  });
});
