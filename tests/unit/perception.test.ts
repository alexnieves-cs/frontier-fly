import { describe, expect, it } from 'vitest';
import raw from '../fixtures/simpleRoom.json';
import { buildingFromJson } from '../../src/shared/serialize';
import { cameraLookingAt, detect, fpvCameraFromDrone, illuminationOf, occlusionFraction, projectAabb, recallProbability, roomCellKeys, SeenEmptyTracker, indoorCells } from '../../src/perception';
import { frameOnTargets, placeIndoor, resolveScene } from '../../src/targets';
import { bodyAabb, centerOf } from '../../src/targets/motion';
import { generateBuilding } from '../../src/gen/building';
import type { SensorMode } from '../../src/shared/types';
import type { ResolvedTarget, TargetClass, Vec3 } from '../../src/targets/types';

const spec = buildingFromJson(raw);
const RATE_TOL = 0.1;

function light(timeOfDay: number, flashlight: boolean, interior = false, sensorMode: SensorMode = 'rgb') {
  return {
    timeOfDay,
    interiorLightsOn: new Uint8Array([interior ? 1 : 0]),
    flashlightOn: flashlight,
    sensorMode,
  };
}

function targetAt(classId: TargetClass, distance: number, speed = 0, height = 1.75, hx = 0.22, hy = 0.16): ResolvedTarget {
  const feet: Vec3 = [0, distance, 0];
  const aabb = bodyAabb(feet, 0, hx, hy, height);
  const decoy = classId === 'mannequin' || classId === 'coat' || classId === 'decoy_pack' || classId === 'decoy_case';
  const looksLike = classId === 'mannequin' || classId === 'coat' ? 'person' as const : classId === 'decoy_pack' ? 'backpack' as const : classId === 'decoy_case' ? 'toolbox' as const : undefined;
  return {
    id: `${classId}-0`,
    classId,
    category: classId === 'person' ? 'person' : decoy ? 'decoy' : 'object',
    state: classId === 'person' ? (speed > 0 ? 'walking' : 'standing') : 'static',
    feet,
    center: centerOf(aabb),
    yaw: 0,
    aabb,
    speed,
    floor: 0,
    roomIndex: 0,
    occluders: [],
    looksLike,
  };
}

function cameraAt(distance: number) {
  return cameraLookingAt([0, 0, 1.2], [0, distance, 1.05], [0, 0, 1], (70 * Math.PI) / 180, 16 / 9);
}

function hitRate(distance: number, lighting: ReturnType<typeof light>, speed: number, classId: TargetClass, trials: number, kind: 'see' | 'fp' = 'see') {
  let hits = 0;
  let expected = 0;
  for (let i = 0; i < trials; i++) {
    const target = targetAt(classId, distance, speed, classId === 'phone' ? 0.15 : classId === 'medkit' ? 0.22 : 1.75, classId === 'phone' ? 0.04 : 0.22, classId === 'phone' ? 0.015 : 0.16);
    const camera = cameraAt(distance);
    const dets = detect({ seed: 8000 + i, camera, targets: [target], lighting });
    const hit = kind === 'fp'
      ? dets.some((det) => det.kind === 'fp' && det.classId === target.looksLike)
      : dets.some((det) => det.trueClass === classId && det.kind !== 'fp');
    if (hit) hits += 1;
    const projected = projectAabb(camera, target.aabb);
    const illumination = illuminationOf({
      timeOfDay: lighting.timeOfDay,
      interiorOn: (lighting.interiorLightsOn[0] ?? 0) > 0,
      flashlightOn: lighting.flashlightOn,
      inFlashlight: lighting.flashlightOn,
      sensorMode: lighting.sensorMode,
    });
    const recallClass = kind === 'fp' && target.looksLike ? target.looksLike : classId;
    const recall = recallProbability({
      classId: recallClass,
      distance,
      occlusion: 0,
      illumination,
      speed,
      cameraSpeed: 0,
      area: projected?.area ?? 1,
    });
    expected += kind === 'fp' ? 0.58 * recall : recall;
  }
  return { hits: hits / trials, expected: expected / trials };
}

describe('simulated detector', () => {
  it('is deterministic for a seed and does not depend on target order', () => {
    const targets = [targetAt('person', 4), targetAt('backpack', 3.2, 0, 0.46, 0.18, 0.12), targetAt('mannequin', 5)];
    targets[1]!.id = 'object-backpack-1';
    targets[2]!.id = 'decoy-mannequin-2';
    const camera = cameraAt(4);
    const lighting = light(12, false, true);
    const a = detect({ seed: 19, camera, targets, lighting });
    const b = detect({ seed: 19, camera, targets: [...targets].reverse(), lighting });
    expect(a).toEqual(b);
    expect(detect({ seed: 20, camera, targets, lighting })).not.toEqual(a);
  });

  it('projects a centered target to the middle of the frame', () => {
    const camera = cameraLookingAt([0, 0, 1.5], [0, 6, 1.5], [0, 0, 1], Math.PI / 2, 1);
    const projected = projectAabb(camera, { min: [-0.2, 5.6, 1.2], max: [0.2, 6.2, 1.8] });
    expect(projected).not.toBeNull();
    expect(projected!.bbox.x + projected!.bbox.w / 2).toBeCloseTo(0.5, 1);
    expect(projected!.bbox.y + projected!.bbox.h / 2).toBeCloseTo(0.5, 1);
  });

  it('drops recall as distance grows, and drops it further at night', () => {
    const day = (distance: number) => recallProbability({ classId: 'person', distance, occlusion: 0, illumination: 1, speed: 0, cameraSpeed: 0, area: 1 });
    expect(day(2)).toBeGreaterThan(0.9);
    expect(day(9)).toBeCloseTo(0.5, 1);
    expect(day(16)).toBeLessThan(0.08);
    expect(day(2)).toBeGreaterThan(day(9));
    expect(day(9)).toBeGreaterThan(day(16));
    const nightIllum = illuminationOf({ timeOfDay: 23, interiorOn: false, flashlightOn: false, inFlashlight: false, sensorMode: 'rgb' });
    const phoneNight = recallProbability({ classId: 'phone', distance: 2, occlusion: 0, illumination: nightIllum, speed: 0, cameraSpeed: 0, area: 1 });
    const phoneDay = recallProbability({ classId: 'phone', distance: 2, occlusion: 0, illumination: 1, speed: 0, cameraSpeed: 0, area: 1 });
    const personNight = recallProbability({ classId: 'person', distance: 2, occlusion: 0, illumination: nightIllum, speed: 0, cameraSpeed: 0, area: 1 });
    expect(nightIllum).toBeLessThan(0.15);
    expect(personNight).toBeLessThan(day(2) * 0.2);
    expect(phoneNight / phoneDay).toBeLessThan(personNight / day(2));
    const flash = illuminationOf({ timeOfDay: 23, interiorOn: false, flashlightOn: true, inFlashlight: true, sensorMode: 'rgb' });
    expect(flash).toBeGreaterThan(0.7);
  });

  it('penalizes walking more than standing at the same range', () => {
    const standing = recallProbability({ classId: 'person', distance: 3, occlusion: 0, illumination: 1, speed: 0, cameraSpeed: 0, area: 1 });
    const walking = recallProbability({ classId: 'person', distance: 3, occlusion: 0, illumination: 1, speed: 1.15, cameraSpeed: 0, area: 1 });
    expect(walking).toBeLessThan(standing * 0.8);
  });

  it('matches the calibrated distance and lighting curves within tolerance', () => {
    const near = hitRate(2.5, light(12, false, true), 0, 'person', 280);
    const mid = hitRate(9, light(12, false, true), 0, 'person', 280);
    const far = hitRate(16, light(12, false, true), 0, 'person', 280);
    const dark = hitRate(2.5, light(23, false, false), 0, 'person', 280);
    expect(Math.abs(near.hits - near.expected)).toBeLessThan(RATE_TOL);
    expect(Math.abs(mid.hits - mid.expected)).toBeLessThan(RATE_TOL);
    expect(Math.abs(far.hits - far.expected)).toBeLessThan(RATE_TOL);
    expect(Math.abs(dark.hits - dark.expected)).toBeLessThan(RATE_TOL);
    expect(near.hits).toBeGreaterThan(mid.hits);
    expect(mid.hits).toBeGreaterThan(far.hits);
    expect(dark.hits).toBeLessThan(near.hits - 0.4);
  });

  it('clusters false positives on decoys and rarely calls a medkit a person', () => {
    const mannequin = hitRate(4, light(12, false, true), 0, 'mannequin', 280, 'fp');
    const coat = hitRate(4, light(12, false, true), 0, 'coat', 280, 'fp');
    expect(Math.abs(mannequin.hits - mannequin.expected)).toBeLessThan(RATE_TOL);
    expect(coat.hits).toBeLessThan(mannequin.hits - 0.15);
    let medkitPerson = 0;
    for (let i = 0; i < 200; i++) {
      const target = targetAt('medkit', 3, 0, 0.22, 0.14, 0.07);
      const dets = detect({ seed: 100 + i, camera: cameraAt(3), targets: [target], lighting: light(12, false, true) });
      if (dets.some((det) => det.classId === 'person')) medkitPerson += 1;
    }
    expect(medkitPerson).toBe(0);
    const camera = cameraAt(4);
    const decoy = targetAt('mannequin', 4);
    const projected = projectAabb(camera, decoy.aabb)!;
    const cx = projected.bbox.x + projected.bbox.w / 2;
    const cy = projected.bbox.y + projected.bbox.h / 2;
    let clustered = 0;
    let fps = 0;
    for (let i = 0; i < 80; i++) {
      const dets = detect({ seed: 300 + i, camera, targets: [decoy], lighting: light(12, false, true) });
      for (const det of dets.filter((item) => item.kind === 'fp')) {
        fps += 1;
        const dx = det.bbox.x + det.bbox.w / 2 - cx;
        const dy = det.bbox.y + det.bbox.h / 2 - cy;
        if (Math.hypot(dx, dy) < 0.03) clustered += 1;
      }
    }
    expect(fps).toBeGreaterThan(10);
    expect(clustered).toBe(fps);
  });

  it('misses a person hidden behind a wall', () => {
    // y=4.5 sits on a drywall cell of the east wall. y=5 is a glass strip.
    const camera = cameraLookingAt([3, 4.5, 1.4], [8, 4.5, 1.2], [0, 0, 1], Math.PI / 2, 1.2);
    const outside = targetAt('person', 5);
    outside.feet = [14, 4.5, 0];
    outside.aabb = bodyAabb(outside.feet, 0, 0.22, 0.16, 1.75);
    outside.center = centerOf(outside.aabb);
    const blocked = occlusionFraction(camera, outside.aabb, spec, []);
    expect(blocked).toBeGreaterThan(0.9);
    for (let i = 0; i < 30; i++) {
      const dets = detect({ seed: i, camera, targets: [outside], spec, lighting: light(12, false, true) });
      expect(dets).toEqual([]);
    }
  });

  it('sees the showcase lineup from the framed camera', () => {
    const building = generateBuilding({ seed: 11, floors: 1, w: 14, d: 12, stage: 0 });
    const scene = placeIndoor(building, { seed: 11, showcase: true });
    const resolved = resolveScene(scene.targets, 0.4);
    const frame = frameOnTargets(building, resolved, 3.8);
    const camera = cameraLookingAt(frame.eye, frame.look, [0, 0, 1], (110 * Math.PI) / 180, 16 / 9);
    const dets = detect({ seed: 11, camera, targets: resolved, spec: building, lighting: light(13, false, true) });
    expect(dets.some((det) => det.classId === 'person' || det.trueClass === 'person')).toBe(true);
  });
});

describe('seen-empty confirm-clear', () => {
  const day = light(12, false, true);

  function cover(tracker: SeenEmptyTracker, targets: ResolvedTarget[]) {
    const eye: Vec3 = [5, 5, 1.45];
    for (const yaw of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) {
      const look: Vec3 = [eye[0] + Math.cos(yaw) * 4, eye[1] + Math.sin(yaw) * 4, 1.2];
      const camera = cameraLookingAt(eye, look, [0, 0, 1], (150 * Math.PI) / 180, 1.2);
      tracker.observe(camera, targets, day);
    }
  }

  it('confirms an empty room only after it has been looked through', () => {
    const tracker = new SeenEmptyTracker(indoorCells(spec), 'person', spec);
    const keys = roomCellKeys(spec, 0, 0);
    expect(tracker.report(keys, []).status).toBe('incomplete');
    cover(tracker, []);
    const report = tracker.report(keys, []);
    expect(report.status).toBe('clear');
    expect(report.seenEmpty).toBe(keys.length);
  });

  it('does not confirm-clear a room that still holds a person', () => {
    const tracker = new SeenEmptyTracker(indoorCells(spec), 'person', spec);
    const person = targetAt('person', 5);
    person.feet = [6, 5, 0];
    person.aabb = bodyAabb(person.feet, 0, 0.22, 0.16, 1.75);
    person.center = centerOf(person.aabb);
    cover(tracker, [person]);
    const report = tracker.report(roomCellKeys(spec, 0, 0), [person]);
    expect(report.status).toBe('occupied');
    expect(report.occupied).toBeGreaterThan(0);
    expect(tracker.seenEmptyKeys().some((key) => key === '0:6:5')).toBe(false);
  });

  it('does not mark a dark room seen until the flashlight covers it', () => {
    const tracker = new SeenEmptyTracker(indoorCells(spec), 'person', spec);
    const camera = fpvCameraFromDrone([4, 5, 1.4], 0, 1.4);
    tracker.observe(camera, [], light(23, false, false));
    expect(tracker.seenEmptyKeys()).toEqual([]);
    tracker.observe(camera, [], light(23, true, false));
    expect(tracker.seenEmptyKeys().length).toBeGreaterThan(0);
  });
});
