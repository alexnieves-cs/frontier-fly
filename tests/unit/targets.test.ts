import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { CLIP_NAMES } from '../../src/targets/assets';
import { resolveScene } from '../../src/targets/motion';
import { placeIndoor, placeOutdoor } from '../../src/targets/placement';
import { JOINT_NAMES, samplePersonMotion } from '../../src/targets/skeleton';
import { PERSON_STATES } from '../../src/targets/types';
import { placementIssues } from '../../src/targets/validate';
import type { OutdoorArea, PlacedTarget } from '../../src/targets/types';

function speedOf(sample: NonNullable<ReturnType<typeof samplePersonMotion>>, name: string): number {
  const joint = sample.joints.find((item) => item.name === name)!;
  return Math.hypot(joint.velocity[0], joint.velocity[1], joint.velocity[2]);
}

describe('target placement', () => {
  it('is deterministic for a seed', () => {
    const spec = generateBuilding({ seed: 4, floors: 1, w: 16, d: 16, stage: 2 });
    const a = placeIndoor(spec, { seed: 4, people: 4, objects: 6, decoys: 2 });
    const b = placeIndoor(spec, { seed: 4, people: 4, objects: 6, decoys: 2 });
    expect(a).toEqual(b);
    expect(resolveScene(a.targets, 1.25)).toEqual(resolveScene(b.targets, 1.25));
  });

  it('keeps indoor bodies on reachable floor cells', () => {
    for (let seed = 1; seed <= 8; seed++) {
      const spec = generateBuilding({ seed, floors: 1, w: 18, d: 16, stage: seed % 2 === 0 ? 3 : 2 });
      const scene = placeIndoor(spec, { seed, people: 4, objects: 6, decoys: 2 });
      expect(placementIssues(scene, spec), `seed ${seed}`).toEqual([]);
      expect(scene.targets.filter((target) => target.classId === 'person').length).toBe(4);
      expect(scene.targets.filter((target) => target.category === 'object').length).toBe(6);
      expect(scene.targets.filter((target) => target.category === 'decoy').length).toBe(2);
    }
  });

  it('lines up every state, object, and decoy in the showcase room', () => {
    const spec = generateBuilding({ seed: 11, floors: 1, w: 14, d: 12, stage: 0 });
    const scene = placeIndoor(spec, { seed: 11, showcase: true });
    expect(placementIssues(scene, spec)).toEqual([]);
    const states = new Set(scene.targets.filter((target) => target.classId === 'person').map((target) => target.state));
    expect([...states].sort()).toEqual([...PERSON_STATES].sort());
    for (const id of ['backpack', 'phone', 'toolbox', 'medkit', 'bottle', 'radio', 'helmet', 'blanket', 'mannequin', 'coat', 'decoy_pack', 'decoy_case']) {
      expect(scene.targets.some((target) => target.classId === id), id).toBe(true);
    }
    const hiding = scene.targets.find((target) => target.state === 'hiding');
    expect(hiding?.occluders.length).toBe(1);
  });

  it('keeps outdoor bodies inside the area and out of obstacles', () => {
    const area: OutdoorArea = {
      id: 'meadow',
      minX: 0,
      minY: 0,
      maxX: 30,
      maxY: 24,
      groundZ: 0,
      obstacles: [{ minX: 14, minY: 10, maxX: 17, maxY: 14 }],
      paths: [[[3, 4], [10, 4], [10, 8], [3, 8]]],
    };
    for (const seed of [2, 9, 15]) {
      const scene = placeOutdoor(area, { seed, people: 4, objects: 4, decoys: 2 });
      expect(placementIssues(scene), `seed ${seed}`).toEqual([]);
      expect(scene.targets.filter((target) => target.classId === 'person').length).toBe(4);
      const walker = scene.targets.find((target) => target.state === 'walking');
      expect(walker?.path?.length).toBeGreaterThanOrEqual(2);
      const moved = resolveScene(scene.targets, 2.2).find((target) => target.id === walker?.id);
      expect(moved && Math.hypot(moved.feet[0] - walker!.feet[0], moved.feet[1] - walker!.feet[1])).toBeGreaterThan(0.4);
    }
  });
});

describe('person motion hook', () => {
  const walker: PlacedTarget = {
    id: 'person-walking-0',
    classId: 'person',
    category: 'person',
    state: 'walking',
    feet: [2, 2, 0],
    yaw: 0,
    hx: 0.22,
    hy: 0.16,
    height: 1.75,
    speed: 1.15,
    path: [[2, 2, 0], [6, 2, 0]],
    floor: 0,
    roomIndex: 0,
    occluders: [],
  };

  it('reports joint velocities for a walking person and almost none while standing', () => {
    const moving = samplePersonMotion(walker, 0.35);
    const standing = samplePersonMotion({ ...walker, id: 'person-standing-0', state: 'standing', path: undefined, speed: 0 }, 0.35);
    expect(moving?.joints.map((joint) => joint.name)).toEqual([...JOINT_NAMES]);
    const swing = Math.max(speedOf(moving!, 'foot.l'), speedOf(moving!, 'foot.r'));
    expect(swing).toBeGreaterThan(0.4);
    expect(Math.max(...standing!.joints.map((joint) => Math.hypot(...joint.velocity)))).toBeLessThan(0.08);
    expect(samplePersonMotion(walker, 0.35)).toEqual(moving);
    expect(samplePersonMotion({ ...walker, classId: 'backpack', category: 'object', state: 'static' }, 0.2)).toBeNull();
  });

  it('lays an injured person on the ground and crouches someone hiding', () => {
    const lying = samplePersonMotion({ ...walker, id: 'person-lying-0', state: 'lying', path: undefined, speed: 0 }, 0);
    const hiding = samplePersonMotion({ ...walker, id: 'person-hiding-0', state: 'hiding', path: undefined, speed: 0 }, 0);
    const head = lying!.joints.find((joint) => joint.name === 'head')!;
    const foot = lying!.joints.find((joint) => joint.name === 'foot.l')!;
    expect(head.position[2]).toBeLessThan(0.6);
    expect(Math.hypot(head.position[0] - foot.position[0], head.position[1] - foot.position[1])).toBeGreaterThan(1);
    expect(hiding!.joints.find((joint) => joint.name === 'head')!.position[2]).toBeLessThan(1.2);
  });
});

describe('humanoid asset', () => {
  it('packs only the civilian KayKit clips', () => {
    const data = readFileSync('public/assets/targets/humanoid.glb');
    expect(data.subarray(0, 4).toString()).toBe('glTF');
    const jsonLength = data.readUInt32LE(12);
    const json = JSON.parse(data.subarray(20, 20 + jsonLength).toString()) as { animations: Array<{ name: string }>; asset: { extras: { license: string } } };
    const names = json.animations.map((clip) => clip.name).sort();
    expect(names).toEqual([...CLIP_NAMES].sort());
    expect(names.some((name) => /death|hit|attack|shoot/i.test(name))).toBe(false);
    expect(json.asset.extras.license).toBe('CC0-1.0');
  });
});
