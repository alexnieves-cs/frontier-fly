import { describe, expect, it } from 'vitest';
import type { Contact, DronePose, MissionWorld, V3 } from '../../src/mission/autonomy/contracts';
import { createPerceptionSource, gimbalFootprint, GIMBAL_MIN_AGL_M, GIMBAL_RANGE_SCALE } from '../../src/mission/autonomy/perception';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { createPerceptionDetectionSource } from '../../src/mission/perceptionSource';
import { pathLength, planPath, pointAtDistance } from '../../src/mission/path';
import { indoorCells } from '../../src/perception/seenEmpty';
import { CELL } from '../../src/shared/constants';
import { PERSON_SIZE } from '../../src/targets/catalog';
import type { PlacedTarget } from '../../src/targets/types';
import { sampleHeight } from '../../src/world/outdoor/terrain';
import type { Opening } from '../../src/world/outdoor/types';
import { outdoorWorldProvider } from '../../src/mission/autonomy/world';

interface Setup {
  world: MissionWorld;
  opening: Opening;
  /** Local feet of the planted person. */
  feet: V3;
}

function roomIndexAt(world: MissionWorld, ix: number, iy: number): number {
  const rooms = world.buildings[0]!.shell.spec.floors[0]!.rooms;
  return rooms.findIndex((room) => ix >= room.x && ix < room.x + room.w && iy >= room.z && iy < room.z + room.d);
}

/** A one-building world with a standing person planted 2.5 m inside a door-sized opening. */
function doorwaySetup(options: { timeOfDay?: number; interiorLights?: 'on' | 'off' } = {}): Setup {
  for (let seed = 1; seed < 200; seed++) {
    const world = buildMissionWorld({ seed, buildings: 1, targets: [], ...options });
    const shell = world.buildings[0]!.shell;
    const opening = shell.openings.find((item) => item.passable && item.floor === 0 && (item.kind === 'ajar_door' || item.kind === 'garage_gap'));
    if (!opening) continue;
    const local: V3 = [
      opening.center[0] - shell.origin[0] - opening.normal[0] * 2.5,
      opening.center[1] - shell.origin[1] - opening.normal[1] * 2.5,
      0,
    ];
    const floor = shell.spec.floors[0]!;
    const ix = Math.floor(local[0]);
    const iy = Math.floor(local[1]);
    if (floor.cells[iy * floor.w + ix] !== CELL.EMPTY) continue;
    const roomIndex = roomIndexAt(world, ix, iy);
    if (roomIndex < 0) continue;
    const size = PERSON_SIZE.standing;
    const person: PlacedTarget = {
      id: 'bldg-0/person-standing-test',
      classId: 'person',
      category: 'person',
      state: 'standing',
      feet: [ix + 0.5, iy + 0.5, 0],
      yaw: 0,
      hx: size.hx,
      hy: size.hy,
      height: size.height,
      speed: 0,
      floor: 0,
      roomIndex,
      occluders: [],
    };
    world.buildings[0]!.scene = { ...world.buildings[0]!.scene, targets: [person] };
    return { world, opening, feet: person.feet };
  }
  throw new Error('no doorway seed');
}

function toWorld(world: MissionWorld, local: V3): V3 {
  const o = world.buildings[0]!.shell.origin;
  return [local[0] + o[0], local[1] + o[1], local[2] + o[2]];
}

function poseLooking(from: V3, at: V3): DronePose {
  return { pos: from, yaw: Math.atan2(at[1] - from[1], at[0] - from[0]), vel: [0, 0, 0] };
}

function hits(contacts: Contact[], targetId: string): Contact[] {
  return contacts.filter((contact) => contact.truth.targetId === targetId && contact.truth.kind === 'tp');
}

function run(world: MissionWorld, pose: DronePose, frames: number): Contact[] {
  const source = createPerceptionSource(world);
  const out: Contact[] = [];
  for (let i = 0; i < frames; i++) out.push(...source.observe(pose, i * 0.2).contacts);
  return out;
}

/** A spot inside the person's room, 2 to 4 m away, on an empty cell. */
function insidePose(setup: Setup): DronePose {
  const floor = setup.world.buildings[0]!.shell.spec.floors[0]!;
  const room = floor.rooms[roomIndexAt(setup.world, Math.floor(setup.feet[0]), Math.floor(setup.feet[1]))]!;
  let best: V3 | null = null;
  for (let iy = room.z; iy < room.z + room.d; iy++) {
    for (let ix = room.x; ix < room.x + room.w; ix++) {
      if (floor.cells[iy * floor.w + ix] !== CELL.EMPTY) continue;
      const d = Math.hypot(ix + 0.5 - setup.feet[0], iy + 0.5 - setup.feet[1]);
      if (d >= 2 && d <= 4 && (!best || d > Math.hypot(best[0] - setup.feet[0], best[1] - setup.feet[1]))) best = [ix + 0.5, iy + 0.5, 1.4];
    }
  }
  if (!best) throw new Error('no inside spot');
  const center = toWorld(setup.world, [setup.feet[0], setup.feet[1], 0.9]);
  return poseLooking(toWorld(setup.world, best), center);
}

describe('perception source', () => {
  it('sees a person through an opening and from inside the room, not through a solid wall', () => {
    const setup = doorwaySetup();
    const { world, opening } = setup;
    const id = 'bldg-0/person-standing-test';
    const center = toWorld(world, [setup.feet[0], setup.feet[1], 0.9]);

    const outsideDoor: V3 = [opening.center[0] + opening.normal[0] * 2, opening.center[1] + opening.normal[1] * 2, world.buildings[0]!.shell.origin[2] + 1.4];
    const throughDoor = hits(run(world, poseLooking(outsideDoor, center), 20), id);
    expect(throughDoor.length).toBeGreaterThan(0);
    expect(throughDoor[0]!.buildingId).toBe('bldg-0');

    const inside = hits(run(world, insidePose(setup), 20), id);
    expect(inside.length).toBeGreaterThan(0);

    // Same face, offset along the wall so the sight line crosses solid wall.
    const shell = world.buildings[0]!.shell;
    const blocked = (point: V3): boolean => {
      const local = [point[0] - shell.origin[0], point[1] - shell.origin[1]];
      const feet = setup.feet;
      const n = opening.normal;
      // Wall cell layer the sight line crosses: half a cell inside the outer face.
      const faceOffset = (opening.center[0] - shell.origin[0]) * Math.abs(n[0]) + (opening.center[1] - shell.origin[1]) * Math.abs(n[1]);
      const axis = Math.abs(n[0]) > 0.5 ? 0 : 1;
      const t = (faceOffset - feet[axis]!) / (local[axis]! - feet[axis]!);
      const cross = [feet[0] + (local[0]! - feet[0]) * t, feet[1] + (local[1]! - feet[1]) * t];
      const ix = Math.floor(cross[0]!);
      const iy = Math.floor(cross[1]!);
      return !shell.openings.some((item) => item.apertureCells.some((cell) => Math.abs(cell.x - ix) <= 1 && Math.abs(cell.z - iy) <= 1));
    };
    const candidates = [3, -3, 4, -4, 5, -5].map((off): V3 => [outsideDoor[0] + opening.tangent[0] * off, outsideDoor[1] + opening.tangent[1] * off, outsideDoor[2]]);
    const wallSpot = candidates.find(blocked);
    expect(wallSpot).toBeDefined();
    expect(Math.hypot(wallSpot![0] - center[0], wallSpot![1] - center[1])).toBeLessThan(7);
    expect(hits(run(world, poseLooking(wallSpot!, center), 40), id)).toEqual([]);

    // Control: knock out the walls and the same pose sees the person.
    const floor = shell.spec.floors[0]!;
    const cells = floor.cells.map((cell) => (cell === CELL.WALL ? CELL.EMPTY : cell));
    const open: MissionWorld = {
      ...world,
      buildings: [{ ...world.buildings[0]!, shell: { ...shell, openings: [], spec: { ...shell.spec, floors: [{ ...floor, cells }, ...shell.spec.floors.slice(1)] } } }],
    };
    expect(hits(run(open, poseLooking(wallSpot!, center), 40), id).length).toBeGreaterThan(0);
  });

  it('the roof hides people inside from the aerial camera', () => {
    for (const seed of [1, 4, 6, 9]) {
      const world = buildMissionWorld({ seed, buildings: 2, targets: [{ classId: 'person', state: 'lying', where: 'indoor', building: 1 }] });
      const building = world.buildings[1]!;
      const person = building.scene.targets[0]!;
      const o = building.shell.origin;
      const above: V3 = [person.feet[0] + o[0], person.feet[1] + o[1], o[2] + building.heightM + 6];
      const source = createPerceptionSource(world);
      const seen: Contact[] = [];
      for (let i = 0; i < 20; i++) seen.push(...source.observe({ pos: above, yaw: i * 0.3, vel: [0, 0, 0] }, i * 0.2).contacts);
      expect(hits(seen, person.id)).toEqual([]);
    }
  });

  it('estimates the position within a couple of meters and reports a unit bearing', () => {
    const setup = doorwaySetup();
    const found = hits(run(setup.world, insidePose(setup), 20), 'bldg-0/person-standing-test');
    const truth = createPerceptionSource(setup.world).truthTargets().find((target) => target.id === 'bldg-0/person-standing-test')!;
    expect(found.length).toBeGreaterThan(0);
    for (const contact of found) {
      expect(Math.hypot(contact.estimate[0] - truth.world[0], contact.estimate[1] - truth.world[1], contact.estimate[2] - truth.world[2])).toBeLessThan(2);
      expect(Math.hypot(...contact.bearing)).toBeCloseTo(1, 6);
      expect(contact.classId).toBe('person');
    }
  });

  it('night with lights and flashlight off lowers recall', () => {
    const day = doorwaySetup({ timeOfDay: 12, interiorLights: 'off' });
    const night = doorwaySetup({ timeOfDay: 23.5, interiorLights: 'off' });
    expect(night.world.flashlightOn).toBe(false);
    const dayHits = hits(run(day.world, insidePose(day), 60), 'bldg-0/person-standing-test').length;
    const nightHits = hits(run(night.world, insidePose(night), 60), 'bldg-0/person-standing-test').length;
    expect(dayHits).toBeGreaterThan(20);
    expect(nightHits).toBeLessThan(dayHits / 2);
    night.world.flashlightOn = true;
    const lit = hits(run(night.world, insidePose(night), 60), 'bldg-0/person-standing-test').length;
    expect(lit).toBeGreaterThan(nightHits);
  });

  it('is deterministic for a world seed and frame sequence', () => {
    const world = buildMissionWorld({ seed: 19, targets: [{ classId: 'person', state: 'walking', where: 'indoor' }, { classId: 'person', state: 'lying', where: 'outdoor' }], decoys: 2 });
    const mission = missionForWorld(world, { missionType: 'find_person' });
    const path = planPath(mission);
    const poses: DronePose[] = [];
    for (let d = 0; d < 200; d += 1) {
      const at = pointAtDistance(path, d);
      poses.push({ pos: [at.x, at.y, sampleHeight(world.outdoor.terrain, at.x, at.y) + (d % 40 < 20 ? 12 : 2)], yaw: at.yawRad, vel: [1, 0, 0] });
    }
    const a = createPerceptionSource(world);
    const b = createPerceptionSource(world);
    const runA = poses.map((pose, i) => a.observe(pose, i * 0.2));
    const runB = poses.map((pose, i) => b.observe(pose, i * 0.2));
    expect(JSON.stringify(runA)).toBe(JSON.stringify(runB));
    expect(JSON.stringify(a.clearStatus('outdoor', 'person'))).toBe(JSON.stringify(b.clearStatus('outdoor', 'person')));
  });

  it('confirm-clear: an indoor sweep clears an empty building and flags an occupied one', () => {
    const sweep = (world: MissionWorld) => {
      const source = createPerceptionSource(world);
      const building = world.buildings[0]!;
      const before = source.clearStatus(building.id, 'person');
      const o = building.shell.origin;
      let t = 0;
      for (const cell of indoorCells(building.shell.spec)) {
        const [, ix, iy] = cell.key.split(':').map(Number) as [number, number, number];
        if ((ix + iy) % 2 !== 0) continue;
        for (const yaw of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) {
          source.observe({ pos: [cell.center[0] + o[0], cell.center[1] + o[1], o[2] + cell.floor * 3 + 1.4], yaw, vel: [0, 0, 0] }, (t += 0.2));
        }
      }
      return { before, after: source.clearStatus(building.id, 'person'), objects: source.clearStatus(building.id, 'object') };
    };
    const empty = sweep(buildMissionWorld({ seed: 12, buildings: 1, targets: [] }));
    expect(empty.before.status).toBe('incomplete');
    expect(empty.before.seenEmpty).toBe(0);
    expect(empty.after.status).not.toBe('occupied');
    expect(empty.after.seenEmpty).toBeGreaterThan(empty.after.total * 0.9);
    expect(empty.after.unseen).toBeLessThan(empty.before.unseen);
    const occupied = sweep(buildMissionWorld({ seed: 12, buildings: 1, targets: [{ classId: 'person', state: 'sitting', where: 'indoor' }] }));
    expect(occupied.after.status).toBe('occupied');
    expect(occupied.objects.status).not.toBe('occupied');
  });

  it('an unknown scope is incomplete and empty', () => {
    const source = createPerceptionSource(buildMissionWorld({ seed: 2, targets: [] }));
    expect(source.clearStatus('bldg-99', 'person')).toEqual({ status: 'incomplete', seenEmpty: 0, occupied: 0, unseen: 0, total: 0 });
  });

  it('aerial sweep: finds an outdoor person, covers the woods, and stays within the frame budget', () => {
    const world = buildMissionWorld({ seed: 11, targets: [{ classId: 'person', state: 'lying', where: 'outdoor' }] });
    const mission = missionForWorld(world, { missionType: 'find_person' });
    const source = createPerceptionSource(world);
    const path = planPath(mission);
    const total = pathLength(path);
    const found: Contact[] = [];
    let frames = 0;
    const t0 = performance.now();
    for (let d = 0; d < total; d += 0.6) {
      const at = pointAtDistance(path, d);
      const pose: DronePose = { pos: [at.x, at.y, sampleHeight(world.outdoor.terrain, at.x, at.y) + 12], yaw: at.yawRad, vel: [3 * Math.cos(at.yawRad), 3 * Math.sin(at.yawRad), 0] };
      found.push(...source.observe(pose, d / 3).contacts.filter((contact) => contact.truth.kind === 'tp' && contact.classId === 'person'));
      frames += 1;
    }
    const msPerFrame = (performance.now() - t0) / frames;
    expect(msPerFrame).toBeLessThan(2);
    expect(found.length).toBeGreaterThan(0);
    const truth = source.truthTargets().find((target) => target.classId === 'person')!;
    expect(truth.buildingId).toBeNull();
    const best = found.reduce((a, b) => (b.confidence > a.confidence ? b : a));
    expect(Math.hypot(best.estimate[0] - truth.world[0], best.estimate[1] - truth.world[1])).toBeLessThan(3);
    const clear = source.clearStatus('outdoor', 'person');
    expect(clear.status).toBe('occupied');
    expect(clear.seenEmpty / clear.total).toBeGreaterThan(0.95);
  });

  it('exports a gimbal footprint wider than the 8 m sweep lane at sweep height', () => {
    expect(GIMBAL_RANGE_SCALE).toBeGreaterThan(1.5);
    const fp = gimbalFootprint(12);
    expect(fp.halfAcrossM).toBeGreaterThan(4);
    expect(fp.halfAlongM).toBeGreaterThan(2);
    expect(gimbalFootprint(GIMBAL_MIN_AGL_M - 3).halfAcrossM).toBe(0);
    expect(gimbalFootprint(60).halfAcrossM).toBe(0);
  });
});

describe('perception-backed DetectionSource', () => {
  it('turns the sweep into detections with mission ids, deterministically', () => {
    const world = buildMissionWorld({ seed: 11, targets: [{ classId: 'person', state: 'lying', where: 'outdoor' }] });
    const mission = missionForWorld(world, { missionType: 'find_person' });
    const source = createPerceptionDetectionSource({ world: { targets: [{ classId: 'person', state: 'lying', where: 'outdoor' }] } });
    const snapshot = outdoorWorldProvider.load(11);
    const detections = source.catalog(mission, snapshot);
    const person = detections.find((item) => item.id === 'det-tgt-person');
    expect(person).toBeDefined();
    expect(person!.kind).toBe('person');
    expect(person!.entryRequested).toBe(false);
    const truth = world.outdoorScene.targets[0]!;
    expect(Math.hypot(person!.x - truth.feet[0], person!.y - truth.feet[1])).toBeLessThan(3);
    expect(source.catalog(mission, snapshot)).toEqual(detections);
  });
});

describe('demo scenario worlds', () => {
  /** Frames from every empty cell of the target's room, facing it. */
  function roomLook(world: MissionWorld): { frames: number; tp: number } {
    const building = world.buildings[0]!;
    const target = building.scene.targets[0]!;
    const o = building.shell.origin;
    const floor = building.shell.spec.floors[target.floor]!;
    const room = floor.rooms.find((r) => target.feet[0] >= r.x && target.feet[0] < r.x + r.w && target.feet[1] >= r.z && target.feet[1] < r.z + r.d)!;
    const source = createPerceptionSource(world);
    let frames = 0;
    let tp = 0;
    for (let iy = room.z; iy < room.z + room.d; iy++) {
      for (let ix = room.x; ix < room.x + room.w; ix++) {
        if (floor.cells[iy * floor.w + ix] !== CELL.EMPTY) continue;
        const pos: V3 = [ix + 0.5 + o[0], iy + 0.5 + o[1], o[2] + 1.4];
        const pose = poseLooking(pos, [target.feet[0] + o[0], target.feet[1] + o[1], o[2]]);
        frames += 1;
        tp += hits(source.observe(pose, frames * 0.2).contacts, target.id).length;
      }
    }
    return { frames, tp };
  }

  it('cabin-day: the lying person is seen from inside the room', () => {
    const world = buildMissionWorld({ seed: 20, buildings: 1, targets: [{ classId: 'person', state: 'lying', where: 'indoor' }], timeOfDay: 12 });
    const look = roomLook(world);
    expect(look.tp).toBeGreaterThan(look.frames / 2);
  });

  it('backpack-night: the flashlight finds the backpack with the lights off', () => {
    const world = buildMissionWorld({ seed: 17, buildings: 1, targets: [{ classId: 'backpack', where: 'indoor' }], timeOfDay: 23, flashlightOn: true, interiorLights: 'off' });
    expect(roomLook(world).tp).toBeGreaterThan(0);
  });

  it('compound-clear: only decoys are planted', () => {
    const world = buildMissionWorld({ seed: 37, buildings: 2, targets: [], decoys: 2 });
    const truth = createPerceptionSource(world).truthTargets();
    expect(truth).toHaveLength(2);
    expect(truth.every((target) => target.category === 'decoy')).toBe(true);
  });
});
