import { generateOutdoor } from '../../world/outdoor/generate';
import { placeIndoor } from '../../targets/placement';
import type { TargetScene } from '../../targets/types';
import { ringInsideRing } from '../polygon';
import { suggestedTargets } from '../sample';
import type { Mission } from '../types';
import type { MissionBuilding, MissionForWorldOptions, MissionWorld, MissionWorldOptions } from './contracts';

// STARTER: minimal builder so other workstreams can run. The world workstream replaces it.

function roomTotal(shell: MissionBuilding['shell']): number {
  return shell.spec.floors.reduce((sum, floor) => sum + floor.rooms.length, 0);
}

export function buildMissionWorld(options: MissionWorldOptions): MissionWorld {
  const outdoor = generateOutdoor({ seed: options.seed, buildings: options.buildings });
  const buildings: MissionBuilding[] = outdoor.buildings.map((shell, index) => {
    const floor = shell.spec.floors[0]!;
    const w = floor.w * shell.spec.cellSize;
    const d = floor.d * shell.spec.cellSize;
    const [ox, oy] = shell.origin;
    const people = (options.targets ?? []).filter((t) => t.where === 'indoor' && (t.building ?? 0) === index && t.classId === 'person').length;
    const objects = (options.targets ?? []).filter((t) => t.where === 'indoor' && (t.building ?? 0) === index && t.classId !== 'person').length;
    const scene = placeIndoor(shell.spec, { seed: (options.seed + 31 * index) >>> 0, people, objects, decoys: 0 });
    return {
      id: `bldg-${shell.id}`,
      label: index === 0 ? 'Cabin' : `Building ${index + 1}`,
      shell,
      footprint: [
        { x: ox, y: oy },
        { x: ox + w, y: oy },
        { x: ox + w, y: oy + d },
        { x: ox, y: oy + d },
      ],
      heightM: shell.spec.floors.length * shell.spec.floorHeight,
      scene,
      interiorLightsOn: new Uint8Array(roomTotal(shell)).fill(options.interiorLights === 'off' ? 0 : 1),
    };
  });
  const outdoorScene: TargetScene = { seed: options.seed >>> 0, space: 'outdoor', targets: [] };
  return {
    seed: options.seed >>> 0,
    outdoor,
    buildings,
    outdoorScene,
    timeOfDay: options.timeOfDay ?? 12,
    flashlightOn: options.flashlightOn ?? false,
    sensorMode: options.sensorMode ?? 'rgb',
    home: [outdoor.spawn[0], outdoor.spawn[1], outdoor.spawn[2]],
    homeYaw: outdoor.spawnYaw,
  };
}

export function missionForWorld(world: MissionWorld, options: MissionForWorldOptions): Mission {
  return {
    schemaVersion: 1,
    id: `msn-${world.seed}`,
    name: options.name ?? 'Woods search',
    seed: world.seed,
    notes: 'Civilian search and rescue practice. Everything is simulated.',
    geofence: { points: world.outdoor.geofence.polygon.map(([x, y]) => ({ x, y })) },
    noFlyZones: world.outdoor.noFly
      .map((zone) => ({
        id: `nf-${zone.id}`,
        name: zone.name,
        polygon: { points: zone.polygon.map(([x, y]) => ({ x, y })) },
        reason: 'No-fly zone from the site plan.',
      }))
      .filter((zone) => ringInsideRing(zone.polygon.points, world.outdoor.geofence.polygon.map(([x, y]) => ({ x, y })))),
    altitude: { minM: 3, maxM: 30 },
    missionType: options.missionType,
    targets: options.targets ?? suggestedTargets(options.missionType),
    requiredConfidence: options.requiredConfidence ?? 0.8,
    entryPolicy: options.entryPolicy ?? 'auto',
    budgets: options.budgets ?? { batteryPct: 80, timeSec: 900 },
    startPose: { x: world.home[0], y: world.home[1], z: 3, yawRad: world.homeYaw },
  };
}
