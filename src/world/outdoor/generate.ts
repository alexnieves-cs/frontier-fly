import { generateBuilding } from '../../gen/building';
import { CELL_SIZE } from '../../shared/constants';
import { RNG } from '../../shared/rng';
import type { BuildingSpec } from '../../shared/types';
import { planColliders } from './colliders';
import { MAX_BUILDINGS, WORLD_SIZE } from './constants';
import { scatterForest, shellAabb } from './forest';
import { containBlocks, makeGeofence, makeNoFlyZones, type Block } from './geofence';
import { clamp } from './math';
import { furnishShell } from './openings';
import { applyClearings, createTerrain, dipFootprint, sampleHeight } from './terrain';
import type { BuildingShell, Clearing, Corridor, OutdoorParams, OutdoorWorld } from './types';
import { validateOutdoor } from './validate';

/**
 * Seeded outdoor mission: a geofenced forest, clearings, and building shells
 * whose interiors come from the indoor BSP generator. The drone starts outside
 * and can enter a shell only through an opening that clears the airframe.
 */
export function generateOutdoor(params: OutdoorParams): OutdoorWorld {
  let seed = params.seed >>> 0;
  let last: string[] = [];
  for (let attempt = 0; attempt < 10; attempt++) {
    const world = buildWorld(seed, params);
    if (world) {
      const errors = validateOutdoor(world);
      if (errors.length === 0) return world;
      last = errors;
    }
    seed = (seed + 1) >>> 0;
  }
  throw new Error(`generateOutdoor failed for seed ${params.seed}: ${last.join('; ')}`);
}

function buildWorld(seed: number, params: OutdoorParams): OutdoorWorld | null {
  const rng = new RNG(seed, 'outdoor');
  const size = WORLD_SIZE;
  const terrain = createTerrain(seed, size, undefined);
  const cx = size / 2;
  const cy = size / 2;
  const count = clamp(params.buildings ?? 1 + (seed % 3), 1, MAX_BUILDINGS);
  const placed: BuildingShell[] = [];
  for (let i = 0; i < count; i++) {
    const floors = i === 1 ? 2 : 1;
    const w = 10 + ((seed + i * 3) % 3);
    const d = 10 + ((seed + i * 5) % 3);
    const spec = makeInterior((seed + 1000 + i * 97) >>> 0, floors, w, d);
    const shell = placeShell(i, spec, placed, cx, cy, size, rng);
    if (!shell) return null;
    placed.push(shell);
  }

  const clearings: Clearing[] = [
    {
      x: cx,
      y: cy,
      inner: 7,
      outer: 12,
      height: sampleHeight(terrain, cx, cy),
    },
  ];
  for (const shell of placed) {
    const floor = shell.spec.floors[0]!;
    const half = Math.hypot(floor.w, floor.d) * shell.spec.cellSize * 0.5;
    const px = shell.origin[0] + (floor.w * shell.spec.cellSize) / 2;
    const py = shell.origin[1] + (floor.d * shell.spec.cellSize) / 2;
    clearings.push({
      x: px,
      y: py,
      inner: half + 1.4,
      outer: half + 7,
      height: sampleHeight(terrain, px, py),
    });
  }
  applyClearings(terrain, clearings);
  for (let i = 0; i < placed.length; i++) {
    const shell = placed[i]!;
    const pad = clearings[i + 1]!;
    shell.origin[2] = pad.height;
    const floor = shell.spec.floors[0]!;
    dipFootprint(
      terrain,
      shell.origin[0],
      shell.origin[1],
      shell.origin[0] + floor.w * CELL_SIZE,
      shell.origin[1] + floor.d * CELL_SIZE,
      pad.height,
    );
  }

  const ids = { next: 0 };
  for (const shell of placed) {
    if (!furnishShell(shell, cx, cy, rng, ids, params.entrances ?? 1)) return null;
  }

  const blocks: Block[] = placed.map((shell) => {
    const box = shellAabb(shell, 0);
    return { minX: box.minX, minY: box.minY, maxX: box.maxX, maxY: box.maxY };
  });
  const geofence = makeGeofence(cx, cy, size * 0.4, rng);
  containBlocks(geofence, blocks, cx, cy);
  const corridors: Corridor[] = placed.map((shell) => ({
    ax: cx,
    ay: cy,
    bx: shell.approach[0],
    by: shell.approach[1],
    halfWidth: 3.4,
  }));
  // A second way in gets its own short clear lane out from the wall, so trees and no-fly zones keep
  // off its approach the way the trail keeps off the first. Not a trail: world.corridors is unchanged.
  const lanes: Corridor[] = (params.entrances ?? 1) >= 2 ? placed.flatMap((shell) => sideLanes(shell)) : [];
  const keepOut = lanes.length ? [...corridors, ...lanes] : corridors;
  const noFly = makeNoFlyZones(cx, cy, blocks, keepOut, rng);
  const spawnZ = sampleHeight(terrain, cx, cy) + 1.7;
  const first = placed[0]!;
  const spawnYaw = Math.atan2(first.approach[1] - cy, first.approach[0] - cx);
  const trees = scatterForest(terrain, placed, keepOut, geofence, cx, cy, rng);
  const world: OutdoorWorld = {
    seed,
    size,
    terrain,
    clearings,
    corridors,
    trees,
    buildings: placed,
    geofence,
    noFly,
    spawn: [cx, cy, spawnZ],
    spawnYaw,
    colliders: [],
  };
  world.colliders = planColliders(world);
  return world;
}

/** Clear lanes out from every passable wall opening but the first (the trail serves that one). */
function sideLanes(shell: BuildingShell): Corridor[] {
  // The first passable opening may be a roof hatch, so skip it before keeping wall openings only.
  const doors = shell.openings.filter((opening) => opening.passable).slice(1).filter((opening) => opening.normal[2] === 0);
  return doors.map((opening) => ({
    ax: opening.center[0] + opening.normal[0] * 0.5,
    ay: opening.center[1] + opening.normal[1] * 0.5,
    bx: opening.center[0] + opening.normal[0] * SIDE_LANE_M,
    by: opening.center[1] + opening.normal[1] * SIDE_LANE_M,
    halfWidth: 1.8,
  }));
}

/** Length of the clear lane out from a second way in, m. */
const SIDE_LANE_M = 7;

function makeInterior(seed: number, floors: number, w: number, d: number): BuildingSpec {
  if (floors > 1) {
    try {
      return generateBuilding({ seed, floors, w, d, stage: 4 });
    } catch {
      return generateBuilding({ seed, floors: 1, w, d, stage: 2 });
    }
  }
  return generateBuilding({ seed, floors: 1, w, d, stage: 2 });
}

function placeShell(
  id: number,
  spec: BuildingSpec,
  existing: readonly BuildingShell[],
  cx: number,
  cy: number,
  size: number,
  rng: RNG,
): BuildingShell | null {
  const floor = spec.floors[0]!;
  const footprintW = floor.w * spec.cellSize;
  const footprintD = floor.d * spec.cellSize;
  const count = Math.max(1, existing.length + 1);
  for (let attempt = 0; attempt < 12; attempt++) {
    const angle = (id / count) * Math.PI * 2 + attempt * 0.55 + rng.range(-0.2, 0.2);
    const dist = (count === 1 ? 16 : 18) + rng.range(-1, 1.5);
    const ox = cx + Math.cos(angle) * dist - footprintW / 2;
    const oy = cy + Math.sin(angle) * dist - footprintD / 2;
    if (ox < 4 || oy < 4 || ox + footprintW > size - 4 || oy + footprintD > size - 4) continue;
    const shell: BuildingShell = {
      id,
      spec,
      origin: [ox, oy, 0],
      openings: [],
      approach: [ox + footprintW / 2, oy, 1],
    };
    const box = shellAabb(shell, 2.5);
    const hit = existing.some((other) => {
      const prev = shellAabb(other, 2.5);
      return box.minX < prev.maxX && box.maxX > prev.minX && box.minY < prev.maxY && box.maxY > prev.minY;
    });
    if (!hit) return shell;
  }
  return null;
}
