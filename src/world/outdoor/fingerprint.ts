import type { OutdoorWorld } from './types';

/** Stable digest of the generated world. Two calls with one seed must match. */
export function fingerprint(world: OutdoorWorld): string {
  let hash = world.seed >>> 0;
  const mix = (value: number) => {
    hash = Math.imul(hash ^ Math.round(value * 1000), 16777619) >>> 0;
  };
  mix(world.buildings.length);
  mix(world.trees.length);
  mix(world.colliders.length);
  mix(world.spawn[0]);
  mix(world.spawn[1]);
  mix(world.spawn[2]);
  mix(world.spawnYaw);
  for (let i = 0; i < world.terrain.heights.length; i += 7) mix(world.terrain.heights[i]!);
  for (const tree of world.trees) {
    mix(tree.x);
    mix(tree.y);
    mix(tree.z);
    mix(tree.trunkHeight);
    mix(tree.canopyRadius);
    hash = Math.imul(hash ^ tree.kind.length, 16777619) >>> 0;
    mix(tree.lod);
  }
  for (const shell of world.buildings) {
    mix(shell.origin[0]);
    mix(shell.origin[1]);
    mix(shell.origin[2]);
    mix(shell.spec.seed);
    for (const floor of shell.spec.floors) {
      for (let i = 0; i < floor.cells.length; i++) hash = Math.imul(hash ^ floor.cells[i]!, 16777619) >>> 0;
    }
    for (const opening of shell.openings) {
      hash = Math.imul(hash ^ opening.kind.length ^ (opening.passable ? 17 : 3), 16777619) >>> 0;
      mix(opening.width);
      mix(opening.height);
      mix(opening.angle);
      mix(opening.clearWidth);
      mix(opening.center[0]);
      mix(opening.center[1]);
      mix(opening.center[2]);
    }
  }
  for (const vertex of world.geofence.polygon) {
    mix(vertex[0]);
    mix(vertex[1]);
  }
  for (const zone of world.noFly) {
    mix(zone.minZ);
    mix(zone.maxZ);
    for (const vertex of zone.polygon) {
      mix(vertex[0]);
      mix(vertex[1]);
    }
  }
  return hash.toString(16).padStart(8, '0');
}
