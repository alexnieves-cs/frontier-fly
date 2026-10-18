import type { BuildingSpec, FloorGrid } from './types';

interface FloorJson {
  w: number;
  d: number;
  cells: number[] | Uint8Array;
  rooms: FloorGrid['rooms'];
  surfaces?: number[] | Uint8Array;
}

interface BuildingJson {
  seed: number;
  cellSize: number;
  floorHeight: number;
  floors: FloorJson[];
  spawn: [number, number, number] | number[];
  reachableVoxels: number;
}

export function buildingFromJson(raw: BuildingJson): BuildingSpec {
  return {
    seed: raw.seed,
    cellSize: raw.cellSize,
    floorHeight: raw.floorHeight,
    spawn: [raw.spawn[0] ?? 0, raw.spawn[1] ?? 0, raw.spawn[2] ?? 0],
    reachableVoxels: raw.reachableVoxels,
    floors: raw.floors.map((floor) => ({
      w: floor.w,
      d: floor.d,
      rooms: floor.rooms,
      cells: floor.cells instanceof Uint8Array ? floor.cells : Uint8Array.from(floor.cells),
      surfaces:
        floor.surfaces == null
          ? undefined
          : floor.surfaces instanceof Uint8Array
            ? floor.surfaces
            : Uint8Array.from(floor.surfaces),
    })),
  };
}

export function buildingToJson(spec: BuildingSpec): BuildingJson {
  return {
    seed: spec.seed,
    cellSize: spec.cellSize,
    floorHeight: spec.floorHeight,
    spawn: spec.spawn,
    reachableVoxels: spec.reachableVoxels,
    floors: spec.floors.map((floor) => ({
      w: floor.w,
      d: floor.d,
      rooms: floor.rooms,
      cells: Array.from(floor.cells),
      surfaces: floor.surfaces ? Array.from(floor.surfaces) : undefined,
    })),
  };
}
