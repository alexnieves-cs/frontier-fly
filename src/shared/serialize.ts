import type { BuildingSpec, FloorGrid, FurniturePlan, HazardLayout } from './types';

/** HazardLayout with the smoke masks as plain arrays. */
type HazardJson = Omit<HazardLayout, 'smoke'> & { smoke: { visibility: number; cells: (number[] | Uint8Array)[] } | null };

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
  furniture?: FurniturePlan;
  hazards?: HazardJson;
}

export function buildingFromJson(raw: BuildingJson): BuildingSpec {
  return {
    seed: raw.seed,
    cellSize: raw.cellSize,
    floorHeight: raw.floorHeight,
    spawn: [raw.spawn[0] ?? 0, raw.spawn[1] ?? 0, raw.spawn[2] ?? 0],
    reachableVoxels: raw.reachableVoxels,
    ...(raw.furniture
      ? {
          furniture: {
            props: raw.furniture.props.map((prop) => ({ ...prop })),
            boxes: raw.furniture.boxes.map((box) => ({ ...box })),
          },
        }
      : {}),
    ...(raw.hazards
      ? {
          hazards: {
            ...raw.hazards,
            params: { ...raw.hazards.params },
            closedDoors: raw.hazards.closedDoors.map((door) => ({ floor: door.floor, cells: [...door.cells] })),
            smoke: raw.hazards.smoke
              ? {
                  visibility: raw.hazards.smoke.visibility,
                  cells: raw.hazards.smoke.cells.map((c) => (c instanceof Uint8Array ? c : Uint8Array.from(c))),
                }
              : null,
          },
        }
      : {}),
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
    ...(spec.furniture
      ? {
          furniture: {
            props: spec.furniture.props.map((prop) => ({ ...prop })),
            boxes: spec.furniture.boxes.map((box) => ({ ...box })),
          },
        }
      : {}),
    ...(spec.hazards
      ? {
          hazards: {
            ...spec.hazards,
            params: { ...spec.hazards.params },
            closedDoors: spec.hazards.closedDoors.map((door) => ({ floor: door.floor, cells: [...door.cells] })),
            smoke: spec.hazards.smoke ? { visibility: spec.hazards.smoke.visibility, cells: spec.hazards.smoke.cells.map((c) => Array.from(c)) } : null,
          },
        }
      : {}),
    floors: spec.floors.map((floor) => ({
      w: floor.w,
      d: floor.d,
      rooms: floor.rooms,
      cells: Array.from(floor.cells),
      surfaces: floor.surfaces ? Array.from(floor.surfaces) : undefined,
    })),
  };
}
