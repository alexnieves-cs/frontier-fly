import type { BuildingSpec } from '../shared/types';
import type { Aabb, OutdoorArea, ResolvedTarget, Vec3 } from '../targets/types';
import { inFront, projectAabb, type CameraView } from './camera';
import { FLASH_HALF, RESOLVE_DISTANCE, SEEN_LIGHT_MIN, SEEN_OCCLUSION_MAX, illuminationOf } from './model';
import { pointOcclusion } from './occlusion';
import type { DetectRequest } from './detector';
import { clamp, dist, dot, norm, sub } from './vec';

export interface GridCell {
  key: string;
  center: Vec3;
  floor: number;
  roomIndex: number;
}

export interface ClearReport {
  status: 'clear' | 'occupied' | 'incomplete';
  seenEmpty: number;
  occupied: number;
  unseen: number;
  total: number;
}

export type ClearCategory = 'person' | 'object';

function angleTo(cam: CameraView, point: Vec3): number {
  return Math.acos(clamp(dot(cam.forward, norm(sub(point, cam.position))), -1, 1));
}

/** Reachable empty cells. The sample point is at chest height inside the column. */
export function indoorCells(spec: BuildingSpec): GridCell[] {
  const cells: GridCell[] = [];
  let roomIndex = 0;
  for (let f = 0; f < spec.floors.length; f++) {
    const floor = spec.floors[f]!;
    const roomOf = new Int16Array(floor.w * floor.d).fill(-1);
    for (const room of floor.rooms) {
      for (let iy = room.z; iy < room.z + room.d; iy++) {
        for (let ix = room.x; ix < room.x + room.w; ix++) roomOf[iy * floor.w + ix] = roomIndex;
      }
      roomIndex += 1;
    }
    for (let iy = 0; iy < floor.d; iy++) {
      for (let ix = 0; ix < floor.w; ix++) {
        const i = iy * floor.w + ix;
        if (floor.cells[i] !== 0) continue;
        const rid = roomOf[i] ?? -1;
        if (rid < 0) continue;
        cells.push({
          key: `${f}:${ix}:${iy}`,
          center: [(ix + 0.5) * spec.cellSize, (iy + 0.5) * spec.cellSize, f * spec.floorHeight + 1.15],
          floor: f,
          roomIndex: rid,
        });
      }
    }
  }
  return cells;
}

export function outdoorCells(area: OutdoorArea, step = 1): GridCell[] {
  const cells: GridCell[] = [];
  for (let y = area.minY; y < area.maxY; y += step) {
    for (let x = area.minX; x < area.maxX; x += step) {
      const ix = Math.floor(x / step);
      const iy = Math.floor(y / step);
      const blocked = (area.obstacles ?? []).some((box) => x >= box.minX && x < box.maxX && y >= box.minY && y < box.maxY);
      if (blocked) continue;
      cells.push({
        key: `o:${ix}:${iy}`,
        center: [x + step * 0.5, y + step * 0.5, area.groundZ + 1.15],
        floor: 0,
        roomIndex: -1,
      });
    }
  }
  return cells;
}

export function roomCellKeys(spec: BuildingSpec, floor: number, roomId: number): string[] {
  return indoorCells(spec)
    .filter((cell) => cell.floor === floor && cell.roomIndex === roomId)
    .map((cell) => cell.key);
}

function cellBox(cell: GridCell, spec: BuildingSpec | null): Aabb {
  const cs = spec?.cellSize ?? 1;
  const ix = Number(cell.key.split(':')[1]);
  const iy = Number(cell.key.split(':')[2]);
  const z0 = spec ? cell.floor * spec.floorHeight : cell.center[2] - 1.15;
  if (cell.key.startsWith('o:')) {
    return {
      min: [cell.center[0] - 0.5, cell.center[1] - 0.5, z0],
      max: [cell.center[0] + 0.5, cell.center[1] + 0.5, z0 + 2],
    };
  }
  return {
    min: [ix * cs, iy * cs, z0],
    max: [(ix + 1) * cs, (iy + 1) * cs, z0 + (spec?.floorHeight ?? 3)],
  };
}

function overlaps(a: Aabb, b: Aabb): boolean {
  return a.min[0] < b.max[0] && a.max[0] > b.min[0] && a.min[1] < b.max[1] && a.max[1] > b.min[1] && a.min[2] < b.max[2] && a.max[2] > b.min[2];
}

function occupied(cell: GridCell, targets: ResolvedTarget[], category: ClearCategory, spec: BuildingSpec | null): boolean {
  const box = cellBox(cell, spec);
  return targets.some((target) => {
    if (category === 'person' && target.classId !== 'person') return false;
    if (category === 'object' && target.category !== 'object') return false;
    return overlaps(box, target.aabb);
  });
}

/**
 * Ground-truth seen-empty map for confirm-clear.
 * A cell is seen-empty only after the camera resolved it and no target of
 * the chosen category occupied it. Detector misses do not clear a cell,
 * and a person walking into a cleared cell makes confirm-clear fail.
 */
export class SeenEmptyTracker {
  private readonly empty = new Set<string>();

  constructor(
    private readonly cells: GridCell[],
    private readonly category: ClearCategory,
    private readonly spec: BuildingSpec | null = null,
  ) {}

  /** `rangeScale` stretches the resolve distance for a longer lens. See DetectRequest.rangeScale. */
  observe(camera: CameraView, targets: ResolvedTarget[], lighting: DetectRequest['lighting'], occluders: Aabb[] = [], rangeScale = 1): void {
    const base = this.category === 'person' ? RESOLVE_DISTANCE.person : RESOLVE_DISTANCE.object;
    const reach = base * (rangeScale > 0 ? rangeScale : 1);
    for (const cell of this.cells) {
      if (!inFront(camera, cell.center)) continue;
      const distance = dist(camera.position, cell.center);
      if (distance > reach) continue;
      const projected = projectAabb(camera, {
        min: [cell.center[0] - 0.15, cell.center[1] - 0.15, cell.center[2] - 0.2],
        max: [cell.center[0] + 0.15, cell.center[1] + 0.15, cell.center[2] + 0.2],
      });
      if (!projected) continue;
      const occ = pointOcclusion(this.spec, camera.position, cell.center, occluders);
      if (occ > SEEN_OCCLUSION_MAX) continue;
      const interiorOn = (lighting.interiorLightsOn[cell.roomIndex] ?? 0) > 0;
      const inFlashlight = lighting.flashlightOn && angleTo(camera, cell.center) <= FLASH_HALF;
      const light = illuminationOf({
        timeOfDay: lighting.timeOfDay,
        interiorOn,
        flashlightOn: lighting.flashlightOn,
        inFlashlight,
        sensorMode: lighting.sensorMode,
      });
      if (light < SEEN_LIGHT_MIN) continue;
      if (occupied(cell, targets, this.category, this.spec)) this.empty.delete(cell.key);
      else this.empty.add(cell.key);
    }
  }

  seenEmptyKeys(): string[] {
    return [...this.empty].sort();
  }

  report(keys: string[], targets: ResolvedTarget[]): ClearReport {
    let seenEmpty = 0;
    let occupiedCount = 0;
    let unseen = 0;
    const byKey = new Map(this.cells.map((cell) => [cell.key, cell]));
    for (const key of keys) {
      const cell = byKey.get(key);
      const isOccupied = cell ? occupied(cell, targets, this.category, this.spec) : false;
      if (isOccupied) occupiedCount += 1;
      else if (this.empty.has(key)) seenEmpty += 1;
      else unseen += 1;
    }
    const status = occupiedCount > 0 ? 'occupied' : unseen > 0 ? 'incomplete' : 'clear';
    return { status, seenEmpty, occupied: occupiedCount, unseen, total: keys.length };
  }
}

export function clearLabel(report: ClearReport): string {
  if (report.status === 'clear') return `confirm-clear ${report.seenEmpty}/${report.total}`;
  if (report.status === 'occupied') return `occupied ${report.occupied} · seen-empty ${report.seenEmpty}/${report.total}`;
  return `incomplete · seen-empty ${report.seenEmpty}/${report.total}`;
}
