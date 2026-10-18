import * as THREE from 'three';
import { CELL, SURFACE } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';
import { buildingAabbs } from '../gen/parts';
import { materialFor, type MaterialSet } from './three/materials';

export interface BuiltLevel {
  root: THREE.Group;
  floors: THREE.Group[];
  ceilings: THREE.Mesh[];
  lights: { light: THREE.PointLight; roomIndex: number }[];
}

function addBox(
  parent: THREE.Object3D,
  cx: number,
  cy: number,
  cz: number,
  hx: number,
  hy: number,
  hz: number,
  material: THREE.Material,
  shadow: boolean,
): THREE.Mesh {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(hx * 2, hz * 2, hy * 2), material);
  mesh.position.set(cx, cz, cy);
  mesh.castShadow = shadow;
  mesh.receiveShadow = true;
  parent.add(mesh);
  return mesh;
}

/** Sim x/y/z-up becomes Three x / y-up / z. */
export function buildBuilding(spec: BuildingSpec, mats: MaterialSet, shadows: boolean): BuiltLevel {
  const root = new THREE.Group();
  const floors: THREE.Group[] = [];
  const ceilings: THREE.Mesh[] = [];
  const lights: BuiltLevel['lights'] = [];
  const boxes = buildingAabbs(spec);

  spec.floors.forEach((floor, f) => {
    const group = new THREE.Group();
    group.name = `floor-${f}`;
    const H = spec.floorHeight;
    for (const box of boxes) {
      const onFloor = box.kind === 'slab' ? Math.abs(box.cz - f * H) < 0.2 || (f === 0 && box.cz < 0.2) : Math.abs(box.cz - (f * H + H / 2)) < 0.2;
      if (!onFloor) continue;
      if (box.kind === 'slab' && box.cz > f * H + 0.2) continue;
      const mat = box.kind === 'window' ? mats.glass : materialFor(mats, box.surface, box.kind);
      addBox(group, box.cx, box.cy, box.kind === 'slab' && f === 0 ? 0.02 : box.cz, box.hx, box.hy, box.kind === 'slab' ? 0.04 : box.hz, mat, shadows && box.kind !== 'window');
    }
    for (let z = 1; z < floor.d - 1; z++) {
      for (let x = 1; x < floor.w - 1; x++) {
        const cell = floor.cells[z * floor.w + x]!;
        if (cell !== CELL.DOOR) continue;
        const cx = (x + 0.5) * spec.cellSize;
        const cy = (z + 0.5) * spec.cellSize;
        const base = f * H;
        addBox(group, cx, cy, base + H - 0.08, 0.48, 0.48, 0.06, mats.frame, shadows);
        addBox(group, cx, cy, base + 0.08, 0.48, 0.48, 0.06, mats.frame, shadows);
      }
    }
    const ceil = new THREE.Mesh(
      new THREE.BoxGeometry(floor.w * spec.cellSize, 0.06, floor.d * spec.cellSize),
      mats.ceiling,
    );
    ceil.position.set((floor.w * spec.cellSize) / 2, (f + 1) * H - 0.03, (floor.d * spec.cellSize) / 2);
    ceil.receiveShadow = true;
    group.add(ceil);
    ceilings.push(ceil);

    for (const room of floor.rooms) {
      const cx = (room.x + room.w / 2) * spec.cellSize;
      const cy = (room.z + room.d / 2) * spec.cellSize;
      const light = new THREE.PointLight('#ffe2c0', 2.4, Math.max(room.w, room.d) * 1.8, 1.4);
      light.position.set(cx, f * H + H - 0.28, cy);
      light.castShadow = false;
      group.add(light);
      lights.push({ light, roomIndex: lights.length });
    }
    floors.push(group);
    root.add(group);
  });

  const top = boxes.find((box) => box.kind === 'slab' && box.surface === SURFACE.CONCRETE && box.cz > spec.floors.length * spec.floorHeight);
  if (top) {
    addBox(root, top.cx, top.cy, top.cz, top.hx, top.hy, top.hz, mats.concrete, false);
  }
  return { root, floors, ceilings, lights };
}
