import * as THREE from 'three';
import { CELL } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec } from '../shared/types';
import type { MaterialSet } from './three/materials';

export type PropKind = 'table' | 'chair' | 'shelf' | 'plant' | 'sofa';

export interface PropPlacement {
  kind: PropKind;
  /** Sim-space meters. y is up. */
  x: number;
  y: number;
  z: number;
  rot: number;
  floor: number;
}

function cellBlocked(spec: BuildingSpec, floor: number, x: number, z: number): boolean {
  const grid = spec.floors[floor];
  if (!grid) return true;
  const gx = Math.floor(x / spec.cellSize);
  const gz = Math.floor(z / spec.cellSize);
  if (gx < 0 || gz < 0 || gx >= grid.w || gz >= grid.d) return true;
  const cell = grid.cells[gz * grid.w + gx] ?? CELL.WALL;
  if (cell !== CELL.EMPTY) return true;
  const sx = Math.floor(spec.spawn[0] / spec.cellSize);
  const sz = Math.floor(spec.spawn[1] / spec.cellSize);
  const sf = Math.floor(spec.spawn[2] / spec.floorHeight);
  return floor === sf && gx === sx && gz === sz;
}

/** Seeded furniture layout. Props stay inside rooms and off doors, shafts, and the spawn cell. */
export function planProps(spec: BuildingSpec): PropPlacement[] {
  const rng = new RNG(spec.seed, 'props');
  const props: PropPlacement[] = [];
  spec.floors.forEach((floor, f) => {
    for (const room of floor.rooms) {
      if (room.w < 3 || room.d < 3) continue;
      const y = f * spec.floorHeight;
      const cx = (room.x + room.w / 2) * spec.cellSize;
      const cz = (room.z + room.d / 2) * spec.cellSize;
      const candidates: PropPlacement[] = [
        { kind: 'table', x: cx, y, z: cz, rot: rng.range(-0.2, 0.2), floor: f },
        { kind: 'chair', x: cx + 0.62, y, z: cz + 0.05, rot: Math.PI / 2, floor: f },
        { kind: 'chair', x: cx - 0.15, y, z: cz + 0.62, rot: Math.PI, floor: f },
        {
          kind: 'shelf',
          x: (room.x + 0.55) * spec.cellSize,
          y,
          z: (room.z + 1.1) * spec.cellSize,
          rot: 0,
          floor: f,
        },
      ];
      if (room.w >= 4 && room.d >= 4) {
        candidates.push({
          kind: rng.bool(0.5) ? 'sofa' : 'plant',
          x: (room.x + room.w - 0.8) * spec.cellSize,
          y,
          z: (room.z + room.d - 0.8) * spec.cellSize,
          rot: rng.range(0, Math.PI),
          floor: f,
        });
      }
      for (const prop of candidates) {
        if (cellBlocked(spec, f, prop.x, prop.z)) continue;
        props.push(prop);
      }
    }
  });
  return props;
}

function add(parent: THREE.Object3D, geo: THREE.BufferGeometry, mat: THREE.Material, x: number, y: number, z: number): THREE.Mesh {
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(x, y, z);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  parent.add(mesh);
  return mesh;
}

/** Original furniture meshes. Surfaces reuse the room PBR materials. */
export function buildProps(spec: BuildingSpec, mats: MaterialSet): THREE.Group {
  const root = new THREE.Group();
  root.name = 'props';
  const wood = mats.door;
  const cloth = mats.wall;
  const leaf = new THREE.MeshStandardMaterial({ color: '#3f6b45', roughness: 0.7, metalness: 0 });
  const pot = mats.concrete;
  for (const prop of planProps(spec)) {
    const group = new THREE.Group();
    group.position.set(prop.x, prop.y, prop.z);
    group.rotation.y = prop.rot;
    if (prop.kind === 'table') {
      add(group, new THREE.BoxGeometry(1.1, 0.04, 0.62), wood, 0, 0.74, 0);
      add(group, new THREE.BoxGeometry(0.06, 0.72, 0.06), wood, -0.48, 0.36, -0.24);
      add(group, new THREE.BoxGeometry(0.06, 0.72, 0.06), wood, 0.48, 0.36, -0.24);
      add(group, new THREE.BoxGeometry(0.06, 0.72, 0.06), wood, -0.48, 0.36, 0.24);
      add(group, new THREE.BoxGeometry(0.06, 0.72, 0.06), wood, 0.48, 0.36, 0.24);
    } else if (prop.kind === 'chair') {
      add(group, new THREE.BoxGeometry(0.42, 0.04, 0.42), wood, 0, 0.46, 0);
      add(group, new THREE.BoxGeometry(0.42, 0.46, 0.04), cloth, 0, 0.7, -0.19);
      add(group, new THREE.BoxGeometry(0.04, 0.46, 0.04), wood, -0.18, 0.23, -0.18);
      add(group, new THREE.BoxGeometry(0.04, 0.46, 0.04), wood, 0.18, 0.23, -0.18);
      add(group, new THREE.BoxGeometry(0.04, 0.46, 0.04), wood, -0.18, 0.23, 0.18);
      add(group, new THREE.BoxGeometry(0.04, 0.46, 0.04), wood, 0.18, 0.23, 0.18);
    } else if (prop.kind === 'shelf') {
      add(group, new THREE.BoxGeometry(0.7, 1.3, 0.28), wood, 0, 0.7, 0);
      add(group, new THREE.BoxGeometry(0.64, 0.02, 0.24), mats.floor, 0, 0.45, 0);
      add(group, new THREE.BoxGeometry(0.64, 0.02, 0.24), mats.floor, 0, 0.9, 0);
    } else if (prop.kind === 'sofa') {
      add(group, new THREE.BoxGeometry(1.3, 0.32, 0.55), cloth, 0, 0.32, 0);
      add(group, new THREE.BoxGeometry(1.3, 0.4, 0.1), cloth, 0, 0.62, -0.22);
      add(group, new THREE.BoxGeometry(0.1, 0.28, 0.55), cloth, -0.6, 0.48, 0);
      add(group, new THREE.BoxGeometry(0.1, 0.28, 0.55), cloth, 0.6, 0.48, 0);
    } else {
      add(group, new THREE.CylinderGeometry(0.12, 0.1, 0.18, 12), pot, 0, 0.1, 0);
      add(group, new THREE.SphereGeometry(0.2, 12, 8), leaf, 0, 0.36, 0);
    }
    root.add(group);
  }
  return root;
}
