import * as THREE from 'three';
import type { BuildingSpec, PropKind, PropPlacement } from '../shared/types';
import type { MaterialSet } from './three/materials';

export type { PropKind, PropPlacement };

/**
 * Furniture placements for the renderer. The layout is planned headlessly by
 * planFurniture (src/gen/furniture.ts) and stored on the spec by
 * generateBuilding, so these meshes sit exactly on the physics colliders. A
 * spec without a plan (hand-written fixtures, older JSON) has no furniture in
 * physics either, so it renders none.
 */
export function planProps(spec: BuildingSpec): PropPlacement[] {
  return spec.furniture?.props ?? [];
}

function add(parent: THREE.Object3D, geo: THREE.BufferGeometry, mat: THREE.Material, x: number, y: number, z: number): THREE.Mesh {
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(x, y, z);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  parent.add(mesh);
  return mesh;
}

/**
 * Original furniture meshes. Surfaces reuse the room PBR materials. Part sizes
 * must stay inside the collision parts in PROP_PARTS (src/gen/furniture.ts).
 */
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
