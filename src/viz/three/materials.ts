import * as THREE from 'three';
import { SURFACE } from '../../shared/constants';
import type { QualityPreset } from '../../shared/types';
import { makeTextures, type SurfaceTextures } from '../textures';

export interface MaterialSet {
  textures: SurfaceTextures;
  wall: THREE.MeshStandardMaterial;
  concrete: THREE.MeshStandardMaterial;
  floor: THREE.MeshStandardMaterial;
  glass: THREE.MeshPhysicalMaterial;
  metal: THREE.MeshStandardMaterial;
  frame: THREE.MeshStandardMaterial;
  ceiling: THREE.MeshStandardMaterial;
  depth: THREE.MeshDepthMaterial;
}

export function makeMaterials(seed: number, quality: QualityPreset): MaterialSet {
  const textures = makeTextures(seed);
  const wall = new THREE.MeshStandardMaterial({
    map: textures.wall,
    roughness: 0.86,
    metalness: 0.02,
    roughnessMap: quality === 'high' ? textures.wallRough : null,
  });
  const concrete = new THREE.MeshStandardMaterial({
    map: textures.concrete,
    roughness: 0.95,
    metalness: 0.04,
    color: '#b7bcc2',
  });
  const floor = new THREE.MeshStandardMaterial({
    map: textures.floor,
    roughness: 0.72,
    metalness: 0.03,
  });
  const glass = new THREE.MeshPhysicalMaterial({
    color: '#c9e7ff',
    roughness: 0.06,
    metalness: 0,
    transmission: quality === 'high' ? 0.86 : 0,
    transparent: true,
    opacity: quality === 'high' ? 1 : 0.28,
    thickness: 0.05,
    envMapIntensity: 0.6,
  });
  const metal = new THREE.MeshStandardMaterial({
    map: textures.metal,
    roughness: 0.28,
    metalness: 0.85,
    color: '#d7dee6',
  });
  const frame = new THREE.MeshStandardMaterial({ color: '#5c4634', roughness: 0.6, metalness: 0.05 });
  const ceiling = new THREE.MeshStandardMaterial({ color: '#d9d4cc', roughness: 0.9, metalness: 0 });
  const depth = new THREE.MeshDepthMaterial();
  return { textures, wall, concrete, floor, glass, metal, frame, ceiling, depth };
}

export function materialFor(set: MaterialSet, surface: number, kind: string): THREE.Material {
  if (kind === 'window' || surface === SURFACE.GLASS) return set.glass;
  if (kind === 'slab') return set.floor;
  if (surface === SURFACE.CONCRETE) return set.concrete;
  if (surface === SURFACE.METAL) return set.metal;
  if (surface === SURFACE.WOOD) return set.floor;
  return set.wall;
}
