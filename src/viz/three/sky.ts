import * as THREE from 'three';
import type { SkySample } from '../../shared/types';

export function applySkyBackdrop(scene: THREE.Scene, sky: SkySample): void {
  const color = new THREE.Color(sky.skyColor[0], sky.skyColor[1], sky.skyColor[2]);
  scene.background = color;
  scene.fog = new THREE.Fog(color, 18, 48);
}
