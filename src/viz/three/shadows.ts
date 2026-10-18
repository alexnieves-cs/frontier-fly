import * as THREE from 'three';
import type { QualityPreset } from '../../shared/types';

export function configureShadows(
  renderer: THREE.WebGLRenderer,
  sun: THREE.DirectionalLight,
  quality: QualityPreset,
  span: number,
): void {
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = quality === 'high' ? THREE.PCFSoftShadowMap : THREE.BasicShadowMap;
  const half = Math.max(8, span * 0.6);
  const cam = sun.shadow.camera;
  cam.near = 0.5;
  cam.far = 80;
  cam.left = -half;
  cam.right = half;
  cam.top = half;
  cam.bottom = -half;
  sun.shadow.mapSize.set(quality === 'high' ? 2048 : 1024, quality === 'high' ? 2048 : 1024);
  cam.updateProjectionMatrix();
}
