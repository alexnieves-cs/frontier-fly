import * as THREE from 'three';
import type { SkySample } from '../../shared/types';

export function applySkyBackdrop(
  scene: THREE.Scene,
  sky: SkySample,
  hdri?: THREE.Texture | null,
  envMap?: THREE.Texture | null,
): void {
  const night = sky.night;
  const color = new THREE.Color(sky.skyColor[0], sky.skyColor[1], sky.skyColor[2]);
  if (night > 0.55) color.setRGB(0.012, 0.015, 0.03);
  if (hdri && night < 0.35) scene.background = hdri;
  else scene.background = color;
  if (envMap) scene.environment = envMap;
  scene.environmentIntensity = night > 0.55 ? 0.08 : 1.15 * (1 - night * 0.85);
  scene.fog = new THREE.Fog(color, night > 0.55 ? 5 : 16, night > 0.55 ? 20 : 52);
}
