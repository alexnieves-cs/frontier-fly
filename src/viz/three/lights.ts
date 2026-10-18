import * as THREE from 'three';
import type { SkySample } from '../../shared/types';

export function makeSun(): THREE.DirectionalLight {
  const sun = new THREE.DirectionalLight('#fff4e0', 4);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.04;
  return sun;
}

export function applySkyLights(
  sun: THREE.DirectionalLight,
  ambient: THREE.AmbientLight,
  hemi: THREE.HemisphereLight,
  sky: SkySample,
  flashlightOn: boolean,
): void {
  const dir = sky.sunDir;
  sun.position.set(dir[0] * 24, Math.max(0.4, dir[2]) * 24, dir[1] * 24);
  sun.intensity = sky.sunIntensity;
  sun.color.setRGB(sky.sunColor[0], sky.sunColor[1], sky.sunColor[2]);
  sun.visible = sky.sunIntensity > 0.05;
  ambient.intensity = sky.ambient + (flashlightOn ? 0.04 : 0);
  hemi.intensity = 0.25 + sky.ambient * 0.8;
  hemi.color.setRGB(sky.skyColor[0], sky.skyColor[1], sky.skyColor[2]);
  hemi.groundColor.setRGB(0.15, 0.13, 0.1);
}
