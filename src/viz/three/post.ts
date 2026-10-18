import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { SSAOPass } from 'three/addons/postprocessing/SSAOPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import type { QualityPreset } from '../../shared/types';

export interface PostStack {
  composer: EffectComposer;
  bloom: UnrealBloomPass;
  ssao: SSAOPass | null;
}

export function makePost(
  renderer: THREE.WebGLRenderer,
  scene: THREE.Scene,
  camera: THREE.Camera,
  quality: QualityPreset,
): PostStack {
  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));
  let ssao: SSAOPass | null = null;
  if (quality === 'high') {
    ssao = new SSAOPass(scene, camera, renderer.domElement.width, renderer.domElement.height);
    ssao.kernelRadius = 10;
    ssao.minDistance = 0.002;
    ssao.maxDistance = 0.12;
    composer.addPass(ssao);
  }
  const bloom = new UnrealBloomPass(new THREE.Vector2(renderer.domElement.width, renderer.domElement.height), 0.22, 0.4, 0.86);
  if (quality === 'low') bloom.enabled = false;
  composer.addPass(bloom);
  composer.addPass(new OutputPass());
  return { composer, bloom, ssao };
}
