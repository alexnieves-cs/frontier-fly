import * as THREE from 'three';
import { AfterimagePass } from 'three/addons/postprocessing/AfterimagePass.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { SMAAPass } from 'three/addons/postprocessing/SMAAPass.js';
import { SSAOPass } from 'three/addons/postprocessing/SSAOPass.js';
import { TAARenderPass } from 'three/addons/postprocessing/TAARenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import type { CameraMode, QualityPreset } from '../../shared/types';

const LensShader = {
  uniforms: {
    tDiffuse: { value: null as THREE.Texture | null },
    amount: { value: 0 },
  },
  vertexShader: /* glsl */ `
    varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }
  `,
  fragmentShader: /* glsl */ `
    uniform sampler2D tDiffuse;
    uniform float amount;
    varying vec2 vUv;
    void main() {
      vec2 c = vUv - 0.5;
      float r2 = dot(c, c);
      vec2 bend = c * r2 * amount;
      float shift = amount * 0.0016;
      float r = texture2D(tDiffuse, vUv + bend + c * shift).r;
      float g = texture2D(tDiffuse, vUv + bend).g;
      float b = texture2D(tDiffuse, vUv + bend - c * shift).b;
      float a = texture2D(tDiffuse, vUv + bend).a;
      vec3 color = vec3(r, g, b);
      float vig = smoothstep(0.78, 0.22, r2);
      color *= mix(1.0, vig, clamp(amount * 0.45, 0.0, 0.55));
      gl_FragColor = vec4(color, a);
    }
  `,
};

export interface PostStack {
  composer: EffectComposer;
  bloom: UnrealBloomPass;
  ssao: SSAOPass | null;
  taa: TAARenderPass | null;
  render: RenderPass;
  afterimage: AfterimagePass;
  lens: ShaderPass;
  setView: (mode: CameraMode, speed: number) => void;
}

export function makePost(
  renderer: THREE.WebGLRenderer,
  scene: THREE.Scene,
  camera: THREE.Camera,
  quality: QualityPreset,
): PostStack {
  const composer = new EffectComposer(renderer);
  const render = new RenderPass(scene, camera);
  composer.addPass(render);
  let ssao: SSAOPass | null = null;
  let taa: TAARenderPass | null = null;
  if (quality === 'high') {
    ssao = new SSAOPass(scene, camera, renderer.domElement.width, renderer.domElement.height);
    ssao.kernelRadius = 12;
    ssao.minDistance = 0.0015;
    ssao.maxDistance = 0.09;
    composer.addPass(ssao);
    taa = new TAARenderPass(scene, camera);
    taa.sampleLevel = 0;
    taa.accumulate = false;
    taa.enabled = false;
  }
  const bloom = new UnrealBloomPass(new THREE.Vector2(renderer.domElement.width, renderer.domElement.height), 0.28, 0.45, 0.84);
  bloom.enabled = quality === 'high';
  composer.addPass(bloom);
  if (quality === 'high') {
    const w = renderer.domElement.width || 1280;
    const h = renderer.domElement.height || 720;
    composer.addPass(new SMAAPass(w, h));
  }
  const afterimage = new AfterimagePass(0.82);
  afterimage.enabled = false;
  composer.addPass(afterimage);
  const lens = new ShaderPass(LensShader);
  lens.uniforms['amount']!.value = 0;
  composer.addPass(lens);
  composer.addPass(new OutputPass());
  if (taa) composer.passes.splice(1, 0, taa);
  return {
    composer,
    bloom,
    ssao,
    taa,
    render,
    afterimage,
    lens,
    setView(mode: CameraMode, speed: number) {
      const fpv = mode === 'fpv';
      const still = speed < 0.08 && !fpv;
      afterimage.enabled = fpv;
      afterimage.uniforms['damp'].value = fpv ? 0.78 : 0;
      lens.uniforms['amount']!.value = fpv ? 1.35 : 0.12;
      if (taa) {
        taa.enabled = still;
        taa.accumulate = still;
        render.enabled = !still;
      }
    },
  };
}
