import type { QualityPreset } from '../shared/types';

export type QualityChoice = QualityPreset | 'auto';

/** Which Three.js backend to try. WebGPU is opt-in (`?renderer=webgpu`). */
export type RendererChoice = 'webgl2' | 'webgpu';

/**
 * Auto resolves to the full WebGL2 stack (HDRI, shadows, SSAO, bloom, SMAA)
 * unless the GL context is a software rasterizer, where it drops to low.
 */
export function resolveQuality(choice: QualityChoice, softwareGL = false): QualityPreset {
  if (choice === 'low') return 'low';
  if (choice === 'high') return 'high';
  return softwareGL ? 'low' : 'high';
}

export function readRendererChoice(search: string): RendererChoice {
  return new URLSearchParams(search).get('renderer') === 'webgpu' ? 'webgpu' : 'webgl2';
}

/** WebGPU is only attempted on explicit request; its path skips the post stack. */
export function shouldTryWebGPU(renderer: RendererChoice, quality: QualityChoice): boolean {
  return renderer === 'webgpu' && quality !== 'low';
}

/** True when a WebGL renderer string names a software rasterizer such as SwiftShader or llvmpipe. */
export function isSoftwareRendererName(name: string): boolean {
  return /swiftshader|llvmpipe|softpipe|software/i.test(name);
}
