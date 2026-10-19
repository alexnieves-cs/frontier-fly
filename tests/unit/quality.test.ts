import { describe, expect, it } from 'vitest';
import { isSoftwareRendererName, readRendererChoice, resolveQuality, shouldTryWebGPU } from '../../src/viz/quality';

describe('quality preset', () => {
  it('auto resolves to the full WebGL2 high stack unless GL is software', () => {
    expect(resolveQuality('auto')).toBe('high');
    expect(resolveQuality('auto', false)).toBe('high');
    expect(resolveQuality('auto', true)).toBe('low');
  });

  it('honors an explicit preset', () => {
    expect(resolveQuality('low', false)).toBe('low');
    expect(resolveQuality('high', true)).toBe('high');
  });

  it('only tries WebGPU on explicit opt-in', () => {
    expect(readRendererChoice('')).toBe('webgl2');
    expect(readRendererChoice('?quality=high')).toBe('webgl2');
    expect(readRendererChoice('?renderer=webgpu')).toBe('webgpu');
    expect(readRendererChoice('?renderer=bogus')).toBe('webgl2');
    expect(shouldTryWebGPU('webgl2', 'auto')).toBe(false);
    expect(shouldTryWebGPU('webgl2', 'high')).toBe(false);
    expect(shouldTryWebGPU('webgpu', 'high')).toBe(true);
    expect(shouldTryWebGPU('webgpu', 'auto')).toBe(true);
    expect(shouldTryWebGPU('webgpu', 'low')).toBe(false);
  });

  it('spots software rasterizers', () => {
    expect(isSoftwareRendererName('ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device (Subzero)))')).toBe(true);
    expect(isSoftwareRendererName('llvmpipe (LLVM 15.0.7, 256 bits)')).toBe(true);
    expect(isSoftwareRendererName('ANGLE (NVIDIA, NVIDIA GeForce RTX 3070)')).toBe(false);
  });
});
