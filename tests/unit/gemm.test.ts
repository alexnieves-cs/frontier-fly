import { describe, expect, it } from 'vitest';
import { createGemm } from '../../src/rl/gemm';
import { InferPolicy } from '../../src/rl/infer';
import { RNG } from '../../src/shared/rng';

/** The original single-accumulator kernel; the blocked kernel must match it bit for bit. */
const ORIGINAL =
  'AGFzbQEAAAABCwFgB39/f39/f38AAwIBAAUDAQBABxECBm1lbW9yeQIABGdlbW0AAAqIAgGFAgQGfwF7AX0BfyAFQQJ0IQwgBUF8cSEPA0BBACEIIAAgByAMbGohCgNAIAEgCCAMbGohC0MAAAAA/RMhDUEAIQkCQANAIAkgD08NASANIAogCUECdGr9AAIAIAsgCUECdGr9AAIA/eYB/eQBIQ0gCUEEaiEJDAALCyAN/R8AIA39HwGSIA39HwIgDf0fA5KSIQ4CQANAIAkgBU8NASAOIAogCUECdGoqAgAgCyAJQQJ0aioCAJSSIQ4gCUEBaiEJDAALCyAOIAIgCEECdGoqAgCSIQ4gAyAHIAZsIAhqQQJ0aiAOOAIAIAhBAWohCCAIIAZJDQALIAdBAWohByAHIARJDQALCw==';

function originalGemm(x: Float32Array, n: number, inDim: number, w: Float32Array, b: Float32Array, outDim: number): Float32Array {
  const instance = new WebAssembly.Instance(new WebAssembly.Module(Uint8Array.from(atob(ORIGINAL), (c) => c.charCodeAt(0))));
  const memory = instance.exports.memory as WebAssembly.Memory;
  const gemm = instance.exports.gemm as (...args: number[]) => void;
  const floats = x.length + w.length + b.length + n * outDim;
  memory.grow(Math.ceil((floats * 4) / 65536) + 1);
  const f32 = new Float32Array(memory.buffer);
  const xo = 0;
  const wo = xo + x.length;
  const bo = wo + w.length;
  const yo = bo + b.length;
  f32.set(x, xo);
  f32.set(w, wo);
  f32.set(b, bo);
  gemm(xo * 4, wo * 4, bo * 4, yo * 4, n, inDim, outDim);
  return f32.slice(yo, yo + n * outDim);
}

describe('SIMD gemm', () => {
  const kernel = createGemm();

  it.skipIf(!kernel)('is bit-identical to the original kernel, including odd shapes and wide inputs', () => {
    const rng = new RNG(5, 'gemm');
    const shapes: [number, number, number][] = [
      [1, 306, 256],
      [8, 306, 256],
      [7, 257, 129],
      [3, 5, 7],
      [2, 3, 1],
      [9, 256, 128],
      [8, 128, 4],
      [5, 1001, 13],
      // More rows than fit the scratch area at this width: exercises the batch split.
      [40, 1000, 64],
    ];
    for (const [n, inDim, outDim] of shapes) {
      const k = createGemm()!;
      const x = Float32Array.from({ length: n * inDim }, () => rng.range(-1, 1));
      const w = Float32Array.from({ length: outDim * inDim }, () => rng.range(-0.1, 0.1));
      const b = Float32Array.from({ length: outDim }, () => rng.range(-1, 1));
      const ws = k.upload(w);
      const bs = k.upload(b);
      const got = k.apply(x, n, inDim, ws, bs, outDim, false);
      const want = originalGemm(x, n, inDim, w, b, outDim);
      expect(new Uint32Array(got.buffer)).toEqual(new Uint32Array(want.buffer));
    }
  });

  it('batched inference equals per-row inference for a ~1000-float observation', () => {
    const dim = 1000;
    const policy = new InferPolicy(dim, 4, [256, 128], 3);
    const rng = new RNG(8, 'obs');
    const n = 40;
    const obs = Float32Array.from({ length: n * dim }, () => rng.range(-2, 2));
    const batch = policy.actWith(obs, n, new RNG(1), true);
    for (let i = 0; i < n; i++) {
      const one = policy.actWith(obs.subarray(i * dim, (i + 1) * dim), 1, new RNG(1), true);
      expect(Array.from(batch.action.subarray(i * 4, i * 4 + 4))).toEqual(Array.from(one.action));
      expect(batch.value[i]).toBe(one.value[0]);
    }
  });
});
