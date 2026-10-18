import { HIDDEN, OBS_DIM } from '../shared/constants';

/**
 * SIMD f32 gemm used by policy inference. The module is assembled from a
 * hand-written WASM text program (no third-party weights). If SIMD is missing,
 * callers keep the scalar fallback.
 *
 * y[i, j] = bias[j] + dot(x[i], W[j])
 * W is [out, in], row-major. Pointers are byte offsets.
 */
const WASM_B64 =
  'AGFzbQEAAAABCwFgB39/f39/f38AAwIBAAUDAQBABxECBm1lbW9yeQIABGdlbW0AAAqIAgGFAgQGfwF7AX0BfyAFQQJ0IQwgBUF8cSEPA0BBACEIIAAgByAMbGohCgNAIAEgCCAMbGohC0MAAAAA/RMhDUEAIQkCQANAIAkgD08NASANIAogCUECdGr9AAIAIAsgCUECdGr9AAIA/eYB/eQBIQ0gCUEEaiEJDAALCyAN/R8AIA39HwGSIA39HwIgDf0fA5KSIQ4CQANAIAkgBU8NASAOIAogCUECdGoqAgAgCyAJQQJ0aioCAJSSIQ4gCUEBaiEJDAALCyAOIAIgCEECdGoqAgCSIQ4gAyAHIAZsIAhqQQJ0aiAOOAIAIAhBAWohCCAIIAZJDQALIAdBAWohByAHIARJDQALCw==';

const MAX_BATCH = 64;
/** Row stride of the input/output scratch: the widest layer, rounded up to 4 floats. */
const STRIDE = Math.ceil(Math.max(OBS_DIM, ...HIDDEN, 320) / 4) * 4;
/** Input scratch at [0, ACT_FLOATS), output scratch at [ACT_FLOATS, 2 * ACT_FLOATS), weights after. */
const ACT_FLOATS = MAX_BATCH * STRIDE;

export interface GemmKernel {
  upload(data: Float32Array): number;
  reset(): void;
  apply(
    x: Float32Array,
    n: number,
    inDim: number,
    weight: number,
    bias: number,
    outDim: number,
    tanhOut: boolean,
  ): Float32Array;
}

type GemmExport = (
  x: number,
  w: number,
  b: number,
  y: number,
  n: number,
  inDim: number,
  outDim: number,
) => void;

let moduleCache: WebAssembly.Module | null | undefined;

function simdModule(): WebAssembly.Module | null {
  if (moduleCache !== undefined) return moduleCache;
  try {
    const binary = Uint8Array.from(atob(WASM_B64), (char) => char.charCodeAt(0));
    moduleCache = new WebAssembly.Module(binary);
  } catch {
    moduleCache = null;
  }
  return moduleCache;
}

export function createGemm(): GemmKernel | null {
  const compiled = simdModule();
  if (!compiled) return null;
  try {
    const instance = new WebAssembly.Instance(compiled);
    const memory = instance.exports.memory as WebAssembly.Memory;
    const gemm = instance.exports.gemm as GemmExport;
    let f32 = new Float32Array(memory.buffer);
    let cursor = ACT_FLOATS * 2;
    return {
      reset() {
        cursor = ACT_FLOATS * 2;
      },
      upload(data: Float32Array) {
        if (cursor + data.length > f32.length) {
          const pages = Math.ceil(((cursor + data.length) * 4 - memory.buffer.byteLength) / 65536) + 1;
          memory.grow(pages);
          f32 = new Float32Array(memory.buffer);
        }
        const index = cursor;
        f32.set(data, index);
        cursor += data.length;
        return index;
      },
      apply(x, n, inDim, weight, bias, outDim, tanhOut) {
        const y = new Float32Array(n * outDim);
        // Wider-than-STRIDE layers still fit: shrink the batch so neither scratch overflows.
        const rows = Math.max(1, Math.min(MAX_BATCH, Math.floor(ACT_FLOATS / Math.max(inDim, outDim, 1))));
        if (Math.max(inDim, outDim) > ACT_FLOATS) throw new Error('gemm layer wider than scratch');
        let done = 0;
        while (done < n) {
          const batch = Math.min(rows, n - done);
          f32.set(x.subarray(done * inDim, (done + batch) * inDim), 0);
          gemm(0, weight * 4, bias * 4, ACT_FLOATS * 4, batch, inDim, outDim);
          const produced = f32.subarray(ACT_FLOATS, ACT_FLOATS + batch * outDim);
          const dest = done * outDim;
          if (tanhOut) {
            for (let i = 0; i < produced.length; i++) y[dest + i] = Math.tanh(produced[i]!);
          } else {
            y.set(produced, dest);
          }
          done += batch;
        }
        return y;
      },
    };
  } catch {
    return null;
  }
}
