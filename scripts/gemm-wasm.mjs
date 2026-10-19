// Generator for the SIMD gemm in src/rl/gemm.ts (node scripts/gemm-wasm.mjs [wRows=3] [xRows=2]).
//
// Every output y[i, j] is computed with exactly the same f32 operations, in the
// same order, as the original single-accumulator kernel (OLD below): a 4-lane
// accumulator over k in steps of 4, lanes summed as (l0 + l1) + (l2 + l3), the
// scalar tail added one element at a time, then the bias. The new kernel only
// interleaves several (i, j) dot products (wRows weight rows x xRows input
// rows, each with its own accumulator), which hides the add latency chain and
// reuses each loaded x / w vector. Results are bit-identical
// (tests/unit/gemm.test.ts checks against OLD).
/* global process, Buffer, WebAssembly */
const OLD = 'AGFzbQEAAAABCwFgB39/f39/f38AAwIBAAUDAQBABxECBm1lbW9yeQIABGdlbW0AAAqIAgGFAgQGfwF7AX0BfyAFQQJ0IQwgBUF8cSEPA0BBACEIIAAgByAMbGohCgNAIAEgCCAMbGohC0MAAAAA/RMhDUEAIQkCQANAIAkgD08NASANIAogCUECdGr9AAIAIAsgCUECdGr9AAIA/eYB/eQBIQ0gCUEEaiEJDAALCyAN/R8AIA39HwGSIA39HwIgDf0fA5KSIQ4CQANAIAkgBU8NASAOIAogCUECdGoqAgAgCyAJQQJ0aioCAJSSIQ4gCUEBaiEJDAALCyAOIAIgCEECdGoqAgCSIQ4gAyAHIAZsIAhqQQJ0aiAOOAIAIAhBAWohCCAIIAZJDQALIAdBAWohByAHIARJDQALCw==';

const ROWS = Number(process.argv[2] ?? 3);
const XR = Number(process.argv[3] ?? 2); // x rows per block

const uleb = (n) => {
  const out = [];
  do {
    let b = n & 0x7f;
    n >>>= 7;
    if (n) b |= 0x80;
    out.push(b);
  } while (n);
  return out;
};
const sleb = (n) => {
  const out = [];
  for (;;) {
    const b = n & 0x7f;
    n >>= 7;
    if ((n === 0 && !(b & 0x40)) || (n === -1 && b & 0x40)) {
      out.push(b);
      return out;
    }
    out.push(b | 0x80);
  }
};
const code = [];
const op = (...bytes) => code.push(...bytes.flat());
const get = (l) => op(0x20, uleb(l));
const set = (l) => op(0x21, uleb(l));
const i32 = (v) => op(0x41, sleb(v));
const add = () => op(0x6a);
const mul = () => op(0x6c);
const shl = () => op(0x74);
const and = () => op(0x71);
const geU = () => op(0x4f);
const brIf = (d) => op(0x0d, uleb(d));
const br = (d) => op(0x0c, uleb(d));
const block = () => op(0x02, 0x40);
const loop = () => op(0x03, 0x40);
const end = () => op(0x0b);
const v128load = () => op(0xfd, 0x00, 0x02, 0x00);
const f32x4mul = () => op(0xfd, 0xe6, 0x01);
const f32x4add = () => op(0xfd, 0xe4, 0x01);
const lane = (k) => op(0xfd, 0x1f, k);
const f32add = () => op(0x92);
const f32mul = () => op(0x94);
const f32load = () => op(0x2a, 0x02, 0x00);
const f32store = () => op(0x38, 0x02, 0x00);
const zero4 = () => op(0x43, 0, 0, 0, 0, 0xfd, 0x13);

// params
const X = 0, W = 1, Bp = 2, Y = 3, N = 4, IN = 5, OUT = 6;
let next = 7;
const I = next++, J = next++, K = next++, ROWB = next++, KEND = next++, JEND = next++;
const XROW = Array.from({ length: XR }, () => next++);
const WROW = Array.from({ length: ROWS }, () => next++);
const nI32 = next - 7;
const ACC = Array.from({ length: XR * ROWS }, () => next++);
const XV = Array.from({ length: XR }, () => next++);
const nV128 = XR * ROWS + XR;
const S = next++;

// rowBytes = inDim << 2 ; kEnd = inDim & -4 ; jEnd = outDim - outDim % ROWS ; nEnd = n - n % XR
get(IN); i32(2); shl(); set(ROWB);
get(IN); i32(-4); and(); set(KEND);
get(OUT); get(OUT); i32(ROWS); op(0x70); /* i32.rem_u */ op(0x6b); /* sub */ set(JEND);

// finalize output (xr, m) using acc local A, x row local XRw, w row local Wr; column index = J + m
function finalize(xr, m) {
  const A = ACC[xr * ROWS + m];
  get(A); lane(0); get(A); lane(1); f32add(); get(A); lane(2); get(A); lane(3); f32add(); f32add(); set(S);
  get(KEND); set(K);
  block(); loop();
  get(K); get(IN); geU(); brIf(1);
  get(S);
  get(XROW[xr]); get(K); i32(2); shl(); add(); f32load();
  get(WROW[m]); get(K); i32(2); shl(); add(); f32load();
  f32mul(); f32add(); set(S);
  get(K); i32(1); add(); set(K);
  br(0); end(); end();
  // s = s + b[j + m]
  get(S); get(Bp); get(J); i32(m); add(); i32(2); shl(); add(); f32load(); f32add(); set(S);
  // y[((i + xr) * outDim + j + m)] = s
  get(Y);
  get(I); i32(xr); add(); get(OUT); mul(); get(J); add(); i32(m); add(); i32(2); shl(); add();
  get(S); f32store();
}

function rowsBlock(rows, xrs) {
  // w rows
  for (let m = 0; m < rows; m++) {
    get(W); get(J); i32(m); add(); get(ROWB); mul(); add(); set(WROW[m]);
  }
  for (let xr = 0; xr < xrs; xr++) for (let m = 0; m < rows; m++) { zero4(); set(ACC[xr * ROWS + m]); }
  i32(0); set(K);
  block(); loop();
  get(K); get(KEND); geU(); brIf(1);
  for (let xr = 0; xr < xrs; xr++) { get(XROW[xr]); get(K); i32(2); shl(); add(); v128load(); set(XV[xr]); }
  for (let m = 0; m < rows; m++) {
    // reuse via local: load w row chunk once per m, apply to all x rows
    for (let xr = 0; xr < xrs; xr++) {
      const A = ACC[xr * ROWS + m];
      get(A);
      get(XV[xr]);
      get(WROW[m]); get(K); i32(2); shl(); add(); v128load();
      f32x4mul(); f32x4add(); set(A);
    }
  }
  get(K); i32(4); add(); set(K);
  br(0); end(); end();
  for (let xr = 0; xr < xrs; xr++) for (let m = 0; m < rows; m++) finalize(xr, m);
}

function iBody(xrs) {
  for (let xr = 0; xr < xrs; xr++) { get(X); get(I); i32(xr); add(); get(ROWB); mul(); add(); set(XROW[xr]); }
  // j = 0; while j < jEnd: block of ROWS
  i32(0); set(J);
  block(); loop();
  get(J); get(JEND); geU(); brIf(1);
  rowsBlock(ROWS, xrs);
  get(J); i32(ROWS); add(); set(J);
  br(0); end(); end();
  // remaining single rows
  block(); loop();
  get(J); get(OUT); geU(); brIf(1);
  rowsBlock(1, xrs);
  get(J); i32(1); add(); set(J);
  br(0); end(); end();
}

// i loop, XR rows at a time, then leftovers one at a time
i32(0); set(I);
if (XR > 1) {
  block(); loop();
  get(I); i32(XR - 1); add(); get(N); geU(); brIf(1);
  iBody(XR);
  get(I); i32(XR); add(); set(I);
  br(0); end(); end();
}
block(); loop();
get(I); get(N); geU(); brIf(1);
iBody(1);
get(I); i32(1); add(); set(I);
br(0); end(); end();
end();

const locals = [...uleb(3), ...uleb(nI32), 0x7f, ...uleb(nV128), 0x7b, 1, 0x7d];
const body = [...locals, ...code];
const funcBody = [...uleb(body.length), ...body];
const codeSec = [1, ...funcBody];
const header = Buffer.from(OLD, 'base64');
// reuse everything up to the code section (id 0x0a) from the old module
let p = 8;
const pre = [];
while (p < header.length) {
  const id = header[p];
  let q = p + 1, size = 0, shift = 0, b;
  do { b = header[q++]; size |= (b & 0x7f) << shift; shift += 7; } while (b & 0x80);
  if (id === 0x0a) break;
  pre.push(...header.subarray(p, q + size));
  p = q + size;
}
const bytes = Uint8Array.from([...header.subarray(0, 8), ...pre, 0x0a, ...uleb(codeSec.length), ...codeSec]);
new WebAssembly.Module(bytes);
process.stdout.write(Buffer.from(bytes).toString('base64'));
