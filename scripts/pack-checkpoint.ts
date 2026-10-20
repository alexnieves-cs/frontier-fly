/**
 * Packs a JSON checkpoint into the binary form the site loads, and checks the
 * result decodes to the same numbers.
 *
 *   npm run pack:checkpoint -- ckpt.json [public/checkpoints/pretrained.bin]
 */
import { readFileSync, writeFileSync } from 'node:fs';
import { parseCheckpoint, serializeCheckpointBinary, SHIPPED_CHECKPOINT_PATH } from '../src/rl/checkpoint';

const [src, out = SHIPPED_CHECKPOINT_PATH] = process.argv.slice(2).filter((a) => a !== '--');
if (!src) {
  console.error('usage: npm run pack:checkpoint -- <checkpoint.json> [out.bin]');
  process.exit(1);
}
const checkpoint = parseCheckpoint(readFileSync(src));
const bytes = serializeCheckpointBinary(checkpoint);
const back = parseCheckpoint(bytes);
const same = checkpoint.weights.every((w, i) => w.every((v, j) => Math.fround(v) === back.weights[i][j]));
if (!same) throw new Error('packed weights do not round-trip');
writeFileSync(out, bytes);
console.log(`wrote ${out}: ${(bytes.length / 1e6).toFixed(2)} MB (from ${(readFileSync(src).length / 1e6).toFixed(2)} MB)`);
