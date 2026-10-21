// Renders detector training or evaluation frames from a block of world seeds.
//   npx vite-node scripts/vision/make-dataset.ts -- --from 66001 --to 66400 --per 60 --out <dir> [--shard 0/4]
// Writes <dir>/frames-<shard>.u8 (N x 120 x 160 x 3 RGB) and <dir>/labels-<shard>.jsonl (one line per frame).
import { closeSync, mkdirSync, openSync, writeFileSync, writeSync } from 'node:fs';
import { worldFrames } from '../../src/vision/dataset';
import { loadHumanoidPosesSync } from '../../src/vision/node';

const args = process.argv.slice(2).filter((a) => a !== '--');
const arg = (name: string, def: string): string => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1]! : def;
};
const from = Number(arg('from', '66001'));
const to = Number(arg('to', '66010'));
const per = Number(arg('per', '60'));
const out = arg('out', 'test-results/vision-data');
const [shard, shards] = arg('shard', '0/1').split('/').map(Number) as [number, number];
const randomize = arg('randomize', '1') === '1';
mkdirSync(out, { recursive: true });
const poses = loadHumanoidPosesSync();
const fd = openSync(`${out}/frames-${shard}.u8`, 'w');
const lines: string[] = [];
let n = 0;
const t0 = Date.now();
for (let seed = from; seed <= to; seed++) {
  if ((seed - from) % shards !== shard) continue;
  for (const item of worldFrames(seed, per, poses, randomize)) {
    writeSync(fd, item.frame.rgb);
    const r = (v: number): number => Math.round(v * 10000) / 10000;
    lines.push(JSON.stringify({
      seed, index: item.index, kind: item.kind, night: item.night, gain: r(item.frame.gain),
      labels: item.labels.map((l) => ({ c: l.classId, k: l.cls, x: r(l.x), y: r(l.y), w: r(l.w), h: r(l.h), px: l.pixels, d: r(l.distance), id: l.targetId })),
    }));
    n++;
  }
}
closeSync(fd);
writeFileSync(`${out}/labels-${shard}.jsonl`, lines.join('\n') + '\n');
console.log(`shard ${shard}/${shards}: ${n} frames in ${((Date.now() - t0) / 1000).toFixed(0)} s`);
