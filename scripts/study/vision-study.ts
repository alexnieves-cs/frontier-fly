// Search-by-sight mission bench in parallel shards (src/eval/vision.ts), merged into docs/vision-bench.json.
//
//   npm run study:vision            (install tfjs-node first: npm install --no-save @tensorflow/tfjs-node@4.22.0)
//   vite-node scripts/study/vision-study.ts -- --shard 0 --of 4 --out /tmp/v0.jsonl
//   vite-node scripts/study/vision-study.ts -- --merge /tmp/v0.jsonl ... --out docs/vision-bench.json
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { flyVisionMission, VISION_SEED_BASE, VISION_SEEDS_PER_SCENARIO, visionBenchReport, visionJobs, type VisionFlight } from '../../src/eval/vision';
import { frameDetector } from '../../src/vision/detector';
import { loadDetectorFromDisk, loadHumanoidPosesSync, useNativeTf } from '../../src/vision/node';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const seeds = Number(arg('seeds', String(VISION_SEEDS_PER_SCENARIO)));
const base = Number(arg('base', String(VISION_SEED_BASE)));

if (process.argv.includes('--merge')) {
  const files = process.argv.slice(process.argv.indexOf('--merge') + 1).filter((a) => !a.startsWith('--') && a !== arg('out', '') && a !== arg('seeds', '') && a !== arg('base', ''));
  const flights: VisionFlight[] = files.flatMap((f) =>
    readFileSync(f, 'utf8').split('\n').filter(Boolean).map((l) => JSON.parse(l) as VisionFlight),
  );
  const commit = execSync('git rev-parse --short HEAD').toString().trim();
  const out = arg('out', 'docs/vision-bench.json');
  writeFileSync(out, `${JSON.stringify(visionBenchReport(flights, commit, seeds, base), null, 2)}\n`);
  console.log(`wrote ${out} from ${flights.length} flights`);
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `vision-${shard}.jsonl`);
  const backend = await useNativeTf();
  const detector = frameDetector(await loadDetectorFromDisk());
  const poses = loadHumanoidPosesSync();
  const mine = visionJobs(seeds, base).filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    const f = await flyVisionMission(mine[i]!, poses, detector);
    lines.push(JSON.stringify(f));
    writeFileSync(out, lines.join('\n') + '\n');
    console.log(`shard ${shard} (${backend}): ${i + 1}/${mine.length} ${f.scenario} ${f.seed} ${f.indoor} ${f.perception} ${f.outcome} ${f.frames} frames (${Math.round((Date.now() - t0) / 1000)} s)`);
  }
}
