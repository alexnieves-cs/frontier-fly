// Runs a sharded study script in parallel and merges the shards.
//
//   node scripts/study/parallel.mjs coverage-study docs/study.json
//   node scripts/study/parallel.mjs mission-study docs/mission-bench.json
//
// Shards default to the number of cores (STUDY_SHARDS overrides). Extra arguments go to every shard.
import { spawn } from 'node:child_process';
import console from 'node:console';
import process from 'node:process';
import { mkdtempSync } from 'node:fs';
import { availableParallelism, tmpdir } from 'node:os';
import { join } from 'node:path';

const [name, out, ...extra] = process.argv.slice(2);
if (!name || !out) {
  console.error('usage: node scripts/study/parallel.mjs <study> <out.json> [args...]');
  process.exit(2);
}
const script = `scripts/study/${name}.ts`;
const shards = Number(process.env.STUDY_SHARDS ?? availableParallelism());
const dir = mkdtempSync(join(tmpdir(), `${name}-`));
const files = Array.from({ length: shards }, (_, i) => join(dir, `shard-${i}.jsonl`));

function run(args) {
  return new Promise((resolve, reject) => {
    const child = spawn('npx', ['vite-node', script, '--', ...args], { stdio: 'inherit' });
    child.on('exit', (code) => (code === 0 ? resolve() : reject(new Error(`${script} ${args.join(' ')} exited ${code}`))));
  });
}

await Promise.all(files.map((file, i) => run(['--shard', String(i), '--of', String(shards), '--out', file, ...extra])));
await run(['--merge', ...files, '--out', out, ...extra]);
