/**
 * Prints the shipped-policy coverage table against the targets.
 * Usage: npm run eval:coverage [-- path/to/checkpoint.json] [--teacher]
 */
import { readFileSync } from 'node:fs';
import { evalCoverage, formatCoverageTable } from '../src/eval/coverage';
import { checkpointMismatch, parseCheckpoint } from '../src/rl/checkpoint';
import { InferPolicy } from '../src/rl/infer';
import { RNG } from '../src/shared/rng';
import { frontierAction } from '../src/sim/frontier';

const args = process.argv.slice(2);
const teacher = args.includes('--teacher');
const path = args.find((a) => !a.startsWith('--')) ?? 'public/checkpoints/pretrained.json';

async function main() {
  const started = Date.now();
  if (teacher) {
    const report = await evalCoverage({
      act: (_obs, env) => {
        const pose = env.snapshot();
        return frontierAction(env.building!, env.visitedGrid, pose.pos, pose.yaw);
      },
    });
    console.log(formatCoverageTable(report, 'privileged frontier teacher'));
  } else {
    const checkpoint = parseCheckpoint(readFileSync(path, 'utf8'));
    const mismatch = checkpointMismatch(checkpoint);
    if (mismatch) {
      console.error(`${path}: ${mismatch}`);
      process.exitCode = 1;
      return;
    }
    const policy = new InferPolicy(checkpoint.obsDim, checkpoint.actDim, checkpoint.hidden, 1);
    policy.setWeights(checkpoint.weights.map((row) => Float32Array.from(row).buffer));
    const rng = new RNG(1);
    const report = await evalCoverage({ act: (obs) => policy.actWith(obs, 1, rng, true).action });
    console.log(formatCoverageTable(report, path));
  }
  console.log(`eval took ${((Date.now() - started) / 1000).toFixed(1)} s`);
}

void main();
