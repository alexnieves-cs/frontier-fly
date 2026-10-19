/**
 * Tiny behavior-cloning sanity run: teacher-only data, a few epochs, no PPO.
 * It checks that an observation layout carries the teacher's signal, not that
 * the result is shippable. Usage: npx vite-node scripts/bc-sanity.ts -- out.json [epochs]
 * Then: npm run eval:coverage -- out.json
 */
import { writeFileSync } from 'node:fs';
import { generateBuilding } from '../src/gen/building';
import { Curriculum } from '../src/rl/curriculum';
import { InferPolicy } from '../src/rl/infer';
import { TfTrainer } from '../src/rl/model';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../src/shared/constants';
import { RNG } from '../src/shared/rng';
import { createEnv, defaultEpisodeConfig } from '../src/sim/env';
import { frontierAction } from '../src/sim/frontier';
import { lightingPreset } from '../src/sim/lighting';

const args = process.argv.slice(2).filter((a) => a !== '--');
const out = args[0] ?? '/tmp/bc-sanity.json';
const epochs = Number(args[1] ?? 6);

async function main() {
  const started = Date.now();
  const policy = new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, 11);
  const trainer = new TfTrainer(policy, 1e-3);
  await trainer.ready();
  const env = await createEnv();
  const curriculum = new Curriculum();
  const jobs = [
    ...[20, 23, 26, 29].map((seed) => ({ stage: 0, seed, limit: 600 })),
    ...[80, 85, 90].map((seed) => ({ stage: 3, seed, limit: 900 })),
  ];
  const cap = jobs.reduce((s, j) => s + j.limit, 0);
  const obsBuf = new Float32Array(cap * OBS_DIM);
  const actBuf = new Float32Array(cap * ACT_DIM);
  let n = 0;
  for (const job of jobs) {
    const params = curriculum.genParams(job.stage, new RNG(job.seed, 'teach'));
    const spec = generateBuilding(params);
    const nRooms = spec.floors.reduce((r, f) => r + f.rooms.length, 0);
    const cfg = defaultEpisodeConfig(job.stage, nRooms);
    cfg.maxSteps = job.stage <= 1 ? 1000 : 2400;
    cfg.stallSteps = 500;
    cfg.maxCollisions = 16;
    cfg.dynamics = 'kinematic';
    cfg.lighting = lightingPreset('day', nRooms);
    let obs = env.reset(spec, params.seed, cfg);
    for (let t = 0; t < job.limit; t++) {
      const pose = env.snapshot();
      const teacher = frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw);
      obsBuf.set(obs, n * OBS_DIM);
      actBuf.set(teacher, n * ACT_DIM);
      n += 1;
      const step = env.step(teacher);
      obs = step.obs;
      if (step.done) break;
    }
  }
  console.log(`collected ${n} teacher samples (obs ${OBS_DIM}) in ${((Date.now() - started) / 1000).toFixed(1)} s`);
  await trainer.imitate(obsBuf, actBuf, n, {
    epochs,
    minibatch: 256,
    onEpoch: (epoch, loss) => console.log(`epoch ${epoch + 1} loss ${loss.toFixed(4)}`),
  });
  const checkpoint = {
    version: 1,
    obsVersion: OBS_DIM === 306 ? 1 : 2,
    obsDim: OBS_DIM,
    actDim: ACT_DIM,
    hidden: [...HIDDEN],
    weights: policy.getWeights().map((buf) => Array.from(new Float32Array(buf))),
    obsMean: new Array(OBS_DIM).fill(0),
    obsVar: new Array(OBS_DIM).fill(1),
    obsCount: 1,
    rewVar: 1,
    stage: 3,
  };
  writeFileSync(out, JSON.stringify(checkpoint));
  console.log(`wrote ${out} in ${((Date.now() - started) / 1000).toFixed(1)} s`);
}

void main();
