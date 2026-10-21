// Quadrotor diagnostics on the planner-tuning buildings (9101+), never on study seeds.
//
//   vite-node scripts/study/quad-diag.ts -- [--pilots classical,guarded] [--n 8] [--quad 1]
//
// For each episode: coverage, collisions, why it ended, and for each collision the
// horizontal and vertical speed just before it and the height above the floor.
import { loadPolicy, startEpisode, type Job, type PilotId } from './coverage-study';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const pilots = arg('pilots', 'classical,guarded').split(',') as PilotId[];
const n = Number(arg('n', '8'));
const quad = Number(arg('quad', '1'));
const lighting = arg('lighting', 'day') as Job['lighting'];
const policy = loadPolicy(arg('checkpoint', SHIPPED_CHECKPOINT_PATH));

for (const pilot of pilots) {
  let covSum = 0;
  let colSum = 0;
  const hits: { vh: number; vz: number; h: number }[] = [];
  const ends: Record<string, number> = {};
  for (let i = 0; i < n; i++) {
    const job: Job = { condition: 'diag', stage: 3, seed: 9101 + i, lighting, quad, pilot };
    const { env, spec, cfg, act, obs: first } = await startEpisode(job, policy);
    let obs = first;
    let coverage = 0;
    let collisions = 0;
    let end = 'steps';
    for (let t = 0; t < cfg.maxSteps; t++) {
      const s = env.snapshot();
      const step = env.step(act(obs, env));
      obs = step.obs;
      if (step.info.collisions > collisions) {
        const floor = Math.floor(s.pos[2] / spec.floorHeight);
        hits.push({ vh: Math.hypot(s.vel[0], s.vel[1]), vz: s.vel[2], h: s.pos[2] - floor * spec.floorHeight });
      }
      coverage = step.info.coverage;
      collisions = step.info.collisions;
      if (step.done) {
        end = coverage >= 0.95 ? 'success' : step.truncated ? 'stall' : collisions > 15 ? 'crashes' : 'other';
        break;
      }
    }
    env.dispose();
    covSum += coverage;
    colSum += collisions;
    ends[end] = (ends[end] ?? 0) + 1;
    console.log(`${pilot} seed ${job.seed}: coverage ${(coverage * 100).toFixed(1)}% collisions ${collisions} end ${end}`);
  }
  const q = (xs: number[], p: number) => (xs.length ? [...xs].sort((a, b) => a - b)[Math.floor(p * (xs.length - 1))]!.toFixed(2) : '-');
  console.log(
    `== ${pilot}: mean coverage ${((covSum / n) * 100).toFixed(1)}%, collisions/ep ${(colSum / n).toFixed(1)}, ends ${JSON.stringify(ends)}`,
  );
  console.log(
    `   at impact: speed h median ${q(hits.map((x) => x.vh), 0.5)} p90 ${q(hits.map((x) => x.vh), 0.9)}; vz median ${q(hits.map((x) => x.vz), 0.5)}; height median ${q(hits.map((x) => x.h), 0.5)} p10 ${q(hits.map((x) => x.h), 0.1)} p90 ${q(hits.map((x) => x.h), 0.9)}`,
  );
}
