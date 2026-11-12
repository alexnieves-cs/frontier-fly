// How much does goal choice matter on the quadrotor? Planner variants on quad tuning buildings (66301+).
//   vite-node scripts/study/picker-ceiling.ts -- [--n 8] [--base 66301] [--variants a,b] [--shard 0 --of 1]
import { classicalPilot } from '../../src/eval/pilots';
import { QUAD_BRAKING } from '../../src/sim/lidarSafety';
import { RulePicker } from '../../src/sim/picker';
import type { SensorPlannerOptions } from '../../src/sim/sensorPlanner';
import { startEpisode, type Job } from './coverage-study';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}
const n = Number(arg('n', '8'));
const base = Number(arg('base', '66301'));
const shard = Number(arg('shard', '0'));
const of = Number(arg('of', '1'));
const lighting = arg('lighting', 'day') as Job['lighting'];
const quad = Number(arg('quad', '1'));
const B = { braking: QUAD_BRAKING, tracker: {} } satisfies SensorPlannerOptions;
const variants: Record<string, SensorPlannerOptions> = {
  base: B,
  plain: {},
  rule: { ...B, picker: new RulePicker() },
  mom1: { ...B, momentum: 1 },
  mom2: { ...B, momentum: 2 },
  mom4: { ...B, momentum: 4 },
  edge0: { ...B, edgeBonus: 0 },
  edge1: { ...B, edgeBonus: 1 },
  replan20: { ...B, replanEvery: 20 },
  speed1_5: { ...B, speed: 1.5 },
  speed3: { ...B, speed: 3 },
};
const only = arg('variants', '');
for (const [name, opts] of Object.entries(variants)) {
  if (only && !only.split(',').includes(name)) continue;
  for (let i = shard; i < n; i += of) {
    const t0 = Date.now();
    const job: Job = { condition: 'diag', stage: 3, seed: base + i, lighting, quad, pilot: 'heuristic' as Job['pilot'] };
    const { env, cfg, obs: first } = await startEpisode(job, (o) => o);
    const p = classicalPilot(opts);
    p.reset();
    let obs = first;
    let c = 0;
    let crashed = false;
    let steps = 0;
    for (let t = 0; t < cfg.maxSteps; t++) {
      const step = env.step(p.act(obs, env));
      obs = step.obs;
      c = step.info.coverage;
      steps = t + 1;
      if (step.done) {
        crashed = step.info.crashed;
        break;
      }
    }
    env.dispose();
    console.log(JSON.stringify({ variant: name, seed: base + i, lighting, coverage: c, crashed, steps, ms: Date.now() - t0 }));
  }
}
