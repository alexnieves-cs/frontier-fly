// Why the policy crashes on quadrotor dynamics: the shipped policy behind simple action wrappers,
// on the planner-tuning buildings (9101+), never on study seeds.
//
//   vite-node scripts/study/quad-policy-diag.ts -- [--n 16] [--checkpoint path] [--base 9101]
import { guardedAction } from '../../src/eval/pilots';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { MAX_YAW_RATE } from '../../src/shared/constants';
import type { ExplorerEnv } from '../../src/sim/env';
import { QUAD_BRAKING } from '../../src/sim/lidarSafety';
import { loadPolicy, startEpisode, type Job } from './coverage-study';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}
const n = Number(arg('n', '16'));
const base = Number(arg('base', '9101'));
const policy = loadPolicy(arg('checkpoint', SHIPPED_CHECKPOINT_PATH));

type Wrap = (a: Float32Array, obs: Float32Array, env: ExplorerEnv) => Float32Array;
const yawCap = (cap: number): Wrap => (a) => {
  const lim = cap / MAX_YAW_RATE;
  a[3] = Math.max(-lim, Math.min(lim, a[3]!));
  return a;
};
const speed = (k: number): Wrap => (a) => {
  a[0]! *= k;
  a[1]! *= k;
  return a;
};
const brake: Wrap = (a, obs, env) => guardedAction(a, obs, env, QUAD_BRAKING);
const chain = (...ws: Wrap[]): Wrap => (a, obs, env) => ws.reduce((x, w) => w(x, obs, env), a);
const variants: Record<string, Wrap> = {
  policy: (a) => a,
  'yaw0.3': yawCap(0.3),
  'yaw0.6': yawCap(0.6),
  'speed0.5': speed(0.5),
  'speed0.5+yaw0.6': chain(speed(0.5), yawCap(0.6)),
  brake,
  'brake+yaw0.3': chain(yawCap(0.3), brake),
  'brake+yaw0.6': chain(yawCap(0.6), brake),
  'brake+yaw0.6+speed0.7': chain(speed(0.7), yawCap(0.6), brake),
};
const only = arg('variants', '');
for (const [name, wrap] of Object.entries(variants)) {
  if (only && !only.split(',').includes(name)) continue;
  let cov = 0;
  let crashes = 0;
  let coll = 0;
  for (let i = 0; i < n; i++) {
    const job: Job = { condition: 'diag', stage: 3, seed: base + i, lighting: 'day', quad: 1, pilot: 'policy' };
    const { env, cfg, obs: first } = await startEpisode(job, policy);
    let obs = first;
    let c = 0;
    for (let t = 0; t < cfg.maxSteps; t++) {
      const step = env.step(wrap(policy(obs), obs, env));
      obs = step.obs;
      c = step.info.coverage;
      if (step.done) {
        if (step.info.crashed) crashes++;
        coll += step.info.collisions;
        break;
      }
    }
    env.dispose();
    cov += c;
  }
  console.log(`${name.padEnd(24)} coverage ${((cov / n) * 100).toFixed(1)}%  crashes ${crashes}/${n}  contacts/ep ${(coll / n).toFixed(2)}`);
}
