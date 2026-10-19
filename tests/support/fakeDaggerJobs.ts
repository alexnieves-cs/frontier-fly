/** Test job module for NodeRolloutPool: a stand-in DAgger collector with the src/rl/dagger.ts shape. */
import { ACT_DIM } from '../../src/shared/constants';
import { frontierAction } from '../../src/sim/frontier';
import { makeDaggerJob, type CollectDaggerArgs } from '../../src/train/daggerJob';
import type { JobFn } from '../../src/train/rolloutCore';

function fakeCollectDagger({ env, spec, cfg, policy, beta, steps, rng }: CollectDaggerArgs) {
  const dim = policy.obsDim;
  const obs = new Float32Array(steps * dim);
  const actions = new Float32Array(steps * ACT_DIM);
  const weights = new Float32Array(steps);
  let o = env.reset(spec, 7, cfg);
  let n = 0;
  for (; n < steps; n++) {
    const pose = env.snapshot();
    const teacher = frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw);
    obs.set(o, n * dim);
    actions.set(teacher, n * ACT_DIM);
    weights[n] = 1;
    const act = rng.next() < beta ? teacher : policy.actWith(o, 1, rng, true).action;
    const step = env.step(act);
    o = step.obs;
    if (step.done) {
      n += 1;
      break;
    }
  }
  return {
    obs: obs.slice(0, n * dim),
    actions: actions.slice(0, n * ACT_DIM),
    weights: weights.slice(0, n),
    n,
    pooledPhysics: env.physicsWorld !== null,
  };
}

export const jobs: Record<string, JobFn> = {
  fakeDagger: makeDaggerJob(fakeCollectDagger),
  boom: () => {
    throw new Error('boom job');
  },
};
