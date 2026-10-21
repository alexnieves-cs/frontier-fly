import { beforeAll, describe, expect, it } from 'vitest';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { classicalPilot } from '../../src/eval/pilots';
import { createEnv } from '../../src/sim/env';
import { approachSpeed, CommandShaper, cornerSpeed, profileSpeed, resolveTracker, uTurnSpeed } from '../../src/sim/tracker';
import { initPhysics } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

const t = resolveTracker({});

describe('tracker speed profile', () => {
  it('slows for sharper corners and never below the floor', () => {
    expect(cornerSpeed(0, t, 3)).toBe(3);
    expect(cornerSpeed(Math.PI / 4, t, 3)).toBeGreaterThan(cornerSpeed(Math.PI / 2, t, 3));
    expect(cornerSpeed(Math.PI, t, 3)).toBeCloseTo(cornerSpeed(Math.PI / 2, t, 3));
    expect(cornerSpeed(Math.PI, resolveTracker({ accel: 0.01 }), 3)).toBe(t.minSpeed);
  });

  it('stops within the distance it is given', () => {
    for (const d of [0.5, 1, 3]) {
      const v = approachSpeed(0, d, t);
      expect(v * t.reaction + (v * v) / (2 * t.accel)).toBeCloseTo(d, 6);
    }
    expect(approachSpeed(1, 0, t)).toBe(1);
  });

  it('caps the speed before a right-angle turn ahead, not on a straight', () => {
    const straight = profileSpeed(0, 0, [[2, 0], [4, 0]], 3, t);
    const turn = profileSpeed(0, 0, [[0.5, 0], [0.5, 2]], 3, t);
    expect(straight).toBe(3);
    expect(turn).toBeLessThan(3);
    // A lookahead cut is not a corner.
    expect(profileSpeed(0, 0, [[1, 0]], 3, resolveTracker({ endShare: 0.2 }), false)).toBe(3);
  });

  it('takes the lane U-turn at the speed its half-voxel radius allows', () => {
    expect(uTurnSpeed(t, 3)).toBeCloseTo(Math.sqrt(t.accel * 0.25));
  });
});

describe('command shaper', () => {
  it('ramps the command at the acceleration limit and starts from the measured velocity', () => {
    const s = new CommandShaper();
    const [x0] = s.shape(2, 0, 4, 0.05, [1, 0]);
    expect(x0).toBeCloseTo(1.2);
    let x = x0;
    for (let i = 0; i < 10; i++) [x] = s.shape(2, 0, 4, 0.05);
    expect(x).toBe(2);
    s.hold(0, 0);
    expect(s.shape(2, 0, 4, 0.05)[0]).toBeCloseTo(0.2);
  });
});

describe('planner with tracker', () => {
  it('leaves the default planner alone and flies the quad without hitting walls on a dev building', async () => {
    const env = await createEnv();
    const { spec, envSeed, cfg } = evalEpisodeSetup(3, 9101, 'day');
    cfg.dynamics = 'quad';
    cfg.maxSteps = 400;
    const p = classicalPilot({ tracker: {} });
    expect(classicalPilot().planner.tracker).toBeNull();
    p.reset();
    let obs = env.reset(spec, envSeed, cfg);
    let info = { coverage: 0, collisions: 0 };
    for (let i = 0; i < cfg.maxSteps; i++) {
      const s = env.step(p.act(obs, env));
      obs = s.obs;
      info = s.info;
      if (s.done) break;
    }
    env.dispose();
    expect(info.coverage).toBeGreaterThan(0.1);
    expect(info.collisions).toBeLessThanOrEqual(2);
  });
});
