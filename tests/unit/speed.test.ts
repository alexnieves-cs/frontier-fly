import { beforeAll, describe, expect, it } from 'vitest';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { classicalPilot } from '../../src/eval/pilots';
import { DEFAULT_BATTERY, hoverPowerW } from '../../src/mission/autonomy/battery';
import { createEnv } from '../../src/sim/env';
import { guardAction } from '../../src/sim/sensorPlanner';
import { initPhysics } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

/** Fly full forward for `steps` and return the top horizontal speed. */
async function topSpeed(speedH: number | undefined, dynamics: 'kinematic' | 'quad', steps = 60): Promise<number> {
  const env = await createEnv();
  const { spec, envSeed, cfg } = evalEpisodeSetup(0, 9101, 'day');
  cfg.dynamics = dynamics;
  if (speedH !== undefined) cfg.speedH = speedH;
  env.reset(spec, envSeed, cfg);
  let top = 0;
  for (let t = 0; t < steps; t++) {
    const s = env.step(new Float32Array([1, 0, 0, 0]));
    const v = env.snapshot().vel;
    top = Math.max(top, Math.hypot(v[0], v[1]));
    if (s.done || s.info.collisions > 0) break;
  }
  env.dispose();
  return top;
}

describe('speed cap', () => {
  it('defaults to 2 m/s and scales a full action to the episode cap', async () => {
    expect(await topSpeed(undefined, 'kinematic')).toBeLessThanOrEqual(2.001);
    expect(await topSpeed(4, 'kinematic')).toBeGreaterThan(3);
    expect(await topSpeed(4, 'quad')).toBeGreaterThan(2.3);
  });

  it('keeps the guard in physical units at any cap', () => {
    const lidar = new Float32Array(32).fill(8);
    const a = guardAction([0.5, 0, 0, 0], 1.15, 0, lidar, undefined, undefined, undefined, 4);
    expect(a[0]).toBeCloseTo(0.5, 5);
  });
});

describe('energy', () => {
  it('meters hover at the battery model hover power', async () => {
    const env = await createEnv();
    const { spec, envSeed, cfg } = evalEpisodeSetup(0, 9101, 'day');
    env.reset(spec, envSeed, cfg);
    let wh = 0;
    for (let t = 0; t < 200; t++) wh = env.step(new Float32Array(4)).info.energyWh;
    env.dispose();
    // 10 s of hover (the drone settles to its flight height first, which costs a little more).
    const expected = (hoverPowerW(DEFAULT_BATTERY) * 10) / 3600;
    expect(wh).toBeGreaterThan(expected * 0.98);
    expect(wh).toBeLessThan(expected * 1.1);
  });

  it('a momentum-aware planner flies a tuning building without collisions', async () => {
    const env = await createEnv();
    const { spec, envSeed, cfg } = evalEpisodeSetup(3, 9103, 'day');
    cfg.speedH = 4;
    const pilot = classicalPilot({ speed: 4, momentum: 1 });
    pilot.reset();
    let obs = env.reset(spec, envSeed, cfg);
    let info = { coverage: 0, collisions: 0, energyWh: 0 };
    for (let t = 0; t < 600; t++) {
      const s = env.step(pilot.act(obs, env));
      obs = s.obs;
      info = s.info;
      if (s.done) break;
    }
    env.dispose();
    expect(info.collisions).toBe(0);
    expect(info.coverage).toBeGreaterThan(0.2);
    expect(info.energyWh).toBeGreaterThan(0);
  });
});
