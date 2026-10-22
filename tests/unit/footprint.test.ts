import { beforeAll, describe, expect, it } from 'vitest';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { CoverageCurve, curveArea, EpisodeFootprint, FootprintCoverage, timeTo } from '../../src/eval/footprint';
import { classicalPilot } from '../../src/eval/pilots';
import { createEnv } from '../../src/sim/env';
import { initPhysics } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

/** Fly the sensor-only planner for `steps` and return flown coverage and the footprint record. */
async function fly(stage: number, seed: number, steps: number) {
  const env = await createEnv();
  const { spec, envSeed, cfg } = evalEpisodeSetup(stage, seed, 'day');
  let obs = env.reset(spec, envSeed, cfg);
  const pilot = classicalPilot();
  pilot.reset();
  const fp = new EpisodeFootprint();
  fp.start(env, spec, steps);
  let flown = 0;
  for (let t = 0; t < steps; t++) {
    const s = env.step(pilot.act(obs, env));
    obs = s.obs;
    flown = s.info.coverage;
    fp.step(env, t + 1, flown);
    if (s.done) break;
  }
  const rec = fp.finish(flown);
  env.dispose();
  return { flown, rec };
}

describe('sensor-footprint coverage', () => {
  it('grows with the radius, is stopped by walls, and covers more than the flown voxels', async () => {
    const env = await createEnv();
    const { spec, envSeed, cfg } = evalEpisodeSetup(3, 9101, 'day');
    env.reset(spec, envSeed, cfg);
    const fp = new FootprintCoverage();
    fp.reset(spec);
    const p = env.position;
    fp.observe(env.world!, p[0], p[1], p[2]);
    const [r15, r3, r8] = fp.all();
    expect(r15!).toBeGreaterThan(0);
    expect(r15!).toBeLessThan(r3!);
    expect(r3!).toBeLessThan(r8!);
    // One look from the start cannot see every room through the walls.
    expect(r8!).toBeLessThan(0.9);
    env.dispose();
  });

  it('scores a short flight above its flown coverage, deterministically', async () => {
    const a = await fly(3, 9102, 300);
    const b = await fly(3, 9102, 300);
    expect(a.rec).toEqual(b.rec);
    expect(a.rec.footprint['3']!).toBeGreaterThan(a.flown);
    expect(a.rec.curve.flown).toHaveLength(20);
    expect(a.rec.curve.footprint).toHaveLength(20);
    // Coverage never falls, and the curve's last point is the final value.
    for (let k = 1; k < 20; k++) expect(a.rec.curve.footprint[k]!).toBeGreaterThanOrEqual(a.rec.curve.footprint[k - 1]!);
    expect(a.rec.curve.footprint[19]).toBe(a.rec.footprint['3']);
  });
});

describe('coverage curves', () => {
  it('samples at fixed fractions of the budget and holds the last value after an early end', () => {
    const c = new CoverageCurve(100, 4);
    for (let s = 1; s <= 60; s++) c.step(s, [s / 100]);
    c.finish([0.6]);
    expect(c.series(0)).toEqual([0.25, 0.5, 0.6, 0.6]);
  });

  it('measures area and the time to a level', () => {
    expect(curveArea([1, 1, 1, 1])).toBe(1);
    expect(curveArea([0, 0.5, 1, 1])).toBeCloseTo(0.625);
    // Samples at 30, 60, 90 and 120 s: 0.4 at 60 s and 0.6 at 90 s, so 0.5 half way, at 75 s.
    expect(timeTo([0.2, 0.4, 0.6, 0.8], 0.5, 120)).toBeCloseTo(75);
    expect(timeTo([0.1, 0.2], 0.5, 60)).toBeNull();
  });
});
