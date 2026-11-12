import { beforeAll, describe, expect, it } from 'vitest';
import { classicalPilot } from '../../src/eval/pilots';
import { createEnv } from '../../src/sim/env';
import { QUAD_BRAKING } from '../../src/sim/lidarSafety';
import { LinearPicker, MlpPicker, PICK_DIM, PICK_FEATURES, RulePicker, initMlp, ruleWeights, type GoalPicker, type PickCandidates } from '../../src/sim/picker';
import { initPhysics } from '../../src/sim/world';
import { Cma, jacobiEigen } from '../../src/train/cma';
import { episodeSetup } from '../../src/train/pickerJob';

beforeAll(async () => {
  await initPhysics();
});

/** Fly `steps` of one quad tuning flight; returns every action and how many choices the picker made. */
async function fly(picker: GoalPicker | undefined, steps: number): Promise<{ actions: number[]; picks: number; seen: PickCandidates[] }> {
  const env = await createEnv();
  const setup = episodeSetup({ stage: 3, seed: 66301, lighting: 'day', quad: 1 });
  const seen: PickCandidates[] = [];
  const spy: GoalPicker | undefined = picker && {
    pick: (c) => {
      if (seen.length < 50) seen.push({ ...c, feats: c.feats.slice(0, c.n * PICK_DIM), ruleScore: c.ruleScore.slice(0, c.n) });
      return picker.pick(c);
    },
  };
  const p = classicalPilot({ braking: QUAD_BRAKING, tracker: {}, ...(spy ? { picker: spy } : {}) });
  p.reset();
  let obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
  const actions: number[] = [];
  for (let t = 0; t < steps; t++) {
    const a = p.act(obs, env);
    actions.push(...a);
    const s = env.step(a);
    obs = s.obs;
    if (s.done) break;
  }
  env.dispose();
  return { actions, picks: p.planner.picks, seen };
}

describe('goal picker', () => {
  it('the rule picker flies exactly like the planner without one', async () => {
    const a = await fly(undefined, 600);
    const b = await fly(new RulePicker(), 600);
    expect(b.picks).toBeGreaterThan(20);
    expect(b.actions).toEqual(a.actions);
  }, 60_000);

  it('the rule weights pick like the rule, and features are finite and bounded', async () => {
    const lin = new LinearPicker(ruleWeights());
    const r = await fly(new RulePicker(), 600);
    expect(r.seen.length).toBeGreaterThan(10);
    for (const c of r.seen) {
      expect(lin.pick(c)).toBe(new RulePicker().pick(c));
      for (const v of c.feats) {
        expect(Number.isFinite(v)).toBe(true);
        expect(Math.abs(v)).toBeLessThan(10);
      }
      if (c.current >= 0) expect(c.feats[c.current * PICK_DIM + PICK_FEATURES.indexOf('isCurrent')]).toBe(1);
    }
  }, 60_000);

  it('an MLP picker records one row per real choice', async () => {
    let s = 3;
    const rand = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    const mlp = new MlpPicker(initMlp(8, rand), 1, rand);
    const rec = mlp.recorder();
    const c: PickCandidates = { n: 3, feats: new Float32Array(3 * PICK_DIM).map(() => rand()), ruleScore: new Float64Array([1, 2, 3]), current: -1 };
    rec.now(7, 0.25);
    const k = mlp.pick(c);
    mlp.pick({ ...c, n: 1 });
    const t = rec.take();
    expect(t.steps).toEqual(Int32Array.from([7]));
    expect(t.chosen[0]).toBe(k);
    expect(t.logp[0]).toBeLessThan(0);
  });
});

describe('CMA-ES', () => {
  it('diagonalizes a symmetric matrix', () => {
    const M = [
      [4, 1, 0],
      [1, 3, 1],
      [0, 1, 2],
    ];
    const { values, vectors } = jacobiEigen(M);
    for (let j = 0; j < 3; j++) {
      for (let i = 0; i < 3; i++) {
        const mv = M[i]!.reduce((a, m, k) => a + m * vectors[k]![j]!, 0);
        expect(mv).toBeCloseTo(values[j]! * vectors[i]![j]!, 8);
      }
    }
  });

  it('finds the minimum of a shifted, ill-conditioned quadratic', () => {
    const target = [1, -2, 3, 0.5];
    const f = (x: number[]) => x.reduce((a, v, i) => a + 10 ** i * (v - target[i]!) ** 2, 0);
    const cma = new Cma([0, 0, 0, 0], 1, { seed: 3 });
    for (let g = 0; g < 300; g++) {
      const xs = cma.ask();
      cma.tell(xs, xs.map(f));
    }
    cma.mean.forEach((v, i) => expect(v).toBeCloseTo(target[i]!, 3));
  });
});
