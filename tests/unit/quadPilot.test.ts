import { readFileSync } from 'node:fs';
import { beforeAll, describe, expect, it } from 'vitest';
import { classicalPilot, quadPilot } from '../../src/eval/pilots';
import { createEnv } from '../../src/sim/env';
import { MlpPicker } from '../../src/sim/picker';
import { QUAD_PICKER_BRAKING } from '../../src/sim/quadPilot';
import { initPhysics } from '../../src/sim/world';
import { episodeSetup, type PickerSpec } from '../../src/train/pickerJob';

beforeAll(async () => {
  await initPhysics();
});

async function actions(p: ReturnType<typeof classicalPilot>, steps: number): Promise<{ actions: number[]; picks: number }> {
  const env = await createEnv();
  const setup = episodeSetup({ stage: 3, seed: 66301, lighting: 'day', quad: 1 });
  p.reset();
  let obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
  const out: number[] = [];
  for (let t = 0; t < steps; t++) {
    const a = p.act(obs, env);
    out.push(...a);
    const s = env.step(a);
    obs = s.obs;
    if (s.done) break;
  }
  env.dispose();
  return { actions: out, picks: p.planner.picks };
}

describe('default quad pilot', () => {
  it('flies exactly the frozen study spec (results/picker/default.json)', async () => {
    const spec = (JSON.parse(readFileSync('results/picker/default.json', 'utf8')) as { spec: Extract<PickerSpec, { kind: 'mlp' }> }).spec;
    expect(spec.planner?.braking).toEqual(QUAD_PICKER_BRAKING);
    const ref = classicalPilot({ tracker: {}, ...spec.planner, picker: new MlpPicker(spec.weights, 0) });
    const a = await actions(quadPilot(), 300);
    const b = await actions(ref, 300);
    expect(a.picks).toBeGreaterThan(0);
    expect(a.actions).toEqual(b.actions);
  }, 60_000);
});
