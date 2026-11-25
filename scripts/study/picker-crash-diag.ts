// Crash diagnosis for the quad goal picker (docs/QUAD_PICKER.md, "Making it the default").
// Flies the diag practice block (66701-66764, both lightings) and prints the last steps before each crash.
//   npx vite-node scripts/study/picker-crash-diag.ts -- [picker.json|rule] [set]
import { readFileSync } from 'node:fs';
import { classicalPilot } from '../../src/eval/pilots';
import { RNG } from '../../src/shared/rng';
import { ExplorerEnv } from '../../src/sim/env';
import { QUAD_BRAKING } from '../../src/sim/lidarSafety';
import { LIDAR_MAX } from '../../src/shared/constants';
import * as C from '../../src/shared/constants';
import { initPhysics } from '../../src/sim/world';
import { episodeSetup, makePicker, type PickerSpec } from '../../src/train/pickerJob';

const a = process.argv.slice(2).filter((x) => x !== '--');
const p = a[0] ?? 'rule';
const spec: PickerSpec = p === 'rule' ? { kind: 'rule' } : (JSON.parse(readFileSync(p, 'utf8')) as { spec: PickerSpec }).spec;
const [lo, hi] = (a[1] ?? '66701-66764').split('-').map(Number) as [number, number];
await initPhysics();
const env = new ExplorerEnv();
let crashes = 0;
let n = 0;
for (let seed = lo; seed <= hi; seed++)
  for (const lighting of ['day', 'night-dark'] as const) {
    const e = { stage: 3, seed, lighting, quad: 1 };
    const setup = episodeSetup(e);
    const picker = makePicker(spec, new RNG(1, 'picker'));
    const pl = classicalPilot({ braking: QUAD_BRAKING, tracker: {}, ...(spec.planner ?? {}), ...(picker ? { picker } : {}) });
    pl.reset();
    let obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
    const hist: string[] = [];
    let lastGoal = -1;
    let sinceGoal = 0;
    n++;
    for (let t = 0; t < setup.cfg.maxSteps; t++) {
      const act = pl.act(obs, env);
      const s = env.snapshot();
      const P = pl.planner as unknown as { goal: number; sinceReplan: number };
      if (P.goal !== lastGoal) {
        lastGoal = P.goal;
        sinceGoal = 0;
      } else sinceGoal++;
      let minL = Infinity;
      for (let i = 0; i < C.LIDAR_H; i++) minL = Math.min(minL, obs[C.OBS.lidar + i]! * LIDAR_MAX);
      const sp = Math.hypot(s.vel[0], s.vel[1]);
      hist.push(
        `t${t} v=(${s.vel[0].toFixed(2)},${s.vel[1].toFixed(2)},${s.vel[2].toFixed(2)}) |v|=${sp.toFixed(2)} z=${s.pos[2].toFixed(2)} act=(${Array.from(act, (x) => x.toFixed(2)).join(',')}) minLidar=${minL.toFixed(2)} sinceGoal=${sinceGoal}`,
      );
      if (hist.length > 12) hist.shift();
      const step = env.step(act);
      obs = step.obs;
      if (step.done) {
        if (step.info.crashed) {
          crashes++;
          console.log(`CRASH ${seed} ${lighting} t=${t} cov=${step.info.coverage.toFixed(2)}`);
          for (const h of hist) console.log('   ' + h);
        }
        break;
      }
    }
  }
console.log(`${p}: ${crashes}/${n} crashes`);
