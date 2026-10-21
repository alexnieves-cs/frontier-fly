// Dev-seed check of floor switching (src/sim/climber.ts) on two-floor buildings, tuning seeds 9101-9108.
//   vite-node scripts/study/climb-diag.ts -- [--stall 200,400,inf] [--stage 4]
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { classicalPilot, climbingPilot } from '../../src/eval/pilots';
import { createEnv } from '../../src/sim/env';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}
const stage = Number(arg('stage', '4'));
const stalls = arg('stall', 'none,inf,300').split(',');
for (const st of stalls) {
  const covs: number[] = [];
  const notes: string[] = [];
  let coll = 0;
  for (let seed = 9101; seed <= 9108; seed++) {
    const { spec, envSeed, cfg } = evalEpisodeSetup(stage, seed, 'day');
    const p =
      st === 'none'
        ? null
        : climbingPilot({}, st.startsWith('s') ? { floorSteps: (Number(st.slice(1)) * cfg.maxSteps) / spec.floors.length } : st.startsWith('f') ? { floorSteps: Number(st.slice(1)) } : { stallSteps: st === 'inf' ? Infinity : Number(st) });
    const base = st === 'none' ? classicalPilot() : null;
    (p ?? base!).reset();
    const env = await createEnv();
    let obs = env.reset(spec, envSeed, cfg);
    let cov = 0;
    let c = 0;
    const floors = new Set<number>();
    let t = 0;
    for (; t < cfg.maxSteps; t++) {
      const step = env.step((p ?? base!).act(obs, env));
      obs = step.obs;
      cov = step.info.coverage;
      c = step.info.collisions;
      floors.add(step.info.floor);
      if (step.done) break;
    }
    env.dispose();
    coll += c;
    covs.push(cov);
    notes.push(`${(cov * 100).toFixed(0)}%/${floors.size}f${p ? `/${p.climber.phaseName}/sh${p.climber.shaftsFound[0]}/t${t}` : ''}${c ? `/c${c}` : ''}`);
  }
  const mean = covs.reduce((a, b) => a + b, 0) / covs.length;
  console.log(`stage ${stage} stall=${st}: coverage ${(mean * 100).toFixed(1)}% collisions ${(coll / covs.length).toFixed(2)}  ${notes.join(' ')}`);
}
