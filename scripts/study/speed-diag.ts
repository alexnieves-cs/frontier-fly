/**
 * Dev-seed sweep of the sensor planner's speed cap in the coverage env
 * (tuning seeds 9101-9108 only). Prints coverage, collisions and energy per
 * speed and pilot setting.
 *
 *   npx vite-node scripts/study/speed-diag.ts -- --stage 3 --speeds 2,3,4,5 [--quad] [--variants filter,mom] [--mom 1]
 */
import { createEnv } from '../../src/sim/env';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { classicalPilot } from '../../src/eval/pilots';
import { QUAD_BRAKING, type SensorPlannerOptions } from '../../src/sim/sensorPlanner';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const MOM = Number(arg('mom', '1'));
const stage = Number(arg('stage', '3'));
const speeds = arg('speeds', '2,3,4,5').split(',').map(Number);
const seeds = Array.from({ length: Number(arg('seeds', '8')) }, (_, i) => 9101 + i);
const lighting = arg('lighting', 'day') as 'day' | 'night-dark';
const allVariants: Record<string, (v: number) => SensorPlannerOptions> = {
  filter: (v) => ({ speed: v }),
  brake: (v) => ({ speed: v, braking: QUAD_BRAKING }),
  noFace: (v) => ({ speed: v, faceTravel: false }),
  mom: (v) => ({ speed: v, momentum: MOM }),
  momNoFace: (v) => ({ speed: v, faceTravel: false, momentum: MOM }),
  momBrake: (v) => ({ speed: v, braking: QUAD_BRAKING, momentum: MOM }),
};

const quad = process.argv.includes('--quad');
const only = arg('variants', '').split(',').filter(Boolean);
const variants = Object.fromEntries(Object.entries(allVariants).filter(([k]) => !only.length || only.includes(k)));
const env = await createEnv();
for (const [name, opts] of Object.entries(variants)) {
  for (const v of speeds) {
    let cov = 0;
    let coll = 0;
    let wh = 0;
    for (const seed of seeds) {
      const { spec, envSeed, cfg } = evalEpisodeSetup(stage, seed, lighting);
      cfg.speedH = v;
      if (quad) cfg.dynamics = 'quad';
      const pilot = classicalPilot(opts(v));
      pilot.reset();
      let obs = env.reset(spec, envSeed, cfg);
      let info = { coverage: 0, collisions: 0, energyWh: 0 };
      for (let t = 0; t < cfg.maxSteps; t++) {
        const s = env.step(pilot.act(obs, env));
        obs = s.obs;
        info = s.info;
        if (s.done) break;
      }
      cov += info.coverage;
      coll += info.collisions;
      wh += info.energyWh;
    }
    const n = seeds.length;
    console.log(`${name} v=${v}: coverage ${((cov / n) * 100).toFixed(1)}%  collisions/ep ${(coll / n).toFixed(2)}  energy ${(wh / n).toFixed(2)} Wh  coverage/Wh ${((cov / wh) * 100).toFixed(1)}%`);
  }
}
env.dispose();
