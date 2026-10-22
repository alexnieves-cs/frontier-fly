// Footprint-coverage sanity check on the dev buildings (9101-9108): flown vs footprint coverage per pilot.
//
//   vite-node scripts/study/footprint-diag.ts -- [--seeds 8] [--pilots classical,classicalTrack,teacher] [--conditions hard,quad]
import { EpisodeFootprint } from '../../src/eval/footprint';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { CONDITIONS, startEpisode, type PilotId } from './coverage-study';
import { loadPolicy } from './common';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const seeds = Number(arg('seeds', '8'));
const pilots = arg('pilots', 'teacher,classical,classicalTrack,policy').split(',') as PilotId[];
const conditions = arg('conditions', 'hard,quad').split(',');
const policy = loadPolicy(SHIPPED_CHECKPOINT_PATH);
const pct = (v: number) => `${(v * 100).toFixed(1)}`;
for (const cid of conditions) {
  const c = CONDITIONS.find((x) => x.id === cid)!;
  for (const pilot of pilots) {
    const rows: number[][] = [];
    const t0 = Date.now();
    let stepsTotal = 0;
    for (let i = 0; i < seeds; i++) {
      const job = { condition: c.id, stage: c.stage, seed: 9101 + i, lighting: 'day' as const, quad: c.quad ?? 0, pilot };
      const ep = await startEpisode(job, policy);
      const fp = new EpisodeFootprint();
      fp.start(ep.env, ep.spec, ep.cfg.maxSteps);
      let obs = ep.obs;
      let flown = 0;
      for (let t = 0; t < ep.cfg.maxSteps; t++) {
        const s = ep.env.step(ep.act(obs, ep.env));
        obs = s.obs;
        flown = s.info.coverage;
        fp.step(ep.env, t + 1, flown);
        stepsTotal += 1;
        if (s.done) break;
      }
      const rec = fp.finish(flown);
      ep.env.dispose();
      rows.push([flown, rec.footprint['1.5']!, rec.footprint['3']!, rec.footprint['8']!]);
    }
    const m = (k: number) => rows.reduce((a, r) => a + r[k]!, 0) / rows.length;
    console.log(`${cid.padEnd(5)} ${pilot.padEnd(20)} flown ${pct(m(0))}  fp1.5 ${pct(m(1))}  fp3 ${pct(m(2))}  fp8 ${pct(m(3))}  (${Math.round((stepsTotal / (Date.now() - t0)) * 1000)} steps/s)`);
  }
}
