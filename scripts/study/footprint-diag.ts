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
    const extra: number[][] = [];
    const t0 = Date.now();
    let stepsTotal = 0;
    for (let i = 0; i < seeds; i++) {
      const job = { condition: c.id, stage: c.stage, seed: 9101 + i, lighting: 'day' as const, quad: c.quad ?? 0, pilot };
      const ep = await startEpisode(job, policy);
      const fp = new EpisodeFootprint();
      fp.start(ep.env, ep.spec, ep.cfg.maxSteps);
      let obs = ep.obs;
      let flown = 0;
      let dist = 0;
      let slow = 0;
      const rooms = new Set<number>();
      const floor0 = ep.spec.floors[0]!;
      let prev = [...ep.env.position];
      for (let t = 0; t < ep.cfg.maxSteps; t++) {
        const s = ep.env.step(ep.act(obs, ep.env));
        obs = s.obs;
        flown = s.info.coverage;
        fp.step(ep.env, t + 1, flown);
        stepsTotal += 1;
        const p = ep.env.position;
        const d = Math.hypot(p[0] - prev[0]!, p[1] - prev[1]!);
        dist += d;
        if (d < 0.2 * 0.05) slow += 1;
        prev = [...p];
        const gx = Math.floor(p[0] / ep.spec.cellSize);
        const gz = Math.floor(p[1] / ep.spec.cellSize);
        for (const room of floor0.rooms) if (gx >= room.x && gx < room.x + room.w && gz >= room.z && gz < room.z + room.d) rooms.add(room.id);
        if (s.done) break;
      }
      extra.push([dist, slow / 20, rooms.size / Math.max(1, floor0.rooms.length)]);
      const rec = fp.finish(flown);
      ep.env.dispose();
      rows.push([flown, rec.footprint['1.5']!, rec.footprint['3']!, rec.footprint['8']!]);
    }
    const m = (k: number) => rows.reduce((a, r) => a + r[k]!, 0) / rows.length;
    console.log(`${cid.padEnd(5)} ${pilot.padEnd(20)} flown ${pct(m(0))}  fp1.5 ${pct(m(1))}  fp3 ${pct(m(2))}  fp8 ${pct(m(3))}  path ${(extra.reduce((a, r) => a + r[0]!, 0) / extra.length).toFixed(0)} m  under 0.2 m/s ${(extra.reduce((a, r) => a + r[1]!, 0) / extra.length).toFixed(0)} s  rooms entered ${pct(extra.reduce((a, r) => a + r[2]!, 0) / extra.length)}%  (${Math.round((stepsTotal / (Date.now() - t0)) * 1000)} steps/s)`);
  }
}
