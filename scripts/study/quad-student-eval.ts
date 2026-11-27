/**
 * Fly a quad-student checkpoint (alone and behind the braking shield) and the default quad pilot
 * on the same day and night quad episodes, paired by seed (docs/QUAD_STUDENT.md).
 *
 * Usage: vite-node scripts/study/quad-student-eval.ts -- --ckpt best.bin --base 180001 --n 40
 *          [--no-expert] [--planner] [--out result.json]
 *
 * Practice seeds are 180001-180400. The gate (181001-181160) and the report block (185001-185332)
 * are flown once each, by checkpoints already picked on practice.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import { parseCheckpoint } from '../../src/rl/checkpoint';
import { paddedWeights } from '../../src/rl/quadStudent';
import { NodeRolloutPool } from '../../src/train/nodePool';
import type { PickerEpisode } from '../../src/train/pickerJob';
import type { EvalEpisode, ExpertKnobs, StudentWeights } from '../../src/train/quadDistillJob';

const args = process.argv.slice(2);
const arg = (name: string) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const pct = (v: number) => `${(v * 100).toFixed(1)}%`;

const base = Number(arg('base') ?? 180001);
const n = Number(arg('n') ?? 40);
const workers = Number(arg('workers') ?? availableParallelism());
const eps: PickerEpisode[] = [];
for (let i = 0; i < n; i++) for (const lighting of ['day', 'night-dark'] as const) eps.push({ stage: 3, seed: base + i, lighting, quad: 1 });

const pool = new NodeRolloutPool({ workers, envsPerWorker: 1, seed: 1, worldPool: { stages: [0], perStage: 1 }, jobModules: ['src/train/quadDistillJob.ts'], autoDagger: false });
await pool.init();

async function fly(student: StudentWeights | null, shield: boolean, expert?: ExpertKnobs): Promise<EvalEpisode[]> {
  const parts: PickerEpisode[][] = Array.from({ length: workers }, () => []);
  eps.forEach((e, i) => parts[i % workers]!.push(e));
  const out = await Promise.all(parts.map((list, w) => (list.length ? pool.runJob<EvalEpisode[]>('quadEval', { episodes: list, student, shield, expert }, w) : Promise.resolve([]))));
  return out.flat();
}

function line(name: string, rs: EvalEpisode[]) {
  const steps = rs.reduce((a, r) => a + r.steps, 0);
  const s = {
    n: rs.length,
    coverage: rs.reduce((a, r) => a + r.coverage, 0) / rs.length,
    crashes: rs.filter((r) => r.crashed).length,
    interventionRate: rs.reduce((a, r) => a + r.interventions, 0) / Math.max(1, steps),
  };
  console.log(`${name.padEnd(18)} ${pct(s.coverage)} crashes ${s.crashes}/${s.n}${s.interventionRate > 0 ? ` interventions ${pct(s.interventionRate)}` : ''}`);
  return { ...s, episodes: rs };
}

const result: Record<string, unknown> = { base, n, flights: eps.length };
const ckPath = arg('ckpt');
if (ckPath) {
  const ck = parseCheckpoint(readFileSync(ckPath));
  const student = { hidden: ck.hidden, weights: paddedWeights(ck) };
  result.alone = line('student alone', await fly(student, false));
  result.shielded = line('student + shield', await fly(student, true));
}
if (!args.includes('--no-expert')) result.expert = line('default quad pilot', await fly(null, false));
if (args.includes('--planner')) result.planner = line('planner only', await fly(null, false, { noPicker: true }));
const out = arg('out');
if (out) writeFileSync(out, JSON.stringify(result, null, 1));
await pool.close();
