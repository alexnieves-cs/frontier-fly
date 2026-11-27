/** How robust is each expert variant to small command noise? (docs/QUAD_STUDENT.md, practice seeds only.) */
import { availableParallelism } from 'node:os';
import { NodeRolloutPool } from '../../src/train/nodePool';
import type { DaggerChunk, DaggerPayload, ExpertKnobs } from '../../src/train/quadDistillJob';

const variants: Record<string, ExpertKnobs | undefined> = JSON.parse(process.argv[2] ?? '{"default":null}');
const noise = Number(process.argv[3] ?? 0.08);
const n = Number(process.argv[4] ?? 24);
const workers = availableParallelism();
const pool = new NodeRolloutPool({ workers, envsPerWorker: 1, seed: 1, worldPool: { stages: [0], perStage: 1 }, jobModules: ['src/train/quadDistillJob.ts'], autoDagger: false });
await pool.init();
const eps = Array.from({ length: n }, (_, i) => [{ stage: 3, seed: 180201 + i, lighting: 'day' as const, quad: 1 }, { stage: 3, seed: 180201 + i, lighting: 'night-dark' as const, quad: 1 }]).flat();
for (const [name, expert] of Object.entries(variants)) {
  for (const nz of [0, noise]) {
    const parts = Array.from({ length: workers }, (_, w) => eps.filter((_, i) => i % workers === w));
    const res = await Promise.all(parts.map((list, w) => pool.runJob<DaggerChunk>('quadDagger', { episodes: list, student: null, beta: 1, noise: nz, vzNoise: nz > 0 ? 0.1 : 0, crashBoost: 1, crashLookback: 0, divergeBoost: 1, expert: expert ?? undefined, seed: 7 + w } satisfies DaggerPayload, w)));
    const all = res.flatMap((r) => r.episodes);
    const cov = all.reduce((a, e) => a + e.coverage, 0) / all.length;
    console.log(`${name} noise ${nz}: ${(cov * 100).toFixed(1)}% crashes ${all.filter((e) => e.crashed).length}/${all.length}`);
  }
}
await pool.close();
