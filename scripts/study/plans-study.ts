// Real-plan study: every pilot on the converted Swiss Dwellings apartments, day and night.
//
//   npm run study:plans                    # all shards in parallel, then merge into docs/plans-bench.json
//   vite-node scripts/study/plans-study.ts -- --shard 0 --of 4 --out /tmp/p0.jsonl [--split dev]
//   vite-node scripts/study/plans-study.ts -- --merge /tmp/p0.jsonl ... --out docs/plans-bench.json
//
// The eval split (60 apartments, env seeds 72001+) is only measured. The converter was developed on
// the dev split (20 apartments from other buildings, env seeds 74001+); nothing here is tuned.
// Control: for every plan, a generated stage-3 building of the same grid size (seeds 73001+, dev
// 75001+), so a gap between the two conditions is about layout, not apartment size.
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { generateBuilding } from '../../src/gen/building';
import { parsePlanSet, PLAN_SET_PATH, planBuilding, planEpisodeSetup, type PlanRecord } from '../../src/plans';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { CELL } from '../../src/shared/constants';
import type { BuildingSpec } from '../../src/shared/types';
import { lightingPreset, type LightingPreset } from '../../src/sim/lighting';
import { bootstrapCi, loadPolicy, runJob, type Job, type PilotId, type Result } from './coverage-study';

export const PLAN_PILOTS: PilotId[] = ['teacher', 'classical', 'policy', 'guarded', 'waypoint', 'heuristic'];
const LIGHTING: LightingPreset[] = ['day', 'night-dark'];
const CONDITIONS = ['real', 'matched'] as const;
const MATCHED_SEED_BASE = { dev: 75001, eval: 73001 } as const;
/** Episodes per plan, condition and lighting; repeat r uses env seed + 100 r. */
const REPEATS = 3;

interface PlanJob extends Job {
  plan: string;
  rep: number;
}

interface PlanResult extends Result {
  plan: string;
  rep: number;
}

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const set = parsePlanSet(JSON.parse(readFileSync(PLAN_SET_PATH, 'utf8')));
const split = arg('split', 'eval');
const plans = set.plans.filter((p) => p.split === split);
const byId = new Map<string, PlanRecord>(plans.map((p) => [p.id, p]));

function jobs(): PlanJob[] {
  const out: PlanJob[] = [];
  for (const condition of CONDITIONS) {
    for (const plan of plans) {
      for (const lighting of LIGHTING) {
        for (let rep = 0; rep < REPEATS; rep++) {
          for (const pilot of PLAN_PILOTS) out.push({ condition, stage: 3, seed: 0, lighting, quad: 0, pilot, plan: plan.id, rep });
        }
      }
    }
  }
  return out;
}

function summarize(rows: PlanResult[]) {
  const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
  const cells = [];
  for (const condition of CONDITIONS) {
    for (const lighting of LIGHTING) {
      for (const pilot of PLAN_PILOTS) {
        const rs = rows.filter((r) => r.condition === condition && r.lighting === lighting && r.pilot === pilot);
        if (!rs.length) continue;
        const cov = rs.map((r) => r.coverage);
      // Repeats of one plan are not independent: the interval resamples plans (their mean over repeats).
      const perPlan = new Map<string, number[]>();
      for (const r of rs) {
        if (!perPlan.has(r.plan)) perPlan.set(r.plan, []);
        perPlan.get(r.plan)!.push(r.coverage);
      }
      const planMeans = [...perPlan.values()].map((xs) => mean(xs));
        cells.push({
          condition,
          lighting,
          pilot,
          episodes: rs.length,
          coverage: mean(cov),
          coverageCi: bootstrapCi(planMeans),
          collisionsPerEpisode: mean(rs.map((r) => r.collisions)),
          episodesWithCollision: rs.filter((r) => r.collisions > 0).length,
          episodesAbove60: rs.filter((r) => r.coverage >= 0.6).length,
        });
      }
    }
  }
  return cells;
}

/** Real minus size-matched generated coverage, paired by plan. */
function gaps(rows: PlanResult[]) {
  const out = [];
  for (const lighting of LIGHTING) {
    for (const pilot of PLAN_PILOTS) {
      // Mean over repeats per plan, then paired by plan.
      const pick = (condition: string) => {
        const sums = new Map<string, number[]>();
        for (const r of rows) {
          if (r.condition !== condition || r.lighting !== lighting || r.pilot !== pilot) continue;
          if (!sums.has(r.plan)) sums.set(r.plan, []);
          sums.get(r.plan)!.push(r.coverage);
        }
        return new Map([...sums].map(([id, xs]) => [id, xs.reduce((a, b) => a + b, 0) / xs.length]));
      };
      const real = pick('real');
      const matched = pick('matched');
      const diffs = [...real].filter(([id]) => matched.has(id)).map(([id, c]) => c - matched.get(id)!);
      if (!diffs.length) continue;
      out.push({ lighting, pilot, pairs: diffs.length, realMinusMatched: diffs.reduce((a, b) => a + b, 0) / diffs.length, ci: bootstrapCi(diffs) });
    }
  }
  return out;
}

/** Layout measures of a one-floor building, to say how real plans differ from generated ones. */
function layout(spec: BuildingSpec) {
  const f = spec.floors[0]!;
  let walkable = 0;
  let narrow = 0;
  let doors = 0;
  for (let z = 1; z < f.d - 1; z++) {
    for (let x = 1; x < f.w - 1; x++) {
      const i = z * f.w + x;
      if (f.cells[i] === CELL.WALL) continue;
      walkable += 1;
      if (f.cells[i] === CELL.DOOR) doors += 1;
      // Narrow: walls on both sides along x or along z (a 1-cell corridor or doorway).
      const wx = f.cells[i - 1] === CELL.WALL && f.cells[i + 1] === CELL.WALL;
      const wz = f.cells[i - f.w] === CELL.WALL && f.cells[i + f.w] === CELL.WALL;
      if (wx || wz) narrow += 1;
    }
  }
  return { rooms: f.rooms.length, walkable, doorCells: doors, narrowShare: narrow / Math.max(1, walkable), props: spec.furniture?.props.length ?? 0 };
}

function layoutComparison() {
  const mean = (xs: number[]) => xs.reduce((a, b) => a + b, 0) / Math.max(1, xs.length);
  const real = plans.map((p) => layout(planBuilding(p)!.spec));
  const matched = plans.map((p, k) => {
    const [w, d] = planBuilding(p)!.gridSize;
    return layout(generateBuilding({ seed: MATCHED_SEED_BASE[p.split] + k, floors: 1, w, d, stage: 3 }));
  });
  const summary = (xs: ReturnType<typeof layout>[]) => ({
    rooms: mean(xs.map((x) => x.rooms)),
    walkable: mean(xs.map((x) => x.walkable)),
    doorCells: mean(xs.map((x) => x.doorCells)),
    narrowShare: mean(xs.map((x) => x.narrowShare)),
    props: mean(xs.map((x) => x.props)),
  });
  return { real: summary(real), matched: summary(matched) };
}

/** Converted size of each plan, for the report's plan table. */
function planStats() {
  return plans.map((p) => {
    const c = planBuilding(p)!;
    const f = c.spec.floors[0]!;
    return {
      id: p.id,
      source: p.source,
      areas: p.areas.length,
      rooms: f.rooms.length,
      roomTypes: c.roomTypes,
      planArea: Math.round(c.planArea * 10) / 10,
      planSize: c.planSize.map((v) => Math.round(v * 10) / 10),
      grid: c.gridSize,
      walkableCells: c.walkableCells,
      reachableVoxels: c.spec.reachableVoxels,
      props: c.spec.furniture?.props.length ?? 0,
      doorCells: c.doors,
    };
  });
}

if (process.argv.includes('--merge')) {
  const outPath = arg('out', 'docs/plans-bench.json');
  const files = process.argv.slice(process.argv.indexOf('--merge') + 1).filter((a) => !a.startsWith('--') && a !== outPath && a !== split);
  const rows: PlanResult[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as PlanResult),
  );
  const report = {
    schema: 1,
    commit: execSync('git rev-parse --short HEAD').toString().trim(),
    checkpoint: arg('checkpoint', SHIPPED_CHECKPOINT_PATH),
    dataset: set.dataset,
    split,
    repeats: REPEATS,
    plans: planStats(),
    pilots: PLAN_PILOTS,
    cells: summarize(rows),
    gaps: gaps(rows),
    layout: layoutComparison(),
    episodes: rows,
  };
  writeFileSync(outPath, `${JSON.stringify(report, null, 2)}\n`);
  console.log(`wrote ${outPath} from ${rows.length} episodes`);
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `plans-${shard}.jsonl`);
  const policy = loadPolicy(arg('checkpoint', SHIPPED_CHECKPOINT_PATH));
  const mine = jobs().filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    const job = mine[i]!;
    const plan = byId.get(job.plan)!;
    const setup = planEpisodeSetup(set, plan, job.lighting, job.rep);
    if (job.condition === 'matched') {
      const [w, d] = planBuilding(plan)!.gridSize;
      const seed = MATCHED_SEED_BASE[plan.split] + plans.indexOf(plan);
      setup.spec = generateBuilding({ seed, floors: 1, w, d, stage: 3 });
      setup.envSeed = setup.spec.seed + 100 * job.rep;
      const nRooms = setup.spec.floors[0]!.rooms.length;
      setup.cfg.lighting = lightingPreset(job.lighting, nRooms);
    }
    const r = await runJob({ ...job, seed: setup.envSeed }, policy, setup);
    lines.push(JSON.stringify({ ...r, plan: job.plan, rep: job.rep }));
    if (i % 25 === 0) {
      writeFileSync(out, lines.join('\n') + '\n');
      console.log(`shard ${shard}: ${i + 1}/${mine.length} (${Math.round((Date.now() - t0) / 1000)} s)`);
    }
  }
  writeFileSync(out, lines.join('\n') + '\n');
  console.log(`shard ${shard}: done, ${mine.length} episodes in ${Math.round((Date.now() - t0) / 1000)} s`);
}
