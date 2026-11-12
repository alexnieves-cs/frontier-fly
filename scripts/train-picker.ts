// Goal-picker study on the quadrotor (docs/QUAD_PICKER.md): the sensor planner with braking filter and
// trajectory tracker flies; a picker chooses its next frontier goal.
//
//   npm run train:picker -- --mode eval --picker rule --set tune
//   npm run train:picker -- --mode cma --arm planner --gens 60 --out-dir /tmp/picker-planner
//   npm run train:picker -- --mode cma --arm linear --gens 60 --out-dir /tmp/picker-linear
//   npm run train:picker -- --mode ppo --hidden 32 --out-dir /tmp/picker-mlp
//
// Seeds (roadmap/next-wave-tracks.md): training flights draw quad buildings 66001-66299; every
// selection uses the tuning block 66301-66332, day and night; the report block 89001-89332 is flown
// once, with --set report --final, after everything is frozen.
import { mkdirSync, readFileSync, writeFileSync, appendFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import { join } from 'node:path';
import { RNG } from '../src/shared/rng';
import { Cma } from '../src/train/cma';
import { NodeRolloutPool } from '../src/train/nodePool';
import type { PickerEpisode, PickerEpisodeResult, PickerSpec, PlannerKnobs } from '../src/train/pickerJob';
import { initMlp, PICK_DIM, PICK_FEATURES, ruleWeights, type MlpWeights } from '../src/sim/picker';
import { buildDecisions, initCritic, PickerLearner, PPO_PICKER_DEFAULTS, type PpoPickerConfig } from '../src/train/pickerPpo';

const args = process.argv.slice(2);
export function arg(name: string): string | undefined {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
}
export const num = (name: string, d: number) => Number(arg(name) ?? d);
export const has = (name: string) => args.includes(`--${name}`);

export const TRAIN_SEEDS = [66001, 66299] as const;
export const TUNE_SEEDS = [66301, 66332] as const;
export const REPORT_SEEDS = [89001, 89332] as const;
/** A second selection block, for choosing among finished runs without reusing the tuning block they were picked on. */
export const VAL2_SEEDS = [66401, 66464] as const;
const LIGHTS = ['day', 'night-dark'] as const;

export function seedSet(set: string): PickerEpisode[] {
  const block = set === 'tune' ? TUNE_SEEDS : set === 'val2' ? VAL2_SEEDS : set === 'report' ? REPORT_SEEDS : set === 'published' ? ([50001, 50030] as const) : null;
  if (!block) throw new Error(`unknown set ${set}`);
  if (set === 'report' && !has('final')) throw new Error('the report block is flown once, after everything is frozen: pass --final');
  const out: PickerEpisode[] = [];
  for (let s = block[0]; s <= block[1]; s++) for (const lighting of LIGHTS) out.push({ stage: 3, seed: s, lighting, quad: 1 });
  return out;
}

export function trainBatch(rng: RNG, n: number): PickerEpisode[] {
  const out: PickerEpisode[] = [];
  for (let i = 0; i < n; i++) {
    const seed = TRAIN_SEEDS[0] + Math.floor(rng.next() * (TRAIN_SEEDS[1] - TRAIN_SEEDS[0] + 1));
    out.push({ stage: 3, seed, lighting: rng.next() < 0.5 ? 'day' : 'night-dark', quad: 1 });
  }
  return out;
}

/** Fly every (spec, episode) pair, spread over the workers longest-first; results in input order. */
export async function fly(pool: NodeRolloutPool, workers: number, items: { spec: PickerSpec; episode: PickerEpisode }[], seed = 1): Promise<PickerEpisodeResult[]> {
  const per: { i: number }[][] = Array.from({ length: workers }, () => []);
  items.forEach((_, i) => per[i % workers]!.push({ i }));
  const parts = await Promise.all(
    per.map((list, w) =>
      list.length ? pool.runJob<PickerEpisodeResult[]>('pickerEpisodes', { items: list.map(({ i }) => items[i]!), seed: seed * 1000 + w }, w) : Promise.resolve([]),
    ),
  );
  const out = new Array<PickerEpisodeResult>(items.length);
  per.forEach((list, w) => list.forEach(({ i }, k) => (out[i] = parts[w]![k]!)));
  return out;
}

export const CRASH_COST = 0.1;
export interface Summary {
  n: number;
  coverage: number;
  crashes: number;
  score: number;
}
export function summarize(rs: PickerEpisodeResult[]): Summary {
  const n = rs.length;
  const coverage = rs.reduce((a, r) => a + r.coverage, 0) / n;
  const crashes = rs.filter((r) => r.crashed).length;
  return { n, coverage, crashes, score: coverage - (CRASH_COST * crashes) / n };
}
const pct = (s: Summary) => `${(s.coverage * 100).toFixed(1)}% crashes ${s.crashes}/${s.n}`;

/** Search space of each CMA arm: x (CMA coordinates, start 0) to a picker spec. */
interface Arm {
  dim: number;
  names: string[];
  spec(x: number[]): PickerSpec;
}
/** --init-planner <best.json of the planner arm>: the linear arm searches around that tuned planner. */
const basePlanner = (): PlannerKnobs => {
  const p = arg('init-planner');
  if (!p) return {};
  const b = JSON.parse(readFileSync(p, 'utf8')) as { spec: PickerSpec };
  return b.spec.planner ?? {};
};
const replanOf = (x: number) => Math.max(1, Math.min(40, Math.round(5 * 2 ** x)));
const ARMS: Record<string, () => Arm> = {
  // The planner's own rule with its three selection knobs tuned on the quad.
  planner: () => ({
    dim: 3,
    names: ['edgeBonus', 'momentum', 'log2(replanEvery/5)'],
    spec: (x) => ({ kind: 'rule', planner: { edgeBonus: Math.max(0, 0.3 + 0.3 * x[0]!), momentum: Math.max(0, 1.5 * x[1]!), replanEvery: replanOf(x[2]!) } }),
  }),
  // A linear score over every per-candidate feature, starting from the rule (context features are
  // the same for every candidate, so they cannot change a linear choice and are left out).
  linear: () => {
    const perCand = PICK_FEATURES.map((f, i) => ({ f, i })).filter(({ f }) => !['timeLeft', 'speed', 'nCand', 'bias'].includes(f));
    const base = ruleWeights();
    return {
      dim: perCand.length + 1,
      names: [...perCand.map(({ f }) => f), 'log2(replanEvery/5)'],
      spec: (x) => {
        const w = Array.from(base);
        perCand.forEach(({ f, i }, k) => (w[i] = base[i]! + (f === 'ruleScore' ? 10 : 3) * x[k]!));
        return { kind: 'linear', w, planner: { ...basePlanner(), replanEvery: replanOf(x[perCand.length]! + Math.log2((basePlanner().replanEvery ?? 5) / 5)) } };
      },
    };
  },
};

/**
 * Decision-level PPO for the MLP picker. It starts as the planner rule (zero output layer over a
 * linear skip set to the rule weights, or --init <best.json> of a linear CMA run), so iteration 0
 * flies like the planner (or that linear picker) and learning only has to find improvements.
 */
async function ppo(pool: NodeRolloutPool, workers: number, outDir: string): Promise<void> {
  const seed = num('seed', 1);
  const rng = new RNG(seed, 'picker-ppo');
  const rand = () => rng.next();
  let lin: ArrayLike<number> = ruleWeights();
  // --init: a linear best.json (its weights and planner settings) or a rule best.json of the
  // tuned-planner arm (the rule weights over that planner's settings).
  let planner: PlannerKnobs | undefined;
  const init = arg('init');
  if (init) {
    const b = JSON.parse(readFileSync(init, 'utf8')) as { spec: PickerSpec };
    if (b.spec.kind === 'linear') lin = b.spec.w;
    else if (b.spec.kind !== 'rule') throw new Error('--init wants a linear or rule best.json');
    planner = b.spec.planner;
  }
  const scale = num('logit-scale', 1);
  const actor: MlpWeights = initMlp(num('hidden', 32), rand, Array.from(lin, (v) => v * scale));
  const critic = initCritic(num('critic-hidden', 32), rand);
  const cfg: PpoPickerConfig = {
    ...PPO_PICKER_DEFAULTS,
    lr: num('lr', PPO_PICKER_DEFAULTS.lr),
    entropy: num('entropy', PPO_PICKER_DEFAULTS.entropy),
    crashCost: num('crash-cost', PPO_PICKER_DEFAULTS.crashCost),
    epochs: num('epochs', PPO_PICKER_DEFAULTS.epochs),
    gamma: num('gamma', PPO_PICKER_DEFAULTS.gamma),
  };
  const learner = new PickerLearner(actor, critic, cfg, rand);
  const iters = num('iters', 200);
  const batch = num('batch', 32);
  const evalEvery = num('eval-every', 10);
  const warm = num('critic-warmup', 3);
  const tune = seedSet('tune');
  const log = join(outDir, 'log.jsonl');
  writeFileSync(log, '');
  let best = -Infinity;
  const evaluate = async (it: number) => {
    const spec: PickerSpec = { kind: 'mlp', weights: actor, temperature: 0, planner };
    const s = summarize(await fly(pool, workers, tune.map((episode) => ({ spec, episode }))));
    appendFileSync(log, JSON.stringify({ iter: it, tune: s }) + '\n');
    console.log(`iter ${it} tune ${pct(s)}`);
    if (s.score > best) {
      best = s.score;
      writeFileSync(join(outDir, 'best.json'), JSON.stringify({ arm: 'mlp', iter: it, tune: s, spec: { ...spec, weights: actor } }, null, 1));
    }
    writeFileSync(join(outDir, `iter-${it}.json`), JSON.stringify({ arm: 'mlp', iter: it, tune: s, spec: { ...spec, weights: actor } }));
  };
  await evaluate(0);
  for (let it = 1; it <= iters; it++) {
    const t0 = Date.now();
    const spec: PickerSpec = { kind: 'mlp', weights: actor, temperature: 1, record: true, planner };
    const rs = await fly(pool, workers, trainBatch(rng, batch).map((episode) => ({ spec, episode })), it);
    const d = buildDecisions(rs, critic, cfg);
    const savedLr = cfg.lr;
    // Critic warm-up: the first iterations fit the critic only (actor step size zero).
    const st = it <= warm ? learner.updateCriticOnly(d) : learner.update(d);
    cfg.lr = savedLr;
    const s = summarize(rs);
    console.log(
      `iter ${it} train ${pct(s)} decisions ${d.m} kl ${st.kl.toFixed(4)} clip ${st.clipFrac.toFixed(3)} ent ${st.entropy.toFixed(3)} vloss ${st.vLoss.toFixed(2)} (${((Date.now() - t0) / 1000).toFixed(0)} s)`,
    );
    appendFileSync(log, JSON.stringify({ iter: it, train: s, decisions: d.m, ...st }) + '\n');
    if (it % evalEvery === 0 || it === iters) await evaluate(it);
  }
}

async function main(): Promise<void> {
  const mode = arg('mode') ?? 'eval';
  const workers = num('workers', Math.max(1, availableParallelism()));
  const pool = new NodeRolloutPool({ workers, envsPerWorker: 1, seed: 5, worldPool: { stages: [0], perStage: 1 }, jobModules: ['src/train/pickerJob.ts'], autoDagger: false });
  await pool.init();
  const outDir = arg('out-dir') ?? '/tmp/frontier-fly-picker';
  mkdirSync(outDir, { recursive: true });
  try {
    if (mode === 'eval') {
      const p = arg('picker') ?? 'rule';
      const spec: PickerSpec = p === 'rule' ? { kind: 'rule' } : (JSON.parse(readFileSync(p, 'utf8')) as { spec: PickerSpec }).spec;
      const set = arg('set') ?? 'tune';
      const eps = seedSet(set);
      const t0 = Date.now();
      const rs = await fly(pool, workers, eps.map((episode) => ({ spec, episode })));
      const s = summarize(rs);
      console.log(`${p} [${set}] ${pct(s)} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
      if (arg('out')) writeFileSync(arg('out')!, JSON.stringify({ picker: p, set, spec, summary: s, episodes: rs.map((r) => ({ ...r, trace: undefined })) }, null, 1));
    } else if (mode === 'cma') {
      const armName = arg('arm') ?? 'planner';
      const arm = ARMS[armName]!();
      const gens = num('gens', 60);
      const batch = num('batch', 32);
      const evalEvery = num('eval-every', 5);
      const seed = num('seed', 1);
      const cma = new Cma(new Array(arm.dim).fill(0), num('sigma', 0.5), { seed, lambda: arg('lambda') ? num('lambda', 0) : undefined });
      const rng = new RNG(seed, 'picker-batches');
      const tune = seedSet('tune');
      const log = join(outDir, 'log.jsonl');
      writeFileSync(log, '');
      let best = { score: -Infinity, gen: -1, x: [] as number[], spec: null as PickerSpec | null, tune: null as Summary | null };
      const evalMean = async (gen: number) => {
        const spec = arm.spec(cma.mean);
        const s = summarize(await fly(pool, workers, tune.map((episode) => ({ spec, episode }))));
        const line = { gen, tune: s, mean: cma.mean, sigma: cma.sigma, spec };
        appendFileSync(log, JSON.stringify(line) + '\n');
        console.log(`gen ${gen} tune ${pct(s)} sigma ${cma.sigma.toFixed(3)} x ${cma.mean.map((v) => v.toFixed(2)).join(' ')}`);
        if (s.score > best.score) {
          best = { score: s.score, gen, x: [...cma.mean], spec, tune: s };
          writeFileSync(join(outDir, 'best.json'), JSON.stringify({ arm: armName, names: arm.names, ...best }, null, 1));
        }
      };
      await evalMean(0);
      for (let g = 1; g <= gens; g++) {
        const t0 = Date.now();
        const xs = cma.ask();
        const eps = trainBatch(rng, batch);
        const items = xs.flatMap((x) => eps.map((episode) => ({ spec: arm.spec(x), episode })));
        const rs = await fly(pool, workers, items, g);
        const scores = xs.map((_, k) => summarize(rs.slice(k * batch, (k + 1) * batch)).score);
        cma.tell(xs, scores.map((s) => -s));
        const mean = scores.reduce((a, b) => a + b, 0) / scores.length;
        console.log(`gen ${g} batch mean ${(mean * 100).toFixed(1)} best ${(Math.max(...scores) * 100).toFixed(1)} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
        appendFileSync(log, JSON.stringify({ gen: g, batchMean: mean, batchBest: Math.max(...scores) }) + '\n');
        if (g % evalEvery === 0 || g === gens) await evalMean(g);
      }
      console.log(`best gen ${best.gen}: ${best.tune && pct(best.tune)}`);
    } else if (mode === 'ppo') {
      await ppo(pool, workers, outDir);
    } else throw new Error(`unknown mode ${mode}`);
  } finally {
    await pool.close();
  }
}

if (process.env.PICKER_NO_MAIN !== '1') await main();
export { PICK_DIM };
