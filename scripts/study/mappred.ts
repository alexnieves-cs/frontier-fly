// Learned map prediction (src/sim/mapPredict.ts): collect, train, evaluate.
//
//   vite-node scripts/study/mappred.ts -- --collect --from 1 --to 120 --out /tmp/mp-train.bin
//   vite-node scripts/study/mappred.ts -- --collect --from 501 --to 530 --out /tmp/mp-val.bin
//   vite-node scripts/study/mappred.ts -- --fit --train /tmp/mp-train.bin --val /tmp/mp-val.bin --out public/models/mappred.json
//
// Buildings come from trainingBuilding(stage, seed, 'mappred'): a seed stream no
// evaluation draws. The sensor-only planner flies them; every few steps a handful
// of frontier candidates are sampled and labelled with their true hidden gain.
import { readFileSync, writeFileSync } from 'node:fs';
import * as tf from '@tensorflow/tfjs';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { GainLabeler, OracleGain, reachablePlane } from '../../src/eval/gainLabels';
import { classicalPilot } from '../../src/eval/pilots';
import { sensorFrame } from '../../src/sim/sensorPlanner';
import { evalLikeConfig, trainingBuilding } from '../../src/rl/dagger';
import { RNG } from '../../src/shared/rng';
import { createEnv } from '../../src/sim/env';
import {
  ClassIntegral,
  classifyFloor,
  GAIN_INPUT,
  GAIN_RADIUS,
  GAIN_SCALE,
  gainFeatures,
  GainModel,
  LearnedGain,
  parseGainModel,
  serializeGainModel,
  UnknownGain,
  unknownGain,
  type GainModelData,
} from '../../src/sim/mapPredict';
import { commandToAction, SensorPlanner } from '../../src/sim/sensorPlanner';
import { OccupancyMap } from '../../src/sim/sensorMap';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

/** Dataset file: int32 count, then per sample GAIN_INPUT uint8 features (×255) and a float32 label. */
interface Dataset {
  n: number;
  x: Uint8Array;
  y: Float32Array;
  /** The unknown-area baseline for each sample, for comparison. */
  base: Float32Array;
}

function writeDataset(path: string, d: Dataset): void {
  const head = new Int32Array([d.n]);
  writeFileSync(path, Buffer.concat([Buffer.from(head.buffer), Buffer.from(d.x.buffer, 0, d.n * GAIN_INPUT), Buffer.from(d.y.buffer, 0, d.n * 4), Buffer.from(d.base.buffer, 0, d.n * 4)]));
}

function readDataset(path: string): Dataset {
  const buf = readFileSync(path);
  const n = buf.readInt32LE(0);
  const x = new Uint8Array(buf.buffer.slice(buf.byteOffset + 4, buf.byteOffset + 4 + n * GAIN_INPUT));
  const off = 4 + n * GAIN_INPUT;
  const y = new Float32Array(buf.buffer.slice(buf.byteOffset + off, buf.byteOffset + off + n * 4));
  const base = new Float32Array(buf.buffer.slice(buf.byteOffset + off + n * 4, buf.byteOffset + off + n * 8));
  return { n, x, y, base };
}

async function collect(from: number, to: number, out: string): Promise<void> {
  const every = Number(arg('every', '20'));
  const perStep = Number(arg('per', '6'));
  const cap = (to - from + 1) * 2 * 140 * perStep;
  const x = new Uint8Array(cap * GAIN_INPUT);
  const y = new Float32Array(cap);
  const base = new Float32Array(cap);
  let n = 0;
  const feat = new Float32Array(GAIN_INPUT);
  const labeler = new GainLabeler();
  const ii = new ClassIntegral();
  const t0 = Date.now();
  for (let seed = from; seed <= to; seed++) {
    for (const stage of [3, 4]) {
      const spec = trainingBuilding(stage, seed, 'mappred');
      const cfg = evalLikeConfig(stage, spec);
      const env = await createEnv();
      let obs = env.reset(spec, seed, cfg);
      const planner = new SensorPlanner();
      const rng = new RNG(seed, `mappred-${stage}`);
      let occ: OccupancyMap | null = null;
      const maxSteps = (cfg as { maxSteps?: number }).maxSteps ?? 2400;
      for (let t = 0; t < maxSteps; t++) {
        const f = sensorFrame(env, obs);
        occ ??= new OccupancyMap(f.vw, f.vd, f.floors);
        const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
        occ.mark(floor, f.pos[0], f.pos[1], f.yaw, f.lidar);
        f.occupied = occ.occupied;
        if (t % every === 0 && n < cap - perStep) {
          const plane = f.vw * f.vd;
          const cls = classifyFloor({ vw: f.vw, vd: f.vd, base: floor * plane, seen: f.seen, visited: f.visited, occupied: f.occupied }, new Uint8Array(plane));
          ii.build(cls, f.vw, f.vd);
          const { reach } = reachablePlane(spec, floor);
          // Candidates: known-flyable, unflown voxels with something unknown within reach.
          const cands: number[] = [];
          for (let k = 0; k < plane; k++) {
            if (cls[k] !== 1) continue;
            const cx = k % f.vw;
            const cy = (k - cx) / f.vw;
            const r = GAIN_RADIUS;
            if (ii.count(2, cx - r, cy - r, cx + r + 1, cy + r + 1) > 0) cands.push(k);
          }
          for (let s = 0; s < perStep && cands.length; s++) {
            const k = cands.splice(Math.floor(rng.next() * cands.length), 1)[0]!;
            const cx = k % f.vw;
            const cy = (k - cx) / f.vw;
            gainFeatures(ii, cx, cy, feat, Math.floor(rng.next() * 8));
            for (let i = 0; i < GAIN_INPUT; i++) x[n * GAIN_INPUT + i] = Math.round(feat[i]! * 255);
            y[n] = labeler.label(reach, cls, f.vw, f.vd, cx, cy);
            base[n] = unknownGain(ii, cx, cy);
            n += 1;
          }
        }
        const step = env.step(commandToAction(planner.act(f), f.yaw, new Float32Array(4)));
        obs = step.obs;
        if (step.done) break;
      }
      env.dispose();
    }
    console.log(`seed ${seed}: ${n} samples (${Math.round((Date.now() - t0) / 1000)} s)`);
  }
  writeDataset(out, { n, x, y, base });
  console.log(`wrote ${out}: ${n} samples`);
}

function toTensor(d: Dataset, idx: Int32Array | null): { x: tf.Tensor2D; y: tf.Tensor2D } {
  const rows = idx ?? Int32Array.from({ length: d.n }, (_, i) => i);
  const xs = new Float32Array(rows.length * GAIN_INPUT);
  const ys = new Float32Array(rows.length);
  for (let r = 0; r < rows.length; r++) {
    const i = rows[r]!;
    for (let j = 0; j < GAIN_INPUT; j++) xs[r * GAIN_INPUT + j] = d.x[i * GAIN_INPUT + j]! / 255;
    ys[r] = d.y[i]! / GAIN_SCALE;
  }
  return { x: tf.tensor2d(xs, [rows.length, GAIN_INPUT]), y: tf.tensor2d(ys, [rows.length, 1]) };
}

/** Mean absolute error and rank correlation of a predictor against the labels. */
function score(pred: Float32Array, label: Float32Array): { mae: number; spearman: number; r2: number } {
  const n = label.length;
  let mae = 0;
  let mean = 0;
  for (let i = 0; i < n; i++) {
    mae += Math.abs(pred[i]! - label[i]!);
    mean += label[i]!;
  }
  mean /= n;
  let ssRes = 0;
  let ssTot = 0;
  for (let i = 0; i < n; i++) {
    ssRes += (pred[i]! - label[i]!) ** 2;
    ssTot += (label[i]! - mean) ** 2;
  }
  const rank = (a: Float32Array) => {
    const order = Array.from({ length: n }, (_, i) => i).sort((p, q) => a[p]! - a[q]!);
    const r = new Float32Array(n);
    order.forEach((o, i) => (r[o] = i));
    return r;
  };
  const ra = rank(pred);
  const rb = rank(label);
  let num = 0;
  let da = 0;
  let db = 0;
  const m = (n - 1) / 2;
  for (let i = 0; i < n; i++) {
    num += (ra[i]! - m) * (rb[i]! - m);
    da += (ra[i]! - m) ** 2;
    db += (rb[i]! - m) ** 2;
  }
  return { mae: mae / n, spearman: num / Math.sqrt(da * db), r2: 1 - ssRes / ssTot };
}

/** Least-squares fit a·u + b of the baseline to the labels (so its MAE is fair). */
function fitLinear(u: Float32Array, label: Float32Array): Float32Array {
  const n = u.length;
  let su = 0;
  let sl = 0;
  let suu = 0;
  let sul = 0;
  for (let i = 0; i < n; i++) {
    su += u[i]!;
    sl += label[i]!;
    suu += u[i]! * u[i]!;
    sul += u[i]! * label[i]!;
  }
  const a = (n * sul - su * sl) / (n * suu - su * su);
  const b = (sl - a * su) / n;
  return Float32Array.from(u, (v) => Math.max(0, a * v + b));
}

async function train(trainPath: string, valPath: string, out: string): Promise<void> {
  await tf.setBackend('cpu');
  const hidden = arg('hidden', '128,64').split(',').map(Number);
  const epochs = Number(arg('epochs', '12'));
  const tr = readDataset(trainPath);
  const va = readDataset(valPath);
  console.log(`train ${tr.n} samples, val ${va.n}`);
  const model = tf.sequential();
  hidden.forEach((h, i) => model.add(tf.layers.dense({ units: h, activation: 'relu', inputShape: i === 0 ? [GAIN_INPUT] : undefined })));
  model.add(tf.layers.dense({ units: 1 }));
  model.compile({ optimizer: tf.train.adam(1e-3), loss: tf.losses.huberLoss });
  const val = toTensor(va, null);
  const chunk = 20000;
  const rng = new RNG(5, 'mappred-train');
  let best = Infinity;
  let bestWeights: tf.Tensor[] | null = null;
  for (let e = 0; e < epochs; e++) {
    const order = Int32Array.from({ length: tr.n }, (_, i) => i);
    for (let i = order.length - 1; i > 0; i--) {
      const j = Math.floor(rng.next() * (i + 1));
      [order[i], order[j]] = [order[j]!, order[i]!];
    }
    for (let c = 0; c < tr.n; c += chunk) {
      const { x, y } = toTensor(tr, order.subarray(c, Math.min(tr.n, c + chunk)));
      await model.fit(x, y, { batchSize: 256, epochs: 1, shuffle: false, verbose: 0 });
      x.dispose();
      y.dispose();
    }
    const vl = (model.evaluate(val.x, val.y) as tf.Scalar).dataSync()[0]!;
    console.log(`epoch ${e + 1}: val huber ${vl.toFixed(5)}`);
    if (vl < best) {
      best = vl;
      bestWeights?.forEach((w) => w.dispose());
      bestWeights = model.getWeights().map((w) => w.clone());
    }
  }
  model.setWeights(bestWeights!);
  const ws = model.getWeights();
  const layers: GainModelData['layers'] = [];
  for (let i = 0; i < ws.length; i += 2) layers.push({ w: ws[i]!.dataSync() as Float32Array, b: ws[i + 1]!.dataSync() as Float32Array });
  const data: GainModelData = { version: 1, input: GAIN_INPUT, hidden, layers };
  // Validation metrics through the pure-TS forward pass the planner uses.
  const gm = new GainModel(data);
  const xi = new Float32Array(GAIN_INPUT);
  const pred = new Float32Array(va.n);
  for (let i = 0; i < va.n; i++) {
    for (let j = 0; j < GAIN_INPUT; j++) xi[j] = va.x[i * GAIN_INPUT + j]! / 255;
    pred[i] = gm.predict(xi);
  }
  const learned = score(pred, va.y);
  const baseline = score(fitLinear(va.base, va.y), va.y);
  console.log('learned', learned, 'unknown-area baseline (linear fit on val)', baseline);
  data.meta = {
    trainSamples: tr.n,
    valSamples: va.n,
    valMae: learned.mae,
    valSpearman: learned.spearman,
    valR2: learned.r2,
    baselineMae: baseline.mae,
    baselineSpearman: baseline.spearman,
    baselineR2: baseline.r2,
  };
  writeFileSync(out, serializeGainModel(data));
  console.log(`wrote ${out}`);
}

/** Coverage of the planner with each gain source and weight on the planner-tuning buildings (9101-9108). */
async function tune(modelPath: string): Promise<void> {
  const sources = arg('sources', 'learned,unknown,oracle').split(',');
  const data = sources.includes('learned') ? parseGainModel(readFileSync(modelPath, 'utf8')) : null;
  const weights = arg('weights', '0,0.03,0.06,0.1').split(',').map(Number);
  const stages = arg('stages', '3,4').split(',').map(Number);
  for (const stage of stages) {
    for (const src of sources) {
      for (const w of weights) {
        if (w === 0 && src !== sources[0]) continue;
        const covs: number[] = [];
        const early: number[] = [];
        for (let seed = 9101; seed <= 9108; seed++) {
          const { spec, envSeed, cfg } = evalEpisodeSetup(stage, seed, 'day');
          const gain = w === 0 ? undefined : src === 'learned' ? new LearnedGain(data!) : src === 'unknown' ? new UnknownGain() : new OracleGain(spec);
          const pilot = classicalPilot({ gain, gainWeight: w });
          pilot.reset();
          const env = await createEnv();
          let obs = env.reset(spec, envSeed, cfg);
          let cov = 0;
          for (let t = 0; t < cfg.maxSteps; t++) {
            const step = env.step(pilot.act(obs, env));
            obs = step.obs;
            cov = step.info.coverage;
            if (t === Math.floor(cfg.maxSteps / 4)) early.push(cov);
            if (step.done) break;
          }
          env.dispose();
          covs.push(cov);
        }
        const mean = covs.reduce((a, b) => a + b, 0) / covs.length;
        const meanEarly = early.reduce((a, b) => a + b, 0) / Math.max(1, early.length);
        console.log(`stage ${stage} ${w === 0 ? 'none' : src} w=${w}: coverage ${(mean * 100).toFixed(1)}%, at a quarter of the budget ${(meanEarly * 100).toFixed(1)}%  [${covs.map((c) => (c * 100).toFixed(0)).join(' ')}]`);
      }
    }
  }
}

if (process.argv.includes('--tune')) {
  await tune(arg('model', 'public/models/mappred.json'));
} else if (process.argv.includes('--collect')) {
  await collect(Number(arg('from', '1')), Number(arg('to', '10')), arg('out', '/tmp/mappred.bin'));
} else if (process.argv.includes('--fit')) {
  await train(arg('train', ''), arg('val', ''), arg('out', 'public/models/mappred.json'));
}
