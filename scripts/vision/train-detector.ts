// Trains the camera detector (src/vision/detector.ts) on frames from make-dataset.ts.
//   npm install --no-save @tensorflow/tfjs-node@4.22.0
//   npx vite-node scripts/vision/train-detector.ts -- --train <dir> --val <dir> --epochs 24 --out <dir>
// Without tfjs-node it falls back to the cpu backend, which is far too slow for a full run.
import { mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import * as tf from '@tensorflow/tfjs';
import { buildDetectorModel, GRID_H, GRID_W, IN_H, IN_W, NUM_CLASSES } from '../../src/vision/detector';

const args = process.argv.slice(2).filter((a) => a !== '--');
const arg = (name: string, def: string): string => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1]! : def;
};

interface Label { k: number; x: number; y: number; w: number; h: number; px: number }
interface Item { labels: Label[] }

function loadSet(dir: string): { items: Item[]; pixels: Uint8Array } {
  const shards = readdirSync(dir).filter((f) => f.startsWith('labels-')).map((f) => f.slice(7, -6)).sort();
  const items: Item[] = [];
  const parts: Buffer[] = [];
  for (const s of shards) {
    const lines = readFileSync(`${dir}/labels-${s}.jsonl`, 'utf8').trim().split('\n').filter(Boolean);
    for (const line of lines) items.push(JSON.parse(line) as Item);
    parts.push(readFileSync(`${dir}/frames-${s}.u8`));
  }
  const pixels = new Uint8Array(Buffer.concat(parts));
  if (pixels.length !== items.length * IN_H * IN_W * 3) throw new Error(`frame/label count mismatch in ${dir}`);
  return { items, pixels };
}

/** Training target for one frame: heatmaps, (offset, log size), center mask, heat-loss weight. */
const TARGET_CHANNELS = NUM_CLASSES + 6;
const MIN_LABEL_PX = 8;

function writeTarget(labels: Label[], flip: boolean, y: Float32Array, o: number): void {
  const C = TARGET_CHANNELS;
  for (let i = 0; i < GRID_H * GRID_W; i++) y[o + i * C + NUM_CLASSES + 5] = 1;
  for (const l of labels) {
    if (l.k < 0) continue;
    const x0 = flip ? 1 - l.x - l.w : l.x;
    const cx = (x0 + l.w / 2) * GRID_W;
    const cy = (l.y + l.h / 2) * GRID_H;
    const ix = Math.min(GRID_W - 1, Math.max(0, Math.floor(cx)));
    const iy = Math.min(GRID_H - 1, Math.max(0, Math.floor(cy)));
    if (l.px < MIN_LABEL_PX) {
      // Too few pixels to ask for: neither a positive nor a negative.
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const yy = iy + dy;
          const xx = ix + dx;
          if (yy < 0 || xx < 0 || yy >= GRID_H || xx >= GRID_W) continue;
          y[o + (yy * GRID_W + xx) * C + NUM_CLASSES + 5] = 0;
        }
      }
      continue;
    }
    const wc = l.w * GRID_W;
    const hc = l.h * GRID_H;
    const sx = Math.max(0.5, wc / 6);
    const sy = Math.max(0.5, hc / 6);
    for (let yy = Math.max(0, iy - 4); yy <= Math.min(GRID_H - 1, iy + 4); yy++) {
      for (let xx = Math.max(0, ix - 4); xx <= Math.min(GRID_W - 1, ix + 4); xx++) {
        const g = xx === ix && yy === iy ? 1 : Math.exp(-((xx + 0.5 - cx) ** 2) / (2 * sx * sx) - ((yy + 0.5 - cy) ** 2) / (2 * sy * sy));
        const j = o + (yy * GRID_W + xx) * C + l.k;
        if (g > y[j]!) y[j] = Math.min(g, xx === ix && yy === iy ? 1 : 0.98);
      }
    }
    const j = o + (iy * GRID_W + ix) * C;
    y[j + NUM_CLASSES] = cx - ix;
    y[j + NUM_CLASSES + 1] = cy - iy;
    y[j + NUM_CLASSES + 2] = Math.log(Math.max(wc, 0.05));
    y[j + NUM_CLASSES + 3] = Math.log(Math.max(hc, 0.05));
    y[j + NUM_CLASSES + 4] = 1;
  }
}

function batchTensors(set: { items: Item[]; pixels: Uint8Array }, idx: number[], augment: boolean): { xs: tf.Tensor4D; ys: tf.Tensor4D } {
  const n = idx.length;
  const frame = IN_H * IN_W * 3;
  const x = new Float32Array(n * frame);
  const y = new Float32Array(n * GRID_H * GRID_W * TARGET_CHANNELS);
  idx.forEach((k, b) => {
    const flip = augment && Math.random() < 0.5;
    const src = k * frame;
    const dst = b * frame;
    if (!flip) for (let i = 0; i < frame; i++) x[dst + i] = set.pixels[src + i]! / 255;
    else {
      for (let r = 0; r < IN_H; r++) {
        for (let c = 0; c < IN_W; c++) {
          const s = src + (r * IN_W + (IN_W - 1 - c)) * 3;
          const d = dst + (r * IN_W + c) * 3;
          x[d] = set.pixels[s]! / 255;
          x[d + 1] = set.pixels[s + 1]! / 255;
          x[d + 2] = set.pixels[s + 2]! / 255;
        }
      }
    }
    writeTarget(set.items[k]!.labels, flip, y, b * GRID_H * GRID_W * TARGET_CHANNELS);
  });
  return {
    xs: tf.tensor4d(x, [n, IN_H, IN_W, 3]),
    ys: tf.tensor4d(y, [n, GRID_H, GRID_W, TARGET_CHANNELS]),
  };
}

/** CenterNet focal loss on the heatmaps plus L1 on offset and log size at centers. */
function detectorLoss(yTrue: tf.Tensor, yPred: tf.Tensor): tf.Scalar {
  return tf.tidy(() => {
    const C = NUM_CLASSES;
    const logits = yPred.slice([0, 0, 0, 0], [-1, -1, -1, C]);
    const gt = yTrue.slice([0, 0, 0, 0], [-1, -1, -1, C]);
    const reg = yPred.slice([0, 0, 0, C], [-1, -1, -1, 4]);
    const regT = yTrue.slice([0, 0, 0, C], [-1, -1, -1, 4]);
    const mask = yTrue.slice([0, 0, 0, C + 4], [-1, -1, -1, 1]);
    const weight = yTrue.slice([0, 0, 0, C + 5], [-1, -1, -1, 1]);
    const p = tf.sigmoid(logits).clipByValue(1e-4, 1 - 1e-4);
    const pos = gt.greaterEqual(0.999).toFloat();
    const posLoss = tf.log(p).mul(tf.square(tf.sub(1, p))).mul(pos).neg();
    const negLoss = tf.log(tf.sub(1, p)).mul(tf.square(p)).mul(tf.pow(tf.sub(1, gt), 4)).mul(tf.sub(1, pos)).neg();
    const nPos = tf.maximum(pos.sum(), 1);
    const heat = posLoss.add(negLoss).mul(weight).sum().div(nPos);
    const nMask = tf.maximum(mask.sum(), 1);
    const l1 = tf.abs(reg.sub(regT)).mul(mask).mul(tf.tensor1d([1, 1, 0.5, 0.5])).sum().div(nMask);
    return heat.add(l1) as tf.Scalar;
  });
}

async function main(): Promise<void> {
  let backend = 'cpu';
  try {
    const name = '@tensorflow/tfjs-node';
    await import(/* @vite-ignore */ name);
    await tf.setBackend('tensorflow');
    backend = 'tensorflow';
  } catch {
    await tf.setBackend('cpu');
  }
  const trainDir = arg('train', 'test-results/vision-data/train');
  const valDir = arg('val', 'test-results/vision-data/calib');
  const epochs = Number(arg('epochs', '24'));
  const batch = Number(arg('batch', '32'));
  const out = arg('out', 'test-results/vision-model');
  mkdirSync(out, { recursive: true });
  const train = loadSet(trainDir);
  const val = loadSet(valDir);
  console.log(`backend ${backend}; ${train.items.length} train frames, ${val.items.length} val frames`);
  const model = buildDetectorModel();
  console.log(`params ${model.countParams()}`);
  const steps = Math.floor(train.items.length / batch);
  const lr0 = Number(arg('lr', '0.002'));
  const opt = tf.train.adam(lr0);
  const order = train.items.map((_, i) => i);
  let rng = 12345;
  const rand = (): number => ((rng = (Math.imul(rng, 1103515245) + 12345) >>> 0) / 4294967296);
  const history: Array<{ epoch: number; loss: number; val: number; sec: number }> = [];
  for (let epoch = 0; epoch < epochs; epoch++) {
    // Cosine learning-rate decay with a short warm-up.
    const lr = epoch === 0 ? lr0 * 0.3 : lr0 * 0.5 * (1 + Math.cos((Math.PI * epoch) / epochs));
    (opt as unknown as { learningRate: number }).learningRate = lr;
    for (let i = order.length - 1; i > 0; i--) {
      const j = Math.floor(rand() * (i + 1));
      [order[i], order[j]] = [order[j]!, order[i]!];
    }
    const t0 = Date.now();
    let sum = 0;
    for (let s = 0; s < steps; s++) {
      const { xs, ys } = batchTensors(train, order.slice(s * batch, (s + 1) * batch), true);
      const cost = opt.minimize(() => detectorLoss(ys, model.apply(xs, { training: true }) as tf.Tensor), true) as tf.Scalar;
      const loss = (await cost.data())[0]!;
      cost.dispose();
      sum += loss;
      xs.dispose();
      ys.dispose();
      if (s % 200 === 0) console.log(`  epoch ${epoch + 1} step ${s}/${steps} loss ${loss.toFixed(4)}`);
    }
    let vsum = 0;
    let vn = 0;
    for (let s = 0; s + batch <= val.items.length; s += batch) {
      const idx = Array.from({ length: batch }, (_, k) => s + k);
      const { xs, ys } = batchTensors(val, idx, false);
      const pred = model.predict(xs) as tf.Tensor;
      const l = detectorLoss(ys, pred);
      vsum += l.dataSync()[0]!;
      vn++;
      tf.dispose([xs, ys, pred, l]);
    }
    const rec = { epoch: epoch + 1, loss: sum / steps, val: vsum / Math.max(1, vn), sec: Math.round((Date.now() - t0) / 1000) };
    history.push(rec);
    console.log(`epoch ${rec.epoch}: train ${rec.loss.toFixed(4)} val ${rec.val.toFixed(4)} lr ${lr.toExponential(2)} (${rec.sec} s)`);
    await saveModel(model, out);
    writeFileSync(`${out}/history.json`, JSON.stringify(history, null, 1));
  }
}

async function saveModel(model: tf.LayersModel, dir: string): Promise<void> {
  await model.save(
    tf.io.withSaveHandler(async (artifacts) => {
      const weights = artifacts.weightData as ArrayBuffer;
      writeFileSync(`${dir}/weights.bin`, Buffer.from(weights));
      writeFileSync(
        `${dir}/model.json`,
        JSON.stringify({
          format: 'layers-model',
          generatedBy: 'frontier-fly scripts/vision/train-detector.ts',
          modelTopology: artifacts.modelTopology,
          weightsManifest: [{ paths: ['weights.bin'], weights: artifacts.weightSpecs }],
        }),
      );
      return { modelArtifactsInfo: { dateSaved: new Date(), modelTopologyType: 'JSON' } };
    }),
  );
}

void main();
