// Calibrates the camera detector on held-out practice frames and scores it on fresh eval frames.
//   npx vite-node scripts/vision/eval-detector.ts -- --model <dir> --calib <dir> --eval <dir> [--write-meta] [--out docs/vision-detector.json]
// --write-meta stores the fitted calibration in <model>/meta.json. Calibration is fitted on --calib only.
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import * as tf from '@tensorflow/tfjs';
import { DETECT_CLASSES } from '../../src/vision/dataset';
import { calibrate, decodeOutput, frameTensor, IN_H, IN_W, type Calibration, type VisionDetection } from '../../src/vision/detector';
import { loadDetectorFromDisk, useNativeTf } from '../../src/vision/node';

const args = process.argv.slice(2).filter((a) => a !== '--');
const arg = (name: string, def: string): string => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1]! : def;
};

interface Label { c: string; k: number; x: number; y: number; w: number; h: number; px: number; d: number; id: string }
interface Item { seed: number; kind: string; night: boolean; gain: number; labels: Label[] }

function loadSet(dir: string): { items: Item[]; pixels: Uint8Array } {
  const shards = readdirSync(dir).filter((f) => f.startsWith('labels-')).map((f) => f.slice(7, -6)).sort();
  const items: Item[] = [];
  const parts: Buffer[] = [];
  for (const s of shards) {
    for (const line of readFileSync(`${dir}/labels-${s}.jsonl`, 'utf8').trim().split('\n').filter(Boolean)) items.push(JSON.parse(line) as Item);
    parts.push(readFileSync(`${dir}/frames-${s}.u8`));
  }
  return { items, pixels: new Uint8Array(Buffer.concat(parts)) };
}

const MIN_PX = 8;
const FRAME = IN_H * IN_W * 3;

function iou(a: { x: number; y: number; w: number; h: number }, b: { x: number; y: number; w: number; h: number }): number {
  const x0 = Math.max(a.x, b.x);
  const y0 = Math.max(a.y, b.y);
  const x1 = Math.min(a.x + a.w, b.x + b.w);
  const y1 = Math.min(a.y + a.h, b.y + b.h);
  const inter = Math.max(0, x1 - x0) * Math.max(0, y1 - y0);
  return inter / Math.max(1e-9, a.w * a.h + b.w * b.h - inter);
}

async function predictAll(model: tf.LayersModel, set: { items: Item[]; pixels: Uint8Array }): Promise<VisionDetection[][]> {
  const out: VisionDetection[][] = [];
  const B = 64;
  for (let s = 0; s < set.items.length; s += B) {
    const n = Math.min(B, set.items.length - s);
    const frames = Array.from({ length: n }, (_, i) => set.pixels.subarray((s + i) * FRAME, (s + i + 1) * FRAME));
    const x = frameTensor(frames);
    const y = model.predict(x) as tf.Tensor;
    const data = (await y.data()) as Float32Array;
    const per = data.length / n;
    for (let i = 0; i < n; i++) out.push(decodeOutput(data.subarray(i * per, (i + 1) * per), { minScore: 0.05 }));
    tf.dispose([x, y]);
  }
  return out;
}

interface Matched {
  score: number;
  cls: string;
  /** 1 true positive, 0 false positive, -1 matched a box too small to ask for (ignored). */
  tp: 1 | 0 | -1;
}

/** Greedy matching per frame, highest score first, IoU >= 0.5 and same class. */
function match(items: Item[], preds: VisionDetection[][]): { matched: Matched[]; gtCount: Record<string, number>; hit: Map<string, number> } {
  const matched: Matched[] = [];
  const gtCount: Record<string, number> = {};
  // Best score at which each (frame, label) was found, for recall curves.
  const hit = new Map<string, number>();
  items.forEach((item, f) => {
    const gts = item.labels.filter((l) => l.k >= 0);
    for (const g of gts) if (g.px >= MIN_PX) gtCount[g.c] = (gtCount[g.c] ?? 0) + 1;
    const used = new Set<number>();
    for (const p of preds[f]!) {
      let best = -1;
      let bestIou = 0.5;
      gts.forEach((g, gi) => {
        if (used.has(gi) || g.c !== p.classId) return;
        const v = iou(p.bbox, g);
        if (v >= bestIou) {
          bestIou = v;
          best = gi;
        }
      });
      if (best >= 0) {
        used.add(best);
        const g = gts[best]!;
        if (g.px < MIN_PX) {
          matched.push({ score: p.score, cls: p.classId, tp: -1 });
          continue;
        }
        matched.push({ score: p.score, cls: p.classId, tp: 1 });
        hit.set(`${f}:${best}`, p.score);
      } else matched.push({ score: p.score, cls: p.classId, tp: 0 });
    }
  });
  return { matched, gtCount, hit };
}

function averagePrecision(m: Matched[], total: number): number {
  const sorted = m.filter((x) => x.tp >= 0).sort((a, b) => b.score - a.score);
  let tp = 0;
  let fp = 0;
  const pts: Array<[number, number]> = [];
  for (const x of sorted) {
    if (x.tp) tp++;
    else fp++;
    pts.push([tp / Math.max(1, total), tp / (tp + fp)]);
  }
  // All-point interpolation.
  let ap = 0;
  let prevR = 0;
  for (let i = 0; i < pts.length; i++) {
    const r = pts[i]![0];
    const pMax = Math.max(...pts.slice(i).map((q) => q[1]));
    ap += (r - prevR) * pMax;
    prevR = r;
  }
  return ap;
}

function fitCalibration(m: Matched[]): Calibration {
  const data = m.filter((x) => x.tp >= 0).map((x) => {
    const s = Math.min(1 - 1e-6, Math.max(1e-6, x.score));
    return { z: Math.log(s / (1 - s)), y: x.tp };
  });
  let a = 1;
  let b = 0;
  // Newton steps on the logistic log-likelihood.
  for (let it = 0; it < 50; it++) {
    let ga = 0;
    let gb = 0;
    let haa = 1e-6;
    let hab = 0;
    let hbb = 1e-6;
    for (const { z, y } of data) {
      const p = 1 / (1 + Math.exp(-(a * z + b)));
      const e = p - y;
      ga += e * z;
      gb += e;
      const w = p * (1 - p);
      haa += w * z * z;
      hab += w * z;
      hbb += w;
    }
    const det = haa * hbb - hab * hab;
    a -= (hbb * ga - hab * gb) / det;
    b -= (haa * gb - hab * ga) / det;
  }
  return { a: Math.round(a * 1e4) / 1e4, b: Math.round(b * 1e4) / 1e4 };
}

/** Expected calibration error over 10 confidence bins. */
function ece(m: Matched[], cal?: Calibration): number {
  const rows = m.filter((x) => x.tp >= 0).map((x) => ({ c: calibrate(x.score, cal), y: x.tp }));
  let e = 0;
  for (let k = 0; k < 10; k++) {
    const bin = rows.filter((r) => r.c >= k / 10 && (k === 9 ? r.c <= 1 : r.c < (k + 1) / 10));
    if (!bin.length) continue;
    const conf = bin.reduce((s, r) => s + r.c, 0) / bin.length;
    const acc = bin.reduce((s, r) => s + r.y, 0) / bin.length;
    e += (bin.length / rows.length) * Math.abs(conf - acc);
  }
  return e;
}

const r3 = (v: number): number => Math.round(v * 1000) / 1000;

async function main(): Promise<void> {
  const backend = await useNativeTf();
  const modelDir = arg('model', 'public/models/vision-detector');
  const calibSet = loadSet(arg('calib', 'test-results/vision-data/calib'));
  const evalSet = loadSet(arg('eval', 'test-results/vision-data/eval'));
  const bundle = await loadDetectorFromDisk(modelDir).catch(async () => {
    // No meta.json yet: load the bare model.
    const json = JSON.parse(readFileSync(`${modelDir}/model.json`, 'utf8'));
    const buf = readFileSync(`${modelDir}/weights.bin`);
    const model = await tf.loadLayersModel(tf.io.fromMemory({ modelTopology: json.modelTopology, weightSpecs: json.weightsManifest[0].weights, weightData: buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength) }));
    return { model, calibration: { a: 1, b: 0 }, minScore: 0.15 };
  });
  console.log(`backend ${backend}; calib ${calibSet.items.length} frames, eval ${evalSet.items.length} frames`);

  const calibPreds = await predictAll(bundle.model, calibSet);
  const calibMatch = match(calibSet.items, calibPreds);
  const calibration = fitCalibration(calibMatch.matched);
  const minScore = 0.15;
  if (args.includes('--write-meta')) {
    writeFileSync(`${modelDir}/meta.json`, `${JSON.stringify({ calibration, minScore, classes: [...DETECT_CLASSES], trainedOn: 'rendered frames, world seeds 66001-66400 (80 per world); calibrated on 66501-66560' }, null, 2)}\n`);
  }

  const t0 = Date.now();
  const preds = await predictAll(bundle.model, evalSet);
  const msPerFrame = (Date.now() - t0) / evalSet.items.length;
  const { matched, gtCount, hit } = match(evalSet.items, preds);
  const perClass: Record<string, { ap: number; labels: number }> = {};
  for (const c of DETECT_CLASSES) {
    if (!gtCount[c]) continue;
    perClass[c] = { ap: r3(averagePrecision(matched.filter((m) => m.cls === c), gtCount[c]!)), labels: gtCount[c]! };
  }
  // Operating point: calibrated confidence >= 0.5.
  const opScore = (() => {
    for (let s = 0.05; s < 1; s += 0.005) if (calibrate(s, calibration) >= 0.5) return s;
    return 0.5;
  })();
  const recallBy = (filter: (l: Label, item: Item) => boolean, bins: number[]): Array<{ from: number; to: number; n: number; recall: number }> => {
    const rows: Array<{ from: number; to: number; n: number; recall: number }> = [];
    for (let i = 0; i + 1 < bins.length; i++) {
      let n = 0;
      let k = 0;
      evalSet.items.forEach((item, f) => {
        item.labels.forEach((l, li) => {
          if (l.k < 0 || l.px < MIN_PX || !filter(l, item) || l.d < bins[i]! || l.d >= bins[i + 1]!) return;
          const gi = item.labels.filter((x) => x.k >= 0).indexOf(l);
          n++;
          if ((hit.get(`${f}:${gi}`) ?? 0) >= opScore) k++;
          void li;
        });
      });
      rows.push({ from: bins[i]!, to: bins[i + 1]!, n, recall: n ? r3(k / n) : 0 });
    }
    return rows;
  };
  const bins = [0, 2, 4, 6, 8, 10, 14];
  const personDay = recallBy((l, it) => l.c === 'person' && !it.night, bins);
  const personNight = recallBy((l, it) => l.c === 'person' && it.night, bins);
  const objectsDay = recallBy((l, it) => l.c !== 'person' && !it.night, bins);
  const objectsNight = recallBy((l, it) => l.c !== 'person' && it.night, bins);
  // Decoys: how often a visible decoy draws its look-alike's class at the operating point.
  const lookalike: Record<string, string> = { mannequin: 'person', coat: 'person', decoy_pack: 'backpack', decoy_case: 'toolbox' };
  const decoys: Record<string, { seen: number; fooled: number }> = {};
  evalSet.items.forEach((item, f) => {
    for (const l of item.labels) {
      if (l.k >= 0 || l.px < MIN_PX) continue;
      const want = lookalike[l.c];
      if (!want) continue;
      const row = (decoys[l.c] ??= { seen: 0, fooled: 0 });
      row.seen++;
      if (preds[f]!.some((p) => p.classId === want && p.score >= opScore && iou(p.bbox, l) >= 0.3)) row.fooled++;
    }
  });
  // Background false positives per frame at the operating point: detections matching no label at all.
  let bgFp = 0;
  evalSet.items.forEach((item, f) => {
    for (const p of preds[f]!) {
      if (p.score < opScore) continue;
      if (!item.labels.some((l) => iou(p.bbox, l) >= 0.1)) bgFp++;
    }
  });
  const day = evalSet.items.filter((i) => !i.night).length;
  const report = {
    generatedBy: 'scripts/vision/eval-detector.ts',
    evalSeeds: '67001-67060 (40 frames per world, default palette)',
    calibSeeds: '66501-66560',
    frames: evalSet.items.length,
    dayFrames: day,
    nightFrames: evalSet.items.length - day,
    minLabelPixels: MIN_PX,
    calibration,
    operatingScore: r3(opScore),
    eceRaw: r3(ece(matched)),
    eceCalibrated: r3(ece(matched, calibration)),
    mAP50: r3(Object.values(perClass).reduce((s, c) => s + c.ap, 0) / Math.max(1, Object.keys(perClass).length)),
    perClass,
    recall: { personDay, personNight, objectsDay, objectsNight },
    decoys: Object.fromEntries(Object.entries(decoys).map(([k, v]) => [k, { ...v, rate: v.seen ? r3(v.fooled / v.seen) : 0 }])),
    backgroundFpPerFrame: r3(bgFp / evalSet.items.length),
    msPerFrame: r3(msPerFrame),
    backend,
  };
  console.log(JSON.stringify(report, null, 1));
  const out = arg('out', '');
  if (out) writeFileSync(out, `${JSON.stringify(report, null, 2)}\n`);
}

void main();
