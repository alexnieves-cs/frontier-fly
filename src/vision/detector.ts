import * as tf from '@tensorflow/tfjs';
import { DETECT_CLASSES, type DetectClass } from './dataset';
import type { Frame } from './scene';

/**
 * A small CenterNet-style detector for the drone camera. It reads one
 * 160 x 120 RGB frame and predicts, on a 20 x 15 grid (stride 8), a heatmap
 * per class plus a box size and a sub-cell center offset. Peaks of the
 * heatmap are detections. Trained by scripts/vision/train-detector.ts on
 * rendered frames from seeds 66001-66400; see REPORT.md "Search by sight".
 */
export const GRID_W = 20;
export const GRID_H = 15;
export const IN_W = 160;
export const IN_H = 120;
export const NUM_CLASSES = DETECT_CLASSES.length;
/** Heatmap channels, then (offset x, offset y, log w, log h) in grid cells. */
export const OUT_CHANNELS = NUM_CLASSES + 4;

function convBlock(x: tf.SymbolicTensor, filters: number, stride: number, name: string): tf.SymbolicTensor {
  let y = tf.layers
    .conv2d({ filters, kernelSize: 3, strides: stride, padding: 'same', useBias: false, kernelInitializer: 'heNormal', name: `${name}_conv` })
    .apply(x) as tf.SymbolicTensor;
  y = tf.layers.batchNormalization({ name: `${name}_bn` }).apply(y) as tf.SymbolicTensor;
  return tf.layers.reLU({ name: `${name}_relu` }).apply(y) as tf.SymbolicTensor;
}

export function buildDetectorModel(): tf.LayersModel {
  const input = tf.input({ shape: [IN_H, IN_W, 3], name: 'frame' });
  let x = convBlock(input, 16, 2, 'b1');
  x = convBlock(x, 24, 1, 'b2');
  x = convBlock(x, 32, 2, 'b3');
  const mid = convBlock(x, 32, 1, 'b4');
  x = convBlock(mid, 64, 2, 'b5');
  x = convBlock(x, 64, 1, 'b6');
  x = convBlock(x, 64, 1, 'b7');
  const skip = convBlock(mid, 32, 2, 'skip');
  x = tf.layers.concatenate({ name: 'fuse' }).apply([x, skip]) as tf.SymbolicTensor;
  x = convBlock(x, 64, 1, 'head');
  const out = tf.layers
    .conv2d({ filters: OUT_CHANNELS, kernelSize: 1, padding: 'same', name: 'out' })
    .apply(x) as tf.SymbolicTensor;
  const model = tf.model({ inputs: input, outputs: out, name: 'frontier_fly_vision' });
  // CenterNet's prior: start every heatmap near p = 0.01 so the many empty cells do not swamp the loss.
  const head = model.getLayer('out');
  const [kernel] = head.getWeights();
  const b = new Float32Array(OUT_CHANNELS);
  for (let k = 0; k < NUM_CLASSES; k++) b[k] = -4.6;
  head.setWeights([kernel!, tf.tensor1d(b)]);
  return model;
}

/** Score calibration, fitted on held-out practice frames: conf = sigmoid(a * logit + b). */
export interface Calibration {
  a: number;
  b: number;
}

export interface VisionDetection {
  classId: DetectClass;
  /** Calibrated probability that this is a real target of this class. */
  confidence: number;
  /** Raw heatmap peak. */
  score: number;
  /** Normalized box, origin top-left. */
  bbox: { x: number; y: number; w: number; h: number };
}

export interface DecodeOptions {
  /** Raw heatmap peak below which nothing is reported. */
  minScore?: number;
  /** Boxes overlapping more than this are merged across classes. */
  nmsIou?: number;
  calibration?: Calibration;
}

function iou(a: VisionDetection['bbox'], b: VisionDetection['bbox']): number {
  const x0 = Math.max(a.x, b.x);
  const y0 = Math.max(a.y, b.y);
  const x1 = Math.min(a.x + a.w, b.x + b.w);
  const y1 = Math.min(a.y + a.h, b.y + b.h);
  const inter = Math.max(0, x1 - x0) * Math.max(0, y1 - y0);
  return inter / Math.max(1e-9, a.w * a.h + b.w * b.h - inter);
}

export function calibrate(score: number, cal?: Calibration): number {
  if (!cal) return score;
  const s = Math.min(1 - 1e-6, Math.max(1e-6, score));
  return 1 / (1 + Math.exp(-(cal.a * Math.log(s / (1 - s)) + cal.b)));
}

/** Peaks of the raw output map [GRID_H, GRID_W, OUT_CHANNELS] (logits). */
export function decodeOutput(out: Float32Array, opts: DecodeOptions = {}): VisionDetection[] {
  const minScore = opts.minScore ?? 0.2;
  const C = OUT_CHANNELS;
  const found: VisionDetection[] = [];
  const at = (iy: number, ix: number, k: number): number => out[(iy * GRID_W + ix) * C + k]!;
  for (let iy = 0; iy < GRID_H; iy++) {
    for (let ix = 0; ix < GRID_W; ix++) {
      for (let k = 0; k < NUM_CLASSES; k++) {
        const logit = at(iy, ix, k);
        const score = 1 / (1 + Math.exp(-logit));
        if (score < minScore) continue;
        let peak = true;
        for (let dy = -1; dy <= 1 && peak; dy++) {
          for (let dx = -1; dx <= 1; dx++) {
            if (!dx && !dy) continue;
            const y = iy + dy;
            const x = ix + dx;
            if (y < 0 || x < 0 || y >= GRID_H || x >= GRID_W) continue;
            if (at(y, x, k) > logit) {
              peak = false;
              break;
            }
          }
        }
        if (!peak) continue;
        const ox = Math.min(1, Math.max(0, at(iy, ix, NUM_CLASSES)));
        const oy = Math.min(1, Math.max(0, at(iy, ix, NUM_CLASSES + 1)));
        const w = Math.exp(Math.min(3.2, at(iy, ix, NUM_CLASSES + 2))) / GRID_W;
        const h = Math.exp(Math.min(3.2, at(iy, ix, NUM_CLASSES + 3))) / GRID_H;
        const cx = (ix + ox) / GRID_W;
        const cy = (iy + oy) / GRID_H;
        const x0 = Math.max(0, cx - w / 2);
        const y0 = Math.max(0, cy - h / 2);
        found.push({
          classId: DETECT_CLASSES[k]!,
          score,
          confidence: calibrate(score, opts.calibration),
          bbox: { x: x0, y: y0, w: Math.min(1, cx + w / 2) - x0, h: Math.min(1, cy + h / 2) - y0 },
        });
      }
    }
  }
  found.sort((a, b) => b.score - a.score);
  const keep: VisionDetection[] = [];
  const nms = opts.nmsIou ?? 0.55;
  for (const d of found) if (!keep.some((k) => iou(k.bbox, d.bbox) > nms)) keep.push(d);
  return keep;
}

/** Frame pixels as the network input, 0..1. */
export function frameTensor(frames: ArrayLike<Uint8Array>): tf.Tensor4D {
  const n = frames.length;
  const data = new Float32Array(n * IN_H * IN_W * 3);
  for (let i = 0; i < n; i++) {
    const rgb = frames[i]!;
    const o = i * IN_H * IN_W * 3;
    for (let j = 0; j < rgb.length; j++) data[o + j] = rgb[j]! / 255;
  }
  return tf.tensor4d(data, [n, IN_H, IN_W, 3]);
}

export interface FrameDetector {
  detect(frame: Pick<Frame, 'rgb'>): VisionDetection[];
  dispose(): void;
}

export interface DetectorBundle {
  model: tf.LayersModel;
  calibration: Calibration;
  minScore: number;
}

export function frameDetector(bundle: DetectorBundle): FrameDetector {
  return {
    detect(frame) {
      const out = tf.tidy(() => {
        const x = frameTensor([frame.rgb]);
        return bundle.model.predict(x) as tf.Tensor;
      });
      const data = out.dataSync() as Float32Array;
      out.dispose();
      return decodeOutput(data, { minScore: bundle.minScore, calibration: bundle.calibration });
    },
    dispose() {
      bundle.model.dispose();
    },
  };
}

/** Shipped detector files, next to the page. */
export const DETECTOR_DIR = 'models/vision-detector';

export interface DetectorMeta {
  calibration: Calibration;
  minScore: number;
  classes: string[];
  trainedOn: string;
}

/** Browser loader: fetches model.json, weights.bin and meta.json from the deployed base. */
export async function loadDetectorFromUrl(base = `${import.meta.env.BASE_URL}${DETECTOR_DIR}`): Promise<DetectorBundle> {
  const [model, meta] = await Promise.all([
    tf.loadLayersModel(`${base}/model.json`),
    fetch(`${base}/meta.json`).then((r) => r.json() as Promise<DetectorMeta>),
  ]);
  return { model, calibration: meta.calibration, minScore: meta.minScore };
}
