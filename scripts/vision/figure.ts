// REPORT figure: eval-world camera frames with the shipped detector's boxes, written to docs/report/vision-frames.png.
//   npx vite-node scripts/vision/figure.ts -- [--seeds 67001-67060] [--out docs/report/vision-frames.png]
// Frames are picked by a fixed rule (the first frames per condition that hold a labelled target), not by result.
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname } from 'node:path';
import { worldFrames } from '../../src/vision/dataset';
import { frameDetector, type VisionDetection } from '../../src/vision/detector';
import { loadDetectorFromDisk, loadHumanoidPosesSync, useNativeTf } from '../../src/vision/node';
import { encodePng } from '../../src/vision/png';

const args = process.argv.slice(2).filter((a) => a !== '--');
const arg = (name: string, def: string): string => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1]! : def;
};
const [from, to] = arg('seeds', '67001-67060').split('-').map(Number) as [number, number];
const out = arg('out', 'docs/report/vision-frames.png');
const COLS = 4;
const ROWS = 2;
const S = 2;
const W = 160 * S;
const H = 120 * S;

const COLOR: Record<string, [number, number, number]> = { person: [255, 90, 79] };
const OBJECT: [number, number, number] = [255, 197, 61];

function rect(img: Uint8Array, stride: number, x0: number, y0: number, x1: number, y1: number, c: [number, number, number]): void {
  const put = (x: number, y: number) => {
    if (x < 0 || y < 0 || x >= stride || y * stride * 3 >= img.length) return;
    const o = (y * stride + x) * 3;
    img[o] = c[0];
    img[o + 1] = c[1];
    img[o + 2] = c[2];
  };
  for (let t = 0; t < 2; t++) {
    for (let x = x0; x <= x1; x++) {
      put(x, y0 + t);
      put(x, y1 - t);
    }
    for (let y = y0; y <= y1; y++) {
      put(x0 + t, y);
      put(x1 - t, y);
    }
  }
}

await useNativeTf();
const detector = frameDetector(await loadDetectorFromDisk());
const poses = loadHumanoidPosesSync();
// Two rows: day frames, then night frames. Each tile has at least one labelled person or object.
const picks: Array<{ rgb: Uint8Array; dets: VisionDetection[] }> = [];
const wantNight = [false, false, false, false, true, true, true, true];
for (let seed = from; seed <= to && picks.length < COLS * ROWS; seed++) {
  for (const item of worldFrames(seed, 40, poses, false)) {
    const slot = picks.length;
    if (slot >= COLS * ROWS) break;
    if (item.night !== wantNight[slot]) break;
    if (!item.labels.some((l) => l.cls >= 0 && l.pixels >= 60)) continue;
    picks.push({ rgb: item.frame.rgb, dets: detector.detect(item.frame).filter((d) => d.confidence >= 0.5) });
    break;
  }
}
const sheet = new Uint8Array(COLS * W * ROWS * H * 3).fill(255);
const stride = COLS * W;
picks.forEach((p, i) => {
  const ox = (i % COLS) * W;
  const oy = Math.floor(i / COLS) * H;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const s = (Math.floor(y / S) * 160 + Math.floor(x / S)) * 3;
      const d = ((oy + y) * stride + ox + x) * 3;
      sheet[d] = p.rgb[s]!;
      sheet[d + 1] = p.rgb[s + 1]!;
      sheet[d + 2] = p.rgb[s + 2]!;
    }
  }
  for (const det of p.dets) {
    const x0 = ox + Math.round(det.bbox.x * W);
    const y0 = oy + Math.round(det.bbox.y * H);
    const x1 = ox + Math.round((det.bbox.x + det.bbox.w) * W) - 1;
    const y1 = oy + Math.round((det.bbox.y + det.bbox.h) * H) - 1;
    rect(sheet, stride, x0, y0, x1, y1, COLOR[det.classId] ?? OBJECT);
  }
});
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, encodePng(stride, ROWS * H, sheet));
console.log(`wrote ${out}: ${picks.length} frames`);
