#!/usr/bin/env node
// Renders docs/media/social-preview.png (1280x640), the image GitHub shows when the repo link is shared.
// Uses screenshots already in docs/media/. Run after `npm run media` or `npm run test:e2e`.
import { execFileSync } from 'node:child_process';
import { mkdtempSync, readFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { chromium } from '@playwright/test';

const OUT = 'docs/media/social-preview.png';
const tmp = mkdtempSync(join(tmpdir(), 'social-'));
let n = 0;
/** Crops a fraction of the image with ffmpeg and returns it as a data URL. */
function img(path, crop) {
  const out = join(tmp, `${n++}.png`);
  const f = `crop=iw*${crop.w}:ih*${crop.h}:iw*${crop.x}:ih*${crop.y}`;
  execFileSync('ffmpeg', ['-y', '-loglevel', 'error', '-i', path, '-vf', f, out]);
  return `data:image/png;base64,${readFileSync(out).toString('base64')}`;
}
// The sim page screenshots include the side rails. VIEW is the 3D viewport as a fraction of the page.
const VIEW = { x: 0.194, y: 0.058, w: 0.556, h: 0.594 };
const FULL = { x: 0, y: 0, w: 1, h: 1 };
const shots = {
  main: img(process.env.SOCIAL_MAIN ?? 'docs/media/night.png', VIEW),
  top: img(process.env.SOCIAL_TOP ?? 'docs/media/outdoor-chase.png', FULL),
  bottom: img(process.env.SOCIAL_BOTTOM ?? 'docs/media/targets-fpv.png', VIEW),
};
const tile = (src, cls = '') => `<img class="tile ${cls}" src="${src}">`;

const html = `<!doctype html>
<html><head><meta charset="utf-8"><style>
  * { box-sizing: border-box; margin: 0; }
  body { width: 1280px; height: 640px; overflow: hidden; background: #0b1117; color: #e8eef3;
         font-family: Inter, "Segoe UI", system-ui, -apple-system, sans-serif; }
  .wrap { display: grid; grid-template-columns: 560px 1fr; height: 100%; }
  .text { padding: 64px 40px 56px 64px; display: flex; flex-direction: column; }
  .kicker { font-size: 18px; letter-spacing: .14em; text-transform: uppercase; color: #5fd3b0; font-weight: 600; }
  h1 { font-size: 76px; line-height: 1; margin-top: 18px; font-weight: 800; letter-spacing: -.02em; }
  p { font-size: 27px; line-height: 1.32; color: #b9c6d0; margin-top: 26px; }
  .chips { margin-top: auto; display: flex; flex-wrap: wrap; gap: 10px; }
  .chip { font-size: 17px; padding: 8px 14px; border-radius: 999px; background: #15212b; border: 1px solid #24343f; color: #cfdbe3; }
  .shots { position: relative; padding: 40px 40px 40px 0; display: grid; grid-template-rows: 1fr 1fr; grid-template-columns: 1.45fr 1fr; gap: 14px; }
  .tile { width: 100%; height: 100%; border-radius: 14px; border: 1px solid #24343f; object-fit: cover; }
  .main { grid-row: 1 / span 2; }
</style></head><body><div class="wrap">
  <div class="text">
    <div class="kicker">Browser drone autopilot</div>
    <h1>frontier-fly</h1>
    <p>A drone that learns to search buildings by itself, from the woods to the last room, entirely in your browser.</p>
    <div class="chips">
      <span class="chip">PPO in TF.js</span><span class="chip">Rapier physics</span>
      <span class="chip">Three.js / WebGPU</span><span class="chip">Lidar + IMU</span><span class="chip">Night flight</span>
    </div>
  </div>
  <div class="shots">
    ${tile(shots.main, 'main')}
    ${tile(shots.top)}
    ${tile(shots.bottom)}
  </div>
</div></body></html>`;

const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH || undefined });
const page = await browser.newPage({ viewport: { width: 1280, height: 640 } });
await page.setContent(html);
await page.screenshot({ path: OUT });
await browser.close();
console.log(`wrote ${OUT}`);
