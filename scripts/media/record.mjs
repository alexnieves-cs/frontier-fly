#!/usr/bin/env node
// Records the demo clips in headless Chromium and encodes them for the README.
//
//   npm run media                       build, serve dist/ with COOP/COEP, record every scenario
//   npm run media -- training fpv-night record only these ids
//   BASE_URL=http://host/ npm run media record against a server that is already running
//
// Writes docs/media/<id>.mp4 (H.264, 1280x720), docs/media/<id>.gif (README excerpt), and
// docs/media/media.json with sizes, durations, and the frame rate each page reported.
// Every file is kept under 5 MB. A clip that cannot fit is re-encoded smaller; the raw
// capture stays in test-results/media/ so a larger cut can be attached to a GitHub Release.
import { execFileSync, spawn } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync, rmSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { chromium } from '@playwright/test';
import { SCENARIOS } from './scenarios.mjs';

const OUT = 'docs/media';
const RAW = 'test-results/media';
const MAX_BYTES = 5_000_000;
const SIZE = { width: 1280, height: 720 };
const PORT = 4174;
const SIM_FPS = 20;
const MAX_WALL_MS = 12 * 60_000;

function ffmpeg(args) {
  execFileSync('ffmpeg', ['-y', '-loglevel', 'error', ...args], { stdio: 'inherit' });
}

function probeSeconds(file) {
  const out = execFileSync('ffprobe', ['-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', file], { encoding: 'utf8' });
  return Number(out.trim());
}

async function startServer() {
  if (process.env.BASE_URL) return { base: process.env.BASE_URL.replace(/\/?$/, '/'), stop: () => undefined };
  if (!process.env.SKIP_BUILD) execFileSync('npm', ['run', 'build'], { stdio: 'inherit' });
  const child = spawn('npx', ['vite', 'preview', '--host', '127.0.0.1', '--port', String(PORT), '--strictPort'], { stdio: 'ignore' });
  const base = `http://127.0.0.1:${PORT}/`;
  for (let i = 0; i < 100; i++) {
    try {
      if ((await fetch(base)).ok) return { base, stop: () => child.kill() };
    } catch {
      // not up yet
    }
    await new Promise((r) => setTimeout(r, 200));
  }
  child.kill();
  throw new Error('vite preview did not start');
}

/** H.264 MP4. Tries a few quality steps until the file is under the limit. */
function encodeMp4(raw, skip, seconds, dest) {
  for (const [crf, width] of [
    [26, 1280],
    [30, 1280],
    [32, 960],
    [35, 854],
  ]) {
    ffmpeg([
      '-ss', String(skip), '-t', String(seconds), '-i', raw,
      '-vf', `scale=${width}:-2:flags=lanczos,fps=30`,
      '-c:v', 'libx264', '-preset', 'slow', '-crf', String(crf), '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an',
      dest,
    ]);
    if (statSync(dest).size <= MAX_BYTES) return { crf, width };
  }
  return { crf: 35, width: 854 };
}

/** Palette GIF excerpt. Drops width and frame rate until it fits. */
function encodeGif(raw, start, length, dest) {
  for (const [width, fps] of [
    [720, 12],
    [640, 10],
    [560, 10],
    [480, 8],
  ]) {
    ffmpeg([
      '-ss', String(start), '-t', String(length), '-i', raw,
      '-filter_complex', `fps=${fps},scale=${width}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle`,
      dest,
    ]);
    if (statSync(dest).size <= MAX_BYTES) return { width, fps };
  }
  return { width: 480, fps: 8 };
}

/**
 * Captures frames with the CDP screencast (Chromium sends a JPEG whenever the page repaints) and
 * stitches them into an intermediate MP4 using each frame's timestamp. This avoids depending on
 * Playwright's bundled ffmpeg build, and recording starts only once the page is ready.
 */
async function record(browser, base, scenario) {
  const dir = join(RAW, scenario.id);
  rmSync(dir, { recursive: true, force: true });
  mkdirSync(dir, { recursive: true });
  const context = await browser.newContext({ viewport: SIZE, locale: 'en-US' });
  const page = await context.newPage();
  const errors = [];
  page.on('pageerror', (error) => errors.push(String(error)));
  const url = `${scenario.page === 'index.html' ? '' : scenario.page}${scenario.query}`;
  await page.goto(base + url);
  await page.waitForSelector('#app[data-ready="1"]', { timeout: 90_000 });

  const cdp = await context.newCDPSession(page);
  const frames = [];
  cdp.on('Page.screencastFrame', (frame) => {
    const file = `f${String(frames.length).padStart(5, '0')}.jpg`;
    writeFileSync(join(dir, file), Buffer.from(frame.data, 'base64'));
    frames.push({ file, ts: frame.metadata.timestamp ?? Date.now() / 1000 });
    cdp.send('Page.screencastFrameAck', { sessionId: frame.sessionId }).catch(() => undefined);
  });
  await cdp.send('Page.startScreencast', { format: 'jpeg', quality: 90, maxWidth: SIZE.width, maxHeight: SIZE.height });
  const t0 = Date.now();
  if (scenario.act) await scenario.act(page);
  if (scenario.timing === 'sim') {
    // One repaint is one 50 ms sim step whenever the page draws slower than 20 fps, which is
    // always the case under software WebGL. Keep capturing until there are enough frames.
    const want = scenario.seconds * SIM_FPS;
    const deadline = Date.now() + MAX_WALL_MS;
    while (frames.length < want && Date.now() < deadline) await page.waitForTimeout(500);
  } else {
    const elapsed = (Date.now() - t0) / 1000;
    if (elapsed < scenario.seconds) await page.waitForTimeout((scenario.seconds - elapsed) * 1000);
  }
  await cdp.send('Page.stopScreencast');
  const wallSeconds = (Date.now() - t0) / 1000;
  const end = frames.length ? frames[0].ts + wallSeconds : 0;

  const attrs = await page.evaluate(() => {
    const app = document.querySelector('#app');
    return { fps: app?.getAttribute('data-fps'), backend: app?.getAttribute('data-backend') };
  });
  const renderer = await page.evaluate(() => {
    const gl = document.createElement('canvas').getContext('webgl2');
    const ext = gl?.getExtension('WEBGL_debug_renderer_info');
    return ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : null;
  });
  await context.close();
  if (frames.length < 2) throw new Error(`${scenario.id}: only ${frames.length} frames captured`);

  // Wall timing keeps each frame on screen for as long as it was on the page. Sim timing plays
  // one frame per sim step, so the drone moves at its true speed even though rendering was slow.
  const list = frames.map((f, i) => {
    const next = i + 1 < frames.length ? frames[i + 1].ts : Math.max(end, f.ts + 1 / 30);
    const duration = scenario.timing === 'sim' ? 1 / SIM_FPS : Math.max(0.001, next - f.ts);
    return `file '${f.file}'\nduration ${duration.toFixed(4)}`;
  });
  list.push(`file '${frames[frames.length - 1].file}'`);
  writeFileSync(join(dir, 'frames.txt'), `${list.join('\n')}\n`);
  const raw = join(dir, 'raw.mp4');
  ffmpeg([
    '-f', 'concat', '-safe', '0', '-i', join(dir, 'frames.txt'),
    '-vf', `fps=30,scale=${SIZE.width}:${SIZE.height}:force_original_aspect_ratio=decrease,pad=${SIZE.width}:${SIZE.height}:(ow-iw)/2:(oh-ih)/2`,
    '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '16', '-pix_fmt', 'yuv420p', raw,
  ]);
  const capturedFps = frames.length / Math.max(0.001, frames[frames.length - 1].ts - frames[0].ts);
  return { raw, errors, url, renderer, frames: frames.length, capturedFps, wallSeconds, fps: attrs.fps ? Number(attrs.fps) : null, backend: attrs.backend ?? null };
}

async function main() {
  const only = process.argv.slice(2);
  const scenarios = SCENARIOS.filter((s) => only.length === 0 || only.includes(s.id));
  mkdirSync(OUT, { recursive: true });
  const manifestPath = join(OUT, 'media.json');
  const previous = existsSync(manifestPath) ? JSON.parse(readFileSync(manifestPath, 'utf8')) : { clips: [] };
  const clips = new Map(previous.clips.map((c) => [c.id, c]));
  const { base, stop } = await startServer();
  const browser = await chromium.launch({
    executablePath: process.env.CHROMIUM_PATH || undefined,
    args: ['--enable-webgl', '--ignore-gpu-blocklist', '--use-gl=angle'],
  });
  let renderer = previous.renderer ?? null;
  const skipped = [];
  const failed = [];
  try {
    for (const scenario of scenarios) {
      if (!process.env.BASE_URL && !existsSync(join('dist', scenario.page))) {
        skipped.push(`${scenario.id} (no dist/${scenario.page})`);
        continue;
      }
      console.log(`recording ${scenario.id}`);
      let shot;
      try {
        shot = await record(browser, base, scenario);
      } catch (error) {
        failed.push(`${scenario.id}: ${error instanceof Error ? error.message.split('\n')[0] : error}`);
        continue;
      }
      renderer = shot.renderer ?? renderer;
      const total = probeSeconds(shot.raw);
      const length = total;
      const mp4 = join(OUT, `${scenario.id}.mp4`);
      const gif = join(OUT, `${scenario.id}.gif`);
      const mp4Enc = encodeMp4(shot.raw, 0, length, mp4);
      const gifLen = Math.min(scenario.gif.length, Math.max(1, length - scenario.gif.start));
      const gifEnc = encodeGif(shot.raw, scenario.gif.start, gifLen, gif);
      for (const error of shot.errors) console.log(`  page error: ${error}`);
      clips.set(scenario.id, {
        id: scenario.id,
        title: scenario.title,
        url: shot.url,
        quality: scenario.quality ?? null,
        hero: Boolean(scenario.hero),
        seconds: Number(length.toFixed(1)),
        mp4: { path: mp4, bytes: statSync(mp4).size, ...mp4Enc },
        gif: { path: gif, bytes: statSync(gif).size, seconds: gifLen, ...gifEnc },
        fps: shot.fps,
        capturedFrames: shot.frames,
        capturedFps: Number(shot.capturedFps.toFixed(1)),
        timing: scenario.timing ?? 'wall',
        wallSeconds: Number(shot.wallSeconds.toFixed(1)),
        backend: shot.backend,
        pageErrors: shot.errors.length,
      });
      console.log(`  ${mp4} ${(statSync(mp4).size / 1e6).toFixed(2)} MB, ${gif} ${(statSync(gif).size / 1e6).toFixed(2)} MB`);
    }
  } finally {
    await browser.close();
    stop();
  }
  const order = SCENARIOS.map((s) => s.id);
  const manifest = {
    recordedAt: new Date().toISOString(),
    renderer,
    viewport: SIZE,
    clips: [...clips.values()].sort((a, b) => order.indexOf(a.id) - order.indexOf(b.id)),
  };
  writeFileSync(manifestPath, `${JSON.stringify(manifest, null, 2)}\n`);
  for (const s of skipped) console.log(`skipped ${s}`);
  for (const f of failed) console.log(`FAILED ${f}`);
  if (failed.length) process.exit(1);
}

await main();
