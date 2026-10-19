#!/usr/bin/env node
// Renders the mission film (film.html) frame by frame in headless Chromium and encodes it.
//
//   npm run media:film                          build, serve dist/, render every scenario
//   npm run media:film -- cabin-day             only these scenario ids
//   BASE_URL=http://host/ npm run media:film    render against a server that is already running
//   FILM_SCALE=1 FILM_FRAMES=0-300              full 1080p, or a frame range for a quick look
//
// The film is deterministic: the mission is flown headless once, and window.__film.render(i)
// draws frame i with every effect on (shadows, bloom, SMAA) however slowly the GPU or software
// renderer runs. Each frame is screenshotted, so the output plays at a true 30 fps even when a
// frame takes seconds to draw. Writes docs/media/film-<scenario>.mp4 and, for the first
// scenario, a short hero loop docs/media/film-hero.mp4 and docs/media/film-hero.gif.
import { execFileSync, spawn } from 'node:child_process';
import { existsSync, mkdirSync, readdirSync, rmSync, statSync, writeFileSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { chromium } from '@playwright/test';

const OUT = 'docs/media';
const RAW = process.env.FILM_RAW ?? 'test-results/film';
const PORT = 4175;
const SCALE = Number(process.env.FILM_SCALE ?? 2 / 3);
const WIDTH = Math.round(1920 * SCALE);
const HEIGHT = Math.round(1080 * SCALE);
/** Flight length in film seconds per scenario (intro and report card come on top). */
const FILMS = [
  { scenario: 'cabin-day', title: 'Woods to cabin: find the injured hiker', length: 62, hero: { start: 14, length: 10 } },
  { scenario: 'backpack-night', title: 'Night search: find the lost backpack', length: 58 },
  { scenario: 'compound-clear', title: 'Compound search: confirm both buildings clear', length: 66 },
];

function ffmpeg(args) {
  execFileSync('ffmpeg', ['-y', '-loglevel', 'error', ...args], { stdio: 'inherit' });
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

function frameRange(total) {
  const spec = process.env.FILM_FRAMES;
  if (!spec) return [0, total - 1];
  const [a, b] = spec.split('-').map(Number);
  return [Math.max(0, a), Math.min(total - 1, Number.isFinite(b) ? b : total - 1)];
}

async function renderFilm(browser, base, film) {
  const dir = join(RAW, film.scenario);
  const page = await browser.newPage({ viewport: { width: WIDTH, height: HEIGHT } });
  page.on('pageerror', (e) => console.error(`  page error: ${e.message}`));
  const url = `${base}film.html?record=1&scenario=${film.scenario}&scale=${SCALE}&length=${film.length}`;
  await page.goto(url);
  await page.waitForFunction(() => window.__film?.ready || window.__film?.error, undefined, { timeout: 600_000 });
  const error = await page.evaluate(() => window.__film.error);
  if (error) throw new Error(`${film.scenario}: ${error}`);
  const total = await page.evaluate(() => window.__film.frames);
  const [from, to] = frameRange(total);
  mkdirSync(dir, { recursive: true });
  const t0 = Date.now();
  // Frames before `from` still have to be stepped (the camera springs are stateful); render(from) does that.
  for (let i = from; i <= to; i++) {
    const file = join(dir, `${String(i).padStart(5, '0')}.jpg`);
    await page.evaluate((k) => window.__film.render(k), i);
    await page.screenshot({ path: file, type: 'jpeg', quality: 92 });
    if ((i - from) % 60 === 0) {
      const done = i - from + 1;
      const per = (Date.now() - t0) / done / 1000;
      console.log(`  ${film.scenario} frame ${i}/${to} (${per.toFixed(1)} s/frame, ~${Math.round(((to - i) * per) / 60)} min left)`);
    }
  }
  await page.close();
  return { dir, from, to, total };
}

function encode(film, { dir, from }) {
  const dest = join(OUT, `film-${film.scenario}.mp4`);
  ffmpeg([
    '-framerate', '30', '-start_number', String(from), '-i', join(dir, '%05d.jpg'),
    '-c:v', 'libx264', '-preset', 'slow', '-crf', '21', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an', dest,
  ]);
  const out = { file: dest, bytes: statSync(dest).size };
  if (film.hero) {
    const hero = join(OUT, 'film-hero.mp4');
    ffmpeg(['-ss', String(film.hero.start), '-t', String(film.hero.length), '-i', dest, '-c:v', 'libx264', '-preset', 'slow', '-crf', '23', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an', hero]);
    const gif = join(OUT, 'film-hero.gif');
    ffmpeg([
      '-ss', String(film.hero.start), '-t', String(film.hero.length), '-i', dest,
      '-vf', 'fps=12,scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=160[p];[b][p]paletteuse=dither=sierra2_4a',
      gif,
    ]);
    Object.assign(out, { hero, gif, gifBytes: statSync(gif).size });
  }
  return out;
}

const wanted = process.argv.slice(2);
const films = wanted.length ? FILMS.filter((f) => wanted.includes(f.scenario)) : FILMS;
const server = await startServer();
const browser = await chromium.launch({
  executablePath: process.env.CHROMIUM_PATH || undefined,
  args: ['--enable-webgl', '--ignore-gpu-blocklist', '--enable-unsafe-swiftshader'],
});
const results = [];
try {
  for (const film of films) {
    console.log(`Rendering ${film.scenario} at ${WIDTH}x${HEIGHT}`);
    const raw = await renderFilm(browser, server.base, film);
    if (!process.env.FILM_FRAMES) results.push({ scenario: film.scenario, title: film.title, frames: raw.total, seconds: raw.total / 30, width: WIDTH, height: HEIGHT, ...encode(film, raw) });
  }
} finally {
  await browser.close();
  server.stop();
}
if (results.length) {
  const index = join(OUT, 'film.json');
  const prev = existsSync(index) ? JSON.parse(readFileSync(index, 'utf8')) : { films: [] };
  const films = [...prev.films.filter((f) => !results.some((r) => r.scenario === f.scenario)), ...results];
  writeFileSync(index, `${JSON.stringify({ renderedAt: new Date().toISOString(), films }, null, 2)}\n`);
  console.log(`wrote ${index}`);
}
if (!process.env.KEEP_FRAMES && existsSync(RAW) && !process.env.FILM_FRAMES) {
  for (const d of readdirSync(RAW)) if (results.some((r) => r.scenario === d)) rmSync(join(RAW, d), { recursive: true });
}
