#!/usr/bin/env node
// Cuts the two-minute project video, docs/media/story.mp4, from the rendered mission films.
//
//   npm run media:story          after `npm run media:film` (or the media workflow's encode step)
//
// Captions only, burned in, no voice: problem, one whole mission, night and a clear call, where it
// fails, the numbers, and who built it. Every number is read from the README's generated headline
// table, so the video says what the benchmarks say at the commit it was cut from.
//
// Inputs: test-results/film/film-<scenario>-master.mp4 and film-<scenario>-phases.json (both written
// by film.mjs). Cards and captions are HTML pages screenshotted in Chromium, so they use the same
// type as the site; ffmpeg cuts, overlays and joins. Writes docs/media/story.mp4 (repo budget) and a
// full-quality test-results/film/story-master.mp4 to post on LinkedIn or YouTube.
import { execFileSync } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync, rmSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { chromium } from '@playwright/test';

const RAW = process.env.FILM_RAW ?? 'test-results/film';
const WORK = join(RAW, 'story');
const OUT = 'docs/media/story.mp4';
const MASTER = join(RAW, 'story-master.mp4');
const MAX_BYTES = 4.6e6;
const W = 1280;
const H = 720;
const FPS = 30;
const REPO = 'github.com/alexnieves-cs/frontier-fly';
const SITE = 'alexnieves-cs.github.io/frontier-fly';

function ffmpeg(args) {
  execFileSync('ffmpeg', ['-y', '-loglevel', 'error', ...args], { stdio: 'inherit' });
}

/** The README's generated headline rows, as { question, measured, on } with Markdown stripped. */
function headline() {
  const md = readFileSync('README.md', 'utf8');
  const block = md.split('<!-- generated:headline:start -->')[1]?.split('<!-- generated:headline:end -->')[0];
  if (!block) throw new Error('README has no generated headline table');
  const rows = block
    .split('\n')
    .filter((l) => l.startsWith('|') && !l.startsWith('| ---') && !l.startsWith('| Question'))
    .map((l) => l.split('|').slice(1, -1).map((c) => c.replace(/\*\*/g, '').trim()))
    .map(([question, measured, on]) => ({ question, measured, on }));
  const find = (re) => {
    const row = rows.find((r) => re.test(r.question));
    if (!row) throw new Error(`README headline has no row matching ${re}`);
    return row;
  };
  return {
    coverage: find(/learned policy add|search/i),
    night: find(/dark/i),
    missions: find(/missions/i),
    falseClear: find(/clear with someone inside/i),
    crash: find(/hit something/i),
    faults: find(/breaks/i),
  };
}

/** First percentage in a cell, e.g. "76.4%". */
const pct = (s) => s.match(/\d+(?:\.\d+)?%/)?.[0] ?? s;

// ------------------------------------------------------------------ cards and captions (HTML)

const CSS = `
  * { box-sizing: border-box; margin: 0; }
  body { width: ${W}px; height: ${H}px; font-family: Inter, 'Inter Display', system-ui, 'DejaVu Sans', sans-serif; color: #e9eef3; }
  .card { width: 100%; height: 100%; padding: 84px 96px; display: flex; flex-direction: column; justify-content: center; gap: 22px;
    background: radial-gradient(1200px 700px at 20% 10%, #16212c 0%, #0a0f14 60%); }
  .kicker { font-size: 20px; letter-spacing: 0.18em; text-transform: uppercase; color: #5ef0dc; font-weight: 600; }
  h1 { font-size: 64px; line-height: 1.05; font-weight: 700; letter-spacing: -0.01em; max-width: 1000px; }
  h2 { font-size: 46px; line-height: 1.12; font-weight: 650; max-width: 1050px; }
  p { font-size: 27px; line-height: 1.45; color: #b9c6d2; max-width: 1000px; }
  .who { margin-top: 18px; font-size: 26px; color: #e9eef3; }
  .who b { color: #ffffff; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 26px 40px; margin-top: 8px; }
  .stat { border-left: 4px solid #5ef0dc; padding: 4px 0 4px 20px; }
  .stat.warn { border-color: #ff7a59; }
  .big { font-size: 54px; font-weight: 700; line-height: 1; }
  .warn .big { color: #ffb199; }
  .lbl { font-size: 21px; line-height: 1.35; color: #b9c6d2; margin-top: 8px; }
  .links { font-size: 25px; line-height: 1.7; color: #b9c6d2; }
  .links b { color: #5ef0dc; font-weight: 600; }
  .cap { position: absolute; left: 64px; right: 64px; bottom: 172px; display: flex; justify-content: center; }
  .cap span { background: rgba(8, 12, 16, 0.78); border-radius: 10px; padding: 14px 24px; font-size: 30px; line-height: 1.35; font-weight: 550;
    max-width: 1060px; text-align: center; box-shadow: 0 6px 30px rgba(0,0,0,0.35); }
  .tag { position: absolute; left: 64px; top: 46px; font-size: 18px; letter-spacing: 0.16em; text-transform: uppercase; font-weight: 650;
    color: #0a0f14; background: #5ef0dc; padding: 7px 12px; border-radius: 6px; }
`;

const esc = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

async function shoot(page, html, file, transparent = false) {
  await page.setContent(`<!doctype html><meta charset="utf-8"><style>${CSS}${transparent ? 'html,body{background:transparent}' : ''}</style>${html}`);
  await page.screenshot({ path: file, omitBackground: transparent });
}

// ------------------------------------------------------------------ film cuts

/** Frame range [a, b) of `seconds` inside the first run of any wanted phase, placed by `align`. */
function pick(phases, wanted, seconds, align = 'start', skip = 0) {
  const n = Math.round(seconds * FPS);
  let a = phases.findIndex((p) => wanted.includes(p));
  if (a < 0) throw new Error(`no ${wanted.join('/')} frames`);
  let b = a;
  while (b < phases.length && wanted.includes(phases[b])) b++;
  const run = b - a;
  let start = align === 'middle' ? a + Math.floor((run - n) / 2) : align === 'end' ? b - n : a + Math.round(skip * FPS);
  start = Math.max(0, Math.min(phases.length - n, start));
  return [start, start + n];
}

function phasesOf(scenario) {
  const file = join(RAW, `film-${scenario}-phases.json`);
  if (!existsSync(file)) throw new Error(`${file} is missing: render the film with film.mjs first`);
  const phases = JSON.parse(readFileSync(file, 'utf8'));
  if (!Array.isArray(phases)) throw new Error(`${file} has no phases`);
  return phases;
}

/** A film cut with its caption faded in over the first 0.5 s and out over the last 0.5 s. */
function filmSegment(scenario, [a, b], caption, tag, out) {
  const master = join(RAW, `film-${scenario}-master.mp4`);
  if (!existsSync(master)) throw new Error(`${master} is missing: render the film with film.mjs first`);
  const dur = (b - a) / FPS;
  const fade = `fade=t=in:st=0:d=0.35,fade=t=out:st=${(dur - 0.35).toFixed(3)}:d=0.35`;
  const capFade = `fade=t=in:st=0.4:d=0.5:alpha=1,fade=t=out:st=${(dur - 0.8).toFixed(3)}:d=0.5:alpha=1`;
  ffmpeg([
    '-ss', (a / FPS).toFixed(3), '-t', dur.toFixed(3), '-i', master,
    '-loop', '1', '-t', dur.toFixed(3), '-i', caption,
    '-loop', '1', '-t', dur.toFixed(3), '-i', tag,
    '-filter_complex',
    `[0:v]scale=${W}:${H}:flags=lanczos,fps=${FPS},setsar=1[v];[1:v]format=rgba,${capFade}[c];[2:v]format=rgba,${capFade}[t];[v][c]overlay=0:0[vc];[vc][t]overlay=0:0,${fade},format=yuv420p[o]`,
    '-map', '[o]', '-r', String(FPS), '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-an', out,
  ]);
}

/** A still card held for `seconds`, with a slow push-in and fades. */
function cardSegment(png, seconds, out) {
  const n = Math.round(seconds * FPS);
  ffmpeg([
    '-loop', '1', '-t', seconds.toFixed(3), '-i', png,
    '-vf', `scale=${W * 2}:${H * 2},zoompan=z='1+0.025*on/${n}':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':d=${n}:s=${W}x${H}:fps=${FPS},fade=t=in:st=0:d=0.4,fade=t=out:st=${(seconds - 0.4).toFixed(3)}:d=0.4,format=yuv420p`,
    '-frames:v', String(n), '-r', String(FPS), '-c:v', 'libx264', '-preset', 'medium', '-crf', '16', '-an', out,
  ]);
}

// ------------------------------------------------------------------ the cut

const h = headline();
rmSync(WORK, { recursive: true, force: true });
mkdirSync(WORK, { recursive: true });
const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH || undefined });
const page = await browser.newPage({ viewport: { width: W, height: H } });

const cards = {
  title: `<div class="card"><div class="kicker">frontier-fly</div><h1>A search-and-rescue drone that flies itself, from the treeline to the last room</h1>
    <p>Physics, sensors, a trained pilot and rendering, all running in a web browser.</p><div class="who">Built by <b>Alex Nieves</b></div></div>`,
  problem: `<div class="card"><div class="kicker">The problem</div><h2>Someone is missing near a cabin in the woods.</h2>
    <p>With no pilot at the sticks, a drone has to sweep the area, find the building, get inside through whatever opening it fits, search every room, and come back with an answer: found, or clear.</p></div>`,
  fails: `<div class="card"><div class="kicker">Where it fails</div><h2>The numbers it is least proud of, measured on seeds it never trained on</h2><div class="grid">
    <div class="stat warn"><div class="big">${esc(pct(h.falseClear.measured))}</div><div class="lbl">of flights call a building clear with someone still inside (it was ${esc(h.falseClear.measured.match(/(?:from|was) (\d+(?:\.\d+)?%)/)?.[1] ?? 'higher')} before unconfirmed people blocked a clear)</div></div>
    <div class="stat warn"><div class="big">${esc(pct(h.crash.measured))}</div><div class="lbl">of flights hit something hard enough to count as a crash</div></div>
    <div class="stat warn"><div class="big">${esc(h.coverage.measured.match(/alone(?: it covers)? [\d.]+%, (\d+ of \d+)/)?.[1] ?? 'Most')}</div><div class="lbl">flights crash when the learned pilot flies alone, without its lidar guard</div></div>
    <div class="stat warn"><div class="big">Sim only</div><div class="lbl">It has not flown a real drone yet. The project guide lists every known limit.</div></div>
  </div></div>`,
  numbers: `<div class="card"><div class="kicker">What it does well</div><div class="grid">
    <div class="stat"><div class="big">${esc(pct(h.coverage.measured))}</div><div class="lbl">of a new, hard building searched by the trained pilot under its lidar guard</div></div>
    <div class="stat"><div class="big">${esc(h.night.measured.match(/(\d+(?:\.\d+)?%) of day/)?.[1] ?? pct(h.night.measured))}</div><div class="lbl">of its daytime coverage at night with the lights off</div></div>
    <div class="stat"><div class="big">${esc(pct(h.missions.measured))}</div><div class="lbl">of whole missions end the right way: target found, or correctly cleared</div></div>
    <div class="stat"><div class="big">${esc(pct(h.faults.measured))}</div><div class="lbl">of flights end safely after a sensor, motor, battery, link, GNSS or wind fault</div></div>
  </div><p style="font-size:22px">Fresh seeds, 95% intervals, and every number regenerated from the benchmark JSON in CI.</p></div>`,
  credits: `<div class="card"><div class="kicker">frontier-fly</div><h2>Built by Alex Nieves</h2>
    <div class="links">Try it live: <b>${SITE}</b><br>Code and tech report: <b>${REPO}</b></div>
    <p style="font-size:22px">TypeScript, three.js, Rapier physics, imitation learning and PPO, PX4 in CI. Civilian search and rescue only.</p></div>`,
};
for (const [name, html] of Object.entries(cards)) await shoot(page, html, join(WORK, `card-${name}.png`));

const captions = [
  ['cabin-day', ['coverage'], 8, 'middle', 'Mission', 'It sweeps a fenced patch of woods on lidar alone, looking for the building.'],
  ['cabin-day', ['approach', 'inspect'], 9, 'middle', 'Mission', 'It circles the building and measures every opening it might fit through.'],
  ['cabin-day', ['entry_select', 'fly_through'], 8, 'start', 'Mission', 'It picks the one it fits and flies in at walking pace.'],
  ['cabin-day', ['indoor_explore'], 12, 'start', 'Mission', 'Inside, a small learned pilot explores room by room. A lidar guard can overrule it.'],
  ['cabin-day', ['confirm'], 8, 'start', 'Mission', 'The camera flags a person. The drone takes three looks before it believes it.'],
  ['cabin-day', ['exit', 'return_home', 'land'], 6, 'end', 'Mission', 'Then it flies home and lands.'],
  ['cabin-day', ['outro'], 5, 'start', 'Mission', 'And files a report.'],
  ['backpack-night', ['indoor_explore'], 7, 'middle', 'Night', 'At night it flies the same mission by flashlight.'],
  ['compound-clear', ['indoor_explore', 'confirm'], 7, 'end', 'All clear', 'When nobody is there it has to say so. That is the risky call.'],
];
const parts = [];
const seg = (name) => {
  const f = join(WORK, `${String(parts.length).padStart(2, '0')}-${name}.mp4`);
  parts.push(f);
  return f;
};
cardSegment(join(WORK, 'card-title.png'), 6, seg('title'));
cardSegment(join(WORK, 'card-problem.png'), 7, seg('problem'));
for (const [i, [scenario, wanted, seconds, align, tag, text]] of captions.entries()) {
  const cap = join(WORK, `cap-${i}.png`);
  const tg = join(WORK, `tag-${i}.png`);
  await shoot(page, `<div class="cap"><span>${esc(text)}</span></div>`, cap, true);
  await shoot(page, `<div class="tag">${esc(tag)}</div>`, tg, true);
  filmSegment(scenario, pick(phasesOf(scenario), wanted, seconds, align), cap, tg, seg(`${scenario}-${wanted[0]}`));
}
cardSegment(join(WORK, 'card-fails.png'), 12, seg('fails'));
cardSegment(join(WORK, 'card-numbers.png'), 11, seg('numbers'));
cardSegment(join(WORK, 'card-credits.png'), 8, seg('credits'));
await browser.close();

const list = join(WORK, 'parts.txt');
writeFileSync(list, parts.map((p) => `file '${p.split('/').pop()}'`).join('\n') + '\n');
ffmpeg(['-f', 'concat', '-safe', '0', '-i', list, '-c', 'copy', '-movflags', '+faststart', MASTER]);
const seconds = Number(execFileSync('ffprobe', ['-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', MASTER]).toString().trim());
// The repo copy: 960 px wide, two-pass at the bitrate that fills the media budget.
const kbps = Math.floor((MAX_BYTES * 8) / seconds / 1000);
const log = join(WORK, 'x264');
const x264 = ['-vf', 'scale=960:-2:flags=lanczos', '-c:v', 'libx264', '-preset', 'slow', '-b:v', `${kbps}k`, '-passlogfile', log, '-pix_fmt', 'yuv420p', '-an'];
ffmpeg(['-i', MASTER, ...x264, '-pass', '1', '-f', 'mp4', '/dev/null']);
ffmpeg(['-i', MASTER, ...x264, '-pass', '2', '-movflags', '+faststart', OUT]);
ffmpeg(['-ss', '2', '-i', MASTER, '-frames:v', '1', '-q:v', '3', OUT.replace(/\.mp4$/, '-poster.jpg')]);
console.log(`wrote ${OUT} (${(statSync(OUT).size / 1e6).toFixed(2)} MB, ${seconds.toFixed(1)} s) and ${MASTER}`);
