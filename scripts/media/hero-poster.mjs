#!/usr/bin/env node
// Encodes the homepage hero's poster from the README hero loop, so the homepage shows a moving
// picture straight away while the live 3D replay loads behind it.
//
//   npm run media:hero                  from docs/media/film-hero.mp4
//   npm run media:hero -- some-clip.mp4 from another clip
//
// Writes public/hero/: poster.webp (the first frame, ~30 kB, painted with the page), and the same
// clip as loop.webm (VP9) and loop.mp4 (H.264 for browsers without VP9). Each loop is about 0.4 MB:
// 960 px wide is enough for a hero that the live replay replaces within seconds. media:film runs
// this after it re-renders the hero clip.
import { execFileSync } from 'node:child_process';
import { mkdirSync, statSync } from 'node:fs';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

export const HERO_DIR = 'public/hero';
/** Budget for each loop file; the encode fails loudly instead of quietly shipping a heavy homepage. */
const MAX_LOOP_BYTES = 700e3;
const MAX_POSTER_BYTES = 80e3;

function ffmpeg(args) {
  execFileSync('ffmpeg', ['-y', '-loglevel', 'error', ...args], { stdio: 'inherit' });
}

export function encodeHeroPoster(source = 'docs/media/film-hero.mp4', out = HERO_DIR) {
  mkdirSync(out, { recursive: true });
  const scale = ['-vf', 'scale=960:-2:flags=lanczos'];
  const poster = join(out, 'poster.webp');
  const webm = join(out, 'loop.webm');
  const mp4 = join(out, 'loop.mp4');
  ffmpeg(['-i', source, '-frames:v', '1', ...scale, '-c:v', 'libwebp', '-quality', '72', poster]);
  ffmpeg(['-i', source, ...scale, '-c:v', 'libvpx-vp9', '-b:v', '0', '-crf', '46', '-row-mt', '1', '-deadline', 'good', '-cpu-used', '1', '-an', webm]);
  ffmpeg(['-i', source, ...scale, '-c:v', 'libx264', '-preset', 'veryslow', '-crf', '31', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an', mp4]);
  const sizes = { poster: statSync(poster).size, webm: statSync(webm).size, mp4: statSync(mp4).size };
  if (sizes.poster > MAX_POSTER_BYTES) throw new Error(`${poster} is ${sizes.poster} bytes, over ${MAX_POSTER_BYTES}`);
  for (const f of [webm, mp4]) {
    const n = statSync(f).size;
    if (n > MAX_LOOP_BYTES) throw new Error(`${f} is ${n} bytes, over ${MAX_LOOP_BYTES}`);
  }
  console.log(`hero poster: ${JSON.stringify(sizes)}`);
  return sizes;
}

if (process.argv[1] === fileURLToPath(import.meta.url)) encodeHeroPoster(process.argv[2]);
