// The homepage's bench replay: real FrontierBench test flights, drawn top-down, side by side on the same
// building and seed. The traces come from npm run bench:replay, which flies the bench's own episode loop, and
// scripts/docs/generate.mjs checks that each trace ends on the score its flight has in bench/results.
import { prefersReducedMotion } from '../ui/siteNav';
import type { ReplayEntry } from './scoreboard';

interface Trace {
  id: string;
  name: string;
  camera: number;
  crashed: boolean;
  collisions: number;
  steps: number;
  every: number;
  pose: number[];
  firstSeen: number[];
}

interface Recording {
  track: string;
  task: { id: string; label: string; maxSteps: number };
  seed: number;
  voxel: number;
  vw: number;
  vd: number;
  reachable: number[];
  traces: Trace[];
}

/** Recordings ship as their own chunks, fetched when the replay scrolls into view. */
const RECORDINGS = import.meta.glob<{ default: Recording }>(['../../docs/replay/*.json', '!../../docs/replay/index.json']);

const STEPS_PER_SECOND = 20;
/** The camera the Hard tracks score: 90° wide, counted out to 3 m. */
const FOV = Math.PI / 2;
const REACH = 3;
const LANE_COLORS = ['#56e0ff', '#ffd166', '#c9a7ff'];

const pct = (v: number): string => `${(v * 100).toFixed(1)}%`;
const esc = (s: string): string => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

/** Index of the first value greater than x in a sorted array. */
function upperBound(sorted: Int32Array, x: number): number {
  let lo = 0;
  let hi = sorted.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (sorted[mid]! <= x) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}

function hexRgb(hex: string): [number, number, number] {
  const n = Number.parseInt(hex.slice(1), 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

/** One pilot's lane: the floor plan, what its camera has seen so far, its trail and its pose. */
class Lane {
  readonly canvas: HTMLCanvasElement;
  readonly live: HTMLElement;
  private readonly ctx: CanvasRenderingContext2D;
  private readonly cells: HTMLCanvasElement;
  private readonly cellCtx: CanvasRenderingContext2D;
  private readonly image: ImageData;
  private readonly seenSorted: Int32Array;
  private readonly total: number;
  private readonly rgb: [number, number, number];
  private scale = 1;

  constructor(
    private readonly rec: Recording,
    readonly trace: Trace,
    root: HTMLElement,
    private readonly color: string,
  ) {
    this.canvas = root.querySelector('canvas')!;
    this.live = root.querySelector('.rp-live')!;
    this.ctx = this.canvas.getContext('2d')!;
    this.cells = document.createElement('canvas');
    this.cells.width = rec.vw;
    this.cells.height = rec.vd;
    this.cellCtx = this.cells.getContext('2d')!;
    this.image = this.cellCtx.createImageData(rec.vw, rec.vd);
    this.rgb = hexRgb(color);
    const seen: number[] = [];
    let total = 0;
    for (let k = 0; k < rec.reachable.length; k++) {
      if (!rec.reachable[k]) continue;
      total += 1;
      if (trace.firstSeen[k]! >= 0) seen.push(trace.firstSeen[k]!);
    }
    this.total = total;
    this.seenSorted = Int32Array.from(seen).sort();
  }

  resize(): void {
    const w = this.canvas.clientWidth || 320;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    this.scale = w / (this.rec.vw * this.rec.voxel);
    this.canvas.width = Math.round(w * dpr);
    this.canvas.height = Math.round(this.rec.vd * this.rec.voxel * this.scale * dpr);
    this.canvas.style.height = `${Math.round(this.rec.vd * this.rec.voxel * this.scale)}px`;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  /** Camera coverage at `step`: the share of reachable voxels seen by then. */
  coverage(step: number): number {
    return this.total ? upperBound(this.seenSorted, step) / this.total : 0;
  }

  draw(step: number): void {
    const { rec, trace, ctx, image } = this;
    const { vw, vd } = rec;
    const px = image.data;
    const [r, g, b] = this.rgb;
    // Voxel layer: walls and outside dark, open floor dim, seen floor in the lane's color.
    for (let y = 0; y < vd; y++)
      for (let x = 0; x < vw; x++) {
        const k = y * vw + x;
        // Image rows run top-down; world y runs up, so flip.
        const o = ((vd - 1 - y) * vw + x) * 4;
        if (!rec.reachable[k]) {
          px[o] = 9;
          px[o + 1] = 14;
          px[o + 2] = 19;
        } else {
          const t = trace.firstSeen[k]!;
          if (t >= 0 && t <= step) {
            px[o] = Math.round(r * 0.55 + 20);
            px[o + 1] = Math.round(g * 0.55 + 20);
            px[o + 2] = Math.round(b * 0.55 + 20);
          } else {
            px[o] = 30;
            px[o + 1] = 42;
            px[o + 2] = 52;
          }
        }
        px[o + 3] = 255;
      }
    this.cellCtx.putImageData(image, 0, 0);
    const w = vw * rec.voxel * this.scale;
    const h = vd * rec.voxel * this.scale;
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, w, h);
    ctx.drawImage(this.cells, 0, 0, w, h);
    const sx = (x: number) => x * this.scale;
    const sy = (y: number) => h - y * this.scale;
    const n = trace.pose.length / 3;
    const last = Math.min(n - 1, Math.floor(step / trace.every));
    // Trail.
    ctx.strokeStyle = 'rgba(238, 244, 247, 0.55)';
    ctx.lineWidth = 1.25;
    ctx.beginPath();
    for (let i = 0; i <= last; i++) {
      const X = sx(trace.pose[i * 3]!);
      const Y = sy(trace.pose[i * 3 + 1]!);
      if (i === 0) ctx.moveTo(X, Y);
      else ctx.lineTo(X, Y);
    }
    ctx.stroke();
    // Camera wedge and the drone.
    const x = trace.pose[last * 3]!;
    const y = trace.pose[last * 3 + 1]!;
    const yaw = trace.pose[last * 3 + 2]!;
    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.22)`;
    ctx.beginPath();
    ctx.moveTo(sx(x), sy(y));
    // Canvas angles run clockwise with y down: negate the world heading.
    ctx.arc(sx(x), sy(y), REACH * this.scale, -yaw - FOV / 2, -yaw + FOV / 2);
    ctx.closePath();
    ctx.fill();
    ctx.save();
    ctx.translate(sx(x), sy(y));
    ctx.rotate(-yaw);
    ctx.fillStyle = this.color;
    ctx.strokeStyle = '#05080b';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(7, 0);
    ctx.lineTo(-5, 4.5);
    ctx.lineTo(-3, 0);
    ctx.lineTo(-5, -4.5);
    ctx.closePath();
    ctx.stroke();
    ctx.fill();
    ctx.restore();
    this.live.textContent = pct(this.coverage(step));
  }
}

function markup(rec: Recording, entry: ReplayEntry, repo: string): string {
  const seconds = rec.task.maxSteps / STEPS_PER_SECOND;
  const lanes = rec.traces
    .map((t, i) => {
      const f = entry.flights.find((x) => x.id === t.id);
      return `<figure class="rp-lane" style="--lane:${LANE_COLORS[i % LANE_COLORS.length]}">
          <figcaption><span class="rp-name">${esc(t.name)}</span><span class="rp-score"><span class="rp-live" aria-hidden="true">0.0%</span><small>scored ${pct(t.camera)}${t.crashed ? ', crashed' : ''}${f ? ` · <a href="${repo}/blob/main/${f.result}" rel="noopener">result file</a>` : ''}</small></span></figcaption>
          <canvas role="img" aria-label="${esc(t.name)}: top-down view of seed ${rec.seed}; it ends with ${pct(t.camera)} of the building in camera view"></canvas>
        </figure>`;
    })
    .join('');
  return `<div class="rp-lanes">${lanes}</div>
      <div class="rp-controls">
        <button type="button" class="rp-play" aria-label="Play">Play</button>
        <input class="rp-scrub" type="range" min="0" max="${rec.task.maxSteps}" step="1" value="0" aria-label="Flight time" />
        <span class="rp-time" aria-live="off">0 s of ${seconds} s</span>
        <label class="rp-speed">Speed <select aria-label="Playback speed"><option value="4">4×</option><option value="8" selected>8×</option><option value="16">16×</option></select></label>
      </div>`;
}

/** Mount the replay of `entry` into `host` once it is near the screen. */
export function mountBenchReplay(host: HTMLElement, entry: ReplayEntry, repo: string): void {
  const load = RECORDINGS[`../../${entry.file}`];
  if (!load) {
    host.innerHTML = '<p class="rp-missing">The recorded flights are missing from this build.</p>';
    return;
  }
  const start = async () => {
    const rec = (await load()).default;
    host.innerHTML = markup(rec, entry, repo);
    host.dataset.ready = '1';
    const lanes = rec.traces.map((t, i) => new Lane(rec, t, host.querySelectorAll<HTMLElement>('.rp-lane')[i]!, LANE_COLORS[i % LANE_COLORS.length]!));
    const play = host.querySelector<HTMLButtonElement>('.rp-play')!;
    const scrub = host.querySelector<HTMLInputElement>('.rp-scrub')!;
    const time = host.querySelector<HTMLElement>('.rp-time')!;
    const speed = host.querySelector<HTMLSelectElement>('.rp-speed select')!;
    const end = rec.task.maxSteps;
    let step = 0;
    let playing = false;
    let last = 0;
    const render = () => {
      for (const l of lanes) l.draw(step);
      scrub.value = String(Math.round(step));
      time.textContent = `${Math.round(step / STEPS_PER_SECOND)} s of ${end / STEPS_PER_SECOND} s`;
    };
    const setPlaying = (on: boolean) => {
      playing = on;
      play.textContent = on ? 'Pause' : step >= end ? 'Replay' : 'Play';
      play.setAttribute('aria-label', play.textContent);
      if (on) {
        if (step >= end) step = 0;
        last = performance.now();
        requestAnimationFrame(tick);
      }
    };
    const tick = (now: number) => {
      if (!playing) return;
      step = Math.min(end, step + ((now - last) / 1000) * STEPS_PER_SECOND * Number(speed.value));
      last = now;
      render();
      if (step >= end) setPlaying(false);
      else requestAnimationFrame(tick);
    };
    const resize = () => {
      for (const l of lanes) l.resize();
      render();
    };
    play.addEventListener('click', () => setPlaying(!playing));
    scrub.addEventListener('input', () => {
      step = Number(scrub.value);
      if (!playing) play.textContent = step >= end ? 'Replay' : 'Play';
      render();
    });
    new ResizeObserver(resize).observe(host);
    // Visitors who asked for less motion get the finished flights; everyone else watches them fly.
    if (prefersReducedMotion()) {
      step = end;
      resize();
      setPlaying(false);
    } else {
      resize();
      setPlaying(true);
    }
  };
  const io = new IntersectionObserver(
    (entries) => {
      if (!entries.some((e) => e.isIntersecting)) return;
      io.disconnect();
      start().catch((error: unknown) => {
        host.innerHTML = '<p class="rp-missing">The replay could not load.</p>';
        console.warn('Bench replay:', error);
      });
    },
    { rootMargin: '200px' },
  );
  io.observe(host);
}
