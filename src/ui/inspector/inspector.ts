import { FLOOR_HEIGHT, LIDAR_H, LIDAR_MAX, MAP_SIZE, OBS } from '../../shared/constants';
import { INPUT_BLOCKS } from './influence';

/**
 * "What the policy sees": the observation vector one lane's drone produces,
 * drawn the way the network receives it. Everything is egocentric, with the
 * drone's forward up and its left on the left.
 */

export interface InspectorInput {
  obs: Float32Array;
  action: Float32Array;
  /** Command shift per INPUT_BLOCKS entry when that block is blanked. */
  influence: Float32Array | null;
  /** What the shipped policy would command here, when another pilot is flying. */
  policyAction: Float32Array | null;
  /** The lane's display name and color. */
  name: string;
  color: string;
  /** True when the lane's pilot actually reads this vector. */
  readsObs: boolean;
  /** One line on what the pilot uses instead, when it does not. */
  knows: string;
}

const AXES = [
  { label: 'Forward', neg: 'back', pos: 'fwd' },
  { label: 'Sideways', neg: 'right', pos: 'left' },
  { label: 'Climb', neg: 'down', pos: 'up' },
  { label: 'Turn', neg: 'right', pos: 'left' },
] as const;

const VISITED = [86, 224, 255];
const FRONTIER = [255, 182, 72];

export function inspectorMarkup(): string {
  const gauges = AXES.map(
    (a, i) => `
      <div class="ins-gauge" data-axis="${i}">
        <span class="ins-gauge-label">${a.label}</span>
        <span class="ins-gauge-track" aria-hidden="true">
          <span class="ins-gauge-fill"></span><span class="ins-gauge-ghost"></span><span class="ins-gauge-mid"></span>
        </span>
        <span class="ins-gauge-value">0.00</span>
      </div>`,
  ).join('');
  const blocks = INPUT_BLOCKS.map(
    (b, i) => `
      <div class="ins-infl" data-block="${i}">
        <span class="ins-infl-label">${b.label}</span>
        <span class="ins-infl-track" aria-hidden="true"><span class="ins-infl-fill"></span></span>
        <span class="ins-infl-value">0%</span>
      </div>`,
  ).join('');
  return `
    <div class="ins-grid">
      <figure class="ins-card">
        <figcaption><b>Lidar ring</b><span>32 beams, 8 m. Arrow: frontier cue</span></figcaption>
        <canvas class="ins-canvas" data-ins="lidar" role="img" aria-label="Lidar ranges around the drone"></canvas>
      </figure>
      <figure class="ins-card">
        <figcaption><b>Local map</b><span>8 m around it, 0.5 m cells</span></figcaption>
        <canvas class="ins-canvas" data-ins="local" role="img" aria-label="Egocentric local map"></canvas>
      </figure>
      <figure class="ins-card">
        <figcaption><b>Coarse map</b><span>32 m around it, 2 m cells</span></figcaption>
        <canvas class="ins-canvas" data-ins="coarse" role="img" aria-label="Egocentric coarse map"></canvas>
      </figure>
      <div class="ins-card ins-out">
        <div class="ins-section">
          <h4>Command <span class="ins-who"></span></h4>
          <div class="ins-gauges">${gauges}</div>
          <p class="ins-ghost-note" hidden><span class="ins-ghost-swatch"></span>The learned policy would command this from the same view.</p>
        </div>
        <div class="ins-section ins-infl-wrap">
          <h4>What it is acting on</h4>
          <div class="ins-infls">${blocks}</div>
          <p class="ins-note ins-infl-note">Blank one input at a time and re-run the network: the bigger the change in command, the more it is relying on that input right now.</p>
        </div>
        <dl class="ins-readouts">
          <div><dt>Height</dt><dd data-ro="alt">–</dd></div>
          <div><dt>Speed</dt><dd data-ro="speed">–</dd></div>
          <div><dt>Frontier</dt><dd data-ro="frontier">–</dd></div>
          <div><dt>No new area</dt><dd data-ro="stall">–</dd></div>
        </dl>
      </div>
    </div>
    <div class="ins-legend">
      <span><i class="sw sw-visited"></i>Flown over</span>
      <span><i class="sw sw-frontier"></i>Seen open, not yet flown (frontier)</span>
      <span><i class="sw sw-wall"></i>Wall or unknown</span>
      <span><i class="sw sw-wp"></i>Waypoint to the nearest frontier</span>
      <span><i class="sw sw-cluster"></i>Biggest frontier cluster</span>
    </div>`;
}

function fit(canvas: HTMLCanvasElement): { ctx: CanvasRenderingContext2D; w: number; h: number; dpr: number } {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = Math.max(1, Math.round(canvas.clientWidth * dpr));
  const h = Math.max(1, Math.round(canvas.clientHeight * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  const ctx = canvas.getContext('2d')!;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  return { ctx, w, h, dpr };
}

/** Body frame (forward, left) to screen offsets with forward up. */
function screen(fwd: number, left: number): [number, number] {
  return [-left, -fwd];
}

function arrow(ctx: CanvasRenderingContext2D, x0: number, y0: number, x1: number, y1: number, head: number): void {
  const a = Math.atan2(y1 - y0, x1 - x0);
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - head * Math.cos(a - 0.45), y1 - head * Math.sin(a - 0.45));
  ctx.lineTo(x1 - head * Math.cos(a + 0.45), y1 - head * Math.sin(a + 0.45));
  ctx.closePath();
  ctx.fill();
}

function droneGlyph(ctx: CanvasRenderingContext2D, x: number, y: number, s: number, color: string): void {
  ctx.save();
  ctx.translate(x, y);
  ctx.fillStyle = '#f4fbff';
  ctx.shadowColor = color;
  ctx.shadowBlur = s;
  ctx.beginPath();
  ctx.moveTo(0, -s);
  ctx.lineTo(s * 0.62, s * 0.7);
  ctx.lineTo(0, s * 0.38);
  ctx.lineTo(-s * 0.62, s * 0.7);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

export class Inspector {
  private readonly lidar: HTMLCanvasElement;
  private readonly local: HTMLCanvasElement;
  private readonly coarse: HTMLCanvasElement;
  private readonly gauges: HTMLElement[];
  private readonly infl: HTMLElement[];
  private readonly ghostNote: HTMLElement;
  private readonly inflWrap: HTMLElement;
  private readonly inflNote: HTMLElement;
  private readonly who: HTMLElement;
  private readonly ro: Record<string, HTMLElement>;

  constructor(private readonly root: HTMLElement) {
    root.innerHTML = inspectorMarkup();
    const q = <T extends Element>(sel: string) => root.querySelector<T>(sel)!;
    this.lidar = q('[data-ins="lidar"]');
    this.local = q('[data-ins="local"]');
    this.coarse = q('[data-ins="coarse"]');
    this.gauges = [...root.querySelectorAll<HTMLElement>('.ins-gauge')];
    this.infl = [...root.querySelectorAll<HTMLElement>('.ins-infl')];
    this.ghostNote = q('.ins-ghost-note');
    this.inflWrap = q('.ins-infl-wrap');
    this.inflNote = q('.ins-infl-note');
    this.who = q('.ins-who');
    this.ro = Object.fromEntries([...root.querySelectorAll<HTMLElement>('[data-ro]')].map((el) => [el.dataset.ro!, el]));
  }

  render(input: InspectorInput): void {
    const { obs } = input;
    this.root.style.setProperty('--lane', input.color);
    this.drawLidar(obs, input.color);
    this.drawMap(this.local, obs, OBS.localVisited, OBS.localFrontier, input.color, false);
    this.drawMap(this.coarse, obs, OBS.coarseVisited, OBS.coarseFrontier, input.color, true);
    this.who.textContent = `· ${input.name}`;

    AXES.forEach((_, i) => {
      const el = this.gauges[i]!;
      const v = input.action[i] ?? 0;
      const fill = el.querySelector<HTMLElement>('.ins-gauge-fill')!;
      fill.style.left = `${50 + Math.min(0, v) * 50}%`;
      fill.style.width = `${Math.abs(v) * 50}%`;
      const ghost = el.querySelector<HTMLElement>('.ins-gauge-ghost')!;
      const g = input.policyAction?.[i];
      ghost.hidden = g === undefined;
      if (g !== undefined) ghost.style.left = `${50 + g * 50}%`;
      el.querySelector('.ins-gauge-value')!.textContent = `${v >= 0 ? '+' : '−'}${Math.abs(v).toFixed(2)}`;
    });
    this.ghostNote.hidden = !input.policyAction;

    this.inflWrap.classList.toggle('muted', !input.influence);
    if (input.influence) {
      const total = input.influence.reduce((s, v) => s + v, 0);
      this.infl.forEach((el, i) => {
        const v = input.influence![i]!;
        const share = total > 1e-6 ? v / total : 0;
        el.querySelector<HTMLElement>('.ins-infl-fill')!.style.width = `${(share * 100).toFixed(1)}%`;
        el.querySelector('.ins-infl-value')!.textContent = `${Math.round(share * 100)}%`;
        el.title = `Command moves by ${v.toFixed(3)} when ${INPUT_BLOCKS[i]!.label.toLowerCase()} is blanked`;
      });
      this.inflNote.textContent =
        total < 0.03
          ? 'Nothing it sees right now changes the command much: it is cruising.'
          : 'Blank one input at a time and re-run the network: the bigger the change in command, the more it is relying on that input right now.';
    } else {
      this.infl.forEach((el) => {
        el.querySelector<HTMLElement>('.ins-infl-fill')!.style.width = '0%';
        el.querySelector('.ins-infl-value')!.textContent = '–';
      });
      this.inflNote.textContent = input.readsObs
        ? 'Attribution is for the learned network. This pilot is a fixed rule that reads the same vector.'
        : `This pilot does not read this vector. ${input.knows}`;
    }

    const alt = obs[OBS.alt]! * FLOOR_HEIGHT;
    const speed = Math.hypot(obs[OBS.vel]! * 2, obs[OBS.vel + 1]! * 2, obs[OBS.vel + 2]!);
    this.ro.alt!.textContent = `${alt.toFixed(2)} m`;
    this.ro.speed!.textContent = `${speed.toFixed(2)} m/s`;
    this.ro.frontier!.textContent = obs[OBS.hasFrontier]! > 0.5 ? `${(obs[OBS.pathLen]! * 20).toFixed(1)} m away` : 'none in reach';
    this.ro.stall!.textContent = `${Math.round(obs[OBS.stall]! * 50 * 0.05 * 10) / 10}${obs[OBS.stall]! >= 1 ? '+' : ''} s`;
  }

  private drawLidar(obs: Float32Array, color: string): void {
    const { ctx, w, h, dpr } = fit(this.lidar);
    ctx.fillStyle = '#060b10';
    ctx.fillRect(0, 0, w, h);
    const cx = w / 2;
    const cy = h / 2;
    const R = Math.min(w, h) / 2 - 8 * dpr;
    const k = R / LIDAR_MAX;
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = dpr;
    for (let m = 2; m <= LIDAR_MAX; m += 2) {
      ctx.beginPath();
      ctx.arc(cx, cy, m * k, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.fillStyle = 'rgba(255,255,255,0.35)';
    ctx.font = `${10 * dpr}px ui-monospace, Menlo, monospace`;
    ctx.fillText('8 m', cx + 3 * dpr, cy - LIDAR_MAX * k + 11 * dpr);

    // Free space the ring sees, then each return.
    const pts: Array<[number, number, boolean]> = [];
    for (let i = 0; i < LIDAR_H; i++) {
      const r = Math.min(1, obs[OBS.lidar + i]!) * LIDAR_MAX;
      const th = (i * 2 * Math.PI) / LIDAR_H;
      const [sx, sy] = screen(Math.cos(th) * r, Math.sin(th) * r);
      pts.push([cx + sx * k, cy + sy * k, obs[OBS.lidar + i]! >= 0.999]);
    }
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath();
    ctx.fillStyle = 'rgba(86, 224, 255, 0.10)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(86, 224, 255, 0.35)';
    ctx.stroke();
    for (const [x, y, miss] of pts) {
      ctx.strokeStyle = miss ? 'rgba(255,255,255,0.12)' : 'rgba(86, 224, 255, 0.25)';
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(x, y);
      ctx.stroke();
      if (!miss) {
        ctx.fillStyle = '#ff6b6b';
        ctx.beginPath();
        ctx.arc(x, y, 2.4 * dpr, 0, Math.PI * 2);
        ctx.fill();
      }
    }
    // Tilted beams, every 45 degrees.
    for (let i = 0; i < 8; i++) {
      const r = Math.min(1, obs[OBS.tilt + i]!) * LIDAR_MAX;
      if (r >= LIDAR_MAX * 0.999) continue;
      const th = (i * Math.PI) / 4;
      const [sx, sy] = screen(Math.cos(th) * r, Math.sin(th) * r);
      ctx.strokeStyle = 'rgba(255, 209, 102, 0.8)';
      ctx.beginPath();
      ctx.arc(cx + sx * k, cy + sy * k, 3.2 * dpr, 0, Math.PI * 2);
      ctx.stroke();
    }

    // Navigation cues: waypoint toward the nearest frontier, and the biggest cluster.
    if (obs[OBS.hasFrontier]! > 0.5) {
      const len = Math.max(1.2, Math.min(LIDAR_MAX, obs[OBS.frontierDist]! * 8));
      const [sx, sy] = screen(obs[OBS.wpDir]! * len, obs[OBS.wpDir + 1]! * len);
      ctx.strokeStyle = ctx.fillStyle = '#ffb648';
      ctx.lineWidth = 2.2 * dpr;
      arrow(ctx, cx, cy, cx + sx * k, cy + sy * k, 8 * dpr);
    }
    if (obs[OBS.clusterDist]! < 0.999) {
      const len = Math.min(LIDAR_MAX, obs[OBS.clusterDist]! * 32);
      const [sx, sy] = screen(obs[OBS.clusterDir]! * len, obs[OBS.clusterDir + 1]! * len);
      ctx.setLineDash([4 * dpr, 4 * dpr]);
      ctx.strokeStyle = ctx.fillStyle = 'rgba(255, 182, 72, 0.6)';
      ctx.lineWidth = 1.5 * dpr;
      arrow(ctx, cx, cy, cx + sx * k, cy + sy * k, 6 * dpr);
      ctx.setLineDash([]);
    }
    droneGlyph(ctx, cx, cy, 7 * dpr, color);
  }

  private drawMap(canvas: HTMLCanvasElement, obs: Float32Array, visitedAt: number, frontierAt: number, color: string, coarse: boolean): void {
    const { ctx, w, h, dpr } = fit(canvas);
    ctx.fillStyle = '#060b10';
    ctx.fillRect(0, 0, w, h);
    const size = Math.min(w, h) - 8 * dpr;
    const cell = size / MAP_SIZE;
    const x0 = (w - size) / 2;
    const y0 = (h - size) / 2;
    for (let ix = 0; ix < MAP_SIZE; ix++) {
      for (let iz = 0; iz < MAP_SIZE; iz++) {
        const v = Math.min(1, obs[visitedAt + iz * MAP_SIZE + ix]!);
        const f = Math.min(1, obs[frontierAt + iz * MAP_SIZE + ix]!);
        // Forward (ix) is up, left (iz) is left.
        const col = MAP_SIZE - 1 - iz;
        const row = MAP_SIZE - 1 - ix;
        const r = 14 + VISITED[0]! * v * 0.75 + FRONTIER[0]! * f * 0.9;
        const g = 20 + VISITED[1]! * v * 0.75 + FRONTIER[1]! * f * 0.9;
        const b = 26 + VISITED[2]! * v * 0.75 + FRONTIER[2]! * f * 0.9;
        ctx.fillStyle = `rgb(${Math.min(255, r) | 0}, ${Math.min(255, g) | 0}, ${Math.min(255, b) | 0})`;
        ctx.fillRect(x0 + col * cell + 0.5, y0 + row * cell + 0.5, cell - 1, cell - 1);
      }
    }
    ctx.fillStyle = 'rgba(255,255,255,0.4)';
    ctx.font = `${10 * dpr}px ui-monospace, Menlo, monospace`;
    ctx.fillText(coarse ? '32 m' : '8 m', x0 + 3 * dpr, y0 + 11 * dpr);
    droneGlyph(ctx, x0 + size / 2, y0 + size / 2, (coarse ? 5 : 7) * dpr, color);
  }
}
