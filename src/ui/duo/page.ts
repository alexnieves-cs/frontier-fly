import { createDuo, TWO_WAYS_IN, type DuoFlight } from '../../mission/autonomy/duo';
import type { LogEntry, RunnerSnapshot } from '../../mission/autonomy/contracts';
import { formatClock, PHASE_LABELS } from '../demo/format';
import { siteHeader } from '../siteNav';
import { DRONE_COLOURS, DRONE_DRAW_SCALE, DuoView, type DroneName, type DuoCamera } from './view';

type Speed = 1 | 4 | 16;
const SPEEDS: Speed[] = [1, 4, 16];
const CAMERAS: { id: DuoCamera; label: string }[] = [
  { id: 'overview', label: 'Overview' },
  { id: 'A', label: 'Follow A' },
  { id: 'B', label: 'Follow B' },
];
/** Wall-clock budget for sim ticks per frame, ms. */
const FRAME_BUDGET_MS = 22;
const LOG_LINES = 6;

interface DroneState {
  wayIn: string | null;
  /** Time of the last log line read, and how many lines at that time were read (the snapshot holds the last 50). */
  lastT: number;
  seenAtLastT: number;
  lastLine: string;
}

/** Ticks between log reads; well inside the 50 lines a snapshot carries. */
const READ_EVERY = 4;

interface DuoApi {
  ready: boolean;
  error: string | null;
  tSec: number;
  done: boolean;
  /** Fly (without drawing) to team time t, then draw one frame with `camera`. For stills. */
  advanceTo(t: number, camera?: DuoCamera): Promise<void>;
  summary(): unknown;
}

declare global {
  interface Window {
    __duo?: DuoApi;
  }
}

const ENTER = /^Entering (.+) through opening (\d+) \(([^)]+)\)/;
const CONFIRMED = /^Confirmed (\w+) at \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)/;

export function startDuo(root: HTMLElement): void {
  const params = new URLSearchParams(location.search);
  const seed = Number.parseInt(params.get('seed') ?? '', 10);
  const page = new DuoPage(root, Number.isFinite(seed) ? seed >>> 0 : TWO_WAYS_IN.world.seed);
  void page.load();
}

class DuoPage {
  private flight: DuoFlight | null = null;
  private view: DuoView | null = null;
  private speed: Speed = 4;
  private paused = false;
  private camera: DuoCamera = 'overview';
  private readonly state: Record<DroneName, DroneState> = {
    A: { wayIn: null, lastT: -Infinity, seenAtLastT: 0, lastLine: '' },
    B: { wayIn: null, lastT: -Infinity, seenAtLastT: 0, lastLine: '' },
  };
  private readonly lines: { t: number; name: DroneName; text: string }[] = [];
  private found: { t: number; name: DroneName } | null = null;
  private ticks = 0;
  private frame = 0;
  private lastWall = 0;
  private readonly api: DuoApi;

  constructor(
    private readonly root: HTMLElement,
    private seed: number,
  ) {
    this.root.innerHTML = this.markup();
    this.api = {
      ready: false,
      error: null,
      tSec: 0,
      done: false,
      advanceTo: (t, camera) => this.advanceTo(t, camera),
      summary: () => this.flight?.result(),
    };
    window.__duo = this.api;
    this.bind();
  }

  async load(): Promise<void> {
    this.api.ready = false;
    cancelAnimationFrame(this.frame);
    this.setStatus('Building the site and both drones…');
    try {
      const flight = await createDuo({ ...TWO_WAYS_IN.world, seed: this.seed }, TWO_WAYS_IN.mission, TWO_WAYS_IN.runner);
      this.flight = flight;
      this.reset();
      const canvas = this.root.querySelector<HTMLCanvasElement>('[data-canvas]')!;
      this.view ??= new DuoView(canvas);
      this.view.setCamera(this.camera);
      await this.view.setWorld(flight.worldA);
      this.view.resize();
      this.root.querySelector('[data-seed]')!.textContent = String(this.seed);
      this.draw();
      this.setStatus('');
      this.api.ready = true;
      this.root.dataset.ready = '1';
      if (!params().has('still')) {
        this.lastWall = performance.now();
        this.frame = requestAnimationFrame(this.tick);
      }
    } catch (error) {
      this.api.error = String(error);
      this.setStatus(`Could not build this flight: ${String(error)}`);
    }
  }

  private reset(): void {
    for (const name of ['A', 'B'] as const) this.state[name] = { wayIn: null, lastT: -Infinity, seenAtLastT: 0, lastLine: '' };
    this.lines.length = 0;
    this.found = null;
    this.ticks = 0;
  }

  private readonly tick = (now: number): void => {
    const flight = this.flight;
    if (!flight) return;
    const wall = Math.min(0.1, (now - this.lastWall) / 1000);
    this.lastWall = now;
    if (!this.paused && !flight.done) {
      const want = Math.round((wall * this.speed) / flight.dt);
      const start = performance.now();
      for (let k = 0; k < want && !flight.done && performance.now() - start < FRAME_BUDGET_MS; k++) {
        flight.step();
        this.track();
      }
    }
    this.draw();
    this.frame = requestAnimationFrame(this.tick);
  };

  private async advanceTo(t: number, camera?: DuoCamera): Promise<void> {
    const flight = this.flight;
    if (!flight) return;
    if (camera) {
      this.camera = camera;
      this.view?.setCamera(camera);
      this.syncButtons();
    }
    while (!flight.done && flight.tSec < t) {
      flight.step();
      this.track();
    }
    this.track(true);
    this.view?.snapCamera();
    this.draw();
  }

  /** Read new log lines every few ticks, so the way in and the find land close to when they happen. */
  private track(force = false): void {
    this.ticks += 1;
    if (!force && this.ticks % READ_EVERY !== 0 && !this.flight!.done) return;
    const flight = this.flight!;
    const snaps = this.snaps();
    const t = flight.tSec;
    if (!flight.a.done) this.view?.trailPoint('A', snaps.A.pose.pos, t);
    if (!flight.b.done && t >= flight.launchGapSec) this.view?.trailPoint('B', snaps.B.pose.pos, t);
    for (const name of ['A', 'B'] as const) {
      const log = snaps[name].log;
      const st = this.state[name];
      const offset = name === 'B' ? flight.launchGapSec : 0;
      let start = 0;
      let same = 0;
      for (; start < log.length; start++) {
        const e = log[start]!;
        if (e.tSec > st.lastT) break;
        if (e.tSec === st.lastT && ++same > st.seenAtLastT) break;
      }
      for (let i = start; i < log.length; i++) this.onLog(name, log[i]!, offset);
      const last = log[log.length - 1];
      if (last) {
        st.lastT = last.tSec;
        st.seenAtLastT = log.filter((e) => e.tSec === last.tSec).length;
      }
    }
  }

  private onLog(name: DroneName, entry: LogEntry, offset: number): void {
    const t = entry.tSec + offset;
    const st = this.state[name];
    const enter = ENTER.exec(entry.message);
    if (enter) {
      st.wayIn = `opening ${enter[2]}, ${enter[3]}`;
      const building = this.flight!.worldA.buildings.find((b) => b.label === enter[1]);
      if (building) this.view?.markEntry(building.id, Number(enter[2]), name);
    }
    const confirmed = CONFIRMED.exec(entry.message);
    if (confirmed) {
      this.view?.markContact([Number(confirmed[2]), Number(confirmed[3]), Number(confirmed[4])], name);
      this.found ??= { t, name };
    }
    if (entry.message.startsWith('Observed opening')) return;
    st.lastLine = entry.message;
    this.lines.push({ t, name, text: entry.message });
    if (this.lines.length > LOG_LINES) this.lines.shift();
  }

  private snaps(): Record<DroneName, RunnerSnapshot> {
    return { A: this.flight!.a.snapshot(), B: this.flight!.b.snapshot() };
  }

  private draw(panel = true): void {
    const flight = this.flight;
    if (!flight || !this.view) return;
    const snaps = this.snaps();
    const t = flight.tSec;
    const flying = { A: !flight.a.done && t > 0, B: !flight.b.done && t >= flight.launchGapSec };
    this.view.render(t, snaps, flying);
    this.api.tSec = t;
    this.api.done = flight.done;
    if (!panel) return;
    this.root.querySelector('[data-clock]')!.textContent = formatClock(t);
    for (const name of ['A', 'B'] as const) {
      const card = this.root.querySelector<HTMLElement>(`[data-drone="${name}"]`)!;
      const snap = snaps[name];
      const waiting = name === 'B' && t < flight.launchGapSec;
      // With its opening chosen, the way round to it is flown as an approach: say where it is going.
      const toOpening = snap.phase === 'approach' && this.state[name].wayIn !== null && !snap.inside;
      card.querySelector('[data-phase]')!.textContent = waiting ? 'On its pad' : toOpening ? 'To its way in' : PHASE_LABELS[snap.phase];
      card.querySelector('[data-way]')!.textContent = this.state[name].wayIn ?? 'Not chosen yet';
      card.querySelector('[data-battery]')!.textContent = `${Math.round(snap.batteryPct)}%`;
      card.querySelector('[data-line]')!.textContent = this.state[name].lastLine;
    }
    const ways = (['A', 'B'] as const).map((n) => this.state[n].wayIn).filter(Boolean);
    this.root.querySelector('[data-found]')!.textContent = this.found ? `${formatClock(this.found.t)} by ${this.found.name}` : 'Not yet';
    this.root.querySelector('[data-ways]')!.textContent =
      ways.length === 2 ? (ways[0] === ways[1] ? 'Same opening' : 'Two different openings') : ways.length === 1 ? 'One so far' : 'None yet';
    const closest = flight.minSeparationM;
    this.root.querySelector('[data-closest]')!.textContent = Number.isFinite(closest) ? `${closest.toFixed(2)} m` : 'n/a';
    this.root.querySelector('[data-log]')!.innerHTML = this.lines
      .map((l) => `<li><span class="duo-t">${formatClock(l.t)}</span><span class="duo-tag" style="--c:${DRONE_COLOURS[l.name]}">${l.name}</span>${escapeHtml(l.text)}</li>`)
      .join('');
    if (flight.done) this.root.querySelector('[data-pause]')!.textContent = 'Done';
  }

  private bind(): void {
    this.root.addEventListener('click', (event) => {
      const el = (event.target as Element).closest<HTMLElement>('[data-speed],[data-camera],[data-pause],[data-restart],[data-next]');
      if (!el) return;
      if (el.dataset.speed) this.speed = Number(el.dataset.speed) as Speed;
      if (el.dataset.camera) {
        this.camera = el.dataset.camera as DuoCamera;
        this.view?.setCamera(this.camera);
      }
      if (el.hasAttribute('data-pause') && this.flight && !this.flight.done) this.paused = !this.paused;
      if (el.hasAttribute('data-restart')) void this.load();
      if (el.hasAttribute('data-next')) {
        this.seed += 1;
        const url = new URL(location.href);
        url.searchParams.set('seed', String(this.seed));
        history.replaceState(null, '', url);
        void this.load();
      }
      this.syncButtons();
    });
    window.addEventListener('resize', () => this.view?.resize());
    this.syncButtons();
  }

  private syncButtons(): void {
    for (const b of this.root.querySelectorAll<HTMLElement>('[data-speed]')) b.setAttribute('aria-pressed', String(Number(b.dataset.speed) === this.speed));
    for (const b of this.root.querySelectorAll<HTMLElement>('[data-camera]')) b.setAttribute('aria-pressed', String(b.dataset.camera === this.camera));
    const pause = this.root.querySelector('[data-pause]');
    if (pause && !this.flight?.done) pause.textContent = this.paused ? 'Play' : 'Pause';
  }

  private setStatus(text: string): void {
    const el = this.root.querySelector<HTMLElement>('[data-status]')!;
    el.textContent = text;
    el.hidden = !text;
  }

  private markup(): string {
    const card = (name: DroneName) => `
      <section class="duo-card" data-drone="${name}" style="--c:${DRONE_COLOURS[name]}">
        <header><span class="duo-dot"></span><h2>Drone ${name}</h2><span class="duo-phase" data-phase></span></header>
        <dl>
          <div><dt>Way in</dt><dd data-way></dd></div>
          <div><dt>Battery</dt><dd data-battery></dd></div>
        </dl>
        <p class="duo-line" data-line></p>
      </section>`;
    return `
      ${siteHeader('duo', { page: 'Two ways in' })}
      <main class="duo">
        <canvas class="duo-canvas" data-canvas aria-label="Two drones searching a cabin in the simulator"></canvas>
        <div class="duo-intro">
          <p class="duo-kicker">Two drones · one cabin · seed <span data-seed></span></p>
          <p class="duo-lede">They split the outside survey, share what they measured, and each takes its own way in.</p>
          <p class="duo-honest">Flown live by the simulator in this tab. The drones, the woods, the cabin and the hiker are all simulated. Drones drawn ${DRONE_DRAW_SCALE}× life size.</p>
        </div>
        <aside class="duo-side">
          ${card('A')}
          ${card('B')}
          <section class="duo-card duo-team">
            <dl>
              <div><dt>Mission time</dt><dd data-clock>0:00</dd></div>
              <div><dt>Hiker confirmed</dt><dd data-found>Not yet</dd></div>
              <div><dt>Ways in used</dt><dd data-ways>None yet</dd></div>
              <div><dt>Closest pass</dt><dd data-closest>n/a</dd></div>
            </dl>
          </section>
        </aside>
        <ol class="duo-log" data-log aria-live="polite"></ol>
        <div class="duo-controls" role="toolbar" aria-label="Playback">
          <div class="duo-group" aria-label="Speed">${SPEEDS.map((s) => `<button data-speed="${s}">${s}×</button>`).join('')}</div>
          <div class="duo-group" aria-label="Camera">${CAMERAS.map((c) => `<button data-camera="${c.id}">${c.label}</button>`).join('')}</div>
          <div class="duo-group"><button data-pause>Pause</button><button data-restart>Restart</button><button data-next>Next seed</button></div>
        </div>
        <p class="duo-status" data-status hidden></p>
      </main>`;
  }
}

function params(): URLSearchParams {
  return new URLSearchParams(location.search);
}

function escapeHtml(s: string): string {
  return s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);
}
