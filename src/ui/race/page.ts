import { SHIPPED_CHECKPOINT_URL } from '../../rl/checkpoint';
import { POLICY_DT } from '../../shared/constants';
import type { BuildingSpec } from '../../shared/types';
import type { LightingPreset } from '../../sim/lighting';
import { Inspector } from '../inspector/inspector';
import { CoverageChart } from './chart';
import { LaneView } from './lane';
import {
  conditionById,
  DEFAULT_PILOTS,
  END_LABELS,
  isRacePilot,
  PILOT_INFO,
  RACE_CONDITIONS,
  STUDY_SEEDS,
  type RacePilot,
  type RaceSetup,
} from './pilots';
import type { FromWorker, LaneFrame, ToWorker } from './protocol';
import { matchesStudy, STUDY_COMMIT, studyCell, studyEpisode, type StudyEpisode } from './study';

const REPO = 'https://github.com/alexnieves-cs/frontier-fly';
const SPEEDS = [
  { id: '1', label: '1×', rate: 20 },
  { id: '4', label: '4×', rate: 80 },
  { id: '10', label: '10×', rate: 200 },
  { id: 'max', label: 'Max', rate: 0 },
] as const;
type SpeedId = (typeof SPEEDS)[number]['id'];

export interface RaceLaneResult {
  pilot: RacePilot;
  coverage: number;
  collisions: number;
  steps: number;
  reason: string;
  study: StudyEpisode | null;
  match: boolean | null;
}

export interface RaceState {
  status: 'loading' | 'racing' | 'paused' | 'finished' | 'error';
  setup: RaceSetup | null;
  step: number;
  lanes: RaceLaneResult[];
  policyLoaded: boolean;
  inspected: number;
  frames: number;
}

declare global {
  interface Window {
    __race?: RaceState;
  }
}

const pct = (v: number, digits = 1) => `${(v * 100).toFixed(digits)}%`;
const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

interface Launch {
  setup: RaceSetup;
  speed: SpeedId;
  inspect: number;
}

function readLaunch(): Launch {
  const q = new URLSearchParams(location.search);
  const condition = conditionById(q.get('condition'));
  const lighting: LightingPreset = q.get('lighting') === 'night' && condition.lighting.includes('night-dark') ? 'night-dark' : 'day';
  const seedParam = Number(q.get('seed'));
  const seed = Number.isInteger(seedParam) && seedParam > 0 ? seedParam : condition.seedBase;
  const pilots = (q.get('pilots') ?? '')
    .split(',')
    .map((s) => s.trim())
    .filter(isRacePilot)
    .filter((p, i, all) => all.indexOf(p) === i)
    .slice(0, 6);
  const speedParam = q.get('speed');
  const speed = (SPEEDS.find((s) => s.id === speedParam)?.id ?? '10') as SpeedId;
  const inspect = Math.max(0, Number(q.get('inspect')) || 0);
  return { setup: { condition: condition.id, seed, lighting, pilots: pilots.length ? pilots : [...DEFAULT_PILOTS] }, speed, inspect };
}

function markup(): string {
  const conditions = RACE_CONDITIONS.map((c) => `<option value="${c.id}">${esc(c.label)}</option>`).join('');
  const speeds = SPEEDS.map((s) => `<button type="button" class="seg" data-speed="${s.id}" aria-pressed="false">${s.label}</button>`).join('');
  return `
  <div class="race">
    <header class="race-top">
      <a class="brand" href="./index.html"><span class="brand-mark" aria-hidden="true"></span>frontier-fly</a>
      <nav class="race-nav" aria-label="Pages">
        <a href="./index.html">Home</a>
        <a href="./race.html" aria-current="page">Race</a>
        <a href="./demo.html">Missions</a>
        <a href="./sim.html">Sim</a>
        <a href="${REPO}" rel="noopener">GitHub</a>
      </nav>
    </header>

    <section class="race-intro">
      <p class="kicker">LIVE PILOT RACE · RUNNING IN YOUR TAB</p>
      <h1>Four pilots, one building. Who searches more of it?</h1>
      <p class="pitch">Each drone gets its own copy of the same building, the same start and the same physics, and has two minutes of flight to cover as much floor as it can.
      Nothing here is prerecorded: every flight is computed live in a Web Worker, and on a study building it lands on exactly the number the published benchmark reports.</p>
    </section>

    <section class="race-controls" aria-label="Race setup">
      <label class="ctl">
        <span class="ctl-label">Building type</span>
        <select id="race-condition">${conditions}</select>
      </label>
      <div class="ctl">
        <span class="ctl-label">Light</span>
        <div class="segs" role="group" aria-label="Lighting">
          <button type="button" class="seg" data-light="day" aria-pressed="true">Day</button>
          <button type="button" class="seg" data-light="night-dark" aria-pressed="false">Night</button>
        </div>
      </div>
      <div class="ctl">
        <span class="ctl-label">Building</span>
        <div class="stepper">
          <button type="button" class="seg icon" id="race-prev" aria-label="Previous building">‹</button>
          <span class="stepper-value" id="race-building">–</span>
          <button type="button" class="seg icon" id="race-next" aria-label="Next building">›</button>
          <button type="button" class="seg" id="race-random" title="A building the study never flew">Random</button>
        </div>
      </div>
      <div class="ctl">
        <span class="ctl-label">Speed</span>
        <div class="segs" role="group" aria-label="Playback speed">${speeds}</div>
      </div>
      <div class="ctl ctl-actions">
        <button type="button" class="btn primary" id="race-play" disabled>Pause</button>
        <button type="button" class="btn" id="race-restart" disabled>Restart</button>
        <button type="button" class="btn ghost" id="race-share">Copy link</button>
      </div>
    </section>

    <p class="race-banner" id="race-banner" hidden></p>

    <main class="race-main">
      <div class="race-lanes" id="race-lanes"></div>
    </main>

    <section class="race-result" id="race-result" hidden aria-live="polite"></section>

    <div class="race-deck">
      <aside class="race-side">
        <div class="clock">
          <div class="clock-row"><span class="clock-time" id="race-time">0.0 s</span><span class="clock-of" id="race-of">of 120 s flight</span></div>
          <div class="clock-bar"><span id="race-progress"></span></div>
          <p class="clock-note" id="race-status">Loading the policy and building the world…</p>
        </div>
        <ol class="board" id="race-board" aria-label="Live standings"></ol>
        <figure class="chart-wrap">
          <figcaption>Coverage over flight time</figcaption>
          <canvas class="chart" id="race-chart" role="img" aria-label="Coverage over time for each pilot"></canvas>
        </figure>
      </aside>
      <section class="race-inspector" aria-labelledby="ins-title">
        <div class="ins-head">
          <div>
            <h2 id="ins-title">What the policy sees</h2>
            <p>The exact numbers the network reads every 50 ms, drawn from the drone's point of view. Pick a lane to look through its sensors.</p>
          </div>
          <div class="segs ins-tabs" id="ins-tabs" role="tablist" aria-label="Lane to inspect"></div>
        </div>
        <div class="inspector" id="race-inspector"></div>
      </section>
    </div>

    <section class="race-notes">
      <h2>Reading the race fairly</h2>
      <div class="notes-grid">
        <div><h3>Coverage</h3><p>The share of the floor the drone could reach that it actually flew over, in 0.5 m cells. Dark areas are unexplored, grey is what its lidar has seen, color is where it has been. Red rings are collisions.</p></div>
        <div><h3>Who knows what</h3><p>The policy, the planner and the follower only have their own sensors. The oracle reads the true floor plan: it is the teacher the policy learned from, there as a ceiling, not a rival.</p></div>
        <div><h3>One building is noise</h3><p>A single race can go either way. The averages at the end come from the published study of 30 buildings per condition, with 95% confidence intervals.</p></div>
        <div><h3>Reproducible</h3><p>Same building, same seed, same result, in any browser. The study numbers come from <code>npm run study:coverage</code> at commit <code>${esc(STUDY_COMMIT)}</code>.</p></div>
      </div>
    </section>

    <footer class="race-foot">
      <span>frontier-fly · civilian search and rescue research · MIT</span>
      <span><a href="${REPO}/blob/main/docs/REPORT.md" rel="noopener">Research report</a> · <a href="${REPO}" rel="noopener">Source</a></span>
    </footer>
  </div>`;
}

function laneMarkup(pilot: RacePilot, index: number): string {
  const info = PILOT_INFO[pilot];
  return `
    <article class="lane" data-lane="${index}" style="--lane:${info.color}" tabindex="0" role="button" aria-label="Inspect the ${esc(info.name)}" aria-pressed="false">
      <header class="lane-head">
        <span class="lane-dot" aria-hidden="true"></span>
        <span class="lane-name"><span class="long">${esc(info.name)}</span><span class="short">${esc(info.short)}</span></span>
        <span class="lane-cov" data-f="cov">0.0%</span>
      </header>
      <div class="lane-view">
        <canvas class="lane-canvas" aria-label="Top-down view of the ${esc(info.name)}"></canvas>
        <span class="lane-floor" data-f="floor" hidden></span>
        <span class="lane-end" data-f="end" hidden></span>
      </div>
      <footer class="lane-foot">
        <span class="lane-knows">${esc(info.knows)}</span>
        <span class="lane-coll" data-f="coll">0 hits</span>
      </footer>
    </article>`;
}

class RacePage {
  private readonly worker: Worker;
  private setup: RaceSetup;
  private speed: SpeedId;
  private policyLoaded = false;
  private spec: BuildingSpec | null = null;
  /** The lanes the worker is flying, which can be fewer than asked when the checkpoint is missing. */
  private pilots: RacePilot[] = [];
  private maxSteps = 2400;
  private views: LaneView[] = [];
  private laneEls: HTMLElement[] = [];
  private latest: Extract<FromWorker, { type: 'frame' }> | null = null;
  private paused = false;
  private finished = false;
  private inspected: number;
  private readonly inspector: Inspector;
  private readonly chart: CoverageChart;
  private frames = 0;
  private lastDom = 0;
  private dirty = false;
  private shownResult = false;
  /** Frames posted for the previous race can still arrive after a restart; drop them until the new start. */
  private awaitingStart = true;
  private readonly el: Record<string, HTMLElement>;

  constructor(private readonly root: HTMLElement) {
    const launch = readLaunch();
    this.setup = launch.setup;
    this.speed = launch.speed;
    this.inspected = launch.inspect;
    root.innerHTML = markup();
    const ids = ['condition', 'building', 'prev', 'next', 'random', 'play', 'restart', 'share', 'banner', 'lanes', 'time', 'of', 'progress', 'status', 'board', 'chart', 'result', 'inspector'];
    this.el = Object.fromEntries(ids.map((id) => [id, root.querySelector<HTMLElement>(`#race-${id}`)!]));
    this.inspector = new Inspector(this.el.inspector!);
    this.chart = new CoverageChart(this.el.chart as HTMLCanvasElement);
    this.worker = new Worker(new URL('./race.worker.ts', import.meta.url), { type: 'module', name: 'frontier-race' });
    this.worker.onmessage = (event: MessageEvent<FromWorker>) => this.onMessage(event.data);
    this.worker.onerror = (event) => this.fail(event.message || 'The race worker failed to start.');
    this.bind();
    this.syncControls();
    this.publish('loading');
    this.send({ type: 'init', checkpointUrl: new URL(SHIPPED_CHECKPOINT_URL, location.href).href });
    requestAnimationFrame(this.draw);
  }

  private send(msg: ToWorker): void {
    this.worker.postMessage(msg);
  }

  private rate(): number {
    return SPEEDS.find((s) => s.id === this.speed)!.rate;
  }

  private bind(): void {
    const select = this.el.condition as HTMLSelectElement;
    select.addEventListener('change', () => {
      const c = conditionById(select.value);
      this.restart({ ...this.setup, condition: c.id, seed: c.seedBase, lighting: c.lighting.includes(this.setup.lighting) ? this.setup.lighting : 'day' });
    });
    this.root.querySelectorAll<HTMLButtonElement>('[data-light]').forEach((b) =>
      b.addEventListener('click', () => this.restart({ ...this.setup, lighting: b.dataset.light as LightingPreset })),
    );
    this.root.querySelectorAll<HTMLButtonElement>('[data-speed]').forEach((b) =>
      b.addEventListener('click', () => {
        this.speed = b.dataset.speed as SpeedId;
        this.send({ type: 'rate', rate: this.rate() });
        this.syncControls();
      }),
    );
    const step = (d: number) => {
      const c = conditionById(this.setup.condition);
      const i = this.setup.seed - c.seedBase;
      const inStudy = i >= 0 && i < STUDY_SEEDS;
      const next = inStudy ? (i + d + STUDY_SEEDS) % STUDY_SEEDS : 0;
      this.restart({ ...this.setup, seed: c.seedBase + next });
    };
    this.el.prev!.addEventListener('click', () => step(-1));
    this.el.next!.addEventListener('click', () => step(1));
    this.el.random!.addEventListener('click', () => this.restart({ ...this.setup, seed: 100_000 + Math.floor(Math.random() * 900_000) }));
    this.el.play!.addEventListener('click', () => {
      if (this.finished) {
        this.restart(this.setup);
        return;
      }
      this.paused = !this.paused;
      this.send({ type: 'pause', paused: this.paused });
      this.syncControls();
      this.publish(this.paused ? 'paused' : 'racing');
    });
    this.el.restart!.addEventListener('click', () => this.restart(this.setup));
    this.el.share!.addEventListener('click', () => {
      const url = location.href;
      const done = () => {
        this.el.share!.textContent = 'Link copied';
        window.setTimeout(() => (this.el.share!.textContent = 'Copy link'), 1600);
      };
      if (navigator.clipboard) void navigator.clipboard.writeText(url).then(done, () => window.prompt('Copy this link', url));
      else window.prompt('Copy this link', url);
    });
    window.addEventListener('keydown', (e) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
      if (e.key === ' ' && document.activeElement === document.body) {
        e.preventDefault();
        (this.el.play as HTMLButtonElement).click();
      }
    });
  }

  private restart(setup: RaceSetup): void {
    this.setup = setup;
    this.finished = false;
    this.paused = false;
    this.shownResult = false;
    this.latest = null;
    this.awaitingStart = true;
    this.el.result!.hidden = true;
    this.el.status!.textContent = 'Building the world…';
    this.syncControls();
    this.writeUrl();
    this.publish('loading');
    this.send({ type: 'start', setup: this.runnableSetup(), rate: this.rate() });
  }

  /** The setup minus lanes that need the checkpoint when it could not load. */
  private runnableSetup(): RaceSetup {
    if (this.policyLoaded) return this.setup;
    const pilots = this.setup.pilots.filter((p) => p !== 'policy' && p !== 'guarded');
    return { ...this.setup, pilots: pilots.length ? pilots : ['classical', 'waypoint', 'teacher'] };
  }

  private writeUrl(): void {
    const q = new URLSearchParams();
    q.set('condition', this.setup.condition);
    q.set('seed', String(this.setup.seed));
    if (this.setup.lighting !== 'day') q.set('lighting', 'night');
    if (this.setup.pilots.join(',') !== DEFAULT_PILOTS.join(',')) q.set('pilots', this.setup.pilots.join(','));
    if (this.speed !== '10') q.set('speed', this.speed);
    history.replaceState(null, '', `${location.pathname}?${q.toString()}`);
  }

  private syncControls(): void {
    const c = conditionById(this.setup.condition);
    (this.el.condition as HTMLSelectElement).value = c.id;
    this.root.querySelectorAll<HTMLButtonElement>('[data-light]').forEach((b) => {
      b.setAttribute('aria-pressed', String(b.dataset.light === this.setup.lighting));
      b.disabled = !c.lighting.includes(b.dataset.light as LightingPreset);
    });
    this.root.querySelectorAll<HTMLButtonElement>('[data-speed]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.speed === this.speed)));
    const i = this.setup.seed - c.seedBase;
    this.el.building!.textContent = i >= 0 && i < STUDY_SEEDS ? `Study ${i + 1} of ${STUDY_SEEDS}` : `Random #${this.setup.seed}`;
    this.el.building!.title = `Seed ${this.setup.seed}`;
    this.el.play!.textContent = this.finished ? 'Race again' : this.paused ? 'Resume' : 'Pause';
  }

  private onMessage(msg: FromWorker): void {
    if (msg.type === 'ready') {
      this.policyLoaded = msg.policy;
      if (!msg.policy) this.banner(`The policy checkpoint could not load here (${msg.note}), so the race runs without it.`);
      this.send({ type: 'start', setup: this.runnableSetup(), rate: this.rate() });
      this.writeUrl();
    } else if (msg.type === 'error') {
      this.fail(msg.message);
    } else if (msg.type === 'started') {
      this.onStarted(msg);
    } else if (msg.type === 'frame') {
      if (!this.awaitingStart) this.onFrame(msg);
    }
  }

  private onStarted(msg: Extract<FromWorker, { type: 'started' }>): void {
    this.awaitingStart = false;
    this.spec = msg.spec;
    this.maxSteps = msg.maxSteps;
    const pilots = msg.setup.pilots;
    this.pilots = pilots;
    this.el.lanes!.innerHTML = pilots.map(laneMarkup).join('');
    this.el.lanes!.dataset.count = String(pilots.length);
    this.laneEls = [...this.el.lanes!.querySelectorAll<HTMLElement>('.lane')];
    this.views = this.laneEls.map((el, i) => {
      const view = new LaneView(el.querySelector('canvas')!, PILOT_INFO[pilots[i]!].color);
      view.reset(msg.spec);
      el.addEventListener('click', () => this.inspect(i));
      el.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          this.inspect(i);
        }
      });
      return view;
    });
    const tabs = this.root.querySelector<HTMLElement>('#ins-tabs')!;
    tabs.innerHTML = pilots
      .map((p, i) => `<button type="button" class="seg" role="tab" data-ins-lane="${i}" style="--lane:${PILOT_INFO[p].color}" aria-selected="false"><i class="tab-dot"></i>${PILOT_INFO[p].short}</button>`)
      .join('');
    tabs.querySelectorAll<HTMLButtonElement>('[data-ins-lane]').forEach((b) => b.addEventListener('click', () => this.inspect(Number(b.dataset.insLane))));
    this.chart.reset(pilots.map((p) => PILOT_INFO[p].color), msg.maxSteps * POLICY_DT);
    this.el.of!.textContent = `of ${Math.round(msg.maxSteps * POLICY_DT)} s flight`;
    this.inspect(Math.min(this.inspected, pilots.length - 1));
    const c = conditionById(msg.setup.condition);
    const night = msg.setup.lighting !== 'day' ? ', at night with the lights off' : '';
    this.el.status!.textContent = `${c.label}${night}. ${msg.spec.floors.length > 1 ? `${msg.spec.floors.length} floors. ` : ''}Seed ${msg.setup.seed}.`;
    this.el.play!.removeAttribute('disabled');
    this.el.restart!.removeAttribute('disabled');
    this.root.dataset.ready = '1';
    this.publish('racing');
  }

  private inspect(lane: number): void {
    this.inspected = lane;
    this.laneEls.forEach((el, i) => {
      el.classList.toggle('inspected', i === lane);
      el.setAttribute('aria-pressed', String(i === lane));
    });
    this.root.querySelectorAll<HTMLElement>('[data-ins-lane]').forEach((b) => b.setAttribute('aria-selected', String(Number(b.dataset.insLane) === lane)));
    this.send({ type: 'inspect', lane });
  }

  private onFrame(msg: Extract<FromWorker, { type: 'frame' }>): void {
    this.latest = msg;
    this.frames += 1;
    this.dirty = true;
    const now = performance.now();
    const t = msg.step * POLICY_DT;
    msg.lanes.forEach((l, i) => {
      this.views[i]?.update(l, now);
      this.chart.push(i, l.steps * POLICY_DT, l.coverage);
    });
    if (msg.finished && !this.finished) {
      this.finished = true;
      this.syncControls();
    }
    // Text updates at about 10 Hz; the canvases redraw every animation frame.
    if (now - this.lastDom > 100 || msg.finished) {
      this.lastDom = now;
      this.el.time!.textContent = `${t.toFixed(1)} s`;
      (this.el.progress as HTMLElement).style.width = `${Math.min(100, (msg.step / this.maxSteps) * 100)}%`;
      this.renderBoard(msg.lanes);
      if (msg.inspect) {
        const info = PILOT_INFO[msg.inspect.pilot];
        this.inspector.render({ ...msg.inspect, name: info.name, color: info.color, readsObs: info.readsObs, knows: info.knows });
      }
    }
    if (msg.finished && !this.shownResult) {
      this.shownResult = true;
      this.renderResult(msg.lanes);
      this.publish('finished');
    } else if (this.frames % 10 === 0) {
      this.publish(this.paused ? 'paused' : 'racing');
    }
  }

  private renderBoard(lanes: LaneFrame[]): void {
    const order = lanes.map((l, i) => ({ l, i })).sort((a, b) => b.l.coverage - a.l.coverage);
    const top = order[0]?.l.coverage || 1;
    this.el.board!.innerHTML = order
      .map(({ l, i }, rank) => {
        const info = PILOT_INFO[this.pilots[i]!];
        const done = l.reason !== 'running';
        return `<li class="row${done ? ' done' : ''}" style="--lane:${info.color}">
          <span class="row-rank">${rank + 1}</span>
          <span class="row-name">${esc(info.name)}<small>${done ? esc(END_LABELS[l.reason as keyof typeof END_LABELS]) : `${l.collisions} hit${l.collisions === 1 ? '' : 's'}`}</small></span>
          <span class="row-bar"><span style="width:${((l.coverage / Math.max(top, 1e-6)) * 100).toFixed(1)}%"></span></span>
          <span class="row-val">${pct(l.coverage)}</span>
        </li>`;
      })
      .join('');
    lanes.forEach((l, i) => {
      const el = this.laneEls[i];
      if (!el) return;
      el.querySelector('[data-f="cov"]')!.textContent = pct(l.coverage);
      el.querySelector('[data-f="coll"]')!.textContent = `${l.collisions} hit${l.collisions === 1 ? '' : 's'}`;
      el.classList.toggle('hit', l.collisions > 0);
      const floor = el.querySelector<HTMLElement>('[data-f="floor"]')!;
      const floors = this.spec?.floors.length ?? 1;
      floor.hidden = floors < 2;
      floor.textContent = `Floor ${l.floor + 1}/${floors}`;
      const end = el.querySelector<HTMLElement>('[data-f="end"]')!;
      end.hidden = l.reason === 'running';
      end.textContent = END_LABELS[l.reason as keyof typeof END_LABELS] ?? '';
      el.classList.toggle('ended', l.reason !== 'running');
    });
  }

  private results(lanes: LaneFrame[]): RaceLaneResult[] {
    const pilots = this.pilots;
    return lanes.map((l, i) => {
      const pilot = pilots[i]!;
      const study = studyEpisode(this.setup.condition, this.setup.seed, this.setup.lighting, pilot);
      const live = { coverage: l.coverage, collisions: l.collisions, steps: l.steps };
      return { pilot, ...live, reason: l.reason, study, match: study ? matchesStudy(live, study) : null };
    });
  }

  private renderResult(lanes: LaneFrame[]): void {
    const rs = this.results(lanes);
    const ranked = [...rs].sort((a, b) => b.coverage - a.coverage);
    const fair = ranked.filter((r) => r.pilot !== 'teacher');
    const winner = fair[0];
    const inStudy = rs.some((r) => r.study);
    const allMatch = inStudy && rs.every((r) => r.match);
    const rows = ranked
      .map((r, k) => {
        const info = PILOT_INFO[r.pilot];
        const cell = studyCell(this.setup.condition, this.setup.lighting, r.pilot);
        const studyCol = r.study
          ? `${pct(r.study.coverage)} <span class="match ${r.match ? 'ok' : 'off'}">${r.match ? '✓ identical' : '≠ differs'}</span>`
          : '<span class="muted">not in study</span>';
        const avg = cell ? `${pct(cell.coverage)} <span class="muted">[${pct(cell.coverageCi[0], 0)}–${pct(cell.coverageCi[1], 0)}]</span>` : '–';
        return `<tr style="--lane:${info.color}">
          <td class="r-rank">${k + 1}</td>
          <td class="r-name"><i class="lane-dot"></i>${esc(info.name)}</td>
          <td class="num strong">${pct(r.coverage)}</td>
          <td class="num">${r.collisions}</td>
          <td>${esc(END_LABELS[r.reason as keyof typeof END_LABELS])}</td>
          <td class="num">${studyCol}</td>
          <td class="num">${avg}</td>
        </tr>`;
      })
      .join('');
    const c = conditionById(this.setup.condition);
    const headline = winner
      ? `${PILOT_INFO[winner.pilot].name} covers the most of this building${fair.length > 1 ? `, ${((winner.coverage - fair[1]!.coverage) * 100).toFixed(1)} points ahead of the ${PILOT_INFO[fair[1]!.pilot].name.toLowerCase()}` : ''}.`
      : 'Race finished.';
    const oracle = rs.find((r) => r.pilot === 'teacher');
    const oracleNote = oracle ? ` The oracle, which reads the floor plan, reached ${pct(oracle.coverage)}.` : '';
    const repro = inStudy
      ? allMatch
        ? `Every lane ended exactly where the published study's flight on this building did: same coverage, same collisions, same step.`
        : `At least one lane differs from the published study on this building. The study was measured at commit ${esc(STUDY_COMMIT)}; a later code change can move it.`
      : 'This building is not one of the 30 the study flew, so there is no published number to compare with. The averages still apply.';
    this.el.result!.innerHTML = `
      <div class="result-head">
        <div>
          <p class="kicker">RESULT</p>
          <h2>${esc(headline)}</h2>
          <p class="result-sub">${esc(oracleNote.trim())}</p>
        </div>
        <span class="repro ${allMatch ? 'ok' : inStudy ? 'off' : ''}">${allMatch ? '✓ Reproduces the study' : inStudy ? 'Differs from the study' : 'Off-study building'}</span>
      </div>
      <div class="table-wrap">
        <table class="result-table">
          <thead><tr><th>#</th><th>Pilot</th><th class="num">Coverage</th><th class="num">Hits</th><th>Ended</th><th class="num">Study, this building</th><th class="num">Study, ${STUDY_SEEDS} buildings</th></tr></thead>
          <tbody>${rows}</tbody>
        </table>
      </div>
      <p class="result-foot">${repro} The right-hand column is the average over all ${STUDY_SEEDS} ${esc(c.label.toLowerCase())} buildings in ${this.setup.lighting === 'day' ? 'daylight' : 'the dark'}, with its 95% confidence interval: that, not one race, is the honest comparison.</p>
      <div class="result-actions">
        <button type="button" class="btn primary" data-act="next">Next building</button>
        <button type="button" class="btn" data-act="again">Race again</button>
      </div>`;
    this.el.result!.hidden = false;
    this.el.result!.querySelector('[data-act="next"]')!.addEventListener('click', () => (this.el.next as HTMLButtonElement).click());
    this.el.result!.querySelector('[data-act="again"]')!.addEventListener('click', () => this.restart(this.setup));
  }

  private draw = (): void => {
    requestAnimationFrame(this.draw);
    const msg = this.latest;
    if (!msg || !this.spec) return;
    const now = performance.now();
    // Keep collision rings fading while paused or finished, otherwise redraw only on new data.
    const animating = this.views.length > 0 && msg.lanes.some((l) => l.collisions > 0);
    if (!this.dirty && !animating) return;
    this.dirty = false;
    this.views.forEach((v, i) => v.draw(msg.lanes[i] ?? null, msg.visited[i] ?? null, msg.seen[i] ?? null, now));
    this.chart.draw();
  };

  private banner(text: string): void {
    this.el.banner!.textContent = text;
    this.el.banner!.hidden = false;
  }

  private fail(message: string): void {
    this.banner(`The race could not run here: ${message}`);
    this.el.status!.textContent = 'Stopped.';
    this.root.dataset.ready = '1';
    this.publish('error');
  }

  private publish(status: RaceState['status']): void {
    window.__race = {
      status,
      setup: this.setup,
      step: this.latest?.step ?? 0,
      lanes: this.latest ? this.results(this.latest.lanes) : [],
      policyLoaded: this.policyLoaded,
      inspected: this.inspected,
      frames: this.frames,
    };
  }
}

export function startRace(root: HTMLElement): void {
  new RacePage(root);
}
