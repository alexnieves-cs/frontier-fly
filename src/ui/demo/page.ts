import type {
  AutonomyReport,
  EntryCandidate,
  LogEntry,
  MissionRunner,
  RunnerOptions,
  RunnerSnapshot,
  RunRecord,
  TruthTarget,
} from '../../mission/autonomy/contracts';
import { buildReport } from '../../mission/autonomy/report';
import { policyFromFile, type NormalizedPolicy } from '../../mission/autonomy/policy';
import { SHIPPED_CHECKPOINT_URL } from '../../rl/checkpoint';
import { createMissionRunner } from '../../mission/autonomy/runner';
import {
  DEFAULT_SCENARIO,
  isScenarioId,
  loadScenario,
  SCENARIOS,
  type LoadedScenario,
  type ScenarioId,
} from '../../mission/autonomy/scenarios';
import { MISSION_TYPE_LABELS } from '../../mission/types';
import type { OutdoorView } from '../../world/outdoor/viz/scene';
import { sampleHeight } from '../../world/outdoor/terrain';
import {
  classLabel,
  formatClock,
  formatPct,
  markdownFor,
  OUTCOME_LABELS,
  PHASE_LABELS,
  PHASE_STEPS,
  rowsFor,
  stepIndex,
  type ReportRow,
} from './format';
import { DemoMap, isNight } from './map';
import { CameraInset, loadVisionKit } from './camera';
import { createVisionSource } from '../../vision/source';
import { siteNavLinks } from '../siteNav';

export type Speed = 1 | 4 | 16 | 'max';
export type Detector = 'analytic' | 'camera';
const DETECTORS: Detector[] = ['analytic', 'camera'];

const SPEEDS: Speed[] = [1, 4, 16, 'max'];
/** Wall-clock budget for sim ticks per frame at Max, ms. Rendering adds a few ms. */
const MAX_FRAME_MS = 24;

export interface DemoState {
  scenario: ScenarioId;
  outcome: string;
  report: AutonomyReport;
  rows: ReportRow[];
  run: Pick<RunRecord, 'outcome' | 'tSec' | 'entries' | 'declaredClear' | 'confirmed' | 'landedHome' | 'batteryStartPct' | 'batteryEndPct'>;
}

declare global {
  interface Window {
    __demo?: DemoState;
  }
}

export function startDemo(root: HTMLElement): void {
  const params = new URLSearchParams(location.search);
  const requested = params.get('scenario');
  const scenario: ScenarioId = isScenarioId(requested) ? requested : DEFAULT_SCENARIO;
  const speed = parseSpeed(params.get('speed'));
  const autoOperator = params.get('operator') === 'approve';
  // Indoors the shipped policy flies first and hands over to frontier exploration on a stall or contact.
  const indoor = params.get('indoor') === 'frontier' ? 'frontier' : 'policy';
  // `camera`: contacts come from the rendered camera and the learned detector (src/vision).
  const detector = params.get('detector') === 'camera' ? 'camera' : 'analytic';
  const demo = new Demo(root, scenario, speed, autoOperator, indoor, detector);
  void demo.load().then(() => {
    if (params.get('autorun') === '1') demo.play();
  });
}

let shipped: Promise<NormalizedPolicy | null> | null = null;

/** The shipped checkpoint, or null when it cannot be fetched. */
function shippedPolicy(): Promise<NormalizedPolicy | null> {
  shipped ??= fetch(SHIPPED_CHECKPOINT_URL)
    .then(async (response) => (response.ok ? policyFromFile(await response.arrayBuffer()) : null))
    .catch(() => null);
  return shipped;
}

function parseSpeed(value: string | null): Speed {
  if (value === 'max') return 'max';
  const n = Number(value);
  return n === 4 || n === 16 ? n : 1;
}

class Demo {
  private loaded: LoadedScenario | null = null;
  private runner: MissionRunner | null = null;
  private map: DemoMap | null = null;
  private snapshot: RunnerSnapshot | null = null;
  private running = false;
  private awaiting: EntryCandidate | null = null;
  private reported = false;
  private truth: TruthTarget[] | null = null;
  private readonly decided = new Map<string, number>();
  private logCount = 0;
  private acc = 0;
  private last = performance.now();
  private token = 0;
  private view: OutdoorView | null = null;
  private viewLoading = false;
  private camera: CameraInset | null = null;

  constructor(
    private readonly root: HTMLElement,
    private scenario: ScenarioId,
    private speed: Speed,
    private readonly autoOperator: boolean,
    private readonly indoor: 'policy' | 'frontier',
    private detector: Detector,
  ) {
    root.classList.add('demo');
    root.innerHTML = markup();
    this.bind();
    this.syncScenarioCards();
    this.syncSpeed();
    this.syncDetector();
    requestAnimationFrame(this.frame);
  }

  async load(): Promise<void> {
    const token = ++this.token;
    this.root.dataset.ready = '0';
    this.root.dataset.done = '0';
    delete this.root.dataset.outcome;
    delete this.root.dataset.error;
    window.__demo = undefined;
    this.running = false;
    this.awaiting = null;
    this.reported = false;
    this.truth = null;
    this.decided.clear();
    this.logCount = 0;
    this.acc = 0;
    this.snapshot = null;
    this.runner = null;
    this.el('event-log').replaceChildren();
    this.el('demo-report').hidden = true;
    this.el('operator-prompt').hidden = true;
    this.syncButtons();
    try {
      const loaded = loadScenario(this.scenario);
      this.loaded = loaded;
      const policy = this.indoor === 'policy' ? await shippedPolicy() : null;
      if (token !== this.token) return;
      this.camera = null;
      this.el('camera-inset').hidden = this.detector !== 'camera';
      if (this.detector === 'camera') {
        this.text('camera-caption', 'Loading the camera detector…');
        const kit = await loadVisionKit();
        if (token !== this.token) return;
        const inset = new CameraInset(this.el('camera-canvas') as HTMLCanvasElement);
        this.camera = inset;
        loaded.source = createVisionSource(loaded.world, kit.poses, kit.detector, { onFrame: (frame, found) => inset.push(frame, found) });
        this.text('camera-caption', 'Drone camera · 160×120 · learned detector');
      }
      const options: RunnerOptions = {
        ...loaded.options,
        operator: this.operatorFor(loaded),
        indoor: policy ? 'auto' : 'frontier',
        policy: policy ?? undefined,
      };
      if (this.map) this.map.setWorld(loaded.world);
      else this.map = new DemoMap(this.el('demo-map') as HTMLCanvasElement, loaded.world);
      this.paintBrief(loaded);
      this.view?.setWorld(loaded.world.outdoor, { timeOfDay: loaded.world.timeOfDay });
      const runner = await createMissionRunner(loaded.mission, loaded.world, loaded.source, options);
      if (token !== this.token) return;
      this.runner = runner;
      this.snapshot = runner.snapshot();
      this.render();
      this.root.dataset.ready = '1';
      this.syncButtons();
    } catch (error) {
      if (token !== this.token) return;
      const message = error instanceof Error ? error.message : String(error);
      this.root.dataset.error = message;
      this.appendLog({ tSec: 0, tone: 'warn', message: `Could not start: ${message}` });
      console.error(error);
    }
  }

  play(): void {
    if (!this.runner || this.runner.done) return;
    this.running = true;
    this.last = performance.now();
    this.syncButtons();
  }

  pause(): void {
    this.running = false;
    this.syncButtons();
  }

  private operatorFor(loaded: LoadedScenario): RunnerOptions['operator'] {
    if (this.autoOperator && loaded.options.operator) {
      const answer = loaded.options.operator;
      return (candidate) => {
        const reply = answer(candidate);
        this.appendLog({ tSec: this.snapshot?.tSec ?? 0, tone: 'action', message: `Operator auto-${reply === 'approve' ? 'approved' : reply === 'deny' ? 'denied' : 'held'} entry.` });
        return reply;
      };
    }
    // A human answers on the page. The sim clock holds while the prompt is open.
    return (candidate) => {
      this.awaiting = candidate;
      return 'pending';
    };
  }

  private answer(approve: boolean): void {
    if (!this.runner || !this.awaiting) return;
    const candidate = this.awaiting;
    this.decided.set(`${candidate.buildingId}:${candidate.openingId}`, candidate.safety);
    this.awaiting = null;
    if (approve) this.runner.approve();
    else this.runner.deny();
    this.snapshot = this.runner.snapshot();
    this.el('operator-prompt').hidden = true;
    this.render();
  }

  private frame = (now: number): void => {
    const realDt = Math.min(0.1, Math.max(0, (now - this.last) / 1000));
    this.last = now;
    const runner = this.runner;
    if (runner && this.running && !runner.done && !this.awaiting) {
      this.advance(runner, realDt);
      this.snapshot = runner.snapshot();
      if (this.snapshot.pendingEntry && !this.awaiting && !this.autoOperator) this.awaiting = this.snapshot.pendingEntry;
    }
    if (runner) this.render();
    if (runner?.done && !this.reported) this.finish(runner);
    requestAnimationFrame(this.frame);
  };

  private advance(runner: MissionRunner, realDt: number): void {
    const dt = this.loaded?.options.dt ?? 0.05;
    const halt = () => runner.done || this.awaiting !== null;
    if (this.speed === 'max') {
      const start = performance.now();
      while (!halt() && performance.now() - start < MAX_FRAME_MS) {
        for (let i = 0; i < 8 && !halt(); i++) runner.step(dt);
      }
      return;
    }
    this.acc += realDt * this.speed;
    let ticks = Math.floor(this.acc / dt);
    this.acc -= ticks * dt;
    const start = performance.now();
    while (ticks > 0 && !halt() && performance.now() - start < MAX_FRAME_MS) {
      runner.step(dt);
      ticks -= 1;
    }
    if (ticks > 0) this.acc = 0;
  }

  private finish(runner: MissionRunner): void {
    this.reported = true;
    this.running = false;
    const loaded = this.loaded!;
    const record = runner.record();
    const report = buildReport(record, loaded.source, loaded.world);
    const rows = rowsFor(report);
    for (const entry of record.entries) this.decided.set(`${entry.buildingId}:${entry.openingId}`, entry.safety);
    this.truth = loaded.source.truthTargets();
    this.paintReport(report, rows, record);
    this.root.dataset.done = '1';
    this.root.dataset.outcome = report.outcome;
    window.__demo = {
      scenario: this.scenario,
      outcome: report.outcome,
      report,
      rows,
      run: {
        outcome: record.outcome,
        tSec: record.tSec,
        entries: record.entries,
        declaredClear: record.declaredClear,
        confirmed: record.confirmed,
        landedHome: record.landedHome,
        batteryStartPct: record.batteryStartPct,
        batteryEndPct: record.batteryEndPct,
      },
    };
    this.render();
    this.syncButtons();
  }

  // ------------------------------------------------------------ render

  private render(): void {
    const snap = this.snapshot;
    const loaded = this.loaded;
    if (!snap || !loaded) return;
    this.map?.draw({ snapshot: snap, truth: this.truth, decided: this.decided, now: performance.now() });
    this.camera?.draw();
    if (this.camera) this.root.dataset.cameraFrames = String(this.camera.frames);
    this.root.dataset.phase = snap.phase;
    this.paintHud(snap, loaded);
    this.paintStepper(snap);
    this.paintLog(snap.log);
    this.paintPrompt();
    if (this.view) {
      const spin = 0.4 + Math.hypot(snap.pose.vel[0], snap.pose.vel[1], snap.pose.vel[2]);
      this.view.update(snap.pose.pos, snap.pose.yaw, spin);
    }
  }

  private paintHud(snap: RunnerSnapshot, loaded: LoadedScenario): void {
    const [x, y, z] = snap.pose.pos;
    const building = snap.inside ? loaded.world.buildings.find((b) => b.id === snap.inside) : undefined;
    const ground = building ? building.shell.origin[2] : sampleHeight(loaded.world.outdoor.terrain, x, y);
    this.text('hud-time', formatClock(snap.tSec));
    this.text('hud-battery', `${snap.batteryPct.toFixed(0)}%`);
    (this.el('hud-battery-bar') as HTMLElement).style.width = `${Math.max(0, Math.min(100, snap.batteryPct))}%`;
    this.el('hud-battery-bar').dataset.low = snap.batteryPct < 30 ? '1' : '0';
    this.text('hud-alt', `${Math.max(0, z - ground).toFixed(1)} m`);
    this.text('hud-speed', `${Math.hypot(snap.pose.vel[0], snap.pose.vel[1], snap.pose.vel[2]).toFixed(1)} m/s`);
    this.text('hud-phase', PHASE_LABELS[snap.phase]);
    this.text('hud-inside', building ? `Inside ${building.label}` : 'Outside');
    this.el('hud-inside').dataset.inside = building ? '1' : '0';
    this.text('hud-finds', String(snap.confirmed.length));
  }

  private paintStepper(snap: RunnerSnapshot): void {
    const current = stepIndex(snap.phase);
    const items = this.el('phase-stepper').querySelectorAll<HTMLElement>('li');
    items.forEach((item, index) => {
      const state = snap.done ? 'done' : index < current ? 'done' : index === current ? 'active' : 'todo';
      if (item.dataset.state !== state) item.dataset.state = state;
      if (state === 'active') item.setAttribute('aria-current', 'step');
      else item.removeAttribute('aria-current');
    });
  }

  private paintLog(log: LogEntry[]): void {
    if (log.length < this.logCount) this.logCount = 0;
    for (let i = this.logCount; i < log.length; i++) this.appendLog(log[i]!);
    this.logCount = log.length;
  }

  private appendLog(entry: LogEntry): void {
    const list = this.el('event-log');
    const item = document.createElement('li');
    item.dataset.tone = entry.tone;
    const time = document.createElement('time');
    time.textContent = formatClock(entry.tSec);
    const text = document.createElement('span');
    text.textContent = entry.message;
    item.append(time, text);
    list.prepend(item);
    while (list.childElementCount > 120) list.lastElementChild?.remove();
  }

  private paintPrompt(): void {
    const prompt = this.el('operator-prompt');
    const candidate = this.awaiting;
    if (!candidate) {
      if (!prompt.hidden) prompt.hidden = true;
      return;
    }
    if (!prompt.hidden && prompt.dataset.key === `${candidate.buildingId}:${candidate.openingId}`) return;
    const building = this.loaded?.world.buildings.find((b) => b.id === candidate.buildingId);
    prompt.dataset.key = `${candidate.buildingId}:${candidate.openingId}`;
    this.text('prompt-title', `Enter ${building?.label ?? candidate.buildingId}?`);
    this.text(
      'prompt-body',
      `${classLabel(candidate.kind)}, ${candidate.clearWidth.toFixed(2)} × ${candidate.clearHeight.toFixed(2)} m clear, ${candidate.clearanceM.toFixed(2)} m margin over the airframe. Safety ${formatPct(candidate.safety)}.`,
    );
    prompt.hidden = false;
    (this.el('operator-approve') as HTMLButtonElement).focus({ preventScroll: true });
  }

  private paintBrief(loaded: LoadedScenario): void {
    const { scenario, mission, world } = loaded;
    this.text('brief-title', scenario.title);
    this.text('brief-blurb', scenario.blurb);
    const facts: [string, string][] = [
      ['Mission', MISSION_TYPE_LABELS[mission.missionType]],
      ['Seed', String(world.seed)],
      ['Time of day', `${String(Math.floor(world.timeOfDay)).padStart(2, '0')}:${String(Math.round((world.timeOfDay % 1) * 60)).padStart(2, '0')}${isNight(world) ? ' · night' : ''}`],
      ['Lighting', world.flashlightOn ? 'Flashlight on' : 'Daylight'],
      ['Entry policy', mission.entryPolicy],
      ['Buildings', String(world.buildings.length)],
    ];
    const list = this.el('brief-facts');
    list.replaceChildren(
      ...facts.flatMap(([label, value]) => {
        const dt = document.createElement('dt');
        dt.textContent = label;
        const dd = document.createElement('dd');
        dd.textContent = value;
        return [dt, dd];
      }),
    );
  }

  private paintReport(report: AutonomyReport, rows: ReportRow[], record: RunRecord): void {
    const panel = this.el('demo-report');
    this.text('report-outcome', OUTCOME_LABELS[report.outcome]);
    panel.dataset.outcome = report.outcome;
    const coverage = report.indoorCoverage > 0 ? `${formatPct(report.outdoorCoverage)} out · ${formatPct(report.indoorCoverage)} in` : formatPct(report.outdoorCoverage);
    const kpis: [string, string, string][] = [
      ['time-to-find', 'Time to find', report.timeToFindSec === null ? '—' : formatClock(report.timeToFindSec)],
      ['coverage', 'Area coverage', coverage],
      ['false-clears', 'False clears', String(report.falseClears)],
      ['false-alarms', 'False alarms', String(report.falseAlarms)],
      ['collisions', 'Collisions', String(report.collisions)],
      ['battery', 'Battery used', `${report.batteryUsedPct.toFixed(1)}%`],
    ];
    this.el('report-kpis').replaceChildren(
      ...kpis.map(([id, label, value]) => {
        const tile = document.createElement('div');
        tile.className = 'kpi';
        tile.dataset.testid = `kpi-${id}`;
        tile.setAttribute('data-testid', `kpi-${id}`);
        const v = document.createElement('strong');
        v.textContent = value;
        const l = document.createElement('span');
        l.textContent = label;
        tile.append(v, l);
        return tile;
      }),
    );
    const table = this.el('report-rows');
    table.replaceChildren(
      ...rows.map((row) => {
        const wrap = document.createElement('div');
        const dt = document.createElement('dt');
        dt.textContent = row.label;
        const dd = document.createElement('dd');
        dd.textContent = row.value;
        wrap.append(dt, dd);
        return wrap;
      }),
    );
    const entries = record.entries.map((entry) => `${entry.buildingId} via opening ${entry.openingId}: ${entry.decision}`);
    const notes = [...report.notes, ...entries, record.landedHome ? 'Landed on the home pad.' : 'Did not land on the home pad.'];
    this.el('report-notes').replaceChildren(
      ...notes.map((note) => {
        const li = document.createElement('li');
        li.textContent = note;
        return li;
      }),
    );
    panel.hidden = false;
  }

  // ------------------------------------------------------------ controls

  private bind(): void {
    for (const scenario of SCENARIOS) {
      this.el(`scenario-${scenario.id}`).addEventListener('click', () => {
        if (this.scenario === scenario.id && this.runner && !this.snapshot?.tSec) return;
        this.scenario = scenario.id;
        this.syncScenarioCards();
        this.syncUrl();
        void this.load();
      });
    }
    this.el('run').addEventListener('click', () => this.play());
    this.el('pause').addEventListener('click', () => this.pause());
    this.el('reset').addEventListener('click', () => void this.load());
    for (const speed of SPEEDS) {
      this.el(`speed-${speed}`).addEventListener('click', () => {
        this.speed = speed;
        this.acc = 0;
        this.syncSpeed();
        this.syncUrl();
      });
    }
    for (const detector of DETECTORS) {
      this.el(`detector-${detector}`).addEventListener('click', () => {
        if (this.detector === detector) return;
        this.detector = detector;
        this.syncDetector();
        this.syncUrl();
        void this.load();
      });
    }
    this.el('operator-approve').addEventListener('click', () => this.answer(true));
    this.el('operator-deny').addEventListener('click', () => this.answer(false));
    this.el('toggle-3d').addEventListener('click', () => void this.toggle3d());
    this.el('copy-report').addEventListener('click', () => {
      const report = window.__demo?.report;
      if (!report) return;
      void navigator.clipboard?.writeText(markdownFor(report)).then(
        () => this.text('copy-report', 'Copied'),
        () => this.text('copy-report', 'Copy failed'),
      );
    });
    window.addEventListener('keydown', (event) => {
      if (event.target instanceof HTMLInputElement) return;
      if (event.code === 'Space') {
        event.preventDefault();
        if (this.running) this.pause();
        else this.play();
      }
    });
    window.addEventListener('resize', () => {
      this.view?.resize();
      this.render();
    });
  }

  private async toggle3d(): Promise<void> {
    const pip = this.el('pip');
    const button = this.el('toggle-3d');
    if (this.view) {
      this.view.dispose();
      this.view = null;
      pip.hidden = true;
      button.setAttribute('aria-pressed', 'false');
      return;
    }
    if (this.viewLoading || !this.loaded) return;
    this.viewLoading = true;
    try {
      const { OutdoorView } = await import('../../world/outdoor/viz/scene');
      pip.hidden = false;
      const canvas = this.el('pip-canvas') as HTMLCanvasElement;
      const view = new OutdoorView(canvas);
      view.setWorld(this.loaded.world.outdoor, { timeOfDay: this.loaded.world.timeOfDay });
      view.setCamera('chase');
      view.resize();
      this.view = view;
      button.setAttribute('aria-pressed', 'true');
      this.render();
    } catch (error) {
      pip.hidden = true;
      this.appendLog({ tSec: this.snapshot?.tSec ?? 0, tone: 'warn', message: `3D view unavailable: ${error instanceof Error ? error.message : String(error)}` });
    } finally {
      this.viewLoading = false;
    }
  }

  private syncButtons(): void {
    const ready = this.runner !== null;
    const done = this.runner?.done ?? false;
    (this.el('run') as HTMLButtonElement).disabled = !ready || done || this.running;
    (this.el('pause') as HTMLButtonElement).disabled = !ready || done || !this.running;
    this.text('run', this.snapshot && this.snapshot.tSec > 0 && !done ? 'Resume' : 'Run');
    this.root.dataset.running = this.running ? '1' : '0';
  }

  private syncScenarioCards(): void {
    for (const scenario of SCENARIOS) this.el(`scenario-${scenario.id}`).setAttribute('aria-pressed', String(scenario.id === this.scenario));
    this.root.dataset.scenario = this.scenario;
  }

  private syncSpeed(): void {
    for (const speed of SPEEDS) this.el(`speed-${speed}`).setAttribute('aria-pressed', String(speed === this.speed));
    this.root.dataset.speed = String(this.speed);
  }

  private syncDetector(): void {
    for (const detector of DETECTORS) this.el(`detector-${detector}`).setAttribute('aria-pressed', String(detector === this.detector));
    this.root.dataset.detector = this.detector;
  }

  private syncUrl(): void {
    const params = new URLSearchParams(location.search);
    params.set('scenario', this.scenario);
    params.set('speed', String(this.speed));
    if (this.detector === 'camera') params.set('detector', 'camera');
    else params.delete('detector');
    params.delete('autorun');
    history.replaceState(null, '', `${location.pathname}?${params.toString()}`);
  }

  private el(testid: string): HTMLElement {
    const node = this.root.querySelector<HTMLElement>(`[data-testid="${testid}"]`);
    if (!node) throw new Error(`missing ${testid}`);
    return node;
  }

  private text(testid: string, value: string): void {
    const node = this.el(testid);
    if (node.textContent !== value) node.textContent = value;
  }
}

function markup(): string {
  const cards = SCENARIOS.map(
    (scenario, index) => `
      <button type="button" class="card" data-testid="scenario-${scenario.id}" aria-pressed="false">
        <span class="card-index">${String.fromCharCode(65 + index)}</span>
        <span class="card-body">
          <strong>${scenario.title}</strong>
          <span class="tags">${scenario.tags.map((tag) => `<span>${tag}</span>`).join('')}</span>
        </span>
      </button>`,
  ).join('');
  const steps = PHASE_STEPS.map((step, index) => `<li data-step="${step.key}" data-state="todo"><span class="dot">${index + 1}</span><span class="name">${step.label}</span></li>`).join('');
  const speeds = SPEEDS.map((speed) => `<button type="button" data-testid="speed-${speed}" aria-pressed="false">${speed === 'max' ? 'Max' : `${speed}×`}</button>`).join('');
  return `
    <header class="top">
      <div class="brand">
        <span class="mark" aria-hidden="true"></span>
        <div>
          <p class="eyebrow">frontier-fly</p>
          <h1>Autonomous search and rescue</h1>
        </div>
      </div>
      <nav class="nav" aria-label="Pages">${siteNavLinks('demo')}</nav>
    </header>
    <main class="layout">
      <section class="rail" aria-label="Scenario">
        <h2>Scenario</h2>
        <div class="cards">${cards}</div>
        <div class="brief">
          <h3 data-testid="brief-title"></h3>
          <p data-testid="brief-blurb"></p>
          <dl data-testid="brief-facts"></dl>
        </div>
        <h2>Run</h2>
        <div class="transport">
          <button type="button" class="primary" data-testid="run" disabled>Run</button>
          <button type="button" data-testid="pause" disabled>Pause</button>
          <button type="button" data-testid="reset">Reset</button>
        </div>
        <div class="speeds" role="group" aria-label="Sim speed">${speeds}</div>
        <h2>Detector</h2>
        <div class="speeds detectors" role="group" aria-label="Detector">
          <button type="button" data-testid="detector-analytic" aria-pressed="true" title="Calibrated recall model, no pixels">Analytic</button>
          <button type="button" data-testid="detector-camera" aria-pressed="false" title="Rendered 160×120 camera read by a learned CNN">Camera</button>
        </div>
        <p class="hint">Space toggles run and pause. Everything is simulated: the people, the objects, and the detector. Camera mode renders what the drone sees and runs a small learned detector on it.</p>
      </section>
      <section class="stage" aria-label="Live mission">
        <ol class="stepper" data-testid="phase-stepper">${steps}</ol>
        <div class="hud" data-testid="hud">
          <div><span>Time</span><strong data-testid="hud-time">0:00</strong></div>
          <div class="battery"><span>Battery</span><strong data-testid="hud-battery">100%</strong><i><b data-testid="hud-battery-bar"></b></i></div>
          <div><span>Altitude</span><strong data-testid="hud-alt">0.0 m</strong></div>
          <div><span>Speed</span><strong data-testid="hud-speed">0.0 m/s</strong></div>
          <div class="wide"><span>Phase</span><strong data-testid="hud-phase">Preflight</strong></div>
          <div><span>Where</span><strong data-testid="hud-inside">Outside</strong></div>
          <div><span>Finds</span><strong data-testid="hud-finds">0</strong></div>
        </div>
        <div class="map-wrap">
          <canvas data-testid="demo-map" aria-label="Top-down mission map"></canvas>
          <div class="cam" data-testid="camera-inset" hidden><canvas data-testid="camera-canvas" aria-label="Drone camera with detections"></canvas><span data-testid="camera-caption"></span></div>
          <div class="pip" data-testid="pip" hidden><canvas data-testid="pip-canvas"></canvas><span>3D chase · daylight render</span></div>
          <button type="button" class="toggle-3d" data-testid="toggle-3d" aria-pressed="false">3D chase</button>
          <ul class="legend" aria-label="Map legend">
            <li><i class="lg-fence"></i>Geofence</li>
            <li><i class="lg-nofly"></i>No-fly</li>
            <li><i class="lg-safe"></i>Safe entry</li>
            <li><i class="lg-mid"></i>Marginal</li>
            <li><i class="lg-decoy"></i>Too small</li>
            <li><i class="lg-plan"></i>Plan</li>
            <li><i class="lg-track"></i>Flown</li>
          </ul>
        </div>
      </section>
      <section class="side" aria-label="Operator and report">
        <div class="prompt" data-testid="operator-prompt" role="alertdialog" aria-labelledby="prompt-title" hidden>
          <p class="eyebrow">Operator approval</p>
          <h3 id="prompt-title" data-testid="prompt-title"></h3>
          <p data-testid="prompt-body"></p>
          <p class="hold">The drone holds outside. The sim clock is paused.</p>
          <div class="actions">
            <button type="button" class="primary" data-testid="operator-approve">Approve entry</button>
            <button type="button" data-testid="operator-deny">Deny</button>
          </div>
        </div>
        <div class="report" data-testid="demo-report" hidden>
          <p class="eyebrow">Mission report</p>
          <h3 data-testid="report-outcome"></h3>
          <div class="kpis" data-testid="report-kpis"></div>
          <dl class="rows" data-testid="report-rows"></dl>
          <ul class="notes" data-testid="report-notes"></ul>
          <button type="button" data-testid="copy-report">Copy as Markdown</button>
        </div>
        <h2>Event log</h2>
        <ol class="log" data-testid="event-log" aria-live="polite"></ol>
      </section>
    </main>
  `;
}
