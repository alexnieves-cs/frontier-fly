import * as THREE from 'three';
import type { Contact, Phase, RunnerSnapshot, V3 } from '../mission/autonomy/contracts';
import { DemoMap } from '../ui/demo/map';
import { classLabel, formatClock, OUTCOME_LABELS, PHASE_LABELS, PHASE_STEPS, rowsFor, stepIndex } from '../ui/demo/format';
import type { FilmFrame } from './director';
import { phaseSpans, sampleFlight, type FlightLog } from './flight';
import { phaseColor, toThree } from './scene';

const CARD_SEC = 3.6;

const PHASE_BLURB: Partial<Record<Phase, string>> = {
  preflight: 'Takeoff from the pad',
  coverage: 'Lawnmower sweep of the geofence with a downward camera',
  approach: 'Building detected, flying a clear corridor to it',
  inspect: 'Circling the shell to find and score openings',
  entry_select: 'Picking the safest opening for the airframe',
  await_operator: 'Entry needs an operator approval',
  fly_through: 'Threading the opening at walking pace',
  indoor_explore: 'Exploring rooms, mapping what the camera has cleared',
  confirm: 'Viewing the contact from several angles before confirming',
  exit: 'Leaving through the same opening',
  return_home: 'Flying home on the reserve',
  land: 'Precision landing on the pad',
};

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls: string, parent?: HTMLElement, text?: string): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  node.className = cls;
  if (text != null) node.textContent = text;
  parent?.appendChild(node);
  return node;
}

/** The HUD, captions, minimap and end card drawn over the 3D view. */
export class FilmOverlay {
  readonly root: HTMLElement;
  private readonly callouts: HTMLCanvasElement;
  private readonly clock: HTMLElement;
  private readonly rate: HTMLElement;
  private readonly alt: HTMLElement;
  private readonly speed: HTMLElement;
  private readonly battery: HTMLElement;
  private readonly batteryBar: HTMLElement;
  private readonly phaseName: HTMLElement;
  private readonly steps: HTMLElement[] = [];
  private readonly card: HTMLElement;
  private readonly cardIndex: HTMLElement;
  private readonly cardTitle: HTMLElement;
  private readonly cardText: HTMLElement;
  private readonly endCard: HTMLElement;
  private readonly intro: HTMLElement;
  private readonly map: DemoMap;
  private readonly mapCanvas: HTMLCanvasElement;
  private readonly spans: { phase: Phase; t0: number; t1: number }[];
  private readonly logRows: HTMLElement;
  private lastLogCount = -1;

  constructor(
    parent: HTMLElement,
    private readonly log: FlightLog,
  ) {
    const world = log.loaded.world;
    const scenario = log.loaded.scenario;
    this.spans = phaseSpans(log);
    this.root = el('div', 'film-overlay', parent);
    this.callouts = el('canvas', 'film-callouts', this.root);

    const top = el('div', 'film-top', this.root);
    const brand = el('div', 'film-brand', top);
    el('div', 'film-logo', brand);
    const brandText = el('div', '', brand);
    el('div', 'film-kicker', brandText, 'FRONTIER-FLY · AUTONOMOUS SEARCH AND RESCUE');
    el('div', 'film-mission', brandText, scenario.title);
    const stats = el('div', 'film-stats', top);
    const stat = (label: string) => {
      const box = el('div', 'film-stat', stats);
      el('span', 'film-stat-label', box, label);
      return el('span', 'film-stat-value', box, '');
    };
    this.clock = stat('MISSION');
    this.rate = stat('PLAYBACK');
    this.alt = stat('ALT AGL');
    this.speed = stat('SPEED');
    this.battery = stat('BATTERY');
    this.batteryBar = el('div', 'film-battery-bar', this.battery.parentElement!);
    el('div', 'film-badge', top, world.timeOfDay < 6.5 || world.timeOfDay > 19.5 ? 'NIGHT · FLASHLIGHT' : 'DAYLIGHT');

    this.card = el('div', 'film-card', this.root);
    this.cardIndex = el('div', 'film-card-index', this.card);
    this.cardTitle = el('div', 'film-card-title', this.card);
    this.cardText = el('div', 'film-card-text', this.card);

    const side = el('div', 'film-side', this.root);
    const mapBox = el('div', 'film-map', side);
    this.mapCanvas = el('canvas', 'film-map-canvas', mapBox);
    el('div', 'film-map-label', mapBox, 'MISSION MAP');
    this.map = new DemoMap(this.mapCanvas, world);
    this.logRows = el('div', 'film-log', side);

    const rail = el('div', 'film-rail', this.root);
    this.phaseName = el('div', 'film-phase', rail);
    const stepsRow = el('div', 'film-steps', rail);
    for (const step of PHASE_STEPS) {
      const s = el('div', 'film-step', stepsRow);
      el('span', 'film-step-dot', s);
      el('span', 'film-step-label', s, step.label);
      this.steps.push(s);
    }
    el('div', 'film-autonomy', rail, 'Fully autonomous · seeded simulation · no pilot input');

    this.intro = el('div', 'film-intro', this.root);
    el('div', 'film-intro-kicker', this.intro, `Scenario · seed ${world.seed} · ${world.buildings.length} building${world.buildings.length > 1 ? 's' : ''}`);
    el('div', 'film-intro-title', this.intro, scenario.title);
    el('div', 'film-intro-text', this.intro, scenario.blurb);

    this.endCard = el('div', 'film-end', this.root);
    const report = log.report;
    el('div', 'film-end-kicker', this.endCard, 'MISSION REPORT');
    el('div', `film-end-outcome outcome-${report.outcome}`, this.endCard, OUTCOME_LABELS[report.outcome]);
    const grid = el('div', 'film-end-grid', this.endCard);
    const metric = (value: string, label: string) => {
      const box = el('div', 'film-end-metric', grid);
      el('div', 'film-end-value', box, value);
      el('div', 'film-end-label', box, label);
    };
    metric(formatClock(report.durationSec), 'Mission time');
    metric(report.timeToFindSec == null ? '—' : formatClock(report.timeToFindSec), report.missionType === 'safety_sweep' ? 'Time to find (none present)' : 'Time to find');
    metric(`${Math.round(report.indoorCoverage * 100)}%`, 'Indoor coverage');
    metric(String(report.falseAlarms), 'False alarms');
    metric(String(report.collisions), 'Collisions');
    metric(report.landedHome ? 'Yes' : 'No', 'Landed home');
    const rows = rowsFor(report).filter((r) => /Entries|Battery|Outdoor|False clears|Missed/.test(r.label)).slice(0, 4);
    el('div', 'film-end-foot', this.endCard, rows.map((r) => `${r.label} ${r.value}`).join('  ·  '));
    el('div', 'film-end-sig', this.endCard, `Indoor explorer: ${log.explorer === 'policy' ? 'trained policy' : 'frontier planner'} · detector and people are simulated`);
  }

  resize(width: number, height: number): void {
    this.callouts.width = width;
    this.callouts.height = height;
  }

  update(f: FilmFrame, camera: THREE.PerspectiveCamera): void {
    const { frame, index, pos } = sampleFlight(this.log, f.t);
    const world = this.log.loaded.world;
    const ground = sampleGround(world, pos);
    this.clock.textContent = `T+${formatClock(f.t)}`;
    this.rate.textContent = f.part === 'flight' ? `×${f.rate < 1.25 ? f.rate.toFixed(1) : Math.round(f.rate)}` : '—';
    this.alt.textContent = `${Math.max(0, pos[2] - ground).toFixed(1)} m`;
    this.speed.textContent = `${Math.hypot(frame.vel[0], frame.vel[1], frame.vel[2]).toFixed(1)} m/s`;
    this.battery.textContent = `${Math.round(frame.battery)}%`;
    this.batteryBar.style.setProperty('--level', `${Math.max(0, Math.min(100, frame.battery))}%`);
    const phase = f.part === 'outro' ? 'done' : frame.phase;
    this.phaseName.textContent = (PHASE_LABELS[phase] ?? phase).toUpperCase();
    this.phaseName.style.color = phaseColor(phase);
    const current = stepIndex(phase);
    this.steps.forEach((s, i) => {
      s.classList.toggle('done', i < current);
      s.classList.toggle('active', i === current);
    });

    // Intro title and end card.
    const introOpacity = f.part === 'intro' ? Math.min(1, f.partSec / 0.6) * Math.min(1, (5 - f.partSec) / 0.8) : 0;
    this.intro.style.opacity = String(Math.max(0, introOpacity));
    const endOpacity = f.part === 'outro' ? Math.min(1, Math.max(0, (f.partSec - 0.6) / 0.8)) : 0;
    this.endCard.style.opacity = String(endOpacity);
    this.endCard.style.transform = `translate(-50%, -50%) scale(${0.96 + endOpacity * 0.04})`;
    this.root.classList.toggle('film-is-intro', f.part === 'intro');

    // Lower-third chapter card at the start of each phase run.
    const span = f.part === 'flight' ? this.spans.find((s) => f.t >= s.t0 && f.t <= s.t1) : undefined;
    if (span && span.phase !== 'done') {
      const since = (f.t - span.t0) / Math.max(0.5, f.rate);
      const visible = since < CARD_SEC;
      const fade = visible ? Math.min(1, since / 0.35) * Math.min(1, (CARD_SEC - since) / 0.5) : 0;
      this.card.style.opacity = String(fade);
      this.card.style.transform = `translateX(${(1 - Math.min(1, since / 0.35)) * -24}px)`;
      this.cardIndex.textContent = String(this.spans.indexOf(span) + 1).padStart(2, '0');
      this.cardIndex.style.background = phaseColor(span.phase);
      this.cardTitle.textContent = PHASE_LABELS[span.phase] ?? span.phase;
      this.cardText.textContent = this.subtitle(span) ?? PHASE_BLURB[span.phase] ?? '';
    } else {
      this.card.style.opacity = '0';
    }

    this.drawMap(index, f);
    this.drawLog(f.t);
    this.drawCallouts(f, camera);
  }

  private subtitle(span: { phase: Phase; t0: number }): string | null {
    const entry = this.log.log.find((e) => e.tSec >= span.t0 - 0.05 && e.tSec <= span.t0 + 2 && e.tone !== 'warn');
    if (!entry) return null;
    return entry.message.length > 110 ? `${entry.message.slice(0, 107)}…` : entry.message;
  }

  private drawLog(t: number): void {
    const visible = this.log.log.filter((e) => e.tSec <= t);
    if (visible.length === this.lastLogCount) return;
    this.lastLogCount = visible.length;
    this.logRows.replaceChildren();
    for (const entry of visible.slice(-5)) {
      const row = el('div', `film-log-row tone-${entry.tone}`, this.logRows);
      el('span', 'film-log-time', row, formatClock(entry.tSec));
      el('span', 'film-log-text', row, entry.message.length > 72 ? `${entry.message.slice(0, 69)}…` : entry.message);
    }
  }

  private drawMap(index: number, f: FilmFrame): void {
    const frames = this.log.frames;
    const frame = frames[index]!;
    const track: V3[] = [];
    for (let i = Math.max(0, index - 4000); i <= index; i += 3) track.push(frames[i]!.pos);
    const contacts: Contact[] = [];
    const snapshot: RunnerSnapshot = {
      tSec: f.t,
      phase: frame.phase,
      pose: { pos: frame.pos, yaw: frame.yaw, vel: frame.vel },
      batteryPct: frame.battery,
      inside: frame.inside,
      plan: this.log.plans[frame.plan] ?? [],
      track,
      contacts,
      confirmed: this.log.confirmed.filter((c) => c.tSec <= f.t),
      pendingEntry: null,
      log: [],
      done: f.part === 'outro',
    };
    this.map.draw({ snapshot, truth: null, decided: new Map(), now: f.i * 33 });
  }

  private drawCallouts(f: FilmFrame, camera: THREE.PerspectiveCamera): void {
    const g = this.callouts.getContext('2d')!;
    const w = this.callouts.width;
    const h = this.callouts.height;
    g.clearRect(0, 0, w, h);
    if (f.part === 'intro') return;
    const v = new THREE.Vector3();
    const project = (p: V3, lift = 0): [number, number] | null => {
      toThree(p, v);
      v.y += lift;
      v.project(camera);
      if (v.z > 1 || v.z < -1 || Math.abs(v.x) > 0.92 || Math.abs(v.y) > 0.92) return null;
      return [(v.x * 0.5 + 0.5) * w, (-v.y * 0.5 + 0.5) * h];
    };
    const dist = (p: V3) => camera.position.distanceTo(toThree(p, new THREE.Vector3()));
    const scale = h / 1080;
    // Recent raw detections: thin amber brackets.
    const recent = this.log.frames.slice(Math.max(0, this.indexAt(f.t) - 24), this.indexAt(f.t) + 1).flatMap((fr) => fr.hits);
    // One bracket per object: skip repeat hits and anything near an already
    // drawn or confirmed contact of the same class.
    const drawn: { classId: string; at: V3 }[] = this.log.confirmed.filter((c) => c.tSec <= f.t).map((c) => ({ classId: c.classId, at: c.estimate }));
    for (const hit of recent.reverse()) {
      if (drawn.some((d) => d.classId === hit.classId && Math.hypot(d.at[0] - hit.estimate[0], d.at[1] - hit.estimate[1], d.at[2] - hit.estimate[2]) < 1.2)) continue;
      drawn.push({ classId: hit.classId, at: hit.estimate });
      const c = project(hit.estimate, 0.5);
      if (!c) continue;
      const size = Math.max(22, Math.min(140, (900 / Math.max(1, dist(hit.estimate))) * scale));
      bracket(g, c[0], c[1], size, size * 1.1, 'rgba(255, 209, 102, 0.9)', 2 * scale);
      label(g, c[0] - size / 2, c[1] - size * 0.55 - 8 * scale, `${classLabel(hit.classId).toUpperCase()} ${hit.confidence.toFixed(2)}`, '#ffd166', scale);
    }
    // Confirmed contacts: solid green brackets that stay.
    for (const c of this.log.confirmed) {
      if (c.tSec > f.t) continue;
      const p = project(c.estimate, 0.45);
      if (!p) continue;
      const age = f.t - c.tSec;
      const size = Math.max(30, Math.min(180, (1100 / Math.max(1, dist(c.estimate))) * scale));
      const grow = 1 + 0.6 * Math.exp(-age * 2.2);
      bracket(g, p[0], p[1], size * grow, size * 1.15 * grow, '#3dff9a', 3 * scale);
      label(g, p[0] - (size * grow) / 2, p[1] - (size * 1.15 * grow) / 2 - 10 * scale, `CONFIRMED ${classLabel(c.classId).toUpperCase()} · ${c.confidence.toFixed(2)} · ${c.views} VIEWS`, '#3dff9a', scale);
    }
  }

  private indexAt(t: number): number {
    return sampleFlight(this.log, t).index;
  }
}

function sampleGround(world: FlightLog['loaded']['world'], pos: V3): number {
  // Indoors, altitude is above the building floor; outdoors above terrain.
  for (const b of world.buildings) {
    const spec = b.shell.spec;
    const f = spec.floors[0]!;
    const x = pos[0] - b.shell.origin[0];
    const y = pos[1] - b.shell.origin[1];
    if (x > 0 && y > 0 && x < f.w * spec.cellSize && y < f.d * spec.cellSize && pos[2] < b.shell.origin[2] + spec.floors.length * spec.floorHeight) {
      return b.shell.origin[2];
    }
  }
  const terrain = world.outdoor.terrain;
  const n = terrain.resolution;
  const u = Math.max(0, Math.min(n - 1, (pos[0] / terrain.size) * (n - 1)));
  const v = Math.max(0, Math.min(n - 1, (pos[1] / terrain.size) * (n - 1)));
  return terrain.heights[Math.round(v) * n + Math.round(u)]!;
}

function bracket(g: CanvasRenderingContext2D, cx: number, cy: number, w: number, h: number, color: string, lw: number): void {
  const x0 = cx - w / 2;
  const y0 = cy - h / 2;
  const x1 = cx + w / 2;
  const y1 = cy + h / 2;
  const k = Math.min(w, h) * 0.24;
  g.strokeStyle = color;
  g.lineWidth = lw;
  g.shadowColor = 'rgba(0,0,0,0.6)';
  g.shadowBlur = 4;
  g.beginPath();
  g.moveTo(x0, y0 + k);
  g.lineTo(x0, y0);
  g.lineTo(x0 + k, y0);
  g.moveTo(x1 - k, y0);
  g.lineTo(x1, y0);
  g.lineTo(x1, y0 + k);
  g.moveTo(x1, y1 - k);
  g.lineTo(x1, y1);
  g.lineTo(x1 - k, y1);
  g.moveTo(x0 + k, y1);
  g.lineTo(x0, y1);
  g.lineTo(x0, y1 - k);
  g.stroke();
  g.shadowBlur = 0;
}

function label(g: CanvasRenderingContext2D, x: number, y: number, text: string, color: string, scale: number): void {
  g.font = `600 ${Math.round(15 * scale)}px "JetBrains Mono", ui-monospace, SFMono-Regular, Menlo, monospace`;
  const pad = 6 * scale;
  const tw = g.measureText(text).width;
  g.fillStyle = 'rgba(6, 12, 18, 0.78)';
  g.fillRect(x, y - 18 * scale, tw + pad * 2, 22 * scale);
  g.fillStyle = color;
  g.fillRect(x, y - 18 * scale, 3 * scale, 22 * scale);
  g.fillText(text, x + pad, y - 2 * scale);
}
