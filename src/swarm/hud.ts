import * as THREE from 'three';
import type { DroneSample, StudyRow, SwarmBundle, Track } from './film';
import { DRONE_COLORS } from './scene';

const MODE_LABEL = { waiting: 'Waiting', searching: 'Searching', returning: 'Heading back', done: 'Finished' } as const;
const NUM = ['zero', 'one', 'two', 'three', 'four', 'five', 'six'];

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls: string, parent?: HTMLElement, text?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  e.className = cls;
  if (text !== undefined) e.textContent = text;
  parent?.appendChild(e);
  return e;
}

function clock(t: number): string {
  const m = Math.floor(t / 60);
  const s = t - m * 60;
  return `${String(m).padStart(2, '0')}:${s.toFixed(1).padStart(4, '0')}`;
}

const pct = (x: number | null, digits = 1) => (x === null ? 'n/a' : `${(x * 100).toFixed(digits)}%`);
const cap = (s: string) => s.charAt(0).toUpperCase() + s.slice(1);

interface DroneCard {
  root: HTMLElement;
  mode: HTMLElement;
  share: HTMLElement;
  bar: HTMLElement;
  ekf: HTMLElement;
  tilt: HTMLElement;
  speed: HTMLElement;
  label: HTMLElement;
}

/**
 * Everything drawn over the 3D view: the title, the clock, one card per drone, the coverage curve
 * against one PX4 drone on the same building, what the base station knows, an event ticker, labels
 * that follow the drones, and the results at the end.
 */
export class SwarmHud {
  readonly root: HTMLElement;
  private readonly title: HTMLElement;
  private readonly intro: HTMLElement;
  private readonly clockValue: HTMLElement;
  private readonly speedPill: HTMLElement;
  private readonly cards: DroneCard[] = [];
  private readonly covValue: HTMLElement;
  private readonly covSolo: HTMLElement;
  private covSoloValue: HTMLElement | null = null;
  private readonly chart: HTMLCanvasElement;
  private readonly baseValue: HTMLElement;
  private readonly baseBar: HTMLElement;
  private readonly ticker: HTMLElement;
  private readonly baseLabel: HTMLElement;
  private readonly results: HTMLElement;
  private readonly v = new THREE.Vector3();

  constructor(
    stage: HTMLElement,
    private readonly b: SwarmBundle,
    private readonly track: Track,
  ) {
    const n = b.drones;
    this.root = el('div', 'sw-hud', stage);

    this.title = el('div', 'sw-title', this.root);
    el('div', 'sw-kicker', this.title, 'frontier-fly · drone team');
    el('div', 'sw-h1', this.title, `${cap(NUM[n] ?? String(n))} drones, ${NUM[n] ?? n} autopilots`);
    el('div', 'sw-sub', this.title, `Each drone flown by its own PX4. A ${b.size} × ${b.size} m building it never trained on, no GNSS indoors.`);

    this.intro = el('div', 'sw-intro', this.root);
    el('div', 'sw-intro-kicker', this.intro, 'Simulation · PX4 v1.15.4 software in the loop');
    el('div', 'sw-intro-h1', this.intro, `${cap(NUM[n] ?? String(n))} drones search one building`);
    el(
      'div',
      'sw-intro-sub',
      this.intro,
      `They share their maps over a radio that walls weaken, split the unexplored edges between them, and each flies on its own PX4 autopilot and EKF2 estimator.`,
    );

    const clockBox = el('div', 'sw-clock', this.root);
    el('div', 'sw-clock-label', clockBox, 'Sim time');
    this.clockValue = el('div', 'sw-clock-value', clockBox);
    this.speedPill = el('div', 'sw-speed', clockBox);

    const col = el('div', 'sw-drones', this.root);
    for (let d = 0; d < n; d++) {
      const root = el('div', 'sw-card', col);
      root.style.setProperty('--c', DRONE_COLORS[d % DRONE_COLORS.length]!);
      const head = el('div', 'sw-card-head', root);
      el('span', 'sw-dot', head);
      el('span', 'sw-card-name', head, `Drone ${d + 1}`);
      const mode = el('span', 'sw-card-mode', head);
      const row = el('div', 'sw-card-row', root);
      const share = el('div', 'sw-card-share', row);
      const track = el('div', 'sw-card-track', row);
      const bar = el('div', 'sw-card-bar', track);
      const stats = el('div', 'sw-card-stats', root);
      const stat = (label: string) => {
        const s = el('div', 'sw-stat', stats);
        const v = el('div', 'sw-stat-v', s);
        el('div', 'sw-stat-l', s, label);
        return v;
      };
      const ekf = stat('EKF error');
      const tilt = stat('tilt');
      const speed = stat('speed');
      const label = el('div', 'sw-label', this.root, `D${d + 1}`);
      label.style.setProperty('--c', DRONE_COLORS[d % DRONE_COLORS.length]!);
      this.cards.push({ root, mode, share, bar, ekf, tilt, speed, label });
    }
    this.baseLabel = el('div', 'sw-label sw-label-base', this.root, 'Base');

    const cov = el('div', 'sw-cov', this.root);
    const covHead = el('div', 'sw-cov-head', cov);
    const covLeft = el('div', '', covHead);
    el('div', 'sw-cov-label', covLeft, 'Floor flown');
    this.covValue = el('div', 'sw-cov-value', covLeft);
    const covRight = el('div', 'sw-cov-right', covHead);
    this.covSolo = el('div', 'sw-cov-solo', covRight);
    if (b.solo) {
      el('span', '', this.covSolo, '1 drone at this time');
      this.covSoloValue = el('b', '', this.covSolo);
    }
    this.chart = el('canvas', 'sw-chart', cov);
    this.chart.width = 560 * 2;
    this.chart.height = 170 * 2;
    const legend = el('div', 'sw-cov-legend', cov);
    el('span', 'sw-leg sw-leg-team', legend, `${n} drones`);
    if (b.solo) el('span', 'sw-leg sw-leg-solo', legend, '1 drone, same building');
    el('span', 'sw-leg sw-leg-base', legend, 'Map at the base');

    const baseBox = el('div', 'sw-base', this.root);
    el('div', 'sw-base-label', baseBox, 'Base station has');
    this.baseValue = el('div', 'sw-base-value', baseBox);
    const bt = el('div', 'sw-base-track', baseBox);
    this.baseBar = el('div', 'sw-base-bar', bt);
    el('div', 'sw-base-note', baseBox, 'of the map, relayed over the mesh. Links: brighter where a packet is more likely to get through.');

    this.ticker = el('div', 'sw-ticker', this.root);

    el('div', 'sw-credit', this.root, `PX4 v1.15.4 SITL × ${n} in lockstep at 250 Hz · EKF2 on external vision, no GNSS · simulated building, radio and sensors`);

    this.results = el('div', 'sw-results', this.root);
    this.buildResults();
  }

  private buildResults(): void {
    const b = this.b;
    const r = b.result;
    const n = b.drones;
    const card = this.results;
    el('div', 'sw-res-kicker', card, `Building ${b.seed} · held-out test set`);
    const doneAt = r.steps * 0.05;
    el('div', 'sw-res-h1', card, `${pct(r.coverage)} of the floor in ${doneAt.toFixed(0)} s`);
    const grid = el('div', 'sw-res-grid', card);
    const cell = (v: string, l: string, cls = '') => {
      const c = el('div', `sw-res-cell ${cls}`, grid);
      el('div', 'sw-res-v', c, v);
      el('div', 'sw-res-l', c, l);
    };
    cell(r.t80 === null ? 'never' : `${r.t80.toFixed(1)} s`, `${n} drones to 80%`, 'is-team');
    const solo = b.solo?.result;
    if (solo) cell(solo.t80 === null ? 'never' : `${solo.t80.toFixed(1)} s`, solo.t80 === null ? `1 drone to 80% (${pct(solo.coverage)} in the full ${b.budgetS.toFixed(0)} s)` : '1 drone to 80%', 'is-solo');
    // Close passes: drones whose centres came within 0.35 m (the study counts the 50 ms steps).
    const passes = this.track.events.filter((e) => e.warn).length;
    cell(
      `${passes} · ${r.collisions} · ${r.crashes}`,
      passes ? `close pass${passes > 1 ? 'es' : ''} (under 0.35 m for ${(r.droneContacts * 0.05).toFixed(2)} s) · wall hits · crashes` : 'close passes · wall hits · crashes',
      passes ? 'is-warn' : '',
    );
    const ekf = (r.px4 ?? []).map((p) => p.ekfErrRmsM);
    if (ekf.length) cell(`${Math.min(...ekf).toFixed(2)} to ${Math.max(...ekf).toFixed(2)} m`, 'EKF2 position error, RMS');

    const rows = b.study.rows.filter((x) => x.buildings > 0);
    if (b.study.buildings > 0 && rows.length) {
      el('div', 'sw-res-sub', card, `Across ${b.study.buildings} held-out ${b.size} m buildings, mean coverage in the ${b.budgetS.toFixed(0)} s budget`);
      const table = el('div', 'sw-res-table', card);
      const sizes = [...new Set(rows.map((x) => x.drones))].sort();
      const head = el('div', 'sw-res-tr is-head', table);
      el('div', 'sw-res-td', head, '');
      for (const s of sizes) el('div', 'sw-res-td', head, `${s} drone${s > 1 ? 's' : ''}`);
      const ARM_LABEL: Record<StudyRow['arm'], string> = { px4: 'PX4 in the loop', quad: 'Quad model', kinematic: 'Kinematic' };
      for (const arm of ['px4', 'quad', 'kinematic'] as const) {
        const tr = el('div', `sw-res-tr is-${arm}`, table);
        el('div', 'sw-res-td', tr, ARM_LABEL[arm]);
        for (const s of sizes) {
          const row = rows.find((x) => x.arm === arm && x.drones === s);
          const td = el('div', 'sw-res-td', tr, row ? pct(row.coverage, 0) : 'n/a');
          // The episode stops at 95%, so with enough drones coverage saturates; the time to 80% keeps the difference.
          if (row && row.reached80) el('span', 'sw-res-t80', td, row.t80Mean === null ? '' : `80% at ${row.t80Mean.toFixed(0)} s${row.reached80 < 1 ? ` (${Math.round(row.reached80 * row.buildings)}/${row.buildings})` : ''}`);
        }
      }
    }
  }

  private drawChart(t: number): void {
    const c = this.chart;
    const g = c.getContext('2d')!;
    const W = c.width;
    const H = c.height;
    const padL = 64;
    const padB = 40;
    const padT = 14;
    const padR = 16;
    const T = Math.max(this.b.budgetS, this.track.end);
    const X = (s: number) => padL + (s / T) * (W - padL - padR);
    const Y = (v: number) => H - padB - v * (H - padB - padT);
    g.clearRect(0, 0, W, H);
    g.font = '500 22px Inter, "Segoe UI", Helvetica, Arial, sans-serif';
    g.fillStyle = 'rgba(200,215,225,0.65)';
    g.strokeStyle = 'rgba(255,255,255,0.08)';
    g.lineWidth = 2;
    for (const v of [0, 0.5, 1]) {
      g.beginPath();
      g.moveTo(padL, Y(v));
      g.lineTo(W - padR, Y(v));
      g.stroke();
      g.textAlign = 'right';
      g.fillText(`${v * 100}%`, padL - 10, Y(v) + 8);
    }
    g.textAlign = 'center';
    for (let s = 0; s <= T + 1e-6; s += T > 150 ? 60 : 30) g.fillText(`${s} s`, X(s), H - 8);
    // The 80% line.
    g.setLineDash([8, 8]);
    g.strokeStyle = 'rgba(255,211,107,0.55)';
    g.beginPath();
    g.moveTo(padL, Y(0.8));
    g.lineTo(W - padR, Y(0.8));
    g.stroke();
    g.setLineDash([]);
    const curve = (vals: (k: number) => number, len: number, dt: number, upTo: number, color: string, width: number, fill?: string) => {
      g.beginPath();
      let lastX = padL;
      let any = false;
      for (let k = 0; k < len; k++) {
        const s = k * dt;
        if (s > upTo) break;
        const x = X(s);
        const y = Y(vals(k));
        if (!any) g.moveTo(x, y);
        else g.lineTo(x, y);
        lastX = x;
        any = true;
      }
      if (!any) return;
      g.strokeStyle = color;
      g.lineWidth = width;
      g.stroke();
      if (fill) {
        g.lineTo(lastX, Y(0));
        g.lineTo(padL, Y(0));
        g.closePath();
        g.fillStyle = fill;
        g.fill();
      }
    };
    const F = this.b.frames;
    const solo = this.b.solo;
    // The one-drone run shows to the same moment, so the gap opens as the film plays.
    if (solo) curve((k) => solo.coverage[k]!, solo.coverage.length, solo.dt, t, 'rgba(190,200,210,0.75)', 3);
    curve((k) => F[k]![2]!, F.length, this.b.dt, t, 'rgba(232,251,255,0.45)', 2.5);
    curve((k) => F[k]![1]!, F.length, this.b.dt, t, '#ffffff', 4.5, 'rgba(62,224,255,0.14)');
    const cov = this.track.coverage(t);
    g.fillStyle = '#ffffff';
    g.beginPath();
    g.arc(X(Math.min(t, this.track.end)), Y(cov), 7, 0, Math.PI * 2);
    g.fill();
  }

  update(tau: number, t: number, speed: number, samples: DroneSample[], camera: THREE.PerspectiveCamera, droneWorld: THREE.Vector3[], baseWorld: THREE.Vector3, introEnd: number, simEndFilm: number): void {
    const introFade = Math.max(0, Math.min(1, (introEnd - 0.4 - tau) / 0.8));
    const introIn = Math.max(0, Math.min(1, (tau - 0.3) / 0.8));
    this.intro.style.opacity = String(introFade * introIn);
    const live = tau > introEnd - 0.6 && tau < simEndFilm + 1.2;
    const liveOp = Math.max(0, Math.min(1, (tau - (introEnd - 0.6)) / 0.8)) * Math.max(0, Math.min(1, (simEndFilm + 1.2 - tau) / 0.8));
    this.root.style.setProperty('--live', String(liveOp));
    const resOp = Math.max(0, Math.min(1, (tau - simEndFilm - 1.4) / 0.9));
    this.results.style.opacity = String(resOp);
    this.results.style.transform = `translateY(-50%) translateX(${(1 - resOp) * 24}px)`;

    this.clockValue.textContent = clock(t);
    this.speedPill.textContent = speed <= 0 ? (tau < introEnd ? 'Ready' : 'Done') : speed < 1.05 ? 'True speed' : `${speed.toFixed(1).replace(/\.0$/, '')}× speed`;
    this.speedPill.classList.toggle('is-fast', speed > 1.05);

    const shares = this.track.shares(t);
    for (let d = 0; d < this.cards.length; d++) {
      const c = this.cards[d]!;
      const s = samples[d]!;
      c.mode.textContent = MODE_LABEL[s.mode];
      c.root.dataset.mode = s.mode;
      c.share.textContent = `${(shares[d]! * 100).toFixed(0)}%`;
      c.bar.style.width = `${(shares[d]! * 100).toFixed(1)}%`;
      c.ekf.textContent = `${s.ekfErr.toFixed(2)} m`;
      c.tilt.textContent = `${((Math.acos(Math.cos(s.roll) * Math.cos(s.pitch)) * 180) / Math.PI).toFixed(0)}°`;
      c.speed.textContent = `${Math.hypot(s.vx, s.vy).toFixed(1)} m/s`;
      this.place(c.label, droneWorld[d]!, camera, live && s.mode !== 'waiting' ? liveOp : 0, -34);
    }
    this.place(this.baseLabel, baseWorld, camera, live ? liveOp : 0, -48);

    const cov = this.track.coverage(t);
    this.covValue.textContent = `${(cov * 100).toFixed(1)}%`;
    const solo = this.b.solo;
    if (solo) {
      const k = Math.min(solo.coverage.length - 1, Math.round(t / solo.dt));
      this.covSoloValue!.textContent = `${(solo.coverage[k]! * 100).toFixed(1)}%`;
    }
    this.drawChart(t);
    const bk = this.track.baseKnown(t);
    this.baseValue.textContent = `${(bk * 100).toFixed(0)}%`;
    this.baseBar.style.width = `${(bk * 100).toFixed(1)}%`;

    // The last few events, fading out after a few seconds of film.
    const recent = this.track.events.filter((e) => e.t <= t && e.t > t - 6 * Math.max(1, speed));
    this.ticker.replaceChildren();
    for (const e of recent.slice(-4)) {
      const age = (t - e.t) / Math.max(1, speed);
      const item = el('div', 'sw-tick', this.ticker, e.text);
      if (e.drone !== null) item.style.setProperty('--c', DRONE_COLORS[e.drone % DRONE_COLORS.length]!);
      else item.classList.add(e.warn ? 'is-warn' : 'is-team');
      item.style.opacity = String(liveOp * Math.min(1, age / 0.3) * Math.max(0, Math.min(1, (6 - age) / 1.2)));
    }
  }

  private place(label: HTMLElement, world: THREE.Vector3, camera: THREE.PerspectiveCamera, opacity: number, dy: number): void {
    this.v.copy(world).project(camera);
    const px = ((this.v.x + 1) / 2) * 1920;
    const py = ((1 - this.v.y) / 2) * 1080 + dy;
    // Never under the panels: the drone cards on the right, the coverage and base panels bottom left.
    const underPanel = px > 1560 || (px < 670 && py > 680) || (px >= 670 && px < 1020 && py > 800) || (px < 900 && py < 130);
    const vis = this.v.z < 1 && Math.abs(this.v.x) < 1.05 && Math.abs(this.v.y) < 1.05 && !underPanel;
    label.style.opacity = vis ? String(opacity) : '0';
    label.style.transform = `translate(${px}px, ${py}px) translate(-50%, -50%)`;
  }
}
