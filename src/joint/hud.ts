import * as THREE from 'three';
import { dist, epochAt, solutionAt, spoofedAt, truthAt, type GateState, type LegBundle } from './leg';
import { LEG_COLORS } from './scene';

const STATE_LABEL: Record<GateState, string> = {
  NO_FIX: 'NO MAP FIX YET',
  TRUSTED: 'GPS TRUSTED',
  SUSPECT: 'GPS SUSPECT',
  REJECTED: 'GPS REJECTED',
};

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls: string, parent: HTMLElement, text?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  e.className = cls;
  if (text !== undefined) e.textContent = text;
  parent.appendChild(e);
  return e;
}

const fmt = (v: number | null, d = 1) => (v === null || !Number.isFinite(v) ? '–' : v >= 100 ? Math.round(v).toString() : v.toFixed(d));

/**
 * The leg film's overlay: what the drone's camera sees beside the old map patch it is matched to,
 * the numbers that matter (DSS's true error, its protection level, the alert limit, the GPS gate),
 * a running chart of all of them, captions, and the imagery credit.
 */
export class LegHud {
  private readonly root: HTMLElement;
  private readonly camImg: HTMLImageElement;
  private readonly mapCanvas: HTMLCanvasElement;
  private readonly matchLine: HTMLElement;
  private readonly gate: HTMLElement;
  private readonly errV: HTMLElement;
  private readonly plV: HTMLElement;
  private readonly gpsV: HTMLElement;
  private readonly bar: { err: HTMLElement; pl: HTMLElement; alert: HTMLElement };
  private readonly caption: HTMLElement;
  private readonly clock: HTMLElement;
  private readonly scaleNote: HTMLElement;
  private readonly chart: HTMLCanvasElement;
  private readonly endCard: HTMLElement;
  private readonly tags: { drone: HTMLElement; ghost: HTMLElement; svg: SVGSVGElement; lines: SVGLineElement[] };
  private mapImg: HTMLImageElement | null = null;
  private readonly frames = new Map<number, HTMLImageElement>();

  constructor(
    stage: HTMLElement,
    private readonly leg: LegBundle,
    private readonly base: string,
  ) {
    this.root = el('div', 'leg-hud', stage);
    const top = el('div', 'leg-title', this.root);
    el('div', 'leg-kicker', top, 'GNSS-denied ingress · real aerial imagery');
    el('div', 'leg-h1', top, 'The drone finds itself on a 2009 map, from 2013 camera frames');
    el('div', 'leg-sub', top, `Madrid, PNOA tile ${leg.title.split(':')[0]!.replace('Madrid ', '')} · Denied Signal Stack footage lab`);

    const pip = el('div', 'leg-pip', this.root);
    const camBox = el('div', 'leg-pane', pip);
    el('div', 'leg-pane-label', camBox, 'Drone camera · rendered from PNOA 2013');
    this.camImg = el('img', 'leg-cam', camBox);
    const mapBox = el('div', 'leg-pane', pip);
    el('div', 'leg-pane-label', mapBox, 'Onboard map · PNOA 2009, four years older');
    this.mapCanvas = el('canvas', 'leg-map', mapBox);
    this.mapCanvas.width = 400;
    this.mapCanvas.height = 300;
    this.matchLine = el('div', 'leg-match', pip);

    const panel = el('div', 'leg-panel', this.root);
    this.gate = el('div', 'leg-gate', panel);
    const row = (label: string, cls: string) => {
      const r = el('div', `leg-row ${cls}`, panel);
      el('div', 'leg-row-label', r, label);
      return el('div', 'leg-row-value', r);
    };
    this.errV = row('DSS error (vs truth)', 'is-dss');
    this.plV = row('Protection level', 'is-pl');
    row('Alert limit', 'is-alert').textContent = `${leg.alertLimitM} m`;
    this.gpsV = row('GPS receiver error', 'is-gps');
    const gauge = el('div', 'leg-gauge', panel);
    this.bar = { err: el('div', 'leg-gauge-err', gauge), pl: el('div', 'leg-gauge-pl', gauge), alert: el('div', 'leg-gauge-alert', gauge) };
    el('div', 'leg-gauge-note', panel, 'Safe while the error stays inside the protection level, and usable while that is inside the alert limit.');

    const legend = el('div', 'leg-legend', this.root);
    for (const [cls, text] of [
      ['is-truth', 'True path'],
      ['is-dss', 'DSS estimate'],
      ['is-gps', 'GPS (spoofed)'],
      ['is-fix', 'Map fix used'],
      ['is-reject', 'Map fix rejected'],
    ] as const) {
      const i = el('div', `leg-legend-item ${cls}`, legend);
      el('span', 'leg-swatch', i);
      el('span', '', i, text);
    }

    this.caption = el('div', 'leg-caption', this.root);
    this.clock = el('div', 'leg-clock', this.root);
    this.scaleNote = el('div', 'leg-scale', this.root, 'Drone drawn larger than life at this distance');
    this.chart = el('canvas', 'leg-chart', this.root);
    this.chart.width = 1840;
    this.chart.height = 150;
    el('div', 'leg-credit', this.root, `${leg.ground.attribution} · ${leg.map.attribution}. A simulated flight over real imagery, not a flight test.`);
    this.endCard = el('div', 'leg-end', this.root);
    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    svg.setAttribute('class', 'leg-tag-lines');
    svg.setAttribute('viewBox', '0 0 1920 1080');
    this.root.insertBefore(svg, this.root.firstChild);
    const lines = [0, 1].map(() => {
      const l = document.createElementNS('http://www.w3.org/2000/svg', 'line');
      svg.appendChild(l);
      return l;
    });
    this.tags = { drone: el('div', 'leg-tag is-dss', this.root), ghost: el('div', 'leg-tag is-gps', this.root), svg, lines };
  }

  /** Labels on the drone and on its GPS ghost, pinned to where the 3D view draws them. */
  private label(i: number, tag: HTMLElement, at: THREE.Vector3 | null, camera: THREE.Camera, text: string, side: 1 | -1, up: 1 | -1): void {
    const line = this.tags.lines[i]!;
    const v = at?.clone().project(camera);
    const on = v && v.z < 1 && Math.abs(v.x) < 0.92 && Math.abs(v.y) < 0.92;
    tag.style.opacity = on ? '1' : '0';
    line.style.opacity = on ? '1' : '0';
    if (!on || !v) return;
    const x = ((v.x + 1) / 2) * 1920;
    const y = ((1 - v.y) / 2) * 1080;
    const lx = x + side * 70;
    const ly = y - up * 64;
    tag.textContent = text;
    tag.style.left = `${lx}px`;
    tag.style.top = `${ly}px`;
    tag.style.transform = side > 0 ? 'translate(0, -50%)' : 'translate(-100%, -50%)';
    line.setAttribute('x1', String(x));
    line.setAttribute('y1', String(y));
    line.setAttribute('x2', String(lx));
    line.setAttribute('y2', String(ly));
  }

  async load(): Promise<void> {
    const img = new Image();
    img.src = new URL(this.leg.map.file, this.base).href;
    await img.decode();
    this.mapImg = img;
    // Every frame up front: a recorder must never screenshot a frame whose picture has not decoded.
    await Promise.all(
      this.leg.epochs.map(async (_, i) => {
        const f = new Image();
        f.src = new URL(`${this.leg.camera.framesDir}/${String(i).padStart(4, '0')}.jpg`, this.base).href;
        await f.decode();
        this.frames.set(i, f);
      }),
    );
  }

  private drawMap(i: number): void {
    const g = this.mapCanvas.getContext('2d')!;
    const leg = this.leg;
    const e = leg.epochs[i]!;
    const W = this.mapCanvas.width;
    const H = this.mapCanvas.height;
    g.fillStyle = '#0a1117';
    g.fillRect(0, 0, W, H);
    if (!this.mapImg) return;
    const ext = leg.map.extent;
    const mpp = (ext.xmax - ext.xmin) / leg.map.w;
    // A 140 m window on the old map, centred where the fix landed (or on the truth with no fix).
    const c = e.fix.xy ?? e.truth;
    const viewM = 140;
    const sx = (c[0] - viewM / 2 - ext.xmin) / mpp;
    const sy = (ext.ymax - (c[1] + (viewM * H) / W / 2)) / mpp;
    g.drawImage(this.mapImg, sx, sy, viewM / mpp, (viewM * H) / W / mpp, 0, 0, W, H);
    const px = W / viewM;
    const toPx = (x: number, y: number) => [W / 2 + (x - c[0]) * px, H / 2 - (y - c[1]) * px] as const;
    // The 96 m patch the matcher looked for, at the fix, turned to the drone's heading.
    const th = ((e.heading ?? 0) * Math.PI) / 180;
    const half = leg.camera.patchM / 2;
    const corners = [
      [-half, half],
      [half, half],
      [half, -half],
      [-half, -half],
    ].map(([a, b]) => toPx(c[0] + a! * Math.cos(th) + b! * Math.sin(th), c[1] - a! * Math.sin(th) + b! * Math.cos(th)));
    g.lineWidth = 3;
    g.strokeStyle = e.fix.used ? LEG_COLORS.fix : e.fix.xy ? LEG_COLORS.reject : 'rgba(255,255,255,0.5)';
    g.beginPath();
    corners.forEach(([x, y], k) => (k ? g.lineTo(x, y) : g.moveTo(x, y)));
    g.closePath();
    g.stroke();
    // The true position, for the viewer: the matcher never sees it.
    const [tx, ty] = toPx(e.truth[0], e.truth[1]);
    g.strokeStyle = '#ffffff';
    g.lineWidth = 2;
    g.beginPath();
    g.moveTo(tx - 9, ty);
    g.lineTo(tx + 9, ty);
    g.moveTo(tx, ty - 9);
    g.lineTo(tx, ty + 9);
    g.stroke();
  }

  private drawChart(t: number): void {
    const g = this.chart.getContext('2d')!;
    const W = this.chart.width;
    const H = this.chart.height;
    const leg = this.leg;
    const E = leg.epochs;
    const t0 = E[0]!.t;
    const t1 = E[E.length - 1]!.t;
    const padL = 54;
    const padB = 26;
    const top = 10;
    const yMax = 40;
    const X = (tt: number) => padL + ((tt - t0) / (t1 - t0)) * (W - padL - 10);
    const Y = (m: number) => top + (1 - Math.min(m, yMax) / yMax) * (H - top - padB);
    g.clearRect(0, 0, W, H);
    g.fillStyle = 'rgba(6,12,17,0.72)';
    g.fillRect(0, 0, W, H);
    g.font = '500 15px Inter, Helvetica, Arial, sans-serif';
    g.fillStyle = 'rgba(214,226,234,0.75)';
    g.strokeStyle = 'rgba(255,255,255,0.08)';
    g.lineWidth = 1;
    for (const m of [0, 10, 20, 30, 40]) {
      g.beginPath();
      g.moveTo(padL, Y(m));
      g.lineTo(W - 10, Y(m));
      g.stroke();
      g.fillText(`${m} m`, 8, Y(m) + 5);
    }
    for (let s = 0; s <= t1; s += 10) g.fillText(`${s} s`, X(s) - 10, H - 6);
    const k = epochAt(leg, t);
    // Gate state strip.
    for (let i = 0; i < k; i++) {
      const st = E[i]!.gated.state;
      g.fillStyle = st === 'REJECTED' ? 'rgba(255,77,94,0.55)' : st === 'SUSPECT' ? 'rgba(255,181,71,0.6)' : st === 'TRUSTED' ? 'rgba(61,255,154,0.35)' : 'rgba(150,150,150,0.3)';
      g.fillRect(X(E[i]!.t), H - padB - 4, X(E[i + 1]!.t) - X(E[i]!.t) + 0.5, 4);
    }
    // Protection level as a band under its line.
    g.beginPath();
    g.moveTo(X(t0), Y(0));
    for (let i = 0; i <= k; i++) g.lineTo(X(E[i]!.t), Y(E[i]!.gated.pl ?? yMax));
    g.lineTo(X(E[k]!.t), Y(0));
    g.closePath();
    g.fillStyle = 'rgba(86,224,255,0.13)';
    g.fill();
    const line = (pick: (i: number) => number | null, color: string, width: number, dash: number[] = []) => {
      g.beginPath();
      g.setLineDash(dash);
      let on = false;
      for (let i = 0; i <= k; i++) {
        const v = pick(i);
        if (v === null) {
          on = false;
          continue;
        }
        if (on) g.lineTo(X(E[i]!.t), Y(v));
        else g.moveTo(X(E[i]!.t), Y(v));
        on = true;
      }
      g.strokeStyle = color;
      g.lineWidth = width;
      g.stroke();
      g.setLineDash([]);
    };
    line((i) => E[i]!.gated.pl, 'rgba(86,224,255,0.85)', 1.5, [6, 4]);
    line((i) => (E[i]!.t >= leg.spoof.startS ? dist(E[i]!.spoofed, E[i]!.truth) : null), LEG_COLORS.gps, 2.5);
    line((i) => dist(E[i]!.gated.pub, E[i]!.truth), LEG_COLORS.dss, 2.5);
    g.setLineDash([10, 6]);
    g.strokeStyle = LEG_COLORS.alert;
    g.lineWidth = 1.6;
    g.beginPath();
    g.moveTo(padL, Y(leg.alertLimitM));
    g.lineTo(W - 10, Y(leg.alertLimitM));
    g.stroke();
    g.setLineDash([]);
    g.fillStyle = LEG_COLORS.alert;
    g.fillText('alert limit', W - 96, Y(leg.alertLimitM) - 6);
    g.strokeStyle = '#ffffff';
    g.lineWidth = 1.5;
    g.beginPath();
    g.moveTo(X(Math.min(t, t1)), top);
    g.lineTo(X(Math.min(t, t1)), H - padB);
    g.stroke();
  }

  update(t: number, droneScale: number, camera: THREE.Camera, drone: THREE.Vector3, ghost: THREE.Vector3 | null): void {
    const leg = this.leg;
    const E = leg.epochs;
    const t0 = E[0]!.t;
    const t1 = E[E.length - 1]!.t;
    const tc = Math.max(t0, Math.min(t1, t));
    const k = epochAt(leg, tc);
    const e = E[k]!;
    const frame = this.frames.get(k);
    if (frame && this.camImg.src !== frame.src) this.camImg.src = frame.src;
    this.drawMap(k);
    const f = e.fix;
    this.matchLine.className = `leg-match ${f.used ? 'is-used' : f.xy ? 'is-rejected' : 'is-none'}`;
    this.matchLine.textContent = f.used
      ? `Map fix used · correlation peak ${fmt(f.peak, 2)}, ${fmt(f.ratio, 1)}× the runner-up`
      : f.xy
        ? `Map fix rejected · it disagreed with the track`
        : `No usable match · ${f.reason.startsWith('peak') ? 'peak too weak or ambiguous' : f.reason}`;

    const tr = truthAt(leg, tc);
    const est = solutionAt(leg, tc, 'gated');
    const err = dist(est, [tr.x, tr.y]);
    const pl = e.gated.pl;
    const gpsErr = dist(spoofedAt(leg, tc), [tr.x, tr.y]);
    this.errV.textContent = `${fmt(err)} m`;
    this.plV.textContent = `${fmt(pl)} m`;
    this.plV.parentElement!.classList.toggle('is-over', (pl ?? 0) > leg.alertLimitM);
    this.gpsV.textContent = `${fmt(gpsErr)} m`;
    // The ghost is drawn where blind fusion (GPS always trusted) puts the drone, so its label quotes that error.
    const blindErr = dist(solutionAt(leg, tc, 'blind'), [tr.x, tr.y]);
    const labels = droneScale > 2.5 && t >= t0 - 0.5 && t <= t1 + 0.6;
    // Each label on the side away from the other object, the drone's below and the ghost's above, so they never collide.
    const gx = ghost ? ghost.clone().project(camera).x : null;
    const dx = drone.clone().project(camera).x;
    const ghostRight = gx === null || gx >= dx;
    this.label(0, this.tags.drone, labels ? drone : null, camera, `Drone · DSS off by ${fmt(err)} m`, ghostRight ? -1 : 1, ghost ? -1 : 1);
    this.label(1, this.tags.ghost, labels ? ghost : null, camera, `GPS-trusting autopilot · ${fmt(blindErr, 0)} m off`, ghostRight ? 1 : -1, 1);
    const st = e.gated.state;
    this.gate.className = `leg-gate is-${st.toLowerCase()}`;
    this.gate.textContent = tc >= leg.spoof.startS && st === 'TRUSTED' ? 'GPS TRUSTED · being spoofed' : STATE_LABEL[st];
    const scaleMax = 30;
    const pct = (v: number | null) => `${Math.min(100, ((v ?? 0) / scaleMax) * 100)}%`;
    this.bar.err.style.width = pct(err);
    this.bar.pl.style.width = pct(pl);
    this.bar.alert.style.left = pct(leg.alertLimitM);

    const ts = leg.spoof.startS;
    const trj = leg.firstRejectS.gated;
    let cap = '';
    if (t < t0 + 0.5) cap = 'No GNSS is fused. Every position below comes from matching the camera to an old map.';
    else if (t < ts - 1) cap = 'Each patch of ground the camera sees is searched for on the 2009 map. Green: a fix the filter takes.';
    else if (t < ts + 4) cap = `GNSS spoofing starts at ${ts} s: the receiver is dragged ${leg.spoof.rateMps} m/s to the north-east.`;
    else if (trj !== null && t < trj && (pl ?? 0) > leg.alertLimitM)
      cap = `GPS and the map disagree, so DSS widens its bound to ${fmt(pl, 0)} m rather than vouch for a dragged position.`;
    else if (trj !== null && t < trj) cap = 'The gate compares each GPS reading with where the map fixes say the drone is.';
    else if (trj !== null && t < trj + 6) cap = `GPS rejected at ${trj.toFixed(1)} s. DSS keeps navigating on the map alone.`;
    else if (t < t1 - 6) cap = 'In red, where a GPS-trusting autopilot thinks it is. DSS stays on the true path.';
    else if (t <= t1) cap = 'The protection level is the bound DSS publishes: the true error never crossed it on this leg.';
    this.caption.textContent = cap;
    this.caption.style.opacity = cap ? '1' : '0';
    this.clock.textContent = `t = ${Math.max(0, tc).toFixed(1)} s`;
    this.scaleNote.style.opacity = droneScale > 1.6 ? '1' : '0';
    this.drawChart(tc);

    if (t > t1 + 0.8) {
      const end = E[E.length - 1]!;
      const vErr = dist(end.vision.pub, end.truth);
      const blindErr = dist(end.blind.pub, end.truth);
      this.endCard.style.opacity = String(Math.min(1, (t - t1 - 0.8) / 1.2));
      this.endCard.replaceChildren();
      el('div', 'leg-end-kicker', this.endCard, 'Handoff at the treeline');
      const grid = el('div', 'leg-end-grid', this.endCard);
      const stat = (v: string, label: string, cls = '') => {
        const s = el('div', `leg-end-stat ${cls}`, grid);
        el('div', 'leg-end-v', s, v);
        el('div', 'leg-end-l', s, label);
      };
      stat(`${fmt(dist(end.gated.pub, end.truth))} m`, 'DSS error at handoff', 'is-dss');
      stat(`${fmt(end.gated.pl)} m`, 'protection level', 'is-pl');
      stat(`${fmt(blindErr, 0)} m`, 'GPS-trusting autopilot', 'is-gps');
      stat(`${fmt(vErr)} m`, 'map only, GPS never used', '');
      el('div', 'leg-end-note', this.endCard, (end.gated.pl ?? 99) <= leg.alertLimitM ? 'Under the 10 m alert limit: the drone is cleared to start the building search.' : 'Over the 10 m alert limit: the drone holds at the treeline.');
    } else this.endCard.style.opacity = '0';
  }
}
