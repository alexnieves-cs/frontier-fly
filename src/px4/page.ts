import '../world/outdoor/viz/outdoor.css';
import './px4.css';
import { REPO_URL, siteNav } from '../ui/siteNav';
import { buildMissionWorld } from '../mission/autonomy/world';
import { woodsSampleMission } from '../mission/outdoorWorld';
import { buildFlightPlan, exportStem, type FlightPlan } from '../mission/export/flightPlan';
import { qgcPlanJson } from '../mission/export/formats';
import { GEO_ORIGINS } from '../mission/export/geo';
import { routeFromCoverage, type MissionRoute } from '../mission/export/route';
import { sampleHeight } from '../world/outdoor/terrain';
import type { OutdoorWorld } from '../world/outdoor/types';
import { drawOutdoorMap, mapProject } from '../world/outdoor/viz/map';
import { OutdoorView } from '../world/outdoor/viz/scene';
import type { StartMessage, WorkerOut } from './bridge.worker';

/**
 * px4.html: the browser sim as PX4's simulator. PX4 SITL flies the woods
 * through scripts/px4/bridge.mjs; this page runs the physics and sensors (in
 * a worker) and draws the vehicle where PX4 put it, with the exported
 * mission's route and the flown track on the map.
 */

type State = Extract<WorkerOut, { type: 'state' }>;

const DEFAULT_SEED = 4242;

async function start(root: HTMLElement): Promise<void> {
  const params = new URLSearchParams(location.search);
  const seed = Number(params.get('seed') ?? DEFAULT_SEED) >>> 0;
  const url = params.get('ws') ?? 'ws://127.0.0.1:4561';
  const speed = Math.max(0, Number(params.get('speed') ?? 1));
  root.innerHTML = markup(seed, url);
  const boot = must<HTMLElement>(root, '#boot');
  try {
    const world = buildMissionWorld({ seed });
    // The mission editor's default woods mission, so Download .plan matches its export.
    const mission = woodsSampleMission(seed);
    const route = routeFromCoverage(mission, world);
    const plan = buildFlightPlan(mission, route, {
      home: GEO_ORIGINS['px4-sitl'],
      firmware: 'px4',
      altitudeRef: 'relative',
      finish: 'rtl',
      video: false,
    });
    const outdoor = world.outdoor;
    const view = new OutdoorView(must<HTMLCanvasElement>(root, '#view'));
    view.setWorld(outdoor);
    const ui = new Px4Page(root, view, outdoor, route, plan);
    must<HTMLButtonElement>(root, '#download').addEventListener('click', () => ui.downloadPlan());
    window.addEventListener('resize', () => view.resize());

    const worker = new Worker(new URL('./bridge.worker.ts', import.meta.url), { type: 'module', name: 'frontier-px4' });
    worker.onmessage = (ev: MessageEvent<WorkerOut>) => ui.onWorker(ev.data);
    const pad = { x: route.home.x, y: route.home.y, groundZ: route.home.groundZ };
    const startMsg: StartMessage = { type: 'start', url, home: GEO_ORIGINS['px4-sitl'], pad, terrain: outdoor.terrain, speed };
    worker.postMessage(startMsg);
    boot.remove();
    root.dataset.ready = '1';
    root.dataset.seed = String(seed);
    ui.loop();
  } catch (error) {
    boot.textContent = error instanceof Error ? error.message : String(error);
    console.error(error);
  }
}

class Px4Page {
  private state: State | null = null;
  private readonly track: [number, number][] = [];
  private maxAgl = 0;
  private flown = 0;

  constructor(
    private readonly root: HTMLElement,
    private readonly view: OutdoorView,
    private readonly world: OutdoorWorld,
    private readonly route: MissionRoute,
    private readonly plan: FlightPlan,
  ) {
    must<HTMLElement>(root, '#plan-summary').textContent =
      `${plan.summary.waypoints} waypoints, ${plan.fence.length - 1} no-fly zones, about ${plan.summary.estimatedSec} s`;
    for (const mode of ['chase', 'fpv'] as const) {
      must<HTMLButtonElement>(root, `#cam-${mode}`).addEventListener('click', () => {
        view.setCamera(mode);
        must<HTMLButtonElement>(root, '#cam-chase').dataset.on = mode === 'chase' ? '1' : '0';
        must<HTMLButtonElement>(root, '#cam-fpv').dataset.on = mode === 'fpv' ? '1' : '0';
      });
    }
  }

  onWorker(m: WorkerOut): void {
    if (m.type === 'status') {
      this.root.dataset.link = m.link;
      must<HTMLElement>(this.root, '#link').textContent = m.text;
      must<HTMLElement>(this.root, '#link').dataset.link = m.link;
      must<HTMLElement>(this.root, '#help').hidden = m.link === 'px4';
      return;
    }
    const prev = this.state;
    this.state = m;
    const agl = m.pos[2] - sampleHeight(this.world.terrain, m.pos[0], m.pos[1]);
    if (m.armed) this.maxAgl = Math.max(this.maxAgl, agl);
    if (prev && !m.onGround) this.flown += Math.hypot(m.pos[0] - prev.pos[0], m.pos[1] - prev.pos[1]);
    const last = this.track[this.track.length - 1];
    if (!last || Math.hypot(m.pos[0] - last[0], m.pos[1] - last[1]) > 0.5) this.track.push([m.pos[0], m.pos[1]]);
    const ds = this.root.dataset;
    ds.armed = m.armed ? '1' : '0';
    ds.simTime = m.simTimeSec.toFixed(1);
    ds.maxAgl = this.maxAgl.toFixed(2);
    ds.flownM = this.flown.toFixed(1);
    ds.timeouts = String(m.timeouts);
    const speed = Math.hypot(m.vel[0], m.vel[1], m.vel[2]);
    set(this.root, '#hud-armed', m.armed ? 'armed' : 'disarmed');
    set(this.root, '#hud-time', `${m.simTimeSec.toFixed(1)} s`);
    set(this.root, '#hud-rtf', `${m.rtf.toFixed(2)}× · ${Math.round(m.stepsPerSec)} Hz`);
    set(this.root, '#hud-alt', `${agl.toFixed(1)} m`);
    set(this.root, '#hud-speed', `${speed.toFixed(1)} m/s`);
    set(this.root, '#hud-att', `${deg(m.roll)}° / ${deg(m.pitch)}°`);
    const bars = this.root.querySelectorAll<HTMLElement>('.motor i');
    m.motors.forEach((u, i) => {
      const bar = bars[i];
      if (bar) bar.style.height = `${Math.round(Math.max(0, Math.min(1, u)) * 100)}%`;
    });
  }

  loop = (): void => {
    const s = this.state;
    const pad = this.route.home;
    const pos: [number, number, number] = s ? s.pos : [pad.x, pad.y, pad.groundZ];
    const yaw = s ? s.yaw : 0;
    const spin = s ? s.motors.reduce((a, b) => a + b, 0) * 0.6 : 0;
    this.view.update(pos, yaw, spin);
    const map = must<HTMLCanvasElement>(this.root, '#map');
    const ctx = map.getContext('2d');
    if (ctx) {
      drawOutdoorMap(ctx, this.world, { x: pos[0], y: pos[1], yaw }, map.clientWidth, map.clientHeight);
      this.drawRoute(ctx, map.clientWidth, map.clientHeight);
    }
    requestAnimationFrame(this.loop);
  };

  private drawRoute(ctx: CanvasRenderingContext2D, w: number, h: number): void {
    const { toX, toY } = mapProject(this.world, w, h);
    const pad = this.route.home;
    ctx.save();
    ctx.setLineDash([4, 3]);
    ctx.strokeStyle = 'rgba(255, 210, 120, 0.9)';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(toX(pad.x), toY(pad.y));
    for (const leg of this.route.legs) for (const p of leg.points) ctx.lineTo(toX(p.x), toY(p.y));
    ctx.lineTo(toX(pad.x), toY(pad.y));
    ctx.stroke();
    ctx.setLineDash([]);
    // The geofence is cyan on this map, so the flown track is white.
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    this.track.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(toX(x), toY(y)) : ctx.lineTo(toX(x), toY(y))));
    ctx.stroke();
    ctx.restore();
  }

  downloadPlan(): void {
    const name = `${exportStem(this.plan)}-px4.plan`;
    const href = URL.createObjectURL(new Blob([qgcPlanJson(this.plan)], { type: 'application/json' }));
    const a = document.createElement('a');
    a.href = href;
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(href), 1000);
  }
}

function deg(rad: number): string {
  return ((rad * 180) / Math.PI).toFixed(0);
}

function set(root: ParentNode, selector: string, text: string): void {
  const node = root.querySelector(selector);
  if (node) node.textContent = text;
}

function markup(seed: number, url: string): string {
  return `
    <header class="topbar">
      <div class="brand"><strong>frontier-fly</strong><h1>PX4 in the loop</h1></div>
      ${siteNav('px4')}
      <div class="controls">
        <a href="${REPO_URL}/blob/main/docs/PX4.md" rel="noopener">how it works</a>
        <span class="muted">woods seed ${seed}</span>
        <span class="muted" id="plan-summary"></span>
        <button id="download" type="button" data-testid="px4-download">Download .plan</button>
      </div>
    </header>
    <main class="stage">
      <canvas id="view" role="img" aria-label="3D view of the drone over the woods"></canvas>
      <canvas id="map" width="240" height="240" role="img" aria-label="Top-down map with the mission route"></canvas>
      <div class="cam">
        <button id="cam-chase" type="button" data-on="1">chase</button>
        <button id="cam-fpv" type="button">FPV</button>
      </div>
      <div class="hud px4-hud">
        <span id="link" data-link="connecting">Starting…</span>
        <span>PX4 <strong id="hud-armed">—</strong></span>
        <span>sim time <strong id="hud-time">0 s</strong></span>
        <span>lockstep <strong id="hud-rtf">—</strong></span>
        <span>agl <strong id="hud-alt">0 m</strong></span>
        <span>speed <strong id="hud-speed">0 m/s</strong></span>
        <span>roll / pitch <strong id="hud-att">0° / 0°</strong></span>
        <div class="motors" role="img" aria-label="PX4 motor outputs">
          <span class="motor"><i></i></span><span class="motor"><i></i></span><span class="motor"><i></i></span><span class="motor"><i></i></span>
        </div>
      </div>
      <aside class="help" id="help">
        <h2>Let PX4 fly this page</h2>
        <p>This page is PX4's simulator: PX4 sends motor commands, the page answers with IMU, compass, barometer and GNSS readings, in lockstep. It needs PX4 SITL and a small relay running on your computer.</p>
        <ol>
          <li>Build PX4 once: <code>make px4_sitl_default</code> in PX4-Autopilot (v1.15).</li>
          <li>Start the relay: <code>node scripts/px4/bridge.mjs</code> in this repository.</li>
          <li>Start PX4 with the generic quad: <code>PX4_SYS_AUTOSTART=10016 build/px4_sitl_default/bin/px4</code>.</li>
          <li>Open QGroundControl, load the plan from <b>Download .plan</b>, upload it and start the mission. Or run <code>python3 scripts/px4/flight_check.py file.plan</code>.</li>
        </ol>
        <p class="muted">Relay: <code>${escapeHtml(url)}</code>. Simulators and practice fields only.</p>
      </aside>
      <div class="boot" id="boot" role="status">generating the woods…</div>
    </main>
  `;
}

function escapeHtml(s: string): string {
  return s.replace(/[&<>"']/gu, (c) => `&#${c.charCodeAt(0)};`);
}

function must<T extends Element>(root: ParentNode, selector: string): T {
  const node = root.querySelector(selector);
  if (!node) throw new Error(`missing ${selector}`);
  return node as T;
}

const root = document.querySelector('#app');
if (root instanceof HTMLElement) void start(root);
