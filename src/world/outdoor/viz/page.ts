import './outdoor.css';
import type { DynParams } from '../../../shared/types';
import { makeDroneState, scaleAction } from '../../../sim/drone';
import { KinematicDynamics } from '../../../sim/dynamics/kinematic';
import { actionFromKeys } from '../../../sim/manual';
import { generateOutdoor } from '../generate';
import { containsPoint, inNoFly } from '../geofence';
import { initOutdoorPhysics, OutdoorPhysics } from '../physics';
import { sampleHeight } from '../terrain';
import type { OutdoorWorld } from '../types';
import { drawOutdoorMap } from './map';
import { OutdoorView, type OutdoorCamera } from './scene';

const DYN: DynParams = {
  mass: 0.4,
  inertia: [0.002, 0.002, 0.004],
  arm: 0.1,
  kf: 1e-5,
  km: 1e-7,
  motorTau: 0.03,
  dragLin: [0.1, 0.1, 0.1],
  dragQuad: [0.05, 0.05, 0.05],
};

export async function startOutdoor(root: HTMLElement): Promise<void> {
  const params = new URLSearchParams(location.search);
  const shot = params.get('shot');
  root.innerHTML = markup();
  if (shot === 'map' || shot === 'chase' || shot === 'fpv') document.body.dataset.shot = shot;
  const seedInput = must<HTMLInputElement>(root, '#seed');
  seedInput.value = params.get('seed') ?? '4';
  const canvas = must<HTMLCanvasElement>(root, '#view');
  const map = must<HTMLCanvasElement>(root, '#map');
  const view = new OutdoorView(canvas);
  const boot = must<HTMLElement>(root, '#boot');
  try {
    await initOutdoorPhysics();
    let world = generateOutdoor({ seed: readSeed(seedInput) });
    let physics = new OutdoorPhysics(world);
    view.setWorld(world);
    const camera: OutdoorCamera = shot === 'fpv' ? 'fpv' : 'chase';
    view.setCamera(camera);
    const session = new Session(root, view, map, world, physics, camera);
    session.advanceForShot(shot);
    boot.remove();
    root.dataset.ready = '1';
    root.dataset.buildings = String(world.buildings.length);
    root.dataset.seed = String(world.seed);
    session.loop();
    must<HTMLButtonElement>(root, '#regen').addEventListener('click', () => {
      physics.free();
      world = generateOutdoor({ seed: readSeed(seedInput) });
      physics = new OutdoorPhysics(world);
      view.setWorld(world);
      session.reset(world, physics);
    });
    window.addEventListener('resize', () => view.resize());
  } catch (error) {
    boot.textContent = error instanceof Error ? error.message : String(error);
    console.error(error);
  }
}

class Session {
  private readonly keys = new Set<string>();
  private readonly dyn = new KinematicDynamics();
  private readonly cmd = new Float32Array(4);
  private state;
  private camera: OutdoorCamera;
  private last = performance.now();
  private frames = 0;
  private fpsSamples = 0;
  private fpsStamp = performance.now();

  constructor(
    private readonly root: HTMLElement,
    private readonly view: OutdoorView,
    private readonly map: HTMLCanvasElement,
    private world: OutdoorWorld,
    private physics: OutdoorPhysics,
    camera: OutdoorCamera,
  ) {
    this.camera = camera;
    this.state = makeDroneState(world.spawn, world.spawnYaw);
    this.dyn.reset(this.state, DYN);
    this.bind();
    this.syncCameraButtons();
  }

  reset(world: OutdoorWorld, physics: OutdoorPhysics): void {
    this.world = world;
    this.physics = physics;
    this.state = makeDroneState(world.spawn, world.spawnYaw);
    this.dyn.reset(this.state, DYN);
    this.root.dataset.buildings = String(world.buildings.length);
    this.root.dataset.seed = String(world.seed);
  }

  /** Slide along the corridor so a still frame shows a shell, not only the spawn clearing. */
  advanceForShot(shot: string | null): void {
    if (!shot) return;
    const yaw = this.state.yaw;
    const step: [number, number, number] = [Math.cos(yaw) * 0.35, Math.sin(yaw) * 0.35, 0];
    for (let i = 0; i < 18; i++) {
      const moved = this.physics.move(this.state.pos, step);
      this.state.pos[0] = moved.pos[0];
      this.state.pos[1] = moved.pos[1];
      this.state.pos[2] = moved.pos[2];
    }
  }

  loop = (): void => {
    const now = performance.now();
    const dt = Math.max(0.001, Math.min(0.05, (now - this.last) / 1000));
    this.last = now;
    const action = actionFromKeys(this.keys);
    scaleAction(action, this.cmd);
    const sub = 2;
    for (let i = 0; i < sub; i++) {
      const x0 = this.state.pos[0];
      const y0 = this.state.pos[1];
      const z0 = this.state.pos[2];
      this.dyn.step(this.cmd, dt / sub);
      const delta: [number, number, number] = [this.state.pos[0] - x0, this.state.pos[1] - y0, this.state.pos[2] - z0];
      this.state.pos[0] = x0;
      this.state.pos[1] = y0;
      this.state.pos[2] = z0;
      const moved = this.physics.move(this.state.pos, delta);
      this.state.pos[0] = moved.pos[0];
      this.state.pos[1] = moved.pos[1];
      this.state.pos[2] = moved.pos[2];
    }
    const spin = 0.4 + Math.hypot(this.state.vel[0], this.state.vel[1], this.state.vel[2]);
    this.view.update(this.state.pos, this.state.yaw, spin);
    const mapCtx = this.map.getContext('2d');
    if (mapCtx) {
      drawOutdoorMap(mapCtx, this.world, { x: this.state.pos[0], y: this.state.pos[1], yaw: this.state.yaw }, this.map.clientWidth, this.map.clientHeight);
    }
    this.paintHud();
    this.frames += 1;
    if (now - this.fpsStamp > 500) {
      const fps = (this.frames * 1000) / (now - this.fpsStamp);
      this.frames = 0;
      this.fpsStamp = now;
      this.fpsSamples += 1;
      // The first window includes shader compile and the one-shot shadow pass.
      if (this.fpsSamples >= 2) {
        this.root.dataset.fps = fps.toFixed(1);
        this.root.dataset.fpsSamples = String(this.fpsSamples);
      }
    }
    requestAnimationFrame(this.loop);
  };

  private paintHud(): void {
    const speed = Math.hypot(this.state.vel[0], this.state.vel[1], this.state.vel[2]);
    const ground = sampleHeight(this.world.terrain, this.state.pos[0], this.state.pos[1]);
    const fenced = containsPoint(this.world.geofence, this.state.pos[0], this.state.pos[1]);
    const denied = inNoFly(this.world.noFly, this.state.pos[0], this.state.pos[1], this.state.pos[2]);
    const ranges = this.physics.lidar(this.state.pos, this.state.yaw);
    let near = 1;
    for (const value of ranges) if (value < near) near = value;
    must<HTMLElement>(this.root, '#hud-speed').textContent = `${speed.toFixed(1)} m/s`;
    must<HTMLElement>(this.root, '#hud-alt').textContent = `${(this.state.pos[2] - ground).toFixed(1)} m`;
    must<HTMLElement>(this.root, '#hud-fence').textContent = denied ? 'no-fly' : fenced ? 'inside' : 'geofence';
    must<HTMLElement>(this.root, '#hud-lidar').textContent = `${(near * 16).toFixed(1)} m`;
    must<HTMLElement>(this.root, '#hud-open').textContent = nearestOpening(this.world, this.state.pos);
  }

  private bind(): void {
    window.addEventListener('keydown', (event) => {
      this.keys.add(event.code);
      if (event.code === 'Digit1') this.setCamera('fpv');
      if (event.code === 'Digit2') this.setCamera('chase');
    });
    window.addEventListener('keyup', (event) => this.keys.delete(event.code));
    must<HTMLButtonElement>(this.root, '#cam-fpv').addEventListener('click', () => this.setCamera('fpv'));
    must<HTMLButtonElement>(this.root, '#cam-chase').addEventListener('click', () => this.setCamera('chase'));
  }

  private setCamera(mode: OutdoorCamera): void {
    this.camera = mode;
    this.view.setCamera(mode);
    this.syncCameraButtons();
  }

  private syncCameraButtons(): void {
    must<HTMLButtonElement>(this.root, '#cam-fpv').dataset.on = this.camera === 'fpv' ? '1' : '0';
    must<HTMLButtonElement>(this.root, '#cam-chase').dataset.on = this.camera === 'chase' ? '1' : '0';
  }
}

function nearestOpening(world: OutdoorWorld, pos: [number, number, number]): string {
  let best = '';
  let bestDist = Infinity;
  for (const shell of world.buildings) {
    for (const opening of shell.openings) {
      const dx = opening.center[0] - pos[0];
      const dy = opening.center[1] - pos[1];
      const dz = opening.center[2] - pos[2];
      const dist = Math.hypot(dx, dy, dz);
      if (dist < bestDist) {
        bestDist = dist;
        best = `${opening.passable ? 'enter' : 'decoy'} ${opening.kind.replaceAll('_', ' ')} ${dist.toFixed(1)}m`;
      }
    }
  }
  return best;
}

function readSeed(input: HTMLInputElement): number {
  const value = Number(input.value);
  return Number.isFinite(value) ? value >>> 0 : 1;
}

function markup(): string {
  return `
    <header class="topbar">
      <div class="brand"><strong>frontier-fly</strong><span>outdoor world</span></div>
      <a href="./index.html">indoor sim</a>
      <div class="controls">
        <label>seed <input id="seed" inputmode="numeric" /></label>
        <button id="regen" type="button">new world</button>
      </div>
    </header>
    <div class="stage">
      <canvas id="view"></canvas>
      <canvas id="map" width="240" height="240"></canvas>
      <div class="cam">
        <button id="cam-fpv" type="button">1 FPV</button>
        <button id="cam-chase" type="button">2 chase</button>
      </div>
      <div class="hud">
        <span>speed <strong id="hud-speed">0</strong></span>
        <span>agl <strong id="hud-alt">0</strong></span>
        <span>fence <strong id="hud-fence">inside</strong></span>
        <span>lidar <strong id="hud-lidar">—</strong></span>
        <span id="hud-open"></span>
        <span>WASD fly · QE yaw · Space/C up/down</span>
      </div>
      <div class="boot" id="boot">generating terrain…</div>
    </div>
  `;
}

function must<T extends Element>(root: ParentNode, selector: string): T {
  const node = root.querySelector(selector);
  if (!node) throw new Error(`missing ${selector}`);
  return node as T;
}

const root = document.querySelector('#app');
if (root instanceof HTMLElement) void startOutdoor(root);
