import './styles.css';
import { generateBuilding } from '../gen/building';
import { parsePlanSet, PLAN_SET_URL, PLAN_STAGE, planBuilding, type PlanRecord, type PlanSet } from '../plans';
import { OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { MetricsPayload } from '../shared/messages';
import type { BuildingSpec, CameraMode, DroneState, LightingState, QualityPreset, SensorMode } from '../shared/types';
import { actorOnly, checkpointMismatch, parseCheckpoint, SHIPPED_POLICY_URL, saveCheckpointIdb, loadCheckpointIdb, type Checkpoint } from '../rl/checkpoint';
import type { InferPolicy } from '../rl/infer';
import type { RunningNorm } from '../rl/normalize';
import { makeDroneState } from '../sim/drone';
import { createEnv, type ExplorerEnv } from '../sim/env';
import { heuristicAction } from '../sim/heuristic';
import { actionFromKeys, zeroAction } from '../sim/manual';
import { cameraLookingAt, clearLabel, detect, drawDetections, fpvCameraFromDrone, indoorCells, roomCellKeys, SeenEmptyTracker, type Detection } from '../perception';
import { frameOnTargets, placeIndoor, resolveScene, type TargetScene } from '../targets';
import { TargetView } from '../targets/meshes';
import { drawHud } from '../viz/cameras/hud';
import { ThreeView } from '../viz/renderer';
import type { ShowcaseSnap } from '../workers/rollout';
import { TrainPool } from '../workers/pool';
import { RewardChart } from './charts';
import { checkpointFrom, downloadCheckpoint, policyFromCheckpoint } from './checkpointIo';
import { readLaunchParams, writeLaunchParams, type LaunchParams } from './controls';
import { drawCoverage } from './coverageMap';
import { renderDetections } from './detectionPanel';
import { sessionEpisodeConfig, STAGE_SIZE } from './episode';
import { drawHeatmap, type HeatmapData } from './heatmap';
import { battery, copyLight, formatClock, modeLabel } from './readouts';
import { ReplayBuffer, replayFrame, type StoredEpisode } from './replay';
import { fitCanvas, must, shell } from './shell';
import { showStartError } from './startError';

/** True when this page can share memory with its rollout workers (cross-origin isolated). */
function canTrainHere(): boolean {
  return typeof SharedArrayBuffer !== 'undefined' && globalThis.crossOriginIsolated !== false;
}

export async function startApp(root: HTMLElement = document.querySelector('#app')!): Promise<void> {
  const launch = readLaunchParams();
  root.innerHTML = shell();
  const viewCanvas = must<HTMLCanvasElement>(root, '#view');
  const pipCanvas = must<HTMLCanvasElement>(root, '#pip');
  const hud = must<HTMLCanvasElement>(root, '#hud');
  const pipHud = must<HTMLCanvasElement>(root, '#pip-hud');
  const map = must<HTMLCanvasElement>(root, '#map');
  const heat = must<HTMLCanvasElement>(root, '#heat');
  const view = new ThreeView(viewCanvas, pipCanvas);
  const bootNote = must<HTMLElement>(root, '#boot-note');
  try {
    await view.init(launch.quality, launch.renderer);
    const env = await createEnv();
    const session = new Session(root, view, env, launch);
    await session.boot();
    bootNote.remove();
    session.loop();
    // `?train=1` starts PPO on load (the landing page's "Train it in your tab" link) and points at the button.
    if (launch.train) {
      const train = must<HTMLButtonElement>(root, '#train-btn');
      train.classList.add('attention');
      train.click();
    }
  } catch (error) {
    showStartError(bootNote, error);
    console.error(error);
  }
  void hud;
  void pipHud;
  void map;
  void heat;
}

class Session {
  private spec: BuildingSpec | null = null;
  private plans: PlanSet | null = null;
  private obs = new Float32Array(OBS_DIM);
  private shown: DroneState = makeDroneState([1, 1, 1]);
  /** Lidar rays in the 3D view: off at start so the drone and the room read first. */
  private showRays = false;
  private lighting: LightingState = { timeOfDay: 12, interiorLightsOn: new Uint8Array(), flashlightOn: false, sensorMode: 'rgb' };
  private policy: InferPolicy | null = null;
  private norm: RunningNorm | null = null;
  /** True when the checkpoint was trained on raw observations (identity stats; no ±5 clip). */
  private rawObs = false;
  /** The loaded checkpoint had no value head; saves and downloads leave it out too. */
  private actorOnly = false;
  private pool: TrainPool | null = null;
  private training = false;
  private paused = false;
  private pilot: LaunchParams['pilot'];
  private camera: CameraMode;
  private readonly keys = new Set<string>();
  private trail: Array<[number, number, number]> = [];
  private lidar: number[] | null = null;
  private coverage = 0;
  private collisions = 0;
  private reward = 0;
  private visited: Uint8Array | null = null;
  private floor = 0;
  private readonly replay = new ReplayBuffer();
  private playing: StoredEpisode | null = null;
  private playIndex = 0;
  private playSpeed = 1;
  private playOn = false;
  private readonly charts: RewardChart[] = [];
  private lastStep = 0;
  private lastMap = 0;
  private steps = 0;
  private best = 0;
  private lightingLock = false;
  private heatmap: HeatmapData | null = null;
  private heatmapArmed = false;
  private readonly actRng = new RNG(7, 'ui');
  private readonly targetView = new TargetView();
  private targetScene: TargetScene | null = null;
  private detections: Detection[] = [];
  private tracker: SeenEmptyTracker | null = null;
  private clearText = '';
  private framed: { eye: [number, number, number]; look: [number, number, number]; fov: number } | null = null;

  constructor(
    private readonly root: HTMLElement,
    private readonly view: ThreeView,
    private readonly env: ExplorerEnv,
    private launch: LaunchParams,
  ) {
    this.pilot = launch.pilot;
    this.camera = launch.camera;
  }

  async boot(): Promise<void> {
    this.bind();
    this.view.addSceneObject(this.targetView.group);
    this.view.setCamera(this.camera);
    this.syncButtons();
    await this.loadShipped();
    if (this.launch.plan) await this.loadPlans();
    this.rebuild(this.launch.seed, false);
    this.resize();
    const chartHost = [
      [this.q('#chart-return'), '#49f0a0'],
      [this.q('#chart-cover'), '#3ee0c5'],
      [this.q('#chart-entropy'), '#8ec5ff'],
    ] as const;
    for (const [el, color] of chartHost) {
      try {
        this.charts.push(new RewardChart(el, color));
      } catch (error) {
        console.warn('chart init failed', error);
      }
    }
    window.addEventListener('resize', () => this.resize());
  }

  loop(): void {
    const tick = (now: number) => {
      if (!this.paused && !this.playing && now - this.lastStep > 50) {
        this.lastStep = now;
        if (this.training) {
          /* showcase messages advance the drone */
        } else {
          this.stepLocal();
        }
      }
      if (this.playing && this.playOn) {
        this.playIndex += this.playSpeed * 0.35;
        const ep = this.playing;
        if (this.playIndex >= ep.states.length - 1) {
          this.playIndex = ep.states.length - 1;
          this.playOn = false;
        }
        this.shown = replayFrame(ep, this.playIndex, 0, 0);
        const slider = this.q<HTMLInputElement>('#scrub');
        slider.value = String(Math.floor(this.playIndex));
      }
      this.paint();
      if (now - this.lastMap > 120) {
        this.lastMap = now;
        this.paintMap();
      }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }

  private rebuild(seed: number, pushUrl: boolean): void {
    let stage = this.stageNumber();
    let size = STAGE_SIZE[stage] ?? STAGE_SIZE[3]!;
    const showcase = this.launch.shot?.startsWith('targets') ?? false;
    if (showcase) {
      stage = 0;
      size = { floors: 1, w: 14, d: 12 };
    }
    const plan = showcase ? null : this.launchPlan();
    const real = plan ? planBuilding(plan) : null;
    if (real) stage = PLAN_STAGE;
    this.spec = real ? real.spec : generateBuilding({ seed, floors: size.floors, w: size.w, d: size.d, stage });
    this.root.dataset.plan = real ? plan!.id : '';
    const nRooms = this.spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
    const lights = new Uint8Array(nRooms);
    if (this.launch.lights) lights.fill(1);
    const cfg = sessionEpisodeConfig(stage, nRooms, {
      timeOfDay: this.launch.tod,
      interiorLightsOn: lights,
      flashlightOn: this.launch.flashlight,
      sensorMode: this.launch.sensor,
    });
    this.obs = this.env.reset(this.spec, seed, cfg);
    this.lighting = copyLight(this.env.episodeLighting);
    this.shown = this.env.snapshot();
    this.trail = [];
    this.visited = new Uint8Array(this.env.visitedGrid);
    this.coverage = 0;
    this.reward = 0;
    this.collisions = 0;
    this.lidar = Array.from(this.obs.slice(0, 40));
    this.targetScene = placeIndoor(this.spec, showcase ? { seed, showcase: true } : { seed, people: 4, objects: 6, decoys: 3 });
    this.tracker = new SeenEmptyTracker(indoorCells(this.spec), 'person', this.spec);
    if (showcase) {
      const distance = this.launch.shot === 'targets-room' ? 6.4 : 3.8;
      const fov = this.launch.shot === 'targets-room' ? 64 : 110;
      const frame = frameOnTargets(this.spec, resolveScene(this.targetScene.targets, 0), distance);
      this.framed = { eye: frame.eye, look: frame.look, fov };
      this.view.setFramedView(frame.eye, frame.look, fov, this.launch.shot === 'targets-night');
    } else {
      this.framed = null;
      this.view.setFramedView(null, null);
    }
    this.view.load(this.spec);
    this.q<HTMLInputElement>('#seed').value = String(seed);
    this.q<HTMLInputElement>('#tod').value = String(this.lighting.timeOfDay);
    this.launch.seed = seed;
    if (pushUrl) writeLaunchParams(this.launch);
    this.root.dataset.ready = '0';
    this.root.dataset.seed = String(seed);
    this.root.dataset.shot = this.launch.shot ?? '';
  }

  private stepLocal(): void {
    let action = zeroAction();
    if (this.keys.size > 0) action = actionFromKeys(this.keys);
    else if (this.pilot === 'policy' && this.policy && this.norm) {
      // Checkpoints trained on raw observations (identity stats) fly on raw input, like the eval.
      const normed = this.rawObs ? this.obs : this.norm!.apply(this.obs, 1);
      action = this.policy.actWith(normed, 1, this.actRng, true).action;
    } else if (this.pilot === 'heuristic' || this.pilot === 'policy' || this.launch.shot) {
      // 'policy' without a usable checkpoint (none yet, or refused) flies the lidar heuristic.
      action = heuristicAction(this.obs);
    }
    const step = this.env.step(action);
    this.obs = step.obs;
    this.shown = this.env.snapshot();
    this.coverage = step.info.coverage;
    this.collisions = step.info.collisions;
    this.floor = step.info.floor;
    this.reward += step.reward;
    this.lidar = Array.from(step.obs.slice(0, 40));
    this.visited = new Uint8Array(this.env.visitedGrid);
    this.pushTrail(this.shown);
    this.replay.note(this.shown, this.reward, step.info.coverage > 0 ? this.env.stepCount : 1);
    this.steps += 1;
    if (step.done) {
      this.replay.close();
      this.best = Math.max(this.best, this.reward);
      this.reward = 0;
      const spec = this.spec!;
      const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
      const cfg = sessionEpisodeConfig(this.stageNumber(), nRooms, copyLight(this.lighting));
      this.obs = this.env.reset(spec, (this.launch.seed + this.steps) >>> 0, cfg);
      this.lighting = copyLight(this.env.episodeLighting);
    }
  }

  private paint(): void {
    if (!this.spec) return;
    this.view.setLidar(this.showRays ? this.lidar : null);
    this.view.setTrail(this.trail.slice(-80));
    this.view.setCollision(this.collisions > 0 && this.env.stepCount % 8 < 2);
    if (this.visited) this.view.setVisited(this.spec, this.visited, this.floor);
    this.view.update(this.shown, this.lighting);
    this.senseTargets();
    const hooks = this.view.hooks();
    const overlay = this.camera === 'fpv' || this.launch.shot === 'fpv' || this.framed !== null;
    this.drawHudCanvas(this.q('#hud'), this.camera === 'fpv' || this.launch.shot === 'fpv', overlay);
    this.drawHudCanvas(this.q('#pip-hud'), true, this.framed !== null);
    const text = this.q('#readout');
    const speed = Math.hypot(this.shown.vel[0], this.shown.vel[1], this.shown.vel[2]);
    text.textContent = `SIM ${this.shown.t.toFixed(1)}s\nALT ${this.shown.pos[2].toFixed(2)} m\nSPD ${speed.toFixed(2)} m/s\nX ${this.shown.pos[0].toFixed(1)}  Y ${this.shown.pos[1].toFixed(1)}`;
    this.q('#clock').textContent = formatClock(this.lighting.timeOfDay);
    this.q('#fps').textContent = `${hooks.fps.toFixed(0)} fps`;
    this.q('#backend').textContent = hooks.backend.replace(/^webgl/iu, 'WebGL').replace(/^webgpu/iu, 'WebGPU');
    this.q('#cov-num').textContent = `${Math.round(this.coverage * 100)}%`;
    this.q('#alt-num').textContent = `${this.shown.pos[2].toFixed(2)} m`;
    this.q('#spd-num').textContent = `${speed.toFixed(2)} m/s`;
    this.q('#bat-num').textContent = `${battery(this.shown.t)}%`;
    const mode = modeLabel(this.lighting);
    this.q('#mode-label').textContent = mode.charAt(0) + mode.slice(1).toLowerCase();
    this.root.dataset.fps = hooks.fps.toFixed(1);
    this.root.dataset.backend = hooks.backend;
    const visualsReady = !this.launch.shot?.startsWith('targets') || this.targetView.loaded;
    if (this.view.frameCount > 2 && visualsReady) this.root.dataset.ready = '1';
  }

  private paintMap(): void {
    if (!this.spec) return;
    const map = this.q<HTMLCanvasElement>('#map');
    fitCanvas(map);
    drawCoverage(map, this.spec, this.visited, this.shown, this.floor);
    // The policy map stays hidden until training sends its first heatmap.
    this.q('#policy-panel').hidden = !this.heatmap;
    if (!this.heatmap) return;
    const heat = this.q<HTMLCanvasElement>('#heat');
    fitCanvas(heat);
    drawHeatmap(heat, this.heatmap);
  }

  private senseTargets(): void {
    if (!this.spec || !this.targetScene || !this.tracker) return;
    const time = this.shown.t;
    const resolved = resolveScene(this.targetScene.targets, time);
    this.targetView.sync(resolved, this.targetScene.targets, time);
    const aspect = this.view.sensingAspect();
    const camera = this.framed
      ? cameraLookingAt(this.framed.eye, this.framed.look, [0, 0, 1], (this.framed.fov * Math.PI) / 180, aspect)
      : fpvCameraFromDrone(this.shown.pos, this.shown.yaw, aspect);
    const speed = Math.hypot(this.shown.vel[0], this.shown.vel[1]);
    this.detections = detect({
      seed: this.launch.seed,
      camera,
      targets: resolved,
      spec: this.spec,
      lighting: this.lighting,
      cameraSpeed: this.framed ? 0 : speed,
    });
    this.tracker.observe(camera, resolved, this.lighting, resolved.flatMap((target) => target.occluders));
    const keys = roomCellKeys(this.spec, this.floor, 0);
    const report = this.tracker.report(keys.length ? keys : indoorCells(this.spec).map((cell) => cell.key), resolved);
    this.clearText = clearLabel(report);
    this.root.dataset.detections = String(this.detections.length);
    this.root.dataset.clear = report.status;
    renderDetections(this.root, this.detections, this.clearText);
  }

  private drawHudCanvas(canvas: HTMLCanvasElement, enabled: boolean, overlay = false): void {
    fitCanvas(canvas);
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    if (!enabled) ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (enabled) {
    const speed = Math.hypot(this.shown.vel[0], this.shown.vel[1], this.shown.vel[2]);
    drawHud(ctx, canvas.width, canvas.height, {
      alt: this.shown.pos[2],
      speed,
      yaw: this.shown.yaw,
      battery: battery(this.shown.t) / 100,
      coverage: this.coverage,
      mode: modeLabel(this.lighting),
      collision: this.view.collisionFlash,
      lidar: this.lidar,
      fps: this.view.hooks().fps,
    });
    }
    if (overlay) drawDetections(ctx, canvas.width, canvas.height, this.detections);
  }

  /** The first-visit caption over the view goes once the visitor does anything. */
  private dismissCaption(): void {
    this.root.querySelector('#stage-caption')?.setAttribute('hidden', '');
  }

  private bind(): void {
    window.addEventListener('keydown', () => this.dismissCaption(), { once: true });
    this.root.addEventListener('click', (event) => {
      const target = (event.target as HTMLElement).closest<HTMLElement>('[data-act]');
      if (!target) return;
      this.dismissCaption();
      const act = target.dataset.act;
      if (act === 'cam') this.setCamera((target.dataset.cam as CameraMode) ?? 'chase');
      else if (act === 'tab') this.setCamera(target.dataset.cam === 'overview' ? 'overview' : target.dataset.cam === 'fpv' ? 'fpv' : 'chase');
      else if (act === 'day') this.setTod(this.lighting.timeOfDay >= 18 ? 13 : 22.5);
      else if (act === 'lights') this.setLights(!this.lightsOn());
      else if (act === 'flash') this.setFlash(!this.lighting.flashlightOn);
      else if (act === 'sensor') this.setSensor((target.dataset.sensor as SensorMode) ?? 'rgb');
      else if (act === 'pilot') this.setPilot(target.dataset.pilot as LaunchParams['pilot']);
      else if (act === 'pause') this.togglePause();
      else if (act === 'train') void this.toggleTrain();
      else if (act === 'seed') this.rebuild(Number(this.q<HTMLInputElement>('#seed').value) || 1, true);
      else if (act === 'rays') {
        this.showRays = !this.showRays;
        target.setAttribute('aria-pressed', String(this.showRays));
        this.q('#rays-label').textContent = this.showRays ? 'Rays on' : 'Show rays';
      } else if (act === 'quality') this.setQuality(target.dataset.quality === 'low' ? 'low' : 'high');
      else if (act === 'save') this.downloadCheckpoint();
      else if (act === 'idb-save') void this.idbSave();
      else if (act === 'idb-load') void this.idbLoad();
      else if (act === 'replay') this.startReplay(target.dataset.which as 'latest' | 'best' | 'worst');
      else if (act === 'speed') {
        this.playSpeed = Number(target.dataset.speed) || 1;
        this.playOn = true;
      }
    });
    this.q<HTMLInputElement>('#tod').addEventListener('input', () => {
      this.setTod(Number(this.q<HTMLInputElement>('#tod').value));
    });
    this.q<HTMLSelectElement>('#stage').addEventListener('change', () => {
      const raw = this.q<HTMLSelectElement>('#stage').value;
      this.launch.stage = raw === 'auto' ? 'auto' : Number(raw);
      this.pool?.setStage(this.launch.stage === 'auto' ? null : this.launch.stage);
      this.rebuild(this.launch.seed, true);
    });
    this.q<HTMLInputElement>('#scrub').addEventListener('input', () => {
      if (!this.playing) return;
      this.playOn = false;
      this.playIndex = Number(this.q<HTMLInputElement>('#scrub').value);
      this.shown = replayFrame(this.playing, this.playIndex, 0, 0);
    });
    this.q<HTMLInputElement>('#file').addEventListener('change', () => {
      const file = this.q<HTMLInputElement>('#file').files?.[0];
      if (!file) return;
      void file.arrayBuffer().then((bytes) => this.applyCheckpoint(parseCheckpoint(bytes)));
    });
    window.addEventListener('keydown', (event) => {
      if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement) return;
      this.keys.add(event.code);
      if (event.code === 'Digit1') this.setCamera('fpv');
      if (event.code === 'Digit2') this.setCamera('chase');
      if (event.code === 'Digit3') this.setCamera('overview');
      if (event.code === 'KeyP') this.togglePause();
      if (event.code === 'KeyL') this.setLights(!this.lightsOn());
      if (event.code === 'KeyF') this.setFlash(!this.lighting.flashlightOn);
      if (event.code === 'KeyT') void this.toggleTrain();
    });
    window.addEventListener('keyup', (event) => this.keys.delete(event.code));
    const stage = this.q('#stage-wrap');
    stage.addEventListener('pointerdown', (event) => {
      if (this.camera !== 'overview') return;
      const startX = event.clientX;
      const startY = event.clientY;
      const move = (ev: PointerEvent) => this.view.orbit(ev.clientX - startX, ev.clientY - startY);
      const up = () => {
        window.removeEventListener('pointermove', move);
        window.removeEventListener('pointerup', up);
      };
      window.addEventListener('pointermove', move);
      window.addEventListener('pointerup', up);
    });
    stage.addEventListener('wheel', (event) => {
      if (this.camera === 'overview') this.view.zoom(event.deltaY * 0.01);
    }, { passive: true });
  }

  private async toggleTrain(): Promise<void> {
    if (this.training) {
      this.pool?.pause();
      this.training = false;
      this.syncButtons();
      return;
    }
    if (!canTrainHere()) {
      // Rollout workers share SharedArrayBuffers, which need a cross-origin isolated page. On GitHub Pages that
      // isolation comes from coi-serviceworker, so a browser that blocks service workers cannot train.
      const note = this.q('#train-note');
      note.textContent =
        'Training needs SharedArrayBuffer, which this browser has not enabled for the page. That usually means service workers are blocked, as in some private windows. Open the page in a normal window and reload once. Flying and replays still work.';
      note.hidden = false;
      return;
    }
    this.playing = null;
    this.playOn = false;
    if (!this.pool) {
      this.pool = new TrainPool({
        onReady: () => {
          this.q('#link').textContent = 'Training';
        },
        onShowcase: (snap) => this.onShowcase(snap),
        onMetrics: (metrics) => this.onMetrics(metrics),
        onHeatmap: (msg) => {
          this.heatmap = { w: msg.w, d: msg.d, values: msg.values, arrows: msg.arrows };
        },
      });
      this.pool.start(this.launch.seed);
    } else {
      this.pool.resume();
    }
    this.training = true;
    if (this.launch.stage !== 'auto') this.pool.setStage(this.launch.stage);
    this.syncButtons();
    if (!this.heatmapArmed) {
      this.heatmapArmed = true;
      window.setInterval(() => {
        if (this.training) this.pool?.requestHeatmap(this.floor);
      }, 4000);
    }
  }

  private onShowcase(snap: ShowcaseSnap): void {
    if (this.playing) return;
    if (!this.spec || this.spec.seed !== snap.spec.seed) {
      this.spec = snap.spec;
      this.view.load(snap.spec);
      this.trail = [];
      this.targetScene = placeIndoor(this.spec, { seed: snap.spec.seed, people: 4, objects: 6, decoys: 3 });
      this.tracker = new SeenEmptyTracker(indoorCells(this.spec), 'person', this.spec);
    }
    this.shown = {
      pos: [snap.pos[0], snap.pos[1], snap.pos[2]],
      vel: [snap.vel[0], snap.vel[1], snap.vel[2]],
      yaw: snap.yaw,
      t: snap.step / 20,
      quat: [snap.quat[0], snap.quat[1], snap.quat[2], snap.quat[3]],
      motors: Float32Array.from(snap.motors.length ? snap.motors : [0, 0, 0, 0]),
      roll: snap.roll,
      pitch: snap.pitch,
    };
    this.coverage = snap.coverage;
    this.collisions = snap.collisions;
    this.floor = snap.floor;
    this.reward = snap.reward;
    this.lidar = snap.lidar;
    this.visited = snap.visited;
    if (!this.lightingLock) this.lighting = copyLight(snap.lighting);
    this.pushTrail(this.shown);
    this.replay.note(this.shown, snap.reward, snap.step);
    if (snap.step <= 2) this.replay.close();
  }

  private onMetrics(metrics: MetricsPayload): void {
    this.charts[0]?.push(metrics.update, metrics.episodeReturn);
    this.charts[1]?.push(metrics.update, metrics.coverage * 100);
    this.charts[2]?.push(metrics.update, metrics.entropy);
    this.q('#ep-num').textContent = String(metrics.update);
    this.q('#ret-num').textContent = metrics.episodeReturn.toFixed(1);
    this.q('#best-num').textContent = metrics.bestReward.toFixed(1);
    this.q('#rate-num').textContent = `${Math.round(metrics.stepsPerSec)}`;
    this.q('#stage-num').textContent = String(metrics.stage);
    this.q('#kl-num').textContent = metrics.kl.toFixed(3);
    this.best = metrics.bestReward;
    this.resize();
  }

  /** Real apartments for `?plan=`. A failed fetch falls back to generated buildings. */
  private async loadPlans(): Promise<void> {
    try {
      const response = await fetch(`${import.meta.env.BASE_URL}${PLAN_SET_URL}`);
      if (response.ok) this.plans = parsePlanSet(await response.json());
    } catch {
      this.plans = null;
    }
  }

  /** The plan named by `?plan=`: an id, or an index into the eval split (wrapping). */
  private launchPlan(): PlanRecord | null {
    const key = this.launch.plan;
    if (!this.plans || !key) return null;
    const byId = this.plans.plans.find((p) => p.id === key);
    if (byId) return byId;
    const evalPlans = this.plans.plans.filter((p) => p.split === 'eval');
    const index = Number(key);
    if (!Number.isInteger(index) || evalPlans.length === 0) return null;
    return evalPlans[((index % evalPlans.length) + evalPlans.length) % evalPlans.length]!;
  }

  private async loadShipped(): Promise<void> {
    try {
      const response = await fetch(SHIPPED_POLICY_URL);
      if (!response.ok) return;
      this.applyCheckpoint(parseCheckpoint(await response.arrayBuffer()), false);
    } catch {
      /* flying still works without a checkpoint */
    }
  }

  private applyCheckpoint(checkpoint: Checkpoint, announce = true): void {
    const mismatch = checkpointMismatch(checkpoint);
    if (mismatch) {
      // Refuse instead of feeding the network a different observation layout.
      console.warn(`frontier-fly: ${mismatch}; flying the heuristic pilot instead`);
      const link = this.q('#link');
      link.textContent = `Checkpoint obs v${checkpoint.obsVersion} refused`;
      link.title = `${mismatch}. Flying the heuristic pilot instead.`;
      return;
    }
    const { policy, norm, rawObs } = policyFromCheckpoint(checkpoint);
    this.policy = policy;
    this.norm = norm;
    this.rawObs = rawObs;
    this.actorOnly = checkpoint.actorOnly === true;
    if (this.pilot !== 'keys') this.pilot = 'policy';
    if (announce) this.q('#link').textContent = `Checkpoint stage ${checkpoint.stage}`;
  }

  private downloadCheckpoint(): void {
    const checkpoint = this.currentCheckpoint();
    if (checkpoint) downloadCheckpoint(checkpoint);
  }

  private currentCheckpoint(): Checkpoint | null {
    if (!this.policy || !this.norm) return null;
    const checkpoint = checkpointFrom(this.policy, this.norm, this.stageNumber());
    return this.actorOnly ? actorOnly(checkpoint) : checkpoint;
  }

  private async idbSave(): Promise<void> {
    const checkpoint = this.currentCheckpoint();
    if (!checkpoint) return;
    await saveCheckpointIdb(checkpoint);
    this.q('#link').textContent = 'Saved';
  }

  private async idbLoad(): Promise<void> {
    const checkpoint = await loadCheckpointIdb();
    if (checkpoint) this.applyCheckpoint(checkpoint);
  }

  private setCamera(mode: CameraMode): void {
    this.camera = mode;
    this.launch.camera = mode;
    this.view.setCamera(mode);
    writeLaunchParams(this.launch);
    this.syncButtons();
  }

  private setTod(hour: number): void {
    this.lightingLock = true;
    this.lighting.timeOfDay = Math.max(0, Math.min(24, hour));
    this.launch.tod = this.lighting.timeOfDay;
    this.q<HTMLInputElement>('#tod').value = String(this.lighting.timeOfDay);
    this.env.setLighting({ timeOfDay: this.lighting.timeOfDay });
    this.pool?.setLighting({ timeOfDay: this.lighting.timeOfDay });
    const night = this.lighting.timeOfDay < 6 || this.lighting.timeOfDay >= 18;
    if (night && !this.lighting.flashlightOn) this.setFlash(true);
  }

  private setLights(on: boolean): void {
    this.lightingLock = true;
    this.launch.lights = on;
    this.lighting.interiorLightsOn.fill(on ? 1 : 0);
    this.env.setLighting({ lightsOn: on });
    this.pool?.setLighting({ lightsOn: on });
    this.syncButtons();
  }

  private setFlash(on: boolean): void {
    this.lightingLock = true;
    this.lighting.flashlightOn = on;
    this.launch.flashlight = on;
    this.env.setLighting({ flashlightOn: on });
    this.pool?.setLighting({ flashlightOn: on });
    this.syncButtons();
  }

  private setSensor(mode: SensorMode): void {
    this.lightingLock = true;
    this.lighting.sensorMode = mode;
    this.launch.sensor = mode;
    this.env.setLighting({ sensorMode: mode });
    this.pool?.setLighting({ sensorMode: mode });
    this.syncButtons();
  }

  private setPilot(pilot: LaunchParams['pilot']): void {
    this.pilot = pilot;
    this.launch.pilot = pilot;
    this.syncButtons();
  }

  private setQuality(quality: QualityPreset): void {
    this.launch.quality = quality;
    void this.view.setQuality(quality);
    this.syncButtons();
  }

  private togglePause(): void {
    this.paused = !this.paused;
    if (this.training) {
      if (this.paused) this.pool?.pause();
      else this.pool?.resume();
    }
    this.syncButtons();
  }

  private startReplay(which: 'latest' | 'best' | 'worst'): void {
    const ep = this.replay.pick(which);
    if (!ep) return;
    this.playing = ep;
    this.playIndex = 0;
    this.playOn = true;
    const slider = this.q<HTMLInputElement>('#scrub');
    slider.max = String(ep.states.length - 1);
    slider.value = '0';
    this.q('#replay-label').textContent = `${which} · ${ep.reward.toFixed(1)}`;
  }

  private syncButtons(): void {
    this.root.querySelectorAll<HTMLElement>('[data-cam]').forEach((el) => {
      el.classList.toggle('active', el.dataset.cam === this.camera);
    });
    this.root.querySelectorAll<HTMLElement>('[data-act="pilot"]').forEach((el) => {
      el.classList.toggle('active', el.dataset.pilot === this.pilot);
    });
    this.root.querySelectorAll<HTMLElement>('[data-act="sensor"]').forEach((el) => {
      el.classList.toggle('active', el.dataset.sensor === this.lighting.sensorMode);
    });
    this.q('#train-btn').textContent = this.training ? 'Pause train' : 'Train';
    this.q('#pause-btn').textContent = this.paused ? 'Resume' : 'Pause';
    this.q('#light-label').textContent = this.lightsOn() ? 'On' : 'Off';
    this.q('#day-label').textContent = this.lighting.timeOfDay >= 6 && this.lighting.timeOfDay < 18 ? 'Day' : 'Night';
    this.q<HTMLSelectElement>('#stage').value = String(this.launch.stage);
  }

  private stageNumber(): number {
    if (this.launch.stage === 'auto') return this.launch.shot ? 3 : 3;
    return this.launch.stage;
  }

  private lightsOn(): boolean {
    return this.lighting.interiorLightsOn.length === 0 || this.lighting.interiorLightsOn.some((v) => v > 0);
  }

  private pushTrail(state: DroneState): void {
    const last = this.trail[this.trail.length - 1];
    if (last && Math.hypot(last[0] - state.pos[0], last[1] - state.pos[1]) < 0.08) return;
    this.trail.push([state.pos[0], state.pos[1], state.pos[2]]);
    if (this.trail.length > 240) this.trail.shift();
  }

  private resize(): void {
    this.view.resize();
    const width = this.q('#chart-return').clientWidth || 280;
    for (const chart of this.charts) chart.resize(Math.max(120, width));
  }

  private q<T extends Element = HTMLElement>(sel: string): T {
    return must<T>(this.root, sel);
  }
}
