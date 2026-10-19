import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { SMAAPass } from 'three/addons/postprocessing/SMAAPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { policyFromJson, type NormalizedPolicy } from '../mission/autonomy/policy';
import { DEFAULT_SCENARIO, isScenarioId, SCENARIOS, type ScenarioId } from '../mission/autonomy/scenarios';
import { buildTimeline, Director, FILM_FPS, type FilmFrame } from './director';
import { recordFlight, sampleFlight, type FlightLog } from './flight';
import { FilmOverlay } from './overlay';
import { FilmScene } from './scene';

/** Width and height the film is composed at. The stage scales to fit the window. */
const STAGE_W = 1920;
const STAGE_H = 1080;

const GradeShader = {
  uniforms: {
    tDiffuse: { value: null as THREE.Texture | null },
    vignette: { value: 0.32 },
    lift: { value: new THREE.Vector3(0.0, 0.004, 0.012) },
  },
  vertexShader: /* glsl */ `
    varying vec2 vUv;
    void main() { vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }
  `,
  fragmentShader: /* glsl */ `
    uniform sampler2D tDiffuse;
    uniform float vignette;
    uniform vec3 lift;
    varying vec2 vUv;
    void main() {
      vec4 c = texture2D(tDiffuse, vUv);
      vec2 d = vUv - 0.5;
      float v = 1.0 - vignette * smoothstep(0.18, 0.75, dot(d, d) * 1.6);
      vec3 col = c.rgb * v + lift * (1.0 - c.rgb);
      // Gentle S-curve for contrast.
      col = mix(col, col * col * (3.0 - 2.0 * col), 0.18);
      gl_FragColor = vec4(col, c.a);
    }
  `,
};

export interface FilmApi {
  ready: boolean;
  frames: number;
  fps: number;
  scenario: ScenarioId;
  /** Render film frame i (frames must be requested in order for the camera springs). */
  render(i: number): void;
  error?: string;
}

declare global {
  interface Window {
    __film?: FilmApi;
  }
}

async function shippedPolicy(): Promise<NormalizedPolicy | null> {
  try {
    const response = await fetch('checkpoints/pretrained.json');
    return response.ok ? policyFromJson(await response.text()) : null;
  } catch {
    return null;
  }
}

export type FilmStatus = 'flying' | 'building' | 'ready' | 'error';

export interface FilmOptions {
  /**
   * Embed the film in a container on another page: no controls bar, the stage
   * fits the container instead of the window, playback starts on its own and
   * loops, and URL parameters are ignored.
   */
  embed?: boolean;
  /** Scenario to fly when embedded (the film page reads `?scenario=`). */
  scenario?: ScenarioId;
  /** Render scale when embedded. Defaults to 0.75, or 0.5 on narrow containers. */
  renderScale?: number;
  /** Progress callback, so an embedding page can show its own loading state. */
  onStatus?: (status: FilmStatus, message: string) => void;
}

export async function startFilm(root: HTMLElement, options: FilmOptions = {}): Promise<void> {
  const embed = options.embed === true;
  const params = new URLSearchParams(embed ? '' : location.search);
  const requested = embed ? (options.scenario ?? null) : params.get('scenario');
  const scenario: ScenarioId = isScenarioId(requested) ? requested : DEFAULT_SCENARIO;
  const recording = !embed && params.get('record') === '1';
  // Recording: render at `scale` x 1920x1080 (the HUD scales with it) and
  // optionally fit the flight into `length` seconds of film.
  const renderScale = recording
    ? Math.min(1, Math.max(0.25, Number(params.get('scale') ?? 1) || 1))
    : embed
      ? Math.min(1, Math.max(0.25, options.renderScale ?? (root.clientWidth < 700 ? 0.5 : 0.75)))
      : 1;
  const flightSec = Number(params.get('length')) || undefined;
  const status = options.onStatus ?? (() => undefined);
  const api: FilmApi = { ready: false, frames: 0, fps: FILM_FPS, scenario, render: () => undefined };
  window.__film = api;

  root.classList.add('film-root');
  if (recording) root.classList.add('film-recording');
  if (embed) root.classList.add('film-embed');
  const stage = document.createElement('div');
  stage.className = 'film-stage';
  stage.style.width = `${STAGE_W}px`;
  stage.style.height = `${STAGE_H}px`;
  root.appendChild(stage);
  const canvas = document.createElement('canvas');
  canvas.className = 'film-canvas';
  stage.appendChild(canvas);
  const loading = document.createElement('div');
  loading.className = 'film-loading';
  loading.textContent = 'Flying the mission headless…';
  stage.appendChild(loading);
  const fit = () => {
    const s = recording ? renderScale : embed ? embedScale(root) : Math.min(window.innerWidth / STAGE_W, (window.innerHeight - 64) / STAGE_H);
    // An embedded stage is centred on the container (see .film-embed in film.css).
    stage.style.transform = embed ? `translate(-50%, -50%) scale(${s})` : `scale(${s})`;
    root.style.setProperty('--stage-scale', String(s));
  };
  fit();
  if (embed) new ResizeObserver(fit).observe(root);
  else window.addEventListener('resize', fit);
  status('flying', loading.textContent);

  try {
    const indoor = params.get('indoor') === 'frontier' ? null : await shippedPolicy();
    const log = await recordFlight(scenario, indoor);
    loading.textContent = 'Building the world…';
    status('building', loading.textContent);
    const scene = new FilmScene(canvas, log);
    scene.renderer.setPixelRatio(renderScale);
    scene.resize(STAGE_W, STAGE_H);
    await scene.load();
    const director = new Director(log, scene, scene.occluders);
    director.setAspect(STAGE_W / STAGE_H);
    const overlay = new FilmOverlay(stage, log);
    overlay.resize(STAGE_W, STAGE_H);
    const composer = new EffectComposer(scene.renderer);
    composer.setPixelRatio(renderScale);
    composer.setSize(STAGE_W, STAGE_H);
    composer.addPass(new RenderPass(scene.scene, director.camera));
    composer.addPass(new UnrealBloomPass(new THREE.Vector2(STAGE_W / 2, STAGE_H / 2), scene.night ? 0.55 : 0.22, 0.5, scene.night ? 0.62 : 0.88));
    composer.addPass(new OutputPass());
    composer.addPass(new ShaderPass(GradeShader));
    composer.addPass(new SMAAPass(STAGE_W * renderScale, STAGE_H * renderScale));
    const timeline = buildTimeline(log, flightSec);
    let cursor = -1;
    const show = (i: number) => {
      const target = Math.max(0, Math.min(timeline.length - 1, i));
      // The camera springs are stateful: replay any skipped frames without rendering them.
      if (target < cursor) cursor = -1;
      for (let k = cursor + 1; k < target; k++) director.frame(timeline[k]!);
      const f = timeline[target]!;
      director.frame(f);
      apply(scene, log, f);
      composer.render();
      overlay.update(f, director.camera);
      cursor = target;
    };
    api.frames = timeline.length;
    api.render = show;
    loading.remove();
    show(0);
    api.ready = true;
    root.dataset.ready = '1';
    status('ready', '');
    if (embed) autoplay(root, timeline, show);
    else if (!recording) controls(root, timeline, show, scenario, log);
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    api.error = message;
    loading.textContent = `Could not build the film: ${message}`;
    status('error', message);
    console.error(error);
  }
}

/**
 * Scale for an embedded stage: cover the container when that crops little,
 * otherwise contain it so the HUD at the edges stays in frame.
 */
function embedScale(root: HTMLElement): number {
  const w = Math.max(1, root.clientWidth);
  const h = Math.max(1, root.clientHeight);
  const contain = Math.min(w / STAGE_W, h / STAGE_H);
  const cover = Math.max(w / STAGE_W, h / STAGE_H);
  return cover / contain <= 1.08 ? cover : contain;
}

/** Embedded playback: plays on its own, holds the report card, then loops. Pauses while hidden. */
function autoplay(root: HTMLElement, timeline: FilmFrame[], show: (i: number) => void): void {
  const HOLD_SEC = 3;
  let visible = true;
  new IntersectionObserver((entries) => {
    visible = entries.some((e) => e.isIntersecting);
  }).observe(root);
  let frame = 0;
  let hold = 0;
  let last = performance.now();
  let acc = 0;
  const tick = (now: number) => {
    const dt = Math.min(0.25, (now - last) / 1000);
    last = now;
    if (visible && !document.hidden) {
      if (frame >= timeline.length - 1) {
        hold += dt;
        if (hold >= HOLD_SEC) {
          hold = 0;
          acc = 0;
          frame = 0;
          show(frame);
        }
      } else {
        acc += dt * FILM_FPS;
        const steps = Math.floor(acc);
        if (steps > 0) {
          acc -= steps;
          frame = Math.min(timeline.length - 1, frame + steps);
          show(frame);
        }
      }
    }
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}

function apply(scene: FilmScene, log: FlightLog, f: FilmFrame): void {
  const { frame, index, pos, yaw } = sampleFlight(log, f.t);
  scene.update({
    t: f.t,
    pos,
    yaw,
    vel: frame.vel,
    index,
    inside: frame.inside,
    coverage: frame.phase === 'coverage',
  });
}

function controls(root: HTMLElement, timeline: FilmFrame[], show: (i: number) => void, scenario: ScenarioId, log: FlightLog): void {
  const bar = document.createElement('div');
  bar.className = 'film-controls';
  root.appendChild(bar);
  const play = document.createElement('button');
  play.type = 'button';
  play.className = 'film-play';
  bar.appendChild(play);
  const scrub = document.createElement('input');
  scrub.type = 'range';
  scrub.min = '0';
  scrub.max = String(timeline.length - 1);
  scrub.value = '0';
  scrub.className = 'film-scrub';
  bar.appendChild(scrub);
  const picker = document.createElement('div');
  picker.className = 'film-picker';
  for (const s of SCENARIOS) {
    const a = document.createElement('a');
    a.href = `?scenario=${s.id}`;
    a.textContent = s.title;
    if (s.id === scenario) a.setAttribute('aria-current', 'true');
    picker.appendChild(a);
  }
  bar.appendChild(picker);
  const back = document.createElement('a');
  back.href = `demo.html?scenario=${scenario}`;
  back.className = 'film-link';
  back.textContent = 'Open in the mission console';
  bar.appendChild(back);
  void log;

  let playing = true;
  let frame = 0;
  let last = performance.now();
  let acc = 0;
  const sync = () => {
    play.textContent = playing ? 'Pause' : 'Play';
    play.setAttribute('aria-pressed', String(playing));
  };
  play.addEventListener('click', () => {
    playing = !playing;
    if (frame >= timeline.length - 1) frame = 0;
    sync();
  });
  scrub.addEventListener('input', () => {
    frame = Number(scrub.value);
    show(frame);
  });
  window.addEventListener('keydown', (e) => {
    if (e.code === 'Space') {
      e.preventDefault();
      play.click();
    }
  });
  sync();
  const tick = (now: number) => {
    const dt = Math.min(0.25, (now - last) / 1000);
    last = now;
    if (playing) {
      acc += dt * FILM_FPS;
      const steps = Math.floor(acc);
      if (steps > 0) {
        acc -= steps;
        frame = Math.min(timeline.length - 1, frame + steps);
        show(frame);
        scrub.value = String(frame);
        if (frame >= timeline.length - 1) {
          playing = false;
          sync();
        }
      }
    }
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}
