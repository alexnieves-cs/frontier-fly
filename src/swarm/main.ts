import './swarm.css';
import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { SMAAPass } from 'three/addons/postprocessing/SMAAPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { INTRO_S, SwarmDirector } from './director';
import { fetchSwarm, Track } from './film';
import { SwarmHud } from './hud';
import { SwarmScene } from './scene';

const STAGE_W = 1920;
const STAGE_H = 1080;
const FPS = 30;
/** The films this build ships, under public/swarm/. */
const FILMS = [
  { id: 'hard-83101-4', label: '4 drones · 20 m building' },
  { id: 'large-83601-4', label: '4 drones · 40 m building' },
  // The one PX4 team flight on the hard test set where two drones passed closer than 0.35 m.
  { id: 'hard-83102-4', label: 'The close pass · 20 m building' },
];

interface SwarmApi {
  ready: boolean;
  error: string | null;
  frames: number;
  fps: number;
  /** Render frame i. Every frame is a pure function of its time, so any order works. */
  render(i: number): void;
  /** Film time, sim time and shot of frame i (for tooling). */
  info?(i: number): { tau: number; t: number; shot: string; cam: number[] };
}

declare global {
  interface Window {
    __swarm?: SwarmApi;
  }
}

const GradeShader = {
  uniforms: {
    tDiffuse: { value: null as THREE.Texture | null },
    vignette: { value: 0.34 },
    lift: { value: new THREE.Vector3(0.0, 0.008, 0.02) },
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
      float v = 1.0 - vignette * smoothstep(0.16, 0.75, dot(d, d) * 1.6);
      vec3 col = c.rgb * v + lift * (1.0 - c.rgb);
      col = mix(col, col * col * (3.0 - 2.0 * col), 0.18);
      gl_FragColor = vec4(col, c.a);
    }
  `,
};

async function start(root: HTMLElement): Promise<void> {
  const params = new URLSearchParams(location.search);
  const filmId = FILMS.some((f) => f.id === params.get('film')) ? params.get('film')! : FILMS[0]!.id;
  const recording = params.get('record') === '1';
  const renderScale = recording ? Math.min(1, Math.max(0.25, Number(params.get('scale') ?? 1) || 1)) : Math.min(1.5, window.devicePixelRatio || 1);
  const api: SwarmApi = { ready: false, error: null, frames: 0, fps: FPS, render: () => undefined };
  window.__swarm = api;
  root.classList.add('sw-root');
  if (recording) root.classList.add('sw-recording');
  const stage = document.createElement('div');
  stage.className = 'sw-stage';
  stage.style.width = `${STAGE_W}px`;
  stage.style.height = `${STAGE_H}px`;
  root.appendChild(stage);
  const canvas = document.createElement('canvas');
  canvas.className = 'sw-canvas';
  stage.appendChild(canvas);
  const loading = document.createElement('div');
  loading.className = 'sw-loading';
  loading.textContent = 'Loading the flight…';
  stage.appendChild(loading);

  const fit = () => {
    const s = recording ? renderScale : Math.min(window.innerWidth / STAGE_W, (window.innerHeight - 56) / STAGE_H);
    stage.style.transform = `scale(${s})`;
    if (!recording) stage.style.marginTop = `${Math.max(0, (window.innerHeight - 56 - STAGE_H * s) / 2)}px`;
  };
  fit();
  window.addEventListener('resize', fit);

  try {
    const bundle = await fetchSwarm(new URL(`./swarm/${filmId}.json`, location.href).href);
    const track = new Track(bundle);
    loading.textContent = 'Building the scene…';
    const scene = new SwarmScene(canvas, bundle, track);
    scene.renderer.setPixelRatio(renderScale);
    scene.renderer.setSize(STAGE_W, STAGE_H, false);
    await scene.load();
    const hud = new SwarmHud(stage, bundle, track);
    // ?hud=0 renders the 3D view alone (clean stills, and for checking the camera work).
    if (params.get('hud') === '0') stage.classList.add('sw-nohud');
    const director = new SwarmDirector(track, scene.W, scene.D);
    director.camera.aspect = STAGE_W / STAGE_H;
    director.camera.updateProjectionMatrix();

    // ?fx=nobloom,nosmaa,noshadow,nolights switches effects off, to measure what a frame costs.
    const fx = new Set((params.get('fx') ?? '').split(','));
    if (fx.has('noshadow')) scene.renderer.shadowMap.enabled = false;
    if (fx.has('nolights')) scene.scene.traverse((o) => { if (o instanceof THREE.PointLight) o.visible = false; });
    const composer = new EffectComposer(scene.renderer);
    composer.setPixelRatio(renderScale);
    composer.setSize(STAGE_W, STAGE_H);
    composer.addPass(new RenderPass(scene.scene, director.camera));
    if (!fx.has('nobloom')) composer.addPass(new UnrealBloomPass(new THREE.Vector2(STAGE_W / 2, STAGE_H / 2), 0.55, 0.5, 0.78));
    composer.addPass(new OutputPass());
    composer.addPass(new ShaderPass(GradeShader));
    if (!fx.has('nosmaa')) composer.addPass(new SMAAPass(STAGE_W * renderScale, STAGE_H * renderScale));

    const frames = Math.ceil(director.total * FPS) + 1;
    const show = (i: number) => {
      const tau = Math.max(0, Math.min(frames - 1, i)) / FPS;
      const { t, wide } = director.frame(tau);
      scene.update({ t, camera: director.camera, width: STAGE_W * renderScale, height: STAGE_H * renderScale, wide });
      composer.render();
      hud.update(tau, t, director.speedAt(tau), scene.samples, director.camera, scene.droneWorld, scene.baseWorld, INTRO_S, director.simEndFilm);
    };
    api.frames = frames;
    api.render = show;
    api.info = (i) => {
      const tau = Math.max(0, Math.min(frames - 1, i)) / FPS;
      const { t } = director.frame(tau);
      const p = director.camera.position;
      return { tau, t, shot: director.shotName, cam: [p.x, p.y, p.z].map((v) => Math.round(v * 100) / 100) };
    };
    loading.remove();
    show(Number(params.get('frame') ?? 0) || 0);
    api.ready = true;
    root.dataset.ready = '1';
    if (!recording) controls(root, frames, show, filmId);
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    api.error = message;
    loading.textContent = `Could not build the film: ${message}`;
  }
}

/** Play, pause, scrub and switch films. Hidden while recording. */
function controls(root: HTMLElement, frames: number, show: (i: number) => void, filmId: string): void {
  const bar = document.createElement('div');
  bar.className = 'sw-controls';
  root.appendChild(bar);
  const play = document.createElement('button');
  play.type = 'button';
  play.className = 'sw-play';
  bar.appendChild(play);
  const scrub = document.createElement('input');
  scrub.type = 'range';
  scrub.min = '0';
  scrub.max = String(frames - 1);
  scrub.value = '0';
  scrub.setAttribute('aria-label', 'Film position');
  bar.appendChild(scrub);
  for (const f of FILMS) {
    const a = document.createElement('a');
    a.className = `sw-film${f.id === filmId ? ' is-current' : ''}`;
    a.href = `?film=${f.id}`;
    a.textContent = f.label;
    if (f.id === filmId) a.setAttribute('aria-current', 'page');
    bar.appendChild(a);
  }
  let frame = 0;
  let playing = !matchMedia('(prefers-reduced-motion: reduce)').matches;
  let last = performance.now();
  let acc = 0;
  const label = () => {
    play.textContent = playing ? 'Pause' : 'Play';
    play.setAttribute('aria-pressed', String(playing));
  };
  label();
  play.addEventListener('click', () => {
    playing = !playing;
    if (playing && frame >= frames - 1) frame = 0;
    last = performance.now();
    label();
  });
  scrub.addEventListener('input', () => {
    frame = Number(scrub.value);
    show(frame);
  });
  window.addEventListener('keydown', (e) => {
    if (e.key === ' ' && e.target === document.body) {
      e.preventDefault();
      play.click();
    }
  });
  const tick = (now: number) => {
    if (playing) {
      acc += (now - last) / 1000;
      const step = Math.floor(acc * FPS);
      if (step > 0) {
        acc -= step / FPS;
        frame = Math.min(frames - 1, frame + step);
        show(frame);
        scrub.value = String(frame);
        if (frame >= frames - 1) {
          playing = false;
          label();
        }
      }
    }
    last = now;
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}

const root = document.querySelector('#app');
if (root instanceof HTMLElement) void start(root);
