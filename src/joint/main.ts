import './joint.css';
import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { SMAAPass } from 'three/addons/postprocessing/SMAAPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { INTRO_S, LegDirector, OUTRO_S } from './director';
import { LegHud } from './hud';
import { fetchLeg } from './leg';
import { LegScene } from './scene';

const STAGE_W = 1920;
const STAGE_H = 1080;
const FPS = 30;
/** The legs this build ships, under public/joint/. */
const LEGS = [
  { id: '0582_15-03', label: 'Madrid 0582 · 15-03' },
  { id: '0559_16-06', label: 'Madrid 0559 · 16-06' },
];

interface JointApi {
  ready: boolean;
  error: string | null;
  frames: number;
  fps: number;
  /** Render frame i. Every frame is a pure function of its time, so any order works. */
  render(i: number): void;
}

declare global {
  interface Window {
    __joint?: JointApi;
  }
}

const GradeShader = {
  uniforms: {
    tDiffuse: { value: null as THREE.Texture | null },
    vignette: { value: 0.3 },
    lift: { value: new THREE.Vector3(0.0, 0.006, 0.016) },
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
      col = mix(col, col * col * (3.0 - 2.0 * col), 0.16);
      gl_FragColor = vec4(col, c.a);
    }
  `,
};

async function start(root: HTMLElement): Promise<void> {
  const params = new URLSearchParams(location.search);
  const legId = LEGS.some((l) => l.id === params.get('leg')) ? params.get('leg')! : LEGS[0]!.id;
  const recording = params.get('record') === '1';
  const renderScale = recording ? Math.min(1, Math.max(0.25, Number(params.get('scale') ?? 1) || 1)) : Math.min(1.5, window.devicePixelRatio || 1);
  const api: JointApi = { ready: false, error: null, frames: 0, fps: FPS, render: () => undefined };
  window.__joint = api;
  root.classList.add('joint-root');
  if (recording) root.classList.add('joint-recording');
  const stage = document.createElement('div');
  stage.className = 'joint-stage';
  stage.style.width = `${STAGE_W}px`;
  stage.style.height = `${STAGE_H}px`;
  root.appendChild(stage);
  const canvas = document.createElement('canvas');
  canvas.className = 'joint-canvas';
  stage.appendChild(canvas);
  const loading = document.createElement('div');
  loading.className = 'joint-loading';
  loading.textContent = 'Loading the leg…';
  stage.appendChild(loading);

  const fit = () => {
    const s = recording ? renderScale : Math.min(window.innerWidth / STAGE_W, (window.innerHeight - 56) / STAGE_H);
    stage.style.transform = `scale(${s})`;
    if (!recording) stage.style.marginTop = `${Math.max(0, (window.innerHeight - 56 - STAGE_H * s) / 2)}px`;
  };
  fit();
  window.addEventListener('resize', fit);

  try {
    const base = new URL(`./joint/${legId}/`, location.href).href;
    const leg = await fetchLeg(new URL('leg.json', base).href);
    loading.textContent = 'Loading imagery and camera frames…';
    const scene = new LegScene(canvas, leg);
    scene.renderer.setPixelRatio(renderScale);
    scene.renderer.setSize(STAGE_W, STAGE_H, false);
    const hud = new LegHud(stage, leg, base);
    // ?hud=0 renders the 3D view alone (clean stills, and for checking the camera work).
    if (params.get('hud') === '0') stage.classList.add('joint-nohud');
    await Promise.all([scene.load(base), hud.load()]);
    const director = new LegDirector(leg, scene);
    director.camera.aspect = STAGE_W / STAGE_H;
    director.camera.updateProjectionMatrix();

    const composer = new EffectComposer(scene.renderer);
    composer.setPixelRatio(renderScale);
    composer.setSize(STAGE_W, STAGE_H);
    composer.addPass(new RenderPass(scene.scene, director.camera));
    composer.addPass(new UnrealBloomPass(new THREE.Vector2(STAGE_W / 2, STAGE_H / 2), 0.32, 0.45, 0.82));
    composer.addPass(new OutputPass());
    composer.addPass(new ShaderPass(GradeShader));
    composer.addPass(new SMAAPass(STAGE_W * renderScale, STAGE_H * renderScale));

    const E = leg.epochs;
    const tStart = E[0]!.t - INTRO_S;
    const tEnd = E[E.length - 1]!.t + OUTRO_S;
    const frames = Math.ceil((tEnd - tStart) * FPS) + 1;
    const timeOf = (i: number) => tStart + Math.max(0, Math.min(frames - 1, i)) / FPS;
    const show = (i: number) => {
      const t = timeOf(i);
      director.frame(t);
      scene.update({ t, camera: director.camera, width: STAGE_W, height: STAGE_H });
      composer.render();
      hud.update(t, scene.droneScale, director.camera, scene.droneWorld, scene.ghostWorld);
    };
    api.frames = frames;
    api.render = show;
    loading.remove();
    show(0);
    api.ready = true;
    root.dataset.ready = '1';
    if (!recording) controls(root, frames, show, legId);
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    api.error = message;
    loading.textContent = `Could not build the leg: ${message}`;
  }
}

/** Play, pause, scrub and switch legs. Hidden while recording. */
function controls(root: HTMLElement, frames: number, show: (i: number) => void, legId: string): void {
  const bar = document.createElement('div');
  bar.className = 'joint-controls';
  root.appendChild(bar);
  const play = document.createElement('button');
  play.type = 'button';
  play.className = 'joint-play';
  bar.appendChild(play);
  const scrub = document.createElement('input');
  scrub.type = 'range';
  scrub.min = '0';
  scrub.max = String(frames - 1);
  scrub.value = '0';
  scrub.setAttribute('aria-label', 'Film position');
  bar.appendChild(scrub);
  for (const l of LEGS) {
    const a = document.createElement('a');
    a.className = `joint-leg${l.id === legId ? ' is-current' : ''}`;
    a.href = `?leg=${l.id}`;
    a.textContent = l.label;
    if (l.id === legId) a.setAttribute('aria-current', 'page');
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
