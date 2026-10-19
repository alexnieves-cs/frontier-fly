import * as THREE from 'three';
import { buildDrone, type DroneRig } from '../../../viz/droneMesh';
import { applyPbr, makeMaterials } from '../../../viz/three/materials';
import { SiteDressing } from '../../../viz/site/site';
import { loadPbrSet } from '../../../viz/textures';
import { sampleHeight } from '../terrain';
import type { OutdoorWorld } from '../types';

type PbrSet = Awaited<ReturnType<typeof loadPbrSet>>;

export type OutdoorCamera = 'fpv' | 'chase';

const base = import.meta.env.BASE_URL;

/**
 * Live 3D view of an outdoor site: the same sky, terrain, forest, fence and
 * textured building shells as the mission film (SiteDressing), plus the drone
 * and a chase or FPV camera.
 */
export class OutdoorView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(58, 1, 0.08, 3000);
  private readonly drone: DroneRig;
  private readonly chasePos = new THREE.Vector3();
  private readonly chaseLook = new THREE.Vector3();
  private chaseReady = false;
  private readonly soft: boolean;
  private world: OutdoorWorld | null = null;
  private site: SiteDressing | null = null;
  private mode: OutdoorCamera = 'chase';
  private spin = 0;
  private frames = 0;
  private fps = 0;
  private lastFps = performance.now();
  private pbr: Promise<PbrSet | null> | null = null;

  constructor(canvas: HTMLCanvasElement) {
    this.soft = softwareWebgl();
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: !this.soft, preserveDrawingBuffer: true });
    // Software GL is fill-bound: render three quarters of the pixels' resolution and let the browser upscale.
    this.renderer.setPixelRatio(this.soft ? 0.75 : Math.min(window.devicePixelRatio || 1, 1.5));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 0.9;
    this.renderer.shadowMap.enabled = !this.soft;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    // Trees and shells do not move, so the shadow map is rendered once per world.
    this.renderer.shadowMap.autoUpdate = false;
    this.drone = buildDrone();
    // Twice life size so it reads from a chase distance outdoors.
    this.drone.group.scale.setScalar(2);
    this.drone.group.traverse((o) => (o.castShadow = true));
    this.scene.add(this.drone.group);
    this.resize();
  }

  get framesPerSecond(): number {
    return this.fps;
  }

  /** Build the site. `timeOfDay` (hours) picks day or a moonlit night; default mid-morning. */
  setWorld(world: OutdoorWorld, opts: { timeOfDay?: number } = {}): void {
    this.disposeWorld();
    this.world = world;
    const timeOfDay = opts.timeOfDay ?? 10.5;
    const site = new SiteDressing(this.renderer, this.scene, world, { seed: world.seed, timeOfDay, quality: this.soft ? 'low' : 'high' });
    site.build();
    this.renderer.toneMappingExposure = site.night ? 1.25 : 0.9;
    const mats = makeMaterials(world.seed, this.soft ? 'low' : 'high');
    for (const shell of world.buildings) {
      const built = site.buildShell(shell, mats);
      // A warm interior glow. Software GL skips it by day, where it barely shows.
      if (this.soft && !site.night) continue;
      const light = new THREE.PointLight('#ffd7ae', 8, Math.max(built.width, built.depth) * 1.4, 1.4);
      light.position.set(built.width / 2, 2.1, built.depth / 2);
      built.root.add(light);
    }
    this.site = site;
    this.renderer.shadowMap.needsUpdate = true;
    // Link every shader now, so the first flown frames do not stall on compiles.
    this.renderer.compile(this.scene, this.camera);
    if (!this.soft) {
      this.pbr ??= loadPbrSet(`${base}textures/`).catch(() => null);
      void this.pbr.then((set) => {
        if (!set || this.site !== site) return;
        applyPbr(mats, set);
        this.renderer.shadowMap.needsUpdate = true;
      });
    }
  }

  setCamera(mode: OutdoorCamera): void {
    this.mode = mode;
    this.camera.fov = mode === 'fpv' ? 104 : 58;
    this.camera.updateProjectionMatrix();
  }

  update(pos: [number, number, number], yaw: number, rotors: number): void {
    const world = this.world;
    if (!world) return;
    this.drone.group.visible = this.mode !== 'fpv';
    this.drone.group.position.set(pos[0], pos[2], pos[1]);
    this.drone.group.rotation.set(0, -yaw, 0);
    this.spin += rotors;
    for (const rotor of this.drone.rotors) rotor.rotation.y = this.spin;
    if (this.mode === 'fpv') this.placeFpv(pos, yaw);
    else this.placeChase(world, pos, yaw);
    this.renderer.render(this.scene, this.camera);
    this.frames += 1;
    const now = performance.now();
    if (now - this.lastFps > 500) {
      this.fps = (this.frames * 1000) / (now - this.lastFps);
      this.frames = 0;
      this.lastFps = now;
    }
  }

  resize(): void {
    const canvas = this.renderer.domElement;
    const width = canvas.clientWidth || 1;
    const height = canvas.clientHeight || 1;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  dispose(): void {
    this.disposeWorld();
    this.renderer.dispose();
  }

  private placeFpv(pos: [number, number, number], yaw: number): void {
    const nx = Math.cos(yaw);
    const ny = Math.sin(yaw);
    this.camera.position.set(pos[0] + nx * 0.12, pos[2] + 0.04, pos[1] + ny * 0.12);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(pos[0] + nx * 8, pos[2] - 0.85, pos[1] + ny * 8);
  }

  /** Damped chase: trails a few metres behind and above, eases toward the heading. */
  private placeChase(world: OutdoorWorld, pos: [number, number, number], yaw: number): void {
    const nx = Math.cos(yaw);
    const ny = Math.sin(yaw);
    const x = pos[0] - nx * 5.2;
    const y = pos[1] - ny * 5.2;
    let z = pos[2] + 2.1;
    const ground = sampleHeight(world.terrain, x, y) + 0.7;
    if (z < ground) z = ground;
    const want = new THREE.Vector3(x, z, y);
    const look = new THREE.Vector3(pos[0] + nx * 3, pos[2] + 0.4, pos[1] + ny * 3);
    if (!this.chaseReady || this.chasePos.distanceTo(want) > 25) {
      this.chasePos.copy(want);
      this.chaseLook.copy(look);
      this.chaseReady = true;
    } else {
      this.chasePos.lerp(want, 0.08);
      this.chaseLook.lerp(look, 0.16);
    }
    this.camera.position.copy(this.chasePos);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(this.chaseLook);
  }

  private disposeWorld(): void {
    this.chaseReady = false;
    this.site?.dispose();
    this.site = null;
  }
}

function softwareWebgl(): boolean {
  const probe = document.createElement('canvas');
  const gl = probe.getContext('webgl2');
  if (!gl) return true;
  const debug = gl.getExtension('WEBGL_debug_renderer_info');
  const name = debug ? String(gl.getParameter(debug.UNMASKED_RENDERER_WEBGL)) : '';
  gl.getExtension('WEBGL_lose_context')?.loseContext();
  return /swiftshader|llvmpipe|softpipe|software/i.test(name);
}
