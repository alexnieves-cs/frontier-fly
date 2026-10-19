import * as THREE from 'three';
import { VOXEL } from '../shared/constants';
import type { BuildingSpec, CameraMode, DroneState, LightingState, QualityPreset, Renderer } from '../shared/types';
import { skyFromTime } from '../sim/lighting';
import { boomScale } from './cameras/chase';
import { placeFpv } from './cameras/fpv';
import { placeOverview } from './cameras/overview';
import { buildDrone } from './droneMesh';
import { buildBuilding, type BuiltLevel } from './meshBuilding';
import { loadPbrSet } from './textures';
import { makeLidarLines, updateLidarLines } from './rays';
import { applySkyLights, makeSun } from './three/lights';
import { applyPbr, makeMaterials, type MaterialSet } from './three/materials';
import { makePost, type PostStack } from './three/post';
import { configureShadows } from './three/shadows';
import { applySkyBackdrop } from './three/sky';
import { isSoftwareRendererName, resolveQuality, shouldTryWebGPU, type QualityChoice, type RendererChoice } from './quality';

export interface ViewHooks {
  fps: number;
  backend: string;
}

/**
 * Three.js view. WebGL2 with AgX, HDRI, soft shadows, bloom, SSAO, and SMAA by
 * default. WebGPU is opt-in (`?renderer=webgpu`) because that path skips the
 * HDRI, shadows, and post stack.
 */
export class ThreeView implements Renderer {
  private gl: THREE.WebGLRenderer | null = null;
  private gpu: { domElement: HTMLCanvasElement; render: (s: THREE.Scene, c: THREE.Camera) => void; setSize: (w: number, h: number, u?: boolean) => void; dispose: () => void; setPixelRatio: (n: number) => void } | null = null;
  private pip: THREE.WebGLRenderer | null = null;
  private readonly scene = new THREE.Scene();
  private readonly fpvCam = new THREE.PerspectiveCamera(110, 1, 0.05, 80);
  private readonly mainCam = new THREE.PerspectiveCamera(58, 1, 0.08, 120);
  private readonly sun = makeSun();
  private readonly ambient = new THREE.AmbientLight('#dfe7f2', 0.35);
  private readonly hemi = new THREE.HemisphereLight('#c5d7ee', '#2a241c', 0.4);
  private readonly rig = buildDrone();
  private readonly rays = makeLidarLines();
  private readonly trailGeom = new THREE.BufferGeometry();
  private readonly trail = new THREE.Line(this.trailGeom, new THREE.LineBasicMaterial({ color: '#7dffe1' }));
  private readonly visited: THREE.InstancedMesh;
  private readonly beam: THREE.Mesh;
  private readonly flash: THREE.SpotLight;
  private readonly tmp = new THREE.Vector3();
  private readonly chase = new THREE.Vector3(2, 2, 2);
  private mats: MaterialSet | null = null;
  private built: BuiltLevel | null = null;
  private spec: BuildingSpec | null = null;
  private post: PostStack | null = null;
  private mode: CameraMode = 'chase';
  private quality: QualityPreset = 'high';
  private sensor: LightingState['sensorMode'] = 'rgb';
  private theta = 0.8;
  private phi = 0.9;
  private radius = 14;
  private backend: 'webgpu' | 'webgl2' = 'webgl2';
  private running = false;
  private frames = 0;
  private fps = 0;
  private lastFps = performance.now();
  private fpsCount = 0;
  private disposed = false;
  private lidar: ArrayLike<number> | null = null;
  private collision = false;
  private readonly clock = new THREE.Clock();
  private hdri: THREE.Texture | null = null;
  private envMap: THREE.Texture | null = null;
  private readonly frameSpot = new THREE.SpotLight('#fff6e4', 0, 22, Math.PI / 7, 0.35, 1);
  private framed: { eye: THREE.Vector3; look: THREE.Vector3; fov: number; beam: boolean } | null = null;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly pipCanvas: HTMLCanvasElement,
  ) {
    const geo = new THREE.BoxGeometry(VOXEL * 0.92, 0.02, VOXEL * 0.92);
    const mat = new THREE.MeshBasicMaterial({ color: '#1ee0c0', transparent: true, opacity: 0.35 });
    this.visited = new THREE.InstancedMesh(geo, mat, 2048);
    this.visited.count = 0;
    this.visited.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    const beamGeo = new THREE.ConeGeometry(0.55, 3.6, 20, 1, true);
    this.beam = new THREE.Mesh(
      beamGeo,
      new THREE.MeshBasicMaterial({
        color: '#e7fff6',
        transparent: true,
        opacity: 0.09,
        depthWrite: false,
        side: THREE.DoubleSide,
        blending: THREE.AdditiveBlending,
      }),
    );
    this.beam.rotation.z = -Math.PI / 2;
    this.beam.position.set(1.9, 0, 0);
    this.flash = new THREE.SpotLight('#fff6e4', 0, 14, Math.PI / 4.5, 0.45, 1);
    this.flash.position.set(0.14, 0, 0);
    this.flash.target.position.set(5, -0.45, 0);
    this.flash.castShadow = true;
    this.flash.shadow.mapSize.set(1024, 1024);
    this.rig.group.add(this.beam, this.flash, this.flash.target);
    this.scene.add(this.sun, this.ambient, this.hemi, this.rig.group, this.rays, this.trail, this.visited, this.frameSpot, this.frameSpot.target);
    this.sun.target.position.set(0, 0, 0);
    this.scene.add(this.sun.target);
  }

  async init(quality: QualityChoice = 'auto', renderer: RendererChoice = 'webgl2'): Promise<void> {
    const gpuOk = shouldTryWebGPU(renderer, quality) ? await this.tryWebGPU() : false;
    if (!gpuOk) this.makeWebGL();
    this.quality = resolveQuality(quality, this.softwareGL());
    this.pip = new THREE.WebGLRenderer({ canvas: this.pipCanvas, antialias: true, alpha: true });
    this.pip.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    this.pip.outputColorSpace = THREE.SRGBColorSpace;
    this.pip.toneMapping = THREE.AgXToneMapping;
    this.resize();
    this.running = true;
    this.clock.start();
    const loop = () => {
      if (!this.running || this.disposed) return;
      this.paint();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  private async tryWebGPU(): Promise<boolean> {
    const gpuNav = navigator.gpu;
    if (!gpuNav || this.quality === 'low') return false;
    try {
      const adapter = await Promise.race([
        gpuNav.requestAdapter(),
        new Promise<null>((resolve) => setTimeout(() => resolve(null), 350)),
      ]);
      if (!adapter) return false;
      const mod = await import('three/webgpu');
      const gpu = new mod.WebGPURenderer({ canvas: this.canvas, antialias: true });
      await gpu.init();
      gpu.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
      gpu.outputColorSpace = THREE.SRGBColorSpace;
      gpu.toneMapping = THREE.AgXToneMapping;
      this.gpu = gpu;
      this.backend = 'webgpu';
      return true;
    } catch {
      this.gpu = null;
      return false;
    }
  }

  private makeWebGL(): void {
    const gl = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, preserveDrawingBuffer: true });
    gl.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    gl.outputColorSpace = THREE.SRGBColorSpace;
    gl.toneMapping = THREE.AgXToneMapping;
    gl.toneMappingExposure = 1;
    this.gl = gl;
    this.backend = 'webgl2';
    void this.loadSky();
  }

  private softwareGL(): boolean {
    if (!this.gl) return false;
    const ctx = this.gl.getContext();
    const debug = ctx.getExtension('WEBGL_debug_renderer_info');
    return debug ? isSoftwareRendererName(String(ctx.getParameter(debug.UNMASKED_RENDERER_WEBGL))) : false;
  }

  private async loadSky(): Promise<void> {
    if (!this.gl || this.hdri) return;
    try {
      const { RGBELoader } = await import('three/addons/loaders/RGBELoader.js');
      const tex = await new RGBELoader().loadAsync(`${import.meta.env.BASE_URL}hdri/kloofendal_43d_clear_puresky_1k.hdr`);
      tex.mapping = THREE.EquirectangularReflectionMapping;
      const pmrem = new THREE.PMREMGenerator(this.gl);
      this.envMap = pmrem.fromEquirectangular(tex).texture;
      pmrem.dispose();
      this.hdri = tex;
      this.scene.environment = this.envMap;
    } catch (error) {
      console.warn('hdri unavailable', error);
    }
  }

  load(spec: BuildingSpec): void {
    this.spec = spec;
    if (this.built) {
      this.scene.remove(this.built.root);
      this.built.root.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        if (mesh.geometry) mesh.geometry.dispose();
      });
    }
    this.mats?.wall.dispose();
    this.mats = makeMaterials(spec.seed, this.quality);
    const mats = this.mats;
    void loadPbrSet(`${import.meta.env.BASE_URL}textures/`)
      .then((maps) => {
        if (this.mats === mats) applyPbr(mats, maps);
      })
      .catch((error: unknown) => console.warn('pbr unavailable', error));
    this.built = buildBuilding(spec, this.mats, this.quality === 'high');
    this.scene.add(this.built.root);
    const span = Math.max(spec.floors[0]!.w, spec.floors[0]!.d) * spec.cellSize;
    this.radius = span * 0.85;
    if (this.gl) {
      configureShadows(this.gl, this.sun, this.quality, span);
      this.post?.composer.dispose();
      try {
        this.post = makePost(this.gl, this.scene, this.mainCam, this.quality);
      } catch (error) {
        console.warn('post stack unavailable', error);
        this.post = null;
      }
    }
    let shadowBudget = this.quality === 'high' ? 4 : 0;
    for (const entry of this.built.lights) {
      entry.light.castShadow = shadowBudget > 0;
      if (shadowBudget > 0) shadowBudget -= 1;
    }
    this.resize();
  }

  update(s: DroneState, l: LightingState): void {
    const yaw = s.yaw;
    this.rig.group.position.set(s.pos[0], s.pos[2], s.pos[1]);
    this.rig.group.rotation.set(s.pitch, -yaw, s.roll);
    const spin = 0.45 + (s.motors[0] ?? 0) * 0.004;
    for (const rotor of this.rig.rotors) rotor.rotation.y += spin;
    this.rig.updateWash(s.t, s.motors[0] ?? 400);
    this.tmp.set(s.pos[0], s.pos[2] + 0.02, s.pos[1]);
    updateLidarLines(this.rays, this.tmp, yaw, this.lidar);
    const sky = skyFromTime(l.timeOfDay);
    applySkyLights(this.sun, this.ambient, this.hemi, sky, l.flashlightOn);
    applySkyBackdrop(this.scene, sky, this.hdri, this.envMap);
    this.post?.setView(this.mode, Math.hypot(s.vel[0], s.vel[1], s.vel[2]));
    const flashOn = l.flashlightOn || sky.night > 0.55;
    this.flash.intensity = flashOn ? (l.sensorMode === 'ir' ? 28 : 18) : 0;
    this.flash.color.set(l.sensorMode === 'ir' ? '#b6ff9a' : '#fff4dd');
    this.beam.visible = flashOn;
    (this.beam.material as THREE.MeshBasicMaterial).color.set(l.sensorMode === 'ir' ? '#9dff7a' : '#fff6e8');
    if (this.built) {
      for (const entry of this.built.lights) {
        const on = l.interiorLightsOn[entry.roomIndex] ?? 1;
        entry.light.intensity = on ? 3.2 * (0.35 + sky.night * 1.4) : 0;
      }
      const floor = Math.max(0, Math.min(this.built.floors.length - 1, Math.floor(s.pos[2] / (this.spec?.floorHeight ?? 3))));
      this.built.floors.forEach((group, index) => {
        group.visible = this.mode !== 'overview' || index <= floor;
      });
      this.built.ceilings.forEach((ceil, index) => {
        ceil.visible = this.mode !== 'overview' || index < floor;
      });
    }
    this.sensor = l.sensorMode;
    if (this.mats) this.scene.overrideMaterial = l.sensorMode === 'depth' ? this.mats.depth : null;
    const exposure = 0.85 + sky.sunIntensity * 0.06 + (flashOn ? 0.15 : 0) + (l.sensorMode === 'ir' ? 0.2 : 0);
    if (this.gl) this.gl.toneMappingExposure = exposure;
    if (this.pip) this.pip.toneMappingExposure = exposure * 1.05;
    this.placeCameras(s);
  }

  setCamera(m: CameraMode): void {
    this.mode = m;
  }

  setQuality(q: QualityPreset): void {
    this.quality = q;
    if (this.spec) this.load(this.spec);
  }

  setLidar(ranges: ArrayLike<number> | null): void {
    this.lidar = ranges;
  }

  setCollision(on: boolean): void {
    this.collision = on;
  }

  setTrail(points: Array<[number, number, number]>): void {
    const arr = new Float32Array(points.length * 3);
    for (let i = 0; i < points.length; i++) {
      const p = points[i]!;
      arr[i * 3] = p[0];
      arr[i * 3 + 1] = p[2];
      arr[i * 3 + 2] = p[1];
    }
    this.trailGeom.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    this.trail.visible = points.length > 1;
  }

  addSceneObject(obj: THREE.Object3D): void {
    this.scene.add(obj);
  }

  /** Lock the main view on a sim-space pose. Used by the target debug shots. */
  setFramedView(
    eye: [number, number, number] | null,
    look: [number, number, number] | null,
    fov = 110,
    beam = false,
  ): void {
    if (!eye || !look) {
      this.framed = null;
      this.frameSpot.intensity = 0;
      return;
    }
    this.framed = {
      eye: new THREE.Vector3(eye[0], eye[2], eye[1]),
      look: new THREE.Vector3(look[0], look[2], look[1]),
      fov,
      beam,
    };
  }

  sensingAspect(): number {
    return this.mainCam.aspect || 1;
  }

  setVisited(spec: BuildingSpec, visited: Uint8Array, floor: number): void {
    const grid = spec.floors[0]!;
    const vw = grid.w * 2;
    const vd = grid.d * 2;
    const dummy = new THREE.Object3D();
    let n = 0;
    const base = floor * vw * vd;
    for (let i = 0; i < vw * vd && n < 2048; i++) {
      if (!visited[base + i]) continue;
      const vx = i % vw;
      const vy = Math.floor(i / vw);
      dummy.position.set((vx + 0.5) * VOXEL, floor * spec.floorHeight + 0.03, (vy + 0.5) * VOXEL);
      dummy.updateMatrix();
      this.visited.setMatrixAt(n, dummy.matrix);
      n += 1;
    }
    this.visited.count = n;
    this.visited.instanceMatrix.needsUpdate = true;
  }

  orbit(dx: number, dy: number): void {
    this.theta -= dx * 0.005;
    this.phi = Math.max(0.2, Math.min(1.3, this.phi + dy * 0.004));
  }

  zoom(delta: number): void {
    this.radius = Math.max(4, Math.min(40, this.radius + delta));
  }

  resize(): void {
    const w = this.canvas.clientWidth || 800;
    const h = this.canvas.clientHeight || 600;
    this.mainCam.aspect = w / Math.max(1, h);
    this.mainCam.updateProjectionMatrix();
    this.fpvCam.aspect = (this.pipCanvas.clientWidth || 320) / Math.max(1, this.pipCanvas.clientHeight || 180);
    this.fpvCam.updateProjectionMatrix();
    this.gl?.setSize(w, h, false);
    this.gpu?.setSize(w, h, false);
    this.pip?.setSize(this.pipCanvas.clientWidth || 320, this.pipCanvas.clientHeight || 180, false);
    if (this.post && this.gl) {
      this.post.composer.setSize(w, h);
    }
  }

  hooks(): ViewHooks {
    return { fps: this.fps, backend: this.backend };
  }

  get collisionFlash(): boolean {
    return this.collision;
  }

  get frameCount(): number {
    return this.frames;
  }

  dispose(): void {
    this.disposed = true;
    this.running = false;
    this.post?.composer.dispose();
    this.gl?.dispose();
    this.gpu?.dispose();
    this.pip?.dispose();
    this.scene.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      mesh.geometry?.dispose();
    });
  }

  private placeCameras(s: DroneState): void {
    const pos = this.rig.group.position;
    const dt = Math.min(0.05, this.clock.getDelta());
    if (this.framed) {
      const frame = this.framed;
      this.mainCam.position.copy(frame.eye);
      this.mainCam.up.set(0, 1, 0);
      this.mainCam.fov = frame.fov;
      this.mainCam.lookAt(frame.look);
      this.mainCam.updateProjectionMatrix();
      this.fpvCam.position.copy(this.mainCam.position);
      this.fpvCam.quaternion.copy(this.mainCam.quaternion);
      this.fpvCam.up.copy(this.mainCam.up);
      this.fpvCam.fov = frame.fov;
      this.fpvCam.aspect = this.mainCam.aspect;
      this.fpvCam.updateProjectionMatrix();
      this.frameSpot.position.copy(frame.eye);
      this.frameSpot.target.position.copy(frame.look);
      this.frameSpot.intensity = frame.beam ? 46 : 0;
      return;
    }
    this.frameSpot.intensity = 0;
    placeFpv(this.fpvCam, pos, s.yaw, s.vel[2]);
    if (!this.spec) return;
    const noseX = Math.cos(s.yaw);
    const noseY = Math.sin(s.yaw);
    const from: [number, number, number] = [s.pos[0], s.pos[1], s.pos[2]];
    const aim: [number, number, number] = [s.pos[0] - noseX * 2.6, s.pos[1] - noseY * 2.6, s.pos[2] + 1.15];
    const scale = boomScale(this.spec, from, aim);
    const destX = s.pos[0] + (aim[0] - s.pos[0]) * scale;
    const destY = s.pos[2] + 1.15 * scale;
    const destZ = s.pos[1] + (aim[1] - s.pos[1]) * scale;
    const k = 1 - Math.exp(-4.5 * Math.min(0.05, Math.max(dt, 0.001)));
    this.chase.x += (destX - this.chase.x) * k;
    this.chase.y += (destY - this.chase.y) * k;
    this.chase.z += (destZ - this.chase.z) * k;
    if (this.mode === 'chase') {
      this.mainCam.position.copy(this.chase);
      this.mainCam.lookAt(pos.x, pos.y + 0.15, pos.z);
    } else if (this.mode === 'overview') {
      placeOverview(this.mainCam, pos, this.theta, this.phi, this.radius);
    } else {
      this.mainCam.position.copy(this.fpvCam.position);
      this.mainCam.quaternion.copy(this.fpvCam.quaternion);
      this.mainCam.fov = 110;
      this.mainCam.updateProjectionMatrix();
    }
  }

  private paint(): void {
    const now = performance.now();
    this.fpsCount += 1;
    if (now - this.lastFps > 500) {
      this.fps = (this.fpsCount * 1000) / (now - this.lastFps);
      this.fpsCount = 0;
      this.lastFps = now;
    }
    const cam = this.mode === 'fpv' ? this.fpvCam : this.mainCam;
    if (this.gpu) this.gpu.render(this.scene, cam);
    else if (this.gl && this.post && this.sensor === 'rgb' && this.quality === 'high') {
      const renderPass = this.post.composer.passes[0] as unknown as { camera: THREE.Camera };
      renderPass.camera = cam;
      if (this.post.ssao) this.post.ssao.camera = cam;
      if (this.post.taa) this.post.taa.camera = cam;
      this.post.composer.render();
    } else if (this.gl) {
      this.gl.render(this.scene, cam);
    }
    this.pip?.render(this.scene, this.fpvCam);
    this.frames += 1;
  }
}
