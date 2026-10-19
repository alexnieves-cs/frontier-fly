import * as THREE from 'three';
import type { MissionBuilding, MissionWorld, V3 } from '../mission/autonomy/contracts';
import { gimbalFootprint } from '../mission/autonomy/perception';
import { TargetView } from '../targets/meshes';
import { resolveScene } from '../targets/motion';
import { buildDrone, type DroneRig } from '../viz/droneMesh';
import { applyPbr, makeMaterials, type MaterialSet } from '../viz/three/materials';
import { SiteDressing } from '../viz/site/site';
import { glowTexture, scanGrid } from '../viz/site/textures';
import { loadPbrSet } from '../viz/textures';
import type { FlightLog } from './flight';

const base = import.meta.env.BASE_URL;

/** Sim frame (x east, y north, z up) to Three (x, y up, z). */
export function toThree(p: V3 | [number, number, number], out = new THREE.Vector3()): THREE.Vector3 {
  return out.set(p[0], p[2], p[1]);
}

export interface FilmState {
  t: number;
  pos: V3;
  yaw: number;
  vel: V3;
  /** Frame index into the flight log. */
  index: number;
  inside: string | null;
  coverage: boolean;
}

interface BuiltShell {
  building: MissionBuilding;
  root: THREE.Group;
  roof: THREE.Object3D[];
  ceilings: THREE.Object3D[];
  targets: TargetView;
  lights: THREE.PointLight[];
}

/** Everything that draws the mission world. Owns no camera logic. */
export class FilmScene {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly night: boolean;
  /** Building meshes the director keeps the camera in front of. */
  readonly occluders: THREE.Object3D[] = [];
  private readonly world: MissionWorld;
  private readonly log: FlightLog;
  private readonly site: SiteDressing;
  private readonly drone: DroneRig;
  private readonly droneRoot = new THREE.Group();
  private readonly navLights: THREE.Sprite[] = [];
  private readonly shells: BuiltShell[] = [];
  private outdoorTargets: TargetView | null = null;
  private trail!: THREE.Line;
  private trailPositions!: Float32Array;
  private trailColors!: Float32Array;
  private plan!: THREE.Line;
  private planIndex = -1;
  private footprint!: THREE.Mesh;
  private coverageCanvas!: HTMLCanvasElement;
  private coverageTexture!: THREE.CanvasTexture;
  private coveragePainted = 0;
  private readonly markers = new THREE.Group();
  private markerPool: THREE.Mesh[] = [];
  private homePad!: THREE.Group;
  private spin = 0;
  private cutaway = 0;
  private time = 0;

  constructor(canvas: HTMLCanvasElement, log: FlightLog) {
    this.log = log;
    this.world = log.loaded.world;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true, powerPreference: 'high-performance' });
    this.site = new SiteDressing(this.renderer, this.scene, this.world.outdoor, { seed: this.world.seed, timeOfDay: this.world.timeOfDay, quality: 'high' });
    this.night = this.site.night;
    this.renderer.setPixelRatio(1);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = this.night ? 1.25 : 0.9;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.drone = buildDrone();
  }

  /** Build the whole world. Resolves once textures and the humanoid rig are loaded. */
  async load(): Promise<void> {
    this.site.build();
    this.occluders.push(...this.site.occluders);
    this.buildCoverageLayer();
    const mats = makeMaterials(this.world.seed, 'high');
    try {
      applyPbr(mats, await loadPbrSet(`${base}textures/`));
    } catch {
      /* canvas fallbacks stay */
    }
    // Under the film's brighter sun, white plaster blows out; use a warm grey.
    if (mats.wall instanceof THREE.MeshStandardMaterial) mats.wall.color.set('#c4bcb0');
    for (const building of this.world.buildings) this.shells.push(this.buildShell(building, mats));
    if (this.world.outdoorScene.targets.length) {
      this.outdoorTargets = new TargetView();
      this.scene.add(this.outdoorTargets.group);
    }
    this.buildDroneRig();
    this.buildOverlays();
    await Promise.all([...this.shells.map((s) => s.targets.wait()), this.outdoorTargets?.wait()]);
    this.syncTargets(0);
  }

  resize(width: number, height: number): void {
    this.renderer.setSize(width, height, false);
  }

  /** 0 = closed building, 1 = roof and ceilings hidden so the camera sees in. */
  setCutaway(amount: number): void {
    this.cutaway = amount;
    for (const shell of this.shells) {
      for (const roof of shell.roof) roof.visible = amount < 0.5;
      for (const ceiling of shell.ceilings) ceiling.visible = amount < 0.5;
    }
  }

  get cutawayAmount(): number {
    return this.cutaway;
  }

  update(state: FilmState): void {
    this.time = state.t;
    const p = toThree(state.pos);
    this.droneRoot.position.copy(p);
    this.droneRoot.rotation.set(0, -state.yaw, 0);
    // Bank into the velocity like a real multirotor.
    const speed = Math.hypot(state.vel[0], state.vel[1]);
    const fwd = state.vel[0] * Math.cos(state.yaw) + state.vel[1] * Math.sin(state.yaw);
    const side = -state.vel[0] * Math.sin(state.yaw) + state.vel[1] * Math.cos(state.yaw);
    this.drone.group.rotation.set(Math.max(-0.35, Math.min(0.35, side * 0.09)), 0, Math.max(-0.35, Math.min(0.35, -fwd * 0.09)));
    this.spin += 0.9 + speed * 0.1;
    for (const rotor of this.drone.rotors) rotor.rotation.y = this.spin;
    this.drone.updateWash(state.t, 0.6 + speed * 0.1);
    const blink = Math.floor(state.t * 1.5) % 2 === 0;
    const strobe = state.t % 1.2 < 0.08;
    this.navLights.forEach((sprite, i) => {
      if (i < 2) sprite.material.opacity = 0.95;
      else sprite.material.opacity = (i === 2 ? strobe : blink) ? 1 : 0.08;
    });
    this.updateTrail(state.index);
    this.updateNightFill(state.index);
    this.updatePlan(state.index);
    // Planner paths are outdoor routes; inside they only slice through walls.
    this.plan.visible = this.plan.geometry.drawRange.count > 1 && !state.inside;
    this.updateFootprint(state);
    this.updateCoverage(state.index);
    this.updateMarkers(state);
    this.syncTargets(state.t);
  }

  render(camera: THREE.Camera): void {
    this.renderer.render(this.scene, camera);
  }

  /** Ground height in sim coordinates. */
  groundAt(x: number, y: number): number {
    return this.site.groundAt(x, y);
  }

  /** Building the drone is in, or nearest building to a point. */
  buildingById(id: string): MissionBuilding | undefined {
    return this.world.buildings.find((b) => b.id === id);
  }

  // ---------------------------------------------------------------- build

  /** Coverage layer: the terrain surface, lifted a little, alpha from a canvas the run paints. */
  private buildCoverageLayer(): void {
    this.coverageCanvas = document.createElement('canvas');
    this.coverageCanvas.width = 256;
    this.coverageCanvas.height = 256;
    this.coverageTexture = new THREE.CanvasTexture(this.coverageCanvas);
    const geo = this.site.terrain.geometry.clone();
    geo.setAttribute('uv', geo.getAttribute('uv1'));
    const overlay = new THREE.Mesh(
      geo,
      new THREE.MeshBasicMaterial({
        color: '#7ff4ff',
        map: scanGrid(this.world.outdoor.size),
        alphaMap: this.coverageTexture,
        opacity: this.night ? 0.22 : 0.55,
        transparent: true,
        depthWrite: false,
        polygonOffset: true,
        polygonOffsetFactor: -2,
        polygonOffsetUnits: -2,
        toneMapped: false,
      }),
    );
    overlay.renderOrder = 2;
    this.scene.add(overlay);
  }

  private buildShell(building: MissionBuilding, mats: MaterialSet): BuiltShell {
    const built = this.site.buildShell(building.shell, mats);
    const { root } = built;
    const spec = building.shell.spec;
    this.occluders.push(root);
    // Room lights follow the scenario's per-room switch.
    const lights: THREE.PointLight[] = [];
    let index = 0;
    spec.floors.forEach((floor, f) => {
      for (const room of floor.rooms) {
        const on = building.interiorLightsOn[index++] === 1;
        if (!on) continue;
        const H = spec.floorHeight;
        const light = new THREE.PointLight('#ffd9b0', 6, Math.max(room.w, room.d) * spec.cellSize * 1.6, 1.6);
        light.position.set((room.x + room.w / 2) * spec.cellSize, f * H + H - 0.35, (room.z + room.d / 2) * spec.cellSize);
        root.add(light);
        lights.push(light);
        const fixture = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.04, 16), new THREE.MeshBasicMaterial({ color: '#fff3dc' }));
        fixture.position.copy(light.position).setY(f * H + H - 0.1);
        root.add(fixture);
      }
    });
    const targets = new TargetView();
    root.add(targets.group);
    return { building, root, roof: built.roof, ceilings: built.ceilings, targets, lights };
  }

  private buildDroneRig(): void {
    this.drone.group.scale.setScalar(1.6);
    this.droneRoot.add(this.drone.group);
    const glow = glowTexture();
    const colors = ['#ff3344', '#33ff88', '#ffffff', '#ffffff'];
    const offsets: [number, number, number][] = [
      [-0.02, 0.02, -0.17],
      [-0.02, 0.02, 0.17],
      [-0.13, 0.05, 0],
      [0.14, -0.04, 0],
    ];
    colors.forEach((c, i) => {
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: glow, color: c, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, fog: false }));
      sprite.scale.setScalar((i === 2 ? 0.3 : 0.17) * (this.night ? 0.7 : 1));
      sprite.position.set(...offsets[i]!);
      this.droneRoot.add(sprite);
      this.navLights.push(sprite);
    });
    if (this.world.flashlightOn || this.night) {
      const spot = new THREE.SpotLight('#fff1d6', 40, 24, 0.42, 0.45, 1.6);
      spot.position.set(0.16, -0.02, 0);
      spot.target.position.set(4, -1.6, 0);
      spot.castShadow = true;
      spot.shadow.mapSize.set(512, 512);
      spot.shadow.camera.near = 0.2;
      this.droneRoot.add(spot, spot.target);
    }
    this.scene.add(this.droneRoot);
  }

  private buildOverlays(): void {
    const frames = this.log.frames;
    this.trailPositions = new Float32Array(frames.length * 3);
    this.trailColors = new Float32Array(frames.length * 3);
    const c = new THREE.Color();
    frames.forEach((frame, i) => {
      toThree(frame.pos).toArray(this.trailPositions, i * 3);
      c.set(phaseColor(frame.phase));
      c.toArray(this.trailColors, i * 3);
    });
    const trailGeo = new THREE.BufferGeometry();
    trailGeo.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
    trailGeo.setAttribute('color', new THREE.BufferAttribute(this.trailColors, 3));
    trailGeo.setDrawRange(0, 0);
    this.trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.95, fog: false, toneMapped: false }));
    this.trail.frustumCulled = false;
    this.trail.renderOrder = 4;
    this.scene.add(this.trail);
    const planGeo = new THREE.BufferGeometry();
    planGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 512), 3));
    this.plan = new THREE.Line(planGeo, new THREE.LineDashedMaterial({ color: '#ffffff', dashSize: 0.8, gapSize: 0.6, transparent: true, opacity: 0.55, fog: false, toneMapped: false }));
    this.plan.frustumCulled = false;
    this.plan.renderOrder = 4;
    this.scene.add(this.plan);
    this.footprint = new THREE.Mesh(
      new THREE.CircleGeometry(1, 48),
      new THREE.MeshBasicMaterial({ color: '#7cf3ff', transparent: true, opacity: 0.18, depthWrite: false, fog: false, toneMapped: false }),
    );
    this.footprint.rotation.x = -Math.PI / 2;
    this.footprint.renderOrder = 3;
    this.scene.add(this.footprint);
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.97, 1, 64),
      new THREE.MeshBasicMaterial({ color: '#b8fbff', transparent: true, opacity: 0.7, depthWrite: false, fog: false, toneMapped: false }),
    );
    ring.position.z = 0.01;
    this.footprint.add(ring);
    this.scene.add(this.markers);
    // Home pad.
    const home = this.world.home;
    this.homePad = new THREE.Group();
    const pad = new THREE.Mesh(new THREE.CylinderGeometry(1.1, 1.1, 0.05, 40), new THREE.MeshStandardMaterial({ color: '#2a3138', roughness: 0.6 }));
    pad.receiveShadow = true;
    this.homePad.add(pad);
    const h = new THREE.Mesh(
      new THREE.RingGeometry(0.75, 0.85, 40),
      new THREE.MeshBasicMaterial({ color: '#f5c542', side: THREE.DoubleSide, toneMapped: false }),
    );
    h.rotation.x = -Math.PI / 2;
    h.position.y = 0.03;
    this.homePad.add(h);
    const letter = new THREE.Group();
    for (const [w, d, x, z] of [
      [0.08, 0.6, -0.18, 0],
      [0.08, 0.6, 0.18, 0],
      [0.36, 0.08, 0, 0],
    ] as const) {
      const bar = new THREE.Mesh(new THREE.PlaneGeometry(w, d), new THREE.MeshBasicMaterial({ color: '#f5c542', toneMapped: false }));
      bar.rotation.x = -Math.PI / 2;
      bar.position.set(x, 0.031, z);
      letter.add(bar);
    }
    this.homePad.add(letter);
    this.homePad.position.set(home[0], this.groundAt(home[0], home[1]) + 0.02, home[1]);
    this.scene.add(this.homePad);
  }

  // --------------------------------------------------------------- update

  /** First frame of the building visit that frame `index` is part of, or -1 when outside. */
  private visitStart(index: number): number {
    const inside = this.log.frames[index]?.inside ?? null;
    if (!inside) return -1;
    let from = index;
    while (from > 0 && this.log.frames[from - 1]!.inside === inside) from--;
    return from;
  }

  /**
   * At night the interior is lit only by the drone's flashlight, which reads as black from the
   * cutaway camera. Ease up a cool moonlight fill over the first 1.5 s inside, and back down
   * outside. Driven by flight time, so any frame renders the same on its own.
   */
  private updateNightFill(index: number): void {
    if (!this.night) return;
    const from = this.visitStart(index);
    const f = from < 0 ? 0 : Math.min(1, (this.log.frames[index]!.t - this.log.frames[from]!.t) / 1.5);
    const k = f * f * (3 - 2 * f);
    this.site.hemi.intensity = 0.7 + k * 1.1;
  }

  private updateTrail(index: number): void {
    // Inside a building, draw only this visit's indoor path: the outdoor inspection loop would
    // otherwise cross the cutaway view at roof height.
    const from = Math.max(0, this.visitStart(index));
    this.trail.geometry.setDrawRange(from, index + 1 - from);
  }

  private updatePlan(index: number): void {
    const frame = this.log.frames[index]!;
    if (frame.plan === this.planIndex) return;
    this.planIndex = frame.plan;
    const plan = this.log.plans[frame.plan] ?? [];
    const attr = this.plan.geometry.getAttribute('position') as THREE.BufferAttribute;
    const n = Math.min(plan.length, attr.count);
    for (let i = 0; i < n; i++) attr.setXYZ(i, plan[i]![0], plan[i]![2], plan[i]![1]);
    attr.needsUpdate = true;
    this.plan.geometry.setDrawRange(0, n);
    this.plan.computeLineDistances();
  }

  private updateFootprint(state: FilmState): void {
    const ground = this.groundAt(state.pos[0], state.pos[1]);
    const agl = state.pos[2] - ground;
    const show = state.coverage && !state.inside && agl > 3;
    this.footprint.visible = show;
    if (!show) return;
    const fp = gimbalFootprint(agl);
    const r = Math.min(15, Math.max(fp.halfAlongM, fp.halfAcrossM));
    this.footprint.position.set(state.pos[0], ground + 0.15, state.pos[1]);
    this.footprint.scale.setScalar(r);
  }

  private updateCoverage(index: number): void {
    // Paint every outdoor coverage tick up to this frame onto the overlay canvas.
    if (index < this.coveragePainted) {
      const g = this.coverageCanvas.getContext('2d')!;
      g.clearRect(0, 0, 256, 256);
      this.coveragePainted = 0;
    }
    if (index === this.coveragePainted) return;
    const g = this.coverageCanvas.getContext('2d')!;
    const size = this.world.outdoor.size;
    const s = 256 / size;
    g.fillStyle = '#ffffff';
    for (let i = this.coveragePainted; i <= index; i += 2) {
      const frame = this.log.frames[i]!;
      if (frame.phase !== 'coverage' || frame.inside) continue;
      const ground = this.groundAt(frame.pos[0], frame.pos[1]);
      const agl = frame.pos[2] - ground;
      if (agl < 3) continue;
      const fp = gimbalFootprint(agl);
      const r = Math.min(15, Math.max(fp.halfAlongM, fp.halfAcrossM));
      g.beginPath();
      g.arc(frame.pos[0] * s, frame.pos[1] * s, r * s, 0, Math.PI * 2);
      g.fill();
    }
    this.coveragePainted = index;
    this.coverageTexture.needsUpdate = true;
  }

  private updateMarkers(state: FilmState): void {
    // Confirmed contacts: a beacon ring and a light pillar at the estimate.
    const confirmed = this.log.confirmed.filter((c) => c.tSec <= state.t);
    while (this.markerPool.length < confirmed.length) {
      const group = new THREE.Mesh(
        new THREE.RingGeometry(0.42, 0.5, 48),
        new THREE.MeshBasicMaterial({ color: '#3dff9a', transparent: true, opacity: 0.95, side: THREE.DoubleSide, depthWrite: false, depthTest: false, toneMapped: false }),
      );
      group.rotation.x = -Math.PI / 2;
      group.renderOrder = 9;
      const pillar = new THREE.Mesh(
        new THREE.CylinderGeometry(0.05, 0.05, 6, 8, 1, true),
        new THREE.MeshBasicMaterial({ color: '#3dff9a', transparent: true, opacity: 0.35, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false }),
      );
      pillar.rotation.x = Math.PI / 2;
      pillar.position.z = -3;
      group.add(pillar);
      this.markers.add(group);
      this.markerPool.push(group);
    }
    this.markerPool.forEach((marker, i) => {
      const c = confirmed[i];
      marker.visible = Boolean(c);
      if (!c) return;
      const age = state.t - c.tSec;
      const pulse = 1 + 0.25 * Math.sin(age * 5) * Math.exp(-age * 0.3);
      marker.position.set(c.estimate[0], c.estimate[2] + 0.05, c.estimate[1]);
      marker.scale.setScalar(pulse * (c.truth.kind === 'tp' ? 1 : 0.7));
    });
  }

  private syncTargets(t: number): void {
    for (const shell of this.shells) shell.targets.sync(resolveScene(shell.building.scene.targets, t), shell.building.scene.targets, t);
    if (this.outdoorTargets) this.outdoorTargets.sync(resolveScene(this.world.outdoorScene.targets, t), this.world.outdoorScene.targets, t);
  }

  /** World-space points the director frames, in Three coordinates. */
  shellCenter(id: string): THREE.Vector3 | null {
    const building = this.buildingById(id);
    if (!building) return null;
    const spec = building.shell.spec;
    const f = spec.floors[0]!;
    return new THREE.Vector3(
      building.shell.origin[0] + (f.w * spec.cellSize) / 2,
      building.shell.origin[2] + 1.5,
      building.shell.origin[1] + (f.d * spec.cellSize) / 2,
    );
  }

  shellSize(id: string): number {
    const building = this.buildingById(id);
    if (!building) return 10;
    const f = building.shell.spec.floors[0]!;
    return Math.max(f.w, f.d) * building.shell.spec.cellSize;
  }

  get timeSec(): number {
    return this.time;
  }
}

export function phaseColor(phase: string): string {
  switch (phase) {
    case 'coverage':
      return '#56e0ff';
    case 'approach':
    case 'inspect':
      return '#ffd166';
    case 'entry_select':
    case 'await_operator':
    case 'fly_through':
      return '#ff9f43';
    case 'indoor_explore':
    case 'confirm':
      return '#3dff9a';
    case 'exit':
    case 'return_home':
    case 'land':
      return '#c3a6ff';
    default:
      return '#ffffff';
  }
}
