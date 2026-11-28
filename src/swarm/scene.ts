import * as THREE from 'three';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { CELL } from '../shared/constants';
import { RadioModel, type Point3 } from '../team/radio';
import { buildDrone, type DroneRig } from '../viz/droneMesh';
import { buildBuilding } from '../viz/meshBuilding';
import { loadPbrSet } from '../viz/textures';
import { applyPbr, makeMaterials } from '../viz/three/materials';
import type { DroneSample, SwarmBundle, Track } from './film';

/** One colour per drone, shared with the HUD (swarm.css). */
export const DRONE_COLORS = ['#3ee0ff', '#ff5ca8', '#ffbe3d', '#9dff5c'] as const;
const BASE_COLOR = '#e8fbff';

/** Height the walls are cut at, m: below the ceiling, so the camera sees into every room, and above the drones. */
export const CUT_H = 1.55;
/** Height of the floor's top face (buildBuilding's ground slab), m; the tiles sit just above it. */
const FLOOR_TOP = 0.06;
/** Seconds of path each drone trails behind it. */
const TRAIL_S = 16;
/** Seconds between the team's map exchanges (TeamSim exchanges every 10 steps of 50 ms). */
const EXCHANGE_S = 0.5;

const base = import.meta.env.BASE_URL;

/** Sim x, y horizontal and z up becomes Three x, z horizontal and y up (as buildBuilding does). */
export function toThree(x: number, y: number, z: number, out = new THREE.Vector3()): THREE.Vector3 {
  return out.set(x, z, y);
}

function glowTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d')!;
  const grad = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.22, 'rgba(255,255,255,0.55)');
  grad.addColorStop(0.55, 'rgba(255,255,255,0.12)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 128, 128);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

/** A soft-edged rounded square: one coverage tile. */
function tileTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d')!;
  g.shadowColor = 'rgba(255,255,255,0.9)';
  g.shadowBlur = 6;
  g.fillStyle = 'rgba(255,255,255,0.55)';
  const r = 10;
  g.beginPath();
  g.roundRect(7, 7, 50, 50, r);
  g.fill();
  g.shadowBlur = 0;
  g.strokeStyle = 'rgba(255,255,255,1)';
  g.lineWidth = 3;
  g.stroke();
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

/** Dark ground with a 1 m grid and a brighter 5 m grid. */
function groundTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 512;
  const g = c.getContext('2d')!;
  g.fillStyle = '#20262d';
  g.fillRect(0, 0, 512, 512);
  // Some grain, so the ground reads as a surface and not as a flat colour.
  const img = g.getImageData(0, 0, 512, 512);
  let s = 1234567;
  for (let i = 0; i < img.data.length; i += 4) {
    s = (s * 1103515245 + 12345) >>> 0;
    const n = ((s >>> 16) & 31) - 16;
    img.data[i] = Math.max(0, img.data[i]! + n);
    img.data[i + 1] = Math.max(0, img.data[i + 1]! + n);
    img.data[i + 2] = Math.max(0, img.data[i + 2]! + n);
  }
  g.putImageData(img, 0, 0);
  g.strokeStyle = 'rgba(150,190,215,0.10)';
  g.lineWidth = 2;
  for (let i = 0; i <= 512; i += 102.4) {
    g.beginPath();
    g.moveTo(i, 0);
    g.lineTo(i, 512);
    g.moveTo(0, i);
    g.lineTo(512, i);
    g.stroke();
  }
  g.strokeStyle = 'rgba(150,200,230,0.22)';
  g.lineWidth = 3;
  g.strokeRect(0, 0, 512, 512);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  return t;
}

function fatLine(opts: { color?: string; width: number; opacity?: number; dashed?: boolean; dash?: number; vertexColors?: boolean; additive?: boolean }): Line2 {
  const mat = new LineMaterial({
    color: opts.color ?? '#ffffff',
    linewidth: opts.width,
    transparent: true,
    opacity: opts.opacity ?? 1,
    dashed: !!opts.dashed,
    dashSize: opts.dash ?? 0.3,
    gapSize: (opts.dash ?? 0.3) * 0.8,
    vertexColors: !!opts.vertexColors,
    depthWrite: false,
    blending: opts.additive ? THREE.AdditiveBlending : THREE.NormalBlending,
  });
  const line = new Line2(new LineGeometry(), mat);
  line.frustumCulled = false;
  line.renderOrder = 6;
  return line;
}

function setLine(line: Line2, pts: number[], colors?: number[]): void {
  if (pts.length < 6) {
    line.visible = false;
    return;
  }
  line.visible = true;
  const geo = new LineGeometry();
  geo.setPositions(pts);
  if (colors) geo.setColors(colors);
  line.geometry.dispose();
  line.geometry = geo;
  if ((line.material as LineMaterial).dashed) line.computeLineDistances();
}

interface DroneView {
  pivot: THREE.Group;
  rig: DroneRig;
  halo: THREE.Sprite;
  light: THREE.PointLight;
  spot: THREE.Mesh;
  trail: Line2;
  goalRing: THREE.Mesh;
  goalLine: Line2;
  homeRing: THREE.Mesh;
  color: THREE.Color;
}

interface LinkView {
  a: number;
  b: number;
  line: Line2;
  pulses: [THREE.Sprite, THREE.Sprite];
  phase: number;
}

export interface SwarmView {
  /** Sim time, s. */
  t: number;
  camera: THREE.PerspectiveCamera;
  width: number;
  height: number;
  /** 0 in close shots, 1 in the widest: how much the drones are drawn larger than life to stay readable. */
  wide: number;
}

/**
 * The swarm replay in 3D: the building with its ceilings off, one PX4-flown drone per colour, the
 * floor tiles each drone flew first lighting up in its colour, their trails and current goals, the
 * base station outside the door, and the radio links between everyone, brighter where a packet is
 * more likely to arrive. Every frame is a function of its time alone.
 */
export class SwarmScene {
  readonly scene = new THREE.Scene();
  readonly renderer: THREE.WebGLRenderer;
  /** Building footprint, m (sim x and y). */
  readonly W: number;
  readonly D: number;
  readonly H: number;
  /** Where each drone and the base were drawn on the last update, for the HUD labels. */
  readonly droneWorld: THREE.Vector3[];
  readonly baseWorld: THREE.Vector3;
  readonly samples: DroneSample[] = [];
  private readonly radio: RadioModel;
  private readonly drones: DroneView[] = [];
  private readonly links: LinkView[] = [];
  private tiles!: THREE.InstancedMesh;
  private readonly glow = glowTexture();
  private readonly tmpM = new THREE.Matrix4();
  private readonly tmpS = new THREE.Vector3();
  private readonly tmpP = new THREE.Vector3();
  private readonly tmpC = new THREE.Color();
  private readonly flat = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0));
  private readonly path: number[] = [];
  private beacon!: THREE.Sprite;
  private beaconLamp!: THREE.Mesh;

  constructor(
    canvas: HTMLCanvasElement,
    readonly bundle: SwarmBundle,
    readonly track: Track,
  ) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true, powerPreference: 'high-performance' });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.localClippingEnabled = true;
    const spec = bundle.spec;
    this.W = spec.floors[0]!.w * spec.cellSize;
    this.D = spec.floors[0]!.d * spec.cellSize;
    this.H = spec.floorHeight;
    this.radio = new RadioModel(spec);
    this.droneWorld = Array.from({ length: bundle.drones }, () => new THREE.Vector3());
    this.baseWorld = toThree(...bundle.base);
  }

  async load(): Promise<void> {
    const spec = this.bundle.spec;
    const size = Math.max(this.W, this.D);
    const cx = this.W / 2;
    const cz = this.D / 2;

    // Blue hour: a cool sky, a low warm key light, and haze that the ground fades into.
    const haze = new THREE.Color('#0c131a');
    this.scene.background = haze;
    this.scene.fog = new THREE.Fog(haze, size * 2.2, size * 6);
    this.scene.add(new THREE.HemisphereLight('#a9c3e6', '#231d19', 0.9));
    const key = new THREE.DirectionalLight('#ffd8ae', 2.1);
    key.position.set(cx - size * 0.9, size * 1.25, cz + size * 0.55);
    key.target.position.set(cx, 0, cz);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    const r = size * 0.85;
    Object.assign(key.shadow.camera, { left: -r, right: r, top: r, bottom: -r, near: 1, far: size * 4 });
    key.shadow.bias = -0.0004;
    key.shadow.normalBias = 0.03;
    this.scene.add(key, key.target);
    const fill = new THREE.DirectionalLight('#7ea2ff', 0.45);
    fill.position.set(cx + size, size * 0.6, cz - size);
    this.scene.add(fill);

    const gtex = groundTexture();
    const span = size * 14;
    gtex.repeat.set(span / 5, span / 5);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(span, span), new THREE.MeshStandardMaterial({ map: gtex, roughness: 0.96, metalness: 0.02 }));
    ground.rotation.x = -Math.PI / 2;
    ground.position.set(cx, -0.005, cz);
    ground.receiveShadow = true;
    this.scene.add(ground);

    const mats = makeMaterials(spec.seed, 'high');
    try {
      applyPbr(mats, await loadPbrSet(`${base}textures/`));
    } catch {
      /* the canvas textures stay */
    }
    if (mats.wall instanceof THREE.MeshStandardMaterial) mats.wall.color.set('#bdb6ab');
    // A section cut: everything above CUT_H is clipped away, and the cut faces are capped below.
    const cut = [new THREE.Plane(new THREE.Vector3(0, -1, 0), CUT_H)];
    for (const m of [mats.wall, mats.concrete, mats.glass, mats.metal, mats.frame, mats.door]) {
      m.clippingPlanes = cut;
      m.clipShadows = true;
    }
    const built = buildBuilding(spec, mats, true);
    // Ceilings, the roof slab and the room lights off: the camera looks in from above, and the
    // drones light their own surroundings.
    for (const c of built.ceilings) c.visible = false;
    for (const { light } of built.lights) light.visible = false;
    const roofY = spec.floors.length * this.H - 0.05;
    for (const child of built.root.children) if (child instanceof THREE.Mesh && child.position.y > roofY) child.visible = false;
    this.scene.add(built.root);
    this.buildCaps();

    this.buildTiles();
    this.buildBase();
    for (let d = 0; d < this.bundle.drones; d++) this.drones.push(this.buildDrone(d));
    const nodes = this.bundle.drones + 1;
    for (let a = 0; a < nodes; a++) {
      for (let b = a + 1; b < nodes; b++) {
        const line = fatLine({ width: 1.6, opacity: 0.6, additive: true });
        this.scene.add(line);
        const pulse = () => {
          const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: this.glow, color: '#ffffff', transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, fog: false }));
          s.renderOrder = 8;
          this.scene.add(s);
          return s;
        };
        this.links.push({ a, b, line, pulses: [pulse(), pulse()], phase: ((a * 7 + b * 3) % 10) / 10 });
      }
    }
  }

  /** Light caps on every cut wall, so from above the building reads as its floor plan. */
  private buildCaps(): void {
    const spec = this.bundle.spec;
    const cs = spec.cellSize;
    const cells: [number, number, number][] = [];
    spec.floors.forEach((floor, f) => {
      for (let y = 0; y < floor.d; y++) {
        for (let x = 0; x < floor.w; x++) {
          const c = floor.cells[y * floor.w + x];
          if (c === CELL.WALL || c === CELL.SHAFT) cells.push([(x + 0.5) * cs, (y + 0.5) * cs, f * spec.floorHeight + CUT_H]);
        }
      }
    });
    const geo = new THREE.PlaneGeometry(cs, cs);
    geo.rotateX(-Math.PI / 2);
    const caps = new THREE.InstancedMesh(geo, new THREE.MeshStandardMaterial({ color: '#c4bdb2', roughness: 0.9, metalness: 0 }), Math.max(1, cells.length));
    const m = new THREE.Matrix4();
    cells.forEach(([x, y, z], k) => caps.setMatrixAt(k, m.makeTranslation(x, z, y)));
    caps.count = cells.length;
    caps.receiveShadow = true;
    this.scene.add(caps);
  }

  private buildTiles(): void {
    const tr = this.track;
    const n = tr.tileT.length;
    const geo = new THREE.PlaneGeometry(tr.voxel * 0.98, tr.voxel * 0.98);
    const mat = new THREE.MeshBasicMaterial({ map: tileTexture(), transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false, fog: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
    this.tiles = new THREE.InstancedMesh(geo, mat, Math.max(1, n));
    this.tiles.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.tiles.setColorAt(0, new THREE.Color());
    this.tiles.instanceColor!.setUsage(THREE.DynamicDrawUsage);
    this.tiles.count = 0;
    this.tiles.frustumCulled = false;
    this.tiles.renderOrder = 3;
    this.scene.add(this.tiles);
  }

  private buildBase(): void {
    const g = new THREE.Group();
    const metal = new THREE.MeshStandardMaterial({ color: '#9aa6b2', roughness: 0.4, metalness: 0.8 });
    const dark = new THREE.MeshStandardMaterial({ color: '#1d242c', roughness: 0.6, metalness: 0.3 });
    const h = this.bundle.base[2];
    for (let k = 0; k < 3; k++) {
      const a = (k / 3) * Math.PI * 2;
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.022, h * 1.08, 8), metal);
      leg.position.set(Math.cos(a) * 0.22, h * 0.5, Math.sin(a) * 0.22);
      leg.rotation.set(Math.sin(a) * 0.22, 0, -Math.cos(a) * 0.22);
      leg.castShadow = true;
      g.add(leg);
    }
    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.9, 10), metal);
    mast.position.y = h + 0.35;
    mast.castShadow = true;
    g.add(mast);
    const box = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.22, 0.26), dark);
    box.position.y = h;
    box.castShadow = true;
    g.add(box);
    for (const s of [-1, 1]) {
      const ant = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.012, 0.42, 8), dark);
      ant.position.set(s * 0.1, h + 0.92, 0);
      ant.rotation.z = s * 0.18;
      g.add(ant);
    }
    this.beaconLamp = new THREE.Mesh(new THREE.SphereGeometry(0.05, 16, 12), new THREE.MeshBasicMaterial({ color: BASE_COLOR, toneMapped: false }));
    this.beaconLamp.position.y = h + 0.82;
    g.add(this.beaconLamp);
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.7, 0.76, 64), new THREE.MeshBasicMaterial({ color: BASE_COLOR, transparent: true, opacity: 0.55, depthWrite: false, toneMapped: false, side: THREE.DoubleSide }));
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.02;
    g.add(ring);
    const [bx, by] = this.bundle.base;
    g.position.set(bx, 0, by);
    this.scene.add(g);
    this.beacon = new THREE.Sprite(new THREE.SpriteMaterial({ map: this.glow, color: BASE_COLOR, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, fog: false }));
    this.beacon.position.set(bx, h + 0.82, by);
    this.beacon.renderOrder = 7;
    this.scene.add(this.beacon);
  }

  private buildDrone(d: number): DroneView {
    const color = new THREE.Color(DRONE_COLORS[d % DRONE_COLORS.length]);
    const rig = buildDrone();
    rig.group.traverse((o) => {
      if (o instanceof THREE.Mesh) o.castShadow = true;
    });
    const pivot = new THREE.Group();
    pivot.add(rig.group);
    // A coloured band under the body, so each airframe is recognisable up close.
    const band = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.008, 0.095), new THREE.MeshBasicMaterial({ color, toneMapped: false }));
    band.position.y = -0.018;
    rig.group.add(band);
    this.scene.add(pivot);
    const halo = new THREE.Sprite(new THREE.SpriteMaterial({ map: this.glow, color, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, fog: false }));
    halo.renderOrder = 7;
    this.scene.add(halo);
    const light = new THREE.PointLight(color, 5, 5.5, 1.6);
    this.scene.add(light);
    const spot = new THREE.Mesh(
      new THREE.CircleGeometry(1, 40),
      new THREE.MeshBasicMaterial({ map: this.glow, color, transparent: true, opacity: 0.6, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false, fog: false }),
    );
    spot.rotation.x = -Math.PI / 2;
    spot.renderOrder = 4;
    this.scene.add(spot);
    const trail = fatLine({ width: 3.2, vertexColors: true, additive: true });
    this.scene.add(trail);
    const goalRing = new THREE.Mesh(
      new THREE.RingGeometry(0.2, 0.27, 48),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, depthWrite: false, toneMapped: false, side: THREE.DoubleSide, blending: THREE.AdditiveBlending }),
    );
    goalRing.rotation.x = -Math.PI / 2;
    goalRing.renderOrder = 6;
    this.scene.add(goalRing);
    const goalLine = fatLine({ color: `#${color.getHexString()}`, width: 1.6, opacity: 0.75, dashed: true, dash: 0.22, additive: true });
    this.scene.add(goalLine);
    const homeRing = new THREE.Mesh(
      new THREE.RingGeometry(0.34, 0.4, 48),
      new THREE.MeshBasicMaterial({ color: '#ffd36b', transparent: true, opacity: 0.9, depthWrite: false, toneMapped: false, side: THREE.DoubleSide }),
    );
    homeRing.rotation.x = -Math.PI / 2;
    homeRing.renderOrder = 6;
    this.scene.add(homeRing);
    return { pivot, rig, halo, light, spot, trail, goalRing, goalLine, homeRing, color };
  }

  /** Pose everything for one moment. */
  update(view: SwarmView): void {
    const { t, camera } = view;
    const tr = this.track;
    for (const d of this.drones) (d.trail.material as LineMaterial).resolution.set(view.width, view.height);
    for (const d of this.drones) (d.goalLine.material as LineMaterial).resolution.set(view.width, view.height);
    for (const l of this.links) (l.line.material as LineMaterial).resolution.set(view.width, view.height);

    // Coverage tiles: each one pops in at the moment it was first flown, flashes, and settles.
    const count = tr.tilesUpTo(t);
    this.tiles.count = count;
    for (let k = 0; k < count; k++) {
      const age = t - tr.tileT[k]!;
      const pop = Math.min(1, age / 0.35);
      const s = 0.55 + 0.45 * (1 - Math.pow(1 - pop, 3)) + 0.12 * Math.exp(-age / 0.25);
      this.tmpS.set(s, s, s);
      this.tmpP.set(tr.tileX[k]!, tr.tileZ[k]! + FLOOR_TOP + 0.015, tr.tileY[k]!);
      this.tmpM.compose(this.tmpP, this.flat, this.tmpS);
      this.tiles.setMatrixAt(k, this.tmpM);
      const bright = 0.42 + 1.9 * Math.exp(-age / 0.55);
      this.tmpC.set(DRONE_COLORS[tr.tileDrone[k]! % DRONE_COLORS.length]!).multiplyScalar(bright);
      this.tiles.setColorAt(k, this.tmpC);
    }
    this.tiles.instanceMatrix.needsUpdate = true;
    if (this.tiles.instanceColor) this.tiles.instanceColor.needsUpdate = true;

    this.samples.length = 0;
    for (let i = 0; i < this.drones.length; i++) {
      const v = this.drones[i]!;
      const s = tr.sample(t, i);
      this.samples.push(s);
      const p = toThree(s.x, s.y, s.z, this.droneWorld[i]);
      // Larger than life when the camera is far, so a 0.3 m drone still reads in a 40 m shot.
      const dist = camera.position.distanceTo(p);
      const scale = THREE.MathUtils.clamp(1 + (dist / 7 - 1) * view.wide, 1, 3.2);
      v.pivot.position.copy(p);
      v.pivot.rotation.set(0, -s.yaw, 0);
      v.pivot.scale.setScalar(scale);
      v.rig.group.rotation.set(-s.roll, 0, -s.pitch);
      v.rig.rotors.forEach((r, k) => (r.rotation.y = (k % 2 ? -1 : 1) * s.rotor + k));
      const thrust = (s.motors[0] + s.motors[1] + s.motors[2] + s.motors[3]) / 4;
      v.rig.updateWash(t, s.mode === 'waiting' ? thrust * 0.6 : thrust);
      const active = s.mode === 'searching' || s.mode === 'returning';
      const dim = s.mode === 'waiting' ? 0.35 : s.mode === 'done' ? 0.25 : 1;
      v.halo.position.copy(p);
      v.halo.scale.setScalar((0.5 + 0.38 * scale) * (0.8 + 0.2 * dim));
      v.halo.material.opacity = 0.75 * dim;
      v.light.position.set(p.x, p.y - 0.12, p.z);
      v.light.intensity = 5 * dim;
      v.spot.position.set(p.x, Math.floor(s.z / this.H) * this.H + FLOOR_TOP + 0.025, p.z);
      v.spot.scale.setScalar(0.55 + 0.25 * scale);
      (v.spot.material as THREE.MeshBasicMaterial).opacity = 0.55 * dim;

      // Trail: the last TRAIL_S seconds, fading to nothing at its tail.
      if (s.mode !== 'waiting') {
        const t0 = Math.max(0, t - TRAIL_S);
        const pts = tr.path(i, t0, t, this.path);
        const out: number[] = [];
        const col: number[] = [];
        const m = pts.length / 3;
        for (let k = 0; k < m; k++) {
          out.push(pts[k * 3]!, pts[k * 3 + 2]! - 0.04, pts[k * 3 + 1]!);
          const f = Math.pow(k / Math.max(1, m - 1), 1.6) * (s.mode === 'done' ? 0.4 : 1);
          col.push(v.color.r * f, v.color.g * f, v.color.b * f);
        }
        setLine(v.trail, out, col);
      } else v.trail.visible = false;

      // The goal the drone is flying to, at flight height.
      if (s.goal && active) {
        const pulse = 1 + 0.12 * Math.sin(t * 6 + i);
        v.goalRing.visible = true;
        v.goalRing.position.set(s.goal[0], s.z, s.goal[1]);
        v.goalRing.scale.setScalar(pulse * (0.8 + 0.2 * scale));
        setLine(v.goalLine, [p.x, p.y, p.z, s.goal[0], s.z, s.goal[1]]);
      } else {
        v.goalRing.visible = false;
        v.goalLine.visible = false;
      }
      v.homeRing.visible = s.mode === 'returning';
      if (v.homeRing.visible) {
        v.homeRing.position.set(p.x, p.y - 0.16 * scale, p.z);
        v.homeRing.scale.setScalar(scale * (1 + 0.1 * Math.sin(t * 8)));
      }
    }

    // Radio: a line for every pair that can hear each other, brighter where a packet is more
    // likely to get through, and a packet pulse each way at every exchange.
    const lamp = 0.65 + 0.35 * Math.max(0, Math.sin(t * 2 * Math.PI * 0.8));
    this.beacon.scale.setScalar(1.3 + 0.5 * lamp);
    this.beacon.material.opacity = 0.5 + 0.4 * lamp;
    const pos = (k: number): Point3 | null => {
      if (k === 0) return this.bundle.base;
      const s = this.samples[k - 1]!;
      return s.mode === 'searching' || s.mode === 'returning' ? [s.x, s.y, s.z] : null;
    };
    for (const l of this.links) {
      const a = pos(l.a);
      const b = pos(l.b);
      const p = a && b ? this.radio.deliveryProbability(a, b) : 0;
      if (!a || !b || p < 0.03) {
        l.line.visible = false;
        l.pulses[0].visible = l.pulses[1].visible = false;
        continue;
      }
      const A = toThree(a[0], a[1], a[2]);
      const B = toThree(b[0], b[1], b[2]);
      this.tmpC.set(linkColor(p));
      setLine(l.line, [A.x, A.y, A.z, B.x, B.y, B.z]);
      const mat = l.line.material as LineMaterial;
      mat.color.copy(this.tmpC);
      mat.opacity = 0.12 + 0.5 * p;
      const ph = ((t / EXCHANGE_S + l.phase) % 1 + 1) % 1;
      const fade = Math.sin(Math.PI * ph);
      for (let dir = 0; dir < 2; dir++) {
        const sp = l.pulses[dir]!;
        sp.visible = true;
        const w = dir === 0 ? ph : 1 - ph;
        sp.position.lerpVectors(A, B, w);
        sp.scale.setScalar(0.42 + 0.25 * view.wide);
        sp.material.color.copy(this.tmpC);
        sp.material.opacity = fade * (0.25 + 0.75 * p);
      }
    }
  }
}

/** Red where a packet rarely arrives, amber at even odds, mint when it nearly always does. */
export function linkColor(p: number): string {
  const c = new THREE.Color('#ff5a5a').lerp(new THREE.Color('#ffc44d'), Math.min(1, p * 2));
  if (p > 0.5) c.lerp(new THREE.Color('#6dffcf'), (p - 0.5) * 2);
  return `#${c.getHexString()}`;
}
