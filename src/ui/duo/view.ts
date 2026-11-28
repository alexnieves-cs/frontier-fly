import * as THREE from 'three';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import type { MissionWorld, RunnerSnapshot, V3 } from '../../mission/autonomy/contracts';
import { TargetView } from '../../targets/meshes';
import { resolveScene } from '../../targets/motion';
import { buildDrone, type DroneRig } from '../../viz/droneMesh';
import { applyPbr, makeMaterials } from '../../viz/three/materials';
import { SiteDressing, type SiteShell } from '../../viz/site/site';
import { loadPbrSet } from '../../viz/textures';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { Opening } from '../../world/outdoor/types';

export type DuoCamera = 'overview' | 'A' | 'B';

/** Drone colours, shared with the page's cards. */
export const DRONE_COLOURS = { A: '#38e8cc', B: '#ffb347' } as const;
export type DroneName = keyof typeof DRONE_COLOURS;

/** Drones are drawn at this multiple of life size so they read from the overview camera. */
export const DRONE_DRAW_SCALE = 1.6;

const TRAIL_POINTS = 8000;
/** Sim seconds between trail samples. */
const TRAIL_STEP_SEC = 0.15;

interface DroneDraw {
  rig: DroneRig;
  /** The drone's letter in a ring, a constant size on screen, so it reads from the overview. */
  tag: THREE.Sprite;
  trail: Line2;
  points: number[];
  lastTrailT: number;
  dirty: boolean;
  spin: number;
}

interface ShellDraw {
  id: string;
  built: SiteShell;
  targets: TargetView;
  scene: MissionWorld['buildings'][number]['scene'];
}

const base = import.meta.env.BASE_URL;

/** Three.js position for a sim point (sim z is up, Three y is up). */
function three(p: readonly number[]): THREE.Vector3 {
  return new THREE.Vector3(p[0]!, p[2]!, p[1]!);
}

/**
 * The Two ways in view: the site the drones fly (SiteDressing, the same as the demo and the film),
 * both drones with their trails, each passable opening outlined in the colour of the drone that
 * went in by it, and the confirmed contact. Roofs lift while a drone is inside, so the camera sees in.
 */
export class DuoView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(52, 1, 0.08, 3000);
  private readonly soft: boolean;
  private site: SiteDressing | null = null;
  private world: MissionWorld | null = null;
  private readonly drones = new Map<DroneName, DroneDraw>();
  private readonly shells: ShellDraw[] = [];
  private readonly openingFrames = new Map<string, THREE.LineSegments>();
  private readonly contacts = new THREE.Group();
  private readonly overlay = new THREE.Group();
  private mode: DuoCamera = 'overview';
  private readonly camPos = new THREE.Vector3();
  private readonly camLook = new THREE.Vector3();
  private camReady = false;

  constructor(canvas: HTMLCanvasElement) {
    const quality = new URLSearchParams(location.search).get('quality');
    this.soft = quality === 'high' ? false : quality === 'low' ? true : softwareWebgl();
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: !this.soft, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(this.soft ? 0.6 : Math.min(window.devicePixelRatio || 1, 1.5));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 0.9;
    this.renderer.shadowMap.enabled = !this.soft;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.shadowMap.autoUpdate = false;
    this.scene.add(this.contacts, this.overlay);
    this.resize();
  }

  setCamera(mode: DuoCamera): void {
    this.mode = mode;
    this.camReady = false;
  }

  /** Build the site, both drones and the opening outlines for one flight. */
  async setWorld(world: MissionWorld): Promise<void> {
    this.clear();
    this.world = world;
    const site = new SiteDressing(this.renderer, this.scene, world.outdoor, {
      seed: world.outdoor.seed,
      timeOfDay: world.timeOfDay,
      quality: this.soft ? 'low' : 'high',
    });
    site.build();
    this.site = site;
    this.renderer.toneMappingExposure = site.night ? 1.25 : 0.9;
    const mats = makeMaterials(world.outdoor.seed, this.soft ? 'low' : 'high');
    for (const building of world.buildings) {
      const built = site.buildShell(building.shell, mats);
      const light = new THREE.PointLight('#ffd7ae', 7, Math.max(built.width, built.depth) * 1.4, 1.4);
      light.position.set(built.width / 2, 2.1, built.depth / 2);
      built.root.add(light);
      const targets = new TargetView();
      built.root.add(targets.group);
      this.shells.push({ id: building.id, built, targets, scene: building.scene });
      for (const opening of building.shell.openings) {
        if (!opening.passable) continue;
        const frame = openingFrame(opening);
        this.openingFrames.set(`${building.id}:${opening.id}`, frame);
        this.overlay.add(frame);
      }
    }
    for (const name of ['A', 'B'] as const) {
      const rig = buildDrone();
      rig.group.scale.setScalar(DRONE_DRAW_SCALE);
      rig.group.traverse((o) => (o.castShadow = true));
      const tag = new THREE.Sprite(
        new THREE.SpriteMaterial({ map: tagTexture(name), transparent: true, depthTest: false, depthWrite: false, sizeAttenuation: false, toneMapped: false }),
      );
      tag.scale.setScalar(0.032);
      tag.center.set(0.5, -0.35);
      tag.renderOrder = 30;
      const material = new LineMaterial({ color: DRONE_COLOURS[name], linewidth: 3, transparent: true, opacity: 0.92, toneMapped: false });
      material.resolution.set(this.renderer.domElement.width, this.renderer.domElement.height);
      const trail = new Line2(new LineGeometry(), material);
      trail.frustumCulled = false;
      trail.visible = false;
      trail.renderOrder = 19;
      this.scene.add(rig.group, tag, trail);
      this.drones.set(name, { rig, tag, trail, points: [], lastTrailT: -Infinity, dirty: false, spin: 0 });
    }
    await Promise.all(this.shells.map((s) => s.targets.wait()));
    this.renderer.shadowMap.needsUpdate = true;
    this.renderer.compile(this.scene, this.camera);
    if (!this.soft) {
      const set = await loadPbrSet(`${base}textures/`).catch(() => null);
      if (set && this.site === site) {
        applyPbr(mats, set);
        this.renderer.shadowMap.needsUpdate = true;
      }
    }
  }

  /** Outline an opening in the colour of the drone that went in by it. */
  markEntry(buildingId: string, openingId: number, name: DroneName): void {
    const frame = this.openingFrames.get(`${buildingId}:${openingId}`);
    if (!frame) return;
    const material = frame.material as THREE.LineBasicMaterial;
    material.color.set(DRONE_COLOURS[name]);
    material.opacity = 1;
  }

  /** A ring where a drone confirmed a contact. */
  markContact(pos: V3, name: DroneName): void {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.55, 0.7, 48),
      new THREE.MeshBasicMaterial({ color: DRONE_COLOURS[name], side: THREE.DoubleSide, transparent: true, opacity: 0.95, depthTest: false, toneMapped: false }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.copy(three(pos)).setY(pos[2] + 0.05);
    ring.renderOrder = 22;
    this.contacts.add(ring);
  }

  /** Draw one frame. `t` is team time, s. */
  render(t: number, snaps: Record<DroneName, RunnerSnapshot>, flying: Record<DroneName, boolean>): void {
    const world = this.world;
    if (!world) return;
    for (const name of ['A', 'B'] as const) {
      const d = this.drones.get(name)!;
      const snap = snaps[name];
      const p = snap.pose.pos;
      d.rig.group.position.copy(three(p));
      d.rig.group.rotation.set(0, -snap.pose.yaw, 0);
      d.spin += flying[name] ? 1.4 : 0;
      for (const rotor of d.rig.rotors) rotor.rotation.y = d.spin;
      d.tag.position.copy(three(p)).setY(p[2] + 0.25);
      d.tag.visible = this.mode !== name;
      if (flying[name]) this.trailPoint(name, p, t);
      if (d.dirty && d.points.length >= 6) {
        // LineGeometry keeps a fixed buffer per setPositions, so rebuild it once per frame as the trail grows.
        d.trail.geometry.dispose();
        const geometry = new LineGeometry();
        geometry.setPositions(d.points);
        d.trail.geometry = geometry;
        d.trail.computeLineDistances();
        d.trail.visible = true;
        d.dirty = false;
      }
    }
    // Roofs and ceilings lift while a drone is in, or passing through an opening of, the building.
    for (const shell of this.shells) {
      const open = (['A', 'B'] as const).some((n) => snaps[n].inside === shell.id || (snaps[n].phase === 'fly_through' && near(shell, snaps[n].pose.pos)));
      for (const roof of shell.built.roof) roof.visible = !open;
      for (const ceiling of shell.built.ceilings) ceiling.visible = !open;
      shell.targets.sync(resolveScene(shell.scene.targets, t), shell.scene.targets, t);
    }
    for (const ring of this.contacts.children) ring.scale.setScalar(1 + 0.12 * Math.sin(t * 3));
    this.placeCamera(world, t, snaps);
    this.renderer.render(this.scene, this.camera);
  }

  resize(): void {
    const canvas = this.renderer.domElement;
    const width = canvas.clientWidth || 1;
    const height = canvas.clientHeight || 1;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    for (const d of this.drones.values()) (d.trail.material as LineMaterial).resolution.set(canvas.width, canvas.height);
  }

  /** Add a point to a drone's trail (kept every TRAIL_STEP_SEC of team time). */
  trailPoint(name: DroneName, p: V3, t: number): void {
    const d = this.drones.get(name);
    if (!d || t - d.lastTrailT < TRAIL_STEP_SEC || d.points.length >= TRAIL_POINTS * 3) return;
    d.points.push(p[0], p[2], p[1]);
    d.lastTrailT = t;
    d.dirty = true;
  }

  /** Jump the damped camera straight to its target on the next frame (for stills). */
  snapCamera(): void {
    this.camReady = false;
  }

  dispose(): void {
    this.clear();
    this.renderer.dispose();
  }

  private placeCamera(world: MissionWorld, t: number, snaps: Record<DroneName, RunnerSnapshot>): void {
    let want: THREE.Vector3;
    let look: THREE.Vector3;
    if (this.mode === 'overview') {
      // Orbit the midpoint of the two drones and the nearest building, far enough to hold all three.
      const a = snaps.A.pose.pos;
      const b = snaps.B.pose.pos;
      const building = world.buildings[0];
      const n = building?.footprint.length ?? 0;
      const c = building && n ? building.footprint.reduce((acc, q) => [acc[0]! + q.x / n, acc[1]! + q.y / n], [0, 0]) : [a[0], a[1]];
      const mid = [(a[0] + b[0] + c[0]!) / 3, (a[1] + b[1] + c[1]!) / 3];
      const spread = Math.max(Math.hypot(a[0] - c[0]!, a[1] - c[1]!), Math.hypot(b[0] - c[0]!, b[1] - c[1]!), Math.hypot(a[0] - b[0], a[1] - b[1]));
      const radius = Math.max(16, spread * 1.05 + 7);
      const angle = 3.6 + t * 0.012;
      const ground = sampleHeight(world.outdoor.terrain, mid[0]!, mid[1]!);
      want = new THREE.Vector3(mid[0]! + Math.cos(angle) * radius, ground + radius * 0.68, mid[1]! + Math.sin(angle) * radius);
      look = new THREE.Vector3(mid[0]!, ground + 2.5, mid[1]!);
    } else {
      const snap = snaps[this.mode];
      const p = snap.pose.pos;
      const nx = Math.cos(snap.pose.yaw);
      const ny = Math.sin(snap.pose.yaw);
      // Indoors the roof is lifted, so follow from high behind and look down past the wall tops.
      const inside = snap.inside !== null;
      const back = inside ? 3.4 : 5.2;
      const up = inside ? 6 : 2.1;
      const x = p[0] - nx * back;
      const y = p[1] - ny * back;
      const z = Math.max(p[2] + up, sampleHeight(world.outdoor.terrain, x, y) + 0.7);
      want = new THREE.Vector3(x, z, y);
      look = inside ? new THREE.Vector3(p[0] + nx * 0.8, p[2] - 0.6, p[1] + ny * 0.8) : new THREE.Vector3(p[0] + nx * 2.5, p[2] + 0.3, p[1] + ny * 2.5);
    }
    if (!this.camReady || this.camPos.distanceTo(want) > 40) {
      this.camPos.copy(want);
      this.camLook.copy(look);
      this.camReady = true;
    } else {
      this.camPos.lerp(want, 0.06);
      this.camLook.lerp(look, 0.12);
    }
    this.camera.position.copy(this.camPos);
    this.camera.lookAt(this.camLook);
  }

  private clear(): void {
    this.site?.dispose();
    this.site = null;
    for (const d of this.drones.values()) {
      this.scene.remove(d.rig.group, d.tag, d.trail);
      d.trail.geometry.dispose();
    }
    this.drones.clear();
    for (const frame of this.openingFrames.values()) {
      this.overlay.remove(frame);
      frame.geometry.dispose();
    }
    this.openingFrames.clear();
    this.contacts.clear();
    this.shells.length = 0;
    this.camReady = false;
  }
}

function near(shell: ShellDraw, p: V3): boolean {
  const o = shell.built.root.position;
  return p[0] > o.x - 2 && p[0] < o.x + shell.built.width + 2 && p[1] > o.z - 2 && p[1] < o.z + shell.built.depth + 2;
}

/** A thin rectangle just outside a passable opening; white until a drone goes in by it. */
function openingFrame(opening: Opening): THREE.LineSegments {
  const n = new THREE.Vector3(opening.normal[0], opening.normal[2], opening.normal[1]);
  const u = new THREE.Vector3(opening.tangent[0], opening.tangent[2], opening.tangent[1]).normalize();
  const v = new THREE.Vector3().crossVectors(n, u).normalize();
  if (v.y < 0 || (Math.abs(v.y) < 1e-6 && v.z < 0)) v.negate();
  const c = three(opening.center).addScaledVector(n, 0.06);
  const hw = opening.width / 2 + 0.06;
  const hh = opening.height / 2 + 0.06;
  const corners = [
    c.clone().addScaledVector(u, -hw).addScaledVector(v, -hh),
    c.clone().addScaledVector(u, hw).addScaledVector(v, -hh),
    c.clone().addScaledVector(u, hw).addScaledVector(v, hh),
    c.clone().addScaledVector(u, -hw).addScaledVector(v, hh),
  ];
  const points: THREE.Vector3[] = [];
  for (let i = 0; i < 4; i++) points.push(corners[i]!, corners[(i + 1) % 4]!);
  const frame = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({ color: '#ffffff', transparent: true, opacity: 0.45, depthTest: false, toneMapped: false }),
  );
  frame.renderOrder = 18;
  return frame;
}

/** A filled ring in the drone's colour with its letter, drawn once per drone. */
function tagTexture(name: DroneName): THREE.CanvasTexture {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 128;
  const g = canvas.getContext('2d')!;
  g.fillStyle = 'rgba(5, 10, 14, 0.78)';
  g.beginPath();
  g.arc(64, 64, 56, 0, Math.PI * 2);
  g.fill();
  g.lineWidth = 10;
  g.strokeStyle = DRONE_COLOURS[name];
  g.stroke();
  g.fillStyle = DRONE_COLOURS[name];
  g.font = '700 64px Inter, Segoe UI, Helvetica, Arial, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(name, 64, 68);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
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
