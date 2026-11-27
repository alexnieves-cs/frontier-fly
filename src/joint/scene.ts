import * as THREE from 'three';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { buildDrone, type DroneRig } from '../viz/droneMesh';
import { dist, epochAt, solutionAt, spoofedAt, truthAt, type LegBundle } from './leg';

/** Colours shared with the HUD (joint.css). */
export const LEG_COLORS = {
  truth: '#f4f7fa',
  dss: '#56e0ff',
  gps: '#ff4d5e',
  fix: '#3dff9a',
  reject: '#ffb547',
  alert: '#ffd166',
} as const;

/** Seconds a fix stays lit on the ground after it is matched. */
const FLASH_S = 0.9;
/** Height the ground marks float at, m, so they never fight the photo for depth. */
const MARK_H = 0.35;

function fatLine(color: string, width: number, opts: { dashed?: boolean; opacity?: number; dash?: number } = {}): Line2 {
  const mat = new LineMaterial({
    color,
    linewidth: width,
    transparent: true,
    opacity: opts.opacity ?? 1,
    dashed: !!opts.dashed,
    dashSize: opts.dash ?? 2,
    gapSize: (opts.dash ?? 2) * 0.8,
    depthWrite: false,
  });
  const line = new Line2(new LineGeometry(), mat);
  line.frustumCulled = false;
  line.renderOrder = 5;
  return line;
}

function setLine(line: Line2, pts: number[]): void {
  if (pts.length < 6) {
    line.visible = false;
    return;
  }
  line.visible = true;
  const geo = new LineGeometry();
  geo.setPositions(pts);
  line.geometry.dispose();
  line.geometry = geo;
  if ((line.material as LineMaterial).dashed) line.computeLineDistances();
}

function discTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d')!;
  const grad = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.35, 'rgba(255,255,255,0.65)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 128, 128);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function scanTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d')!;
  g.fillStyle = 'rgba(86,224,255,0.10)';
  g.fillRect(0, 0, 256, 256);
  g.strokeStyle = 'rgba(86,224,255,0.55)';
  g.lineWidth = 2;
  for (let i = 0; i <= 256; i += 32) {
    g.beginPath();
    g.moveTo(i, 0);
    g.lineTo(i, 256);
    g.moveTo(0, i);
    g.lineTo(256, i);
    g.stroke();
  }
  g.strokeStyle = 'rgba(255,255,255,0.9)';
  g.lineWidth = 6;
  g.strokeRect(3, 3, 250, 250);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

export interface LegView {
  /** Leg time, s (may run before 0 and past the end for the intro and outro). */
  t: number;
  camera: THREE.PerspectiveCamera;
  /** Pixel size of the render target, for the fat lines. */
  width: number;
  height: number;
}

/**
 * The outdoor leg in 3D: the camera-year orthophoto as the ground, the drone at its true pose, the
 * camera's footprint and the patch it matches, and on the ground the true path, the spoofed GPS
 * track and DSS's estimate with its protection level and the 10 m alert limit. Everything is a
 * function of time, so any frame can be rendered on its own.
 */
export class LegScene {
  readonly scene = new THREE.Scene();
  readonly renderer: THREE.WebGLRenderer;
  private readonly cx: number;
  private readonly cy: number;
  private drone!: DroneRig;
  private ghost!: DroneRig;
  private readonly trails: Record<'truth' | 'dss' | 'gps', Line2>;
  private readonly tether: Line2;
  private readonly plRing: THREE.Mesh;
  private readonly plFill: THREE.Mesh;
  private readonly alert: Line2;
  private readonly footFull: Line2;
  private readonly frustum: Line2[];
  private readonly patch: THREE.Mesh;
  private readonly patchEdge: Line2;
  private readonly fixDots: THREE.InstancedMesh;
  private readonly ripples: THREE.Mesh[] = [];
  private readonly dssDot: THREE.Mesh;
  private readonly gpsDot: THREE.Mesh;
  private readonly shadow: THREE.Mesh;
  /** Screen-sized glows on the drone and its ghost, so both read in the wide shots. */
  private readonly beacon: THREE.Sprite;
  private readonly ghostBeacon: THREE.Sprite;
  /** Drone draw scale chosen on the last update (1 = true size). */
  droneScale = 1;
  /** Where the drone and its GPS ghost were drawn on the last update (ghost null when hidden), for the HUD's labels. */
  readonly droneWorld = new THREE.Vector3();
  ghostWorld: THREE.Vector3 | null = null;

  constructor(
    canvas: HTMLCanvasElement,
    readonly leg: LegBundle,
  ) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    const ext = leg.ground.extent;
    this.cx = (ext.xmin + ext.xmax) / 2;
    this.cy = (ext.ymin + ext.ymax) / 2;

    // Background and fog share one colour, so the ground fades into haze at the horizon instead of ending.
    const haze = new THREE.Color('#1a2833');
    this.scene.background = haze;
    this.scene.fog = new THREE.Fog(haze, 1400, 5200);
    this.scene.add(new THREE.HemisphereLight('#dfefff', '#3a3226', 1.3));
    const sun = new THREE.DirectionalLight('#fff2dc', 2.4);
    sun.position.set(-400, 900, 300);
    this.scene.add(sun);

    this.trails = {
      truth: fatLine(LEG_COLORS.truth, 2.2, { opacity: 0.85 }),
      dss: fatLine(LEG_COLORS.dss, 3.2),
      gps: fatLine(LEG_COLORS.gps, 3.2),
    };
    Object.values(this.trails).forEach((l) => this.scene.add(l));
    this.tether = fatLine('#ffffff', 1.4, { opacity: 0.7, dashed: true, dash: 0.8 });
    this.scene.add(this.tether);

    const ringGeo = new THREE.RingGeometry(0.93, 1, 96);
    ringGeo.rotateX(-Math.PI / 2);
    this.plRing = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color: LEG_COLORS.dss, transparent: true, opacity: 0.95, depthWrite: false, fog: false }));
    this.plRing.renderOrder = 6;
    const fillGeo = new THREE.CircleGeometry(1, 96);
    fillGeo.rotateX(-Math.PI / 2);
    this.plFill = new THREE.Mesh(fillGeo, new THREE.MeshBasicMaterial({ color: LEG_COLORS.dss, transparent: true, opacity: 0.14, depthWrite: false, fog: false }));
    this.plFill.renderOrder = 4;
    this.scene.add(this.plRing, this.plFill);
    this.alert = fatLine(LEG_COLORS.alert, 2.4, { dashed: true, dash: 1.6 });
    this.scene.add(this.alert);

    this.footFull = fatLine(LEG_COLORS.dss, 1.3, { opacity: 0.55 });
    this.scene.add(this.footFull);
    this.frustum = Array.from({ length: 4 }, () => {
      const l = fatLine(LEG_COLORS.dss, 1.2, { opacity: 0.38 });
      this.scene.add(l);
      return l;
    });
    const patchGeo = new THREE.PlaneGeometry(1, 1);
    patchGeo.rotateX(-Math.PI / 2);
    this.patch = new THREE.Mesh(
      patchGeo,
      new THREE.MeshBasicMaterial({ map: scanTexture(), transparent: true, opacity: 0.5, depthWrite: false, fog: false, blending: THREE.AdditiveBlending }),
    );
    this.patch.renderOrder = 3;
    this.scene.add(this.patch);
    this.patchEdge = fatLine('#ffffff', 2.2, { opacity: 0.9 });
    this.scene.add(this.patchEdge);

    const dotGeo = new THREE.CircleGeometry(1, 20);
    dotGeo.rotateX(-Math.PI / 2);
    this.fixDots = new THREE.InstancedMesh(dotGeo, new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.9, depthWrite: false, fog: false }), leg.epochs.length);
    this.fixDots.renderOrder = 4;
    this.fixDots.frustumCulled = false;
    const m = new THREE.Matrix4();
    leg.epochs.forEach((e, i) => {
      const p = e.fix.xy ?? e.truth;
      m.compose(this.w(p[0], p[1], MARK_H * 0.5), new THREE.Quaternion(), new THREE.Vector3(1.1, 1, 1.1));
      this.fixDots.setMatrixAt(i, m);
      this.fixDots.setColorAt(i, new THREE.Color(e.fix.used ? LEG_COLORS.fix : e.fix.xy ? LEG_COLORS.reject : '#6b7780'));
    });
    this.scene.add(this.fixDots);

    const glow = discTexture();
    for (let i = 0; i < 6; i++) {
      const g = new THREE.RingGeometry(0.82, 1, 64);
      g.rotateX(-Math.PI / 2);
      const r = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color: LEG_COLORS.fix, transparent: true, opacity: 0, depthWrite: false, fog: false }));
      r.renderOrder = 7;
      this.ripples.push(r);
      this.scene.add(r);
    }
    const dot = (color: string, size: number) => {
      const geo = new THREE.PlaneGeometry(size, size);
      geo.rotateX(-Math.PI / 2);
      const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ map: glow, color, transparent: true, depthWrite: false, fog: false, blending: THREE.AdditiveBlending }));
      mesh.renderOrder = 8;
      this.scene.add(mesh);
      return mesh;
    };
    this.dssDot = dot(LEG_COLORS.dss, 4.5);
    this.gpsDot = dot(LEG_COLORS.gps, 4.5);
    const sh = new THREE.CircleGeometry(1, 32);
    sh.rotateX(-Math.PI / 2);
    const beacon = (color: string) => {
      const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: glow, color, transparent: true, depthWrite: false, depthTest: false, fog: false, sizeAttenuation: false, blending: THREE.AdditiveBlending }));
      sp.renderOrder = 10;
      this.scene.add(sp);
      return sp;
    };
    this.beacon = beacon('#cfefff');
    this.ghostBeacon = beacon(LEG_COLORS.gps);
    this.shadow = new THREE.Mesh(sh, new THREE.MeshBasicMaterial({ map: glow, color: '#000000', transparent: true, opacity: 0.45, depthWrite: false, fog: false }));
    this.shadow.renderOrder = 2;
    this.scene.add(this.shadow);
  }

  /** Leg metres (x east, y north, h up) to the scene: x east, y up, z south, centred on the tile. */
  w(x: number, y: number, h = 0, out = new THREE.Vector3()): THREE.Vector3 {
    return out.set(x - this.cx, h, -(y - this.cy));
  }

  async load(base: string): Promise<void> {
    const loader = new THREE.TextureLoader();
    const tex = await loader.loadAsync(new URL(this.leg.ground.file, base).href);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = Math.min(8, this.renderer.capabilities.getMaxAnisotropy());
    const ext = this.leg.ground.extent;
    const W = ext.xmax - ext.xmin;
    const H = ext.ymax - ext.ymin;
    const geo = new THREE.PlaneGeometry(W, H);
    geo.rotateX(-Math.PI / 2);
    const ground = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ map: tex }));
    this.scene.add(ground);
    // Around the tile, when the bundle has them: the neighbouring tiles of the same flight, coarser and dimmed so the
    // tile the run used still reads as the data. Its edge stays marked below.
    const ctx = this.leg.context;
    if (ctx) {
      const ctex = await loader.loadAsync(new URL(ctx.file, base).href);
      ctex.colorSpace = THREE.SRGBColorSpace;
      ctex.anisotropy = tex.anisotropy;
      const ce = ctx.extent;
      const cgeo = new THREE.PlaneGeometry(ce.xmax - ce.xmin, ce.ymax - ce.ymin);
      cgeo.rotateX(-Math.PI / 2);
      const around = new THREE.Mesh(cgeo, new THREE.MeshBasicMaterial({ map: ctex, color: '#7f898f' }));
      this.w((ce.xmin + ce.xmax) / 2, (ce.ymin + ce.ymax) / 2, -0.3, around.position);
      this.scene.add(around);
    }
    // Beyond the imagery: a dark plain with a faint survey grid. The tile's own edge is marked on top.
    const far = new THREE.Mesh(
      new THREE.PlaneGeometry(12000, 12000).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: '#0f1a21' }),
    );
    far.position.y = -0.6;
    this.scene.add(far);
    const grid = new THREE.GridHelper(12000, 120, '#1e3340', '#16262f');
    grid.position.y = -0.5;
    this.scene.add(grid);
    const edge = fatLine('#9db0bc', 1.5, { opacity: 0.6 });
    const c = [this.w(ext.xmin, ext.ymin, 0.2), this.w(ext.xmax, ext.ymin, 0.2), this.w(ext.xmax, ext.ymax, 0.2), this.w(ext.xmin, ext.ymax, 0.2), this.w(ext.xmin, ext.ymin, 0.2)];
    setLine(edge, c.flatMap((v) => [v.x, v.y, v.z]));
    this.scene.add(edge);

    this.drone = buildDrone();
    this.scene.add(this.drone.group);
    this.ghost = buildDrone();
    this.ghost.group.traverse((o) => {
      if (o instanceof THREE.Mesh) {
        const mat = (o.material as THREE.Material).clone();
        mat.transparent = true;
        mat.opacity = 0.55;
        if ('color' in mat && mat.color instanceof THREE.Color) mat.color.set(LEG_COLORS.gps);
        // Only lit materials have an emissive uniform; setting one on a basic material breaks the render.
        if (mat instanceof THREE.MeshStandardMaterial || mat instanceof THREE.MeshLambertMaterial || mat instanceof THREE.MeshPhongMaterial) {
          mat.emissive.set(LEG_COLORS.gps);
          mat.emissiveIntensity = 0.6;
        }
        o.material = mat;
      }
    });
    this.scene.add(this.ghost.group);
  }

  private trailPts(t: number, pick: (i: number) => [number, number] | null, from = 0): number[] {
    const E = this.leg.epochs;
    const last = epochAt(this.leg, t);
    const pts: number[] = [];
    const v = new THREE.Vector3();
    for (let i = from; i <= last; i++) {
      const p = pick(i);
      if (!p) continue;
      this.w(p[0], p[1], MARK_H, v);
      pts.push(v.x, v.y, v.z);
    }
    if (t > E[0]!.t && t < E[E.length - 1]!.t) {
      const p = pick(-1);
      if (p) {
        this.w(p[0], p[1], MARK_H, v);
        pts.push(v.x, v.y, v.z);
      }
    }
    return pts;
  }

  /** Pose the scene for time t, seen from `view.camera`. */
  update(view: LegView): void {
    const leg = this.leg;
    const E = leg.epochs;
    const t = Math.max(E[0]!.t, Math.min(E[E.length - 1]!.t, view.t));
    const res = new THREE.Vector2(view.width, view.height);
    this.scene.traverse((o) => {
      if (o instanceof Line2) (o.material as LineMaterial).resolution.copy(res);
    });

    const tr = truthAt(leg, t);
    const dronePos = this.w(tr.x, tr.y, tr.agl);
    const camDist = view.camera.position.distanceTo(dronePos);
    // True size up close; drawn larger in wide shots so it stays visible (the HUD says so).
    this.droneScale = Math.max(1, Math.min(60, camDist / 9));
    const yaw = THREE.MathUtils.degToRad(90 - tr.heading);
    const g = this.drone.group;
    g.position.copy(dronePos);
    this.droneWorld.copy(dronePos);
    g.rotation.set(0, yaw, 0);
    // Lean into the motion a little, as a multirotor would at 15 m/s.
    g.rotateZ(-0.16);
    g.scale.setScalar(this.droneScale);
    this.drone.rotors.forEach((r, i) => (r.rotation.y = view.t * 140 * (i % 2 ? 1 : -1)));
    this.drone.updateWash(view.t, 300);
    this.shadow.position.copy(this.w(tr.x, tr.y, 0.15));
    // Fades in as the drone shrinks on screen; gone in the close chase where the airframe reads itself.
    const bv = Math.max(0, Math.min(1, (this.droneScale - 2.5) / 4));
    this.beacon.visible = bv > 0;
    this.beacon.position.copy(dronePos);
    this.beacon.scale.setScalar(0.03 + 0.012 * bv);
    (this.beacon.material as THREE.SpriteMaterial).opacity = 0.9 * bv;
    this.shadow.scale.setScalar(Math.max(1.2, this.droneScale * 0.35));

    // Trails: the truth, the GPS the receiver reports (spoofed from the spoof's start), DSS's estimate.
    const live = (mode: 'gated') => (i: number) => (i < 0 ? solutionAt(leg, t, mode) : E[i]![mode].pub);
    setLine(this.trails.truth, this.trailPts(t, (i) => (i < 0 ? [tr.x, tr.y] : E[i]!.truth)));
    setLine(this.trails.dss, this.trailPts(t, live('gated')));
    const s0 = Math.max(0, epochAt(leg, leg.spoof.startS) - 1);
    setLine(this.trails.gps, t >= leg.spoof.startS - 0.3 ? this.trailPts(t, (i) => (i < 0 ? spoofedAt(leg, t) : E[i]!.spoofed), s0) : []);

    const k = epochAt(leg, t);
    const ep = E[k]!;
    const est = solutionAt(leg, t, 'gated') ?? [tr.x, tr.y];
    const pl = ep.gated.pl ?? 0;
    const ec = this.w(est[0], est[1], MARK_H);
    this.dssDot.position.copy(ec).setY(MARK_H + 0.1);
    const sp = spoofedAt(leg, t);
    this.gpsDot.position.copy(this.w(sp[0], sp[1], MARK_H + 0.1));
    this.gpsDot.visible = t >= leg.spoof.startS;
    const plShown = Math.min(pl, 400);
    this.plRing.position.copy(ec).setY(MARK_H + 0.05);
    this.plFill.position.copy(ec).setY(MARK_H);
    this.plRing.scale.setScalar(Math.max(0.5, plShown));
    this.plFill.scale.setScalar(Math.max(0.5, plShown));
    const over = pl > leg.alertLimitM;
    (this.plRing.material as THREE.MeshBasicMaterial).color.set(over ? LEG_COLORS.alert : LEG_COLORS.dss);
    (this.plFill.material as THREE.MeshBasicMaterial).color.set(over ? LEG_COLORS.alert : LEG_COLORS.dss);
    (this.plFill.material as THREE.MeshBasicMaterial).opacity = 0.14 * Math.max(0.15, Math.min(1, 25 / Math.max(1, pl)));
    const circle: number[] = [];
    for (let a = 0; a <= 96; a++) {
      const th = (a / 96) * Math.PI * 2;
      circle.push(ec.x + leg.alertLimitM * Math.cos(th), MARK_H + 0.08, ec.z + leg.alertLimitM * Math.sin(th));
    }
    setLine(this.alert, circle);
    const tc = this.w(tr.x, tr.y, MARK_H + 0.06);
    const e = dist(est, [tr.x, tr.y]) ?? 0;
    setLine(this.tether, e > 0.4 ? [ec.x, MARK_H + 0.06, ec.z, tc.x, tc.y, tc.z] : []);

    // The camera: its full footprint, the frustum edges, and the 96 m patch it matches.
    const h = tr.agl;
    const half = Math.tan(THREE.MathUtils.degToRad(leg.camera.hfovDeg / 2)) * h;
    const halfV = half * (9 / 16);
    const th = THREE.MathUtils.degToRad(tr.heading);
    const fwd = [Math.sin(th), Math.cos(th)];
    const right = [Math.cos(th), -Math.sin(th)];
    const corner = (a: number, b: number) => this.w(tr.x + right[0]! * a + fwd[0]! * b, tr.y + right[1]! * a + fwd[1]! * b, 0.25);
    const full = [corner(-half, halfV), corner(half, halfV), corner(half, -halfV), corner(-half, -halfV)];
    setLine(this.footFull, [...full, full[0]!].flatMap((v) => [v.x, v.y, v.z]));
    this.frustum.forEach((l, i) => setLine(l, [dronePos.x, dronePos.y - 0.05 * this.droneScale, dronePos.z, full[i]!.x, full[i]!.y, full[i]!.z]));
    const p = leg.camera.patchM / 2;
    const pc = [corner(-p, p), corner(p, p), corner(p, -p), corner(-p, -p)];
    setLine(this.patchEdge, [...pc, pc[0]!].flatMap((v) => [v.x, v.y + 0.05, v.z]));
    this.patch.position.copy(this.w(tr.x, tr.y, 0.22));
    this.patch.rotation.set(0, -th, 0);
    this.patch.scale.set(leg.camera.patchM, 1, leg.camera.patchM);

    // Fixes: dots appear as they are matched; the latest ones ripple and light the patch.
    let lit = 0;
    let litColor: string = LEG_COLORS.fix;
    let r = 0;
    this.fixDots.count = k + 1;
    for (let i = k; i >= 0 && r < this.ripples.length; i--) {
      const age = t - E[i]!.t;
      if (age > FLASH_S) break;
      const f = E[i]!.fix;
      if (!f.xy) continue;
      const ring = this.ripples[r++]!;
      const u = age / FLASH_S;
      ring.visible = true;
      ring.position.copy(this.w(f.xy[0], f.xy[1], MARK_H + 0.12));
      ring.scale.setScalar(2 + u * 16);
      const mat = ring.material as THREE.MeshBasicMaterial;
      mat.color.set(f.used ? LEG_COLORS.fix : LEG_COLORS.reject);
      mat.opacity = (1 - u) * 0.9;
      const glowNow = Math.exp(-age / 0.3);
      if (glowNow > lit) {
        lit = glowNow;
        litColor = f.used ? LEG_COLORS.fix : LEG_COLORS.reject;
      }
    }
    for (; r < this.ripples.length; r++) this.ripples[r]!.visible = false;
    const pm = this.patch.material as THREE.MeshBasicMaterial;
    pm.opacity = 0.1 + 0.45 * lit;
    pm.color.set(lit > 0.05 ? litColor : '#ffffff');
    (this.patchEdge.material as LineMaterial).color.set(lit > 0.05 ? litColor : '#ffffff');

    // The GPS-trusting autopilot's belief: a red ghost of the drone where blind fusion puts it.
    const blind = solutionAt(leg, t, 'blind');
    const off = dist(blind, [tr.x, tr.y]) ?? 0;
    this.ghost.group.visible = t >= leg.spoof.startS && off > 2;
    this.ghostWorld = null;
    this.ghostBeacon.visible = this.ghost.group.visible && this.droneScale > 2.5;
    if (blind) {
      this.ghost.group.position.copy(this.w(blind[0], blind[1], tr.agl));
      this.ghost.group.rotation.copy(g.rotation);
      this.ghost.group.scale.setScalar(this.droneScale);
      this.ghostBeacon.position.copy(this.ghost.group.position);
      this.ghostWorld = this.ghost.group.visible ? this.ghost.group.position.clone() : null;
      this.ghostBeacon.scale.setScalar(0.042);
      (this.ghostBeacon.material as THREE.SpriteMaterial).opacity = 0.9 * bv;
      this.ghost.rotors.forEach((ro, i) => (ro.rotation.y = view.t * 140 * (i % 2 ? 1 : -1)));
    }
  }
}
