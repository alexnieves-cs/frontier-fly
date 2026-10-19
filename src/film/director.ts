import * as THREE from 'three';
import type { Phase } from '../mission/autonomy/contracts';
import { sampleFlight, type FlightLog } from './flight';
import { toThree, type FilmScene } from './scene';

/** How many sim seconds play per film second, by phase. */
const RATE: Record<Phase, number> = {
  preflight: 1.5,
  coverage: 9,
  approach: 4,
  inspect: 5,
  entry_select: 1.5,
  await_operator: 2,
  fly_through: 1,
  indoor_explore: 4,
  confirm: 1.6,
  exit: 2.5,
  return_home: 8,
  land: 2.5,
  recover: 2,
  done: 2,
};

export const FILM_FPS = 30;
const INTRO_SEC = 5;
const OUTRO_SEC = 7;

export type ShotKind = 'intro' | 'chase-high' | 'aerial' | 'chase' | 'orbit' | 'low-chase' | 'cutaway' | 'close' | 'land' | 'outro';

export interface FilmFrame {
  /** Film frame index. */
  i: number;
  /** Sim time shown on this frame. */
  t: number;
  /** Playback rate at this frame, sim seconds per film second. */
  rate: number;
  part: 'intro' | 'flight' | 'outro';
  /** Seconds into the intro or outro. */
  partSec: number;
}

/**
 * Film time to sim time. Each phase plays at its own rate, slowed near the
 * moments worth watching (entry, confirmations, touchdown), with the rate
 * eased so speed changes never jump.
 */
/**
 * Film frames for a flight. Fast phases play at RATE times real time; first
 * contacts and building entries drop to near real time. With `flightSec`, the
 * fast-phase rates are scaled together so the flight part runs about that long.
 */
export function buildTimeline(log: FlightLog, flightSec?: number): FilmFrame[] {
  let pace = 1;
  if (flightSec) {
    let lo = 0.25;
    let hi = 40;
    for (let k = 0; k < 24; k++) {
      pace = Math.sqrt(lo * hi);
      if (flightFrames(log, pace).length > flightSec * FILM_FPS) lo = pace;
      else hi = pace;
    }
  }
  const frames: FilmFrame[] = [];
  const dt = 1 / FILM_FPS;
  for (let i = 0; i < INTRO_SEC * FILM_FPS; i++) frames.push({ i: frames.length, t: 0, rate: 0, part: 'intro', partSec: i * dt });
  for (const f of flightFrames(log, pace)) frames.push({ ...f, i: frames.length });
  const end = log.frames[log.frames.length - 1]!.t;
  for (let i = 0; i < OUTRO_SEC * FILM_FPS; i++) frames.push({ i: frames.length, t: end, rate: 0, part: 'outro', partSec: i * dt });
  return frames;
}

function flightFrames(log: FlightLog, pace: number): FilmFrame[] {
  const frames: FilmFrame[] = [];
  const dt = 1 / FILM_FPS;
  const end = log.frames[log.frames.length - 1]!.t;
  const keys = log.confirmed.map((c) => c.tSec);
  const entries = log.record.entries.map((e) => e.tSec);
  let t = 0;
  let rate = 1.5;
  while (t < end) {
    const { frame } = sampleFlight(log, t);
    const base = RATE[frame.phase] ?? 3;
    let target = base >= 2 ? base * pace : base;
    for (const k of keys) if (t > k - 4 && t < k + 2.5) target = Math.min(target, 1.2);
    for (const k of entries) if (t > k - 1 && t < k + 4) target = Math.min(target, 1.4);
    if (end - t < 4) target = Math.min(target, 1.5);
    rate += (target - rate) * (1 - Math.exp(-dt / 0.45));
    frames.push({ i: frames.length, t, rate, part: 'flight', partSec: 0 });
    t += rate * dt;
  }
  return frames;
}

interface Pose {
  pos: THREE.Vector3;
  look: THREE.Vector3;
  fov: number;
  cutaway: number;
}

/**
 * Picks a shot per frame and moves a camera toward it with critically damped
 * springs. Stateful, so frames must be rendered in order (the recorder does).
 */
export class Director {
  readonly camera = new THREE.PerspectiveCamera(50, 16 / 9, 0.05, 3000);
  shot: ShotKind = 'intro';
  private readonly pos = new THREE.Vector3();
  private readonly look = new THREE.Vector3();
  private fov = 50;
  private cutaway = 0;
  private started = false;
  private heading = 0;
  private lastShot: ShotKind | null = null;
  private readonly ray = new THREE.Raycaster();
  private pull = 1;
  private crampedUntil = -1;
  private readonly site: THREE.Vector3;
  private readonly home: THREE.Vector3;

  constructor(
    private readonly log: FlightLog,
    private readonly scene: FilmScene,
    private readonly occluders: THREE.Object3D[],
  ) {
    const size = log.loaded.world.outdoor.size;
    this.site = new THREE.Vector3(size / 2, scene.groundAt(size / 2, size / 2), size / 2);
    this.home = toThree(log.loaded.world.home);
  }

  setAspect(aspect: number): void {
    this.camera.aspect = aspect;
    this.camera.updateProjectionMatrix();
  }

  /** Place the camera for one film frame. */
  frame(f: FilmFrame): void {
    const { pos, yaw, frame } = sampleFlight(this.log, f.t);
    const drone = toThree(pos);
    // Frame along the direction of travel (the drone often flies sideways), eased so turns pan smoothly.
    const ahead = sampleFlight(this.log, f.t + 1.2).pos;
    const behind = sampleFlight(this.log, f.t - 0.6).pos;
    const dx = ahead[0] - behind[0];
    const dy = ahead[1] - behind[1];
    const travel = Math.hypot(dx, dy) > 0.6 ? Math.atan2(dy, dx) : yaw;
    if (!this.started) this.heading = travel;
    let dh = travel - this.heading;
    dh -= Math.round(dh / (Math.PI * 2)) * Math.PI * 2;
    this.heading += dh * 0.06;
    const fwd = new THREE.Vector3(Math.cos(this.heading), 0, Math.sin(this.heading));
    const shot = this.pick(f, frame.phase, frame.inside);
    const want = this.pose(shot, f, drone, fwd, frame.inside);
    const cut = !this.started || (shot !== this.lastShot && cutBetween(this.lastShot, shot));
    if (cut) {
      this.pos.copy(want.pos);
      this.look.copy(want.look);
      this.fov = want.fov;
    } else {
      const k = shot === 'low-chase' || shot === 'close' ? 0.22 : shot === 'chase' ? 0.14 : 0.07;
      this.pos.lerp(want.pos, k);
      this.look.lerp(want.look, Math.min(1, k * 1.8));
      this.fov += (want.fov - this.fov) * 0.08;
    }
    this.cutaway += (want.cutaway - this.cutaway) * (cut ? 1 : 0.2);
    this.started = true;
    this.lastShot = shot;
    this.shot = shot;
    // Snap in front of an occluder at once, ease back out once it clears.
    const clear = this.clearance(this.pos, drone, shot);
    this.pull = cut || clear < this.pull ? clear : this.pull + (clear - this.pull) * 0.08;
    const offset = this.pos.clone().sub(drone);
    const reach = offset.length();
    this.camera.position.copy(drone).addScaledVector(offset, reach > 1e-3 ? Math.max(0.35 / reach, this.pull) : 1);
    // A room too tight for a follow shot: go overhead for a few seconds instead
    // of pressing the lens against the drone.
    if (frame.inside && shot !== 'cutaway' && reach * this.pull < 1.6) this.crampedUntil = f.i + FILM_FPS * 3;
    this.camera.lookAt(this.look);
    this.camera.fov = this.fov;
    this.camera.updateProjectionMatrix();
    this.scene.setCutaway(this.cutaway);
  }

  private pick(f: FilmFrame, phase: Phase, inside: string | null): ShotKind {
    if (f.part === 'intro') return 'intro';
    if (f.part === 'outro') return 'outro';
    if (inside && f.i < this.crampedUntil) return 'cutaway';
    switch (phase) {
      case 'preflight':
        return 'chase';
      case 'coverage': {
        // Alternate a following shot with a high aerial every few film seconds.
        const block = Math.floor(f.i / (FILM_FPS * 6));
        return block % 2 === 0 ? 'chase-high' : 'aerial';
      }
      case 'approach':
        return 'chase';
      case 'inspect':
        return 'orbit';
      case 'entry_select':
      case 'await_operator':
      case 'fly_through':
        return inside ? 'cutaway' : 'low-chase';
      case 'indoor_explore':
        return 'cutaway';
      case 'confirm':
        return 'close';
      case 'exit':
        return inside ? 'cutaway' : 'low-chase';
      case 'return_home':
        return 'chase-high';
      case 'land':
      case 'done':
        return 'land';
      default:
        return 'chase';
    }
  }

  private pose(shot: ShotKind, f: FilmFrame, drone: THREE.Vector3, fwd: THREE.Vector3, inside: string | null): Pose {
    const side = new THREE.Vector3(-fwd.z, 0, fwd.x);
    switch (shot) {
      case 'intro': {
        const u = f.partSec / INTRO_SEC;
        const a = -0.9 + u * 0.9;
        const r = 58 - u * 18;
        const pos = new THREE.Vector3(this.home.x + Math.cos(a) * r, this.site.y + 34 - u * 22, this.home.z + Math.sin(a) * r);
        const look = this.site.clone().lerp(this.home, u).add(new THREE.Vector3(0, 1, 0));
        return { pos, look, fov: 45, cutaway: 0 };
      }
      case 'chase-high': {
        const pos = drone.clone().addScaledVector(fwd, -13).add(new THREE.Vector3(0, 7.5, 0)).addScaledVector(side, 3);
        return { pos, look: drone.clone().addScaledVector(fwd, 6).add(new THREE.Vector3(0, -2, 0)), fov: 52, cutaway: 0 };
      }
      case 'aerial': {
        const toDrone = drone.clone().sub(this.site).setY(0);
        const pos = this.site.clone().addScaledVector(toDrone, -0.35).add(new THREE.Vector3(18, 46, 26));
        return { pos, look: drone.clone().lerp(this.site, 0.25), fov: 40, cutaway: 0 };
      }
      case 'chase': {
        const pos = drone.clone().addScaledVector(fwd, -8).add(new THREE.Vector3(0, 3.4, 0)).addScaledVector(side, 1.5);
        return { pos, look: drone.clone().addScaledVector(fwd, 5).add(new THREE.Vector3(0, -0.6, 0)), fov: 54, cutaway: 0 };
      }
      case 'orbit': {
        const id = this.nearestBuilding(drone);
        const center = id ? this.scene.shellCenter(id)! : drone.clone();
        const out = drone.clone().sub(center).setY(0).normalize();
        const tangent = new THREE.Vector3(-out.z, 0, out.x);
        const pos = drone.clone().addScaledVector(out, 7).addScaledVector(tangent, -4).add(new THREE.Vector3(0, 2.6, 0));
        return { pos, look: drone.clone().lerp(center, 0.45), fov: 52, cutaway: 0 };
      }
      case 'low-chase': {
        const pos = drone.clone().addScaledVector(fwd, -3.2).add(new THREE.Vector3(0, 0.75, 0));
        return { pos, look: drone.clone().addScaledVector(fwd, 4).add(new THREE.Vector3(0, -0.15, 0)), fov: 62, cutaway: 0 };
      }
      case 'cutaway': {
        const id = inside ?? this.nearestBuilding(drone);
        const center = id ? this.scene.shellCenter(id)! : drone.clone();
        const span = id ? this.scene.shellSize(id) : 10;
        const look = drone.clone().lerp(center, 0.35);
        const pos = look.clone().add(new THREE.Vector3(span * 0.32, span * 1.05 + 4, span * 0.62));
        return { pos, look, fov: 46, cutaway: 1 };
      }
      case 'close': {
        const target = this.nextConfirmTarget(f.t);
        const look = target ? drone.clone().lerp(target, 0.6) : drone.clone().addScaledVector(fwd, 2);
        if (inside) {
          // Indoors, rooms are too tight for an over-the-shoulder shot: look steeply
          // down through the removed ceiling at the drone and what it is checking.
          const pos = look.clone().addScaledVector(fwd, -2.2).addScaledVector(side, 1.6).add(new THREE.Vector3(0, 6.5, 0));
          return { pos, look, fov: 50, cutaway: 1 };
        }
        const pos = drone.clone().addScaledVector(fwd, -2.4).addScaledVector(side, 1.4).add(new THREE.Vector3(0, 2.6, 0));
        return { pos, look, fov: 55, cutaway: 1 };
      }
      case 'land': {
        const pos = this.home.clone().add(new THREE.Vector3(5.5, 1.6, 4.5));
        return { pos, look: drone.clone().add(new THREE.Vector3(0, -0.2, 0)), fov: 46, cutaway: 0 };
      }
      case 'outro': {
        const u = f.partSec / OUTRO_SEC;
        const pos = this.home.clone().add(new THREE.Vector3(6 + u * 10, 2 + u * 22, 5 + u * 12));
        return { pos, look: this.home.clone().lerp(this.site, u * 0.6), fov: 46, cutaway: 0 };
      }
    }
  }

  private nearestBuilding(p: THREE.Vector3): string | null {
    let best: string | null = null;
    let bestD = Infinity;
    for (const b of this.log.loaded.world.buildings) {
      const c = this.scene.shellCenter(b.id)!;
      const d = c.distanceToSquared(p);
      if (d < bestD) {
        bestD = d;
        best = b.id;
      }
    }
    return best;
  }

  private nextConfirmTarget(t: number): THREE.Vector3 | null {
    const next = this.log.confirmed.find((c) => c.tSec >= t - 3) ?? this.log.confirmed[this.log.confirmed.length - 1];
    return next ? toThree(next.estimate) : null;
  }

  /**
   * Fraction of the drone-to-camera offset that is clear of walls and trees. Rays
   * also go to points around the camera so a trunk or canopy just beside the lens
   * (which the centre ray misses) still pulls it in.
   */
  private clearance(want: THREE.Vector3, drone: THREE.Vector3, shot: ShotKind): number {
    if (shot === 'aerial' || shot === 'cutaway' || shot === 'outro') return 1;
    const dir = want.clone().sub(drone);
    const dist = dir.length();
    if (dist < 1e-3) return 1;
    dir.divideScalar(dist);
    const side = new THREE.Vector3().crossVectors(dir, UP);
    if (side.lengthSq() < 1e-6) side.set(1, 0, 0);
    side.normalize();
    const up = new THREE.Vector3().crossVectors(side, dir).normalize();
    let frac = 1;
    for (const [a, b] of PROBES) {
      const probe = want.clone().addScaledVector(side, a).addScaledVector(up, b).sub(drone);
      const len = probe.length();
      this.ray.set(drone, probe.divideScalar(len));
      this.ray.far = len;
      // Roofs and ceilings hidden for a cutaway still sit in the occluder list.
      const hit = this.ray.intersectObjects(this.occluders, true).find((h) => shown(h.object));
      if (hit) frac = Math.min(frac, (hit.distance - 0.3) / len);
    }
    return Math.max(0, frac);
  }
}

const UP = new THREE.Vector3(0, 1, 0);

function shown(object: THREE.Object3D): boolean {
  for (let o: THREE.Object3D | null = object; o; o = o.parent) if (!o.visible) return false;
  return true;
}
/** Lens-side probe offsets (side, up) in metres around the wanted camera position. */
const PROBES: [number, number][] = [
  [0, 0],
  [1.4, 0],
  [-1.4, 0],
  [0, 1.0],
  [0, -0.8],
];

function cutBetween(from: ShotKind | null, to: ShotKind): boolean {
  if (!from) return true;
  // Hard cuts between unrelated framings; the rest blend.
  const groups: Record<ShotKind, number> = {
    intro: 0,
    'chase-high': 1,
    aerial: 2,
    chase: 1,
    orbit: 1,
    'low-chase': 1,
    cutaway: 3,
    close: 3,
    land: 4,
    outro: 4,
  };
  return groups[from] !== groups[to];
}
