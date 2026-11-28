import * as THREE from 'three';
import type { Track } from './film';
import { CUT_H, toThree } from './scene';

/** Film seconds before the sim starts (title over the establishing shot) and after it ends (results). */
export const INTRO_S = 4;
export const OUTRO_S = 8;
/** Sim seconds shown at true speed while the drones launch. */
const LAUNCH_S = 5;
/** Sim seconds over which playback ramps from true speed to the cruise speed. */
const RAMP_S = 2.5;
/** Film seconds a shot blends into the next. */
const BLEND_S = 1.6;

interface Cam {
  pos: THREE.Vector3;
  target: THREE.Vector3;
  fov: number;
  /** 0 close, 1 wide (drone draw scale). */
  wide: number;
}

type Shot =
  | { kind: 'establish' }
  | { kind: 'entry' }
  | { kind: 'orbit'; az0: number; az1: number; elev: number; radius: number; follow: number }
  | { kind: 'chase'; drone: number }
  | { kind: 'crane' }
  | { kind: 'top' };

interface Cue {
  /** Film time the shot starts. */
  at: number;
  shot: Shot;
}

function smooth(x: number): number {
  const c = Math.max(0, Math.min(1, x));
  return c * c * (3 - 2 * c);
}

/**
 * The camera work and the clock. Playback runs at true speed for the launch, then ramps to a cruise
 * speed so the whole search fits in about half a minute; the sim freezes for the title and the
 * results. Every frame is a function of its film time.
 */
export class SwarmDirector {
  readonly camera = new THREE.PerspectiveCamera(42, 16 / 9, 0.05, 2000);
  readonly speed: number;
  /** Sim time of the first close pass, which plays at true speed (null if none). */
  private readonly slowAt: number | null;
  /** Film time at which the sim ends (after this the results show). */
  readonly simEndFilm: number;
  readonly total: number;
  private readonly filmAt: Float64Array;
  private readonly step = 0.02;
  private readonly cues: Cue[] = [];
  private readonly cx: number;
  private readonly cy: number;
  private readonly size: number;
  private readonly a = { pos: new THREE.Vector3(), target: new THREE.Vector3(), fov: 42, wide: 1 };
  private readonly b = { pos: new THREE.Vector3(), target: new THREE.Vector3(), fov: 42, wide: 1 };
  /** The shot's name on the last frame (for debugging and the HUD). */
  shotName = '';

  constructor(
    private readonly track: Track,
    W: number,
    D: number,
  ) {
    this.cx = W / 2;
    this.cy = D / 2;
    this.size = Math.max(W, D);
    const T = track.end;
    // A close pass between two drones plays at true speed, with the camera on it.
    const pass = track.events.find((e) => e.warn && e.pair);
    this.slowAt = pass ? pass.t : null;
    // Cruise speed: the search after the launch in about 32 film seconds, in half steps, 2x to 6x.
    this.speed = Math.max(2, Math.min(6, Math.round(((T - LAUNCH_S - RAMP_S) / 32) * 2) / 2));
    const n = Math.ceil(T / this.step) + 1;
    this.filmAt = new Float64Array(n);
    for (let k = 1; k < n; k++) {
      const t = (k - 0.5) * this.step;
      this.filmAt[k] = this.filmAt[k - 1]! + this.step / this.rate(t);
    }
    this.simEndFilm = INTRO_S + this.filmAt[n - 1]!;
    this.total = this.simEndFilm + OUTRO_S;

    // Shots, placed by sim time and converted to film time.
    const f = (t: number) => INTRO_S + this.filmOf(t);
    const last = track.n - 1;
    const second = Math.min(last, track.n > 2 ? 2 : 1);
    this.cues.push({ at: 0, shot: { kind: 'establish' } });
    this.cues.push({ at: INTRO_S - 0.4, shot: { kind: 'entry' } });
    this.cues.push({ at: f(LAUNCH_S + 0.5), shot: { kind: 'orbit', az0: -2.1, az1: -1.35, elev: 0.95, radius: 0.92, follow: 0.25 } });
    this.cues.push({ at: f(T * 0.36), shot: { kind: 'chase', drone: 0 } });
    this.cues.push({ at: f(T * 0.5), shot: { kind: 'orbit', az0: 0.05, az1: -0.65, elev: 0.74, radius: 0.78, follow: 0.55 } });
    if (pass?.pair) {
      this.cues.push({ at: f(Math.max(T * 0.55, pass.t - 7)), shot: { kind: 'chase', drone: pass.pair[0] } });
      if (pass.t + 3 < T - 4) this.cues.push({ at: f(pass.t + 3), shot: { kind: 'crane' } });
    } else {
      this.cues.push({ at: f(T * 0.66), shot: { kind: 'chase', drone: second } });
      this.cues.push({ at: f(T * 0.8), shot: { kind: 'crane' } });
    }
    this.cues.push({ at: this.simEndFilm + 0.2, shot: { kind: 'top' } });
    this.cues.sort((a, b) => a.at - b.at);
  }

  /** Playback speed (sim seconds per film second) at sim time t. */
  private rate(t: number): number {
    const r = t < LAUNCH_S ? 1 : 1 + (this.speed - 1) * smooth((t - LAUNCH_S) / RAMP_S);
    if (this.slowAt === null) return r;
    // True speed from 3 s before a close pass to 1.5 s after it, easing in and out over 2 s.
    const out = Math.max(0, this.slowAt - 3 - t, t - this.slowAt - 1.5);
    return r + (1 - r) * (1 - smooth(Math.min(1, out / 2)));
  }

  /** Film seconds after the intro at which sim time t plays. */
  private filmOf(t: number): number {
    const x = Math.max(0, Math.min(this.filmAt.length - 1, t / this.step));
    const k = Math.floor(x);
    const k1 = Math.min(this.filmAt.length - 1, k + 1);
    return this.filmAt[k]! + (this.filmAt[k1]! - this.filmAt[k]!) * (x - k);
  }

  /** Sim time shown at film time tau. */
  simTime(tau: number): number {
    const u = tau - INTRO_S;
    if (u <= 0) return 0;
    const fa = this.filmAt;
    if (u >= fa[fa.length - 1]!) return this.track.end;
    let lo = 0;
    let hi = fa.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (fa[mid]! <= u) lo = mid;
      else hi = mid;
    }
    return (lo + (u - fa[lo]!) / Math.max(1e-9, fa[hi]! - fa[lo]!)) * this.step;
  }

  /** Playback speed shown at film time tau (0 while frozen). */
  speedAt(tau: number): number {
    if (tau < INTRO_S || tau > this.simEndFilm) return 0;
    return this.rate(this.simTime(tau));
  }

  /** Aim the camera for film time tau. */
  frame(tau: number): { t: number; wide: number } {
    const t = this.simTime(tau);
    let i = 0;
    while (i + 1 < this.cues.length && this.cues[i + 1]!.at <= tau) i++;
    const cue = this.cues[i]!;
    this.place(cue.shot, tau, t, cue.at, this.a);
    let cam: Cam = this.a;
    if (i > 0) {
      const w = smooth((tau - cue.at) / BLEND_S);
      if (w < 1) {
        const prev = this.cues[i - 1]!;
        this.place(prev.shot, tau, t, prev.at, this.b);
        this.b.pos.lerp(this.a.pos, w);
        this.b.target.lerp(this.a.target, w);
        this.b.fov += (this.a.fov - this.b.fov) * w;
        this.b.wide += (this.a.wide - this.b.wide) * w;
        cam = this.b;
      }
    }
    this.shotName = cue.shot.kind;
    this.camera.position.copy(cam.pos);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(cam.target);
    this.camera.fov = cam.fov;
    this.camera.updateProjectionMatrix();
    return { t, wide: cam.wide };
  }

  private orbitPoint(az: number, elev: number, radius: number, target: THREE.Vector3, out: THREE.Vector3): void {
    out.set(target.x + Math.cos(az) * Math.cos(elev) * radius, target.y + Math.sin(elev) * radius, target.z + Math.sin(az) * Math.cos(elev) * radius);
  }

  private place(shot: Shot, tau: number, t: number, start: number, cam: Cam): void {
    const S = this.size;
    const center = new THREE.Vector3(this.cx, 0.6, this.cy);
    const spawn = this.track.b.spec.spawn;
    const base = this.track.b.base;
    switch (shot.kind) {
      case 'establish': {
        // A slow push in toward the entrance, from high and wide.
        const u = smooth(tau / (INTRO_S + 1));
        cam.target.copy(center).lerp(toThree(spawn[0], spawn[1], 0.6), 0.25 * u);
        this.orbitPoint(-1.85 + 0.3 * u, 0.62 - 0.08 * u, S * (1.55 - 0.3 * u), cam.target, cam.pos);
        cam.fov = 40;
        cam.wide = 1;
        return;
      }
      case 'entry': {
        // Over the base station's shoulder and the cut front wall, looking in as the drones launch.
        const u = smooth((tau - start) / 8);
        const tx = spawn[0];
        const ty = spawn[1];
        const dx = tx - base[0];
        const dy = ty - base[1];
        const l = Math.hypot(dx, dy) || 1;
        cam.target.copy(toThree(tx + (dx / l) * (1 + 1.5 * u), ty + (dy / l) * (1 + 1.5 * u), 0.8));
        cam.pos.copy(toThree(base[0] - (dx / l) * 3.2 - (dy / l) * (2.6 - 1.4 * u), base[1] - (dy / l) * 3.2 + (dx / l) * (2.6 - 1.4 * u), 6.2 + 0.8 * u));
        cam.fov = 48;
        cam.wide = 0.3;
        return;
      }
      case 'orbit': {
        const dur = 14;
        const u = Math.min(1, (tau - start) / dur);
        const c = this.track.centroid(t);
        cam.target.copy(center).lerp(toThree(c[0], c[1], 0.6), shot.follow);
        this.orbitPoint(shot.az0 + (shot.az1 - shot.az0) * u, shot.elev, S * shot.radius * 1.25, cam.target, cam.pos);
        cam.fov = 40;
        cam.wide = 1;
        return;
      }
      case 'chase': {
        // Over the shoulder, from above the walls, along the drone's smoothed direction of travel.
        const tr = this.track;
        let hx = 0;
        let hy = 0;
        for (let k = -6; k <= 3; k++) {
          const s = tr.sample(Math.max(0, t + k * 0.25), shot.drone);
          const sp = Math.hypot(s.vx, s.vy);
          if (sp > 0.05) {
            hx += s.vx / sp;
            hy += s.vy / sp;
          }
        }
        const s = tr.sample(t, shot.drone);
        if (Math.hypot(hx, hy) < 0.3) {
          hx = Math.cos(s.yaw);
          hy = Math.sin(s.yaw);
        }
        const h = Math.hypot(hx, hy);
        hx /= h;
        hy /= h;
        // Average the drone's position over a short window too, so the frame does not shake.
        let mx = 0;
        let my = 0;
        let mz = 0;
        for (let k = -4; k <= 4; k++) {
          const q = tr.sample(Math.max(0, t + k * 0.1), shot.drone);
          mx += q.x / 9;
          my += q.y / 9;
          mz += q.z / 9;
        }
        // Steep (about 70 degrees down), so the cut walls read as a floor plan and not a canyon,
        // with the drone a little below centre and its way ahead in view.
        cam.pos.copy(toThree(mx - hx * 2.3, my - hy * 2.3, Math.max(CUT_H + 5.4, mz + 6.6)));
        cam.target.copy(toThree(mx + hx * 0.4, my + hy * 0.4, mz - 0.5));
        cam.fov = 44;
        cam.wide = 0;
        return;
      }
      case 'crane': {
        // Rise to look straight down at the whole floor, turning slowly.
        const u = smooth((tau - start) / Math.max(4, this.simEndFilm - start + 1));
        cam.target.copy(center);
        this.orbitPoint(-0.85 - 0.6 * u, 0.9 + 0.42 * u, S * (1.1 + 0.4 * u), cam.target, cam.pos);
        cam.fov = 40;
        cam.wide = 1;
        return;
      }
      case 'top': {
        // Nearly straight down on the finished floor plan, with the building in the left half of the
        // frame and the results on the right.
        const u = (tau - start) / OUTRO_S;
        cam.target.copy(center);
        this.orbitPoint(-Math.PI / 2 + 0.05 * u, 1.36, S * 1.78, cam.target, cam.pos);
        const fwd = new THREE.Vector3().subVectors(cam.target, cam.pos).normalize();
        const right = new THREE.Vector3().crossVectors(fwd, new THREE.Vector3(0, 1, 0)).normalize();
        cam.pos.addScaledVector(right, S * 0.56);
        cam.target.addScaledVector(right, S * 0.56);
        cam.fov = 40;
        cam.wide = 1;
        return;
      }
    }
  }
}
