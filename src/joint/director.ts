import * as THREE from 'three';
import { solutionAt, spoofedAt, truthAt, type LegBundle } from './leg';
import type { LegScene } from './scene';

/** Seconds of establishing shot before the leg's first epoch, and of hold after its last. */
export const INTRO_S = 5;
export const OUTRO_S = 6;

interface Shot {
  name: string;
  /** Leg time the shot is fully in. */
  from: number;
  to: number;
  pose: (t: number) => { eye: THREE.Vector3; look: THREE.Vector3; fov: number };
}

const smooth = (u: number) => u * u * (3 - 2 * u);

/**
 * The camera for the leg film, a pure function of time so frames render in any order: an establishing
 * orbit over the whole tile, a chase behind the drone with its camera footprint on the ground, a high
 * tracking view as the spoof pulls the GPS track away, a side view with the GPS-trusting ghost, and a
 * pull-back over the whole flight. Shots cross-fade over BLEND_S.
 */
export class LegDirector {
  readonly camera = new THREE.PerspectiveCamera(50, 16 / 9, 0.05, 20000);
  private readonly shots: Shot[];
  private static readonly BLEND_S = 2.6;

  constructor(
    private readonly leg: LegBundle,
    private readonly scene: LegScene,
  ) {
    const E = leg.epochs;
    const t0 = E[0]!.t;
    const t1 = E[E.length - 1]!.t;
    const ts = leg.spoof.startS;
    const tr = leg.firstRejectS.gated ?? ts + 10;
    const w = (x: number, y: number, h = 0) => scene.w(x, y, h);
    const ext = leg.ground.extent;
    const span = Math.max(ext.xmax - ext.xmin, ext.ymax - ext.ymin);
    const centre = w((ext.xmin + ext.xmax) / 2, (ext.ymin + ext.ymax) / 2, 0);
    const drone = (t: number) => {
      const s = truthAt(leg, t);
      return { s, p: w(s.x, s.y, s.agl), g: w(s.x, s.y, 0) };
    };
    const fwdOf = (heading: number) => {
      const th = THREE.MathUtils.degToRad(heading);
      return new THREE.Vector3(Math.sin(th), 0, -Math.cos(th));
    };
    const rightOf = (heading: number) => {
      const th = THREE.MathUtils.degToRad(heading);
      return new THREE.Vector3(Math.cos(th), 0, Math.sin(th));
    };

    // Which side of the path has more imagery. The oblique shots sit on that side and look across the path,
    // so what lies beyond the tile reads as a dark horizon at the top of the frame, not a void underfoot.
    const midE = E[Math.floor(E.length / 2)]!;
    const thMid = THREE.MathUtils.degToRad(truthAt(leg, midE.t).heading);
    const reach = (sx: number, sy: number) => {
      const tx = sx > 0 ? (ext.xmax - midE.truth[0]) / sx : sx < 0 ? (ext.xmin - midE.truth[0]) / sx : Infinity;
      const ty = sy > 0 ? (ext.ymax - midE.truth[1]) / sy : sy < 0 ? (ext.ymin - midE.truth[1]) / sy : Infinity;
      return Math.min(tx, ty);
    };
    const interior = reach(Math.cos(thMid), -Math.sin(thMid)) >= reach(-Math.cos(thMid), Math.sin(thMid)) ? 1 : -1;

    this.shots = [
      {
        // South of the tile, looking north over the real imagery towards where the drone starts.
        name: 'establish',
        from: t0 - INTRO_S,
        to: t0 + 0.5,
        pose: (t) => {
          const u = (t - (t0 - INTRO_S)) / (INTRO_S + 0.5);
          const a = -0.42 + u * 0.3;
          const d = drone(t0).g;
          const look = centre.clone().lerp(d, 0.45 + 0.3 * smooth(u));
          const R = span * (0.52 - 0.12 * smooth(u));
          const eye = look.clone().add(new THREE.Vector3(Math.sin(a) * R, R * 0.72, Math.cos(a) * R));
          return { eye, look, fov: 44 };
        },
      },
      {
        // Behind and above the drone: the drone in the top third, its camera footprint below it.
        name: 'chase',
        from: t0 + 3,
        to: Math.min(ts - 3, t0 + 15),
        pose: (t) => {
          const { s, p, g } = drone(t);
          const f = fwdOf(s.heading);
          const r = rightOf(s.heading);
          const sway = Math.sin((t - t0) * 0.25) * 8;
          const eye = p.clone().addScaledVector(f, -30).addScaledVector(r, interior * (7 + sway)).add(new THREE.Vector3(0, 30, 0));
          const look = p.clone().lerp(g.clone().addScaledVector(f, 6), 0.42);
          return { eye, look, fov: 64 };
        },
      },
      {
        // High and oblique as the spoof starts: the red GPS track peels away from the true path.
        name: 'track',
        from: ts - 0.5,
        to: tr + 4,
        pose: (t) => {
          const { s, g } = drone(t);
          const sp = spoofedAt(leg, t);
          const mid = g.clone().lerp(w(sp[0], sp[1], 0), 0.5);
          const f = fwdOf(s.heading);
          const r = rightOf(s.heading);
          const eye = mid.clone().addScaledVector(f, -95).addScaledVector(r, interior * 30).add(new THREE.Vector3(0, 265, 0));
          const look = mid.clone().addScaledVector(f, 18);
          return { eye, look, fov: 42 };
        },
      },
      {
        // The drone and the red ghost of where a GPS-trusting autopilot thinks it is.
        name: 'ghost',
        from: tr + 7,
        to: Math.min(t1 - 9, tr + 17),
        pose: (t) => {
          const { s, g } = drone(t);
          const b = solutionAt(leg, t, 'blind') ?? [s.x, s.y];
          const gg = w(b[0], b[1], 0);
          const mid = g.clone().lerp(gg, 0.5);
          const r = rightOf(s.heading);
          const f = fwdOf(s.heading);
          const sep = g.distanceTo(gg);
          const eye = mid.clone().addScaledVector(f, -70).addScaledVector(r, interior * (55 + sep * 0.7)).add(new THREE.Vector3(0, 215, 0));
          return { eye, look: mid.clone().addScaledVector(f, 10).add(new THREE.Vector3(0, 78, 0)), fov: 46 };
        },
      },
      {
        name: 'pullback',
        from: t1 - 6,
        to: t1 + OUTRO_S,
        pose: (t) => {
          const u = smooth(Math.max(0, Math.min(1, (t - (t1 - 6)) / 9)));
          const { g } = drone(Math.min(t, t1));
          const pathMid = w((E[0]!.truth[0] + E[E.length - 1]!.truth[0]) / 2, (E[0]!.truth[1] + E[E.length - 1]!.truth[1]) / 2, 0);
          const look = g.clone().lerp(pathMid, u);
          const height = 160 + u * span * 0.5;
          const eye = look.clone().add(new THREE.Vector3(-height * 0.12, height, height * 0.62));
          return { eye, look, fov: 44 };
        },
      },
    ];
  }

  /** Which shot owns time t, for the HUD's captions. */
  shotAt(t: number): string {
    let best = this.shots[0]!;
    for (const s of this.shots) if (t >= s.from - LegDirector.BLEND_S / 2) best = s;
    return best.name;
  }

  private dronePos(t: number): THREE.Vector3 {
    const s = truthAt(this.leg, t);
    return this.scene.w(s.x, s.y, s.agl);
  }

  frame(t: number): void {
    const B = LegDirector.BLEND_S;
    // The shot in force, or the blend from one into the next across the gap between them.
    let i = 0;
    while (i < this.shots.length - 1 && t > this.shots[i]!.to) i++;
    const cur = this.shots[i]!;
    const prev = i > 0 ? this.shots[i - 1]! : null;
    let pose = cur.pose(t);
    if (prev && t < cur.from) {
      const gap = Math.max(B, cur.from - prev.to);
      const u = smooth(Math.max(0, Math.min(1, (t - (cur.from - gap)) / gap)));
      const a = prev.pose(t);
      pose = {
        eye: a.eye.clone().lerp(pose.eye, u),
        look: a.look.clone().lerp(pose.look, u),
        fov: a.fov + (pose.fov - a.fov) * u,
      };
    }
    this.camera.position.copy(pose.eye);
    this.camera.lookAt(pose.look);
    this.camera.fov = pose.fov;
    // Near plane from the distance to the drone, the closest thing worth drawing: a fixed 5 cm near
    // plane leaves too little depth precision a kilometre out, and the ground marks z-fight the photo.
    this.camera.near = Math.max(0.05, Math.min(12, 0.3 * pose.eye.distanceTo(this.dronePos(t))));
    this.camera.updateProjectionMatrix();
  }
}
