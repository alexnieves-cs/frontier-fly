import { RNG } from '../shared/rng';
import type { PersonState, TargetClass } from '../targets/types';
import type { Paint } from './raster';

/** Surface paints for the camera. Mirrors the 3D view's materials (src/viz/three, src/targets/meshes.ts). */
export interface Palette {
  wall: Paint[];
  floor: Paint;
  ceiling: Paint;
  frame: Paint;
  ground: Paint;
  trunk: Paint;
  pine: Paint;
  leaf: Paint;
  crate: Paint;
  chair: Paint;
  furniture: Record<'table' | 'chair' | 'shelf' | 'plant' | 'sofa', Paint>;
  skin: Paint;
  pants: Paint;
  mannequin: Paint;
  /** Shirt for a person; the 3D view colors people by state. */
  shirt(state: PersonState | 'static', id: string): Paint;
  prop(classId: TargetClass): Paint;
  /** Dark detail parts: backpack straps, toolbox handle. */
  strap: Paint;
  cross: Paint;
  /** Multiplier on every light, for exposure variety in training. */
  lightGain: number;
}

export function paint(hex: string, grain = 0, gloss = 0, heat = 0): Paint {
  const n = parseInt(hex.slice(1), 16);
  // sRGB hex to linear, so lights add the way they do in the 3D view.
  const lin = (c: number): number => Math.pow(c / 255, 2.2);
  return { r: lin((n >> 16) & 255), g: lin((n >> 8) & 255), b: lin(n & 255), grain, heat, gloss };
}

const SHIRT: Record<PersonState, string> = {
  standing: '#e2b08a',
  sitting: '#4c8dff',
  lying: '#e25b4a',
  walking: '#3cba6a',
  hiding: '#6d7c8a',
};

const PROP: Partial<Record<TargetClass, string>> = {
  medkit: '#f4f7fb',
  phone: '#2a3344',
  toolbox: '#d4893a',
  decoy_case: '#d4893a',
  backpack: '#3d6b4f',
  decoy_pack: '#3d6b4f',
  bottle: '#7fd0ea',
  radio: '#243044',
  helmet: '#f0d36a',
  blanket: '#c46a62',
  coat: '#c4a574',
};

/** Wall paint by SURFACE class: drywall, concrete, wood, glass (drawn as wood in the camera view), metal. */
const WALLS = ['#d8d2c8', '#9a9a96', '#8a6a4a', '#8a6a4a', '#9ea4ab'];

export function defaultPalette(): Palette {
  const props = new Map<TargetClass, Paint>();
  return {
    wall: WALLS.map((hex, i) => paint(hex, i === 1 ? 0.22 : 0.12)),
    floor: paint('#9c7a56', 0.28),
    ceiling: paint('#d9d4cc', 0.08),
    frame: paint('#5c4634', 0.1),
    ground: paint('#5d5638', 0.4),
    trunk: paint('#5a4330', 0.3),
    pine: paint('#2f5a34', 0.35),
    leaf: paint('#47703a', 0.35),
    crate: paint('#8a5a32', 0.25),
    chair: paint('#6b4630', 0.15),
    furniture: {
      table: paint('#7a5636', 0.2),
      chair: paint('#6b4630', 0.15),
      shelf: paint('#806040', 0.2),
      plant: paint('#3f7a3a', 0.3),
      sofa: paint('#5a6680', 0.15),
    },
    skin: paint('#d9a582', 0, 0.05, 1),
    pants: paint('#30384a', 0.05, 0, 0.9),
    mannequin: paint('#d5dae1', 0, 0.6),
    shirt: (state) => paint(SHIRT[state === 'static' ? 'standing' : state], 0.05, 0, 0.9),
    prop(classId) {
      let p = props.get(classId);
      if (!p) {
        p = paint(PROP[classId] ?? '#cfd3d8', 0.04, classId === 'phone' || classId === 'radio' ? 0.3 : 0.05);
        props.set(classId, p);
      }
      return p;
    },
    strap: paint('#1d2a22', 0.05),
    cross: paint('#d64545'),
    lightGain: 1,
  };
}

function jitter(p: Paint, rng: RNG, amount: number): Paint {
  const k = 1 + (rng.next() - 0.5) * amount;
  const t = (rng.next() - 0.5) * amount * 0.5;
  return {
    ...p,
    r: Math.max(0, Math.min(1, p.r * k * (1 + t))),
    g: Math.max(0, Math.min(1, p.g * k)),
    b: Math.max(0, Math.min(1, p.b * k * (1 - t))),
  };
}

function hsv(h: number, s: number, v: number): Paint {
  const f = (n: number): number => {
    const k = (n + h * 6) % 6;
    return v - v * s * Math.max(0, Math.min(k, 4 - k, 1));
  };
  return { r: Math.pow(f(5), 2.2), g: Math.pow(f(3), 2.2), b: Math.pow(f(1), 2.2), grain: 0.05, gloss: 0, heat: 0.9 };
}

/**
 * Domain randomization for training frames: surface tints, exposure, and
 * shirts drawn from the whole hue circle, so the detector does not learn
 * "a person is a blue or red blob". The look-alike decoys keep their tells
 * (plain plastic, no straps, no handle): those are what it must learn.
 */
export function randomPalette(seed: number): Palette {
  const rng = new RNG(seed >>> 0, 'palette');
  const base = defaultPalette();
  const shirts = new Map<string, Paint>();
  const props = new Map<TargetClass, Paint>();
  return {
    ...base,
    wall: base.wall.map((p) => jitter(p, rng, 0.5)),
    floor: jitter(base.floor, rng, 0.6),
    ceiling: jitter(base.ceiling, rng, 0.3),
    ground: jitter(base.ground, rng, 0.5),
    leaf: jitter(base.leaf, rng, 0.4),
    pine: jitter(base.pine, rng, 0.4),
    crate: jitter(base.crate, rng, 0.4),
    furniture: {
      table: jitter(base.furniture.table, rng, 0.5),
      chair: jitter(base.furniture.chair, rng, 0.5),
      shelf: jitter(base.furniture.shelf, rng, 0.5),
      plant: jitter(base.furniture.plant, rng, 0.4),
      sofa: hsv(rng.next(), 0.2 + rng.next() * 0.4, 0.3 + rng.next() * 0.4),
    },
    skin: jitter(base.skin, rng, 0.6),
    pants: jitter(base.pants, rng, 0.8),
    shirt(state, id) {
      let p = shirts.get(id);
      if (!p) {
        const r = new RNG(seed ^ hashId(id), 'shirt');
        p = r.next() < 0.3 ? base.shirt(state, id) : hsv(r.next(), 0.15 + r.next() * 0.8, 0.25 + r.next() * 0.7);
        shirts.set(id, p);
      }
      return p;
    },
    prop(classId) {
      let p = props.get(classId);
      if (!p) {
        p = jitter(base.prop(classId), rng, 0.3);
        props.set(classId, p);
      }
      return p;
    },
    lightGain: 0.6 + rng.next() * 0.8,
  };
}

function hashId(id: string): number {
  let h = 2166136261;
  for (let i = 0; i < id.length; i++) h = Math.imul(h ^ id.charCodeAt(i), 16777619);
  return h >>> 0;
}
