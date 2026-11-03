/**
 * The physics validation's test flights, shared by every simulator.
 *
 *   npx vite-node scripts/physics/maneuvers.ts -- --out test-results/physics/maneuvers.json
 *
 * writes them as JSON for scripts/physics/fly_px4.py (PX4 in offboard mode)
 * and scripts/physics/native.ts (frontier-fly's own flight controller) reads
 * them directly. Positions and velocities are NED meters from the takeoff
 * pad, as PX4 takes them. An axis with a position holds it with a P loop; an
 * axis with a velocity flies that velocity. Each flight starts in a hover at
 * `start` and runs its segments from t = 0.
 */
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname } from 'node:path';
import { missionQuadParams } from '../../src/mission/autonomy/quadVehicle';
import { px4Geometry } from '../../src/px4/frontierPlant';
import { DEFAULT_BATTERY } from '../../src/mission/autonomy/battery';
import type { GustSpec } from '../../src/sim/dynamics/gust';
import type { DynParams } from '../../src/shared/types';

export type Sim = 'sih' | 'bridge' | 'native';
type Axis = number | null;

export interface Segment {
  dur: number;
  pos?: [Axis, Axis, Axis];
  vel?: [Axis, Axis, Axis];
}

export interface Maneuver {
  id: string;
  title: string;
  sims: Sim[];
  start: [number, number, number];
  segments: Segment[];
  wind?: GustSpec;
  /** Fixed pack state of charge, 0..1: the motors' top speed follows the loaded pack voltage (pack.ts). */
  soc?: number;
  /** Near-surface aerodynamics on (aero.ts), with an optional ceiling this far above the pad, m. */
  aero?: { ceilingM?: number };
}

const H = 5;
const hold = (dur: number, d = -H): Segment => ({ dur, pos: [0, 0, d] });

export const MANEUVERS: Maneuver[] = [
  {
    id: 'step-forward',
    title: 'Forward speed step: hover, 4 m/s north for 6 s, then stop',
    sims: ['sih', 'bridge', 'native'],
    start: [0, 0, -H],
    segments: [hold(3), { dur: 6, vel: [4, 0, null], pos: [null, null, -H] }, { dur: 8, vel: [0, 0, null], pos: [null, null, -H] }],
  },
  {
    id: 'step-climb',
    title: 'Climb step: hover, climb at 1.5 m/s for 3 s, then hold height',
    sims: ['sih', 'bridge', 'native'],
    start: [0, 0, -H],
    segments: [hold(3), { dur: 3, vel: [null, null, -1.5], pos: [0, 0, null] }, { dur: 6, vel: [null, null, 0], pos: [0, 0, null] }],
  },
  {
    id: 'gust-light',
    title: 'Position hold in light wind: 2 m/s mean, 0.75 m/s gusts',
    sims: ['bridge', 'native'],
    start: [0, 0, -H],
    segments: [hold(60)],
    wind: { meanMps: 2, dirDeg: 30, gustMps: 0.75, gustTauSec: 3, seed: 1 },
  },
  {
    id: 'gust-moderate',
    title: 'Position hold in moderate wind: 4 m/s mean, 1.5 m/s gusts',
    sims: ['bridge', 'native'],
    start: [0, 0, -H],
    segments: [hold(60)],
    wind: { meanMps: 4, dirDeg: 30, gustMps: 1.5, gustTauSec: 3, seed: 2 },
  },
  ...[1, 0.3, 0.08].map(
    (soc): Maneuver => ({
      id: `sag-${Math.round(soc * 100)}`,
      title: `Full-power dash on a ${Math.round(soc * 100)}% pack: 5 m/s north while climbing at 1.5 m/s`,
      sims: ['bridge', 'native'],
      start: [0, 0, -H],
      segments: [hold(3), { dur: 4, vel: [5, 0, -1.5] }, { dur: 6, vel: [0, 0, 0] }],
      soc,
    }),
  ),
  {
    id: 'ceiling',
    title: 'Creep up under a ceiling 6 m above the pad: hold 0.6 m below it, then 0.12 m (about one rotor radius)',
    sims: ['bridge', 'native'],
    start: [0, 0, -5.4],
    segments: [hold(5, -5.4), hold(8, -5.88), hold(5, -5.4)],
    aero: { ceilingM: 6 },
  },
  {
    id: 'ground',
    title: 'Hover low over the ground: 2 m, then 0.12 m (about one rotor radius), then back to 2 m',
    sims: ['bridge', 'native'],
    start: [0, 0, -2],
    segments: [hold(4, -2), hold(8, -0.12), hold(5, -2)],
    aero: {},
  },
];

/** The airframe every simulator flies: the one airframe (shared/airframe.ts) the missions fly. */
export function airframe(): { dyn: DynParams; rotorRadiusM: number } {
  return { dyn: missionQuadParams(), rotorRadiusM: DEFAULT_BATTERY.rotorDiameterM / 2 };
}

/** PX4 parameters that put PX4 on the same airframe, for each PX4-side simulator. */
export function px4Params(sim: 'sih' | 'bridge'): Record<string, number> {
  const { dyn } = airframe();
  const tMax = dyn.kf * 1100 * 1100;
  const hover = (dyn.mass * 9.81) / 4 / tMax;
  const common: Record<string, number> = {
    MPC_TILTMAX_AIR: 55,
    MPC_THR_HOVER: +hover.toFixed(3),
    MPC_XY_VEL_MAX: 12,
    COM_RCL_EXCEPT: 4,
    NAV_RCL_ACT: 0,
    COM_OF_LOSS_T: 2,
  };
  if (sim === 'bridge') {
    const geo = px4Geometry(dyn);
    const p: Record<string, number> = { ...common, THR_MDL_FAC: 1 };
    geo.forEach((r, i) => {
      p[`CA_ROTOR${i}_PX`] = +r.px.toFixed(4);
      p[`CA_ROTOR${i}_PY`] = +r.py.toFixed(4);
      p[`CA_ROTOR${i}_KM`] = +r.km.toFixed(4);
    });
    return p;
  }
  const a = dyn.arm * Math.SQRT1_2;
  const ratio = dyn.km / dyn.kf;
  // SIH's motor order and spin are fixed in its equations (airframe 10040): 0 front-right and 1 rear-left counter-clockwise.
  const sih: Record<string, number> = {
    ...common,
    THR_MDL_FAC: 0,
    SIH_MASS: dyn.mass,
    SIH_IXX: +dyn.inertia[0].toFixed(6),
    SIH_IYY: +dyn.inertia[1].toFixed(6),
    SIH_IZZ: +dyn.inertia[2].toFixed(6),
    SIH_T_MAX: +tMax.toFixed(4),
    SIH_Q_MAX: +(ratio * tMax).toFixed(5),
    SIH_L_ROLL: +a.toFixed(4),
    SIH_L_PITCH: +a.toFixed(4),
    SIH_KDV: +sihDrag(dyn).toFixed(3),
    SIH_KDW: 0,
    SIH_T_TAU: dyn.motorTau,
  };
  const pos: [number, number, number][] = [
    [1, 1, 1],
    [-1, -1, 1],
    [1, -1, -1],
    [-1, 1, -1],
  ];
  pos.forEach(([x, y, s], i) => {
    sih[`CA_ROTOR${i}_PX`] = +(x * a).toFixed(4);
    sih[`CA_ROTOR${i}_PY`] = +(y * a).toFixed(4);
    sih[`CA_ROTOR${i}_KM`] = +(s * ratio).toFixed(4);
  });
  return sih;
}

/**
 * SIH has one linear drag coefficient. Frontier-fly's drag is linear plus
 * quadratic: fit the line through the origin over 0..5 m/s (least squares,
 * uniform speeds), which matches at about 4 m/s.
 */
export function sihDrag(p: DynParams): number {
  let num = 0;
  let den = 0;
  for (let v = 0.05; v <= 5; v += 0.05) {
    const f = p.dragLin[0] * v + p.dragQuad[0] * v * v;
    num += f * v;
    den += v * v;
  }
  return num / den;
}

if (process.env.VITEST === undefined && process.argv.includes('--out')) {
  const out = process.argv[process.argv.indexOf('--out') + 1]!;
  mkdirSync(dirname(out), { recursive: true });
  writeFileSync(out, `${JSON.stringify({ maneuvers: MANEUVERS, params: { sih: px4Params('sih'), bridge: px4Params('bridge') } }, null, 1)}\n`);
  console.log(`wrote ${MANEUVERS.length} maneuvers to ${out}`);
}
