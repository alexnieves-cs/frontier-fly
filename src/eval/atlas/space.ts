// The failure atlas's search space: every way a mission can vary, the declared operating
// distribution over it (the "nominal"), and the proposals the cross-entropy search learns.
//
// A case is a point in this space plus two nuisance seeds (the site layout and where targets are
// planted), which are always drawn as the nominal draws them and never searched. Every searched
// dimension is categorical: discrete choices directly, continuous ones as BINS equal bins of the unit
// interval with a uniform draw inside the bin. A distribution is then one probability vector per
// dimension, so the cross-entropy update is a weighted count and the likelihood ratio between two
// distributions is exact.
import { OBJECT_IDS } from '../../targets/catalog';
import { PERSON_STATES } from '../../targets/types';
import type { Fault } from '../../mission/autonomy/faults';
import type { MissionForWorldOptions, MissionWorldOptions, TargetWant } from '../../mission/autonomy/contracts';
import { RNG } from '../../shared/rng';

/** Bins per continuous dimension. */
export const BINS = 10;

export const MISSIONS = ['find-person', 'sweep-occupied', 'sweep-empty', 'find-object'] as const;
export const FAULTS = ['none', 'lidar-flagged', 'lidar-silent', 'lidar-noise', 'camera-dropout', 'gnss-loss', 'motor', 'battery-sag', 'comms-loss'] as const;
export const LIGHTS = ['on', 'off', 'seeded'] as const;

export interface Dim {
  id: string;
  /** What a reader sees on the atlas page. */
  label: string;
  /** Category names; continuous dimensions list their bins. */
  values: readonly string[];
  continuous: boolean;
  /** The nominal probability of each value. */
  nominal: readonly number[];
}

const uniform = (n: number): number[] => Array.from({ length: n }, () => 1 / n);
const bins = (fmt: (lo: number, hi: number) => string): string[] =>
  Array.from({ length: BINS }, (_, k) => fmt(k / BINS, (k + 1) / BINS));
const continuous = (id: string, label: string, fmt: (lo: number, hi: number) => string): Dim => ({
  id,
  label,
  values: bins(fmt),
  continuous: true,
  nominal: uniform(BINS),
});

/** Mean wind for a unit draw u, m/s: 6u^2, so half of all flights see under 1.5 m/s and one in seven over 4. */
export const windMeanMps = (u: number): number => 6 * u * u;
const hours = (h: number): string => `${String(Math.floor(h)).padStart(2, '0')}:${String(Math.round((h % 1) * 60)).padStart(2, '0')}`;

/**
 * The declared operating distribution. The rates the atlas publishes are rates under THIS
 * distribution: a mix of the four mission kinds, one to three buildings, any time of day, mostly
 * light wind, and a sensor or airframe fault on two flights in five. The conditional rates on the
 * page let a reader re-weight it.
 */
export const DIMS: readonly Dim[] = [
  { id: 'mission', label: 'Mission', values: MISSIONS, continuous: false, nominal: [0.35, 0.25, 0.2, 0.2] },
  { id: 'buildings', label: 'Buildings', values: ['1', '2', '3'], continuous: false, nominal: [0.4, 0.35, 0.25] },
  { id: 'personState', label: 'Person pose', values: PERSON_STATES, continuous: false, nominal: uniform(PERSON_STATES.length) },
  { id: 'object', label: 'Object', values: OBJECT_IDS, continuous: false, nominal: uniform(OBJECT_IDS.length) },
  { id: 'decoys', label: 'Decoys', values: ['0', '1', '2', '3'], continuous: false, nominal: uniform(4) },
  { id: 'lights', label: 'Interior lights', values: LIGHTS, continuous: false, nominal: uniform(LIGHTS.length) },
  { id: 'fault', label: 'Fault', values: FAULTS, continuous: false, nominal: [0.6, ...Array.from({ length: FAULTS.length - 1 }, () => 0.4 / (FAULTS.length - 1))] },
  continuous('targetBuilding', 'Target building (share of the list)', (lo, hi) => `${lo.toFixed(1)}-${hi.toFixed(1)}`),
  continuous('timeOfDay', 'Time of day', (lo, hi) => `${hours(lo * 24)}-${hours(hi * 24)}`),
  continuous('windMean', 'Mean wind, m/s', (lo, hi) => `${windMeanMps(lo).toFixed(1)}-${windMeanMps(hi).toFixed(1)}`),
  continuous('windDir', 'Wind from, deg', (lo, hi) => `${Math.round(lo * 360)}-${Math.round(hi * 360)}`),
  continuous('windGust', 'Gusts, share of the mean', (lo, hi) => `${(0.2 + 0.4 * lo).toFixed(2)}-${(0.2 + 0.4 * hi).toFixed(2)}`),
  continuous('windOnset', 'Wind arrives, s', (lo, hi) => `${Math.round(lo * 240)}-${Math.round(hi * 240)}`),
  continuous('faultOnset', 'Fault onset (share of its window)', (lo, hi) => `${lo.toFixed(1)}-${hi.toFixed(1)}`),
  continuous('faultSeverity', 'Fault severity', (lo, hi) => `${lo.toFixed(1)}-${hi.toFixed(1)}`),
  continuous('faultAngle', 'Fault bearing or motor', (lo, hi) => `${Math.round(lo * 360)}-${Math.round(hi * 360)}`),
];

export const DIM_INDEX: Readonly<Record<string, number>> = Object.fromEntries(DIMS.map((d, i) => [d.id, i]));

/** One probability vector per dimension, in DIMS order. */
export type Proposal = number[][];

export function nominalProposal(): Proposal {
  return DIMS.map((d) => [...d.nominal]);
}

/** A sampled point: the chosen category (or bin) per dimension and, for continuous ones, the value in [0, 1). */
export interface AtlasCase {
  /** Index within its run; seeds derive from it. */
  i: number;
  layoutSeed: number;
  placementSeed: number;
  k: number[];
  u: number[];
}

/** Draws case i of a run. Deterministic in (seedBase, i, proposal). */
export function sampleCase(proposal: Proposal, seedBase: number, i: number): AtlasCase {
  const rng = new RNG((seedBase * 7919 + i) >>> 0, 'atlas-case');
  const k: number[] = [];
  const u: number[] = [];
  for (let d = 0; d < DIMS.length; d++) {
    const p = proposal[d]!;
    let r = rng.next();
    let pick = p.length - 1;
    for (let j = 0; j < p.length; j++) {
      r -= p[j]!;
      if (r < 0) {
        pick = j;
        break;
      }
    }
    k.push(pick);
    u.push(DIMS[d]!.continuous ? (pick + rng.next()) / BINS : 0);
  }
  return { i, layoutSeed: seedBase + i, placementSeed: (seedBase * 31 + i * 7 + 3) >>> 0, k, u };
}

/** log of q(case) for a proposal; the uniform draw inside a bin cancels in every ratio, so it is left out. */
export function logDensity(proposal: Proposal, c: AtlasCase): number {
  let l = 0;
  for (let d = 0; d < DIMS.length; d++) l += Math.log(proposal[d]![c.k[d]!]!);
  return l;
}

/** Defensive mixture: alpha of the nominal and the rest of the learned proposal, per dimension. */
export function mixture(proposal: Proposal, alpha: number): Proposal {
  const nominal = nominalProposal();
  return proposal.map((p, d) => p.map((x, j) => alpha * nominal[d]![j]! + (1 - alpha) * x));
}

/** Likelihood ratio p/q of a case drawn from q. */
export function weight(sampling: Proposal, c: AtlasCase): number {
  return Math.exp(logDensity(nominalProposal(), c) - logDensity(sampling, c));
}

// ------------------------------------------------------------ decoding

export interface DecodedCase {
  mission: (typeof MISSIONS)[number];
  buildings: number;
  targetBuilding: number;
  personState: (typeof PERSON_STATES)[number];
  object: string;
  decoys: number;
  lights: (typeof LIGHTS)[number];
  timeOfDay: number;
  flashlightOn: boolean;
  wind: { meanMps: number; dirDeg: number; gustMps: number; onsetSec: number };
  fault: (typeof FAULTS)[number];
  faultPlan: Fault[];
}

const val = (c: AtlasCase, id: string): string => DIMS[DIM_INDEX[id]!]!.values[c.k[DIM_INDEX[id]!]!]!;
const unit = (c: AtlasCase, id: string): number => c.u[DIM_INDEX[id]!]!;

/** Hours of the day when the operator flies with the flashlight on. */
export const isDark = (h: number): boolean => h < 6.5 || h >= 19.5;

function faultPlan(kind: DecodedCase['fault'], onset: number, sev: number, angle: number): Fault[] {
  switch (kind) {
    case 'none':
      return [];
    case 'lidar-flagged':
    case 'lidar-silent': {
      const flagged = kind === 'lidar-flagged';
      return [
        {
          id: 'lidar',
          kind: 'lidar_sector',
          trigger: { phase: 'indoor_explore', afterSec: 5 + 40 * onset },
          durationSec: Infinity,
          centerDeg: angle * 360 - 180,
          widthDeg: flagged ? 45 + 90 * sev : 30 + 60 * sev,
          mode: flagged ? 'flagged' : 'silent',
        },
      ];
    }
    case 'lidar-noise':
      return [{ id: 'noise', kind: 'lidar_noise', trigger: { atSec: 0 }, durationSec: Infinity, sigmaM: 0.05 + 0.2 * sev, missRate: 0.02 + 0.1 * sev }];
    case 'camera-dropout':
      return [{ id: 'cam', kind: 'camera_dropout', trigger: { phase: 'coverage', afterSec: 10 + 110 * onset }, durationSec: sev < 0.7 ? 3 + (40 * sev) / 0.7 : Infinity }];
    case 'gnss-loss':
      return [{ id: 'gnss', kind: 'gnss_loss', trigger: { phase: 'coverage', afterSec: 10 + 110 * onset }, durationSec: 2 + 58 * sev, driftMps: 0.15 }];
    case 'motor':
      return [{ id: 'motor', kind: 'motor', trigger: { atSec: 30 + 150 * onset }, durationSec: Infinity, motor: Math.min(3, Math.floor(angle * 4)) as 0 | 1 | 2 | 3, efficiency: 0.95 - 0.45 * sev }];
    case 'battery-sag':
      return [{ id: 'pack', kind: 'battery_sag', trigger: { atSec: 60 + 150 * onset }, durationSec: Infinity, dropPct: 5 + 25 * sev, drawScale: 1 + 0.5 * sev }];
    case 'comms-loss':
      return [{ id: 'link', kind: 'comms_loss', trigger: { atSec: 20 + 130 * onset }, durationSec: sev < 0.6 ? 4 + 50 * sev : Infinity }];
  }
}

export function decode(c: AtlasCase): DecodedCase {
  const buildings = Number(val(c, 'buildings'));
  const timeOfDay = unit(c, 'timeOfDay') * 24;
  const meanMps = windMeanMps(unit(c, 'windMean'));
  const fault = val(c, 'fault') as DecodedCase['fault'];
  const wind = {
    meanMps,
    dirDeg: unit(c, 'windDir') * 360,
    gustMps: meanMps * (0.2 + 0.4 * unit(c, 'windGust')),
    onsetSec: unit(c, 'windOnset') * 240,
  };
  const plan = faultPlan(fault, unit(c, 'faultOnset'), unit(c, 'faultSeverity'), unit(c, 'faultAngle'));
  if (meanMps >= 0.05) {
    plan.push({ id: 'wind', kind: 'wind', trigger: { atSec: wind.onsetSec }, durationSec: Infinity, meanMps, dirDeg: wind.dirDeg, gustMps: wind.gustMps, gustTauSec: 3 });
  }
  return {
    mission: val(c, 'mission') as DecodedCase['mission'],
    buildings,
    targetBuilding: Math.min(buildings - 1, Math.floor(unit(c, 'targetBuilding') * buildings)),
    personState: val(c, 'personState') as DecodedCase['personState'],
    object: val(c, 'object'),
    decoys: Number(val(c, 'decoys')),
    lights: val(c, 'lights') as DecodedCase['lights'],
    timeOfDay,
    flashlightOn: isDark(timeOfDay),
    wind,
    fault,
    faultPlan: plan,
  };
}

/** The world, mission and faults a case flies. */
export function caseSetup(c: AtlasCase): { world: MissionWorldOptions; mission: MissionForWorldOptions; faults: Fault[]; decoded: DecodedCase } {
  const d = decode(c);
  const targets: TargetWant[] =
    d.mission === 'find-person' || d.mission === 'sweep-occupied'
      ? [{ classId: 'person', state: d.personState, where: 'indoor', building: d.targetBuilding }]
      : d.mission === 'find-object'
        ? [{ classId: d.object as TargetWant['classId'], where: 'indoor', building: d.targetBuilding }]
        : [];
  const budgets = { batteryPct: 85, timeSec: 600 + 300 * d.buildings };
  const mission: MissionForWorldOptions =
    d.mission === 'find-person'
      ? {
          missionType: 'find_person',
          name: 'Atlas: find a person',
          targets: [{ id: 'tgt-person', kind: 'person', label: 'Missing person (simulated)', minCount: 1, minConfidence: null }],
          requiredConfidence: 0.8,
          entryPolicy: 'auto',
          budgets,
        }
      : d.mission === 'find-object'
        ? {
            missionType: 'find_objects',
            name: 'Atlas: find an object',
            targets: [{ id: 'tgt-object', kind: 'object', label: 'Dropped object', minCount: 1, minConfidence: null }],
            requiredConfidence: 0.8,
            entryPolicy: 'auto',
            budgets,
          }
        : {
            missionType: 'safety_sweep',
            name: 'Atlas: confirm clear',
            targets: [{ id: 'tgt-clear', kind: 'clearance', label: 'Confirm every building is clear', minCount: 1, minConfidence: null }],
            requiredConfidence: 0.8,
            entryPolicy: 'ask',
            budgets,
          };
  const world: MissionWorldOptions = {
    seed: c.layoutSeed,
    placementSeed: c.placementSeed,
    buildings: d.buildings,
    timeOfDay: d.timeOfDay,
    flashlightOn: d.flashlightOn,
    interiorLights: d.lights,
    targets,
    decoys: d.decoys,
  };
  return { world, mission, faults: d.faultPlan, decoded: d };
}
