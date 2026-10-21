import { RNG } from '../../shared/rng';
import type { Fault } from './faults';

/**
 * Fault scenarios for the fault bench. Each one injects a fault (or a
 * compound of faults) into an ordinary mission, with timing and parameters
 * drawn from the flight seed, and names the hazard it exercises in
 * docs/SAFETY_CASE.md and what a safe response looks like.
 *
 * Responses, from best to worst for the mission (all are safe):
 *   completed      the mission reaches its normal outcome despite the fault
 *   returned_home  the drone gives up and lands on the pad
 *   landed         the drone lands in place, away from the pad
 * Completing or returning home is always safe. Landing in place is safe only
 * where the fault can make getting home impossible (a motor too weak to fly
 * on, a long GNSS outage, a pack that sagged below the trip home, wind the
 * airframe can't make headway against). A flight
 * is unsafe if it is still in the air at the time limit, lands away from home
 * anywhere else, collides, declares a scope clear that holds a person, or
 * flies through an operator-approved entry while the link was down.
 */

export type SafeResponse = 'completed' | 'returned_home' | 'landed';

export interface FaultScenario {
  id: string;
  title: string;
  /** Hazard ids in docs/SAFETY_CASE.md. */
  hazards: string[];
  faults(seed: number): Fault[];
  /** Responses that count as safe for this scenario. */
  safe: SafeResponse[];
  /** The response the failsafe is designed to produce. */
  intended: SafeResponse;
}

function rng(seed: number, id: string): RNG {
  return new RNG(seed >>> 0, `fault-scenario/${id}`);
}

export const FAULT_SCENARIOS: readonly FaultScenario[] = [
  {
    id: 'nominal',
    title: 'No fault (control)',
    hazards: [],
    faults: () => [],
    safe: ['completed', 'returned_home'],
    intended: 'completed',
  },
  {
    id: 'lidar-flagged-indoor',
    title: 'Front lidar arc fails (flagged) inside the building',
    hazards: ['H3'],
    faults: (seed) => {
      const r = rng(seed, 'lidar-flagged-indoor');
      return [{ id: 'lidar', kind: 'lidar_sector', trigger: { phase: 'indoor_explore', afterSec: r.range(5, 25) }, durationSec: Infinity, centerDeg: r.range(-20, 20), widthDeg: 90, mode: 'flagged' }];
    },
    safe: ['completed', 'returned_home'],
    intended: 'returned_home',
  },
  {
    id: 'lidar-silent-indoor',
    title: 'Side lidar arc fails silently (reads open) inside the building',
    hazards: ['H3'],
    faults: (seed) => {
      const r = rng(seed, 'lidar-silent-indoor');
      return [{ id: 'lidar', kind: 'lidar_sector', trigger: { phase: 'indoor_explore', afterSec: r.range(5, 25) }, durationSec: Infinity, centerDeg: r.pick([90, -90]) + r.range(-15, 15), widthDeg: 60, mode: 'silent' }];
    },
    safe: ['completed', 'returned_home'],
    intended: 'returned_home',
  },
  {
    id: 'lidar-noise',
    title: 'Degraded lidar: 0.15 m noise and 5% missed returns all flight',
    hazards: ['H3'],
    faults: () => [{ id: 'noise', kind: 'lidar_noise', trigger: { atSec: 0 }, durationSec: Infinity, sigmaM: 0.15, missRate: 0.05 }],
    safe: ['completed', 'returned_home'],
    intended: 'completed',
  },
  {
    id: 'camera-short',
    title: 'Camera frames drop for 6 s during the outdoor sweep',
    hazards: ['H4'],
    faults: (seed) => [{ id: 'cam', kind: 'camera_dropout', trigger: { phase: 'coverage', afterSec: rng(seed, 'camera-short').range(10, 60) }, durationSec: 6 }],
    safe: ['completed', 'returned_home'],
    intended: 'completed',
  },
  {
    id: 'camera-lost',
    title: 'Camera fails for good',
    hazards: ['H4'],
    faults: (seed) => [{ id: 'cam', kind: 'camera_dropout', trigger: { phase: 'coverage', afterSec: rng(seed, 'camera-lost').range(10, 60) }, durationSec: Infinity }],
    safe: ['completed', 'returned_home'],
    intended: 'returned_home',
  },
  {
    id: 'gnss-short',
    title: 'GNSS fix lost for 3 s outdoors',
    hazards: ['H5'],
    faults: (seed) => [{ id: 'gnss', kind: 'gnss_loss', trigger: { phase: 'coverage', afterSec: rng(seed, 'gnss-short').range(10, 60) }, durationSec: 3, driftMps: 0.15 }],
    safe: ['completed', 'returned_home'],
    intended: 'completed',
  },
  {
    id: 'gnss-lost',
    title: 'GNSS fix lost for 40 s outdoors',
    hazards: ['H5'],
    faults: (seed) => [{ id: 'gnss', kind: 'gnss_loss', trigger: { phase: 'coverage', afterSec: rng(seed, 'gnss-lost').range(10, 60) }, durationSec: 40, driftMps: 0.15 }],
    safe: ['completed', 'returned_home', 'landed'],
    intended: 'returned_home',
  },
  {
    id: 'motor-weak',
    title: 'One motor drops to 80% thrust',
    hazards: ['H6'],
    faults: (seed) => {
      const r = rng(seed, 'motor-weak');
      return [{ id: 'motor', kind: 'motor', trigger: { atSec: r.range(30, 150) }, durationSec: Infinity, motor: r.int(4) as 0 | 1 | 2 | 3, efficiency: 0.8 }];
    },
    safe: ['completed', 'returned_home'],
    intended: 'returned_home',
  },
  {
    id: 'motor-severe',
    title: 'One motor drops to 55% thrust',
    hazards: ['H6'],
    faults: (seed) => {
      const r = rng(seed, 'motor-severe');
      return [{ id: 'motor', kind: 'motor', trigger: { atSec: r.range(30, 150) }, durationSec: Infinity, motor: r.int(4) as 0 | 1 | 2 | 3, efficiency: 0.55 }];
    },
    safe: ['completed', 'returned_home', 'landed'],
    intended: 'landed',
  },
  {
    id: 'battery-sag',
    title: 'Weak cell: gauge drops 25 points, then drains 1.4x faster',
    hazards: ['H7'],
    faults: (seed) => [{ id: 'pack', kind: 'battery_sag', trigger: { atSec: rng(seed, 'battery-sag').range(60, 180) }, durationSec: Infinity, dropPct: 25, drawScale: 1.4 }],
    safe: ['completed', 'returned_home', 'landed'],
    intended: 'returned_home',
  },
  {
    id: 'comms-short',
    title: 'Operator link drops for 8 s',
    hazards: ['H1'],
    faults: (seed) => [{ id: 'link', kind: 'comms_loss', trigger: { atSec: rng(seed, 'comms-short').range(20, 120) }, durationSec: 8 }],
    safe: ['completed', 'returned_home'],
    intended: 'completed',
  },
  {
    id: 'comms-lost',
    title: 'Operator link lost for good',
    hazards: ['H1', 'H2'],
    faults: (seed) => [{ id: 'link', kind: 'comms_loss', trigger: { atSec: rng(seed, 'comms-lost').range(20, 120) }, durationSec: Infinity }],
    safe: ['completed', 'returned_home'],
    intended: 'returned_home',
  },
  {
    id: 'wind-light',
    title: 'Light wind inside the envelope: 2 m/s mean, 0.75 m/s gusts',
    hazards: ['H8'],
    faults: (seed) => [{ id: 'wind', kind: 'wind', trigger: { atSec: 0 }, durationSec: Infinity, meanMps: 2, dirDeg: rng(seed, 'wind-light').range(0, 360), gustMps: 0.75, gustTauSec: 3 }],
    safe: ['completed', 'returned_home'],
    intended: 'completed',
  },
  {
    id: 'wind-moderate',
    title: 'Moderate wind over the 3 m/s limit: 4 m/s mean, 1.5 m/s gusts',
    hazards: ['H8'],
    faults: (seed) => [{ id: 'wind', kind: 'wind', trigger: { atSec: 0 }, durationSec: Infinity, meanMps: 4, dirDeg: rng(seed, 'wind-moderate').range(0, 360), gustMps: 1.5, gustTauSec: 3 }],
    safe: ['completed', 'returned_home'],
    intended: 'returned_home',
  },
  {
    id: 'wind-strong',
    title: 'Strong wind arrives mid-mission: 8 m/s mean, 2 m/s gusts',
    hazards: ['H8'],
    faults: (seed) => {
      const r = rng(seed, 'wind-strong');
      return [{ id: 'wind', kind: 'wind', trigger: { atSec: r.range(30, 120) }, durationSec: Infinity, meanMps: 8, dirDeg: r.range(0, 360), gustMps: 2, gustTauSec: 3 }];
    },
    safe: ['completed', 'returned_home', 'landed'],
    intended: 'returned_home',
  },
  {
    id: 'compound',
    title: 'Light wind, a weak cell and a lost link together',
    hazards: ['H1', 'H7', 'H8'],
    faults: (seed) => {
      const r = rng(seed, 'compound');
      return [
        { id: 'wind', kind: 'wind', trigger: { atSec: 0 }, durationSec: Infinity, meanMps: 2, dirDeg: r.range(0, 360), gustMps: 0.75, gustTauSec: 3 },
        { id: 'pack', kind: 'battery_sag', trigger: { atSec: r.range(60, 180) }, durationSec: Infinity, dropPct: 15, drawScale: 1.25 },
        { id: 'link', kind: 'comms_loss', trigger: { atSec: r.range(20, 120) }, durationSec: Infinity },
      ];
    },
    safe: ['completed', 'returned_home', 'landed'],
    intended: 'returned_home',
  },
];

export function faultScenario(id: string): FaultScenario {
  const s = FAULT_SCENARIOS.find((x) => x.id === id);
  if (!s) throw new Error(`Unknown fault scenario ${id}`);
  return s;
}
