/**
 * FrontierBench Hard: a second, harder track beside FrontierBench v3 (bench/tasks.ts), which it
 * leaves frozen. v3's footprint column saturates: the view-first planner sees 98-100% of most
 * tasks within 3 m. The hard track asks for more on four axes at once:
 *
 * - Scale: a 40 x 40 m building (four hard floor plans tiled 2 x 2, team/largeBuilding.ts), four
 *   times the area, with the same 120 s a 20 m building gets. No pilot can finish it.
 * - Time: the hard 20 m buildings in 30 s instead of 120 s.
 * - Airframe and light: the randomized quadrotor, on the large building and in the dark.
 * - Sensor: the headline is a forward 90° camera footprint (3 m, along the drone's heading),
 *   not the 360° ring. A pilot has to turn to look, the way a real gimbal-less camera drone does.
 *
 * Every v3 rule holds: the crash rule ends a flight, scores are means over tasks, the dev split is
 * for tuning and the test split is scored once.
 */
import type { DriftProfile } from './drift';
import type { BenchRun, BenchTask } from './tasks';

export const HARD_VERSION = 1;

export const HARD_TASKS: readonly BenchTask[] = [
  { id: 'large', label: '40 x 40 m building, 120 s', stage: 3, dynamics: 'kinematic', lighting: 'day', speedH: 2, dr: 0, large: true, maxSteps: 2400, camera: true },
  { id: 'large-quad', label: '40 x 40 m building, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true },
  { id: 'sprint', label: 'Rooms, hallways, loops in 30 s', stage: 3, dynamics: 'kinematic', lighting: 'day', speedH: 2, dr: 0, maxSteps: 600, camera: true },
  { id: 'quad-dark', label: 'Quadrotor, lights off', stage: 3, dynamics: 'quad', lighting: 'night-dark', speedH: 2, dr: 1, camera: true },
  { id: 'floors-quad', label: 'Several floors, quadrotor', stage: 4, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, camera: true },
];

/** Dev 92001+ (tune freely), test 93001+ (score once); task i offsets the block by 100 i. */
export const HARD_SEEDS = { dev: { base: 92001, count: 20 }, test: { base: 93001, count: 30 } } as const;

/** A split, or a fresh ledger block ({ base, count }) scored once for a report. */
export function hardRuns(split: 'dev' | 'test' | { base: number; count: number }, taskIndex: number): BenchRun[] {
  const { base, count } = typeof split === 'string' ? HARD_SEEDS[split] : split;
  return Array.from({ length: count }, (_, i) => ({ seed: base + taskIndex * 100 + i }));
}

/**
 * The return track (round 2): a search drone that does not come back loses the flight, so each
 * task's camera footprint counts only if the drone ends within HOME_RADIUS of take-off (bench/run.ts).
 * Every explorer pilot gets the same return supervisor (baselines.ts HOME_RETURN), so the tasks
 * compare how much each sees in the time it can afford to be away. Same crash rule and camera.
 */
export const RETURN_TASKS: readonly BenchTask[] = [
  { id: 'return-large-quad', label: '40 x 40 m building and back, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true },
  { id: 'return-wind-quad', label: '40 x 40 m and back, quadrotor, 1 m/s wind, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true, wind: 1 },
  { id: 'return-dark-quad', label: 'Rooms, hallways, loops and back, quadrotor, lights off, 40 s', stage: 3, dynamics: 'quad', lighting: 'night-dark', speedH: 2, dr: 1, maxSteps: 800, camera: true, home: true },
  { id: 'return-sprint', label: 'Rooms, hallways, loops and back in 30 s', stage: 3, dynamics: 'kinematic', lighting: 'day', speedH: 2, dr: 0, maxSteps: 600, camera: true, home: true },
];

/** Return track seeds: dev 96001+ (tune freely), test 97001+ (score once); task i offsets the block by 100 i. */
export const RETURN_SEEDS = { dev: { base: 96001, count: 20 }, test: { base: 97001, count: 30 } } as const;

/**
 * The drift track (round 3): no pilot reads its true pose. Each flies on the src/loc estimator's
 * pose (IMU, optical flow, range finder, lidar matched against its own map), with its seen and flown
 * maps painted from that estimate (bench/drift.ts). The score is still measured on the truth, so a
 * drone that thinks it is home but is not scores 0. Lights off, the flow sensor drops most samples.
 */
export const DRIFT_TASKS: readonly BenchTask[] = [
  { id: 'drift-large-quad', label: '40 x 40 m and back on its own pose, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true, drift: true },
  { id: 'drift-wind-quad', label: '40 x 40 m and back on its own pose, quadrotor, 1 m/s wind, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true, wind: 1, drift: true },
  { id: 'drift-dark-quad', label: 'Rooms and back on its own pose, quadrotor, lights off, 40 s', stage: 3, dynamics: 'quad', lighting: 'night-dark', speedH: 2, dr: 1, maxSteps: 800, camera: true, home: true, drift: true },
  { id: 'drift-quad', label: 'Rooms, hallways, loops on its own pose, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, camera: true, drift: true },
];

/** Drift track seeds: dev 98001+ (tune freely), test 99001+ (score once); task i offsets the block by 100 i. */
export const DRIFT_SEEDS = { dev: { base: 98001, count: 20 }, test: { base: 99001, count: 30 } } as const;

/**
 * The sparse ranging suite of the hard drift track: the localization lidar has 24 beams to 4 m at 10 Hz
 * (the class of a ring of small multizone time-of-flight sensors on a sub-500 g drone) instead of round 3's
 * 90 beams to 8 m. In a 40 m building most sweeps reach a wall on one side or none, so the estimator coasts
 * on the IMU and optical flow across open floor and long corridors, the way a light drone does.
 * IMU, flow and range finder stay round 3's.
 */
export const SPARSE_RANGING: DriftProfile = { lidarRange: 4, noise: { beams: 24 } };

/**
 * The hard drift track: round 3's return tasks on the sparse ranging suite, plus a 240 s mission on the
 * 40 m building so drift has time to grow. Every task asks the drone to come home on its own estimate.
 */
export const DRIFT_HARD_TASKS: readonly BenchTask[] = [
  { id: 'sparse-large-quad', label: '40 x 40 m and back, sparse ranging, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true, drift: true, driftProfile: SPARSE_RANGING },
  { id: 'sparse-long-quad', label: '40 x 40 m and back, sparse ranging, quadrotor, 240 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 4800, camera: true, home: true, drift: true, driftProfile: SPARSE_RANGING },
  { id: 'sparse-wind-quad', label: '40 x 40 m and back, sparse ranging, quadrotor, 1 m/s wind, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true, wind: 1, drift: true, driftProfile: SPARSE_RANGING },
  { id: 'sparse-dark-quad', label: 'Rooms and back, sparse ranging, quadrotor, lights off, 40 s', stage: 3, dynamics: 'quad', lighting: 'night-dark', speedH: 2, dr: 1, maxSteps: 800, camera: true, home: true, drift: true, driftProfile: SPARSE_RANGING },
];

/**
 * Loop closure in flight (bench:hard --loop): the estimator runs as odometry only (IMU, flow, range finder)
 * and every lidar sweep goes to the pose-graph back end (loc/graph.ts), which does all the scan matching and
 * closes loops; the pilot flies on the graph's pose. Picked on the hard drift dev split 110001+ only.
 */
export const LOOP_IN_FLIGHT: DriftProfile = {
  estimator: { mode: 'odometry' },
  loop: { accumulate: 6, keyDist: 0.6, keyYaw: 0.5, seqMinPts: 20, loopMinPts: 50, loopMinGap: 20, loopHalfWindow: 6, seqWindow: 20, loopSearchMax: 4, loopSearchGrowth: 0.04 },
};

/** Hard drift track seeds: dev 110001+ (tune freely), test 111001+ (score once); task i offsets the block by 100 i. */
export const DRIFT_HARD_SEEDS = { dev: { base: 110001, count: 20 }, test: { base: 111001, count: 30 } } as const;

export type DriftTrack = 'hard' | 'return' | 'drift' | 'drift-hard';

export function trackTasks(track: DriftTrack): readonly BenchTask[] {
  return { hard: HARD_TASKS, return: RETURN_TASKS, drift: DRIFT_TASKS, 'drift-hard': DRIFT_HARD_TASKS }[track];
}

export function trackRuns(track: DriftTrack, split: 'dev' | 'test' | { base: number; count: number }, taskIndex: number): BenchRun[] {
  const { base, count } = typeof split === 'string' ? { hard: HARD_SEEDS, return: RETURN_SEEDS, drift: DRIFT_SEEDS, 'drift-hard': DRIFT_HARD_SEEDS }[track][split] : split;
  return Array.from({ length: count }, (_, i) => ({ seed: base + taskIndex * 100 + i }));
}
