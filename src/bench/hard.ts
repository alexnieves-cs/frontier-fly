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
 * The degraded-sensor track (round 4): the drift track's estimate, with sensors that are broken or
 * break mid-flight (bench/drift.ts SensorFaults). A fault hits the pilot's lidar and the estimator's
 * alike, so a dead sector is blind to the safety layer, the map and the scan matcher at once.
 */
export const SENSOR_TASKS: readonly BenchTask[] = [
  { id: 'sector-large-quad', label: '40 x 40 m and back, quadrotor, a 90° lidar sector dead, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true, drift: true, faults: { lidarSector: Math.PI / 2 } },
  { id: 'flowless-dark-quad', label: 'Rooms and back, quadrotor, lights off, flow sensor dead, 40 s', stage: 3, dynamics: 'quad', lighting: 'night-dark', speedH: 2, dr: 1, maxSteps: 800, camera: true, home: true, drift: true, faults: { flowDead: true } },
  { id: 'sparse-quad', label: 'Rooms and back, quadrotor, 1 lidar ray in 4, 60 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, maxSteps: 1200, camera: true, home: true, drift: true, faults: { lidarKeep: 4 } },
  { id: 'failing-quad', label: 'Rooms, quadrotor, 120° of lidar and the flow sensor fail at 20-60 s, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, camera: true, drift: true, faults: { lidarSector: (2 * Math.PI) / 3, flowDead: true, failAt: [20, 60] } },
];

/** Degraded-sensor seeds: dev 140001+ (tune freely), test 141001+ (score once); task i offsets the block by 100 i. */
export const SENSOR_SEEDS = { dev: { base: 140001, count: 20 }, test: { base: 141001, count: 30 } } as const;

/**
 * The short-range ToF ring suite of the hard drift track: the localization lidar has 24 beams to 4 m at 10 Hz
 * (the class of a ring of small multizone time-of-flight sensors on a sub-500 g drone) instead of round 3's
 * 90 beams to 8 m. In a 40 m building most sweeps reach a wall on one side or none, so the estimator coasts
 * on the IMU and optical flow across open floor and long corridors, the way a light drone does.
 * IMU, flow and range finder stay round 3's.
 */
export const TOF_RING: DriftProfile = { lidarRange: 4, noise: { beams: 24 } };

/**
 * The hard drift track: round 3's return tasks on the short-range ToF ring suite, plus a 240 s mission on the
 * 40 m building so drift has time to grow. Every task asks the drone to come home on its own estimate.
 */
export const DRIFT_HARD_TASKS: readonly BenchTask[] = [
  { id: 'tof-large-quad', label: '40 x 40 m and back, short-range ToF ring, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true, drift: true, driftProfile: TOF_RING },
  { id: 'tof-long-quad', label: '40 x 40 m and back, short-range ToF ring, quadrotor, 240 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 4800, camera: true, home: true, drift: true, driftProfile: TOF_RING },
  { id: 'tof-wind-quad', label: '40 x 40 m and back, short-range ToF ring, quadrotor, 1 m/s wind, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, home: true, wind: 1, drift: true, driftProfile: TOF_RING },
  { id: 'tof-dark-quad', label: 'Rooms and back, short-range ToF ring, quadrotor, lights off, 40 s', stage: 3, dynamics: 'quad', lighting: 'night-dark', speedH: 2, dr: 1, maxSteps: 800, camera: true, home: true, drift: true, driftProfile: TOF_RING },
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

/**
 * The overwatch track (round 5, bench/overwatch.ts): a lost person walks a loop through four rooms of
 * the take-off floor, starting far from take-off. The score is the share of the flight they are in the
 * forward camera's view from at least 2 m; closer counts nothing and is reported as a standoff breach.
 * A search-and-rescue task: the person never reacts to the drone, and the drone has no reason to close in.
 */
export const OVERWATCH_TASKS: readonly BenchTask[] = [
  { id: 'watch-rooms', label: 'Find a lost person and keep them in view, rooms, 120 s', stage: 3, dynamics: 'kinematic', lighting: 'day', speedH: 2, dr: 0, maxSteps: 2400, camera: true, person: true },
  { id: 'watch-quad', label: 'Find a lost person and keep them in view, rooms, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, maxSteps: 2400, camera: true, person: true },
  { id: 'watch-large-quad', label: 'Find a lost person and keep them in view, 40 x 40 m, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, person: true },
];

/** Overwatch seeds: dev 150001+ (tune freely), test 151001+ (score once); task i offsets the block by 100 i. */
export const OVERWATCH_SEEDS = { dev: { base: 150001, count: 20 }, test: { base: 151001, count: 30 } } as const;

/**
 * The integrity track (round 6, bench/integrity.ts): drift-track flights where the pilot also reports a
 * protection level each step. Three tasks where the estimate goes bad on some flights (the short-range
 * ToF ring, sensors failing mid-flight) and one where it stays good, so a pilot cannot win by never
 * trusting itself.
 */
export const INTEGRITY_TASKS: readonly BenchTask[] = [
  { id: 'pl-tof-large-quad', label: 'Report pose integrity, 40 x 40 m, short-range ToF ring, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, large: true, maxSteps: 2400, camera: true, drift: true, driftProfile: TOF_RING, integrity: true },
  { id: 'pl-tof-dark-quad', label: 'Report pose integrity, rooms, short-range ToF ring, lights off, quadrotor, 40 s', stage: 3, dynamics: 'quad', lighting: 'night-dark', speedH: 2, dr: 1, maxSteps: 800, camera: true, drift: true, driftProfile: TOF_RING, integrity: true },
  { id: 'pl-failing-quad', label: 'Report pose integrity, rooms, 120° of lidar and flow fail at 20-60 s, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, camera: true, drift: true, faults: { lidarSector: (2 * Math.PI) / 3, flowDead: true, failAt: [20, 60] }, integrity: true },
  { id: 'pl-quad', label: 'Report pose integrity, rooms, healthy sensors, quadrotor, 120 s', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1, camera: true, drift: true, integrity: true },
];

/** Integrity seeds: dev 160001+ (tune freely), test 161001+ (score once); task i offsets the block by 100 i. */
export const INTEGRITY_SEEDS = { dev: { base: 160001, count: 20 }, test: { base: 161001, count: 30 } } as const;

export type HardTrack = 'hard' | 'return' | 'drift' | 'sensor' | 'drift-hard' | 'overwatch' | 'integrity';

export const HARD_TRACKS: readonly HardTrack[] = ['hard', 'return', 'drift', 'sensor', 'drift-hard', 'overwatch', 'integrity'];

export function trackTasks(track: HardTrack): readonly BenchTask[] {
  return { hard: HARD_TASKS, return: RETURN_TASKS, drift: DRIFT_TASKS, sensor: SENSOR_TASKS, 'drift-hard': DRIFT_HARD_TASKS, overwatch: OVERWATCH_TASKS, integrity: INTEGRITY_TASKS }[track];
}

export function trackRuns(track: HardTrack, split: 'dev' | 'test' | { base: number; count: number }, taskIndex: number): BenchRun[] {
  const { base, count } = typeof split === 'string' ? { hard: HARD_SEEDS, return: RETURN_SEEDS, drift: DRIFT_SEEDS, sensor: SENSOR_SEEDS, 'drift-hard': DRIFT_HARD_SEEDS, overwatch: OVERWATCH_SEEDS, integrity: INTEGRITY_SEEDS }[track][split] : split;
  return Array.from({ length: count }, (_, i) => ({ seed: base + taskIndex * 100 + i }));
}
