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
import type { BenchRun, BenchTask } from "./tasks";

export const HARD_VERSION = 1;

export const HARD_TASKS: readonly BenchTask[] = [
  {
    id: "large",
    label: "40 x 40 m building, 120 s",
    stage: 3,
    dynamics: "kinematic",
    lighting: "day",
    speedH: 2,
    dr: 0,
    large: true,
    maxSteps: 2400,
    camera: true,
  },
  {
    id: "large-quad",
    label: "40 x 40 m building, quadrotor, 120 s",
    stage: 3,
    dynamics: "quad",
    lighting: "day",
    speedH: 2,
    dr: 1,
    large: true,
    maxSteps: 2400,
    camera: true,
  },
  {
    id: "sprint",
    label: "Rooms, hallways, loops in 30 s",
    stage: 3,
    dynamics: "kinematic",
    lighting: "day",
    speedH: 2,
    dr: 0,
    maxSteps: 600,
    camera: true,
  },
  {
    id: "quad-dark",
    label: "Quadrotor, lights off",
    stage: 3,
    dynamics: "quad",
    lighting: "night-dark",
    speedH: 2,
    dr: 1,
    camera: true,
  },
  {
    id: "floors-quad",
    label: "Several floors, quadrotor",
    stage: 4,
    dynamics: "quad",
    lighting: "day",
    speedH: 2,
    dr: 1,
    camera: true,
  },
];

/** Dev 92001+ (tune freely), test 93001+ (score once); task i offsets the block by 100 i. */
export const HARD_SEEDS = {
  dev: { base: 92001, count: 20 },
  test: { base: 93001, count: 30 },
} as const;

/** A split, or a fresh ledger block ({ base, count }) scored once for a report. */
export function hardRuns(
  split: "dev" | "test" | { base: number; count: number },
  taskIndex: number,
): BenchRun[] {
  const { base, count } = typeof split === "string" ? HARD_SEEDS[split] : split;
  return Array.from({ length: count }, (_, i) => ({
    seed: base + taskIndex * 100 + i,
  }));
}
