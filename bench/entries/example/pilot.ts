// A minimal FrontierBench entry: the sensor-only planner, cruising at 1.6 m/s whatever the cap.
// Copy this folder, change the pilot, then score it on the test split:
//
//   npm run bench:score -- --pilot bench/entries/<name>/pilot.ts --split test --out bench/entries/<name>/result.json
//
// A pilot sees only the observation and the sensor frame it is handed (src/bench/pilot.ts), never
// the floor plan. Tune on the dev split; score the test split once.
import { plannerPilot } from '../../../src/bench/baselines';
import type { BenchPilotFactory } from '../../../src/bench/pilot';

const factory: BenchPilotFactory = () => plannerPilot('Example: planner cruising at 1.6 m/s', { speed: 1.6 });
export default factory;
