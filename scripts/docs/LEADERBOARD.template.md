# FrontierBench leaderboard

FrontierBench is this repository's indoor exploration benchmark, frozen so that anyone can score a pilot the same way [REPORT.md](REPORT.md) scores ours. A pilot flies six tasks, 30 test buildings each, and its **score** is the mean fraction of the reachable building it flew through, averaged over tasks. Each building gets a fixed flight time: 1,000 policy steps (50 s) for one room, 2,400 (120 s) for the rest.

## Leaderboard

{{BENCH_BOARD}}

The last-but-one column is the share of all 180 test flights that ended without touching anything. Baselines are re-scored by `npm run bench:board`; submitted entries are re-scored from their code by CI (`npm run bench:verify`) on every pull request that touches them.

## Tasks

{{BENCH_TASKS}}

Task definitions and seeds are in [`src/bench/tasks.ts`](../src/bench/tasks.ts): a **dev** split (seeds 81001+, 20 buildings per task) to tune on, and a **test** split (82001+, 30 per task) to score once. Both are public, as everything in a static repository is. The leaderboard relies on entrants not tuning on test seeds, and on CI re-scoring each entry from its code.

## What a pilot sees

A pilot implements `BenchPilot` ([`src/bench/pilot.ts`](../src/bench/pilot.ts)). Each step it gets:

- the observation vector ([OBS_LAYOUT.md](../src/sim/OBS_LAYOUT.md)): lidar, IMU, altitude, its own coarse maps and frontier features;
- the sensor frame: its pose, its 32-beam lidar in meters, the voxels it has seen and flown, and its measured velocity.

It never sees the floor plan. It returns the env's action: forward, left, up and yaw rate, each in [-1, 1], with full horizontal action equal to the task's speed cap.

## Submit an entry

1. Copy [`bench/entries/example`](../bench/entries/example) to `bench/entries/<your-name>/` and write your pilot in `pilot.ts` (default export: a function returning a `BenchPilot`). It may import anything under `src/`.
2. Tune on dev: `npm run bench:score -- --pilot bench/entries/<your-name>/pilot.ts --split dev`.
3. Score on test once: `npm run bench:score -- --pilot bench/entries/<your-name>/pilot.ts --split test --out bench/entries/<your-name>/result.json`.
4. Run `npm run docs` and open a pull request. CI re-scores the entry and fails if the result differs.

Learned pilots can ship their weights in the entry folder. Keep each file under a few MB.
