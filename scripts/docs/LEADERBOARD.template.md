# FrontierBench leaderboard

FrontierBench is this repository's indoor exploration benchmark, frozen so that anyone can score a pilot the same way [REPORT.md](REPORT.md) scores ours. A pilot flies seven tasks: six on 30 generated test buildings each, and one on 60 real apartment plans, and its **score** is the mean fraction of the reachable building it flew through, averaged over tasks. Each building gets a fixed flight time: 1,000 policy steps (50 s) for one room, 2,400 (120 s) for the rest.

## Leaderboard

{{BENCH_BOARD}}

The **footprint score** is reported beside the score and does not rank: the mean share of the reachable building that came within 3 m of the drone's sensor, by ground-truth rays at its altitude, the way exploration papers count coverage ([REPORT.md](REPORT.md#coverage-the-way-exploration-papers-measure-it), `src/eval/footprint.ts`). The score counts only voxels the drone flew through, which rewards tight sweep lanes. The last-but-one column is the share of all 240 test flights that ended without touching anything. Baselines are re-scored by `npm run bench:board`; submitted entries are re-scored from their code by CI (`npm run bench:verify`) on every pull request that touches them.

## Tasks

{{BENCH_TASKS}}

Task definitions and seeds are in [`src/bench/tasks.ts`](../src/bench/tasks.ts): a **dev** split (seeds 81001+, 20 buildings per task) to tune on, and a **test** split (82001+, 30 per task) to score once. The `apartments` task flies the 20 dev and 60 eval plans of the real-plan set ([REAL_PLANS.md](REAL_PLANS.md)), converted from Swiss Dwellings 3.0.0 (CC BY 4.0), with env seeds 84001+ (dev) and 84101+ (test).

Version 2 added the `apartments` task, so scores from version 1 (six tasks) are not comparable and were re-run. Both are public, as everything in a static repository is. The leaderboard relies on entrants not tuning on test seeds, and on CI re-scoring each entry from its code.

## Team track

{{BENCH_TEAM}}

The team track ([`src/bench/team.ts`](../src/bench/team.ts)) flies up to four sensor-only planners into one building through the same entry, sharing maps over the lossy, wall-attenuated radio of the [drone-team study](REPORT.md). Two conditions, 30 test buildings each: `hard` (the 20 x 20 m stage-3 buildings, 120 s) and `large` (four of them tiled 2 x 2, 40 x 40 m, 240 s, where the radio no longer reaches everywhere). The **score** is the mean fraction of the reachable building the team flew through. *Base knows* is the part of that the radio outside had heard about by the end, which is what an operator could act on, and *80% in* is the mean time to 80% coverage over the flights that got there (the share in brackets).

An entry here is a team configuration, not a new pilot: the number of drones (1 to 4), which fly as relays, how frontiers are split between them, cruise speed (at most 2 m/s), and how often they talk. The radio, buildings and time budget are fixed. Seeds are 83001+ (dev, 10 per condition) and 83101+ (test, 30 per condition), with the `large` condition offset by 500. To enter, add `team.ts` (default export: a `TeamEntry`) to your entry folder, tune with `npm run bench:team -- --entry bench/entries/<your-name>/team.ts --split dev`, then score test once with `--split test --out bench/entries/<your-name>/team-result.json`. CI re-scores it the same way as single-drone entries.

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
