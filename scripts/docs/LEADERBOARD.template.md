# FrontierBench leaderboard

FrontierBench is this repository's indoor exploration benchmark, frozen so that anyone can score a pilot the same way [REPORT.md](REPORT.md) scores ours. A pilot flies seven tasks: six on 30 generated test buildings each, and one on 60 real apartment plans, and its **score** is the mean fraction of the reachable building it flew through, averaged over tasks. Each building gets a fixed flight time: 1,000 policy steps (50 s) for one room, 2,400 (120 s) for the rest. Version 3 flies under the same crash rule as every study in the report: a contact faster than 0.5 m/s ends the flight, and coverage counts what was flown until then.

## Leaderboard

{{BENCH_BOARD}}

The **footprint score** is reported beside the score and does not rank: the mean share of the reachable building that came within 3 m of the drone's sensor, by ground-truth rays at its altitude, the way exploration papers count coverage ([REPORT.md](REPORT.md#coverage-the-way-exploration-papers-measure-it), `src/eval/footprint.ts`). The score counts only voxels the drone flew through, which rewards tight sweep lanes. *Flights ended by a crash* counts the test flights the crash rule ended, and *flights that touched nothing* the share with no contact at all, slow ones included. *Score vs the row above* is the score gap in points with a paired 95% bootstrap interval (both pilots fly the same buildings; flights are resampled within each task): where it says *tied* the interval includes zero and the two ranks should be read as one. The quad-aware planner leads the plain planner only through the `quad` task, the one task where they fly differently. The `room` task is one empty room that almost every pilot covers equally, so it is a sanity check and does not separate pilots. The planner and its variants were tuned on the dev split; the three published methods use their papers' parameters and were not tuned, which favors the planner. Baselines are re-scored by `npm run bench:board`; submitted entries are re-scored from their code by CI (`npm run bench:verify`) on every pull request that touches them.

### Published methods

Three baselines reimplement published exploration methods on the same sensor maps, flight layer and safety filter as the planner (`src/sim/explorers`): nearest frontier (Yamauchi 1997), receding-horizon next-best-view (NBVP, Bircher et al. 2016) and online boustrophedon coverage (BA\*, Viet et al. 2013). The two mapping methods stop once nothing in lidar range is unknown, so they rank low on flown coverage. Judge them on the footprint. NBVP leads the sweeping planner there; the view-first planner (`planner-view`, [VIEW_FIRST.md](VIEW_FIRST.md)), which picks goals by what its own lidar has not yet had within 3 m, leads NBVP on the same test flights and on a fresh report-only block:

{{BENCH_PUBLISHED}}

[REPORT.md](REPORT.md#against-published-exploration-methods) describes each method and its adaptations.

## Tasks

{{BENCH_TASKS}}

Task definitions and seeds are in [`src/bench/tasks.ts`](../src/bench/tasks.ts): a **dev** split (seeds 81001+, 20 buildings per task) to tune on, and a **test** split (82001+, 30 per task) to score once. The `apartments` task flies the 20 dev and 60 eval plans of the real-plan set ([REAL_PLANS.md](REAL_PLANS.md)), converted from Swiss Dwellings 3.0.0 (CC BY 4.0), with env seeds 84001+ (dev) and 84101+ (test).

Version 2 added the `apartments` task, so scores from version 1 (six tasks) are not comparable and were re-run. Version 3 (November 2026) keeps the tasks and seeds and changes two rules: crashes end the flight, and the energy column prices the current 0.45 kg airframe (`src/shared/airframe.ts`) instead of the 1.2 kg quad of v2. Under v2 a contact cost nothing, so a pilot that slid along walls lost no coverage for it; the learned policy ranked first there and beat the planner on the quadrotor task while crashing in every quadrotor flight of the report's coverage study. Seeds are public, as everything in a static repository is. The leaderboard relies on entrants not tuning on test seeds, and on CI re-scoring each entry from its code.

### Version 2, for reference

Scored without the crash rule and with the old airframe's power model, so these numbers are not comparable with the board above. The result files are in [`bench/results/v2/`](../bench/results/v2).

{{BENCH_V2}}

## FrontierBench Hard

v3's footprint column saturates: the view-first planner sees 99% of the test buildings within 3 m. The hard track ([`src/bench/hard.ts`](../src/bench/hard.ts), `npm run bench:hard`) leaves v3 frozen and asks for more on four axes at once. **Scale**: a 40 x 40 m building (four hard plans tiled 2 x 2) with the 120 s a 20 m building gets, so no pilot finishes. **Time**: the 20 m buildings in 30 s. **Airframe and light**: the randomized quadrotor on the large building, in the dark and across floors. **Sensor**: the score is a forward 90° *camera* footprint, the share of the reachable building that came within 3 m of the drone *in front of it* (the same ground-truth rays as the ring, kept to 45° either side of the heading), so a pilot has to turn to look. The 360° ring footprint and flown coverage are reported beside it. The crash rule holds. Seeds: dev 92001+ (20 per task), test 93001+ (30 per task, scored once).

{{BENCH_HARD}}

{{BENCH_HARD_TASKS}}

The hard track found two flaws in the view-first planner on its dev split, both fixed and tuned on dev only. On 40 m buildings it treated a corridor as done once it had viewed the mapped part, though the sparse lidar ring had not mapped the corridor to its end (v2: fly to the edge of unknown space before sweeping). And its edge-of-unknown test counted mapped walls as unknown, so in some large buildings it spent the flight working along the walls of one quarter (v3). v3 also aims the quadrotor's forward camera at what it has not yet had in view. On a fresh report-only block flown once by every pilot (95001+, 900 flights) v3 scores 85.8% against NBVP's 64.5% and nearest frontier's 57.1% ([VIEW_FIRST.md](VIEW_FIRST.md)). That block left one tie: nearest frontier (Yamauchi) on `large`, the kinematic 40 m building (75.3% against 76.0%, paired gap +0.6, 95% interval −3.4 to +5.2). The planner cleared every mapped corner before opening an unmapped room, and a 40 m building never runs out of mapped corners. v4 lets the edge of unmapped space compete with them by path cost, on the kinematic airframe only (on the quadrotor's own pose estimate it cost 11 points). On a fresh block flown once (153001+) v4 scores 84.5% on `large` against nearest frontier's 73.4% (+11.1, +6.6 to +16.1), and 87.0% overall against v3's 84.7% ([VIEW_FIRST.md](VIEW_FIRST.md#version-4-unmapped-rooms-compete-from-the-start-the-40-m-tie)).

### Return track

A search drone that does not come back loses what it saw, and no pilot on the hard track had to come back. The return track ([`src/bench/hard.ts`](../src/bench/hard.ts) `RETURN_TASKS`, `npm run bench:hard -- --track return`) scores the same forward camera footprint, but a flight counts only if the drone ends within 1.5 m of where it took off, on its floor, without a crash; otherwise it scores 0. Four tasks: the 40 m building and back on the quadrotor, the same in a steady 1 m/s wind, the 20 m rooms in the dark in 40 s, and a 30 s kinematic sprint. So the tasks compare exploration, not homing code, every explorer pilot gets the same return supervisor (`HOME_RETURN` in [`src/bench/baselines.ts`](../src/bench/baselines.ts)): every half second it plans the known-free path home, and once that path at 0.9 × the task speed plus 5 s no longer fits in the time left (or the explorer says it is done) it flies the sensor-only planner home. Its two numbers were chosen on dev with the baselines, not the view-first planner, as the fastest return that did not lose more flights. Seeds: dev 96001+ (20 per task), test 97001+ (30 per task, scored once). The guarded policy is not an explorer pilot, so it has no supervisor and is not on this board.

{{BENCH_RETURN}}

{{BENCH_RETURN_TASKS}}

The view-first planner brings back 120 of 120 flights with no crash and leads every task; most of the baselines' lost flights are crashes. On the 30 s sprint v3's lead over nearest frontier (+6.0, −2.0 to +14.2) was within noise; v4's rule on the kinematic airframe makes it +12.1 (+6.4 to +18.2). A goal-scoring change that looked like +4.6 points on dev (preferring goals with more unknown space around them) was worth +1.0 on these test flights and cost a little on the other tracks, so it was not shipped ([VIEW_FIRST.md](VIEW_FIRST.md)).

### Drift track

Every other board hands the pilot its true position. A real drone indoors has no GPS: it knows where it is only from its own sensors, and that estimate drifts. On the drift track ([`src/bench/drift.ts`](../src/bench/drift.ts), `DRIFT_TASKS` in [`src/bench/hard.ts`](../src/bench/hard.ts), `npm run bench:hard -- --track drift`) nothing in the pilot's loop reads the truth. Its pose comes from the indoor estimator in [`src/loc`](../src/loc) (gyro, accelerometer, optical flow, range finder, and a 10 Hz lidar matched against the map it builds as it flies), its seen and flown maps are painted from that estimate, and its measured velocity is turned by the estimated heading. Its commands are flown in the real body frame, so a heading error turns every command. The score is still measured on the true pose, so a drone that thinks it is home but is not scores 0. Lights off, the flow sensor drops most of its samples. Three of the four tasks are return tasks with the shared return supervisor. Each pilot also flies the same seeds on the true pose (`--truth`), the paired control that prices the estimate. Seeds: dev 98001+ (20 per task), test 99001+ (30 per task, scored once).

{{BENCH_DRIFT}}

{{BENCH_DRIFT_TASKS}}

On its own estimate the view-first planner scores 78.7%, with 89 of 90 return flights home and no crash in 120 flights. That is +22.8 points over NBVP (95% interval +19.4 to +26.6) and +27.7 over nearest frontier (+23.5 to +32.1). The estimate itself costs it nothing measurable: the same flights on the true pose score 78.5% (difference +0.2, −1.4 to +1.5). No pilot loses more than noise to the estimate, because the estimator holds the pose to 0.12 to 0.22 m on average (worst 0.80 m, in the wind). So this board shows the pilots do not depend on ground truth; it does not show that drift is hard for them. One oddity: the planner + quad tracker scores 3.4 points higher on its estimate than on the truth (+2.0 to +4.8); we have not found why. Nothing was tuned for this track: every pilot is the one on the boards above, and dev met the round's goal before any change.

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
4. Run `npm run docs` and open a pull request. CI re-scores the entry and fails if the result differs. The re-score checks every number the table prints, and it runs your pilot in a worker thread of its own (`scripts/bench/sandbox.ts`): the pilot gets a copy of each observation and sensor frame and hands back four numbers, so it cannot reach the scorer, the env or the building it is flying, and it sees no environment variables. That is not the whole defence. The test seeds are public, so a maintainer still reads each entry's code before it is merged, and an entry built around answers worked out offline is refused. Team-track entries (`team.ts`) still run in the scorer's process.

Learned pilots can ship their weights in the entry folder. Keep each file under a few MB.

**From Python.** The [`frontierfly`](../python/README.md) package scores a Python pilot on the same flights (`python -m frontierfly.bench --policy my_module:MyPolicy --split dev`); its tests check that the flights are bit-identical to `bench:score`. A Python result is not yet a leaderboard entry, because CI re-scores entries from TypeScript code; port the pilot to `pilot.ts` to submit it.
