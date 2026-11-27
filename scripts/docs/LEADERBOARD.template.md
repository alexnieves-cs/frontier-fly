# FrontierBench leaderboard

FrontierBench is this repository's indoor exploration benchmark, frozen so that anyone can score a pilot the same way [REPORT.md](REPORT.md) scores ours. A pilot flies seven tasks: six on 30 generated test buildings each, and one on 60 real apartment plans, and its **score** is the mean fraction of the reachable building it flew through, averaged over tasks. Each building gets a fixed flight time: 1,000 policy steps (50 s) for one room, 2,400 (120 s) for the rest. Version 3 flies under the same crash rule as every study in the report: a contact faster than 0.5 m/s ends the flight, and coverage counts what was flown until then.

## Leaderboard

{{BENCH_BOARD}}

The **footprint score** is reported beside the score and does not rank: the mean share of the reachable building that came within 3 m of the drone's sensor, by ground-truth rays at its altitude, the way exploration papers count coverage ([REPORT.md](REPORT.md#coverage-the-way-exploration-papers-measure-it), `src/eval/footprint.ts`). The score counts only voxels the drone flew through, which rewards tight sweep lanes. *Flights ended by a crash* counts the test flights the crash rule ended, and *flights that touched nothing* the share with no contact at all, slow ones included. *Score vs the row above* is the score gap in points with a paired 95% bootstrap interval (both pilots fly the same buildings; flights are resampled within each task): where it says *tied* the interval includes zero and the two ranks should be read as one. The quad-aware planner leads the plain planner only through the `quad` task, the one task where they fly differently. The `room` task is one empty room that almost every pilot covers equally, so it is a sanity check and does not separate pilots. The planner and its variants were tuned on the dev split; the published methods use their papers' parameters and were not tuned, apart from information gain's path-length discount λ, which its paper leaves free and which was picked on the hard track's dev split. That favors the planner. Baselines are re-scored by `npm run bench:board`; submitted entries are re-scored from their code by CI (`npm run bench:verify`) on every pull request that touches them.

### Published methods

Five baselines reimplement published exploration methods on the same sensor maps, flight layer and safety filter as the planner (`src/sim/explorers`): nearest frontier (Yamauchi 1997), greedy information gain (Gonzalez-Banos & Latombe 2002), rapid frontier (Cieslewski et al. 2017), receding-horizon next-best-view (NBVP, Bircher et al. 2016) and online boustrophedon coverage (BA\*, Viet et al. 2013). A random walk with no map and no plan (`src/sim/explorers/randomWalk.ts`) is the floor. The four mapping methods stop once nothing in lidar range is unknown, so they rank low on flown coverage, below the random walk. Judge them on the footprint and on the camera tracks below. NBVP leads the sweeping planner there; the view-first planner (`planner-view`, [VIEW_FIRST.md](VIEW_FIRST.md)), which picks goals by what its own lidar has not yet had within 3 m, leads NBVP on the same test flights and on a fresh report-only block:

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

The hard track found two flaws in the view-first planner on its dev split, both fixed and tuned on dev only. On 40 m buildings it treated a corridor as done once it had viewed the mapped part, though the sparse lidar ring had not mapped the corridor to its end (v2: fly to the edge of unknown space before sweeping). And its edge-of-unknown test counted mapped walls as unknown, so in some large buildings it spent the flight working along the walls of one quarter (v3). v3 also aims the quadrotor's forward camera at what it has not yet had in view. On a fresh report-only block flown once by every pilot (95001+, 900 flights) v3 scores 85.8% against NBVP's 64.5% and nearest frontier's 57.1% ([VIEW_FIRST.md](VIEW_FIRST.md)). Nearest frontier (Yamauchi) still ties it on `large`, the kinematic 40 m building (75.3% against 76.0%, paired gap +0.6, 95% interval −3.4 to +5.2), where leaving each room from the doorway is the right call when time is short.

### Return track

A search drone that does not come back loses what it saw, and no pilot on the hard track had to come back. The return track ([`src/bench/hard.ts`](../src/bench/hard.ts) `RETURN_TASKS`, `npm run bench:hard -- --track return`) scores the same forward camera footprint, but a flight counts only if the drone ends within 1.5 m of where it took off, on its floor, without a crash; otherwise it scores 0. Four tasks: the 40 m building and back on the quadrotor, the same in a steady 1 m/s wind, the 20 m rooms in the dark in 40 s, and a 30 s kinematic sprint. So the tasks compare exploration, not homing code, every explorer pilot gets the same return supervisor (`HOME_RETURN` in [`src/bench/baselines.ts`](../src/bench/baselines.ts)): every half second it plans the known-free path home, and once that path at 0.9 × the task speed plus 5 s no longer fits in the time left (or the explorer says it is done) it flies the sensor-only planner home. Its two numbers were chosen on dev with the baselines, not the view-first planner, as the fastest return that did not lose more flights. Seeds: dev 96001+ (20 per task), test 97001+ (30 per task, scored once). The guarded policy is not an explorer pilot, so it has no supervisor and is not on this board.

{{BENCH_RETURN}}

{{BENCH_RETURN_TASKS}}

The view-first planner brings back 120 of 120 flights with no crash and leads every task; most of the baselines' lost flights are crashes. Its lead is +20.8 points over nearest frontier (95% interval +16.1 to +26.1) and +24.3 over NBVP (+20.6 to +28.4), but on the 30 s sprint the lead over nearest frontier (+6.0, −2.0 to +14.2) is within noise. A goal-scoring change that looked like +4.6 points on dev (preferring goals with more unknown space around them) was worth +1.0 on these test flights and cost a little on the other tracks, so it was not shipped ([VIEW_FIRST.md](VIEW_FIRST.md)).

### Drift track

Every other board hands the pilot its true position. A real drone indoors has no GPS: it knows where it is only from its own sensors, and that estimate drifts. On the drift track ([`src/bench/drift.ts`](../src/bench/drift.ts), `DRIFT_TASKS` in [`src/bench/hard.ts`](../src/bench/hard.ts), `npm run bench:hard -- --track drift`) nothing in the pilot's loop reads the truth. Its pose comes from the indoor estimator in [`src/loc`](../src/loc) (gyro, accelerometer, optical flow, range finder, and a 10 Hz lidar matched against the map it builds as it flies), its seen and flown maps are painted from that estimate, and its measured velocity is turned by the estimated heading. Its commands are flown in the real body frame, so a heading error turns every command. The score is still measured on the true pose, so a drone that thinks it is home but is not scores 0. Lights off, the flow sensor drops most of its samples. Three of the four tasks are return tasks with the shared return supervisor. Each pilot also flies the same seeds on the true pose (`--truth`), the paired control that prices the estimate. Seeds: dev 98001+ (20 per task), test 99001+ (30 per task, scored once).

{{BENCH_DRIFT}}

{{BENCH_DRIFT_TASKS}}

On its own estimate the view-first planner scores 78.7%, with 89 of 90 return flights home and no crash in 120 flights. That is +22.8 points over NBVP (95% interval +19.4 to +26.6) and +27.7 over nearest frontier (+23.5 to +32.1). The estimate itself costs it nothing measurable: the same flights on the true pose score 78.5% (difference +0.2, −1.4 to +1.5). No pilot loses more than noise to the estimate, because the estimator holds the pose to 0.12 to 0.22 m on average (worst 0.80 m, in the wind). So this board shows the pilots do not depend on ground truth; it does not show that drift is hard for them. One oddity: the planner + quad tracker scores 3.4 points higher on its estimate than on the truth (+2.0 to +4.8); we have not found why. Nothing was tuned for this track: every pilot is the one on the boards above, and dev met the round's goal before any change.

### Hard drift track

The drift track's estimator holds the pose to about 0.2 m, so flying on it costs nothing. The hard drift track (`DRIFT_HARD_TASKS` in [`src/bench/hard.ts`](../src/bench/hard.ts), `npm run bench:hard -- --track drift-hard`) gives the estimator the ranging a light drone actually carries: a ring of 24 beams that reach 4 m (`TOF_RING`, the class of a few small multizone time-of-flight sensors), instead of 90 beams to 8 m. IMU, optical flow and range finder are unchanged. In a 40 m building most sweeps see one wall or none, so the estimate coasts on the IMU and flow across open floor and down long corridors. Every task is a return task: the 40 m building and back in 120 s, the same in 240 s so drift has time to grow, the same in 1 m/s wind, and dark rooms and back in 40 s. Each pilot also flies the same seeds on the true pose (`--truth`). The `+ loop closure` row is the view-first planner flying on the pose-graph localizer of [`src/loc/graph.ts`](../src/loc/graph.ts) (`bench:hard --loop`, `LOOP_IN_FLIGHT`). Seeds: dev 110001+ (20 per task, plus 110021-110060 used as a held-out dev check), test 111001+ (30 per task, scored once).

{{BENCH_DRIFT_HARD}}

{{BENCH_DRIFT_HARD_TASKS}}

Here drift hurts. On its own estimate the view-first planner scores 48.1%; the same flights on the true pose score 77.1%, a loss of 29.0 points (95% interval −35.5 to −22.7). The cause is coming home: 77 of 120 flights end within 1.5 m of take-off on the estimate, against 118 of 120 on the truth, while the share of the building it sees on the way barely moves on the long mission (83.9% vs 93.3% before the return rule). The mean pose error is 0.79 m, four times round 3's, and the worst flight drifts 5.7 m. It still leads every baseline: +14.2 points over nearest frontier (+6.5 to +21.7) and +16.7 over NBVP (+9.1 to +23.9). Drift costs it more than it costs them (NBVP −16.0, nearest frontier −12.3) because it ranges further from home in the time it has, so it has further to come back on a drifting estimate.

Loop closure did not earn a place in flight. The `+ loop closure` row runs the estimator as odometry (IMU, flow, range finder) and hands every sweep to the pose graph, which matches keyframes of several sweeps (a single 24-beam sweep has too few returns to match), closes loops against keyframes passed earlier, and gives the pilot its optimised pose. It brings more flights home (90 of 120 against 77) but sees less on the way (360° footprint 69.3% against 77.6%), because a closure moves the pose by up to metres at once and the planner's maps, painted from the old pose, are left behind. Net: −0.3 points (−8.4 to +8.1), a tie. Behind the estimator instead of replacing it, it was worse still on dev. Its setting was picked on dev 110001-110020, where it looked about 10 points better than the estimator; on the held-out dev flights 110021-110060 it was 3.7 points worse (46.7% vs 50.4%), so it is reported here as tried, not shipped. What would likely make it pay: re-painting the planner's maps from the corrected keyframes after a closure, which this track's pilots do not do. Nothing else was tuned for this track; every pilot is the one on the boards above.

### Degraded-sensor track

The drift track's sensors all work. On the degraded-sensor track (`SENSOR_TASKS` in [`src/bench/hard.ts`](../src/bench/hard.ts), faults in [`src/bench/drift.ts`](../src/bench/drift.ts), `npm run bench:hard -- --track sensor`) they do not, and the pilot still flies on its own pose estimate. A fault hits what the pilot reads and what the estimator reads alike, so a dead lidar sector is blind to the safety filter, the map and the scan matcher at once. Four tasks: the 40 m building and back with a 90° lidar sector dead from take-off; dark rooms and back in 40 s with the optical-flow sensor dead (so the estimator has the gyro, accelerometer and lidar only); rooms and back in 60 s with only one lidar ray in four returning; and rooms for 120 s where 120° of lidar and the flow sensor fail together at a random time between 20 and 60 s. Each pilot also flies the same seeds with healthy sensors (`--healthy`), the paired control that prices the faults. Seeds: dev 140001+ (20 per task), test 141001+ (30 per task, scored once).

{{BENCH_SENSOR}}

{{BENCH_SENSOR_TASKS}}

With broken sensors the view-first planner scores 80.0%, with 88 of 90 return flights home and 3 crashes in 120 flights (all on the mid-flight failure task). Every baseline crashes in 20% to 51% of flights. The lead is +27.0 points over NBVP (95% interval +21.7 to +32.2) and +33.0 over nearest frontier (+27.2 to +38.2). The faults cost it 7.2 points against the same flights with healthy sensors (−9.9 to −5.2), less than NBVP (−11.9) or nearest frontier (−10.5). The view-first planner was changed for this track, on dev only. Its quadrotor safety filter is now map-backed: a lidar ray with no return brakes for the walls and unknown space the drone's own map has along it. When two or more rays keep reading nothing where the map has a wall within 4 m (10 steps in a row), it declares the lidar faulty and flies at 0.6× speed. On dev, that took its crashes from 18 of 80 to 0. The cost is speed: the slow-down holds for the whole flight, so the 40 m task with a dead sector from take-off drops 18 points against healthy sensors. A version that slowed only toward the dead rays false-alarmed on healthy flights (−13 points on the return track's dev split) and was dropped. The same change made no difference on the healthy tracks' dev splits (hard 86.9% vs 86.6%, return 72.1% vs 72.5%, drift 78.8% vs 79.1%). The baselines fly the shared safety filter without it.

### Overwatch track

The tracks above score how much of a building a pilot sees. The overwatch track (`OVERWATCH_TASKS` in [`src/bench/hard.ts`](../src/bench/hard.ts), the person in [`src/bench/overwatch.ts`](../src/bench/overwatch.ts), `npm run bench:hard -- --track overwatch`) asks what search and rescue asks next: once you find the lost person, can you keep them in view while help comes? A simulated person walks a fixed loop through up to four rooms at 0.6 m/s, starting in the half of the building far from take-off, and never reacts to the drone. The sensor frame carries the camera's sighting (range, bearing and a noisy position, 15 cm 1σ) whenever the person is inside the forward camera's 90° view, within 8 m, on the same floor and not hidden by walls or furniture, and null otherwise. The **score** is the share of the flight the person is in view from 2 m or more. Nothing counts closer than 2 m, and the time spent closer is reported, so the track rewards a standoff, not an approach. A flight a crash ends early counts the person out of view for the rest of its 120 s, and the stall cut that ends other flights after 10 s without new ground is off, so a pilot can hold still and watch. Three tasks: rooms on the kinematic airframe, rooms on the randomized quadrotor, and the 40 m building on the quadrotor, 120 s each. Seeds: dev 150001+ (20 per task), test 151001+ (30 per task, scored once).

This is overwatch for a rescue, not pursuit. The person walks a route chosen before take-off and does not flee, hide or respond, and the pilot is scored for watching from a distance, never for closing in.

{{BENCH_OVERWATCH}}

{{BENCH_OVERWATCH_TASKS}}


### Integrity track

Every pilot above flies on its pose estimate and says nothing about how good it is. The integrity track (`INTEGRITY_TASKS` in [`src/bench/hard.ts`](../src/bench/hard.ts), scoring in [`src/bench/integrity.ts`](../src/bench/integrity.ts), `npm run bench:hard -- --track integrity`) asks the pilot to say when its pose has gone bad. After every action it may report a **protection level** (`BenchPilot.protectionLevel`, metres): a horizontal radius it claims holds its true position. The terms are satellite navigation's. The **alert limit** is 1.5 m, the return tasks' home radius. A step is *available* when the claimed level is within the alert limit. It is *hazardously misleading* when the pilot claims a level within the limit while its true error is over it, for longer than the 2 s **time to alert** in one run. The **score** is availability minus 10 × the hazardously misleading rate, on every step of the flight. Honest doubt costs a step's availability; confident error costs ten times as much, so a pilot that is confidently wrong for more than a tenth of a flight scores below zero. A flight a crash ends early is unavailable for the rest of its budget, so crashing out of a bad pose does not pay, and the stall cut is off, so every flight runs its full budget on its estimate unless it crashes. A pilot that reports nothing claims a level of 0 on every step: it flies as if its pose were exact, which is what every pilot on the boards above does. Four tasks, all on the quadrotor and its own estimate: the 40 m building with the short-range ToF ring (120 s), dark rooms with the ToF ring (40 s), rooms where 120° of lidar and the flow sensor fail at 20-60 s (120 s), and rooms with healthy sensors (120 s). Seeds: dev 160001+ (20 per task), test 161001+ (30 per task, scored once).

{{BENCH_INTEGRITY}}

{{BENCH_INTEGRITY_TASKS}}


## Team track

{{BENCH_TEAM}}

The team track ([`src/bench/team.ts`](../src/bench/team.ts)) flies up to four sensor-only planners into one building through the same entry, sharing maps over the lossy, wall-attenuated radio of the [drone-team study](REPORT.md). Two conditions, 30 test buildings each: `hard` (the 20 x 20 m stage-3 buildings, 120 s) and `large` (four of them tiled 2 x 2, 40 x 40 m, 240 s, where the radio no longer reaches everywhere). The **score** is the mean fraction of the reachable building the team flew through. *Base knows* is the part of that the radio outside had heard about by the end, which is what an operator could act on, and *80% in* is the mean time to 80% coverage over the flights that got there (the share in brackets).

An entry here is a team configuration, not a new pilot: the number of drones (1 to 4), which fly as relays, how frontiers are split between them, cruise speed (at most 2 m/s), and how often they talk. The radio, buildings and time budget are fixed. Seeds are 83001+ (dev, 10 per condition) and 83101+ (test, 30 per condition), with the `large` condition offset by 500. To enter, add `team.ts` (default export: a `TeamEntry`) to your entry folder, tune with `npm run bench:team -- --entry bench/entries/<your-name>/team.ts --split dev`, then score test once with `--split test --out bench/entries/<your-name>/team-result.json`. CI re-scores it the same way as single-drone entries.

## What a pilot sees

A pilot implements `BenchPilot` ([`src/bench/pilot.ts`](../src/bench/pilot.ts)). Each step it gets:

- the observation vector ([OBS_LAYOUT.md](../src/sim/OBS_LAYOUT.md)): lidar, IMU, altitude, its own coarse maps and frontier features;
- the sensor frame: its pose, its 32-beam lidar in meters, the voxels it has seen and flown, and its measured velocity. On the drift, sensor and integrity tracks the pose is its estimator's, and `health` carries the estimator's own 1σ and state (tracking, degraded, lost). On the overwatch track `person` carries the camera's sighting of the lost person, or null.

It never sees the floor plan. It returns the env's action: forward, left, up and yaw rate, each in [-1, 1], with full horizontal action equal to the task's speed cap. On the integrity track it may also implement `protectionLevel()`, read after each action.

## Submit an entry

1. Copy [`bench/entries/example`](../bench/entries/example) to `bench/entries/<your-name>/` and write your pilot in `pilot.ts` (default export: a function returning a `BenchPilot`). It may import anything under `src/`.
2. Tune on dev: `npm run bench:score -- --pilot bench/entries/<your-name>/pilot.ts --split dev`.
3. Score on test once: `npm run bench:score -- --pilot bench/entries/<your-name>/pilot.ts --split test --out bench/entries/<your-name>/result.json`.
4. Run `npm run docs` and open a pull request. CI re-scores the entry and fails if the result differs. The re-score checks every number the table prints, and it runs your pilot in a worker thread of its own (`scripts/bench/sandbox.ts`): the pilot gets a copy of each observation and sensor frame and hands back four numbers, so it cannot reach the scorer, the env or the building it is flying, and it sees no environment variables. That is not the whole defence. The test seeds are public, so a maintainer still reads each entry's code before it is merged, and an entry built around answers worked out offline is refused. Team-track entries (`team.ts`) still run in the scorer's process.

Learned pilots can ship their weights in the entry folder. Keep each file under a few MB.

**From Python.** The [`frontierfly`](../python/README.md) package scores a Python pilot on the same flights (`python -m frontierfly.bench --policy my_module:MyPolicy --split dev`); its tests check that the flights are bit-identical to `bench:score`. A Python result is not yet a leaderboard entry, because CI re-scores entries from TypeScript code; port the pilot to `pilot.ts` to submit it.
