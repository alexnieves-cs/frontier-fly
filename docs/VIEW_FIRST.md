# View-first planner

FrontierBench scores two kinds of coverage ([LEADERBOARD.md](LEADERBOARD.md)). *Flown* counts the 0.5 m voxels the
drone's center flew through. *Footprint* counts what came within 3 m of the sensor, the way exploration papers
score coverage. The sweeping planner was built for the first, and on the second a 2016 next-best-view planner
(NBVP, Bircher et al.) beat it: 89.7% against 87.7% for the quad-aware planner on the v3 test split.

The view-first planner (`planner-view` in `src/bench/baselines.ts`, `SensorPlannerOptions.view` in
`src/sim/sensorPlanner.ts`) changes one rule of the planner. A voxel stops being a goal once the drone's **own**
lidar plane has had it in line of sight within 3 m (72 rays cast over the drone's own map, stopping at anything
it has not seen free), not once the drone has flown through it. The search, the path cost, the safety filters,
the quadrotor tracker and the climber are unchanged. When nothing known is left unviewed, it flies to the edge of
unknown space (unflown voxels next to unmapped ones), and when that is gone too it goes back to the flown-voxel
goals and sweeps with the time left. Each replan starts again from the view goals. It reads no
ground truth; the footprint metric's ground-truth rays never reach the pilot.

Going for views means longer legs at cruise speed. On the quadrotor that brought two crashes on the dev split, so
the quadrotor's braking margin was raised from 0.25 m to 0.30 m, chosen among 0.25, 0.30 and 0.35 m on 100 practice
flights (90501-90600).

## Version 2 of the rule (FrontierBench Hard)

The first version (PR #126) went straight from view goals to sweeping. [FrontierBench Hard](LEADERBOARD.md#frontierbench-hard)'s
40 m buildings showed why that was wrong: the 32-beam lidar ring maps a long corridor only where a beam happens
to run along it, so the drone viewed the mapped part, found nothing left, and swept the first quarter of the
building for the rest of the flight. Adding the edge-of-unknown step lifted the hard dev split's camera score
from 75.1% to 79.0% (crashes 3 to 0 of 100) and the v3 practice block's footprint from 98.3% to 99.5%. It costs
flown coverage (69.2% to 66.4% on practice), because time goes to mapping corridors instead of sweeping.

Because the code changed after the report-only block below had been flown, v2 was checked again on a **new**
report-only block, 94001-94760 (same layout), flown once:

| Pilot | Score (flown) | Footprint | Footprint curve area | Half seen, s | Flights ended by a crash |
| --- | --- | --- | --- | --- | --- |
| Sensor-only planner, view first (v2) | 64.2% | **99.6%** | 0.90 | 13.5 | 0 of 240 |
| Next-best-view RRT (NBVP) | 26.8% | 89.3% | 0.83 | 10.4 | 5 of 240 |
| Sensor-only planner + quad tracker | 68.3% | 88.8% | 0.66 | 39.7 | 1 of 240 |

Paired footprint gap +10.2 points over NBVP (95% interval +8.6 to +11.9), +10.8 over the quad-aware planner (+9.3
to +12.3). On the v3 test split v2 scores 99.0% footprint and 64.6% flown with no crash in 240 flights (v1: 98.7%,
67.9%, 1 crash). On FrontierBench Hard's test split its camera score is 79.1% against NBVP's 65.6% (+13.5, +10.3 to
+16.8), with 2 crashes in 150 flights against NBVP's 25; nearest frontier beats it on the kinematic 40 m task (73.3%
against 68.2%).

## Version 3: a camera to aim, and walls that are not unknown (FrontierBench Hard)

FrontierBench Hard scores a forward 90° camera, and v2 was built for a 360° ring. Round 1 of the hard track set a
goal on its dev split (92001+): a camera score of at least 85%, ahead of every baseline on every task, with no more
crashes. Three attempts, all tuned on dev only:

1. *Narrow the view mask to the camera* (a voxel counts as viewed only when it was within 45° of the heading).
   It helped the quadrotor tasks and hurt the 40 m building, 74.5% to 80.7% overall against v2's 79.0%. Dropped.
2. *Keep the ring for goals, aim the camera.* The planner keeps a second mask of what the camera has had in
   view, and turns toward the 90° window with the most voxels within 3 m it has not (`ViewOptions.look`), up to
   0.8 rad/s; with fewer than 4 left it faces travel as before. 82.2% on dev, no crash in 100 flights. On the
   kinematic airframe, which already turns as fast as it likes along its path, aiming cost coverage on both
   kinematic tasks, so the shipped baseline aims on the quadrotor only.
3. *A bug the per-seed numbers showed.* Some 40 m buildings scored 32% where nearest frontier scored 86%. The
   trace showed the drone working along the walls of one quarter of the building for 1,500 steps: the
   edge-of-unknown test treated a mapped wall voxel as unknown (walls are in the occupancy map, not in the seen
   map), so every wall was a goal. Counting walls as known took that building from 32% to 70% and the dev
   camera score to **86.6%**, 1 crash in 100 flights.

A crash check on 240 more quadrotor flights from the dev range (92001-92080 per task) gave 2 crashes with aiming
and 9 without it, so aiming does not add crashes.

The code was then frozen and a fresh report-only block, **95001-95430** (30 flights per task, task i at 95001 + 100 i),
was flown once by every pilot, including v2 from the commit before this change. Results: `bench/results/hard-95001/`.

| Pilot | Camera score | Ring footprint | Flown | Crashes | large | large-quad | sprint | quad-dark | floors-quad |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| View-first planner v3 | **85.8%** | 90.5% | 26.0% | 4 of 150 | 76.0 | 76.6 | 94.1 | 98.7 | 83.7 |
| View-first planner v2 (before this change) | 78.9% | 87.9% | 25.5% | 2 of 150 | 69.0 | 70.7 | 93.1 | 91.2 | 70.7 |
| Next-best-view RRT (NBVP) | 64.5% | 69.8% | 17.8% | 18 of 150 | 58.8 | 43.2 | 78.3 | 93.4 | 49.0 |
| Nearest frontier (Yamauchi) | 57.1% | 65.8% | 10.0% | 21 of 150 | 75.3 | 41.7 | 68.2 | 66.1 | 34.3 |
| Boustrophedon (BA*) | 45.0% | 48.5% | 23.2% | 13 of 150 | 26.7 | 24.1 | 44.2 | 78.0 | 51.9 |
| Sensor-only planner + quad tracker | 39.1% | 42.5% | 24.1% | 5 of 150 | 24.2 | 16.7 | 42.5 | 68.7 | 43.6 |
| Guarded policy | 38.7% | 42.4% | 20.2% | 43 of 150 | 31.6 | 18.1 | 48.9 | 61.6 | 33.2 |

Paired camera-score gaps (flights resampled within each task, 2000 draws): +21.3 points over NBVP (95% interval
+18.7 to +24.1), +28.7 over nearest frontier (+25.8 to +31.7), +6.9 over v2 (+4.6 to +9.2). On the kinematic 40 m
task the gap to nearest frontier is +0.6 (−3.4 to +5.2): **a tie, not a win**, so the "every task" half of the goal
is met on dev (74.8 against 72.0) but not shown on fresh seeds. Crashes: 4 of 150, the fewest of any pilot but two
more than v2 on the same block; with counts this small (and 2 against 9 on the 240-flight dev check) that is not
evidence either way. On the hard test split (93001+) v3 scores 86.0% (v2 79.1%) with 3 crashes in 150 (v2 2).

The wall fix also changes v3 on FrontierBench v3: test split 99.1% footprint (v2 99.0%), 67.4% flown (v2 64.6%), no
crash in 240 flights. `bench/results/planner-view.json` and `bench/results/hard/planner-view.json` now hold v3.

## Results of version 1

Seed blocks are in the project's seed ledger. Tuning used the FrontierBench dev split and a practice block
(90001+). The report-only block **91001-91630** (30 flights per generated task; the apartments task flies the 60
evaluation plans with fresh env seeds 91701-91760, so its plan geometry has been flown before) was flown once, by
the three pilots below, after the v1 code was frozen. Results: `bench/results/view-first/`.

| Pilot | Score (flown) | Footprint | Footprint curve area | Half seen, s | Flights ended by a crash | room | hard | dark | fast | floors | quad | apartments |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Sensor-only planner, view first | 67.9% | **99.4%** | 0.90 | 13.6 | 0 of 240 | 100.0 | 99.9 | 100.0 | 100.0 | 97.8 | 100.0 | 98.1 |
| Next-best-view RRT (NBVP, Bircher et al. 2016) | 27.0% | 89.6% | 0.84 | 10.7 | 7 of 240 | 100.0 | 91.1 | 97.0 | 94.9 | 58.9 | 89.2 | 96.3 |
| Sensor-only planner + quad tracker | 68.5% | 89.2% | 0.66 | 39.8 | 0 of 240 | 100.0 | 88.3 | 96.0 | 99.2 | 76.4 | 70.5 | 94.0 |

Paired over the same flights (flights resampled within each task, 2000 draws), the view-first planner's footprint
score is **+9.8 points** above NBVP's (95% interval +8.4 to +11.2) and +10.2 above the quad-aware planner's (+9.0 to
+11.4). On the published test split v1 had 98.7% footprint and 67.9% flown coverage, with 1 crash in 240 flights,
the same count as the quad-aware planner (NBVP 89.7%, 5 crashes). `bench/results/planner-view.json` now holds v2.

## Limits

- **The rule was designed with the metric in view.** It uses the metric's 3 m radius, and v3's camera aiming uses
  the hard track's 90° field of view, so the result says the planner can be pointed at footprint coverage, not that
  a real camera would see as much: no occlusion by the drone itself, no motion blur, no detection model.
- **NBVP still sees half the building sooner** (10.4 s against 13.5 s on the 94001 block). It wins early and then
  stops; the view-first planner keeps going until nothing is left unviewed.
- NBVP runs on its paper's parameters and was not tuned; the view-first planner was tuned on the dev split and the
  practice block. The crash counts are small (0 against 7 and 0 in 240), so they say "no more crashes", not "safer".

Reproduce: `npm run bench:score -- --pilot planner-view --base 91001 --plans eval` (and `--pilot nbvp`,
`--pilot planner-tracker`); v3's fresh block: `npm run bench:hard -- --only planner-view,nbvp,yamauchi --base 95001 --count 30 --out bench/results/hard-95001`.
