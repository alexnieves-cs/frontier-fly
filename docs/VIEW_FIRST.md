# View-first planner

FrontierBench scores two kinds of coverage ([LEADERBOARD.md](LEADERBOARD.md)). *Flown* counts the 0.5 m voxels the
drone's center flew through. *Footprint* counts what came within 3 m of the sensor, the way exploration papers
score coverage. The sweeping planner was built for the first, and on the second a 2016 next-best-view planner
(NBVP, Bircher et al.) beat it: 89.7% against 87.7% for the quad-aware planner on the v3 test split.

The view-first planner (`planner-view` in `src/bench/baselines.ts`, `SensorPlannerOptions.view` in
`src/sim/sensorPlanner.ts`) changes one rule of the planner. A voxel stops being a goal once the drone's **own**
lidar plane has had it in line of sight within 3 m (72 rays cast over the drone's own map, stopping at anything
it has not seen free), not once the drone has flown through it. The search, the path cost, the safety filters,
the quadrotor tracker and the climber are unchanged. When nothing on the floor is left unviewed, the planner goes
back to the flown-voxel goals and sweeps with the time left, so it gives up almost no flown coverage. It reads no
ground truth; the footprint metric's ground-truth rays never reach the pilot.

Going for views means longer legs at cruise speed. On the quadrotor that brought two crashes on the dev split, so
the quadrotor's braking margin was raised from 0.25 m to 0.30 m, chosen among 0.25, 0.30 and 0.35 m on 100 practice
flights (90501-90600).

## Results

Seed blocks are in the project's seed ledger. Tuning used the FrontierBench dev split and a practice block
(90001+). The report-only block **91001-91630** (30 flights per generated task; the apartments task flies the 60
evaluation plans with fresh env seeds 91701-91760, so its plan geometry has been flown before) was flown once, by
the three pilots below, after the code was frozen. Results: `bench/results/view-first/`.

| Pilot | Score (flown) | Footprint | Footprint curve area | Half seen, s | Flights ended by a crash | room | hard | dark | fast | floors | quad | apartments |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Sensor-only planner, view first | 67.9% | **99.4%** | 0.90 | 13.6 | 0 of 240 | 100.0 | 99.9 | 100.0 | 100.0 | 97.8 | 100.0 | 98.1 |
| Next-best-view RRT (NBVP, Bircher et al. 2016) | 27.0% | 89.6% | 0.84 | 10.7 | 7 of 240 | 100.0 | 91.1 | 97.0 | 94.9 | 58.9 | 89.2 | 96.3 |
| Sensor-only planner + quad tracker | 68.5% | 89.2% | 0.66 | 39.8 | 0 of 240 | 100.0 | 88.3 | 96.0 | 99.2 | 76.4 | 70.5 | 94.0 |

Paired over the same flights (flights resampled within each task, 2000 draws), the view-first planner's footprint
score is **+9.8 points** above NBVP's (95% interval +8.4 to +11.2) and +10.2 above the quad-aware planner's (+9.0 to
+11.4). On the published test split, scored once afterwards, it has 98.7% footprint and 67.9% flown coverage, with
1 crash in 240 flights, the same count as the quad-aware planner (NBVP 89.7%, 5 crashes).

## Limits

- **The rule was designed with the metric in view.** It uses the metric's 3 m radius and its 360° horizontal
  plane, so the result says the planner can be pointed at footprint coverage, not that it would see more with a
  real camera. A forward camera with a narrow field of view would need goals that also say which way to look.
- **NBVP still sees half the building sooner** (10.7 s against 13.6 s on the fresh block). It wins early and then
  stops; the view-first planner keeps going until nothing is left unviewed.
- NBVP runs on its paper's parameters and was not tuned; the view-first planner was tuned on the dev split and the
  practice block. The crash counts are small (0 against 7 and 0 in 240), so they say "no more crashes", not "safer".

Reproduce: `npm run bench:score -- --pilot planner-view --base 91001 --plans eval` (and `--pilot nbvp`,
`--pilot planner-tracker`).
