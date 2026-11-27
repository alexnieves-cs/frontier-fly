# Changelog

Notable changes to frontier-fly. Each release on GitHub carries the full notes and the tech report PDF, and
Zenodo archives it under the concept DOI [10.5281/zenodo.23717793](https://doi.org/10.5281/zenodo.23717793).

## Unreleased

### FrontierBench: the scoreboard, published baselines, overwatch and integrity (2026-11-27)

- The homepage and the README open on a generated scoreboard: 8 tracks by 8 pilots, every cell from
  `bench/results` with its test seed block (`docs/scoreboard.json`). Below it, two test flights replayed in the
  browser, recorded from the bench's own episodes (`npm run bench:replay`) and checked against the result files
  when the docs are generated.
- Baselines on every track and the same seeds: random walk, greedy information gain (Gonzalez-Banos & Latombe
  2002) and rapid frontier (Cieslewski et al. 2017).
- `npm run bench:hard -- --track overwatch` (dev 150001+, test 151001+): find a lost person who never reacts to
  the drone and keep them in view from 2 m or more, with one watch layer shared by every explorer pilot.
  Information gain leads at 61.8%; the view-first planner is 2nd at 56.3% (v4).
- `npm run bench:hard -- --track integrity` (dev 160001+, test 161001+): the pilot reports a protection level,
  scored as availability minus 10 x the hazardously misleading rate (alert limit 1.5 m, time to alert 2 s). BA*
  coverage leads at 81.5% by reporting nothing; the view-first planner, reporting its estimator's 1 sigma, is 6th
  at 55.7%. Draft writeup: `docs/INTEGRITY.md`.
- CONTRIBUTING: no capability lands without a scored track and a number attached.

### Fix the planner's known weak spots (2026-11-27)

- View-first planner v4 (`ViewOptions.frontiers`, kinematic airframe): the edge of unmapped space competes with
  unviewed voxels by path cost from the start. The kinematic 40 m tie with nearest frontier is now a lead: fresh
  block 153001+ 84.5% vs 73.4% (+11.1, +6.6 to +16.1); hard 87.0% vs v3's 84.7%. Hard test 87.8% (v3 86.0%), return
  test 75.5% (74.0%). Off on the quadrotor, where it cost 11 points flying on the pose estimate.
- Quadrotor mission touches at stairwells: the shaft-centring probe now centres on the stretch it saw open and
  settles between probes, and indoors the up and down range finders veto a climb or descent into a slab. Practice
  compound sweeps: touched flights 8 to 0 (estimate) and 4 to 0 (true pose) of 152; fresh 79401+: 8 to 4 of 240,
  the four left on one seed with a different cause.

### FrontierBench Hard: the hard drift track (2026-11-27)

- `npm run bench:hard -- --track drift-hard`: four return tasks on the drift estimate with a short-range
  ranging ring (24 beams to 4 m, `TOF_RING` in `src/bench/hard.ts`) instead of 90 beams to 8 m: the 40 m
  building and back in 120 s and 240 s, in wind, and dark rooms. `DriftProfile` in `src/bench/drift.ts` sets
  the sensor suite and estimator per task.
- Test 111001+: view-first 48.1% on its estimate vs 77.1% on the true pose (−29.0, CI −35.5 to −22.7; 77 vs
  118 of 120 flights home); still +14.2 over nearest frontier and +16.7 over NBVP.
- Loop closure in flight (`--loop`, `LoopCloser` with multi-sweep keyframes, `accumulate`): 47.8%, a tie
  (−0.3, −8.4 to +8.1). More flights home (90 of 120) but less seen. Looked +10 on tuning seeds, −3.7 on
  held-out dev, so not the default.

### FrontierBench Hard: the degraded-sensor track (2026-11-27)

- `npm run bench:hard -- --track sensor`: four quadrotor tasks on the drift estimate with sensor faults that hit
  the pilot and the estimator alike (`SensorFaults` in `src/bench/drift.ts`): a dead 90° lidar sector, a dead
  flow sensor in the dark, one lidar ray in four, and 120° of lidar plus flow failing at 20-60 s. `--healthy`
  flies the same seeds with working sensors.
- `SensorPlannerOptions.mapFill` (on for the view-first planner on the quadrotor): a no-return lidar ray brakes
  for the walls and unknown space on the drone's own map, and a lidar the map keeps contradicting slows the
  drone to 0.6x. Dev crashes 18 -> 0 of 80; no change on the healthy tracks' dev splits.
- Test 141001+: view-first 80.0% (3 crashes in 120) vs 87.2% with healthy sensors; NBVP 53.0% (52 crashes),
  nearest frontier 47.0% (61), BA* 46.0% (24), quad-aware planner 30.7% (42).

### FrontierBench Hard: the drift track (2026-11-26)

- `npm run bench:hard -- --track drift`: four quadrotor tasks flown with no ground truth in the pilot's loop. The
  pose is the src/loc estimator's (IMU, flow, range finder, lidar matched to its own map), the seen and flown maps
  are painted from it (`src/bench/drift.ts`), and the score stays on the true pose. `--truth` flies the same seeds
  on the true pose as a paired control.
- Test 99001+: view-first planner 78.7% (89 of 90 home, no crash) vs 78.5% on the true pose; NBVP 55.9%
  (20 crashes), nearest frontier 51.0% (28), BA* 41.5%, quad-aware planner 36.2%. Mean pose error 0.12 to 0.22 m.

### FrontierBench Hard: the return track (2026-11-26)

- `npm run bench:hard -- --track return`: four tasks (40 m and back on the quadrotor, the same in 1 m/s wind, dark
  rooms in 40 s, a 30 s sprint) where the camera score counts only if the drone ends within 1.5 m of take-off.
  Every explorer pilot shares one return supervisor (`HOME_RETURN`); `SensorPlanner` gained `homing` and
  `homeDistance()`; bench tasks take `home` and `wind`.
- Test 97001+: view-first planner 74.0% with 120 of 120 home and no crash, nearest frontier 53.1% (17 crashes),
  NBVP 49.7% (22), BA* 32.3%, quad-aware planner 26.0%.
- Tried and not shipped: information-gain goal scoring (+4.6 on dev, +1.0 on test, slightly worse on v3).

### View-first planner v3: camera aiming and a wall fix (2026-11-26)

- `ViewOptions.look`: the planner keeps a camera view mask and turns a 90° forward camera toward what it has not
  had in view (quadrotor only in `planner-view`, up to 0.8 rad/s). Fix: the edge-of-unknown test counted mapped
  walls as unknown, so in some 40 m buildings the drone worked along walls (one dev building 32% to 70%).
- Fresh report-only hard block 95001+ (900 flights, every pilot once): camera score 85.8% (v2 78.9%) against NBVP
  64.5%, nearest frontier 57.1%; ties nearest frontier on the kinematic 40 m task (+0.6, −3.4 to +5.2). Crashes
  4 of 150 (v2 2, NBVP 18). Hard test 86.0% (v2 79.1%); v3 test 99.1% footprint, 67.4% flown, 0 crashes.
- `bench:hard --base N --count M` flies a ledger block.

### FrontierBench Hard (2026-11-26)

- New track `npm run bench:hard` (src/bench/hard.ts): 40 x 40 m buildings in 120 s (kinematic and quadrotor), 30 s
  sprints, the quadrotor in the dark and across floors, scored by a forward 90° camera footprint (3 m, along the
  heading). v3 stays frozen. Test split 93001+: view-first planner 79.1%, NBVP 65.6%, nearest frontier 58.4%,
  boustrophedon 44.6%, quad-aware planner 39.5%, guarded policy 38.6%.
- View-first planner v2: fly to the edge of unknown space before sweeping (the hard track's 40 m buildings showed
  v1 giving up on corridors the lidar had not mapped to the end). Fresh block 94001+: 99.6% footprint vs NBVP 89.3%,
  0 crashes in 240; v3 test 99.0% footprint, 64.6% flown (v1 67.9%).
- `FootprintCoverage` takes a field of view; bench episodes on camera tasks record `camera`.

### The learned quad picker is the default quad explorer (2026-11-25)

- `quadPilot()` (src/eval/pilots.ts, src/sim/quadPilot.ts): the planner with braking filter and tracker, its goal
  chosen by the frozen learned picker, planning with a more cautious braking model (0.25 s reaction, 4 m/s²).
- Report-only seeds 89401-89732, flown once: 54.2% coverage with 1 crash in 664 flights, against 45.9% and 30 crashes
  for the planner-only quad pilot. Most of the crash cut is the braking model; the coverage lead is the picker.
  Missions, teams and FrontierBench are unchanged. Write-up in [docs/QUAD_PICKER.md](docs/QUAD_PICKER.md).

### View-first planner (2026-11-25)

- New baseline `planner-view` (`SensorPlannerOptions.view`): goals are voxels the drone's own lidar has not had in
  line of sight within 3 m, then a sweep of unflown voxels. On a fresh report-only block (91001+) its footprint
  coverage is 99.4% against NBVP's 89.6% (paired +9.8, 95% interval +8.4 to +11.2), with 0 crashes in 240 flights
  (NBVP 7, quad-aware planner 0) and about the same flown coverage. FrontierBench test: 98.7% footprint, 1 crash in
  240. docs/VIEW_FIRST.md.
- `bench:score --base N` flies a seed block outside the bench's splits.

### Claims audit (2026-11-17)

No study was rerun and no number changed; the wording around the numbers did.

- README results: the mission row now says its four scenarios are staged, and a new row gives the rate over the
  failure atlas's mix of conditions (41.2% of 6,000 flights end as expected). One false-clear number (the atlas's,
  at the shipped code) replaces three. The collision rate is the 6,000-flight plain draw, since the collision search
  never produced an importance-sampled estimate. The fault row breaks "ends safely" into finished, flew home,
  landed in place and crashed. The policy row counts flights that touched a wall, and the dark row says the drone
  flies on lidar.
- Leaderboard: crash counts instead of rounded shares (the leader has 1 crash in 240 flights, not 0%), and a paired
  95% interval between neighbouring ranks; ranks 2 to 4 are tied.
- Model card: the policy alone crashes on 7 of 30 hard flights (it said "most"), and the guarded coverage number has
  no planner fallback in it.
- Report limits: staged missions, free slow contacts, the built-in controller braking harder than PX4, and the one
  report block (79201+, quadrotor on its pose estimate) where a bug was traced on a published seed.

## 1.2.0 (2026-11-12)

The wrap-up release. frontier-fly is at a stable stopping point: everything below was finished, measured and
documented, and no new features or studies are planned.

### Benchmark

- **FrontierBench v3 under the crash rule.** A flight ends on a contact faster than the crash speed and energy
  is priced on the current airframe, as in every study in the report. Episodes record crashes, the summaries
  report crashed flights and crash share, and the leaderboard has a crash column. The v2 results move to
  `bench/results/v2/` and stay on the leaderboard page for reference (#120).
- **Baselines re-scored under v3.** The quad-aware planner leads (68.0%, no crashes); the planner scores 66.2%
  and the guarded policy 62.7%, with 21% of its flights ended by a crash. New baseline `planner-tracker`, the
  report's quadrotor pilot (#120).
- **A learned goal picker for the quadrotor.** The sensor planner can hand its next-goal choice to a picker
  trained with decision-level PPO from the tuned planner. On report-only quadrotor buildings it covers 54.8%
  against 46.2% for the published pilot and 51.1% for the tuned planner, with 21 against 27 crashes in 664
  flights. Write-up in [docs/QUAD_PICKER.md](docs/QUAD_PICKER.md) (#117).

### Missions

- **A safer way out of buildings.** When the lidar shield blocks the route to the exit, the drone no longer
  flies the unshielded fly-through from metres away; it re-routes, then lands in place if the opening stays
  out of reach (#120).
- Mission bench re-run on its published seed block: expected outcomes unchanged in every arm; the quadrotor on
  the pose estimate touches something in 5 of 120 flights (was 6) (#120).

### Docs, media and release

- Films, the story video, clips and stills re-rendered from the release code (#119).
- The guide and report say what the replayed touches have in common, and generated notes cover the saturated
  easy stage and the night ratio (#120).
- A **Project status** section in the README, and this changelog.
- `frontierfly` is not on PyPI yet, so the README and `python/README.md` now say to install it from a checkout.
  The `pypi` workflow still builds and tests the wheel on every Python change, publishes on a published GitHub
  release (or by hand) once the trusted publisher is set up and the `PYPI_PUBLISHING` variable is `true`, and
  skips the upload with a notice instead of failing until then ([docs/RELEASING.md](docs/RELEASING.md)).
- `ci.yml` also runs on pushes to main, so its badge reflects main; Dependabot holds TypeScript 7 until
  typescript-eslint supports it; the Python package version follows `package.json`, checked by a test (#120).
- Credits links use https.

## 1.1.0 (2026-11-11)

False clears fell from 3.55% to 0.25% of flights, a crash rule and a retrained policy, one shared quadrotor
airframe validated against PX4 SITL, indoor hazards (doors, glass, smoke) and an opt-in thermal camera,
published baselines on FrontierBench, a headless CLI and Docker image, the `frontierfly` Python package, a
Crazyflie explorer, and a published negative result on quadrotor policy training.
[Release notes](https://github.com/alexnieves-cs/frontier-fly/releases/tag/v1.1.0).

## 1.0.0 (2026-10-28)

First release. [Release notes](https://github.com/alexnieves-cs/frontier-fly/releases/tag/v1.0.0).
