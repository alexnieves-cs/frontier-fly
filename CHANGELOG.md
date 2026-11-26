# Changelog

Notable changes to frontier-fly. Each release on GitHub carries the full notes and the tech report PDF, and
Zenodo archives it under the concept DOI [10.5281/zenodo.23717793](https://doi.org/10.5281/zenodo.23717793).

## Unreleased

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
