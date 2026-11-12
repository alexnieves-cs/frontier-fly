# Changelog

Notable changes to frontier-fly. Each release on GitHub carries the full notes and the tech report PDF, and
Zenodo archives it under the concept DOI [10.5281/zenodo.23717793](https://doi.org/10.5281/zenodo.23717793).

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
