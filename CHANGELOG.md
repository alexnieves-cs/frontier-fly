# Changelog

Notable changes to frontier-fly. Each release on GitHub carries the full notes and the tech report PDF, and
Zenodo archives it under the concept DOI [10.5281/zenodo.23717793](https://doi.org/10.5281/zenodo.23717793).

## Unreleased

### Crazyflie: ready for a first real flight (software side)

Nothing has flown on hardware yet; everything below is tested against the simulator and a simulated radio.

- **Bring-up.** `scripts/crazyflie/bringup.py scan` finds the Crazyradio and the drone's URI; `bringup.py link`
  measures the link for 20 s with the props off (decks, telemetry freshness and gaps, cflib's round trip and link
  quality) and never starts the motors.
- **Tighter limits for the first flights.** hover, yaw and square fly at 0.3 m/s at most, under a 0.9 m ceiling,
  with a 1.5 m motor cut and 40 or 60 s in the air. The guard's log line says which rule trimmed a command.
- **One flight log for the drone and the sim.** Version 1 of the log (`flightlog.py`) records what flew where, every
  control step, the sim's true pose and the radio's statistics, and is checked on read.
- **A simulated radio.** `--mock-radio` adds packet loss, delay, blackouts and the firmware's commander watchdog
  between `fly.py` and the simulated drone. The simulated drone also has a `bench` profile with the bench model's
  latency and flow drift.
- **Sim versus real.** `compare.py` replays a flight's arena and mode in the sim and writes a report: the measures
  side by side, where the real flight falls outside the sim's spread, the fitted velocity lag and latency, and a
  track picture. `--landed X,Y` turns a tape-measured landing spot into the real flow drift.
- **Kit and checklist.** `docs/CRAZYFLIE_FIRST_FLIGHT.md`.
- **Landing waits for touchdown.** The landing used to cut the motors when its setpoint ramp ran out, which in
  the sim left the drone about 0.2 m up, and higher when the radio dropped out mid-landing. It now holds the
  setpoint close to the measured height and stops the motors once the drone reads under 8 cm.
- **An error mid-flight lands the drone.** An exception between take-off and touchdown (the planner process
  dying, a reply that is not JSON or has a field missing, a bug in a mode's step) used to go straight out of
  `fly.py`, which closed the radio link with the motors on; on the real drone the firmware would have cut them
  2 s later, wherever it was. It now lands at once and logs an `error` event (outcome `error-landed`). If the
  landing loop fails too, `CflibLink.land_onboard` hands the descent to the firmware, then the motors stop.
- **A planner sending NaN no longer passes as a finished flight.** A reply with a null or non-finite number
  refreshed the stale-command watchdog every step, so the drone hovered until the time limit and the run
  reported `complete`. It now counts as no reply: the guard hovers, then lands after 2 s.
- A flight log that cannot be written (a full disk) no longer stops a flight, and a planner that dies at start-up
  is reported at once instead of after 60 s.
- 49 new tests (78 in the Crazyflie suite).

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
