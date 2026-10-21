# PX4 in the loop

The mission editor [exports QGroundControl plans](QGC_EXPORT.md). This page is about what happens when a real autopilot gets one. Two things are checked here:

1. **The flight check.** PX4 SITL (v1.15.4) loads the exact `.plan` files the editor's **Download** button writes, then flies each one. A script checks the flight against the plan. CI does this on every push (`.github/workflows/px4.yml`).
2. **The PX4 bridge.** frontier-fly can be PX4's simulator. PX4's controllers and estimator fly frontier-fly's own physics and sensors over the generated woods, either headless in Node or in the browser at [`px4.html`](https://alexnieves-cs.github.io/frontier-fly/px4.html).

Everything here runs in software and on simulators. Fly a real aircraft only where and how your local rules allow.

## Results

The latest run, made in the cloud container with `scripts/px4/report.ts`. Every row is one plan flown on a fresh PX4 instance.

<!-- px4-results:start -->
| Simulator | Plan | Result | Waypoints | Off route, max / p95 (m) | Height error (m) | Fence margin (m) | Landed from target (m) | Flight (s, plan estimate) | PX4 estimate vs truth, p95 / max (m) | Closest tree (m) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| PX4's own simulator (SIH) | Practice site, land at the pad | pass | 25/25 | 1.15 / 0.80 | 0.25 | 0.89 | 0.36 | 361 (286) | – | – |
| PX4's own simulator (SIH) | Woods seed 11, video on, land at the pad | pass | 10/10 | 1.32 / 0.66 | 0.32 | 2.16 | 0.29 | 113 (61) | – | – |
| PX4's own simulator (SIH) | Woods seed 11, return to launch | pass | 9/9 | 1.51 / 0.69 | 0.31 | 2.16 | 1.70 | 142 (61) | – | – |
| PX4's own simulator (SIH) | Woods seed 4242, return to launch | pass | 12/12 | 1.56 / 1.07 | 0.47 | 2.26 | 2.21 | 144 (61) | – | – |
| PX4's own simulator (SIH) | Woods seed 5150, return to launch | pass | 12/12 | 1.38 / 0.87 | 0.37 | 2.14 | 1.12 | 145 (60) | – | – |
| frontier-fly physics, headless bridge | Practice site, land at the pad | pass | 25/25 | 0.78 / 0.58 | 0.13 | 1.45 | 0.01 | 324 (286) | 0.23 / 0.37 | – |
| frontier-fly physics, headless bridge | Woods seed 11, video on, land at the pad | pass | 10/10 | 0.92 / 0.61 | 0.29 | 3.12 | 0.01 | 102 (61) | 0.27 / 0.35 | 5.7 |
| frontier-fly physics, headless bridge | Woods seed 11, return to launch | pass | 9/9 | 1.42 / 0.62 | 0.29 | 3.18 | 0.02 | 123 (61) | 0.13 / 0.21 | 5.7 |
| frontier-fly physics, headless bridge | Woods seed 4242, return to launch | pass | 12/12 | 1.37 / 0.64 | 0.47 | 3.35 | 0.01 | 129 (61) | 0.28 / 0.40 | 3.7 |
| frontier-fly physics, headless bridge | Woods seed 5150, return to launch | pass | 12/12 | 1.29 / 0.57 | 0.29 | 3.15 | 0.02 | 125 (60) | 0.10 / 0.14 | 5.9 |
| frontier-fly page in Chromium | Woods seed 4242, return to launch | pass | 12/12 | 1.38 / 0.63 | 0.47 | 3.33 | 0.01 | 129 (61) | – | – |
<!-- px4-results:end -->

How to read the table:

- **Off route** is the horizontal distance from each airborne position sample to the planned path (pad, every waypoint, then back to the pad for a return). A multicopter rounds each corner inside its 2 m acceptance radius, so a meter or so is expected.
- **Height error** is the gap between the height when PX4 reached a waypoint and the height in the plan.
- **Fence margin** is the smallest distance from any airborne sample to an edge of the geofence or a no-fly zone, on the allowed side. Negative would mean a breach.
- **Flight time** is longer than the plan's estimate on the woods because PX4's return-to-launch first climbs to its default return height, `RTL_RETURN_ALT` = 30 m, and then descends at landing speed. The plan's estimate assumes the sim's own flight profile.
- **PX4 estimate vs truth** is only measured where frontier-fly is the simulator, because only there do we know the true position. It compares PX4's EKF2 position, as PX4 reports it over MAVLink, with the simulated true position at the same time. It says the simulated IMU, compass, barometer and GNSS are consistent enough for PX4's estimator to converge on them.
- **Closest tree** is the true track's closest approach to any trunk or canopy in the sim world. PX4 knows nothing about the trees. The planner's sweep height keeps the route above them.

## What the flight check does

`scripts/px4/flight_check.py` talks to PX4 over MAVLink with pymavlink, the way QGroundControl does:

1. It waits for a 3D GNSS fix, a home position and passing preflight checks.
2. It uploads the plan's geofence (inclusion polygon and every no-fly zone as an exclusion polygon) and then its mission items, with the MAVLink mission protocol. PX4 runs its own feasibility checks on both.
3. It switches to Mission mode, arms, and records PX4's position, mission progress, landed state and status messages until PX4 lands and disarms.
4. It checks the flight against the plan and writes a JSON report.

| Check | Passes when |
| --- | --- |
| Mission and fence upload | PX4 accepts both uploads (`MAV_MISSION_ACCEPTED`) |
| Takeoff | the vehicle climbs to at least 80% of the takeoff height |
| Waypoints | every waypoint is reached, in order. PX4's `MISSION_ITEM_REACHED` counts. A waypoint PX4 did not report also counts if the track passed within its acceptance radius at its height, because PX4 reports only the latest item and two items reached close together can share one report. The report lists which waypoints were judged that way. |
| Route | no airborne sample is more than 6 m from the planned path |
| Height | within 2 m of the planned height at each reached waypoint |
| Fence | no airborne sample outside the geofence or inside a no-fly zone |
| Landing | landed and disarmed within 5 m of the land point (PX4's home for a return to launch). PX4 lands by its GNSS-based estimate, which can wander a few meters during a flight. |
| No failsafe | PX4 sent no failsafe or fence warning |

The plans come from `scripts/px4/export-plans.ts`. It uses the same code as the editor's Download button: the planner's route, `buildFlightPlan`, then `toQgcPlan`. It uses the default woods mission for seeds 11, 4242 and 5150 and the flat practice site. All of them use the PX4 SITL home. The woods are the export tests' own seeds, and nothing is tuned on them.

### What it found

The first run failed. PX4 refused the practice-site plan with `Geofence violation for waypoint 3`. The editor's sweep preview runs its lanes from fence edge to fence edge, and it routes detours along the edges of no-fly zones, so some waypoints sat exactly on a fence line. The exporter now pulls every waypoint at least 2 m inside the fence (`FENCE_CLEARANCE_M` in `src/mission/export/route.ts`). It does this by shortening lanes and by stepping away from no-fly zone edges. A unit test holds every exported route to that clearance.

## The bridge: frontier-fly as PX4's simulator

PX4 SITL can fly against an external simulator. Its `simulator_mavlink` module connects to TCP port 4560 and runs in **lockstep**. The simulator sends `HIL_SENSOR` with a timestamp from its own clock. PX4 advances its clock to that time, runs its estimator and controllers, and answers with `HIL_ACTUATOR_CONTROLS`, which are the four motor outputs. Then the simulator steps its physics and the cycle repeats, 250 times per simulated second. Neither side can run ahead of the other, so the simulation runs as fast as both can go and gives the same result at any speed.

The simulator side is in `src/px4/`:

| File | Role |
| --- | --- |
| `mavlink.ts` | The MAVLink 2 messages a PX4 simulator needs: an encoder for `HIL_SENSOR`, `HIL_GPS` and `HEARTBEAT`, and a streaming parser for `HIL_ACTUATOR_CONTROLS`. The unit tests check every byte against pymavlink's own encoder (`tests/fixtures/px4/mavlink-frames.json`). |
| `quad.ts` | A quadrotor driven at motor level. PX4 is the controller, so this model takes the four motor outputs and nothing else. It models rotor speed with a first-order lag, thrust and drag torque per rotor, rigid-body rotation, linear drag and ground contact. The rotor layout and spin directions match PX4's generic quad SITL airframe `10016_none_iris`. Mass, inertia and rotor constants are those of the 3DR Iris that PX4's own simulators use, so PX4's default gains fly it without tuning. |
| `session.ts` | The lockstep loop and the sensors. The IMU runs at 250 Hz. The compass and barometer run at 50 Hz. The compass uses Earth's field from PX4's own World Magnetic Model tables, so heading checks pass. The barometer uses the standard atmosphere. GNSS runs at 10 Hz. Each sensor has small, seeded noise. |
| `bridge.worker.ts`, `page.ts` | The browser version. A worker runs the lockstep loop, so rendering never holds PX4's clock back. The page draws the vehicle in the same 3D woods as the outdoor page, with the planned route and the flown track on the map. |

In Node, `scripts/px4/sim-node.ts` listens on TCP 4560 itself. A browser cannot accept a TCP connection, so `scripts/px4/bridge.mjs` accepts PX4's connection and relays the bytes to the page over a WebSocket. It has no dependencies and binds to 127.0.0.1 only. All decoding, physics and sensors run in the page.

The ground under the vehicle is the woods terrain from the plan's own seed, measured from the pad. The plan's heights are relative to the pad and follow the terrain, so PX4 flies the same 3D profile the planner made.

### Try it

You need a PX4-Autopilot checkout at v1.15. Everything below runs locally.

```bash
# once, in PX4-Autopilot
make px4_sitl_default

# in this repository
node scripts/px4/bridge.mjs
# open https://alexnieves-cs.github.io/frontier-fly/px4.html (or npm run dev, then /px4.html)

# in PX4-Autopilot: the generic quad, which waits for an external simulator on TCP 4560
cd build/px4_sitl_default && mkdir -p rootfs && cd rootfs
PX4_SYS_AUTOSTART=10016 ../bin/px4 -d ../etc
```

Then open QGroundControl, which connects to PX4 by itself. In QGroundControl, load the plan from the page's **Download .plan** button, upload it and start the mission. Or run the flight check instead of QGroundControl:

```bash
pip install pymavlink
python3 scripts/px4/flight_check.py ~/Downloads/*-seed4242-px4.plan
```

The page's `?seed=` picks the woods (default 4242). `?ws=` points at another bridge address. `?speed=0` runs the lockstep as fast as PX4 answers instead of in real time. Restart PX4 and the bridge after reloading the page, because PX4's clock cannot go back to zero.

### Run the checks

```bash
export PX4_BUILD=/path/to/PX4-Autopilot/build/px4_sitl_default
npx vite-node scripts/px4/export-plans.ts -- --set full --out test-results/px4/plans
scripts/px4/run-check.sh test-results/px4/plans test-results/px4/sih sih         # PX4's own simulator
scripts/px4/run-check.sh test-results/px4/plans test-results/px4/bridge bridge   # frontier-fly physics, headless
CI=1 scripts/px4/run-browser-check.sh test-results/px4/plans/woods-4242-rtl.plan test-results/px4/browser
npx vite-node scripts/px4/report.ts -- --sih test-results/px4/sih --bridge test-results/px4/bridge --browser test-results/px4/browser --write
```

`PX4_SIM_SPEED_FACTOR` sets how much faster than real time the headless checks run. CI and the table above use 2. At 4, PX4 sometimes raised a failsafe the flight did not cause: once its own battery simulator stopped publishing for 5 s of PX4 time, and once a SIH run failed safe just after takeoff. The flights themselves were clean.

## Limits

- Only the pre-planned part of a mission is a plan. Confirming a contact, entering a building and searching indoors are decided in flight by frontier-fly's own autonomy, not by PX4.
- The bridge vehicle is a rigid body with first-order motors and linear drag. It has no blade flapping, no ground effect, no battery model and no wind gusts. Collisions with trees and buildings are measured (closest tree) but not simulated. PX4 is never told about obstacles.
- PX4's GNSS here has no multipath or dropouts, so the estimator has an easy time. The fault bench in [SAFETY_CASE.md](SAFETY_CASE.md) covers degraded sensors with frontier-fly's own failsafe, not with PX4's.
- On GitHub's runners, PX4's own simulator (SIH) has twice stopped publishing its simulated barometer and compass about a second after takeoff. PX4 then terminated the flight, and the check failed with `No valid data from Baro 0`. It has not happened in the cloud container or with frontier-fly as the simulator, and the cause is not known yet. A failed flight prints PX4's warnings to help find it.
- The checks pin PX4 to v1.15.4. A newer PX4 may change defaults such as the return height.
- ArduPilot is not flown yet. Its `.plan` and `.waypoints` exports are checked only against QGroundControl's loaders ([QGC_EXPORT.md](QGC_EXPORT.md)).

## Code

| File | Role |
| --- | --- |
| `scripts/px4/export-plans.ts` | Writes the `.plan` files and a `.meta.json` next to each (pad, seed, plan summary) |
| `scripts/px4/flight_check.py` | Uploads, flies and checks one plan over MAVLink |
| `scripts/px4/run-check.sh` | Flies every plan in a folder on a fresh PX4, with SIH or the headless bridge |
| `scripts/px4/run-browser-check.sh` | PX4, the bridge, Chromium on `px4.html`, and the flight check together |
| `scripts/px4/sim-node.ts` | The headless simulator. It also writes the true track and the closest approach to any tree. |
| `scripts/px4/bridge.mjs` | The TCP-to-WebSocket relay |
| `scripts/px4/report.ts` | Collects reports into `docs/px4-flight-check.json` and the table above. `--check` keeps them in sync in CI. |
| `scripts/px4/mavlink_fixtures.py` | Regenerates the pymavlink test frames |
| `src/px4/` | The MAVLink codec, the quad, the sensors, the lockstep session and the page |
