# Safety case

This document argues that frontier-fly's autonomous search-and-rescue mission behaves safely inside its simulated operating envelope, including when parts of the drone fail. It lists the hazards, the mitigation for each, and the test or benchmark that checks it.

**What this is and isn't.** frontier-fly is a browser simulator, and this is a safety case for the simulated autonomy stack. It is not an airworthiness argument for a real aircraft, and no claim here transfers to hardware without flight testing. Its job is to make the autonomy's safety reasoning explicit and testable, the way a real program would (UL 4600 for autonomous products and the JARUS SORA operational risk assessment are the models followed). The project is civilian and defensive: a drone looking for a simulated person in a simulated building, with an operator in the loop for entry decisions.

**Where the pieces are.** The fault model is `src/mission/autonomy/faults.ts`, the failsafe `src/mission/autonomy/failsafe.ts`, the fault scenarios `src/mission/autonomy/faultScenarios.ts`, and the runner wiring `src/mission/autonomy/runner.ts` (`advanceFaults`, `checkHealth`, `believed`). Unit tests are in `tests/unit/autonomy.faults.test.ts`. The fault bench (`src/eval/faults.ts`, `npm run study:faults`) flies every scenario on fresh seeds; its results close this document (section 7).

## 1. System and operating envelope

The drone takes off from a pad, sweeps a geofenced wooded area, finds buildings, picks an entry opening (asking the operator when the opening is marginal), explores inside, confirms a target from several bearings or confirms the area clear, exits, returns, and lands. The mission layer is a phase machine (`src/mission/autonomy/runner.ts`) over planners for each phase, with two monitors checked every tick:

- the **safety monitor** (`safety.ts`) keeps a healthy drone inside the mission envelope: geofence and no-fly zones with lookahead, the altitude band, the battery reserve, and a collision escalation ladder;
- the **failsafe** (`failsafe.ts`) decides whether the drone is still healthy, from its own symptoms, and degrades or ends the mission when it isn't.

The runner carries out the stronger of the two actions. Actions rank `none < hold < backtrack < exit_building < return_home < land`. Inside a building, "return home" always exits through the entry opening first.

**Operating envelope assumed by this case.**

| Parameter | Envelope | Source |
|---|---|---|
| Airframe | 1.2 kg quad, 10 in props, 4S 5000 mAh, thrust-to-weight 2; rigid-body dynamics with an angle-mode flight controller whose velocity loop holds ground speed | `battery.ts`, `quadVehicle.ts` |
| Speed caps | 5.5 m/s outdoors, 1 m/s indoors | `vehicle.ts` |
| Wind | Mean wind under 3 m/s (20 s average; gusts pass through). Above it the mission ends, because the planners' clearances near trees and buildings are sized for light air. Over 6 m/s on the 2 s estimate it ends at once | `failsafe.ts` `windLimitMps`, `windSevereMps` |
| Lighting | Day and night (flashlight); detector recall falls at night | `perception.ts` |
| Area | Inside the mission geofence, outside no-fly zones, within the altitude band | `safety.ts` |
| Buildings | Generated shells, entered only through openings with at least 0.38 m airframe clearance | `planner/entry.ts` |
| People | Simulated. No real person, animal, or property is ever involved | SPEC.md |

## 2. Top-level argument

Written in Goal Structuring Notation (GSN) style, flattened to a list.

- **G0.** The mission autonomy is acceptably safe within the envelope in section 1.
  - **C0 (context).** "Safe" means: no flight ends in the air or away from home when getting home was possible; no collisions beyond the documented contact rate; no area is declared clear while it holds a person; no building is entered without either a safe opening or the operator's approval.
  - **S0 (strategy).** Argue over each hazard in the hazard log (section 3), with nominal and faulted conditions handled separately.
  - **G1.** On a healthy drone, every nominal hazard is mitigated. *Evidence:* safety monitor unit tests, the mission bench (`docs/mission-bench.json`: 0 collisions, 0 false clears, 0 false alarms in 270 flights on fresh seeds).
  - **G2.** Every fault in the fault model is detected from symptoms the drone actually has, within a bounded time. *Evidence:* failsafe unit tests; detection latency per fault scenario on the fault bench.
  - **G3.** After detection, the response leaves the drone in a safe state (section 4). *Evidence:* the fault bench's safe-outcome rate per scenario.
  - **G4.** Faults that produce no symptom are named, with their consequence, rather than assumed away (section 5).
  - **A0 (assumption).** The simulator's models (energy, dynamics, sensor noise, detector recall) are representative enough to compare designs. This is the main limitation, discussed in section 6.

## 3. Hazard log

Severity: **Cat** catastrophic (uncontrolled flight or harm in a real deployment), **Maj** major (mission failure with a safety-relevant consequence such as a false clear), **Min** minor (mission degraded, drone safe). Likelihood is qualitative for the simulated envelope.

| ID | Hazard | Cause | Effect | Sev | Mitigation | Unit evidence | Bench evidence |
|---|---|---|---|---|---|---|---|
| H1 | Fly-away or loss of oversight | Operator link lost | No one can abort; the drone keeps flying | Cat | Missions are autonomous, so flight continues for `commsRthSec` (20 s) outdoors or `commsInsideSec` (60 s) inside, where walls block the link, then the drone returns home | `failsafe` "rides out a short link loss and returns home after a long one" | `comms-short`, `comms-lost`, `compound` |
| H2 | Entering a building without consent | Link lost while a marginal opening waits for approval | The drone enters an opening the operator never approved | Maj | While the link is down every entry request is treated as denied (`denyAsks`) | same test, `denyAsks` assertions | `comms-lost` (entries recorded as denied) |
| H3 | Collision indoors | Lidar arc fails, either flagged or silently reading open, or noisy ranges | The obstacle guard sees open space where a wall is | Maj | Flagged beams are unknown at once and treated as blocked, never open. Silent beams are caught when they report no return where the drone's own map has a wall 1 to 6.5 m away, seen by the lidar at about the same height, for 20 scans (1 s) on 3 adjacent beams, then latched. Speed drops; a blind arc of 45 degrees or more inside sends the drone out of the building | `failsafe` "treats flagged lidar beams as unknown…", "catches a silent lidar arc…", "does not call a long corridor a silent fault" | `lidar-flagged-indoor`, `lidar-silent-indoor`, `lidar-noise` |
| H4 | False clear | Camera frames lost | Rooms or woods are marked searched without being seen | Maj | Seen-empty maps only grow from detector frames (`perception.ts`), so a camera outage leaves scopes incomplete instead of clear. After `cameraRthSec` (10 s) without frames the search ends and the drone returns | `failsafe` "keeps searching through a short camera dropout…" | `camera-short`, `camera-lost` |
| H5 | Geofence breach or lost drone | GNSS fix lost outdoors | Dead-reckoning error grows until the drone is not where it thinks | Cat | Hold up to `gnssHoldSec` (5 s) for the fix, then return on dead reckoning at reduced speed; land in place after `gnssLandSec` (90 s). Inside buildings GNSS is expected to be absent and is ignored | `failsafe` "holds, then returns home, then lands…" | `gnss-short`, `gnss-lost` |
| H6 | Loss of control | Motor or prop degradation | Too little thrust or torque to follow the plan; overshoot near walls | Cat | ESC telemetry below 90% of command for 0.3 s latches a motor fault. Speed caps shrink to the remaining authority; the drone returns home, or lands at once when thrust-to-weight is under 1.3 | `failsafe` "goes home on a weak motor and lands on a severe one"; `control authority` tests | `motor-weak`, `motor-severe` |
| H7 | Battery exhaustion in flight | Weak cell: sudden gauge drop, faster drain | The healthy energy model underestimates the trip home | Cat | The failsafe integrates the model's predicted drain beside the gauge. A step of 3 points or a drain 1.15x faster than predicted over 20 s flags an anomaly, every energy estimate is scaled by the measured ratio, and the scaled reserve check sends the drone home. The safety monitor's critical-level landing (8%) stays as the last line | `failsafe` "notices a gauge step and a fast drain…"; `fault injector` "drains a sagging pack faster…" | `battery-sag`, `compound` |
| H8 | Collision or unable to get home in wind | Wind and gusts beyond what the close-in planners' clearances allow | Gusts push the drone into trees or walls; a headwind drains the pack | Maj | Inspection legs keep 0.25 m more air off walls and corners than other legs. The headwind toward home scales the energy estimate (cruise over ground speed). A 20 s mean wind over 3 m/s for 5 s ends the mission and sends the drone home; over 6 m/s for 2 s the drone first climbs above the roofline. Going home, 30 s without closing 3 m on the pad means the airframe can't beat the wind, so it lands where it is. The landing loop integrates position error so a steady wind doesn't leave it off the pad | `failsafe` "returns home in sustained strong wind…", "lands when it makes no headway home in wind" | `wind-light`, `wind-moderate`, `wind-strong`, `compound` |
| H9 | Collision or false clear from a wrong pose indoors | Indoor estimator degraded or lost (odometry frozen, scan matching fails) | The map and the path home are offset from the real walls | Maj | Pose 1-sigma over 0.35 m (or `ok` false) for 1 s: leave the building slowly. Over 1 m or `lost`: land in place, because flying blind indoors is worse than landing | `failsafe` "leaves a building when the pose estimate degrades…" | Indoor localization track (A3) adds the scenarios |
| H10 | Geofence or no-fly breach | Planner command, overshoot | Flight over areas outside the mission's permission | Cat | Lookahead hold, return home if it persists 3 s, immediate return on an actual breach | `autonomy.safety.test.ts` | mission bench |
| H11 | Altitude excursion | Terrain, planner error | Too low over the woods or above the ceiling | Maj | Hold to correct; low flight allowed only near home and buildings in close-in phases | `autonomy.safety.test.ts` | mission bench |
| H12 | Repeated collisions | Clutter, planner error | Damage, loss of control | Maj | Escalating ladder within 30 s: hold, backtrack, exit, return home, land | `autonomy.safety.test.ts` | mission bench |
| H13 | False alarm | Decoys, detector confusion | Rescuers sent to a mannequin | Min | Confirmation from at least 3 views at least 25 degrees apart, with a per-class confidence threshold | `autonomy.planner.test.ts` | mission bench (0 false alarms) |
| H14 | Unsafe entry | Narrow or hazardous opening | Collision in the opening | Maj | Opening safety score from clearance, kind and obstruction: below 0.5 never, 0.5 to 0.75 ask, above 0.75 auto | `autonomy.planner.test.ts` | mission bench |

## 4. Safe states and response design

The responses follow common flight-controller practice (PX4's failsafe actions and ArduPilot's battery and GCS failsafes are the reference points), adapted to a mission that spends time inside buildings:

- **Continue.** Faults with no safety consequence (a short link drop, a short camera gap, a narrow lidar gap, moderate wind) slow the drone or narrow what it may do, and the mission goes on.
- **Return home.** The default for anything that makes finishing the mission unsafe but leaves the drone able to fly. Inside, the drone first retraces its way to the entry opening, which it has already flown through, so the exit route is known-free.
- **Land in place.** Only when flying further is the bigger risk: thrust too low to hold altitude, an indoor pose estimate that is lost, a GNSS outage long enough that the pad can't be found, wind the airframe can't make headway against, or a critical battery.

Two design rules hold across every response. First, unknown is never treated as safe: a flagged lidar beam is an obstacle, a missing camera frame is unseen area, a missing operator answer is a denial. Second, once the drone is going home it never turns back toward the mission (the runner's `returning` latch), so a fault that clears a moment later does not restart a mission whose margins were already spent.

## 5. Faults the drone cannot detect

These are listed so nobody assumes they are covered.

- **A silent lidar arc facing unmapped space.** The cross-check needs the drone's own map to predict a wall. A sector that fails before the drone has mapped the walls it faces looks like open space until the map disagrees. Speed and the collision ladder remain the mitigation. The `lidar-silent-indoor` scenario measures how often this matters.
- **A motor fault without ESC telemetry.** Detection uses per-motor telemetry (DShot-style). An airframe without it would only see tracking error, which this failsafe does not use yet.
- **A slow pack degradation under 15%.** Below the rate threshold the anomaly is not flagged; the reserve margin of 20% absorbs it.
- **Wind without an estimate.** The wind check needs the flight controller's wind estimate (modelled as the true wind low-passed over 2 s); indoors there is none and none is needed.
- **A silent lidar arc while the drone climbs or descends.** The map cross-check only trusts walls seen at about the current height, so it waits until the drone has mapped at its new height.

## 6. Limitations of the evidence

- Everything is simulated. The energy model is momentum theory with fixed efficiencies; the fault bench flies the project's own rigid-body quadrotor model (the kinematic vehicle has no velocity loop, so it drifts in wind); sensor noise and detector recall are parametric. GNSS dead-reckoning drift is 0.15 m/s, a flow-aided figure; IMU-only drift would be far worse.
- The fault bench uses fresh world seeds that were never used for tuning (section 7), but its fault parameters were chosen by hand. A different fault distribution could change the rates.
- The quad collides on about 3% of fault-free flights (the `nominal` row), so a few collisions in every row are the airframe's baseline rather than the fault's doing.
- **Known open issues from the final run.** In 8 m/s wind an emergency landing waits to be over an open column, and a drone that can't make headway may never get there: 12 of 60 `wind-strong` flights were still airborne at the time limit (none were in the first run). Descending in place after a timeout is the obvious fix; it is not in this version because it was found on the evaluation seeds. A strong wind front that arrives while the drone inspects within a metre or two of a wall pushes it into the wall before the 2 s detector fires. `gnss-lost` and `wind-moderate` remain the weakest in-envelope scenarios, mostly through contacts on the trip home and landings drifting off the pad.
- The thresholds in `DEFAULT_FAILSAFE` were set from the airframe model and flight-controller defaults, not tuned on bench seeds. If any are tuned later, it will be on development seeds and recorded here.

## 7. Fault bench

**Protocol.** Each fault scenario in `faultScenarios.ts` runs on the three mission scenarios with fresh seeds (a block no other study uses), flying the quadrotor with the sensor-only frontier explorer indoors. Fault timing and parameters come from the flight seed. Per scenario the bench reports: safe-outcome rate with a Wilson 95% interval, the share of flights that produced the intended response, detection latency (first failsafe event minus fault start), collisions per flight, false clears, false alarms, entries made while the link was down, and landings away from home.

**Results.** {{FAULT_BENCH_NOTE}}

{{FAULT_BENCH_TABLE}}

{{FAULT_BENCH_SUMMARY}}

{{FAULT_BENCH_HISTORY}}
