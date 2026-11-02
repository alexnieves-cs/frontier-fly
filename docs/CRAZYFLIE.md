# A real flight on a Crazyflie

frontier-fly's sensor-only explorer is the same code that searches buildings in the sim. This page is the plan for flying it on a real 33 g quadrotor, a Bitcraze Crazyflie, in an ordinary room. It has three parts:

1. **What is done in the cloud.** A Crazyflie version of the explorer, a room-scale simulator, a 60-arena benchmark, a flight script with a safety guard, and 29 tests that fly the script against the simulator in CI (`.github/workflows/crazyflie.yml`).
2. **What to buy.** About US$500 to $700 of parts.
3. **How to fly it.** A safety checklist and a ladder of seven steps, from "connect with the props off" to "explore two rooms". Each step has a pass rule, and you only move up a step when the one below passes.

Nothing here has flown on hardware yet. Every number below is from simulation.

## What the explorer has to work with

The sim drone has a 32-ray lidar. A Crazyflie with a [Multi-ranger deck](https://www.bitcraze.io/products/multi-ranger-deck/) has four horizontal time-of-flight beams (front, back, left, right, up to 4 m) and one pointing up. The [Flow deck v2](https://www.bitcraze.io/products/flow-deck-v2/) adds a downward range sensor and an optical-flow camera, so the Crazyflie knows its height and roughly where it is. That position estimate drifts a few percent of the distance flown.

`src/crazyflie/explorer.ts` adapts the planner (`src/sim/sensorPlanner.ts`, unchanged) to four beams:

- **Mapping uses only what was measured.** Each step the four beams are written into the seen-free and occupancy maps at the pose they were taken from. The flight starts with a slow 360° turn, so the four beams sweep a full circle before the drone moves.
- **The braking filter gets a full ring.** The four beam directions use this step's readings. The other 28 directions are ray-cast through the drone's own map of walls it has already seen.
- **Turn, then fly.** Horizontal speed fades to zero while the nose is more than 30° off the direction of travel, so a measured beam always looks where the drone is going.
- **Sweep the nose.** While cruising, the nose swings ±29° (0.5 rad) at 0.8 Hz. The front beam then fans across corners just beside the path, which a single beam would miss. On the 20 dev arenas this took corner strikes from 1 or 2 per 20 flights to none.
- **The room is the geofence.** The arena's outer box is blocked in the map, so no planned path leaves it. Inner walls and boxes are not given to the planner. It has to find them.

Speed is 0.35 m/s, the cruise height is 0.5 m, and the stopping model assumes 0.3 s of reaction (20 Hz telemetry, the radio round trip, and one planner step) and 1.5 m/s² of braking (`src/crazyflie/profile.ts`).

## Simulation results

`npm run crazyflie:bench` flies 60 generated arenas (seeds 80001-80060). An arena is 3 to 6 m by 3 to 5 m, half of them have a dividing wall with a 0.9 to 1.2 m doorway, and each has one to four boxes taller than the flight height. Two numbers were picked on dev arenas 1-20 and never tuned on this block: the braking margin (0.15 m) and the nose sweep (0.5 rad).

The simulated Crazyflie (`src/crazyflie/sim.ts`) flies the stock velocity loop as a lagged, acceleration-limited response with 0.1 s of command latency. Its range beams have 2 cm noise and a 1% dropout. In the drift arms the flow-deck estimate has a scale error of about 4% per axis and a heading misalignment, and both stay fixed for the flight. A flight ends when the explorer has no frontier left, or after 240 s.

<!-- crazyflie-bench:start -->
| Arm | Coverage, mean | Coverage, worst 10% | Crashes | Closest pass under 5 cm | Flight time, mean (s) | Position error at the end (m) |
| --- | --- | --- | --- | --- | --- | --- |
| Multi-ranger (4 beams), flow drift | 90.9% | 64.2% | 0 / 60 | 4 / 60 | 111 | 0.11 |
| Multi-ranger (4 beams), perfect position | 96.1% | 89.4% | 0 / 60 | 4 / 60 | 112 | 0 |
| 32-ray ring (the sim drone), flow drift | 96.4% | 91.9% | 0 / 60 | 0 / 60 | 80 | 0.11 |
<!-- crazyflie-bench:end -->

**Coverage** is the share of floor the drone can reach that came within 1 m of its true track, in line of sight. A **crash** is the 7 cm prop disc touching a wall or box. **Closest pass** is the closest that disc came to anything.

What the table says:

- Four beams cost little when the position is perfect: 96.1% against 96.4%. They take about 40% longer, because the drone has to turn to look.
- Drift is the bigger cost. With a 4% flow error, the map's walls stop lining up with themselves. In the worst flights the explorer then decides the room is done while a corner is still unseen (worst 10% coverage drops to 64%). The fix on hardware is good floor texture and short flights. A fix in software would be re-localizing against the map's own walls with `src/loc`'s scan matcher. That is future work.
- No flight crashed. Four in 60 passed within 3 to 4 cm of something. In each one the drone was squeezing into a gap of about 0.4 m between a box and a wall. The flight setup below closes such gaps.

## What to buy

Bitcraze sells the exact kit as the **STEM ranging bundle**: Crazyflie 2.1+, Flow deck v2, Multi-ranger deck and Crazyradio 2.0. [Bitcraze store](https://store.bitcraze.io/products/stem-ranging-bundle). Prices change; these are the retail listings found on 2026-11-02.

| Item | Why | Price seen |
| --- | --- | --- |
| Crazyflie 2.1+ (brushed) | The drone. Pick the brushed 2.1+ over the Brushless: it weighs less and carries less energy into a wall, which matters more for a first flight than its shorter flight time. | in the bundle |
| Flow deck v2 | Height and position. Without it the Crazyflie cannot hold a position indoors. | ~$63 alone ([RobotShop](https://robotshop.com/products/bitcraze-crazyflie-2x-flow-deck-v2)) |
| Multi-ranger deck | The four horizontal beams the explorer maps with. | ~£83 alone ([RobotShop UK](https://uk.robotshop.com/products/bitcraze-crazyflie-2x-multi-ranger-deck)) |
| Crazyradio 2.0 | The USB radio the laptop talks through. | in the bundle |
| 2 to 3 spare 250 mAh batteries and a 1S charger | Each battery is good for about 5 minutes with both decks. | small |
| Spare propellers and motor mounts | They break. | small |
| Prop guards (optional) | Lower the cost of a touch; check the take-off weight stays under the 42 g limit with both decks. | small |
| Masking tape and a tape measure | To lay out and measure the arena. | |

For reference: RobotShop lists the Brushless version of the STEM ranging bundle at [$673](https://www.robotshop.com/products/bitcraze-stem-ranging-bundle-crazyflie-21-brushless), and the Brushless Crazyflie alone at [$480](https://www.robotshop.com/products/bitcraze-crazyflie-21-brushless). The software runs on any laptop with Node 22, Python 3.10+ and a free USB port.

## The flight space

- **Indoors, one room, nobody else in it.** No people, pets or anything fragile inside the arena. Close the door.
- **Floor with texture.** The flow sensor needs a visible pattern: a rug, wood grain or tape crosses on a plain floor. A glossy, plain floor makes the position drift fast, and the guard lands when the estimate jumps.
- **Good, even light.** No sunlight patches on the floor.
- **Walls and boxes the beams can see.** Cardboard boxes at least 0.6 m tall make good obstacles. Avoid glass, mirrors, chair legs, and anything thinner than about 5 cm (the beams pass beside them).
- **No narrow gaps.** Push a box against the wall, or leave at least 0.8 m. This is the case the bench's four close passes came from.
- **Measure it.** Write the room as a JSON file like [`scripts/crazyflie/arenas/example-room.json`](../scripts/crazyflie/arenas/example-room.json): the width `w` and depth `d` from one corner, inner walls, boxes, and the take-off spot and heading (radians, counter-clockwise from the `w` wall). Check it with `npm run crazyflie:arena -- my-room.json`. The take-off spot must be at least 0.4 m from everything.

Local rules: a 33 g drone flown indoors in your own home is outside most countries' outdoor drone rules, but check yours.

## Safety guard

Every command goes through `scripts/crazyflie/guard.py` before it reaches the radio. The guard trusts only the drone's own telemetry and the arena box, never the planner. Tests in `scripts/crazyflie/tests/test_guard.py` cover each rule.

| Rule | Limit | Action |
| --- | --- | --- |
| Horizontal speed | 0.5 m/s | clamp |
| Yaw rate | 90°/s | clamp |
| Height setpoint | 0.25 to 1.0 m | clamp |
| Within 0.3 m of the arena edge | no velocity outward | clamp |
| Any ranger under 0.2 m | no velocity toward it | clamp |
| Planner silent | 0.5 s | hover |
| Planner silent | 2 s | land |
| Telemetry lost | 0.5 s | land (the firmware cuts the motors itself after 2 s without commands) |
| Position estimate jumps | over 0.5 m in one step | land |
| Outside the arena | over 0.15 m | land |
| Battery under load | under 3.4 V for 1 s | land |
| Flight time | the mode's limit + 30 s | land |
| Measured height | over 1.4 m | land |
| Tilt | over 30° | land |
| Tilt | over 60° (a flip) | cut motors |
| Measured height | over 2.0 m (a fly-away) | cut motors |
| Ctrl-C | first press | land |
| Ctrl-C | second press | cut motors |

Before take-off, `fly.py` refuses to fly if a deck is missing, the battery is under 3.9 V at rest, the height reads over 0.15 m on the ground, any beam reads under 0.35 m, or the estimator did not reset to the take-off point.

## Flight ladder

Set up once on the laptop:

```sh
git clone https://github.com/alexnieves-cs/frontier-fly && cd frontier-fly
npm ci
python3 -m venv .venv && . .venv/bin/activate
pip install -r scripts/crazyflie/requirements.txt
```

Update the Crazyflie firmware to the latest release with the [Bitcraze client](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/) (`cfclient`), and note the radio URI it shows (for example `radio://0/80/2M/E7E7E7E7E7`). On Linux, install the Crazyradio udev rules from the [Bitcraze docs](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/).

Each step writes a flight log to `flights/`. Do one step per battery. If a step fails, stop and send the log.

| Step | Command (add `--uri <yours>`) | Props | Passes when |
| --- | --- | --- | --- |
| 0. Rehearse | `python scripts/crazyflie/fly.py --arena my-room.json --mode explore --sim` | no drone | `"outcome": "complete"` and `"crashed": false` in your room's file |
| 1. Check | `... --mode check` | **off** | both decks found; the rangers read what a tape measure says (±5 cm); turning the drone by hand turns the yaw; pushing it by hand moves x and y the right way |
| 2. Hover | `... --mode hover` | on | it takes off to 0.4 m, drifts under 20 cm in 10 s, and lands |
| 3. Yaw | `... --mode yaw` | on | it turns left 90°, and `"yaw_ok": true`. The script lands at once if the estimate turned the wrong way. |
| 4. Square | `... --mode square` | on | a 0.6 m square at 0.2 m/s, landing within 15 cm of the take-off spot |
| 5. Explore, empty room | `... --mode explore` in a room with no boxes | on | it spins, maps the room, lands on its own, and never comes within 0.2 m of a wall |
| 6. Explore with obstacles | `... --mode explore` with two boxes, then two rooms with a doorway | on | as step 5, with no touches |

Run steps 2 to 6 standing outside the arena, with a hand on Ctrl-C. Keep the battery above 3.7 V at rest between flights.

## After a flight

Send the `flights/*.jsonl` logs. Each line has the telemetry, the planner's command and the guard's verdict. With them the next step in the cloud is a report that compares the real flight with the same arena in the simulator: the track, the coverage, the closest passes and the guard events.

## Files

| Path | What it is |
| --- | --- |
| `src/crazyflie/profile.ts` | airframe, deck and flight constants |
| `src/crazyflie/arena.ts` | arena format, ray casting, generator, checks |
| `src/crazyflie/explorer.ts` | the four-beam adapter around `SensorPlanner` |
| `src/crazyflie/sim.ts` | the room-scale simulated Crazyflie |
| `scripts/crazyflie/bench.ts` | the 60-arena benchmark (`docs/crazyflie-bench.json`) |
| `scripts/crazyflie/planner.ts` | the explorer as a JSON-lines process |
| `scripts/crazyflie/fly.py` | the flight script: preflight, ladder modes, take-off, landing |
| `scripts/crazyflie/guard.py` | the safety guard |
| `scripts/crazyflie/links.py` | the cflib link and the simulated link |
| `scripts/crazyflie/tests/` | 29 pytest tests: guard rules, end-to-end sim flights, faults, the cflib link on a stand-in Crazyflie |

## Not done yet

- **No hardware flight.** The cflib link is tested against a stand-in Crazyflie object, not a radio.
- **No firmware in the loop.** The simulator models the stock velocity loop; it does not run Crazyflie firmware. [CrazySim](https://github.com/gtfactslab/CrazySim) runs the real firmware with Gazebo and would be the next step if the hardware flights disagree with the sim.
- **No re-localization.** The map drifts with the flow estimate. That is what costs the coverage in the worst flights.
- **One floor, no target search.** This is the explorer only, not the full search-and-rescue mission.
