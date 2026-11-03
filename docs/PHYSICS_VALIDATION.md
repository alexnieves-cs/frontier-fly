# Physics validation against PX4

frontier-fly says its quadrotor has realistic physics. This page checks that claim against PX4, the open-source autopilot most research drones fly, and against published rotor measurements where PX4's simulators have nothing to compare with.

The same nine maneuvers are flown three ways, on one airframe:

| Flown by | Controller | Physics |
| --- | --- | --- |
| **PX4 on SIH** | PX4 v1.15.4's position, velocity, attitude and rate controllers | PX4's own built-in simulator (SIH), with its mass, inertia, thrust, torque, arm length, drag and motor lag set to frontier-fly's airframe |
| **PX4 on frontier-fly physics** | the same PX4 controllers | frontier-fly's `QuadrotorDynamics`, the rigid body the missions and the policy fly, through the lockstep bridge (`scripts/px4/sim-node.ts --plant frontier`) |
| **frontier-fly controller** | frontier-fly's own angle-mode controller, set up as the mission quad flies outdoors | the same `QuadrotorDynamics` |

The first two rows share a controller, so any gap between them is a gap between the two physics models. The last two share the physics, so any gap there is a gap between the two flight controllers.

The numbers on this page were measured on the earlier 1.2 kg mission quad (10 inch props, thrust-to-weight 2.47, 4S 5000 mAh pack). The missions now fly one smaller airframe (`src/shared/airframe.ts`, 0.45 kg ducted quad on 3 inch props, 4S 2420 mAh), and `missionQuadParams()` returns it; this page is due a rerun on it.

Everything runs in the cloud: PX4 SITL builds in CI (`.github/workflows/px4.yml`, job `physics`), and the numbers below came from the same scripts in the cloud container.

## What it found

**1. The physics agree with PX4's own simulator.** Under the same PX4 controllers, frontier-fly's rigid body and PX4's SIH (given the same mass, inertia, thrust and drag) fly the forward step within 8% of each other: 3.63 against 3.37 m/s after 6 s, 4.7 against 4.4 m of stopping distance, and the same 30.5° peak lean. They climb at 1.48 and 1.42 m/s. Most of the gap is drag: SIH has a single linear drag term, fitted here over 0 to 5 m/s, where frontier-fly's drag is linear plus quadratic. (SIH's estimator drifts vertically by about 0.15 m/s after the climb while PX4 believes it is holding, so SIH's stop time and drift in the climb table measure PX4's estimator on SIH's sensors, not the physics.)

**2. frontier-fly's built-in flight controller is far stiffer than PX4's.** On the same physics, PX4 with its default gains takes 5 s to reach 3.6 m/s and never quite reaches 4 m/s in 6 s; it leans 30° and stops gently over 4.7 m. frontier-fly's controller gets there in 0.46 s by leaning 54°, overshoots by 13%, and stops in 1.1 m with a 1.1 m/s swing backwards. Holding position in 4 m/s wind with 1.5 m/s gusts, frontier-fly's controller wanders 0.04 m RMS; PX4 wanders 0.65 m RMS, and 0.66 m in its own estimate, so the gap is control, not estimation. The missions, the crash rule and the braking model are all measured with the stiff controller, so they are optimistic about stopping and about wind compared with a drone that flies on PX4's defaults. A PX4-like controller profile (or randomizing the controller's gains) during training and in the benchmarks is the obvious next step.

**3. A sagging pack costs thrust the shipped model never loses.** The shipped model keeps a thrust-to-weight of 2.47 on any charge. With the pack model it is 2.27 on a full pack under load and 1.80 at 8%. The mission quad has the margin to fly the full-power dash on any charge, but at 8% frontier-fly's controller holds its motors at the limit for 20% of the dash (none with the shipped model) and climbs 0.35 m less. PX4 is not affected here because its gentler velocity loop never asks for that much thrust.

**4. PX4's battery gauge would misread this pack unless it is told the capacity.** With PX4's defaults (no capacity set, cells linear between 3.6 and 4.05 V) the gauge reads 22% when 15% is left, so its low-battery warning comes 1.5 minutes late, and it reads 0% at 7%, where PX4 would already have forced a landing. With the capacity set, PX4 counts current and reads 15% at 15% and 5% at 7%. Both settings read 43% at half charge, because this pack's voltage curve is flatter in the middle than PX4's straight line. frontier-fly's own gauge is the true charge, so its return-home decisions are as good as a well-configured PX4's and better than a default one's.

**5. The shipped ground effect is about ten times too weak, and the ceiling effect is missing.** One rotor radius (0.13 m) above the ground, Cheeseman and Bennett's model, which Sanchez-Cuevas et al. measured on small multirotor rotors, gives 6.7% more thrust; the shipped model gives 2.2%, and it uses world height rather than height above the ground, so it vanishes over terrain and on upper floors. Both controllers handle a hover at 0.12 m with the measured model (3 to 4 cm of height error). The ceiling is different: 0.12 m below a ceiling the measured effect adds about 9% thrust and grows the closer the rotors get, and both PX4 and frontier-fly's controller are pulled into the ceiling, after 2.4 and 3.3 s. The shipped model holds that hover perfectly. Indoors this is a real hazard the sim does not show today.

## Results

<!-- physics-results:start -->
### Step response

| Flown by | Time to 3.6 m/s (s) | Overshoot (%) | Speed after 6 s (m/s) | Peak tilt (°) | Stop within 0.2 m/s (s) | Stop distance (m) | Reverse speed (m/s) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| PX4 on SIH (PX4's own physics, matched airframe) | – | 0 | 3.37 | 30.5 | 7.42 | 4.38 | 0 |
| PX4 on frontier-fly physics | 5.03 | 0 | 3.63 | 30.5 | 5.8 | 4.68 | 0 |
| frontier-fly controller on frontier-fly physics | 0.46 | 12.7 | 3.98 | 54.4 | 1.8 | 1.09 | 1.14 |

![Forward step](media/physics/step-forward.svg)

| Flown by | Time to 1.35 m/s (s) | Overshoot (%) | Climb rate (m/s) | Stop within 0.15 m/s (s) | Drift (m) |
| --- | --- | --- | --- | --- | --- |
| PX4 on SIH (PX4's own physics, matched airframe) | 1.16 | 0 | 1.42 | 5.99 | 0.64 |
| PX4 on frontier-fly physics | 0.56 | 0 | 1.48 | 0.52 | 0.01 |
| frontier-fly controller on frontier-fly physics | 0.3 | 0 | 1.48 | 0.26 | 0 |

![Climb step](media/physics/step-climb.svg)

### Hover in gusts

Light wind (2 m/s, 0.75 m/s gusts):

| Flown by | Offset of the mean (m) | Wander RMS (m) | Wander RMS in PX4's own estimate (m) | Max wander (m) | Peak tilt (°) |
| --- | --- | --- | --- | --- | --- |
| PX4 on frontier-fly physics | 0.02 | 0.21 | 0.22 | 0.43 | 11 |
| frontier-fly controller on frontier-fly physics | 0 | 0.01 | – | 0.03 | 12.7 |

Moderate wind (4 m/s, 1.5 m/s gusts):

| Flown by | Offset of the mean (m) | Wander RMS (m) | Wander RMS in PX4's own estimate (m) | Max wander (m) | Peak tilt (°) |
| --- | --- | --- | --- | --- | --- |
| PX4 on frontier-fly physics | 0.06 | 0.65 | 0.66 | 1.4 | 35.6 |
| frontier-fly controller on frontier-fly physics | 0 | 0.04 | – | 0.08 | 41.1 |

![Gust hold](media/physics/gust-moderate.svg)

### Battery sag

Full-power dash on a 100% pack: 5 m/s north while climbing at 1.5 m/s:

| Flown by | Speed (m/s) | Climb (m/s) | Climbed in 4 s (m) | Motors saturated (% of dash) | Peak tilt (°) |
| --- | --- | --- | --- | --- | --- |
| PX4 on frontier-fly physics | 4.29 | 1.48 | 5.48 | 1 | 34.1 |
| frontier-fly controller on frontier-fly physics | 4.94 | 1.5 | 5.56 | 2 | 54.4 |
| frontier-fly controller, shipped physics (no sag, linear ground effect) | 4.94 | 1.5 | 5.56 | 0 | 54.4 |

Full-power dash on a 30% pack: 5 m/s north while climbing at 1.5 m/s:

| Flown by | Speed (m/s) | Climb (m/s) | Climbed in 4 s (m) | Motors saturated (% of dash) | Peak tilt (°) |
| --- | --- | --- | --- | --- | --- |
| PX4 on frontier-fly physics | 4.29 | 1.48 | 5.52 | 5 | 34.6 |
| frontier-fly controller on frontier-fly physics | 4.93 | 1.51 | 5.38 | 16 | 56.8 |
| frontier-fly controller, shipped physics (no sag, linear ground effect) | 4.94 | 1.5 | 5.56 | 0 | 54.4 |

Full-power dash on a 8% pack: 5 m/s north while climbing at 1.5 m/s:

| Flown by | Speed (m/s) | Climb (m/s) | Climbed in 4 s (m) | Motors saturated (% of dash) | Peak tilt (°) |
| --- | --- | --- | --- | --- | --- |
| PX4 on frontier-fly physics | 4.3 | 1.48 | 5.53 | 5 | 34.3 |
| frontier-fly controller on frontier-fly physics | 4.93 | 1.51 | 5.21 | 20 | 57.4 |
| frontier-fly controller, shipped physics (no sag, linear ground effect) | 4.94 | 1.5 | 5.56 | 0 | 54.4 |

![Sag dash](media/physics/sag-8.svg)

Thrust ceiling as the pack drains (full throttle, loaded voltage):

| Charge | Cell voltage, no load (V) | Full-throttle current (A) | Thrust-to-weight |
| --- | --- | --- | --- |
| 100% | 4.103 | 28.2 | 2.27 |
| 50% | 3.803 | 24.5 | 1.96 |
| 30% | 3.748 | 23.8 | 1.91 |
| 15% | 3.71 | 23.4 | 1.87 |
| 8% | 3.639 | 22.5 | 1.8 |
| shipped model, any charge | – | – | 2.47 |

What PX4's battery gauge would read on this pack in a hover (135.4 W, 8.4 A, 33.1 min to empty), with PX4's default cell voltages (empty 3.6 V, full 4.05 V):

| True charge | Cell voltage under load (V) | PX4, voltage only (default, no capacity set) | PX4, capacity set to 5000 mAh |
| --- | --- | --- | --- |
| 90% | 3.966 | 91% | 90% |
| 50% | 3.749 | 43% | 43% |
| 30% | 3.693 | 31% | 30% |
| 15% | 3.655 | 22% | 15% |
| 7% | 3.553 | 0% | 5% |

### Ground and ceiling effect

Thrust multiplier against the distance from the rotor to the surface, in rotor radii (R = 0.127 m):

| z / R | Shipped (linear, world height) | Ground, Cheeseman–Bennett | Ceiling, Sanchez-Cuevas fit |
| --- | --- | --- | --- |
| 0.25 | 1.029 | 1.333 | 1.817 |
| 0.5 | 1.027 | 1.333 | 1.276 |
| 0.75 | 1.024 | 1.125 | 1.145 |
| 1 | 1.022 | 1.067 | 1.091 |
| 1.5 | 1.017 | 1.029 | 1.046 |
| 2 | 1.012 | 1.016 | 1.028 |
| 3 | 1.002 | 1.007 | 1.013 |
| 4 | 1 | 1.004 | 1.008 |

| Flown by | Closest to ceiling (m) | Hit the ceiling after (s) | Overshoot toward it (m) | Settled height error (m) |
| --- | --- | --- | --- | --- |
| PX4 on frontier-fly physics | 0 | 2.37 | – | – |
| frontier-fly controller on frontier-fly physics | 0 | 3.3 | – | – |
| frontier-fly controller, shipped physics (no sag, linear ground effect) | 0.12 | – | 0 | -0.001 |

![Ceiling](media/physics/ceiling.svg)

| Flown by | Lowest (m) | Settled height error (m) | Climb-out overshoot (m) |
| --- | --- | --- | --- |
| PX4 on frontier-fly physics | 0.145 | 0.037 | -0.019 |
| frontier-fly controller on frontier-fly physics | 0.136 | 0.028 | -0.022 |
| frontier-fly controller, shipped physics (no sag, linear ground effect) | 0.126 | 0.017 | -0.017 |

![Ground](media/physics/ground.svg)

<!-- physics-results:end -->

## What changed in the physics

The shipped model is unchanged by default: no checkpoint, benchmark or golden trace moves. Two opt-in models now exist, and the validation flies with them:

- **`src/sim/dynamics/aero.ts`** sets `QuadrotorDynamics.aero`. It replaces the shipped linear ground effect (8% per meter below 0.4 m of *world* height, so it ignored terrain and floors) with Cheeseman and Bennett's rotor model in height above the surface below, adds the ceiling effect from Sanchez-Cuevas, Heredia and Ollero's measurements, and lets a pack model lower the motors' top speed.
- **`src/mission/autonomy/pack.ts`** is a LiPo pack: open-circuit voltage from Chen and Rincón-Mora (2006), 6 mΩ per cell, and the loaded voltage's effect on motor speed. It also ports PX4's battery estimator line for line, so the page can say what a real PX4 would report on this pack.

Turning them on for training and the missions changes flight outcomes, so it belongs with a policy retrain, not here.

## How it works

| File | Role |
| --- | --- |
| `scripts/physics/maneuvers.ts` | The maneuvers (NED setpoints, wind, pack charge, ceiling) and the PX4 parameters for each simulator |
| `scripts/physics/fly_px4.py` | Sets the parameters, arms in offboard mode, climbs to the start, and flies the segments on PX4's clock |
| `scripts/physics/native.ts` | The same maneuvers on frontier-fly's controller, with PX4's position-loop law and gains above it |
| `scripts/physics/run.sh` | A fresh PX4 per flight, with SIH or with the bridge |
| `scripts/physics/report.ts` | Puts every flight on one time base, measures it, and writes the tables, plots and `docs/physics-validation.json` |
| `src/px4/frontierPlant.ts` | PX4 on `QuadrotorDynamics`: frame conversion (ENU/FLU to NED/FRD), rotor geometry for PX4's control allocation, ground contact |
| `src/px4/rig.ts` | The wind table, the pack and the surfaces, shared by the bridge and the native runner so both feel the same air |
| `src/sim/dynamics/gust.ts` | The fault bench's Gauss-Markov gust model, sampled once at 10 Hz so simulators at different rates replay identical gusts |

Run it:

```bash
export PX4_BUILD=/path/to/PX4-Autopilot/build/px4_sitl_default   # v1.15.4, make px4_sitl_default
scripts/physics/run.sh test-results/physics all
npx vite-node scripts/physics/report.ts -- --in test-results/physics --write
```

## Limits

- PX4's SIH has no wind, ground or ceiling effect and no battery model. Gusts, sag and surfaces are therefore compared only between PX4 and frontier-fly's controller on frontier-fly's physics, and the surface and pack models are checked against the papers, not against PX4.
- SIH thrust is linear in the motor command and its drag is one linear coefficient; frontier-fly's thrust goes with rotor speed squared and its drag is linear plus quadratic. The SIH drag is fitted over 0 to 5 m/s.
- PX4 flies on its estimator, which sees noisy simulated sensors. frontier-fly's controller reads the true state. Some of the hover gap is estimation, not control.
- SIH's ground truth reaches the log over MAVLink with a 50 Hz time stamp, so its timings carry up to 20 ms of jitter.
- The ceiling fit comes from one 9.4 inch rotor and is scaled to this rotor by radius. The pack's internal resistance is an assumption, not a measurement.
