# frontier-fly

**A drone autopilot that searches buildings, from the treeline to the last room, entirely in your browser. Its policy is learned by imitation (behavior cloning + DAgger) from a privileged teacher, with PPO you can run in the tab.**

[![ci](https://github.com/alexnieves-cs/frontier-fly/actions/workflows/ci.yml/badge.svg)](https://github.com/alexnieves-cs/frontier-fly/actions/workflows/ci.yml)
[![pages](https://github.com/alexnieves-cs/frontier-fly/actions/workflows/pages.yml/badge.svg)](https://github.com/alexnieves-cs/frontier-fly/actions/workflows/pages.yml)
[![license: MIT](https://img.shields.io/badge/license-MIT-informational.svg)](LICENSE)

<p align="center">
  <img src="docs/media/film-hero.gif" alt="Mission film: the drone crosses the roof of the cabin and lines up on the garage gap it will fly through" width="720">
</p>

### [▶ Try it live](https://alexnieves-cs.github.io/frontier-fly/) · [Mission film](https://alexnieves-cs.github.io/frontier-fly/film.html) · [Mission console](https://alexnieves-cs.github.io/frontier-fly/demo.html) · [Indoor sim](https://alexnieves-cs.github.io/frontier-fly/sim.html) · [Mission editor](https://alexnieves-cs.github.io/frontier-fly/mission.html) · [Outdoor site](https://alexnieves-cs.github.io/frontier-fly/outdoor.html) · [Benchmarks](docs/BENCHMARKS.md) · [Report](docs/REPORT.md) · [Architecture](docs/ARCHITECTURE.md)

frontier-fly is a search-and-rescue drone simulator and autopilot. A quadrotor reads lidar, an IMU, and its own map of where it has been, and a small neural network decides where to fly next. That network is trained by imitation learning: behavior cloning and DAgger from a privileged frontier-exploration teacher. PPO fine-tuning is implemented and runs in the tab, but the shipped weights come from DAgger. The mission it is built for: sweep a geofenced patch of woods, find the building, pick an opening it actually fits through, search room by room for a missing person or confirm the place is clear, fly home, and file a report.

Everything runs client-side. Physics (Rapier, 400 Hz), sensors, optional PPO training (TensorFlow.js in a worker), and rendering (Three.js on WebGL2; WebGPU opt-in with `?renderer=webgpu`) all happen in the tab. There is no server and no GPU cluster. A seed rebuilds the same world exactly, so every result here can be reproduced.

The project is civilian and defensive only: search, inspection, and confirm-clear. The drone never pursues or engages anyone.

## Missions

### Mission films

Each film replays a real seeded mission, flown headless by the same autopilot, in 3D: chase, aerial, orbit, roof-cutaway and landing shots, with the planner's log, a minimap and the end report on screen. [`film.html`](https://alexnieves-cs.github.io/frontier-fly/film.html) renders them live in the browser at full quality; the MP4s below are 960 px previews recorded with `npm run media:film`.

| Film | Mission |
| --- | --- |
| [Woods to cabin (MP4, 74 s)](docs/media/film-cabin-day.mp4) · [watch live](https://alexnieves-cs.github.io/frontier-fly/film.html?scenario=cabin-day) | Find an injured hiker inside a cabin by day. |
| [Compound search (MP4, 79 s)](docs/media/film-compound-clear.mp4) · [watch live](https://alexnieves-cs.github.io/frontier-fly/film.html?scenario=compound-clear) | Prove two buildings are empty, with an operator approving each entry. |
| [Night search (MP4, 70 s)](docs/media/film-backpack-night.mp4) · [watch live](https://alexnieves-cs.github.io/frontier-fly/film.html?scenario=backpack-night) | Find a lost backpack in a dark building by flashlight. |

### Mission console clips

Three seeded scenarios, each flown autonomously from takeoff to landing. For the recordings, "ask to enter" prompts were auto-approved. The drone sweeps a geofenced forest in coverage lanes, discovers the building, inspects its openings and picks one it clears with margin, searches room by room, confirms a find or declares the building clear, flies home, and writes a report. The clips play at 4× (16× for the compound) and come from [`demo.html`](https://alexnieves-cs.github.io/frontier-fly/demo.html?scenario=cabin-day&autorun=1); results are in the [mission table](#benchmarks). Indoors the shipped policy flies first, and on a stall or its first wall contact the run hands the visit to frontier exploration. The report says which explorer ran.

| <img src="docs/media/mission-cabin-day.gif" alt="Injured hiker in a cabin" width="270"><br>**Find a person.** Daylight, one cabin, auto entry. [MP4](docs/media/mission-cabin-day.mp4) | <img src="docs/media/mission-compound-clear.gif" alt="Confirm a compound is clear" width="270"><br>**Confirm clear.** Two buildings, nobody inside. [MP4](docs/media/mission-compound-clear.mp4) | <img src="docs/media/mission-backpack-night.gif" alt="Lost backpack at night" width="270"><br>**Night search.** Find a backpack in the dark. [MP4](docs/media/mission-backpack-night.mp4) |
| --- | --- | --- |

<details>
<summary>demo.html options</summary>

| Query | Effect |
| --- | --- |
| `scenario` | `cabin-day`, `compound-clear`, or `backpack-night` (seeded in `src/mission/autonomy/scenarios.ts`) |
| `speed` | `1`, `4`, `16`, or `max` |
| `autorun=1` | Start when the page loads |
| `operator=approve` | Approve entry prompts automatically |
| `indoor=frontier` | Skip the shipped policy and use frontier exploration indoors |

</details>

## Features

| | |
| --- | --- |
| <img src="docs/media/training.gif" alt="Live training dashboard" width="420"><br>**Train it in the tab.** Press Train and up to four worker threads roll out episodes into `SharedArrayBuffer`s while a learner worker runs PPO. Return, coverage, entropy, and a policy heatmap update live. [MP4](docs/media/training.mp4) | <img src="docs/media/fpv-night.gif" alt="FPV night flight" width="420"><br>**Fly by flashlight.** The policy never sees pixels, only lidar, IMU, and an egocentric occupancy map, so it flies the same with the lights off. FPV adds the HUD and simulated detections. [MP4](docs/media/fpv-night.mp4) |
| <img src="docs/media/mission-editor.gif" alt="Mission editor, live preview, and report" width="420"><br>**Plan a mission.** Draw a geofence and no-fly zones, set an altitude band, targets, required confidence, an entry policy (auto, ask, never), and budgets. The plan validates against a versioned JSON schema and shares as a link. [MP4](docs/media/mission-editor.mp4) | <img src="docs/media/outdoor-site.png" alt="Seeded outdoor site with a building shell" width="420"><br>**A site, not a room.** Seeded terrain, instanced trees with colliders, and one to four building shells with typed openings: open and broken windows, ajar doors, vents, garage gaps, roof hatches. Only openings that clear the airframe are enterable. |
| <img src="docs/media/targets-fpv.png" alt="Simulated people and detections" width="420"><br>**Something to find.** Simulated people standing, sitting, lying, walking, and hiding, a library of search objects, and decoys. A geometric camera detector misses with distance, occlusion, darkness, and motion blur, and false-alarms on decoys. | <img src="docs/media/day.png" alt="Day chase camera inside a generated building" width="420"><br>**Procedural buildings.** BSP rooms, doors, hallways, multi-floor shafts, and a reachability check. PBR interiors with CC0 textures, an HDRI sky, SSAO, bloom, and SMAA on the high preset. |

## Benchmarks

These tables are generated by `npm run bench && npm run docs` from [`docs/benchmarks.json`](docs/benchmarks.json). Misses are shown as misses. Method, per-seed results, and render frame rates are in [docs/BENCHMARKS.md](docs/BENCHMARKS.md).

<!-- generated:benchmarks:start -->
| Goal | Target | Measured (browser pilot) | Status |
| --- | --- | --- | --- |
| Coverage, easy buildings | ≥ 80.0% | 95.1% mean of 4 | ✅ meets |
| Coverage, new hard buildings | ≥ 60.0% | 67.3% mean of 3 | ✅ meets |
| Night coverage vs day | ≥ 90.0% | 100.0% | ✅ meets |
| Kinematic env step | ≤ 0.3 ms | 0.133 ms | ✅ meets |
| Env steps/s, 4 workers, physics + sensors | ≥ 10,000 | 16,305 | ✅ meets |
| Env steps/s, 4 workers, with inference and resets | ≥ 10,000 | 13,040 | ✅ meets |

| Pilot | Easy day | Easy night | Night / day | Hard day | Hard night | Night / day |
| --- | --- | --- | --- | --- | --- | --- |
| Shipped policy, browser pilot | 95.1% | 95.1% | 100.0% | 67.3% | 67.4% | 100.1% |
| Shipped policy, raw observations | 95.1% | 95.1% | 100.0% | 67.3% | 67.4% | 100.1% |
| Frontier teacher (privileged, upper reference) | 95.1% | 95.1% | 100.0% | 73.3% | 73.3% | 100.0% |
| Waypoint follower (same observation, no learning) | 53.5% | 53.5% | 100.0% | 35.5% | 35.5% | 100.0% |
| Lidar heuristic (baseline) | 29.3% | 29.3% | 100.0% | 13.9% | 16.5% | 118.8% |

| Measurement | Value |
| --- | --- |
| Kinematic env step (1 thread) | 0.133 ms · 7,497 steps/s |
| Quadrotor env step, 400 Hz physics (1 thread) | 0.239 ms · 4,191 steps/s |
| Policy inference, SIMD gemm, batch 8 | 0.076 ms per env |
| 4 workers, physics + sensors (lidar heuristic pilot) | 16,305 env steps/s |
| 4 workers × 8 envs, browser rollout loop: batched policy inference, resets on pooled worlds | 13,040 env steps/s |
<!-- generated:benchmarks:end -->

**Missions.**

<!-- generated:missions:start -->
| Scenario | Outcome | Mission time | Time to find | Outdoor coverage | Indoor coverage | Entries | False clears | False alarms | Missed | Collisions | Battery used | Landed home |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Woods to cabin: find the missing person | found | 3:12 | 2:24 | 29.5% | 100.0% | 1 | 0 | 0 | 0 | 0 | 11.7% | yes |
| Compound search: confirm the area clear | clear | 8:23 | — | 100.0% | 100.0% | 2 | 0 | 0 | 0 | 1 | 30.2% | yes |
| Night search: find the lost backpack | found | 4:12 | 3:31 | 28.6% | 100.0% | 1 | 0 | 0 | 0 | 1 | 15.3% | yes |

Recorded from `main@f5e22fb` by `npm run media`, which reads each run's end-of-mission report from the page. Times are simulated mission time.
<!-- generated:missions:end -->

## How it works

```mermaid
flowchart LR
  Seed(["seed"]) --> Gen["Procedural site<br/>terrain · trees · buildings"]
  Gen --> Sim["Rapier physics<br/>kinematic or 6-DoF quad"]
  Sim --> Obs["1088-float observation<br/>lidar · IMU · frontier · maps"]
  Obs --> Pol["Policy MLP<br/>1088 → 512 → 256 → 4"]
  Pol -- "vx, vy, vz, yaw rate" --> Sim
  Teacher["Privileged frontier teacher<br/>sees the explored map"] -- "labels" --> IL["BC + DAgger<br/>headless, npm run train:indoor"]
  IL -- "shipped weights" --> Pol
  Sim -. "rollouts (SharedArrayBuffer)" .-> PPO["Optional PPO in the tab<br/>TF.js worker, Train button"]
  PPO -. "session weights" .-> Pol
  Sim --> Det["Camera detector<br/>people · objects · decoys"]
  Det --> Mission["Mission layer<br/>geofence · entry policy · report"]
  Sim --> View["Three.js WebGL2<br/>FPV · chase · overview"]
```

**Training.** The shipped checkpoint starts from behavior cloning: a privileged frontier-seeking teacher that can see the explored map flies the curriculum, and the network learns to imitate it from its own sensors. DAgger then lets the network fly while the teacher labels every state it reaches, and the round kept is the one that scores best on held-out validation seeds. PPO with a KL anchor to the cloned policy is implemented, but no PPO fine-tune beat DAgger on validation, so the shipped file has none. In the browser, **Train** runs PPO on the spec reward (new coverage, new rooms and floors, collisions, smoothness, time). A six-stage curriculum grows the buildings from one 8×8 m room to multi-floor layouts with quadrotor dynamics, and randomizes mass, thrust, drag, motor lag, sensor noise, wind, and lighting so the policy cannot overfit one world. The full loop runs in the browser, and the same code runs headless in Node for `npm run train:indoor` and the benchmarks.

**Why it works at night.** The observation has no camera pixels. Lidar and the occupancy map do not care about light, so night coverage is measured against day coverage as a check on that design, not a separate model.

More detail, including the generated module graph and the headless boundary between sim and renderer, is in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Tech stack

| Layer | What | Why |
| --- | --- | --- |
| Language and build | TypeScript 5.7, Vite 6 | One codebase for the browser, workers, and Node |
| Physics | [Rapier](https://rapier.rs) 0.14 (WASM) | Deterministic colliders and fast ray casts for lidar |
| Learning | [TensorFlow.js](https://www.tensorflow.org/js) 4.22, hand-written SIMD WASM gemm | PPO updates on WebGL or CPU, cheap batched inference in env workers |
| Rendering | [Three.js](https://threejs.org) 0.170, WebGL2 (WebGPU opt-in) | PBR, HDRI, shadows, and post passes; `?renderer=webgpu` tries the lighter WebGPU path |
| Charts | [uPlot](https://github.com/leeoniya/uPlot) | Lightweight live training curves |
| Concurrency | Web Workers, `SharedArrayBuffer` | Rollouts without copying |
| Tests | Vitest, Playwright, ESLint | Unit, perf, and headless browser checks in CI |
| Hosting | GitHub Pages + [coi-serviceworker](https://github.com/gzuidhof/coi-serviceworker) | Static hosting with cross-origin isolation |

## Run locally

Needs Node 22.

```bash
npm ci
npm run dev        # http://127.0.0.1:3000 with COOP/COEP headers
npm test           # unit and perf tests
npm run build      # typecheck and production bundle in dist/
```

| Script | What it does |
| --- | --- |
| `npm run test:e2e` | Headless Chrome checks and screenshots |
| `npm run bench` | Coverage and throughput, written to `docs/benchmarks.json` |
| `npm run media` | Record the demo clips into `docs/media/` (needs ffmpeg) |
| `npm run study:coverage` / `npm run study:missions` | Every pilot on fresh buildings, and three indoor explorers on fresh mission worlds, with 95% intervals |
| `npm run docs` | Regenerate `docs/ARCHITECTURE.md`, `docs/BENCHMARKS.md`, `docs/REPORT.md`, and the README tables |
| `npm run check:pages` | Serve `dist/` the way GitHub Pages does and check every page loads |
| `npm run eval:coverage` | Coverage table vs targets for the shipped checkpoint |
| `npm run train:indoor -- --out ckpt.json` | BC + DAgger on every core, kept by held-out validation |

<details>
<summary>Controls and URL parameters</summary>

| Input | Action |
| --- | --- |
| W A S D | Fly forward / left / back / right |
| Q E or arrow left / right | Yaw |
| Space / C | Climb / descend |
| 1 2 3 | FPV, chase, overview |
| F | Flashlight |
| L | Interior lights |
| P | Pause the sim |
| T | Start or pause training |
| Drag in overview | Orbit the cutaway camera |

| Query | Effect |
| --- | --- |
| `seed` | Building and episode seed |
| `shot` | `day`, `dusk`, `night`, `fpv`, `targets`, `targets-room`, or `targets-night` |
| `camera` | `fpv`, `chase`, `overview` |
| `stage` | `0`–`5` or `auto` |
| `train=1` | Start PPO when the page loads |
| `quality` | `low` or `high` |
| `pilot` | `keys`, `heuristic`, or `policy` |
| `tod`, `flash`, `lights` | Time of day (0–24), flashlight `1`, interior lights `0` |

These apply to the indoor sim, [`sim.html`](https://alexnieves-cs.github.io/frontier-fly/sim.html). Old links that put them on the site root are forwarded there. Example: [`sim.html?seed=11&stage=0&camera=fpv&tod=22&flash=1&lights=0&pilot=policy`](https://alexnieves-cs.github.io/frontier-fly/sim.html?seed=11&stage=0&camera=fpv&tod=22&flash=1&lights=0&pilot=policy). Pilot defaults to the keyboard. Shot URLs fly a lidar heuristic so the camera has something to follow. Choose **Policy** to fly the checkpoint.

</details>

## Honest limits

- The shipped policy meets the coverage goals on the fixed eval, but that eval is small: 4 easy and 3 hard seeds. On 48 held-out validation episodes the current checkpoint (a weight average of DAgger rounds 3 and 4) averages 62.2% on hard buildings against 60.8% for the previous one, so expect the hard-building number to move by several points on other seeds. It reaches about 92% of the privileged teacher it imitates. [BENCHMARKS.md](docs/BENCHMARKS.md) has the per-seed numbers.
- Indoors, missions no longer read the floor plan: a sensor-only planner maps from the drone's own lidar and range finders, and the policy flies under a lidar guard with that planner as its fallback. On 90 fresh mission worlds per explorer the sensor-only explorer reaches the expected outcome 93.3% of the time, the policy-first explorer 91.1%, and the old floor-plan planner (kept as a reference) 88.9%, with no collisions in any of the 270 flights. The policy still hands 46% of the buildings it enters to the planner after a stall, and on fresh coverage buildings a classical sensor-only planner covers more than the policy. [REPORT.md](docs/REPORT.md) has the study, the ablations and the failure cases.
- Mission clips show the 2D mission map. Clips in `docs/media/` were recorded in a cloud container with software WebGL, which renders at a few frames per second. FPV and training clips play one captured frame per 50 ms sim step, so flight runs at true speed and the training charts run faster than real time. On a laptop GPU the page runs in real time.
- Door leaves are visual. Furniture collides, but doors are always open in the indoor grid.
- The detector is geometric, not a trained vision model.

## Roadmap

- **Harder missions**: more buildings, closed doors, moving people, and wind on the outdoor leg. The three scenarios above are the first end-to-end set.
- **Generalization**: at least 80% coverage on easy buildings and 60% on unseen hard ones, with night at 90% or more of day.
- **Swarm search**: several drones sharing one site, splitting the area, and handing off rooms. Needs a shared world and an agent axis in the rollout layout.
- **Defensive evasion**: the drone moves out of the way of people and moving obstacles. Dodge only. It never pursues, tracks for harm, or strikes.

## Contributing

Issues and pull requests are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) for setup, checks, and the project's civilian-use scope.

## Credits and license

Code is [MIT](LICENSE). Every third-party asset is CC0 or MIT and listed with its source in [CREDITS.md](CREDITS.md): Poly Haven sky, ambientCG textures, the KayKit mannequin and animations, three.js post passes, and coi-serviceworker.
