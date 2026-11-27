# frontier-fly

**FrontierBench: a benchmark for drones that explore buildings they have never seen, and the browser simulator it runs on. Five published exploration methods, a random walk and the house planners fly the same test seeds on eight tracks, and every number below is regenerated from the bench output on each build.**

[![ci](https://github.com/alexnieves-cs/frontier-fly/actions/workflows/ci.yml/badge.svg)](https://github.com/alexnieves-cs/frontier-fly/actions/workflows/ci.yml)
[![pages](https://github.com/alexnieves-cs/frontier-fly/actions/workflows/pages.yml/badge.svg)](https://github.com/alexnieves-cs/frontier-fly/actions/workflows/pages.yml)
[![release](https://img.shields.io/github/v/release/alexnieves-cs/frontier-fly?sort=semver)](https://github.com/alexnieves-cs/frontier-fly/releases/latest)
[![license: MIT](https://img.shields.io/badge/license-MIT-informational.svg)](LICENSE)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23717793.svg)](https://doi.org/10.5281/zenodo.23717793)

<p align="center">
  <img src="docs/media/film-hero.gif" alt="Mission film: the drone crosses the roof of the cabin and lines up on the garage gap it will fly through" width="720">
</p>

### [▶ Try it live](https://alexnieves-cs.github.io/frontier-fly/) · [Mission film](https://alexnieves-cs.github.io/frontier-fly/film.html) · [How it works](https://alexnieves-cs.github.io/frontier-fly/how.html) · [Safety numbers](https://alexnieves-cs.github.io/frontier-fly/atlas.html) · [Tech report (PDF)](https://alexnieves-cs.github.io/frontier-fly/frontier-fly-report.pdf) · [Project guide](docs/GUIDE.md) · [All docs](docs/README.md)

Built by **Alex Nieves** · [portfolio](https://alexnieves.me) · [GitHub](https://github.com/alexnieves-cs) · [about the project](#about-the-project)

The drone sweeps a geofenced patch of woods, finds the building, picks an opening it fits through, searches room by room for a missing person or confirms the place is clear, flies home and files a report. Physics, sensors, training and rendering all run in the tab, with no server, and a seed rebuilds the same world exactly. Civilian and defensive use only: it searches, inspects and confirms clear, and never pursues anyone.

## FrontierBench scoreboard

<!-- generated:scoreboard:start -->
| Track (test seeds) | View-first planner | Information gain | Rapid frontier | NBVP | Nearest frontier | Random walk | BA* coverage | Sensor-only planner |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **FrontierBench v3** (82001+, 30 per task × 7) | 67.6% | 13.8% | 9.7% | 27.1% | 8.8% | 47.1% | 65.4% | **68.0%** |
| **Hard** (93001+, 30 per task × 5) | **87.8%** | 68.0% | 55.0% | 65.6% | 58.4% | 52.0% | 44.6% | 39.5% |
| **Return** (97001+, 30 per task × 4) | **75.5%** | 62.0% | 50.6% | 49.7% | 53.1% | 37.8% | 32.3% | 26.0% |
| **Drift** (99001+, 30 per task × 4) | **78.7%** | 63.1% | 55.6% | 55.9% | 51.0% | 44.9% | 41.5% | 36.2% |
| **Hard drift** (111001+, 30 per task × 4) | **48.1%** | 35.4% | 32.9% | 31.4% | 34.0% | 27.5% | 23.1% | 15.0% |
| **Degraded sensors** (141001+, 30 per task × 4) | **80.0%** | 50.0% | 48.4% | 53.0% | 47.0% | 42.2% | 46.0% | 30.7% |
| **Overwatch** (151001+, 30 per task × 3) | 56.3% | **61.8%** | 54.1% | 48.2% | 55.7% | 47.2% | 45.8% | 40.8% |
| **Integrity** (161001+, 30 per task × 4) | 55.7% | 56.9% | 72.3% | 52.7% | 59.5% | 66.2% | **81.5%** | −7.9% |

Each cell is the mean over the track's tasks of the per-task mean on the test seeds, every pilot on the same flights; bold is the best in the row. V3 scores the share flown through; the frontier and next-best-view methods stop once their lidar has mapped every room, so they rank low there (judge them on the camera tracks). View-first planner: this repository; Information gain: Gonzalez-Banos & Latombe 2002; Rapid frontier: Cieslewski et al. 2017; NBVP: Bircher et al. 2016; Nearest frontier: Yamauchi 1997; Random walk: the floor, no map and no plan; BA* coverage: Viet et al. 2013; Sensor-only planner: this repository. Columns are ordered by mean rank. Intervals, crashes, the control runs and the commit of every cell are in [docs/scoreboard.json](docs/scoreboard.json) and [docs/LEADERBOARD.md](docs/LEADERBOARD.md).
<!-- generated:scoreboard:end -->

Score your own pilot on any track with `npm run bench:hard -- --pilot bench/entries/<you>/pilot.ts --track <track> --split dev`, then test once; [LEADERBOARD.md](docs/LEADERBOARD.md#submit-an-entry) has the rules, every track's full board and how to submit. The [live scoreboard](https://alexnieves-cs.github.io/frontier-fly/) shows the same numbers with intervals, and replays recorded bench flights in the browser.

## Search-and-rescue results


<!-- generated:headline:start -->
| Question | Measured | On |
| --- | --- | --- |
| Do staged missions end the right way? | 98.3% (94.1–99.5%) found the target or correctly cleared | 120 missions on the quadrotor: four fixed scenarios, calm air, no faults, any person lying still |
| **And over a realistic mix of conditions?** | **41.2%** (40.0–42.5%) end as expected, 53.4% with no fault injected. Most of the rest end *incomplete* and hand back to a person; an empty site is correctly cleared in 37.7% of sweeps | 6,000 flights drawn from the failure atlas's declared mix of missions, people who walk or hide, wind, and a fault on 40% of flights |
| **How often does it call a building clear with someone inside?** | **0.25% per flight** (0.16–0.35%); a plain draw gives 0.20% (12 of 6,000). It was 4.05% before unconfirmed people blocked a clear (0.30% after, both versions on the same 2,000 seeds) | 2,000 importance-sampled flights over the same mix |
| How often does it hit something? | 1.22% per flight (0.97–1.53%) | 6,000 flights drawn from the same mix |
| Does it end safely when something breaks? | 98.6% (97.7–99.2%) of faulted flights end safely: 395 finished the mission, 495 flew home, 64 landed where they were, 6 crashed | 960 flights with sensor, motor, battery, link, GNSS or wind faults |
| What does the learned policy add? | On the simple airframe, more coverage: 75.1% (70.5–79.4%) behind its lidar guard, 1 of 30 crashed, against 65.5% for the sensor-only planner, which touched nothing; alone it covers 67.5%, 7 of 30 crashed and 16 of 30 touched a wall (contacts slower than 0.5 m/s do not end a flight). On quadrotor dynamics it loses: the planner covers 46.9% with 0 crashes, the policy 11.9% and crashes in 30 of 30. A second network trained on the quadrotor, the [quad student](docs/QUAD_STUDENT.md), flies it end to end: 52.4% behind the braking shield with 7 of 664 crashed, against 51.4% for the hand-built planner-only pilot and 54.3% for the default quad pilot that taught it | 30 hard buildings per airframe; the quad student on 664 report-only quad flights |
| Does it need light? | No: it flies on lidar. With every room light off and its flashlight on it covers 77.0% (guarded policy; the planner 65.8%). In this simulator darkness only adds 5% lidar dropout, so this checks the design, not a harder condition | the same buildings |
<!-- generated:headline:end -->

Every number comes from report-only seed blocks that no model or setting was tuned on, with 95% intervals, and is regenerated from the study's JSON by `npm run docs`. The blocks are re-flown when the code changes, so they are held out, not unseen. Read the first two rows together: staged scenarios go well, and a realistic mix of conditions does not. The false-clear rate is the next one to read: it is why this is a research simulator and not a product. The learned policy is the one to read second: nine further training runs did not get it past the planner on the quadrotor ([QUAD_POLICY.md](docs/QUAD_POLICY.md)); a new network trained on the quadrotor itself does, by one point, behind a braking shield ([QUAD_STUDENT.md](docs/QUAD_STUDENT.md)). The [project guide](docs/GUIDE.md#honest-limits) lists the other limits.

## Run it

Everything runs in a browser tab, on desktop or phone. Pick the way that suits you:

| Way | What you get | Needs |
| --- | --- | --- |
| [Live site](https://alexnieves-cs.github.io/frontier-fly/) | Every page: the film, missions, race, drone team, safety numbers, indoor and outdoor flying, the mission planner and PX4 | Nothing, just a browser |
| Your computer | The site, training and benchmarks (`npm run train:ppo`, `npm run study:missions`, ...) | Node 22 |
| Terminal or Docker | The whole mission, FrontierBench and QGroundControl plan export with no browser: `docker run --rm ghcr.io/alexnieves-cs/frontier-fly mission`, see [CLI.md](docs/CLI.md) | Docker, or Node 22 |
| Python | A Gymnasium env (`gym.make("FrontierFly-hard-v0")`) and the FrontierBench scorer, on the same simulator, install from a checkout (not on PyPI yet), see [python/](python/README.md#install) | Python 3.10+ and Node 22 |

Also: [GitHub Codespaces](https://codespaces.new/alexnieves-cs/frontier-fly?quickstart=1), the PX4 autopilot in the loop in simulation ([PX4.md](docs/PX4.md)), and a simulated Crazyflie model ([CRAZYFLIE.md](docs/CRAZYFLIE.md)).

```bash
npm ci && npm run dev   # Node 22, then open http://127.0.0.1:3000
```

The [project guide](docs/GUIDE.md) covers the pages, every benchmark table, the studies, and how to train the policy, and the [docs index](docs/README.md) lists every write-up. [ARCHITECTURE.md](docs/ARCHITECTURE.md) maps the code, and [CONTRIBUTING.md](CONTRIBUTING.md) explains how to send changes. Code is [MIT](LICENSE). Every asset is CC0, MIT, Apache-2.0 or CC BY 4.0 and listed in [CREDITS.md](CREDITS.md).

## Project status

frontier-fly is a simulation project, by design: every flight on this page is simulated, and flying on real hardware is not planned. The simulator is at a stable point as of [v1.2.0](CHANGELOG.md): the site, the CLI and Docker image and the Python package are complete and tested in CI. New work goes into FrontierBench, under one rule: no capability lands without a scored track and a number attached ([CONTRIBUTING.md](CONTRIBUTING.md#the-benchmark-rule)). The limits above stay as measured: fewer than half of missions end as expected over a realistic mix of conditions, the false-clear rate is not zero, and the learned policy loses to the planner on the quadrotor. The built-in flight controller also stops the quadrotor in about a quarter of the distance PX4's default gains need on the same physics ([PHYSICS_VALIDATION.md](docs/PHYSICS_VALIDATION.md)), so crash and braking numbers are optimistic next to a PX4-flown model. [CONTRIBUTING.md](CONTRIBUTING.md) explains how to send changes or a benchmark entry.

## About the project

frontier-fly is built by [Alex Nieves](https://alexnieves.me) ([GitHub](https://github.com/alexnieves-cs)). It started as a browser drone demo and grew into a research-style simulator: a sensor-only planner, a failure atlas, a learned pilot measured against both, PX4 in the loop, a Python package, and a benchmark anyone can enter.

What turned out to be hard, and what the repository does about it:

- **A good average hides rare failures.** The mission bench looked fine, but the failure atlas found the drone calling a building clear with a walking or hiding person inside on about 4% of flights. Tracking people across views and letting an unconfirmed person block a clear took that to 0.3% on the same seeds. The price is that many sweeps now end incomplete instead of clear.
- **Free contacts teach bad habits.** The learned policy covered the most of any pilot, then a crash rule showed it was sliding along walls. Retraining under the rule cut the policy's own crashes from 19 of 30 flights to 7, and the lidar guard takes that to 1. The habit moved below the threshold more than it went away: alone, the retrained policy still touches a wall slowly in 16 of 30 flights, and under the guard in 5.
- **Small fixed evaluations flatter.** The early 86% was partly seed luck. Every headline now comes from held-out seed blocks with 95% intervals, and the generated docs fail CI if a number drifts from its JSON.
- **A learned pilot is not automatically the better pilot.** Behind a guard on a simple airframe the policy covers ten points more than the planner. On quadrotor dynamics it crashes in every flight, and nine training runs (planner imitation, braking labels, memory, quad-only training) closed only part of the gap ([QUAD_POLICY.md](docs/QUAD_POLICY.md)). A network trained on the quadrotor from scratch, by DAgger from the default quad pilot and then shielded RL, flies it end to end and edges past the hand-built planner, but it still trails its teacher and crashes in 15% of flights without its braking shield ([QUAD_STUDENT.md](docs/QUAD_STUDENT.md)). Missions keep the planner as the pilot of last resort.
- **Simulators lie in specific ways.** Kinematic drones stop instantly and quadrotors don't. A braking filter, a quad-aware tracker and PX4 SITL in CI keep the gap visible.

Skills it covers: reinforcement and imitation learning (behavior cloning, DAgger, PPO), quadrotor dynamics and control, lidar mapping and scan-matching localization, PX4 and MAVLink, benchmark design, TypeScript, three.js and WebGL, Python packaging, and CI on GitHub Actions. To cite it, see [CITATION.cff](CITATION.cff).
