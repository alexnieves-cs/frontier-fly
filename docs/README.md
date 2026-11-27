# Documentation

Start with the [project guide](GUIDE.md). It covers every page of the live site, the benchmark tables, how the
policy is trained and the project's honest limits. The pages below go deeper on one topic each.

## Start here

| Page | What it covers |
| --- | --- |
| [GUIDE.md](GUIDE.md) | The missions and their recordings, each page of the site, how to run and train, and the limits |
| [REPORT.md](REPORT.md) | The tech report: methods and every study, with intervals. Also a [PDF](https://alexnieves-cs.github.io/frontier-fly/frontier-fly-report.pdf) |
| [ARCHITECTURE.md](ARCHITECTURE.md) | A map of the code under `src/`, generated from its import graph |

## Results and benchmarks

| Page | What it covers |
| --- | --- |
| [BENCHMARKS.md](BENCHMARKS.md) | Coverage and throughput, and how they are measured |
| [LEADERBOARD.md](LEADERBOARD.md) | FrontierBench: the baselines, the tasks and how to submit an entry |
| [SAFETY_CASE.md](SAFETY_CASE.md) | False clears, collisions and fault recovery, with the runs behind them |
| [ATLAS.md](ATLAS.md) | The failure atlas: importance-sampled searches for rare mistakes |
| [REAL_PLANS.md](REAL_PLANS.md) | The pilots on 60 real apartment floor plans |
| [MULTI_FLOOR.md](MULTI_FLOOR.md) | Multi-floor flight on generated buildings and 60 real multi-storey homes, under the 120 s budget and on a battery |
| [QUAD_POLICY.md](QUAD_POLICY.md) | A negative result: the learned policy flying the quadrotor |
| [QUAD_PICKER.md](QUAD_PICKER.md) | A learned goal picker for the quadrotor planner |
| [VIEW_FIRST.md](VIEW_FIRST.md) | The view-first planner, which beats NBVP on 3 m footprint coverage |
| [PHYSICS_VALIDATION.md](PHYSICS_VALIDATION.md) | The quadrotor model checked against PX4 and rotor measurements |

## Tools and integrations

| Page | What it covers |
| --- | --- |
| [CLI.md](CLI.md) | The headless CLI and the Docker image |
| [PX4.md](PX4.md) | Exported missions flown by PX4 in software-in-the-loop |
| [QGC_EXPORT.md](QGC_EXPORT.md) | Exporting a planned mission to QGroundControl |
| [CRAZYFLIE.md](CRAZYFLIE.md) | The plan for a real Crazyflie flight (simulated only so far) |
| [../python/README.md](../python/README.md) | The Python package: a Gymnasium env and the FrontierBench scorer |
| [RELEASING.md](RELEASING.md) | How the site, the image, the Python package and releases ship |

## Data in this folder

The `*.json` files here are the measured results the pages above are generated from. `npm run docs` rebuilds
the generated pages and the README's results table from them, and CI fails if a page drifts from its data.

| Folder | What |
| --- | --- |
| [atlas/](atlas) | Failure atlas runs, read by `ATLAS.md` |
| [media/](media) | Screenshots, GIFs and films used by the README and the site |
| [report/](report) | Figures for the tech report |
| [history/](history) | The original spec and an early coordination log, kept for the record |

Older runs that the docs still cite are in [results/archive](../results/archive/README.md).
