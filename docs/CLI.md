# Headless CLI and Docker image

`frontier-fly` runs the simulator without a browser: the whole woods-to-building search mission, the
FrontierBench scorer, and the QGroundControl plan export. It is the same TypeScript the live site runs,
bundled into one Node script, so a seed flies the same flight in a tab, in a terminal and in a container.

## Run it

With Docker, nothing else to install (linux/amd64 and linux/arm64):

```bash
docker run --rm ghcr.io/alexnieves-cs/frontier-fly mission --scenario cabin-day
docker run --rm ghcr.io/alexnieves-cs/frontier-fly list
# Files land in /work; mount a folder there to keep them.
docker run --rm -v "$PWD:/work" --user "$(id -u):$(id -g)" ghcr.io/alexnieves-cs/frontier-fly \
  bench --pilot guarded --split test --out guarded.json
```

From a checkout, with Node 22:

```bash
npm ci
npm run cli -- mission --scenario backpack-night --pilot sensorQuad
npm run build:cli && node dist-cli/frontier-fly.mjs --help   # the bundle the image ships
docker build -t frontier-fly .                               # the image itself
```

Progress goes to stderr and results to stdout, so `frontier-fly plan > woods.plan` works. A usage error
(unknown command, option, scenario or pilot) exits with code 2 before anything flies.

## Commands

### `mission`: fly the search-and-rescue mission

Takes off, sweeps the geofenced woods, finds the building, picks an opening and enters, searches the rooms,
flies home, and prints the after-action report (outcome, coverage, false clears, collisions, battery).

| Option | Default | |
| --- | --- | --- |
| `--scenario` | `cabin-day` | `cabin-day`, `compound-clear`, `backpack-night`, or `compound-occupied` (the clear sweep with a person inside, from the mission bench) |
| `--seed` | the scenario's | World seed; the scenarios' own seeds are the demo's |
| `--pilot` | `policy` | Indoor explorer: `policy` (the trained network under its lidar guard, with the planner as fallback), `sensor` (sensor-only planner), `oracle` (planner on the true floor plan, a reference), the `Quad` variants on the quadrotor airframe, and the `Loc` variants on the drone's own pose estimate |
| `--json` | | Report, phases, entries, collisions, battery and log as JSON |
| `--report` | | The report as Markdown |
| `--track` | | The flown track as CSV: `t,x,y,z,phase`, meters in the sim frame (x east, y north) |
| `--log` | | Print the mission log |

A marginal opening asks the operator to approve entry. Headless, the CLI approves, as the mission bench
does, and the report notes how many times it was asked. A mission takes 1 to 6 s of wall time.

### `bench`: score a pilot on FrontierBench

The same flights as `npm run bench:score` (a unit test checks they are identical to `bench/results/`),
from the baselines and the shipped policy. Submitting a pilot of your own goes through the repository; see
[BENCHMARKS.md](BENCHMARKS.md).

| Option | Default | |
| --- | --- | --- |
| `--pilot` | `planner` | `planner`, `planner-momentum`, `waypoint`, `heuristic`, `policy`, `guarded` |
| `--split` | `dev` | `dev` (20 flights a task) or `test` (30, and the 60 eval plans on `apartments`) |
| `--tasks` | all | Comma list: `room`, `hard`, `dark`, `fast`, `floors`, `quad`, `apartments` |
| `--limit` | | First n flights of each task: a smoke run, marked `partial` and not comparable to the leaderboard |
| `--out` | | The result JSON, in the leaderboard's shape |

A full test split (240 flights) takes several minutes on one core.

### `plan`: export a QGroundControl plan

The woods coverage sweep the mission planner page exports, as a QGroundControl `.plan` for PX4 or ArduPilot;
see [QGC_EXPORT.md](QGC_EXPORT.md). `--seed` (4242), `--home` (`px4-sitl` or `ardupilot-sitl`), `--firmware`,
`--finish` (`rtl` or `land`), `--out`.

### `list`

Scenarios, mission pilots, bench pilots and bench tasks.

## What is in the image

`node:22-bookworm-slim`, plus `/opt/frontier-fly/`: the bundled CLI (`frontier-fly.mjs`, about 3 MB), the
actor-only shipped policy, the Swiss Dwellings floor-plan set (CC BY 4.0, see [REAL_PLANS.md](REAL_PLANS.md)),
and the licenses of everything bundled (`BUNDLED.txt` lists the npm packages: Rapier, Apache-2.0). It runs as
the unprivileged `node` user. Images are tagged `latest` (main), `sha-<commit>`, and the version of each
release tag; see [RELEASING.md](RELEASING.md).
