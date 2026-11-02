# frontierfly: frontier-fly for Python

A Gymnasium environment for frontier-fly's indoor task (a drone exploring an
unknown building with lidar) and a scorer for **FrontierBench**, the benchmark
the [frontier-fly report](https://github.com/alexnieves-cs/frontier-fly/blob/main/docs/REPORT.md)
uses. The simulator is the same TypeScript code the [live demo](https://alexnieves-cs.github.io/frontier-fly/)
runs. It is bundled into one Node script, so Python flies exactly the same physics,
sensors and buildings: the test suite checks that Python's FrontierBench flights
are bit-identical to the committed leaderboard.

## Install

You need Python 3.10+ and Node.js 22+ on `PATH`. Nothing from npm is needed at run time.

From a checkout of the repository:

```bash
npm ci && npm run build:py      # bundles the simulator into python/frontierfly/_server/
pip install -e python           # or: pip install ./python
```

The package is not on PyPI yet.

## Train

```python
import gymnasium as gym
import frontierfly

env = gym.make("FrontierFly-hard-v0")
obs, info = env.reset(seed=0)            # seed n = building n (same layout, lighting, noise)
for _ in range(1000):
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    if terminated or truncated:
        obs, info = env.reset()
env.close()
```

| | |
| --- | --- |
| Observation | `Box(1088,)` float32: 32 + 8 lidar rays, IMU, a frontier navigation block, four 16x16 egocentric maps. Layout: [`OBS_LAYOUT.md`](../src/sim/OBS_LAYOUT.md); index it by name with `frontierfly.obs`. |
| Action | `Box(4,)` in [-1, 1]: forward, left, up, yaw rate, scaled to the task's speed cap, 1 m/s and 90 deg/s. 20 steps per second. |
| Reward | New flown volume, new rooms and floors, progress toward the nearest frontier; penalties for impacts, contact, jerky actions and circling (`src/sim/reward.ts`). |
| Terminated | 95% of the reachable volume flown, too many collisions, or out of the building. |
| Truncated | Stalled (500 steps without new volume) or out of time. |
| `info` | `coverage`, `collisions`, `floor`, `success`, `energy_wh`, `new_room`, `new_floor`. |

Registered ids are the FrontierBench tasks: `FrontierFly-room-v0` (one room),
`-hard-v0` (rooms, hallways, loops), `-dark-v0` (lights off), `-fast-v0` (4 m/s),
`-floors-v0` (several floors joined by shafts) and `-quad-v0` (a quadrotor with
randomized dynamics and noise). `FrontierFly-custom-v0` takes your own settings:

```python
env = frontierfly.make(None, stage=4, lighting="night-dark", dynamics="quad", speed=1.5, dr=0.5)
```

Each env runs its own Node process, so `gym.vector.AsyncVectorEnv` spreads over
cores. On a 4-core cloud runner one env steps about 2,600 times a second with
random actions, and four async envs together about 2,500.

### Built-in pilots and imitation data

The repository's baselines run inside the simulator: `planner` (the sensor-only
frontier planner, the strongest), `planner-momentum`, `waypoint`, `heuristic`
(lidar wall-follower), `policy` (the shipped DAgger + PPO network) and `guarded`
(that network under the lidar safety filter).

```python
env.step_pilot("planner")          # the planner flies one step; info["action"] is what it chose
env.pilot_action("planner")        # the action it would take now, as a label
env.sensor_frame()                 # pose, lidar in meters, seen/visited voxel grids
```

[`examples/collect_demos.py`](examples/collect_demos.py) writes a behavior-cloning
dataset from the planner's flights.

## Benchmark

FrontierBench is 7 frozen tasks with fixed seeds: a dev split (tune freely) and a
test split (score once). The score is mean flown coverage over tasks; `safeShare`
is the share of flights that touched nothing. Current standings are in
[`docs/LEADERBOARD.md`](../docs/LEADERBOARD.md).

```bash
python -m frontierfly.bench --pilot planner --split dev
python -m frontierfly.bench --policy my_module:MyPolicy --split test --out result.json
```

```python
from frontierfly.bench import evaluate

result = evaluate(lambda obs: my_net(obs), split="dev", tasks=["room", "hard"])
print(result["score"], result["safeShare"], result["tasks"]["hard"])
```

A policy is a callable `policy(obs) -> action`, or an object with
`act(obs, env)` and an optional `reset(info, env)`; `env.sensor_frame()` gives it
what a bench pilot may see besides the observation (never the floor plan).
[`examples/waypoint_pilot.py`](examples/waypoint_pilot.py) ports the TypeScript
`waypoint` baseline to Python and scores identically. To put a result on the
leaderboard, follow [`docs/BENCHMARKS.md`](../docs/BENCHMARKS.md); entries are
re-scored from their TypeScript pilot code in CI.

## Tests

```bash
npm run test:py      # builds the bundle, then runs python/tests with pytest
```

## Licenses

Code: MIT. The bundled simulator includes [Rapier](https://rapier.rs) (Apache-2.0);
both license texts ship in `frontierfly/_server/`. The real apartment plans
(`_server/assets/plans/swiss-dwellings.json`) are from
[Swiss Dwellings v3.0.0](https://zenodo.org/records/7788422) by Archilyse AG,
doi:10.5281/zenodo.7788422, under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/);
the changes are recorded in the file and in [`docs/REAL_PLANS.md`](../docs/REAL_PLANS.md).
