"""Score a pilot on FrontierBench, the same way ``npm run bench:score`` does.

FrontierBench is a frozen set of indoor exploration tasks with fixed seeds
(``src/bench/tasks.ts``): each task has a dev split (tune on it freely) and a
test split (score on it once). The score is the mean flown coverage over tasks;
``safeShare`` is the share of flights that touched nothing.

    python -m frontierfly.bench --pilot planner --split dev --tasks room,hard
    python -m frontierfly.bench --policy my_module:make_policy --split test --out result.json
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
import time
from typing import Any, Callable

import numpy as np

from .env import FrontierFlyEnv

Policy = Any


def _flights(env: FrontierFlyEnv, task: str, split: str) -> int:
    _, info = env.bench_reset(task, split, 0)
    return int(info["flights"])


def evaluate(
    policy: Policy,
    split: str = "dev",
    tasks: list[str] | None = None,
    *,
    name: str | None = None,
    log: Callable[[str], None] | None = None,
    max_flights: int | None = None,
) -> dict[str, Any]:
    """Fly every flight of ``split`` (or the listed ``tasks``) and score it.

    ``policy`` is one of:

    * a built-in pilot id (``planner``, ``planner-momentum``, ``waypoint``,
      ``heuristic``, ``policy``, ``guarded``), flown inside the simulator;
    * a callable ``policy(obs) -> action``;
    * an object with ``act(obs, env) -> action`` and optionally ``reset(info, env)``,
      called at the start of each flight. ``env.sensor_frame()`` gives the pose,
      lidar and seen/visited grids a bench pilot may use.

    ``max_flights`` caps the flights per task (for smoke tests; a capped run is
    not a FrontierBench score). Returns the same summary ``bench:score`` writes:
    ``score``, ``safeShare``, per-task stats, and every episode.
    """
    if split not in ("dev", "test"):
        raise ValueError('split must be "dev" or "test"')
    env = FrontierFlyEnv("room")
    try:
        all_tasks = [t["id"] for t in env._bridge.hello["tasks"]]
        chosen = tasks or all_tasks
        unknown = [t for t in chosen if t not in all_tasks]
        if unknown:
            raise ValueError(f"unknown task(s) {unknown}; known: {all_tasks}")
        builtin = isinstance(policy, str)
        if builtin and policy not in env.pilots:
            raise ValueError(f"unknown pilot {policy!r}; known: {env.pilots}")
        episodes: list[dict[str, Any]] = []
        t0 = time.time()
        for task in chosen:
            n = _flights(env, task, split)
            if max_flights is not None:
                n = min(n, max_flights)
            mine = []
            for i in range(n):
                obs, info = env.bench_reset(task, split, i)
                if not builtin and hasattr(policy, "reset"):
                    policy.reset(info, env)
                while True:
                    if builtin:
                        obs, _, term, trunc, step_info = env.step_pilot(policy)
                    else:
                        action = policy.act(obs, env) if hasattr(policy, "act") else policy(obs)
                        obs, _, term, trunc, step_info = env.step(np.asarray(action, dtype=np.float32))
                    if term or trunc:
                        mine.append(step_info["episode"])
                        break
            episodes.extend(mine)
            if log:
                cov = sum(e["coverage"] for e in mine) / max(1, len(mine))
                log(f"{task}: {cov * 100:.1f}% over {len(mine)} flights ({time.time() - t0:.0f} s)")
        reply, _ = env._bridge.call("summarize", episodes=episodes)
        hello = env._bridge.hello
    finally:
        env.close()
    label = name or (policy if builtin else getattr(policy, "name", type(policy).__name__))
    return {
        "bench": hello["benchVersion"],
        "name": label,
        "source": f"baseline:{policy}" if builtin else "python",
        "split": split,
        "taskDefs": hello["tasks"],
        **reply["summary"],
        "episodes": episodes,
    }


def _load(spec: str) -> Policy:
    module, _, attr = spec.partition(":")
    obj = getattr(importlib.import_module(module), attr or "policy")
    # A factory (class or zero-argument function) builds the policy.
    if isinstance(obj, type) or (callable(obj) and getattr(obj, "__frontierfly_factory__", False)):
        return obj()
    return obj


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m frontierfly.bench", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    who = parser.add_mutually_exclusive_group()
    who.add_argument("--pilot", default=None, help="built-in pilot id (default: planner)")
    who.add_argument("--policy", default=None, help="module:attr of a policy callable, policy object or policy class")
    parser.add_argument("--split", default="dev", choices=["dev", "test"])
    parser.add_argument("--tasks", default="", help="comma-separated task ids (default: all)")
    parser.add_argument("--max-flights", type=int, default=None, help="cap flights per task (smoke tests only)")
    parser.add_argument("--out", default="", help="write the result JSON here")
    args = parser.parse_args(argv)
    policy: Policy = _load(args.policy) if args.policy else (args.pilot or "planner")
    tasks = [t for t in args.tasks.split(",") if t] or None
    result = evaluate(policy, args.split, tasks, log=print, max_flights=args.max_flights)
    print(f"{result['name']}: score {result['score'] * 100:.1f}%, {result['safeShare'] * 100:.0f}% of flights touched nothing")
    if args.out:
        with open(args.out, "w", encoding="utf8") as fh:
            json.dump(result, fh, indent=2)
            fh.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
