"""Collect imitation-learning data: observations labeled with the sensor-only planner's actions.

    python examples/collect_demos.py --task hard --episodes 20 --out demos.npz

The planner flies (so the data is on its own state distribution); each row is the
observation it saw and the action it took. Seeds start at 1_000_000 so they never
overlap FrontierBench's dev (81001+) or test (82001+) buildings.
"""

from __future__ import annotations

import argparse

import numpy as np

import frontierfly


def collect(task: str, episodes: int, first_seed: int = 1_000_000):
    env = frontierfly.make(task)
    observations, actions, coverage = [], [], []
    try:
        for ep in range(episodes):
            obs, _ = env.reset(seed=first_seed + ep)
            while True:
                next_obs, _, term, trunc, info = env.step_pilot("planner")
                observations.append(obs)
                actions.append(info["action"])
                obs = next_obs
                if term or trunc:
                    coverage.append(info["coverage"])
                    break
    finally:
        env.close()
    return np.stack(observations), np.stack(actions), np.array(coverage)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", default="hard")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--out", default="demos.npz")
    args = parser.parse_args()
    obs, act, cov = collect(args.task, args.episodes)
    np.savez_compressed(args.out, obs=obs, act=act)
    print(f"{len(obs)} steps from {args.episodes} episodes, planner coverage {cov.mean() * 100:.1f}% -> {args.out}")
