"""A FrontierBench pilot written in Python: steer along the observation's frontier waypoint.

This is the TypeScript `waypoint` baseline (src/sim/waypointPilot.ts) ported line by line,
so it shows how a Python pilot plugs into the scorer and should score like its twin:

    python examples/waypoint_pilot.py --split dev --tasks room,hard
    python -m frontierfly.bench --pilot waypoint --split dev --tasks room,hard
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from frontierfly import obs as O
from frontierfly.bench import evaluate


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def waypoint_pilot(observation: np.ndarray) -> np.ndarray:
    fx, fy = float(observation[O.WP_DIR][0]), float(observation[O.WP_DIR][1])
    vz = clamp((1 - float(observation[O.ALT]) * O.FLOOR_HEIGHT_M) * 3, -0.6, 0.6)
    if observation[O.HAS_FRONTIER] < 0.5:
        return np.array([0, 0, vz, 0.5], dtype=np.float32)
    err = math.atan2(fy, fx)
    return np.array(
        [clamp(fx * 0.9, -0.35, 0.9), clamp(fy * 0.75, -0.75, 0.75), vz, clamp(err / (math.pi / 3), -0.8, 0.8)],
        dtype=np.float32,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--split", default="dev", choices=["dev", "test"])
    parser.add_argument("--tasks", default="room,hard")
    args = parser.parse_args()
    result = evaluate(waypoint_pilot, args.split, args.tasks.split(","), name="Waypoint follower (Python)", log=print)
    print(f"score {result['score'] * 100:.1f}%, {result['safeShare'] * 100:.0f}% of flights touched nothing")
