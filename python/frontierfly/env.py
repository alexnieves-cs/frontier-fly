"""Gymnasium environment: a drone exploring an unknown building, frontier-fly's indoor task."""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from ._bridge import Bridge

_CUSTOM_KEYS = ("stage", "lighting", "dynamics", "speed", "dr")


def _snake(info: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in info.items():
        snake = "".join("_" + c.lower() if c.isupper() else c for c in key)
        out[snake] = value
    return out


class FrontierFlyEnv(gym.Env):
    """Explore an unknown building with lidar, as fast and as safely as you can.

    Every episode is a procedurally generated building (rooms, hallways, loops,
    furniture, optionally several floors). The drone starts inside and is
    rewarded for flying through space it has not flown yet. The episode ends
    when 95% of the reachable volume is flown, after too many collisions, when
    the drone stalls, or when time runs out.

    Observation: 1088 float32 values (lidar rings, IMU, a frontier navigation
    block and four 16x16 egocentric maps); the layout is ``src/sim/OBS_LAYOUT.md``.

    Action: 4 values in [-1, 1]: forward, left, up and yaw rate, scaled to the
    task's speed cap (2 m/s by default), 1 m/s and 90 deg/s. Steps are 50 ms (20 Hz).

    Args:
        task: a FrontierBench task id (``room``, ``hard``, ``dark``, ``fast``,
            ``floors``, ``quad``). Pass ``None`` to set the episode yourself with
            the keyword arguments below.
        stage: building difficulty 0..5 (0 = one room, 3 = rooms + hallways + loops,
            4 = several floors, 5 = several floors with harder layouts).
        lighting: ``day``, ``dusk``, ``night-lit``, ``night-dark`` or ``ir``.
        dynamics: ``kinematic`` (velocity tracking) or ``quad`` (a quadrotor model).
        speed: horizontal speed of a full action, m/s.
        dr: domain-randomization scale for the quadrotor (mass, thrust, drag, lag, noise).
        max_steps: override the episode length in steps.

    ``reset(seed=n)`` flies building ``n``: the same seed is the same building,
    lighting and noise. Without a seed, the env draws building seeds from its own
    seeded generator.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        task: str | None = "hard",
        *,
        stage: int | None = None,
        lighting: str | None = None,
        dynamics: str | None = None,
        speed: float | None = None,
        dr: float | None = None,
        max_steps: int | None = None,
    ) -> None:
        custom = {"stage": stage, "lighting": lighting, "dynamics": dynamics, "speed": speed, "dr": dr}
        if task is not None and any(v is not None for v in custom.values()):
            raise ValueError("pass either task=... or task=None with stage/lighting/dynamics/speed/dr, not both")
        self._bridge = Bridge()
        hello = self._bridge.hello
        self.tasks = {t["id"]: t for t in hello["tasks"]}
        if task is not None:
            if task not in self.tasks:
                raise ValueError(f"unknown task {task!r}; known: {', '.join(self.tasks)}")
            if self.tasks[task].get("plans"):
                raise ValueError("the apartments task flies fixed real plans; score it with frontierfly.bench.evaluate")
        self.task = task
        self._episode: dict[str, Any] = {"task": task}
        if task is None:
            self._episode.update(
                stage=0 if stage is None else int(stage),
                lighting=lighting or "day",
                dynamics=dynamics or "kinematic",
                speedH=2.0 if speed is None else float(speed),
                dr=0.0 if dr is None else float(dr),
            )
        if max_steps is not None:
            self._episode["maxSteps"] = int(max_steps)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(hello["obsDim"],), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(hello["actDim"],), dtype=np.float32)
        self.pilots: list[str] = list(hello["pilots"])

    # Gymnasium API -----------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        building = int(seed) if seed is not None else int(self.np_random.integers(0, 2**31 - 1))
        reply, payload = self._bridge.call("reset", seed=building, **self._episode)
        return Bridge.floats(payload), _snake(reply["info"])

    def step(self, action):
        a = np.asarray(action, dtype="<f4").reshape(-1)
        if a.shape[0] != self.action_space.shape[0]:
            raise ValueError(f"action must have {self.action_space.shape[0]} values, got {a.shape[0]}")
        return self._unpack(*self._bridge.call("step", a.tobytes()))

    def close(self) -> None:
        self._bridge.close()
        super().close()

    # Extras ------------------------------------------------------------------

    def step_pilot(self, pilot: str = "planner"):
        """Step with a built-in pilot flying (one round trip). ``info['action']`` is what it chose."""
        return self._unpack(*self._bridge.call("step", pilot=pilot))

    def pilot_action(self, pilot: str = "planner") -> np.ndarray:
        """The action a built-in pilot would take now, e.g. as an imitation-learning label.

        Pilots: ``planner`` (the sensor-only frontier planner, the strongest
        baseline), ``planner-momentum``, ``waypoint``, ``heuristic`` (lidar
        wall-follower), ``policy`` (the shipped DAgger + PPO network) and
        ``guarded`` (that network under the lidar safety filter). Stateful pilots
        keep their own map, so ask the same pilot every step.
        """
        _, payload = self._bridge.call("act", pilot=pilot)
        return Bridge.floats(payload)

    def sensor_frame(self) -> dict[str, Any]:
        """What a FrontierBench pilot may see besides the observation.

        Pose, measured velocity, the 32 horizontal lidar ranges in meters, and two
        voxel grids (0.5 m cells, shape ``floors x vd x vw``): ``seen`` (free space the
        lidar has swept) and ``visited`` (space the drone has flown). Never the floor plan.
        """
        head, payload = self._bridge.call("frame")
        n = head["lidarRays"] * 4
        shape = (head["floors"], head["vd"], head["vw"])
        size = int(np.prod(shape))
        grids = np.frombuffer(payload[n:], dtype=np.uint8)
        return {
            "pos": np.array(head["pos"], dtype=np.float64),
            "yaw": float(head["yaw"]),
            "vel": np.array(head["vel"], dtype=np.float64),
            "alt": float(head["alt"]),
            "lidar": np.frombuffer(payload[:n], dtype="<f4").astype(np.float32),
            "seen": grids[:size].reshape(shape).copy(),
            "visited": grids[size : 2 * size].reshape(shape).copy(),
            "floor_height": float(head["floorHeight"]),
        }

    def bench_reset(self, task: str, split: str, index: int):
        """Start flight ``index`` of a FrontierBench split; the last step's ``info['episode']`` is its score."""
        reply, payload = self._bridge.call("bench_reset", task=task, split=split, index=int(index))
        return Bridge.floats(payload), _snake(reply["info"])

    def _unpack(self, reply: dict[str, Any], payload: bytes):
        info = _snake(reply["info"])
        if "action" in info:
            info["action"] = np.asarray(info["action"], dtype=np.float32)
        return Bridge.floats(payload), float(reply["reward"]), bool(reply["terminated"]), bool(reply["truncated"]), info
