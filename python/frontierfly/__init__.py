"""frontier-fly as a Python library: a Gymnasium env and the FrontierBench scorer.

    import gymnasium as gym
    import frontierfly

    env = gym.make("FrontierFly-hard-v0")
    obs, info = env.reset(seed=0)

The simulator is the same TypeScript code the browser demo runs, bundled into
one Node script; each env starts its own Node 22+ process.
"""

from gymnasium.envs.registration import register

from ._bridge import BridgeError
from .env import FrontierFlyEnv

__all__ = ["FrontierFlyEnv", "BridgeError", "TASKS", "make"]
__version__ = "0.1.0"

#: FrontierBench tasks that work as training envs (``apartments`` flies fixed real plans: score it with ``bench``).
TASKS = ("room", "hard", "dark", "fast", "floors", "quad")

for _task in TASKS:
    register(id=f"FrontierFly-{_task}-v0", entry_point="frontierfly.env:FrontierFlyEnv", kwargs={"task": _task})
register(id="FrontierFly-custom-v0", entry_point="frontierfly.env:FrontierFlyEnv", kwargs={"task": None})


def make(task: str | None = "hard", **kwargs) -> FrontierFlyEnv:
    """Build an env without the Gymnasium registry (no wrappers, no time limit wrapper)."""
    return FrontierFlyEnv(task, **kwargs)
