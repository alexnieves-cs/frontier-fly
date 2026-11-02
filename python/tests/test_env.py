import gymnasium as gym
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

import frontierfly
from frontierfly import obs as O


def test_passes_gymnasium_checker(env):
    check_env(env, skip_render_check=True)


def test_registered_ids_build():
    for task in frontierfly.TASKS:
        e = gym.make(f"FrontierFly-{task}-v0")
        obs, info = e.reset(seed=1)
        assert obs.shape == (O.OBS_DIM,) and obs.dtype == np.float32
        assert info["task"] == task
        e.close()


def test_spaces(env):
    assert env.observation_space.shape == (1088,)
    assert env.action_space.shape == (4,)
    assert env.action_space.low.min() == -1 and env.action_space.high.max() == 1


def test_same_seed_same_episode(env):
    rng = np.random.default_rng(0)
    actions = rng.uniform(-1, 1, size=(200, 4)).astype(np.float32)

    def roll():
        obs, _ = env.reset(seed=4242)
        out = [obs]
        for a in actions:
            obs, reward, term, trunc, info = env.step(a)
            out.append(np.append(obs, [reward, info["coverage"]]))
            if term or trunc:
                break
        return out

    a, b = roll(), roll()
    assert len(a) == len(b)
    for x, y in zip(a, b):
        np.testing.assert_array_equal(x, y)


def test_different_seeds_differ(env):
    a, info_a = env.reset(seed=1)
    b, info_b = env.reset(seed=2)
    assert not np.array_equal(a, b) or info_a["reachable_voxels"] != info_b["reachable_voxels"]


def test_unseeded_resets_follow_the_env_rng(env):
    env.reset(seed=7)
    first = [env.reset()[1]["seed"] for _ in range(3)]
    env.reset(seed=7)
    again = [env.reset()[1]["seed"] for _ in range(3)]
    assert first == again and len(set(first)) == 3


def test_obs_layout_matches_the_simulator(env):
    hello = env._bridge.hello
    assert hello["obs"] == O.OFFSETS
    assert hello["obsDim"] == O.OBS_DIM and hello["obsVersion"] == O.OBS_VERSION
    assert hello["floorHeight"] == O.FLOOR_HEIGHT_M
    obs, _ = env.reset(seed=3)
    assert O.maps(obs).shape == (4, 16, 16)
    assert 0 <= obs[O.ALT] <= 2


def test_episode_ends_and_needs_reset():
    env = frontierfly.make("room", max_steps=30)
    env.reset(seed=1)
    for t in range(30):
        _, _, term, trunc, info = env.step(np.zeros(4, dtype=np.float32))
        if term or trunc:
            break
    assert trunc and not term and t == 29
    with pytest.raises(frontierfly.BridgeError, match="reset"):
        env.step(np.zeros(4, dtype=np.float32))
    env.close()


def test_planner_covers_a_room():
    env = frontierfly.make("room")
    env.reset(seed=5)
    while True:
        _, _, term, trunc, info = env.step_pilot("planner")
        if term or trunc:
            break
    assert info["coverage"] > 0.9 and info["collisions"] == 0
    env.close()


def test_pilot_action_and_frame(env):
    env.reset(seed=11)
    for pilot in env.pilots:
        a = env.pilot_action(pilot)
        assert a.shape == (4,) and np.all(np.abs(a) <= 1)
    f = env.sensor_frame()
    assert f["lidar"].shape == (32,) and f["lidar"].max() <= 8.0 + 1e-6
    assert f["seen"].shape == f["visited"].shape and f["seen"].ndim == 3
    assert f["visited"].sum() >= 1 and f["seen"].sum() > f["visited"].sum()
    _, _, _, _, info = env.step_pilot("planner")
    assert info["action"].shape == (4,)


def test_actions_are_clipped_and_validated(env):
    env.reset(seed=1)
    env.step(np.array([5, -5, 5, -5], dtype=np.float32))
    with pytest.raises(ValueError):
        env.step(np.zeros(3, dtype=np.float32))
    with pytest.raises(frontierfly.BridgeError, match="finite"):
        env.step(np.array([np.nan, 0, 0, 0], dtype=np.float32))


def test_custom_episode():
    env = frontierfly.make(None, stage=4, lighting="night-dark", dynamics="quad", speed=1.5, dr=0.5)
    obs, info = env.reset(seed=9)
    assert info["task"] == "custom" and info["dynamics"] == "quad" and info["floors"] > 1
    assert info["speed_h"] == 1.5
    env.step(env.action_space.sample())
    env.close()


def test_bad_arguments():
    with pytest.raises(ValueError, match="unknown task"):
        frontierfly.make("nope")
    with pytest.raises(ValueError, match="apartments"):
        frontierfly.make("apartments")
    with pytest.raises(ValueError, match="either"):
        frontierfly.make("hard", stage=2)
    env = frontierfly.make(None, lighting="noon")
    with pytest.raises(frontierfly.BridgeError, match="lighting"):
        env.reset(seed=1)
    env.close()


def test_vector_env():
    envs = gym.vector.SyncVectorEnv([lambda: frontierfly.make("room") for _ in range(2)])
    obs, _ = envs.reset(seed=[1, 2])
    assert obs.shape == (2, 1088)
    obs, reward, term, trunc, _ = envs.step(envs.action_space.sample())
    assert reward.shape == (2,)
    envs.close()
