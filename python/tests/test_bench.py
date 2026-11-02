import json
import sys

import numpy as np
import pytest

from frontierfly.bench import evaluate, main


@pytest.mark.parametrize("pilot", ["planner", "policy"])
def test_matches_the_committed_leaderboard(repo, pilot):
    """Python scores bit-identical flights to `npm run bench:score` (bench/results/*.json)."""
    path = repo / "bench" / "results" / f"{pilot}.json"
    if not path.exists():
        pytest.skip("needs the repository checkout")
    ref = json.loads(path.read_text())
    tasks = ["hard", "floors", "quad", "apartments"]
    got = evaluate(pilot, ref["split"], tasks, max_flights=2)
    assert got["bench"] == ref["bench"]
    for e in got["episodes"]:
        r = next(x for x in ref["episodes"] if x["task"] == e["task"] and x["seed"] == e["seed"])
        for key in ("coverage", "collisions", "steps", "energyWh", "halfS", "footprint", "area", "footprintHalfS"):
            assert e[key] == r[key], (e["task"], e["seed"], key)


def test_python_pilot_scores_like_its_typescript_twin(repo):
    sys.path.insert(0, str(repo / "python" / "examples"))
    from waypoint_pilot import waypoint_pilot

    ours = evaluate(waypoint_pilot, "dev", ["room", "hard", "quad"], max_flights=3)
    theirs = evaluate("waypoint", "dev", ["room", "hard", "quad"], max_flights=3)
    assert [e["coverage"] for e in ours["episodes"]] == [e["coverage"] for e in theirs["episodes"]]
    assert ours["score"] == theirs["score"]


class Recorder:
    name = "recorder"

    def __init__(self):
        self.resets = 0

    def reset(self, info, env):
        self.resets += 1
        assert info["split"] == "dev"

    def act(self, obs, env):
        frame = env.sensor_frame()
        assert frame["seen"].ndim == 3
        return np.zeros(4, dtype=np.float32)


def test_policy_object_and_summary():
    p = Recorder()
    res = evaluate(p, "dev", ["room"], max_flights=2)
    assert p.resets == 2 and res["name"] == "recorder" and res["source"] == "python"
    assert set(res["tasks"]) == {"room"} and res["tasks"]["room"]["episodes"] == 2
    assert 0 <= res["score"] < 0.1 and res["safeShare"] == 1


def test_rejects_bad_input():
    with pytest.raises(ValueError):
        evaluate("planner", "train")
    with pytest.raises(ValueError):
        evaluate("nope", "dev", ["room"])
    with pytest.raises(ValueError):
        evaluate("planner", "dev", ["nope"])


def test_cli(tmp_path, capsys):
    out = tmp_path / "r.json"
    assert main(["--pilot", "heuristic", "--tasks", "room", "--max-flights", "1", "--out", str(out)]) == 0
    assert "score" in capsys.readouterr().out
    assert json.loads(out.read_text())["episodes"][0]["task"] == "room"
