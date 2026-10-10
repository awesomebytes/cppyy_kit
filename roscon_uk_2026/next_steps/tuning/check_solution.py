#!/usr/bin/env python3
"""Check the exercise implementation with independent numerical expectations."""
import argparse
import importlib.util
from pathlib import Path
import tempfile
import numpy as np
import tune
from test_tuning import independent_recurrence


def check(path):
    spec = importlib.util.spec_from_file_location("candidate", path)
    candidate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(candidate)
    episodes = [tune.make_episode(seed=seed, samples=200) for seed in (0, 1)]
    config = tune.FilterConfig(tau_s=.037)
    expected_squared = []
    for ep in episodes:
        reference = independent_recurrence(config, ep)
        expected_squared.extend(np.sum((reference - ep["truth_m"])**2, axis=1))
    with tune.PoseFilter(config) as estimator:
        actual = candidate.replay_trial(config, episodes, estimator)
        reversed_actual = candidate.replay_trial(config, list(reversed(episodes)), estimator)
    np.testing.assert_allclose(actual["position_rmse_m"], np.sqrt(np.mean(expected_squared)), atol=1e-12, rtol=1e-12)
    assert {row["episode_id"]: row for row in actual["episodes"]} == {
        row["episode_id"]: row for row in reversed_actual["episodes"]}
    tune.HERE.joinpath("build").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=tune.HERE / "build") as directory:
        a, b = Path(directory) / "a", Path(directory) / "b"
        a.mkdir(); b.mkdir()
        rows = candidate.ask_and_tell(episodes, 10, 12, a)
        repeated = candidate.ask_and_tell(episodes, 10, 12, b)
        assert len(rows) == 10
        assert [row["resolved_config"] for row in rows] == [row["resolved_config"] for row in repeated]
        assert [row["training"]["position_rmse_m"] for row in rows] == [row["training"]["position_rmse_m"] for row in repeated]
        selected = candidate.select_training(list(reversed(rows)))
        assert selected["training"]["position_rmse_m"] == min(row["training"]["position_rmse_m"] for row in rows)
        assert (a / "optuna_trials.json").exists()
    failed = {"number": 99, "status": "FAIL", "error": "declared failure"}
    assert candidate.select_training([failed, *rows])["number"] == selected["number"]
    try:
        candidate.select_training([failed])
    except RuntimeError:
        pass
    else:
        raise AssertionError("all-failed study must reject selection")
    print("Exercise checks passed: independent recurrence, reset/order, seeded trials, selection, persistence.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("implementation", type=Path, nargs="?", default=tune.HERE / "solution.py")
    check(parser.parse_args().implementation)
