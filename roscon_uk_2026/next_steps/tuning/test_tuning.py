"""Independent numerical, lifecycle, selection, persistence and parity checks."""
import math
import json
import numpy as np
import pytest

import tune
from metrics import alignment_lag, position_rmse


def numerical_record(row):
    return {key: value for key, value in row.items() if key != "timing_s"}


def test_rmse_cartesian_units():
    truth = np.zeros((2, 3))
    estimate = np.array([[3., 4., 0.], [0., 0., 0.]])
    assert position_rmse(estimate, truth) == pytest.approx(math.sqrt(12.5))


def test_alignment_measures_known_shift_and_censors_boundary():
    t = np.arange(1001) * 0.005
    def curve(time):
        return np.column_stack((time**2 / 10, np.sin(time**2), np.cos(time * 0.7)))
    truth = curve(t)
    assert alignment_lag(t, truth, truth)["lag_s"] == 0
    delayed = curve(t-0.075)
    result = alignment_lag(t, delayed, truth)
    assert result["lag_s"] == pytest.approx(0.075, abs=0.005)
    assert not result["at_upper_bound"]
    beyond = alignment_lag(t, curve(t-0.40), truth)
    assert beyond["at_upper_bound"]
    assert beyond["lag_s"] == pytest.approx(0.30)


def test_alignment_excludes_interpolation_across_gap():
    t = np.concatenate((np.arange(0, 2, .01), np.arange(3, 5, .01)))
    truth = np.column_stack((t, t**2, t**3))
    result = alignment_lag(t, truth, truth)
    expected = (t >= .5) & (t <= t[-1]-.3) & ~((t > t[199]-.3) & (t < t[200]))
    assert result["common_samples"] == int(expected.sum())
    assert result["lag_s"] == 0


def independent_recurrence(config, episode):
    t, x = episode["timestamps_ns"], episode["positions_m"]
    output = np.empty_like(x)
    state = x[0].copy()
    output[0] = state
    for index in range(1, len(t)):
        dt = (int(t[index]) - int(t[index-1])) * 1e-9
        if dt > config.max_gap_s:
            state = x[index].copy()
        else:
            state = state + (-math.expm1(-dt / config.tau_s)) * (x[index] - state)
        output[index] = state
    return output


def test_native_replay_matches_independent_recurrence_and_cpp_export(tmp_path):
    episode = tune.make_episode(seed=17, samples=200)
    config = tune.FilterConfig(tau_s=.037)
    with tune.PoseFilter(config) as estimator:
        actual = estimator.process(episode["timestamps_ns"], episode["positions_m"])
    reference = independent_recurrence(config, episode)
    np.testing.assert_allclose(actual, reference, atol=1e-12, rtol=1e-12)
    tune.export_config(config, tmp_path / "resolved.config")
    resolved = tune.load_config(tmp_path / "resolved.config")
    assert resolved == config
    standalone = tune.driver_replay(resolved, episode["timestamps_ns"], episode["positions_m"], tmp_path / "driver")
    np.testing.assert_allclose(standalone, reference, atol=1e-12, rtol=1e-12)


def test_reset_and_episode_order_isolation():
    episodes = [tune.make_episode(seed=seed, samples=200) for seed in (0, 2, 3)]
    config = tune.FilterConfig(tau_s=.1)
    with tune.PoseFilter(config) as estimator:
        forward = tune.evaluate(config, episodes, estimator)
        reverse = tune.evaluate(config, list(reversed(episodes)), estimator)
        repeated = tune.evaluate(config, episodes, estimator)
    first = {row["episode_id"]: row for row in forward["episodes"]}
    second = {row["episode_id"]: row for row in reverse["episodes"]}
    assert first == second
    assert numerical_record(forward) == numerical_record(repeated)
    assert forward["position_rmse_m"] == pytest.approx(reverse["position_rmse_m"], abs=1e-15)


def test_trial_order_and_metric_replay():
    episodes = [tune.make_episode(seed=seed, samples=200) for seed in (0, 1)]
    configs = [tune.FilterConfig(tau_s=tau) for tau in (.005, .03, .08, .4)]
    forward = [tune.record_trial(i, config, episodes) for i, config in enumerate(configs)]
    reverse = [tune.record_trial(i, config, episodes) for i, config in enumerate(reversed(configs))]
    a = {row["resolved_config"]["tau_s"]: numerical_record(row["training"]) for row in forward}
    b = {row["resolved_config"]["tau_s"]: numerical_record(row["training"]) for row in reverse}
    assert a == b


def test_seeded_ask_tell_is_reproducible_and_saved(tmp_path):
    episodes = [tune.make_episode(seed=seed, samples=200) for seed in (0, 1)]
    run_a, run_b = tmp_path / "a", tmp_path / "b"
    run_a.mkdir(); run_b.mkdir()
    first = tune.search(episodes, budget=12, seed=34, output=run_a)
    second = tune.search(episodes, budget=12, seed=34, output=run_b)
    assert [row["optuna_params"] for row in first] == [row["optuna_params"] for row in second]
    assert [numerical_record(row["training"]) for row in first] == [numerical_record(row["training"]) for row in second]
    assert json.loads((run_a / "optuna_trials.json").read_text()) == first
    assert tune.best_complete(first)["resolved_config"] == tune.best_complete(second)["resolved_config"]


def test_failed_trial_is_saved_and_told_to_optuna(tmp_path, monkeypatch):
    episodes = [tune.make_episode(seed=0, samples=200)]
    def fail(*args, **kwargs):
        raise ValueError("deliberate test-only replay failure")
    monkeypatch.setattr(tune, "evaluate", fail)
    rows = tune.search(episodes, budget=2, seed=34, output=tmp_path)
    assert all(row["status"] == "FAIL" for row in rows)
    assert all(row["error_type"] == "ValueError" for row in rows)
    assert all("deliberate test-only replay failure" in row["error"] for row in rows)
    assert json.loads((tmp_path / "optuna_trials.json").read_text()) == rows
    study = tune.optuna.load_study(study_name="tau_s", storage=f"sqlite:///{tmp_path / 'study.sqlite3'}")
    assert all(trial.state == tune.optuna.trial.TrialState.FAIL for trial in study.trials)


def test_dataset_identity_and_split():
    first = tune.make_episode(seed=0, samples=200)
    assert tune.dataset_hash(first) == tune.dataset_hash(tune.make_episode(seed=0, samples=200))
    mutated = {**first, "truth_m": first["truth_m"].copy()}
    mutated["truth_m"][0, 0] += 1e-8
    assert tune.dataset_hash(mutated) != tune.dataset_hash(first)
    assert not set(tune.TRAIN_SEEDS) & set(tune.HELD_OUT_SEEDS)
