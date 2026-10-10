#!/usr/bin/env python3
"""Replay saved trial metrics and selected configurations without optimization."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import tune


def replay(directory):
    directory = Path(directory).resolve()
    provenance = json.loads((directory / "provenance.json").read_text())
    for name, expected in provenance["source_sha256"].items():
        actual = hashlib.sha256((tune.ROOT / name).read_bytes()).hexdigest()
        if actual != expected:
            raise AssertionError(f"source changed since run: {name}")
    datasets = {}
    for split, seeds in (("training", provenance["train_seeds"]), ("held_out", provenance["held_out_seeds"])):
        datasets[split] = [tune.make_episode(seed=seed, samples=provenance["samples_per_episode"]) for seed in seeds]
        actual = [{"episode_id": ep["episode_id"], "sha256": tune.dataset_hash(ep)} for ep in datasets[split]]
        assert actual == provenance["datasets"][split]
    count = 0
    for filename in ("optuna_trials.json", "grid_trials.json", "defaults.json"):
        rows = json.loads((directory / filename).read_text())
        if isinstance(rows, dict):
            rows = [rows]
        for row in rows:
            if row["status"] != "COMPLETE":
                continue
            actual = tune.evaluate(tune.FilterConfig(**row["resolved_config"]), datasets["training"])
            np.testing.assert_allclose(actual["position_rmse_m"], row["training"]["position_rmse_m"], rtol=1e-12, atol=1e-12)
            for old, new in zip(row["training"]["episodes"], actual["episodes"]):
                np.testing.assert_allclose(new["position_rmse_m"], old["position_rmse_m"], rtol=1e-12, atol=1e-12)
                assert new["alignment"] == old["alignment"]
            count += 1
    results = json.loads((directory / "results.json").read_text())
    for name, row in results["configurations"].items():
        config = tune.load_config(directory / f"{name}.config")
        assert config.model_dump() == row["resolved_config"]
        actual = tune.evaluate(config, datasets["held_out"])
        np.testing.assert_allclose(actual["position_rmse_m"], row["held_out"]["position_rmse_m"], rtol=1e-12, atol=1e-12)
        for old, new in zip(row["held_out"]["episodes"], actual["episodes"]):
            assert old["alignment"] == new["alignment"]
    chosen = results["chosen_by_training"]
    assert tune.load_config(directory / "chosen.config").model_dump() == results["configurations"][chosen]["resolved_config"]
    config = tune.load_config(directory / "chosen.config")
    with tune.PoseFilter(config) as estimator:
        for ep in datasets["held_out"]:
            estimator.reset()
            native = estimator.process(ep["timestamps_ns"], ep["positions_m"])
            driver = tune.driver_replay(config, ep["timestamps_ns"], ep["positions_m"],
                                       tune.HERE / "build" / "saved-replay" / ep["episode_id"])
            np.testing.assert_allclose(native, driver, rtol=1e-12, atol=1e-12)
    print(f"Reproduced {count} complete training evaluations and 3 held-out configurations; source and dataset hashes match; chosen export passes C++ driver parity.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    replay(parser.parse_args().directory)
