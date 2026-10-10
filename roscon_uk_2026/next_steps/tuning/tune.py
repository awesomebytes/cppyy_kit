#!/usr/bin/env python3
"""Reproducible sequential tuning around whole native pose-filter replays."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import resource
import shlex
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
IMPORT_STARTED = time.perf_counter()
import numpy as np
import optuna
from metrics import alignment_lag, position_rmse
from roscon_uk_2026.next_steps.reverse_core import (
    FilterConfig, PoseFilter, build_native, driver_replay, export_config,
    load_config, make_episode, native_namespace,
)
IMPORT_S = time.perf_counter() - IMPORT_STARTED

TRAIN_SEEDS = (0, 1, 2, 3)
HELD_OUT_SEEDS = (100, 101, 102, 103)
SAMPLER_SEED = 20261004
BOUNDS_S = (0.005, 0.5)
BUDGET = 24
SAMPLES = 500


def dataset_hash(episode):
    digest = hashlib.sha256()
    digest.update(episode["episode_id"].encode())
    for key in ("timestamps_ns", "positions_m", "truth_m"):
        array = np.ascontiguousarray(episode[key])
        digest.update(key.encode())
        digest.update(array.dtype.str.encode())
        digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def evaluate(config, episodes, estimator=None):
    """Every episode starts with reset and crosses into native processing once."""
    owns = estimator is None
    if estimator is not None and estimator.config != config:
        raise ValueError("estimator configuration does not match requested configuration")
    estimator = estimator or PoseFilter(config)
    records, process_s, reset_s, metric_s = [], 0.0, 0.0, 0.0
    try:
        for episode in episodes:
            started = time.perf_counter()
            estimator.reset()
            reset_s += time.perf_counter() - started
            started = time.perf_counter()
            output = estimator.process(episode["timestamps_ns"], episode["positions_m"])
            process_s += time.perf_counter() - started
            started = time.perf_counter()
            timestamps_s = (episode["timestamps_ns"] - episode["timestamps_ns"][0]) * 1e-9
            records.append({"episode_id": episode["episode_id"],
                            "position_rmse_m": position_rmse(output, episode["truth_m"]),
                            "samples": len(output),
                            "alignment": alignment_lag(timestamps_s, output, episode["truth_m"])})
            metric_s += time.perf_counter() - started
    finally:
        if owns:
            estimator.close()
    total_samples = sum(row["samples"] for row in records)
    return {"position_rmse_m": float(np.sqrt(sum(row["position_rmse_m"]**2 * row["samples"]
                                                   for row in records) / total_samples)),
            "mean_alignment_lag_s": float(np.mean([row["alignment"]["lag_s"] for row in records])),
            "episodes": records,
            "timing_s": {"reset": reset_s, "process_including_adapter": process_s, "metrics": metric_s}}


def save_json(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def record_trial(number, config, episodes):
    record = {"number": number, "resolved_config": config.model_dump(), "status": "RUNNING"}
    try:
        record["training"] = evaluate(config, episodes)
        record["status"] = "COMPLETE"
    except Exception as exc:
        record.update(status="FAIL", error_type=type(exc).__name__, error=str(exc), traceback=traceback.format_exc())
    return record


def search(episodes, budget, seed, output):
    sampler = optuna.samplers.TPESampler(seed=seed, n_startup_trials=min(8, budget))
    study = optuna.create_study(direction="minimize", sampler=sampler,
                               storage=f"sqlite:///{output / 'study.sqlite3'}", study_name="tau_s")
    records = []
    for _ in range(budget):
        trial = study.ask()
        config = FilterConfig(tau_s=trial.suggest_float("tau_s", *BOUNDS_S, log=True))
        row = record_trial(trial.number, config, episodes)
        if row["status"] == "COMPLETE":
            study.tell(trial, row["training"]["position_rmse_m"])
        else:
            study.tell(trial, state=optuna.trial.TrialState.FAIL)
        row["optuna_params"] = trial.params
        records.append(row)
        save_json(output / "optuna_trials.json", records)
    return records


def simple_search(episodes, budget, output):
    records = []
    for number, tau in enumerate(np.geomspace(*BOUNDS_S, budget)):
        row = record_trial(number, FilterConfig(tau_s=float(tau)), episodes)
        records.append(row)
        save_json(output / "grid_trials.json", records)
    return records


def best_complete(records):
    complete = [row for row in records if row["status"] == "COMPLETE"]
    if not complete:
        raise RuntimeError("no complete trials")
    return min(complete, key=lambda row: (row["training"]["position_rmse_m"], row["number"]))


def provenance(samples, budget, seed, training, held_out):
    sources = [HERE / "tune.py", HERE / "metrics.py", HERE / "pixi.toml", HERE / "pixi.lock",
               HERE / "test_tuning.py", HERE / "replay_saved.py"]
    core = HERE.parent / "reverse_core"
    sources += sorted(core.glob("*.py")) + sorted(core.glob("native/*"))
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in sources if path.is_file()}
    try:
        revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except subprocess.CalledProcessError:
        revision = None
    return {"experiment": "synthetic Cartesian pose smoothing, v1", "objective": "full-episode pooled position RMSE metres",
            "selection": "training RMSE only; tie order defaults, Optuna, grid",
            "bounds_tau_s": list(BOUNDS_S), "budget_per_search": budget,
            "default_evaluations": 1, "seed": seed, "samples_per_episode": samples,
            "train_seeds": list(TRAIN_SEEDS), "held_out_seeds": list(HELD_OUT_SEEDS),
            "datasets": {"training": [{"episode_id": ep["episode_id"], "sha256": dataset_hash(ep)} for ep in training],
                         "held_out": [{"episode_id": ep["episode_id"], "sha256": dataset_hash(ep)} for ep in held_out]},
            "versions": {name: importlib.metadata.version(name) for name in ("numpy", "cppyy", "pydantic", "optuna")},
            "python": sys.version, "platform": platform.platform(), "git_revision": revision,
            "compiler_version": subprocess.check_output([*shlex.split(os.environ.get("CXX", "c++")), "--version"], text=True),
            "compile_flags": ["-std=c++17", "-O3", "-Wall", "-Wextra", "-Werror"],
            "argv": sys.argv,
            "source_sha256": source_hashes, "lag": {"method": "bounded positive shift of interpolated estimate against truth",
            "resolution_s": 0.005, "maximum_s": 0.3, "settle_s": 0.5, "max_interpolation_gap_s": 0.05,
            "limits": "noise, amplitude distortion and interpolation bias; upper bound is censored; not processing latency"}}


def run(output, budget=BUDGET, seed=SAMPLER_SEED, samples=SAMPLES):
    if budget < 2 or samples < 100:
        raise ValueError("budget must be at least 2 and samples at least 100")
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError("use an empty output directory to preserve prior evidence")
    startup = {"dependency_imports": IMPORT_S}
    started = time.perf_counter()
    paths = build_native()
    startup["native_build_or_cache_lookup"] = time.perf_counter() - started
    started = time.perf_counter()
    native_namespace()
    startup["cppyy_library_and_header_load"] = time.perf_counter() - started
    started = time.perf_counter()
    training = [make_episode(seed=seed, samples=samples) for seed in TRAIN_SEEDS]
    held_out = [make_episode(seed=seed, samples=samples) for seed in HELD_OUT_SEEDS]
    startup["dataset_generation"] = time.perf_counter() - started
    # Warm the binding on training input. Held-out observations never drive selection.
    started = time.perf_counter()
    with PoseFilter() as estimator:
        estimator.process(training[0]["timestamps_ns"], training[0]["positions_m"])
    startup["first_adapter_and_native_process"] = time.perf_counter() - started
    metadata = provenance(samples, budget, seed, training, held_out)
    metadata["native_artifacts"] = {name: str(path) for name, path in paths.items()}
    metadata["startup_s"] = startup
    save_json(output / "provenance.json", metadata)
    experiment_start = time.perf_counter()
    defaults = record_trial(0, FilterConfig(), training)
    save_json(output / "defaults.json", defaults)
    if defaults["status"] != "COMPLETE":
        raise RuntimeError("default evaluation failed; see defaults.json")
    started = time.perf_counter()
    optuna_rows = search(training, budget, seed, output)
    optuna_s = time.perf_counter() - started
    started = time.perf_counter()
    grid_rows = simple_search(training, budget, output)
    grid_s = time.perf_counter() - started
    selected = {"defaults": defaults, "optuna": best_complete(optuna_rows), "grid": best_complete(grid_rows)}
    chosen = min(selected, key=lambda name: selected[name]["training"]["position_rmse_m"])
    results = {"chosen_by_training": chosen, "configurations": {},
               "timing_s": {"optuna_search_total": optuna_s, "grid_search_total": grid_s}}
    # Selection is finished before the first held-out replay.
    save_json(output / "selection.json", {"chosen": chosen, "configurations": selected})
    for name, record in selected.items():
        config = FilterConfig(**record["resolved_config"])
        export_config(config, output / f"{name}.config")
        if load_config(output / f"{name}.config") != config:
            raise AssertionError("export did not round-trip")
        parity = []
        with PoseFilter(config) as estimator:
            for episode in held_out:
                estimator.reset()
                native = estimator.process(episode["timestamps_ns"], episode["positions_m"])
                standalone = driver_replay(config, episode["timestamps_ns"], episode["positions_m"],
                                           output / "driver" / name / episode["episode_id"])
                difference = float(np.max(np.abs(native - standalone)))
                np.testing.assert_allclose(native, standalone, rtol=1e-12, atol=1e-12)
                parity.append({"episode_id": episode["episode_id"], "max_abs_difference_m": difference})
        results["configurations"][name] = {"resolved_config": config.model_dump(), "selected_trial": record["number"],
                "training": record["training"], "held_out": evaluate(config, held_out), "driver_parity": parity}
    export_config(FilterConfig(**selected[chosen]["resolved_config"]), output / "chosen.config")
    results["timing_s"]["experiment_after_startup"] = time.perf_counter() - experiment_start
    results["memory"] = {"process_peak_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
                         "child_peak_rss_mib": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / 1024,
                         "scope": "Linux maximum RSS; includes startup, metrics and storage; child value is maximum, not sum"}
    save_json(output / "results.json", results)
    for name, row in results["configurations"].items():
        print(f"{name}: tau={row['resolved_config']['tau_s']:.9g} s; training RMSE={row['training']['position_rmse_m']:.9g} m; "
              f"held-out RMSE={row['held_out']['position_rmse_m']:.9g} m; alignment lag={row['held_out']['mean_alignment_lag_s']:.6g} s")
    print(f"Chosen from training: {chosen}; evidence: {output}")
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=HERE / "build" / "run")
    parser.add_argument("--budget", type=int, default=BUDGET)
    parser.add_argument("--seed", type=int, default=SAMPLER_SEED)
    parser.add_argument("--samples", type=int, default=SAMPLES)
    args = parser.parse_args()
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    run(args.output, args.budget, args.seed, args.samples)
