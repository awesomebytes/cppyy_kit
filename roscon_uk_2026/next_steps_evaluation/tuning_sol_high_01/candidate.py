"""Complete these three functions. Read README.md and PROMPT.md first."""
import tune


def replay_trial(config, episodes, estimator=None):
    """Match tune.evaluate's result schema. Reset before each whole native batch.

    Pool Cartesian position RMSE by sample count. Report the declared alignment
    lag per episode. Close only instances constructed inside this function.
    """
    owned = estimator is None
    if owned:
        estimator = tune.PoseFilter(config)
    rows = []
    squared_error = 0.0
    samples = 0
    replay_s = 0.0
    metric_s = 0.0
    try:
        for episode in episodes:
            started = tune.time.perf_counter()
            estimator.reset()
            estimate = estimator.process(
                episode["timestamps_ns"], episode["positions_m"]
            )
            replay_s += tune.time.perf_counter() - started

            started = tune.time.perf_counter()
            truth = episode["truth_m"]
            rmse = tune.position_rmse(estimate, truth)
            count = len(truth)
            squared_error += float(tune.np.sum((estimate - truth) ** 2))
            samples += count
            alignment = tune.alignment_lag(
                episode["timestamps_ns"].astype(tune.np.float64) * 1e-9,
                estimate,
                truth,
            )
            rows.append({
                "episode_id": episode["episode_id"],
                "samples": count,
                "position_rmse_m": rmse,
                "alignment": alignment,
            })
            metric_s += tune.time.perf_counter() - started
    finally:
        if owned:
            estimator.close()
    if not samples:
        raise ValueError("expected at least one nonempty episode")
    return {
        "position_rmse_m": float(tune.np.sqrt(squared_error / samples)),
        "mean_alignment_lag_s": float(tune.np.mean([
            row["alignment"]["lag_s"] for row in rows
        ])),
        "samples": samples,
        "episodes": rows,
        "timing_s": {"native_replay": replay_s, "metric": metric_s},
    }


def ask_and_tell(episodes, budget, seed, output):
    """Return and persist complete trial records matching tune.search's schema.

    Use sequential TPESampler(seed, n_startup_trials=min(8,budget)), log tau_s
    bounds tune.BOUNDS_S and SQLite study 'tau_s'. Record failures and tell FAIL.
    Save optuna_trials.json after every trial. Do not use held-out episodes.
    """
    output = tune.Path(output)
    sampler = tune.optuna.samplers.TPESampler(
        seed=seed, n_startup_trials=min(8, budget)
    )
    study = tune.optuna.create_study(
        study_name="tau_s",
        storage=f"sqlite:///{output / 'study.sqlite3'}",
        sampler=sampler,
        direction="minimize",
    )
    records = []
    for _ in range(budget):
        trial = study.ask()
        tau_s = trial.suggest_float("tau_s", *tune.BOUNDS_S, log=True)
        config = tune.FilterConfig(tau_s=tau_s)
        record = {
            "number": trial.number,
            "resolved_config": config.model_dump(),
            "optuna_params": dict(trial.params),
            "status": "RUNNING",
        }
        try:
            record["training"] = replay_trial(config, episodes)
        except Exception as exc:
            record.update(
                status="FAIL",
                error_type=type(exc).__name__,
                error=str(exc),
                traceback=tune.traceback.format_exc(),
            )
            study.tell(trial, state=tune.optuna.trial.TrialState.FAIL)
        else:
            record["status"] = "COMPLETE"
            study.tell(trial, record["training"]["position_rmse_m"])
        records.append(record)
        tune.save_json(output / "optuna_trials.json", records)
    return records


def select_training(records):
    """Return the complete trial with smallest training position_rmse_m.

    Break equal scores by trial number. Raise RuntimeError if none completed.
    """
    complete = [row for row in records if row["status"] == "COMPLETE"]
    if not complete:
        raise RuntimeError("no complete training trials")
    return min(complete, key=lambda row: (
        row["training"]["position_rmse_m"], row["number"]
    ))
