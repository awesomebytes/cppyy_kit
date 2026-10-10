"""Complete these three functions. Read README.md and PROMPT.md first."""
import tune


def replay_trial(config, episodes, estimator=None):
    """Match tune.evaluate's result schema. Reset before each whole native batch.

    Pool Cartesian position RMSE by sample count. Report the declared alignment
    lag per episode. Close only instances constructed inside this function.
    """
    raise NotImplementedError("implement native replay, isolation and metrics")


def ask_and_tell(episodes, budget, seed, output):
    """Return and persist complete trial records matching tune.search's schema.

    Use sequential TPESampler(seed, n_startup_trials=min(8,budget)), log tau_s
    bounds tune.BOUNDS_S and SQLite study 'tau_s'. Record failures and tell FAIL.
    Save optuna_trials.json after every trial. Do not use held-out episodes.
    """
    raise NotImplementedError("implement seeded ask/tell and durable trial history")


def select_training(records):
    """Return the complete trial with smallest training position_rmse_m.

    Break equal scores by trial number. Raise RuntimeError if none completed.
    """
    raise NotImplementedError("implement training-only selection")
