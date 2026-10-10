# Exact exercise prompt

Tune this native estimator on the training recordings within this trial budget.
Compare the defaults on held-out recordings and export the settings.

Read `README.md`, `../reverse_core/CONTRACT.md`, and `skeleton.py`. These inputs
are synthetic Cartesian episodes with independent truth, not recorded sensor
data. Implement the three missing functions in `skeleton.py`. Use Optuna
ask-and-tell and the shared native `PoseFilter`. Reset native state for every
episode. Keep each episode replay in one native batch. Use the fixed bounds,
seed, and trial budget declared in `tune.py`. Persist all trials, including
failures. Select configurations using training RMSE only. Preserve the declared
alignment metric and its limits. Run `check_solution.py skeleton.py` and the
independent checks. Do not modify checks or shared native files. Use Pixi in this
checkout. Do not install skills or change agent settings.

Then run the complete experiment with the repaired functions connected to
`tune.py`. Evaluate defaults, the Optuna selection, and the equal-budget grid
selection on held-out seeds. Export the selected configuration and verify its
standalone C++ replay within `1e-12 m` absolute and `1e-12` relative tolerance.
Save commands, results, dependency versions, failures and any manual repairs.
Report setup, native build/load, replay/adapter, metric and search time separately.
Do not claim that optimization must improve results.
