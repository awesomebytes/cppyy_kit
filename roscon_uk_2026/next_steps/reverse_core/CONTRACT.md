# Shared pose-filter contract

Import from `roscon_uk_2026.next_steps.reverse_core`. Commands require this checkout with its root on `PYTHONPATH`. This is a separately compiled demonstration library written for these experiments. It is not a pre-existing external production estimator.

## Configuration

`FilterConfig(tau_s=0.08, max_gap_s=0.5, frame_id="world")` is a frozen Pydantic 2 model. Both times are finite positive seconds. `max_gap_s >= tau_s`. Numbers accept Python integers or floats; strings and booleans are rejected. `frame_id` matches `[A-Za-z][A-Za-z0-9_/]*` and has at most 128 characters. Unknown fields are rejected. `PoseFilter(config=None)` accepts this model or a mapping validated before native loading or construction. The explicit adapter maps seconds to C++ `Config.time_constant_s`, `Config.reset_gap_s`, and the frame string to `Config.output_frame`.

`export_config(config, path)` writes a resolved UTF-8 text configuration with exactly four lines: `pose_filter_config_v1`, `tau_s=<float>`, `max_gap_s=<float>`, `frame_id=<name>`. `load_config(path)` round-trips through Python validation. The standalone C++ driver reads this documented fixed format. JSON export may use Pydantic's standard JSON serialization, but the native driver does not parse JSON.

## Filter

`PoseFilter.process(timestamps_ns, positions_m)` requires NumPy arrays with native `int64` dtype and shape `(N,)`, and native `float64` dtype and shape `(N, 3)`. Positions are finite Cartesian metres in `frame_id`. Noncontiguous or unaligned arrays are copied to aligned C-contiguous temporaries. Read-only aligned contiguous arrays are accepted. Inputs are borrowed synchronously and are never retained or changed. The result is a new owning C-contiguous `float64[N,3]` NumPy array.

Timestamps are signed nanoseconds from one caller-chosen clock. They must strictly increase within and across calls. For first input after construction/reset, output equals input. Subsequent samples use `alpha = -expm1(-dt_s/tau_s)` and `y += alpha*(x-y)`. If `dt_s > max_gap_s`, output resets to input. Equality with the gap threshold smooths. Differences use extended precision before conversion to seconds to avoid signed int64 overflow. Every batch is validated completely before any native state change. Repeated/decreasing timestamps or nonfinite values raise `ValueError`; type/dtype errors raise `TypeError`; shape errors raise `ValueError`. Invalid batches preserve state. Empty correctly typed/shaped batches return `(0,3)` without state change.

`reset()` clears history and counters. `snapshot()` returns a plain dict with `initialized` (bool), `last_timestamp_ns` (int or None), `filtered_position_m` (three-element list or None), `samples_processed` (int), `gap_resets` (int), and `config` (resolved dict with the three public fields). State storage is constant-size. No sample history is retained.

`close()` releases the native instance and is idempotent. Context-manager exit closes. `process`, `reset`, `snapshot`, and context entry after close raise `RuntimeError`. Each instance is single-threaded; concurrent calls on one instance are unsupported. Separate instances are independent.

## Replay and generated episodes

`make_episode(seed=0, samples=500)` returns `timestamps_ns`, `positions_m`, `truth_m`, and `episode_id`. These are deterministic synthetic observations with varied trajectories and seeded noise. They are not sensor recordings. Episode IDs include generator version, seed, and sample count. Timestamps have variable intervals and one deliberate long gap for episodes with at least 100 samples. Training seeds are `(0, 1, 2, 3)` and held-out seeds are `(100, 101, 102, 103)`. Consumers must keep held-out seeds out of parameter selection.

`write_episode_csv(episode, path)` writes `timestamp_ns,x_m,y_m,z_m` and rows with round-trip floating precision. `driver_replay(config, timestamps_ns, positions_m, work_dir)` exports config/input under `work_dir`, runs the standalone driver, and returns an owning `(N,3)` output array. `build_native()` returns paths named `library`, `driver`, and `include_dir`; builds reside under this directory's ignored `build/`. `native_namespace()` lazily loads the compiled library and public header through cppyy/cppyy_kit. The public C++ namespace is `reverse_demo`; `Config`, `Snapshot`, and `PoseEstimator` are usable directly from C++.

There is no faulty fixture in reverse_core. Generated tests own any deliberate faulty wrapper and label it as a test fixture.
