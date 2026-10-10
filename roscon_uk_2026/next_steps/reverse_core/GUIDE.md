# Validate and replay pose-filter settings

Validate settings in Python, filter timestamped Cartesian positions in compiled C++, and reproduce the output with a standalone C++ executable. This catches configuration errors before the native component is constructed. One resolved settings file serves both entry points.

This experiment provides a separately compiled demonstration estimator. It is written for this repository. It is not a pre-existing external production estimator. Its public header and standalone driver also work without Python.

## Setup

The following commands require this repository checkout. Run them from its root:

```bash
pixi install --locked --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml
cat roscon_uk_2026/next_steps/reverse_core/GUIDE.md
pixi run --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml demo
```

The directory owns its Pixi manifest and lock. Activation sets `PYTHONPATH` to the checkout root so imports use the local `cppyy_kit`. This is a repository example, not an installed-package recipe. No global pip installation is required.

The demo creates the library and executable under ignored `build/`, generates 500 synthetic samples, writes `resolved.cfg` and CSV files under `build/demo/`, and checks exact agreement between cppyy and the driver. Expected deterministic fields include:

```text
"episode_id": "synthetic-pose-v1-seed-0-n-500"
"input_position_rmse_m": 0.08259891457920916
"filtered_position_rmse_m": 0.07207672638703862
"driver_max_abs_difference_m": 0.0
"gap_resets": 1
```

Timing fields vary by machine and process state. This one synthetic episode is not evidence of improvement on real sensors or held-out data.

## Use the filter

This example requires the checkout. Run it with the same Pixi manifest:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml python - <<'PY'
from roscon_uk_2026.next_steps.reverse_core import (
    FilterConfig, PoseFilter, export_config, make_episode,
)

config = FilterConfig(tau_s=0.08, max_gap_s=0.5, frame_id="world")
episode = make_episode(seed=0, samples=500)
with PoseFilter(config) as estimator:
    output_m = estimator.process(episode["timestamps_ns"], episode["positions_m"])
    print(output_m.shape)
    print(estimator.snapshot()["samples_processed"])
export_config(config, "roscon_uk_2026/next_steps/reverse_core/build/example.cfg")
PY
```

Expected output is `(500, 3)` followed by `500`. Input timestamps are native `int64` nanoseconds from one clock. Positions and output are native `float64` metres with three Cartesian coordinates per sample. Inputs must use the configured frame. The filter neither transforms frames nor infers a timestamp clock.

Both time settings are finite positive seconds. `max_gap_s` must be at least `tau_s`. Unknown fields, strings used for numbers, booleans, and invalid frame names fail validation. Python integers are accepted for the floating time fields. The model uses [Pydantic strict validation](https://pydantic.dev/docs/validation/latest/concepts/strict_mode/) and explicit validators. There is one explicit adapter from this validated model to the existing native configuration fields:

| Python field | Native `reverse_demo::Config` field | Meaning |
|---|---|---|
| `tau_s` | `time_constant_s` | Smoothing time constant, seconds |
| `max_gap_s` | `reset_gap_s` | Reset gap threshold, seconds |
| `frame_id` | `output_frame` | Caller-declared Cartesian frame |

The first output equals the input. Later outputs use causal exponential smoothing. A gap strictly greater than `max_gap_s` resets to the new input. A repeated or decreasing timestamp rejects the complete batch without updating state. `reset()` clears history and counters. `close()` releases the owning native object. Context-manager exit calls `close()`.

Aligned contiguous inputs are borrowed for one synchronous call. Noncontiguous or unaligned arrays are copied. Read-only arrays are accepted. The returned NumPy array owns its storage and remains valid after filter closure. See [CONTRACT.md](CONTRACT.md) for exact shapes, exceptions, empty-input behavior, and the single-threaded instance rule.

## Reproduce settings in C++

`export_config` saves all resolved defaults and supplied fields in a fixed four-line format:

```text
pose_filter_config_v1
tau_s=0.080000000000000002
max_gap_s=0.5
frame_id=world
```

The format is positional and versioned. It supports these three fields only. The C++ driver does not parse JSON. `FilterConfig.model_dump_json()` and `FilterConfig.model_validate_json()` support ordinary Pydantic JSON round-trips separately.

`build_native()` returns the exact `driver` path. The executable takes three positional arguments:

```text
pose_filter_driver CONFIG.cfg INPUT.csv OUTPUT.csv
```

The input and output CSV header is `timestamp_ns,x_m,y_m,z_m`. Floating fields use round-trip precision. The driver prints `samples=500 frame_id=world` for the default episode. `driver_replay` handles file export and execution for Python callers. The demo prints the absolute executable path and saves the result in `build/demo/results.json`.

## Check the example and exercise

These commands require the checkout:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml python -m roscon_uk_2026.next_steps.reverse_core.check_exercise
```

The saved solution should print `PASS` for validation, mapping, default export, and native-loader replay. To complete the small exercise, use the exact prompt in [AGENT_PROMPT.md](AGENT_PROMPT.md), fill [skeleton.py](skeleton.py), and select it with `check_exercise --module roscon_uk_2026.next_steps.reverse_core.skeleton`. The skeleton intentionally fails until implemented. No skill installation or agent-setting change is part of this workflow.

[RESULTS.md](RESULTS.md) records measured checks, costs, toolchain failures, and limits. [MAINTAINER.md](MAINTAINER.md) describes native build and loading details. Training seeds and held-out seeds are declared in [CONTRACT.md](CONTRACT.md) for the tuning and report consumers.
