# Prototype a planning policy in Python

Run a Python keep-out policy inside OMPL's native RRTConnect planner. Compare
its path and callback cost with the same policy in C++. This lets a developer
change experimental planning rules without rebuilding OMPL.

This example uses the existing `ompl::base::StateValidityChecker` interface.
It does not implement a sensor or estimator mock. The policy operates on
dimensionless 2D planning coordinates. A configurable x bias shifts a circular
keep-out region.

## Setup

These commands require this repository checkout. Run them from its root.
The existing [root manifest](../../../pixi.toml) and
[lock](../../../pixi.lock) supply the `ompl` environment. Local
`cppyy_kit` and `ompl_kit` imports come from its configured `PYTHONPATH`.
This is a checkout demonstration, not an installed-package proof.

```bash
pixi install --locked -e ompl
```

## Run the task

These commands require this repository checkout. Set `CPPYY_KIT_NO_AUTOPCH=1`
so this experiment does not schedule a background PCH build.

```bash
CPPYY_KIT_NO_AUTOPCH=1 pixi run --locked -e ompl python \
  roscon_uk_2026/next_steps/extensions/demo.py \
  --output roscon_uk_2026/next_steps/extensions/build/evidence.json

CPPYY_KIT_NO_AUTOPCH=1 pixi run --locked -e ompl pytest \
  roscon_uk_2026/next_steps/extensions/test_extensions.py -q
```

Expected result: both policies find an exact path. Their path coordinates,
length, validity decisions, and number of planner calls match exactly. The
checker verifies complete line segments against the circle. The final line is:

```text
PASS: independent policy/path checks, native parity, errors, 25 releases
```

The tests also run biases `0`, `0.05`, and `-0.05`, reject nonfinite settings,
check the circle boundary, and verify that the unfinished policy fails.
An additional test reproduces the coarse-resolution crossing and requires
the independent checker to detect it.
Timing values depend on the machine. [RESULTS.md](RESULTS.md) records the
measured run and an initial motion-checking failure.

## Change the Python behavior

Read [GUIDE.md](GUIDE.md) for the exact virtual, ownership, exception, and
shutdown contracts. Edit [policy.py](policy.py) to change the experiment.
The native planner remains the installed `libompl.so` implementation.
The small adapter in [native.hpp](native.hpp) owns the setup, attaches the
existing policy, and collects outputs. It adds no extension interface.

For an agent task, explicitly read [PROMPT.md](PROMPT.md) and supply a copy
of [policy_skeleton.py](policy_skeleton.py). The saved solution is
[policy.py](policy.py). No skill installation or automatic guide discovery
is required. Fresh-agent completion has not been evaluated.
