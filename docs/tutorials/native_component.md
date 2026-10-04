# Configure and test a C++ component from Python

Keep a C++ smoother alive across batches, validate its settings in Python, and
check reset and rejected inputs against independent expectations. This small
checkout example uses cppyy, the existing compile cache, and pytest. It requires
no ROS API, Pydantic, Hypothesis, or Optuna.

Run these commands from the repository root in its default Pixi environment:

```sh
pixi install --locked
pixi run python examples/native_component/component.py
pixi run python -m pytest examples/native_component/test_component.py -q
```

The example prints:

```text
[0.0, 1.0, 2.5]
{'initialized': True, 'value': 2.5, 'samples': 3}
[4.0]
{"alpha": 0.5}
```

The first run compiles the implementation and loads its shared library. Later
runs reuse the library when the cache inputs match. No speedup is claimed for
this small arithmetic example.

## Map settings into an existing native type

[component.py](https://github.com/awesomebytes/cppyy_kit/blob/main/examples/native_component/component.py) provides a frozen
`Settings` dataclass. Its dimensionless `alpha` must be a finite number in
`(0, 1]`. Booleans and strings are rejected. The adapter rechecks settings before
loading native code, then assigns `alpha` to `Config.smoothing_weight` explicitly.
The C++ constructor checks its own settings for callers that do not use Python.

Save resolved settings with ordinary Python serialization:

```python
from dataclasses import asdict
import json
from pathlib import Path

settings = Settings(alpha=0.25)
Path("settings.json").write_text(json.dumps(asdict(settings)), encoding="utf-8")
restored = Settings(**json.loads(Path("settings.json").read_text(encoding="utf-8")))
```

Run this fragment after importing `Settings` from `component.py`, or from this
example's directory with `from component import Settings`.

For a larger schema, Pydantic can validate defaults and relationships between
fields. It remains an optional dependency. `pydantic_structs` creates new native
structs from supported models; an existing C++ configuration still needs an
explicit field mapping.

## Keep state and define cleanup

The first sample becomes the output. Later samples update it with
`output += alpha * (input - output)`. Inputs must be finite numbers in
`[-1e6, 1e6]`, which bounds the subtraction used by this demonstration. Choose
units and application limits before adapting this rule to a sensor.

One C++ instance holds a value, a flag, and a sample counter. It retains no input
history. `process` copies inputs into a native vector and returns a new Python
list. Each complete batch is checked before state changes. An empty batch leaves
state unchanged. Splitting a sequence across batches preserves the result.

Use the wrapper from its example directory:

```python
from component import Settings, Smoother

with Smoother(Settings(alpha=0.5)) as smoother:
    print(smoother.process([0.0, 2.0]))
    print(smoother.process([4.0]))
    smoother.reset()
    print(smoother.process([4.0]))
```

Expected lists are `[0.0, 1.0]`, `[2.5]`, and `[4.0]`. `reset` clears the value and
counter. Context exit calls idempotent `close`, which releases the owning cppyy
proxy. Operations after close raise `RuntimeError`. Returned lists remain valid.
Each instance is used from one thread. This example creates no native worker.

For components that own background work, stop and join before dropping their
last owner. Existing `nogil` accepts a native nullary callable for blocking waits;
`keep_alive` retains borrowed owners and callbacks. `register_teardown` and
`shutdown` provide ordered cleanup for process resources. Retention alone does
not prevent a borrowed container from reallocating its storage.

## Keep implementation separate from declarations

[smoother.hpp](https://github.com/awesomebytes/cppyy_kit/blob/main/examples/native_component/smoother.hpp) declares the C++
class. [smoother.cpp](https://github.com/awesomebytes/cppyy_kit/blob/main/examples/native_component/smoother.cpp) defines its
methods without Python dependencies. The loader passes the implementation and
header contents to `prebuild`, then gives `cppdef_cached` the same definitions
plus bodiless declarations. Header edits participate in the cache input.
Cling sees the declarations while calls execute the compiled implementation.

With an already built external library, use its normal build instead:
`cppyy.load_library` loads the shared library and `cppyy.include` exposes its
compatible public header. Read `pixi run python -m cppyy_kit guide existing-cpp`
for that workflow. Loading a binary does not compile its source.

The included [driver.cpp](https://github.com/awesomebytes/cppyy_kit/blob/main/examples/native_component/driver.cpp) runs the
same class without Python. From this checkout, compile and run it inside Pixi:

```sh
pixi run python - <<'PY'
from pathlib import Path
import subprocess
from cppyy_kit._compile import compiler_command

directory = Path("build/native_component")
directory.mkdir(parents=True, exist_ok=True)
subprocess.run(compiler_command() + [
    "-std=c++17", "-O2", "examples/native_component/smoother.cpp",
    "examples/native_component/driver.cpp", "-o", str(directory / "smoother"),
], check=True)
PY
pixi run build/native_component/smoother 0.5 0 2 4
```

The checkout's internal compiler helper parses the activated environment's `CXX`,
including a compiler launcher and its arguments. The driver prints `0`, `1`, and
`2.5`, one per line. It accepts the resolved alpha as its first argument; it does
not parse the Python JSON file. The test restores
that JSON in Python, supplies its alpha to the driver, and compares both outputs.

## Test behavior and optional experiments

[test_component.py](https://github.com/awesomebytes/cppyy_kit/blob/main/examples/native_component/test_component.py) compares
native outputs with a Python weighted-sum reference. It checks chunk boundaries,
empty inputs, repeated resets, independent instances, output lifetime, cleanup
after an exception, invalid configuration before native loading, and complete
state preservation when a batch ends with invalid data. Separate checks reach
the C++ constructor and standalone driver. These finite checks do not prove
arbitrary C++ memory safety or thread safety.

Hypothesis can extend these checks with finite bounded sample lists and operation
sequences. Compare replay after reset with a fresh instance, and save reduced
failures as explicit data in ordinary regression tests. Put native crash probes
in subprocesses with deadlines; a crash cannot be reduced as a Python assertion.

Optuna can propose alpha values around complete native replays. Reset before
every episode, keep training and held-out inputs separate, and select using only
training error against independent ground truth. Give a simple grid the same
trial budget. Record failed trials, resolved settings, data identities, versions,
and metrics; export the training-selected setting before held-out evaluation.
Compare it with the default and replay it through the C++ driver. This tutorial
does not supply sensor truth or claim tuning improves real measurements.
