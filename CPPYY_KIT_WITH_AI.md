# Prototype Python and C++ robotics software with a coding agent

Use Python for inputs, configuration, tests, visualization, and experiment
control. Use cppyy and `cppyy_kit` to call existing C++ libraries or move a
measured operation to C++. Give the agent the task guide, your actual inputs,
and a check on the expected result.

## Choose the task

| What you want to do | Guide | First useful result |
|---|---|---|
| Accelerate a Python operation | `accelerate` | Matching outputs and measured before/after costs |
| Bring a C++ library into a Python experiment | `bring-library` | A real library call with explicit dependencies and ownership |
| Configure, test, or tune existing C++ software | `existing-cpp` | Validated settings and a reproducible native run from Python |

### Set up the current workflow

The numeric annotations and `guide` commands below are new in **0.4.0**. The
[integration record](https://github.com/awesomebytes/cppyy_kit/blob/main/EXPERIMENT_INTEGRATION_2026-10-04.md#validation-and-upstream-status)
records local 0.4.0 artifacts, with channel publication still a separate step.
The 0.4.0 upload stopped before publication;
[0.4.1 is pending the release gates](https://github.com/awesomebytes/cppyy_kit/blob/main/RELEASE_0.4.1_2026-10-10.md).
Published 0.3.x packages do not provide these interfaces. Until 0.4.1 publication,
use this source checkout and its locked Pixi environment.

**Repository checkout commands:** run from the `cppyy_kit` repository root.

```bash
pixi install --locked
pixi run python -m cppyy_kit guide
pixi run python -m cppyy_kit status --environment
```

The first command installs the repository environment. `guide` lists the task
topics and installed kit names. `status --environment` reports the selected
compiler, runtime versions, development headers, and `libcppyy`; it does not
load Cling or prove binary compatibility. Keep compilation and execution inside
Pixi so the selected compiler and libraries belong to the same environment.

Read the relevant instructions before asking an agent to edit code:

```bash
pixi run python -m cppyy_kit guide accelerate
pixi run python -m cppyy_kit guide bring-library
pixi run python -m cppyy_kit guide existing-cpp
pixi run -e ompl python -m cppyy_kit guide ompl_kit api
```

These commands print documentation without loading the native library. After
0.4.1 is published, the same commands can read the packaged guides in a
standalone environment. Install the required kit there first. See
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/) for installed-package setup and
[guide discovery](docs/GUIDES.md) for the command reference. Repository demos
and their feature environments require this checkout.

### Give the agent a concrete request

For a Python operation, replace the paths and workload in this request:

```text
Read `pixi run python -m cppyy_kit guide accelerate`. Work in this checkout.
Profile my pipeline.py on recording.npz using Pixi. Keep its public Python API
and outputs. Run test_pipeline.py before editing. Move one measured operation
to C++ using cppyy_kit, then run the same tests. Report the changed operation,
numerical tolerance, warm operation and total pipeline costs, input conversions,
and first-use compilation separately. Keep the baseline for comparison. Do not
claim a speedup unless the measured operation and complete pipeline support it.
```

For a library task, use `bring-library` and name the library version, headers,
real operation, and expected result. For existing C++ software, use `existing-cpp`
and name the compiled implementation, settings, inputs, and behavioral checks.
Ask for an explicit mapping into existing native settings and a reproducible
run, rather than assuming generated structs match the native API.

## Try three small workflows

### Accelerate an array operation

Save this as `kernel.py` in the repository root:

```python
import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray

@cpp
def sum_sq(data: ConstNDArray[np.float64]) -> float:
    """
    double s = 0;
    for (std::size_t i = 0; i < data_size; ++i) {
        s += data[i] * data[i];
    }
    return s;
    """

print(sum_sq(np.array([1, 2, 3], dtype=np.float64)))
```

Run `pixi run python kernel.py`. It prints `14.0`. The annotation supplies a
`const double*` and the generated `data_size` element count. The first call
compiles the function; subsequent calls use it, and later processes can reuse
the compiled artifact. This tiny example demonstrates the interface, not a
performance gain.

Use `NDArray[np.float64]` for writable native access. Both forms require the
documented dtype, alignment, and layout; they do not silently normalize
unsupported arrays. Normalize explicitly when a copy is acceptable, and include
that cost in the comparison. Read [numeric arguments](docs/COMMON_PATTERNS.md)
before adapting this to images, tensors, or point clouds. `@cpp(nogil=True)`
releases the interpreter lock around the C++ body; independent ownership and
thread-safe native work are still required.

### Call an existing library

Save this as `tree.py` in the repository root:

```python
import bt_kit

bt = bt_kit.bringup_bt()
factory = bt.BehaviorTreeFactory()
tree = factory.create_tree_from_text("""
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree"><AlwaysSuccess/></BehaviorTree>
</root>
""")
print(tree.tickWhileRunning() == bt.NodeStatus.SUCCESS)
```

Run `pixi run -e bt python tree.py`. It prints `True`. BehaviorTree.CPP owns
and ticks the tree; Python configures and calls the engine. There is no separate
per-method Python binding file in this example. See [bt_kit](bt_kit/WHY.md) for
Python actions and conditions, or the [OMPL callback tutorial](docs/tutorials/ompl_callbacks.md)
for a native planner with a Python validity checker.

For a library without a kit, cppyy can load its shared library and include
compatible headers. The `bring-library` guide covers environment discovery,
version pinning, `require()` for fetched headers, and compiled adapters for
headers that Cling cannot parse. A kit or adapter still needs the native
dependencies and a defined ownership contract.

### Configure and test an existing C++ component

Run the included smoother from the repository root:

```bash
pixi run python examples/native_component/component.py
pixi run python -m pytest examples/native_component/test_component.py -q
```

The script prints:

```text
[0.0, 1.0, 2.5]
{'initialized': True, 'value': 2.5, 'samples': 3}
[4.0]
{"alpha": 0.5}
```

The tests check independent expected outputs, reset, rejected inputs, instance
isolation, cleanup, and agreement with a standalone C++ driver. Python maps
validated settings into the existing C++ type. One retained native instance
holds the state across batches. The [complete tutorial](docs/tutorials/native_component.md)
shows the source/declaration split, settings export, driver command, and optional
test-generation or tuning experiments. It claims no speedup for this small
arithmetic example.

## Build an example around real robotics inputs

The [ROSCon deep dive](https://github.com/awesomebytes/cppyy_kit/blob/main/roscon_uk_2026/DEEP_DIVE_PRESENTATION.md) follows motion
detection, recorded-data queries, ROS callbacks, a mock controller, and webcam
tracking. Its [evaluation record](https://github.com/awesomebytes/cppyy_kit/blob/main/roscon_uk_2026/EVALUATION.md) preserves inputs,
acceptance checks, agent attempts, and original environment conditions.

| Recorded operation | Python | Native | Scope |
|---|---:|---:|---|
| Motion mask, 250,000 Cartesian observations | 29.28 ms | 1.48 ms | Median of seven warmed calls |
| Recorded-data threshold sweep, 2,735 observations and 41 thresholds | 24.75 ms | 1.14 ms | Query after decoding, two hands |
| ROS replay callback | 2.835 ms | 0.497 ms | Median callback execution |

These are historical measurements from the recorded 0.3.0 environment. They
are not new 0.4.0 benchmarks. The query excludes MCAP loading and decoding;
the callback timing excludes transport and complete end-to-end delay. Failed
agent attempts and environment repairs remain part of the evidence. Fresh-agent
completion is useful evidence for that scaffold, not a general reliability rate.

The [nanoflann experiment](https://github.com/awesomebytes/cppyy_kit/blob/main/roscon_uk_2026/next_steps/nanoflann/RESULTS.md)
demonstrates a second useful route: retain an index in C++ and combine search,
metadata filtering, and centroid computation in one call. With 50,000 indexed
positions, 2,000 queries, and k=8, the warmed native call measured **3.19 ms**
against **31.49 ms** for the adaptive SciPy composition. Direct unfiltered
search was much closer: native with centroid **1.95 ms**, SciPy without centroid
**2.32 ms**. The benefit belongs to the measured composition. The report records
copies, build/import costs, dependencies, and independent brute-force checks.
It is a checkout experiment, not an installed-package proof.

For ML work, use these routes for measured preprocessing, native algorithms,
and Python/C++ integration around the model. These examples do not establish
faster model inference, GPU interoperability, or zero-copy transfer for arbitrary
framework tensors.

With `rclcppyy`, select and measure the relevant explicit native profile or fused
operation. Its default compatibility profile preserves stock `rclpy` behavior.
Enabling that default alone is not a performance result. See its
[backend selection guide](https://github.com/awesomebytes/rclcppyy/blob/main/docs/backend-selection.md).

## What the agent should verify

Start from representative inputs and an existing functional or integration test.
Move one measured operation, then compare outputs exactly or with a stated
numerical tolerance. Check empty inputs, layouts, read-only access, lifetime,
and repeated calls where those conditions belong to the API. Keep a reference
implementation so later changes can repeat the comparison.

Measure the complete operation including conversions, allocation, and copies.
Report startup, first-use compilation, warm calls, total pipeline cost, and agent
elapsed time separately. Include a maintained native Python binding as a control
when one already performs the same computation. Record versions, workload,
command, repeat count, and units. [Benchmark reports](docs/benchmarks.md) and
the [webcam report](docs/webcam_demo/REPORT.md) show this reporting style.

For retained native objects, define the owner and cleanup. Keep borrowed arrays
and callbacks alive and do not reallocate borrowed storage. Stop and join workers
before releasing owners. Buffer validation cannot make arbitrary C++ memory
access safe; tests must cover the actual boundary used by the application.

## Why cppyy_kit helps when AI writes code

An agent can write C++, Python, build files, and bindings. The compiler, ABI,
conversion, packaging, and lifetime decisions still exist. cppyy exposes compatible
C++ APIs at runtime; the kits handle library setup and recurring boundary tasks.
For `@cpp`, the call site, annotations, and ordinary C++ body are visible in one
Python file. This reduces the separate artifacts an experiment must coordinate
and lets the agent test native work with inputs from the existing Python workflow.

Caching can reduce repeated compilation, and automatic PCH can reduce supported
header-parsing costs. Neither removes the compiler/runtime dependencies or every
first-use cost. See [cache and PCH behavior](docs/FREEZE.md).

A useful prototype can later become a maintained C++ library or extension.
Extraction still needs explicit parameters for generated sizes such as
`data_size`, headers, dependencies, a stable API, bindings or a C ABI, and
packaging. It is not automatic application compilation.

## Choose another tool when its model fits

| Route | Useful fit | Work to account for |
|---|---|---|
| Numba, Pythran, or Cython | Supported Python-style numerical computation | Supported types and language subset; compilation and packaging depend on the tool |
| JAX or Taichi | Array or kernel workloads that fit their compiler and execution model | Tracing or kernel semantics, specialization, and compilation costs |
| pybind11, nanobind, Cython, or CPython C API | A deliberate, stable Python API for native code | Binding declarations, extension builds, and maintained packaging |
| `ctypes` | A small existing C-compatible ABI | Exported C functions, signatures, conversions, and ownership |
| cppyy with cppyy_kit | Ordinary C++ libraries or focused native work in a Python experiment | Compatible runtime, headers, binaries, first-use costs, and ownership |

Choose using the actual workload and deployment requirements. Keep the simpler
baseline when native boundary and startup costs outweigh the measured gain.
