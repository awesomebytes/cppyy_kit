# Current source rehearsal

Validated on 10 October 2026. These commands require this repository checkout.
The recorded checks used suite 0.4.0 source at `e792866`. The rehearsal activates
source through Pixi
`PYTHONPATH`. It installs native dependencies and compiler tooling, without an
installed `cppyy-kit` or domain-kit package. It does not demonstrate a published
0.4.1 installation. The separate [release record](https://github.com/awesomebytes/cppyy_kit/blob/main/RELEASE_0.4.1_2026-10-10.md) documents
public-channel verification and a standalone installed-package proof.

From `roscon_uk_2026/`:

```bash
pixi install --locked
pixi run --frozen python -m cppyy_kit status --environment
pixi run --frozen python -m cppyy_kit guide accelerate
pixi run --frozen python -m cppyy_kit guide bring-library
pixi run --frozen python -m cppyy_kit guide existing-cpp
pixi run --frozen python -m cppyy_kit guide control_kit api
pixi run --frozen check
pixi run --frozen check-acceptance
pixi run --frozen verify
```

`solutions/` contains the current runnable solutions. All four inline kernels
now use `ConstNDArray[np.float64]` or `ConstNDArray[np.uint8]` for read-only
inputs, and `NDArray[np.int32]` or `NDArray[np.float64]` for mutable outputs.
The checked annotation borrows an existing array with the exact dtype, native
byte order, alignment, and C-contiguous layout. The existing Python wrappers
still normalize incompatible inputs, with possible conversion costs. Shape,
unit, and mathematical validation remain in those wrappers. The generated C++
algorithm bodies are unchanged.

## Validation

The frozen environment discovers cppyy 3.5.0, GCC/G++ 14.3.0, and
libgcc/libstdcxx 15.2.0. Source discovery resolves `cppyy_kit` to this checkout.
Core task guides and the control, ROS, OpenCV, and OMPL API guides are readable
without importing their native libraries.

| Check | Result |
|---|---|
| `pixi run --frozen check` | 6 passed, including original source hashes, exact presentation prompts, read-only input parity, and independent tracker search |
| `pixi run --frozen check-acceptance` | 35 passed: Python baseline 12, migrated kernel 12, MCAP 5, ROS window query 2, synthetic tracker 4 |
| `pixi run --frozen verify` | Python-oracle parity across 30 random windows and three hold values; recorded MCAP query parity for 2,735 observations |
| ROS replay, baseline and native | Both received all 2,735 observations; both count hashes match the offline sweep |
| Mock ros2_control acceptance | 1 passed: 1,500 cycles with actual GenericSystem/controller-manager execution and tail tracking error below 0.025 rad |
| nanoflann current-source acceptance | 64 independent oracle comparisons passed, including filtering, ties, owned lifetime, empty inputs, and repeated teardown |

ROS checks used Jazzy, CycloneDDS, and separate unused domains. Run them
sequentially from the rehearsal directory:

```bash
pixi run --frozen -e ros python scripts/check_replay.py --baseline --domain-id 177
pixi run --frozen -e ros python scripts/check_replay.py --domain-id 178
ROS_DOMAIN_ID=179 pixi run --frozen -e control python scripts/check_acceptance.py --control
```

The nanoflann example has its own source-activated environment:

```bash
pixi run --frozen --manifest-path next_steps/nanoflann/pixi.toml check
```

These are correctness and migration checks on the local machine. Numerical and
callback timings from these runs are saved under ignored
`build/current-checkout/`; they are not replacement presentation benchmarks.
Rehearse timing measurements sequentially under the documented workload and
cache conditions before adding a new comparison to the talk. No new agent
evaluation or physical-camera trial was run for this migration.

## Historical evidence and new runs

The [original evaluated sources, environment lock, and kernel guide](solutions/evaluated_2026_10_03/README.md)
are preserved byte-for-byte. Their six solution hashes match
[`evaluation/provenance.json`](evaluation/provenance.json). The original
`evaluation/` and `next_steps_evaluation/` reports and measurements were not
rewritten. The original manifest is archival, with task paths relative to its
original rehearsal location.

[`current_rehearsal_provenance.json`](current_rehearsal_provenance.json) identifies
the migrated solution bytes and current environment locks. New numerical and
replay reports include those hashes and the package source path. Their defaults
write to `build/current-checkout/`; the scripts reject output paths inside the
historical evaluation directories.

The nanoflann historical measurement JSON remains intact. Its current lock
describes the migrated rehearsal; an exact original experiment lock snapshot
was not retained. See [the dependency distinction](next_steps/nanoflann/RESULTS.md).

`scripts/run_agent_eval.py` now preserves the explicit source activation and
records package provenance. Each fresh run uses its own compilation and
auto-PCH cache inside its run directory. New run IDs are required. This differs
from the historical installed-package evaluation and its cache conditions.
