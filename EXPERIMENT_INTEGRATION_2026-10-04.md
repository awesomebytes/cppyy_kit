# Experiment integration plan and results

## Objective

Use the reviewed experiments to improve installed cppyy_kit workflows. Keep
domain algorithms in examples or their existing kits. Preserve the experimental
sources and evidence separately from the changes intended for upstream.

## Selected work

| Change | Reason | Acceptance check |
|---|---|---|
| Package the three task guides and existing kit documentation | Installed users need the relevant instructions without a checkout | Read resources from actual Conda artifacts outside the repository |
| Defer native startup for guide and environment commands | Existing package import starts Cling and can install the auto-PCH hook | Block native imports; check commands and normal native API startup |
| Extend `status` with environment diagnostics | Library experiments spent time locating compilers, headers and binary dependencies | Exercise missing files, compiler launchers, bounded version probes and installed usage |
| Coordinate downloaded headers and compiled cache entries | Existing helpers write shared files before publishing complete artifacts | Concurrent cold calls, interrupted operations, retries and offline reuse |
| Validate fetched paths and archive entries | Existing extraction accepts paths outside its destination | Offline malicious tar/zip fixtures and valid archive checks |
| Include library directories in compile identity | They determine linked libraries and runtime search paths | Distinct directories produce distinct cache entries |
| Fix PCL and OpenCV buffer boundaries | Review reproduced unaligned PCL input, undersized image storage and incorrect image depth conversion | Actual native coordinate/image comparisons, padding, views and ownership checks |
| Add native-component and OMPL callback tutorials | Existing APIs already support configuration, testing and native extension points | Small native behavioral tests and independent whole-segment geometry checks |

## Sequence

1. Back up the existing main history and the uncommitted guide changes locally.
2. Review all twelve experiments and compare their APIs with the existing kits.
3. Implement the selected changes on `feat/installed-workflows` in this checkout.
4. Run focused regressions, the default suite, lint and the strict documentation
   build. Test optional kits in their Pixi environments so their checks execute.
5. Build local artifacts. Prove core/rclcpp installed behavior and guide resources
   for the complete suite outside the checkout.
6. Review the final diff independently. Commit with the personal email address,
   push a pull request, monitor CI and merge only when checks pass.

## Experiment decisions

| Experiment | Retained now | Deferred |
|---|---|---|
| Validated configuration and native state | Explicit mapping, reset and instance-isolation patterns in a focused tutorial | Demonstration pose estimator and a generic component base class |
| Generated tests | Independent behavioral checks and optional Hypothesis guidance | A framework for generating tests or reducing native crashes |
| Tuning | Trial isolation, equal budgets, held-out evaluation and export guidance | Task-specific Optuna application and a tuning API |
| Typed MCAP | Buffer bounds, exact dtype and timestamp lessons | New extractor limited to selected schemas and uncompressed input; existing rosbag2/CDR APIs remain available |
| Dataset reports | Explicit clocks, matching rules and provenance guidance | Synthetic-filter report application; supported bgr8 input also needs a rendering correction |
| nanoflann | Header discovery, pinning, retained-owner and adapter-build guidance | A new kit or task-specific filtered-index API |
| Buffer interoperability | Alignment, shape, aliasing, empty-input and owner contracts | Generic buffer leases, GPU or cross-framework DLPack support |
| Native workers | Bounded queues, error handling, stop/join guidance using existing helpers | A worker framework; the small arithmetic workload did not beat NumPy |
| C++ extension points | OMPL subclass tutorial, exceptions, lifetime and segment checks | A new callback abstraction |
| Pydantic, oneTBB and xsimd | Validation and measurement guidance | Parallel/SIMD helpers without a repeatable end-to-end benefit |
| Ceres/manif | Guidance for probing template headers and using a compiled adapter | A new kit; standalone dependency constraints and adapter alignment still need work |
| Ruckig | Guidance for retained native composition and conversion costs | A new kit or physical-control integration; current mock tests do not establish controller constraints or timing |

## Evidence boundaries

The report's timings and fresh-agent completions belong to its recorded
experiments. They are not measurements of these integrated workflows. New
validation results will be recorded below after execution.

Review also investigated a reported empty Pydantic-column defect. The exact
zero-row fixture succeeded in its locked Pixi environment, including fields at
nonzero offsets. No Pydantic implementation change is justified by that probe.

Compiler/runtime constraints already select GCC/G++ 14.3.0 and
libgcc/libstdcxx 15.2.0. This integration does not add another version rule.
Environment discovery does not prove binary compatibility or initialize Cling.

Header fetching and build locks protect helper-managed artifacts. They do not
make arbitrary C++ memory access or shared application state safe. Borrowed
buffers must remain alive and must not be reallocated during native use.

## Validation and upstream status

Implementation and independent review are complete. The following local checks
passed in locked Pixi environments:

| Check | Observed result |
|---|---|
| Final default test task | 369 passed, 161 optional skips; 12.95 seconds |
| PCL native adapter | 2 passed; 7.10 seconds |
| OpenCV new and existing vision suites | 40 passed, no skips; 51.18 seconds |
| Full OMPL kit suite | 21 passed; 6.28 seconds |
| Native component example | 17 passed, including standalone driver parity |
| Independent cache/compiler rerun | 25 passed; publication-failure retry included |
| Lint and strict documentation build | Passed; lint 1.55 seconds, documentation build 0.87 seconds |
| Final core/rclcpp artifact build and recipe tests | Passed; 60 seconds |
| Fresh external Pixi installation | Guides, diagnostics, numeric kernels and serialized same-handle publication passed; 8 seconds |
| Complete suite guide resources | 11 actual Conda artifacts, 23 exact byte comparisons and CLI reads outside checkout; 2.5 seconds |
| ROS cache caller regressions | 45 passed; 70.24 seconds |
| Mocked subscription units followed by native regressions | 34 passed with subscription cache disabled; 17.35 seconds; 5 passed with normal cache; 3.50 seconds |

The nine other wrapper artifacts were built with native recipe tests skipped.
Their resource checks establish packaged guide delivery, not native bringup for
every domain kit. PCL, OpenCV and OMPL native behavior was tested separately in
their feature environments.

Independent review corrected writable-lock requirements on valid read-only cache
hits, ordinary `./` archive compatibility, compiler-launcher capability detection,
installed-proof origin checks and same-process retry after a publication error.
A reader completed the native-component workflow using its documentation and
obtained the documented Python and standalone C++ outputs.

CI now executes focused PCL, OpenCV and OMPL native regressions on x86_64,
alongside the existing x86_64/ARM64 core/rclcpp, installed-package and sanitizer
lanes. Optional capability skips in the default environment are not counted as
native validation. Pull-request checks gate upstream merge.

The first pull-request run exposed six ROS callers that omitted library search
paths when calculating artifact names. These callers now use the same options
as compilation. A new core regression checks path consistency across lookup,
prebuild and runtime loading. Two existing subscription unit tests also passed
fake message types into a real native callable template, affecting later tests
in the same process. They now mock that boundary while retaining the signature
and owning-copy assertions. Independent review confirmed both corrections.
The final installed rclcpp artifact contains all six corrected modules.

The first final local run failed an existing 50 ms jitter benchmark threshold
(79.93 Hz versus an expected 500 to 1500 Hz). Its isolated rerun passed, followed
by the complete default suite. The benchmark code and assertions were unchanged.

The 0.4.0 artifacts remain local. Channel publication is a separate release step.
DBoW2 native packaging remains incomplete: its noarch wrapper and guides do not
contain the user-built native library. The API guide and recipe description now
state that requirement. No presentation or experiment files are included.
