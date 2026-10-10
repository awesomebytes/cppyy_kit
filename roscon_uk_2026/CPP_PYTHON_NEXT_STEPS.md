# Possible next steps for Python and C++ integration

Recorded on 4 October 2026. The selected directions below now have bounded
implementation experiments. They are not general compatibility guarantees or
released kits. The existing five-stage presentation sequence remains in
[PLAN.md](PLAN.md). Its measured results are in [EVALUATION.md](EVALUATION.md).
New implementation results and fresh-agent checks are in
[next_steps/EVALUATION.md](next_steps/EVALUATION.md).

Status reviewed on 10 October 2026 against `9449f51`. The subsequent
[integration report](../EXPERIMENT_INTEGRATION_2026-10-04.md) records delivered
base and per-kit guides, environment diagnostics, cache coordination, buffer
corrections, and native-component/OMPL tutorials. Local 0.4.0 Conda artifact
checks passed; channel publication is still a separate release step. The dated
experiment results below are not new measurements of that integration.

The objective is to make useful native components easier to create, combine,
inspect, and supervise from Python. Evaluate both directions: C++ improving
Python work, and Python reducing the work around existing C++.

## Implementation status

The [implementation plan](IMPLEMENTATION_PLAN.md) assigned twelve GPT-6.1 Sol
high agents separate scopes. The [experiment overview](next_steps/README.md)
provides runnable commands, guides, prompts, and result links.

| Direction | Implemented and checked scope | Remaining scope |
|---|---|---|
| Stateful components | Compiled pose filter with reset, bounded state, batch/chunk equivalence and validation | Streaming bimanual event detector and general ergonomic component helpers |
| Buffers | NumPy/Eigen views, explicit copies, retained owners, CPU NumPy DLPack sharing | Cross-framework tensors, GPU streams, reusable cross-kit APIs |
| Typed MCAP | PoseStamped and rgb8/bgr8 Image extraction, Python decoder and rosbag2 comparisons, public image slice | Arbitrary ROS schemas, native compressed inputs, Arrow exchange |
| Ceres/manif | Observable rigid transform, custom autodiff residual, direct JIT and compiled adapter, equivalent SciPy objective | Time offsets, larger calibration tasks, existing Python-binding performance comparison |
| Ruckig | Persistent 3/6 DoF generation, native tracking, official-binding parity, mock control | Physical dynamics and wall-clock real-time validation |
| nanoflann | New-library recipe, retained exact filtered queries and centroids, independent oracle and SciPy comparison; upstream coordination for helper-managed header/build caches | Shorter live-demo scaffold and explicit validation of concurrent cold experiment initialization |
| oneTBB/xsimd | Separate native paths over validated Pydantic data, checked counts and conversion costs | Workload with a repeatable end-to-end advantage; general parallel helpers |
| Native workers | Bounded native queues, parameter snapshots, errors, drain/stop, joined cleanup and Python progress | I/O-specific Asio use and additional queue topologies when needed |
| Python tests/configuration/tuning | Pydantic adapter, generated native tests, deliberately faulty native fixture, held-out tuning, standalone-driver replay | Workflows on an external production estimator |
| Python extension points/reports | Real OMPL virtual dispatch; typed-fixture error reports with exact timestamp joins and provenance | Sensor-specific interfaces, additional dataset/reference formats |
| Explicit guide delivery | Three task guides, per-kit overview/API guides, read-only CLI and environment diagnostics; local Conda proof covering 11 artifacts and 23 resource comparisons | Channel publication and repeated fresh-agent checks of the current installed workflow |

Fresh Sol-high sessions completed configuration, buffer kernels, tuning functions,
and full nanoflann integration with unchanged acceptance and no human logic
repairs. The latter took 513 seconds and needs a smaller scaffold for live use.
Luna high has not been evaluated on these new experiments. Independent review
found and corrected alignment, empty-input, and overflow defects. These checks
do not establish arbitrary C++ memory safety or data-race guarantees.

## Shared experiment requirements

- Use Pixi and pin dependency versions. Identify whether instructions require
  this checkout or only installed packages.
- Start with one useful operation. Check existing Python bindings and current
  kit capabilities before designing a new API.
- Expose guidance through an explicit read or discovery command. Do not install
  skills or modify agent settings automatically.
- Provide a skeleton, exact prompt, independent correctness check, and a saved
  solution. Evaluate GPT-6 Luna with high reasoning in fresh sessions.
- Record setup, compilation, warmed execution, conversion costs, memory use,
  and agent elapsed time separately. Preserve failures and manual corrections.
- Probe uncertain declarations in a subprocess. A precompiled adapter with a
  small interface is an option when exposing all template headers is impractical.
- Promote a recipe into a helper or kit only after the operation and agent
  workflow work. Compare against a useful baseline, not intentionally slow code.

## 1. Stateful native components

Extend the existing C++ class and cache facilities with a convenient workflow
for custom components. Define construction, reset, state inspection, ownership,
and deterministic cleanup. Reuse history and scratch storage across calls.

First experiment: a bimanual motion detector that accepts samples or chunks and
returns newly confirmed events. Update state incrementally. Compare it with the
current batch rule. Compare streaming behavior separately with the current
window-local ROS rule, which resets state at each window boundary.

Acceptance: identical batch/streaming results under an explicitly agreed rule;
identical results for different chunk boundaries; defined handling of repeated
or decreasing timestamps; bounded storage; successful repeated reset and close.
Measure allocations and Python/native crossings as well as processing time.
Consider reserved vectors and reusable [memory resources](https://eel.is/c++draft/mem.res)
where measurement shows allocation costs.

## 2. Buffer interoperability

Build on the checkout's existing typed NumPy buffer support. Add reusable shape,
stride, ownership, and native-output adapters. Use
[Eigen::Map](https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialMapClass.html)
for compatible matrix views. Investigate span/mdspan-style views under the
configured C++ standard. Explore [DLPack](https://github.com/dmlc/dlpack) for
compatible ML tensors, starting with CPU storage.

First experiment: transform a batch of model-produced Cartesian points, filter
them, and pass the resulting buffer to another native operation. Include a
read-only input and a noncontiguous input with an explicit copy/reject policy.

Acceptance: correct dtype, shape, layout, and results; documented copies;
pointer-identity checks where sharing is claimed; owning objects retained for
the full operation; defined behavior if storage could be resized. GPU support
requires a separate device/stream/lifetime experiment.

## 3. Native recording extraction and analysis

Use the [C++ MCAP reader](https://mcap.dev/docs/cpp/) for topic/time selection and
container access. Treat schema and payload decoding as separate work. Begin
with normal typed ROS messages, such as PoseStamped, JointState, and Image.
Preserve recorded timestamps and return selected fields in batches.

The current HIW recording stores JSON in a ROS String. That is an input-specific
adapter, not a recommended message design or an assumption about MCAP files.
[simdjson](https://simdjson.org/) is optional for actual JSON payloads or metadata.
Investigate [Arrow interoperability](https://arrow.apache.org/docs/format/CDataInterface.html)
later if columnar dataset exchange is useful.

First experiment: extract Cartesian poses and run a temporal query over a typed
MCAP. Use a small generated fixture plus one verified public typed recording.
Handle selected fields without constructing a Python object for every sample.

Acceptance: match the existing decoder's fields, timestamps, ordering, and
query results; reject unsupported schemas with a concrete reason; separate I/O,
decompression, decoding, conversion, and computation timings. Compare against
the existing rosbag2 C++ path as well as the Python loader.

## 4. Ceres and manif feasibility study

[Ceres](https://ceres-solver.readthedocs.io/latest/nnls_tutorial.html) supplies
nonlinear least-squares solving, robust losses, and automatic differentiation.
[manif](https://github.com/artivis/manif) supplies Lie-group operations and
Jacobians. Verify their compatibility with the actual toolchain and cppyy path.

First experiment: estimate a rigid sensor transform from synthetic observations
with known ground truth. Use enough varied motion for an observable problem.
Add noise and outliers. Add time-offset estimation only after the transform
problem works, with trajectories that make the offset observable.

Let an agent define the native residual and construct the solve from Python.
Compare it with [SciPy least_squares](https://docs.scipy.org/doc/scipy/reference/optimize.html)
on the same objective, bounds, data, and stopping conditions. Check available
Python bindings rather than assuming a new wrapper adds value.

Acceptance: parameter and residual errors within declared tolerances; consistent
frame and rotation conventions; derivative checks; interpretable failure on
degenerate inputs; measured boundary/startup costs. Keep this as a feasibility
study until it offers a usable advantage in custom residuals, composition, or
iteration speed.

## 5. Ruckig with custom controller logic

Use [Ruckig](https://github.com/pantor/ruckig) for state-to-state trajectory
generation with velocity, acceleration, and jerk constraints. Start with the
local Community functionality. Its intermediate-waypoint behavior is a separate
Community/Pro consideration.

First experiment: changing target states feed a persistent native trajectory
generator. Combine generation and custom tracking logic in a native step, then
drive the existing control kit's mock hardware. Check Ruckig's own Python binding
as a baseline; assess the value of native composition.

Acceptance: equivalent trajectory samples under matching inputs; constraints
checked between target updates; defined recovery from invalid targets; measured
controller timing distributions. Keep trajectory-generation correctness separate
from physical-robot dynamics and real-time deployment claims.

## 6. nanoflann as the new-library example

Use [nanoflann](https://github.com/jlblancoc/nanoflann) to demonstrate a useful
operation without making a full kit first. Locate installed headers through
the existing [require helper](../cppyy_kit/require.py). Use a pinned,
checksum-verified source only when needed.

First experiment: build and retain an exact nearest-neighbor index over recorded
Cartesian positions. Query a batch and apply a custom selection rule in native
code. Keep indexed storage alive and define whether it can change.

Acceptance: neighbors match a brute-force reference, including ties under the
documented policy; distinguish squared distances from distances; reuse the index;
record build and query times separately. An agent must complete this from the
provided library recipe without an existing nanoflann kit.

## 7. oneTBB and xsimd, including Pydantic-defined data

Investigate inline use of
[oneTBB](https://oneapi-spec.uxlfoundation.org/specifications/oneapi/v1.3-rev-1/elements/onetbb/source/flow_graph)
for parallel algorithms and pipelines, and
[xsimd](https://github.com/xtensor-stack/xsimd) for portable SIMD operations.
Evaluate them independently before combining them.

The existing [Pydantic struct prototype](../cppyy_kit/pydantic_structs.py) already
converts a supported model subset to native structs and vectors. Build on it.
Python validates input; native workers operate on native storage. Python model
instances and validators must not be accessed from a GIL-free native task.

First experiment: a Pydantic model defines detection records with numeric
position, confidence, and frame index. Compute spatial/confidence predicates
and per-frame counts. Compare serial native, oneTBB, and SIMD implementations.
Each task writes disjoint output or uses an explicit reduction. Freeze input
storage during processing. Inspect whether an array-of-structs or columnar
layout suits the operation; conversion costs count toward the result.

Acceptance: matching predicates and integer counts; declared floating reduction
tolerance; results stable across worker counts; validation semantics retained;
bounded thread usage when composed with other native libraries. Compare SIMD
against the compiler's existing vectorization. Consider a parallel map/filter
helper only after this operation demonstrates a repeatable benefit.

## 8. Background native workers supervised by Python

Offer a consistent lifecycle for native computation and I/O workers. Use plain
threads where sufficient. Investigate [Boost.Asio](https://github.com/boostorg/asio)
for asynchronous I/O and timers, and
[Boost.Lockfree](https://www.boost.org/doc/libs/latest/doc/html/lockfree.html)
for appropriate queue topologies. A single-producer/single-consumer queue must
be used under that producer/consumer contract.

First experiment: a native worker processes timestamped inputs while an ordinary
Python thread updates settings and retrieves summaries. Provide start, stop,
drain, snapshot, and context-manager behavior. Define queue capacity and whether
overload blocks, drops oldest data, or drops newest data. Preserve sequence
numbers and report dropped inputs.

Acceptance: numerical results and ordering match the serial reference; overload
follows its policy; stop joins workers; exceptions reach the supervisor; no worker
uses released buffers or invokes Python after teardown. Test Python supervisor
progress during native processing and define synchronization for parameter updates.

Python already supports background I/O through
[threading](https://docs.python.org/3/library/threading.html) and
[asyncio](https://docs.python.org/3/library/asyncio.html). On ordinary GIL-enabled
CPython, CPU-bound Python bytecode does not run in parallel across threads;
native code can release the GIL. Our existing
[nogil helpers](../cppyy_kit/nogil.py) support this direction.
[Free-threaded Python](https://docs.python.org/3/howto/free-threading-python.html)
is another configuration to evaluate separately. Do not assume cppyy or the kits
are compatible with it. Background workers are not uniquely a C++ capability.

## Selection and suggested order

The original units/frame-tagging proposal is not selected as a workstream.
Keep ordinary input checks and documented conventions in the selected examples.

Current order after the completed experiments and upstream integration:

1. Align the presentation manifest, examples and guidance with the current
   `NDArray`/`ConstNDArray` API and locally validated 0.4.0 workflow. Keep the
   original 0.3.0 agent evidence separate.
2. Make the AI entry point task-first. Supply explicit guide reads for measured
   Python acceleration, a new library, and an existing compiled component.
3. Prepare a compact nanoflann scaffold that retains the build and ownership
   contract, leaving the useful filtered query as the live implementation task.
4. Repeat fresh-agent completions and changed-requirement cases. Measure cold
   startup, warmed execution and conversion separately; rehearse the GUI and
   complete presentation on its host.
5. Extend the bounded experiments only when a concrete workload requires it.
   GPU/Arrow interoperability, external production estimators and physical
   control remain separate work.

## Reverse direction: Python improving existing C++ workflows

The bounded versions are implemented in `next_steps/`; the production-library
extensions below remain proposed. Use a pre-existing compiled C++ implementation
through its headers and library. Keep its deployment API and behavior available.
cppyy does not operate on an arbitrary binary without compatible declarations
and dependencies. Prefer small adapters where the library is tightly coupled
to an application executable.

The selected directions are items 2 through 6 from the reverse-direction
discussion. They are expanded below using that numbering. Interactive notebooks
and widgets can support these experiments; they are not a separate selected
workstream. Apply the shared agent evaluation requirements to each direction.

### 2. Find edge cases in existing C++ with Python tests

Use [Hypothesis stateful tests](https://hypothesis.readthedocs.io/en/latest/stateful.html)
to generate inputs and sequences of calls against an existing native component.
This reduces manual work enumerating combinations of state and input history.

First experiment: exercise a timestamped filter or motion detector through
reset, feed, and query. Include empty batches, different chunk sizes, repeated
timestamps, and long gaps. Define the expected handling of each case first.
Check chunking equivalence, reset isolation, and an independent reference where
the algorithm permits one. Use a deliberately faulty test fixture to establish
that the tests detect a known error; do not require a bug in the real component.

Implementation work: provide a fixture that constructs and cleans up native
objects, bounded input strategies, explicit tolerances, and saved reproductions.
Run crash-prone probes in a subprocess. A native crash is not a Python assertion
that Hypothesis can automatically shrink.

Acceptance: the known fault is detected; a reduced failing sequence can be
replayed; correct native behavior passes; failures distinguish binding mistakes
from algorithm errors. Keep a reproduction usable without the test generator.

Agent prompt: "Test this C++ filter's reset and chunking behavior from Python.
Generate edge cases and save a small reproducer for any failure."

### 3. Tune native algorithms with Python optimization

Use [Optuna ask-and-tell](https://optuna.readthedocs.io/en/stable/tutorial/20_recipes/009_ask_and_tell.html)
to propose parameters and collect metrics from complete native replays. Consider
[SciPy optimization](https://docs.scipy.org/doc/scipy/reference/optimize.html)
for appropriate smooth objectives. Python manages trials and results; the existing
C++ implementation performs the rollout.

First experiment: tune filter noise parameters on Cartesian pose recordings with
an independent reference. Measure position error in metres and response lag in
seconds. Declare the objective, parameter bounds, trial budget, and held-out
episodes before tuning. Include the current settings and a simple fixed-budget
search as baselines. Use a synthetic fixture with known truth if real recordings
do not have a reliable reference.

Implementation work: reset native state for every trial, batch each replay into
a small number of calls, and save dataset identity, seeds, settings, and metrics.
Use separate native instances if trials run concurrently. Record invalid or
failed trials explicitly. Controller tuning requires a plant with actual dynamics.

Acceptance: a saved trial reproduces its metrics within declared tolerances;
trial order does not leak state; the exported settings reproduce in the C++
driver; held-out results are reported even if tuning provides no improvement.

Agent prompt: "Tune this native estimator on the training recordings within this
trial budget. Compare the defaults on held-out recordings and export the settings."

### 4. Validate configuration before constructing native components

Use [Pydantic models](https://docs.pydantic.dev/latest/concepts/models/) for defaults,
field validation, and configuration export. Add explicit checks for relationships
between fields. This gives users errors at the configuration boundary rather than
during a native update.

First experiment: define a configuration for an existing estimator or controller.
Validate finite values, declared ranges, matching vector lengths, and frame names.
Specify units in the field names or descriptions. Resolve defaults and reject
unknown fields before creating the native object. Choose strict validation or
document permitted coercions.

Implementation work: map the validated model into the library's existing C++
configuration type. The current [Pydantic struct prototype](../cppyy_kit/pydantic_structs.py)
generates new structs; it does not automatically adapt arbitrary existing structs.
Provide a small explicit adapter and identify supported fields and conversions.
Validate once per configuration change, outside the repeated native computation.

Acceptance: invalid configurations never reach native construction; errors name
the relevant fields; resolved configurations round-trip; the native object receives
the intended values and units. Verify exported settings with the existing C++
configuration loader, using its format or a documented conversion.

Agent prompt: "Validate this configuration in Python, populate the existing C++
configuration type, and save the resolved settings used by the experiment."

### 5. Prototype compatible C++ extension points in Python

Use cppyy's [cross-language inheritance](https://cppyy.readthedocs.io/en/latest/classes.html#cross-inheritance)
or compatible callbacks to supply experimental behavior to an existing engine.
The repository already documents these paths in
[common patterns](../docs/COMMON_PATTERNS.md) and the [control kit](../control_kit/SKILL.md).
Only supported virtual interfaces or callback slots can be replaced this way.

First experiment: supply a Python mock sensor to a native estimator harness.
Inject a scripted dropout, bias, or delayed observation without changing the
estimator. An alternative is a low-frequency stopping policy that reads native
summary metrics. Compare with a simple equivalent C++ implementation.

Implementation work: inspect exact signatures and all required virtual methods.
Retain Python objects for as long as native code references them. Define exception
propagation, thread behavior, and shutdown order. Python callbacks acquire the
GIL. Measure callback cost and frequency before using this path in a repeated loop.

Acceptance: calls originate in the native engine and reach the intended Python
override; outputs match the reference; errors reach the caller; teardown leaves
no active callback referencing a released object. If callback cost dominates,
move the measured operation into C++ and check behavioral equivalence.

Agent prompt: "Implement this C++ sensor interface in Python. Inject the specified
dropout sequence and demonstrate that the native estimator receives it."

### 6. Turn native results into useful dataset reports

Use Python to join native output with episode metadata, annotations, and camera
frames. This supports investigating estimator failures and selecting data for
review or training without adding report-generation code to the C++ algorithm.

First experiment: run an existing native estimator over a typed MCAP. Rank error
windows against a verified reference, then produce a report with episode identity,
timestamps, metrics, plots, and representative images. If no reference exists,
label the ranking as a diagnostic heuristic rather than ground-truth error.

Implementation work: preserve timestamps and frame conventions. Declare the
clock, image-matching direction, and maximum time difference. A
[pandas as-of join](https://pandas.pydata.org/docs/reference/api/pandas.merge_asof.html)
can match sorted timestamped records within a tolerance. Keep unmatched records
visible. Decode images for selected windows rather than the whole recording.
Use [Plotly](https://plotly.com/python/) or a static plotting library for the report.

Acceptance: checked fixtures select the expected windows and frames; missing
images and unmatched timestamps are reported; every result identifies its input
and configuration; the report can be regenerated. Split extraction, native
computation, joins, image decoding, and rendering costs.

Agent prompt: "Find the five largest estimator-error windows in this recording.
Produce a report with plots, matched images, and enough provenance to reproduce it."

## A first reverse-direction demonstration

Use an existing C++ estimator with a reset method, configurable filter parameters,
and a batch or rollout method. Supply one recorded fixture and an independent
reference. A simulated plant is an alternative if it has real dynamics; the
current GenericSystem position mirroring is not a sufficient plant for making
claims about physical controller tuning.

1. Load the compiled implementation from Python through cppyy. Define reset,
   timestamp, and configuration behavior.
2. Validate input and configuration with Pydantic and an explicit native adapter.
3. Use Hypothesis to check reset and chunking invariants before tuning.
4. Use Optuna to propose settings and tell it the metrics from complete native
   rollouts. Keep each rollout native to amortize the boundary crossing.
5. Compare the selected configuration on a held-out fixture. Generate an error
   report with matched images where the fixture provides them.
6. Export the resolved configuration and reproduce it with the existing C++ driver.
7. Add a Python mock sensor as a separate extension experiment if the native
   harness has a compatible interface. Do not add a plugin API solely for the demo.

Start with configuration validation and generated tests. Add tuning and dataset
reports on the same component so the examples reuse setup and correctness checks.
The extension-point experiment can proceed independently when a suitable existing
interface is available. An interactive plot is optional support for the story.

Measure time to make an experiment change, rebuild requirements, adapter size,
results, startup, conversion, and rollout costs. The goal is easier experimentation
around existing C++, not an assumed execution speedup from adding Python.

An optional ML extension lets Python run a model or select high-level actions
while a native component handles frequent updates. It needs explicit timing,
handoff, and stale-output rules. Passing a C++ result into PyTorch does not
automatically provide gradients; differentiable integration requires a supplied
backward computation or an appropriate native derivative path.
