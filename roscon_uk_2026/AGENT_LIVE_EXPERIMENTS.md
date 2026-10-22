# Live experiments with an agent and Python/C++

Working exploration, 20 October 2026. This records the argument behind the talk,
proposed package features and a first working experiment. The measured results
are listed under “Current exploration”.

## Start with a robot question

Ask an agent to use the simulated robot's live observations:

1. Find the red object on the table.
2. Exclude candidates outside a specified reachable region.
3. Track the nearest remaining candidate and report when it moves.

Keep the camera subscription, calibration, recent observations and native
objects alive between requests. Each answer should include an overlay,
observation timestamp, selected object and measured processing time. The agent
can inspect those results and change the computation in the next request.

The first experiment should use the existing perception code and simulator.
The useful result is a working sequence of revisions against live data, with
recorded time to a correct answer for each revision.

## Why this project might still matter

An agent can generate CMake, C++, bindings and a Python interface quickly.
The value of avoiding handwritten boilerplate is therefore smaller than it was.
A prepared extension template is a strong baseline.

The remaining opportunity is reducing the cost of an entire experiment:
connect to data, try an operation, inspect the result, revise it, and retain a
reproducible version. If code generation becomes cheaper, people and agents may
try more experiments. Lower overhead per experiment could then matter more.
This is a hypothesis, not a measured productivity claim.

Three capabilities may contribute:

- **Dynamic native API access.** Inspect types and use additional C++ operations
  while the experiment develops, without first fixing a binding interface.
- **State reuse.** Retain compatible C++ messages, clouds, indices and solver
  objects while Python changes the next operation. This can avoid reconstructing
  state and some representation conversions.
- **Reusable integration.** Apply known ownership, callback, build and shutdown
  rules. The Python ros2_control and Nav2 bridges are concrete examples of
  reusable integration work.

A compact API helps an agent only if its behaviour is predictable and its errors
are useful. Hidden initialization, environment changes and native failures can
consume more context and time than generated build files would have required.
The recent auto-PCH contamination of our baseline is a concrete example.

## Two interfaces over the same operation

Use two workflows matched to the data:

- **Offline and recorded data:** a notebook holds an MCAP excerpt, intermediate
  results, parameter controls and overlays. The agent edits and evaluates cells
  while the user inspects the same results.
- **Online ROS data:** the persistent Python session owns the processing
  operation. Publish results into RViz, Foxglove, Lichtblick or PlotJuggler,
  selected for the output type. A small immediate-mode UI can supplement these
  when needed. The first trial uses RViz markers and sensor displays.

The accepted perception function and its parameters should work with either
source. Keep session execution separate from visualization. Viewer refresh and
sensor processing have their own rates; agent requests revise the operation
rather than execute once per incoming frame.

The first trial uses TIAGo PRO facing its object table, records a short MCAP,
and performs comparable selection revisions on the recording and live data.
It is tracked under [agent_session](next_steps/agent_session/README.md).

## Keep one Python interpreter alive

Evaluate marimo for the offline notebook and its existing agent interface.
An IPython kernel launched inside the selected Pixi environment, with an adapter
based on `jupyter_client`, is the alternative persistent execution backend. A notebook UI is
optional. Jupyter already specifies execution requests, streamed output, rich
results, errors, completion, inspection and kernel lifecycle messages.
[Protocol reference](https://jupyter-client.readthedocs.io/en/stable/messaging.html).

Proposed arrangement:

```text
Agent                         Persistent Python kernel
  execute(code, session_id) →   Python namespace + cppyy
  inspect(name)             →   live native objects and subscriptions
  receive result            ←   summaries, timings, errors, overlays
                                      ↕
                                simulator / ROS
```

Python and C++ objects remain inside the kernel process. Tool replies carry
small descriptions and image artifacts, rather than serialized copies of every
cloud. A new agent turn reconnects to the same session. A kernel restart creates
a new session identity, invalidating references to the old native objects.

Start with a small tool surface:

| Operation | Purpose |
|---|---|
| Execute a cell | Run Python and return its result, output and exception |
| Inspect an object | Retrieve a targeted type/API/state summary |
| Interrupt or restart | Recover control and rebuild the session when necessary |

Use the existing Jupyter client protocol to implement these operations. Match
responses by request ID; return when that execution completes, rather than
assuming that the first output message is its result. Disable interactive stdin
for agent requests. Serialize agent edits to the shared namespace.

Initialize subscriptions and long-lived resources once in an explicit setup
cell. Keep a bounded latest-observation store with timestamps. A computation
uses a retained snapshot so new sensor arrivals do not change its input midway.
The executor must be able to service ROS while the agent is between requests;
GIL behaviour and long native calls need measurement in this arrangement.

This is a proposal to reuse a kernel, not to build another notebook platform.
Jupyter's existing Python kernel can be managed independently of a notebook
frontend. [Kernel architecture](https://jupyter-client.readthedocs.io/en/stable/kernels.html).

## Change the computation while retaining useful state

Python function names can be rebound, but an already registered callback may
still hold the previous callable. Give the running pipeline one explicit
replacement point. Evaluate a candidate operation on a retained observation,
then replace the active operation between frames while holding the appropriate
lock. Report which operation version produced each output.

C++ declarations accumulate in Cling's process. Treat revised native operations
as new versions with distinct symbols or namespaces, rather than assuming that
an existing C++ class or live object can be redefined. cppyy exposes declarations
in a shared interpreter scope. [C++ loading reference](https://cppyy.readthedocs.io/en/latest/toplevel.html).

A Python exception does not roll back earlier mutations. A native crash can end
the kernel. Retain the ordered setup and accepted revision cells, parameters,
input identifiers and environment version outside it. Restart by replaying that
record and recreating native objects; raw pointers and live ROS connections are
not checkpoints. After many native revisions, a restart can also reclaim the
accumulated code and objects.

For the first perception experiment, Gazebo and robot control remain in their
existing processes. Later actuation experiments should send goals through the
existing bounded controller interface. Interactive agent execution is not the
control update clock.

## Where the defensible advantage could be

A persistent interpreter by itself is easy to reproduce. Both competing
approaches should receive one in the comparison. An agent can also load a
newly built extension into a persistent interpreter.

The stronger proposition is a reliable live Python/C++ workspace: small task
code, reusable native objects, observable results, inexpensive revisions and
repeatable recovery. If we demonstrate that combination across real robotics
tasks, the maintained integrations and their accumulated fixes are more
valuable than a bespoke REPL.

Stable APIs may be better served by existing Python bindings or a compiled
extension. A simple colour threshold through `cv2` establishes little about
cppyy_kit. A task requiring new PCL operations on native ROS data is a more
informative comparison.

## Evidence we already have

| Observation | What it supports |
|---|---|
| Tiny sum can start slower through the kit | Cache overhead matters for small programs |
| Eigen example: 1.695 s vanilla versus 0.319 s cached process time; first cache-building run 3.948 s | Reusing compiled work can reduce subsequent startup |
| PCL voxel processing: NumPy 15.34 ms versus PCL 1.31 ms | Native processing can help this operation |
| Python controllers and DWB critics run inside native framework hosts | Framework integration can be reused across small Python policies |

The Eigen comparison is against JIT compilation, not a prebuilt extension.
The voxel comparison attributes acceleration to the native operation, not to
agent productivity. The live composition comparison did not establish a latency
speedup. Sources: [Eigen measurements](slide_assets/eigen_startup/README.md),
[perception evidence](next_steps/pick_place/replay_evidence.json),
[controller notes](SPEAKER_NOTES.md),
[composition deployment](next_steps/pick_place/DEPLOYMENT.md).

## Experiments that can decide the direction

Use the same model, task, input data and correctness criteria. Give both paths
working environment setup, equally usable documentation and existing templates.
Repeat attempts and retain failures.

1. **First correct observation.** Compare kit access with generated
   pybind11/nanobind bindings for an unfamiliar operation. Measure elapsed time,
   tokens, tool calls, repairs and numerical or visual correctness.
2. **Revise a live operation.** Give both approaches a persistent kernel and ask
   for several changes after they work. Measure request-to-verified-result time,
   restarts, data reconstruction and input-to-output latency separately.
3. **Recover and reproduce.** Restart the kernel and replay the accepted
   experiment in a clean environment. Measure recovery time and discrepancies.

The primary metric is time to a correct, inspectable result. Lines of code and
tool calls help explain that result; they are not the objective. Record cold
preparation separately from subsequent edits and steady-state computation.

## Questions to resolve next

- Which task genuinely benefits from direct native object access beyond existing
  bindings?
- How much state can we retain across changes before replacement or recovery
  becomes harder than restarting?
- What concise state summary lets an agent resume accurately without repeatedly
  inspecting the whole namespace?
- Can successful exploratory cells become a normal runnable artifact with a
  short, explicit setup sequence?
- Which integrations repay their maintenance cost often enough to belong in a
  kit?

## Possible wording for the talk

> An assistant can write the wrapper. What I want to make cheaper is the whole
> experiment: connect to the data, try the operation, inspect the result, and
> change it. Suppose the robot is already streaming observations. Can we keep
> that workspace alive and let a person or an agent develop the next operation
> against what the robot is actually seeing?

## Existing tools: research update, 20 October 2026

The interactive workflow can be offered as an optional part of cppyy_kit. Keep
its notebook, UI and agent-transport dependencies in that explicitly selected
integration. The core library should retain a small, explicit import surface.

The initial IPython proposal above is one candidate. Current projects already
provide substantial parts of the desired experience:

| Candidate | Existing functionality | Fit for this experiment |
|---|---|---|
| marimo + marimo pair | Live Python notebook, agent execution and editing, reactive UI, Python source files | Strong first candidate for a shared human/agent workspace |
| JupyterLab + Datalayer Jupyter MCP Server | Agent editing/execution in existing notebook kernels, collaborative documents | Strong alternative for conventional persistent-kernel semantics |

These are documentation findings. Compatibility with our cppyy, ROS executor,
Pixi environment and native object lifetimes needs a local trial.

### marimo and marimo pair

[marimo pair](https://github.com/marimo-team/marimo-pair) is explicitly designed
to put agents inside running notebooks. Its execution tool accesses live
notebook variables. Its scratchpad uses a shallow copy of the globals: new
bindings are temporary, while mutations of shared objects can persist. Durable
changes belong in notebook cells. Its current agent instructions identify
`marimo._code_mode` as a private, unstable API.
[Agent interface and state semantics](https://github.com/marimo-team/marimo-pair/blob/main/skills/marimo-pair/SKILL.md).

This distinction matters for cppyy. A temporary Python binding does not imply
that a C++ declaration or native side effect is temporary. The proposed
integration should version native symbols and give subscriptions an explicit
owner and close operation.

Marimo supplies interactive elements whose changes trigger dependent cells.
This fits sliders for voxel size, thresholds and cluster selection.
[Interactive UI](https://docs.marimo.io/guides/interactivity/).
Reactive execution requires care around setup cells that create native
resources; changing a visualization should not create another ROS subscription.
That is an integration requirement to verify, not an established failure.

Its notebooks run as Python scripts, which helps turn an accepted experiment
into source-controlled code.
[Script execution](https://docs.marimo.io/guides/scripts/).

### Jupyter with an existing agent bridge

[Datalayer's Jupyter MCP Server](https://github.com/datalayer/jupyter-mcp-server)
can connect to a local Jupyter installation. Its architecture separates the
notebook document from its execution backend and supports sharing a kernel with
JupyterLab. [Architecture](https://jupyter-mcp-server.datalayer.tech/architecture/).

Use notebook-cell operations for durable code. The documented `execute_code`
tool is for temporary inspection and calculations, and explicitly advises
against introducing imports or assignments that subsequent notebook execution
depends on. [Execution tool](https://jupyter-mcp-server.datalayer.tech/mcp/tools/execute-code/).

This is a reason to reuse the tool's notebook model rather than treat every
agent call as an unrecorded `exec`. It also gives the user a visible record of
what changed and which output came from it.

### Robotics viewers for live data

The online interface uses existing robotics viewers: RViz for point clouds and
selected-object geometry, Foxglove or Lichtblick for combined message and visual
inspection, and PlotJuggler for numeric time series. Choose the smallest
combination needed for the experiment. Start with RViz, which the simulator
workspace already supplies.

The offline notebook shows retained frames, overlays and parameters. In both
workflows, identify the observation timestamp and operation version so the
agent and user can relate the displayed result to the code that produced it.

## Account for the runtime costs

The interactive session carries a compiler and a dynamic language boundary.
On this machine, the [resident-memory measurement](slide_assets/runtime_costs/README.md)
found 18.3 MiB RSS for the Python harness and 182.6 MiB after importing cppyy.
The increase was about 164 MiB; import took 148 ms. After loading and calling the
Eigen example, vanilla cppyy used 367.1 MiB RSS versus 203.3 MiB with the kit's
prepared function cache and **36.4 MiB with a conventional CMake/pybind11
extension**. All paths use the same Eigen source and NumPy inputs. The extension
build took 3.62 s and its runtime had no compiler-library mappings.
These are three-run medians with warm OS caches and
an existing standard PCH. RSS includes shared pages and all process components.

The follow-up five-way run measured 36.8 MiB for Eigen/pybind11, 36.8 MiB for
Eigen/nanobind and 37.0 MiB for a Rust nalgebra/PyO3 port. The C++ bindings share
the exact source; the Rust port uses the same mathematical contract with a
different factorization. All matched the numerical reference for the recorded
input. For this small operation, removing the runtime compiler matters much more
to resident memory than choosing among these compiled bindings. See the same
measurement guide for build recipes, file sizes and timing scope.

The [equal-contract cloud experiment](slide_assets/runtime_costs/cloud/README.md)
provides a better deployment comparison. Every path shares the same validation.
cppyy, pybind11 and nanobind call one shared compiled kernel. Processing a
307,200-point cloud takes about 0.44–0.46 ms through these interfaces, with
overlapping run-to-run variation. NumPy with reusable scratch takes 4.28 ms;
Rust takes about 0.46 ms. A separately JIT-compiled version takes 0.36 ms,
which reflects different generated code rather than a binding advantage.
For this operation, a conventional extension can preserve throughput while
reducing startup and resident memory. Interactive development value should be
evaluated separately from these runtime measurements.

A [bounded machine-code investigation](slide_assets/runtime_costs/cloud/CODEGEN.md)
found two-point SIMD in the JIT cloud kernel, versus scalar arithmetic in the
original offline GCC build. The JIT function still reads its count and transform
at runtime. There is no observed data-shape specialization or removal of the
Python binding. Two GCC optimization adjustments did not reproduce its
throughput. The subsequent standalone Clang 16.0.6 build did: offline Clang
through nanobind and the JIT both measured about 0.360 ms/frame with all variants
pinned to the same CPU. The source and wrapper were unchanged. This demonstrates
that the observed advantage can be retained in an offline deployment; it is not
an inherent JIT advantage.

Costs to explain in the talk:

| Cost | Consequence and practical response |
|---|---|
| Resident compiler and binding state | Budget memory per workload. A prepared function cache can reduce parsing and generated implementation code while Cling stays loaded. |
| Deferred compilation and binding generation | A new template or call path can introduce latency during use. Exercise expected signatures before a timed loop; measure first use separately. |
| Python/native calls and conversions | Overload resolution, conversion, allocation and callbacks have costs. Keep bulk work native and inspect the actual boundaries. |
| Native ownership | A live proxy or array view needs its underlying storage to remain valid. Retain owners and stop callbacks before destruction. |
| Environment and binary compatibility | Headers, libraries, compiler settings and cached artifacts must agree. Capture versions and verify replay in a clean process. |
| Long-lived interpreter state | Definitions and retained resources complicate revision. Use explicit lifecycles and verify accepted source after a restart. |
| Debugging and timing constraints | JIT debugging needs configuration. Python callbacks and runtime compilation need careful placement in control applications. |

cppyy documents [runtime templates](https://cppyy.readthedocs.io/en/latest/stl.html),
[dispatch and conversion costs](https://cppyy.readthedocs.io/en/latest/philosophy.html),
[ownership controls](https://cppyy.readthedocs.io/en/latest/misc.html) and
[JIT debugging](https://cppyy.readthedocs.io/en/latest/debugging.html).
Our kit can organize preparation, caching and ownership, but those mechanisms
must be evaluated against a conventional compiled extension for each workload.

## Export the accepted experiment, then remove runtime JIT

There are three distinct deliverables:

| Deliverable | What is preserved or prepared |
|---|---|
| Reproducible source | Accepted Python/C++ code, parameters, dependency versions and input specification |
| Prebuilt cppyy artifacts | Compiled functions and prepared headers, reducing runtime preparation |
| Runtime without Cling/JIT | A compiled extension, C ABI library, executable or framework plugin exposing the selected operation |

Our current function cache loads a library and presents declarations to cppyy;
cppyy can still generate call wrappers. PCH preparation preserves parsed
headers. Neither mechanism alone establishes a deployment without JIT.
[Current cache implementation](../cppyy_kit/cache.py),
[freeze and first-use costs](../docs/FREEZE.md).

A proposed final agent step would be:

1. Select accepted functions and their actual dependencies, including imports,
   configuration, calibration and resource setup. Preserve ordinary top-level
   imports in the resulting modules.
2. Turn notebook inputs into explicit arguments and remove exploratory edits.
   Record expected outputs for representative input frames.
3. Keep the accepted C++ algorithm and compile its stable entry points ahead of
   time. Use a small nanobind/pybind11 module for Python-facing C++ APIs, a C ABI
   for suitable buffer/scalar operations, or the existing native plugin boundary
   for framework components.
4. Run the exported artifact in a clean runtime environment containing its
   declared dependencies, without cppyy/Cling or build tools. Compare outputs
   with the accepted interactive version and record startup/processing costs.
5. Retain the source, build recipe and versioned artifact alongside the UI or
   headless entry point.

This uses generated conventional bindings where they are useful: after the
operation has stabilized. Interactive discovery and explicit deployment can
complement each other.

The agent should select and verify the useful code, rather than concatenate all
successful cells. A cell can succeed while relying on hidden state or performing
an unrelated side effect. Native objects must be reconstructed from explicit
setup and data; their process addresses are not an export format.

The initial export target should be one buffer-to-buffer perception operation.
It gives us a clear input/output contract and a small build. Broader session
export can follow demonstrated needs.

### Build the acceptance cases while discovering the operation

Each useful interaction can contribute an input, an expected property and its
reason. Save the relevant sensor excerpt, calibration, parameters, result and
human judgement. For this example, useful cases include the selected red object,
an empty region, a frame with no red candidate, and a distractor. Define position
tolerances, color-selection expectations and behavior when no candidate matches.

The interactive implementation's output is a reference, not proof that the
answer is correct. Check it against the image or independent measurements.
Keep some recordings aside while tuning so the final check also uses unseen
inputs. The existing retained-state and replay reports establish reproducibility
on one scene; they are the starting evidence for this proposed acceptance set.

“Flattening” means extracting the accepted operation and making its dependencies
explicit. The C++ algorithm can become a conventional library, executable or
plugin. Python orchestration can remain with a small compiled binding, or be
rewritten if deployment requires it. Dynamic Python behavior is not automatically
translated into C++ by saving notebook cells.

Rust is another possible implementation target. It requires an explicit port or
a boundary around retained C++ libraries such as PCL. Use the same input/output
contract and acceptance cases to evaluate that port. The language choice follows
deployment needs; the reusable product is the verified operation and its evidence.

## Current exploration

Three Sol 6.1 medium agents implemented and exercised the two interfaces.
The [runnable guide](next_steps/agent_session/README.md) links the source,
recording provenance, screenshots and raw results.

- **Input:** a 14.574-second TIAGo PRO MCAP, with 273 RGB/cloud/depth frames,
  calibration and TF. The recorded scene contains three tabletop objects.
- **Offline interaction:** marimo 0.25.2 retained one Python process and the same
  cloud array while the agent edited selection criteria through its code-mode
  interface. The broad region yielded three color-associated clusters.
  Selecting red and narrowing the region yielded one cluster with 178 visible
  points. The saved notebook was replayed in a separate process.
- **Online interaction:** the same operation processed 13 fresh ROS frames
  with advancing timestamps. RViz showed the source cloud, filtered points,
  selected bounds and RGB overlay. The first processing call took 2218.55 ms;
  subsequent calls had a 4.60 ms median. These timings exclude imports, sensor
  acquisition, decoding, TF and publication. See the
  [live measurement](next_steps/agent_session/evidence/live_verification.json).

The notebook's agent API is an internal marimo interface, so the experiment pins
its version. A dropdown update produced an assertion error; source-cell edits
provided the validated interaction. Keep that integration in the example while
evaluating a stable package interface.

This establishes a working retained-state interaction and shared perception
operation. A comparison against generated bindings should measure complete task
time, tool calls and correctness under the same conditions.

The following export experiment should compile the accepted perception
operation ahead of time and replay the same MCAP without Cling. That is a
separate deliverable from saving the notebook source.

The proposed product workflow is:

```text
live or recorded observations
            ↓
agent + human + persistent Python/C++ session + UI
            ↓
accepted operation and reproducible source
            ↓
ahead-of-time native artifact + ordinary Python/UI or ROS host
```

The reusable contribution from cppyy_kit would be native integration, resource
ownership, explicit preparation and the export contract. Existing notebook and
agent tools can provide the editor, session transport and general UI machinery.

## Package direction agreed on 22 October: interpreter_kit

`interpreter_kit`: a persistent Python interpreter for agents, notebooks and live
applications. Extract the reusable session interface into an optional package.
Keep marimo, live viewers and model integrations outside the core cppyy_kit
installation requirements. Session operations should cover starting/reconnecting,
executing code, inspecting results/errors, displaying output and clean shutdown.

A working interactive experiment should produce an explicit reference capture:

- Working Python and C++ source, configuration and dependency/model identities.
- Selected sample inputs and expected outputs, with comparison tolerances.
- Instructions to reproduce the result in a fresh process.
- The Python API to preserve and measured startup, latency, throughput and memory.

A saved notebook alone is insufficient: stale definitions, device handles and
background threads can make its visible source differ from its running state.
Capture a selected working version and verify its replay against the samples.

The capture becomes the implementation brief for the same agent or a new agent
conversation. The final implementation may use C++ with nanobind/pybind11, Rust
with PyO3, or another suitable implementation. Choose using dependencies and
measurements. Preserve behavior and the Python API, not the prototype language.

Acceptance checks: replay the reference samples within their tolerances, verify
the Python API, and measure startup, steady-state performance and memory in a
fresh process without cppyy/Cling. This is a proposed package workflow; the next
implementation task is extracting the persistent-session code and the capture
interface from the existing demos.
