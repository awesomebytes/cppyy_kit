# Proposed talk redesign: interactive robotics experiments

Updated 21 October 2026. The [slide draft](SLIDE_DRAFT.md) now follows this order with 24 main slides and 10 backups. The [HTML preview](SLIDE_DRAFT.html) includes coloured code and speaker notes. The earlier [script](TALK_SCRIPT.md) remains reference material.

## Main argument

LLMs reduce the effort of writing wrappers and build files. cppyy_kit can still
provide useful native integration and a short feedback loop against retained
robot data and running resources. Use that loop to discover the requirements,
collect examples and failure cases, and build a reference implementation.
These become an implementation brief for a dedicated agent or agent team,
with concrete criteria for judging the result.

Explain the contributions separately: cppyy provides dynamic C++ access;
cppyy_kit and the kits provide reusable setup and integration; marimo and its
agent interface provide the interactive workspace. Agent productivity is a
hypothesis to evaluate through completed tasks.

## Proposed 45-minute sequence

| Minutes | Content | Screen |
| --- | --- | --- |
| 0–3 | Robotics context and language vote | TIAGo PRO clip and familiar ROS graph |
| 3–9 | cppyy, then cppyy_kit | Small example and meaningful cached Eigen example |
| 9–15 | Kit capabilities, centred on rclcppyy | Measured ROS performance and native C++ objects |
| 15–23 | Nav2 policy and Python controller | Minimal integration code and robot demonstration videos |
| 23–26 | What changes when agents can write bindings? | The retained interactive workspace |
| 26–33 | Live coding against the webcam | Agent and viewer side by side; choose and implement the task live |
| 33–38 | From interactive discovery to an implementation brief | Examples, acceptance cases, reference code, runtime costs and target constraints |
| 38–40 | Close | Explore, specify, implement, evaluate |
| 40–45 | Questions | Supporting material |

The webcam is the primary interactive demo. If it cannot run, use the marimo notebook with the robot MCAP to detect blocks on the table. The fallback is an interactive recorded-data task, not a demonstration video. Keep its hidden slide and notebook tab ready.

## Kit capabilities: rclcppyy and native data reuse

Give the revamped rclcppyy library the largest share of this six-minute section:

- About three minutes: introduce its Python-facing ROS API and show a compact
  performance comparison from the revamped library. Label the workload, message
  size, callback path and measured metric. Use results from that version;
  distinguish rclcppyy measurements from lower-level rclcpp_kit pipeline results.
- About one minute: show Python holding a proxy to a C++ ROS message and passing
  the compatible object to C++ processing. Explain how retaining the native
  representation avoids rebuilding message fields as Python objects at that
  boundary. Show the actual conversion or view used when entering PCL or OpenCV.
- About one minute: show the minimal Python ros2_control controller and its
  reusable bridge glue as another kit capability.
- About one minute: show pick-and-place as a short outcome clip connecting
  perception and motion. Keep the detailed WBC explanation in backup material.

Suggested narration:

> I also revamped rclcppyy. There are two things I want to show here: the
> performance we measured, and what happens to the data. We can keep a message
> as a C++ object, let Python decide what to do with it, and pass it into C++
> code. That is useful when the next operation lives in a native library too.

Show this simple data path beside a small real code example:

```text
ROS input → C++ message → native processing → result
                 ↑ Python selects and configures the operation
```

Explain ownership and copy boundaries through the example. Keeping the message
native avoids a representation conversion on compatible paths. Middleware
transport, ROS-to-PCL conversion and processing can still allocate or copy.
Use the [ROS section of the script](TALK_SCRIPT.md#6-bring-ros-into-the-story-before-processing-its-messages)
and [pipeline deployment notes](next_steps/pick_place/DEPLOYMENT.md) for the
existing code and boundary explanations.

## Suggested spoken transitions

> I usually work in robotics. Sensors, control, data, autonomous systems. We are
> at ROSCon, so this probably looks familiar. I set up this robot as a playground.
> Let me show you a few things we can build from Python, using C++ underneath.

After the kit examples:

> Now, there is a question I have been asking myself. An agent can write the C++,
> the bindings and the CMake project for me. That used to be a substantial part
> of the friction. So what is still useful here?

Before the notebook:

> I think part of the answer is being able to try things against the actual data.
> We have the data loaded, the native libraries available, and a result we can
> look at. Let's change the question and see what happens.

Before the webcam:

> That was recorded data. Now the input keeps changing. Can we keep the camera
> running and change what we ask the software to tell us?

Before the implementation handoff:

> During this session, we have figured out what we mean by “works”. We have
> collected examples, found cases where it fails, and made a reference
> implementation. Now we can give all of that to another agent and ask it to
> build an implementation for the machine where we want to run it. And we have
> something concrete to judge its result against.

## Recorded-data demonstration

Use the existing [marimo session](next_steps/agent_session/NOTEBOOK.md) and
[table recording](next_steps/agent_session/CAPTURE.md). Keep the robot video as
context; the MCAP supplies repeatable input on stage.

1. Show the table objects and explain the current cloud filtering and clusters.
2. Ask the agent to find the red object and overlay its selected points.
3. Refine the region and voxel size, inspect the result, and save the accepted
   notebook source. Show the recorded fresh-process replay result.
4. Show the handoff: selected inputs, reviewed expected behavior, a failure
   case, reference code and the deployment requirements.

The implemented experiment retained its kernel and input arrays across three
revisions. It moved from three objects to one red cluster with 178 points.
The saved notebook reproduced the result in a fresh process. See the
[session evidence](next_steps/agent_session/README.md).
Rehearse the spoken agent interaction with bounded requests and legible output.
Show the real edit and a result check, including a correction when needed.
A useful additional acceptance case is a frame without the requested object.

## Webcam demonstration: implement the task live

Start with [live_notebook.py](next_steps/webcam_session/toolbox/rendering/live_notebook.py): one webcam, a pass-through processor and a Viser viewer. Put the agent terminal on the left and the viewer on the right. Keep the marimo tab open in the background. Rehearse resource startup and agent access before the talk. The perception task is chosen and implemented live.

| Step | Action | Point to explain |
| --- | --- | --- |
| Start | Show the current webcam image and choose one bounded task | Work from the scene in front of us. |
| Implement | Ask the agent to add detection, tracking or another useful operation | The interpreter and camera stay alive while processing changes. |
| Inspect | Look at the result; identify one concrete failure or refinement | Visible output helps clarify what we actually want. |
| Refine | Change the processing and check it again | Keep model instances and accepted state when useful. |
| Retain | Save accepted cells and representative inputs/outputs | Supply a reference and acceptance cases to an implementation agent. |

Do not script a tour of features from an earlier webcam session. There is no rain or gesture sequence to reproduce. Use an existing model when appropriate and add a cppyy_kit native operation where the chosen task benefits from one. The depth-to-points slide is an illustrative option, not a promise about the live task.

Suggested transition:

> We have the camera and viewer running. Let's choose something to do with the image, ask the agent to implement it, and look at what actually happens. Then we can change the request while the session stays alive.

If webcam delivery is not workable, switch to the [MCAP notebook](next_steps/agent_session/NOTEBOOK.md) at port 2718. Use the robot recording to identify blocks on the table, select by color, inspect image and cloud results, and refine the region. The saved notebook starts with a red-object selection; inspect that starting state and make a visible change. Keep the input loaded while the analysis is edited. The fallback slide is hidden during the normal slideshow.

The presentation does not use webcam recordings as a fallback. The existing robot navigation and arm videos remain part of the earlier ROS section.

## From interactive discovery to an implementation brief

The interactive exercise produces a specification supported by data and code.
Capture these artifacts as the task becomes clearer:

| Artifact | What to preserve |
| --- | --- |
| Representative inputs | Recordings, selected frames and relevant operating conditions |
| Expected behavior | Reviewed outputs, tolerances and acceptance criteria |
| Failure cases | False detections, absent targets, occlusion and changing conditions |
| Reference implementation | Readable accepted code, dependencies and input/output contracts |
| Operational requirements | Target hardware, latency, memory and deployment constraints |

Reference outputs need review before they become expected answers. Keep separate
held-out evaluation examples so an implementation is assessed beyond the cases
used to develop it. Recordings and labels can also supply training data when a
learning approach is appropriate; otherwise they serve as examples and test data.

Give the brief to a dedicated implementation agent or a team comparing approaches:

> Here is what we want it to do. Here are examples, failures and a working
> reference. Implement it for this target, check its behavior, and measure it.

Define “best” through the requirements. Agents could compare an existing library,
a compiled C++ implementation and a Rust implementation against the same behavior
and resource limits. Evaluate correctness first, then performance and maintenance
cost. Retain useful new failure cases found during implementation.

cppyy_kit supports discovery through native access and interactive feedback.
The implementation can use whichever tools satisfy the brief. Compiling the
accepted kernel is one possible outcome; replacing the algorithm or implementation
language is another. The shared contract and evaluation cases connect the stages.

### Runtime costs inform the deployment requirements

Use one compact measurement slide. In the recorded Eigen experiment, importing
cppyy added about 164 MiB RSS. The cached-kit process reached about 203 MiB after
its first call; native extension processes were about 37 MiB. These are process
measurements for this environment. Separate startup, first use and warmed calls.
Explain deferred compilation, buffer ownership, callback/GIL behavior and native
failures through concrete examples. See [measurements](slide_assets/runtime_costs/README.md).

The cloud investigation provides a concrete implementation comparison. With the same source and
unchanged nanobind wrapper, offline Clang reached about 0.360 ms versus JIT
0.360 ms and original GCC 0.430 ms for 307,200 points. Generated SIMD code
explained the difference. See [conditions and analysis](slide_assets/runtime_costs/cloud/CODEGEN.md).

> We found something that worked interactively, and then we could keep its
> performance in an ordinary compiled library. That is a useful destination
> for an experiment.

Use the demonstrated saved-notebook replay and standalone compiled kernels as
existing evidence. Present the broader agent handoff as the proposed next stage:
an implementation brief, candidate implementations and evaluation against the
captured cases and deployment requirements.

## Material to reuse

| Material | Role |
| --- | --- |
| [Script](TALK_SCRIPT.md), [slides](SLIDE_DRAFT.md), [speaker notes](SPEAKER_NOTES.md) | Existing narration, code, actual controller glue and media cues |
| [Agent workflow discussion](AGENT_LIVE_EXPERIMENTS.md) | LLM argument, persistent sessions and extraction ideas |
| [marimo notebook](next_steps/agent_session/NOTEBOOK.md) | Main interactive example and reproducible replay |
| [Continuous ROS perception](next_steps/agent_session/README.md) | Live robotics evidence and alternative to webcam |
| [Pick-place results](next_steps/pick_place/RESULTS.md) | Short robot outcome clip |
| [Runtime measurements](slide_assets/runtime_costs/README.md) | Tradeoff slide and detailed backup tables |
| [Compiler investigation](slide_assets/runtime_costs/cloud/CODEGEN.md) | Offline artifact preserving measured JIT performance |

The earlier plan covered perception, nanoflann, control, admittance, pick-place,
WBC, composition and optional navigation in sequence. Preserve those sections
as supporting material. Use WBC/pick-place as an outcome clip. Keep solver
details, Nav2, nanoflann, full binding tables and assembly for questions.

A later evaluation can compare the same bounded task in this workspace and
through an agent-generated extension. Record time to a checked result,
corrections, build steps and resource use. This tests the workflow claim directly.
