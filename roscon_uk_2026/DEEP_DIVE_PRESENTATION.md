# It was supposed to be a small Python script

A working spoken script for a roughly 45-minute deep dive. The story follows
one prototype as we keep asking it to do more. Italic notes are cues for the
presenter. The expandable sections hold the exact evaluated prompts and
rehearsal commands. [Evaluation results](EVALUATION.md) and the
[work plan](PLAN.md) sit alongside this script.

The main five-stage timings below are historical measurements from the
3 October cppyy-kit 0.3.0 evaluation. The current rehearsal uses this checkout's
0.4.0 source APIs, including NumPy array annotations and packaged guides.
The archived agent sources and measurements remain separate from migrated
rehearsal solutions. Recheck correctness and measure startup and operation
times in the current environment before using new numbers on stage.

Timing budget: opening 2 minutes, Python 7, MCAP 8, native library 4, ROS 6,
controller 7, webcam 5, closing 2. This leaves 4 minutes for questions or a
demonstration overrun. Use prepared solutions and recorded agent runs.

## Opening: we just wanted to ask the data a question

So, say you're putting together a robot dataset. You've collected some
recordings. You want to train a policy. Before that, you have a very reasonable
question: where does the interesting bit actually start?

The robot waits. Someone gets ready. Then it reaches for something. Then
there's another pause. You'd quite like to cut some of that out before you
spend your afternoon training a robot to wait patiently.

You don't need to solve all of robot learning today. You just want a little
heuristic: tell me when the hand starts moving, keep it active while it's
moving, and don't get excited about every tiny twitch.

So you ask an agent to write some Python. It works. You run it on more data.
Now you're waiting for your script to tell you where the robot was waiting.

That's where we'll start. A small computation that's taking longer than we'd
like. Then we'll do the usual thing that happens to a small robotics script:
ask it to read a bag, run it in ROS, and eventually wonder whether it could be
a controller. We'll finish by pointing a camera at something and moving it
around, because by then we should probably have something moving on screen.

*Show the stops: Python, MCAP, native library, rclpy, controller, webcam.
Leave the setup terminal ready in the background.*

## 1. This loop is annoying me, 7 minutes

Here's our little motion detector. Timestamps go in, hand positions go in,
and a mask comes out telling us which samples count as moving.

There are a couple of details. We have one speed to start moving and another
to stop. We also want the movement to last a little while before we believe
it. Otherwise we've built a very efficient noise detector.

I've supplied the boring bits in [task.py](examples/01_python/task.py): the
input checks, the example data, and the entry point. The agent writes the
missing logic. At this point I haven't told it anything about cppyy.

*Run the baseline prompt. Show the resulting function and its test result.*

<details>
<summary>Exact baseline prompt and acceleration prompt</summary>

First, [01_implement.txt](prompts/01_implement.txt):

```text
Implement motion_mask in task.py according to its docstring. Keep its signature
and the supplied validation. Do not change the tests or other files. Run the
acceptance tests and the 250,000-sample example. Report the results and timing.
This is Cartesian motion-event detection for finding useful intervals in robot
demonstrations. Work only in the supplied task directory. Use the interpreter
and pytest command provided by the evaluation runner.
```

Then use a fresh session with the generated implementation, explicitly supply
[the kernel guide](guides/kernel.md), and use
[01_accelerate.txt](prompts/01_accelerate.txt):

```text
Accelerate motion_mask in task.py using cppyy_kit. Preserve its signature,
validation, thresholds, event timing, and output dtype. Read the explicit kernel
guide supplied by the runner. Move the sequential computation into one native
call over the whole input, keeping Python for validation and output allocation.
Do not change tests or other files. Verify correctness and report warmed operation
time separately from first-call compilation time. Work only in the supplied task
directory. Use the interpreter and pytest command provided by the runner.
```

Both sessions can be reproduced with:

```bash
pixi run python scripts/run_agent_eval.py implement --run rehearsal_01_python
pixi run python scripts/run_agent_eval.py accelerate --run rehearsal_01_native --baseline rehearsal_01_python
```

</details>

The agent gave us a sensible [Python implementation](solutions/python_baseline.py).
It already uses NumPy for the displacement and speed calculations. There's
still a sequential loop deciding when a movement starts and ends.

For 250,000 samples, that took about 29 milliseconds in the
[original evaluation](EVALUATION.md#independent-computation-results).
Fine once. But now suppose we're trying lots of thresholds, going through
more recordings, or doing this repeatedly while data arrives. We'd like that
part to be cheaper.

We have options. We could try Numba, use Cython, or build a C++ extension with
pybind11. Those are all routes we could explore. I'm choosing cppyy because
I also want to see what happens when our next request involves a C++ library
or a native ROS interface. We can start with this loop and keep going.

For the moment, here's the whole idea in a tiny example:

```python
import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray

@cpp(nogil=True)
def sum_sq(data: ConstNDArray[np.float64]) -> float:
    "double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i]*data[i]; return s;"

assert sum_sq(np.array([1, 2, 3], dtype=np.float64)) == 14.0
```

That string is the C++ body. The decorator compiles it, and we call it from
Python. `ConstNDArray[np.float64]` supplies a `const double*` and the total
element count as `data_size`. A mutable output uses `NDArray[T]`. The array
must have the specified dtype, native byte order, alignment and C-contiguous
layout. The decorator rejects a mismatch; any normalization belongs in the
Python wrapper. It borrows the buffer for the call and caches the compiled
kernel for reuse. See the [buffer contract](../docs/COMMON_PATTERNS.md).

I'm going to let the agent write the longer version. I give it the guide,
ask it to keep the behavior, and ask it to move the loop into one native call.
The guide covers the buffer types and sizes so it has something concrete to
follow.

*Open [the native solution](solutions/python_native.py) beside the baseline.
Show the loop, then the Python call that runs it.*

```python
mask = np.zeros(n, dtype=np.int32)
if n < 2:
    return mask

_motion_mask_kernel(t, xyz.reshape(-1), n, enter_speed, exit_speed,
                    hold_seconds, mask)
return mask
```

In the [original evaluation](EVALUATION.md#independent-computation-results),
twelve supplied test cases passed and independent random checks matched.
The warmed computation went from about 29.3 milliseconds to 1.48.
About twenty times faster on this input. The migrated source needs its own
checks; these values describe the saved 0.3.0 run.

We did pay for compilation and startup. We'll show that separately. I want
the number for the loop once it's ready, and the number for getting it ready.
Both matter when we're deciding where to use it.

Now let's give this little script a real recording.

## 2. Someone gave us a humanoid bag, 8 minutes

I went looking for an open humanoid manipulation recording and found
[this G1 pillow episode](DATASET.md). It comes from HIW-500, by BitRobot,
Unitree, and Hugging Face. We have hand states, actions, cameras, grippers,
and other robot data. Plenty to explore.

Our question is still quite modest: when are the hands moving? How much does
that answer change if I adjust the speed threshold?

We could use that to suggest episode cuts, find candidate moments to inspect,
or compare recordings. Later we might look at the gap between the requested
action and the observed state, or find a camera stream that's gone suspiciously
still. There's a lot of useful work around training and inference that starts
with a question like this.

*Show the recorded scene and the topic inspection. Credit the dataset.*

First discovery: the hand poses we want are inside JSON, inside a ROS String.
So the loader in [analyze.py](examples/02_mcap/analyze.py) gives us arrays of
recorded timestamps and left/right hand states. There are 2,735 observations.

```python
payload = json.loads(decoded.data)
state = payload["ee_state"]
timestamps.append(message.log_time)
states.append(state)

timestamps = np.asarray(timestamps, dtype=np.int64)
t = (timestamps - timestamps[0]).astype(np.float64) / 1e9
```

That subtraction happens while the timestamps are still integers. Then we
convert to seconds. It's a small detail, but I'd quite like our motion detector
to use the time the data was recorded.

Now we ask the agent to take our rule and sweep 41 speed thresholds for both
hands. This time it can use cppyy_kit from the start.

<details>
<summary>Exact MCAP prompt and rehearsal commands</summary>

[02_mcap.txt](prompts/02_mcap.txt):

```text
Implement sweep() in analyze.py using cppyy_kit. Read the supplied kernel guide
and the motion-rule reference. The loading boilerplate reads an actual public
Unitree G1 MCAP. Sweep 41 enter-speed thresholds for both recorded end effectors
and return event counts and active durations, exactly as the docstring specifies.
Keep validation, file loading, and the output format. Do not change tests or
other files. Run the tests and analyze the supplied recording. Report decoding
time, first query time, and warmed query time separately. Do not describe motion
events as confirmed grasps or task success. Work only in the task directory.
```

```bash
pixi run python scripts/run_agent_eval.py mcap --run rehearsal_02_mcap
pixi run python scripts/verify.py
```

</details>

Here's the [rehearsal query](solutions/mcap_native.py), migrated from the saved
agent source. Once we've loaded the recording, trying another setting looks
like this:

```python
import numpy as np
from solutions.mcap_native import load_poses, sweep
ns, t, states = load_poses("data/hiw_pillow_episode_0002.mcap")
thresholds = np.linspace(.02, .20, 41)
counts, active_seconds = sweep(t, states, thresholds, hold_seconds=.02)
print(counts[0])  # [89, 82] at 0.02 m/s
```

And here's a useful surprise from actually trying it. With the original
120-millisecond confirmation time, we got zero events. At 20 milliseconds,
the lowest speed threshold gave us 89 events for the left hand and 82 for
the right.

So, have we discovered 171 successful grasps? We have discovered 171 things
our motion rule calls events. Let's look at some of them before we put those
labels into a training dataset.

Even the annotations gave us something to discuss: the episode name mentions
a sofa and floor, while the subtask labels mention a bed and chair. Welcome
to working with somebody else's data. We'll keep the source labels visible
and inspect the recording.

The [recorded query](EVALUATION.md#independent-computation-results) went from
about 24.8 milliseconds in Python to 1.14 in C++.
About twenty-two times faster. Loading and decoding the bag still took about
191 milliseconds in that run. So if I reload the whole file every time, that
part still dominates. If I've already got the arrays and I'm experimenting
with the rule, the faster query is useful straight away.

*Show loading time beside query time, then change a threshold. Pick a detected
window to inspect against the images.*

We started with a loop. Now it's a dataset tool. Before we put it into ROS,
let's add an operation from an existing C++ library.

## 3. Could we add a library we have not wrapped?, 4 minutes

Suppose our next experiment needs nearby Cartesian observations, but only
those with sufficient confidence and enough separation in recorded frames.
We also want the centroid of the selected observations.

SciPy already has a native nearest-neighbor tree. We can use it directly.
Here the question is whether an existing C++ tree and our custom selection
rule can run together in one native operation, while Python controls the
experiment.

The [nanoflann recipe](next_steps/nanoflann/LIBRARY_RECIPE.md) does that without
creating a nanoflann kit. It pins and checks the header, builds a small native
adapter, loads its declarations, and retains the index between queries.
Python validates arrays and allocates outputs. C++ owns a copy of indexed
storage, traverses the actual nanoflann tree, filters candidates and writes
neighbors, squared distances, counts and centroids into output arrays.

*Use the prepared [demo](next_steps/nanoflann/demo.py). Show the retained index
and query, then the native selection rule. Play a short recorded agent excerpt
if time permits. A complete agent run took 513 seconds, so do not wait for a
new implementation during this four-minute segment.*

```python
from index import Index

with Index(points, frame_ids, confidence) as index:
    result = index.query(query_points, query_frames, k=8, frame_gap=128,
                         min_confidence=.6, max_distance=.25)
    print(result["centroids"])
```

The [4 October experiment](next_steps/nanoflann/RESULTS.md#costs-and-useful-comparison)
used 50,000 synthetic positions and 2,000 queries. The warmed composed native
call took 3.19 milliseconds. The adaptive SciPy search plus Python selection
and centroid calculation took 31.49 milliseconds. Direct unfiltered searches
were close: 1.95 milliseconds native, including the centroid, and 2.32 for
SciPy without it. The larger difference came from combining the custom rule
with traversal and centroid calculation. It does not establish a general
nanoflann advantage over SciPy.

Index construction, adapter compilation and import cost are separate. The
independent brute-force checks include ties, empty batches, distance units and
owned storage lifetime. These inputs are synthetic; using the index on the
recorded dataset is a further experiment. One Sol-high agent completed the
provided skeleton with no human logic repair. That is one completion, not a
reliability estimate.

<details>
<summary>Prepared native-library demonstration</summary>

From `roscon_uk_2026/`, use the experiment's own Pixi environment:

```bash
pixi run --manifest-path next_steps/nanoflann/pixi.toml probe
pixi run --manifest-path next_steps/nanoflann/pixi.toml check
pixi run --manifest-path next_steps/nanoflann/pixi.toml demo
```

Run `benchmark` separately during rehearsal and preserve its output with the
environment revision. The historical [results](next_steps/nanoflann/RESULTS.md)
include first-call, build and import costs. The [prompt](next_steps/nanoflann/PROMPT.txt)
and saved [agent evidence](next_steps_evaluation/nanoflann_sol_high_01/report.json)
describe the original completion.

</details>

Now our next request is: can the motion query tell us what is happening while
the data is coming in?

## 4. Could we just put it in ROS?, 6 minutes

Of course we can put it in ROS. There's a subscriber already written in
[node.py](examples/03_ros/node.py). It decodes the message, keeps the latest
256 observations, and runs the same query on that window.

```python
history.append((t, state))
times = np.ascontiguousarray([item[0] for item in history])
states = np.ascontiguousarray([item[1] for item in history])
counts, seconds = rolling_query(times, states, thresholds)
```

Now our computation has a regular job: finish its work before the next round
of data gives it more to do.

I'll ask the agent to accelerate that function. The node still has the same
subscription, the same window, and the same question. We can compare its
answers before and after.

<details>
<summary>Exact rclpy prompt and playback checks</summary>

[03_ros.txt](prompts/03_ros.txt):

```text
Accelerate rolling_query() in this existing rclpy node with cppyy_kit. Read the
supplied kernel guide. Keep the subscription, topic, QoS depth, recorded time
handling, window reset semantics, returned arrays, and callback behavior.
Use one native call per window. Do not change tests or other files. Run the
acceptance tests and compare against the original Python query on randomized
windows. Measure first-call and warmed query times. Do not claim that these
offline checks establish ROS delivery; a separate replay check follows.
Work only in the task directory.
```

```bash
pixi run python scripts/make_replay.py
pixi run python scripts/run_agent_eval.py ros --run rehearsal_03_ros
pixi run -e ros python scripts/check_replay.py --baseline
pixi run -e ros python scripts/check_replay.py
```

For a two-terminal demonstration, start the node first. Both terminals need
the same ROS domain:

```bash
ROS_DOMAIN_ID=78 pixi run -e ros python solutions/ros_native.py
ROS_DOMAIN_ID=78 pixi run -e ros ros2 bag play -i data/poses_replay.mcap mcap --rate 4 --delay 1 --topics /roscon/poses --disable-keyboard-controls
```

The saved implementation is [ros_native.py](solutions/ros_native.py).

</details>

We're playing the recording at four times its original speed. The
[replay adapter](scripts/make_replay.py) puts the original timestamps and a
sequence number into each standard ROS message. Our speed calculation still
uses recorded time. Playing a bag faster shouldn't persuade the heuristic
that the robot grew faster arms.

These counts describe the current window. Each window starts the rule afresh,
so they're not a running total for the whole episode.

Both [original playback trials](EVALUATION.md#ros-playback) received all 2,735 messages, in order, and produced
matching event counts. The median callback went from about 2.84 milliseconds
to 0.50. There's still JSON decoding, checking, and assembling the window
around the accelerated computation. That's why the whole callback gets a
smaller speedup than the loop by itself.

*Show the replay, delivered-message count, and callback timings together.*

So far we've used C++ to make Python computation cheaper. This next bit is
why I wanted to keep exploring cppyy: we can also use Python to work with
an existing C++ interface.

## 5. At this point, someone asks for a controller, 7 minutes

We've got Python talking to native code. We have ROS running. So naturally
someone asks: could we write a ros2_control controller in Python?

Let's try it.

The skeleton in [controller.py](examples/04_control/controller.py) derives
from `ck.ControllerInterface`. That's the native controller interface,
exposed through control_kit. The lifecycle methods and interface declarations
are supplied. The agent fills in the update.

This example runs inside a real controller_manager, with mock hardware. We
read two joint positions, follow a moving reference, and write the commands
back through the claimed interfaces.

<details>
<summary>Exact controller prompt and rehearsal commands</summary>

Read the installed guide with `pixi run -e control python -m cppyy_kit guide control_kit api`,
or give the agent [the source guide](../control_kit/SKILL.md), then use
[04_control.txt](prompts/04_control.txt):

```text
Complete TrackingController.update() in controller.py using the supplied
control_kit guide. Use the period supplied by controller_manager, read the two
claimed position interfaces, apply the documented tracking law, and write the
commands. Preserve lifecycle and measurement boilerplate. Do not change tests
or other files. Run the acceptance test and a three-second 1000 Hz trial. Report
achieved rate, p99 cycle interval, late intervals, and tracking error. State that
the hardware is GenericSystem and that measured scheduling is not a hard real
time guarantee. Work only in the task directory.
```

```bash
pixi run agent-guide control
pixi run python scripts/run_agent_eval.py control --run rehearsal_04_control
pixi run -e control python solutions/control_native.py --rate 1000 --seconds 3
```

</details>

Here's the [update the agent wrote](solutions/control_native.py):

```python
dt = period.seconds()
self.t += dt
references = [0.3 * math.sin(self.t), 0.2 * math.cos(self.t)]
positions = []
for i in range(ck.n_state_interfaces(self)):
    position = ck.read_state(self, i)
    command = position + dt * 20.0 * (references[i] - position)
    ck.write_command(self, i, command)
    positions.append(position)
self.samples.append((self.t, references, positions))
return ck.return_type.OK
```

We've gone from accelerating a function to implementing a native framework
interface from Python. We're still in the same sort of agent workflow:
read the relevant guide, write the missing logic, and check what it did.

In the [original controller evaluation](EVALUATION.md#controller),
we asked for a thousand updates a second. The agent's three-second run
achieved about 996 Hz. The independent run, while other checks were active,
was about 989 Hz. Its tracking error in the second half was about 0.016 radians.

Now, before anyone makes this slide their robot's new control architecture:
look at the timing distribution too. That independent run had a 3.89-millisecond
p99 interval, and 203 intervals longer than 1.5 milliseconds. The average rate
is only part of the story.

The mock hardware mirrors position commands into state. It lets us check the
controller interface and this tracking law. A physical robot needs its own
dynamics, safety, and timing work. We've demonstrated something useful to
prototype; we haven't earned a hard real-time claim.

*Show the update rate, interval distribution, and tracking error on the same
slide. Keep the mock-hardware label visible.*

We've spent quite a while looking at arrays. Let's give ourselves something
we can actually wave at.

## 6. Can we make it follow this?, 5 minutes

Here's the webcam. I want to click on a little patch and follow it as I move
it around. A small image-processing task where we can see what the code is doing.

[tracker.py](examples/05_webcam/tracker.py) already handles capture, grayscale
conversion, mouse clicks, and drawing. The missing bit searches for the most
similar patch in the next frame: an 11-by-11 template, with a search of eight
pixels in each direction.

We ask the agent to write that operation with cppyy_kit.

<details>
<summary>Exact webcam prompt and rehearsal commands</summary>

[05_webcam.txt](prompts/05_webcam.txt):

```text
Implement track() in tracker.py using cppyy_kit and the supplied kernel guide.
Preserve exact integer SSD scoring, row-major tie breaking, clipping, dtype
validation, and return values. Keep capture and GUI boilerplate. Do not change
tests or other files. Run synthetic acceptance tests and report first-call and
warmed timings for an 11x11 template and +/-8-pixel search. Camera hardware is
not required for this agent evaluation. Do not describe synthetic results as a
live camera test. Work only in the task directory.
```

```bash
pixi run python scripts/run_agent_eval.py webcam --run rehearsal_05_webcam
pixi run -e vision python solutions/webcam_native.py --device 0
```

The saved implementation is [webcam_native.py](solutions/webcam_native.py).

</details>

Inside the search, we're comparing pixel values and adding up squared
differences. Here's the little detail that's easy to get wrong:

```cpp
const int difference = static_cast<int>(previous[previous_row + x + dx])
                     - static_cast<int>(current[current_row + cx + dx]);
score += static_cast<unsigned long long>(difference * difference);
```

Convert the pixel values before subtracting them, and use a wide accumulator.
The guide and the tests give the agent a way to check that. We also check
translation, image edges, ties, and buffers that aren't contiguous.

*Click a textured patch and move it slowly. Try a movement the tracker can
follow, then show what happens when it loses the patch.*

It's a small local tracker, so big jumps and occlusion can lose it, and it can
drift. That's useful to show too: we can see where this particular algorithm
stops helping and start changing it.

The [original synthetic search](EVALUATION.md#webcam) measured about 0.026 milliseconds once warm. Building
its kernel took about 546 milliseconds. The actual headless webcam check
tracked 30 frames at about 0.20 milliseconds per search. Camera content and
startup conditions differ between those measurements. The click-and-display
part still needs a rehearsal on the presentation machine.

## Closing: our small script acquired a few responsibilities

We asked a script to find some movement. Then it became a dataset query.
We added a native library and a custom selection rule. Then we wanted the
motion query in a ROS callback. Then we tried a controller. Finally,
we gave it an image-processing job we could poke at live.

That's what interests me about cppyy. If I only need to speed up a numerical
loop, I've got several tools I can try. But I often want to keep adding things:
a C++ library that's already doing useful work, a native ROS API, or a
controller interface I'd like to experiment with from Python.

cppyy_kit gives us the cached kernels and kit setup to work with those pieces.
The agent gets explicit guidance, and we get to spend more of the session
asking what the robot data means and what the program should do next.

The original five-stage agents took roughly 42 to 220 seconds when successful,
and setup failures remain in the evidence. Guide discovery and environment
diagnostics are now implemented in the 0.4.0 source. We have also evaluated
the new-library recipe. Publication, fresh evaluation of the migrated rehearsal
and repeated runs remain on [the plan](PLAN.md).

The same tools also help around existing C++ software: Python can validate
configuration, test resets and chunking, compare trial settings and generate
reports. The [native-component tutorial](../docs/tutorials/native_component.md)
is a concrete follow-up. The C++ component keeps its implementation and Python
organizes the experiment.

For now, take a bit of Python you wish was quicker. Ask your agent to implement
it, test it, and measure it. Then give it the cppyy_kit guide and try moving
the busy part into one native call. See whether the result is useful for your
problem. And if your next thought is “could we also add...”, that's the bit
I'd like to explore with you.

## Backstage: setup and the bits we need ready

These are rehearsal notes, separate from the spoken story. Commands in this
file require this repository checkout and run from `roscon_uk_2026/`.

<details>
<summary>Environment, data, and agent guidance</summary>

```bash
pixi install
pixi install -e ros
pixi install -e control
pixi install -e vision
pixi run agent-guide kernel
pixi run agent-guide control --path
pixi run python -m cppyy_kit guide accelerate
pixi run python -m cppyy_kit guide bring-library
pixi run python -m cppyy_kit guide existing-cpp
pixi run python -m cppyy_kit status --environment
pixi run fetch-data
pixi run inspect-data
```

This rehearsal uses the current source checkout through explicit Pixi
activation. [pixi.lock](pixi.lock) records its native dependencies. The original
0.3.0 evaluation and its package details remain archived separately; it is
not a validation of this environment. The platform is Linux x86-64. Preserve Pixi
activation, including its CXX setting. ROS execution needs local DDS communication.

The guidance command prints text or a path for the user to give the agent.
It installs no skills or agent configuration. `python -m cppyy_kit guide`
reads packaged task and kit resources without native startup. These commands
are new in 0.4.0; use this checkout until that version is published. Installed
artifact checks are recorded in the [integration report](../EXPERIMENT_INTEGRATION_2026-10-04.md).
Environment diagnostics locate the compiler, runtime versions and development
headers; they do not load Cling, check binary compatibility or prove local DDS
communication. Rehearse native import and ROS playback separately. See
[agent_guide.py](scripts/agent_guide.py) and [the kernel guide](guides/kernel.md).

The [runner](scripts/run_agent_eval.py) creates task directories inside this
folder and uses fresh GPT-6 Luna sessions with high reasoning. It consumes
authenticated Codex usage. Its exact prompts and acceptance results are
preserved with the [evaluation record](EVALUATION.md).

</details>

<details>
<summary>Rehearsal gates and honest comparisons</summary>

Use [EVALUATION.md](EVALUATION.md) for the measured conditions and failed trials.
Its results describe the historical saved sources. Revalidate the migrated
solutions with `pixi run check` and `pixi run python scripts/verify.py` before
rehearsal. Repeated one-shot reliability remains to be evaluated.
Repeat each prompt three times in fresh sessions, plus one changed-requirement
case. Check cold and warm startup. Rehearse the webcam GUI and the whole
sequence on the presentation machine.

Keep a recorded agent run and the saved solution ready for each stage. If a live
attempt overruns, tell the audience and switch to the recorded attempt, showing
its prompt and checks. Leave enough time to discuss the result.

The alternatives mentioned in the story have their own documented workflows:
[Numba's numerical JIT](https://numba.readthedocs.io/en/stable/user/5minguide.html),
[building Cython extensions](https://cython.readthedocs.io/en/latest/src/quickstart/build.html),
and [pybind11 C++ bindings](https://pybind11.readthedocs.io/en/stable/basics.html).
The recorded benchmarks compare our Python and cppyy_kit implementations.
We haven't run a performance comparison against those other tools.

Project references: [README](../README.md),
[common patterns](../docs/COMMON_PATTERNS.md),
[cache behavior](../docs/FREEZE.md), and
[control_kit](../control_kit/SKILL.md).
Start agent requests with [CPPYY_KIT_WITH_AI.md](../CPPYY_KIT_WITH_AI.md) and
the [task guides](../docs/GUIDES.md).

</details>

<details>
<summary>Extensions once the main story is ready</summary>

The [nanoflann recipe](next_steps/nanoflann/LIBRARY_RECIPE.md) supplies the
prepared new-library segment. A larger follow-up can apply it to recorded
Cartesian positions and compare the same selection rule on that data. Keep
its build, retained storage, query and teardown contracts visible.

For recorded images, try blur or frozen-image scores, timing JPEG decoding
separately from custom computation. For FK, use joint states with a pinned URDF,
an explicit motor-to-joint mapping, and verified frames and target links.
The current pose query uses the recorded coordinate convention; it doesn't
establish world-frame end-effector poses. These extensions are planned rather
than part of the measured five-stage sequence.

</details>
