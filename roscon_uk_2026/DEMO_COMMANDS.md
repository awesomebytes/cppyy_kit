# Demo commands

These commands require this repository checkout and its prepared environments.
Run one robot demo at a time. **Ctrl-C in its terminal stops it; wait for
`Demo stopped.` before starting the next.** Each robot command opens Gazebo and
RViz. Omit `--gazebo-gui` if you only want RViz.

## 1. Navigation: original behavior

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
./roscon_uk_2026/run_robot_demo.sh baseline --gazebo-gui
```

Wait for **Navigation: active** in RViz's Navigation 2 panel. Localization can
remain inactive because this demo uses simulator-truth localization. Choose
**Nav2 Goal**, then click and drag to set a goal and its final heading. A useful
comparison goal is approximately `(x=-1.85, y=1.35)`, facing east, in `map`.
The robot initially faces north and can travel sideways. Stop with Ctrl-C.

## 2. Navigation: forward-facing Python policy

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
./roscon_uk_2026/run_robot_demo.sh forward --gazebo-gui
```

Wait for navigation to become active and send the same goal. The robot should
turn toward the path before translating. The Python policy runs inside Nav2's
controller server. Stop with Ctrl-C and wait for cleanup.

## 3. Python admittance controller

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
./roscon_uk_2026/run_robot_demo.sh admittance --gazebo-gui
```

Wait for `RVIZ_FORCE_READY` in the terminal. Choose **Interact** in RViz and drag
an axis of the orange handle at the left hand. A 10 cm drag requests 2 N of
virtual force. The arm should move slightly; its reference displacement is
limited to 3 cm. Release the mouse to remove the force and let the arm return.

Stop with Ctrl-C. Cleanup zeros the force and restores the stock arm controller
before shutting down the simulator. Let it finish.

## 4. Live webcam coding

Stop any previous webcam session first: only one process should own camera 0.
In a terminal, start the minimal notebook:

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh edit \
  roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/live_notebook.py \
  --host 127.0.0.1 --port 2721 --no-token --no-sandbox
```

Leave that terminal running. From another terminal, open marimo:

```bash
xdg-open http://127.0.0.1:2721
```

**Run the notebook cells once and keep the browser tab open.** The cells create
the camera, processing function and viewer. Then open the viewer:

```bash
xdg-open http://127.0.0.1:8768
```

You should see the webcam stream. Put the agent terminal on the left and the
viewer on the right. Keep marimo open in another tab.

### Paste this into the new Codex session

```text
We are doing a live webcam coding demo. Work in:
/home/sapf/playground/ros_project/cppyy_kit

Read the Presentation section of:
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/README.md
Also read:
roscon_uk_2026/next_steps/webcam_session/toolbox/README.md

First run this read-only context command from the repository root and use its
instructions and local asset inventory:
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/context.sh

Possible tasks include face boxes/landmarks, hand gestures, body skeletons,
depth/colored point clouds, object tracking and click-prompt segmentation.
Start with dlib for face boxes, MediaPipe for hands/pose/dense landmarks, and
DA3-Small on CUDA for live depth. Keep the other prepared backends available;
choose from the task and measured results, not a fixed demonstration sequence.

Use this notebook as the starting point:
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/live_notebook.py

First check whether marimo is running at http://127.0.0.1:2721.
Inspect sys.version and sys.executable in any existing session. This demo
requires Python 3.12. Do not continue in a retained 3.11 interpreter; explain
that it must be stopped and relaunched with the current documented command.
If it is not, start it using the documented command. If it is running,
connect to that session. Discover its session ID using notebook.sh pair
notebook list --url http://127.0.0.1:2721. Run the cells if needed and verify
that the camera and Viser viewer at http://127.0.0.1:8768 are healthy.
Tell me if I need to open or run something in the browser. Keep the marimo
tab open; the viewer sits beside this terminal.

Start with the plain webcam stream. Ask me what I want to implement, then
build and refine it interactively from my requests. Use the prepared model
environments when useful. For face detection, start with dlib as described in
toolbox/README.md; it was preferred in the earlier webcam exercise.
Inspect the active Python version, installed packages, downloaded assets and
retained model objects before choosing an implementation. Prefer direct dlib
from the prepared rendering environment for face boxes. Keep YuNet, MediaPipe
and other backends available; switch when the task or measured results justify
it. Reuse local assets before downloading or building anything. A subprocess
is a fallback for an actual compatibility need, not the default. Never load a
native extension built for a different Python ABI. Add UI
controls only when the current processing uses them. Use cppyy_kit for computations that benefit from
native code; explain briefly where it helps.

Keep the camera, viewer and loaded models alive between processing edits.
Do not start a second camera owner. Inspect cell dependencies before editing.
Use marimo pair/code mode to save cell edits, not external file writes to the
active notebook. Stop and join workers before replacing their runtime cells.
Check live output, errors and capture/processing rates after each change.
Do not load the completed depth_notebook experiment as the demo.

Save the accepted working code. On request, capture representative inputs,
outputs, parameters and dependencies for a later compiled implementation.
Do not restart or shut down
the session unless needed or requested; explain any required restart first.

The prepared webcam calibration is build/webcam_session/calibration_refined.json
(camera 0, 640x360). The committed copy is:
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/evidence/webcam_calibration_refined.json
Reuse it for undistortion and projection when relevant. Check capture mode and
account for resizing/cropping; calibration alone does not make relative depth metric.

Before implementing native processing, run these commands from the repository
root using the prepared notebook environment:

pixi run -e python312 -m roscon_uk_2026/next_steps/webcam_session/toolbox/depth/pixi.toml python -m cppyy_kit status --environment
pixi run -e python312 -m roscon_uk_2026/next_steps/webcam_session/toolbox/depth/pixi.toml python -m cppyy_kit guide
pixi run -e python312 -m roscon_uk_2026/next_steps/webcam_session/toolbox/depth/pixi.toml python -m cppyy_kit guide accelerate

For an existing C++ library, also read `python -m cppyy_kit guide bring-library`
in that same environment. For a relevant kit, use `guide KIT_NAME api` after
checking that the kit and its native dependencies are available. Read
cppyy_kit/agent_guides/bring-library.md and docs/COMMON_PATTERNS.md as needed.
Use these instructions for cached compilation, @cpp array contracts, headers,
preamble, library paths, nogil, ownership/keep_alive and safe native boundaries.
Load models and compile/warm kernels once, outside the frame callback. Keep
native objects and buffers alive for their required lifetime. Measure first-use
and steady-state costs separately. Reuse existing model bindings; use cppyy_kit
where a native operation helps the task, rather than adding C++ to every step.

First report session health, then ask me what I want to try.
```

To list the live notebook sessions for agent access:

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh pair notebook list \
  --url http://127.0.0.1:2721
```

If the viewer is unavailable, check that the notebook cells ran without errors.
Do not launch a second camera session to fix it.

To stop, ask the agent to stop/join any added workers and execute these calls in
the live notebook, in this order:

```python
viewer.close()
workbench.close()
```

Then press Ctrl-C in the terminal running marimo. Save accepted code before
shutdown. Closing only the viewer tab does not stop the camera.

## 5. Fallback: recorded robot data in marimo

Use this if the webcam demo cannot run. It uses the local recording in
`data/table_scene`, with RGB and point-cloud data of the table objects.

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
roscon_uk_2026/next_steps/agent_session/notebook.sh edit \
  roscon_uk_2026/next_steps/agent_session/notebook.py \
  --host 127.0.0.1 --port 2718 --no-token --no-sandbox
```

Leave that terminal running. In another terminal:

```bash
xdg-open http://127.0.0.1:2718
```

Run the cells and keep the tab open. The saved notebook starts with a red-object
selection. Ask the agent to inspect the scene, select a block by color, show its
image region and 3D cluster, then refine the result. The recording stays fixed
while the analysis changes. Save accepted edits and press Ctrl-C in the marimo
terminal to stop. See the [notebook guide](next_steps/agent_session/NOTEBOOK.md)
for session access and data setup.

### Paste this if switching the agent to the MCAP fallback

```text
Switch to the recorded robot-data demo. Work in:
/home/sapf/playground/ros_project/cppyy_kit

Read roscon_uk_2026/next_steps/agent_session/NOTEBOOK.md.
Use roscon_uk_2026/next_steps/agent_session/notebook.py and the local recording
in data/table_scene. Connect to marimo at http://127.0.0.1:2718, or start it
using the documented command if absent. Discover its session and run the
cells if needed. Tell me if I need to open or run something in the browser.

Inspect the current scene, notebook code and controls. It starts with a
red-object selection. Ask me which block/color to find. Show the selected
image region and corresponding 3D cluster, then refine the selection from
my feedback. Keep the loaded data alive while changing analysis. Use marimo
pair/code mode for saved edits and check outputs/errors after each change.
Use cppyy_kit where native processing helps. Preserve accepted source,
parameters and representative input/output examples for later reuse.

First report session/data health, then ask me which object to find.
```
