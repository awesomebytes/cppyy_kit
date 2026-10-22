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
viewer on the right. Keep marimo open in another tab. Use the agent prompt in the
[Presentation section of the rendering guide](next_steps/webcam_session/toolbox/rendering/README.md#presentation-fresh-live-coding-with-an-mcap-fallback).
Ask for a task, inspect the result, then refine it. The agent should edit the
processing cells while keeping the camera, viewer and loaded models alive.

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
