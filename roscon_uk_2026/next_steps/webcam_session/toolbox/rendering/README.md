# Run the interactive webcam demo

Use the saved [depth notebook](depth_notebook.py) for calibrated relative depth, face/hand/body landmarks, foreground extraction, hand-controlled navigation and native rain. [SESSION_HANDOFF.md](SESSION_HANDOFF.md) is the current reference for dependencies, feature behaviour, code, measurements and complete shutdown. The native image-only example is documented below.

Commands require this repository checkout. Start from the repository root:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh edit \
  roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/depth_notebook.py \
  --host 127.0.0.1 --port 2721 --no-token --no-sandbox
```

Open http://127.0.0.1:2721 and run the notebook cells once. Keep that tab open. Put the agent terminal on the left and http://127.0.0.1:8768 on the right. The viewer displays RGB, depth, a cloud and feature controls. Raised-hand filtering can hide a person until a wrist is raised; check feature toggles if the display appears empty.

Use the locked depth and people environments from [the toolbox guide](../README.md). `bash roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/setup.sh` installs the rendering overlay and verifies/downloads the two face assets. Keep the face worker in its separate people environment. Camera calibration is specific to this webcam and capture mode.

## Prompt for a new agent

```text
Work directly in /home/sapf/playground/ros_project/cppyy_kit.
Read roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/SESSION_HANDOFF.md
and the rendering README.md. They describe our accepted webcam experiment.

First discover whether marimo at http://127.0.0.1:2721 is running.
If it is, connect to the existing notebook session and inspect its health.
If it is not, follow the documented startup command, open its browser tab,
run the cells once, and verify the viewer at http://127.0.0.1:8768.
Discover session IDs each time through rendering/notebook.sh pair notebook list.
Keep the marimo tab open and put Viser beside this terminal.

Check all notebook cell errors, workbench and viewer status, and face,
gesture and rain workers. Then ask what I want to try.

Retain resource-owning cells. Edit accepted code through marimo code mode,
not external writes to the active notebook. Before editing a cell, inspect
its source and downstream dependencies. Stop/join old animation or gesture
workers and remove their handles before replacing their runtime cells.
Use controls for parameter changes. Do not start a second camera owner.

Keep models in their prepared environments. Use cppyy_kit for useful native
operations; the depth unprojection and rain kernel already demonstrate it.
Preserve source-frame alignment and distinguish raw, undistorted and depth-grid
coordinates. Small-model depth is relative. Inspect actual output and save
matching examples, errors and accepted source for the eventual implementation brief.

Keep changes minimal. Check errors and measured rates after each change.
On shutdown, follow the full sequence in SESSION_HANDOFF.md, including the
face subprocess and animation workers. Do not leave background workers running.
```

## Presentation: fresh live coding, with an MCAP fallback

Start the minimal webcam notebook from the repository root:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh edit \
  roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/live_notebook.py \
  --host 127.0.0.1 --port 2721 --no-token --no-sandbox
```

Open `http://127.0.0.1:2721`, run the cells and keep the tab open. Put the agent terminal on the left and `http://127.0.0.1:8768` on the right. This notebook starts with camera capture, pass-through processing and Viser. Choose and implement the perception task live. The file has been syntax-checked; rehearse fresh camera/viewer startup before using it on stage.

Use this prompt for the presentation session:

```text
Work in /home/sapf/playground/ros_project/cppyy_kit.
Read the Presentation section of:
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/README.md
and toolbox/README.md for the prepared model environments.

Use rendering/live_notebook.py as the starting point. Discover whether the
marimo session at http://127.0.0.1:2721 is running; start it with the documented
command if needed. Run its cells and verify camera/viewer health. Keep the
marimo tab open and put Viser at http://127.0.0.1:8768 beside this terminal.
Discover the current session ID through notebook.sh pair notebook list.

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

Start with the webcam stream, then ask me what task to implement. Do not load
or demonstrate the completed depth_notebook experiment. Keep camera and viewer
owners alive. Retain models once loaded. Inspect cell dependencies before
editing. Save accepted edits through marimo code mode, not external writes to
the active notebook. Stop/join workers before replacing their runtime cells.
Inspect installed modules, local assets and retained models first. For face
detection, start with dlib as described in toolbox/README.md; keep all other
backends available and change when the task or measurements justify it.
Add controls only when the current processing uses them.
Use cppyy_kit where a native computation benefits the chosen task.

After each change, inspect output, errors and rates. Preserve useful examples
and accepted source. If the webcam setup cannot work, use the MCAP fallback
below and interactively detect blocks on the table.
```

The fallback is the robot MCAP in a separate marimo notebook:

```bash
roscon_uk_2026/next_steps/agent_session/notebook.sh edit \
  roscon_uk_2026/next_steps/agent_session/notebook.py \
  --host 127.0.0.1 --port 2718 --no-token --no-sandbox
```

Open `http://127.0.0.1:2718`. See [the notebook guide](../../../agent_session/NOTEBOOK.md) for dependencies, agent access and data details. The notebook reads `data/table_scene`; it starts with a red-object selection. Inspect it, choose a block/color, show its image region and 3D cluster, then refine. It retains the recorded input while analysis changes. Keep this tab ready before the talk. This is interactive work on recorded sensor data, not video playback.

For the minimal webcam notebook, close `viewer` and then `workbench` before stopping the server. If the live task adds models or workers, stop and join those resources too. Keep one owner of camera 0.

## Run a native webcam workbench with a persistent interpreter

Use Dear ImGui for a native image window and controls, and optional Viser for
3D clouds and scene interaction. Marimo owns the camera, models, processing
function and shared parameters. Updating processing does not recreate these
resources. Commands below require this repository checkout.

The optional rendering packages and combined model environment are prepared on
this machine. Reproduce them with the [toolbox setup](../README.md), then:

```bash
bash roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/setup.sh
```

Stop the previous session's camera with `camera.close()` before starting a second
camera owner. Start the new notebook:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh edit \
  roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.py \
  --host 127.0.0.1 --port 2721 --no-token --no-sandbox
```

Open <http://127.0.0.1:2721> and run the cells once. A native **Interactive
perception** window opens. Select **Processed** to see the starter overlay;
**Raw camera** shows capture independently of inference. The threshold slider,
label text and enabled checkbox change the processing parameters. Zoom/pan the
image and inspect pixel values. Click an image pixel to store `point=(x,y)` and
`point_sequence` in the controls. Coordinates refer to the displayed image.

This native window is verified on this Linux machine. Its graphics loop runs
on a separate thread, with no JPEG encoding or HTTP polling. Closing the window
leaves capture and inference running. Call `viewer.close()` and then
`workbench.close()` when done. Close any optional Viser viewer too.

## Give a new agent this prompt

> Read roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/README.md and
> toolbox/README.md. Connect to the existing marimo session at port 2721 through
> rendering/notebook.sh pair. Ask what I want to detect. Keep the session and
> native_view cells alive. Load models in retained cells, then edit processing
> through code mode. Inspect workbench.status() and viewer.status() after changes.
> Keep source timestamps, inspect results and preserve accepted code in notebook
> cells. Add GUI controls through viewer.set_gui when useful. Use prepared models.

Find the session:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh pair notebook list \
  --url http://127.0.0.1:2721
```

Use the returned session ID with `pair execute`. The
[original agent instructions](../../README.md#agent-commands) describe code-mode
cell edits. Do not edit the notebook file externally while it is running.

The processing signature is `process(frame, state, controls)`. Return a BGR uint8
image, or a dictionary containing `image` and optional `points` (Nx3) and `colors`
(Nx3 RGB uint8). The worker supplies a private frame copy. Treat published output
arrays as immutable. `state` survives processor changes; mutate it only from the
processing worker. Controls are copied per frame. Prepare new models separately
before installing a closure that uses them. A frame already running finishes with
the previous function; status reports the result's actual revision.

For example, retain `face_model = People('face')` in one cell, then install:

```python
def process(frame, state, controls):
    if controls['enabled']:
        for face in face_model(frame):
            x, y, w, h = face[:4].astype(int)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    return frame

workbench.set_processor(process)
```

To add a live control, put `from imgui_bundle import imgui` in a retained import
cell and save a callback in another cell:

```python
def extra_controls(workbench):
    values = workbench.controls()
    changed, value = imgui.slider_int('Minimum area', values.get('min_area', 100), 0, 5000)
    if changed:
        workbench.configure(min_area=value)

viewer.set_gui(extra_controls)
```

Callbacks run on the graphics thread; keep inference and compilation outside them.
Changing their function does not restart the viewer. Pair ImGui begin/end calls
correctly when adding custom panels. `workbench.snapshot()` gives an owned raw
frame copy for inspection. `latest(processed=True)` returns the most recent result
packet, including source sequence and timestamp; do not modify its arrays.

## Optional Viser 3D view

In a new retained cell:

```python
scene = ViserViewer(workbench, port=8768)
```

Open <http://127.0.0.1:8768>. Return `points` and `colors` from processing to show
the cloud. It has controls, orbiting, point size, fit-to-cloud and scene ray picking.
See [Viser usage and verification](VISER.md). Its image path still uses browser
transport/encoding; use the native window for the local webcam preview.

Viser provides the interactive 3D view in this integration. Its control callbacks
and browser accessibility fit the notebook workflow. Polyscope remains another
reasonable native 3D option if we need a local mesh/geometry viewer later.

## Rates and evidence

Capture and processing use separate threads. At most one latest frame and one
result are retained by the workbench. Slow inference skips intermediate input
frames rather than building a queue. Raw preview follows capture; processed
preview follows inference. An overlay is shown on its source image, not a newer
camera frame. Source timestamps are host timestamps after acquisition; they do
not measure sensor exposure or display presentation latency.

Verified through real marimo `pair` cell edits, using a saved webcam image paced
at 30 Hz so the user's active camera was not taken. With a deliberate 100 ms
processing delay, a five-second interval measured:

| Operation | Rate |
| --- | ---: |
| Capture source | 29.8 Hz |
| Processing | 10.0 Hz |
| Native rendering loop | 59.6 Hz |
| Distinct images submitted to native rendering | 29.6 Hz |

See [raw timing evidence](evidence/slow_processing.json). These are software
counters, not monitor scanout measurements. The native framebuffer was inspected.
The processing cell was edited and restored without replacing camera/session
state. Viser was separately checked in Chromium with a saved 32,079-point cloud
and working text/threshold/ray callbacks. Its browser check is functional, not a
frame-rate benchmark.

Python code holding the GIL or native compilation can still pause threads.
The 100 ms delay used `time.sleep`, which releases the GIL. Heavy kernels should
release the GIL or execute in an appropriate native backend. This integration
keeps capture/render/inference scheduling separate; it does not guarantee latency
for arbitrary user code.

To rehearse without opening the camera, set `PERCEPTION_REPLAY_IMAGE` to an image
path before the launch command. This repeats that image at approximately 30 Hz;
it is explicitly a replay source, not live capture.

See [the live DA3 comparison](DEPTH_COMPARISON.md) for RGB, depth and cloud in one viewer, launch commands, the notebook and measured throughput.

See [phone calibration and Metric-Large measurements](CALIBRATION.md) for the checkerboard workflow, downloaded model and capture-rate investigation.

Persistent calibrated Viser session verification: processing-cell replacement preserved process, session and state identities, with no errors. A five-second sample measured approximately 30 captured and processed frames/s. A separate ten-second hardware-accelerated browser sample measured 27.0 RGB and 27.3 depth image loads/s, with 60.1 animation callbacks/s. These are separate software measurements, not screen latency. See [session evidence](evidence/side_by_side_verified.json) and [browser evidence](evidence/side_by_side_browser_30fps.json).

### Face detection in the notebook process

The rendering setup installs dlib for the depth notebook's Python 3.12 into
`build/webcam_session/rendering_python312`, which the launcher already includes.
Use `import dlib` and retain `dlib.get_frontal_face_detector()` in a model cell.
The older `build/webcam_session/dlib_python` build also targets Python 3.12;
the launcher supplies the rendering overlay so no manual path change is needed. Run `rendering/setup.sh` from its parent path
when preparing a new machine; setup checks a direct detector call. No separate
face process is needed for dlib. The 68-point predictor is needed only for
landmarks, not for face boxes.

### Python and calibration for new sessions

New depth/rendering and segmentation sessions use Python 3.12, matching the
people environment and supported cppyy_kit packages. Their launcher selects
the Pixi `python312` environment. This separate prefix lets an existing 3.11
interpreter finish without replacing its libraries. A running interpreter must
be stopped and restarted once to change Python versions. The rendering overlay
is `build/webcam_session/rendering_python312`; do not mix it with the old 3.11
`rendering_python` directory.

Reuse `build/webcam_session/calibration_refined.json` for camera 0 at 640 × 360.
The committed fallback is `rendering/evidence/webcam_calibration_refined.json`.
It contains the camera matrix and distortion coefficients from 26 views, with
0.146 pixel fitted reprojection RMS. Check the physical camera and frame mode
before use; scale intrinsics for resizing and account for cropping. The existing
`DepthPipeline(calibration=...)` applies cached undistortion and scales the matrix
to the depth grid. Calibration does not turn relative depth into metric depth.

Verified on 22 October 2026 with Python 3.12.15: direct dlib 20.0.1 detector
call, cppyy_kit native compilation/call, DA3-Small CUDA inference (252 × 336
depth), YOLOE CUDA inference (person detection), EdgeSAM mask generation and
AprilTag detector construction in the segmentation environment, and Viser
publication of synthetic frames. Existing downloaded model assets were reused.
These are execution checks, not a new accuracy or throughput benchmark. No
camera was opened for these migration checks.
