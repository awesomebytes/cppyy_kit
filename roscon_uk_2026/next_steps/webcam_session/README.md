# Develop live webcam processing through a persistent marimo kernel

Run these commands from this repository checkout. The setup uses the existing
`pick-place` environment and the optional marimo 0.25.2 installation shared with
the recorded-data notebook. If that installation is missing, follow the
[notebook setup](../agent_session/NOTEBOOK.md).

```bash
roscon_uk_2026/next_steps/agent_session/notebook.sh edit \
  roscon_uk_2026/next_steps/webcam_session/notebook.py \
  --host 127.0.0.1 --port 2719 --no-token --no-sandbox
```

Open <http://127.0.0.1:2719> and run all cells once. Open
<http://127.0.0.1:8766> in a separate browser window. It shows camera 0 with a
native OpenCV mean-intensity overlay and current processing status. The capture
and processing worker lives in the notebook's Python process. The viewer polls
at up to 10 Hz; the worker runs at the available camera/processing rate.

If the viewer is already running, reuse its existing kernel session rather than
starting another camera. Discover that session with `pair notebook list`.
Keep the notebook browser connection open. Start a new Codex session in this
checkout with:

> Read roscon_uk_2026/next_steps/webcam_session/README.md. Connect to the running
> marimo notebook at http://127.0.0.1:2719 through its pair interface. Inspect
> camera.status() and the existing cells. Help me develop webcam processing
> interactively using cppyy_kit/cv_kit. Ask what I want to detect first. Modify
> the processing cell through code mode, keeping the camera session alive.
> Inspect output images and errors after each change. Preserve accepted code in
> the notebook. Keep the implementation small and imports at the top. Use the
> existing environment. Do not restart the kernel or rerun its camera/setup
> cells to change the processing.

A bounded first request could be: “Highlight red objects, draw their bounding
boxes and show their count. Then let me change the color and minimum area.”

## Agent commands

Discover the session; do not reuse a session ID from an earlier run:

```bash
roscon_uk_2026/next_steps/agent_session/notebook.sh pair notebook list \
  --url http://127.0.0.1:2719

roscon_uk_2026/next_steps/agent_session/notebook.sh pair execute \
  --url http://127.0.0.1:2719 --session <session_id> \
  -c 'print(camera.status())'
```

Read and edit the processing cell with the installed code-mode API:

```python
import marimo._code_mode as cm

async with cm.get_context() as ctx:
    cell = ctx.cells["processing"]
    print(cell.code)
    print(cell.status, cell.errors)
```

Pass this code with `pair execute --code-file <path>`, or `--code-file -` and a
shell heredoc. For a change, call `ctx.edit_cell(cell.id, updated_source)` then
`ctx.run_cell(cell.id)` inside the context. Read the existing source first.
Only code-mode cell edits persist. Scratchpad bindings are temporary. Keep
helper functions and imports in saved notebook cells; do not edit the notebook
file externally while its session is active. The private API is pinned to
marimo 0.25.2. `pair --help` and `help(cm)` describe the installed interface.

The processing cell defines `process(frame, state)`, warms its native operation,
then calls `camera.set_processor(process, label="...")`. Only rerun that cell
for processing changes. Existing camera and state identities should remain
constant while `revision` increases and `frames` continues advancing.
Check cell errors as well as the command's success flag.

Inspect the current output without executing notebook code:

```bash
curl -fsS http://127.0.0.1:8766/status
curl -fsS http://127.0.0.1:8766/frame.jpg -o build/webcam_session/preview.jpg
```

The agent can open that image with its image-viewing tool. Within the kernel,
`camera.snapshot()` returns a copy of the latest raw BGR frame. Save selected
inputs, reviewed outputs and failure cases as the behavior becomes clearer.
Do not treat the reference output alone as a correctness check.

## Processing and retained state

Input and output are nonempty HxWx3 uint8 BGR NumPy images. The worker retains
`camera.state` across function replacements. Use it for temporal state updated
by the worker. Prepare expensive new models or native objects in a notebook
cell, then capture them in the replacement function. Avoid mutating objects
that the active worker is using. The function pointer changes between frames;
an in-progress frame finishes with the previous function.

The starter uses `cv_kit.bringup_cv()` and `numpy_to_mat()` to call native
`cv::mean`. Keep the NumPy owner alive while a native view references its buffer.
For changed JIT definitions use a fresh C++ namespace, or reuse stable compiled
functions with different parameters. Existing native definitions remain loaded.

Processing exceptions appear in `/status` and the viewer shows the raw frame;
a corrected function can be installed in the same process. Compilation or
Python work holding the GIL can pause capture and display. The kernel does not
provide a latency guarantee. `processing_ms` excludes capture, JPEG encoding,
browser transfer and display. Native process crashes require a restart.

The repository's MediaPipe configuration is in the separate `pipeline`
environment, which uses a different OpenCV distribution. Check that setup before
adding a pose model to the `pick-place` environment; use the current OpenCV
integration for the initial exercise.

Call `camera.close()` through the agent or notebook when finished. Then stop
the marimo server with Ctrl-C. To select another camera or viewer port, change
the camera cell before starting the session. Stop the existing camera before
intentionally replacing that cell.

## Verified on this machine

The webcam ran through the real marimo agent interface and browser viewer.
A code-mode edit changed the native overlay and installed revision 2 while the
PID, camera identity and state identity remained unchanged. Frame counts advanced
and both captured status reports had no error. See [before](evidence/before.json)
and [after](evidence/after.json). Browser verification confirmed a decoded live
image and updated status text. This checks the interactive mechanism; further
perception tasks are developed in the session.

## Next exercise: recorded robot data

Use the [MCAP notebook](../agent_session/NOTEBOOK.md) at port 2718. It uses the
same marimo pair interface and shell wrapper. The camera notebook keeps live
resources running; the MCAP notebook keeps its recorded RGB/cloud inputs loaded
while the agent revises processing and saves accepted source.

See the [perception tool inventory and integration plan](PERCEPTION_TOOLS.md) for
MediaPipe, Depth Anything 3 and image/depth/cloud association.
