# Inspect images and point clouds with Viser

Use `ViserViewer` beside a retained marimo workbench. It starts a local browser
viewer without opening another camera or replacing the processing worker.
These examples require this repository checkout and the rendering dependency
setup in `build/webcam_session/rendering_python` (Viser 1.1.1).

```python
from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.viser_view import ViserViewer

viewer = ViserViewer(workbench, port=8768)
```

Open <http://127.0.0.1:8768>. The GUI shows the latest image, a raw/processed
selector, a separate estimated-depth image when `depth_image` is present, point
size, and the workbench's processing controls. The 3D scene
shows `points` and RGB `colors` returned by the processor. Drag to orbit; use
**Fit cloud** to frame its current extent. Axes use the input point coordinates;
the initial camera uses OpenCV's +X right, +Y down, +Z forward convention.

Boolean controls become checkboxes. Strings become text inputs. Floats initially
between zero and one become sliders; other numbers become numeric inputs.
Changes call `workbench.configure(...)`; external changes are reflected in the
GUI. New scalar controls added through `configure` appear automatically.

Enable **Pick a 3D ray**, then click the scene. The viewer sends
`configure(ray={'origin': (x,y,z), 'direction': (dx,dy,dz)})` in scene coordinates,
then disables picking. The processor can intersect this ray with its geometry.
This is a ray selection, not an inferred image pixel or measured surface point.

The viewer reads `latest(processed=...)` and refreshes at up to 30 Hz by default (`update_hz` controls the limit). It retains
one scene cloud and image, updating their data. It displays source sequence and
age at server publication from the frame timestamp. This is not browser display age. Capture, processing and viewer rates can differ.
Changing the processor through marimo does not recreate this viewer.

```python
viewer.close()
```

Closing the viewer leaves camera capture, models and processing alive. Recreate
it when changing the viewer implementation itself. Backend exceptions appear
in the status panel rather than silently stopping its update thread.

## Verification

On 20 October 2026, the viewer served a saved webcam image and its 32,079-point
selected person cloud. Headless Chromium connected, decoded the image, rendered
the scene and showed the processing controls. Backend-to-GUI threshold updates
and callback-to-backend text/ray updates were checked. The camera was not opened.
Local artifacts: `build/webcam_session/viser_smoke.png` and `viser_smoke.log`.
Headless Chromium used software WebGL, so this check is not a GPU rendering
performance measurement.

References: [Viser server](https://viser.studio/main/api/core/server/),
[GUI callbacks](https://viser.studio/main/examples/gui/callbacks/),
[scene API](https://viser.studio/main/api/core/scene_api/).

## Measure publication and browser updates

`viewer.status()` reports RGB/depth/cloud publication counts and median/p95
server preparation time for the last 600 updates that published new data. This
includes JPEG preparation and native message construction; it excludes browser
transfer, decoding and drawing.

`measure_viser_browser.py` connects to a running Viser and counts decoded GUI
image loads over a specified interval. Its animation callback frequency measures
browser scheduling, not distinct 3D frames or GPU throughput. It records the
actual WebGL renderer and a screenshot. For this machine's NVIDIA hardware:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/webcam_session/toolbox/depth/pixi.toml \
  bash -c 'PYTHONPATH="$PWD/build/webcam_session/rendering_python:$PYTHONPATH" python \
    roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/measure_viser_browser.py \
    --chrome-vulkan --seconds 15'
```

The explicit system Chrome Vulkan option selected the RTX PRO 2000 GPU here.
Default headed Chrome selected SwiftShader, so hardware use must be checked
from the recorded renderer rather than inferred from a visible browser window.

On the live DA3 comparison run, a 15.001-second sample decoded the RGB GUI image
at 19.66 updates/s and depth at 20.00 updates/s. Browser animation callbacks ran
at 60.06/s. WebGL reported the NVIDIA RTX PRO 2000 through Vulkan ANGLE.
Both images and the colored cloud were visually checked in the screenshot.
Raw browser samples are in [evidence/viser_browser.json](evidence/viser_browser.json).
The live screenshot is `build/webcam_session/depth_comparison/viser_browser.png`.
These numbers describe this local browser session with 336x252 image panels;
they do not establish 60 distinct cloud updates/s or camera-to-screen latency.
