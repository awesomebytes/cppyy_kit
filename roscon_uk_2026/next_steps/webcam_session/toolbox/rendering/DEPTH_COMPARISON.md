# Live webcam depth and cloud

Update: capture now uses 640 × 360 MJPEG at 30 fps. The Small-model pipeline with the native viewer measured 30.07 captured and 30.00 processed frames/s. See [the capture follow-up and calibration](CALIBRATION.md). The measurements below retain the earlier 640 × 480 comparison.

Run from this repository checkout, using the prepared environment:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/depth_demo.sh --viewer viser
```

Open http://127.0.0.1:8768. RGB, coloured depth, a coloured point cloud and controls share the viewer. Use `--viewer polyscope` for a native window with ImGui image panels. Run one camera session at a time. Ctrl-C closes the session. See [environment setup](README.md).

The pipeline captures 640 × 480 webcam frames, runs DA3-SMALL on the NVIDIA GPU, and produces aligned 336 × 252 RGB and depth. The cppyy_kit C++ helper masks and unprojects depth while releasing the GIL. Stride 2 produces 21,168 points. Capture, processing and display use separate workers with latest-frame slots.

## Comparison

| Measurement | Viser | Polyscope |
| --- | ---: | ---: |
| Measurement duration | 15 s | 10 s |
| Sampled results | 299 | 200 |
| Capture rate | 19.93 frames/s | 20.00 frames/s |
| Processing rate | 19.93 frames/s | 20.00 frames/s |
| DA3 per-frame median | 24.70 ms | 23.70 ms |
| DA3 per-frame p95 | 29.07 ms | 27.22 ms |
| Geometry and colouring per-frame median | 1.95 ms | 1.90 ms |

Median is the middle duration. p95 is the duration at or below which 95% of samples fall. These are individual frame durations, not accumulated time.

Both viewers kept up with the camera. Viser was simpler to assemble into one interface. There is no demonstrated throughput advantage for the native viewer at this workload.

Chrome decoded RGB/depth image updates at 19.66/20.00 Hz. Its animation callbacks ran at 60.06 Hz. Polyscope submitted new data at 19.90 Hz and its render loop ran at 59.69 Hz. These are different software counters; 60 Hz does not mean 60 new camera frames per second. Camera-to-screen latency was not measured.

Models were warmed. Runs used sequential live input, not identical replay frames. Both used NVIDIA RTX PRO 2000 Blackwell Laptop inference. Chrome rendered through NVIDIA Vulkan/ANGLE; Polyscope used Mesa Intel graphics. Small timing differences do not establish a faster renderer. Startup is excluded.

Evidence: [pipeline measurements](evidence/depth_comparison.json), [browser measurements](evidence/viser_browser.json). Local camera screenshots remain under the ignored build directory: [Viser](../../../../../build/webcam_session/depth_comparison/viser_browser.png), [Polyscope](../../../../../build/webcam_session/depth_comparison/native_final/polyscope.png).

## Resolution and calibration

336 is our configured processing resolution, not a DA3-SMALL model limit. `DepthPipeline(resolution=504)` uses the same model at higher resolution. Model capacity and processing resolution are separate choices. Higher resolution costs additional computation and memory; its throughput has not been measured here.

This model produces relative depth. Intrinsics are estimated by DA3. Calibration can improve the projection geometry and allow lens undistortion, but does not remove monocular scale ambiguity or guarantee metric depth. Calibration must match the camera capture mode and be adjusted for image resizing/cropping. Depth colours use per-frame percentiles and therefore do not encode a fixed distance scale.

The upstream API documents processing resolution and optional intrinsics: https://github.com/ByteDance-Seed/Depth-Anything-3/blob/main/docs/API.md

## Persistent notebook

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh edit roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/depth_notebook.py --port 2721
```

Keep model and camera cells alive while revising the processing callback. Close the current viewer before replacing it with `PolyscopeViewer(workbench)`. Call `viewer.close()` and `workbench.close()` when done.

Implementation: [pipeline](depth_pipeline.py), [notebook](depth_notebook.py), [standalone runner](depth_demo.py). Add `--seconds 15` to record a timed standalone run. Viser waits for a connected browser. The separate `measure_viser_browser.py` samples browser counters.
