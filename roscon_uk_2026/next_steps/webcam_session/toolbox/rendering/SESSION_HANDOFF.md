# Reuse the interactive webcam experiments

Use [depth_notebook.py](depth_notebook.py) to run calibrated relative depth, face and body landmarks, hand-controlled navigation, foreground extraction, and native virtual rain. The accepted implementations are saved as notebook cells. The session was stopped on 2026-10-21. The camera, face worker, viewer, animation workers, notebook server, and dedicated viewer browser were closed.

## Start on the prepared machine

These commands require this repository checkout. Run them from its root. Keep one owner of camera 0.

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh edit \
  roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/depth_notebook.py \
  --host 127.0.0.1 --port 2721 --no-token --no-sandbox
```

Open <http://127.0.0.1:2721> and run the cells once. Keep that tab open. Open <http://127.0.0.1:8768> beside the terminal. Expect RGB, estimated depth, a coloured foreground cloud, landmarks, and blue rain above the detected head. The rain cell displays its collision count and C++ step time. If the raised-hand filter is enabled, raise a wrist above its shoulder to reveal the person. Keep both services on localhost because notebook pair commands execute Python.

The notebook defaults enable several features. Runtime GUI settings are not saved as new defaults. The [session record](evidence/interactive_session_2026_10_21.json) preserves the final settings, versions, asset hashes, checks, and shutdown results. At shutdown, body pose and the raised-hand filter were disabled; foreground extraction, gestures, and rain were enabled. Gaze corrections live in processing state and need fresh calibration after a restart.

For a camera-free rehearsal, set `PERCEPTION_REPLAY_IMAGE` to a saved image before the launch command. This is a repeated image source, not live video. See [the rendering workflow](README.md).

## Code to reuse

The named cells below are in [depth_notebook.py](depth_notebook.py). Marimo schedules them by dependencies, rather than their textual order.

| Component | Source | What to retain or extract |
| --- | --- | --- |
| Capture and inference | [session.py](session.py), [camera.py](camera.py) | One camera owner, separate capture and processing workers, latest-frame storage, source sequence and timestamp, replaceable processor, persistent state. |
| Calibrated depth | [depth_pipeline.py](depth_pipeline.py), [depth models](../depth/models.py) | Cached undistortion, retained DA3-Small model, aligned RGB/depth, calibrated unprojection, point stride. |
| Viewer | [viser_view.py](viser_view.py) | Retained Viser server, image panels, coloured cloud, controls, camera access, update counters. |
| Face model | `face_model` | Retained MediaPipe subprocess, binary image protocol, dense face and iris landmarks, canonical face model. |
| Face pose and gaze | `processing`, `gaze_controls` | Calibrated PnP, assumed eyeball geometry, eye rays, head direction, optional neutral gaze correction. |
| Hand and body models | `hand_model`, `pose_model`, [people adapters](../people/adapters.py) | OpenCV Zoo palm, hand-pose, person, and body-pose models. |
| Landmark rendering | `face_overlay_view`, `hand_overlay_view`, `pose_overlay_view` | Persistent scene handles, 2D overlays, depth-anchored 3D hand/body skeletons. |
| One-fist pan | `hand_drag_logic` | Finger curl classifier, hand identity matching, press/hold/release, camera-relative pan. |
| Two-fist zoom | `hand_drag_logic`: `TwoHandDrag` | Hand-separation ratio, zoom priority, fixed look-at target, bounded dolly distance. |
| Detection-loss tolerance | `hand_drag_logic`: `TolerantHandDrag` | Retain a grab for two missed processed frames, release on the third, pause motion during gaps, reset the motion baseline on recovery. |
| Gesture worker | `hand_drag_runtime` | Consume each new result sequence once, operate independently of inference, release on stale input or processing errors. |
| Foreground surface | `foreground_surface` | Face-seeded depth selection, connected components, nearby hand components, adaptive depth cutoff, masked RGB/depth/cloud. |
| Raised-hand display filter | `raised_hand_filter` | Shoulder/wrist confidence and height checks, face/person association, pose segmentation masks, separate visual and gesture hand records. |
| Native rain | `rain_native_kernel`, `rain_simulation` | `@rain_cpp(nogil=True)`, retained particle buffers, gravity, depth collisions, normal-based bounce, splashes, independent simulation and publication rates. |

Existing native depth unprojection is in [geometry.py](../../geometry.py). Its `@cpp(nogil=True)` loop filters masked depths, computes XYZ, and returns source pixel indices. The foreground code reuses it. The rain kernel is new C++ written directly in the notebook. Detection remains in model libraries; OpenCV and NumPy handle their existing native operations.

## Requirements and asset preparation

Use the locked environments. Do not merge the model environments merely to make their imports available to one interpreter.

| Requirement | Prepared source or version |
| --- | --- |
| Depth notebook environment | [depth/pixi.toml](../depth/pixi.toml), [lockfile](../depth/pixi.lock), [activation](../depth/activate.sh). Observed Python 3.11.17, NumPy 1.26.4, OpenCV 4.11.0, cppyy 3.5.0, marimo 0.25.2. |
| GPU depth inference | Observed PyTorch 2.9.1+cu128, CUDA 12.8, NVIDIA RTX PRO 2000 Blackwell Generation Laptop GPU. A working CUDA backend is required by the current depth adapter. |
| Native compilation | The depth environment pins `gxx_linux-64=14.*`. Use its launcher so compiler activation and repository imports are available to cppyy_kit. |
| Face-worker environment | [people/pixi.toml](../people/pixi.toml), [lockfile](../people/pixi.lock). Observed Python 3.12.15, MediaPipe 0.10.21, protobuf 4.25.9. |
| Rendering packages | [requirements.txt](requirements.txt), installed into ignored `build/webcam_session/rendering_python` by [setup.sh](setup.sh). Viser 1.1.1 was used. The notebook also imports the optional Polyscope adapter. |
| Webcam | Linux V4L2 camera 0, 640 × 360 MJPEG, requested 30 fps. [camera.py](camera.py) disables variable exposure frame rate while retaining automatic exposure. |
| Calibration | `build/webcam_session/calibration_refined.json`; [numeric copy](evidence/webcam_calibration_refined.json), [calibration procedure](CALIBRATION.md). |

The [toolbox guide](../README.md), [people setup](../people/setup.sh), and [depth setup](../depth/setup.sh) describe asset preparation. The depth setup script runs probes against `build/webcam_session/rotation_0.jpg`. On a fresh machine, provide a saved test image there, or run [probe.py](../depth/probe.py) with `--image PATH` after preparing the environment and pinned upstream checkout. The probe does not open a camera.

Rendering installation, from the repository root:

```bash
bash roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/setup.sh
```

The rendering launcher prepends its package overlay, then invokes the depth environment's marimo. The face worker deliberately clears inherited `PYTHONPATH` and runs the people environment's Python. This prevents the rendering or depth environment from shadowing its MediaPipe installation.

### Model assets

| Asset | Pin and local destination |
| --- | --- |
| DA3-Small | [Checkpoint revision](https://huggingface.co/depth-anything/DA3-SMALL/tree/e08cab65ca0ec38e7826075418411ab90cab4da3). The adapter uses upstream source revision `3d835ec1a5802d64a8b8b15f817a1ab54809bfe4`. Source and Hugging Face cache are under `build/perception_toolbox`. |
| Hand and body models | [OpenCV Zoo revision](https://github.com/opencv/opencv_zoo/tree/48b10fa505aeb51fbd127fc60a5c660040e823b7/models). [fetch.py](../people/fetch.py) verifies the selected ONNX files using Git LFS SHA256 values. Local root: `build/perception_toolbox/people/opencv_zoo/models`. |
| Dense face landmarks | [MediaPipe face_landmarker.task](https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task), saved as `build/webcam_session/face_landmarker.task`. |
| Canonical face geometry | [MediaPipe v0.10.21 OBJ](https://raw.githubusercontent.com/google-ai-edge/mediapipe/v0.10.21/mediapipe/modules/face_geometry/data/canonical_face_model.obj), saved as `build/webcam_session/canonical_face_model.obj`. |

The dense face task and canonical OBJ were added during this session. The rendering setup now verifies/downloads these two files through `prepare_assets.py`. The equivalent manual commands, from the repository root, are:

```bash
mkdir -p build/webcam_session
curl -fL https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task \
  -o build/webcam_session/face_landmarker.task
curl -fL https://raw.githubusercontent.com/google-ai-edge/mediapipe/v0.10.21/mediapipe/modules/face_geometry/data/canonical_face_model.obj \
  -o build/webcam_session/canonical_face_model.obj
sha256sum --check <<'HASHES'
64184e229b263107bc2b804c6625db1341ff2bb731874b0bcc2fe6544e0bc9ff  build/webcam_session/face_landmarker.task
8bac80443397e113f41a8b565ea72c59390bc031d9defab289dba7bc0c54e618  build/webcam_session/canonical_face_model.obj
HASHES
```

For the same physical camera and capture mode, the saved calibration can be restored from the numeric copy. Recalibrate for a different camera, mode, or changed optical setup. Calibration corrects undistortion and projection; it does not make DA3-Small depth metric.

## Data contracts

The processor receives `(frame, state, controls)`. `frame` is a private BGR uint8 copy. `state` survives processor replacements and is written by the processing worker. Controls are copied for each invocation. Models are retained outside this function.

| Data | Convention |
| --- | --- |
| Raw image | 640 × 360 BGR uint8, before undistortion. |
| Detection image | Full-resolution undistorted BGR. Run all detectors before masking or drawing overlays. |
| Depth and base cloud colour grid | Typically 336 × 196 in this session. Read the actual array shape. Scale coordinates and intrinsics to that grid. |
| Processed image | 640 × 360 after overlays and filters. Its dimensions differ from the depth grid. |
| Geometry | Camera coordinates: x right, y down, z forward. Depth and cloud distances have relative scale. |
| Cloud colours | RGB uint8, one colour for each XYZ point. Derive them from the same source frame and selected pixel indices. |
| Result identity | `sequence`, `stamp`, and `revision` come from Workbench. Gesture and rain workers consume this published packet. |

Treat published packets and their arrays as immutable. Rain owns separate mutable simulation buffers and publishes copies to Viser. The native kernel runs with the GIL released; protect simulation-buffer snapshots with `rain_lock`.

### Feature behaviour to preserve

- **Gaze:** calibrated PnP plus assumed eyeball geometry produces an approximate direction. The neutral-offset button is not a validated screen-gaze calibration. [MediaPipe iris tracking does not itself determine where a person is looking](https://github.com/google-ai-edge/mediapipe/blob/master/docs/solutions/iris.md).
- **Pan:** a fist requires at least three curled fingers for acquisition. Two consecutive detections confirm it. A held hand uses a lower curl threshold to reduce flicker. Camera position and look-at move together.
- **Zoom:** two fists take priority over pan. Increasing their separation decreases camera-to-target distance. The look-at point stays fixed. Opening a tracked hand releases immediately.
- **Tracking gaps:** tolerate two consecutive missing processed results. Move nothing during the gap. Reacquisition resets position or separation before movement resumes. The third miss releases. A source older than 0.5 seconds or a processing error releases independently.
- **Foreground:** choose the nearest detected face by central depth. Use a fraction of the gap to the farther scene as the cutoff. Keep the face-connected component and eligible hand components. Retain a face seed for at most two missed frames. Nearby objects at similar depth can remain.
- **Raised-hand filter:** require a confident wrist above its corresponding shoulder by a small margin. Associate faces and hands with accepted poses. Apply their segmentation mask to RGB, depth, and cloud. Keep all accepted raw hand detections in `packet['hands']` for gestures; filtered drawing records are in `display_hands`.
- **Rain:** emit above the detected head. Integrate gravity and velocity, test projected depth crossings, estimate a local normal, bounce, and give splashes a short lifetime. At a top silhouette without valid neighbours, use an upward normal. This is an approximate visible-surface effect, not calibrated fluid simulation. Buffers support up to 4,096 drops. Runtime clamps count to 64–4,096, speed to 0.1–10, and gravity to 0–30.

The current foreground stage selects the nearest eligible face surface. Multi-person admission checks were tested synthetically, but showing every raised person's complete surface at different depths would require extending that stage. Do not describe it as a complete multi-person segmentation system.

## Edit the next experiment without restarting

Discover the session each time. Do not reuse an ID from this document or an earlier run. Commands require the repository checkout:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh pair notebook list \
  --url http://127.0.0.1:2721

roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh pair execute \
  --url http://127.0.0.1:2721 --session SESSION_ID --code-file - <<'PY'
import marimo._code_mode as cm
async with cm.get_context() as ctx:
    cell = ctx.cells['processing']
    print(cell.code, cell.status, cell.errors)
    # ctx.edit_cell(cell.id, replacement_source)
    # ctx.run_cell(cell.id)
print(workbench.status())
print(viewer.status())
PY
```

Retain setup, model, camera, and display cells. Load a new model in its own retained cell. Edit accepted processing code through code mode so it persists. Scratchpad assignments from `pair execute` do not replace saved cell definitions.

Changing a cell can rerun dependent cells. Before editing an animation or gesture runtime, stop its thread, join it, and remove its old GUI/scene handles. Otherwise the notebook can leave duplicate workers or controls. For native edits, use a fresh function name when required by Cling, and probe risky compilation outside the live interpreter first.

After each change, check cell errors, source timestamps, processor revision, Workbench errors, viewer errors, gesture errors, and rain errors. A successful pair command alone does not prove its cells succeeded. Save matching raw input, processed output, depth, masks, intrinsics, sequence, and timestamp when recording an example.

## Shutdown

Run this through the existing notebook session before stopping its server:

```python
rain_stop.set()
rain_thread.join(timeout=3)
hand_drag_stop.set()
hand_drag_thread.join(timeout=3)
viewer.close()
workbench.close()
assert not workbench.status()['capture_running']
assert not workbench.status()['inference_running']
face_worker.stdin.close()
try:
    face_worker.wait(timeout=3)
except subprocess.TimeoutExpired:
    face_worker.terminate()
    face_worker.wait(timeout=3)
face_worker_log.close()
```

Then stop the notebook server with Ctrl-C and close the session browser. Confirm that ports 2721 and 8768 no longer listen. Port 9223 belonged to the dedicated browser used for this session's visual checks. These ports and all session workers were confirmed stopped at handoff.

## Checks, performance, and startup lessons

The durable [session record](evidence/interactive_session_2026_10_21.json) contains compact verification results. Full local evidence, previews, and selected NPZ examples are under [build/webcam_session/rendering_evidence](../../../../../build/webcam_session/rendering_evidence). That directory is ignored build data and will not accompany a clean checkout.

| Check | Recorded result |
| --- | --- |
| Faces | One face, 478 landmarks, two approximate gaze rays. |
| Hands | Two hands, 21 landmarks each, 2D and 3D overlays. |
| Body pose | One pose with 13 visible joints in a seated upper-body view. |
| Gesture checks | Pan, zoom direction, fixed zoom target, release, hand identity, two-frame gap tolerance, and recovery without a jump. |
| Surface checks | Connected torso retained, far plane rejected, disconnected object rejected, detected hand included, invalid depth rejected, face-loss grace, adaptive shallow-gap cutoff. |
| Raised-hand checks | Raised/lowered wrists, low-confidence and out-of-frame rejection, association between two synthetic people; live admission and rejection observed. |
| Rain kernel | Native compilation, finite particle state, synthetic surface collisions, no collisions through an empty mask, buffer-size rejection. |
| Live rain sample | About 58 simulation steps/s and 29 publications/s; 3,820 collisions over about 6 seconds; median kernel call about 0.20 ms. |
| Shutdown | Capture, inference, rain, gesture, viewer, and face worker all stopped; ports 2721, 8768, and 9223 closed. |

Capture stayed near 30 fps. Combined processing samples ranged from about 9–15 fps with different feature settings. The original depth-only run approached 30 fps. These are software counters from different configurations, not matched benchmarks or display-latency measurements. Rain timing includes the Python kernel call overhead but excludes browser rendering. See the linked record before making performance comparisons.

Startup lessons:

1. The depth interpreter had protobuf 7.36.2. Direct use of MediaPipe 0.10.21 failed there. Keep the face task in the prepared people environment, with its own dependencies. An unused direct-install trial was removed from the rendering overlay after shutdown.
2. The embedded Python worker must pass through `textwrap.dedent` before `python -c`. Marimo's saved function indentation otherwise makes that payload invalid. This was fixed after shutdown. The face cell also creates its log directory before opening the log.
3. Do not rerun resource-owning cells to change a threshold. Use `workbench.configure(...)` or a retained GUI callback.
4. The camera's dynamic exposure frame-rate control can reduce capture to about 20 fps. The shared camera helper disables it at startup; the device setting persists after closing the camera.
5. The three gesture classes were grouped in `hand_drag_logic` after shutdown. Their class bodies are unchanged. This resolves marimo's reusable-definition ordering checks for inheritance across cells.
6. The rewritten notebook and extracted worker payload were checked without restarting camera capture. A full fresh startup after the final shutdown fixes has not been rehearsed.

## PCL work discussed but not implemented

PCL 1.15.1 is available in the repository's separate `pcl` environment. [pcl_kit](../../../../../pcl_kit/WHY.md) provides the NumPy/PCL bridge through cppyy. That bridge copies into PCL-owned storage. PCL was not loaded into this live session.

Possible next features are [surface normals](https://pointclouds.org/documentation/tutorials/normal_estimation.html) for rain, [surface triangulation](https://pointclouds.org/documentation/tutorials/greedy_projection.html), [smooth-surface region growing](https://pointclouds.org/documentation/tutorials/region_growing_segmentation.html), and [spatial clustering](https://pointclouds.org/documentation/tutorials/cluster_extraction.html). Start with a saved cloud in the prepared PCL environment. Check data ownership, native-library compatibility, and execution cost before connecting it to the retained live session.
