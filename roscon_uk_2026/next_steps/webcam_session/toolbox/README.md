# Develop webcam perception with prepared models

Use a fresh persistent notebook with the optional model environment. Keep the
model instance and camera alive while editing the processing function. These
commands require this repository checkout.

The environments and assets are prepared on this machine. To reproduce setup:

```bash
bash roscon_uk_2026/next_steps/webcam_session/toolbox/setup.sh
```

This downloads the pinned assets and runs inference on saved inputs. Depth and
object probes use `build/webcam_session/rotation_0.jpg` by default; individual
probe commands accept another image.

For a native image window with independent capture/inference and optional Viser
3D controls, use the [rendering integration](rendering/README.md).

## Start a session

Use `depth` for the combined GPU/model toolbox. It also loads the people and
segmentation adapters. The smaller `people` and `segmentation` environments
are useful for CPU-only exercises. These environments keep model dependencies
out of the existing ROS/webcam environment. Stop the
old camera deliberately before starting another owner of camera 0.

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/notebook.sh depth edit \
  roscon_uk_2026/next_steps/webcam_session/toolbox/notebook.py \
  --host 127.0.0.1 --port 2720 --no-token --no-sandbox
```

Open <http://127.0.0.1:2720>, run its cells, and open <http://127.0.0.1:8767>.
The starter shows the live camera. The new notebook is separate from the user's
existing webcam notebook. The same launcher supports marimo agent commands:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/notebook.sh depth pair notebook list \
  --url http://127.0.0.1:2720
```

Give a new agent this instruction:

> Read toolbox/README.md and ../README.md in the webcam_session directory.
> Connect through marimo pair to port 2720. Inspect the existing cells and
> camera.status(). Keep the camera cell alive. Put model construction in a
> retained setup cell, then revise processing through code mode. Inspect the
> output after each revision and save accepted code in notebook cells.
> Use the selected environment's adapter; do not install into another running
> session. Keep source timestamps when associating images, masks and depth.

The agent mechanism is the same as the verified original session. Model smoke
checks below use saved inputs. A new live rehearsal determines camera-to-display
latency; single-frame inference timing does not establish it.

## Select a model

| Environment | Prepared operations | Instructions |
| --- | --- | --- |
| `people` | YuNet faces, OpenCV Zoo hand/pose landmarks, person segmentation; MediaPipe Tasks | [People](people/README.md) |
| `segmentation` | Click-prompt EdgeSAM, AprilTag 3 | [Segmentation](segmentation/README.md) |
| `depth` | DA3 small, text-prompt YOLOE small, ByteTrack, plus Zoo/EdgeSAM/AprilTag adapters | [Depth and objects](depth/README.md) |

Each directory contains `pixi.toml`, its lockfile, setup instructions and reusable
adapters. Models and third-party repositories are under ignored
`build/perception_toolbox`. Keep model instances outside frame callbacks.

For example, in the `people` environment load a model in a retained cell:

```python
from adapters import People
face_model = People('face')
```

Then install processing in another cell:

```python
def process(frame, state):
    output = frame.copy()
    for face in face_model(frame):
        x, y, width, height = face[:4].astype(int)
        cv2.rectangle(output, (x, y), (x + width, y + height), (0, 255, 0), 2)
    return output

camera.set_processor(process, label='YuNet faces')
```

Changing this cell preserves the camera and model. Model inference can take
longer than a camera interval. Measure result age and use bounded latest-frame
work rather than accumulating frames. The existing camera worker runs the
processing function sequentially, so a slow model lowers its output rate.

## Native operations and 3D

OpenCV, ONNX Runtime, AprilTag and PyTorch already run native operations through
existing Python bindings. Reuse those APIs. cppyy_kit adds value where a custom
native operation, existing C++ object or missing library binding is useful.

`../geometry.py` provides `masked_depth_points(depth, mask, K)`. It fuses mask
selection, finite/range rejection and pinhole unprojection into one cached C++
loop using `@cpp(nogil=True)`. It returns XYZ points plus their original pixel
indices. The NumPy inputs stay owned for the synchronous call. This is a small
native operation that can run without holding the Python GIL.

Resize an object mask with nearest-neighbor interpolation to the depth grid.
Use the intrinsics for that same grid. DA3-SMALL output is relative estimated
depth, not a measured metric distance. Simulator depth uses sensor calibration.
Keep each mask, depth map and image associated with the same source frame.

## Find an object and select its estimated 3D points

Run the combined example without opening a camera:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/webcam_session/toolbox/depth/pixi.toml \
  python roscon_uk_2026/next_steps/webcam_session/toolbox/fuse.py --object person
```

DA3 processes the image; YOLOE detects on that exact processed RGB grid; the
cppyy_kit operation selects and unprojects masked depths. The saved webcam image
produced one person mask and 32,079 points on a 336x252 grid. See
[FUSION_RESULT.json](FUSION_RESULT.json). The output NPZ under
`build/perception_toolbox/evidence/selected_cloud.npz` contains points, pixel
indices, RGB, depth, mask and intrinsics. These coordinates have relative
estimated scale. Both GPU models and the native geometry operation ran in one
process. For a live notebook, retain the two model instances in a setup cell
and call the same operations for each selected frame.

## Recorded checks

- People models ran on a saved 640x480 webcam image. YuNet, OpenCV hand and pose
  each returned one target. CPU warm medians were 8.24, 22.51 and 33.15 ms;
  person segmentation was 12.58 ms (ten calls, two OpenCV threads).
- MediaPipe Tasks hand and pose ran on the same image: two hands, one pose.
- `cv_kit` native `cv::Mat` view and `cv::mean` ran in the people environment.
- EdgeSAM and AprilTag measurements are in [segmentation results](segmentation/RESULTS.json).
- GPU depth/object measurements and exact settings are in [depth guide](depth/README.md).

The `people` environment pins GCC activation through `gxx_linux-64=14.*`.
Installing `compilers` alone did not set `CXX` in this fresh environment and
Cling startup crashed. The locked environment and Pixi launcher include the
working activation. Models were downloaded from official upstream sources;
model licenses are linked from their guides.
