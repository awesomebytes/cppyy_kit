# Depth and text-prompted object masks

Run from this repository checkout. This optional Pixi workspace keeps PyTorch
and its CUDA libraries separate from the existing webcam session.

```bash
bash roscon_uk_2026/next_steps/webcam_session/toolbox/depth/setup.sh
```

The setup downloads DA3-SMALL, YOLOE-11s-seg and its MobileCLIP text encoder,
then runs each on `build/webcam_session/rotation_0.jpg`. Choose another image:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/webcam_session/toolbox/depth/pixi.toml \
  probe objects --image /absolute/path/frame.jpg
```

Depth produces a colored preview and aligned arrays. Object detection produces
an overlay, labels, confidence scores and image-sized masks. Outputs and timings
are in `build/perception_toolbox/evidence`. Model weights and the official DA3
source are under `build/perception_toolbox`; Pixi retains dependency versions in
`pixi.lock`. The DA3 source revision is pinned by `setup.sh`.

## Keep models loaded in the notebook

Use the toolbox depth launcher. In a retained notebook cell:

```python
from models import DepthModel, ObjectModel

depth_model = DepthModel(resolution=336)
object_model = ObjectModel(["person", "cup", "bottle"])
```

For each selected RGB uint8 frame:

```python
depth_result = depth_model(rgb)
objects = object_model(rgb)
```

Change a prompt without reloading the detector:

```python
object_model.set_classes(["hand", "bottle"])
```

The text encoder stays loaded, so class changes do not reread its checkpoint.
Use `object_model(rgb, track=True)` for retained ByteTrack state and `track_ids`.
Two successive calls on the saved frame returned the same person track ID.
Use a fresh ObjectModel when starting a new independent sequence.
Run inference in the session's latest-frame processing worker. Keep each result
paired with the source frame and timestamp. At 336-pixel processing resolution,
DA3 returns a smaller depth grid and the matching `rgb` and predicted `intrinsics`.
Resize an object mask to that grid with nearest-neighbor interpolation before
unprojecting selected pixels. DA3-SMALL depth has **relative estimated scale**;
its point coordinates are not measured metres.

Raw timing samples and memory measurements: [RESULTS.json](RESULTS.json).
Checkpoint URLs, SHA256 digests, source and model revisions: [assets.json](assets.json).
DA3 model loading uses the recorded Hugging Face revision.

## Measurement

Measured on 20 October 2026 using the saved 640x480 webcam frame, NVIDIA RTX PRO
2000 Blackwell laptop GPU, PyTorch 2.9.1+cu128. Each model ran separately. Probe
includes preprocessing, model inference, CPU output conversion and GPU
synchronization. The object probe also includes drawing its overlay. It excludes
capture, browser transport and display. Each result JSON retains all 20 calls
following the first call; p95 on this short sample is descriptive, not a sustained
latency guarantee. GPU memory fields are PyTorch allocator peaks, not total
process or GPU memory.

DA3-SMALL at 336x252 produced finite depth for every pixel. The first inference
took 629 ms; the next 20 calls had median 25.27 ms and p95 36.81 ms. Peak allocated
GPU memory was 245 MiB, reserved 292 MiB. The second call still took 144 ms.
YOLOE detected one person in this frame with confidence 0.909. With its text
encoder retained, first inference took 310 ms; the next 20 calls had median
7.68 ms and p95 12.47 ms. Peak allocated GPU memory was 1347 MiB, reserved
1448 MiB. Cached model initialization took 1.07 seconds. This includes the text
encoder, which remains available for changing prompts.

Model initialization timings can include downloads on the first setup run.
Both models use their existing Python inference APIs. The companion native
geometry operation is the cppyy_kit contribution; model inference itself is
already implemented with native CPU/CUDA operations by PyTorch.

Sources: [DA3](https://github.com/ByteDance-Seed/Depth-Anything-3),
[YOLOE](https://github.com/THU-MIG/yoloe),
[Ultralytics YOLOE](https://docs.ultralytics.com/models/yoloe/).

## Combined toolbox environment

This environment also includes ONNX Runtime 1.23.2 and pupil-apriltags. The shared
launcher adds the people and segmentation adapter paths, so one interpreter can
use OpenCV Zoo people models, EdgeSAM, AprilTag, DA3, YOLOE and native cppyy_kit
geometry. Actual combined-environment checks detected one face, produced an
EdgeSAM mask with score 0.897 and initialized the native AprilTag detector.
PyTorch, OpenCV, ONNX Runtime and a compiled cppyy function also ran in one
process. The installed environment occupies about 8.9 GB, mostly CUDA libraries.
Model files occupy approximately 731 MB including the text encoder.
