# Perception tools for interactive sessions

Use image detections to select corresponding depth pixels and 3D points. Keep
models loaded while changing the selection and geometry operations. Start with
one task, such as highlighting a hand and its corresponding point-cloud region.

## Existing building blocks

| Operation | Existing source |
| --- | --- |
| Live camera, retained processing function, browser output | [Session guide](README.md) |
| Native OpenCV and NumPy-backed Mat views | [cv_kit](../../../../cv_kit/README.md) |
| PCL NumPy conversion and voxel downsampling | `pcl_kit/pcl_kit/__init__.py` |
| Table removal, clusters and image color association | `roscon_uk_2026/next_steps/pick_place/perception.py` |
| Recorded RGB/cloud data and timestamped transforms | [MCAP session](../agent_session/NOTEBOOK.md) |
| MediaPipe holistic pose and hand inference | `retarget_pipeline/perceive.py`, `HolisticDetector` |
| Verified model downloads | `retarget_pipeline/fetch_models.py` |

On 20 October, the holistic model was cached under `build/pipeline/models`.
MediaPipe belongs to the separate `pipeline` environment. The live session's
`pick-place` environment did not contain MediaPipe, PyTorch, Transformers,
ONNX Runtime or Depth Anything 3. Preserve the running environment while
preparing compatible dependencies separately.

## Selected additions

- MediaPipe face, hand and pose tasks. Preserve image landmarks and confidence.
  Use segmentation when selecting an object's pixels: a landmark or bounding
  box alone does not isolate its surface.
- Depth Anything 3 for monocular depth. Evaluate a small model for responsiveness
  and the metric model when estimating distances. Measure the chosen model on
  this machine before promising a frame rate.
- Shared depth unprojection and image-mask-to-cloud selection. Preserve pixel
  indices so selections can be shown consistently in the image and 3D viewer.
- Native voxel filtering, outlier rejection, clustering and region statistics.
  Reuse the existing PCL/OpenCV integration and add small kernels where useful.

Sources: [MediaPipe vision tasks](https://ai.google.dev/edge/api/mediapipe/python/mp/tasks/vision),
[Depth Anything 3](https://github.com/ByteDance-Seed/Depth-Anything-3),
[DA3 API](https://github.com/ByteDance-Seed/Depth-Anything-3/blob/main/docs/API.md).

## Shared data contract

Retain the RGB image, source timestamp, depth map, camera intrinsics and detection
mask together. Record whether depth is measured, metric-model estimated, or
relative. Use the intrinsics corresponding to the resized/cropped depth image.
For rectified pinhole images with depth Z, unproject using
`X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`. Reject invalid depths. Transform to the desired
frame using the matching camera pose. Relative predictions alone do not establish
metric scale. MediaPipe world landmarks use their own reference convention and
must not be treated as measured camera-frame points.

For the simulator, use the recorded sensor depth/cloud and calibration directly.
For the webcam, display estimated depth as estimated geometry. Sample robust depth
inside an object mask rather than treating a box center as its measured location.

## Incremental exercises

1. Detect a hand or colored object and inspect its image overlay.
2. Show aligned depth and select the same region in the cloud.
3. Filter the selected region and report its centroid and extent.
4. Keep the result stable over time and collect examples where it fails.

Keep display responsive with a bounded latest-frame inference path if the model
is slower than capture. Attach the source timestamp to every result and display
its age. Associate a depth result with its source image, rather than silently
applying it to a newer frame. Keep Python orchestration and existing model APIs;
use cppyy_kit for native data handling and geometric operations where it helps.

Hardware query: NVIDIA RTX PRO 2000 Blackwell Generation Laptop GPU, 8151 MiB VRAM.
This establishes available hardware, not inference compatibility or measured speed.

## Research shortlist: additional libraries and models

Reviewed official project sources on 20 October 2026. The priority and hardware
assessments below are integration recommendations, not local benchmarks. Evaluate
small checkpoints individually at batch size one before combining models. Use
recorded frames for first measurements so the current live exercise stays intact.

| Candidate | Interface | Useful live exercise | Integration assessment |
| --- | --- | --- | --- |
| [OpenCV Zoo](https://github.com/opencv/opencv_zoo): YuNet, palm/hand/pose, PP-HumanSeg, NanoDet, VitTrack | C++ OpenCV DNN; Python examples | Faces, finger joints, skeletons, person cutouts, object tracking | First choice for extending cv_kit; several models are suitable for CPU use, leaving GPU capacity for depth. Check each model's backend support. |
| [YOLOE](https://github.com/THU-MIG/yoloe), small checkpoint | PyTorch; exported deployment paths | Change a text prompt from cups to bottles; display masks and select corresponding 3D points | Strong demo candidate. Prepare prompt encoders and weights before the session. Open-vocabulary results need inspection. |
| [EdgeSAM](https://github.com/chongzhou96/EdgeSAM) | PyTorch, supplied ONNX exports | Click an object, isolate its pixels, then display its depth/cloud region | Lightweight image segmentation candidate. Track between selections separately. |
| [SAM 2.1 Tiny](https://github.com/facebookresearch/sam2) | PyTorch | Select an object and propagate its mask through video | 38.9M parameters; plausible 8 GB candidate with bounded input/history. Official speed table uses an A100, not this laptop. Validate a live adapter rather than loading an unbounded video. |
| [ByteTrack](https://github.com/FoundationVision/ByteTrack) | Python and C++ deployment code | Keep temporary object IDs, count crossings, draw trajectories | Complements a detector. IDs are tracks, not persistent personal identities. |
| [LightGlue](https://github.com/cvg/LightGlue) with SuperPoint or ALIKED | PyTorch; linked deployment integrations | Show a reference picture and match it in the camera; estimate a planar homography | Useful for textured objects and visual localization. Bound keypoint count and image size. |
| [AprilTag 3](https://github.com/AprilRobotics/apriltag) | C library | Hold a printed tag; draw axes and a virtual robot/tool attached to it | Low integration cost and no model/GPU requirement. Metric pose requires calibrated intrinsics and known tag size. |
| [Open3D](https://github.com/isl-org/Open3D) | C++ and Python | Remove a table, cluster objects, estimate normals and align clouds | Broad 3D toolbox. Reuse PCL for operations already integrated; add Open3D for specific useful capabilities. |
| [small_gicp](https://github.com/koide3/small_gicp) | C++, Python | Align consecutive depth clouds and show estimated camera motion | Focused native integration example. Use static scene geometry; moving people can corrupt registration. |
| [dlib](https://github.com/davisking/dlib) | C++, Python | Facial landmarks, correlation tracking | Optional alternative to the existing people tools; reuse its bindings unless native composition adds value. |
| [NVIDIA Warp](https://github.com/NVIDIA/warp) | Python kernels compiled for CPU/CUDA | Depth unprojection, spatial queries, particles responding to a hand | Relevant alternative GPU prototyping approach. Compare its role honestly with cppyy_kit. |

### Rust and inference runtimes

- [ort](https://github.com/pykeio/ort) wraps ONNX Runtime for Rust. It is a route
  to running compatible exported detectors/segmenters in a compiled application.
  The underlying runtime also has C/C++ and Python APIs. CUDA provider builds
  must match their runtime requirements; see the
  [official CUDA provider guide](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html).
- [Candle](https://github.com/huggingface/candle) provides Rust ML inference,
  including YOLO detection/pose and Segment Anything examples. Evaluate supported
  operators/models and the CUDA build before choosing it for a specific export.
- [kornia-rs](https://github.com/kornia/kornia-rs) supplies image and 3D processing
  in Rust. It is a possible destination for a measured preprocessing kernel.
- [Parry](https://github.com/dimforge/parry) provides Rust collision and distance
  queries. A demo could show hand/object geometry entering a virtual robot region.
- [Rerun](https://github.com/rerun-io/rerun) has Python, C++ and Rust SDKs for
  synchronized images, clouds, transforms and time series. The repository already
  uses it, so reuse that viewer when moving beyond the simple image window.

Rust libraries need an existing Python binding, a C-compatible interface or a
separate application to integrate with this Python workflow. cppyy does not
reflect arbitrary Rust APIs. C++ libraries are the more direct cppyy_kit examples.

### Three preferred demonstrations

1. **Point at something:** hand landmarks identify a pointing direction; select
   an image region, segment it, and show the matching depth/cloud region. Start
   with user confirmation of the selected object rather than assuming the gesture
   uniquely identifies it.
2. **Find a named object in 3D:** YOLOE mask plus DA3 or simulator depth, followed
   by PCL filtering and a centroid/bounding-box overlay. Keep source timestamps,
   intrinsics and estimated-versus-measured depth visible in the data contract.
3. **Attach a virtual robot tool to a real object:** AprilTag pose plus a rendered
   coordinate frame/tool and distance queries. This is a small, reliable robotics
   example with a visible ground reference.

Use OpenCV Zoo plus AprilTag as the low-setup tier; add YOLOE/EdgeSAM for semantic
selection; add DA3 and 3D processing after measuring inference on this GPU.
Measure first-use time, warm median/p95 latency, peak GPU memory and result age.
Model parameter count alone does not establish runtime memory or frame rate.
