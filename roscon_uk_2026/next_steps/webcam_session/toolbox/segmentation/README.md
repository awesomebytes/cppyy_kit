# Point-prompt segmentation and AprilTag detection

Run these commands from this repository checkout:

```bash
bash roscon_uk_2026/next_steps/webcam_session/toolbox/segmentation/setup.sh
pixi run --manifest-path roscon_uk_2026/next_steps/webcam_session/toolbox/segmentation/pixi.toml \
  python roscon_uk_2026/next_steps/webcam_session/toolbox/segmentation/run.py \
  roscon_uk_2026/next_steps/agent_session/evidence/iteration-1.png --point 320 240
```

The example saves a mask, overlay, printable AprilTag and result JSON under
`build/perception_toolbox/segmentation`. The locked environment is separate from
the running webcam notebook. Downloads total approximately 37 MB.

Use `EdgeSAM` from `segment.py` in a retained worker. Construct it once, call
`set_image(bgr)` to encode a uint8 BGR frame, then call
`mask, score = model.predict([(x, y)])`. Additional point labels are 1 for
foreground and 0 for background. Reuse the embedding for multiple prompts on the
same frame. Encode each new frame before querying its mask. Worker threads should
use separate instances because the selected image and embedding are mutable.

`apriltag_detector()` returns a retained native AprilTag 3 detector. Call
`detector.detect(gray)` on contiguous uint8 grayscale. With calibrated intrinsics
and a measured tag size, its `estimate_tag_pose`, `camera_params` and `tag_size`
arguments enable metric pose estimation.

Keep model inference in a worker receiving the latest frame. Display the latest
completed result without queuing old frames. The measured CPU encoder latency
makes this a lower-rate segmentation worker, rather than a 30 Hz frame callback.

## Measured execution

[RESULTS.json](RESULTS.json) records one execution on the existing 640x480
simulator image, using CPU ONNX Runtime with four intra-operation threads:
209.6 ms first encoder call and 31.8 ms first decoder call. The selected mask had
6,738 pixels. These are first-call observations, not throughput benchmarks or
segmentation accuracy measurements. The AprilTag detector recovered ID 0 from the
official tag image in 4.15 ms. No webcam device or running notebook was modified.

## Native code and sources

ONNX Runtime performs model inference in native code. The `pupil-apriltags`
wrapper invokes native AprilTag 3. Their existing Python APIs are sufficient for
these adapters. cppyy_kit can operate on the resulting masks and depth through
native geometry kernels; using these models alone does not demonstrate a cppyy
speed advantage.

- [Official EdgeSAM models and implementation](https://github.com/chongzhou96/EdgeSAM).
  Preprocessing follows the RGB normalization, resize, padding and mask scaling
  in its ONNX predictor. This adapter uses OpenCV resize, while upstream image
  resize uses Pillow, so interpolation can produce small output differences.
- [Official AprilTag 3](https://github.com/AprilRobotics/apriltag).
- [AprilTag images](https://github.com/AprilRobotics/apriltag-imgs).
- [Python native wrapper](https://github.com/pupil-labs/apriltags).

Downloaded model and fixture hashes are pinned in `models.sha256`.

A subsequent ten-call warm sample gave encoder median 154.5 ms / p95 168.5 ms
and decoder median 19.0 ms / p95 22.9 ms. These include preprocessing and mask
postprocessing respectively, on the same image. Ten calls only provide a small
latency sample, not a reliable tail-latency benchmark. Raw samples are retained.

The environment also includes marimo 0.25.2, cppyy and compiler tools for the
shared interactive notebook. GCC 14 and its activation package match the root
workspace compiler setup and supply the sysroot needed by Cling. The repository
`geometry.masked_depth_points` native operation was
compiled and checked on a 2x2 depth/mask example in this environment.
