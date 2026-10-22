# Face, hand, pose and person segmentation

From this repository checkout:

```bash
bash roscon_uk_2026/next_steps/webcam_session/toolbox/people/setup.sh
pixi run --manifest-path roscon_uk_2026/next_steps/webcam_session/toolbox/people/pixi.toml smoke
```

The command runs four retained OpenCV DNN models on the saved webcam frame
`build/webcam_session/rotation_0.jpg`. Pass `--image PATH` to use another frame.
The camera is never opened. Model files live under `build/perception_toolbox/people`.
`fetch.py` verifies each model against its upstream Git LFS SHA256.

In the people notebook environment:

```python
from adapters import People

faces = People('face')
# Keep this instance in a retained setup cell.
result = faces(camera.snapshot())
```

Operations are `face`, `hand`, `pose`, and `segmentation`. Face rows contain
box x/y/width/height, five image landmarks, and confidence. Other outputs use
the original OpenCV Zoo adapter formats; inspect the pinned upstream adapter
and its demo when drawing landmarks. Person segmentation returns a label image.
Input is uint8 BGR. Each instance retains its networks across frames.

The same environment contains MediaPipe 0.10.21. Existing holistic Tasks data
is under `build/pipeline/models`; `retarget_pipeline/fetch_models.py` downloads
verified pose and hand bundles. The OpenCV models offer people processing
without requiring GPU inference. OpenCV's existing Python binding already
executes native DNN inference; cppyy is useful for custom native operations
and direct native object composition, not for claiming that Python OpenCV was
previously Python-only.

Measured on saved webcam input, CPU with two OpenCV threads, ten warmed calls:
face 8.24 ms median, hand 22.51 ms, pose 33.15 ms, segmentation 12.58 ms.
Each detector found one target in this frame. These short smoke measurements
exclude capture and display. Raw local results: `build/perception_toolbox/people/smoke.json`.

Source and model licenses: [OpenCV Zoo](https://github.com/opencv/opencv_zoo/tree/48b10fa505aeb51fbd127fc60a5c660040e823b7/models).
Each downloaded model directory includes its license.
