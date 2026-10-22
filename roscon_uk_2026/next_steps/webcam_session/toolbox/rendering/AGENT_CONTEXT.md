# Interactive webcam session

Use Python 3.12. Connect to marimo at http://127.0.0.1:2721 and Viser at
http://127.0.0.1:8768. Discover the kernel through `notebook.sh pair notebook list
--url http://127.0.0.1:2721` in this directory. Verify its Python version, current
cells, errors and camera/viewer status. If absent, follow the Presentation section
in README.md to start live_notebook.py. Keep its browser tab open.

Ask what to implement. Possible tasks: face boxes or landmarks; hand detection,
gesture counting or hand-up selection; body skeletons; depth images and colored
point clouds; object detection/tracking or click-prompt segmentation. Choose one
bounded task and refine it from the live output.

## Starting choices

- Face boxes: retained dlib frontal detector, preferred by the presenter in the
  earlier exercise. It uses CPU HOG. Add the 68-point predictor only for landmarks.
- Dense face/iris, hands and body skeletons: prepared MediaPipe Tasks. See
  ../people/README.md and the model-loading cells in depth_notebook.py. Inspect
  compatibility before importing; the people environment is also Python 3.12.
  OpenCV Zoo hand/pose adapters remain available in ../people/adapters.py.
- Depth: retained DA3-Small on CUDA for a first live result. DA3METRIC-LARGE is
  cached for metric-depth work; check its focal scaling and processing budget.
  See ../depth/models.py and CALIBRATION.md. Reuse the calibrated DepthPipeline.
- Objects: YOLOE with retained text classes, and ByteTrack when tracking helps.
- Segmentation: EdgeSAM with a click prompt. AprilTag detection is also prepared.

These are starting choices, not restrictions or universal rankings. Inspect local
assets and loaded instances before downloading, building or changing backend.
Use GPU execution where the backend supports it and measurements justify it.
Check actual CUDA/provider support. Prefer direct compatible imports; use a worker
only when dependency isolation is necessary. Keep one camera owner.

## Editing and verification

Keep camera, viewer and loaded models alive. Use marimo pair/code mode for saved
cell edits; do not write the active notebook externally. Inspect dependencies
before changing cells. Stop/join workers before replacing their owner cells.
Add controls only when processing consumes them. Show cloud controls only when
points exist. Check output, errors, frame age and processing rate after edits.

Reuse camera 0 calibration at 640x360 from
build/webcam_session/calibration_refined.json, or the committed copy in
rendering/evidence/webcam_calibration_refined.json. Validate the physical camera
and capture mode. Account for resizing/cropping. Calibration does not convert
relative depth to metres.

## cppyy_kit context

In the notebook environment run `python -m cppyy_kit status --environment`,
`python -m cppyy_kit guide`, and `python -m cppyy_kit guide accelerate`.
For a C++ library read `guide bring-library`; for a kit read `guide KIT_NAME api`.
See docs/COMMON_PATTERNS.md for header/preamble, array, GIL and lifetime patterns.
Reuse existing native model bindings. Use cppyy_kit for a useful native operation,
compile/warm it once, retain required buffers/owners and measure the full pipeline.

Save accepted source, parameters, representative inputs and reviewed outputs.
A later implementation agent can use that reference to produce a normal C++ or
Rust extension with a Python API and compare correctness, startup, RAM and speed.
