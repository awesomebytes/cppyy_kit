# Tutorial: visual loop closure with Python and C++

This tutorial implements the **place-recognition / loop-closure front-end** of a
visual SLAM system as a Python ROS 2 node. The node calls the C++ OpenCV and DBoW2
libraries through [cppyy](https://cppyy.readthedocs.io), and displays results in
[Rerun](https://rerun.io). The image data stays in C++ from the ROS subscription
through the DBoW2 query.

**Pipeline:**

1. **ORB features** on every frame (Mur-Artal & Tardós, *ORB-SLAM: a Versatile and
   Accurate Monocular SLAM System*, IEEE T-RO 2015).
2. **DBoW2** place recognition: map binary descriptors to a vocabulary of
   "visual words", then compare images by their word histograms (Gálvez-López & Tardós, *Bags of Binary Words for Fast Place Recognition
   in Image Sequences*, IEEE T-RO 2012).
3. A **temporal-consistency gate** so a loop is only confirmed once a candidate
   persists over several frames (the DLoopDetector idea).
4. **(optional)** a **GTSAM pose graph** that uses the confirmed loop to correct a
   drifting trajectory.

This pipeline does not have a complete Python API across these libraries. OpenCV has
`cv2`, but the image is copied between Python and C++. DBoW2 has no Python binding or
conda package. GTSAM's C++ API depends on Boost. cppyy lets Python call OpenCV and
DBoW2 while image data stays in C++. GTSAM uses its Python binding in this tutorial.

You need only this repo and [pixi](https://pixi.sh). Every step is a `pixi run`.

---

## 0. Setup (once)

```bash
pixi install -e vision          # OpenCV 4 (C++ + cv2), rerun-sdk, gtsam, cppyy, ROS 2
pixi run -e vision build-dbow2  # clone + patch + compile DBoW2 -> build/vendor/libDBoW2.so
```

`build-dbow2` prints the two patches it applies and finishes with
`OK -> .../libDBoW2.so`. See [the DBoW2-from-source section](#dbow2-from-source) for
what those patches are and why.

> Run a demo with a display to open a live Rerun window. The window shows the camera
> stream, ORB keypoints, and a processing-time plot. Stages 3 and 4 also show loop
> scores, confirmed loop events, and the corrected trajectory. A
> [blueprint](https://rerun.io) arranges the panels. Without a display, or under
> `pytest`, the demo writes a `.rrd` recording to `build/vision/`. Open it with
> `rerun <file>`. Set `RCLCPPYY_RERUN_SPAWN=1` to force the live viewer or `=0` to
> force a recording. `cv_kit/demos/vision_viz.py` selects the mode.

> By default, the demos use a deterministic synthetic loop sequence and need no
> downloads. The [real-data path](#real-data-tum-rgb-d-and-orbvoc) downloads a TUM RGB-D sequence and the
> ORB vocabulary.

---

## Stage 1: Zero-copy image path

```bash
pixi run -e vision demo-vision-spine
```

Expected output (abridged):

```
  frame 25 ingest=0.029 ms
SUMMARY frames=200 ingest_avg_ms=... ingest_p50_ms=0.02 ingest_max_ms=...
Rerun: live viewer opened -- watch it stream. (headless instead: RCLCPPYY_RERUN_SPAWN=0)
```

(Run it on a machine with a display and a Rerun window opens showing the camera
stream; headless, that last line reads `Rerun recording saved: build/vision/spine.rrd`.)

One process runs two ROS 2 nodes: a publisher emits the sequence as
`sensor_msgs/Image`, and a subscriber, subscribing **via rclcpp_kit**, so its callback
receives the **C++** `sensor_msgs::msg::Image`, logs each frame to Rerun.

### The zero-copy bridge

The key line is `cv_kit.msg_to_mat(msg)`. It wraps the message's pixel buffer
as an OpenCV `cv::Mat` **without copying a single byte**:

```python
mat = cv_kit.msg_to_mat(msg)     # cv::Mat whose .data IS msg.data.data()
```

The `cv::Mat` *aliases* the message's `data` vector, the Mat's data pointer is
**byte-identical** to `msg.data.data()` (the test `test_msg_to_mat_zero_copy_pointer_identity`
asserts exactly this). Compare with the standard rclpy path, where the whole `Image`
message is deserialized into Python then `np.frombuffer(msg.data).reshape(...)`
creates an array copy whose size grows with the image.

> At 640×480, the copy is small and the measured difference is limited because cppyy
> call overhead is similar in size. The zero-copy wrap time stays about the same as
> image size increases, while the rclpy copy time grows with pixel count. At 1920×1080,
> the wrap is about **150×** faster (`pixi run -e vision bench-vision`). The image also
> stays in C++ through ORB and the DBoW2 query. The Mat refers to the message buffer,
> so use it only while the message is alive, such as within its callback.

---

## Stage 2: ORB features

```bash
pixi run -e vision demo-vision-features
```

```
ORB backend: cv::ORB (CPU)
  frame 25: 1000 keypoints, desc 1000x32, orb=4.00 ms
SUMMARY frames=200 orb_avg_ms=~3.7 orb_fps=~270 avg_keypoints=1000 backend=CPU
```

Each zero-copy `cv::Mat` is grayed and run through **C++ `cv::ORB`**; keypoints are
logged as a Rerun `Points2D` overlay on the image. The ORB descriptor is an **N×32
`CV_8U`** matrix, N 256-bit binary descriptors, which is exactly what DBoW2 consumes
next.

> `cv2.ORB` has similar per-frame performance. This example uses the C++ API so the
> image can stay in C++ from the ROS subscription through ORB and the DBoW2 query.

### CUDA

The demo prints `cv::ORB (CPU)` because the conda-forge OpenCV has **no CUDA build**.
`cv_kit` auto-detects this (`cuda_available()` probes for the `cudafeatures2d` module)
and cleanly uses the CPU path, no error. The CPU/GPU choice is a **single branch
point** in `cv_kit.create_orb`, so a CUDA-enabled OpenCV drops in with **no code
change**: `create_orb` then constructs `cv::cuda::ORB` instead.

A CUDA OpenCV build (matching this env's 4.13.0) is available and validated on this
machine's GPU, see **[cv_kit/CUDA_OPENCV.md](../../cv_kit/CUDA_OPENCV.md)** for the
`pixi run -e cudabuild provision-cuda-opencv` steps and the `vision-cuda` env. Measured
there: `cv::cuda::ORB` ~576 fps vs ~110 fps CPU, **~5.3× faster** through the same
cppyy path (thanks to soname shadowing, `cv_kit` needs no change to use it).

---

## Stage 3: Place recognition and loop closure

```bash
pixi run -e vision demo-vision-loop
```

```
Training vocabulary on the sequence (offline pass) ...
  ...trained vocab: 9970 words, k=10 L=4
Streaming synthetic frames for loop detection ...
  LOOP  frame 181 revisits frame 1  score=0.462
  LOOP  frame 182 revisits frame 2  score=0.465
  ...
  LOOP  frame 199 revisits frame 19  score=0.474
SUMMARY frames=200 confirmed_loops=19
  synthetic loop segment (ground truth): frames [180,200) revisit [0,20)
```

One node runs the front-end. By default, it trains a DBoW2 vocabulary on the sequence.
For real data, it loads the ORBvoc vocabulary. Both are used with an `OrbDatabase`. For each
frame we: wrap zero-copy → ORB → **add to the database** → **query** for the most
similar earlier image → run it through the **temporal-consistency gate**.

The synthetic sequence is a sliding window over a fixed textured canvas that travels a
closed circuit whose **last 20 frames retrace the first 20**, a loop closure by construction. The detector reports frame `180+j` as a revisit
of frame `j`.

The live viewer shows the camera stream and ORB keypoints on the left. On the right,
it shows ORB time, loop scores, the current and matched frames, and loop events. The
synthetic sequence produces no loop events during the first lap. It reports a loop
when the sequence retraces its path.

### The temporal-consistency gate

A high BoW score can be a false positive in a textured scene. Following
DLoopDetector, `loop_detector.LoopDetector` confirms a loop when the best candidate persists and
moves coherently over `k` consecutive frames. It ignores recent database entries,
because adjacent frames often match, and requires a minimum BoW score. The detector
does not report the first `k−1` frames of a revisit. This delay is required for
confirmation.

### Regression test

```bash
pixi run -e vision test-vision
```

`test_vision_loop.py` runs the pipeline on the deterministic synthetic sequence and
compares detected loop pairs with a recorded baseline. Precision is 1.0. Vocabulary
training uses `srand` so results are reproducible. The test needs no download.

<a name="dbow2-from-source"></a>
### DBoW2 from source

DBoW2 is **not on conda-forge and has no Python binding**, so `build-dbow2` vendors it:
it clones `dorian3d/DBoW2` into `build/vendor/` (gitignored) and direct-compiles it
with the env's C++ compiler, the same recipe as `scripts/freeze/build_l2_node.py`,
sidestepping DBoW2's CMake (which pulls a DLib dependency the ORB path never needs).
Two small, documented, idempotent patches (kept as a scripted in-place edit, never a
fork):

1. **Compile only the DLib-free ORB sources** (skip `FBrief`/`FSurf64`, which need
   DVision/opencv-contrib); include the specific headers rather than the umbrella
   `DBoW2.h` that would drag them in.
2. **Add an ORB-SLAM2-style `loadFromTextFile` plus a raw binary cache** to
   `TemplatedVocabulary.h`, so we can read the canonical `ORBvoc.txt` (which stock
   DBoW2 can't) and cache it as a fast-loading binary.

`dbow_kit` then mirrors DBoW2's own API (`train_vocabulary`, `make_database`,
`add_image`, `query`), keeping only the fiddly N×32-Mat → `vector<cv::Mat>` descriptor
split in C++.

---

## Stage 4 (optional): Pose-graph correction

```bash
pixi run -e vision demo-vision-posegraph
```

```
confirmed loops: 19 (e.g. [(181, 1), (182, 2), (183, 3)])
mean position error vs ground truth:
  open-loop odometry : 2.188 m
  after pose-graph   : 0.143 m
```

The demo also shows how a loop closure can correct a map. It
builds a 2D pose graph over the synthetic circuit: the odometry is the true circuit
corrupted by an accumulating heading drift (so the open-loop trajectory spirals away),
and each confirmed loop closure adds a `BetweenFactor` tying the revisiting pose back
to the earlier one. **GTSAM's Levenberg-Marquardt** optimizer then pulls the drifted
trajectory toward the ground truth, reducing mean position error by about **15×**.

On the `step` timeline, the
3D view plays back the drive: the **red** open-loop trajectory spiralling away from the
**green** ground truth while the *mean-error* plot on the right climbs; **yellow** loop
loop edges appear as each revisit is confirmed. On the final step, the optimizer runs.
The blue corrected trajectory moves toward the ground truth, and mean error drops by
about 15×. The recording is saved to `build/vision/posegraph.rrd` in headless mode.

> **Why use GTSAM's Python binding?** A cppyy retry after adding
> `libboost-headers` passed the `boost/optional.hpp` include. Two blockers remain: (1) the conda gtsam build's `config.h` sets
> `GTSAM_USE_TBB`, so its headers `#include <tbb/…>`, and the env ships only the tbb
> *runtime* (`libtbb.so`), not the tbb *headers*; and (2) even with tbb headers supplied
> out-of-band, Cling's JIT fails to materialize the static initializer of gtsam's
> namespace-scope `static const KeyFormatter DefaultKeyFormatter` in `Key.h` (an
> internal-linkage `std::function` global). The second is a Cling limitation, not a
> missing dependency. Pose-graph optimization is a batch step, so the demo uses the
> Python binding. The per-frame path (ORB and DBoW2) runs through cppyy. Probe details:
> [cv_kit/REPORT.md](../../cv_kit/REPORT.md) §GTSAM/cppyy.

---

## Stage 5: Benchmarks

```bash
pixi run -e vision bench-vision            # add --orbvoc for the real-vocab timing
```

| Metric | Result (synthetic, CPU; shared machine, directional) |
|---|---|
| Ingest 640×480 | rclcpp_kit ~0.008 ms vs rclpy-copy ~0.010 ms (~1.3×) |
| Ingest 1920×1080 | rclcpp_kit ~0.001 ms vs rclpy-copy ~0.167 ms (**~155×**) |
| ORB (CPU) | ~270 fps (~3.7 ms/frame, 1000 keypoints) |
| Small-vocab train | ~7 s (9970 words, k=10 L=4) |
| Query latency | ~2.8 ms/frame |
| Loop precision / recall | **1.00 / 0.95** (recall < 1 only from the k-frame confirmation delay) |
| Real ORBvoc load | text parse ~2.3 s → binary cache reload ~0.37 s (~6×) |

---

<a name="real-data"></a>
## Real data: TUM RGB-D and ORBvoc

The synthetic sequence runs without downloads. The real-data path uses a TUM SLAM dataset.

```bash
pixi run -e vision dataset-tum         # freiburg3_long_office_household (~1.48 GB) -> data/
pixi run -e vision dataset-orbvoc      # the real ORBvoc.txt (~145 MB) -> data/
pixi run -e vision demo-vision-loop -- \
    --tum data/rgbd_dataset_freiburg3_long_office_household \
    --vocab data/ORBvoc.txt --ignore-recent 300 --min-score 0.045 --consistency 4
```

`freiburg3_long_office_household` is the canonical loop-closure sequence: the handheld
camera circles an office and returns to the start. With the real ORBvoc (971,814
words, k=10 L=6) the front-end detects a revisit around frame 2207
returning to the ~frame-78 start region, with BoW scores ~0.05–0.10 (the normal range
for a large vocabulary on real imagery). The first ORBvoc load parses the 145 MB text
(~2–3 s) and writes a binary cache next to it. Later loads take ~0.3 s.

> Dataset: Sturm et al., *A Benchmark for the Evaluation of RGB-D SLAM Systems*, IROS
> 2012 (TUM CVG), CC BY 4.0. Vocabulary: Gálvez-López & Tardós (via ORB-SLAM2).
>
> Tune these parameters for real data. Use a larger `--ignore-recent` to avoid
> matching nearby frames and a lower `--min-score` because real-image BoW scores are
> small. More robust detection also needs score normalization and geometric
> verification. See [Limits and follow-up work](#limits-and-follow-up-work).

---

## Startup options (L0 to L2)

The examples use **L0**: cppyy JIT-compiles the libraries' headers at bringup
(a one-time ~0.2 s for OpenCV; DBoW2's headers are tiny). To reduce startup latency, use the cppyy_kit freeze tools:

- **L1 (freeze):** create a precompiled header from the kit's headers into a Cling precompiled header so bringup
  skips the header parse. See **[docs/FREEZE.md](../FREEZE.md)**.
- **L2 (lowering):** if a per-frame Python hop ever dominates, emit that step as native
  C++ (the pattern in `scripts/freeze/build_l2_node.py`, which `build_dbow2.py` already
  mirrors). Here the hot path (ORB, the DBoW2 query) is *already* all C++, Python only
  orchestrates, so there is little to lower.

<a name="gaps"></a>
## Limits and follow-up work

- **Robust real-world detection.** The temporal gate uses a raw BoW-score threshold;
  DLoopDetector normalizes by the expected (previous-frame) score, and a real system
  adds **geometric verification** (RANSAC on matched keypoints) before trusting a loop.
- **Relative pose from matches.** Stage 4's loop factors use the ground-truth relative pose;
  a real system estimates it from the matched features (PnP / essential matrix).
- **Track B (loaned messages / zero-copy SHM transport)** is out of scope: the
  zero-copy here is subscription-callback → `cv::Mat`; a loaned-message intra-process
  path would also remove the DDS-level copy.
- **The full ORB-SLAM back-end** (local mapping, bundle adjustment, relocalization) is
  out of scope, this tutorial is the loop-closure *front-end*.

---

## Where the code lives

| File | What |
|---|---|
| `cv_kit/cv_kit/__init__.py` | OpenCV bringup, zero-copy `msg_to_mat`, ORB, CUDA auto-detect |
| `dbow_kit/dbow_kit/__init__.py` | DBoW2 vocabulary + database (train/load/query) |
| `dbow_kit/cpp/build_dbow2.py` | clone + patch + compile DBoW2 |
| `cv_kit/demos/vision_viz.py` | shared Rerun setup: live-viewer-by-default decision + per-demo blueprints |
| `cv_kit/demos/loop_detector.py` | temporal-consistency loop gate |
| `cv_kit/demos/demo_spine.py` / `demo_features.py` / `demo_loop.py` / `demo_posegraph.py` | stage 1–4 demos |
| `cv_kit/demos/train_vocab.py` / `bench_vision.py` | offline vocab trainer / stage 5 bench |
| `scripts/datasets/synthetic_loop.py` / `dataset_publisher.py` / `download_tum_rgbd.py` / `download_orbvoc.py` | data tooling |
| `cv_kit/tests/test_vision_kits.py` / `cv_kit/tests/test_vision_loop.py` | kit tests + the golden test |
| `cv_kit/REPORT.md` | capability results, measurements, and limitations |
