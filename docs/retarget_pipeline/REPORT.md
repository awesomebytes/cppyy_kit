# retarget_pipeline: perception and humanoid retargeting

**Date:** 2026-07-12. **Machine:** laptop with `/dev/video0` and an RTX PRO 2000. MediaPipe
ran on the CPU.

**Environments:** `pipeline` uses the default robostack Jazzy ROS base, cppyy 3.5,
rerun-sdk 0.34.1, and MediaPipe 0.10.35. MediaPipe comes from PyPI and provides
OpenCV 5.0 and NumPy 2.5.1. `retarget-ros` adds pinocchio 4.1.0, example-robot-data,
and rerun-sdk to the default ROS environment. `wbc` is a standalone environment with
pinocchio and rerun-sdk for offline retargeting.

This pipeline records webcam body and hand landmarks, publishes TF, visualizes the
input and robot in Rerun, and maps the motion to a humanoid robot. It writes a
"policy-kickstart" dataset for policy training. MediaPipe inference and the CLIK
inverse-kinematics solve use their existing Python bindings; cppyy builds the `/tf`
message and batch retarget kernel in C++. Recording and replay are supported.

## Run this example

From this checkout, follow the [run-book](#run-book-spot-check-live) to record a
landmark stream and replay it through the humanoid retargeter.

## Results

- Perception runs at 26 to 30 fps from a live webcam. Synthetic input and stream
  round-trips also work. rclcpp_kit publishes TF from C++.
- Retargeting maps upper-body positions with a fixed base. Talos and G1 are supported.
  The remaining end-effector error is about 3 to 8 cm because some targets exceed the
  robot's reachable workspace.
- The C++ TF message builder was 265 times faster than the Python field-by-field loop.
  The C++ retarget glue kernel was 303.8 times faster than the Python loop, with a
  maximum output difference of 7e-8 m.
- In live ROS mode, median end-to-end lag was 2.5 ms and p90 was 3.5 ms. Both processes
  can log to one Rerun viewer.
- The default robot view uses URDF link meshes. On G1, mesh rendering adds about
  0.55 ms per frame over the skeleton view. Perception runs until Ctrl-C by default.
  The `--min-visibility` option suppresses frames when no person is visible.

---

## Architecture: two processes and a landmark stream

```
Process A  (pixi env: pipeline)                         Process B  (pixi env: wbc)
────────────────────────────────                        ──────────────────────────────
webcam (cv2) / synthetic                                landmark stream  (JSONL, replay/tail)
   │                                                          │
MediaPipe HolisticLandmarker  (library primitive)         load Talos/G1 URDF (pinocchio)
   │  pose_world (33) + hands (21×2)                          │
landmark_stream.py  ──writes JSONL──▶  ◀──reads/tails──   retarget glue kernel  (cppyy_kit C++)
   │                                                       coord xform + target map + One-Euro
/tf (75 frames, built in C++ by rclcpp_kit) ◀── cppyy      │
   │                                                       CLIK per frame  (pinocchio bindings)
live Rerun (camera + 2D/3D skeleton + perf)                │
                                                          Rerun (robot skeleton + human + targets)
                                                          + dataset_<robot>.npz  (q, targets, ee_err)
```

The offline workflow uses separate pixi environments because the package versions in
`wbc` cannot share an environment with ROS. A ROS-capable retarget environment is also
available. conda-forge rebuilt pinocchio 4.x against libboost 1.90, so pinocchio and
example-robot-data now resolve with robostack in the default solve group. The
`retarget-ros` environment runs the CLIK retarget process with ROS. By default,
`retarget.py --source tf` receives frames from `/tf` through rclcpp_kit's C++
TransformListener. The listener ingests data outside the GIL and Python reads it on
lookup. `--replay` and `--follow` remain available in the ROS-free `wbc` environment.
Cling still cannot parse the pinocchio model headers with Boost 1.90, so the IK solve
uses pinocchio's Python bindings in either environment.

The landmark stream is the offline interface. It is JSONL, uses only the standard
library and NumPy, and imports in both environments. Process A can record it and
Process B can replay it, so tests can use the same path as a live run. New streams use
the format tag `cppyy_kit.retarget.landmarks`. Older files tagged
`cppyy_kit.m6f.landmarks` are rejected; record them again in the current format.

---

## Phase 1: perception

| Path | Measured |
|---|---|
| **Live webcam + MediaPipe holistic** (640×480) | detect **27–31 ms/frame**, loop **33–39 ms/frame (~26–30 fps**, webcam-capped), **166 % CPU** (MediaPipe multithreads inference), **0 dropped frames** over 100–134-frame runs, person detected 100 % of frames, clean exit 0 |
| **Synthetic headless** (no camera, no model) | **~5.1 ms/frame (~195 fps)**, the CI/rehearsal fallback |
| **Stream round-trip** | 54 frames written → 54 replayed; JSONL, one meta line + one frame/line |
| **/tf publish** | 75 landmark frames (pose 33 + hands 21×2) on `/tf`, message **built in C++** via a `cppyy.cppdef` broadcaster (rclcpp_kit) |

The cppyy contribution in perception is construction of the `/tf` message. The broadcaster's
`TFMessage` is constructed once in C++ (frame names fixed) and each video frame only its
translations are refilled from one flat address (COMMON_PATTERNS §6). The naive baseline
rebuilds the same message by constructing 75 `TransformStamped` proxies and setting their
fields in a Python loop.

| /tf build (75 frames/msg) | ms/message |
|---|--:|
| **A, cppyy_kit C++ builder (refill persistent msg)** | **0.0005** |
| **B, per-field Python loop (rebuild each frame)** | 0.1440 |
| **A speedup** | **265×** |

The C++ builder reuses the message structure. A Python broadcaster typically rebuilds it
for each frame. This reuse is part of the measured speed difference.

Robustness: no webcam / no model → synthetic scene (prints why); webcam unplug mid-run → after
5 failed reads it falls back to synthetic; `RCLCPPYY_RERUN_SPAWN=0` writes a `.rrd` (verified
59 MB with camera+skeletons+plots), a display spawns the native viewer.

---

## Phase 2: retargeting

Upper-body **position** retarget: human world landmarks → EE targets for the two grippers
(scaled by arm-length ratio, clamped into 0.8× the robot's reachable sphere), solved per frame
by a damped **CLIK** (pinocchio bindings) with a posture regulariser, fixed free-flyer base.

| Robot | frames | CLIK solve (median) | EE err median (L / R) | dataset |
|---|--:|--:|--:|---|
| **Talos** (nq 39), webcam stream | 134 | **0.87 ms/frame** | **0.078 / 0.031 m** (mean 0.053) | `dataset_talos.npz` |
| **Talos**, synthetic stream | 54 | 1.00 ms/frame | 0.059 / 0.058 m | N/A |
| **G1** (nq 36, Unitree, configured with the stretch URDF), synthetic | 54 | **0.82 ms/frame** | **0.041 / 0.041 m** | `dataset_g1.npz` |

The mapping also supports G1. It needs a `RobotConfig` with the URDF path and frame names;
select it with `--robot g1`.

The remaining 3 to 8 cm error comes from the fixed-base robot's reachable workspace. Some
human arm poses map to targets at or beyond Talos's reach. CLIK reaches the clamped target
but cannot remove that residual. A solver bug was also fixed: the solve now uses actuated
columns only instead of zeroing the free-flyer velocity afterward. This reduced EE error
from about 27 cm to about 5 cm.

**Dataset:** `build/pipeline/dataset_<robot>.npz` with `q` (F×nq),
`targets` (F×9), `t`, `ee_err`, `joint_names`, `source_stream`, a per-frame joint trajectory +
its Cartesian targets, ready to seed imitation/BC training.

### Retarget glue benchmark and IK limitation

Cling cannot instantiate `pinocchio::Model` from headers in this environment. The
25-type `JointModel` `boost::variant` exceeds the `make_variant_list` template-arity
limit in Boost 1.90. An out-of-process probe confirmed this for the default-double
`Model` and URDF parser. The precompiled library works, so the IK solve uses pinocchio's
Python bindings. The same error occurs when trying to instantiate other pinocchio model
types from headers; see docs/wbc/REPORT.md.

The C++ glue kernel processes the stream in one `cppyy.cppdef` pass. It converts
coordinates, maps targets, and applies the sequential One-Euro filter across frames
(see COMMON_PATTERNS sections 6 and 26).

| Retarget glue (134 frames: xform + target map + One-Euro) | total ms |
|---|--:|
| **A, cppyy_kit C++ kernel (one cppdef pass)** | **0.013** |
| **B, Python per-frame loop** | 3.850 |
| **A speedup** | **303.8×** (max \|A−B\| = 7e-8 m) |

---

## Motion mapping, trunk posture, and head tracking (2026-07-12)

A live review raised four issues: small hand motion, little target movement, no visible
head movement, and a backward trunk lean. A test with known 0.3 m wrist circles measured
correlation and amplitude from the human wrists to the target and solved end-effector.
This measures motion transfer; end-effector error only measures the solve against its
clamped target.

- Wrist-to-target correlation was **0.89–0.97** per axis. Target-to-solved end-effector
  correlation was about **1.0**. Talos end-effectors moved 13–26 cm. Live TF and replay
  produced similar targets (standard deviation within about 1 mm; q standard deviation
  0.587 vs 0.546). The earlier shoulder-relative mapping used an arm-length scale that
  changed as the arm bent. Its 0.8 reach clamp also limited motion, especially on G1,
  where wrist motion was 5–12 cm for a 0.28 m arm.
- With two gripper position tasks and a uniform posture regularizer, the trunk leaned
  52 degrees on Talos and -17 degrees on G1. Per-joint posture weights now hold the legs
  and torso near the reference and leave the arm joints free. Both robots then had 0.0
  degree torso pitch. Wrist correlation and end-effector motion remained at 0.89–0.97
  and 13–26 cm. `test_trunk_stays_upright` checks that pitch stays below 15 degrees.
- Moving Talos's neck changed `head_2_link` position by 0.000 m. Position alone made the
  head appear stationary, but the neck changes orientation. A retest measured about
  28.6 degrees (0.5 rad) of rotation.

The mapping now uses the human wrist position relative to the hip midpoint and maps it
relative to the robot hip. It uses a fixed ratio of robot torso length to human torso
length, then clamps the target to `reach_frac * arm` at the robot shoulder. Both the C++
kernel and Python stepper use the same mapping. Replay, follow, and TF modes share it.
Known-circle tests measured hip-relative correlation of **1.0** for Talos and **0.98–1.0**
for G1. The free-flyer base remains at the hip origin. The end-effector frames are
`gripper_*_base_link` on Talos and `*_wrist_yaw_link` on G1.

`test_retarget_tracks_wrist_motion` checks hip-relative correlation above 0.7 per axis
and non-zero amplitude. `test_trunk_stays_upright` checks trunk pitch below 15 degrees.

---

## Arm amplitude and Talos head tracking (2026-07-12)

The arm reach clamp changed from 0.8 to 0.95. The `--motion-scale` option, default 1.0,
scales the body-proportion ratio. A 240-frame arm sweep measured the following solved
gripper peak-to-peak travel:

| Robot | Scale 1.0 | Scale 1.5 | Previous 0.8 clamp |
|---|---:|---:|---:|
| **Talos** | **0.75 m** per hand | 1.08 m (target grows; arm reach-clamped) | 13–26 cm |
| **G1** | **0.50 m** per hand | ~0.48 m (already reach-clamped) | shorter arms |

At higher scale, G1 remains limited by arm reach. Talos hand travel increased from
13–26 cm to about 0.75 m.

Talos head yaw and pitch come from the ear-midpoint-to-nose direction in MediaPipe
landmarks. Yaw maps to `head_2_joint` RZ; pitch maps to `head_1_joint` RY. The values are
clamped to joint limits and applied after the arm CLIK loop. Tests confirmed the signs:
look left moves the head left, and look up moves it up. Over a yaw/pitch sweep, measured
correlation was **1.00** for yaw and **0.999** for pitch. Yaw output was 1.02 rad for
1.00 rad input. Pitch is limited by `head_1` at about 0.26 rad. The neck joints are held
by the posture regularizer and changed directly by `_apply_head`.
`test_head_tracks_operator_yaw_pitch` checks this behavior.

G1 has no neck joints. `_detect_neck` returns `has_neck = False`, and `_apply_head` does
nothing. `test_g1_head_is_rigid` checks this behavior.

**What tracks what (both robots):**

| Human input | Talos | G1 |
|---|---|---|
| Left / right wrist (rel. to hips) | left / right gripper, hip-relative, ~1:1 amplitude | left / right wrist, hip-relative (~0.5 m sweep) |
| Head yaw / pitch (nose vs ears) | **head yaw + pitch** (neck joints, corr ≥0.999) | no neck joints; rigid head |
| Trunk | pinned upright (posture regulariser) | pinned upright |
| Free-flyer base | locked at hip origin | locked at hip origin |

---

## Scope and limitations

- **ML inference is a library primitive** (MediaPipe, CPU ~30 ms/frame), deliberately NOT
  wrapped in cppyy (the live-webcam demo's benchmark result). cppyy is not used for inference.
- **Measured cppyy results:** /tf marshaling **265×**,
  retarget glue kernel **303.8×**, both Pattern 6/26 (build/refill in C++; keep the sequential
  loop in C++), both with numeric comparison tests.
- **The retarget solve uses pinocchio bindings.** Cling cannot instantiate the required
  pinocchio model from headers in this environment.
- **Retarget fidelity is a partial implementation**: upper-body position-only with a fixed base and about 3 to 8 cm
  reachable-workspace residual. The mapping does not model full-body biomechanics.

---

## Notes for COMMON_PATTERNS

1. **The Boost variant template-arity limit also affects to pinocchio's default-double `Model`, not just exotic
   scalars (2nd instance, updates wbc §20).** Anything that instantiates `pinocchio::Model`
   from headers under Cling (URDF parse, FK on a real robot, a crocoddyl `StateMultibody`) hits
   boost 1.90's `make_variant_list` arity limit on the 25-type `JointModel` variant. Rule: drive
   pinocchio's rigid-body core via its **Python bindings**; cppyy's use in this stack is the
   abstract/custom-model path (crocoddyl action models; see docs/wbc/REPORT.md) and *non-pinocchio* glue kernels,
   not the multibody `Model`.
2. **Build-once-in-C++, refill-per-frame for ROS messages (updates §6).** A persistent C++-side
   message (`TFMessage`) whose data is refilled from a raw address each frame beats
   reconstructing the message's proxies field-by-field in Python (265× for 75 TF frames). The
   general "keep the container in C++" rule, applied to a repeatedly-published message.
3. **Two-env pipeline coupled by a replayable stream file.** When a hard env boundary forces two
   processes (here ROS vs pinocchio/boost), a **tailable/replayable JSONL stream** is the seam:
   live coupling = tail; CI/rehearsal = replay; and a **coordinate-frame contract module** with
   only stdlib+numpy imports without errors in both envs. Recording and replay are available.
4. **First pip dependency in a conda/pixi repo (mediapipe).** Put it in a dedicated feature env
   with `[pypi-dependencies]`; **verify the pip deps' numpy equals the conda numpy** (here both
   2.5.1, no split) and **exclude any conda package the pip dep re-provides** (do NOT compose
   the `vision` feature's conda opencv with mediapipe's pip `opencv-contrib-python`). Compose with
   the ROS default via `solve-group="default"` so the shared stack stays one solve.
5. **MediaPipe 0.10.x API shift (recon fact worth a note).** The legacy `mp.solutions` API is
   gone; only the Tasks API remains. `HolisticLandmarker` gives pose + both hands + face +
   **world landmarks** (metric 3D) in one call; models are `.task` bundles downloaded separately
   (fetch-once cache + synthetic fallback when offline).

---

## Environment and lock changes

- **New `[feature.pipeline]` + `pipeline` env** (`solve-group="default"`): adds `rerun-sdk 0.34.*`
  (conda) and `mediapipe==0.10.35` (**the repo's first pip dependency**, in a
  `[pypi-dependencies]` section). Proven: `pixi install -e pipeline` solves; mediapipe + cv2 +
  rerun + cppyy + rclcpp_kit all import together, numpy stays 2.5.1.
- **Added `rerun-sdk 0.34.*` to `[feature.wbc]`** so Process B can log the retargeted humanoid.
  wbc is standalone, so this only re-locks the wbc env.
- **`pixi.lock` re-locked, purely additive** (1783 insertions, 0 deletions; no existing pin
  moved), because the pipeline env is solve-group=default and wbc is standalone.
- Tasks added: `fetch-models`, `demo-perceive`, `bench-perceive`, `test-pipeline` (pipeline env);
  `demo-retarget`, `bench-retarget`, `test-retarget` (wbc env). `retarget_pipeline` added to the
  `lint` task. The default `test` task is **unchanged**.
- **New `[feature.retarget-ros]` + `retarget-ros` env** (`solve-group="default"`): the ROS-native
  retarget home, `pinocchio` + `example-robot-data` + `rerun-sdk` on top of the default ros-base
  (so rclcpp_kit's C++ TF listener and the pinocchio CLIK run in one process). Proven:
  `pixi install -e retarget-ros` solves; pinocchio 4.1.0 + rerun + cppyy + rclcpp_kit import and
  coexist, Talos loads, the C++ TransformListener creates + shuts down without errors. Tasks:
  `demo-retarget-ros` (`--source tf`), `test-retarget-ros`.
- **Lock shift from adding pinocchio to the default solve-group, narrow and benign.** The only
  shared-package change across the default-group envs (bt, control, ik, moveit, nav2, ompl, pcl,
  pipeline, rclcpp, vision, vision-cuda) is a **libopenblas threading-backend flip** (`0.3.33
  pthreads_` → `0.3.33 openmp_`, **same version**) with `+ llvm-openmp` / `_openmp_mutex` gnu→llvm.
  **No numeric-version changes**; in particular **urdfdom did NOT move** (a worry going in). The
  standalone envs (wbc, cudabuild, docs, pkg) are untouched. Every affected kit suite re-verified
  green (see Gates).

**Pinned model bundle (supply-chain hygiene).** `fetch_models.py` pins each MediaPipe Tasks
bundle's URL **and SHA-256**, verifies the hash after download, and refuses (and removes) a
mismatch. The perception default uses `holistic` (`float16/latest`, downloaded 2026-07-12):
`holistic_landmarker.task`, 13 683 609 bytes, sha256
`e2dab61191e2dcd0a15f943d8e3ed1dce13c82dfa597b9dd39f562975a50c3f8`. (Also pinned: `pose` =
`4eaa5eb7…`, `hand` = `fbc2a300…`.) Caveat: the URL is Google's `.../latest/`, so a bundle
rotation will change the hash and be refused, re-pin, or pass `--allow-hash-mismatch` /
`RETARGET_ALLOW_HASH_MISMATCH=1` to knowingly accept a new bundle. Verified: a cached bundle whose
hash matches is not re-downloaded; a deliberately-wrong pin is refused and the `.part` cleaned.

---

## Gates

Recorded gate results:

- `pixi run lint` → **0**.
- `pixi run test` (default env) → **56 passed, 130 skipped** (unchanged by this lane;
  `retarget_pipeline/tests` is not in the default task).
- `pixi run -e pipeline test-pipeline` → **9 passed**.
- `pixi run -e wbc test-retarget` → **8 passed** (build Talos + G1, C++/Python glue agreement,
  bounded-error retarget + dataset, live `--follow` cold-start survival, mutual-exclusion,
  source dispatch).
- `pixi run -e retarget-ros test-retarget-ros` → **8 passed** (same suite, ROS env).
- **Every affected default-solve-group kit suite re-verified green** after the libopenblas
  backend flip (no OpenMP-runtime crash, no numeric change): `rclcpp` 13, `bt` 49/2skip,
  `control` 7, `ompl` 9, `nav2` 14, `moveit` 11, `ik` 7, `pcl` (accelerate) 3, `vision` 13/14skip.
- `pixi run docs-build` (strict) → clean.
- New env solves (`pixi install -e retarget-ros`); full workspace `pixi lock` succeeds.

---

## Run-book (spot-check live)

```bash
# --- Process A: perception (pipeline env) ---
pixi run -e pipeline fetch-models                                   # one-time: MediaPipe models
ROS_DOMAIN_ID=62 pixi run -e pipeline demo-perceive                 # live webcam + Rerun window
pixi run -e pipeline demo-perceive --source synthetic --duration 10 # no camera (headless-safe)
# record a stream, then the retarget half replays it:
ROS_DOMAIN_ID=62 pixi run -e pipeline demo-perceive --record build/pipeline/demo.jsonl --duration 15
pixi run -e pipeline bench-perceive --replay build/pipeline/demo.jsonl   # /tf-build 265x

# --- Process B: retargeting (wbc env), OFFLINE replay ---
pixi run -e wbc demo-retarget --robot talos --replay build/pipeline/demo.jsonl
pixi run -e wbc demo-retarget --robot g1    --replay build/pipeline/demo.jsonl   # G1 stretch
pixi run -e wbc bench-retarget --replay build/pipeline/demo.jsonl                # glue 303.8x

# tests
pixi run -e pipeline test-pipeline
pixi run -e wbc test-retarget
```

### Live ROS-native teleoperation (two terminals, one viewer)

Perception broadcasts about 75 landmark transforms on `/tf`. Retargeting receives them through
rclcpp_kit's C++ `TransformListener`, which ingests them on its own thread outside the GIL.
The measured ingest speedup was 6.7–14×. Python reads the transforms during lookup. The processes
do not exchange a stream file in this mode. Both log to one Rerun viewer. Perception starts the
viewer, and retargeting connects over gRPC using the same recording id. The viewer shows the
camera, landmark skeleton, retargeted humanoid, and targets.

```bash
# Terminal A, perception: publish /tf and open the shared viewer.
# Run until Ctrl-C. Interactive use does not need --duration.
ROS_DOMAIN_ID=62 pixi run -e pipeline demo-perceive --shared-viewer
# Terminal B, retarget: consume /tf with the C++ listener and connect to the shared viewer.
ROS_DOMAIN_ID=62 pixi run -e retarget-ros demo-retarget-ros --robot g1 --shared-viewer
```

The robot is drawn with URDF link meshes by default (`--robot-viz mesh`), pinocchio
loads the visual `GeometryModel`, each link's STL is logged once as a static `rr.Asset3D`, and per
frame only a `rr.Transform3D` per link updates its pose from FK (`--robot-viz skeleton` falls back
to the joint tree, and any link that is an inline primitive rather than a mesh file is skipped,
for example Talos's wrist-FT/IMU). **Per-frame viz cost** (G1, 35 meshes): mesh **1.31 ms/frame** vs
skeleton 0.76 ms (Δ ~0.55 ms, the geometry-placement update + 35 transform logs); negligible
against the 33 ms frame budget.

`--source tf` is the default when neither `--replay` nor `--follow` is given; it waits up to
`--startup-timeout` (30 s) for the first `/tf` frames and exits when the stream goes idle
(`--idle-timeout`) or on Ctrl-C, writing the dataset. Perception's `--duration` now defaults to
**0 = run until Ctrl-C** (interactive); pass a positive value to cap it (tests / timed benches).
Both processes shut down without errors on SIGINT (stream closed, dataset written, viewer flushed).

A **presence gate** (`--min-visibility`, default 0.5) means perception broadcasts /tf and
records a pose only when the key landmarks' mean visibility is above the threshold. If no person
is visible, retarget's tf mode receives no new frames and sends no robot output. Set
`--min-visibility 0` to disable.

**Measured:** with synthetic perception at 30 fps publishing /tf and a G1 consumer, headless
publish-to-retarget lag was 2.5 ms median, 3.5 ms p90, and 3.8 ms maximum. CLIK took about
1.2 ms/frame, and end-effector error was about 4.6 cm. This was lower than the file-follow
measurement below. In a visual check, perception started one viewer and retargeting connected
to it using the same gRPC endpoint and recording id. The viewer showed the G1 URDF meshes and
human skeleton together. The IK solve uses pinocchio's Python bindings because Cling cannot
instantiate `Model`. The Boost 1.90 rebuild allows pinocchio to resolve in the ROS environment.

### Live teleoperation with a stream file (`--follow`)

Run perception and retargeting at the same time. The `--follow` option reads the growing stream
file and retargets each frame after the writer flushes it (`landmark_stream.follow()`). This works
without a shared ROS graph because the retarget process can run in the ROS-free `wbc` environment.
CI also replays this file format.

```bash
# Terminal A (producer): use a webcam if available, otherwise synthetic input.
ROS_DOMAIN_ID=62 pixi run -e pipeline demo-perceive --record build/pipeline/live.jsonl --duration 30
# Terminal B (consumer): start first. It waits for the file and first frame.
pixi run -e wbc python retarget_pipeline/retarget.py --robot g1 --follow build/pipeline/live.jsonl
```

`--follow` has two timeouts. The startup timeout (`--startup-timeout`, default 30 s) allows time
for the file and first frame to appear. Starting `demo-perceive` takes several seconds while its
environment and model load. The idle timeout (`--idle-timeout`, default 2 s) starts after the last
frame and controls when the dataset is written. The process also exits on EOF or Ctrl-C.
**Measured** (synthetic producer at 30 fps, G1 consumer, 300
frames consumed as produced): **end-to-end producer→consumer lag median 4.4 ms (p90 6.5, max
10.1 ms)**. This is below one 33 ms frame period. CLIK took about 1.3 ms/frame. The measurement
used synthetic input; webcam input uses the same stream handling. In follow mode, each target is
computed by the per-frame Python stepper (`_frame_target` + `_EuroState`, about 0.03 ms), not the
batch C++ glue kernel. The batch kernel benchmark covers offline replay and `--bench`; per-frame
glue time is small compared with the CLIK solve.

The `--follow` path logs to its own Rerun viewer or a headless `.rrd` file. The shared viewer is
available in ROS-native mode when both processes use `--shared-viewer`. Without a display or
under pytest, each process writes its own `.rrd` file.
