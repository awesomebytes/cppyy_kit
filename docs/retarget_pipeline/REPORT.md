# retarget_pipeline — perception → humanoid retargeting capture rig

**Date:** 2026-07-12 · **Envs:** pixi `pipeline` (perception) = default robostack-jazzy
ros-base + `cppyy 3.5` + `rerun-sdk 0.34.1` (conda) + `mediapipe 0.10.35` (pypi, brings
`opencv-contrib-python`/cv2 5.0 + numpy 2.5.1); pixi `wbc` (retarget, standalone) =
`pinocchio 4.0.0` + `rerun-sdk 0.34.1`, Python 3.12, linux-64. **Machine:** quiet laptop,
`/dev/video0`, RTX PRO 2000 (GPU unused — MediaPipe ran on CPU here).

**The ask (locked with the owner):** a minimal-code human-demonstration capture rig —
webcam → body + hand tracking → TF + live Rerun → whole-body retargeting onto a humanoid
(Talos) → a recorded "policy-kickstart" dataset — that bootstraps humanoid policy training.
Hybrid line: ML inference stays commodity Python (MediaPipe); cppyy_kit owns the genuinely
hot glue. Record + replay from day one.

**Verdict:**
- **Phase 1 (perception): WORKS**, live at usable FPS, synthetic-headless fallback, stream
  round-trips, TF via rclcpp_kit built in C++.
- **Phase 2 (retarget): WORKS** (a precise partial — upper-body position retarget, fixed
  base), Talos + **G1 stretch delivered as a zero-code URDF swap**, dataset artifact
  written. The residual (~3–8 cm) is honest reachable-workspace limit, not solver error.
- **The honest cppyy_kit wins are in the glue, measured:** /tf-message marshaling **265×**
  (perception) and the retarget glue kernel **303.8×** (bit-identical). The IK **solve**
  itself is a pinocchio-bindings job — cppyy is **blocked** there by a documented wall
  (below), which is itself a useful finding.

---

## Architecture as built (two processes, one stream seam)

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

The two run in **separate pixi envs** on purpose: pinocchio's conda stack pins libboost 1.86
and the ROS stack pins 1.90 — they cannot share a process (docs/wbc/REPORT.md). The **landmark
stream file is the seam** — a tailable/replayable JSONL contract (`landmark_stream.py`,
stdlib+numpy only, imports in both envs). Record/replay is not a mode bolted on later: A always
can `--record`, B always reads a stream (`--replay`), so CI and rehearsal run the exact live
code path headless.

---

## Phase 1 — perception (GO/NO-GO gate: PASSED)

| Path | Measured |
|---|---|
| **Live webcam + MediaPipe holistic** (640×480) | detect **27–31 ms/frame**, loop **33–39 ms/frame (~26–30 fps**, webcam-capped), **166 % CPU** (MediaPipe multithreads inference), **0 dropped frames** over 100–134-frame runs, person detected 100 % of frames, clean exit 0 |
| **Synthetic headless** (no camera, no model) | **~5.1 ms/frame (~195 fps)** — the CI/rehearsal fallback |
| **Stream round-trip** | 54 frames written → 54 replayed; JSONL, one meta line + one frame/line |
| **/tf publish** | 75 landmark frames (pose 33 + hands 21×2) on `/tf`, message **built in C++** via a `cppyy.cppdef` broadcaster (rclcpp_kit) |

**The cppyy_kit win here (perception glue): building the /tf message.** The broadcaster's
`TFMessage` is constructed once in C++ (frame names fixed) and each video frame only its
translations are refilled from one flat address (COMMON_PATTERNS §6). The naive baseline
rebuilds the same message by constructing 75 `TransformStamped` proxies and setting their
fields in a Python loop.

| /tf build (75 frames/msg) | ms/message |
|---|--:|
| **A — cppyy_kit C++ builder (refill persistent msg)** | **0.0005** |
| **B — per-field Python loop (rebuild each frame)** | 0.1440 |
| **A speedup** | **265×** |

Honest note: part of A's edge is that the message *structure* is reused — but that reuse is
only possible because it lives in C++; a Python broadcaster typically rebuilds per frame. This
is the realistic contrast and the reason to use the helper.

Robustness: no webcam / no model → synthetic scene (prints why); webcam unplug mid-run → after
5 failed reads it falls back to synthetic; `RCLCPPYY_RERUN_SPAWN=0` writes a `.rrd` (verified
59 MB with camera+skeletons+plots), a display spawns the native viewer.

---

## Phase 2 — retargeting (WORKS, precise partial)

Upper-body **position** retarget: human world landmarks → EE targets for the two grippers
(scaled by arm-length ratio, clamped into 0.8× the robot's reachable sphere), solved per frame
by a damped **CLIK** (pinocchio bindings) with a posture regulariser, fixed free-flyer base.

| Robot | frames | CLIK solve (median) | EE err median (L / R) | dataset |
|---|--:|--:|--:|---|
| **Talos** (nq 39) — webcam stream | 134 | **0.87 ms/frame** | **0.078 / 0.031 m** (mean 0.053) | `dataset_talos.npz` |
| **Talos** — synthetic stream | 54 | 1.00 ms/frame | 0.059 / 0.058 m | — |
| **G1** (nq 36, Unitree, **stretch**) — synthetic | 54 | **0.82 ms/frame** | **0.041 / 0.041 m** | `dataset_g1.npz` |

**G1 stretch: delivered as a zero-code swap** — the retarget mapping is model-generic, so G1 is
one `RobotConfig` (URDF path + frame names); `--robot g1` just works.

**The residual is reachable-workspace limit, honestly.** A single human's arm poses map to
targets at/beyond Talos's fixed-base reachable set; the ~3–8 cm residual is the CLIK reaching the
*clamped* target's edge, not a convergence failure. (Solver bug found & fixed en route: locking
the free-flyer by zeroing the base velocity *after* solving the full system discards the
solution's dominant base component — the fix solves over the actuated columns only. Dropped EE
error from ~27 cm to ~5 cm.)

**Dataset artifact ("policy-kickstart"):** `build/pipeline/dataset_<robot>.npz` with `q` (F×nq),
`targets` (F×9), `t`, `ee_err`, `joint_names`, `source_stream` — a per-frame joint trajectory +
its Cartesian targets, ready to seed imitation/BC training.

### The cppyy_kit win here (retarget glue), and the honest boundary on the solve

The natural "lower the CLIK to inline C++ calling pinocchio" move is **BLOCKED in this env**:
instantiating `pinocchio::Model` from headers under Cling trips **boost 1.90's variant
template-arity wall** — pinocchio's 25-type `JointModel` `boost::variant` exceeds
`make_variant_list`'s limit. This is the *same* wall docs/wbc/REPORT.md hit for templated
scalars, now confirmed for the **default-double `Model` + URDF parser** (probed out-of-process:
clean compile error at `JointModelTpl<double>`, not a crash). So the IK **solve is a
pinocchio-bindings job** — the precompiled library carries the variant; the bindings are the
right tool (matching the REPORT's "bindings are fine" cases).

cppyy_kit's real contribution to Process B is the per-frame **glue kernel** — coordinate
transform + target mapping + a **sequential One-Euro landmark filter** — authored in one
`cppyy.cppdef` pass over the whole stream. The One-Euro filter is sequential across frames: the
per-element Python-loop trap (§6/§26).

| Retarget glue (134 frames: xform + target map + One-Euro) | total ms |
|---|--:|
| **A — cppyy_kit C++ kernel (one cppdef pass)** | **0.013** |
| **B — Python per-frame loop** | 3.850 |
| **A speedup** | **303.8×** (max \|A−B\| = 7e-8 m — bit-identical) |

---

## Honest boundaries (library-primitive vs cppyy-won)

- **ML inference is a library primitive** (MediaPipe, CPU ~30 ms/frame) — deliberately NOT
  wrapped in cppyy (the live-webcam demo's honest-headline lesson). No cppyy claim is made on it.
- **cppyy_kit wins are in the glue, and only where measured:** /tf marshaling **265×**,
  retarget glue kernel **303.8×** — both Pattern 6/26 (build/refill in C++; keep the sequential
  loop in C++), both with numeric agreement checks.
- **The retarget solve is bindings, not cppyy** — an honest "no kit needed / kit blocked" cell,
  documented with the exact wall.
- **Retarget fidelity is a precise partial**: upper-body position-only, fixed base, ~3–8 cm
  reachable-workspace residual. No biomechanical claim.

---

## Generic-lesson candidates for COMMON_PATTERNS (for the lead — not added by me)

1. **The boost-variant JIT wall applies to pinocchio's default-double `Model`, not just exotic
   scalars (2nd instance, sharpens wbc §20).** Anything that instantiates `pinocchio::Model`
   from headers under Cling (URDF parse, FK on a real robot, a crocoddyl `StateMultibody`) hits
   boost 1.90's `make_variant_list` arity limit on the 25-type `JointModel` variant. Rule: drive
   pinocchio's rigid-body core via its **Python bindings**; cppyy's win for this stack is the
   abstract/custom-model path (crocoddyl action models; see docs/wbc/REPORT.md) and *non-pinocchio* glue kernels,
   not the multibody `Model`.
2. **Build-once-in-C++, refill-per-frame for ROS messages (sharpens §6).** A persistent C++-side
   message (`TFMessage`) whose data is refilled from a raw address each frame beats
   reconstructing the message's proxies field-by-field in Python (265× for 75 TF frames). The
   general "keep the container in C++" rule, applied to a repeatedly-published message.
3. **Two-env pipeline coupled by a replayable stream file.** When a hard env boundary forces two
   processes (here ROS vs pinocchio/boost), a **tailable/replayable JSONL stream** is the seam:
   live coupling = tail; CI/rehearsal = replay; and a **coordinate-frame contract module** with
   only stdlib+numpy imports cleanly in both envs. Record/replay-from-day-one is a design stance,
   not a mode.
4. **First pip dependency in a conda/pixi repo (mediapipe).** Put it in a dedicated feature env
   with `[pypi-dependencies]`; **verify the pip deps' numpy equals the conda numpy** (here both
   2.5.1 — no split) and **exclude any conda package the pip dep re-provides** (do NOT compose
   the `vision` feature's conda opencv with mediapipe's pip `opencv-contrib-python`). Compose with
   the ROS default via `solve-group="default"` so the shared stack stays one solve.
5. **MediaPipe 0.10.x API shift (recon fact worth a note).** The legacy `mp.solutions` API is
   gone; only the Tasks API remains. `HolisticLandmarker` gives pose + both hands + face +
   **world landmarks** (metric 3D) in one call; models are `.task` bundles downloaded separately
   (fetch-once cache + synthetic fallback when offline).

---

## Env / lock changes (flag for the lead)

- **New `[feature.pipeline]` + `pipeline` env** (`solve-group="default"`): adds `rerun-sdk 0.34.*`
  (conda) and `mediapipe==0.10.35` (**the repo's first pip dependency**, in a
  `[pypi-dependencies]` section). Proven: `pixi install -e pipeline` solves; mediapipe + cv2 +
  rerun + cppyy + rclcpp_kit all import together, numpy stays 2.5.1.
- **Added `rerun-sdk 0.34.*` to `[feature.wbc]`** so Process B can log the retargeted humanoid.
  wbc is standalone, so this only re-locks the wbc env.
- **`pixi.lock` re-locked — purely additive** (1783 insertions, 0 deletions; no existing pin
  moved), because the pipeline env is solve-group=default and wbc is standalone.
- Tasks added: `fetch-models`, `demo-perceive`, `bench-perceive`, `test-pipeline` (pipeline env);
  `demo-retarget`, `bench-retarget`, `test-retarget` (wbc env). `retarget_pipeline` added to the
  `lint` task. The default `test` task is **unchanged** (still 40 passed / 129 skipped).

**Pinned model bundle (supply-chain hygiene).** `fetch_models.py` pins each MediaPipe Tasks
bundle's URL **and SHA-256**, verifies the hash after download, and refuses (and removes) a
mismatch. The perception default uses `holistic` (`float16/latest`, downloaded 2026-07-12):
`holistic_landmarker.task` — 13 683 609 bytes, sha256
`e2dab61191e2dcd0a15f943d8e3ed1dce13c82dfa597b9dd39f562975a50c3f8`. (Also pinned: `pose` =
`4eaa5eb7…`, `hand` = `fbc2a300…`.) Caveat: the URL is Google's `.../latest/`, so a bundle
rotation will change the hash and be refused — re-pin, or pass `--allow-hash-mismatch` /
`RETARGET_ALLOW_HASH_MISMATCH=1` to knowingly accept a new bundle. Verified: a cached bundle whose
hash matches is not re-downloaded; a deliberately-wrong pin is refused and the `.part` cleaned.

---

## Gates

- `pixi run lint` → **0**.
- `pixi run test` (default env) → **40 passed, 129 skipped** (unchanged).
- `pixi run -e pipeline test-pipeline` → **9 passed** (stream contract + synthetic-headless
  round-trip + the /tf-build A>B bench).
- `pixi run -e wbc test-retarget` → **4 passed** (Talos + G1 build, C++/Python glue agreement,
  end-to-end bounded-error retarget + dataset).
- New envs solve (`pixi install -e pipeline`, `pixi install -e wbc`).

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

# --- Process B: retargeting (wbc env) ---
pixi run -e wbc demo-retarget --robot talos --replay build/pipeline/demo.jsonl
pixi run -e wbc demo-retarget --robot g1    --replay build/pipeline/demo.jsonl   # G1 stretch
pixi run -e wbc bench-retarget --replay build/pipeline/demo.jsonl                # glue 303.8x

# tests
pixi run -e pipeline test-pipeline    # 9 passed
pixi run -e wbc test-retarget         # 4 passed
```

For the full live demo both processes log to Rerun; run them as separate viewers, or connect B
to A's viewer with `rr.connect_grpc()` (noted as an option, not wired by default in this spike).
