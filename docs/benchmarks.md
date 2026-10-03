# Benchmark results from one shared machine

**Machine:** Intel Core Ultra 9 285H (16 threads), Ubuntu 24.04.4 LTS, kernel
6.17.0-1028-oem. **Date:** 2026-07-12.

**Env versions (pixi, this checkout):** Python 3.12.13 · cppyy 3.5.0 ·
`ros-jazzy-ros-base` 0.11.0 (robostack-jazzy) · OpenCV 4.13.0 · pinocchio 4.0.0 ·
crocoddyl 3.2.1 · tsid 1.10.0.

The commands and environment versions below identify this measurement pass. Each
kit's `REPORT.md` keeps the original dated results. Differences greater than about
20% are called out here. Other work ran on the machine during these measurements.

## PCL pipeline: cloud stays in C++ end to end

`ROS_DOMAIN_ID=66 pixi run -e pcl bench-pcl`

| variant | avg lat ms | p99 lat ms | CPU% @10Hz | max msgs/s | user LOC |
|---|--:|--:|--:|--:|--:|
| pcl_kit (C++ end-to-end) | 4.10 | 10.65 | 8.7 | 244 | 74 |
| rclpy + NumPy baseline | 62.09 | 80.35 | 64.8 | 16 | 74 |

Latency was 15.1x lower and CPU use was 7.4x lower. Both examples had 74 user LOC.
The historical result in `pcl_kit/REPORT.md` was 14.8x / 9.4x at 76 vs 77 LOC. The
new CPU ratio is about 21% lower than that result. The earlier report is unchanged.

## PCL compile cache: frame-0 first-use JIT vs cached

`pixi run -e pcl bench-cache-pcl`

| config | frame-0 time |
|---|--:|
| JIT (Python VoxelGrid) | 632 ms |
| cached, run 1 (miss, compiles) | 91 ms |
| cached, run 2 (hit) | 89 ms |
| cached, run 3 (hit) | 94 ms |

The first cached run took about 6.9x less time (632 ms to about 91 ms). The historical
result was about 681 ms to 88 ms (~7.7x). Single-frame JIT timings vary between runs.

## bt_kit compile cache: t01 cold-run adoption

`pixi run -e bt validate-cache-bt` (correctness check) then
`pixi run -e bt bench-cache-bt` (the cold-run number):

| config | first_register | first_tick | wall |
|---|--:|--:|--:|
| JIT (no cache) | 218 ms | 9 ms | 1836 ms |
| cached, run 1 (miss, compiles) | 74 ms | 5 ms | 4016 ms |
| cached, run 2 (hit) | 62 ms | 5 ms | 1279 ms |
| cached, run 3 (hit) | 65 ms | 6 ms | 1281 ms |

The historical L0 results in `bt_kit/REPORT.md` were about 233/8/1770 ms for JIT and
60/5/1200 ms for cached runs. The results are within a few percent.

## Vision: cv_kit + dbow_kit synthetic sequence

One-time vendor build: `pixi run -e vision build-dbow2`. Then
`pixi run -e vision bench-vision` (200-frame synthetic loop-closure sequence):

| measure | rclcppyy (C++) | rclpy (Python) | speedup |
|---|--:|--:|--:|
| ingest, 640x480 mono | 0.0079 ms | 0.0099 ms | 1.3x |
| ingest, 1920x1080 mono | 0.0012 ms | 0.1679 ms | 135.8x |

ORB: 280.9 fps (3.56 ms/frame, 1000 keypoints). Vocabulary train (k=10, L=4, 9970
words): 8704 ms; query 3.759 ms/frame; 19 loops confirmed (0 false positives),
precision/recall 1.00/0.95. Historical results were ingest 1.3x / ~155x, ORB ~3.7
ms/frame, vocabulary training ~7 s, and query ~2.8 ms/frame. The 1920x1080 ingest
ratio and vocabulary timings differed more on this run. The historical report also
notes variation on this benchmark. The ORB and 640x480 ingest results are close to
the historical values.

## Webcam demo: A (cppyy_kit C++) vs B (naive Python)

`pixi run -e vision bench-webcam` (synthetic scene, no camera):

| res | tracked | A (C++) | B (Python) | A speedup |
|---|--:|---|---|--:|
| 640x480 | 140 | 4.12 ms · 242.8 fps · 91% cpu | 66.66 ms · 15.0 fps · 99% cpu | 16.18x |
| 1280x720 | 150 | 6.22 ms · 160.9 fps · 94% cpu | 74.83 ms · 13.4 fps · 99% cpu | 12.04x |

The historical results in `docs/webcam_demo/REPORT.md` were 15.4x / 11.9x. The new
results are within a few percent.

## IK benchmark: same Panda, same 200 targets, per-solver subprocess

One-time vendored builds: `pixi run -e ik build-bio-ik` and
`pixi run -e ik build-pick-ik`. Then `pixi run -e ik bench-ik`:

| solver | success | solve/s | pos err (mm) | ori err (deg) |
|---|--:|--:|--:|--:|
| KDL (packaged) | 98.5% | 398 | 0.000 | 0.000 |
| TRAC-IK (packaged) | 98.5% | 903 | 0.001 | 0.000 |
| bio_ik (vendored C++) | 98.5% | 991 | 0.001 | 0.000 |
| pick_ik (vendored C++) | 97.5% | 125 | 0.466 | 0.016 |
| pure-Python DLS (NumPy) | 71.0% | 40 | 0.699 | 0.010 |

The pure-Python solver was 10 to 25x slower than the C++ solvers (398/40 to 991/40),
within the historical range. All 5 solvers ran in this pass. The first attempt was
blocked because bio_ik and pick_ik had not been built. Building both resolved it.

## WBC: custom Crocoddyl action model, Python-derived vs inline-C++

`pixi run -e wbc demo-wbc-lower` (unicycle optimal control, T=100, FDDP):

| variant | cost | iters | solve time | speedup |
|---|--:|--:|--:|--:|
| (A) Python-derived model | 250.039320 | 8 | 6.99 ms | 1.0x |
| (ref) built-in C++ (binding) | 250.039320 | 8 | 0.32 ms | 21.8x |
| (B) cppyy inline C++ model | 250.039320 | 8 | 0.31 ms | 22.9x |

The three variants returned the same cost. The historical speedup was 21.7x
(0.32 ms / 0.34 ms), within run-to-run variation of this result.

## Accelerate example

`pixi run -e pcl test-accelerate` (the differential contract): **3 passed**. Then
`pixi run -e pcl bench-accelerate`:

| variant | median | speedup |
|---|--:|--:|
| naive Python loop | 49.599 ms | 1.0x (base) |
| pcl_kit (C++ VoxelGrid) | 3.044 ms | 16.3x |

The historical result was 15.6x (47.9 ms to 3.07 ms). The new result is within a few
percent.

## Retarget pipeline: perception /tf marshaling + retarget glue kernel

Recorded a **synthetic** landmark stream first (no webcam, no person needed):

```
ROS_DOMAIN_ID=66 pixi run -e pipeline demo-perceive --source synthetic \
    --record build/pipeline/demo.jsonl --duration 15 --no-viz
```

Then, on that recording:

`pixi run -e pipeline bench-perceive --replay build/pipeline/demo.jsonl`
(443 messages, 75 landmark frames/message):

| | ms/message | speedup |
|---|--:|--:|
| A: cppyy_kit C++ /tf builder | 0.0005 | 258.9x |
| B: per-field Python loop | 0.1317 | - |

`pixi run -e wbc bench-retarget --replay build/pipeline/demo.jsonl` (443 frames,
coord transform + target map + One-Euro filter):

| | total time | speedup |
|---|--:|--:|
| A: cppyy_kit C++ kernel (one `cppdef` pass) | 0.038 ms | 341.5x |
| B: Python per-frame loop | 12.850 ms | - |

max \|A−B\| = 4.12e-08 m. Historical results were 265x for /tf marshaling and
303.8x for the glue kernel. The marshaling ratio is close. The glue-kernel ratio is
about 12% higher here. Both timings are sub-millisecond totals over 443 frames, so
scheduling variation can affect the ratios.

## Jitter bench: reduced reference set (a1 / b / c, idle, 60 s each)

`ROS_DOMAIN_ID=66 pixi run -e control python jitter_bench/run_bench.py --variant a1,b,c --condition idle --duration 60 --mlock --cpu 2 --no-hist`

| variant | p50 (µs) | p99 (µs) | p99.9 (µs) | max (µs) | mean (µs) | late % |
|---|--:|--:|--:|--:|--:|--:|
| a1: pure-Python `clock_nanosleep` | 2.7 | 539.6 | 1853.1 | 4095.8 | 23.8 | 0.86% |
| b: cppyy_kit C++ loop (cppdef_cached+nogil) | 2.3 | 594.5 | 1981.2 | 4977.4 | 27.0 | 0.88% |
| c: real ros2_control loop (Python controller) | 2.6 | 757.8 | 2008.8 | 3061.7 | 24.4 | 1.11% |

The first combined run (a1+b+c in one process) recorded a ~1.3 s machine-wide stall
in variant c (max 1 292 591.9 µs). `docs/jitter_bench/REPORT.md` §1 records a
similar stall for variant a1 under load. Other benchmarks were running at the same
time. The c row above is a rerun (`--variant c --condition idle --duration 60 --mlock
--cpu 2`) without that stall. Median values were ~2 to 3 µs across all variants, with
tail values in the low millisecond range. Late percentages were 0.86% to 1.11%,
compared with historical values of 0.65% to 0.70%.

## TF ingest: C++ tf2 listener vs Python callback

`ROS_DOMAIN_ID=66 pixi run -e rclcpp bench-tf`

| scenario | ingest CPU% py / cpp | observed py/cpp ratio |
|---|---|--:|
| idle (no storm) | 0.0 / 0.0 | - |
| 1k tf/s | 3.7 / 0.5 | 7.4x |
| 5k tf/s | 11.2 / 0.8 | 14.0x |
| 10k tf/s | 15.2 / 0.9 | 16.9x |

The historical shared-host pass measured ratios of 6.7 to 14. This run measured 16.9
at 10k tf/s. The mechanism and value correctness are tested separately.

## Auto-PCH: zero-config cold vs warm bringup

Fresh `XDG_CACHE_HOME` (isolated from the machine's real `~/.cache/cppyy_kit`),
timing `rclcpp_kit.bringup_rclcpp()` directly:

```
XDG_CACHE_HOME=<fresh dir> CPPYY_KIT_NO_AUTOPCH=1 pixi run -e rclcpp python -c \
  "import time; t=time.perf_counter(); import rclcpp_kit; \
   rclcpp_kit.bringup_rclcpp(); print('bringup %.3fs' % (time.perf_counter()-t))"
# then the same command without CPPYY_KIT_NO_AUTOPCH=1, same fresh dir, run twice
# (first run schedules the background build at exit; wait for the .pch to appear;
# second run loads it)
```

| run | header parse | bringup total |
|---|--:|--:|
| cold (auto-PCH disabled) | 1.7 s | 1.726 s |
| first run (empty fresh cache) | 1.7 s | 1.735 s |
| **warm run (PCH loaded)** | **~0.0 s** | **0.064 s** |

This run measured a ~27x cold/warm ratio. Header parsing took ~0.0 s in the warm run.
The values are close to the historical ~1.9 s and ~0.06 s results in
`docs/FREEZE.md` §8.

## Not measured

CUDA-accelerated vision (the
`vision-cuda` / `cudabuild` environments, ~5.3x on the RTX PRO 2000 per
`cv_kit/CUDA_OPENCV.md`) was not measured in this pass. It needs its own
environment provisioning, and the base `vision` env's
`cv::cuda` auto-detect reports "absent" on this machine's default
OpenCV build.

## Reproduce this page

Every command above is copy-pasteable as shown. `bench-perceive` / `bench-retarget`
need the synthetic recording step first; `bench-ik` needs the two one-time vendored
builds; `bench-vision` needs the one-time `build-dbow2`; the jitter cell and the
auto-PCH timing are the only two rows without a dedicated pixi task and use the raw
commands shown.
