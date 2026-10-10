# Evaluation record, 3 October 2026

Each milestone has a working saved agent solution. Numerical checks, actual
ROS playback, real controller_manager execution with mock hardware, and a
headless live-camera check passed. This is an initial feasibility evaluation.
It does not establish repeated one-shot reliability. GUI click interaction and
a full presentation rehearsal remain open.

Status note, 10 October 2026: this is the historical 0.3.0 evaluation. Its
agent timings, measurements, source hashes and transcripts retain their
original meaning. The [integration report](../EXPERIMENT_INTEGRATION_2026-10-04.md)
records later current-source checks, packaged task/per-kit guides, environment
diagnostics and local 0.4.0 Conda artifacts. Channel publication and repeated
fresh-agent presentation trials remain separate work.

## Method and environment

The runner launches fresh ephemeral Codex sessions with model `gpt-6-luna` and
`model_reasoning_effort="high"`. It ignores user Codex configuration and supplies
one task prompt. Agents can run checks and correct their own implementations.
One-shot here means one user request without human edits to generated logic.
Failures and environment fixes are reported separately.

The baseline prompt contains no cppyy instructions. Acceleration prompts
explicitly supply a guide. Tests are supplied before editing and hashed before
and after the session. A parent process runs them independently. Saved source
hashes link each [evaluated source](solutions/evaluated_2026_10_03) to its originating run. Parent numerical
checks also use the generated Python baseline as an oracle.

See [provenance.json](evaluation/provenance.json) for the checkout commit, CPU,
source hashes, and UTC date. The host is Linux x86-64 with an Intel Core Ultra
9 285H. The evaluated packages included cppyy-kit 0.3.0, cppyy 3.5.0, NumPy 2.5.3,
MCAP 1.5.0 and mcap-ros2-support 0.5.7. Those sessions used installed packages.
The archived [manifest](solutions/evaluated_2026_10_03/pixi.toml) and
[lock](solutions/evaluated_2026_10_03/pixi.lock) retain that environment.
The current [rehearsal manifest](pixi.toml) activates checkout source and its
[lock](pixi.lock) describes that updated environment. Current validation is
recorded separately in [CURRENT_REHEARSAL.md](CURRENT_REHEARSAL.md).

Numerical agents use a writable-task sandbox with access to their environment's
native cache. ROS execution needs local DDS communication. The successful
controller evaluation therefore uses an unrestricted execution sandbox. It is
still prompted to edit only its task directory. No agent skills, user settings,
worktrees, or additional checkouts are installed or created.

## Actual agent attempts

Each run directory retains the exact prompt, answer, acceptance output, command
list, report, and compressed event transcript. Local uncompressed working runs
remain under ignored `eval_runs/`. Token counts in reports are cumulative usage
from the CLI, not the number of distinct words in the prompt.

| Run | Agent seconds | Commands / failed | Independent acceptance | Interpretation |
|---|---:|---:|---|---|
| [01_implement_a](evaluation/01_implement_a/report.json) | 41.6 | 6 / 0 | 12 passed | Baseline completed |
| [01_accelerate_a](evaluation/01_accelerate_a/report.json) | 170.7 | 20 / 7 | Process crashed | Environment failure |
| [01_accelerate_b](evaluation/01_accelerate_b/report.json) | 229.4 | 23 / 9 | Process crashed | Environment failure |
| [01_accelerate_c](evaluation/01_accelerate_c/report.json) | 79.3 | 6 / 0 | 12 passed | Acceleration completed |
| [01_accelerate_d](evaluation/01_accelerate_d/report.json) | 111.2 | 6 / 0 | 12 passed | Fresh repeat completed |
| [02_mcap_a](evaluation/02_mcap_a/report.json) | 121.7 | 10 / 0 | 5 passed | Native query completed |
| [03_ros_a](evaluation/03_ros_a/report.json) | 162.6 | 12 / 0 | 2 passed | Node computation completed; playback checked separately |
| [04_control_a](evaluation/04_control_a/report.json) | 151.2 | 12 / 6 | 1 passed outside sandbox | Agent could not verify ROS initialization |
| [04_control_b](evaluation/04_control_b/report.json) | 88.5 | 6 / 0 | 1 passed | Controller completed and verified |
| [05_webcam_a](evaluation/05_webcam_a/report.json) | 220.3 | 12 / 2 | 4 passed | Tracker completed after two failed commands |

All original acceptance files retained their hashes. No human repaired the
generated logic. The harness and environment were repaired between trials.
Therefore the first acceleration and controller attempts must remain visible
when discussing reliability. Two successful acceleration trials after correction
are not evidence for every task or a statistically estimated success rate.

The first acceleration attempts did not preserve full Pixi activation and
initially prevented standard cppyy PCH cache writes. The harness now captures
Pixi's activation environment in memory, including its CXX compiler, and permits
the environment cache. A cppyy initialization preflight ran before the successful
trials. The controller's sandbox could not initialize ROS; allowing local DDS
communication resolved that failure in a fresh trial. The agent's first
controller answer correctly reported that it had no valid runtime measurements.

## Independent computation results

See [measurements.json](evaluation/measurements.json) and
[verify.py](scripts/verify.py). The same process benchmarks each implementation,
using the median of seven warmed calls. Thirty random windows at three hold
values compare counts exactly and durations with numerical tolerance. Motion
masks match exactly. Real MCAP arrays are also compared.

| Operation | Input | Python median | Native median | Ratio |
|---|---|---:|---:|---:|
| Motion mask | 250,000 Cartesian observations | 29.28 ms | 1.48 ms | 19.8x |
| MCAP threshold sweep | 2,735 observations, 41 thresholds, two hands | 24.75 ms | 1.14 ms | 21.8x |
| Rolling query | 256 observations, 41 thresholds, two hands | 1.63 ms | 0.080 ms | 20.4x |

Loading and decoding the MCAP took 191.17 ms in this saved run. At a 20 ms hold,
the 0.02 m/s threshold gives 89 left-hand and 82 right-hand events. The original
120 ms hold gives zero events for the tested sweep. These are motion-rule
outputs, not labels of grasps or successful task completion.

These first-call measurements use existing kernel artifacts and shared cppyy
initialization. They are not clean-machine compilation timings. In separate
agent measurements, the Python acceleration process's first call took 46.4 ms,
and the webcam kernel build took 545.8 ms. The latter used a fresh kernel cache;
the environment's cppyy startup infrastructure was already available. Report
startup, warmed operation, and agent elapsed time as different quantities.

## ROS playback

[make_replay.py](scripts/make_replay.py) derives standard ROS String messages
from the original pose stream, adding recorded nanoseconds and sequence numbers.
[check_replay.py](scripts/check_replay.py) launches a real rclpy subscriber and
`ros2 bag play` at 4x. Separate baseline and native trials use separate ROS domains.

Both received all 2,735 messages without repeated or reordered sequence numbers.
Their per-window event-count SHA-256 matches the offline kernel sequence.
Floating durations are checked separately by numerical parity tests.

| Callback measurement | Python | Native rewrite |
|---|---:|---:|
| Median | 2.835 ms | 0.497 ms |
| p99 | 6.287 ms | 1.206 ms |

Sources: [baseline replay](evaluation/ros_replay_baseline.json) and
[native replay](evaluation/ros_replay.json). The trials ran concurrently on
separate domains. Shared-host scheduling and other activity affect timings.
These times measure callback execution, not transport latency or complete
end-to-end delay. The node is a monitor; it issues no robot commands.

An initial playback check exposed a missing `std_msgs` Python dependency; it was
added to the Pixi ROS feature. Another harness attempt ran two players on the
same domain and correctly failed the sequence check. The corrected harness
assigns separate domains. Neither harness failure is counted as a successful
agent or middleware trial.

## Controller

The successful agent completed its real controller-manager acceptance test and
a three-second trial. Its reported achieved rate was 996.4 Hz. The independent
saved trial ran while other checks were active:

- 3,000 cycles at a requested 1,000 Hz; achieved 988.75 Hz.
- p99 cycle interval 3.887 ms.
- 203 intervals exceeded 1.5 times the 1 ms target period.
- Maximum tracking error in the second half was 0.0160 rad.

See [control.json](evaluation/control.json) and the retained stdout/stderr.
The hardware is `mock_components/GenericSystem`. Position commands are mirrored
into state. The measured result verifies the framework interface and tracking
law against that behavior. It does not establish physical-robot dynamics or
hard real-time scheduling. The loop includes initialization transients.

## Webcam

The agent passed four supplied synthetic tests and performed additional reference
checks. The parent added eight randomized SSD comparisons with an independent
integer implementation. The synthetic benchmark uses an 11x11 patch and a
17x17 candidate grid; the agent's warmed median was 0.026 ms.

The parent opened actual device 0, read 640x480 frames, and ran 30 tracking steps.
Median search time was 0.200 ms. No camera images were saved. See
[webcam.json](evaluation/webcam.json). This was a headless capture test. Mouse
selection, display responsiveness, drift, and the presentation camera still
need a GUI rehearsal. The synthetic and live measurements have different input
content and cache conditions; do not treat them as an FPS comparison.

## Measurement commands and next gate

These checkout-only commands use the current rehearsal environment and runnable
solutions. Run from this folder:

```bash
pixi run check
pixi run check-acceptance
pixi run fetch-data
pixi run python scripts/verify.py
pixi run python scripts/make_replay.py
pixi run -e ros python scripts/check_replay.py --baseline
pixi run -e ros python scripts/check_replay.py
pixi run -e control python solutions/control_native.py --rate 1000 --seconds 3
pixi run -e vision python solutions/webcam_native.py --headless --frames 30
```

The original default check had six tests. The current check keeps the historical
source-hash assertions against the archived sources and checks exact
presentation-prompt consistency. It needs no ROS process, camera, or data
download. Current numeric and playback scripts write new measurements under
ignored `build/current-checkout/` by default. They retain current source hashes
and leave `evaluation/` evidence intact. Unique agent run names preserve earlier
agent attempts. See [CURRENT_REHEARSAL.md](CURRENT_REHEARSAL.md) for the commands
actually exercised after migration and their validation scope.

Before the live talk, repeat each successful prompt three times in fresh sessions
and run one changed-requirement case. Check cold and warm startup, rehearse GUI
interaction, and run the full sequence on the presentation machine. Package-only
guide discovery and a nanoflann unwrapped-library experiment have since been
implemented and checked. The full nanoflann fresh-agent task took 513.4 seconds;
its [evaluation](next_steps/EVALUATION.md) recommends a prepared scaffold for a
short live segment. Current presentation work is recorded in [PLAN.md](PLAN.md).

The evidence format follows the principle of checking agent actions and artifacts
in [OpenAI's skill-evaluation guidance](https://developers.openai.com/blog/eval-skills).
The recorded model and reasoning configuration are explicit; do not extrapolate
these trials to another model or configuration.
