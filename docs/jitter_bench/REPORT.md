# jitter_bench: timing jitter in a Python-driven 1 kHz control loop

**Date:** 2026-07-12. **Environment:** pixi `control` (robostack Jazzy and conda-forge),
cppyy 3.5.0, Python 3.12, linux-64. `ROS_DOMAIN_ID=63`. **Machine:** 16 cores,
Ubuntu 24.04, kernel **6.17.0-1028-oem**, `CONFIG_PREEMPT_DYNAMIC=y`, `CONFIG_HZ=1000`.

## Experiment

The benchmark measures loop timing on the stock Ubuntu kernel. It compares four loop
variants during idle periods and CPU load. This is Stage 0, with unprivileged settings.
Stage 1 requires owner actions with sudo, GRUB changes, or a new login; its commands
are documented at the end of this report and were not run.

`CONFIG_PREEMPT_RT` is not enabled in this image. It is not required to test soft
real-time behavior. The stock `PREEMPT_DYNAMIC` kernel provides the scheduling and
timer features used by a 1 kHz loop. PREEMPT_RT can provide tighter worst-case latency
under load.

**Main result:** Setting timer slack to 1 ns with the unprivileged call
`prctl(PR_SET_TIMERSLACK, 1)` reduced median wakeup latency from about 52 µs to 2.4 µs.
With that setting, `mlockall`, and CPU pinning, the Python loop held 1000 Hz with a
median near 2 µs and about 0.7% late cycles while idle. Under load, the C++ loop
(cppyy_kit variant b) retained its median while the Python loop medians rose to about
5 µs. The worst delays under load still depend on scheduling settings.

---

## 0. Method and measurement limits

**Metric: wakeup latency.** Each loop programs absolute wake deadlines
`base + i·period` and records the actual wake time on the same clock used to program the
deadline (`CLOCK_MONOTONIC`). On this system, CPython's `perf_counter` uses that clock,
as checked with `time.get_clock_info`. Wakeup latency is `wake[i] − deadline[i]`, in µs.
This is the metric used by `cyclictest`, so a Stage 1 run can be compared with variant a1.
The report also gives period jitter, the interval between wakes minus the target period.
control_kit REPORT section 4 uses that metric.

**Loop variants:**

| id | loop body | sleep mechanism | where the loop runs |
|----|-----------|-----------------|----------------------|
| **a1** | pure-Python timer + tiny compute | `clock_nanosleep(TIMER_ABSTIME)` | Python |
| **a2** | pure-Python timer + tiny compute | deadline-corrected `time.sleep` | Python |
| **b**  | C++ wait+compute loop | `clock_nanosleep(TIMER_ABSTIME)` in C++ | **C++** (cppyy_kit `nogil`+`cppdef_cached`) |
| **c**  | `ros2_control` `read→update→write`, Python PD controller | `clock_nanosleep` (harness) | Python drives, CM in C++ |

Each variant ran for **60 s at 1 kHz** (60,000 cycles), both idle and under load. The first
100 cycles were excluded to allow for JIT compilation, cache warmup, and scheduler settling.
Each result row states this. The per-cycle control law is a fixed polynomial operation,
matched in Python and C++. It takes much less than the 1 ms period, so the measurements
reflect scheduling time rather than compute throughput.

**Measurement limits:**

1. Other workloads and ROS background threads ran during measurement. `ROS_DOMAIN_ID=63`
   isolates DDS discovery, not CPU use. The idle condition means no benchmark-generated
   load. A quieter machine may have a narrower latency tail.
2. The kernel has `CONFIG_PREEMPT_DYNAMIC=y`, `CONFIG_HZ=1000`, and `NO_HZ_FULL=y`.
   `CONFIG_PREEMPT_RT` is not enabled. The live preemption mode was not read because that
   requires sudo, so it is recorded as unknown. The command line has no `preempt=` override.
3. Stage 0 used unprivileged settings. `mlockall` and `prctl(PR_SET_TIMERSLACK, 1)`
   succeeded. `SCHED_FIFO` needs an rtprio grant; `ulimit -r` is 0 here. The harness tries
   to set it and records the denial. Stage 1 includes instructions for granting rtprio.
   `nice` also needs privilege here and was left unchanged.
4. Every percentile is over the whole 60 s run
   (minus the stated 100-cycle warmup). Outliers are reported, not trimmed.

**Settings used in this run:**
`prctl(PR_SET_TIMERSLACK, 1)`; `mlockall(MCL_CURRENT|MCL_FUTURE)`
(the process fits under the ~8 GB memlock ulimit, with no privilege needed); CPU affinity
pinned the benchmark to **CPU 2**; the load condition pinned **8 busy-loop processes to
CPUs 8–15**. The measurement core was not oversubscribed. `nice` was unchanged and
`SCHED_FIFO` was denied because both need a privilege grant.

---

## 1a. Timer slack setting

Linux's default **timer slack is 50 µs**. The kernel may defer a `clock_nanosleep`,
`futex` / `poll` wakeup by up to that much to batch wakeups and save power. At 1 kHz (1000 µs
period) that 50 µs *is* the median wakeup latency. Setting slack to 1 ns
(`prctl(PR_SET_TIMERSLACK, 1)`, no privilege) removes the batching. Variant a1 was measured
for 8 s per setting while idle on the same machine:

| timer slack | p50 (µs) | mean (µs) | p99 (µs) | p99.9 (µs) | max (µs) |
|---|--:|--:|--:|--:|--:|
| **50 µs (OS default)** | **52.4** | 86.9 | 668.7 | 2196.8 | 3932.8 |
| **1 ns (tuned, `PR_SET_TIMERSLACK`)** | **2.4** | 22.0 | 413.6 | 1767.8 | 3135.5 |

Setting timer slack to 1 ns reduced median wakeup latency by about 22 times. The removed
52 µs came from timer slack, not Python overhead. The harness now sets slack to
1 ns by default (`--timerslack-ns`, default 1); every number in §1 below is with slack tuned.
The p99.9 and maximum changed little. Those delays depend on preemption settings, which
are covered in Stage 1.

---

## 1. Results: reference matrix (60 s per cell, 1 kHz, timer slack = 1 ns)

### Wakeup latency (µs)

60 000 cycles/cell, first 100 dropped; SCHED_OTHER, timer slack 1 ns, mlockall on, bench
pinned to cpu 2; **load** = 8 busy-loop processes pinned to cpus 8–15 (bench core not
oversubscribed). All held 1000.0 Hz mean.

| variant | cond | min | mean | p50 | p99 | p99.9 | max | late % |
|---|---|--:|--:|--:|--:|--:|--:|--:|
| **a1** pure-Python `clock_nanosleep` | idle | 1.3 | 29.2 | **2.4** | 460.9 | 2108.4 | 3138.4 | 0.65 |
| | load | 1.7 | 45.5 | 5.3 | 1137.0 | 2994.0 | 9306.7 | 2.05 |
| **a2** pure-Python `time.sleep` | idle | 1.4 | 35.9 | **2.5** | 657.6 | 2426.0 | 8358.4 | 0.76 |
| | load | 1.4 | 45.5 | 5.7 | 1166.7 | 2647.3 | 3482.1 | 1.92 |
| **b** cppyy_kit C++ loop (nogil+cached) | idle | 1.1 | 30.3 | **2.0** | 461.8 | 2102.9 | 3174.3 | 0.67 |
| | load | 1.2 | 33.6 | **2.1** | 959.1 | 2426.4 | 6781.1 | 1.57 |
| **c** `ros2_control` loop (Python ctrl) | idle | 1.2 | 18.5 | **2.3** | 500.9 | 2257.0 | 3454.7 | 0.70 |
| | load | 1.5 | 41.5 | 5.0 | 1052.4 | 2758.5 | 9805.2 | 1.75 |

The a1 load row is from a repeated measurement. The first attempt caught a 1.3 s
machine-wide stall. Its mean was 14.5 ms and its maximum was 1.30 s, including catch-up
after missed absolute deadlines. Both runs are in `build/jitter_bench.json` and
`build/jitter_a1_load_rerun.json`.

**Summary:** Idle median latency was about 2 µs for all variants. Under load, C++ loop b
kept a median near 2 µs while Python loop medians rose to about 5 µs. Variant b also had
the lowest p99 under load.

### Results

- All four variants had idle median latency between **2.0 and 2.5 µs**. Mean rate was
  **1000.0 Hz**, with **0.65–0.76% late cycles**. Reducing timer slack accounts for the
  low median latency.
- Under load, variant b had **2.1 µs p50** and **959 µs p99**. The Python variants had
  p50 values of 5.3 µs (a1), 5.7 µs (a2), and 5.0 µs (c). Their p99 values were 1137 µs,
  1167 µs, and 1052 µs.
- Variant c ran the `ros2_control` `read→update→write` loop with a Python PD
  controller. Its idle latency was **2.3 µs p50** and **501 µs p99**, with an **18.5 µs**
  mean. The first `update()` call has a roughly 25 ms JIT cost. The harness warmed it
  with 200 updates and excluded the first 100 timer cycles from the statistics.
- The idle maxima for a1 (`clock_nanosleep`) and a2 (`time.sleep`) were 3.1 ms and
  8.4 ms. Their idle medians were close.
- Idle p99.9 was about **2.1–2.4 ms**, with maxima of **3–8 ms**. These measurements ran
  on a shared machine and included CFS preemption on a non-isolated core. The first a1
  load attempt caught a single **1.3 s** machine-wide stall. Its mean was 14.5 ms, and
  its maximum was 1.30 s due to the stall and about 1300 cycles of absolute-deadline
  catch-up. That result was repeated; the table uses the repeated measurement.

### Per-cell latency histograms

**a1 (pure-Python, clock_nanosleep), idle.** 81 % of cycles within 5 µs; the tail is a
few % in the 100 µs–2 ms range.
```
  bucket(us)       count      %  histogram
  1-2               9812  16.4%  ##########
  2-5              38785  64.7%  ########################################
  5-10              4251   7.1%  ####
  10-20             1316   2.2%  #
  20-50              737   1.2%  #
  50-100            1244   2.1%  #
  100-200           1007   1.7%  #
  200-500           2208   3.7%  ##
  500-1000           264   0.4%
  1000-2000          211   0.4%
  2000-5000           65   0.1%
  total            59900 100.0%
```

**b (cppyy_kit C++ loop, nogil+cached), under load.** 91.6 % of cycles were within
5 µs and 42 % were within 2 µs, with 8 busy cores running in the background.
The nogil C++ loop never re-enters the interpreter between wake and next sleep, so the
scheduler sees one long-running C++ thread rather than a Python thread cycling the
interpreter, load perturbs it less.
```
  bucket(us)       count      %  histogram
  1-2              25271  42.2%  ##################################
  2-5              29567  49.4%  ########################################
  5-10               838   1.4%  #
  10-20              497   0.8%  #
  20-50              614   1.0%  #
  50-100             447   0.7%  #
  100-200            507   0.8%  #
  200-500            925   1.5%  #
  500-1000           674   1.1%  #
  1000-2000          422   0.7%  #
  2000-5000          136   0.2%
  >=5000               2   0.0%
  total            59900 100.0%
```
(Every cell's histogram is in `build/jitter_bench.json` via `--json`.)

---

## 2. Python control loop results

With timer slack reduced, `mlockall`, and CPU pinning, all three loop types measured
about 2 µs median latency and 1000 Hz while idle: a Python timer loop, a cppyy C++ loop,
and a `ros2_control` loop. Each had fewer than 1% late cycles. Under load, the C++
loop kept a median near 2 µs while the Python loop medians increased to about 5 µs.

Tail latency remained high on the shared machine. Idle p99.9 was about 2 ms, with rare
larger delays under load. Stage 0 did not use SCHED_FIFO or change the preemption mode.
Stage 1 documents tests with SCHED_FIFO, `preempt=full`, CPU isolation, and an optional
low-latency kernel. PREEMPT_RT may be needed for bounded worst-case latency under
adversarial load. These measurements do not establish such a bound.

This benchmark extends the 1 kHz measurements in control_kit REPORT section 4. It adds
60 s latency histograms, a comparison of `clock_nanosleep`, `time.sleep`, and the C++
loop, and a run under CPU load. `clock_nanosleep` had a lower idle maximum than
`time.sleep`; the C++ loop had lower median latency under load. For hard real-time
deployment, the control law still needs to run in a native pluginlib controller, as
described in control_kit REPORT section 4.

Variant b uses `nogil` and `cppdef_cached`. This keeps interpreter and GIL work out of
the timed loop and avoids first-use call-wrapper JIT in later runs. The benchmark does
not show that these changes remove worst-case delays under contention. Those depend on
scheduling class and preemption settings.

---

## 3. Limitations

1. The machine was shared during measurement. A quiet machine may have lower tail latency.
2. Stage 0 did not use SCHED_FIFO or change the preemption mode. The report does not
   measure whether Stage 1 settings reduce the tail.
3. There is no `cyclictest` reference yet. Variant a1 uses the same clock,
   `clock_nanosleep`, and `mlockall`, but a direct comparison remains for Stage 1.
4. Variant c uses `mock_components/GenericSystem`. A hardware `SystemInterface` would add
   bus I/O latency, which was not measured.
5. Only 1 kHz was measured. The harness accepts `--rate`, but 100 Hz, 2 kHz, and 5 kHz
   runs are not included here.

---

## 4. Reproduce / re-run

```bash
# The full reference matrix (what produced §1), 60 s/cell, timerslack+mlockall+cpu-pinned:
ROS_DOMAIN_ID=63 pixi run -e control bench-jitter          # -> build/jitter_bench.json

# The §1a timer-slack A/B (default vs tuned), fast:
pixi run -e control python jitter_bench/run_bench.py --variant a1 --condition idle \
    --duration 8 --timerslack-ns -1 --mlock --cpu 2 --no-hist    # default 50us
pixi run -e control python jitter_bench/run_bench.py --variant a1 --condition idle \
    --duration 8 --timerslack-ns 1  --mlock --cpu 2 --no-hist    # tuned 1ns

# Fast smoke (no ros2_control; a1/a2/b, 2 s, idle):
pixi run -e control bench-jitter-smoke

# One cell, one command (the Stage-1 rerun shape):
pixi run -e control bench-jitter-cell                      # a1 / idle / 60 s

# Tests (variant c runs in the control env; skips in the default env):
pixi run -e control test-jitter        # or: pixi run test  (jitter_bench/tests included)
```

Any single cell is one command: `python jitter_bench/run_bench.py --variant <id>
--condition <idle|load> --duration <s> --rate <hz> --timerslack-ns <ns> --mlock --cpu <n>
[--sched fifo] [--preempt-label <mode>]`.

---

## Stage 1: owner actions to complete the tuning matrix (documented, not run)

Stage 0 above is the unprivileged reference. The rest of progressive tuning needs
privileges this lane does not have (sudo / GRUB / re-login). **These commands are for the
machine owner** (fact-checked against `/boot/config-6.17.0-1028-oem`); none were run here. No
new kernel is required for any of them, the stock kernel already has the primitives.

### Available features in the stock kernel
Verified present in this kernel's config: `SCHED_FIFO`/`RR`/`DEADLINE`, `CONFIG_FUTEX_PI`,
`CONFIG_HIGH_RES_TIMERS`, `CONFIG_SCHED_HRTICK`, `CONFIG_IRQ_FORCED_THREADING` (so
`threadirqs` works), `CONFIG_NO_HZ_FULL`, `CONFIG_RCU_NOCB_CPU`, `CONFIG_PREEMPT_RCU`,
`CONFIG_RT_MUTEXES`, `isolcpus`, runtime preemption-mode switching (`PREEMPT_DYNAMIC`). The
only thing `CONFIG_PREEMPT_RT` would add on top is bounded worst-case latency under
adversarial load, not a capability gap for a 1 kHz soft-RT loop.

### 1. Grant rtprio + memlock so `SCHED_FIFO` becomes available
```bash
sudo bash -c 'printf "sapf - rtprio 98\nsapf - memlock unlimited\n" > /etc/security/limits.d/99-realtime.conf'
# log out and back in (or reboot) for the limits to take effect; verify:  ulimit -r   # -> 98
```
After this, `--sched fifo` in the harness stops reporting "DENIED" and runs a real
`SCHED_FIFO` loop (the code path is already present; only the grant is missing).

### 2. Switch the runtime preemption mode (PREEMPT_DYNAMIC, live, no reboot)
```bash
cat /sys/kernel/debug/sched/preempt                      # read current (needs sudo)
echo none      | sudo tee /sys/kernel/debug/sched/preempt
echo voluntary | sudo tee /sys/kernel/debug/sched/preempt
echo full      | sudo tee /sys/kernel/debug/sched/preempt
```

### 3. GRUB cmdline on the SAME kernel (reboot, no new kernel), isolate the control core
```bash
# add to GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub, then: sudo update-grub && reboot
preempt=full threadirqs isolcpus=2 nohz_full=2 rcu_nocbs=2
# isolcpus/nohz_full/rcu_nocbs on the measurement core (cpu 2 here) remove the timer tick,
# RCU callbacks and other CPUs' schedulable load from it; threadirqs makes IRQ handlers
# preemptible; preempt=full sets the most aggressive preemption at boot.
```

### 4. Reference baseline + optional free middle step
```bash
sudo apt-get install -y rt-tests
# canonical wakeup-latency reference to sit beside variant a1 (same clock/mechanism):
cyclictest --mlockall --priority=80 --interval=1000 --distance=0 --duration=60 --histogram=2000
# OPTIONAL, no Ubuntu Pro: a low-latency HWE kernel (more aggressive preemption defaults,
# still NOT PREEMPT_RT) from the standard noble-updates archive:
sudo apt-get install -y linux-lowlatency-hwe-24.04     # 6.17.0-35; select at boot
```

### 5. Stage 1 rerun matrix: preemption mode × scheduler
For each preemption mode (step 2 or 3) and each scheduling class, re-run **one command per
cell**, the harness records the preempt mode you pass as a label and applies the scheduler:
```bash
echo full | sudo tee /sys/kernel/debug/sched/preempt
ROS_DOMAIN_ID=63 python jitter_bench/run_bench.py \
    --variant c --condition load --duration 60 --rate 1000 \
    --timerslack-ns 1 --mlock --cpu 2 --sched fifo --prio 80 --preempt-label full \
    --json build/jitter_c_full_fifo_load.json
```
Sweep `--preempt-label {none,voluntary,full}` × `--sched {other,fifo}` × `--variant {a1,b,c}`;
each writes its own JSON, and the summary table + histogram print per run.

**Expected result.** This report has no Stage 1 measurements. At 1 kHz on this class of
kernel, tuned typical and p99 latency may be in the tens of microseconds. GPU or storage
contention can still cause long delays. Stage 1 would measure whether `SCHED_FIFO`,
`preempt=full`, and CPU isolation reduce those delays. A `CONFIG_PREEMPT_RT` kernel may
reduce latency further. This report did not measure those settings. The Stage 0 load
results show latency before those changes.

**Possible follow-up:** the same harness on an embedded board (Raspberry Pi), copy
`jitter_bench/`, `pip install numpy`, run variant a1/b (no ROS needed); variant c needs a ROS
install on the board.
