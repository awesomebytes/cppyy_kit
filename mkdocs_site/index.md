# cppyy_kit

**Drive C++ robotics libraries from Python via [cppyy](https://cppyy.readthedocs.io) —
prototype at Python speed, run at C++ speed, graduate to AOT.**

cppyy_kit is a suite of *kits*: thin Python mirror-APIs over C++ libraries, plus
the friction primitives and the freeze/compile-cache tooling that make them fast
and first-class for both humans and LLM agents.

> Prototype normally (plain Python) → switch to the kits and it gets automatically
> more efficient → write unit/integration tests (the contract) → apply AOT
> (freeze / lower / cache) → show the benchmark difference — **while the code stays
> the same or changes minimally.**

## The measured story

Every claim here is measured; each kit's `REPORT.md` carries the evidence.

| Lever | Result |
|---|---|
| **Freeze** — Cling PCH of a library's headers | header parse **890 ms → 6 ms** (~140×), same tests green frozen |
| **Compile cache** — content-hash `cppdef` → `.so`, `dlopen` thereafter | eliminates the ~**0.69 s** first-use wrapper JIT **persistently** (once per machine, ever) |
| **PCL showcase** — cloud stays in C++ end to end | **14.8× / 9.4×** vs the plain rclpy + NumPy baseline |
| **TF ingest** — C++ `tf2` listener vs Python callback | **6.7–14×** lower ingest CPU |
| **Lower (L2)** — hot leaf emitted as a native C++ node | per-tick cppyy boundary cost removed |

## The optimization ladder

The same code climbs rungs as you need more speed — the kit API does not change:

- **L0 — JIT.** Plain cppyy: headers parsed and per-signature wrappers JIT-compiled
  on first use. Fastest to write.
- **L1 — Freeze.** A prebuilt Cling PCH of the library headers skips the ~890 ms
  header parse (→ ~6 ms). See [Freeze & Cache](docs/FREEZE.md).
- **Compile cache.** The base hashes each `cppdef` and compiles it once to a real
  `.so`, then `dlopen`s it — the ~0.69 s first-use wrapper JIT is paid once per
  machine, not once per process. Composes with freeze.
- **L2 — Lower.** A proven-hot leaf is emitted as a native C++ node (registered
  JIT-free), removing the per-call cppyy boundary entirely.

Read the full ladder in **[Freeze & Cache](docs/FREEZE.md)** and the 22 hard-won
patterns behind it in **[The Patterns](docs/COMMON_PATTERNS.md)**.

## The kits

| Package | What it drives |
|---|---|
| **[cppyy_kit](kits/cppyy_kit.md)** | the ROS-free base: friction primitives + freeze/compile-cache tooling |
| **[rclcpp_kit](rclcpp_kit/WHY.md)** | rclcpp (ROS 2 core): bringup, messages, tf, rosbag2 |
| **[bt_kit](bt_kit/WHY.md)** | BehaviorTree.CPP v4 |
| **[pcl_kit](pcl_kit/WHY.md)** | Point Cloud Library (clouds stay in C++) |
| **[ompl_kit](ompl_kit/WHY.md)** | Open Motion Planning Library |
| **[nav2_kit](nav2_kit/WHY.md)** | Nav2 algorithm cores, composed from Python |
| **[moveit_kit](moveit_kit/WHY.md)** | the full MoveIt 2 C++ API |
| **[control_kit](control_kit/WHY.md)** | a Python ros2_control controller in the real controller_manager |
| **[cv_kit](cv_kit/WHY.md)** | OpenCV C++ (zero-copy `Image` → `cv::Mat`) |
| **[dbow_kit](dbow_kit/WHY.md)** | DBoW2 place recognition / loop closure |

Each kit is a package with a `WHY.md` (the pitch), `REPORT.md` (the evidence), and
`SKILL.md` (the LLM-facing cheat sheet).

## Next steps

- **[Getting Started](getting-started.md)** — install the packages, or develop from the repo.
- **[The Patterns](docs/COMMON_PATTERNS.md)** — the canonical cppyy playbook.
- **[Architecture](docs/ARCHITECTURE_V2.md)** — how the suite is put together.

---

*Origin: extracted and expanded from [rclcppyy](https://github.com/awesomebytes/rclcppyy).
Headline target: ROSCon UK 2026.*
