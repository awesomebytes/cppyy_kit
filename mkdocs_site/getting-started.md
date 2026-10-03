# Getting Started

Two paths: **install** the published conda packages to use a kit in your own
project, or **develop** from the repo to hack on the suite.

## Install (use a kit)

> **Published.** All 11 packages are live on the prefix.dev `awesomebytes` channel
> (browse: <https://repo.prefix.dev/awesomebytes>). The install example below targets
> Linux x86_64; use the *Develop* path to hack on the suite from the repo.

The packages are pure-Python (`noarch`) wrappers; the solver installs their native
dependencies. The published recipes currently target Python 3.12 on Linux x86_64
and ARM64. This manifest targets Linux x86_64; for ARM64, change `platforms` to
`["linux-aarch64"]`. ARM64 also needs the architecture-specific cppyy bridge noted
in [`recipe/cppyy/README.md`](https://github.com/awesomebytes/cppyy_kit/blob/main/recipe/cppyy/README.md).
The `awesomebytes`, `robostack-jazzy`, and `conda-forge` channels provide the packages:

Install [Pixi](https://pixi.sh/latest/installation/) first.

Create and enter a project directory:

```bash
mkdir cppyy-example
cd cppyy-example
```

Save the following as `pixi.toml` in that directory:

```toml
# pixi.toml
[workspace]
name = "cppyy-example"
channels = ["https://prefix.dev/awesomebytes", "robostack-jazzy", "conda-forge"]
platforms = ["linux-64"]

[dependencies]
cppyy-kit = "*"              # ROS-free base (cppyy only)
ros-jazzy-bt-kit = "*"       # a kit — pulls cppyy-kit + behaviortree-cpp
# ros-jazzy-rclcpp-kit, -pcl-kit, -ompl-kit, -nav2-kit, -moveit-kit,
# -control-kit, -cv-kit, -dbow-kit  — install only what you need
```

Create `example.py` in the same directory with this complete BehaviorTree.CPP
example. `AlwaysSuccess` is a built-in node, so no custom node registration is
needed:

```python
import bt_kit
bt = bt_kit.bringup_bt()
xml = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree"><AlwaysSuccess/></BehaviorTree>
</root>
"""
factory = bt.BehaviorTreeFactory()
tree = factory.create_tree_from_text(xml)
status = tree.tickWhileRunning()
print(status == bt.NodeStatus.SUCCESS)  # True
```

Install the packages and run the example from that directory:

```bash
pixi install
pixi run python example.py
```

On first use, cppyy may take longer while it prepares its compilation cache; later
runs can reuse the cache.

Install only what you need — every kit pulls `cppyy-kit`, and the ROS-touching
kits pull `ros-jazzy-rclcpp-kit`, transitively.

## Develop (hack on the suite)

Requires [pixi](https://pixi.sh). Clone and use the workspace envs — the default
env is the ROS/cppyy stack; each kit's C++ dependency is an additive feature env.

```bash
git clone https://github.com/awesomebytes/cppyy_kit
cd cppyy_kit

# lint + the default (auto-skipping) test suite — the CI gate
pixi run lint
pixi run test

# a kit: its demo + test suite run in the kit's feature env
pixi run -e bt   demo-bt-t01     # BehaviorTree.CPP first tree, in short Python
pixi run -e bt   test-bt         # bt_kit + base cppyy_kit tests
pixi run -e ompl demo-ompl-plan  # OMPL 2D plan
pixi run -e nav2 test-nav2       # Nav2 cores from Python
```

In the repo, kits resolve via `PYTHONPATH` (set in `pixi.toml` `[activation.env]`):
the repo root plus each kit dir. ROS-touching kits get the ROS 2 core through the
local `rclcpp_kit` package plus the default `ros-base` env.

## Build the docs

```bash
pixi run -e docs docs-serve    # live preview at http://127.0.0.1:8000
pixi run -e docs docs-build    # strict build into ./site
```

## Where next

- **[The Patterns](docs/COMMON_PATTERNS.md)** — the canonical cppyy playbook (36 patterns).
- **[Freeze & Cache](docs/FREEZE.md)** — the L0→L1→L2 + compile-cache ladder.
- **[Tutorials](docs/tutorials/vision_loop_closure.md)** — end-to-end walkthroughs.
- Per kit: its **Why** (the pitch), **Report** (the evidence), **Skill** (LLM cheat sheet).
