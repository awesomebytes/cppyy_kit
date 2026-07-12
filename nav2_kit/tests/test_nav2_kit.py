#!/usr/bin/env python3
"""Tests for rclcppyy.kits.nav2_kit (Nav2 algorithm cores via cppyy).

Nav2 is an optional dependency (the pixi `nav2` env), absent from the default env.
The whole module therefore auto-skips when the Nav2 headers are not installed, so
the default `pixi run test` is unaffected. Run the real thing with
`pixi run -e nav2 test-nav2`.

All tests share one process; bringup_nav2() is idempotent and each test builds its
own Costmap2D / NavFn, so they stay independent. The pure-core tests use no rclcpp;
the lifecycle-unlock tests (LifecycleNode / Costmap2DROS / Smac 2D / RPP -- M6d) bring
rclcpp up in-process and rely on the ordered teardown (cppyy_kit.register_teardown ->
before rclcpp shutdown) for a clean exit.
"""
import glob
import os

os.environ.setdefault("ROS_DOMAIN_ID", "59")

import numpy as np      # noqa: E402
import pytest           # noqa: E402

_HAVE_NAV2 = bool(glob.glob(os.path.join(os.environ.get("CONDA_PREFIX", ""),
                                         "include", "nav2_costmap_2d")))

pytestmark = pytest.mark.skipif(not _HAVE_NAV2,
                                reason="Nav2 not installed (use the nav2 env)")

if _HAVE_NAV2:
    import nav2_kit


@pytest.fixture(scope="module")
def nav2():
    return nav2_kit.bringup_nav2()


def _world(n=100):
    """A 'two rooms + doorway' occupancy grid (rows=y, cols=x)."""
    g = np.zeros((n, n), dtype=np.uint8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = nav2_kit.LETHAL_OBSTACLE
    g[:, n // 2] = nav2_kit.LETHAL_OBSTACLE
    g[n // 2 - 6:n // 2 + 6, n // 2] = nav2_kit.FREE_SPACE
    return g


def test_bringup_idempotent_and_namespaces(nav2):
    costmap_ns, navfn_ns = nav2
    assert costmap_ns is nav2_kit.bringup_nav2()[0]          # idempotent
    assert hasattr(costmap_ns, "Costmap2D")
    assert hasattr(navfn_ns, "NavFn")


def test_cost_constants(nav2):
    assert nav2_kit.LETHAL_OBSTACLE == 254
    assert nav2_kit.FREE_SPACE == 0
    assert nav2_kit.NO_INFORMATION == 255


def test_costmap_from_numpy_roundtrip(nav2):
    grid = _world(64)
    grid[10, 20] = nav2_kit.LETHAL_OBSTACLE                  # a distinctive cell
    cm = nav2_kit.costmap_from_numpy(grid, resolution=0.05)
    assert int(cm.getSizeInCellsX()) == 64
    assert int(cm.getSizeInCellsY()) == 64
    assert cm.getResolution() == pytest.approx(0.05)
    back = nav2_kit.costmap_to_numpy(cm)
    assert back.shape == (64, 64)
    assert np.array_equal(back, grid)                        # exact roundtrip
    # cppyy returns `unsigned char` as a 1-char str; getCost(mx=20,my=10) == grid[10,20]
    assert ord(cm.getCost(20, 10)) == nav2_kit.LETHAL_OBSTACLE


def test_costmap_rejects_non_2d(nav2):
    with pytest.raises(ValueError):
        nav2_kit.costmap_from_numpy(np.zeros((4, 4, 4), dtype=np.uint8))


def test_plan_navfn_finds_path_through_doorway(nav2):
    grid = _world(100)
    cm = nav2_kit.costmap_from_numpy(grid)
    start, goal = (20, 50), (80, 50)
    path = nav2_kit.plan_navfn(cm, start, goal)
    assert path is not None
    assert path.dtype == np.float32 and path.shape[1] == 2
    # endpoints match the request (start..goal order)
    assert path[0][0] == pytest.approx(start[0], abs=1.0)
    assert path[-1][0] == pytest.approx(goal[0], abs=1.0)
    # no waypoint sits on a lethal cell, and the path crosses the divider (x=50)
    # only within the doorway rows (44..56)
    crossed_ys = []
    for x, y in path:
        mx, my = int(round(x)), int(round(y))
        assert grid[my, mx] != nav2_kit.LETHAL_OBSTACLE
        if abs(x - 50) < 1.0:
            crossed_ys.append(y)
    assert crossed_ys, "path never crossed the divider"
    assert all(44 <= y <= 56 for y in crossed_ys)            # through the doorway


def test_plan_navfn_no_path_returns_none(nav2):
    # goal sealed inside a solid lethal box -> unreachable
    grid = np.zeros((50, 50), dtype=np.uint8)
    grid[20:30, 20:30] = nav2_kit.LETHAL_OBSTACLE
    cm = nav2_kit.costmap_from_numpy(grid)
    assert nav2_kit.plan_navfn(cm, (5, 5), (25, 25)) is None


def test_plan_navfn_open_grid_is_short(nav2):
    """On an empty grid the plan is a near-straight line start->goal."""
    grid = np.zeros((80, 80), dtype=np.uint8)
    cm = nav2_kit.costmap_from_numpy(grid)
    path = nav2_kit.plan_navfn(cm, (10, 10), (70, 70))
    assert path is not None
    assert path[0][0] == pytest.approx(10, abs=1.5)
    assert path[-1][0] == pytest.approx(70, abs=1.5)


def test_warmup_callable(nav2):
    nav2_kit.warmup()               # front-loads first-use JIT; must not raise


# ---------------------------------------------------------------------------
# Lifecycle unlock (M6d): LifecycleNode / Costmap2DROS / Smac 2D / RPP.
# These bring rclcpp up in-process; kept after the pure-core tests so the plain
# cores are exercised first. Each uses local objects (dropped at function exit).
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def lc_node(nav2):
    # One active LifecycleNode reused by the lifecycle tests (clock/logger source).
    return nav2_kit.lifecycle_node("test_nav2_kit_lc")


def test_lifecycle_node_transitions(lc_node):
    # Constructed a real rclcpp_lifecycle::LifecycleNode and drove it to ACTIVE.
    assert str(lc_node.get_current_state().label()) == "active"
    assert lc_node.get_clock().now().nanoseconds() > 0
    # a fresh node left UNCONFIGURED when no transitions requested
    n2 = nav2_kit.lifecycle_node("test_nav2_kit_lc2", transitions=())
    assert str(n2.get_current_state().label()) == "unconfigured"
    n2.configure()
    assert str(n2.get_current_state().label()) == "inactive"


def test_costmap_ros_configure_and_fill(nav2):
    grid = _world(100)
    cm_ros = nav2_kit.costmap_ros("test_nav2_kit_costmap", grid=grid, resolution=0.05)
    master = cm_ros.getCostmap()
    assert int(master.getSizeInCellsX()) == 100
    assert int(master.getSizeInCellsY()) == 100
    back = nav2_kit.costmap_to_numpy(master)
    assert np.array_equal(back, grid)               # numpy fill crossed intact
    assert str(cm_ros.getGlobalFrameID()) == "map"


def test_smac_plan_2d_through_doorway(lc_node):
    grid = _world(100)
    cm = nav2_kit.costmap_from_numpy(grid)
    start, goal = (20, 50), (80, 50)
    path = nav2_kit.smac_plan_2d(cm, start, goal, node=lc_node)
    assert path is not None
    assert path.dtype == np.float32 and path.shape[1] == 2
    # start..goal order (Smac plans goal->start internally; the kit reverses it)
    assert path[0][0] == pytest.approx(start[0], abs=1.5)
    assert path[-1][0] == pytest.approx(goal[0], abs=1.5)
    crossed = []
    for x, y in path:
        mx, my = int(round(x)), int(round(y))
        assert grid[my, mx] != nav2_kit.LETHAL_OBSTACLE      # no lethal waypoint
        if abs(x - 50) < 1.0:
            crossed.append(y)
    assert crossed and all(44 <= y <= 56 for y in crossed)   # through the doorway


def test_smac_plan_2d_no_path_returns_none(lc_node):
    grid = np.zeros((50, 50), dtype=np.uint8)
    grid[20:30, 20:30] = nav2_kit.LETHAL_OBSTACLE            # goal sealed inside a box
    cm = nav2_kit.costmap_from_numpy(grid)
    assert nav2_kit.smac_plan_2d(cm, (5, 5), (25, 25), node=lc_node) is None


def test_smac_accepts_costmap_ros(lc_node):
    # smac_plan_2d also accepts a Costmap2DROS (uses its getCostmap()).
    grid = _world(80)
    cm_ros = nav2_kit.costmap_ros("test_nav2_kit_smac_cmros", grid=grid, resolution=0.05)
    path = nav2_kit.smac_plan_2d(cm_ros, (15, 40), (65, 40), node=lc_node)
    assert path is not None and len(path) > 0


def test_rpp_controller_straight_and_offset(lc_node):
    cm_ros = nav2_kit.costmap_ros("test_nav2_kit_rpp_costmap",
                                  grid=np.zeros((100, 100), dtype=np.uint8),
                                  resolution=0.05)
    rpp = nav2_kit.RPPController(cm_ros, node=lc_node, name="TestFollowPath")
    rpp.set_plan([(1.0 + 0.1 * i, 2.5) for i in range(31)])   # straight line +x
    # aligned on the line -> forward, ~no turn
    v, w = rpp.compute((1.0, 2.5, 0.0))
    assert v > 0.4 and abs(w) < 0.05
    # offset+rotated below the line -> still forward, turns back toward it (w > 0)
    v2, w2 = rpp.compute((2.0, 2.6, -0.2))
    assert v2 > 0.4 and w2 > 0.0
