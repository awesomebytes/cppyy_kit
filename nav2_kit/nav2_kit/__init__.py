"""
nav2_kit -- compose your own Nav stack from Nav2's algorithm cores, in Python, via
cppyy.

Nav2's Python story is client-side only: ``nav2_simple_commander`` sends goals to
the C++ lifecycle servers; every algorithm (planners, controllers, costmap layers)
is a C++ class behind pluginlib. This kit takes the other road -- it drives Nav2's
**algorithm cores directly** from Python, with **no lifecycle servers, no
pluginlib, no tf**: Python owns the loop, C++ owns the math. It mirrors the
libraries' own C++ API against the installed Nav2, JIT-including the headers; there
is no binding to generate.

Two Nav2 cores are cleanly separable and surfaced here (see docs/nav2_kit/REPORT.md
for the full probe matrix, including what is *not*):

  * ``nav2_costmap_2d::Costmap2D`` -- a PLAIN grid class (no node): construct it,
    set costs, read the underlying ``unsigned char`` charmap. The kit adds a
    single-``memcpy`` NumPy<->charmap bridge (the per-cell ``setCost`` loop is
    ~600-3600x slower at 512x512/1024x1024 -- the same bulk-data lesson as pcl_kit).
  * ``nav2_navfn_planner::NavFn`` -- the NavFn Dijkstra/A* planner *algorithm*, which
    operates on the costmap char array with no node at all. The kit wraps its real
    friction: ``calcNavFnAstar`` only builds the potential field, so a plan needs a
    following ``calcPath``; start/goal cross as ``int*``; and the path comes back as
    raw ``float*`` X/Y arrays + a length.

Minimal plan on a synthetic world::

    import numpy as np
    import nav2_kit
    costmap_ns, navfn_ns = nav2_kit.bringup_nav2()

    grid = np.zeros((200, 200), dtype=np.uint8)      # 0 = free
    grid[80:120, 100] = nav2_kit.LETHAL_OBSTACLE     # a wall
    costmap = nav2_kit.costmap_from_numpy(grid, resolution=0.05)
    path = nav2_kit.plan_navfn(costmap, start=(20, 100), goal=(180, 100))  # (N,2) cells

Coordinate convention (kept consistent end to end): a NumPy grid is ``(H, W)`` =
``(rows=y, cols=x)``, indexed ``grid[y, x]``; a costmap cell is ``(mx=x, my=y)``;
NavFn path coordinates are ``(x, y)`` in cells. This is the same row-major layout as
a ``nav_msgs/OccupancyGrid`` (``data[y*W + x]``), so a plan lines up with a published
grid with no flip.

Notes / limits (v0):
    * This is the *algorithm-core* road, not a Nav2 stack: no lifecycle nodes, no
      pluginlib, no tf, no dynamic obstacle/inflation layers, no recovery behaviors.
    * NavFn's ``setCostmap(..., isROS=True)`` rescales ROS cost values (0-254) into
      NavFn's internal band and adds an obstacle border, exactly as the Nav2 server
      does; costs are the ``nav2_costmap_2d`` values (``nav2_kit.LETHAL_OBSTACLE`` ...).
    * Smac (Hybrid-A*) is NOT surfaced: its ``a_star.hpp`` transitively needs OMPL
      headers (absent from the nav2 env) and its collision checker is lifecycle-
      coupled -- see the REPORT. RegulatedPurePursuit's controller plugin is likewise
      lifecycle-coupled; only its header-only regulation math is separable.
    * cppyy returns ``unsigned char`` as a 1-char Python ``str`` -- read a single cell
      with ``ord(costmap.getCost(mx, my))``; the bulk ``costmap_to_numpy`` path avoids
      this.
"""
import os

import cppyy

import cppyy_kit

# The two .so whose symbols the cores resolve against. NavFn's setCost/getCharMap
# are undefined in libnav2_navfn_planner.so (they live in the costmap core), so both
# must be loaded; cppyy finds a symbol's owning library by scanning its search path
# at call time (add_library_path alone does not resolve symbols -- see cppyy_kit).
_NAV2_LIBS = (
    "libnav2_costmap_2d_core.so",
    "libnav2_navfn_planner.so",
)

# Headers: the cost-value constants, the plain Costmap2D grid class, and the NavFn
# planner algorithm. costmap_2d.hpp transitively pulls geometry_msgs / nav_msgs
# message headers, so the ROS include paths must be on the path first (bringup does
# this via rclcppyy's add_ros2_include_paths).
_NAV2_HEADERS = (
    "nav2_costmap_2d/cost_values.hpp",
    "nav2_costmap_2d/costmap_2d.hpp",
    "nav2_navfn_planner/navfn.hpp",
)

# C++ glue compiled once at bringup. The bulk-data copies and the NavFn call
# sequence live here on the C++ side (a per-cell Python loop is ~600-3600x slower,
# and building buffers from Python risks a cppyy SIGSEGV -- see cppyy_kit / pcl_kit).
# Addresses cross as uintptr_t and are reinterpret_cast'd in C++.
_CPP_GLUE = r"""
#include <cstring>
#include <cstdint>
namespace rclcppyy_nav2kit {
using nav2_costmap_2d::Costmap2D;
using nav2_navfn_planner::NavFn;

// numpy(uint8, row-major) -> costmap charmap: one std::memcpy (charmap is a plain
// uint8 buffer of size_x*size_y, same row-major layout as the numpy grid).
inline void load_charmap(Costmap2D& cm, uintptr_t src, std::size_t n) {
  std::memcpy(cm.getCharMap(), reinterpret_cast<const void*>(src), n);
}
// costmap charmap -> numpy(uint8): one std::memcpy out.
inline void dump_charmap(const Costmap2D& cm, uintptr_t dst, std::size_t n) {
  std::memcpy(reinterpret_cast<void*>(dst), cm.getCharMap(), n);
}

// NavFn plan, start..goal order. calcNavFnAstar only propagates the potential
// field; the path must then be traced with calcPath (this is the real NavFn
// friction). setStart/setGoal take int[2]. cancel checker is a C++ no-op. Returns
// the path length in points (0 if no plan). Path is left in NavFn's getPathX/Y.
// NOTE: setNavArr/setCostmap are done by the caller *before* this (setNavArr resets
// the cost array, so it must precede setCostmap).
inline int navfn_plan(NavFn& nav, int sx, int sy, int gx, int gy, bool allow_unknown) {
  int start[2] = {sx, sy};
  int goal[2] = {gx, gy};
  nav.setStart(start);
  nav.setGoal(goal);
  if (!nav.calcNavFnAstar([](){return false;})) return 0;
  return nav.calcPath(nav.nx * nav.ny / 2);
}
// NavFn path (subpixel cell coords) -> caller numpy float32 (N,2), start..goal.
inline int copy_path(NavFn& nav, uintptr_t xy_dst) {
  int n = nav.getPathLen();
  float* px = nav.getPathX();
  float* py = nav.getPathY();
  float* d = reinterpret_cast<float*>(xy_dst);
  for (int i = 0; i < n; ++i) { d[2 * i] = px[i]; d[2 * i + 1] = py[i]; }
  return n;
}
}  // namespace rclcppyy_nav2kit
"""

# Cost-value constants (nav2_costmap_2d), exposed as plain ints. cppyy would present
# these unsigned-char constants as 1-char strings; a plain int is what a user wants.
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
MAX_NON_OBSTACLE = 252
NO_INFORMATION = 255
FREE_SPACE = 0

_COSTMAP_NS = None
_NAVFN_NS = None
_DONE = False


def _ensure(with_ros_paths=True):
    """Bring up the Nav2 cores (headers + .so set + NumPy/NavFn glue). Idempotent."""
    global _COSTMAP_NS, _NAVFN_NS, _DONE
    if _DONE:
        return
    conda = os.environ["CONDA_PREFIX"]
    # costmap_2d.hpp needs the ROS message headers; add_ros2_include_paths registers
    # every ament package's include dir (cheap -- it adds paths, it does not JIT
    # rclcpp). $CONDA_PREFIX/include covers the nav2_navfn_planner/navfn.hpp
    # cross-reference layout.
    if with_ros_paths:
        from rclcpp_kit.bringup_rclcpp import add_ros2_include_paths
        add_ros2_include_paths()
    cppyy.add_include_path(os.path.join(conda, "include"))
    for header in _NAV2_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_NAV2_LIBS, [os.path.join(conda, "lib")])
    cppyy.cppdef(_CPP_GLUE)
    _COSTMAP_NS = cppyy.gbl.nav2_costmap_2d
    _NAVFN_NS = cppyy.gbl.nav2_navfn_planner
    _DONE = True


def bringup_nav2():
    """
    Bring up Nav2's Costmap2D + NavFn cores under cppyy and return the
    ``(nav2_costmap_2d, nav2_navfn_planner)`` namespaces. Idempotent.

    Adds the ROS message + Nav2 include paths, JIT-includes the cost-value / costmap
    / navfn headers, loads ``libnav2_costmap_2d_core.so`` + ``libnav2_navfn_planner.so``
    so calls resolve without ``LD_LIBRARY_PATH``, and compiles the NumPy/NavFn C++
    glue. Use the libraries' own API on the returned namespaces directly
    (``nav2_costmap_2d.Costmap2D``, ``nav2_navfn_planner.NavFn``).
    """
    _ensure()
    return _COSTMAP_NS, _NAVFN_NS


def _glue():
    _ensure()
    return cppyy.gbl.rclcppyy_nav2kit


def costmap_from_numpy(grid, resolution=0.05, origin=(0.0, 0.0), default_value=0):
    """
    Build a ``nav2_costmap_2d::Costmap2D`` from an ``(H, W)`` ``uint8`` occupancy
    grid (rows = y, cols = x), loading it with a single ``std::memcpy``.

    ``grid`` values are ``nav2_costmap_2d`` costs (``nav2_kit.FREE_SPACE`` ...
    ``LETHAL_OBSTACLE`` / ``NO_INFORMATION``); any dtype is coerced to contiguous
    ``uint8``. ``resolution`` is meters/cell; ``origin`` is the ``(x, y)`` world
    position of the grid's lower-left corner. The copy is a single memcpy because the
    charmap is a plain ``size_x*size_y`` ``uint8`` buffer with the same row-major
    layout as the array -- a per-cell ``setCost`` loop is ~600-3600x slower (REPORT).
    Returns the costmap.
    """
    import numpy as np
    arr = np.ascontiguousarray(grid, dtype=np.uint8)
    if arr.ndim != 2:
        raise ValueError(f"expected an (H, W) grid, got shape {arr.shape}")
    h, w = arr.shape
    costmap_ns, _ = bringup_nav2()
    ox, oy = origin
    # First touch of the Costmap2D ctor + the glue call JIT-compiles cppyy wrappers;
    # the notice points at nav2_kit.warmup().
    with cppyy_kit.first_use("nav2_kit.costmap_from_numpy", "nav2_kit.warmup()"):
        costmap = costmap_ns.Costmap2D(w, h, float(resolution), float(ox), float(oy),
                                       int(default_value))
        _glue().load_charmap(costmap, arr.ctypes.data, arr.size)
    return costmap


def costmap_to_numpy(costmap):
    """
    Extract a ``Costmap2D`` charmap to an ``(H, W)`` ``uint8`` NumPy array (rows = y),
    with a single ``std::memcpy`` out. Returns a private copy (safe after the costmap
    is gone).
    """
    import numpy as np
    w = int(costmap.getSizeInCellsX())
    h = int(costmap.getSizeInCellsY())
    out = np.empty((h, w), dtype=np.uint8)
    _glue().dump_charmap(costmap, out.ctypes.data, out.size)
    return out


def plan_navfn(costmap, start, goal, allow_unknown=True):
    """
    Plan a path with NavFn over ``costmap`` from ``start`` to ``goal`` (both
    ``(mx, my)`` cell coordinates), returning the path as an ``(N, 2)`` ``float32``
    NumPy array of ``(x, y)`` cell coordinates (start..goal order), or ``None`` if no
    plan is found.

    Wraps NavFn's real friction: it builds a ``NavFn`` sized to the costmap, feeds it
    the costmap char array with ``setCostmap(..., isROS=True, allow_unknown)``, runs
    ``calcNavFnAstar`` to propagate the potential field, then ``calcPath`` to trace
    the path (``calcNavFnAstar`` alone does *not* populate a path), and copies the raw
    ``float*`` X/Y arrays out. Coordinates are subpixel cells (~1/2-cell spacing);
    convert to world with ``costmap.mapToWorld`` or ``origin + (c+0.5)*resolution``.
    """
    import numpy as np
    _, navfn_ns = bringup_nav2()
    w = int(costmap.getSizeInCellsX())
    h = int(costmap.getSizeInCellsY())
    nav = navfn_ns.NavFn(w, h)
    # setNavArr (re)sizes and resets the cost array, so it must run BEFORE
    # setCostmap fills it (calling it after would wipe the costmap).
    nav.setNavArr(w, h)
    nav.setCostmap(costmap.getCharMap(), True, bool(allow_unknown))
    sx, sy = int(start[0]), int(start[1])
    gx, gy = int(goal[0]), int(goal[1])
    with cppyy_kit.first_use("nav2_kit.plan_navfn", "nav2_kit.warmup()"):
        n = int(_glue().navfn_plan(nav, sx, sy, gx, gy, bool(allow_unknown)))
    if n <= 0:
        return None
    path = np.empty((n, 2), dtype=np.float32)
    _glue().copy_path(nav, path.ctypes.data)
    # Pin the NavFn on the returned array so it can't be collected mid-extraction
    # (defensive; copy_path already ran, but keeps the pattern explicit).
    cppyy_kit.keep_alive(path, nav)
    return path


def warmup():
    """Front-load nav2_kit's one-time first-use JIT during init.

    The first ``costmap_from_numpy`` and first ``plan_navfn`` JIT-compile cppyy call
    wrappers for the Costmap2D ctor, the memcpy glue, and the NavFn plan/extract glue
    (a freeze/PCH does not remove this). This runs one throwaway build+plan on a tiny
    grid so the wrappers are cached process-globally before your first real call.
    """
    import numpy as np

    def _exercise():
        grid = np.zeros((16, 16), dtype=np.uint8)
        cm = costmap_from_numpy(grid, resolution=0.1)
        costmap_to_numpy(cm)
        plan_navfn(cm, (2, 2), (13, 13))

    cppyy_kit.warmup(_exercise)


# ============================================================================
# Lifecycle unlock (M6d): construct a real rclcpp_lifecycle::LifecycleNode
# in-process from Python, and with it the two Nav2 cores the algorithm-core road
# left BLOCKED -- Smac 2D (AStarAlgorithm<Node2D>) and the RegulatedPurePursuit
# controller. Both couplings the REPORT documented dissolve once you can build a
# LifecycleNode from Python (the same move control_kit made for ControllerManager):
#
#   * Smac's GridCollisionChecker ctor wants (Costmap2DROS, unsigned, LifecycleNode).
#     A NULL Costmap2DROS + a real LifecycleNode + the base FootprintCollisionChecker
#     ``setCostmap(Costmap2D*)`` gives it our plain costmap without the ROS wrapper.
#   * RPP's configure() wants (LifecycleNode::WeakPtr, name, tf2_ros::Buffer,
#     Costmap2DROS). We build a real (plugin-free) Costmap2DROS + a tf2_ros::Buffer
#     fed one map->base_link transform, and a tiny C++ GoalChecker stub (RPP's body
#     dereferences goal_checker->getTolerances() despite the header commenting the
#     parameter name -- a null crashes).
#
# a_star.hpp transitively pulls node_hybrid.hpp -> OMPL + Eigen headers; the nav2 env
# ships them (``include/ompl-1.7`` from nav2-smac-planner's deps, ``include/eigen3``),
# so with those two dirs on the include path the header JIT-parses.
#
# These require rclcpp to be initialized. Unlike the pure cores above (no rclcpp), the
# lifecycle bringups init rclcpp; the base ``bringup_nav2()`` stays pure so the plain
# cores and their tests are unaffected. Bringing up Smac/RPP is opt-in and lazy.
# ============================================================================

_LIFECYCLE_LIBS = ("librclcpp_lifecycle.so",)
# Smac 2D (Node2D) needs no OMPL at runtime (a_star.hpp only *parses* node_hybrid.hpp's
# OMPL includes). Hybrid-A* (NodeHybrid) DOES enter OMPL and is not surfaced here -- its
# precomputeDistanceHeuristic (OMPL Dubins/Reeds-Shepp distance table) segfaults
# non-deterministically under Cling; see the REPORT for the flaky-partial evidence.
_SMAC_LIBS = ("libnav2_smac_planner.so",)
_RPP_LIBS = ("libtf2_ros.so", "libnav2_regulated_pure_pursuit_controller.so")

_SMAC_GLUE = r"""
#include <memory>
#include <vector>
#include <functional>
namespace rclcppyy_nav2lc_smac {
using nav2_smac_planner::AStarAlgorithm;
using nav2_smac_planner::Node2D;
using nav2_smac_planner::GridCollisionChecker;
using nav2_smac_planner::MotionModel;
using nav2_smac_planner::SearchInfo;

// GridCollisionChecker with a NULL Costmap2DROS + a real LifecycleNode (for clock/
// logger), then the plain Costmap2D set directly through the base class -- bypasses
// the lifecycle-coupled Costmap2DROS entirely for the 2D (index) collision path.
GridCollisionChecker* make_checker_2d(nav2_costmap_2d::Costmap2D* cm,
                                      rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> null_ros;
  auto* c = new GridCollisionChecker(null_ros, 1, node);
  c->setCostmap(cm);
  return c;
}
void free_checker(GridCollisionChecker* c) { delete c; }

struct SmacPlan2D {
  std::vector<Node2D::Coordinates> path;   // Smac's native order is goal..start
  int iterations = 0;
  bool ok = false;
};

// AStarAlgorithm<Node2D>. createPath takes several non-const int& references and a
// std::function cancel checker -- awkward from Python, so the whole sequence lives
// here (mirrors the NavFn glue). Returns the plan struct by value (cppyy keeps it).
SmacPlan2D smac_plan_2d(GridCollisionChecker* checker,
                        float sx, float sy, float gx, float gy,
                        bool allow_unknown, int max_iterations,
                        float tolerance, double max_planning_time) {
  SmacPlan2D r;
  SearchInfo info;                                  // defaults are fine for 2D
  AStarAlgorithm<Node2D> a_star(MotionModel::TWOD, info);
  int max_on_approach = 1000;
  int terminal_interval = 5000;
  a_star.initialize(allow_unknown, max_iterations, max_on_approach,
                    terminal_interval, max_planning_time, 0.0f, 1u);
  a_star.setCollisionChecker(checker);
  a_star.setStart(sx, sy, 0u);
  a_star.setGoal(gx, gy, 0u);
  int num_iterations = 0;
  r.ok = a_star.createPath(r.path, num_iterations, tolerance, [](){ return false; });
  r.iterations = num_iterations;
  return r;
}
// SmacPlan2D path (goal..start) -> caller numpy float32 (N,2), REVERSED to start..goal.
void copy_path_start_to_goal(const SmacPlan2D& r, uintptr_t xy_dst) {
  float* d = reinterpret_cast<float*>(xy_dst);
  std::size_t n = r.path.size();
  for (std::size_t i = 0; i < n; ++i) {
    d[2 * i] = r.path[n - 1 - i].x;
    d[2 * i + 1] = r.path[n - 1 - i].y;
  }
}
}  // namespace rclcppyy_nav2lc_smac
"""

_RPP_GLUE = r"""
#include <cmath>
#include <memory>
#include <string>
namespace rclcppyy_nav2lc_rpp {
using RPP = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;

// tf2_ros::Buffer's ctor is templated on the node type (the "overload soup" the
// rclcpp_kit.tf module avoids); build it in C++ with the defaults.
std::shared_ptr<tf2_ros::Buffer> make_buffer(rclcpp::Clock::SharedPtr clock) {
  return std::make_shared<tf2_ros::Buffer>(clock);
}
// A single map->base_link transform (the robot's pose in the global frame), set
// static so the lookup resolves at any stamp -- enough for RPP's plan transform.
void set_robot_tf(std::shared_ptr<tf2_ros::Buffer> buf,
                  const std::string& global_frame, const std::string& base_frame,
                  double x, double y, double yaw) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = global_frame;
  t.child_frame_id = base_frame;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.rotation.z = std::sin(yaw / 2.0);
  t.transform.rotation.w = std::cos(yaw / 2.0);
  buf->setTransform(t, "nav2_kit", true);
}

std::shared_ptr<RPP> make_rpp() { return std::make_shared<RPP>(); }

// configure() takes a WeakPtr; build it from the shared node here.
void configure_rpp(std::shared_ptr<RPP> rpp,
                   std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                   const std::string& name,
                   std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> wp = node;
  rpp->configure(wp, name, tf, costmap_ros);
}

// RPP.computeVelocityCommands dereferences goal_checker->getTolerances() (the header
// comments the parameter name, but the body uses it), so a null crashes. A stub that
// reports a fixed XY tolerance is all RPP needs to run standalone.
struct GoalCheckerStub : public nav2_core::GoalChecker {
  double xy_tol_;
  explicit GoalCheckerStub(double xy_tol) : xy_tol_(xy_tol) {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr&, const std::string&,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose&, const geometry_msgs::msg::Pose&,
                     const geometry_msgs::msg::Twist&) override { return false; }
  bool getTolerances(geometry_msgs::msg::Pose& pose_tol,
                     geometry_msgs::msg::Twist&) override {
    pose_tol.position.x = xy_tol_;
    pose_tol.position.y = xy_tol_;
    return true;
  }
};
std::shared_ptr<nav2_core::GoalChecker> make_goal_checker(double xy_tol) {
  return std::make_shared<GoalCheckerStub>(xy_tol);
}

geometry_msgs::msg::TwistStamped compute(std::shared_ptr<RPP> rpp,
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& vel,
    std::shared_ptr<nav2_core::GoalChecker> gc) {
  return rpp->computeVelocityCommands(pose, vel, gc.get());
}
}  // namespace rclcppyy_nav2lc_rpp
"""

_LC_BASE_DONE = False
_SMAC_DONE = False
_RPP_DONE = False
_SMAC_NS = None
_RPP_NS = None
_LC_TRACKED = []            # lifecycle objects to drop before rclcpp shutdown (Pattern 14)
_LC_TEARDOWN_REGISTERED = False


def _rclcpp():
    """Bring rclcpp up and ensure it is initialized (idempotent)."""
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    return rclcpp


def _register_lc_teardown():
    # Pattern 14: a LifecycleNode / Costmap2DROS owns DDS entities (and a bond timer);
    # its destructor must run while the rclcpp context is still valid. rclcpp_kit
    # registers shutdown_rclcpp() first, so it runs LAST (LIFO); dropping our tracked
    # objects here runs BEFORE it. Registered once, best-effort.
    global _LC_TEARDOWN_REGISTERED
    if _LC_TEARDOWN_REGISTERED:
        return
    _LC_TEARDOWN_REGISTERED = True

    def _drop():
        while _LC_TRACKED:
            _LC_TRACKED.pop()          # reverse creation order; drops the kit's ref
    cppyy_kit.register_teardown(_drop)


def _ensure_lifecycle_base():
    """Bring up the LifecycleNode + Costmap2DROS surface (rclcpp init + headers +
    librclcpp_lifecycle + the costmap core). Idempotent."""
    global _LC_BASE_DONE
    if _LC_BASE_DONE:
        return
    bringup_nav2()                     # ROS include paths + costmap/navfn cores + glue
    _rclcpp()
    conda = os.environ["CONDA_PREFIX"]
    cppyy.add_include_path(os.path.join(conda, "include", "eigen3"))
    cppyy.add_library_path(os.path.join(conda, "lib"))
    cppyy.include("rclcpp_lifecycle/lifecycle_node.hpp")
    cppyy.include("nav2_costmap_2d/costmap_2d_ros.hpp")
    cppyy_kit.load_libraries(_LIFECYCLE_LIBS, [os.path.join(conda, "lib")])
    _register_lc_teardown()
    _LC_BASE_DONE = True


def _ensure_smac():
    """Bring up Smac 2D (OMPL/Eigen include paths + a_star/node_2d/collision_checker
    headers + libnav2_smac_planner + the Smac glue). Idempotent."""
    global _SMAC_DONE, _SMAC_NS
    if _SMAC_DONE:
        return
    _ensure_lifecycle_base()
    conda = os.environ["CONDA_PREFIX"]
    cppyy.add_include_path(os.path.join(conda, "include", "ompl-1.7"))
    cppyy.include("nav2_smac_planner/a_star.hpp")
    cppyy.include("nav2_smac_planner/node_2d.hpp")
    cppyy.include("nav2_smac_planner/collision_checker.hpp")
    cppyy_kit.load_libraries(_SMAC_LIBS, [os.path.join(conda, "lib")])
    cppyy.cppdef(_SMAC_GLUE)
    _SMAC_NS = cppyy.gbl.rclcppyy_nav2lc_smac
    _SMAC_DONE = True


def _ensure_rpp():
    """Bring up the RegulatedPurePursuit controller (RPP + goal_checker + tf2_ros
    buffer headers + libtf2_ros + the RPP controller .so + the RPP glue). Idempotent."""
    global _RPP_DONE, _RPP_NS
    if _RPP_DONE:
        return
    _ensure_lifecycle_base()
    conda = os.environ["CONDA_PREFIX"]
    cppyy.include("tf2_ros/buffer.h")
    cppyy.include("nav2_regulated_pure_pursuit_controller/"
                  "regulated_pure_pursuit_controller.hpp")
    cppyy.include("nav2_core/goal_checker.hpp")
    cppyy.include("geometry_msgs/msg/pose_stamped.hpp")
    cppyy.include("geometry_msgs/msg/twist.hpp")
    cppyy.include("nav_msgs/msg/path.hpp")
    cppyy_kit.load_libraries(_RPP_LIBS, [os.path.join(conda, "lib")])
    cppyy.cppdef(_RPP_GLUE)
    _RPP_NS = cppyy.gbl.rclcppyy_nav2lc_rpp
    _RPP_DONE = True


def _parameter_value(value):
    """A Python scalar / homogeneous list -> ``rclcpp::ParameterValue`` (bool / int /
    float / str and lists thereof) -- the shapes Nav2 node parameters take."""
    pv = cppyy.gbl.rclcpp.ParameterValue
    std = cppyy.gbl.std
    if isinstance(value, bool):
        return pv(value)
    if isinstance(value, int):
        return pv(int(value))
    if isinstance(value, float):
        return pv(float(value))
    if isinstance(value, str):
        return pv(std.string(value))
    if isinstance(value, (list, tuple)):
        if all(isinstance(v, str) for v in value):
            return pv(std.vector["std::string"]([std.string(v) for v in value]))
        if all(isinstance(v, bool) for v in value):
            return pv(std.vector["bool"](list(value)))
        if all(isinstance(v, int) for v in value):
            return pv(std.vector["int64_t"](list(value)))
        return pv(std.vector["double"]([float(v) for v in value]))
    raise cppyy_kit.CppyyKitError("nav2_kit: unsupported parameter value %r" % (value,))


def _parameter_overrides(parameters):
    """A ``{name: value}`` dict -> ``std::vector<rclcpp::Parameter>`` for NodeOptions."""
    std = cppyy.gbl.std
    Param = cppyy.gbl.rclcpp.Parameter
    vec = std.vector["rclcpp::Parameter"]()
    for name, value in (parameters or {}).items():
        vec.push_back(Param(std.string(name), _parameter_value(value)))
    return vec


def lifecycle_node(name, parameters=None, transitions=("configure", "activate"),
                   namespace=""):
    """
    Construct a real ``rclcpp_lifecycle::LifecycleNode`` in this process and run the
    requested lifecycle ``transitions`` on it (default: configure -> activate). This
    is the key that fits every lifecycle-coupled Nav2 ctor -- it is a plain(ish) class
    like ``rclcpp::Node``, built from Python via ``make_shared`` with a
    ``NodeOptions`` carrying ``parameters`` as overrides.

    ``parameters`` is a ``{name: value}`` dict (scalars / homogeneous lists);
    ``transitions`` is a subset of ``("configure", "activate")`` (pass ``()`` to leave
    it UNCONFIGURED). Requires rclcpp (brought up + initialized here). The node is
    tracked for ordered teardown before rclcpp shutdown (Pattern 14). Returns the
    ``LifecycleNode`` shared_ptr -- call any of its methods (``get_clock``,
    ``get_logger``, ``configure``/``activate``/``deactivate``, ...) directly.
    """
    _ensure_lifecycle_base()
    std = cppyy.gbl.std
    opts = cppyy.gbl.rclcpp.NodeOptions()
    # LifecycleNode declares nothing itself, so auto-declare makes the overrides visible
    # as real parameters (unlike Costmap2DROS, which declares its own -- see costmap_ros).
    if parameters:
        opts.automatically_declare_parameters_from_overrides(True)
        opts.parameter_overrides(_parameter_overrides(parameters))
    with cppyy_kit.first_use("nav2_kit.lifecycle_node", "nav2_kit.warmup_lifecycle()"):
        node = std.make_shared["rclcpp_lifecycle::LifecycleNode"](
            std.string(name), std.string(namespace), opts)
    for t in transitions:
        if t == "configure":
            node.configure()
        elif t == "activate":
            node.activate()
        else:
            raise cppyy_kit.CppyyKitError("nav2_kit: unknown transition %r" % (t,))
    _LC_TRACKED.append(node)
    return node


def costmap_ros(name="costmap", grid=None, resolution=0.05, width_m=None, height_m=None,
                origin=(0.0, 0.0), robot_radius=0.1, rolling_window=False,
                global_frame="map", robot_base_frame="base_link",
                track_unknown_space=False, parameters=None, configure=True):
    """
    Construct a real, plugin-free ``nav2_costmap_2d::Costmap2DROS`` in this process and
    (by default) ``configure`` it -- a LifecycleNode subclass whose ``getCostmap()``
    yields a fillable master ``Costmap2D``. This is what Smac's collision checker and
    RPP need; here it carries **no layers** (``plugins: []``), so ``configure`` builds a
    blank grid you own and fill from NumPy (no static map, no tf, no sensor pipeline).

    If ``grid`` (an ``(H, W)`` uint8 cost array) is given, ``width_m``/``height_m``/
    ``resolution`` default so the master costmap matches its shape, and the grid is
    memcpy'd in after configure. Extra ``parameters`` (a dict) are added as node
    parameter overrides. Do **not** call ``activate`` unless you want the map-update
    thread (which, with no plugins, would otherwise leave your fill intact but is
    unnecessary). Requires rclcpp. Tracked for ordered teardown. Returns the
    ``Costmap2DROS`` shared_ptr.
    """
    import numpy as np
    _ensure_lifecycle_base()
    std = cppyy.gbl.std
    arr = None
    if grid is not None:
        arr = np.ascontiguousarray(grid, dtype=np.uint8)
        if arr.ndim != 2:
            raise ValueError(f"expected an (H, W) grid, got shape {arr.shape}")
        h, w = arr.shape
        if width_m is None:
            width_m = w * resolution
        if height_m is None:
            height_m = h * resolution
    if width_m is None or height_m is None:
        raise ValueError("give a grid, or both width_m and height_m")
    params = {
        "use_sim_time": False,
        "global_frame": global_frame,
        "robot_base_frame": robot_base_frame,
        "rolling_window": bool(rolling_window),
        "width": int(round(width_m)),
        "height": int(round(height_m)),
        "resolution": float(resolution),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "robot_radius": float(robot_radius),
        "track_unknown_space": bool(track_unknown_space),
        "plugins": [],           # NO layers -> a blank, fillable master costmap
        "filters": [],
    }
    params.update(parameters or {})
    opts = cppyy.gbl.rclcpp.NodeOptions()
    # Costmap2DROS declares its own parameters, so auto-declare would double-declare
    # (ParameterAlreadyDeclaredException) -- overrides alone are honored by its declares.
    opts.parameter_overrides(_parameter_overrides(params))
    with cppyy_kit.first_use("nav2_kit.costmap_ros", "nav2_kit.warmup_lifecycle()"):
        cm_ros = std.make_shared["nav2_costmap_2d::Costmap2DROS"](opts)
        if configure:
            cm_ros.configure()
    if arr is not None and configure:
        master = cm_ros.getCostmap()
        _glue().load_charmap(master, arr.ctypes.data, arr.size)
    _LC_TRACKED.append(cm_ros)
    return cm_ros


def smac_plan_2d(costmap, start, goal, node=None, allow_unknown=True,
                 tolerance=0.5, max_iterations=1000000, max_planning_time=5.0):
    """
    Plan a path with Nav2's **Smac 2D** planner (``AStarAlgorithm<Node2D>``) over
    ``costmap`` (a plain ``Costmap2D`` from :func:`costmap_from_numpy`, or a
    ``Costmap2DROS`` -- its ``getCostmap()`` is used) from ``start`` to ``goal`` (both
    ``(mx, my)`` cell coordinates). Returns an ``(N, 2)`` ``float32`` NumPy array of
    ``(x, y)`` cell coordinates in **start..goal** order (Smac plans goal->start
    internally; this reverses it to match :func:`plan_navfn`), or ``None`` if no plan.

    Builds a ``GridCollisionChecker`` from a NULL ``Costmap2DROS`` + the given (or a
    freshly built) ``LifecycleNode``, sets the plain costmap on it, then runs
    ``createPath``. ``node`` is any ``LifecycleNode`` (for clock/logger); one is created
    if omitted. Requires rclcpp.
    """
    import numpy as np
    _ensure_smac()
    if hasattr(costmap, "getCostmap"):     # a Costmap2DROS
        costmap = costmap.getCostmap()
    if node is None:
        node = lifecycle_node("nav2_kit_smac", transitions=())
    checker = _SMAC_NS.make_checker_2d(costmap, node)
    try:
        with cppyy_kit.first_use("nav2_kit.smac_plan_2d", "nav2_kit.warmup_lifecycle()"):
            plan = _SMAC_NS.smac_plan_2d(
                checker, float(start[0]), float(start[1]), float(goal[0]),
                float(goal[1]), bool(allow_unknown), int(max_iterations),
                float(tolerance), float(max_planning_time))
        n = int(plan.path.size())
        if not plan.ok or n == 0:
            return None
        path = np.empty((n, 2), dtype=np.float32)
        _SMAC_NS.copy_path_start_to_goal(plan, path.ctypes.data)
        return path
    finally:
        _SMAC_NS.free_checker(checker)


class RPPController:
    """
    Nav2's real ``RegulatedPurePursuitController`` (the C++ controller plugin), driven
    from Python. Construct it with a configured :func:`costmap_ros` and a parent
    :func:`lifecycle_node`; the wrapper hides RPP's cppyy frictions -- the ``WeakPtr``
    parent, the templated ``tf2_ros::Buffer`` ctor, the ``GoalChecker`` its body
    dereferences, and the ROS message building for the plan / pose.

    ``rpp.set_plan(path_xy)`` takes world ``(x, y)`` waypoints (or a C++
    ``nav_msgs/Path``); ``rpp.compute(pose)`` takes ``(x, y, theta)`` in the global
    frame and returns ``(v, w)`` -- one regulated-pure-pursuit step against the real
    controller. Parameters (``desired_linear_vel``, ``lookahead_dist``, ...) can be
    overridden on the parent node via ``parameters`` (declared under ``name.``).
    """

    def __init__(self, costmap_ros, node=None, name="FollowPath", parameters=None,
                 tf_buffer=None, goal_xy_tolerance=0.25):
        _ensure_rpp()
        std = cppyy.gbl.std
        self._ns = _RPP_NS
        self.name = name
        self.global_frame = str(costmap_ros.getGlobalFrameID())
        self.base_frame = str(costmap_ros.getBaseFrameID())
        if node is None:
            node = lifecycle_node("nav2_kit_rpp_parent", transitions=())
        self.node = node
        # RPP declares its params under "<name>." on the parent node during configure()
        # (declare_parameter_if_not_declared), so pre-declare our overrides on the node
        # -- whether it was created here or passed in -- so RPP reads OUR values.
        self._declare_params(node, name, parameters)
        self.costmap_ros = costmap_ros
        self.tf = tf_buffer or self._ns.make_buffer(node.get_clock())
        self.gc = self._ns.make_goal_checker(float(goal_xy_tolerance))
        self.rpp = self._ns.make_rpp()
        with cppyy_kit.first_use("nav2_kit.RPPController", "nav2_kit.warmup_lifecycle()"):
            self._ns.configure_rpp(self.rpp, node, std.string(name), self.tf, costmap_ros)
        self.rpp.activate()
        _LC_TRACKED.append(self)

    @staticmethod
    def _declare_params(node, name, parameters):
        std = cppyy.gbl.std
        Param = cppyy.gbl.rclcpp.Parameter
        for key, value in (parameters or {}).items():
            pname = std.string(f"{name}.{key}")
            pv = _parameter_value(value)
            try:
                node.declare_parameter(pname, pv)
            except Exception:                        # already declared -> set instead
                node.set_parameter(Param(pname, pv))

    def set_plan(self, path, frame=None):
        """Set the global plan. ``path`` is either a C++ ``nav_msgs/Path`` or an
        iterable of world ``(x, y)`` waypoints (built into a Path in ``frame`` /
        the costmap's global frame)."""
        if hasattr(path, "poses"):          # already a nav_msgs/Path
            self.rpp.setPlan(path)
            return
        msg = cppyy.gbl.nav_msgs.msg.Path()
        msg.header.frame_id = frame or self.global_frame
        for wx, wy in path:
            ps = cppyy.gbl.geometry_msgs.msg.PoseStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            msg.poses.push_back(ps)
        self.rpp.setPlan(msg)

    def compute(self, pose, velocity=(0.0, 0.0)):
        """One control step. ``pose`` is ``(x, y, theta)`` in the global frame;
        ``velocity`` is the current ``(v, w)``. Publishes the pose as the tf
        ``global_frame -> base_frame`` transform (RPP transforms the plan into the
        robot frame), then returns the commanded ``(v, w)``."""
        import math
        std = cppyy.gbl.std
        x, y, th = pose
        self._ns.set_robot_tf(self.tf, std.string(self.global_frame),
                              std.string(self.base_frame), float(x), float(y), float(th))
        ps = cppyy.gbl.geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(float(th) / 2.0)
        ps.pose.orientation.w = math.cos(float(th) / 2.0)
        vel = cppyy.gbl.geometry_msgs.msg.Twist()
        vel.linear.x = float(velocity[0])
        vel.angular.z = float(velocity[1])
        cmd = self._ns.compute(self.rpp, ps, vel, self.gc)
        return float(cmd.twist.linear.x), float(cmd.twist.angular.z)


def warmup_lifecycle():
    """Front-load the lifecycle-unlock first-use JIT (LifecycleNode + Costmap2DROS
    ctors, the Smac plan glue, the RPP configure/compute glue) on throwaway objects so
    the first live call does not stall. Requires rclcpp. Best-effort per available
    feature (Smac / RPP)."""
    import numpy as np
    _ensure_lifecycle_base()

    def _thunk():
        g = np.zeros((16, 16), dtype=np.uint8)
        cm = costmap_from_numpy(g, resolution=0.1)
        node = lifecycle_node("nav2_kit_warmup", transitions=())
        smac_plan_2d(cm, (2, 2), (13, 13), node=node)
        cmr = costmap_ros("nav2_kit_warmup_costmap", grid=g, resolution=0.1)
        rpp = RPPController(cmr, node=node)
        rpp.set_plan([(0.2, 0.2), (0.5, 0.2), (0.8, 0.2)])
        rpp.compute((0.2, 0.2, 0.0))

    cppyy_kit.warmup(_thunk)
