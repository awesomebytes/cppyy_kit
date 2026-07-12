"""
The IK solver registry for the benchmark.

Every solver is described by one ``Solver`` record so the harness (``run_bench.py``)
can treat them uniformly: run each in its own subprocess against the same target set
and collect the same metrics. Three families:

  * ``moveit`` -- a MoveIt ``kinematics::KinematicsBase`` plugin loaded IN-PROCESS via
    ``moveit_kit``'s pluginlib recipe (REPORT.md 2.2 / COMMON_PATTERNS 19). This is
    the whole point: KDL and trac_ik are packaged; **bio_ik and pick_ik are C++-only
    and unpackaged**, vendored-built (ik_bench/vendor/) and discovered by the SAME
    pluginlib-by-lookup-name path -- Cling never parses their headers.
  * ``python`` -- the pure-NumPy DLS baseline (ik_bench/panda.py). No C++ at all.

A ``moveit`` solver is *available* iff pluginlib can find its lookup name in the
ament index (packaged plugins are always there; a vendored one appears once its
build has installed the ament marker + plugin xml + .so, and its prefix is on
``AMENT_PREFIX_PATH``). Unavailable solvers show up as an honest ``blocked`` row.
"""
import os

# The vendored-build install prefixes (added to AMENT_PREFIX_PATH so pluginlib's
# ament-index lookup finds bio_ik / pick_ik exactly like a packaged plugin).
VENDOR_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "build", "vendor")


def bio_ik_prefix():
    return os.path.join(VENDOR_ROOT, "bio_ik_install")


def pick_ik_prefix():
    return os.path.join(VENDOR_ROOT, "pick_ik_install")


class Solver:
    def __init__(self, key, label, kind, plugin=None, params=None,
                 prefix=None, note=""):
        self.key = key          # cli id / worker arg
        self.label = label      # table display name
        self.kind = kind        # "moveit" | "python"
        self.plugin = plugin    # pluginlib lookup name (moveit)
        self.params = params or {}  # per-group params (moveit), under robot_description_kinematics.<group>
        self.prefix = prefix    # extra AMENT_PREFIX_PATH entry (vendored plugins)
        self.note = note


GROUP = "panda_arm"
TIP = "panda_link8"

# trac_ik reads its config under robot_description_kinematics.<group> (a
# generate_parameter_library ParamListener; the prefix is fixed in the plugin's
# initialize()). "Speed" = return the first valid solution (the fair, KDL-like
# mode); Distance/Manipulation modes run to timeout to optimise the solution.
_TRAC_IK_PARAMS = {"solve_type": "Speed", "epsilon": 1e-5,
                   "kinematics_solver_timeout": 0.05, "position_only_ik": False}

# pick_ik: gradient-descent + memetic global solver (PickNik). Params (a g_p_l
# ParamListener) live under robot_description_kinematics.<group>. "global" lets the
# initial guess be far from the goal; the thresholds match the benchmark tolerance.
_PICK_IK_PARAMS = {
    "mode": "global",
    "position_threshold": 0.001,
    "orientation_threshold": 0.01,
}

REGISTRY = [
    Solver("kdl", "KDL (packaged)", "moveit",
           plugin="kdl_kinematics_plugin/KDLKinematicsPlugin",
           note="MoveIt's default numeric Jacobian solver (Orocos KDL)."),
    Solver("trac_ik", "TRAC-IK (packaged)", "moveit",
           plugin="trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
           params=_TRAC_IK_PARAMS,
           note="TRAC-IK Speed mode (KDL + SQP, TRACLabs)."),
    Solver("bio_ik", "bio_ik (vendored C++)", "moveit",
           plugin="bio_ik/BioIKKinematicsPlugin",
           params={"mode": "bio2_memetic"},
           prefix=bio_ik_prefix(),
           note="Evolutionary/memetic global IK (PickNik fork). Not packaged -> "
                "vendored-source build."),
    Solver("pick_ik", "pick_ik (vendored C++)", "moveit",
           plugin="pick_ik/PickIkPlugin",
           params=_PICK_IK_PARAMS,
           prefix=pick_ik_prefix(),
           note="Gradient-descent + memetic global IK (PickNik). Not packaged -> "
                "vendored-source build; generate_parameter_library-heavy."),
    Solver("pure_python", "pure-Python DLS (NumPy)", "python",
           note="Damped-least-squares Jacobian IK in NumPy (ik_bench/panda.py). "
                "No MoveIt, no cppyy -- the honest Python baseline."),
]

BY_KEY = {s.key: s for s in REGISTRY}
