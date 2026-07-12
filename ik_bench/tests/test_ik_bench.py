#!/usr/bin/env python3
"""Tests for the ik_bench IK benchmark suite (M6c).

Two gates so the suite is honest outside the env:
  * ``_HAVE_PANDA`` -- the panda URDF is locatable via the ament index (the pure-
    NumPy FK / DLS + harness-metadata tests need only this; present in the moveit/ik
    envs).
  * ``_HAVE_MOVEIT`` -- MoveIt headers installed (the FK-agreement + KDL FK(IK)~=pose
    tests bring up moveit_kit).

Run the real thing: ``pixi run -e ik test-ik``. Outside the env everything skips.
"""
import os

import pytest

try:
    from ament_index_python.packages import get_package_share_directory
    get_package_share_directory("moveit_resources_panda_description")
    _HAVE_PANDA = True
except Exception:
    _HAVE_PANDA = False

_HAVE_MOVEIT = os.path.isdir(
    os.path.join(os.environ.get("CONDA_PREFIX", ""), "include", "moveit_core"))

pytestmark = pytest.mark.skipif(
    not _HAVE_PANDA, reason="panda model not found (use the ik env)")

if _HAVE_PANDA:
    import numpy as np
    from ik_bench import solvers as S
    from ik_bench.panda import PandaChain, dls_ik, pose_error, matrix_to_pose


# ---- harness smoke ---------------------------------------------------------
def test_registry_shape():
    """The solver registry is well-formed: unique keys, valid kinds, the expected
    solvers present."""
    keys = [s.key for s in S.REGISTRY]
    assert len(keys) == len(set(keys))
    assert {"kdl", "trac_ik", "bio_ik", "pick_ik", "pure_python"} <= set(keys)
    for s in S.REGISTRY:
        assert s.kind in ("moveit", "python")
        if s.kind == "moveit":
            assert s.plugin  # a pluginlib lookup name


def test_panda_chain_parses():
    """The URDF parses to the 7-DOF panda_arm chain with sane joint limits."""
    chain = PandaChain()
    assert chain.n == 7
    assert len(chain.origins) == 7 and len(chain.axes) == 7
    assert np.all(chain.lower < chain.upper)


def test_fk_shape_and_reachability():
    """FK returns a valid homogeneous transform inside the panda's reach."""
    chain = PandaChain()
    q = np.zeros(7)
    t = chain.fk(q)
    assert t.shape == (4, 4)
    assert abs(np.linalg.det(t[:3, :3]) - 1.0) < 1e-9  # proper rotation
    assert np.linalg.norm(t[:3, 3]) < 1.5              # within arm reach


def test_jacobian_matches_finite_difference():
    """The analytic geometric Jacobian matches a finite-difference of FK."""
    chain = PandaChain()
    rng = np.random.default_rng(1)
    q = chain.random_config(rng, margin=0.1)
    jac = chain.jacobian(q)
    eps = 1e-6
    base = chain.fk(q)
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = eps
        pos_fd = (chain.fk(q + dq)[:3, 3] - base[:3, 3]) / eps
        assert np.allclose(jac[:3, i], pos_fd, atol=1e-4)


# ---- solver correctness: FK(IK(pose)) ~= pose ------------------------------
def test_pure_python_fk_of_ik_matches_pose():
    """The Python DLS baseline: FK of its IK solution reproduces the target pose."""
    chain = PandaChain()
    rng = np.random.default_rng(7)
    hits = 0
    for _ in range(10):
        q_target = chain.random_config(rng, margin=0.1)
        target = matrix_to_pose(chain.fk(q_target))
        seed = chain.random_config(rng, margin=0.1)
        ok, q, _ = dls_ik(chain, target, seed, pos_tol=1e-3, ori_tol=1e-2,
                          time_budget=0.2, rng=rng)
        if ok:
            pe, oe = pose_error(chain.fk(q), target)
            assert pe < 1e-3 and oe < 1e-2   # FK(IK(pose)) ~= pose
            hits += 1
    assert hits >= 7   # a capable-enough baseline on easy reachable targets


@pytest.mark.skipif(not _HAVE_MOVEIT, reason="MoveIt not installed")
def test_numpy_fk_agrees_with_moveit():
    """The NumPy FK matches MoveIt's own FK to ~1e-9 -- so the benchmark's targets
    (generated with MoveIt FK) are measured consistently for the Python row."""
    import cppyy
    import moveit_kit
    moveit = moveit_kit.bringup_moveit()
    cfg = moveit_kit.panda_config()
    model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
    jmg = model.getJointModelGroup(S.GROUP)
    state = moveit.core.RobotState(model)
    chain = PandaChain()
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = chain.random_config(rng, margin=0.05)
        v = cppyy.gbl.std.vector["double"]([float(x) for x in q])
        state.setJointGroupPositions(jmg, v)
        state.update()
        tf = state.getGlobalLinkTransform(S.TIP)
        tr = tf.translation()
        quat = cppyy.gbl.Eigen.Quaterniond(tf.rotation())
        mv_pose = (tr[0], tr[1], tr[2], quat.x(), quat.y(), quat.z(), quat.w())
        pe, oe = pose_error(chain.fk(q), mv_pose)
        assert pe < 1e-6 and oe < 1e-6


@pytest.mark.skipif(not _HAVE_MOVEIT, reason="MoveIt not installed")
def test_kdl_plugin_fk_of_ik_matches_pose():
    """The KDL plugin path (as the benchmark drives it): load via pluginlib, solve
    IK to a reachable pose, and FK of the solution reproduces it."""
    import cppyy
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    import moveit_kit
    moveit = moveit_kit.bringup_moveit(with_kinematics=True)
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    cfg = moveit_kit.panda_config()
    model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)
    jmg = model.getJointModelGroup(S.GROUP)
    node = moveit_kit.make_node("test_ikbench_kdl")
    assert moveit_kit.load_kinematics_solver(
        node, model, S.GROUP, plugin="kdl_kinematics_plugin/KDLKinematicsPlugin")
    state = moveit.core.RobotState(model)
    state.setToDefaultValues(jmg, "ready")
    state.update()
    target = cppyy.gbl.Eigen.Isometry3d(state.getGlobalLinkTransform(S.TIP))
    tr = target.translation()
    state.setToRandomPositions(jmg)
    state.update()
    assert state.setFromIK(jmg, target, 0.2)
    state.update()
    got = state.getGlobalLinkTransform(S.TIP).translation()
    err = sum((got[i] - tr[i]) ** 2 for i in range(3)) ** 0.5
    assert err < 1e-3
