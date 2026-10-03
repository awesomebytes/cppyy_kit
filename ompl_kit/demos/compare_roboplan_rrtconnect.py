#!/usr/bin/env python3
"""Compare RoboPlan and OMPL RRT-Connect on the bundled UR5 scene.

Run in the optional OMPL + RoboPlan Pixi environment. Each trial uses the same
endpoints, UR5 self-collision model, joint bounds, and 0.05-radian edge-check
spacing. Setup and one warmup solve per planner are outside the timed solves.
OMPL's validity callback crosses Python/C++, while RoboPlan checks in C++.
"""
import math
import statistics
import time
from pathlib import Path

import numpy as np

import ompl_kit
from roboplan import core, example_models, rrt


GROUP = "arm"
SEED = 42
TRIALS = 5
TIME_LIMIT = 1.0
EDGE_STEP = 0.05
OBSTACLE_CENTER = (0.35, 0.0, 0.55)
OBSTACLE_SIZE = (0.22, 0.22, 0.22)


def collision_free_pair(scene, context, baseline_scene, baseline_context, q_indices):
    """Find a pair clear in the baseline whose direct edge hits the added box."""
    baseline_context.setRngSeed(SEED)
    for _ in range(20):
        start = baseline_context.randomCollisionFreePositions(1000)
        if start is None:
            continue
        if (not scene.isValidConfiguration(start) or context.hasCollisions(start)):
            continue
        for _ in range(100):
            goal = baseline_context.randomCollisionFreePositions(1000)
            if goal is None:
                continue
            fixed_goal = start.copy()
            fixed_goal[q_indices] = goal[q_indices]
            if np.linalg.norm(fixed_goal[q_indices] - start[q_indices]) < 1.0:
                continue
            if (not baseline_scene.isValidConfiguration(fixed_goal)
                    or baseline_context.hasCollisions(fixed_goal)
                    or not scene.isValidConfiguration(fixed_goal)
                    or context.hasCollisions(fixed_goal)):
                continue
            baseline_edge_clear = edge_is_valid(
                baseline_scene, baseline_context, q_indices, start,
                start[q_indices], fixed_goal[q_indices])
            obstacle_edge_clear = edge_is_valid(
                scene, context, q_indices, start,
                start[q_indices], fixed_goal[q_indices])
            if baseline_edge_clear and not obstacle_edge_clear:
                return start, fixed_goal
    raise RuntimeError(
        "Could not find endpoints valid in both scenes with a clear baseline "
        "edge and an edge blocked by the world obstacle after 20 starts x "
        "100 goals"
    )


def edge_is_valid(scene, context, q_indices, reference, start, goal):
    """Check a whole straight group-space segment at no more than EDGE_STEP."""
    distance = float(np.linalg.norm(goal - start))
    steps = max(1, math.ceil(distance / EDGE_STEP))
    for i in range(steps + 1):
        full_q = reference.copy()
        full_q[q_indices] = start + (goal - start) * (i / steps)
        if not bool(scene.isValidConfiguration(full_q)) or context.hasCollisions(full_q):
            return False
    return True


def path_is_valid(scene, context, q_indices, reference, points, start, goal,
                  lower, upper):
    if not points:
        return False
    if any(point.shape != start.shape for point in points):
        return False
    if (not np.allclose(points[0], start, atol=1e-9, rtol=0.0)
            or not np.allclose(points[-1], goal, atol=1e-9, rtol=0.0)):
        return False
    for point in points:
        if (not np.all(np.isfinite(point)) or np.any(point < lower)
                or np.any(point > upper)):
            return False
    first = reference.copy()
    first[q_indices] = points[0]
    if not scene.isValidConfiguration(first) or context.hasCollisions(first):
        return False
    return all(
        edge_is_valid(scene, context, q_indices, reference, a, b)
        for a, b in zip(points, points[1:])
    )


def make_ompl(ob, og, scene, lower, upper, q_indices, reference, context,
              start, goal):
    dim = len(start)
    space = ob.RealVectorStateSpace(dim)
    bounds = ob.RealVectorBounds(dim)
    for i, (lo, hi) in enumerate(zip(lower, upper)):
        bounds.setLow(i, float(lo))
        bounds.setHigh(i, float(hi))
    space.setBounds(bounds)
    extent = float(space.getMaximumExtent())
    if extent <= EDGE_STEP:
        raise RuntimeError("UR5 OMPL state-space extent is unexpectedly small")

    setup = og.SimpleSetup(ob.StateSpacePtr(space))

    def valid(state):
        q = np.asarray([state[i] for i in range(dim)], dtype=np.float64)
        full_q = reference.copy()
        full_q[q_indices] = q
        return bool(scene_valid(scene, context, full_q))

    setup.setStateValidityChecker(ompl_kit.validity_checker(valid, owner=setup))
    setup.getSpaceInformation().setStateValidityCheckingResolution(EDGE_STEP / extent)
    state_type = ob.ScopedState[ob.RealVectorStateSpace]
    start_state = state_type(setup.getStateSpace())
    goal_state = state_type(setup.getStateSpace())
    for i in range(dim):
        start_state[i] = float(start[i])
        goal_state[i] = float(goal[i])
    setup.setStartAndGoalStates(start_state, goal_state)
    setup.setPlanner(ob.PlannerPtr(og.RRTConnect(setup.getSpaceInformation())))
    setup.setup()
    return setup


def scene_valid(scene, context, full_q):
    return bool(scene.isValidConfiguration(full_q)) and not context.hasCollisions(full_q)


def run_ompl(ob, og, scene, context, q_indices, reference, lower, upper, start, goal):
    results = []
    warmup = make_ompl(ob, og, scene, lower, upper, q_indices, reference,
                       context, start, goal)
    warmup.solve(TIME_LIMIT)
    for trial in range(TRIALS):
        setup = make_ompl(ob, og, scene, lower, upper, q_indices, reference,
                          context, start, goal)
        begin = time.perf_counter()
        solved = bool(setup.solve(TIME_LIMIT))
        elapsed = time.perf_counter() - begin
        if not solved:
            results.append((elapsed, False, False))
            continue
        points = [np.asarray(p, dtype=np.float64)
                  for p in ompl_kit.path_to_list(setup.getSolutionPath(), dim=len(start))]
        results.append((elapsed, True, path_is_valid(scene, context, q_indices,
                                                     reference, points, start, goal,
                                                     lower, upper)))
    return results


def run_roboplan(scene, context, q_indices, joint_names, reference,
                 lower, upper, start, goal):
    results = []
    options = rrt.RRTOptions(
        group_name=GROUP,
        max_planning_time=TIME_LIMIT,
        collision_check_step_size=EDGE_STEP,
        collision_check_use_bisection=False,
        rrt_connect=True,
        rrt_star=False,
        fast_return=True,
    )
    warmup = rrt.RRT(scene, options)
    warmup.setRngSeed(SEED)
    try:
        warmup.plan(
            core.JointConfiguration(joint_names, np.asarray(start, dtype=np.float64)),
            core.JointConfiguration(joint_names, np.asarray(goal, dtype=np.float64)),
        )
    except RuntimeError:
        pass  # Warmup is unmeasured; measured trials still report their own results.
    for trial in range(TRIALS):
        planner = rrt.RRT(scene, options)
        planner.setRngSeed(SEED + trial)
        start_cfg = core.JointConfiguration(joint_names, np.asarray(start, dtype=np.float64))
        goal_cfg = core.JointConfiguration(joint_names, np.asarray(goal, dtype=np.float64))
        begin = time.perf_counter()
        try:
            path = planner.plan(start_cfg, goal_cfg)
        except RuntimeError:
            path = None
        elapsed = time.perf_counter() - begin
        if path is None:
            results.append((elapsed, False, False))
            continue
        points = [np.asarray(q, dtype=np.float64) for q in path.positions]
        results.append((elapsed, True, path_is_valid(scene, context, q_indices,
                                                     reference, points, start, goal,
                                                     lower, upper)))
    return results


def summarize(name, results):
    valid_times = [elapsed for elapsed, _, valid in results if valid]
    solved = sum(solved for _, solved, _ in results)
    invalid = solved - len(valid_times)
    median = (f"{statistics.median(valid_times) * 1000:.2f} ms"
              if len(valid_times) == len(results) else "n/a")
    print(f"{name:8} solved {solved}/{len(results)}, "
          f"validated {len(valid_times)}/{len(results)}, "
          f"invalid paths {invalid}; median solve {median}")
    return statistics.median(valid_times) if len(valid_times) == len(results) else None


def main():
    models = Path(str(example_models.get_package_models_dir())) / "ur_robot_model"
    urdf = models / "ur5_gripper.urdf"
    srdf = models / "ur5_gripper.srdf"
    if not urdf.is_file() or not srdf.is_file():
        raise FileNotFoundError(f"Expected bundled RoboPlan UR5 files under {models}")

    description = core.loadUrdfSceneDescription(str(urdf), [str(models.parent)])
    scene = core.Scene("ur5", description)
    scene.importSrdf(srdf.read_text())
    baseline_description = core.loadUrdfSceneDescription(
        str(urdf), [str(models.parent)])
    baseline_scene = core.Scene("ur5_baseline", baseline_description)
    baseline_scene.importSrdf(srdf.read_text())
    obstacle_pose = np.eye(4, dtype=np.float64, order="F")
    obstacle_pose[:3, 3] = OBSTACLE_CENTER
    scene.addBoxGeometry(
        "tutorial_obstacle", "base_link", core.Box(*OBSTACLE_SIZE),
        obstacle_pose, np.array([0.9, 0.15, 0.1, 1.0], dtype=np.float64),
    )
    group = scene.getJointGroupInfo(GROUP)
    q_indices = np.asarray(group.q_indices, dtype=np.int64)
    joint_names = list(group.joint_names)
    if group.has_continuous_dofs:
        raise RuntimeError("This demo requires a bounded, Euclidean UR5 arm group")
    limits = scene.getPositionLimitVectors(GROUP, collapsed=True)
    lower, upper = (np.asarray(limits[0]), np.asarray(limits[1]))
    if len(q_indices) != len(lower) or len(joint_names) != len(lower):
        raise RuntimeError("UR5 group ordering/limits are inconsistent")

    # SceneContext snapshots collision geometry, so create it after the obstacle.
    context = core.SceneContext(scene)
    baseline_context = core.SceneContext(baseline_scene)
    full_start, full_goal = collision_free_pair(
        scene, context, baseline_scene, baseline_context, q_indices)
    # Freeze every non-arm variable at the start sample for both planners.
    reference = full_start.copy()
    scene.setJointPositions(reference)
    context.setJointPositions(reference)
    start, goal = full_start[q_indices].copy(), full_goal[q_indices].copy()
    if not path_is_valid(scene, context, q_indices, reference, [start], start,
                         start, lower, upper) or not path_is_valid(
        scene, context, q_indices, reference, [goal], goal, goal, lower, upper
    ):
        raise RuntimeError("Sampled UR5 endpoint failed collision validation")

    ob, og = ompl_kit.bringup_ompl()
    # OMPL's process-global RNG can only be seeded before its first sample.
    ompl_kit.set_seed(SEED)
    print(f"UR5 group={GROUP}, joints={joint_names}, trials={TRIALS}, "
          f"seed={SEED} (OMPL stream), RoboPlan seeds={SEED}..{SEED + TRIALS - 1}, "
          f"edge step={EDGE_STEP:g} rad, "
          f"time limit={TIME_LIMIT:g} s/trial")
    print(f"World box at {OBSTACLE_CENTER}, size={OBSTACLE_SIZE}; "
          "selected endpoints are valid in both scenes; straight interpolation "
          "is clear without the box and blocked with it.")
    roboplan_results = run_roboplan(scene, context, q_indices, joint_names,
                                    reference, lower, upper, start, goal)
    ompl_results = run_ompl(ob, og, scene, context, q_indices, reference,
                            lower, upper, start, goal)
    rp = summarize("RoboPlan", roboplan_results)
    om = summarize("OMPL", ompl_results)
    if rp is not None and om is not None:
        winner = "RoboPlan" if rp < om else "OMPL" if om < rp else "tie"
        ratio = max(rp, om) / min(rp, om)
        print(f"Median validated-solve-time ranking: {winner} "
              f"({ratio:.2f}x; all trials validated).")
    else:
        print("Median validated-solve-time ranking: unavailable (at least one "
              "planner had an unvalidated or failed trial).")
    print("Timing is illustrative: OMPL calls its Python validity callback; "
          "RoboPlan performs planner collision checks in C++. One unmeasured "
          "warmup solve is run per planner; the OMPL RNG then advances as a "
          "single seeded stream across its measured trials.")


if __name__ == "__main__":
    main()
