#!/usr/bin/env python3
"""Compare three OMPL planners on one 2D circular-obstacle problem.

Each planner runs in a fresh process so the global OMPL RNG can use the same
seed. Results include raw solve time and path length; RRTstar uses the full
time budget to improve its solution, so solve times are not directly comparable.
"""
import argparse
import json
import math
import subprocess
import sys
import tempfile
import time
from pathlib import Path

START = (0.1, 0.1)
GOAL = (0.9, 0.9)
PLANNERS = ("RRTConnect", "RRTstar", "PRM")
COLORS = {"RRTConnect": "#1769aa", "RRTstar": "#d1495b", "PRM": "#2a9d63"}
VALIDATION_STEP = 0.005


def valid_point(point, obstacle):
    x, y = point
    cx, cy, radius = obstacle
    return (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0
            and (x - cx) ** 2 + (y - cy) ** 2 > radius ** 2)


def validate_path(points, obstacle):
    if not points or any(not valid_point(point, obstacle) for point in points):
        return False
    if (not all(math.isclose(a, b, abs_tol=1e-9)
                for a, b in zip(points[0], START))
            or not all(math.isclose(a, b, abs_tol=1e-9)
                       for a, b in zip(points[-1], GOAL))):
        return False
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, math.ceil(distance / VALIDATION_STEP))
        for i in range(steps + 1):
            t = i / steps
            if not valid_point((start[0] + t * (end[0] - start[0]),
                                start[1] + t * (end[1] - start[1])), obstacle):
                return False
    return True


def plan_one(planner_name, obstacle, seed, time_limit):
    import cppyy
    import ompl_kit

    ob, og = ompl_kit.bringup_ompl()
    if planner_name == "PRM":
        # cppyy reads the installed header; no generated PRM binding is needed.
        cppyy.include("ompl/geometric/planners/prm/PRM.h")
    ompl_kit.set_seed(seed)

    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)
    setup = og.SimpleSetup(ob.StateSpacePtr(space))
    cx, cy, radius = obstacle

    def is_valid(state):
        return (state[0] - cx) ** 2 + (state[1] - cy) ** 2 > radius ** 2

    setup.setStateValidityChecker(ompl_kit.validity_checker(is_valid, owner=setup))
    setup.getSpaceInformation().setStateValidityCheckingResolution(
        VALIDATION_STEP / float(space.getMaximumExtent()))
    state_type = ob.ScopedState[ob.RealVectorStateSpace]
    start, goal = state_type(setup.getStateSpace()), state_type(setup.getStateSpace())
    start[0], start[1] = START
    goal[0], goal[1] = GOAL
    setup.setStartAndGoalStates(start, goal)
    planner = getattr(og, planner_name)
    setup.setPlanner(ob.PlannerPtr(planner(setup.getSpaceInformation())))
    setup.setup()

    begin = time.perf_counter()
    solved = bool(setup.solve(time_limit))
    elapsed = time.perf_counter() - begin
    points, length = [], None
    if solved:
        path = setup.getSolutionPath()
        points = [list(point) for point in ompl_kit.path_to_list(path, dim=2)]
        length = float(path.length())
    return {"planner": planner_name, "solved": solved, "valid":
            solved and validate_path(points, obstacle), "seconds": elapsed,
            "length": length, "points": points}


def svg_path(points):
    return " ".join(f"{70 + x * 500:.2f},{550 - y * 500:.2f}" for x, y in points)


def write_svg(path, obstacle, results):
    cx, cy, radius = obstacle
    ox, oy = 70 + cx * 500, 550 - cy * 500
    pieces = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 680 610" '
        'width="680" height="610">',
        '<rect width="680" height="610" fill="white"/>',
        '<rect x="70" y="50" width="500" height="500" fill="#f6f8fa" '
        'stroke="#344054" stroke-width="2"/>',
        f'<circle cx="{ox:.2f}" cy="{oy:.2f}" r="{radius * 500:.2f}" '
        'fill="#f4a6a6" stroke="#a32222" stroke-width="2"/>',
        '<text x="70" y="30" font-family="sans-serif" font-size="16">'
        'OMPL planner paths (unit square)</text>',
    ]
    for result in results:
        if result["valid"]:
            color = COLORS[result["planner"]]
            pieces.append(f'<polyline points="{svg_path(result["points"])}" '
                          f'fill="none" stroke="{color}" stroke-width="3" '
                          'stroke-linejoin="round" stroke-linecap="round"/>')
    sx, sy = 70 + START[0] * 500, 550 - START[1] * 500
    gx, gy = 70 + GOAL[0] * 500, 550 - GOAL[1] * 500
    pieces.extend([
        f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="6" fill="#111827"/>',
        f'<circle cx="{gx:.2f}" cy="{gy:.2f}" r="6" fill="#111827"/>',
        '<text x="70" y="585" font-family="sans-serif" font-size="13">'
        'Start ●   Goal ●   Obstacle (red)</text>',
    ])
    for i, result in enumerate(results):
        y = 70 + i * 22
        color = COLORS[result["planner"]]
        state = (f'length {result["length"]:.3f}' if result["valid"]
                 else "no valid path")
        pieces.append(f'<text x="430" y="{y}" fill="{color}" '
                      f'font-family="sans-serif" font-size="13">'
                      f'{result["planner"]}: {state}</text>')
    pieces.append("</svg>")
    path.write_text("\n".join(pieces), encoding="utf-8")


def child_args(args, planner, result_file):
    command = [sys.executable, str(Path(__file__).resolve()), "--child",
               "--planner", planner, "--seed", str(args.seed),
               "--time-limit", str(args.time_limit), "--result-file", str(result_file)]
    for flag, value in zip(("--obstacle-x", "--obstacle-y", "--obstacle-radius"),
                           (args.obstacle_x, args.obstacle_y, args.obstacle_radius)):
        command.extend((flag, str(value)))
    return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--obstacle-x", type=float, default=0.5)
    parser.add_argument("--obstacle-y", type=float, default=0.5)
    parser.add_argument("--obstacle-radius", type=float, default=0.25)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--time-limit", type=float, default=1.0,
                        help="solve cap in seconds per planner")
    parser.add_argument("--svg", type=Path, default=Path("ompl_planner_sweep.svg"))
    parser.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--planner", choices=PLANNERS, help=argparse.SUPPRESS)
    parser.add_argument("--result-file", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    obstacle = (args.obstacle_x, args.obstacle_y, args.obstacle_radius)
    if (args.time_limit <= 0 or args.obstacle_radius <= 0
            or not all(0.0 < coordinate < 1.0 for coordinate in obstacle[:2])
            or not all(valid_point(point, obstacle) for point in (START, GOAL))):
        parser.error("use positive time/radius, an in-bounds obstacle center, "
                     "and an obstacle that leaves start and goal free")

    if args.child:
        if not args.planner or not args.result_file:
            parser.error("internal child invocation is missing planner/result file")
        result = plan_one(args.planner, obstacle, args.seed, args.time_limit)
        args.result_file.write_text(json.dumps(result), encoding="utf-8")
        return

    results = []
    with tempfile.TemporaryDirectory(prefix="ompl-sweep-") as directory:
        for planner in PLANNERS:
            result_file = Path(directory) / f"{planner}.json"
            completed = subprocess.run(child_args(args, planner, result_file),
                                       text=True, capture_output=True)
            if completed.returncode:
                raise RuntimeError(f"{planner} child failed:\n{completed.stderr}\n"
                                   f"{completed.stdout}")
            results.append(json.loads(result_file.read_text(encoding="utf-8")))

    print(f"Circle center=({obstacle[0]:g}, {obstacle[1]:g}), "
          f"radius={obstacle[2]:g}; seed={args.seed}; cap={args.time_limit:g}s each")
    print("Times are solve() time only. RRTstar spends the budget improving its path.")
    for result in results:
        length = f'{result["length"]:.4f}' if result["length"] is not None else "n/a"
        status = "valid" if result["valid"] else (
            "invalid path" if result["solved"] else "no solution")
        print(f'{result["planner"]:9} {status:12} '
              f'{result["seconds"] * 1000:8.2f} ms  length {length}')
    write_svg(args.svg, obstacle, results)
    print(f"SVG: {args.svg}")
    failed = [result["planner"] for result in results if not result["valid"]]
    if failed:
        print("No independently validated path from: " + ", ".join(failed),
              file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
