"""Independent checks. Does not import the submitted Python policy."""
import math


def expected(point, bias):
    x, y = point
    # Independent geometry uses Euclidean distance rather than the policy's
    # squared-distance calculation. The circle boundary is invalid.
    return int(0 <= x <= 1 and 0 <= y <= 1
               and math.hypot(x - (0.5 - bias), y - 0.5) > 0.25)


def check_path(result):
    values = result["path_xy"]
    points = list(zip(values[::2], values[1::2]))
    assert points[0] == (0.1, 0.1) and points[-1] == (0.9, 0.9)
    assert all(expected(p, result["bias_x"]) for p in points)
    length = 0.0
    center = (0.5 - result["bias_x"], 0.5)
    # Check each continuous segment's minimum distance from the circle center.
    # This is stronger than checking only OMPL's discrete interpolation samples.
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        norm2 = dx * dx + dy * dy
        projection = 0.0 if norm2 == 0 else (
            (center[0] - a[0]) * dx + (center[1] - a[1]) * dy) / norm2
        t = max(0.0, min(1.0, projection))
        nearest = (a[0] + t * dx, a[1] + t * dy)
        assert math.dist(center, nearest) > 0.25, "segment intersects keep-out circle"
        length += math.sqrt(norm2)
    assert math.isclose(length, result["path_length"], rel_tol=1e-12)


def check_results(results):
    py, native, lifecycle = results
    for result in [py, native]:
        assert result["exact"]
        assert result["solve_calls"] > 7
        reference = [expected(p, result["bias_x"]) for p in result["points"]]
        assert result["decisions"] == reference
        assert result["accepted"] == sum(reference) * result["repeats"]
        check_path(result)
    for field in ["decisions", "accepted", "solve_calls", "path_xy", "path_length"]:
        assert py[field] == native[field], f"Python/native mismatch: {field}"
    assert lifecycle["released"] == 25
    assert [x["calls"] for x in lifecycle["exceptions"]] == [2, 7]
    assert all(x["type"] == "ValueError" for x in lifecycle["exceptions"])
    assert lifecycle["boundary_nonfinite_decisions"] == [[0, 0, 0, 0, 1, 1, 0, 0, 0]] * 2
    assert lifecycle["invalid_bias_rejections"] == 6


if __name__ == "__main__":
    import json
    import sys
    check_results(json.load(open(sys.argv[1]))["results"])
    print("PASS: saved evidence checked independently")
