"""Measure startup, retained builds, native query, conversion and SciPy work."""
from time import perf_counter
_startup = perf_counter()
import json
import os
import platform
from pathlib import Path
import resource
import statistics
import subprocess

import index as adapter
from index import Index, np
ADAPTER_STARTUP_SECONDS = perf_counter() - _startup
_start = perf_counter()
import scipy
from scipy.spatial import cKDTree
SCIPY_IMPORT_SECONDS = perf_counter() - _start
from acceptance import compare, reference


def scipy_filtered(tree, points, point_frames, confidence, queries, frames,
                   k, frame_gap, min_confidence, max_distance):
    start = perf_counter()
    candidates = tree.query_ball_point(queries, max_distance, eps=0, workers=1)
    search_seconds = perf_counter() - start
    start = perf_counter()
    ids = np.full((len(queries), k), -1, dtype=np.int64)
    d2 = np.full((len(queries), k), np.inf)
    counts = np.zeros(len(queries), dtype=np.int64)
    centroids = np.full((len(queries), 3), np.nan)
    for j, neighbors in enumerate(candidates):
        selected = np.asarray(neighbors, dtype=np.int64)
        keep = confidence[selected] >= min_confidence
        if frame_gap >= 0:
            keep &= np.abs(point_frames[selected]-frames[j]) > frame_gap
        selected = selected[keep]
        squared = np.sum((points[selected]-queries[j])**2, axis=1)
        # Recheck the inclusive radius using the oracle's double arithmetic.
        keep = squared <= max_distance**2
        selected, squared = selected[keep], squared[keep]
        order = np.lexsort((selected, squared))[:k]
        chosen = selected[order]
        count = len(chosen)
        ids[j, :count] = chosen
        d2[j, :count] = squared[order]
        counts[j] = count
        if count:
            centroids[j] = points[chosen].mean(axis=0)
    selection_seconds = perf_counter() - start
    return dict(ids=ids, distances_squared=d2, counts=counts, centroids=centroids,
                search_seconds=search_seconds, selection_seconds=selection_seconds,
                total_seconds=search_seconds+selection_seconds)


def median(values):
    return float(statistics.median(values))


def scipy_adaptive(tree, points, point_frames, confidence, queries, frames,
                   k, frame_gap, min_confidence, max_distance):
    """Bulk kNN, growing candidates only for unresolved rows and boundary ties."""
    start = perf_counter()
    ids = np.full((len(queries), k), -1, dtype=np.int64)
    d2 = np.full((len(queries), k), np.inf)
    counts = np.zeros(len(queries), dtype=np.int64)
    centroids = np.full((len(queries), 3), np.nan)
    pending = np.arange(len(queries))
    budget = min(len(points), max(32, 4*k))
    search_seconds = 0.
    while len(pending):
        search_start = perf_counter()
        _, candidates = tree.query(queries[pending], k=list(range(1, budget+1)),
                                   eps=0, workers=1,
                                   distance_upper_bound=np.nextafter(max_distance, np.inf))
        search_seconds += perf_counter()-search_start
        unresolved = []
        for row, j in enumerate(pending):
            selected = candidates[row]
            selected = selected[selected < len(points)]
            squared = np.sum((points[selected]-queries[j])**2, axis=1)
            eligible = (confidence[selected] >= min_confidence) & (squared <= max_distance**2)
            if frame_gap >= 0:
                eligible &= np.abs(point_frames[selected]-frames[j]) > frame_gap
            choices = np.flatnonzero(eligible)
            chosen = choices[np.lexsort((selected[choices], squared[choices]))[:k]]
            count = len(chosen)
            exhausted = len(selected) < budget or budget == len(points)
            boundary_resolved = count == k and squared[-1] > squared[chosen[-1]]
            if not exhausted and not boundary_resolved:
                unresolved.append(j)
                continue
            counts[j] = count
            ids[j, :count] = selected[chosen]
            d2[j, :count] = squared[chosen]
            if count:
                centroids[j] = points[selected[chosen]].mean(axis=0)
        pending = np.asarray(unresolved, dtype=np.int64)
        budget = min(len(points), 2*budget)
    total_seconds = perf_counter()-start
    return dict(ids=ids, distances_squared=d2, counts=counts, centroids=centroids,
                search_seconds=search_seconds, selection_seconds=total_seconds-search_seconds,
                total_seconds=total_seconds)


def rss_kib():
    for line in Path("/proc/self/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1])


def main():
    rng = np.random.default_rng(4026)
    n, m, k, repeats = 50000, 2000, 8, 5
    points = rng.uniform(-1., 1., (n, 3))
    point_frames = np.arange(n, dtype=np.int64)
    confidence = rng.uniform(0., 1., n)
    queries = rng.uniform(-1., 1., (m, 3))
    frames = rng.integers(0, n, m, dtype=np.int64)
    policy = dict(k=k, frame_gap=128, min_confidence=.6, max_distance=.25)
    before = rss_kib()
    with Index(points, point_frames, confidence) as native:
        after = rss_kib()
        start = perf_counter()
        tree = cKDTree(points, leafsize=16, compact_nodes=True, copy_data=True)
        scipy_build = perf_counter() - start
        first = native.query(queries, frames, **policy)
        sample_expected = reference(points, point_frames, confidence,
                                    queries[:32], frames[:32], **policy)
        compare({key: first[key][:32] for key in sample_expected}, sample_expected)
        scipy_first = scipy_filtered(tree, points, point_frames, confidence,
                                     queries, frames, **policy)
        compare(first, scipy_first)
        adaptive_first = scipy_adaptive(tree, points, point_frames, confidence,
                                        queries, frames, **policy)
        compare(first, adaptive_first)
        native_runs = [native.query(queries, frames, **policy) for _ in range(repeats)]
        scipy_runs = [scipy_filtered(tree, points, point_frames, confidence,
                                    queries, frames, **policy) for _ in range(repeats)]
        adaptive_runs = [scipy_adaptive(tree, points, point_frames, confidence,
                                       queries, frames, **policy) for _ in range(repeats)]
        # Compare an already available binding on its direct vectorized kNN API.
        plain = native.query(queries, frames, k=k)
        scipy_plain = []
        for _ in range(repeats):
            start = perf_counter()
            distances, neighbors = tree.query(queries, k=k, eps=0, workers=1)
            scipy_plain.append(perf_counter()-start)
        np.testing.assert_array_equal(plain["ids"], neighbors)
        np.testing.assert_allclose(plain["distances_squared"], distances**2, rtol=2e-14)
        plain_native = [native.query(queries, frames, k=k) for _ in range(repeats)]
        start = perf_counter()
        reference(points, point_frames, confidence, queries[:32], frames[:32], **policy)
        brute_seconds = perf_counter()-start
        result = {
            "parameters": dict(n=n, m=m, k=k, repeats=repeats, seed=4026, **{a:b for a,b in policy.items() if a != "k"}),
            "versions": {"python": platform.python_version(), "numpy": np.__version__,
                         "scipy": scipy.__version__, "cppyy": adapter.cppyy.__version__,
                         "cppyy_kit_path": adapter.cppyy_kit.__file__,
                         "nanoflann": adapter.VERSION, "header_sha256": adapter.ACTUAL_HEADER_SHA256,
                         "compiler": subprocess.check_output([os.environ.get("CXX", "c++"), "--version"], text=True).splitlines()[0],
                         "platform": platform.platform(), "machine": platform.machine()},
            "startup_seconds": {"adapter_import_total": ADAPTER_STARTUP_SECONDS,
                                "cppyy_kit_numpy_import": adapter.IMPORT_SECONDS,
                                "header_discovery_verification": adapter.HEADER_SECONDS,
                                "adapter_prebuild": adapter.PREBUILD_SECONDS,
                                "adapter_load_declarations": adapter.COMPILE_SECONDS,
                                "scipy_import": SCIPY_IMPORT_SECONDS},
            "cache": adapter.COMPILE,
            "build_seconds": {"input_validation_conversion": native.input_conversion_seconds,
                              "constructor_call": native.constructor_seconds,
                              "native_copy_and_tree_build": native.build_seconds,
                              "scipy_copy_and_tree_build": scipy_build},
            "first_filtered_call_seconds": {key:first[key] for key in ("native_seconds", "call_seconds", "conversion_seconds")},
            "warmed_filtered_seconds": {"native_query_and_centroid": median([r["native_seconds"] for r in native_runs]),
                                        "native_call": median([r["call_seconds"] for r in native_runs]),
                                        "validation_allocation": median([r["conversion_seconds"] for r in native_runs]),
                                        "output_conversion": 0.,
                                        "scipy_radius_search_and_candidate_lists": median([r["search_seconds"] for r in scipy_runs]),
                                        "scipy_python_selection_and_centroid": median([r["selection_seconds"] for r in scipy_runs]),
                                        "scipy_total": median([r["total_seconds"] for r in scipy_runs]),
                                        "scipy_adaptive_knn_search": median([r["search_seconds"] for r in adaptive_runs]),
                                        "scipy_adaptive_python_selection": median([r["selection_seconds"] for r in adaptive_runs]),
                                        "scipy_adaptive_total": median([r["total_seconds"] for r in adaptive_runs])},
            "warmed_unfiltered_seconds": {"native_call_including_centroid": median([r["call_seconds"] for r in plain_native]),
                                          "scipy_query_without_centroid": median(scipy_plain)},
            "numpy_brute_force_32_queries_seconds": brute_seconds,
            "memory_kib": {"process_rss_before_native_build": before, "process_rss_after_native_build": after,
                           "process_peak_rss": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                           "native_input_copy_logical": n*(3*8+8+8)/1024},
            "acceptance": "PASS: 32 independent oracle queries, all 2000 filtered SciPy radius/adaptive queries, all 2000 direct SciPy kNN queries",
        }
    destination = Path(__file__).with_name("build") / "benchmark.json"
    destination.parent.mkdir(exist_ok=True)
    destination.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
