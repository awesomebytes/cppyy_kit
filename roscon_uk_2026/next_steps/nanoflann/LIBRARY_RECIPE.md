# Library recipe: retained filtered nanoflann queries

Read this guide explicitly before completing the skeleton. It is a task recipe,
not an installed skill. Commands require this repository checkout and run from
`roscon_uk_2026/next_steps/nanoflann`.

1. Run `pixi install --locked`. Use the locked compiler and runtime pair. Read
   the local `cppyy_kit/require.py`, `cache.py` and `_compile.py` if needed.
   Pixi activates this checkout's `cppyy_kit` and its pinned compiler dependencies.
   Use the recommended top-level `cppyy_kit.require`, `prebuild` and
   `cppdef_cached` interfaces for library setup and compilation. Their backing
   modules are implementation details. They select the compiler automatically.
   For compiler, header, and native runtime discovery:

   ```bash
   pixi run python -m cppyy_kit status --environment
   ```

   Do not search for or hard-code compiler installation paths.
2. Call `cppyy_kit.require` with header `nanoflann.hpp`. Search the active Conda
   prefix and `/usr/include` and `/usr/local/include` first. Only fetch when
   no installed header resolves. Fallback URL:
   `https://raw.githubusercontent.com/jlblancoc/nanoflann/v1.6.3/include/nanoflann.hpp`.
   Expected SHA256 is
   `e47f12ae2fc339cd2eb9ca148af837bd1025ff7e301ecf1860675ff1c73dd071`.
   Use `cache_dir=build/vendor`. Current `require` verifies cached contents and
   serializes same-library fetches across processes. Rehash the resolved header
   against this experiment's pin because installed headers are accepted before
   download/cache discovery. An installed different header must
   fail with a concrete version/checksum message for this pinned experiment.
3. Inspect the [v1.6.3 header and API](https://github.com/jlblancoc/nanoflann/blob/v1.6.3/include/nanoflann.hpp).
   Define an adaptor with `kdtree_get_point_count`, `kdtree_get_pt`, and
   `kdtree_get_bbox`. A false bbox result lets the tree compute its own bounds.
   Compose `KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, Cloud, double, std::size_t>, Cloud,
   3, std::size_t>`. The constructor builds the index under default flags.
   Own immutable vectors in the same native object and destroy the tree before
   its referenced storage. Disable copies/moves that could invalidate references.
4. Implement a custom `findNeighbors` result set. It needs `size`, `full`,
   `worstDist`, `addPoint`, and `sort`. Keep at most `min(k,n)` eligible results.
   Track candidates by `(squared distance, original index)`. Reject low
   confidence, temporally excluded and out-of-radius candidates in `addPoint`.
   Continue traversal after rejected candidates. Postfiltering the nearest
   unfiltered `k` candidates cannot implement this operation correctly.
5. Pass `SearchParameters(0)` for exact traversal. `L2_Simple_Adaptor` returns
   squared distances. The leaf comparison is strict `distance < worstDist`.
   Use a bound that includes equal-distance and radius-boundary candidates,
   such as the next representable value above the current worst squared distance
   or radius. Resolve ties by original index inside `addPoint`. Do not stop
   traversal merely because the result set has reached capacity.
6. Declare the custom class with out-of-line constructor/query/destructor and
   an opaque implementation pointer. Validate pointer counts and values. Accept
   typed contiguous aligned NumPy buffers. Use `np.require` with C and aligned
   requirements: a C-contiguous array can still have an unaligned pointer.
   Match `np.float64` buffers with C++ `double*` and `np.int64` buffers with
   `std::int64_t*` from `<cstdint>`. Use const pointers for read-only inputs.
   Keep these types identical in declarations and definitions, including frame
   IDs, output indices and counts. In this locked Linux environment,
   `std::int64_t` is `long`; cppyy distinguishes its NumPy buffer format from
   `long long`, even though both have 64 bits. Do not substitute `long long*`
   for `std::int64_t*` merely because their sizes match.
   Copy input data once during construction.
   Write selected IDs, squared distances, counts and mean centroids directly
   to allocated output arrays in one native batch. No Python callbacks occur.
7. Define `NANOFLANN_NO_THREADS` before including the header. This experiment
   builds serially. The initial direct Cling probe hit unresolved std::once TLS
   symbols in the optional threaded tree-build path. Use
   `cppyy_kit.prebuild(code, decls=decls, ...)` followed by
   `cppyy_kit.cppdef_cached` with identical options and bodiless declarations.
   Templates then compile with the pinned native compiler, and Cling receives
   the small class declaration. Keep sources/artifacts under ignored `build/`.
8. Run template/first-call probes as a separate Python process. The supplied
   `pixi run probe` exercises the saved solution, not an unfinished candidate.
   cppyy rejects zero-length NumPy pointer arguments. A typed one-element
   sentinel is safe when explicit zero count guarantees no dereference.
9. Run the independent checks unchanged:

   ```bash
   NN_SOLUTION=build/agent_solution.py pixi run check
   sha256sum acceptance.py
   ```

   Acceptance covers 64 independent brute-force comparisons, ties across tree
   leaves, duplicates, radius 0, k greater than n, filtering beyond the nearest
   unfiltered k, empty arrays, invalid finite/range/shape inputs, input mutation
   and resizing, repeated retained queries, explicit close and repeated teardown.
   Also review the candidate to establish that actual nanoflann executes in
   native code. Numerical output alone cannot establish which implementation ran.

Use the [upstream README](https://github.com/jlblancoc/nanoflann/tree/v1.6.3)
for library design and its squared L2 distance convention. SciPy already exposes
[cKDTree.query](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.query.html)
and [query_ball_point](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.query_ball_point.html).
The custom operation combines metadata filtering and centroid accumulation with
tree traversal. Its usefulness comes from that composition and retained owned
storage. It does not require a full nanoflann kit.
