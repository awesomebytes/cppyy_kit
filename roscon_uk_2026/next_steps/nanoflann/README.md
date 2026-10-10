# Select nearby observations from different frames

Find nearby recorded Cartesian positions while excluding observations from
the query's neighboring frames. Require a minimum observation confidence.
Return the selected positions' centroid for a local correspondence estimate.
The index stays in native memory across query batches.

Setup and commands below require this repository checkout. They use the locked
Linux x86-64 Pixi environment and the checkout's `cppyy_kit`. They are not
installed-package instructions.

Current `require` serializes fetches for the same library and verifies offline
cache reuse. Compilation helpers coordinate writes to the same artifact.
Run benchmark processes sequentially so their CPU and memory measurements do
not compete. Concurrent query/close is outside the index's contract.

```bash
cd roscon_uk_2026/next_steps/nanoflann
pixi install --locked
pixi run probe
pixi run check
pixi run demo
```

The demo indexes four observations in metres. It excludes frame IDs within two
of query frame 0, rejects confidence below 0.5, and uses a 0.5 m search radius.
Only original point index 2 qualifies. Expected output:

```text
indices: [[2, -1]]
squared distances (m^2): [[0.04000000000000001, inf]]
distances (m): [[0.2, inf]]
selected counts: [1]
selected centroid (m): [[0.2, 0.0, 0.0]]
```

Use the saved implementation from a script in this directory:

```python
from index import Index

with Index([[0., 0., 0.], [.2, 0., 0.]], [0, 20], [1., .8]) as index:
    result = index.query([[0., 0., 0.]], [0], k=2,
                         frame_gap=2, min_confidence=.5, max_distance=.5)
    print(result["ids"])                # [[1, -1]]
    print(result["centroids"])          # [[0.2, 0.0, 0.0]]
```

`k` is an integer from 1 to 4096. Eligible observations have confidence greater
than or equal to `min_confidence`, Euclidean distance less than or equal to
`max_distance`, and absolute frame difference greater than `frame_gap`. Frame
gap -1 disables temporal exclusion. Frame IDs are integers in [0, 2**62].
Confidence and its threshold are finite values in [0, 1]. Coordinates must be
finite with absolute value at most 1e150. Radius is in [0, 1e150] or positive
infinity. These bounds keep squared-distance arithmetic finite.

The query uses nanoflann's L2 metric with approximation epsilon 0. Returned
distances are **squared** distances. Use `np.sqrt` to obtain distances. Results
are ordered by computed double squared distance, then original point index.
Exact equal-distance ties and duplicate positions use the smaller index,
including ties at the last selected neighbor and on the inclusive radius.
There is no tolerance-based tie grouping. Floating-point distance calculations
are not exact real-number geometry.

An empty index, or fewer than `k` eligible points, produces -1 index padding
and infinite distance padding. Counts specify the valid prefix. A query with no
eligible points has a NaN centroid. An empty query returns zero rows.

Construction copies positions, frames and confidence into native vectors. The
tree references those vectors. The class has no input resize or update API.
Python may mutate, resize or release its source arrays after construction.
Rebuild a new index when observations change. NumPy conversion accepts compatible
array-like values and normalizes dtype/layout. Strided and unaligned arrays are copied;
read-only contiguous query buffers can be read directly during the synchronous
call. Do not mutate query inputs from another thread during a call.

Outputs are newly allocated NumPy arrays. Native code writes into them before
returning. They remain valid after another query or index close. `close()` is
idempotent. Queries after close fail. Use a context manager for explicit cleanup.
Concurrent query/close and thread-safety are outside this experiment's contract.

Run `pixi run benchmark` for separate startup, adapter compilation, native copy
and tree build, warmed batch query, validation/allocation, and SciPy timings.
See [RESULTS.md](RESULTS.md) for measurements and limitations. The fixture uses
synthetic positions, not a verified recorded dataset.

For an agent task, explicitly read [LIBRARY_RECIPE.md](LIBRARY_RECIPE.md), or run
`pixi run guide`. The missing implementation is [skeleton.py](skeleton.py).
[PROMPT.txt](PROMPT.txt) is the exact task prompt. The unchanged
[acceptance.py](acceptance.py) compares a candidate selected through
`NN_SOLUTION` with an independent NumPy oracle. The saved solution is
[index.py](index.py) with [native.cpp](native.cpp). A fresh Sol high-reasoning
completion passed the unchanged checks; its scope and evidence are linked in
[RESULTS.md](RESULTS.md).
