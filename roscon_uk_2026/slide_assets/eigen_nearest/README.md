# Find the closest point with Eigen

From this repository checkout, run the same C++ function using plain cppyy or the kit's compiled-function cache:

```sh
pixi run python roscon_uk_2026/slide_assets/eigen_nearest/demo.py plain
pixi run python roscon_uk_2026/slide_assets/eigen_nearest/demo.py cached
```

Both return point index 86 for 10,000 seeded XYZ points and check the index against NumPy. The input is contiguous float64 XYZ data in sensor coordinates. The function scans once, maps coordinates without copying, and keeps the smallest squared distance. Empty input returns -1. The demo uses finite coordinates and a count within the range of int.

NumPy can also perform this operation efficiently. This example illustrates library integration and compilation reuse. It does not establish faster computation than NumPy.

To reproduce the startup measurements:

```sh
pixi run python roscon_uk_2026/slide_assets/eigen_nearest/benchmark.py
```

Recorded 21 October 2026 on this machine:

| Prepared path | Median whole-process time, 3 launches |
| --- | ---: |
| cppyy with its standard PCH | 0.496 s |
| cppyy with an Eigen PCH | 0.299 s |
| cppyy_kit compiled-function cache | 0.272 s |

First cache-building process: 1.362 s. Separate Eigen PCH preparation: 1.652 s.

Each sample starts a fresh Python process and includes imports, native preparation, one computation, reference validation and exit. OS caches are warm. The kit auto-PCH startup hook is disabled; inherited PCH settings are removed. Standard and function-cache paths use cppyy's existing standard PCH. Raw samples and conditions are in [results.json](results.json), with output in [terminal.txt](terminal.txt). These figures measure startup, not warmed computation throughput. Memory was not measured in this run.
