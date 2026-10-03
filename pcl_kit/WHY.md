# Why pcl_kit: PCL from Python via cppyy

`pcl_kit` lets you drive [PCL](https://pointclouds.org), the standard C++ point
cloud library, from Python: the real C++ code owns the cloud and runs the
filters, while Python only orchestrates and moves data in/out as NumPy arrays or
ROS 2 `PointCloud2` messages. PCL has **no maintained Python binding** (the old
`python-pcl` shipped a fixed handful of point types and is abandoned), so this
capability does not otherwise exist, and because it is the *same* C++ library
reading the *same* headers, cppyy instantiates templates for *any* point type on
demand, including ones no binding ever shipped.

This doc explains what that gives you over the C++ workflow, and the two distinct
ways to use it. For the API, see [SKILL.md](SKILL.md); for the feasibility
evidence, copy accounting, gaps, and benchmarks, see [REPORT.md](REPORT.md).

---

## Compare the official VoxelGrid tutorial in C++ and Python

On the left, the official
["Downsampling a PointCloud using a VoxelGrid filter"](https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html)
tutorial from pointclouds.org, verbatim, **and its build system**. On the right,
the complete runnable file this repo ships, `scripts/pcl_kit_demos/d01_voxel_numpy.py`.
The filtering code, `VoxelGrid`, `setInputCloud`, `setLeafSize`, `filter`, is
identical; the Python drops the file I/O for a NumPy bridge and drops the entire
build system.

### C++, `voxel_grid.cpp` + `CMakeLists.txt` (official tutorial)

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main ()
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data from a .pcd file on disk.
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud);

  std::cerr << "PointCloud before filtering: "
            << cloud->width * cloud->height << " data points." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: "
            << cloud_filtered->width * cloud_filtered->height
            << " data points." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered);
  return (0);
}
```

```cmake
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(voxel_grid)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (voxel_grid voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})
```

This C++ example still needs the `CMakeLists.txt` above, a
`cmake . && make` (or `colcon build`) to configure/compile/link against a dozen
`libpcl_*` libraries, **and** the `table_scene_lms400.pcd` sample file on disk,
before you can execute the binary.

### Python, `d01_voxel_numpy.py` (pcl_kit, shipped in this repo)

```python
#!/usr/bin/env python
"""A NumPy point cloud goes into a PCL VoxelGrid and comes back out as NumPy."""
import numpy as np

import pcl_kit

pcl = pcl_kit.bringup_pcl(with_ros=False)          # NumPy-only, skip the ROS JIT

points = np.random.default_rng(0).random((100_000, 3), dtype=np.float32)
cloud = pcl_kit.cloud_from_numpy(points)           # one C++ memcpy into the cloud

vox = pcl.VoxelGrid[pcl.PointXYZ]()                # PCL's own API, verbatim
vox.setInputCloud(cloud.makeShared())
vox.setLeafSize(0.05, 0.05, 0.05)
downsampled = pcl.PointCloud[pcl.PointXYZ]()
vox.filter(downsampled)

out = pcl_kit.cloud_to_numpy(downsampled)          # strided copy back to (M,3)
print(f"input:  {points.shape[0]} points")
print(f"output: {out.shape[0]} points after 0.05 m VoxelGrid")
```

Run it directly: `pixi run -e pcl demo-pcl-voxel`. Prints
`input: 100000 points / output: 8000 points after 0.05 m VoxelGrid`.

### Differences shown in the examples

- **No compile step, no CMake.** The C++ program needs a `CMakeLists.txt`,
  `find_package(PCL)`, and a build that links a dozen `libpcl_*` libraries before
  it can run; the Python file runs the instant you invoke it. The only startup
  cost is a one-time **~1.3 s** cppyy bringup (JIT-including the headers + loading
  the `.so` set), and only what you touch is JIT-compiled.
- **No wrapper, no codegen, no fixed type list.** Nothing is generated.
  `VoxelGrid`, `setInputCloud`, `setLeafSize`, `filter` are PCL's own names, the
  Python reads like the C++. And because cppyy instantiates templates from the
  headers on demand, `pcl.VoxelGrid[pcl.PointXYZINormal]` works even though no
  binding ever shipped that specialization (see [REPORT.md](REPORT.md) section 1).
- **NumPy input and output.** The bridge copies an `(N,4)` float32 array with
  `std::memcpy` and an `(N,3)` array with a strided C++ loop. Both paths took about
  0.5 ms for 100k points, compared with about 46 ms for the Python loop. The input
  is copied into PCL-owned point storage. The output can use a zero-copy view (see the
  copy-accounting table in [REPORT.md](REPORT.md) section 3).
- **Same library, full ecosystem.** It is the same `libpcl_*.so`, so every PCL
  algorithm (filters, KdTree, segmentation, registration, features) is reachable
  the moment you `cppyy.include` its header, no per-feature binding work.

The compiled C++ program has no JIT startup cost and uses static type checking. It
also reads and writes PCD files. pcl_kit does not yet expose PCD I/O; its demos use
NumPy or ROS messages. See [REPORT.md](REPORT.md) section 5 for other gaps.

---

## Two ways to use it

### Mode A: use PCL from Python
Prototype point-cloud processing with NumPy arrays crossing into real PCL
algorithms from Python. The maintained C++ PCL library runs on point types selected
by the script. `d01_voxel_numpy.py` shows this mode. It supports filter experiments,
parameter changes, and tests with short edit-run cycles.

### Mode B: keep the cloud in C++ in a ROS 2 pipeline
The ROS path. A leaf subscribes to a `sensor_msgs/PointCloud2` **as a C++
message** (via rclcppyy), hands it straight to `pcl::fromROSMsg`, runs a PCL filter,
and republishes via `pcl::toROSMsg`, **Python never touches a point**.
`scripts/pcl_kit_demos/d02_ros_pipeline.py` is the showcase: a self-contained
synthetic 100k-point publisher at 10 Hz plus a subscribe -> VoxelGrid -> republish
pipeline, all in one process, all data in C++ end to end.

This mode uses C++ PCL to process ROS messages from Python without generating a
wrapper or building the demo. The comparison, `d03_baseline_rclpy.py`, writes the
same pipeline the way you would without the kit (deserialize to NumPy, voxel-
downsample in NumPy, re-serialize). `bench_pcl_pipeline.py` runs both:

| Variant | avg latency | CPU% @10 Hz | user LOC |
|---|--:|--:|--:|
| **pcl_kit (C++ end-to-end)** | **3.8 ms** | **6.9 %** | 76 |
| rclpy + NumPy baseline | 56.5 ms | 60.5 % | 77 |

In this measurement, pcl_kit had about 15x lower latency and used about 9x less
CPU. The demos had similar line counts. These provisional measurements used a
shared machine; see [REPORT.md](REPORT.md) section 4.

---

## Advantages of the cppyy approach

Grounded in the spike's measured numbers (see [REPORT.md](REPORT.md) sections 3-4):

- **No code generation, no wrapper build.** `python x.py` is the whole workflow;
  contrast the tutorial's `find_package(PCL)` + `cmake && make`. Bringup is a
  one-time ~1.3 s JIT (~3.3 s if you also pull in the ROS message headers).
- **Header-following, so it tracks the installed version.** cppyy reads PCL's own
  headers at runtime, so pcl_kit matches the installed PCL version (1.15 here). No hand-maintained binding to drift out of sync.
- **On-demand template instantiation over any point type.** `PointCloud<T>` /
  `VoxelGrid<T>` are instantiated the moment you use them, for stock *and* custom
  `T`, the fixed-type-list limitation of old bindings is gone.
- **The cloud stays in C++ across the ROS boundary.** Straight off an rclcppyy
  subscription, the message is a C++ `PointCloud2`; `fromROSMsg`/`toROSMsg` keep
  every point in C++. Python pays a language boundary only where you put a Python
  line, never per point.
- **A prototype-to-native lowering path.** As with bt_kit, this is the L0 rung:
  prototype with cppyy JIT today; later *freeze* to a precompiled dictionary (no
  JIT at startup); later *lower* hot paths to compiled C++, same PCL calls, same
  structure. (L1/L2 are the research direction, see PLAN.md.)

---

## Limits

pcl_kit is a v0 spike. The NumPy bridge is PointXYZ-only (other fields round-trip
through ROS messages or your own `cppdef`), custom point types need a small C++
`cppdef` block (with `alignas(16)`, not the trailing `EIGEN_ALIGN16` macro, Cling
rejects it), only VoxelGrid's template-impl header is pre-included, and a few raw
cppyy operations (Python per-point loops, building aligned storage from Python)
are slow or segfault, which is exactly why the kit keeps cppyy behind a curated
surface. The full list is in [REPORT.md](REPORT.md) section 5.
