# pcl_kit — cheat sheet for a coding agent

You are writing Python that drives the **Point Cloud Library (PCL)** — a C++ point
cloud library — through `pcl_kit`. The kit **mirrors PCL's C++ API**:
`bringup_pcl()` returns the real `pcl` namespace and you use `pcl.PointCloud`,
`pcl.VoxelGrid`, `setInputCloud`, `setLeafSize`, `filter` exactly as in the PCL
tutorials. The kit only removes the cppyy friction (bringup, NumPy<->cloud copies,
the ROS message bridge). You do **not** need to know cppyy.

(For *why* this exists and the C++-vs-Python comparison, see [WHY.md](WHY.md).)

**Requires** the `pcl` pixi env: `pixi run -e pcl python your_script.py`.

**Golden rules**
- Call `pcl = pcl_kit.bringup_pcl()` once; it returns the `pcl` namespace.
  Pass `with_ros=False` if you only need NumPy (skips the ~1.9 s pcl_conversions
  JIT). Bringup is idempotent.
- Instantiate templates with subscript: `pcl.PointCloud[pcl.PointXYZ]`,
  `pcl.VoxelGrid[pcl.PointXYZ]` — **any** point type works on demand
  (`pcl.PointXYZI`, `pcl.PointXYZRGB`, `pcl.PointNormal`, ...).
- A filter needs a shared pointer as input: `vox.setInputCloud(cloud.makeShared())`.
- Move bulk data with the kit's bridges, never a Python per-point loop.
- Keep the `cloud` referenced while you use a `copy=False` NumPy view of it.

---

## Pattern 1 — NumPy cloud -> VoxelGrid -> NumPy  (the minimal path)
*Use for:* filtering/processing a cloud you have as a NumPy `(N,3)` float array.

```python
import numpy as np
import pcl_kit
pcl = pcl_kit.bringup_pcl(with_ros=False)

points = np.random.rand(100_000, 3).astype(np.float32)   # (N,3) or (N,4)
cloud = pcl_kit.cloud_from_numpy(points)                 # ONE C++ memcpy

vox = pcl.VoxelGrid[pcl.PointXYZ]()                      # PCL's own API
vox.setInputCloud(cloud.makeShared())
vox.setLeafSize(0.05, 0.05, 0.05)
out = pcl.PointCloud[pcl.PointXYZ]()
vox.filter(out)

down = pcl_kit.cloud_to_numpy(out)                       # (M,3) float32, safe copy
print(cloud.size(), "->", out.size())
```
`cloud_from_numpy` accepts `(N,3)` (strided copy) or `(N,4)` (single memcpy; the
4th column is the padding lane). `cloud_to_numpy(out, copy=False)` returns a
near-free zero-copy view instead — but it aliases the cloud's storage, so keep
`out` alive while you use the view.

---

## Pattern 2 — ROS PointCloud2 pipeline, cloud stays in C++  (the money path)
*Use for:* a ROS 2 node that filters `sensor_msgs/PointCloud2` without ever
materializing points in Python. Subscribe via rclcpp_kit so the callback gets the
**C++** message.

```python
import os; os.environ.setdefault("ROS_DOMAIN_ID", "43")
import rclcpp_kit
import pcl_kit

rclcpp = rclcpp_kit.bringup_rclcpp()
pcl = pcl_kit.bringup_pcl()                 # with_ros=True (default)
from sensor_msgs.msg import PointCloud2

node = rclcpp.Node("cloud_filter")
out_pub = node.create_publisher(PointCloud2, "points_out", 10)

def on_cloud(msg):                          # msg is a C++ PointCloud2
    cloud = pcl_kit.cloud_from_msg(msg)     # pcl::fromROSMsg, no Python per-point
    vox = pcl.VoxelGrid[pcl.PointXYZ]()
    vox.setInputCloud(cloud.makeShared())
    vox.setLeafSize(0.05, 0.05, 0.05)
    out = pcl.PointCloud[pcl.PointXYZ]()
    vox.filter(out)
    out_pub.publish(pcl_kit.msg_from_cloud(out))   # pcl::toROSMsg

sub = node.create_subscription(PointCloud2, "points_in", on_cloud, 10)
executor = rclcpp.executors.SingleThreadedExecutor()
executor.add_node(node)
executor.spin()                             # or spin_some() in a loop
```
`cloud_from_msg(msg, point_type=pcl.PointXYZI)` instantiates a different point type
on demand. `msg_from_cloud(cloud, msg=existing)` fills an existing message in place.
See `scripts/pcl_kit_demos/d02_ros_pipeline.py` for the full self-contained showcase.

---

## Pattern 3 — a point type no binding ever shipped  (on-demand templates)
*Use for:* stock PCL point types beyond XYZ. Just subscript with the type; the ROS
bridge takes a `point_type` argument.

```python
pcl = pcl_kit.bringup_pcl()
cloud = pcl.PointCloud[pcl.PointXYZINormal]()      # instantiated on demand
# ... or straight off a ROS message with intensity:
cloud_i = pcl_kit.cloud_from_msg(msg, point_type=pcl.PointXYZI)
```

---

## Pattern 4 — a fully custom point type  (cppdef)
*Use for:* a struct with your own fields (e.g. a LiDAR point with `ring`). Two
rules the REPORT nailed down: use `struct alignas(16)` (**not** the trailing
`} EIGEN_ALIGN16;` macro — Cling rejects it), and include the template *impl*
headers so the filter instantiates for your type.

```python
import cppyy
import pcl_kit
pcl = pcl_kit.bringup_pcl(with_ros=False)

cppyy.cppdef(r"""
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/pcl_base.hpp>                  // needed for PCLBase<T>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>        // needed for VoxelGrid<T>

struct alignas(16) MyLidarPoint {                 // alignas PREFIX, not EIGEN_ALIGN16
  PCL_ADD_POINT4D;                                // x, y, z (+ padding)
  float intensity;
  std::uint16_t ring;
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyLidarPoint,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)(std::uint16_t, ring, ring))
""")

MyPoint = cppyy.gbl.MyLidarPoint
cloud = pcl.PointCloud[MyPoint]()
vox = pcl.VoxelGrid[MyPoint]()                    # works over the custom type
```

---

## Pattern 5 — compile-cached VoxelGrid, and warmup for the rest
*Use for:* any node/loop whose **first** frame must not be a latency outlier. The
dominant first-frame cost is cppyy JIT-instantiating PCL's `VoxelGrid<PointXYZ>`
(~0.6 s). `pcl_kit.voxel_downsample(cloud, leaf)` runs a `VoxelGrid` **compiled once
into the kit's `.so`** (`cppdef_cached`), so its first use is ~5 ms and persistent —
prefer it over building `pcl.VoxelGrid[...]` by hand in a hot path. The showcase
frame-0 drops ~681 ms → ~88 ms. First run on a machine pays a one-time ~3 s `.so`
build; no compiler → it falls back to the Python VoxelGrid path (`pcl_kit._CACHED`).

```python
import numpy as np, pcl_kit

pcl_kit.bringup_pcl()                       # caches the glue + voxel_downsample here
cloud = pcl_kit.cloud_from_numpy(pts)
out = pcl_kit.voxel_downsample(cloud, 0.05)  # compiled VoxelGrid; ~5 ms first use
```

`warmup(with_ros=True)` still front-loads what the cache doesn't yet cover — the
`pcl_conversions` `toROSMsg`/`fromROSMsg` round-trip (the same cacheable pattern, a
compiled conversion helper, is the next step). Pass `with_ros=False` for the
NumPy-only path. See docs/kits/COMMON_PATTERNS.md §23 (cache) and FREEZE.md §4.

---

## Gotchas (short version)
- **Don't** convert clouds with a Python per-point loop — it is ~90x slower than
  the kit's C++ memcpy and building the aligned storage from Python can **segfault**
  the process. Use `cloud_from_numpy` / `cloud_to_numpy`.
- **Don't** spell a custom point struct with `} EIGEN_ALIGN16;` — Cling parse-errors
  and a failed `cppdef` can crash on transaction revert. Use `struct alignas(16)`.
- For a filter over a **novel** point type, include its `impl/*.hpp` (e.g.
  `pcl/filters/impl/voxel_grid.hpp`) or you get unresolved-symbol errors. VoxelGrid's
  is pre-included by the kit.
- A filter's input is a shared pointer: `setInputCloud(cloud.makeShared())`.
- `cloud_to_numpy(cloud, copy=False)` aliases PCL memory — keep the cloud alive.
- Use `bringup_pcl(with_ros=False)` for NumPy-only work to skip the ROS JIT; the
  ROS bridges (`cloud_from_msg` / `msg_from_cloud`) require `with_ros=True`.
