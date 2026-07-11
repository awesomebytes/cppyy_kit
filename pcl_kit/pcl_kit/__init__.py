"""
pcl_kit -- drive the Point Cloud Library (PCL) from Python via cppyy.

PCL is a large templated C++ library with no maintained Python binding (the old
python-pcl shipped a fixed handful of point types and is unmaintained). This kit
is a thin cppyy glue layer that **mirrors the C++ API**: you construct
``pcl.PointCloud[pcl.PointXYZ]``, ``pcl.VoxelGrid[pcl.PointXYZ]``, call
``setInputCloud`` / ``setLeafSize`` / ``filter`` -- the same names and shapes as
the official PCL tutorials -- directly on the returned ``pcl`` namespace. Because
cppyy instantiates templates on demand from PCL's own headers, **any** point type
works, including ones no binding ever shipped (see the REPORT).

The kit's only job is to remove the cppyy friction that PCL has:
  * bringup -- locate the install, add the include paths (PCL, Eigen, and the ROS
    message headers), JIT-include the core + impl headers, and ``load_library``
    the ``libpcl_*.so`` set so symbols resolve without ``LD_LIBRARY_PATH``;
  * NumPy bridging -- the fast path copies an ``(N,3)``/``(N,4)`` float32 array
    into a ``PointCloud<PointXYZ>`` with a single ``std::memcpy`` in C++ (doing
    the per-point copy in a Python loop is ~90x slower -- see the REPORT);
  * the ROS bridge -- ``cloud_from_msg`` / ``msg_from_cloud`` wrap
    ``pcl::fromROSMsg`` / ``pcl::toROSMsg`` (pcl_conversions), so a C++
    ``sensor_msgs::msg::PointCloud2`` (e.g. straight off an rclcppyy subscription)
    goes into a PCL cloud and back with no Python-side per-point touch.

Minimal NumPy pipeline::

    import pcl_kit
    pcl = pcl_kit.bringup_pcl(with_ros=False)

    cloud = pcl_kit.cloud_from_numpy(points)           # (N,3) float32 -> cloud
    vox = pcl.VoxelGrid[pcl.PointXYZ]()                # PCL's own API, verbatim
    vox.setInputCloud(cloud.makeShared())
    vox.setLeafSize(0.05, 0.05, 0.05)
    out = pcl.PointCloud[pcl.PointXYZ]()
    vox.filter(out)
    down = pcl_kit.cloud_to_numpy(out)                 # cloud -> (M,3) float32

ROS money path (all data stays in C++)::

    pcl = pcl_kit.bringup_pcl()                        # with_ros=True (default)
    cloud = pcl_kit.cloud_from_msg(cpp_pointcloud2)    # fromROSMsg, no Python touch
    ... filter with PCL ...
    out_msg = pcl_kit.msg_from_cloud(out_cloud)        # toROSMsg

Notes / limits (v0):
    * The NumPy bridge is PointXYZ-only (the x,y,z float path). Other point types
      round-trip through ROS messages, or via your own ``cppyy.cppdef`` helper.
    * ``cloud_to_numpy(cloud, copy=False)`` returns a zero-copy view that *aliases*
      the cloud's storage -- keep the cloud alive while you use the view.
    * Custom point types work but must be declared with ``struct alignas(16)``
      (Cling rejects the trailing ``EIGEN_ALIGN16`` macro) -- see the REPORT.
"""
import ctypes
import glob
import os

import cppyy

import cppyy_kit

# The libpcl_*.so set whose symbols the common filters (VoxelGrid, PCLBase,
# getMinMax3D, ...) resolve against. cppyy discovers a symbol's owning library by
# scanning its search path at call time, so every .so we call into must be loaded
# explicitly (add_library_path alone does not resolve symbols).
_PCL_LIBS = (
    "libpcl_common.so",
    "libpcl_octree.so",
    "libpcl_kdtree.so",
    "libpcl_search.so",
    "libpcl_sample_consensus.so",
    "libpcl_filters.so",
)

# Core PCL headers + the template impl headers. Including the impls lets Cling
# instantiate PointCloud<T> / VoxelGrid<T> / PCLBase<T> for point types that were
# never precompiled into any .so (the on-demand claim -- custom types work).
_PCL_HEADERS = (
    "pcl/point_types.h",
    "pcl/point_cloud.h",
    "pcl/impl/pcl_base.hpp",
    "pcl/filters/voxel_grid.h",
    "pcl/filters/impl/voxel_grid.hpp",
)

# C++ glue compiled once at bringup. The NumPy <-> cloud copies live here on the
# C++ side: PointXYZ is a 16-byte aligned struct (x,y,z at offsets 0/4/8, 4 bytes
# padding), so an (N,4) float32 array maps 1:1 to the point storage (one memcpy),
# while (N,3) needs a strided per-point copy. Doing this in a Python loop is ~90x
# slower and building the storage from Python risks a cppyy SIGSEGV -- so it stays
# in C++, addressed via raw pointers passed as uintptr_t.
_CPP_GLUE = r"""
namespace rclcppyy_pclkit {
using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

// Non-inline throughout so these export as real symbols from the compile-cache .so
// (Python calls them via cloud_from_numpy / cloud_to_numpy).

// (N,4) contiguous float32 -> single std::memcpy (PointXYZ == 16 bytes).
void xyz_from_array4(uintptr_t src, std::size_t n, CloudXYZ& c) {
  c.resize(n);
  std::memcpy(c.points.data(), reinterpret_cast<const void*>(src), n * sizeof(pcl::PointXYZ));
  c.width = static_cast<std::uint32_t>(n); c.height = 1; c.is_dense = true;
}

// (N,3) contiguous float32 -> strided per-point copy (skips the padding lane).
void xyz_from_array3(uintptr_t src, std::size_t n, CloudXYZ& c) {
  const float* s = reinterpret_cast<const float*>(src);
  c.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    c.points[i].x = s[3 * i]; c.points[i].y = s[3 * i + 1]; c.points[i].z = s[3 * i + 2];
  }
  c.width = static_cast<std::uint32_t>(n); c.height = 1; c.is_dense = true;
}

// cloud -> caller-owned (N,3) float32 buffer (strided copy out).
void xyz_to_array3(const CloudXYZ& c, uintptr_t dst) {
  float* d = reinterpret_cast<float*>(dst);
  for (std::size_t i = 0; i < c.size(); ++i) {
    d[3 * i] = c.points[i].x; d[3 * i + 1] = c.points[i].y; d[3 * i + 2] = c.points[i].z;
  }
}

// Address of the (16-byte-stride) point storage, for a zero-copy NumPy view.
uintptr_t xyz_data_addr(CloudXYZ& c) {
  return reinterpret_cast<uintptr_t>(c.points.data());
}
}  // namespace rclcppyy_pclkit
"""

# Compile-cache extension: the VoxelGrid downsample, compiled once into the .so so
# the ~0.6 s first-use JIT of pcl::VoxelGrid<PointXYZ>'s template methods (measured;
# a freeze/PCH does NOT remove it) is paid at .so build time instead of on the first
# live filter call. The Python-driven mirror path (pcl.VoxelGrid[...] directly) still
# works and is what the fallback / prototyping surface uses. Kept out of _CPP_GLUE so
# the JIT fallback only cppdef's the small bridge glue.
_VOXEL_CODE = r"""
namespace rclcppyy_pclkit {
// Alias the input cloud (no copy) into the ConstPtr the filter wants.
void voxel_downsample(const CloudXYZ& in, float lx, float ly, float lz, CloudXYZ& out) {
  CloudXYZ::ConstPtr ptr(&in, [](const CloudXYZ*){});
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(ptr);
  vox.setLeafSize(lx, ly, lz);
  vox.filter(out);
}
}  // namespace rclcppyy_pclkit
"""

# Headers the standalone .so translation unit needs (the in-process cppdef inherits
# bringup's includes; a compiled .so does not). $CONDA_PREFIX/include (boost etc.),
# the PCL and Eigen dirs are added as -I via include_paths in _adopt_glue.
_CACHED_INCLUDES = (
    "#include <cstring>\n#include <cstdint>\n"
    "#include <pcl/point_types.h>\n#include <pcl/point_cloud.h>\n"
    "#include <pcl/filters/voxel_grid.h>\n#include <pcl/filters/impl/voxel_grid.hpp>\n"
)

# Bodiless declarations Cling needs to call into the .so on a cache hit.
_CACHED_DECLS = r"""
#include <cstdint>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
namespace rclcppyy_pclkit {
  using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
  void xyz_from_array4(uintptr_t, std::size_t, CloudXYZ&);
  void xyz_from_array3(uintptr_t, std::size_t, CloudXYZ&);
  void xyz_to_array3(const CloudXYZ&, uintptr_t);
  uintptr_t xyz_data_addr(CloudXYZ&);
  void voxel_downsample(const CloudXYZ&, float, float, float, CloudXYZ&);
}
"""

_PCL = None
_CORE_DONE = False
_ROS_DONE = False
# True once the compile-cache .so (bridge glue + voxel_downsample) is loaded; False
# falls back to plain cppdef of the bridge glue + the Python-driven VoxelGrid path.
_CACHED = False
_ADOPT_NOTICE_SHOWN = False


def _pcl_include_dir():
    """Locate the versioned PCL include dir (include/pcl-<major>.<minor>)."""
    conda = os.environ.get("CONDA_PREFIX", "")
    dirs = sorted(glob.glob(os.path.join(conda, "include", "pcl-*")))
    if not dirs:
        raise RuntimeError(
            "No PCL include dir (pcl-*) found under $CONDA_PREFIX/include. "
            "Install the pcl environment first: pixi install -e pcl"
        )
    return dirs[-1]


def _ensure_core():
    """Bring up core PCL (headers + libs + NumPy-bridge glue). Idempotent."""
    global _PCL, _CORE_DONE
    if _CORE_DONE:
        return
    conda = os.environ["CONDA_PREFIX"]
    cppyy.add_include_path(_pcl_include_dir())
    cppyy.add_include_path(os.path.join(conda, "include", "eigen3"))
    for header in _PCL_HEADERS:
        cppyy.include(header)
    # Load the .so set so cppyy resolves their symbols at call time (see cppyy_kit).
    cppyy_kit.load_libraries(_PCL_LIBS, [os.path.join(conda, "lib")])
    _adopt_glue(conda)
    _PCL = cppyy.gbl.pcl
    _CORE_DONE = True


def _adopt_glue(conda):
    """Compile-cache the bridge glue + voxel_downsample into a .so (first-use JIT
    paid once at build time), falling back to plain cppdef of the bridge glue + the
    Python VoxelGrid path when no compiler is available (capability/fallback). The
    cache is a pure optimisation, never a correctness dependency."""
    global _CACHED, _ADOPT_NOTICE_SHOWN
    if os.environ.get("CPPYY_KIT_NO_CACHE") == "1":
        _CACHED = False
        cppyy.cppdef(_CPP_GLUE)
        return
    source = _CACHED_INCLUDES + _CPP_GLUE + _VOXEL_CODE
    try:
        cppyy_kit.cppdef_cached(
            source, decls=_CACHED_DECLS, name="pcl_glue",
            include_paths=[os.path.join(conda, "include"), _pcl_include_dir(),
                           os.path.join(conda, "include", "eigen3")],
            library_paths=[os.path.join(conda, "lib")],
            libraries=["pcl_common", "pcl_filters", "pcl_octree", "pcl_kdtree",
                       "pcl_search", "pcl_sample_consensus"])
        _ = cppyy.gbl.rclcppyy_pclkit.voxel_downsample
        _CACHED = True
    except Exception as exc:  # no compiler / a compile-or-parse issue
        _CACHED = False
        if not _ADOPT_NOTICE_SHOWN and os.environ.get("RCLCPPYY_JIT_NOTICE", "1") != "0":
            _ADOPT_NOTICE_SHOWN = True
            cppyy_kit._compile._stderr(
                "[pcl_kit] compile cache unavailable (%s); using the JIT path "
                "(call pcl_kit.warmup()). Silence: RCLCPPYY_JIT_NOTICE=0." % exc)
        cppyy.cppdef(_CPP_GLUE)


def _ensure_ros():
    """Add the ROS message includes and pull in pcl_conversions (fromROSMsg /
    toROSMsg). Idempotent; separated from core so a NumPy-only pipeline can skip
    the ~2 s sensor_msgs JIT with ``bringup_pcl(with_ros=False)``."""
    global _ROS_DONE
    if _ROS_DONE:
        return
    # Reuse rclcppyy's ROS include-path machinery (adds every ament package's
    # include dir; cheap -- it registers paths, it does not JIT rclcpp).
    from rclcpp_kit.bringup_rclcpp import add_ros2_include_paths
    add_ros2_include_paths()
    cppyy.include("pcl_conversions/pcl_conversions.h")
    _ROS_DONE = True


def bringup_pcl(with_ros=True):
    """
    Bring up PCL under cppyy and return the ``pcl`` namespace. Idempotent.

    Adds the PCL / Eigen (and, when ``with_ros``, the ROS message) include paths,
    JIT-includes the core + impl headers, loads the ``libpcl_*.so`` set so calls
    resolve without ``LD_LIBRARY_PATH``, and compiles the NumPy-bridge C++ glue.
    With ``with_ros=True`` (default) it also pulls in pcl_conversions so
    ``cloud_from_msg`` / ``msg_from_cloud`` work.

    Returns ``cppyy.gbl.pcl`` -- use PCL's own API on it directly
    (``pcl.PointCloud``, ``pcl.VoxelGrid``, ...).
    """
    _ensure_core()
    if with_ros:
        _ensure_ros()
    return _PCL


def warmup(with_ros=False):
    """Front-load pcl_kit's one-time first-use JIT during init.

    The first NumPy->cloud->filter->NumPy pipeline JIT-compiles cppyy call
    wrappers for the glue and the VoxelGrid template methods (~0.45 s here); a
    freeze/PCH does not remove this. This runs one throwaway voxel pipeline on a
    tiny cloud so the wrappers are cached process-globally before your first real
    frame. Call once during init (e.g. a node's __init__). Pass with_ros=True to
    also warm the pcl_conversions path if you use cloud_from_msg / msg_from_cloud.

    On the compile-cache path the glue and the VoxelGrid are already compiled into
    the kit's ``.so``, so the only first-use left to move is the pcl_conversions
    round-trip (``with_ros=True``); warmup stays useful there and is cheap otherwise.
    """
    bringup_pcl(with_ros=with_ros)

    def _exercise():
        import numpy as np
        points = np.zeros((8, 3), dtype=np.float32)
        cloud = cloud_from_numpy(points)
        # Exercise the actual downsample path (cached helper, or the Python VoxelGrid
        # on the fallback path -- warmup front-loads whichever this env uses).
        out = voxel_downsample(cloud, 0.1)
        cloud_to_numpy(out)
        if with_ros:
            # also warm the pcl_conversions round-trip (toROSMsg / fromROSMsg).
            cloud_from_msg(msg_from_cloud(cloud))

    cppyy_kit.warmup(_exercise)


def _as_f32(array):
    """Coerce to a C-contiguous float32 ndarray (copies only if needed)."""
    import numpy as np
    return np.ascontiguousarray(array, dtype=np.float32)


def cloud_from_numpy(array):
    """
    Build a ``pcl::PointCloud<pcl::PointXYZ>`` from an ``(N,3)`` or ``(N,4)``
    array (any dtype; coerced to float32).

    Copy semantics: exactly **one** copy into PCL's aligned storage -- an ``(N,4)``
    float32 input is a single ``std::memcpy``; an ``(N,3)`` input is a strided
    per-point copy in C++. True zero-copy in is impossible: NumPy owns its buffer
    and PCL owns 16-byte-aligned point storage. Both paths are ~0.5 ms at N=100k
    (vs ~46 ms for a naive Python loop). Returns the cloud.
    """
    pcl = bringup_pcl(with_ros=False)
    arr = _as_f32(array)
    if arr.ndim != 2 or arr.shape[1] not in (3, 4):
        raise ValueError(f"expected an (N,3) or (N,4) array, got shape {arr.shape}")
    n = arr.shape[0]
    # First touch of the PointCloud<PointXYZ> template + the glue call JIT-compiles
    # cppyy wrappers; the notice points at pcl_kit.warmup() (the filter algorithms
    # add more first-use JIT that only a full warmup pipeline front-loads).
    with cppyy_kit.first_use("pcl_kit.cloud_from_numpy", "pcl_kit.warmup()"):
        cloud = pcl.PointCloud[pcl.PointXYZ]()
        glue = cppyy.gbl.rclcppyy_pclkit
        if arr.shape[1] == 4:
            glue.xyz_from_array4(arr.ctypes.data, n, cloud)
        else:
            glue.xyz_from_array3(arr.ctypes.data, n, cloud)
    return cloud


def cloud_to_numpy(cloud, copy=True):
    """
    Extract a ``PointCloud<PointXYZ>`` to an ``(N,3)`` float32 NumPy array.

    ``copy=True`` (default) returns a private strided copy (~0.35 ms at N=100k) --
    safe to keep after the cloud is gone. ``copy=False`` returns a zero-copy view
    that **aliases** the cloud's 16-byte-stride storage (the x,y,z columns of an
    ``(N,4)`` view); it is nearly free but the caller must keep ``cloud`` alive for
    as long as the view is used.
    """
    import numpy as np
    bringup_pcl(with_ros=False)
    glue = cppyy.gbl.rclcppyy_pclkit
    n = int(cloud.size())
    if copy:
        out = np.empty((n, 3), dtype=np.float32)
        if n:
            glue.xyz_to_array3(cloud, out.ctypes.data)
        return out
    addr = int(glue.xyz_data_addr(cloud))
    buf = (ctypes.c_float * (n * 4)).from_address(addr)
    # Pin the cloud on the backing buffer so the view does not outlive its
    # storage silently (see cppyy_kit.keep_alive).
    cppyy_kit.keep_alive(buf, cloud)
    view = np.frombuffer(buf, dtype=np.float32).reshape(n, 4)
    return view[:, :3]


def voxel_downsample(cloud, leaf, out=None):
    """Voxel-grid downsample a ``PointCloud<PointXYZ>`` -- the compile-cached fast
    path for the pcl_kit showcase's core op. ``leaf`` is a float (cubic voxel) or an
    ``(lx, ly, lz)`` triple; ``out`` is filled if given, else a fresh cloud is made.

    When the compile cache is active this runs a ``VoxelGrid<PointXYZ>`` compiled
    into the kit's ``.so`` -- the ~0.6 s first-use JIT of the filter's template
    methods is gone (measured ~594 ms -> ~5 ms). Without the cache it falls back to
    the Python-driven mirror path (``pcl.VoxelGrid[pcl.PointXYZ]`` directly), which
    is exactly what a user would write by hand and what ``warmup()`` front-loads.
    Returns the downsampled cloud.
    """
    pcl = bringup_pcl(with_ros=False)
    lx, ly, lz = (leaf, leaf, leaf) if isinstance(leaf, (int, float)) else leaf
    if out is None:
        out = pcl.PointCloud[pcl.PointXYZ]()
    if _CACHED:
        cppyy.gbl.rclcppyy_pclkit.voxel_downsample(cloud, lx, ly, lz, out)
    else:
        with cppyy_kit.first_use("pcl_kit.voxel_downsample", "pcl_kit.warmup()"):
            vox = pcl.VoxelGrid[pcl.PointXYZ]()
            vox.setInputCloud(cloud.makeShared())
            vox.setLeafSize(lx, ly, lz)
            vox.filter(out)
    return out


def cloud_from_msg(msg, point_type=None):
    """
    Convert a C++ ``sensor_msgs::msg::PointCloud2`` into a
    ``pcl::PointCloud<point_type>`` via ``pcl::fromROSMsg`` -- no Python-side
    per-point touch. ``point_type`` defaults to ``pcl.PointXYZ``; pass any PCL
    point type (e.g. ``pcl.PointXYZI``) to instantiate that specialization on
    demand. Returns the cloud.
    """
    pcl = bringup_pcl(with_ros=True)
    pt = point_type if point_type is not None else pcl.PointXYZ
    cloud = pcl.PointCloud[pt]()
    pcl.fromROSMsg(msg, cloud)  # C++ name, verbatim; PointT deduced from `cloud`
    return cloud


def msg_from_cloud(cloud, msg=None):
    """
    Convert a ``pcl::PointCloud<T>`` into a C++ ``sensor_msgs::msg::PointCloud2``
    via ``pcl::toROSMsg``. If ``msg`` is given it is filled in place (and its
    header preserved); otherwise a fresh message is created. Returns the message.
    """
    pcl = bringup_pcl(with_ros=True)
    if msg is None:
        msg = cppyy.gbl.sensor_msgs.msg.PointCloud2()
    pcl.toROSMsg(cloud, msg)  # C++ name, verbatim
    return msg
