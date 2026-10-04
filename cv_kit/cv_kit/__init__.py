"""Use OpenCV's C++ API from Python through cppyy.

`cv_kit` wraps C++ ROS `sensor_msgs::msg::Image` buffers as `cv::Mat` objects,
extracts ORB features, and converts between Mats and NumPy arrays. It can use CUDA
OpenCV when the required module is available. Use it with `dbow_kit` to keep the
image and descriptor data in C++ through a DBoW2 query.

The kit exposes OpenCV through the `cv` namespace returned by `bringup_cv()` and
adds helpers for bringup, image buffers, ORB, and CUDA detection. `cv::Mat` cannot
be constructed from a Python integer address through cppyy, and OpenCV's `CV_8U`
type constants are preprocessor macros. Small `cppdef` helpers handle these cases.

Lifetime: `msg_to_mat()` and `mat_to_numpy(copy=False)` return views into existing
storage. Keep the message or Mat that owns the storage alive while using the view.
For ROS messages, use the Mat inside the owning callback."""
import ctypes
import os
import sys

import cppyy

import cppyy_kit

_CV_LIBS = (
    "libopencv_core.so",
    "libopencv_imgproc.so",
    "libopencv_features2d.so",
)

# Core + features2d headers. imgproc gives us cvtColor (color -> gray for ORB).
_CV_HEADERS = ("opencv2/core.hpp", "opencv2/imgproc.hpp", "opencv2/features2d.hpp")

# C++ glue compiled once at bringup. Two things force this into C++:
#   1. cv::Mat's (rows, cols, type, void* data, step) constructor cannot be called
#      from Python -- cppyy will not convert a Python int to the void* data arg. So
#      Mat-from-buffer must be a C++ helper taking the address as uintptr_t.
#   2. CV_8UC1 / CV_8U / ... are C preprocessor macros, invisible to cppyy. We
#      re-expose the handful we need as real constants.
# The keypoint/descriptor extractors also live here so the per-frame marshaling is
# a single C++ memcpy/loop rather than a Python per-element crossing.
_CPP_GLUE = r"""
namespace rclcppyy_cvkit {
  const int CV_8U_    = CV_8U;
  const int CV_8UC1_  = CV_8UC1;
  const int CV_8UC3_  = CV_8UC3;
  const int CV_8UC4_  = CV_8UC4;
  const int CV_16UC1_ = CV_16UC1;

  // Address of a message's byte buffer (sensor_msgs::msg::Image::data is a
  // std::vector<uint8_t>); used to alias it as a Mat with no copy.
  inline uintptr_t vec_data_addr(const std::vector<uint8_t>& v) {
    return reinterpret_cast<uintptr_t>(v.data());
  }
  inline size_t vec_size(const std::vector<uint8_t>& v) { return v.size(); }

  // Wrap an existing byte buffer as a Mat of the given cv type WITHOUT copying.
  inline cv::Mat mat_from_buffer(int rows, int cols, int type, uintptr_t data, size_t step) {
    return cv::Mat(rows, cols, type, reinterpret_cast<void*>(data), step);
  }
  inline uintptr_t mat_data_addr(const cv::Mat& m) {
    return reinterpret_cast<uintptr_t>(m.data);
  }
  inline size_t mat_step(const cv::Mat& m) { return m.step; }
  inline size_t mat_elem_size(const cv::Mat& m) { return m.elemSize(); }
  inline void copy_mat_to_vec(const cv::Mat& m, std::vector<uint8_t>& out) {
    const size_t row_bytes = static_cast<size_t>(m.cols) * m.elemSize();
    out.resize(static_cast<size_t>(m.rows) * row_bytes);
    for (int row = 0; row < m.rows; ++row) {
      std::memcpy(out.data() + static_cast<size_t>(row) * row_bytes,
                  m.ptr(row), row_bytes);
    }
  }

  // Copy an Nx32 CV_8U descriptor Mat into a caller-owned (N*32) byte buffer.
  inline void copy_u8_mat(const cv::Mat& m, uintptr_t dst) {
    cv::Mat mc = m.isContinuous() ? m : m.clone();
    std::memcpy(reinterpret_cast<void*>(dst), mc.data,
                static_cast<size_t>(mc.rows) * mc.cols * mc.elemSize());
  }
  // Copy keypoint (x,y) pairs into a caller-owned (N*2) float buffer.
  inline void keypoints_xy(const std::vector<cv::KeyPoint>& kps, uintptr_t dst) {
    float* d = reinterpret_cast<float*>(dst);
    for (size_t i = 0; i < kps.size(); ++i) { d[2*i] = kps[i].pt.x; d[2*i+1] = kps[i].pt.y; }
  }
}
"""

# encoding -> (glue constant name, bytes-per-pixel). Covers the encodings a mono/
# color camera or the dataset publisher emit. rgb8/bgr8 differ only in channel
# order (irrelevant to the Mat wrap; convert with cvtColor if you care).
_ENCODING = {
    "mono8": ("CV_8UC1_", 1), "8UC1": ("CV_8UC1_", 1),
    "bgr8": ("CV_8UC3_", 3), "rgb8": ("CV_8UC3_", 3), "8UC3": ("CV_8UC3_", 3),
    "bgra8": ("CV_8UC4_", 4), "rgba8": ("CV_8UC4_", 4),
    "mono16": ("CV_16UC1_", 2), "16UC1": ("CV_16UC1_", 2),
}

_CV = None
_DONE = False
_CUDA = None  # tri-state: None = not yet probed


def _conda():
    return os.environ["CONDA_PREFIX"]


def bringup_cv():
    """Bring up OpenCV under cppyy and return the ``cv`` namespace. Idempotent.

    Adds ``$CONDA_PREFIX/include/opencv4``, JIT-includes core/imgproc/features2d,
    loads the ``libopencv_*.so`` set so calls resolve without ``LD_LIBRARY_PATH``,
    and compiles the Mat<->buffer glue. Use OpenCV's own API on the returned
    namespace directly (``cv.ORB.create()``, ``cv.cvtColor``, ...).
    """
    global _CV, _DONE
    if _DONE:
        return _CV
    conda = _conda()
    inc = os.path.join(conda, "include", "opencv4")
    if not os.path.isdir(inc):
        raise RuntimeError(
            "OpenCV headers not found at %s. Install the vision env: "
            "pixi install -e vision" % inc)
    cppyy.add_include_path(inc)
    for header in _CV_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_CV_LIBS, [os.path.join(conda, "lib")])
    cppyy.cppdef(_CPP_GLUE)
    _CV = cppyy.gbl.cv
    _DONE = True
    return _CV


def cuda_available():
    """Whether this OpenCV build has the CUDA features2d module (``cv::cuda::ORB``).

    Auto-detected once: the conda-forge OpenCV has **no** CUDA build, so this
    returns False here and the CPU ``cv::ORB`` path is used. A user-supplied
    CUDA-enabled ``libopencv`` (e.g. JetPack, or a self-built OpenCV with
    ``-DWITH_CUDA``) makes it True with no other change to this kit. Detection is
    by header presence (``opencv2/cudafeatures2d.hpp``) plus a live
    ``cv::cuda::getCudaEnabledDeviceCount() > 0`` check.
    """
    global _CUDA
    if _CUDA is not None:
        return _CUDA
    header = os.path.join(_conda(), "include", "opencv4", "opencv2", "cudafeatures2d.hpp")
    if not os.path.isfile(header):
        _CUDA = False
        return _CUDA
    try:
        bringup_cv()
        cppyy.include("opencv2/core/cuda.hpp")
        cppyy.include("opencv2/cudafeatures2d.hpp")
        cppyy_kit.load_libraries(("libopencv_cudafeatures2d.so",), [os.path.join(_conda(), "lib")])
        _CUDA = int(cppyy.gbl.cv.cuda.getCudaEnabledDeviceCount()) > 0
    except Exception:
        _CUDA = False
    return _CUDA


def _glue():
    bringup_cv()
    return cppyy.gbl.rclcppyy_cvkit


def msg_to_mat(image_msg):
    """Wrap a C++ ROS Image data buffer as a `cv::Mat` without copying it.

The Mat retains the message that owns its data vector. Do not resize that vector
while using the Mat. Dimensions, row step and payload length must agree. For
16-bit encodings, recorded byte order must match the host. Unsupported encodings
or malformed buffers raise `ValueError`."""
    bringup_cv()
    glue = _glue()
    enc = str(image_msg.encoding)
    if enc not in _ENCODING:
        raise ValueError(
            "cv_kit.msg_to_mat: unsupported encoding %r (known: %s)"
            % (enc, ", ".join(sorted(_ENCODING))))
    const_name, bpp = _ENCODING[enc]
    height, width, step = int(image_msg.height), int(image_msg.width), int(image_msg.step)
    if not (0 <= height <= 2**31 - 1 and 0 <= width <= 2**31 - 1):
        raise ValueError("image dimensions must fit nonnegative C++ int values")
    if step < width * bpp:
        raise ValueError("image step is smaller than its pixel row")
    if bpp == 2 and step % 2:
        raise ValueError("16-bit image step must be a multiple of two bytes")
    if int(glue.vec_size(image_msg.data)) != height * step:
        raise ValueError("image data length must equal height * step")
    recorded_endian = image_msg.is_bigendian
    endian = ord(recorded_endian) if isinstance(recorded_endian, str) else int(recorded_endian)
    if endian not in (0, 1):
        raise ValueError("image is_bigendian must be 0 or 1")
    if bpp == 2 and bool(endian) != (sys.byteorder == "big"):
        raise ValueError("16-bit image byte order must match the host")
    type_int = int(getattr(glue, const_name))
    addr = int(glue.vec_data_addr(image_msg.data))
    if height and width and not addr:
        raise ValueError("nonempty image has a null data buffer")
    mat = glue.mat_from_buffer(height, width, type_int, addr, step)
    # Retain the message so the buffer remains alive with this Mat.
    cppyy_kit.keep_alive(mat, image_msg)
    return mat


def mat_to_msg(mat, msg=None, encoding=None):
    """Copy a ``cv::Mat`` into a C++ ``sensor_msgs::msg::Image``.

    The forward Image-to-Mat adapter aliases storage; this reverse direction is
    an explicit row-aware C++ copy because the message must own its output buffer.
    The selected encoding must match the pixel size. A 16-bit output records the
    host byte order.
    """
    glue = _glue()
    channels = int(mat.channels())
    elem_size = int(glue.mat_elem_size(mat))
    inferred = {1: "mono8", 3: "bgr8", 4: "bgra8"}.get(channels)
    selected = encoding or inferred
    if selected not in _ENCODING:
        raise ValueError("mat_to_msg requires a supported encoding")
    expected_size = _ENCODING[selected][1]
    if elem_size != expected_size:
        raise ValueError(
            "encoding %s expects %d bytes/pixel, Mat has %d" % (
                selected, expected_size, elem_size))
    if msg is None:
        from rclcpp_kit.bringup_rclcpp import add_ros2_include_paths
        add_ros2_include_paths()
        cppyy.include("sensor_msgs/msg/image.hpp")
        msg = cppyy.gbl.sensor_msgs.msg.Image()
    msg.height = int(mat.rows)
    msg.width = int(mat.cols)
    msg.encoding = selected
    msg.is_bigendian = int(expected_size == 2 and sys.byteorder == "big")
    msg.step = int(mat.cols) * elem_size
    glue.copy_mat_to_vec(mat, msg.data)
    return msg


_TYPE_ADAPTER = None


def type_adapter():
    """Return and register the Image-to-OpenCV adapter capability."""
    global _TYPE_ADAPTER
    if _TYPE_ADAPTER is None:
        from rclcpp_kit.type_adapter import (
            AdapterCapabilities,
            TypeAdapter,
            register_type_adapter,
        )
        capabilities = AdapterCapabilities(
            name="sensor_msgs.image/opencv.mat",
            ros_type="sensor_msgs::msg::Image",
            native_type="cv::Mat",
            to_native_copy="zero_copy",
            from_native_copy="cpp_copy",
            retains_source_owner=True,
            mutable_alias=True,
            limitations=(
                "zero-copy Mat aliases Image.data",
                "use the alias only while its owning message remains alive",
                "reverse conversion copies rows into message-owned storage",
            ),
        )
        _TYPE_ADAPTER = register_type_adapter(
            TypeAdapter(capabilities, msg_to_mat, mat_to_msg))
    return _TYPE_ADAPTER


def numpy_to_mat(array):
    """Wrap a contiguous 8-bit NumPy image (``(H,W)`` gray or ``(H,W,3)`` color) as
    a ``cv::Mat`` with **no copy**. The Mat aliases the NumPy buffer -- keep the
    array alive while you use the Mat. Used to feed synthetic frames to the C++ ORB
    without a round-trip through a ROS message."""
    import numpy as np
    arr = np.ascontiguousarray(array)
    if arr.dtype != np.uint8:
        raise ValueError("numpy_to_mat expects uint8, got %s" % arr.dtype)
    glue = _glue()
    if arr.ndim == 2:
        h, w = arr.shape
        type_int = int(glue.CV_8UC1_)
        step = w
    elif arr.ndim == 3 and arr.shape[2] == 3:
        h, w = arr.shape[:2]
        type_int = int(glue.CV_8UC3_)
        step = w * 3
    else:
        raise ValueError("numpy_to_mat expects (H,W) or (H,W,3) uint8, got %s" % (arr.shape,))
    mat = glue.mat_from_buffer(h, w, type_int, arr.ctypes.data, step)
    cppyy_kit.keep_alive(mat, arr)
    return mat


def mat_to_numpy(mat, copy=True):
    """Extract a two-dimensional unsigned 8-bit ``cv::Mat`` to NumPy.

    ``copy=True`` (default) is a private copy, safe after the Mat is gone.
    ``copy=False`` is a zero-copy view that **aliases** the Mat's storage (respecting
    its row ``step``) and retains the Mat and its pinned owners. Do not reallocate
    that storage while using the view. Returns ``(H,W)`` for 1 channel and
    ``(H,W,C)`` for multiple channels, including 3-channel and 4-channel images.
    Other pixel depths raise ``ValueError``. Empty Mats return empty arrays.
    """
    import numpy as np
    glue = _glue()
    if int(mat.depth()) != int(glue.CV_8U_):
        raise ValueError("mat_to_numpy requires unsigned 8-bit pixels")
    if int(mat.dims) not in (0, 2):
        raise ValueError("mat_to_numpy requires a two-dimensional Mat")
    rows, cols = int(mat.rows), int(mat.cols)
    ch = int(mat.channels())
    shape = (rows, cols, ch) if ch > 1 else (rows, cols)
    if not rows or not cols:
        return np.empty(shape, dtype=np.uint8)
    step = int(glue.mat_step(mat))
    addr = int(glue.mat_data_addr(mat))
    row_bytes = cols * ch
    if step < row_bytes or not addr:
        raise ValueError("nonempty Mat has an invalid row step or null data buffer")
    # A submatrix need not own padding after its final row.
    total = (rows - 1) * step + row_bytes
    buf = (ctypes.c_uint8 * total).from_address(addr)
    strides = (step, ch, 1) if ch > 1 else (step, 1)
    view = np.ndarray(shape, dtype=np.uint8, buffer=buf, strides=strides)
    if copy:
        return view.copy()
    cppyy_kit.keep_alive(buf, mat)
    return view


def to_gray(mat):
    """Return a single-channel 8-bit Mat. Return the input when it is already grayscale;
otherwise convert it to grayscale. ORB requires a grayscale image."""
    cv = bringup_cv()
    if int(mat.channels()) == 1:
        return mat
    out = cv.Mat()
    cv.cvtColor(mat, out, cv.COLOR_BGR2GRAY)
    return out


class OrbDetector:
    """Thin wrapper over ``cv::ORB`` (CPU) or ``cv::cuda::ORB`` (GPU) with a single
    branch point in ``detect_and_compute``, so the CPU/GPU choice is one ``if``.

    Mirrors OpenCV: construct via :func:`create_orb`, then call
    ``detect_and_compute(mat)`` -> ``(keypoints, descriptors)`` where ``keypoints``
    is a ``std::vector<cv::KeyPoint>`` and ``descriptors`` an ``Nx32 CV_8U`` Mat.
    """

    def __init__(self, orb, use_cuda):
        self._orb = orb
        self._use_cuda = use_cuda
        self._cv = bringup_cv()

    @property
    def use_cuda(self):
        return self._use_cuda

    def detect_and_compute(self, mat, mask=None):
        cv = self._cv
        kps = cppyy.gbl.std.vector[cv.KeyPoint]()
        desc = cv.Mat()
        m = mask if mask is not None else cv.Mat()
        if self._use_cuda:
            # GPU branch: upload, detect, download. Exercised only when a CUDA
            # OpenCV build is present (cuda_available()); coded so it activates
            # with no other change to callers.
            g_img = cv.cuda.GpuMat()
            g_img.upload(mat)
            g_desc = cv.cuda.GpuMat()
            self._orb.detectAndCompute(g_img, cv.cuda.GpuMat(), kps, g_desc)
            g_desc.download(desc)
        else:
            self._orb.detectAndCompute(mat, m, kps, desc)
        return kps, desc


def create_orb(nfeatures=1000, use_cuda=None):
    """Create an `OrbDetector`. With `use_cuda=None`, select CUDA when available.
`nfeatures` sets the target feature count. The default is 500."""
    cv = bringup_cv()
    want_cuda = cuda_available() if use_cuda is None else use_cuda
    if want_cuda:
        if not cuda_available():
            raise RuntimeError("cv_kit: use_cuda=True but no CUDA OpenCV build present")
        orb = cv.cuda.ORB.create(nfeatures)
    else:
        with cppyy_kit.first_use("cv_kit.create_orb", "cv_kit.warmup()"):
            orb = cv.ORB.create(nfeatures)
    return OrbDetector(orb, want_cuda)


def descriptors_to_numpy(desc):
    """Copy an ``Nx32 CV_8U`` ORB descriptor Mat to an ``(N,32)`` uint8 NumPy
    array (one C++ memcpy). Empty Mat -> ``(0,32)``."""
    import numpy as np
    glue = _glue()
    n, cols = int(desc.rows), int(desc.cols)
    if n == 0:
        return np.empty((0, 32), dtype=np.uint8)
    out = np.empty((n, cols), dtype=np.uint8)
    glue.copy_u8_mat(desc, out.ctypes.data)
    return out


def keypoints_to_numpy(kps):
    """Copy keypoint pixel coordinates to an ``(N,2)`` float32 NumPy array
    (x, y) -- for e.g. a Rerun ``Points2D`` overlay."""
    import numpy as np
    glue = _glue()
    n = int(kps.size())
    if n == 0:
        return np.empty((0, 2), dtype=np.float32)
    out = np.empty((n, 2), dtype=np.float32)
    glue.keypoints_xy(kps, out.ctypes.data)
    return out


def warmup(nfeatures=1000):
    """Run the first-use OpenCV JIT and ORB setup before processing live frames.
Call this once during initialization."""
    import numpy as np
    bringup_cv()

    def _exercise():
        img = np.zeros((64, 64), dtype=np.uint8)
        img[16:48, 16:48] = 255
        det = create_orb(nfeatures)
        kps, desc = det.detect_and_compute(numpy_to_mat(img))
        descriptors_to_numpy(desc)
        keypoints_to_numpy(kps)

    cppyy_kit.warmup(_exercise)
