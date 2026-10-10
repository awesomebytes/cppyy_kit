"""Extract timestamped arrays from two explicitly supported typed ROS 2 schemas."""
from dataclasses import dataclass, field
import hashlib
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

try:
    from .schemas import IMAGE, POSE
except ImportError:
    from schemas import IMAGE, POSE

HERE = Path(__file__).resolve().parent
_native = None
SETUP_TIMINGS = {}


@dataclass
class PoseBatch:
    log_time_ns: np.ndarray
    publish_time_ns: np.ndarray
    header_time_ns: np.ndarray
    position_m: np.ndarray
    quaternion_xyzw: np.ndarray
    frame_id: str
    timings_ms: dict = field(default_factory=dict)


@dataclass
class ImageBatch:
    log_time_ns: np.ndarray
    publish_time_ns: np.ndarray
    header_time_ns: np.ndarray
    data: np.ndarray
    offsets: np.ndarray
    width: np.ndarray
    height: np.ndarray
    step: np.ndarray
    encoding: str
    frame_id: str
    timings_ms: dict = field(default_factory=dict)
    is_bigendian: np.ndarray = field(default_factory=lambda: np.empty(0, np.uint8))


def build_adapter():
    """Compile against installed mcap_vendor. No downloaded C++ sources."""
    started = time.perf_counter()
    compilation_ms = 0.0
    prefix = Path(sys.prefix)
    header = prefix / "include/mcap_vendor/mcap/reader.hpp"
    if not header.is_file():
        raise RuntimeError("use the existing roscon_uk_2026 Pixi ros environment")
    compiler = prefix / "bin/c++"
    flags = ["-std=c++17", "-O3", "-shared", "-fPIC"]
    compiler_version = subprocess.check_output([str(compiler), "--version"])
    identity = hashlib.sha256((HERE / "native.cpp").read_bytes() +
                              (HERE / "native.hpp").read_bytes() + header.read_bytes() +
                              (prefix / "lib/libmcap.so").read_bytes() + compiler_version +
                              str(flags).encode() + str(prefix).encode()).hexdigest()
    output = HERE / "build" / identity / "libtyped_mcap.so"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.with_suffix(".lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not output.exists():
            temporary = output.with_name("libtyped_mcap." + str(os.getpid()) + ".so")
            compiling = time.perf_counter()
            subprocess.run([str(compiler), *flags,
                            str(HERE / "native.cpp"), "-I" + str(prefix / "include/mcap_vendor"),
                            "-L" + str(prefix / "lib"), "-Wl,-rpath," + str(prefix / "lib"),
                            "-lmcap", "-o", str(temporary)], check=True)
            compilation_ms = 1000 * (time.perf_counter() - compiling)
            temporary.replace(output)
    SETUP_TIMINGS["adapter_build_or_cache_ms"] = 1000 * (time.perf_counter() - started)
    SETUP_TIMINGS["adapter_compile_ms"] = compilation_ms
    return output


def _load():
    global _native
    if _native is None:
        library = build_adapter()
        started = time.perf_counter()
        import cppyy_kit  # Apply the kit's existing PCH and compiler setup first.
        import cppyy
        cppyy.load_library(str(library))
        cppyy.include(str(HERE / "native.hpp"))
        _native = cppyy.gbl.typed_mcap
        SETUP_TIMINGS["cppyy_load_declarations_ms"] = 1000 * (time.perf_counter() - started)
    return _native


def _array(vector, dtype):
    # One bulk copy per column. Returned arrays own storage independently of C++.
    if not vector.size():
        return np.empty(0, dtype=dtype)
    return np.array(np.asarray(vector.data()).reshape(-1)[:vector.size()], dtype=dtype, copy=True)


def _extract(path, topic, start_ns, end_ns, image=False, headers_only=False):
    start = 0 if start_ns is None else int(start_ns)
    end = 2**64 - 1 if end_ns is None else int(end_ns)
    if not 0 <= start <= end <= 2**64 - 1:
        raise ValueError("expected 0 <= start_ns <= end_ns <= uint64 maximum")
    native = _load()
    result = native.extract(str(path), topic, start, end, image, headers_only, IMAGE if image else POSE)
    started = time.perf_counter()
    timestamps = [_array(result.log_ns, np.uint64), _array(result.publish_ns, np.uint64),
                  _array(result.header_ns, np.int64)]
    timings = {"container_io_parse_ms": result.container_ms, "cdr_decode_append_ms": result.decode_ms,
               "decompression_ms": 0.0}
    if image:
        batch = ImageBatch(*timestamps, _array(result.data, np.uint8), _array(result.offsets, np.uint64),
                           _array(result.width, np.uint32), _array(result.height, np.uint32),
                           _array(result.step, np.uint32), str(result.encoding), str(result.frame_id), timings)
        batch.is_bigendian = _array(result.is_bigendian, np.uint8)
    else:
        batch = PoseBatch(*timestamps, _array(result.position, np.float64).reshape(-1, 3),
                          _array(result.quaternion, np.float64).reshape(-1, 4), str(result.frame_id), timings)
    timings["numpy_owned_copy_ms"] = 1000 * (time.perf_counter() - started)
    return batch


def extract_poses(path, topic="/pose", start_ns=None, end_ns=None):
    """Filter by MCAP log time [start_ns,end_ns); preserve file order and duplicates."""
    return _extract(path, topic, start_ns, end_ns)


def extract_images(path, topic="/camera/image", start_ns=None, end_ns=None, headers_only=False):
    """Return rgb8/bgr8 pixels, or validate bounds without copying pixels."""
    return _extract(path, topic, start_ns, end_ns, True, headers_only)


def extract_image_headers(path, topic="/camera/image", start_ns=None, end_ns=None):
    return extract_images(path, topic, start_ns, end_ns, headers_only=True)


def count_speed_above(batch, threshold_m_s=0.3):
    """Count consecutive positive-time intervals with Cartesian speed > threshold."""
    if not np.isfinite(threshold_m_s) or threshold_m_s < 0:
        raise ValueError("speed threshold must be finite and nonnegative")
    timestamps = np.asarray(batch.log_time_ns)
    if timestamps.ndim != 1 or timestamps.dtype != np.uint64:
        raise ValueError("expected uint64 log_time_ns[N]")
    timestamps = np.require(timestamps, requirements=["C", "A"])
    positions = np.require(batch.position_m, dtype=np.float64, requirements=["C", "A"])
    if positions.shape != (len(timestamps), 3) or not np.isfinite(positions).all():
        raise ValueError("expected finite position_m[N,3]")
    if not len(timestamps):
        return 0
    return int(_load().count_speed_above(timestamps, positions.reshape(-1), len(timestamps), threshold_m_s))


def generate_fixture(path, samples=600, seed=7):
    """Write real PoseStamped/Image CDR with independently known synthetic truth."""
    from mcap_ros2.writer import Writer
    from mcap.writer import CompressionType
    if samples < 1:
        raise ValueError("samples must be positive")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    base_ns = 1_700_000_000_000_000_000
    metadata = {"episode_id": "typed-pose-seed-" + str(seed), "seed": seed, "samples": samples,
                "frame_id": "map", "position_unit": "m", "pose_period_ns": 10_000_000,
                "header_clock": "synthetic_unix_ns", "publish_clock": "synthetic_unix_ns",
                "log_clock": "synthetic_unix_ns", "log_minus_header_ns": 2_000_000,
                "truth_topic": "/truth", "pose_topic": "/pose", "image_topic": "/camera/image",
                "image_missing_header_interval_ns": [base_ns + 2_000_000_000, base_ns + 3_000_000_000],
                "truth_kind": "analytic synthetic trajectory", "compression": "none"}
    with Writer(str(path), compression=CompressionType.NONE) as writer:
        pose = writer.register_msgdef("geometry_msgs/msg/PoseStamped", POSE)
        image = writer.register_msgdef("sensor_msgs/msg/Image", IMAGE)
        writer._writer.add_metadata("episode", {k: json.dumps(v) for k, v in metadata.items()})
        for i in range(samples):
            ns = base_ns + i * 10_000_000
            t = i * 0.01
            truth = np.array([0.2 * t, 0.1 * np.sin(t), 0.03 * np.cos(2*t)])
            observed = truth + rng.normal(0, 0.002, 3)
            # Six known bias bursts support deterministic failure-window reports.
            if i % 100 in range(25, 45):
                observed[0] += 0.025 + 0.004 * (i // 100)
            header = {"stamp": {"sec": ns // 10**9, "nanosec": ns % 10**9}, "frame_id": "map"}
            for topic, xyz in [("/pose", observed), ("/truth", truth)]:
                message = {"header": header, "pose": {"position": dict(zip("xyz", xyz)),
                           "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}}}
                writer.write_message(topic, pose, message, ns + 2_000_000, ns, i)
            if i % 10 == 0 and not 200 <= i < 300:
                pixels = np.empty((12, 16, 3), dtype=np.uint8)
                pixels[:, :, 0] = i % 256
                pixels[:, :, 1] = np.arange(16, dtype=np.uint8)
                pixels[:, :, 2] = np.arange(12, dtype=np.uint8)[:, None]
                message = {"header": header, "height": 12, "width": 16, "encoding": "rgb8",
                           "is_bigendian": 0, "step": 48, "data": pixels.reshape(-1).tolist()}
                writer.write_message("/camera/image", image, message, ns + 2_000_000, ns, i // 10)
    metadata["mcap_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    sidecar = path.with_suffix(".json")
    sidecar.write_text(json.dumps(metadata, indent=2) + "\n")
    return sidecar


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--generate", action="store_true")
    parser.add_argument("--samples", type=int, default=600)
    args = parser.parse_args()
    if args.generate:
        generate_fixture(args.path, args.samples)
    poses = extract_poses(args.path)
    print(json.dumps({"samples": len(poses.log_time_ns), "frame": poses.frame_id,
                      "timings_ms": poses.timings_ms, "setup_ms": SETUP_TIMINGS}, indent=2))
