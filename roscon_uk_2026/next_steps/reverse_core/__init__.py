"""Validated Python access to a separately compiled demonstration pose filter."""
from __future__ import annotations

import csv
import fcntl
import hashlib
import os
from pathlib import Path
import re
import shlex
import subprocess
import threading
from collections.abc import Mapping

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

ROOT = Path(__file__).resolve().parent
TRAINING_SEEDS = (0, 1, 2, 3)
HELD_OUT_SEEDS = (100, 101, 102, 103)


class FilterConfig(BaseModel):
    """Resolved settings. Time fields are seconds; positions use Cartesian metres."""
    model_config = ConfigDict(strict=True, frozen=True, extra="forbid", allow_inf_nan=False)
    tau_s: float = Field(default=0.08, gt=0, description="Smoothing time constant in seconds")
    max_gap_s: float = Field(default=0.5, gt=0, description="Reset gap threshold in seconds")
    frame_id: str = Field(default="world", min_length=1, max_length=128)

    @field_validator("tau_s", "max_gap_s", mode="before")
    @classmethod
    def reject_booleans(cls, value):
        if isinstance(value, bool):
            raise ValueError("time fields require numeric seconds, not booleans")
        return value

    @field_validator("frame_id")
    @classmethod
    def frame_name(cls, value):
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]*", value) is None:
            raise ValueError("frame_id must match [A-Za-z][A-Za-z0-9_/]*")
        return value

    @model_validator(mode="after")
    def time_relationship(self):
        if self.max_gap_s < self.tau_s:
            raise ValueError("max_gap_s must be >= tau_s")
        return self


def _resolve(config=None) -> FilterConfig:
    if config is None:
        return FilterConfig()
    if isinstance(config, FilterConfig):
        # Revalidate even model_construct() or model_copy(update=...) values.
        return FilterConfig.model_validate(config.model_dump())
    if isinstance(config, Mapping):
        return FilterConfig.model_validate(dict(config))
    raise TypeError("config must be a FilterConfig or mapping")


def export_config(config, path) -> Path:
    """Save a validated, resolved configuration in the native driver's fixed format."""
    config = _resolve(config)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"pose_filter_config_v1\ntau_s={config.tau_s:.17g}\n"
        f"max_gap_s={config.max_gap_s:.17g}\nframe_id={config.frame_id}\n",
        encoding="utf-8",
    )
    return path


def load_config(path) -> FilterConfig:
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    if len(lines) != 4 or lines[0] != "pose_filter_config_v1":
        raise ValueError("expected exactly four resolved configuration lines")
    keys = ("tau_s", "max_gap_s", "frame_id")
    values = {}
    for key, line in zip(keys, lines[1:]):
        prefix = key + "="
        if not line.startswith(prefix):
            raise ValueError("configuration key/order mismatch")
        value = line[len(prefix):]
        values[key] = value if key == "frame_id" else float(value)
    return FilterConfig.model_validate(values)


def build_native(force=False) -> dict[str, Path]:
    """Compile the independent library and driver. Cache by source and compiler."""
    compiler = shlex.split(os.environ.get("CXX", "c++"))
    version = subprocess.check_output([*compiler, "--version"], text=True)
    flags = ["-std=c++17", "-O3", "-Wall", "-Wextra", "-Werror"]
    digest = hashlib.sha256((version + repr(compiler) + repr(flags)).encode())
    for name in ("pose_filter.hpp", "pose_filter.cpp", "driver.cpp"):
        digest.update((ROOT / "native" / name).read_bytes())
    directory = ROOT / "build" / digest.hexdigest()[:16]
    directory.mkdir(parents=True, exist_ok=True)
    library = directory / "libreverse_pose.so"
    driver = directory / "pose_filter_driver"
    with (ROOT / "build" / "compile.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if force or not library.is_file() or not driver.is_file():
            library_tmp = directory / "libreverse_pose.so.tmp"
            driver_tmp = directory / "pose_filter_driver.tmp"
            subprocess.run([
                *compiler, *flags, "-fPIC", "-shared", str(ROOT / "native/pose_filter.cpp"),
                "-I", str(ROOT / "native"), "-o", str(library_tmp),
            ], check=True)
            library_tmp.replace(library)
            subprocess.run([
                *compiler, *flags, str(ROOT / "native/driver.cpp"),
                "-I", str(ROOT / "native"), "-L", str(directory), "-lreverse_pose",
                "-Wl,-rpath,$ORIGIN", "-o", str(driver_tmp),
            ], check=True)
            driver_tmp.replace(driver)
    return {"library": library, "driver": driver, "include_dir": ROOT / "native"}


_native = None
_load_lock = threading.Lock()


def native_namespace():
    """Load the compiled implementation and compatible public declarations once."""
    global _native
    with _load_lock:
        if _native is None:
            paths = build_native()
            # Local cppyy_kit comes from the explicit repository PYTHONPATH.
            import cppyy_kit
            import cppyy
            cppyy.add_include_path(str(paths["include_dir"]))
            cppyy_kit.load_libraries([str(paths["library"])])
            cppyy.include(str(paths["include_dir"] / "pose_filter.hpp"))
            _native = cppyy.gbl.reverse_demo
    return _native


def to_native_config(config):
    """Populate the existing native Config fields explicitly after validation."""
    resolved = _resolve(config)
    native = native_namespace().Config()
    native.time_constant_s = resolved.tau_s
    native.reset_gap_s = resolved.max_gap_s
    native.output_frame = resolved.frame_id
    native.validate()
    return native


def _arrays(timestamps_ns, positions_m):
    if not isinstance(timestamps_ns, np.ndarray) or not isinstance(positions_m, np.ndarray):
        raise TypeError("timestamps_ns and positions_m must be NumPy arrays")
    if timestamps_ns.dtype != np.dtype(np.int64) or positions_m.dtype != np.dtype(np.float64):
        raise TypeError("timestamps_ns must have native int64 dtype; positions_m native float64 dtype")
    if timestamps_ns.ndim != 1 or positions_m.shape != (len(timestamps_ns), 3):
        raise ValueError("expected timestamps_ns[N] and positions_m[N,3]")
    if not np.isfinite(positions_m).all():
        raise ValueError("positions_m must be finite")
    # Comparisons avoid int64 subtraction overflow at the timestamp boundaries.
    if np.any(timestamps_ns[1:] <= timestamps_ns[:-1]):
        raise ValueError("timestamps_ns must strictly increase")
    return (np.require(timestamps_ns, requirements=["C", "A"]),
            np.require(positions_m, requirements=["C", "A"]))


class PoseFilter:
    """Single-threaded owning wrapper. Calls borrow buffers only synchronously."""
    def __init__(self, config=None):
        self._config = _resolve(config)
        native_config = to_native_config(self._config)
        self._estimator = native_namespace().PoseEstimator(native_config)

    @property
    def config(self):
        return self._config

    def _require_open(self):
        if self._estimator is None:
            raise RuntimeError("PoseFilter is closed")
        return self._estimator

    def process(self, timestamps_ns, positions_m):
        estimator = self._require_open()
        timestamps, positions = _arrays(timestamps_ns, positions_m)
        state = estimator.snapshot()
        if len(timestamps) and state.initialized and int(timestamps[0]) <= state.last_timestamp_ns:
            raise ValueError("timestamps_ns must strictly increase across calls")
        output = np.empty((len(timestamps), 3), dtype=np.float64)
        if len(timestamps):
            estimator.process(timestamps, positions.reshape(-1), len(timestamps), output.reshape(-1))
        return output

    def reset(self):
        self._require_open().reset()

    def snapshot(self):
        state = self._require_open().snapshot()
        return {
            "initialized": bool(state.initialized),
            "last_timestamp_ns": int(state.last_timestamp_ns) if state.initialized else None,
            "filtered_position_m": [float(state.position_m[i]) for i in range(3)] if state.initialized else None,
            "samples_processed": int(state.samples_processed),
            "gap_resets": int(state.gap_resets),
            "config": self._config.model_dump(),
        }

    def close(self):
        # This proxy owns the allocation. Dropping its last reference destroys it.
        self._estimator = None

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()


def make_episode(seed=0, samples=500) -> dict:
    """Deterministic synthetic Cartesian motion and noisy observations."""
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    if isinstance(samples, bool) or not isinstance(samples, (int, np.integer)) or samples < 0:
        raise ValueError("samples must be a nonnegative integer")
    seed, samples = int(seed), int(samples)
    rng = np.random.default_rng(seed)
    intervals = rng.integers(8_000_000, 14_000_001, size=samples, dtype=np.int64)
    if samples >= 100:
        intervals[samples // 2] += 750_000_000
    timestamps = np.cumsum(intervals, dtype=np.int64) + 1_700_000_000_000_000_000
    if samples:
        seconds = (timestamps - timestamps[0]).astype(np.float64) * 1e-9
    else:
        seconds = np.empty(0)
    phase = rng.uniform(-np.pi, np.pi, size=3)
    frequency = rng.uniform(0.2, 0.65, size=3)
    amplitude = rng.uniform(0.08, 0.35, size=3)
    velocity = rng.uniform(-0.035, 0.035, size=3)
    truth = np.empty((samples, 3), dtype=np.float64)
    for a in range(3):
        truth[:, a] = amplitude[a] * np.sin(2*np.pi*frequency[a]*seconds + phase[a]) + velocity[a]*seconds
    # Smooth localized motion changes differ between independently seeded episodes.
    truth += rng.uniform(-0.08, 0.08, size=3) * np.tanh((seconds[:, None]-1.6) / 0.12)
    noise_std = rng.uniform(0.025, 0.065)
    noise = rng.normal(0, noise_std, size=(samples, 3))
    if samples:
        outliers = rng.random(samples) < 0.015
        noise[outliers] += rng.normal(0, 0.16, size=(int(outliers.sum()), 3))
    return {
        "timestamps_ns": timestamps, "positions_m": np.ascontiguousarray(truth + noise),
        "truth_m": truth, "episode_id": f"synthetic-pose-v1-seed-{seed}-n-{samples}",
    }


def write_episode_csv(episode, path) -> Path:
    timestamps, positions = _arrays(episode["timestamps_ns"], episode["positions_m"])
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(("timestamp_ns", "x_m", "y_m", "z_m"))
        for timestamp, position in zip(timestamps, positions):
            writer.writerow((int(timestamp), *(format(float(x), ".17g") for x in position)))
    return path


def driver_replay(config, timestamps_ns, positions_m, work_dir) -> np.ndarray:
    """Replay via the C++ executable without cppyy or Python filtering."""
    resolved = _resolve(config)
    timestamps, positions = _arrays(timestamps_ns, positions_m)
    directory = Path(work_dir)
    directory.mkdir(parents=True, exist_ok=True)
    config_path = export_config(resolved, directory / "resolved.cfg")
    input_path = write_episode_csv({"timestamps_ns": timestamps, "positions_m": positions}, directory / "input.csv")
    output_path = directory / "output.csv"
    paths = build_native()
    subprocess.run([str(paths["driver"]), str(config_path), str(input_path), str(output_path)],
                   check=True, capture_output=True, text=True)
    rows = []
    with output_path.open(newline="", encoding="utf-8") as result:
        reader = csv.reader(result)
        if next(reader) != ["timestamp_ns", "x_m", "y_m", "z_m"]:
            raise ValueError("unexpected driver output header")
        for index, row in enumerate(reader):
            if index >= len(timestamps) or int(row[0]) != int(timestamps[index]):
                raise ValueError("driver timestamp mismatch")
            rows.append([float(value) for value in row[1:]])
    if len(rows) != len(timestamps):
        raise ValueError("driver output row count mismatch")
    return np.array(rows, dtype=np.float64).reshape(len(timestamps), 3)
