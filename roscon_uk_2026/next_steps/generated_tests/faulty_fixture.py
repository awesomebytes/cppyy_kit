"""Separate compiled C++ reset fault. The production implementation is unchanged."""

import hashlib
import os
from pathlib import Path
import shlex
import subprocess

from roscon_uk_2026.next_steps.reverse_core import PoseFilter, build_native, to_native_config

ROOT = Path(__file__).resolve().parent
_namespace = None


def faulty_namespace():
    global _namespace
    if _namespace is None:
        paths = build_native()
        compiler = shlex.split(os.environ.get("CXX", "c++"))
        key = hashlib.sha256()
        key.update(str(paths["library"]).encode())
        key.update(subprocess.check_output([*compiler, "--version"]))
        for name in ("faulty.hpp", "faulty.cpp"):
            key.update((ROOT / "native" / name).read_bytes())
        directory = ROOT / "build" / key.hexdigest()[:16]
        directory.mkdir(parents=True, exist_ok=True)
        library = directory / "libdeliberate_reset_fault.so"
        if not library.is_file():
            temporary = directory / "libdeliberate_reset_fault.so.tmp"
            subprocess.run([
                *compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                "-fPIC", "-shared", str(ROOT / "native" / "faulty.cpp"),
                "-I", str(paths["include_dir"]), "-I", str(ROOT / "native"),
                "-L", str(paths["library"].parent), "-lreverse_pose",
                "-Wl,-rpath," + str(paths["library"].parent), "-o", str(temporary),
            ], check=True)
            temporary.replace(library)
        import cppyy
        cppyy.load_library(str(library))
        cppyy.include(str(ROOT / "native" / "faulty.hpp"))
        _namespace = cppyy.gbl.generated_test_fault
    return _namespace


class MissingResetFilter(PoseFilter):
    """Use the production Python adapter over the deliberate native fixture."""
    def __init__(self, config=None):
        super().__init__(config)
        self._estimator = faulty_namespace().MissingResetEstimator(to_native_config(self.config))
