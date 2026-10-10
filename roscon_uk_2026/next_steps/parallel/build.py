"""Build all baseline and library kernels with the same compiler flags."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from cppyy_kit import pydantic_structs as pyd
from model import Detection

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build"


def build(force=False):
    BUILD.mkdir(exist_ok=True)
    source, namespace, header = pyd.emit_cpp(Detection)
    schema = header + f"\nusing DetectionType = {namespace}::Detection;\n"
    schema_path = BUILD / "detection_schema.h"
    schema_path.write_text(schema)
    implementation = HERE / os.environ.get("DETECTION_SOURCE", "native.cpp")
    flags = ["-shared", "-fPIC", "-std=c++17", "-O3", "-march=native", "-ffp-contract=off"]
    cmd = [os.environ.get("CXX", "c++"), *flags,
           "-I" + str(HERE), "-I" + str(BUILD), "-I" + sys.prefix + "/include",
           str(implementation), "-o", str(BUILD / "libdetection.so"),
           "-L" + sys.prefix + "/lib", "-Wl,-rpath," + sys.prefix + "/lib", "-ltbb",
           "-fopt-info-vec-optimized=" + str(BUILD / "vectorization.txt")]
    digest = hashlib.sha256((schema + implementation.read_text()
                             + (HERE / "native.h").read_text() + repr(cmd)).encode()).hexdigest()
    stamp = BUILD / "compile.json"
    if not force and stamp.exists() and (BUILD / "libdetection.so").exists():
        data = json.loads(stamp.read_text())
        if data.get("digest") == digest:
            return {**data, "cache_hit": True, "compile_seconds_this_run": 0.}
    (BUILD / "vectorization.txt").unlink(missing_ok=True)
    start = time.perf_counter()
    result = subprocess.run(cmd, capture_output=True, text=True)
    data = {"command": cmd, "digest": digest, "seconds": time.perf_counter() - start,
            "compiler": subprocess.check_output([cmd[0], "--version"], text=True).splitlines()[0],
            "returncode": result.returncode, "stderr": result.stderr}
    stamp.write_text(json.dumps(data, indent=2))
    if result.returncode:
        raise RuntimeError(result.stderr)
    return {**data, "cache_hit": False, "compile_seconds_this_run": data["seconds"]}


if __name__ == "__main__":
    print(json.dumps(build(force="--force" in sys.argv), indent=2))
