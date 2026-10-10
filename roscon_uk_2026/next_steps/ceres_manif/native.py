"""Load the small compiled adapter, without parsing Ceres/manif in Cling."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
_LOADED = {}

def load(mode="compiled"):
    if mode in _LOADED: return _LOADED[mode]
    if _LOADED: raise ValueError("run JIT and compiled modes in separate processes")
    start = time.perf_counter()
    import cppyy
    from cppyy_kit._compile import compile_shared
    prefix = Path(sys.prefix)
    if mode == "jit":
        for path in (ROOT,prefix/"include",prefix/"include/eigen3"):
            cppyy.add_include_path(str(path))
        for name in ("gflags","glog","ceres"):
            cppyy.load_library(str(prefix/"lib"/("lib"+name+".so")))
        cppyy.cppdef("#define GLOG_USE_GLOG_EXPORT\n"+(ROOT/"adapter.cpp").read_text())
        _LOADED[mode]=(cppyy.gbl.calibration,dict(mode="jit",header_jit_seconds=time.perf_counter()-start))
        return _LOADED[mode]
    if mode != "compiled": raise ValueError("unknown native loading mode")
    sources = [ROOT / name for name in ("adapter.cpp", "adapter.hpp", "residual.hpp")]
    compiler = os.environ.get("CXX", "c++")
    identity = hashlib.sha256(b"".join(p.read_bytes() for p in sources)
                             + (ROOT/"pixi.lock").read_bytes()
                             + str(prefix).encode()
                             + subprocess.check_output([compiler,"--version"])).hexdigest()[:16]
    library = ROOT / "build" / ("libcalibration_"+identity+".so")
    compile_seconds = 0.0
    if not library.exists():
        t = time.perf_counter()
        compile_shared(str(ROOT / "adapter.cpp"), str(library),
                       include_paths=[str(prefix / "include"),str(prefix / "include/eigen3")],
                       library_paths=[str(prefix / "lib")], libraries=["ceres", "glog", "gflags"],
                       defines=["GLOG_USE_GLOG_EXPORT"], std="c++17", opt="-O2")
        compile_seconds = time.perf_counter()-t
    cppyy.load_library(str(library))
    cppyy.include(str(ROOT / "adapter.hpp"))
    _LOADED[mode]=(cppyy.gbl.calibration,dict(mode="compiled",adapter_compile_seconds=compile_seconds,
                                            adapter_load_seconds=time.perf_counter()-start-compile_seconds,
                                            library=str(library)))
    return _LOADED[mode]

if __name__ == "__main__":
    print(json.dumps(load()[1], indent=2))
