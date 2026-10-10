"""Check Eigen/C++17 and Cling in separate processes before running the demo."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent


def main():
    build = ROOT / "build"
    build.mkdir(exist_ok=True)
    source = build / "probe.cpp"
    source.write_text('#include "native.hpp"\n#include <iostream>\nint main() {\n'
                      'std::cout << buffer_demo::eigen_version() << " " << buffer_demo::cpp_standard();\n'
                      'double p[3]={1,2,3}; return buffer_demo::energy(p,3)==14 ? 0 : 1;\n}\n')
    command = [os.environ.get("CXX", "c++"), "-std=c++17", "-O2", "-I" + str(ROOT),
               "-I" + str(Path(sys.prefix) / "include/eigen3"), str(source),
               "-o", str(build / "probe")]
    start = time.perf_counter()
    compiled = subprocess.run(command, capture_output=True, text=True)
    evidence = {"compiler_command": command, "compile_seconds": time.perf_counter() - start,
                "compiler_returncode": compiled.returncode, "compiler_stderr": compiled.stderr}
    if compiled.returncode == 0:
        direct = subprocess.run([str(build / "probe")], capture_output=True, text=True)
        evidence["direct"] = {"returncode": direct.returncode, "stdout": direct.stdout, "stderr": direct.stderr}
    child = subprocess.run([sys.executable, "-c", "import buffers, cppyy; buffers.load_native(); "
                            "import numpy as np; print(cppyy.gbl.buffer_demo.eigen_version(), "
                            "cppyy.gbl.buffer_demo.cpp_standard()); "
                            "assert buffers.energy_kernel(np.array([[1., 2., 3.]])) == 14"],
                           cwd=ROOT, capture_output=True, text=True)
    evidence["cling"] = {"returncode": child.returncode, "stdout": child.stdout, "stderr": child.stderr}
    print(json.dumps(evidence, indent=2))
    return 0 if compiled.returncode == 0 and evidence["direct"]["returncode"] == 0 and child.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
