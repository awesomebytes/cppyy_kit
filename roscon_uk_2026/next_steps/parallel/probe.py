"""Parse and instantiate uncertain library templates in child processes."""
import json
from pathlib import Path
import subprocess
import sys
import time

CASES = {
    "tbb_inline": r'''
import cppyy
cppyy.load_library("libtbb")
cppyy.cppdef("""
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/task_arena.h>
int probe_tbb() {
 int out[17]{};
 oneapi::tbb::task_arena arena(2);
 arena.execute([&] { oneapi::tbb::parallel_for(0, 17, [&](int i){out[i]=i;}); });
 int sum=0; for(auto x:out)sum+=x; return sum;
}
""")
assert cppyy.gbl.probe_tbb() == 136
print("oneTBB inline template probe passed")
''',
    "xsimd_inline": r'''
import cppyy
cppyy.cppdef("""
#include <xsimd/xsimd.hpp>
double probe_simd() {
 using B=xsimd::batch<double>;
 double out[B::size];
 (B(2.)*B(3.)).store_unaligned(out);
 return out[0];
}
""")
assert cppyy.gbl.probe_simd() == 6.
print("xsimd inline template probe passed")
''',
}


def main():
    results = {}
    for name, source in CASES.items():
        start = time.perf_counter()
        try:
            run = subprocess.run([sys.executable, "-c", source], capture_output=True, text=True, timeout=60)
            data = {"returncode": run.returncode, "stdout": run.stdout, "stderr": run.stderr}
        except subprocess.TimeoutExpired as exc:
            data = {"returncode": "timeout", "stdout": str(exc.stdout), "stderr": str(exc.stderr)}
        results[name] = {**data, "seconds": time.perf_counter() - start, "source": source}
    output = Path(__file__).resolve().parent / "build"
    output.mkdir(exist_ok=True)
    (output / "probes.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
