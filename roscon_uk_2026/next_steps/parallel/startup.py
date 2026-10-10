"""Measure child process wall time for fresh and reused wrapper caches."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

SOURCE = '''
import time
start = time.perf_counter()
from processing import Batch, load
imported = time.perf_counter()
metadata = load()
loaded = time.perf_counter()
b = Batch([dict(x=3., y=4., z=0., confidence=.7, frame=0)])
prepared = time.perf_counter()
calls={}
for method in ("serial_aos", "serial_columns", "xsimd", "tbb"):
 t=time.perf_counter(); b.run(method,workers=2); calls[method]=time.perf_counter()-t
import json
print(json.dumps(dict(import_seconds=imported-start, native_load_seconds=loaded-imported,
 first_prepare_seconds=prepared-loaded, first_calls_seconds=calls, compile=metadata)))
'''


def main():
    directory = Path(__file__).resolve().parent / "build"
    directory.mkdir(exist_ok=True)
    cache = directory / ("startup-cache-" + str(time.time_ns()))
    results = {"cache_directory": str(cache), "child_source": SOURCE}
    for name in ("fresh_wrapper_cache", "reused_wrapper_cache"):
        start = time.perf_counter()
        env = {**os.environ, "CPPYY_KIT_CACHE_DIR": str(cache)}
        child = subprocess.run([sys.executable, "-c", SOURCE], env=env,
                               capture_output=True, text=True, timeout=60)
        results[name] = {"process_wall_seconds": time.perf_counter()-start,
                         "returncode": child.returncode, "stderr": child.stderr,
                         "stdout": child.stdout}
        if child.returncode == 0:
            results[name]["timings"] = json.loads(child.stdout.strip().splitlines()[-1])
        else:
            raise RuntimeError(child.stderr)
    (directory / "startup.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
