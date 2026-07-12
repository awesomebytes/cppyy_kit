#!/usr/bin/env python3
"""Benchmarks for the pydantic_structs spike -- the proof behind the three win
claims in design/pydantic_structs.md. Runnable:

    python cppyy_kit/design/bench_pydantic_structs.py            # all benches
    python cppyy_kit/design/bench_pydantic_structs.py _rss_models  # (internal, one mode)

Needs pydantic v2 + numpy + a cppyy toolchain. The default pixi env has no
pydantic; run under an env that provides it. Memory numbers are measured in
**separate subprocesses** (one per representation) so a Python list's heap
high-water mark does not contaminate the vector's measurement.

Model: Detection{x,y,z,score: float; label: str} (flat, per the plan). The
compute task is a filter+centroid: mean of (x,y,z) over items with score > 0.5.
"""
import os
import subprocess
import sys
import time

N = 1_000_000
THRESHOLD = 0.5


def _have_deps():
    try:
        import numpy  # noqa: F401
        import pydantic  # noqa: F401
        return True
    except ImportError:
        return False


def _model():
    from pydantic import BaseModel

    class Detection(BaseModel):
        x: float
        y: float
        z: float
        score: float
        label: str

    return Detection


def _rss_mb():
    try:
        import psutil
        return psutil.Process().memory_info().rss / 2**20
    except ImportError:
        import resource
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


# --- compute kernel (cppdef_cached over the struct vector) ------------------
def _build_kernel(struct):
    import cppyy
    import cppyy_kit
    hdr = os.path.basename(struct.header)
    ns = struct.cpp_name
    defs = (
        '#include "%s"\n#include <cstdint>\n#include <cstddef>\n'
        'namespace pyd_bench {\n'
        'int64_t filter_centroid(uintptr_t base, std::size_t n, double thr, uintptr_t outa){\n'
        '  const %s* d = reinterpret_cast<const %s*>(base);\n'
        '  double* out = reinterpret_cast<double*>(outa);\n'
        '  double sx=0,sy=0,sz=0; int64_t k=0;\n'
        '  for (std::size_t i=0;i<n;++i){ if(d[i].score>thr){ sx+=d[i].x; sy+=d[i].y; sz+=d[i].z; ++k; } }\n'
        '  if(k){ out[0]=sx/k; out[1]=sy/k; out[2]=sz/k; } else { out[0]=out[1]=out[2]=0; }\n'
        '  return k; }\n}\n' % (hdr, ns, ns))
    decls = ('#include <cstdint>\n#include <cstddef>\n'
             'namespace pyd_bench { int64_t filter_centroid(uintptr_t, std::size_t, double, uintptr_t); }\n')
    cppyy_kit.cppdef_cached(defs, decls=decls, name="pyd_bench_filter_centroid",
                            include_paths=[struct.header_dir])
    return cppyy.gbl.pyd_bench.filter_centroid


def _build_sum_kernel(struct):
    import cppyy
    import cppyy_kit
    hdr = os.path.basename(struct.header)
    ns = struct.cpp_name
    defs = (
        '#include "%s"\n#include <cstdint>\n#include <cstddef>\n'
        'namespace pyd_bench {\n'
        'double sum_score(uintptr_t base, std::size_t n){\n'
        '  const %s* d = reinterpret_cast<const %s*>(base);\n'
        '  double s=0; for (std::size_t i=0;i<n;++i) s+=d[i].score; return s; }\n}\n' % (hdr, ns, ns))
    decls = ('#include <cstdint>\n#include <cstddef>\n'
             'namespace pyd_bench { double sum_score(uintptr_t, std::size_t); }\n')
    cppyy_kit.cppdef_cached(defs, decls=decls, name="pyd_bench_sum_score",
                            include_paths=[struct.header_dir])
    return cppyy.gbl.pyd_bench.sum_score


def _bench_compute():
    import cppyy_kit  # noqa: F401
    import numpy as np
    from cppyy_kit import pydantic_structs as pyd

    Detection = _model()
    rng = np.random.default_rng(0)
    cols = {k: rng.random(N) for k in ("x", "y", "z", "score")}
    models = [Detection(x=float(cols["x"][i]), y=float(cols["y"][i]),
                        z=float(cols["z"][i]), score=float(cols["score"][i]),
                        label="obj") for i in range(N)]
    S = pyd.cpp_struct(Detection)
    vec = pyd.cpp_vector_columnar(Detection, cols)
    kernel = _build_kernel(S)
    base = S._helper("_vec_data")(vec)
    out = np.zeros(3, np.float64)

    def py_models():
        sx = sy = sz = 0.0
        k = 0
        for m in models:
            if m.score > THRESHOLD:
                sx += m.x
                sy += m.y
                sz += m.z
                k += 1
        return (sx / k, sy / k, sz / k, k)

    def cpp_vec():
        k = kernel(base, N, THRESHOLD, out.ctypes.data)
        return (out[0], out[1], out[2], k)

    def numpy_cols():
        m = cols["score"] > THRESHOLD
        return (cols["x"][m].mean(), cols["y"][m].mean(), cols["z"][m].mean(), int(m.sum()))

    def best(fn, reps=5):
        fn()  # warm (JIT / cache / branch predictor)
        t = min(_time(fn) for _ in range(reps))
        return t

    # pure contiguous reduction (sum of score) -- the case that should favor numpy
    sum_kernel = _build_sum_kernel(S)

    def py_sum():
        s = 0.0
        for m in models:
            s += m.score
        return s

    def cpp_sum():
        return sum_kernel(base, N)

    def np_sum():
        return float(cols["score"].sum())

    r_py = py_models()
    r_cpp = cpp_vec()
    r_np = numpy_cols()
    ok = (abs(r_py[0] - r_cpp[0]) < 1e-9 and abs(r_py[0] - r_np[0]) < 1e-9 and r_py[3] == r_cpp[3] == r_np[3])

    tp = best(py_models)
    tc = best(cpp_vec)
    tn = best(numpy_cols)
    print("\n== Claim 2: hot compute over %s Detection ==" % f"{N:,}")
    print("  (A) filter+centroid (score>%.1f, branchy fused reduction) -- agree=%s, kept=%d" % (THRESHOLD, ok, r_py[3]))
    print("      %-30s %9.1f ms   (1.0x)" % ("pure Python over models", tp * 1e3))
    print("      %-30s %9.3f ms   (%.0fx)" % ("C++ kernel over vector<Struct>", tc * 1e3, tp / tc))
    print("      %-30s %9.3f ms   (%.0fx)" % ("numpy columnar (mask+gather)", tn * 1e3, tp / tn))
    sp, sc, snp = best(py_sum), best(cpp_sum), best(np_sum)
    print("  (B) sum(score) (pure contiguous reduction) -- the case that favors numpy")
    print("      %-30s %9.3f ms   (1.0x)" % ("pure Python over models", sp * 1e3))
    print("      %-30s %9.3f ms   (%.0fx)" % ("C++ kernel over vector<Struct>", sc * 1e3, sp / sc))
    print("      %-30s %9.3f ms   (%.0fx)" % ("numpy columnar (.sum())", snp * 1e3, sp / snp))
    print("  honest read: numpy wins the PURE contiguous reduction (B) -- it is the incumbent")
    print("  for columnar math. The C++-struct kernel wins the BRANCHY fused one (A), because")
    print("  numpy's mask+gather allocates while the C++ loop is a single alloc-free pass -- and")
    print("  because the struct keeps the model's nested/mixed shape a flat numpy array cannot.")


def _time(fn):
    t0 = time.perf_counter()
    fn()
    return time.perf_counter() - t0


# --- memory: measured in separate subprocesses ------------------------------
def _rss_models():
    Detection = _model()
    import numpy as np
    rng = np.random.default_rng(0)
    xs, ys, zs, ss = (rng.random(N) for _ in range(4))
    base = _rss_mb()
    data = [Detection(x=float(xs[i]), y=float(ys[i]), z=float(zs[i]),
                      score=float(ss[i]), label="obj") for i in range(N)]
    import gc
    gc.collect()
    print("RSS_DELTA_MB %.1f" % (_rss_mb() - base))
    return len(data)


def _rss_vector():
    import gc
    import numpy as np
    from cppyy_kit import pydantic_structs as pyd
    Detection = _model()
    rng = np.random.default_rng(0)
    cols = {k: rng.random(N) for k in ("x", "y", "z", "score")}
    base = _rss_mb()
    vec = pyd.cpp_vector_columnar(Detection, cols)
    for i in range(N):
        vec[i].label = "obj"
    del cols
    gc.collect()
    print("RSS_DELTA_MB %.1f" % (_rss_mb() - base))
    return vec.size()


def _rss_numpy():
    import gc
    import numpy as np
    base = _rss_mb()
    cols = {k: np.random.default_rng(0).random(N) for k in ("x", "y", "z", "score")}
    labels = np.array(["obj"] * N)
    gc.collect()
    keep = sum(int(v.nbytes) for v in cols.values()) + labels.nbytes
    print("RSS_DELTA_MB %.1f" % (_rss_mb() - base))
    return keep


def _bench_memory():
    print("\n== Claim 1: compact storage (%s Detection, RSS delta, subprocess each) ==" % f"{N:,}")
    modes = [("list[Detection] (pydantic models)", "_rss_models"),
             ("std::vector<Struct> (+ string labels)", "_rss_vector"),
             ("numpy columns (4×float64 + labels)", "_rss_numpy")]
    results = {}
    for label, mode in modes:
        proc = subprocess.run([sys.executable, os.path.abspath(__file__), mode],
                              capture_output=True, text=True)
        mb = None
        for line in proc.stdout.splitlines():
            if line.startswith("RSS_DELTA_MB"):
                mb = float(line.split()[1])
        if mb is None:
            print("  %-40s  FAILED\n%s" % (label, proc.stderr[-500:]))
            continue
        results[mode] = mb
        print("  %-40s %8.1f MB" % (label, mb))
    if "_rss_models" in results and "_rss_vector" in results and results["_rss_vector"]:
        print("  -> vector<Struct> is %.1fx smaller than the model list." %
              (results["_rss_models"] / results["_rss_vector"]))


# --- type-check transcript --------------------------------------------------
def _bench_typecheck():
    from cppyy_kit import pydantic_structs as pyd
    Detection = _model()
    S = pyd.cpp_struct(Detection)
    inc = '#include "%s"' % os.path.basename(S.header)

    def kern(expr):
        return inc + ("\nnamespace tc { double f(%s* d, std::size_t n){ double s=0; "
                      "for (std::size_t i=0;i<n;++i) s+=%s; return s; } }" % (S.cpp_name, expr))

    print("\n== Claim 3: 'free' compile-time type checks (out-of-process) ==")
    for tag, expr in [("correct  (d[i].score)", "d[i].score"),
                      ("typo     (d[i].scoree)", "d[i].scoree"),
                      ("misuse   (d[i].label as double)", "d[i].label")]:
        ok, msg = pyd.check_kernel(kern(expr))
        print("  %-32s -> ok=%s" % (tag, ok))
        if not ok:
            print("      %s" % msg[:150])


def main():
    if not _have_deps():
        print("pydantic + numpy required; skipping (default env has no pydantic).")
        return 0
    print("pydantic_structs benchmarks  (N=%s)" % f"{N:,}")
    _bench_memory()
    _bench_compute()
    _bench_typecheck()
    return 0


_MODES = {"_rss_models": _rss_models, "_rss_vector": _rss_vector, "_rss_numpy": _rss_numpy}

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in _MODES:
        _MODES[sys.argv[1]]()
    else:
        sys.exit(main())
