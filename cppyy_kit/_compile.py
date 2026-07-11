"""
cppyy_kit._compile -- the direct-compile recipe, factored out of the freeze/L2
scripts so the compile cache (``cppyy_kit.cache``) and the vendored-source builds
share one code path.

A single ``$CXX -shared -fPIC`` invocation turns a C++ translation unit into a
real ``.so`` that cppyy can ``load_library`` -- this is the mechanism behind the
L2 lowering (``scripts/freeze/build_l2_node.py``) and the §21 vendored-source
builds, generalized. ``cppyy_toolchain()`` adds what a *trampoline* .so needs (the
Python + CPyCppyy headers and ``libcppyy``) so compiled glue can convert C++
objects to Python proxies and call Python callables directly -- the pattern that
lets the cache eliminate cppyy's first-use call-wrapper JIT (see cache.py).
"""
import glob
import os
import subprocess
import sys
import sysconfig


class CompileError(RuntimeError):
    """A direct-compile invocation failed; message carries the compiler stderr."""


def compiler():
    """The env's C++ compiler ($CXX, else ``c++``)."""
    return os.environ.get("CXX") or "c++"


def cppyy_toolchain():
    """Include/link flags a *trampoline* .so needs: the Python C-API and CPyCppyy
    public headers (``CPyCppyy/API.h`` for ``Instance_FromVoidPtr`` etc.) and the
    ``libcppyy`` extension it calls into. Merge into a ``compile_shared`` call when
    the source uses ``PyObject*``/CPyCppyy. Returns a dict of lists."""
    py_inc = sysconfig.get_path("include")            # .../include/pythonX.Y (CPyCppyy lives here)
    site = sysconfig.get_path("purelib")
    libcppyy = sorted(glob.glob(os.path.join(site, "libcppyy.*.so")))
    if not libcppyy:
        raise CompileError(
            "libcppyy not found under %s -- is cppyy installed in this env?" % site)
    # Link by exact filename (-l:) since the soname is versioned/ABI-tagged.
    return {
        "include_paths": [py_inc],
        "link_paths": [site],
        "link_args": ["-l:" + os.path.basename(libcppyy[0]), "-Wl,-rpath," + site],
    }


def compile_shared(sources, out_path, include_paths=(), library_paths=(),
                   libraries=(), link_args=(), std="c++17", opt="-O2",
                   defines=(), extra_flags=()):
    """Compile ``sources`` (one path or a list) into the shared library
    ``out_path`` with the env's C++ compiler. RPATHs every ``library_paths`` entry
    so the result resolves its dependencies without ``LD_LIBRARY_PATH``. Raises
    ``CompileError`` with the compiler stderr on failure. Returns ``out_path``.

    The output is written atomically (compiled to ``out_path + .tmp`` then
    renamed) so an interrupted/failed compile never leaves a half-written .so that
    a later run would mistake for a valid cache entry.
    """
    if isinstance(sources, str):
        sources = [sources]
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    tmp = out_path + ".tmp.%d" % os.getpid()
    cmd = [compiler(), "-shared", "-fPIC", "-std=" + std]
    if opt:
        cmd.append(opt)
    cmd += ["-D" + d for d in defines]
    cmd += ["-I" + p for p in include_paths]
    cmd += list(extra_flags)
    cmd += list(sources)
    cmd += ["-o", tmp]
    cmd += ["-L" + p for p in library_paths]
    cmd += ["-l" + lib for lib in libraries]
    cmd += ["-Wl,-rpath," + p for p in library_paths]
    cmd += list(link_args)
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise CompileError("compile failed (%d):\n%s\n%s"
                           % (proc.returncode, " ".join(cmd), proc.stderr.strip()))
    os.replace(tmp, out_path)
    return out_path


def _stderr(msg):
    sys.stderr.write(msg if msg.endswith("\n") else msg + "\n")
