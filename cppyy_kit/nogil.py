"""
cppyy_kit.nogil releases the GIL around a blocking C++ call.

**cppyy holds the GIL during a blocking C++ call** (COMMON_PATTERNS §13). Other
Python threads cannot run during that call, even if it runs on a Python thread.
``nogil(fn)`` uses a compiled shim to release the GIL (``Py_BEGIN_ALLOW_THREADS``)
while it invokes a C++ callable, then reacquires it when the call returns.

    def worker(): ...                    # a normal Python thread, runs concurrently
    threading.Thread(target=worker).start()
    cppyy_kit.nogil(cppyy.gbl.mylib.blocking_spin)    # C++ blocks here, GIL released

Measured (test_nogil.py): a 500 ms C++ sleep called directly lets a co-thread
advance about one tick. Through ``nogil``, it advances about 470 ticks while the
C++ call runs.

Rules:
* ``fn`` must be a **C++** nullary callable: a cppyy-bound C++ ``void()`` function
  or a ``std::function<void()>``. A *Python* callable would re-acquire the GIL to
  run (cppyy takes the GIL to enter Python), defeating the point; bind arguments and
  results in C++ (a ``cppdef``/``@cpp`` nullary wrapper that stores its result in a
  C++ object you read afterwards). This mirrors §13: run the blocking work on a C++
  path, not a Python one.
* **Callback-into-Python caveat:** if ``fn`` calls back into Python while the GIL is
  released, that callback must re-acquire the GIL first. A cppyy Python callback does
  this for you (it takes the GIL on entry), but hand-written C++ that touches
  ``PyObject*`` under ``nogil`` must ``PyGILState_Ensure()``/``Release`` around it.
"""
import threading

_SHIM = r"""
#include <Python.h>
#include <functional>
namespace cppyy_kit_nogil {
class GilRelease {
  PyThreadState* state_;
public:
  GilRelease() : state_(PyEval_SaveThread()) {}
  ~GilRelease() noexcept { PyEval_RestoreThread(state_); }
  GilRelease(const GilRelease&) = delete;
  GilRelease& operator=(const GilRelease&) = delete;
};

void run_nogil(std::function<void()> f) {
  GilRelease released;
  f();
}
}
"""
_DECLS = r"""
#include <Python.h>
#include <functional>
namespace cppyy_kit_nogil { void run_nogil(std::function<void()> f); }
"""
_READY = False
_LOCK = threading.Lock()


def _ensure():
    """Compile the GIL-release shim once. Thread-safe (double-checked lock): the
    first ``nogil()`` calls may arrive from several threads at once. Without the
    lock each would re-run the ``cppdef``, and Cling would emit a "redefinition"
    error. Once built, the shim can be called without acquiring the lock."""
    global _READY, cppyy, cache
    if _READY:                       # fast path: no lock once the shim exists
        return
    with _LOCK:
        if _READY:                   # re-check under the lock
            return
        from . import _ensure_runtime
        _ensure_runtime()
        import cppyy
        from . import cache
        cache.cppdef_cached(_SHIM, decls=_DECLS, name="nogil_shim", trampoline=True)
        _READY = True


def nogil(fn):
    """Run the nullary **C++** callable ``fn`` with the GIL released (module
    docstring). Returns None because ``fn`` is ``void()``. Pass results through a C++
    object it writes. Raises if ``fn`` isn't acceptable as ``std::function<void()>``."""
    _ensure()
    cppyy.gbl.cppyy_kit_nogil.run_nogil(fn)


async def run_async(fn, executor=None):
    """Await a blocking C++ callable without stalling the asyncio event loop: run
    ``fn`` in a thread (``run_in_executor``) *with the GIL released*, so both the
    executor's C++ work and the event loop make progress. ``fn`` is the same
    nullary C++ callable ``nogil`` takes."""
    import asyncio
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(executor, lambda: nogil(fn))
