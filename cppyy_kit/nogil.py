"""
cppyy_kit.nogil -- release the GIL around a blocking C++ call.

The corrected GIL evidence (COMMON_PATTERNS §13): **cppyy does not release the GIL
on a blocking C++ call**, so a blocking call made from Python holds the GIL for its
whole duration and starves every other Python thread -- you cannot overlap it with
Python work by putting it on a *Python* thread. ``nogil(fn)`` fixes that for a C++
callable: a compiled shim drops the GIL (``Py_BEGIN_ALLOW_THREADS``) around invoking
``fn`` and re-takes it after, so concurrent Python threads run during the call.

    def worker(): ...                    # a normal Python thread, runs concurrently
    threading.Thread(target=worker).start()
    cppyy_kit.nogil(cppyy.gbl.mylib.blocking_spin)    # C++ blocks here, GIL released

Measured (test_nogil.py): a 500 ms C++ sleep called directly lets a co-thread
advance ~1 tick; through ``nogil`` it advances ~470 -- i.e. the co-thread runs the
whole time.

Rules:
* ``fn`` must be a **C++** nullary callable -- a cppyy-bound C++ ``void()`` function
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
import cppyy

from . import cache

_SHIM = r"""
#include <Python.h>
#include <functional>
namespace cppyy_kit_nogil {
void run_nogil(std::function<void()> f) {
  Py_BEGIN_ALLOW_THREADS
  f();
  Py_END_ALLOW_THREADS
}
}
"""
_DECLS = r"""
#include <Python.h>
#include <functional>
namespace cppyy_kit_nogil { void run_nogil(std::function<void()> f); }
"""
_READY = False


def _ensure():
    global _READY
    if _READY:
        return
    cache.cppdef_cached(_SHIM, decls=_DECLS, name="nogil_shim", trampoline=True)
    _READY = True


def nogil(fn):
    """Run the nullary **C++** callable ``fn`` with the GIL released (module
    docstring). Returns None -- ``fn`` is ``void()``; surface results through a C++
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
