"""Slice 2.5a2 (PLAN-mte-unlock.md Addendum v3): the callable-lifetime
reaper. ``ManagedCallbackEntityImpl`` (Slice 2.5a) ties the native *entity*'s
lifetime to C++, but left the *callable* pinned only via
``cppyy_kit.keep_alive`` on a Python wrapper object -- whose GC timing is
not bound to the native entity at all (proven exploitable: a destroyer
closing a peer's subscription mid-flight, then forcing an immediate
``gc.collect()``, reproduced the original crash 11/60). ``_pinned_std_function``
fixes this unconditionally: every copy of the returned ``std::function``
value owns its own ``Py_INCREF``'d reference, released only when that
specific copy is destroyed -- via a thread-safe queue a GIL-holding reaper
drains, never a direct ``Py_DECREF`` from whatever thread happens to run the
destructor.
"""
import gc
import sys
import threading

import cppyy

from _run_helper import format_output, run_helper
from rclcpp_kit import direct_entities


_TEST_HELPERS_SOURCE = r"""
#include <functional>
#include <thread>
#include <utility>

namespace rclcpp_kit_callable_reaper_test_v1 {
std::function<void()>* heap_allocate_nullary(std::function<void()> value) {
  return new std::function<void()>(std::move(value));
}

void destroy_on_native_thread(std::function<void()>* holder) {
  std::thread worker([holder]() {
    delete holder;
  });
  worker.join();
}
}
"""
_TEST_HELPERS_NAMESPACE = "rclcpp_kit_callable_reaper_test_v1"
_TEST_HELPERS_LOCK = threading.Lock()


def _install_test_helpers():
    if hasattr(cppyy.gbl, _TEST_HELPERS_NAMESPACE):
        return
    with _TEST_HELPERS_LOCK:
        if hasattr(cppyy.gbl, _TEST_HELPERS_NAMESPACE):
            return
        cppyy.cppdef(_TEST_HELPERS_SOURCE)


def test_pinned_std_function_increfs_construction_and_dispatches():
    calls = []

    def callback():
        calls.append(1)

    before = sys.getrefcount(callback)
    pinned = direct_entities._pinned_std_function("void()", callback)
    after_construct = sys.getrefcount(callback)
    assert after_construct == before + 1

    pinned()
    assert calls == [1]

    del pinned
    gc.collect()
    drained = direct_entities.drain_callable_reaper()
    assert drained >= 1
    assert sys.getrefcount(callback) == before


def test_copying_the_pinned_value_increfs_again_and_both_copies_dispatch():
    calls = []

    def callback():
        calls.append(1)

    before = sys.getrefcount(callback)
    pinned = direct_entities._pinned_std_function("void()", callback)
    copy = cppyy.gbl.std.function["void()"](pinned)
    after_copy = sys.getrefcount(callback)
    assert after_copy == before + 2, (
        "copy-constructing the std::function value must INCREF again -- "
        "each copy independently owns a strong reference"
    )

    pinned()
    copy()
    assert calls == [1, 1]

    del pinned, copy
    gc.collect()
    direct_entities.drain_callable_reaper()
    assert sys.getrefcount(callback) == before


def test_destroying_a_copy_on_a_native_thread_defers_instead_of_crashing():
    """The core safety property: a std::function copy's destructor must
    never touch the Python C API directly when it runs on a thread with no
    GIL held (a native worker dropping the entity's last reference during a
    wait-set rebuild). This constructs a copy, destroys it on a genuinely
    separate ``std::thread`` (no GIL), and asserts that does not crash --
    and that the release only actually lands once drained on a GIL thread.
    """
    _install_test_helpers()
    helpers = getattr(cppyy.gbl, _TEST_HELPERS_NAMESPACE)

    calls = []

    def callback():
        calls.append(1)

    before = sys.getrefcount(callback)
    pinned = direct_entities._pinned_std_function("void()", callback)

    # Heap-allocate a second, independent copy (its own INCREF happens here,
    # on this GIL-holding thread, matching the construct-once-at-entity-
    # construction invariant), then destroy THAT copy on a native thread.
    holder = helpers.heap_allocate_nullary(pinned)
    after_heap_copy = sys.getrefcount(callback)
    assert after_heap_copy == before + 2

    helpers.destroy_on_native_thread(holder)
    # Must not have crashed to get here. The release must be queued, not
    # yet applied -- no drain has run since the native-thread destroy.
    assert direct_entities.pending_callable_reaper_count() >= 1
    assert sys.getrefcount(callback) == after_heap_copy, (
        "a native-thread destroy must defer the Py_DECREF, never apply it "
        "directly off-thread"
    )

    del pinned
    gc.collect()
    drained = direct_entities.drain_callable_reaper()
    assert drained >= 2
    assert sys.getrefcount(callback) == before


def test_drain_is_idempotent_and_safe_with_nothing_pending():
    assert direct_entities.pending_callable_reaper_count() == 0
    assert direct_entities.drain_callable_reaper() == 0


def test_undrained_release_leaks_by_design_instead_of_crashing_at_exit():
    """Leak-safety semantics: a queued release that is never drained must
    not crash the process -- it leaks the Python object, which is the
    documented, accepted tradeoff (leak-safe beats crash-safe). Runs in a
    subprocess so an actually-undrained, still-pending release at
    interpreter exit is a real, observed condition, not just asserted.
    """
    import os
    import subprocess

    script = r"""
import sys
import cppyy
from rclcpp_kit import direct_entities

_HELPERS = r'''
#include <functional>
#include <thread>
#include <utility>
namespace rclcpp_kit_callable_reaper_leak_test_v1 {
std::function<void()>* heap_allocate_nullary(std::function<void()> value) {
  return new std::function<void()>(std::move(value));
}
void destroy_on_native_thread(std::function<void()>* holder) {
  std::thread worker([holder]() { delete holder; });
  worker.join();
}
}
'''
cppyy.cppdef(_HELPERS)
helpers = cppyy.gbl.rclcpp_kit_callable_reaper_leak_test_v1

def callback():
    pass

pinned = direct_entities._pinned_std_function("void()", callback)
holder = helpers.heap_allocate_nullary(pinned)
helpers.destroy_on_native_thread(holder)
assert direct_entities.pending_callable_reaper_count() >= 1
# Deliberately exit WITHOUT draining -- the queued release must leak, not
# crash the interpreter on shutdown.
sys.stdout.write("LEAK_SAFE_EXIT_OK\n")
sys.stdout.flush()
"""
    process = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True, text=True, timeout=60,
        env=os.environ.copy(),
    )
    assert process.returncode == 0, (
        "process:\nSTDOUT:\n%s\nSTDERR:\n%s" %
        (process.stdout, process.stderr)
    )
    assert "LEAK_SAFE_EXIT_OK" in process.stdout


def test_gc_after_close_of_an_in_flight_peer_does_not_crash():
    """The probe that found Slice 2.5a incomplete (11/60 crashes pre-2.5a2):
    a destroyer closes a peer's subscription while that peer's callback is
    genuinely mid-flight, then drops all Python refs + forces gc.collect().
    Must now stay crash-free across every one of 60 iterations."""
    # Outer timeout must clear the helper's own WATCHDOG_SECONDS (120s) with
    # margin, so a real hang surfaces as a stack dump, not a blind kill.
    # Outer timeout must clear the helper's own WATCHDOG_SECONDS (350s) with
    # margin (bumped from 120s/200s after this run observed 60 iterations
    # taking longer than that under sustained system memory pressure --
    # confirmed via `free`/`/proc/swaps` showing swap fully committed, not a
    # hang: the watchdog fired mid-time.sleep(), an unconditional call that
    # always returns, meaning the whole run was simply slower than the
    # window, not stuck).
    process = run_helper("_gc_after_close_helper.py", timeout=420)
    assert process.returncode == 0, format_output(process)
    assert "GC_AFTER_CLOSE_ALL_OK" in process.stdout, format_output(process)


def test_gc_after_quiescent_close_stays_clean():
    """Regression guard: closing (and aggressively GC'ing) only after the
    peer callback has already genuinely returned was clean even before
    Slice 2.5a2; it must stay clean after."""
    process = run_helper("_gc_after_quiescent_close_helper.py", timeout=200)
    assert process.returncode == 0, format_output(process)
    assert "GC_AFTER_QUIESCENT_ALL_OK" in process.stdout, format_output(process)


def test_gc_during_rcl_wait_does_not_crash():
    """Extra coverage (superseded as the primary discriminator by the
    marshal-window stress below, per PLAN-mte-unlock.md Addendum v3.1 --
    Python cannot reliably hit the true marshal window on its own, but this
    parked-rcl_wait/idle-subscription scenario is still worth keeping):
    dropping an idle subscription's only Python reference and forcing
    gc.collect() while a live MultiThreadedExecutor worker may be parked in
    rcl_wait, holding a wait-set-local strong copy of the entity -- no
    callback ever dispatched. Must stay crash-free across every iteration."""
    process = run_helper("_gc_during_rclwait_helper.py", timeout=200)
    assert process.returncode == 0, format_output(process)
    assert "GC_DURING_RCLWAIT_ALL_OK" in process.stdout, format_output(process)


def test_marshal_window_stress_does_not_crash():
    """The discriminating proof for the reaper (PLAN-mte-unlock.md Addendum
    v3.1): a worker that has obtained the executable and committed to
    dispatch, mid-marshal (cppyy converting the message into Python args,
    the shim not yet entered), is invisible to any callback-quiescence
    counter by construction -- it increments only once the shim is
    entered. Using set_marshal_window_hook to widen that window on demand
    (not reliably reachable from pure Python otherwise), this closes the
    subscription -- dropping every Python reference and forcing
    gc.collect() -- while a worker is parked exactly inside that window,
    then lets it proceed to actually invoke the callable.

    Confirmed genuinely discriminating: an isolated experiment with a
    naive (unprotected, no-reaper) callable wrapper reproduced the crash
    immediately on iteration 0 with the classic 'terminate called without
    an active exception' signature. With the reaper (this suite's
    production code), this must stay crash-free across every iteration --
    the entity's own stored callback member (which a mid-marshal worker's
    AnyExecutable keeps alive regardless of any Python wrapper's fate) now
    itself owns the Python callable's reference, unconditionally.
    """
    process = run_helper("_marshal_window_stress_helper.py", timeout=200)
    assert process.returncode == 0, format_output(process)
    assert "MARSHAL_WINDOW_ALL_OK" in process.stdout, format_output(process)


def test_parameter_bridge_teardown_under_worker_dispatch_does_not_crash():
    """Slice 2.5a4 (PLAN-mte-unlock.md Addendum v3.2): on/pre/post-set-
    parameters callbacks are worker-dispatched (synchronously inside
    set_parameters, from a worker thread or the node's parameter service on
    a remote request) and were not covered by the product's in-flight
    counter at all -- only the reaper protects them. Self-closes the
    bridge from within its own callback, dropping every reference and
    forcing gc.collect(), on a worker thread genuinely dispatching through
    set_parameters. Must stay crash-free across every iteration."""
    process = run_helper(
        "_native_parameter_teardown_under_dispatch_helper.py", timeout=150)
    assert process.returncode == 0, format_output(process)
    assert "PARAM_EXTERNAL_CLOSE_OK" in process.stdout, format_output(process)
    assert "PARAM_TEARDOWN_ALL_OK" in process.stdout, format_output(process)
