"""Tests for Slice 2.5a2's callable-lifetime reaper
(PLAN-mte-unlock.md Addendum v3).

Slice 2.5a ties the native entity's lifetime to C++. The Python callable was
still pinned to a Python wrapper with ``cppyy_kit.keep_alive``. Destroying a
peer subscription during a callback and collecting the wrapper reproduced the
original crash in 11 of 60 runs. ``_pinned_std_function`` stores a Python
reference in each ``std::function`` copy. Its destructor queues that reference
for release on a thread holding the GIL.
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
    wait-set rebuild). This constructs a copy and destroys it on a separate
    ``std::thread`` (no GIL), and asserts that does not crash.
    The release occurs only after it is drained on a GIL thread.
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
    """A queued release that is never drained must not crash the process.
    It leaks the Python object. The subprocess checks that the release is
    still pending at interpreter exit.
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
    """Regression test for Slice 2.5a2 (11/60 crashes before the change):
    close a peer's subscription while its callback is running, then drop all
    Python references and call gc.collect(). The test runs 60 iterations."""
    # Outer timeout must clear the helper's own WATCHDOG_SECONDS (120s) with
    # margin, so a real hang surfaces as a stack dump, not a blind kill.
    # Outer timeout must clear the helper's own WATCHDOG_SECONDS (350s) with
    # margin (bumped from 120s/200s after this run observed 60 iterations
    # taking longer than that under sustained system memory pressure --
    # confirmed via `free`/`/proc/swaps` showing swap fully committed, not a
    # hang: the watchdog fired mid-time.sleep(), an unconditional call that
    # always returns, meaning the run was slower than the
    # window, not stuck).
    process = run_helper("_gc_after_close_helper.py", timeout=420)
    assert process.returncode == 0, format_output(process)
    assert "GC_AFTER_CLOSE_ALL_OK" in process.stdout, format_output(process)


def test_gc_after_quiescent_close_stays_clean():
    """Check that closing and collecting after the peer callback returns
    remains clean. This was also clean before Slice 2.5a2."""
    # ARM64 needed 121s for 59 iterations here; keep the hang watchdog
    # separate from realistic full-loop runtime.
    process = run_helper("_gc_after_quiescent_close_helper.py", timeout=420)
    assert process.returncode == 0, format_output(process)
    assert "GC_AFTER_QUIESCENT_ALL_OK" in process.stdout, format_output(process)


def test_gc_during_rcl_wait_does_not_crash():
    """Check garbage collection of an idle subscription during ``rcl_wait``.

    A MultiThreadedExecutor worker may hold a wait-set-local reference to the
    subscription without dispatching a callback. The test drops the Python
    reference and calls ``gc.collect()`` while the worker is waiting. The
    marshal-window test below covers the interval Python cannot control.
    """
    process = run_helper("_gc_during_rclwait_helper.py", timeout=200)
    assert process.returncode == 0, format_output(process)
    assert "GC_DURING_RCLWAIT_ALL_OK" in process.stdout, format_output(process)


def test_marshal_window_stress_does_not_crash():
    """Check the reaper while cppyy converts callback arguments.

    The quiescence counter starts after the containment shim runs, so it cannot
    see a worker that has obtained the executable but is still converting
    arguments. The test hook pauses the worker there, closes the subscription,
    collects Python references, then lets the callback run.

    A wrapper without the reaper crashed on the first iteration of an isolated
    experiment. With the reaper, the stored ``std::function`` owns the Python
    reference while the worker holds the executable.
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
    calling gc.collect() on a worker thread dispatching through
    set_parameters. Must stay crash-free across every iteration."""
    process = run_helper(
        "_native_parameter_teardown_under_dispatch_helper.py", timeout=150)
    assert process.returncode == 0, format_output(process)
    assert "PARAM_EXTERNAL_CLOSE_OK" in process.stdout, format_output(process)
    assert "PARAM_TEARDOWN_ALL_OK" in process.stdout, format_output(process)
