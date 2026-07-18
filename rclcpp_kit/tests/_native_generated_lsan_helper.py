#!/usr/bin/env python3
"""Prebuild and explicitly leak-check sanitized generated pipeline glue."""

from __future__ import annotations

import argparse
import ctypes
import gc
import json
from pathlib import Path
import time

from rclcpp_kit.native import native
from std_msgs.msg import String


LEAK_BYTES = 65537
TIMEOUT_S = 20.0
TRANSFORMS = {
    "clean": r"""
static thread_local bool lsan_scope_initialized = []() {
  auto disable = reinterpret_cast<void (*)()>(
    dlsym(RTLD_DEFAULT, "__lsan_disable"));
  if (disable == nullptr) {
    throw std::runtime_error("__lsan_disable is unavailable");
  }
  disable();
  return true;
}();
(void)lsan_scope_initialized;
auto enable = reinterpret_cast<void (*)()>(
  dlsym(RTLD_DEFAULT, "__lsan_enable"));
auto disable = reinterpret_cast<void (*)()>(
  dlsym(RTLD_DEFAULT, "__lsan_disable"));
if (enable == nullptr || disable == nullptr) {
  throw std::runtime_error("LSan scope functions are unavailable");
}
enable();
auto * probe = new volatile unsigned char[65537];
probe[0] = 0x5a;
delete[] probe;
disable();
output.data = input.data + ":clean";
""",
    "intentional-leak": r"""
static thread_local bool lsan_scope_initialized = []() {
  auto disable = reinterpret_cast<void (*)()>(
    dlsym(RTLD_DEFAULT, "__lsan_disable"));
  if (disable == nullptr) {
    throw std::runtime_error("__lsan_disable is unavailable");
  }
  disable();
  return true;
}();
(void)lsan_scope_initialized;
auto enable = reinterpret_cast<void (*)()>(
  dlsym(RTLD_DEFAULT, "__lsan_enable"));
auto disable = reinterpret_cast<void (*)()>(
  dlsym(RTLD_DEFAULT, "__lsan_disable"));
if (enable == nullptr || disable == nullptr) {
  throw std::runtime_error("LSan scope functions are unavailable");
}
enable();
auto * probe = new volatile unsigned char[65537];
probe[0] = 0x5a;
asm volatile("" : : "r"(probe) : "memory");
disable();
output.data = input.data + ":intentional-leak";
""",
}


def _create_pipeline(ros, case):
    topic_case = case.replace("-", "_")
    node = ros.create_node("native_generated_lsan_%s" % topic_case)
    return node, ros.create_fused_pipeline(
        node,
        String,
        String,
        "/native_generated_lsan/%s/in" % topic_case,
        "/native_generated_lsan/%s/out" % topic_case,
        TRANSFORMS[case],
        delivery="batch",
        batch_size=1,
        queue_capacity=4,
        includes=("dlfcn.h", "stdexcept"),
    )


def prebuild():
    results = {}
    pipelines = []
    with native(["native-generated-lsan-prebuild"]) as ros:
        for case in TRANSFORMS:
            _, pipeline = _create_pipeline(ros, case)
            pipelines.append(pipeline)
            result = pipeline.compile_result
            assert result.get("so")
            assert result.get("cached") is False
            assert result.get("reason") == "miss-built"
            results[case] = str(Path(result["so"]).resolve())
    assert all(pipeline.closed for pipeline in pipelines)
    print(json.dumps(results, sort_keys=True), flush=True)
    print("NATIVE_GENERATED_LSAN_PREBUILD_OK", flush=True)


def _spin_until(executor, predicate, description):
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        executor.spin_some()
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("timed out waiting for %s" % description)


def _explicit_lsan_check():
    runtime = ctypes.CDLL(None)
    check = runtime.__lsan_do_recoverable_leak_check
    check.argtypes = []
    check.restype = ctypes.c_int
    return int(check())


def run_case(case, evidence_path, *, after_session_close):
    topic_case = case.replace("-", "_")
    expected = "payload:" + case
    observed = []
    check_result = None
    with native(["native-generated-lsan-" + case]) as ros:
        node, pipeline = _create_pipeline(ros, case)
        peer = ros.create_node("native_generated_lsan_peer_" + case.replace("-", "_"))
        executor = ros.create_executor()
        executor.add_node(node)
        executor.add_node(peer)
        source = peer.create_publisher(
            String, "/native_generated_lsan/%s/in" % topic_case, 10)
        sink = peer.create_subscription(
            String,
            "/native_generated_lsan/%s/out" % topic_case,
            lambda message: observed.append(str(message.data)),
            10,
        )
        assert sink is not None

        _spin_until(
            executor,
            lambda: source.get_subscription_count() == 1,
            "generated subscription discovery",
        )
        source.publish(String(data="payload"))
        _spin_until(executor, lambda: observed == [expected], "generated output")

        stats = pipeline.stats()
        assert stats.received == 1
        assert stats.processed == 1
        assert stats.published == 1
        assert stats.dropped == 0
        assert stats.exceptions == 0
        assert stats.python_boundary_crossings == 0
        assert stats.compile_cache_hits == 1
        assert stats.compile_cache_misses == 0
        artifact = Path(pipeline.compile_result["so"]).resolve()
        assert artifact.is_file()
        assert str(artifact) in Path("/proc/self/maps").read_text()

        # This joins the batch worker that performed the allocation. Its stack can
        # no longer keep an intentionally lost pointer reachable from LSan roots.
        pipeline.close()
        assert pipeline.closed

        if not after_session_close:
            # Python, Cling, the ROS context, and DDS remain live and reachable.
            # Exit-time leak checking is disabled by the invoking harness.
            check_result = _explicit_lsan_check()

    if after_session_close:
        # Drop every Python proxy after the managed close, but keep the interpreter
        # and Cling runtime alive. The generated-code allocation window is checked
        # after native session/context/DDS teardown without interpreter-exit noise.
        sink = source = executor = peer = node = pipeline = ros = None
        for _ in range(3):
            gc.collect()
        check_result = _explicit_lsan_check()

    evidence = {
        "schema": "rclcpp_kit.generated-lsan/v1",
        "case": case,
        "artifact": str(artifact),
        "artifact_loaded": True,
        "compile_cache_hit": True,
        "explicit_check_before_interpreter_shutdown": True,
        "session_closed_before_check": after_session_close,
        "python_proxies_dropped_before_check": after_session_close,
        "generated_messages": stats.processed,
        "python_boundary_crossings": stats.python_boundary_crossings,
        "leak_bytes": LEAK_BYTES if case == "intentional-leak" else 0,
        "lsan_check_result": check_result,
    }
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        "NATIVE_GENERATED_LSAN_CHECK case=%s result=%d scope=%s" % (
            case,
            check_result,
            "closed-session" if after_session_close else "live-runtime",
        ),
        flush=True,
    )

    expected_result = 1 if case == "intentional-leak" else 0
    if check_result != expected_result:
        raise AssertionError(
            "explicit LSan result for %s was %r, expected %d" % (
                case, check_result, expected_result))
    return check_result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prebuild", action="store_true")
    parser.add_argument("--case", choices=tuple(TRANSFORMS))
    parser.add_argument("--evidence", type=Path)
    parser.add_argument("--after-session-close", action="store_true")
    args = parser.parse_args()
    if args.prebuild:
        if (args.case is not None or args.evidence is not None
                or args.after_session_close):
            parser.error(
                "--prebuild does not accept --case, --evidence, or "
                "--after-session-close")
        prebuild()
        return 0
    if args.case is None or args.evidence is None:
        parser.error("--case and --evidence are required for a leak-check run")
    return run_case(
        args.case,
        args.evidence,
        after_session_close=args.after_session_close,
    )


if __name__ == "__main__":
    raise SystemExit(main())
