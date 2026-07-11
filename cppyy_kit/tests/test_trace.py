#!/usr/bin/env python3
"""Tests for cppyy_kit.trace -- the 8a boundary tracer.

Pure-Python plus one cppyy crossing (a std::function wrap at the trivial
``int(int)`` signature, no domain library), so these run in the default env too.
"""
import json

import pytest

import cppyy_kit
from cppyy_kit import trace


@pytest.fixture(autouse=True)
def _clean_trace():
    """Ensure each test starts/ends with tracing off and the buffer empty."""
    trace._ENABLED = False
    trace._EVENTS = []
    yield
    trace._ENABLED = False
    trace._EVENTS = []


def test_off_by_default_is_noop():
    assert trace.enabled() is False
    trace.record("thing", signature="int(int)")     # must not accumulate
    span = trace.span("thing", signature="int(int)")
    span.done()
    assert trace._EVENTS == []


def test_records_when_on():
    trace.start()
    trace.record("point", detail="x")
    with trace.span("timed", signature="void()"):
        pass
    m = trace.stop()
    kinds = [e["kind"] for e in m["events"]]
    assert kinds == ["point", "timed"]
    # the span event carries a duration; the point event does not
    timed = next(e for e in m["events"] if e["kind"] == "timed")
    assert "duration_ms" in timed and timed["duration_ms"] >= 0
    assert "duration_ms" not in m["events"][0]


def test_manifest_shape_and_instantiations():
    trace.start()
    trace.record("callback", signature="bool(int)", duration_ms=10.0)
    trace.record("callback", signature="bool(int)", duration_ms=6.0)
    trace.record("std_function", signature="void(double)", duration_ms=4.0)
    trace.record("load_libraries", sonames=["liba.so", "libb.so"])
    trace.record("cppdef_cached", cached=True)
    trace.record("cppdef_cached", cached=False)
    m = trace.stop()

    assert m["version"] == 1
    assert m["event_count"] == 6
    by_kind = m["summary"]["by_kind"]
    assert by_kind["callback"]["count"] == 2
    assert by_kind["callback"]["total_ms"] == 16.0
    assert m["summary"]["libraries"] == ["liba.so", "libb.so"]
    assert m["summary"]["cache"] == {"hits": 1, "misses": 1}

    # instantiation manifest: distinct signatures, sorted by descending cost
    inst = m["instantiations"]
    assert [row["signature"] for row in inst] == ["bool(int)", "void(double)"]
    assert inst[0]["count"] == 2 and inst[0]["total_ms"] == 16.0


def test_stop_writes_json(tmp_path):
    out = tmp_path / "t.json"
    trace.start(str(out))
    trace.record("callback", signature="int(int)")
    trace.stop()
    assert out.exists()
    m = json.loads(out.read_text())
    assert m["event_count"] == 1


def test_report_formatter_renders():
    m = {
        "env_tag": "17.6.32.8", "event_count": 2, "duration_ms": 12.3,
        "summary": {"by_kind": {"callback": {"count": 1, "total_ms": 5.0}},
                    "libraries": ["libx.so"], "cache": {"hits": 1, "misses": 0}},
        "instantiations": [{"signature": "bool(int)", "count": 1, "total_ms": 5.0}],
    }
    text = trace._fmt_report(m)
    assert "boundary trace" in text
    assert "bool(int)" in text
    assert "1 hit(s), 0 miss(es)" in text
    assert "libx.so" in text


def test_std_function_crossing_is_traced():
    """The cppyy_kit crossing points feed the trace: a std_function wrap shows up
    with its C++ signature (and zero events when tracing is off)."""
    def inc(x):
        return x + 1

    # off: no event
    cppyy_kit.std_function("int(int)", inc)
    assert trace._EVENTS == []

    trace.start()
    fn = cppyy_kit.std_function("int(int)", inc)
    m = trace.stop()
    assert int(fn(41)) == 42
    sigs = [e.get("signature") for e in m["events"] if e["kind"] == "std_function"]
    assert "int(int)" in sigs
