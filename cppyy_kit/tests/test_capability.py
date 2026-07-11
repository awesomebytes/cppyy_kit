#!/usr/bin/env python3
"""Tests for cppyy_kit.capability -- the detect/fallback/introspect registry.
Pure Python; runs in any env."""
import pytest

from cppyy_kit import capability


@pytest.fixture(autouse=True)
def _isolate_registry():
    saved = dict(capability._REGISTRY)
    try:
        yield
    finally:
        capability._REGISTRY.clear()
        capability._REGISTRY.update(saved)


def test_available_true_and_cached():
    calls = {"n": 0}

    def detect():
        calls["n"] += 1
        return True

    capability.register("t_ok", detect, "always on")
    assert capability.available("t_ok") is True
    assert capability.available("t_ok") is True
    assert calls["n"] == 1                      # probed once, then cached
    assert capability.available("t_ok", recheck=True) is True
    assert calls["n"] == 2                      # recheck re-probes


def test_unavailable_with_detail_tuple():
    capability.register("t_no", lambda: (False, "not built with X"), "optional X")
    assert capability.available("t_no") is False
    assert capability.detail("t_no") == "not built with X"


def test_probe_raise_is_unavailable_with_reason():
    def boom():
        raise RuntimeError("nope")

    capability.register("t_boom", boom)
    assert capability.available("t_boom") is False
    assert "probe raised: nope" in capability.detail("t_boom")


def test_set_state_records_adoption_outcome():
    capability.set_state("t_adopted", True, description="decided at bringup")
    assert capability.available("t_adopted") is True
    capability.set_state("t_adopted", False, "toolchain vanished")
    assert capability.available("t_adopted") is False
    assert capability.detail("t_adopted") == "toolchain vanished"


def test_unknown_capability_raises():
    with pytest.raises(KeyError):
        capability.available("does_not_exist")


def test_status_and_report_shape():
    capability.register("t_a", lambda: True, "cap a")
    capability.register("t_b", lambda: (False, "why"), "cap b")
    st = capability.status()
    assert st["t_a"]["available"] is True and st["t_a"]["description"] == "cap a"
    assert st["t_b"]["available"] is False and st["t_b"]["detail"] == "why"
    text = capability.report()
    assert "[yes] t_a: cap a" in text
    assert "[no ] t_b: cap b  -- why" in text


def test_base_compile_cache_registered():
    # The base always registers this one; in the test env (compiler + cppyy) it's on.
    st = capability.status()
    assert "compile_cache" in st
    assert st["compile_cache"]["available"] is True
