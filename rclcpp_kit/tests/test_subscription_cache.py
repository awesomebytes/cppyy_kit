#!/usr/bin/env python3
"""Tests for rclcpp_kit.subscription_cache -- the compiled, cached subscription
trampoline that removes the per-message-type ``create_subscription<MsgT>`` JIT.

The pure-source/naming/path tests are hermetic. The miss-path test redirects the
cache to a tmpdir and mocks the background spawn, so no real ``.so`` is built. The
end-to-end delivery of a cached trampoline is covered by the pub/sub roundtrip suite
(test_pubsub_roundtrip), which routes through this cache on a warm run.
"""
import os

import pytest

from rclcpp_kit import subscription_cache as sc


def test_trampoline_source_is_well_formed():
    fn, code, decls = sc.trampoline_source("sensor_msgs::msg::Image", "sensor_msgs/msg/image.hpp")
    assert fn == "rk_make_sub_sensor_msgs_msg_Image"
    for text in (code, decls):
        assert "#include <rclcpp/rclcpp.hpp>" in text
        assert "#include <sensor_msgs/msg/image.hpp>" in text
        assert fn in text
    # The definition instantiates the template; the decls are bodiless.
    assert "create_subscription<sensor_msgs::msg::Image>" in code
    assert "create_subscription<" not in decls
    assert code.count("{") >= 1 and "{" not in decls


def test_fn_name_and_package_are_symbol_safe():
    assert sc._fn_name("std_msgs::msg::String") == "rk_make_sub_std_msgs_msg_String"
    assert sc._package_of("sensor_msgs::msg::Image") == "sensor_msgs"
    # A templated/spaced type still yields a valid C identifier.
    assert " " not in sc._fn_name("a::b<c, d>")
    assert "::" not in sc._fn_name("a::b")


def test_cache_dir_is_under_xdg_and_versioned(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path))
    d = sc._cache_dir()
    assert d.startswith(str(tmp_path / "cppyy_kit" / "subs"))
    # version-tagged final component (matches the compile cache's tag)
    assert os.path.basename(d)  # non-empty tag


def test_miss_returns_none_and_schedules_background_build(monkeypatch, tmp_path):
    # An empty cache dir -> miss: make_subscription returns None (caller falls back to
    # the plain path this run) and schedules a background build for next run.
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path))
    monkeypatch.setattr(sc, "_LOADED", set())
    monkeypatch.setattr(sc, "_PENDING", {})
    spawned = []
    monkeypatch.setattr(sc.subprocess, "Popen", lambda *a, **k: spawned.append(a[0]))

    # node/callback are unused on the miss path (it returns before the trampoline call).
    result = sc.make_subscription(None, "sensor_msgs::msg::Image",
                                  "sensor_msgs/msg/image.hpp", "t", None, None)
    assert result is None
    assert "sensor_msgs::msg::Image" in sc._PENDING

    sc._build_pending_at_exit()
    assert len(spawned) == 1
    cmd = spawned[0]
    assert cmd[:3] == [os.sys.executable, "-m", "rclcpp_kit._sub_prebuild"]
    assert cmd[3] == "sensor_msgs::msg::Image" and cmd[4] == "sensor_msgs/msg/image.hpp"


def test_message_header_derivation():
    # Needs the Python message class; runs in the rclcpp env where sensor_msgs exists.
    sensor_msgs = pytest.importorskip("sensor_msgs.msg")
    from rclcpp_kit.bringup_rclcpp import message_header
    assert message_header(sensor_msgs.Image) == "sensor_msgs/msg/image.hpp"
    # A non-message object yields None (caller falls back to the plain path).
    assert message_header(object()) is None
