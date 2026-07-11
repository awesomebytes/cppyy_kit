#!/usr/bin/env python3
"""Tests for cppyy_kit.stubgen -- the .pyi generator. Pure Python + a bt_kit import
(bt_kit imports without bringup), so this runs in the default env."""
import ast
import types

from cppyy_kit import stubgen


def _valid_python(text):
    ast.parse(text)                  # a .pyi is valid Python syntax; must parse
    return text


def _fake_module():
    mod = types.ModuleType("fakekit")

    def do_thing(a, b=1, *rest):
        pass
    do_thing.__module__ = "fakekit"

    class Widget:
        def __init__(self, x):
            pass

        def poke(self):
            pass

        def _hidden(self):
            pass
    Widget.__module__ = "fakekit"

    mod.do_thing = do_thing
    mod.Widget = Widget
    mod.LIMIT = 7
    mod.NAME = "hi"
    mod._private = 3                 # excluded (underscore)
    import os as _os
    mod.os = _os                     # excluded (foreign module)
    return mod


def test_shapes_functions_classes_constants():
    text = _valid_python(stubgen.stub_module(_fake_module()))
    assert "from typing import Any" in text
    assert "LIMIT: int" in text and 'NAME: str' in text
    assert "def do_thing(a, b = ..., *rest) -> Any: ..." in text
    assert "class Widget:" in text
    assert "def poke(self) -> Any: ..." in text
    assert "def __init__(self, x) -> Any: ..." in text
    assert "_hidden" not in text          # underscore method excluded
    assert "_private" not in text and "def os" not in text


def test_bt_kit_public_surface():
    import bt_kit
    text = _valid_python(stubgen.stub_module(bt_kit))
    for sym in ("SUCCESS: int", "def bringup_bt(", "def warmup(", "class BtXmlError(ValueError):"):
        assert sym in text, "missing %r" % sym


def test_cppyy_kit_reexports_captured():
    # The base package re-exports from submodules (.cache/.require/._cpp/.nogil);
    # those must appear in its stub, not just names defined in __init__.
    import cppyy_kit
    text = _valid_python(stubgen.stub_module(cppyy_kit))
    for sym in ("def cppdef_cached(", "def require(", "def cpp(", "def nogil(",
                "def callback(", "class HandleRegistry:"):
        assert sym in text, "missing %r" % sym
