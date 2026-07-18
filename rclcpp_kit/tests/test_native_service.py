import pytest

from _run_helper import format_output, run_helper
import cppyy_kit
from rclcpp_kit.native_service import _compile_native_glue, create_native_service


def test_empty_callback_is_rejected_before_type_resolution():
    with pytest.raises(ValueError, match="callback_body"):
        create_native_service(None, None, object, "service", "  ")


def test_native_glue_prebuild_failure_uses_cppdef_fallback(
        monkeypatch, tmp_path):
    failure = cppyy_kit._compile.CompileError("expected compiler failure")
    expected = {"cached": False, "reason": "build-failed", "so": None}
    calls = []

    def fail_prebuild(code, **options):
        raise failure

    def fallback(code, **options):
        calls.append((code, options))
        return expected

    monkeypatch.setattr(cppyy_kit, "prebuild", fail_prebuild)
    monkeypatch.setattr(cppyy_kit, "cppdef_cached", fallback)
    options = {
        "decls": "int native_fallback();",
        "name": "native_fallback",
        "include_paths": (),
        "libraries": (),
        "directory": str(tmp_path),
    }

    result = _compile_native_glue("int native_fallback() { return 1; }", options)

    assert result is expected
    assert calls == [("int native_fallback() { return 1; }", options)]


def test_native_service_interoperates_with_stock_client():
    proc = run_helper("_native_service_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_SERVICE_OK" in proc.stdout
    assert "NATIVE_SERVICE_TEARDOWN_OK" in proc.stdout
