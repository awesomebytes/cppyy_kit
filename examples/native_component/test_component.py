"""Behavioral checks against Python arithmetic and an independent C++ driver."""
import json
import math
import subprocess

import pytest

from cppyy_kit._compile import compiler_command
from examples.native_component import component
from examples.native_component.component import Settings, Smoother


def reference(values, alpha):
    if not values:
        return []
    output = [values[0]]
    for value in values[1:]:
        output.append((1 - alpha) * output[-1] + alpha * value)
    return output


@pytest.mark.parametrize("alpha", [0.125, 0.5, 1.0])
def test_reference_chunking_empty_and_reset(alpha):
    values = [0.0, 2.0, 4.0, -2.0, 10.0]
    with Smoother(Settings(alpha)) as smoother:
        actual = smoother.process(values[:2])
        before = smoother.snapshot()
        assert smoother.process([]) == []
        assert smoother.snapshot() == before
        actual += smoother.process(values[2:])
        assert actual == pytest.approx(reference(values, alpha))
        assert smoother.snapshot() == {"initialized": True, "value": actual[-1], "samples": 5}
        smoother.reset()
        assert smoother.snapshot() == {"initialized": False, "value": 0.0, "samples": 0}
        assert smoother.process([20.0]) == [20.0]
        smoother.reset()
        smoother.reset()
        assert smoother.process(values) == actual


def test_instances_outputs_and_context_cleanup():
    with Smoother() as first, Smoother() as second:
        assert bool(first._native.__python_owns__)
        inputs = [1.0, 3.0]
        output = first.process(inputs)
        inputs[:] = [100.0, 100.0]
        assert second.process([20.0]) == [20.0]
        assert first.snapshot()["samples"] == 2
    assert output == [1.0, 2.0]
    first.close()
    for operation in (first.reset, first.snapshot, first.__enter__, lambda: first.process([])):
        with pytest.raises(RuntimeError, match="closed"):
            operation()
    with pytest.raises(ValueError, match="body"):
        with Smoother() as smoother:
            raise ValueError("body")
    with pytest.raises(RuntimeError, match="closed"):
        smoother.snapshot()


@pytest.mark.parametrize("value", [0.0, -1.0, 1.1, math.nan, math.inf, True, "0.5"])
def test_invalid_configuration_precedes_native_loading(value, monkeypatch):
    def forbidden():
        pytest.fail("invalid settings reached native loading")
    monkeypatch.setattr(component, "native_namespace", forbidden)
    with pytest.raises((TypeError, ValueError)):
        Smoother(Settings(value))
    unchecked = Settings()
    object.__setattr__(unchecked, "alpha", value)
    with pytest.raises((TypeError, ValueError)):
        Smoother(unchecked)


@pytest.mark.parametrize("invalid", [math.nan, math.inf, -math.inf, 1e6 + 1])
def test_native_rejection_preserves_complete_state(invalid):
    with Smoother() as smoother:
        smoother.process([1.0, 3.0])
        before = smoother.snapshot()
        with pytest.raises(Exception, match="finite"):
            smoother.process([5.0, invalid])
        assert smoother.snapshot() == before
        assert smoother.process([4.0]) == [3.0]


def test_native_configuration_is_checked_independently():
    native = component.native_namespace()
    config = native.Config()
    config.smoothing_weight = 0.0
    with pytest.raises(Exception, match="weight"):
        native.Smoother(config)


def test_settings_export_and_standalone_driver(tmp_path):
    settings = Settings(alpha=0.25)
    saved = tmp_path / "settings.json"
    saved.write_text(json.dumps({"alpha": settings.alpha}), encoding="utf-8")
    restored = Settings(**json.loads(saved.read_text(encoding="utf-8")))
    executable = tmp_path / "smoother"
    subprocess.run([*compiler_command(), "-std=c++17", "-O2",
                    str(component.ROOT / "smoother.cpp"), str(component.ROOT / "driver.cpp"),
                    "-o", str(executable)], check=True, capture_output=True, text=True, timeout=30)
    values = [0.0, 2.0, 4.0, -2.0]
    result = subprocess.run([str(executable), str(restored.alpha), *map(str, values)],
                            check=True, capture_output=True, text=True, timeout=5)
    with Smoother(restored) as smoother:
        assert list(map(float, result.stdout.splitlines())) == smoother.process(values)
