"""Independent acceptance checks. Default uses the saved solution."""
from __future__ import annotations
import argparse
import importlib
from pathlib import Path
from unittest.mock import patch
import numpy as np
from pydantic import ValidationError
from . import ROOT, native_namespace


def check(module_name):
    module = importlib.import_module(module_name)
    directory = ROOT / "build" / "exercise"
    directory.mkdir(parents=True, exist_ok=True)
    calls = 0

    def forbidden():
        nonlocal calls
        calls += 1
        raise AssertionError("invalid values reached the native boundary")

    with patch.object(module, "native_namespace", forbidden):
        for invalid in ({"tau_s": -1.}, {"tau_s": "0.08"}, {"max_gap_s": float("nan")},
                        {"tau_s": 0.4, "max_gap_s": 0.1}, {"frame_id": "bad frame"}, {"unrecognized": 1}):
            try:
                module.validate_and_export(invalid, directory / "invalid.cfg")
            except ValidationError:
                pass
            else:
                raise AssertionError(f"accepted invalid settings: {invalid!r}")
        assert calls == 0
    path = directory / "resolved.cfg"
    resolved, native = module.validate_and_export({"tau_s": .125, "frame_id": "map/camera"}, path)
    assert resolved.model_dump() == {"tau_s": .125, "max_gap_s": .5, "frame_id": "map/camera"}
    assert native.time_constant_s == .125
    assert native.reset_gap_s == .5
    assert str(native.output_frame) == "map/camera"
    # Reopen the file through the native deployment loader.
    deployed = native_namespace().load_config_file(str(path))
    estimator = native_namespace().PoseEstimator(deployed)
    timestamps = np.array([0, 125_000_000, 625_000_001], dtype=np.int64)
    positions = np.array([0., 0., 0., 1., 2., 3., 10., 20., 30.])
    output = np.empty_like(positions)
    estimator.process(timestamps, positions, 3, output)
    expected = np.array([0., 0., 0., *(np.array([1., 2., 3.])*(1-np.exp(-1))), 10., 20., 30.])
    np.testing.assert_allclose(output, expected, rtol=1e-14, atol=1e-14)
    assert estimator.snapshot().gap_resets == 1
    print("PASS: validation before native loading, explicit field/units mapping, default export, native-loader replay")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module", default="roscon_uk_2026.next_steps.reverse_core.solution")
    args = parser.parse_args()
    check(args.module)


if __name__ == "__main__":
    main()
