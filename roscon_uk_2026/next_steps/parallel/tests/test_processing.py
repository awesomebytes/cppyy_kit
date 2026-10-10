"""Independent arithmetic and validation checks, with no timing assertions."""
import concurrent.futures
import gc
import math
from pathlib import Path
import sys

import numpy as np
import pytest
from pydantic import ValidationError

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from model import Detection, validate
from processing import Batch, lanes
from cppyy_kit import pydantic_structs as pyd


def records(n):
    return [dict(x=float((i % 29)-14)/2, y=float((i % 23)-11)/2,
                 z=float((i % 17)-8)/2, confidence=float(i % 11)/10, frame=i % 7)
            for i in range(n)]


def expected(data, frames=7, radius=5., threshold=.7):
    flags = [int(row["confidence"] >= threshold and
                 ((row["x"]*row["x"] + row["y"]*row["y"]) + row["z"]*row["z"]) <= radius*radius)
             for row in data]
    counts = [sum(flag for row, flag in zip(data, flags) if row["frame"] == f)
              for f in range(frames)]
    return flags, counts


@pytest.mark.parametrize("n", [0, 1, 3, 17, 257, 123457])
def test_exact_flags_and_counts_multiple_workers_repeated(n):
    data = records(n)
    batch = Batch(data, frames=7)
    flags, counts = expected(data)
    for method, workers in [("serial_aos", 1), ("serial_columns", 1), ("xsimd", 1),
                            ("tbb", 1), ("tbb", 2), ("tbb", 4)]:
        for _ in range(3):
            result, histogram, peak = batch.run(method, workers=workers, instrument=True)
            assert result.tolist() == flags
            assert histogram.tolist() == counts
            if method == "tbb":
                assert 0 <= peak <= workers
                if n:
                    assert peak >= 1


def test_every_simd_tail_and_boundaries():
    width = lanes()
    fixtures = [dict(x=3., y=4., z=0., confidence=.7, frame=0),
                dict(x=math.nextafter(5., math.inf), y=0., z=0., confidence=1., frame=1),
                dict(x=0., y=0., z=0., confidence=math.nextafter(.7, -math.inf), frame=2),
                dict(x=0., y=0., z=0., confidence=math.nextafter(.7, math.inf), frame=3)]
    for n in range(0, width*3+1):
        data = [fixtures[i % 4].copy() for i in range(n)]
        batch = Batch(data, frames=7)
        want_flags, want_counts = expected(data)
        for method in ("serial_aos", "serial_columns", "xsimd", "tbb"):
            flags, counts, _ = batch.run(method, workers=2)
            assert flags.tolist() == want_flags
            assert counts.tolist() == want_counts


@pytest.mark.parametrize("field,value", [("x", float("nan")), ("y", float("inf")),
                                         ("z", -1001.), ("confidence", -.001),
                                         ("confidence", 1.001), ("frame", -1),
                                         ("frame", 4096), ("frame", 1.5),
                                         ("frame", True), ("x", "1.0")])
def test_invalid_records_rejected_before_native_conversion(field, value):
    row = records(1)[0]
    row[field] = value
    with pytest.raises(ValidationError):
        Batch([row], frames=7)


def test_frames_and_bypassed_model_constraints():
    for frames in (0, 4097, True, 1.5):
        with pytest.raises(ValueError):
            validate([], frames)
    row = records(1)[0]
    row["frame"] = 7
    with pytest.raises(ValueError, match="outside"):
        Batch([row], frames=7)
    bad = Detection.model_construct(x=0., y=0., z=0., confidence=1.5, frame=0)
    with pytest.raises(ValidationError):
        validate([bad])
    with pytest.raises(ValidationError):
        Detection(**records(1)[0], extra=1)


def test_input_lifetime_and_source_mutation():
    data = records(10003)
    want_flags, want_counts = expected(data)
    batch = Batch(data, frames=7)
    data[0]["confidence"] = -123.
    del data
    gc.collect()
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        jobs = [pool.submit(batch.run, method, workers=2) for method in ("tbb", "xsimd")]
        for job in jobs:
            flags, counts, _ = job.result(timeout=20)
            assert flags.tolist() == want_flags
            assert counts.tolist() == want_counts
    assert all(not a.flags.writeable for a in batch._columns.values())


def test_prototype_general_and_columnar_paths_match():
    models = validate(records(31), frames=7)
    vector = pyd.cpp_vector(Detection, models)
    batch = Batch(records(31), frames=7)
    for field in Detection.model_fields:
        assert np.array_equal(pyd.column(vector, Detection, field),
                              pyd.column(batch._records, Detection, field))


def test_runtime_does_not_reenter_python_validation(monkeypatch):
    import processing
    batch = Batch(records(10003), frames=7)
    def forbidden(*args, **kwargs):
        raise AssertionError("validation was called while processing native storage")
    monkeypatch.setattr(processing, "validate", forbidden)
    for method in ("serial_aos", "serial_columns", "xsimd", "tbb"):
        batch.run(method, workers=2)


def test_columns_only_layout_and_fixed_size():
    batch = Batch(records(11), frames=7, layout="columns")
    assert batch.costs["aos_fill_ms"] == 0
    assert batch.native_bytes == 11*40
    assert batch.run("xsimd")[1].sum() == batch.run("serial_columns")[1].sum()
    with pytest.raises(ValueError):
        batch.run("tbb")
    with pytest.raises(AttributeError):
        batch.n = 123
    with pytest.raises(AttributeError):
        batch.frames = 123


@pytest.mark.parametrize("kwargs", [{"workers": 0}, {"workers": 33}, {"workers": True},
                                     {"radius": float("nan")}, {"radius": -1.},
                                     {"threshold": 1.1}, {"method": "unknown"}])
def test_invalid_settings(kwargs):
    with pytest.raises(ValueError):
        Batch([], frames=7).run(**kwargs)
