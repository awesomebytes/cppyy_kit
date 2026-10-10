"""Deterministic checks for configuration, native deployment, and buffer lifetime."""
import gc
import json
from pathlib import Path
import subprocess

import numpy as np
import pytest
from pydantic import ValidationError

import roscon_uk_2026.next_steps.reverse_core as core
from roscon_uk_2026.next_steps.reverse_core import (
    FilterConfig, PoseFilter, driver_replay, export_config, load_config, make_episode,
)


@pytest.mark.parametrize("values", [
    {"tau_s": 0}, {"tau_s": -0.1}, {"tau_s": float("nan")},
    {"max_gap_s": float("inf")}, {"max_gap_s": 0.01},
    {"tau_s": "0.08"}, {"tau_s": True}, {"frame_id": "bad frame"},
    {"frame_id": ""}, {"frame_id": 123}, {"frame_id": "x"*129}, {"unknown": 1},
])
def test_invalid_config_never_loads_native(monkeypatch, values):
    def forbidden():
        pytest.fail("invalid configuration reached native loading")
    monkeypatch.setattr(core, "native_namespace", forbidden)
    with pytest.raises(ValidationError):
        PoseFilter(values)


def test_unvalidated_model_rechecked(monkeypatch):
    monkeypatch.setattr(core, "native_namespace", lambda: pytest.fail("invalid model loaded native"))
    with pytest.raises(ValidationError):
        PoseFilter(FilterConfig.model_construct(tau_s=-1))
    with pytest.raises(ValidationError):
        PoseFilter(FilterConfig().model_copy(update={"frame_id": "bad frame"}))


def test_resolved_config_roundtrip_and_native_fields(tmp_path):
    config = FilterConfig(tau_s=0.125, max_gap_s=2, frame_id="map/camera_1")
    assert load_config(export_config(config, tmp_path / "config.cfg")) == config
    assert FilterConfig.model_validate_json(config.model_dump_json()) == config
    with pytest.raises(ValidationError):
        config.tau_s = 2
    native = core.to_native_config(config)
    assert native.time_constant_s == 0.125
    assert native.reset_gap_s == 2
    assert str(native.output_frame) == "map/camera_1"
    loaded = core.native_namespace().load_config_file(str(tmp_path / "config.cfg"))
    assert loaded.time_constant_s == native.time_constant_s
    assert loaded.reset_gap_s == native.reset_gap_s
    assert str(loaded.output_frame) == str(native.output_frame)


@pytest.mark.parametrize("contents", [
    "", "pose_filter_config_v1\ntau_s=.1\nmax_gap_s=.5\nframe_id=world\nextra\n",
    "pose_filter_config_v1\nmax_gap_s=.5\ntau_s=.1\nframe_id=world\n",
    "pose_filter_config_v1\ntau_s=nan\nmax_gap_s=.5\nframe_id=world\n",
])
def test_python_and_native_loader_reject_bad_config(tmp_path, contents):
    path = tmp_path / "invalid.cfg"
    path.write_text(contents)
    with pytest.raises((ValueError, ValidationError)):
        load_config(path)
    with pytest.raises(Exception):
        core.native_namespace().load_config_file(str(path))


def test_seconds_metres_and_exact_gap_threshold():
    config = FilterConfig(tau_s=0.1, max_gap_s=0.5)
    timestamps = np.array([0, 100_000_000, 600_000_000, 1_100_000_001], dtype=np.int64)
    positions = np.array([[0., 2., -1.], [1., 4., 1.], [2., 6., 3.], [10., 8., 7.]])
    with PoseFilter(config) as filt:
        output = filt.process(timestamps, positions)
        expected = np.empty_like(positions)
        expected[0] = positions[0]
        expected[1] = positions[0] + (-np.expm1(-1))*(positions[1]-positions[0])
        expected[2] = expected[1] + (-np.expm1(-5))*(positions[2]-expected[1])
        expected[3] = positions[3]
        np.testing.assert_allclose(output, expected, rtol=1e-14, atol=1e-14)
        assert filt.snapshot()["gap_resets"] == 1


def test_timestamp_int64_extremes_do_not_overflow():
    timestamps = np.array([np.iinfo(np.int64).min, np.iinfo(np.int64).max], dtype=np.int64)
    positions = np.array([[1., 2., 3.], [4., 5., 6.]])
    with PoseFilter() as filt:
        np.testing.assert_array_equal(filt.process(timestamps, positions), positions)
        assert filt.snapshot()["gap_resets"] == 1


def test_finite_extreme_positions_remain_finite():
    positions = np.array([[1e308, -1e308, 0], [-1e308, 1e308, 1e308]])
    with PoseFilter() as filt:
        result = filt.process(np.array([0, 80_000_000], dtype=np.int64), positions)
        assert np.isfinite(result).all()


def test_native_rejects_invalid_batch_transactionally():
    ns = core.native_namespace()
    estimator = ns.PoseEstimator(ns.Config())
    good_t = np.array([1], dtype=np.int64)
    good_x = np.array([1., 2., 3.])
    estimator.process(good_t, good_x, 1, np.empty(3))
    before = estimator.snapshot()
    for timestamps, positions in [
        (np.array([2, 2], dtype=np.int64), np.ones(6)),
        (np.array([2, 3], dtype=np.int64), np.array([1., 2., 3., 4., np.nan, 6.])),
    ]:
        output = np.full(6, -123.)
        with pytest.raises(Exception):
            estimator.process(timestamps, positions, 2, output)
        after = estimator.snapshot()
        assert after.last_timestamp_ns == before.last_timestamp_ns
        assert after.samples_processed == before.samples_processed
        np.testing.assert_array_equal(output, -123.)


def test_shapes_dtypes_no_mutation_and_empty():
    with PoseFilter() as filt:
        filt.process(np.array([1], dtype=np.int64), np.array([[1., 2., 3.]]))
        before = filt.snapshot()
        invalid = [
            (np.array([2, 1], dtype=np.int64), np.ones((2, 3)), ValueError),
            (np.array([2, 3], dtype=np.int64), np.array([[1., 2., 3.], [1., np.inf, 3.]]), ValueError),
            (np.array([1], dtype=np.int64), np.ones((1, 3)), ValueError),
            (np.array([2], dtype=np.int32), np.ones((1, 3)), TypeError),
            (np.array([2], dtype=np.int64), np.ones((1, 3), dtype=np.float32), TypeError),
            (np.array([2], dtype=np.int64), np.ones((1, 2)), ValueError),
            (np.array([[2]], dtype=np.int64), np.ones((1, 3)), ValueError),
            ([2], np.ones((1, 3)), TypeError),
        ]
        for t, x, exception in invalid:
            with pytest.raises(exception):
                filt.process(t, x)
            assert filt.snapshot() == before
        result = filt.process(np.empty(0, dtype=np.int64), np.empty((0, 3), dtype=np.float64))
        assert result.shape == (0, 3)
        assert filt.snapshot() == before


def test_noncontiguous_readonly_buffers_and_owning_output():
    episode = make_episode(samples=120)
    timestamps = episode["timestamps_ns"][::2]
    positions = episode["positions_m"][::2]
    expected_input = positions.copy()
    timestamps.setflags(write=False)
    positions.setflags(write=False)
    with PoseFilter() as filt:
        output = filt.process(timestamps, positions)
    assert output.flags.owndata and output.flags.c_contiguous
    assert not np.shares_memory(output, positions)
    np.testing.assert_array_equal(positions, expected_input)
    del timestamps, positions, episode
    gc.collect()
    assert np.isfinite(output).all()
    # cppyy also accepts truly contiguous readonly arrays directly.
    t = np.array([1, 2], dtype=np.int64)
    x = np.ones((2, 3)); t.setflags(write=False); x.setflags(write=False)
    with PoseFilter() as filt:
        np.testing.assert_array_equal(filt.process(t, x), x)


def test_chunk_reset_independence_and_close():
    episode = make_episode(seed=7, samples=175)
    t, x = episode["timestamps_ns"], episode["positions_m"]
    with PoseFilter() as full, PoseFilter() as chunked:
        expected = full.process(t, x)
        result = np.concatenate([chunked.process(t[:21], x[:21]), chunked.process(t[21:100], x[21:100]),
                                 chunked.process(t[100:], x[100:])])
        np.testing.assert_array_equal(result, expected)
        chunked.reset()
        assert chunked.snapshot()["samples_processed"] == 0
        np.testing.assert_array_equal(chunked.process(t, x), expected)
        assert bool(chunked._estimator.__python_owns__)
    chunked.close()
    for operation in (chunked.reset, chunked.snapshot, chunked.__enter__, lambda: chunked.process(t, x)):
        with pytest.raises(RuntimeError):
            operation()
    for _ in range(100):
        with PoseFilter() as filt:
            filt.process(t[:1], x[:1])


@pytest.mark.parametrize("samples", [0, 1, 125])
def test_driver_matches_cppyy(tmp_path, samples):
    episode = make_episode(seed=4, samples=samples)
    config = FilterConfig(tau_s=0.123, max_gap_s=0.6, frame_id="map/local")
    with PoseFilter(config) as filt:
        expected = filt.process(episode["timestamps_ns"], episode["positions_m"])
    actual = driver_replay(config, episode["timestamps_ns"], episode["positions_m"], tmp_path)
    np.testing.assert_array_equal(actual, expected)


def test_deterministic_varied_synthetic_episodes():
    first = make_episode(seed=10, samples=180)
    repeated = make_episode(seed=10, samples=180)
    different = make_episode(seed=11, samples=180)
    for key in ("timestamps_ns", "positions_m", "truth_m"):
        np.testing.assert_array_equal(first[key], repeated[key])
        assert not np.array_equal(first[key], different[key])
    assert first["episode_id"] == "synthetic-pose-v1-seed-10-n-180"
    assert np.max(np.diff(first["timestamps_ns"])) > 500_000_000
    assert first["positions_m"].dtype == np.float64
    assert first["timestamps_ns"].dtype == np.int64
    assert set(core.TRAINING_SEEDS).isdisjoint(core.HELD_OUT_SEEDS)


def test_native_driver_rejects_invalid_input_without_output(tmp_path):
    paths = core.build_native()
    config = export_config(FilterConfig(), tmp_path / "config.cfg")
    input_path = tmp_path / "bad.csv"
    input_path.write_text("timestamp_ns,x_m,y_m,z_m\n1,0,0,0\n1,1,1,1\n")
    output = tmp_path / "output.csv"
    result = subprocess.run([str(paths["driver"]), str(config), str(input_path), str(output)], capture_output=True, text=True)
    assert result.returncode == 2
    assert "strictly increase" in result.stderr
    assert not output.exists()


def test_unaligned_buffers_are_copied_before_native_borrow():
    timestamps = np.ndarray((2,), dtype=np.int64, buffer=bytearray(17), offset=1)
    positions = np.ndarray((2, 3), dtype=np.float64, buffer=bytearray(49), offset=1)
    timestamps[:] = [0, 100_000_000]
    positions[:] = [[0., 0., 0.], [1., 2., 3.]]
    assert not timestamps.flags.aligned and not positions.flags.aligned
    aligned_t, aligned_x = core._arrays(timestamps, positions)
    assert aligned_t.flags.aligned and aligned_x.flags.aligned
    assert not np.shares_memory(timestamps, aligned_t)
    assert not np.shares_memory(positions, aligned_x)
    with PoseFilter() as filt:
        output = filt.process(timestamps, positions)
    np.testing.assert_allclose(output[1], positions[1]*(1-np.exp(-.1/.08)), rtol=1e-14)
