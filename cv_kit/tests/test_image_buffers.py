"""OpenCV image-buffer contracts; these checks do not require DBoW2."""
import gc
import os
import sys
import weakref
from types import SimpleNamespace

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    not os.path.isdir(os.path.join(os.environ.get("CONDA_PREFIX", ""),
                                  "include", "opencv4")),
    reason="OpenCV is not installed (use the vision environment)",
)


@pytest.fixture(scope="module")
def cv():
    import cv_kit
    return cv_kit.bringup_cv()


def image_message(width, height, step, data, encoding="mono8", endian=0):
    import cppyy
    from rclcpp_kit.bringup_rclcpp import add_ros2_include_paths
    add_ros2_include_paths()
    cppyy.include("sensor_msgs/msg/image.hpp")
    message = cppyy.gbl.sensor_msgs.msg.Image()
    message.width, message.height, message.step = width, height, step
    message.encoding, message.is_bigendian = encoding, endian
    message.data = data
    return message


@pytest.mark.parametrize("width,height,step,data,reason", [
    (2, 2, 2, [1], "data length"),
    (2, 2, 2, [1, 2, 3, 4, 5], "data length"),
    (3, 1, 2, [1, 2], "step"),
])
def test_image_rejects_invalid_buffer(cv, width, height, step, data, reason):
    import cv_kit
    message = image_message(width, height, step, data)
    with pytest.raises(ValueError, match=reason):
        cv_kit.msg_to_mat(message)


@pytest.mark.parametrize("dimension", [-1, 2**31])
def test_image_rejects_dimensions_before_native_constructor(cv, dimension):
    import cv_kit
    message = image_message(1, 1, 1, [0])
    fields = SimpleNamespace(width=dimension, height=1, step=1,
                             data=message.data, encoding="mono8", is_bigendian=0)
    with pytest.raises(ValueError, match="dimensions"):
        cv_kit.msg_to_mat(fields)


@pytest.mark.parametrize("channels,encoding", [(1, "mono8"), (3, "rgb8"), (4, "rgba8")])
def test_padded_image_rows_and_owner_lifetime(cv, channels, encoding):
    import cv_kit
    width, height = 3, 2
    step = width * channels + 2
    storage = np.arange(height * step, dtype=np.uint8).reshape(height, step)
    expected = storage[:, :width * channels].copy()
    expected = expected.reshape(height, width, channels) if channels > 1 else expected
    message = image_message(width, height, step, storage.ravel().tolist(), encoding)
    message_ref = weakref.ref(message)
    mat = cv_kit.msg_to_mat(message)
    view = cv_kit.mat_to_numpy(mat, copy=False)
    owned = cv_kit.mat_to_numpy(mat, copy=True)
    assert view.strides[0] == step
    assert np.array_equal(view, expected)
    message.data[0] = 201
    assert int(view.flat[0]) == 201
    assert np.array_equal(owned, expected)
    del message, mat
    gc.collect()
    assert message_ref() is not None
    assert int(view.flat[0]) == 201
    del view
    gc.collect()
    assert message_ref() is None
    assert np.array_equal(owned, expected)


def test_mono16_requires_native_byte_order_and_rejects_numpy_conversion(cv):
    import cv_kit
    native_endian = int(sys.byteorder == "big")
    message = image_message(2, 1, 4, [52, 18, 120, 86], "mono16", native_endian)
    mat = cv_kit.msg_to_mat(message)
    assert int(mat.depth()) != int(cv_kit._glue().CV_8U_)
    copied = cv_kit.mat_to_msg(mat, encoding="mono16")
    assert ord(copied.is_bigendian) == native_endian
    assert int(cv_kit.msg_to_mat(copied).depth()) == int(mat.depth())
    with pytest.raises(ValueError, match="unsigned 8-bit"):
        cv_kit.mat_to_numpy(mat)
    message.is_bigendian = 1 - native_endian
    with pytest.raises(ValueError, match="byte order"):
        cv_kit.msg_to_mat(message)
    message.is_bigendian = 2
    with pytest.raises(ValueError, match="is_bigendian"):
        cv_kit.msg_to_mat(message)
    message.is_bigendian = native_endian
    message.step = 5
    message.data = [52, 18, 120, 86, 0]
    with pytest.raises(ValueError, match="multiple of two"):
        cv_kit.msg_to_mat(message)


def test_submatrix_uses_only_final_pixel_extent_and_retains_owner(cv):
    import cv_kit
    source = np.arange(3 * 5 * 3, dtype=np.uint8).reshape(3, 5, 3).copy()
    expected = source[1:, 1:, :].copy()
    source_ref = weakref.ref(source)
    parent = cv_kit.numpy_to_mat(source)
    roi = parent(cv.Rect(1, 1, 4, 2))
    # OpenCV owns a reference to native Mat storage, but this Mat aliases NumPy.
    # Retain the parent carrying that external owner's pin.
    import cppyy_kit
    cppyy_kit.keep_alive(roi, parent)
    view = cv_kit.mat_to_numpy(roi, copy=False)
    owned = cv_kit.mat_to_numpy(roi)
    assert np.array_equal(view, expected)
    assert view.strides == (15, 3, 1)
    assert len(view.base) == 27  # Old rows * step exposed 30 bytes, beyond source.
    assert source.ctypes.data + source.nbytes == view.ctypes.data + len(view.base)
    del roi, parent, source
    gc.collect()
    assert source_ref() is not None
    assert np.array_equal(view, expected)
    del view
    gc.collect()
    assert source_ref() is None
    assert np.array_equal(owned, expected)


def test_empty_mats_and_images(cv):
    import cv_kit
    for mat in (cv.Mat(), cv_kit.numpy_to_mat(np.empty((0, 3), dtype=np.uint8)),
                cv_kit.msg_to_mat(image_message(0, 0, 0, []))):
        for copy in (True, False):
            array = cv_kit.mat_to_numpy(mat, copy=copy)
            assert array.size == 0 and array.dtype == np.uint8


def test_nonempty_null_mat_is_rejected(cv):
    import cppyy
    import cv_kit
    cppyy.cppdef("""
        namespace image_buffer_tests {
        cv::Mat null_data() {
            cv::Mat image(2, 2, CV_8UC1);
            image.data = nullptr;
            return image;
        }
        }
    """)
    mat = cppyy.gbl.image_buffer_tests.null_data()
    with pytest.raises(ValueError, match="null data buffer"):
        cv_kit.mat_to_numpy(mat)
