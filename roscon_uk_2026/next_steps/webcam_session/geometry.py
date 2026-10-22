"""Native masked depth unprojection for image/cloud experiments."""
import numpy as np

from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray, NDArray


@cpp(nogil=True)
def _unproject(depth: ConstNDArray[np.float32], mask: ConstNDArray[np.uint8],
               xyz: NDArray[np.float32], pixels: NDArray[np.int64],
               width: int, fx: float, fy: float, cx: float, cy: float,
               near: float, far: float) -> int:
    """
    int count = 0;
    for (std::size_t i = 0; i < depth_size; ++i) {
        const float z = depth[i];
        if (!mask[i] || !(z > near && z < far)) continue;
        xyz[3*count] = (double(i % width) - cx) * z / fx;
        xyz[3*count+1] = (double(i / width) - cy) * z / fy;
        xyz[3*count+2] = z;
        pixels[count] = i;
        ++count;
    }
    return count;
    """


def masked_depth_points(depth, mask, intrinsics, *, near=0.0, far=100.0):
    """Return float32 XYZ and flat source-pixel indices for selected valid depths.

    depth is axial Z, in the units desired for XYZ. Intrinsics describe this
    rectified image with zero skew. Predicted relative depth does not give metres.
    Inputs are normalized to contiguous float32/uint8 buffers when necessary.
    Output storage is allocated for the entire image; returned arrays are views.
    """
    depth = np.require(depth, dtype=np.float32, requirements=['C', 'A'])
    mask = np.asarray(mask, dtype=bool)
    if depth.ndim != 2 or mask.shape != depth.shape or not depth.size:
        raise ValueError('depth and mask must have the same nonempty HxW shape')
    if depth.size > np.iinfo(np.int32).max:
        raise ValueError('Image is too large for the native point count')
    k = np.asarray(intrinsics, dtype=np.float64)
    if (k.shape != (3, 3) or not np.isfinite(k).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1])
            or k[0, 1] != 0 or k[1, 0] != 0):
        raise ValueError('Expected finite zero-skew pinhole intrinsics')
    if not np.isfinite([near, far]).all() or not 0 <= near < far:
        raise ValueError('Expected finite bounds with 0 <= near < far')
    mask = np.require(mask, dtype=np.uint8, requirements=['C', 'A'])
    xyz = np.empty((depth.size, 3), dtype=np.float32)
    pixels = np.empty(depth.size, dtype=np.int64)
    count = _unproject(depth, mask, xyz, pixels, depth.shape[1],
                       k[0, 0], k[1, 1], k[0, 2], k[1, 2], near, far)
    return xyz[:count], pixels[:count]
