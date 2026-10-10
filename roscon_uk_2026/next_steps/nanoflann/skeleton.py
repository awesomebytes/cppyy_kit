"""Complete the retained index described by LIBRARY_RECIPE.md.

Keep acceptance.py unchanged. Use its independent NumPy reference only for
verification. It is not an implementation of the required native operation.
"""
import numpy as np


class Index:
    def __init__(self, xyz, frames, confidence):
        """Validate shape/values; copy into immutable native owned storage.

        xyz: (n,3) coordinates. frames: (n,) integers. confidence: (n,) floats.
        Build a retained nanoflann KDTreeSingleIndexAdaptor with dimension 3.
        """
        raise NotImplementedError("Build and retain the native index")

    def query(self, xyz, frames, k=4, frame_gap=-1, min_confidence=0.0,
              max_distance=np.inf):
        """Batch exact filtered kNN plus centroid in one native call.

        Return ids/distances_squared (m,k), counts (m,), centroids (m,3).
        Pad missing ids with -1 and squared distances with infinity.
        Empty centroids contain NaN. Sort by (squared distance, original index).
        """
        raise NotImplementedError("Implement the native selection result set")

    def close(self):
        """Release storage; repeated close succeeds; query after close fails."""
        raise NotImplementedError("Define native teardown")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
