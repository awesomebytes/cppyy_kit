# dbow_kit

Use DBoW2 from Python through cppyy for ORB place recognition and loop-closure
detection. DBoW2 has no Python binding and is not packaged on conda-forge.
[`cv_kit`](../cv_kit/WHY.md) provides the ORB descriptor Mats.

## Build

Build the vendored source once:

```bash
pixi run -e vision build-dbow2  # Builds build/vendor/libDBoW2.so
```

## Use

```python
import dbow_kit

voc = dbow_kit.train_vocabulary(all_descriptors)  # Small vocabulary; no download
voc = dbow_kit.load_vocabulary("data/ORBvoc.txt")  # Real ORBvoc; binary-cached
db = dbow_kit.OrbDatabase(voc)
db.add(dbow_kit.descriptors_from_mat(orb_descriptors))
```

ORB descriptors are 256 bits, or 32 bytes. An image uses an `Nx32 CV_8U`
`cv::Mat`. DBoW2 expects one `1x32` Mat per feature in a
`std::vector<cv::Mat>`. `descriptors_from_mat` creates that vector in C++.

See the [vision loop-closure tutorial](../docs/tutorials/vision_loop_closure.md)
for the complete pipeline.
