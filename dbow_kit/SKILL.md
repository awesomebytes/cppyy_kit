# dbow_kit

Use DBoW2 from Python through cppyy for ORB place recognition. It is useful when
you want to compare image descriptors against a vocabulary without maintaining a
separate Python binding. Pair it with [`cv_kit`](../cv_kit/SKILL.md), which
provides C++ ORB descriptors.

The published Pixi package is `ros-jazzy-dbow-kit`. For package setup and the
supported Pixi environment, start with the [Getting Started guide](https://awesomebytes.github.io/cppyy_kit/getting-started/).

## Install and try the demo

From this repository checkout, DBoW2's source is built once in the `vision`
environment, then the existing loop demo exercises vocabulary training,
database queries, and loop confirmation:

```bash
pixi install -e vision
pixi run -e vision build-dbow2
pixi run -e vision demo-vision-loop
```

The deterministic synthetic sequence reports 19 confirmed revisits in 200
frames and needs no dataset download. See the [vision loop-closure
tutorial](../docs/tutorials/vision_loop_closure.md) for the stages and the
optional real-data route.

## API shapes

- `train_vocabulary(all_descriptors)` takes a sequence of nonempty C++ descriptor
  Mats. Obtain these from `cv_kit.create_orb(...).detect_and_compute(...)` on
  frames as shown by the existing feature and loop demos.
- `make_database(vocabulary)` creates a searchable database.
- `add_image(database, descriptors)` adds one image's `N x 32` ORB descriptor
  Mat (one 32-byte row per feature, stored as 8-bit unsigned values);
  `query(database, descriptors, max_results=...)` returns matching image IDs and
  scores.
- `load_vocabulary(path)` accepts a vocabulary file such as `ORBvoc.txt`; the
  first load also writes a binary cache beside the text file.

The [DBoW2 report](REPORT.md) and [`cv_kit/REPORT.md`](../cv_kit/REPORT.md)
document build and benchmark details.
