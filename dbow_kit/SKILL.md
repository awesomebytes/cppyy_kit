# dbow_kit API reference

Use DBoW2 from Python through cppyy for ORB place recognition. It is useful when
you want to compare image descriptors against a vocabulary without maintaining a
separate Python binding. Pair it with
[`cv_kit`](https://awesomebytes.github.io/cppyy_kit/cv_kit/SKILL/), which
provides C++ ORB descriptors.

`ns = dbow_kit.bringup_dbow()` loads DBoW2 and returns the C++ helper namespace
`rclcppyy_dbow`, including `OrbVocabulary` and `OrbDatabase` typedefs.

In a Pixi project configured with the channels in
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/), install
`pixi add ros-jazzy-dbow-kit` and run scripts with
`pixi run python your_script.py`. See the
[kit overview](https://awesomebytes.github.io/cppyy_kit/dbow_kit/WHY/).

The current package recipe installs the Python wrapper and guides, but does not
include the DBoW2 shared library and headers. `bringup_dbow()` resolves those files
under the checkout's `build/vendor` directory. The standalone installed package
cannot bring up DBoW2 yet. Use the repository build and demo commands below for a
working native setup.

## Repository demo

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
tutorial](https://awesomebytes.github.io/cppyy_kit/docs/tutorials/vision_loop_closure/)
for the stages and the
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

The [DBoW2 report](https://awesomebytes.github.io/cppyy_kit/dbow_kit/REPORT/) and
[OpenCV report](https://awesomebytes.github.io/cppyy_kit/cv_kit/REPORT/)
document build and benchmark details.
