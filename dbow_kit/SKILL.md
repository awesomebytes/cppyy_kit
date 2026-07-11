# dbow_kit — SKILL (seed)

> Seed cheat sheet. A full LLM-facing SKILL.md is an **M2** deliverable
> (PLAN §M2). This points at the authoritative sources.

**What:** drive DBoW2 (Gálvez-López & Tardós, "Bags of Binary Words") from
Python via cppyy for ORB place recognition / loop-closure detection. Pairs with
[`cv_kit`](../cv_kit/) (which produces the ORB descriptor Mats).

**Why (impossible → possible):** DBoW2 has **no Python binding** and is **not**
packaged on conda-forge. cppyy makes it drivable from Python without writing a
binding. See [`WHY.md`](WHY.md) and [`REPORT.md`](REPORT.md).

**One-time build (vendored from source):**
```
pixi run -e vision build-dbow2      # compiles dbow_kit/cpp/build_dbow2.py -> build/vendor/libDBoW2.so
```

**Use:**
```python
import dbow_kit
voc = dbow_kit.train_vocabulary(all_descriptors)   # small, zero-download; or
voc = dbow_kit.load_vocabulary("data/ORBvoc.txt")  # real ORBvoc (binary-cached)
db = dbow_kit.OrbDatabase(voc)
db.add(dbow_kit.descriptors_from_mat(orb_descriptors))   # Nx32 CV_8U -> vector<cv::Mat>
```

**Descriptor layout:** ORB = 256-bit = 32 bytes; one image is an `Nx32 CV_8U`
`cv::Mat`. DBoW2 wants a `std::vector<cv::Mat>` of `1x32` rows —
`descriptors_from_mat` does that split in C++.

**Full story:** [`docs/tutorials/vision_loop_closure.md`](../docs/tutorials/vision_loop_closure.md).
Demos live in [`cv_kit/demos/`](../cv_kit/demos/) (the loop-closure pipeline is joint cv+dbow).
