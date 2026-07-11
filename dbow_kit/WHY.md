# dbow_kit — WHY (seed)

> Seed. Full pitch folds in with the M2 SKILL.md pass. Evidence:
> [`REPORT.md`](REPORT.md); end-to-end: [`docs/tutorials/vision_loop_closure.md`](../docs/tutorials/vision_loop_closure.md).

DBoW2 is the canonical bag-of-binary-words place-recognition library behind
ORB-SLAM. It has **no Python binding** and is **not on conda-forge** — normally
that means writing and maintaining a pybind wrapper before you can touch it from
Python.

cppyy removes that step: dbow_kit vendors DBoW2's headers, compiles the small
`.so` once (`build-dbow2`), and mirrors its ORB API (`OrbVocabulary`,
`OrbDatabase`) directly in Python. Descriptors flow in from `cv_kit`'s C++
`cv::ORB` as `cv::Mat`s and never leave C++. The result: a real loop-closure
detector, assembled and orchestrated from a short Python script, running at C++
speed — the "impossible → possible" case for cppyy.
