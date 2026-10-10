# 0.4.1 release status and the 0.4.0 upload incident

Version 0.4.1 is being prepared. Publication and public-channel installation
remain pending. Do not treat successful source or local-artifact tests as a
published release.

## 0.4.0 attempt

Tag `v0.4.0` points to `e7928667f40baa09efb91f49a7fc384a006bff99`.
The [release run](https://github.com/awesomebytes/cppyy_kit/actions/runs/38051439265)
passed source tests and sanitizers on Linux x86_64 and ARM64, all eleven suite
artifact installation proofs, the ARM cppyy bridge proof, and all twelve
provenance/SBOM gates. The upload preflight then stopped before any upload:

```text
remote package identity already exists with different bytes:
linux-aarch64/cppyy-3.5.0-py312h7e7ac48_2.conda
```

The bridge recipe summary and description changed after the published 0.3.0
release, while its build number remained 2. The upstream source, patch and
native component pins were unchanged. Rebuilding produced a different artifact
under an existing immutable package identity.

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| Failed 0.4.0 run's new bridge | 74,879 | `f90c74e01227ba068b5e909f41aa5e49491032ed9167e5f4b2a488ea1e0b8275` |
| Bridge from the successful published 0.3.0 run | 74,993 | `63f2bf6d0dc593db531453ba2a732011290fc9afb984c3d48993957b834bbeab` |

The second artifact was recovered from the retained
[successful 0.3.0 workflow](https://github.com/awesomebytes/cppyy_kit/actions/runs/36337183758).
The failed run's channel verifier established the identity collision. A local
attempt to download the public package separately received HTTP 403 after its
redirect; that local download did not establish an additional channel hash.

The valid existing bridge and failed 0.4.0 tag are retained. No remote artifact
was deleted or overwritten, and no byte-verification gate was bypassed.

## Mitigation and validation

The conventional follow-up release uses suite version 0.4.1 and ARM bridge build
3. Native implementation and component versions are unchanged. Package-build
and installed-proof helpers now read the suite workspace version instead of
embedding a stale version. The version bump helper updates the workflow's
suite SBOM matrix as well as recipes and build metadata.

The tagged 0.4.0 source results remain useful revision-bound evidence:

| Gate | Result |
|---|---|
| Main default suite | 369 passed, 161 optional skips |
| Core source tests, each architecture | 352 passed, 32 optional skips |
| rclcpp source tests, each architecture | 266 passed, no skips |
| PCL/OpenCV/OMPL native boundary lanes | 2 / 12 / 12 passed |
| Source sanitizers and installed core proofs, each architecture | Passed |
| Strict documentation build and Pages deployment | Passed |

The [main CI run](https://github.com/awesomebytes/cppyy_kit/actions/runs/38051439120)
and [documentation run](https://github.com/awesomebytes/cppyy_kit/actions/runs/38051439082)
passed. These results do not replace the new 0.4.1 release gates.

Next action: commit the reviewed metadata fix, tag 0.4.1, and run the complete
normal release workflow. Verify every published artifact by exact bytes and
run the documented NumPy and library examples in a fresh public-channel Pixi
project before marking publication complete.

Local 0.4.1 preparation passed 29 focused release/ARM-package/SBOM/upload tests,
the default suite (370 passed, 161 optional skips), lint, strict documentation,
and six rehearsal checks. The changed core build/proof helpers built both
0.4.1 artifacts and verified installed guide discovery, numeric kernels and
serialized same-handle publication outside the checkout. This check exposed a
missing Python dependency in the packaging environment; Python 3.12 is now
declared there. Existing native/default/documentation environment locks are
unchanged. Queries for the new ARM bridge build 3 and base 0.4.1 identities
returned HTTP 404 before the attempt, establishing that these identities were
absent at that time.
