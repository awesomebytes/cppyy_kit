# 0.4.1 release status and the 0.4.0 upload incident

Version **0.4.1 is published** on the `awesomebytes` channel. Tag `v0.4.1`
points to `5668d4920706b226d3bd54e6548e65ca39b0bd45`. The release workflow
verified all twelve published package identities by exact SHA-256 and size.
A separate fresh public-channel Pixi project ran the documented NumPy and
BehaviorTree.CPP examples successfully.

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

The complete 0.4.1 release passed the normal gates; publication details follow
below. The failed 0.4.0 tag remains unchanged.

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

## Published 0.4.1 verification

The [release workflow](https://github.com/awesomebytes/cppyy_kit/actions/runs/38054631219),
[main CI](https://github.com/awesomebytes/cppyy_kit/actions/runs/38054631543)
and [strict documentation deployment](https://github.com/awesomebytes/cppyy_kit/actions/runs/38054631489)
all passed for the exact tagged source.

| Gate | 0.4.1 result |
|---|---|
| Main default suite | 370 passed, 161 optional skips |
| Core source tests, each architecture | 353 passed, 32 optional skips |
| rclcpp source tests, each architecture | 266 passed, no skips |
| PCL/OpenCV/OMPL native boundary lanes | 2 / 12 / 12 passed |
| Source sanitizers and installed core proofs, each architecture | Passed |
| Eleven suite artifact installation proofs and ARM bridge proof | Passed |
| Twelve provenance/SPDX SBOM gates | Passed |
| Published package byte verification | All twelve exact |
| Standalone installed NumPy / BehaviorTree.CPP examples | `14.0` / `True` |

The new ARM bridge identity is
`linux-aarch64/cppyy-3.5.0-py312h7e7ac48_3.conda`: 75,012 bytes,
SHA-256 `cc9d4a8012b95fa5bebd946008fdbc62488a95f1f8812a9bfbda6903fe784437`.
Downloaded workflow artifacts were independently compared with the postflight
hash and size records. Canonical package URLs, byte-verification evidence,
checksums and installed-environment provenance are preserved as assets on the
[GitHub release](https://github.com/awesomebytes/cppyy_kit/releases/tag/v0.4.1).
Temporary signed redirect URLs are omitted from the public evidence.

## Fresh installed workflow

The Linux x86_64 proof used the exact two-stage setup from the AI guide:

```bash
pixi init cppyy-example -c https://prefix.dev/awesomebytes -c conda-forge
cd cppyy-example
pixi add "cppyy-kit>=0.4.1" numpy
pixi run python -m cppyy_kit guide
pixi run python -m cppyy_kit status --environment
pixi run python kernel.py
pixi workspace channel add robostack-jazzy
pixi add ros-jazzy-bt-kit
pixi run python tree.py
```

The isolated proof directory was named `published-example041`; its package
origins were asserted to be under the fresh environment's `site-packages`,
and Conda metadata confirmed both suite packages were 0.4.1. `PYTHONPATH` was
cleared for execution, and task-specific `XDG_CACHE_HOME` and
`CPPYY_KIT_CACHE_DIR` directories isolated native caches. No compiler or library
path overrides were added. Pixi supplied GCC 14.3.0, Python 3.12.15 and NumPy
2.5.3. The inline `ConstNDArray[np.float64]` kernel returned `14.0`; the
BehaviorTree.CPP tree returned `True`. Task guides `accelerate`, `bring-library`
and `existing-cpp`, plus the installed BT API guide, were discovered successfully.
This proves the documented installed workflow; it makes no startup-time or
performance claim. ARM installed-artifact and source gates ran in CI; this
additional standalone NumPy/BT project was exercised on x86_64.

The separate `rclcppyy` checkout passed its bounded 17-test routine CI smoke
and a `ConstNDArray` probe against clean suite source `5668d492` on
Jazzy/CycloneDDS (domain 171, 83.95 seconds). Its certified package/source pins
remain 0.3.0; this source check does not establish full product parity or
qualify a new product release. The ROSCon source rehearsal retains its original
revision-bound measurements separately from this installed-package proof.
