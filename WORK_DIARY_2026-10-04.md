# Work diary: 2026-10-04

## Remove the array marker

- Removed the `cpp.arr` annotation helper and its implementation branches. Numeric arrays use `NDArray[T]` or inferred numeric annotations. Explicit C++ pointer annotations remain available for advanced buffer types; callers supply a separate length argument when needed.
- Migrated the two existing pointer-and-size tests to typed NumPy annotations and updated the argument reference. Historical work records were preserved.
- Validation: focused decorator tests passed (18 tests, 1.61 seconds); the default suite passed (197 passed, 135 skipped, 7.69 seconds); lint and the strict documentation build passed.
- Proposed `Annotated[NDArray[T], "const"]` for read-only numeric inputs. This would generate `const T*`, retain dtype and layout checks, and accept both writable and read-only storage. The proposal is not implemented.
- The untracked RosCon rehearsal still uses published 0.3.x packages. Its environment and executable examples need updating together before it uses the new annotation API. No RosCon files were included in this change.
- Changes are on the local `feat/array-annotations` branch. No push or release publication was made.

## Const array annotations and example precision

- Added the public `ConstNDArray[T]` alias with a lazy package export. It uses NumPy's array typing, generates `const T*`, and accepts writable or read-only buffers. Mutable `NDArray[T]` continues to require writable storage.
- Retained dtype, native byte order, alignment, and C-contiguous layout checks. Constness remains part of inferred specializations and cache keys. Builtin scalar decoration remains NumPy-free.
- Updated the generated public typing stub and stub generator to preserve the generic alias as an explicit re-export. Installed-package proofs now exercise a read-only float64 input through the public alias.
- Changed the six introductory sum-of-squares examples to `ConstNDArray[np.float64]`. Float32 remains supported and tested; its smaller storage is useful for some workloads, but this example has no measured reason to narrow ordinary Python floating-point values.
- Validation: 105 focused tests passed in 5.58 seconds; the full default suite passed with 206 passed and 135 optional skips in 9.81 seconds. Lint and strict docs passed, and the exact README kernel printed `14.0`. After the explicit stub re-export adjustment, the three stub tests passed in 0.30 seconds and targeted lint passed.
- Built the 0.4.0 core packages locally in about 57 seconds. Fresh installed-environment proofs passed in about 5 seconds, including read-only numeric input and same-handle serialized publication.

## Ideas for using cppyy_kit with AI coding tools

- Added `CPPYY_KIT_WITH_AI.md` as a separate review draft. It covers alternatives, fewer manually maintained files, ordinary C++ and existing libraries, Python workflow tests, profiling, conversion and compilation costs, extraction into a native project, and suitable demonstration ideas.
- Checked alternative-tool descriptions against their official documentation. The author's priority for functional and integration tests is labelled directly, with focused boundary unit tests retained as complementary checks.
- Reviewed the draft's claims and local links. It stays outside the published site until the user reviews it alongside the talk.

## NumPy annotation namespace

- Moved `ConstNDArray` to `cppyy_kit.numpy_types`. The module also exports NumPy's `NDArray`, `ArrayLike`, and `DTypeLike` aliases and the supported numeric scalar dtype classes unchanged. General NumPy typing hints do not add new `@cpp` conversion rules.
- Removed the package-level const export and its lazy import and stub generation machinery. Importing `cppyy_kit` or decorating a builtin scalar function still does not import NumPy.
- The first examples now use plain `NDArray[np.float64]` imported from `cppyy_kit.numpy_types`. Read-only input is introduced later with `ConstNDArray[np.float64]` from the same module. Updated the installed proofs and the AI review draft to match.
- Validation: 106 focused tests passed in 5.68 seconds; the full default suite passed with 207 passed and 135 optional skips in 9.46 seconds. Lint and strict documentation build passed; the exact README example printed `14.0`.
- Built and proved the installed 0.4.0 core packages from the final namespace layout (about 60 seconds for the build and 5 seconds for the proof). The installed test imports `ConstNDArray` from `cppyy_kit.numpy_types` and exercises read-only float64 input.
- The user authorized pushing this branch. The RosCon folder remains untracked and unchanged.
