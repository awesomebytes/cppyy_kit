# Work diary: 2026-10-04

## Remove the array marker

- Removed the `cpp.arr` annotation helper and its implementation branches. Numeric arrays use `NDArray[T]` or inferred numeric annotations. Explicit C++ pointer annotations remain available for advanced buffer types; callers supply a separate length argument when needed.
- Migrated the two existing pointer-and-size tests to typed NumPy annotations and updated the argument reference. Historical work records were preserved.
- Validation: focused decorator tests passed (18 tests, 1.61 seconds); the default suite passed (197 passed, 135 skipped, 7.69 seconds); lint and the strict documentation build passed.
- Proposed `Annotated[NDArray[T], "const"]` for read-only numeric inputs. This would generate `const T*`, retain dtype and layout checks, and accept both writable and read-only storage. The proposal is not implemented.
- The untracked RosCon rehearsal still uses published 0.3.x packages. Its environment and executable examples need updating together before it uses the new annotation API. No RosCon files were included in this change.
- Changes are on the local `feat/array-annotations` branch. No push or release publication was made.
