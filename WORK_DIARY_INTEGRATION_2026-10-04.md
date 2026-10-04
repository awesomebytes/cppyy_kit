# Integration work diary

## Initial review

- Preserved local main history, the existing diff and the new guide files in a
  private Git backup directory.
- Created `feat/installed-workflows` in the existing checkout.
- Reviewed the experiment overview, implementation plan, evaluation and safety
  review. Delegated all twelve experiment scopes and independent core/API reviews.
- Confirmed existing numeric annotations already validate alignment and handle
  empty buffers. Retained these APIs rather than adding overlapping adapters.
- Reproduced unaligned PCL normalization and the OpenCV image-size/depth defects.
- Confirmed the precise empty Pydantic-column fixture works. No fix was made.
- Selected installed guides, lightweight diagnostics, artifact coordination,
  fetched-path validation, buffer fixes and two focused tutorials.
- Kept the experimental/presentation directory unchanged and outside commits.

## Implementation and validation

- Implemented package resources for three task topics and ten kit references.
- Deferred native startup for guide/environment commands and preserved callable
  exports, mutable teardown state and native initialization order.
- Added environment discovery and compiler-launcher support.
- Added staged, validated header fetches and coordinated compile publication,
  including read-only warm reuse and retry after filesystem failures.
- Fixed PCL alignment and OpenCV bounds, depth and submatrix view extents.
- Added native-component and OMPL callback examples with behavioral regressions.
- Updated all ten kit API references and corrected compile-cache invalidation
  claims and DBoW2 packaging descriptions.
- Independent review found and verified the additional compatibility corrections
  listed in the integration report.
- Default tests passed: 368 passed, 161 optional skips. Feature suites passed:
  PCL 2, vision 40 and OMPL 21. Lint and strict docs passed.
- Built eleven local Conda artifacts. Verified all 23 canonical guide resources
  outside the checkout and proved real installed core/rclcpp native behavior.
- Added focused native-domain CI jobs. Existing dual-architecture core/rclcpp
  installed-package and sanitizer checks remain enabled.
- Kept package artifacts and validation logs local. Upstream checks and merge
  are tracked in the pull request; no channel upload is part of this work.

## Pull-request corrections

- The initial pull-request run passed lint, default tests and all three native
  domain jobs. The x86_64 ROS suite failed, so the obsolete run was cancelled.
- Corrected six ROS artifact lookups to include library paths, matching
  compilation. The trampoline lookup also includes its toolchain paths.
- Reproduced three later failures caused by a mocked subscription unit compiling
  fake types into Cling. Corrected two unit mocks without changing production
  initialization or weakening their assertions.
- The locked CI environment passed 45 cache/native tests, 34 tests with the
  subscription cache disabled and 5 normal-cache checks. Independent review
  passed three additional focused checks and found no remaining issues.
- Rebuilt core/rclcpp artifacts in 60 seconds. A fresh external installation
  passed native checks in 8 seconds. All eleven artifacts passed 23 exact guide
  resource comparisons and CLI checks in 2.5 seconds. Preserved initial artifacts.
- Final default task passed 369 tests with 161 optional skips in 12.95 seconds;
  lint passed in 1.55 seconds and strict docs built in 0.87 seconds. An initial
  jitter timing threshold failed, then its isolated and full-suite reruns passed.
  The benchmark and its assertions were unchanged.
