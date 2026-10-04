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
