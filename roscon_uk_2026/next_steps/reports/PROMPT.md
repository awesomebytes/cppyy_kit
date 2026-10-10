# Exact agent prompt

Complete `roscon_uk_2026/next_steps/reports/skeleton.py` in this checkout. Read
`roscon_uk_2026/next_steps/reports/GUIDE.md` first. Implement `rank_windows` and
`match_images` from their contracts. Find the five largest estimator-error
windows in the typed recording. Use fixed-duration, half-open windows. Rank by
mean error, select disjoint windows, and declare ties. Match representative
images in integer nanoseconds using an explicit clock, direction, and tolerance.
Keep unmatched queries visible. Handle duplicate image timestamps and epoch
values without float precision loss. Do not change acceptance tests or copy the
saved implementation in `report_core.py`. Run the independent checks against the
skeleton with `REPORT_CORE_MODULE`. Save the command, exit status, elapsed time,
and unchanged test SHA-256. Use the existing report CLI and fixture generation
commands in the guide to produce portable HTML and machine-readable provenance.
Keep diagnostic residuals distinct from reference error. Work only in the
reports directory. Do not install skills, change a shared environment, create a
checkout, or use a worktree.
