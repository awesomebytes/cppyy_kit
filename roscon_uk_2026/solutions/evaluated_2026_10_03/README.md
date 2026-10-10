# Original evaluated sources and environment

These six Python files are byte-for-byte copies of the evaluated solutions used
for the 3 October 2026 measurements. Their hashes match
[`evaluation/provenance.json`](../../evaluation/provenance.json) and the original
agent run reports. The original Pixi manifest, lock, and kernel guide are kept
here to record the published 0.3 package environment.

The manifest is an archival snapshot, with task paths relative to its original
`roscon_uk_2026/` location. It is not the current rehearsal entry point.

The runnable solutions one directory above use the current checkout's NumPy
annotations. Their imports and annotations were migrated on 10 October 2026.
The original C++ algorithm bodies are unchanged. Historical timings do not
measure these migrated sources. See [`CURRENT_REHEARSAL.md`](../../CURRENT_REHEARSAL.md).
