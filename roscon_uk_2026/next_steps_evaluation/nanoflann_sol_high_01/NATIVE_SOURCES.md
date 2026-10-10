# Preserve and restore the generated native implementation

The candidate expects its generated declarations and implementation under
`build/agent_native`. Build directories are ignored by Git. The exact generated
inputs are saved in [native_sources](native_sources), with checksums in
[native_sources.json](native_sources.json). The candidate Python file and its
recorded checksum are unchanged after evaluation.

From this evaluation directory, restore the inputs before replay:

```bash
mkdir -p build/agent_native
cp native_sources/retained.hpp native_sources/retained.cpp build/agent_native/
```

The pinned nanoflann header is fetched or reused through the candidate's require
call. Follow the recorded acceptance command and use the original locked
nanoflann Pixi environment. Regenerated binary paths and compilation costs can
differ. These files are an agent-produced solution, not a new kit.
