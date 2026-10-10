# Failed adapter recipe and correction

The inline library subprocess probes passed. Two errors occurred in the first
attempt to use the existing `@cpp(nogil=True)` helper as an adapter. They were
recipe errors, not demonstrated library ABI failures.

The wrapper body began with `#include "native.h"`. This put an `extern "C"`
linkage block inside a C++ function during cached compilation. GCC reported:

```text
native.h:5:8: error: expected unqualified-id before string constant
cpp__aos_7e472fa9a1bf.cpp:11:1: error: 'detection_aos' was not declared in this scope
```

The helper retained its documented uncached fallback. The first call then
rejected the input address because Python `int` annotations map to C++ `int`:

```text
ValueError: could not convert argument 1 (integer 94476266012848 out of range for int)
```

The corrected wrapper uses an explicit `std::uintptr_t` annotation for native
addresses and calls a typed native function pointer. The pointer comes from a
retained library handle. Its source uses only the standard scalar headers
already emitted by the helper. No core package change was needed. All saved
solution checks pass with cached wrappers.

The missing-logic skeleton also failed its expected smoke check. Its separate
[diagnostic](skeleton_failure.txt) reports `logic_error: TODO parallel_for`.
The saved solution was rebuilt and retested after that check.
