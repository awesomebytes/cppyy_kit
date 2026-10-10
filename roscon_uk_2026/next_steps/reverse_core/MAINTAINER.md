# Native demonstration implementation

[native/pose_filter.hpp](native/pose_filter.hpp) contains public declarations. [native/pose_filter.cpp](native/pose_filter.cpp) contains the algorithm and the fixed-format loader. [native/driver.cpp](native/driver.cpp) reads CSV, invokes the estimator, and writes CSV. This implementation has no dependency on cppyy, Python, Pydantic, or ROS.

`build_native()` invokes the activated `CXX` compiler with C++17, `-O3`, `-Wall`, `-Wextra`, and `-Werror`. It first builds `libreverse_pose.so`, then links `pose_filter_driver` with an `$ORIGIN` runtime path. It hashes all three source files, compiler version, compiler command, and explicit flags. Binaries reside in `build/<hash>/`. A file lock prevents consumers in separate processes from compiling the same output concurrently. Temporary outputs are renamed after successful compilation. `build_native(force=True)` measures a complete rebuild.

`native_namespace()` lazily imports the local `cppyy_kit`, loads the shared object with `cppyy_kit.load_libraries`, and exposes only the compatible public header through cppyy. The implementation is compiled ahead of loading. The Python wrapper explicitly populates `Config.time_constant_s`, `Config.reset_gap_s`, and `Config.output_frame`. It does not generate a replacement C++ struct.

The library independently checks configuration and complete batches. The wrapper validates array types, shapes, finite values, ordering, alignment, and contiguous layout before borrowing pointers. It rejects cross-call timestamp regression before entering `process`. Native validation repeats finite and timestamp checks for direct C++ users. It validates the whole batch before writing output or state. Direct native callers must provide correctly sized, non-overlapping buffers and keep them alive during the call.

The timestamp difference is computed in extended precision before conversion to double seconds. It avoids signed int64 subtraction overflow. Output arithmetic computes the update with extended intermediates so opposite finite positions near the floating limits do not overflow subtraction. State contains one position, one timestamp, two counters, a flag, and copied configuration. The frame string is limited to 128 characters. Caller-owned batch and result buffers scale with sample count; estimator storage does not.

The Python cppyy proxy owns the estimator allocation. `close()` drops that reference. The library copies the configuration at construction and never retains input pointers. One estimator must not receive concurrent calls. Use separate instances for independent trials.

The locked Linux x86_64 environment uses GCC/G++ 14.3.0 with libgcc/libstdcxx 15.2.0. An initial unpinned environment crashed in Cling startup with GCC/G++ 15.3 and libstdcxx 16.2. The existing repository [package recipe](../../../recipe/cppyy-kit/recipe.yaml) records the same compatibility pins. This scope does not claim validation on another platform.
