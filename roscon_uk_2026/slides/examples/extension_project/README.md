# Build and call the sum-of-squares extension

From this repository checkout, with its Pixi environment:

```sh
pixi run cmake -S roscon_uk_2026/slides/examples/extension_project -B build/slide_extension -DCMAKE_BUILD_TYPE=Release
pixi run cmake --build build/slide_extension -j2
pixi run env PYTHONPATH="$PWD/build/slide_extension" python roscon_uk_2026/slides/examples/extension_project/demo.py
```

Expected output: `14.0`.

`kernel.cpp` implements the computation. `bindings.cpp` exposes it through pybind11. `api.py` optionally provides convenient input conversion. `demo.py` calls that API. The header and separate convenience wrapper are choices for this example, not mandatory files for every extension. Distribution can use a Python package and install the extension instead of setting PYTHONPATH.

The slide comparison uses `../with_cppyy_kit.py`. It explicitly supplies a float64 NumPy array; the extension's convenience wrapper also accepts lists and converts them.
