# Score candidate positions with Eigen and OpenMP

From this repository checkout and its activated default Pixi environment:

```sh
pixi run env OMP_DYNAMIC=FALSE OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1 python examples/particle_scores/demo.py
```

The example scores XY positions against distances to known landmarks. It checks
every score against NumPy and prints a best candidate near `[1.08, -2.03]`, close
to the simulated position `[1, -2]`. It requires Eigen headers and GCC with OpenMP.

`headers` places library includes outside the generated function. `preamble`
checks that OpenMP is enabled. `link_args=['-fopenmp']` builds and loads the shared
library on the first call; it requires enabled caching. `nogil=True` releases the
GIL during native computation, while the OpenMP pragma distributes independent
candidates across C++ workers. Thread progress and parallel computation are
separate effects.

Arrays contain contiguous float64 XY pairs. There is one range per landmark and
one output score per candidate. Keep inputs unchanged and use nonoverlapping
output storage during each call. The C++ body checks dimensions and thread count.
The first call includes compilation and runtime startup; later calls reuse code.
These decorator options are source additions after the published 0.4.1 package.
