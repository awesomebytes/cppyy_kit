# Implement the OMPL policy contract

Use `StateValidityChecker` when a Python experiment needs to define which
native planner states are allowed. This is an existing OMPL policy slot.
It is not a mock hardware sensor interface. OMPL leaves state validity to the
application. See its [state validation documentation](https://ompl.kavrakilab.org/stateValidation.html).

## Native dispatch and required methods

The installed OMPL 1.7.0 header has one pure virtual method:

```cpp
virtual bool isValid(const ompl::base::State* state) const = 0;
```

The class also has implemented `isValid` overloads with distance and direction
outputs, two implemented `clearance` overloads, an implemented equality virtual,
and a default virtual destructor. Those methods do not require Python overrides
for this experiment. The signature and complete class are in
[OMPL 1.7.0 StateValidityChecker.h](https://github.com/ompl/ompl/blob/1.7.0/src/ompl/base/StateValidityChecker.h).
The exact header inspected here is under
`.pixi/envs/ompl/include/ompl-1.7/ompl/base/StateValidityChecker.h` in this checkout.

Derive directly from `ob.StateValidityChecker`. Construct the base with
`super().__init__(si)`. Define the Python method `isValid(self, state)` when
the class is created. cppyy creates the virtual dispatcher. See
[cppyy cross-inheritance](https://cppyy.readthedocs.io/en/latest/classes.html#cross-inheritance).

`Engine.solve()` calls `SimpleSetup::solve()`. OMPL's compiled RRTConnect and
motion validator call `SpaceInformation::isValid()`, which dispatches to the
Python override. `solve_calls` counts those native-initiated calls. The batch
measurement is a separate native loop through `SpaceInformation::isValid()`.
No Python loop directly calls the override in either measurement.

## State and policy semantics

`make_policy(ob)` returns the derived class. Its constructor accepts
`(si, bias_x=0.0, fail_at=None)`. Coordinates and bias are dimensionless.
`bias_x` must be finite. The constructor rejects NaN and infinity.

For each `isValid` call:

1. Increment `self.calls` exactly once.
2. If this count equals `self.fail_at`, raise
   `ValueError("scripted validity policy failure")`.
3. Read `x` and `y` from the provided native real-vector state.
4. Return false for any coordinate outside the closed unit square, including
   NaN and infinity.
5. Return true only when `(x + bias_x - 0.5)^2 + (y - 0.5)^2 > 0.25^2`.

The circle boundary is invalid. Positive bias moves the keep-out center left.
For bias `0.05`, the seven demo points produce `[1, 0, 0, 1, 1, 0, 0]`.
The state pointer is borrowed for one call. Do not retain it or free it.
RTTI downcasting lets this real-vector state support `state[0]` and `state[1]`.
These accesses do not generalize to every OMPL state space.

The native engine uses motion-checking resolution `0.001`, a fraction of the
state space's maximum extent. It samples intermediate states. The independent
checker additionally computes every solution segment's minimum distance to the
circle center. Smaller discrete resolution is not a proof of continuous safety
for arbitrary geometry. See [OMPL motion validation](https://ompl.kavrakilab.org/stateValidation.html).

## Ownership, exceptions, and shutdown

Use `Session` as a context manager. Its `policy` attribute is a strong Python
reference. C++ stores a `shared_ptr` with a no-op deleter built by `Engine.attach()`.
This pointer borrows the policy. It does not own or delete the Python object.
Do not drop the strong reference while an engine is open.

`Session.close()` calls `Engine.close()` first. Native close clears planner
state, replaces the checker with OMPL's all-valid checker, and destroys the
setup. Only then does Python remove its engine and policy references. Close is
idempotent. A retained engine rejects calls after close, before dispatching any
callback. A released policy's base still contains a borrowed `SpaceInformation`
pointer, so never call it after close. Recreate a session to restart.

The lifecycle test forces garbage collection while a session is open, invokes
the native driver successfully, closes it, and checks weak-reference release.
It also verifies that close does not change callback counts. It repeats this
sequence 25 times.

A Python `ValueError` raised inside either the native batch call or
`RRTConnect::solve()` reaches the Python caller with its type and message in the
tested versions. The context manager closes the engine on both success and
exception. Do not continue a solve after a callback exception. cppyy documents
cross-language exceptions in its [exception reference](https://cppyy.readthedocs.io/en/latest/exceptions.html).

All operations here are synchronous and single-threaded. A callback executes
Python under the GIL. The example does not release the GIL on the enclosing
native call, create native workers, or claim concurrent checker safety. Do not
mutate policy settings or close the engine during a call. OMPL requires
thread-safe validators when used by concurrent planners.

## Measure before retaining Python in a hot loop

The benchmark checks seven pre-converted points 30,000 times per sample, with
five samples and an untimed first call. It times the native loop, counts actual
dispatches, and records the Python wall clock separately. Both implementations
include the same geometry and a callback counter. The native implementation is
a JIT-compiled concrete subclass of the same OMPL interface.

The planner runs each implementation in a fresh process with seed 41. OMPL's
global seed must precede sampling. The check compares actual coordinates rather
than assuming equal seeds imply equal behavior. Keep Python for experimental
policies when its cost is acceptable. Move this operation into the equivalent
native checker when frequent calls dominate, then rerun the equivalence checks.

Bringup, adapter header parsing, first-use construction, first batch dispatch,
first solve wrapper, input conversion, native execution, wall time, process
time, and peak RSS are separate fields in [evidence.json](evidence.json).
cppyy compiles wrappers and methods lazily. Header parsing is not a complete
compilation measurement. First-use fields include that deferred JIT work.
