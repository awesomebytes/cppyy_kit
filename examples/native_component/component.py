"""Validate settings and own an existing C++ class through cppyy.

Run from the checkout: pixi run python examples/native_component/component.py
"""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import threading

ROOT = Path(__file__).resolve().parent
_NATIVE = None
_LOAD_LOCK = threading.Lock()


@dataclass(frozen=True)
class Settings:
    """Dimensionless smoothing fraction. Larger values follow input sooner."""
    alpha: float = 0.5

    def __post_init__(self):
        if type(self.alpha) not in (int, float):
            raise TypeError("alpha must be a number")
        if not math.isfinite(self.alpha) or not 0 < self.alpha <= 1:
            raise ValueError("alpha must be finite and in (0, 1]")


def native_namespace():
    """Prebuild definitions, then give cppyy only public declarations."""
    global _NATIVE
    with _LOAD_LOCK:
        if _NATIVE is None:
            import cppyy
            from cppyy_kit import cppdef_cached, prebuild
            declarations = (ROOT / "smoother.hpp").read_text(encoding="utf-8")
            implementation = (ROOT / "smoother.cpp").read_text(encoding="utf-8")
            # Include header contents in the cache input so header edits rebuild it.
            definitions = declarations + implementation.replace('#include "smoother.hpp"', "")
            options = {"decls": declarations, "name": "native_component_example"}
            prebuild(definitions, **options)
            cppdef_cached(definitions, **options)
            _NATIVE = cppyy.gbl.component_example
    return _NATIVE


class Smoother:
    """Own one native instance. Use each instance from one thread."""
    def __init__(self, settings=None):
        if settings is None:
            settings = Settings()
        if not isinstance(settings, Settings):
            raise TypeError("settings must be Settings")
        # Recheck before loading native code, including manually altered instances.
        self.settings = Settings(**asdict(settings))
        native = native_namespace()
        config = native.Config()
        config.smoothing_weight = self.settings.alpha
        self._native = native.Smoother(config)

    def _require_open(self):
        if self._native is None:
            raise RuntimeError("smoother is closed")
        return self._native

    def process(self, values):
        native = self._require_open()
        import cppyy
        # Conversion finishes before the native component can mutate its state.
        inputs = cppyy.gbl.std.vector["double"](values)
        return list(native.process(inputs))

    def reset(self):
        self._require_open().reset()

    def snapshot(self):
        state = self._require_open().snapshot()
        return {"initialized": bool(state.initialized), "value": float(state.value),
                "samples": int(state.samples)}

    def close(self):
        self._native = None

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()


def main():
    settings = Settings(alpha=0.5)
    with Smoother(settings) as smoother:
        print(smoother.process([0.0, 2.0, 4.0]))
        print(smoother.snapshot())
        smoother.reset()
        print(smoother.process([4.0]))
    print(json.dumps(asdict(settings), sort_keys=True))


if __name__ == "__main__":
    main()
