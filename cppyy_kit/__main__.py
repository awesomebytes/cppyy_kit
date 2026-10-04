"""cppyy_kit command-line entry: ``python -m cppyy_kit <group> ...``.

Groups:

    python -m cppyy_kit trace report trace.json     # boundary-trace report
    python -m cppyy_kit stubgen bt_kit -o out.pyi   # .pyi for a kit's surface
    python -m cppyy_kit status                       # capability report
    python -m cppyy_kit guide accelerate             # packaged agent guidance

(``python -m cppyy_kit.trace report ...`` also works but Python's runpy prints a
harmless double-import warning for it; prefer this form.)
"""
import sys

def main(argv):
    if argv and argv[0] == "guide":
        from . import guides
        return guides._main(argv[1:])
    if argv and argv[0] == "trace":
        from . import trace
        return trace._main(argv[1:])
    if argv and argv[0] == "stubgen":
        from . import stubgen
        return stubgen._main(argv[1:])
    if argv and argv[0] == "status":
        if "--environment" in argv[1:]:
            from . import diagnostics
            return diagnostics._main(argv[1:])
        from . import capability
        return capability._main(argv[1:])
    sys.stderr.write(
        "usage: python -m cppyy_kit {trace report <f.json> | stubgen <module> | "
        "status [--environment] | guide [topic [overview|api]]}\n")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
