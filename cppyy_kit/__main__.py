"""cppyy_kit command-line entry: ``python -m cppyy_kit <group> ...``.

Groups:

    python -m cppyy_kit trace report trace.json     # boundary-trace report
    python -m cppyy_kit stubgen bt_kit -o out.pyi   # .pyi for a kit's surface

(``python -m cppyy_kit.trace report ...`` also works but Python's runpy prints a
harmless double-import warning for it; prefer this form.)
"""
import sys

from . import trace
from . import stubgen


def main(argv):
    if argv and argv[0] == "trace":
        return trace._main(argv[1:])
    if argv and argv[0] == "stubgen":
        return stubgen._main(argv[1:])
    sys.stderr.write("usage: python -m cppyy_kit {trace report <f.json> | stubgen <module>}\n")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
