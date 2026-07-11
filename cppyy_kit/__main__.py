"""cppyy_kit command-line entry: ``python -m cppyy_kit <group> ...``.

Currently one group, the boundary tracer's report formatter::

    python -m cppyy_kit trace report trace.json

(``python -m cppyy_kit.trace report trace.json`` also works but Python's runpy
prints a harmless double-import warning for it; prefer this form.)
"""
import sys

from . import trace


def main(argv):
    if argv and argv[0] == "trace":
        return trace._main(argv[1:])
    sys.stderr.write("usage: python -m cppyy_kit trace report <trace.json>\n")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
