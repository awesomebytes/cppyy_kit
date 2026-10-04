"""Serialize cache publication between Linux processes and Python threads."""
from contextlib import contextmanager
import fcntl
import os
import threading


_LOCKS = {}
_GUARD = threading.Lock()


@contextmanager
def file_lock(path):
    """Hold an advisory lock on a persistent file for this operation."""
    path = os.path.realpath(os.fspath(path))
    with _GUARD:
        lock = _LOCKS.setdefault(path, threading.Lock())
    with lock:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "a+b") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle, fcntl.LOCK_UN)
