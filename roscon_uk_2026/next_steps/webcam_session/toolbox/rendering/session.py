"""Separate capture and inference, retaining only the latest frame and result."""
import atexit
import os
import threading
import time
import traceback

import numpy as np

from .camera import open_camera


class Workbench:
    """Notebook-owned camera and replaceable processing, with bounded pending work.

    A supplied source is a callable returning a BGR image or None at end of input.
    It must pace itself. Otherwise camera specifies an OpenCV capture device.
    """

    def __init__(self, camera=0, source=None):
        self.state = {}
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._controls = {}
        self._processor = lambda frame, state, controls: frame
        self._revision = 0
        self._raw = None
        self._result = None
        self._captured = 0
        self._processed = 0
        self._error = None
        self._capture_error = None
        self._processing_ms = 0.0
        self._started = time.monotonic()
        self._capture = None
        if source is None:
            self._capture = open_camera(camera)
        self._source = source
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._processing_thread = threading.Thread(target=self._process_loop, daemon=True)
        self._capture_thread.start()
        self._processing_thread.start()
        atexit.register(self.close)

    def configure(self, **values):
        with self._condition:
            self._controls.update(values)

    def controls(self):
        with self._condition:
            return dict(self._controls)

    def set_processor(self, function):
        if not callable(function):
            raise TypeError('Expected process(frame, state, controls)')
        with self._condition:
            self._processor = function
            self._revision += 1
        return self._revision

    def latest(self, processed=False):
        """Return a published packet. Treat its arrays as read-only."""
        with self._condition:
            return self._result if processed else self._raw

    def snapshot(self):
        packet = self.latest()
        if packet is None:
            raise RuntimeError('Waiting for first frame')
        return packet['image'].copy()

    def status(self):
        with self._condition:
            elapsed = time.monotonic() - self._started
            result = self._result
            return dict(pid=os.getpid(), session_id=id(self), state_id=id(self.state),
                        captured=self._captured, processed=self._processed,
                        capture_fps=self._captured / max(elapsed, 0.001),
                        inference_fps=self._processed / max(elapsed, 0.001),
                        processing_ms=self._processing_ms, revision=self._revision,
                        result_revision=result['revision'] if result else None,
                        result_age_ms=(time.monotonic()-result['stamp'])*1000 if result else None,
                        error=self._error, capture_error=self._capture_error,
                        capture_running=self._capture_thread.is_alive(),
                        inference_running=self._processing_thread.is_alive())

    def _capture_loop(self):
        try:
            while not self._stop.is_set():
                if self._source is None:
                    ok, frame = self._capture.read()
                    if not ok:
                        raise RuntimeError('Camera stopped returning frames')
                else:
                    frame = self._source()
                    if frame is None:
                        break
                with self._condition:
                    self._captured += 1
                    self._raw = dict(image=frame, sequence=self._captured,
                                     stamp=time.monotonic())
                    self._condition.notify_all()
        except Exception:
            with self._condition:
                self._capture_error = traceback.format_exc()
        finally:
            if self._capture is not None:
                self._capture.release()

    def _process_loop(self):
        previous = 0
        while not self._stop.is_set():
            with self._condition:
                ready = self._condition.wait_for(
                    lambda: self._stop.is_set() or
                    (self._raw is not None and self._raw['sequence'] != previous), timeout=0.2)
                if self._stop.is_set():
                    return
                if not ready:
                    continue
                packet = self._raw
                previous = packet['sequence']
                function, revision = self._processor, self._revision
                controls = dict(self._controls)
            started = time.perf_counter()
            error = None
            try:
                # Processing may modify this copy; the raw preview stays independent.
                output = function(packet['image'].copy(), self.state, controls)
                result = dict(output) if isinstance(output, dict) else dict(image=output)
                image = result['image']
                if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
                        or image.ndim != 3 or image.shape[2] != 3 or not image.size):
                    raise ValueError('Result image must be nonempty HxWx3 uint8 BGR')
            except Exception:
                error = traceback.format_exc()
                result = dict(image=packet['image'])
            result.update(sequence=previous, stamp=packet['stamp'], revision=revision)
            with self._condition:
                self._result = result
                self._processed += 1
                self._processing_ms = (time.perf_counter() - started) * 1000
                self._error = error

    def close(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        self._capture_thread.join(timeout=3)
        self._processing_thread.join(timeout=3)
