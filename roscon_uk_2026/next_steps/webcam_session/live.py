"""Persistent camera worker and browser viewer for a notebook-owned session."""
import atexit
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading
import time
import traceback

import cv2
import numpy as np

VIEWER = b'''<!doctype html><title>Interactive webcam</title>
<style>body{background:#161616;color:white;font:18px monospace}img{max-width:95vw}</style>
<img id="frame"><pre id="status"></pre><script>
async function update(){try{
 const response=await fetch('/frame.jpg');
 if(response.ok){const url=URL.createObjectURL(await response.blob());
 const old=frame.src;frame.src=url;if(old.startsWith('blob:'))URL.revokeObjectURL(old);}
 status.textContent=JSON.stringify(await (await fetch('/status')).json(),null,2);
}catch(e){document.getElementById('status').textContent=String(e)}
setTimeout(update,100)}
// Use explicit DOM references; window.status is a browser property.
const frame=document.getElementById('frame');
const status=document.getElementById('status');update();</script>'''


class LiveCamera:
    """Own one camera; swap Python processing functions between frames."""

    def __init__(self, camera=0, port=8766):
        self.state = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._processor = lambda frame, state: frame
        self._revision = 0
        self._label = 'raw camera'
        self._jpeg = None
        self._latest = None
        self._status = {}
        self._capture = cv2.VideoCapture(camera)
        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError(f'Cannot open camera {camera}')
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/':
                    body, content_type = VIEWER, 'text/html'
                elif self.path == '/status':
                    body = json.dumps(owner.status()).encode()
                    content_type = 'application/json'
                elif self.path == '/frame.jpg':
                    with owner._lock:
                        body = owner._jpeg
                    content_type = 'image/jpeg'
                    if body is None:
                        self.send_error(503, 'Waiting for first frame')
                        return
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        try:
            self._server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        except Exception:
            self._capture.release()
            raise
        self.url = f'http://127.0.0.1:{self._server.server_port}'
        self._viewer = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._viewer.start()
        self._worker.start()
        atexit.register(self.close)

    def set_processor(self, function, label='custom'):
        """Install a callable; the next frame uses it, retaining shared state."""
        if not callable(function):
            raise TypeError('Expected process(frame, state) callable')
        with self._lock:
            self._processor = function
            self._revision += 1
            self._label = label
        return self.status()

    def status(self):
        with self._lock:
            return dict(self._status, pid=os.getpid(), session_id=id(self),
                        state_id=id(self.state), installed_revision=self._revision,
                        running=self._worker.is_alive())

    def snapshot(self):
        """Copy the latest raw BGR frame for agent inspection or warmup."""
        with self._lock:
            if self._latest is None:
                raise RuntimeError('Waiting for camera frame')
            return self._latest.copy()

    def _run(self):
        frames = 0
        started = time.monotonic()
        try:
            while not self._stop.is_set():
                ok, frame = self._capture.read()
                if not ok:
                    raise RuntimeError('Camera stopped returning frames')
                with self._lock:
                    function, revision, label = self._processor, self._revision, self._label
                    self._latest = frame.copy()
                tick = time.perf_counter()
                error = None
                try:
                    output = function(frame, self.state)
                    if (not isinstance(output, np.ndarray) or output.dtype != np.uint8
                            or output.ndim != 3 or output.shape[2] != 3 or not output.size):
                        raise ValueError('Return a nonempty HxWx3 uint8 BGR image')
                except Exception:
                    error = traceback.format_exc()
                    output = frame
                processing_ms = (time.perf_counter() - tick) * 1000
                ok, encoded = cv2.imencode('.jpg', output)
                if not ok:
                    raise RuntimeError('JPEG encoding failed')
                frames += 1
                with self._lock:
                    self._jpeg = encoded.tobytes()
                    self._status = dict(frames=frames, revision=revision, label=label,
                                        processing_ms=processing_ms, error=error,
                                        uptime_s=time.monotonic()-started,
                                        updated_unix=time.time())
        except Exception:
            with self._lock:
                self._status['error'] = traceback.format_exc()
        finally:
            self._capture.release()

    def close(self):
        if self._stop.is_set():
            return
        self._stop.set()
        self._server.shutdown()
        self._server.server_close()
        self._worker.join(timeout=3)
        self._viewer.join(timeout=3)
