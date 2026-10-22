"""Local browser controls for the admittance demo's virtual force.

``start_server(frame="base")`` returns a running ``ForceUI``. Call
``get_force()`` in the demo loop and ``close()`` during teardown. Commands expire
unless the browser refreshes them while a direction button is held.
"""

import html
import json
import math
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


_PAGE = r"""<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Admittance force controls</title>
<style>
body {
  font: 17px system-ui; background: #102126; color: #e7f3ef;
  max-width: 660px; margin: 40px auto; padding: 20px;
}
h1 { font-size: 30px; }
p { line-height: 1.5; color: #b9d1cb; }
.controls { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin: 24px 0; }
button {
  font: 600 24px system-ui; padding: 22px; border: 1px solid #4a8375;
  border-radius: 12px; background: #234d45; color: white;
  touch-action: none; user-select: none;
}
button.held { background: #97ebbf; color: #102126; }
button:focus-visible { outline: 3px solid #97ebbf; }
label { display: flex; align-items: center; gap: 12px; }
input { font: inherit; width: 80px; padding: 8px; border-radius: 6px; }
output {
  display: block; font: 16px monospace; background: #193239;
  padding: 16px; border-radius: 8px; white-space: pre-wrap;
}
#stop { width: 100%; background: #793d37; font-size: 18px; }
small { color: #b9d1cb; }
</style>
<h1>Push the virtual hand</h1>
<p>Hold a direction to apply a virtual force. Release to let the spring return the hand.
This command is synthetic, not a force sensor measurement.</p>
<p>Force frame: <strong>__FRAME__</strong>. Torque is zero.</p>
<label>
  Force <input id="magnitude" type="number" min="0" max="__MAX__" step="0.1" value="__DEFAULT__"> N
  <small>(maximum __MAX__ N)</small>
</label>
<div class="controls">
  <button data-axis="0" data-sign="-1">−X</button><button data-axis="0" data-sign="1">+X</button>
  <button data-axis="1" data-sign="-1">−Y</button><button data-axis="1" data-sign="1">+Y</button>
  <button data-axis="2" data-sign="-1">−Z</button><button data-axis="2" data-sign="1">+Z</button>
</div>
<button id="stop">Release force</button>
<p><small>
  Hold with a mouse, touch, or keyboard Space/Enter.
  Commands expire after __TIMEOUT__ s without a refresh.
</small></p>
<output id="status">Connecting…</output>
<script>
const token = '__TOKEN__', maxForce = __MAX__;
const status = document.querySelector('#status');
const magnitude = document.querySelector('#magnitude');
let active = null, sequence = Date.now() * 1000;

async function send(path, body) {
  const response = await fetch(path, {
    method: 'POST',
    headers: {'Content-Type': 'application/json', 'X-Force-Token': token},
    body: JSON.stringify(body),
    keepalive: true
  });
  if (!response.ok) throw Error('Force command rejected');
}

function release() {
  active = null;
  sequence++;
  document.querySelectorAll('.held').forEach(button => button.classList.remove('held'));
  send('/release', {sequence}).catch(() => {
    status.textContent = 'Disconnected. Force expires automatically.';
  });
}

async function refresh() {
  if (!active) return;
  const button = active, current = sequence;
  const value = Math.min(maxForce, Math.max(0, Number(magnitude.value) || 0));
  magnitude.value = value;
  try {
    await send('/force', {
      axis: Number(button.dataset.axis),
      direction: Number(button.dataset.sign),
      magnitude: value,
      sequence: current
    });
  } catch (error) {
    if (current === sequence) release();
    status.textContent = 'Disconnected. Force expires automatically.';
  }
}

function hold(button) {
  if (active === button) return;
  release();
  sequence++;
  active = button;
  button.classList.add('held');
  refresh();
}

for (const button of document.querySelectorAll('[data-axis]')) {
  button.addEventListener('pointerdown', event => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    hold(button);
  });
  button.addEventListener('pointerup', release);
  button.addEventListener('pointercancel', release);
  button.addEventListener('lostpointercapture', () => {
    if (active === button) release();
  });
  button.addEventListener('keydown', event => {
    if (event.key === ' ' || event.key === 'Enter') {
      event.preventDefault();
      hold(button);
    }
  });
  button.addEventListener('keyup', event => {
    if (event.key === ' ' || event.key === 'Enter') release();
  });
  button.addEventListener('blur', () => {
    if (active === button) release();
  });
}
document.querySelector('#stop').addEventListener('click', release);
window.addEventListener('blur', release);
window.addEventListener('pagehide', release);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) release();
});
setInterval(refresh, 80);
setInterval(async () => {
  try {
    const response = await fetch('/state');
    if (!response.ok) throw Error();
    const state = await response.json();
    const lines = ['Force [N]: ' + state.force.map(value => value.toFixed(2)).join(', ')];
    if (state.position) {
      lines.push('Position [m]: ' + state.position.map(value => value.toFixed(4)).join(', '));
    }
    if (state.velocity) {
      lines.push('Velocity [m/s]: ' + state.velocity.map(value => value.toFixed(4)).join(', '));
    }
    status.textContent = lines.join('\n');
  } catch (error) {
    status.textContent = 'Disconnected. Force expires automatically.';
  }
}, 200);
</script>
</html>"""


class ForceUI:
    """Thread-safe, expiring force command and its localhost HTTP server."""

    def __init__(self, *, port=8765, frame="base", max_force=5.0, timeout=0.3):
        if not math.isfinite(max_force) or not 0 < max_force <= 5.0:
            raise ValueError("max_force must be in (0, 5] N")
        if not math.isfinite(timeout) or not 0 < timeout <= 1.0:
            raise ValueError("timeout must be in (0, 1] seconds")
        self._lock = threading.Lock()
        self._force = (0.0, 0.0, 0.0)
        self._deadline = 0.0
        self._state = {}
        self._sequence = -1
        self._closed = False
        self._max_force = max_force
        self._timeout = timeout
        self._token = secrets.token_urlsafe(24)
        page = (_PAGE.replace("__FRAME__", html.escape(str(frame)))
                .replace("__MAX__", str(max_force))
                .replace("__DEFAULT__", str(min(2.0, max_force)))
                .replace("__TIMEOUT__", str(timeout))
                .replace("__TOKEN__", self._token)).encode()
        ui = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def reply(self, status, payload, content_type="application/json"):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                self.wfile.write(payload)

            def do_GET(self):
                if self.path == "/":
                    self.reply(200, page, "text/html; charset=utf-8")
                elif self.path == "/state":
                    with ui._lock:
                        state = dict(ui._state)
                        state["force"] = ui._snapshot()
                    self.reply(200, json.dumps(state).encode())
                else:
                    self.reply(404, b'{}')

            def do_POST(self):
                if not secrets.compare_digest(self.headers.get("X-Force-Token", ""), ui._token):
                    self.reply(403, b'{}')
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 1024:
                        raise ValueError("invalid body length")
                    payload = json.loads(self.rfile.read(length))
                    sequence = payload.get("sequence")
                    if sequence is not None and (type(sequence) is not int or sequence < 0):
                        raise ValueError("invalid command sequence")
                    if self.path == "/release":
                        force = (0.0, 0.0, 0.0)
                    elif self.path == "/force":
                        axis = payload["axis"]
                        direction = payload["direction"]
                        magnitude = float(payload["magnitude"])
                        if (type(axis) is not int or axis not in (0, 1, 2)
                                or type(direction) is not int or direction not in (-1, 0, 1)
                                or not math.isfinite(magnitude)
                                or not 0 <= magnitude <= ui._max_force):
                            raise ValueError("invalid force")
                        values = [0.0, 0.0, 0.0]
                        values[axis] = direction * magnitude
                        force = tuple(values)
                    else:
                        self.reply(404, b'{}')
                        return
                    with ui._lock:
                        # Ignore a hold request that arrives after its release.
                        if not ui._closed and (sequence is None or sequence >= ui._sequence):
                            if sequence is not None:
                                ui._sequence = sequence
                            ui._force = force
                            ui._deadline = time.monotonic() + ui._timeout
                    self.reply(200, b'{}')
                except (ValueError, TypeError, KeyError, AttributeError):
                    self.reply(400, b'{}')

        self._server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self._server.daemon_threads = True
        self.url = f"http://127.0.0.1:{self._server.server_port}/"
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def _snapshot(self):
        if not self._closed and time.monotonic() < self._deadline:
            return self._force
        return (0.0, 0.0, 0.0)

    def get_force(self):
        """Return the current XYZ virtual force in N, or zero after expiry."""
        with self._lock:
            return self._snapshot()

    def set_state(self, *, position=None, velocity=None):
        """Set optional XYZ display values in m and m/s."""
        state = {}
        for name, values in (("position", position), ("velocity", velocity)):
            if values is not None:
                values = tuple(float(v) for v in values)
                if len(values) != 3 or not all(math.isfinite(v) for v in values):
                    raise ValueError(f"{name} must contain three finite values")
                state[name] = values
        with self._lock:
            self._state = state

    def close(self):
        """Clear the command and stop the server."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._deadline = 0.0
        self._server.shutdown()
        self._server.server_close()
        self._thread.join()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def start_server(**kwargs):
    """Start localhost controls; accept ForceUI's keyword arguments."""
    return ForceUI(**kwargs)
