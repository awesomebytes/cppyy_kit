"""Browser image/3D viewer attached to a retained perception workbench."""
from collections import deque
import json
import threading
import time

import numpy as np
import viser


class ViserViewer:
    """Show workbench frames and clouds; forward GUI edits to its controls."""

    def __init__(self, workbench, port=8768, update_hz=30):
        if not 0 < update_hz <= 60:
            raise ValueError("update_hz must be in (0, 60]")
        self._interval = 1.0 / update_hz
        self.workbench = workbench
        self.server = viser.ViserServer(host='127.0.0.1', port=port)
        self._stop = threading.Event()
        self._cloud = None
        self._metrics_lock = threading.Lock()
        self._update_times = deque(maxlen=600)
        self._counts = {'image_updates': 0, 'depth_updates': 0, 'cloud_updates': 0}
        self._started = time.monotonic()
        self._last_status_time = 0.0
        self._control_handles = {}
        self.error = None
        self.server.scene.set_up_direction('-y')
        self._origin = self.server.scene.add_frame(
            '/origin', axes_length=0.2, axes_radius=0.005, visible=False)
        self.image = self.server.gui.add_image(
            np.zeros((240, 320, 3), dtype=np.uint8), label='Camera', format='jpeg')
        self.depth_image = self.server.gui.add_image(
            np.zeros((240, 320, 3), dtype=np.uint8), label='Estimated depth',
            format='jpeg', visible=False)
        self.mode = self.server.gui.add_dropdown(
            'Image source', ('processed', 'raw'), initial_value='processed')
        self.point_size = self.server.gui.add_slider(
            'Point size', min=0.001, max=0.05, step=0.001, initial_value=0.005, visible=False)
        self.fit = self.server.gui.add_button('Fit cloud', visible=False)
        self.fit.on_click(lambda event: self._connect(event.client) if event.client else None)
        self.pick = self.server.gui.add_checkbox('Pick a 3D ray', initial_value=False, visible=False)
        self._folder = self.server.gui.add_folder('Processing controls', visible=False)
        self._status = self.server.gui.add_markdown('Waiting for a frame.')
        self._sync_controls()
        self.server.on_client_connect(self._connect)
        self.server.scene.on_click()(self._clicked)
        self._thread = threading.Thread(target=self._run, name='perception-viser', daemon=True)
        self._thread.start()

    def _connect(self, client):
        center = np.array([0.0, 0.0, 1.0])
        distance = 3.0
        if self._cloud is not None and len(self._cloud.points):
            lo = self._cloud.points.min(axis=0)
            hi = self._cloud.points.max(axis=0)
            center = (lo + hi) / 2
            distance = max(float(np.linalg.norm(hi - lo)) * 1.5, 0.1)
        client.camera.position = center + [0.0, 0.0, -distance]
        client.camera.look_at = center
        client.camera.up_direction = (0.0, -1.0, 0.0)

    def _clicked(self, event):
        if self.pick.value:
            self.workbench.configure(ray={
                'origin': tuple(event.ray_origin),
                'direction': tuple(event.ray_direction),
            })
            self.pick.value = False

    def _sync_controls(self):
        controls = self.workbench.controls()
        for name in list(self._control_handles):
            if name not in controls:
                self._control_handles.pop(name).remove()
        for name, value in controls.items():
            if not isinstance(value, (bool, int, float, str)):
                continue
            handle = self._control_handles.get(name)
            if handle is None:
                with self._folder:
                    if isinstance(value, bool):
                        handle = self.server.gui.add_checkbox(name, initial_value=value)
                    elif isinstance(value, float) and 0 <= value <= 1:
                        handle = self.server.gui.add_slider(
                            name, min=0.0, max=1.0, step=0.01, initial_value=value)
                    elif isinstance(value, (int, float)):
                        handle = self.server.gui.add_number(name, initial_value=value)
                    else:
                        handle = self.server.gui.add_text(name, initial_value=value)
                handle.on_update(self._control_callback(name))
                self._control_handles[name] = handle
            elif handle.value != value:
                handle.value = value

        self._folder.visible = bool(self._control_handles)

    def _control_callback(self, name):
        def changed(event):
            if event.client is not None:
                self.workbench.configure(**{name: event.target.value})
        return changed

    def _update(self):
        published = False
        self._sync_controls()
        frame = self.workbench.latest(processed=self.mode.value == 'processed')
        if frame is None:
            return
        key = (self.mode.value, frame['sequence'])
        if key != getattr(self, '_image_key', None):
            self.image.image = np.ascontiguousarray(frame['image'][..., ::-1])
            self._image_key = key
            with self._metrics_lock:
                self._counts['image_updates'] += 1
            published = True
        processed = frame if self.mode.value == 'processed' else self.workbench.latest(processed=True)
        depth_image = None if processed is None else processed.get('depth_image')
        self.depth_image.visible = depth_image is not None
        if (depth_image is not None
                and processed['sequence'] != getattr(self, '_depth_sequence', None)):
            self.depth_image.image = np.ascontiguousarray(depth_image[..., ::-1])
            self._depth_sequence = processed['sequence']
            with self._metrics_lock:
                self._counts['depth_updates'] += 1
            published = True
        points = None if processed is None else processed.get('points')
        has_cloud = points is not None and len(points) > 0
        for handle in (self.point_size, self.fit, self.pick, self._origin):
            handle.visible = has_cloud
        if not has_cloud:
            self.pick.value = False
            if self._cloud is not None:
                self._cloud.visible = False
        elif processed['sequence'] != getattr(self, '_cloud_sequence', None):
            points = np.asarray(points, dtype=np.float32)
            colors = processed.get('colors')
            if colors is None:
                colors = np.full(points.shape, (80, 190, 255), dtype=np.uint8)
            else:
                colors = np.asarray(colors, dtype=np.uint8)
            if points.ndim != 2 or points.shape[1] != 3 or colors.shape != points.shape:
                raise ValueError('Expected points and RGB colors with shape Nx3')
            valid = np.isfinite(points).all(axis=1)
            with self.server.atomic():
                if self._cloud is None:
                    self._cloud = self.server.scene.add_point_cloud(
                        '/points', points=points[valid], colors=colors[valid],
                        point_size=self.point_size.value, precision='float32')
                else:
                    self._cloud.points = points[valid]
                    self._cloud.colors = colors[valid]
                    self._cloud.visible = True
            self._cloud_sequence = processed['sequence']
            with self._metrics_lock:
                self._counts['cloud_updates'] += 1
            published = True
        if self._cloud is not None:
            self._cloud.point_size = self.point_size.value
        now = time.monotonic()
        if now - self._last_status_time > 0.25:
            status = dict(self.workbench.status())
            status['queued_sequence'] = frame['sequence']
            status['queued_age_ms'] = round((now - frame['stamp']) * 1000, 1)
            if self.error:
                status['viewer_error'] = self.error
            self._status.content = '```json\n' + json.dumps(status, indent=2, default=str) + '\n```'
            self._last_status_time = now
        return published

    def status(self):
        """Server publication counts and preparation time, excluding browser work."""
        with self._metrics_lock:
            stats = dict(self._counts)
            times = list(self._update_times)
        stats['elapsed_s'] = time.monotonic() - self._started
        stats['publication_samples'] = len(times)
        stats['update_ms_median'] = float(np.median(times)) if times else None
        stats['update_ms_p95'] = float(np.percentile(times, 95)) if times else None
        stats['error'] = self.error
        return stats

    def _run(self):
        while not self._stop.is_set():
            try:
                start = time.perf_counter()
                published = self._update()
                if published:
                    with self._metrics_lock:
                        self._update_times.append((time.perf_counter() - start) * 1000)
                self.error = None
            except Exception as exc:
                self.error = f'{type(exc).__name__}: {exc}'
                self._status.content = f'Viewer error: `{self.error}`'
            self._stop.wait(self._interval)

    def close(self):
        """Stop this viewer, leaving the camera, processor and models alive."""
        if self._stop.is_set():
            return
        self._stop.set()
        self._thread.join(timeout=2)
        self.server.stop()
