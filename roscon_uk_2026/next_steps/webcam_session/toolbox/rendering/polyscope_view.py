"""Unified native RGB/depth/cloud display using Polyscope's own ImGui panels."""
import threading
import time
import traceback

import numpy as np
import polyscope as ps
import polyscope.imgui as ui


class PolyscopeViewer:
    def __init__(self, workbench):
        self.workbench = workbench
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._sequence = None
        self._cloud = None
        self._point_count = 0
        self._fit_requested = True
        self._ticks = 0
        self._updates = 0
        self._times = []
        self._error = None
        self.screenshot_path = None
        self._started = time.monotonic()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not self._ready.wait(15):
            raise RuntimeError('Polyscope startup timed out')
        if self._error:
            raise RuntimeError(self._error)

    def _controls(self):
        ui.SetWindowPos((10, 560))
        ui.SetWindowSize((305, 230))
        ui.SetWindowPos('RGB + depth', (330, 10), 2)
        ui.SetWindowSize('RGB + depth', (690, 320), 2)
        controls = self.workbench.controls()
        changed, value = ui.SliderInt('Point stride', int(controls.get('stride',2)), 1, 8)
        if changed:
            self.workbench.configure(stride=value)
        if ui.Button('Fit cloud'):
            self._fit_requested = True
        ui.TextUnformatted('DA3 relative estimated depth, not metres')
        packet = self.workbench.latest(processed=True)
        if packet:
            ui.TextUnformatted(f"{len(packet.get('points',[]))} points | source {packet['sequence']}")
        status = self.workbench.status()
        ui.TextUnformatted(f"Capture {status['capture_fps']:.1f} Hz | inference {status['inference_fps']:.1f} Hz")

    def _update(self):
        packet = self.workbench.latest(processed=True)
        if packet is None or packet['sequence'] == self._sequence:
            return
        start = time.perf_counter()
        # One image quantity avoids Polyscope's mutually exclusive root images.
        combined = np.concatenate((packet['image'][..., ::-1],
                                   packet['depth_image'][..., ::-1]), axis=1)
        ps.add_color_image_quantity('RGB + depth', combined.astype(np.float32)/255,
                                    enabled=True, show_in_imgui_window=True)
        points = packet['points']
        if len(points):
            if self._cloud is None or len(points) != self._point_count:
                self._cloud = ps.register_point_cloud('Depth cloud', points, enabled=True,
                                                       radius=0.002, point_render_mode='quad')
                self._point_count = len(points)
            else:
                self._cloud.update_point_positions(points)
            self._cloud.add_color_quantity('RGB colors', packet['colors']/255., enabled=True)
            if self._fit_requested:
                low, high = np.percentile(points, [1, 99], axis=0)
                center = (low + high)/2
                distance = max(float(np.linalg.norm(high-low))*1.4, 1.)
                ps.look_at_dir(center + [0,0,-distance], center, [0,-1,0])
                self._fit_requested = False
        self._sequence = packet['sequence']
        self._updates += 1
        self._times.append((time.perf_counter()-start)*1000)
        self._times = self._times[-600:]

    def _run(self):
        initialized = False
        try:
            ps.set_program_name('RGB / depth / cloud')
            ps.set_window_size(1200,800)
            ps.set_use_prefs_file(False)
            ps.init()
            initialized = True
            ps.set_up_dir('neg_y_up')
            ps.set_ground_plane_mode('none')
            ps.set_max_fps(60)
            ps.set_user_callback(self._controls)
            self._ready.set()
            while not self._stop.is_set() and not ps.window_requests_close():
                start=time.monotonic()
                self._update()
                ps.frame_tick()
                self._ticks += 1
                if self.screenshot_path:
                    self._fit_requested = True
                    self._sequence = None
                    self._update()
                    ps.frame_tick()
                    ps.screenshot(self.screenshot_path, transparent_bg=False, include_UI=True)
                    self.screenshot_path = None
                self._stop.wait(max(0,1/60-(time.monotonic()-start)))
        except Exception:
            self._error = traceback.format_exc()
            self._ready.set()
        finally:
            if initialized:
                ps.shutdown()

    def status(self):
        return dict(running=self._thread.is_alive(), updates=self._updates,
                    render_ticks=self._ticks, error=self._error,
                    update_ms_median=float(np.median(self._times)) if self._times else None,
                    update_ms_p95=float(np.percentile(self._times,95)) if self._times else None)

    def close(self):
        self._stop.set()
        self._thread.join(5)
