"""Native image controls for the Linux marimo workbench, without image encoding."""
import threading
import time
import traceback

from imgui_bundle import hello_imgui, imgui, immvision


class ImGuiViewer:
    """Run the native window on its own Linux GUI thread; keep the kernel free."""

    def __init__(self, workbench):
        self.workbench = workbench
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._error = None
        self._mode = 0
        self._draw = None
        self._gui_error = None
        self._ticks = 0
        self._shown = 0
        self._last_sequence = None
        self._started = time.monotonic()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=15):
            raise RuntimeError('Native viewer startup timed out')
        if self._error:
            raise RuntimeError(self._error)

    def set_gui(self, function):
        """Replace extra controls drawn after the standard controls each frame."""
        self._draw = function

    def status(self):
        elapsed = max(time.monotonic() - self._started, 0.001)
        return dict(running=self._thread.is_alive(), render_ticks=self._ticks,
                    render_fps=self._ticks/elapsed, unique_images=self._shown,
                    image_updates_fps=self._shown/elapsed, error=self._error)

    def _gui(self):
        controls = self.workbench.controls()
        _, self._mode = imgui.combo('View', self._mode, ['Raw camera', 'Processed'])
        changed, threshold = imgui.slider_float('Threshold', controls['threshold'], 0., 1.)
        if changed:
            self.workbench.configure(threshold=threshold)
        changed, label = imgui.input_text('Label', controls['label'])
        if changed:
            self.workbench.configure(label=label)
        changed, enabled = imgui.checkbox('Enabled', controls['enabled'])
        if changed:
            self.workbench.configure(enabled=enabled)
        if self._draw:
            try:
                self._draw(self.workbench)
                self._gui_error = None
            except Exception:
                self._gui_error = traceback.format_exc()
        if self._gui_error:
            imgui.text_wrapped(self._gui_error)
        status = self.workbench.status()
        imgui.text(f"Capture {status['capture_fps']:.1f} Hz | "
                   f"inference {status['inference_fps']:.1f} Hz | "
                   f"processing {status['processing_ms']:.1f} ms")
        if status['result_age_ms'] is not None:
            imgui.text(f"Result age {status['result_age_ms']:.1f} ms | "
                       f"revision {status['result_revision']}")
        if status['error'] or status['capture_error']:
            imgui.text_wrapped(status['error'] or status['capture_error'])
        packet = self.workbench.latest(processed=bool(self._mode))
        if packet is not None:
            key = (self._mode, packet['sequence'])
            self._params.refresh_image = key != self._last_sequence
            if key != self._last_sequence:
                self._shown += 1
                self._last_sequence = key
            immvision.image('Camera', packet['image'], self._params)
            mouse = self._params.mouse_info
            if mouse.is_mouse_hovering and imgui.is_mouse_clicked(0):
                point = mouse.mouse_position
                self.workbench.configure(point=(float(point[0]), float(point[1])),
                                         point_sequence=packet['sequence'])
        self._ticks += 1

    def _run(self):
        initialized = False
        try:
            self._params = immvision.ImageParams()
            self._params.image_display_size = (640, 480)
            immvision.use_bgr_color_order()
            runner = hello_imgui.RunnerParams()
            runner.app_window_params.window_title = 'Interactive perception'
            runner.app_window_params.window_geometry.size = (900, 820)
            runner.callbacks.show_gui = self._gui
            runner.fps_idling.enable_idling = False
            runner.fps_idling.vsync_to_monitor = False
            hello_imgui.manual_render.setup_from_runner_params(runner)
            initialized = True
            self._ready.set()
            while not self._stop.is_set() and not runner.app_shall_exit:
                tick = time.monotonic()
                hello_imgui.manual_render.render()
                self._stop.wait(max(0, 1/60 - (time.monotonic()-tick)))
        except Exception:
            self._error = traceback.format_exc()
        finally:
            self._ready.set()
            if initialized:
                hello_imgui.manual_render.tear_down()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=5)
