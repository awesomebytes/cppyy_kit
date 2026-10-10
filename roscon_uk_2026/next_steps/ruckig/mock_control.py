"""Separate checkout integration probe using the existing control Pixi environment.

The controller manager and GenericSystem hardware are real ros2_control objects.
GenericSystem mirrors commands; this probe has no physical plant dynamics.
"""
import importlib.metadata
import json
import os
import time
from pathlib import Path
os.environ.setdefault('ROS_DOMAIN_ID', '83')
import numpy as np
import control_kit as ck
from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
from native import session, set_target, vector

JOINTS = ['ruckig_a', 'ruckig_b', 'ruckig_c']


class Tracking(ck.ControllerInterface):
    def __init__(self):
        super().__init__()
        self.engine = session(3)
        self.tick = 0
        self.samples = []
        self.update_ns = []
        self.recalculations = []

    def on_init(self):
        return ck.CallbackReturn.SUCCESS

    def command_interface_configuration(self):
        return ck.interface_config([f'{joint}/position' for joint in JOINTS])

    def state_interface_configuration(self):
        return ck.interface_config([f'{joint}/position' for joint in JOINTS])

    def on_configure(self, state):
        return ck.CallbackReturn.SUCCESS

    def on_activate(self, state):
        self.engine.reset(vector([0.]*3), vector([0.]*3), vector([0.]*3))
        self.tick = 0
        self.samples.clear()
        return ck.CallbackReturn.SUCCESS

    def on_deactivate(self, state):
        return ck.CallbackReturn.SUCCESS

    def update(self, stamp, period):
        start = time.perf_counter_ns()
        if self.tick in [0, 400, 900]:
            set_target(self.engine, {0: [.6, -.3, .2], 400: [-.2, .4, -.1], 900: [.1, 0., .2]}[self.tick])
        measured = [ck.read_state(self, i) for i in range(3)]
        sample = self.engine.step(vector(measured))
        for i in range(3):
            ck.write_command(self, i, sample.command[i])
        self.samples.append((self.tick, list(sample.position), measured, period.seconds()))
        if sample.new_calculation:
            self.recalculations.append(self.tick)
        self.tick += 1
        self.update_ns.append(time.perf_counter_ns()-start)
        return ck.return_type.OK


def main():
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    ck.bringup_control()
    rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS), update_rate=1000)
    controller = Tracking()
    try:
        rig.add_python_controller(controller, 'ruckig_tracking')
        assert rig.configure('ruckig_tracking')
        assert rig.activate(['ruckig_tracking'])
        start = time.perf_counter()
        cycles = rig.run(seconds=3.2, rate_hz=1000)
        elapsed = time.perf_counter()-start
        tail = controller.samples[-250:]
        error = max(abs(reference-position) for _, refs, positions, _ in tail
                    for reference, position in zip(refs, positions))
        periods = [dt for _, _, _, dt in controller.samples]
        assert controller.recalculations == [0, 400, 900]
        assert all(np.isfinite(dt) and dt >= 0 for dt in periods)
        assert cycles >= 3000 and error < 1e-6
        result = {'status': 'PASS', 'cycles': cycles, 'elapsed_s': elapsed,
                  'callbacks': len(controller.samples), 'target_update_ticks': controller.recalculations,
                  'tail_max_reference_error_rad': error,
                  'callback_p50_us': float(np.percentile(controller.update_ns, 50)/1000),
                  'callback_p99_us': float(np.percentile(controller.update_ns, 99)/1000),
                  'generator_dt_s': .001, 'manager_period_min_s': min(periods), 'manager_period_max_s': max(periods), 'manager_period_p50_s': float(np.median(periods)), 'hardware': 'mock_components/GenericSystem',
                  'versions': {name: importlib.metadata.version(name) for name in
                               ['ros-jazzy-control-kit', 'cppyy', 'numpy']},
                  'scope': 'Python callback; native reference advances 1 ms per callback independently of wall time; no plant dynamics or real-time claim'}
        (Path(__file__).parent / 'mock_results.json').write_text(json.dumps(result, indent=2)+'\n')
        print('RESULT ' + json.dumps(result))
    finally:
        rig._teardown()


if __name__ == '__main__':
    main()
