"""Minimal position-holding controller for the control_kit Python adapter."""


class HoldPosition:
    def configure(self, joints, options):
        pass

    def activate(self, measured_positions):
        self.reference = list(measured_positions)

    def update(self, measured_positions, dt):
        return self.reference

    def set_target(self, values):
        return False  # This example holds its activation pose.
