"""Shared OFFLINE fixtures. No motorbridge import and no hardware access."""
from types import SimpleNamespace

class FakeMotor:
    def __init__(self, position):
        self.state = SimpleNamespace(pos=position, vel=0., torq=0., status_code=0,
                                     t_mos=30., t_rotor=30.)
        self.commands = []

    def ensure_mode(self, *args):
        return True

    def get_state(self):
        return self.state

    def request_feedback(self):
        pass

    def send_mit(self, pos, vel, kp, kd, tau):
        self.commands.append((pos, vel, kp, kd, tau))
        self.state.torq = tau

class FakeController:
    def __init__(self, motors):
        self.motors = motors

    def enable_all(self):
        for motor in self.motors:
            motor.state.status_code = 1

    def disable_all(self):
        for motor in self.motors:
            motor.state.status_code = 0

    def poll_feedback_once(self):
        pass
