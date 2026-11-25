"""Legacy v8 wrist guard, retained for history tests; NOT used by v9 control.

Active arm recovery policy lives in arm_motion_guard.py.
"""

SOFT_LIMIT = .50
IMMEDIATE_LIMIT = .65
CONFIRM_S = .05
RECOVERY_LIMIT = .46
NEUTRAL = .08


class WristSpeedGuard:
    def __init__(self, joint):
        self.joint = joint
        self.sequence = None
        self.first_high_at = None
        self.holding = False
        self.recovery_frames = 0

    def update(self, *, velocity, sequence, received_at, neutral):
        speed = abs(velocity)
        if speed >= IMMEDIATE_LIMIT:
            raise RuntimeError(f"J{self.joint}明显超速 v={velocity:+.3f}rad/s")
        if self.sequence is not None and sequence < self.sequence:
            raise RuntimeError(f"J{self.joint}真实CAN反馈序号回退")
        if sequence != self.sequence:
            self.sequence = sequence
            if speed > SOFT_LIMIT:
                self.holding = True
                self.recovery_frames = 0
                if self.first_high_at is None:
                    self.first_high_at = received_at
                elif received_at - self.first_high_at >= CONFIRM_S:
                    raise RuntimeError(f"J{self.joint}新CAN反馈确认持续超速 v={velocity:+.3f}rad/s")
            else:
                self.first_high_at = None
                self.recovery_frames = self.recovery_frames + 1 if self.holding and speed <= RECOVERY_LIMIT else 0
        if self.holding and self.recovery_frames >= 2 and neutral:
            self.holding = False
            self.recovery_frames = 0
        return self.holding
