"""Direction-aware contact motion guard, without I/O or torque promotion.

Positive J7 motion closes the jaws. A fixed capture point bounds total passive
compression; a separate high-water mark detects reopening. Neither is a moving
position target, and neither can request additional torque.
"""
import math

SETTLE_TRAVEL_RAD = math.radians(12.)
REOPEN_TRAVEL_RAD = math.radians(2.)
CONFIRM_SECONDS = .15
CONFIRM_FRAMES = 3


def policy():
    return {"settle_travel_deg": round(math.degrees(SETTLE_TRAVEL_RAD),6),
            "reopen_travel_deg": round(math.degrees(REOPEN_TRAVEL_RAD),6),
            "confirm_ms": CONFIRM_SECONDS * 1000,
            "confirm_frames": CONFIRM_FRAMES}


class ContactMotionGuard:
    def __init__(self, position):
        self.anchor = self.peak = position
        self.elapsed = 0.
        self.suspect = None
        self.frames = 0
        self.since = None

    def capture_hold(self, position):
        # Explicit clamping has ended. Bound passive settling from here, but
        # never erase evidence that the jaws have already reopened.
        self.anchor = position
        self.peak = max(self.peak, position)

    def update(self, position, dt, fresh, explicit_closing=False):
        self.elapsed += dt
        if not fresh:
            return None
        self.peak = max(self.peak, position)
        if self.peak - position > REOPEN_TRAVEL_RAD:
            suspect = "reopening"
        elif not explicit_closing and position - self.anchor > SETTLE_TRAVEL_RAD:
            suspect = "excess_compression"
        else:
            suspect = None
        if suspect != self.suspect:
            self.suspect, self.frames = suspect, 0
            self.since = self.elapsed if suspect else None
        if suspect is None:
            return None
        self.frames += 1
        if self.frames < CONFIRM_FRAMES or self.elapsed - self.since + 1e-12 < CONFIRM_SECONDS:
            return None
        return ("夹爪反向张开持续超过2°，受控刹停" if suspect == "reopening" else
                "夹爪被动收拢持续超过12°，受控刹停")

    def snapshot(self, position):
        return {"anchor_rad": self.anchor, "peak_rad": self.peak,
                "compression_deg": math.degrees(position-self.anchor),
                "reopening_deg": math.degrees(self.peak-position),
                "suspect": self.suspect, "fresh_frames": self.frames,
                "suspect_ms": 0. if self.since is None else (self.elapsed-self.since)*1000}
