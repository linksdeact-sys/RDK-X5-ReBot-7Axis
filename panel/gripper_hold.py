"""No-I/O grip confirmation and bounded torque-hold state machine."""
from dataclasses import dataclass
import math
from .experience_policy import GRIP_CONTROL_PAUSE_S

HOLD_TORQUE_MAX = .20  # Legacy gauge-pulse/profile cap only, NOT the 2Nm policy.
HOLD_TORQUE_SLEW = .20
HOLD_TRAVEL = math.radians(2.)
CONTACT_TRAVEL = .02
CONTACT_SECONDS = .15
CONTACT_TIMEOUT = 3.
CONTROL_NAME = "velocity-torque-pause-v6"


@dataclass(frozen=True)
class GripperCommand:
    velocity: float = 0.
    kd: float = .2
    tau_ff: float = 0.
    state: str = "IDLE"
    status: str = "J7 等待扳机"

    def mit(self, raw_position):
        return (float(raw_position), self.velocity, 0., self.kd, self.tau_ff)

    def estimate(self, measured_velocity):
        return self.tau_ff + self.kd*(self.velocity-measured_velocity)


class GripHold:
    def __init__(self, limits, torque=None, calibration=None):
        if torque is not None and (type(torque) not in (int, float) or
                                   not math.isfinite(torque) or not 0 < torque <= HOLD_TORQUE_MAX):
            raise ValueError("保持力矩必须是经过标定的正值且不超过0.20Nm")
        self.limits, self.torque, self.calibration = limits, torque, calibration
        self.close_locked = False
        self.lock_position = None
        self.state, self.force = "IDLE", 0.
        self.anchor = self.close_start = None
        self.stall_time = self.wait_time = 0.
        self.stall_frames = 0
        self.reason = ""

    @property
    def enabled(self):
        return self.limits is not None and (self.torque is not None or self.calibration is not None)

    def cancel(self, reason, *, inhibit=True):
        self.state, self.force = ("INHIBITED" if inhibit else "IDLE"), 0.
        self.anchor = self.close_start = None
        self.stall_time = self.wait_time = 0.
        self.stall_frames = 0
        self.reason = reason
        if self.calibration:
            self.calibration.tick(ready=False, armed=False, safe=False, dt=0., position=0.)

    def _ramp(self, target, dt):
        step = HOLD_TORQUE_SLEW * min(dt, .02)
        self.force += max(-step, min(step, target-self.force))
        self.force = max(0., min(self.force, HOLD_TORQUE_MAX))

    def before(self, *, position, dt, value, released, armed, allowed, fresh, torque=0.):
        if not self.enabled:
            return None
        if not allowed or dt > GRIP_CONTROL_PAUSE_S:
            self.cancel("J7 保持已取消 · 输入/控制不可用")
            return GripperCommand(state=self.state, status=self.reason)
        if self.close_locked:
            if fresh and armed and value < 0 and position <= self.lock_position-HOLD_TRAVEL:
                self.close_locked = False
                self.lock_position = None
                if self.state == "INHIBITED":
                    self.state = "IDLE"
            elif self.state not in ("HOLD", "RELEASING") and not (armed and value < 0):
                return GripperCommand(state="CLOSE_LOCKED", status="J7 收拢已锁止 · 只能LB＋LT张开解除")
        if self.state == "INHIBITED":
            if released and fresh:
                self.state = "IDLE"
            elif value >= 0 or not armed:
                return GripperCommand(state=self.state, status=self.reason)
        if self.state not in ("CANDIDATE", "HOLD", "CALIBRATION_READY", "RELEASING"):
            if not armed or value <= 0:
                self.close_start = None
                self.stall_time = self.stall_frames = 0
            return None
        lo, hi = self.limits
        if not lo < position < hi or self.anchor is None or abs(position-self.anchor) > HOLD_TRAVEL:
            self.cancel("J7 接触丢失/偏移或到达端点 · 需重新夹取")
            return GripperCommand(state=self.state, status=self.reason)
        if self.state == "CANDIDATE":
            self.wait_time += dt
            if value < 0 and armed:
                self.cancel("J7 取消接触，开始张开", inhibit=False)
                return None
            if self.wait_time > CONTACT_TIMEOUT:
                self.cancel("J7 接触确认超时 · 松开扳机后重试")
                return GripperCommand(state=self.state, status=self.reason)
            if released and fresh:
                self.state = "CALIBRATION_READY" if self.calibration else "HOLD"
            else:
                return GripperCommand(state=self.state, status="J7 检测到接触 · 松开RT确认保持")
        if self.state == "CALIBRATION_READY":
            if value < 0 and armed:
                self.cancel("J7 标定接触已释放", inhibit=False)
                return None
            target = self.calibration.tick(ready=True, armed=armed and released,
                                           safe=allowed, dt=dt, position=position, command_tau=self.force)
            self._ramp(target, dt if fresh else 0.)
            # Calibration deadman release must cancel torque immediately.
            if not target or not armed or not released:
                self.force = 0.
            return GripperCommand(kd=.2, tau_ff=self.force, state=self.state,
                                  status="J7 测力脉冲中" if target else "J7 标定接触就绪 · 按住LB进行测力")
        if self.state == "HOLD" and value < 0 and armed:
            self.state = "RELEASING"
        if self.state == "RELEASING":
            self._ramp(0., dt)
            if self.force <= 1e-12:
                self.cancel("J7 夹紧力已释放", inhibit=False)
                if self.close_locked and not (armed and value < 0):
                    return GripperCommand(state="CLOSE_LOCKED", status="J7 收拢已锁止 · 只能LB＋LT张开解除")
                return None
            return GripperCommand(kd=.2, tau_ff=self.force,
                                  state=self.state, status="J7 正在撤掉夹紧力")
        self._ramp(self.torque, dt if fresh else 0.)
        return GripperCommand(kd=.2,tau_ff=self.force,state="HOLD",status="J7 夹紧保持 · 原人工测力档（非实时测力）")

    def observe(self, *, position, velocity, torque, value, armed, vref, fresh, sample_dt):
        if not self.enabled:
            return None
        if value <= 0 or not armed:
            self.close_start = None
            self.stall_time = self.stall_frames = 0
            return None
        if self.close_start is None:
            self.close_start = position
        if not fresh:
            return None
        lo, hi = self.limits
        threshold = .02
        eligible = (position-self.close_start >= CONTACT_TRAVEL and hi-position > HOLD_TRAVEL
                    and position > lo and vref > .08 and abs(velocity) < .015 and torque >= threshold)
        if eligible:
            self.stall_time += sample_dt
            self.stall_frames += 1
        else:
            self.stall_time = self.stall_frames = 0
        if self.stall_frames >= 3 and self.stall_time >= CONTACT_SECONDS:
            self.state, self.anchor, self.force = "CANDIDATE", position, 0.
            self.wait_time = 0.
            return GripperCommand(state=self.state, status="J7 检测到接触 · 松开RT确认保持")
        return None
