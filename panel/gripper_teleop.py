"""Bounded J7 commissioning policy; no CAN I/O and no URDF/control coupling."""

import math

from .gamepad_input import trigger_intent, triggers_neutral, valid_triggers
from .gripper_hold import GripHold, GripperCommand
from .gripper_torque import TorqueClamp
from .experience_policy import GRIP_CONTROL_PAUSE_S,GRIP_SOFT_SPEED_RAD_S

MECHANICAL_DEG = (49.3, 241.3)
CAN_MARGIN_DEG = 5.0  # Minimum software inset, not a measured braking distance.
ENVELOPE_DEG = (MECHANICAL_DEG[0] + CAN_MARGIN_DEG, MECHANICAL_DEG[1] - CAN_MARGIN_DEG)
ENVELOPE_RAD = tuple(map(math.radians, ENVELOPE_DEG))
MAX_SPEED_RAD_S = .375  # 1.5x the previous 0.25rad/s; approximately 21.5deg/s.
LOCAL_KD = 1.2
# Derived telemetry estimate only, NOT a total-torque/pinch safety limit.
ZERO_SPEED_TORQUE_ESTIMATE_NM = LOCAL_KD * MAX_SPEED_RAD_S
IDLE_KD = .2
BRAKE_KD = .4
MAX_ACCEL_RAD_S2 = .5
SOFT_WALL_RAD = math.radians(8.)
HARD_SPEED_RAD_S = 1.5
SOFT_SPEED_RAD_S = GRIP_SOFT_SPEED_RAD_S
RECOVERY_SPEED_RAD_S = .20
FEEDBACK_SOFT_AGE_MS = 120.
MIN_SPAN_RAD = .03
WATCHDOG_MS = (100., 250.)


def range_from_points(points, *, complete=True):
    """Validate captured points without ever expanding the software envelope."""
    if not isinstance(points, dict):
        return None
    opened, inward = points.get("open"), points.get("closed")
    for value in (opened, inward):
        if value is not None and (type(value) not in (int, float) or not math.isfinite(value)
                                  or not ENVELOPE_RAD[0] <= value <= ENVELOPE_RAD[1]):
            return None
    if opened is None or inward is None:
        return None if complete else (opened, inward)
    return (opened, inward) if inward - opened >= MIN_SPAN_RAD else None


def validate_gripper_range(points):
    return range_from_points(points)


def gripper_phase(raw_position):
    """Resolve equivalent whole-turn encoder representations in the known arc.

    No motor zero is changed. A mechanically different zero/calibration still
    requires re-teaching; raw positions are retained in telemetry.
    """
    if not math.isfinite(raw_position) or abs(raw_position) > 12.5:
        return raw_position
    lo, hi = map(math.radians, (MECHANICAL_DEG[0]-1., MECHANICAL_DEG[1]+1.))
    turns = round(((lo+hi)*.5 - raw_position) / math.tau)
    candidate = raw_position + turns*math.tau
    return candidate if turns and lo <= candidate <= hi else raw_position


def firmware_timeout_ms(raw):
    """Damiao TIMEOUT register 9, in units of 50 microseconds."""
    if type(raw) is not int or not WATCHDOG_MS[0] * 20 <= raw <= WATCHDOG_MS[1] * 20:
        raise RuntimeError("J7 驱动器 CAN 超时未在 100～250 ms 内，夹爪遥操未启用")
    return raw * .05


def clamp(x, lo, hi):
    return min(hi, max(lo, x))


class GripperTeleop:
    """Generate a local MIT velocity command, never cached-velocity torque."""

    def __init__(self, limits=None, *, hold_torque=None, calibration=None, torque_limited=False):
        if limits is not None:
            verified = validate_gripper_range(dict(zip(("open", "closed"), limits))) if len(limits) == 2 else None
            if verified is None:
                raise ValueError("J7 软件范围未标定或越界")
            limits = verified
        self.limits = limits
        self.velocity_ref = 0.
        self.status = "J7 等待扳机" if limits else "J7 夹爪锁定"
        self.close_blocked = False
        self.no_progress_s = 0.
        self.mit_kd = IDLE_KD
        self.sequence = None
        self.elapsed_since_frame = 0.
        self.blocked_direction = 0
        self.speed_hold = False
        self.recovery_frames = 0
        self.pause_count = 0
        self.pause_reason = None
        self.pause_ff = 0.
        self.pause_hard = False
        self.pause_trigger=None
        self._pause_context={}
        if torque_limited and (hold_torque is not None or calibration is not None):
            raise ValueError("电机限矩模式不能叠加旧测力档")
        self.hold = TorqueClamp(limits) if torque_limited else GripHold(limits,hold_torque,calibration)
        self.torque_ff = 0.
        self.control_state = "IDLE"

    @property
    def command(self):
        return GripperCommand(self.velocity_ref, self.mit_kd, self.torque_ff, self.control_state, self.status)

    def contact_motion_snapshot(self, position):
        snapshot=getattr(self.hold,"motion_snapshot",None)
        return snapshot(position) if snapshot else None

    def _apply(self, command, velocity):
        self.velocity_ref, self.mit_kd, self.torque_ff = command.velocity, command.kd, command.tau_ff
        self.control_state, self.status = command.state, command.status
        return command.estimate(velocity)

    def _brake(self, status, kd=IDLE_KD):
        self.velocity_ref = 0.
        self.mit_kd = kd
        self.torque_ff = 0.
        self.control_state = "BLOCKED" if self.blocked_direction else "IDLE"
        self.status = status

    def _fault(self, reason):
        self.hold.cancel(reason)
        self._brake(reason)
        self.control_state = "INHIBITED"
        raise RuntimeError(reason)

    def _pause(self, reason, *, hard=False):
        if not self.speed_hold:
            self.pause_ff=max(0.,min(2.,self.torque_ff))
            self.pause_count+=1
            self.pause_reason=reason
            self.pause_trigger={**self._pause_context,"reason":reason}
            self.recovery_frames=0
            self.hold.cancel(reason)
        elif hard and not self.pause_hard:
            self.pause_count+=1
            self.pause_reason=reason
            self.pause_trigger={**self._pause_context,"reason":reason}
        self.speed_hold=True
        self.pause_hard=self.pause_hard or hard

    def _pause_step(self, velocity, dt):
        # Preserve arm support; never call disable or throw for a healthy J7
        # speed event. Bleed existing clamp effort instead of clearing it.
        self.pause_ff=max(0.,self.pause_ff-2.*min(dt,.02))
        budget=max(0.,2.-self.pause_ff)
        kd=min(BRAKE_KD,budget/(abs(velocity)+.02))
        return self._apply(GripperCommand(kd=kd,tau_ff=self.pause_ff,state="PAUSED",
            status=f"J7 保护暂停 · {self.pause_reason}；手臂保持，释放LB并回中恢复"),velocity)

    def update(self, *, position, velocity, torque, triggers, armed, dt,
               feedback_sequence=None, feedback_age_ms=None, hold_allowed=None, allow_grasp=True):
        if self.limits is None:
            return 0.
        if not all(math.isfinite(v) for v in (position, velocity, torque, dt)) or dt <= 0:
            self._fault("J7 反馈或控制周期无效")
        lo, hi = self.limits
        if not math.radians(MECHANICAL_DEG[0] - 1) <= position <= math.radians(MECHANICAL_DEG[1] + 1):
            self._fault("J7 超出已知机械行程，禁止继续驱动")
        if (type(feedback_sequence) is not int or feedback_sequence <= 0
                or feedback_age_ms is None or not math.isfinite(feedback_age_ms)
                or not 0 <= feedback_age_ms <= 150):
            self._fault("J7 缺少新鲜有效的 CAN 反馈时间戳")
        if self.sequence is not None and feedback_sequence < self.sequence:
            self._fault("J7 CAN 反馈序号回退")
        fresh = feedback_sequence != self.sequence
        self.elapsed_since_frame = min(.15, self.elapsed_since_frame + dt)
        sample_dt = self.elapsed_since_frame if fresh and self.sequence is not None else 0.
        self.sequence = feedback_sequence
        if fresh:
            self.elapsed_since_frame = 0.
        value = trigger_intent(triggers)
        released = triggers_neutral(triggers)
        self._pause_context={"cycle_ms":dt*1000,"velocity_rad_s":velocity,
                             "feedback_age_ms":feedback_age_ms,"position_rad":position,
                             "force_before_nm":self.torque_ff,"state_before":self.control_state,
                             "contact_motion":self.contact_motion_snapshot(position)}
        if dt>GRIP_CONTROL_PAUSE_S and self.torque_ff>0.:
            self._pause(f"控制周期 {dt*1000:.2f}ms 超过 {GRIP_CONTROL_PAUSE_S*1000:.0f}ms")
        if abs(velocity)>HARD_SPEED_RAD_S:
            self._pause(f"硬超速 {velocity:+.3f}rad/s（阈值1.5）",hard=True)
            self.recovery_frames = 0
        elif abs(velocity)>=SOFT_SPEED_RAD_S or feedback_age_ms>=FEEDBACK_SOFT_AGE_MS:
            self._pause("软超速或反馈短暂延迟")
            self.recovery_frames=0
        elif fresh and self.speed_hold:
            self.recovery_frames = self.recovery_frames + 1 if abs(velocity) <= RECOVERY_SPEED_RAD_S else 0
        if self.speed_hold:
            estimate=self._pause_step(velocity,dt)
            if not (fresh and released and not armed and self.recovery_frames>=2 and self.pause_ff<=1e-12):
                return estimate
            self.speed_hold = False
            self.recovery_frames = 0
            self.pause_hard=False
            self.pause_reason=None
            self._brake("J7 暂停已恢复 · 等待重新按LB")
            return -self.mit_kd*velocity
        override = self.hold.before(position=position, dt=dt, value=value, released=released,
            armed=armed, allowed=(armed if hold_allowed is None else hold_allowed) and valid_triggers(triggers), fresh=fresh, torque=torque)
        if override is not None:
            self.no_progress_s = 0.
            if override.state=="PAUSE_REQUIRED":
                self._pause_context["contact_motion"]=self.contact_motion_snapshot(position)
                self._pause(override.status)
                return self._pause_step(velocity,dt)
            return self._apply(override, velocity)
        # Neutral, lost input or a late tick never retain a motion command.
        if not armed or dt > GRIP_CONTROL_PAUSE_S or not value:
            self.no_progress_s = 0.
            if released:
                self.close_blocked = False
                self.blocked_direction = 0
            self._brake("J7 零速阻尼 · 不保持载荷")
            return -self.mit_kd * velocity
        direction = 1 if value > 0. else -1
        if self.blocked_direction and direction != self.blocked_direction:
            self.blocked_direction = 0
            self.close_blocked = False
            self.no_progress_s = 0.
        if self.blocked_direction:
            self._brake("J7 行程受阻 · 释放扳机或反向离开")
            return -self.mit_kd * velocity
        if (value > 0. and position >= hi) or (value < 0. and position <= lo):
            self.no_progress_s = 0.
            self._brake("J7 软件端点 · 可向反方向离开")
            return -self.mit_kd * velocity

        distance = hi-position if value > 0. else position-lo
        wish = MAX_SPEED_RAD_S * value * clamp(distance/SOFT_WALL_RAD, 0., 1.)
        if self.velocity_ref * wish < 0.:
            self.no_progress_s = 0.
            self._brake("J7 反向过渡")
            return -self.mit_kd * velocity
        # Only a distinct received frame may increase the speed reference.
        # A smaller request or wall always takes effect, including stale reads.
        h = min(sample_dt, .05)
        if fresh:
            self.velocity_ref += clamp(wish-self.velocity_ref, -MAX_ACCEL_RAD_S2*h, MAX_ACCEL_RAD_S2*h)
        self.velocity_ref = math.copysign(min(abs(self.velocity_ref), abs(wish)), wish)
        self.mit_kd = LOCAL_KD
        self.torque_ff = 0.
        self.control_state = "CLOSING" if value > 0. else "OPENING"
        contact = self.hold.observe(position=position, velocity=velocity, torque=torque, value=value,
            armed=armed, vref=self.velocity_ref, fresh=fresh, sample_dt=sample_dt) if allow_grasp else None
        if contact is not None:
            self.no_progress_s = 0.
            return self._apply(contact, velocity)
        # Detection applies to opening too: never force through an unknown jam.
        if fresh and abs(self.velocity_ref) >= .9*abs(wish) and abs(wish) > .08 and abs(velocity) < .015:
            self.no_progress_s += sample_dt
            if self.no_progress_s >= .35:
                self.blocked_direction = direction
                self.close_blocked = direction > 0
                # A motion stall is not a measured-force/contact event. Keep
                # this temporary stop separate from GripHold's threshold latch;
                # releasing the trigger may retry, at the same torque budget.
                self._brake("J7 未检测到运动 · 本次暂停，未确认夹取；松开扳机可重试，或反向离开")
                return -self.mit_kd * velocity
        elif fresh:
            self.no_progress_s = 0.
        self.status = "J7 收拢" if value > 0. else "J7 张开"
        # This is only an estimate for telemetry/simulation. Actual regulation
        # uses current motor velocity inside the drive: Kp=0, Kd>0, tau_ff=0.
        return self.mit_kd * (self.velocity_ref-velocity)
