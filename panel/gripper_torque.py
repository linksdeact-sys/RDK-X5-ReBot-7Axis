"""J7 motor-torque clamp, in Nm only. No jaw-force/geometry conversion."""
from .gripper_hold import GripperCommand, HOLD_TRAVEL, CONTACT_TRAVEL, CONTACT_SECONDS
from .experience_policy import GRIP_CONTROL_PAUSE_S
from .gripper_contact_motion import ContactMotionGuard

TORQUE_LIMIT_NM = 2.0
TORQUE_TRIP_NM = 1.99  # Conservative feedback-quantization margin, not a force value.
CLAMP_SLEW_NM_S = .5
RELEASE_SLEW_NM_S = 2.
CONTACT_LOAD_NM = .05  # Contact evidence only; NOT the final torque limit.


class TorqueClamp:
    """Move using local velocity regulation, then ramp only under held RT.

    The 2Nm ceiling bounds commanded feedforward and guards feedback; it is not
    a hardware current limiter and cannot guarantee a physical impact peak.
    """
    def __init__(self, limits):
        self.limits = limits
        self.torque = self.calibration = None  # No legacy N-based profile.
        self.torque_limit = TORQUE_LIMIT_NM
        self.state, self.force, self.reason = "IDLE", 0., ""
        self.close_locked, self.lock_position = False, None
        self.anchor = self.close_start = None
        self.contact_motion = None
        self.stall_frames, self.stall_time = 0, 0.

    @property
    def enabled(self): return self.limits is not None

    def cancel(self, reason, *, inhibit=True):
        self.state = "INHIBITED" if inhibit else "IDLE"
        self.force, self.reason = 0., reason
        self.anchor = self.close_start = None
        self.contact_motion = None
        self.stall_frames, self.stall_time = 0, 0.
        # Once the ceiling has tripped, neutral/retry cannot ratchet it closed.

    def _lock(self, position):
        was_clamping = self.state == "CLAMPING"
        self.close_locked, self.lock_position = True, position
        self.anchor, self.state = position, "HOLD"
        if self.contact_motion is None:
            self.contact_motion = ContactMotionGuard(position)
        elif was_clamping:
            self.contact_motion.capture_hold(position)

    def motion_snapshot(self, position):
        return self.contact_motion.snapshot(position) if self.contact_motion else None

    def _command(self):
        return GripperCommand(kd=0., tau_ff=self.force, state=self.state,
            status=("J7 达到2 N·m软件上限 · 收拢锁止，只能LB＋LT释放" if self.close_locked
                    else f"J7 {'逐步夹紧' if self.state=='CLAMPING' else '保持当前夹紧力矩'} {self.force:.2f}/2.00 N·m"))

    def before(self, *, position, dt, value, released, armed, allowed, fresh, torque=0.):
        if not self.enabled: return None
        if not allowed or dt>GRIP_CONTROL_PAUSE_S:
            self.cancel("J7 夹紧已取消 · 输入/控制不可用")
            return GripperCommand(state=self.state,status=self.reason)
        if fresh and torque>=TORQUE_TRIP_NM and not self.close_locked and (
                self.state in ("CLAMPING","HOLD") or (armed and value>0)):
            self._lock(position)
        if self.close_locked:
            if fresh and armed and value<0 and position<=self.lock_position-HOLD_TRAVEL:
                self.close_locked, self.lock_position = False, None
                if self.state=="INHIBITED": self.state="IDLE"
            elif self.state not in ("HOLD","RELEASING") and not (armed and value<0):
                return GripperCommand(state="CLOSE_LOCKED",status="J7 收拢已锁止 · 只能LB＋LT实际张开解除")
        if self.state=="INHIBITED":
            if released and fresh: self.state="IDLE"
            elif value>=0 or not armed:
                return GripperCommand(state=self.state,status=self.reason)
        if self.state not in ("CLAMPING","HOLD","RELEASING"):
            if not armed or value<=0:
                self.close_start=None
                self.stall_frames,self.stall_time=0,0.
            return None
        lo,hi=self.limits
        if not lo<position<hi or self.anchor is None:
            return GripperCommand(kd=0.,tau_ff=self.force,state="PAUSE_REQUIRED",
                status="夹紧状态/软件端点异常，受控刹停")
        # A valid opening request has priority over drift detection: it first
        # unloads existing effort, then the ordinary velocity controller opens.
        if armed and value<0: self.state="RELEASING"
        if self.state!="RELEASING":
            compressing=armed and value>0 and not self.close_locked
            if self.state=="CLAMPING" and not compressing:
                self.anchor=position
                if self.contact_motion:
                    self.contact_motion.capture_hold(position)
                else:
                    self.contact_motion=ContactMotionGuard(position)
            if self.contact_motion is None:
                self.contact_motion=ContactMotionGuard(self.anchor)
            reason=self.contact_motion.update(position,dt,fresh,explicit_closing=compressing)
            if reason:
                return GripperCommand(kd=0.,tau_ff=self.force,state="PAUSE_REQUIRED",
                    status=reason)
        if self.state=="RELEASING":
            self.force=max(0.,self.force-RELEASE_SLEW_NM_S*min(dt,.02))
            if self.force<=1e-12:
                self.cancel("J7 夹紧力矩已释放",inhibit=False)
                if self.close_locked and not (armed and value<0):
                    return GripperCommand(state="CLOSE_LOCKED",status="J7 收拢已锁止 · 请用LB＋LT张开")
                return None
            return GripperCommand(kd=0.,tau_ff=self.force,state="RELEASING",status="J7 正在撤力后张开")
        if self.close_locked:
            self.state="HOLD"
        elif armed and value>0:
            self.state="CLAMPING"
            if fresh:
                self.force=min(TORQUE_LIMIT_NM,self.force+CLAMP_SLEW_NM_S*min(dt,.02)*value)
                if self.force>=TORQUE_LIMIT_NM: self._lock(position)
        else:
            self.state="HOLD" # RT or LB release freezes, never adds torque.
        return self._command()

    def observe(self, *, position, velocity, torque, value, armed, vref, fresh, sample_dt):
        if not self.enabled or not armed or value<=0:
            self.close_start=None
            self.stall_frames,self.stall_time=0,0.
            return None
        if not fresh: return None
        if torque>=TORQUE_TRIP_NM:
            self.force=0. # A surprise peak must not promote force to 2Nm.
            self._lock(position)
            return self._command()
        if self.close_start is None: self.close_start=position
        lo,hi=self.limits
        eligible=(position-self.close_start>=CONTACT_TRAVEL and hi-position>HOLD_TRAVEL
                  and position>lo and vref>.08 and abs(velocity)<.015 and torque>=CONTACT_LOAD_NM)
        if eligible:
            self.stall_frames+=1
            self.stall_time+=sample_dt
        else: self.stall_frames,self.stall_time=0,0.
        if self.stall_frames<3 or self.stall_time<CONTACT_SECONDS: return None
        # Transfer only existing positive stall effort, not the 2Nm maximum.
        self.force=max(0.,min(torque,1.2*vref,TORQUE_LIMIT_NM))
        self.anchor,self.state=position,"CLAMPING"
        self.contact_motion=ContactMotionGuard(position)
        return self._command()
