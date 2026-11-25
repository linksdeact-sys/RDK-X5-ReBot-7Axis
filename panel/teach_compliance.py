"""reBot-style arm compliance, independently blended from task lifecycle.

Reference: Seeed-Projects/reBotArm_control_py/controllers/gravity_compensation.py
DEFAULT_KP=2, DEFAULT_KD=1 and smoothstep transition. The SDK still supplies
direction-corrected g(q). J7 hand teaching is intentionally a separate adapter.
"""
import math
from .experience_policy import ARM_IMMEDIATE_SPEED_RAD_S,ARM_SOFT_SPEED_RAD_S

DRAG_KP = 2.0
DRAG_KD = 1.0
TRANSITION_SECONDS = .5
SOURCE = "https://github.com/Seeed-Projects/reBotArm_control_py/blob/main/reBotArm_control_py/controllers/gravity_compensation.py"


def policy():
    return {"name":"rebot-current-position-compliance","arm_drag_kp":DRAG_KP,
            "arm_drag_kd":DRAG_KD,"transition_seconds":TRANSITION_SECONDS,
            "target":"measured-position-after-transition","source":SOURCE,
            "drag_soft_speed_pause":False,"drag_tracking_error_pause":False,
            "hard_speed_rad_s":ARM_IMMEDIATE_SPEED_RAD_S,"automatic_mode_soft_speed_rad_s":ARM_SOFT_SPEED_RAD_S,
            "j7":"custom-hand-push-zero-kp-no-grasp-feedforward"}


class TeachCompliance:
    """Continuous gains/target when moving between drag and stiff holding.

    No I/O and no timer thread. Repeated feedback must not restart the blend.
    Late ticks cannot jump the fade forward more than 20ms.
    """
    def __init__(self):
        self.mode=None
        self.elapsed=0.
        self.kp=self.kd=self.target=None
        self.from_kp=self.from_kd=self.anchor=None

    def step(self, mode, desired, measured, dt, hold_kp, hold_kd):
        if mode not in ("DRAG","HOLD","REPLAY"):raise ValueError("未知示教柔顺模式")
        arrays=[list(map(float,a)) for a in (desired,measured,hold_kp,hold_kd)]
        if any(len(a)!=6 or any(not math.isfinite(x) for x in a) for a in arrays):
            raise ValueError("示教柔顺参数需要六轴有限数值")
        if not math.isfinite(dt) or dt<=0:raise ValueError("示教柔顺周期无效")
        desired,measured,hold_kp,hold_kd=arrays
        if any(not 0<=k<=500 for k in hold_kp) or any(not 0<=k<=5 for k in hold_kd):
            raise ValueError("示教柔顺增益超出MIT范围")
        key="DRAG" if mode=="DRAG" else "HOLD"
        if self.kp is None:
            self.kp,self.kd,self.target=hold_kp[:],hold_kd[:],measured[:]
        changed=key!=self.mode
        if changed:
            self.mode=key;self.elapsed=0.
            self.from_kp,self.from_kd,self.anchor=self.kp[:],self.kd[:],self.target[:]
        else:self.elapsed=min(TRANSITION_SECONDS,self.elapsed+min(dt,.02))
        u=self.elapsed/TRANSITION_SECONDS;s=u*u*(3.-2.*u)
        kp=[DRAG_KP]*6 if key=="DRAG" else hold_kp
        kd=[DRAG_KD]*6 if key=="DRAG" else hold_kd
        target=measured if key=="DRAG" else desired
        self.kp=[a+s*(b-a) for a,b in zip(self.from_kp,kp)]
        self.kd=[a+s*(b-a) for a,b in zip(self.from_kd,kd)]
        self.target=[a+s*(b-a) for a,b in zip(self.anchor,target)] if s<1. else target[:]
        return self.target[:],self.kp[:],self.kd[:]
