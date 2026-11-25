"""Recoverable arm motion guard; hardware validity is checked separately."""
GRAVITY_STARTUP_SECONDS=.5
from .experience_policy import ARM_SOFT_SPEED_RAD_S,ARM_IMMEDIATE_SPEED_RAD_S,ARM_SPEED_CONFIRM_S,ARM_SPEED_CONFIRM_FRAMES
class ArmMotionGuard:
    def __init__(self):
        self.active=False
        self.reason=None
        self.axes=[]
        self.count=0
        self.error_since=[None]*6
        self.sequences=None
        self.recovery=0
        self.high_since=[None]*6
        self.high_frames=[0]*6

    def update(self,positions,velocities,targets,sequences,now,neutral,diagnostic_origin=None,*,manual_drag=False):
        if self.sequences is not None and any(a<b for a,b in zip(sequences,self.sequences)):
            raise RuntimeError("手臂真实CAN反馈序号回退")
        reasons=[];axes=[]
        fresh=self.sequences is None or all(a>b for a,b in zip(sequences,self.sequences))
        for i in range(6):
            if fresh:
                if not manual_drag and abs(velocities[i])>ARM_SOFT_SPEED_RAD_S:
                    if self.high_since[i] is None:self.high_since[i]=now
                    self.high_frames[i]+=1
                else:self.high_since[i]=None;self.high_frames[i]=0
            confirmed=(self.high_since[i] is not None and self.high_frames[i]>=ARM_SPEED_CONFIRM_FRAMES
                       and now-self.high_since[i]>=ARM_SPEED_CONFIRM_S and fresh)
            if abs(velocities[i])>=ARM_IMMEDIATE_SPEED_RAD_S or confirmed:
                reasons.append(f"J{i+1}超速 {velocities[i]:+.3f}rad/s")
                axes.append(i+1)
            error=abs(positions[i]-targets[i])
            # Drag mode follows measured position and has no tracking task.
            # Keep the hard speed gate, but do not turn deliberate hand motion
            # into a stiff servo hold via teleop-only soft/tracking thresholds.
            self.error_since[i]=(self.error_since[i] if self.error_since[i] is not None else now) if not manual_drag and error>.12 else None
            if self.error_since[i] is not None and now-self.error_since[i]>=.2:
                reasons.append(f"J{i+1}跟踪误差持续超限")
                axes.append(i+1)
            if diagnostic_origin is not None and abs(positions[i]-diagnostic_origin[i])>.13962634:
                reasons.append(f"J{i+1}超出诊断范围")
                axes.append(i+1)
        if reasons:
            if not self.active:self.count+=1
            self.active=True;self.reason=" / ".join(reasons);self.axes=sorted(set(axes));self.recovery=0
        if fresh:
            self.sequences=list(sequences)
            settled=all(abs(v)<.4 for v in velocities) and all(abs(q-t)<.08 for q,t in zip(positions,targets))
            self.recovery=self.recovery+1 if self.active and not reasons and settled else 0
        if self.active and self.recovery>=2 and neutral:
            self.active=False;self.reason=None;self.axes=[]
        return self.active


def startup_scale(elapsed,duration=GRAVITY_STARTUP_SECONDS):
    x=max(0.,min(1.,elapsed/duration))
    return x*x*(3.-2.*x)
