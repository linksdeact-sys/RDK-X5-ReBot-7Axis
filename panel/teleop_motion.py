"""Trajectory references only. Position and damping loops run inside the motor."""

import math

from .gamepad_input import triggers_neutral

CONTROL_REVISION = "continuous-mit-v11"
CONTINUOUS_SPEED = (math.radians(24.0),)*6
CONTINUOUS_ACCEL = 0.6
CONTINUOUS_JERK = 6.0


def braking_margin(speed):
    """Conservative reference braking space, not a measured hardware distance."""
    return max(0.12, speed*speed/(2*CONTINUOUS_ACCEL)
               + abs(speed)*(CONTINUOUS_ACCEL/CONTINUOUS_JERK+0.025+0.02)+0.02)


def clamp(value, low, high):
    return min(high, max(low, value))


class SmoothDrive:
    SPEED = 0.05
    ACCEL = 0.20
    JERK = 2.0
    TRAVEL = math.radians(3.0)

    def __init__(self, positions, limits, selected_axis=0):
        if selected_axis is not None and (type(selected_axis) is not int or not 0 <= selected_axis < 6):
            raise ValueError("验证版仅允许选择J1～J6中的一个关节")
        if len(positions) != 7 or len(limits) != 6 or any(
                not math.isfinite(v) for v in positions) or any(
                not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi for lo, hi in limits):
            raise ValueError("关节位置或范围无效")
        self.continuous = selected_axis is None
        self.selected_axis = selected_axis
        self.origin = list(positions)
        self.target = list(positions)
        self.velocity = [0.0]*7
        self.acceleration = [0.0]*7
        self.limits = list(limits)
        self.need_rearm = True
        self.driving = False
        self.status = "等待LB释放并回中"
        self.wall = []
        self.timing_holds = 0
        self.filtered = [0.0]*7
        self.engaged = [False]*7
        self.tracking = []
        self.speed_holds = frozenset()
        self.pause_reason = None
        self.local_pause_event = 0

    def safety_hold(self, previous_target, reason, event_id):
        """Keep last safe references and gains; do not unload or chase input."""
        new_event=event_id!=self.local_pause_event
        self.local_pause_event=event_id
        self.target[:6]=previous_target[:6]
        self.velocity=[0.]*7
        self.acceleration=[0.]*7
        self.filtered=[0.]*7
        self.engaged=[False]*7
        self.need_rearm=True
        self.driving=False
        self.pause_reason=reason
        self.status=f"保护暂停·位置/重力保持 · {reason} · 释放LB并回中恢复"
        return new_event

    def pause(self, reason):
        self.velocity = [0.0]*7
        self.acceleration = [0.0]*7
        self.filtered = [0.0]*7
        self.need_rearm = True
        self.driving = False
        self.pause_reason = reason
        self.timing_holds += 1
        self.status = "短暂停顿·本地保持，释放LB并回中"

    def step(self, input_state, dt, measured=None, *, speed_holds=()):
        if not math.isfinite(dt) or dt <= 0 or dt > 0.25:
            raise RuntimeError(f"控制周期严重超时 dt={dt*1000:.1f}ms")
        raw = input_state["intent"]
        triggers = input_state.get("triggers")
        neutral = (all(abs(value) < 0.10 for value in raw)
                   and (triggers is None or triggers_neutral(triggers)))
        jitter_limit = 0.10 if self.continuous else 0.025
        if not input_state["connected"] or dt > jitter_limit:
            self.need_rearm = True
        elif neutral and not input_state["deadman"]:
            self.need_rearm = False
            self.pause_reason = None
        if dt > jitter_limit:
            # Freeze the reference; motor-side damping continues at full strength.
            self.pause(f"周期抖动 {dt*1000:.0f}ms")
            return
        self.driving = input_state["connected"] and input_state["deadman"] and not self.need_rearm
        self.status = (("连续遥操中" if self.continuous else "单轴验证中") if self.driving else "手柄断连·本地保持" if not input_state["connected"]
                       else "等待释放LB并回中" if self.need_rearm else "平滑减速 / 本地位置保持")
        if self.continuous:
            self._continuous_step(raw, min(dt, 0.02), measured, frozenset(speed_holds))
            if self.speed_holds:
                joints = "/".join(f"J{i+1}" for i in sorted(self.speed_holds))
                self.status += f" · {joints} 已保持，速度恢复后回中解除"
            return
        i = self.selected_axis
        value = raw[i] if self.driving else 0.0
        magnitude = clamp((abs(value)-0.12)/0.88, 0.0, 1.0)
        wish = math.copysign(0.7*magnitude+0.3*magnitude**3, value)*self.SPEED
        lo = max(self.limits[i][0], self.origin[i]-self.TRAVEL)
        hi = min(self.limits[i][1], self.origin[i]+self.TRAVEL)
        # A start slightly beyond a URDF boundary may only move back inward.
        lo, hi = min(lo, self.origin[i]), max(hi, self.origin[i])
        distance = hi-self.target[i] if wish > 0 else self.target[i]-lo
        wish *= clamp(distance/0.01, 0.0, 1.0)
        self.wall = [i+1] if distance <= 1e-6 else []
        desired_acc = clamp((wish-self.velocity[i])/0.05, -self.ACCEL, self.ACCEL)
        self.acceleration[i] += clamp(desired_acc-self.acceleration[i], -self.JERK*dt, self.JERK*dt)
        previous_v = self.velocity[i]
        self.velocity[i] = clamp(previous_v+self.acceleration[i]*dt, -self.SPEED, self.SPEED)
        if abs(wish) < 1e-9 and (previous_v*self.velocity[i] <= 0 or
                                (abs(self.velocity[i]) < 1e-4 and abs(self.acceleration[i]) < 0.01)):
            self.velocity[i] = self.acceleration[i] = 0.0
        candidate = self.target[i]+self.velocity[i]*dt
        self.target[i] = clamp(candidate, lo, hi)
        if candidate != self.target[i]:
            self.velocity[i] = self.acceleration[i] = 0.0

    def _continuous_step(self, raw, dt, measured, speed_holds):
        if measured is None or len(measured) != 7 or any(not math.isfinite(v) for v in measured):
            raise ValueError("连续遥操需要七轴实际位置，禁止积累开环目标")
        self.wall, self.tracking = [], []
        for i in range(6):
            lo, hi = self.limits[i]
            # Never expand the actual URDF limits. A pose outside a boundary
            # can only move inward; do not snap its reference to the boundary.
            lo, hi = min(lo, self.origin[i]), max(hi, self.origin[i])
            if i in speed_holds:
                if i not in self.speed_holds:
                    self.target[i] = clamp(measured[i], lo, hi)
                self.velocity[i] = self.acceleration[i] = self.filtered[i] = 0.0
                self.engaged[i] = False
                continue
            value = raw[i] if self.driving else 0.0
            if abs(value) > 0.12:
                self.engaged[i] = True
            elif abs(value) < 0.08:
                self.engaged[i] = False
            magnitude = clamp((abs(value)-0.08)/0.92, 0.0, 1.0) if self.engaged[i] else 0.0
            mapped = math.copysign(0.7*magnitude+0.3*magnitude**3, value)
            self.filtered[i] += dt/(0.025+dt)*(mapped-self.filtered[i])
            speed = CONTINUOUS_SPEED[i]
            wish = self.filtered[i]*speed
            distance = hi-self.target[i] if wish >= 0 else self.target[i]-lo
            wall_width = braking_margin(speed)
            wish *= clamp(distance/wall_width, 0.0, 1.0)
            if distance < wall_width and abs(self.filtered[i]) > 0.01:
                self.wall.append(i+1)
            max_acc, max_jerk = CONTINUOUS_ACCEL, CONTINUOUS_JERK
            desired_acc = clamp((wish-self.velocity[i])/0.05, -max_acc, max_acc)
            self.acceleration[i] += clamp(desired_acc-self.acceleration[i], -max_jerk*dt, max_jerk*dt)
            previous = self.velocity[i]
            self.velocity[i] = clamp(previous+self.acceleration[i]*dt, -speed, speed)
            if abs(wish) < 1e-4 and (previous*self.velocity[i] <= 0 or
                    (abs(self.velocity[i]) < 1e-4 and abs(self.acceleration[i]) < 0.01)):
                self.velocity[i] = self.acceleration[i] = self.filtered[i] = 0.0
            candidate = self.target[i]+self.velocity[i]*dt
            # This is following-error anti-windup, NOT a travel limit around
            # startup. It moves along with the actual joint for unlimited travel.
            lead = 0.06
            if abs(candidate-measured[i]) > lead and (candidate-measured[i])*self.velocity[i] > 0:
                self.tracking.append(i+1)
                if self.velocity[i] > 0:
                    candidate = min(candidate, max(self.target[i], measured[i]+lead))
                else:
                    candidate = max(candidate, min(self.target[i], measured[i]-lead))
                self.velocity[i] = self.acceleration[i] = 0.0
            self.target[i] = clamp(candidate, lo, hi)
            if self.target[i] != candidate:
                self.velocity[i] = self.acceleration[i] = 0.0
        self.speed_holds = speed_holds

    def snapshot(self):
        return {"status": self.status, "driving": self.driving, "need_rearm": self.need_rearm,
                "selected_axis": None if self.continuous else self.selected_axis+1,
                "mode": "continuous" if self.continuous else "single-axis", "target_rad": self.target[:],
                "velocity_rad_s": self.velocity[:], "boundary_axes": self.wall[:],
                "tracking_axes": self.tracking[:], "timing_holds": self.timing_holds,
                "speed_hold_axes": [i+1 for i in sorted(self.speed_holds)],
                "pause_reason": self.pause_reason, "gripper_ready": False}
