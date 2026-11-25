"""Bounded panel gravity assist, using the official reBot dynamics model."""

import math
import threading
import time

import numpy as np
from motorbridge import Mode
from reBotArm_control_py.controllers import GravityCompensation

from .feedback_state import MAX_RX_AGE_MS, snapshot_motor, times_from_rx

RATE_HZ = 100
PERIOD = 1 / RATE_HZ
TORQUE_LIMITS = np.array([28., 28., 28., 10., 10., 10.])


def _smoothstep(u):
    return u * u * (3.0 - 2.0 * u)


class GravityBlend:
    """Follow the measured pose, easing in gravity and easing out stiffness."""

    def __init__(self, duration, filter_tau=.03):
        if not math.isfinite(duration) or duration <= 0 or not math.isfinite(filter_tau) or filter_tau <= 0:
            raise ValueError("重力渐入时间和滤波时间必须为有限正数")
        self.duration, self.filter_tau = duration, filter_tau
        self.time = None
        self.elapsed = 0.
        self.anchor = self.filtered = None

    def step(self, now, dt, q, model, hold_kp, hold_kd, compliant_kp, compliant_kd):
        if self.time is None:
            self.anchor, self.filtered = q.copy(), model.copy()
        else:
            # Do not jump the transition or the filter forward after a late tick.
            h = max(0., min(float(dt), now - self.time, 2 * PERIOD))
            self.elapsed += h
            self.filtered += (-math.expm1(-h / self.filter_tau)) * (model - self.filtered)
        self.time = now
        blend = _smoothstep(min(1., self.elapsed / self.duration))
        return (blend, self.anchor + blend * (q - self.anchor),
                hold_kp + blend * (compliant_kp - hold_kp),
                hold_kd + blend * (compliant_kd - hold_kd),
                self.filtered * blend)


class PanelGravityController(GravityCompensation):
    """Own an explicit gravity-only tick; never use the lossy group TX helper."""

    def __init__(self, robot, *, rx_provider, check_start=lambda: None, **kwargs):
        super().__init__(robot, **kwargs)
        if not callable(rx_provider):
            raise ValueError("重力补偿需要真实 CAN 接收时间戳")
        self.rx_provider, self.check_start = rx_provider, check_start
        self.blend = GravityBlend(self._transition_duration)
        self._halt = threading.Event()
        self._energized = False
        self._enabled_at = None
        self._last_tick = None
        self._read_axis = 0
        self._fault = None
        self._frame = {}
        self.latest_states = self.last_mit = None

    def start(self):
        robot = self.rebotarm
        if self._running or self._halt.is_set():
            raise RuntimeError("重力会话不能重复启动")
        self._resolve_params(robot.arm.num_joints)
        if (robot.arm.num_joints != 6 or len(robot.joint_names) != 7
                or not np.isin(self._joint_direction, (-1., 1.)).all()
                or not np.isfinite(self._tau_scale).all() or np.any(self._tau_scale < 0.)):
            raise ValueError("重力模型轴数、方向或倍率无效")
        for index, name in enumerate(robot.joint_names):
            self.check_start()
            try:
                robot._motor_map[name].ensure_mode(Mode.MIT, 1000)
            except Exception as exc:
                raise RuntimeError(f"J{index+1} 重力启用前 MIT 模式确认失败: {exc}") from exc
        baseline = [item["sequence"] for item in self.rx_provider()]
        self.check_start()
        for name in robot.joint_names:
            robot._motor_map[name].request_feedback()
        deadline = time.monotonic() + MAX_RX_AGE_MS / 1000
        while True:
            self.check_start()
            robot._ctrl_map["damiao"].poll_feedback_once()
            rx = self.rx_provider()
            if len(rx) == len(baseline) == 7 and all(r["sequence"] > seq for r, seq in zip(rx, baseline)):
                states, _ = self._feedback(robot, allowed_status=(0,), rx=rx)
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("MIT 模式确认后未收到七轴新失能反馈")
            time.sleep(.005)
        self._model(np.array([state.pos for state in states[:6]]))
        # Validate static gains before enabling rather than discovering bad config in a tick.
        for gains, maximum in ((self._hold_kp, 500.), (self._hold_kd, 5.),
                               (self._compliant_kp, 500.), (self._compliant_kd, 5.),
                               (robot.gripper._mit_kp, 500.), (robot.gripper._mit_kd, 5.)):
            if not np.isfinite(gains).all() or np.any(gains < 0.):
                raise ValueError("重力控制增益必须为有限非负数")
            np.clip(gains, 0., maximum, out=gains)
        for index, name in enumerate(robot.joint_names):
            self.check_start()
            self._energized = True  # An exception may occur after the command went out.
            try:
                robot._motor_map[name].enable()
            except Exception as exc:
                raise RuntimeError(f"J{index+1} 重力使能发送失败: {exc}") from exc
        self.check_start()
        self._enabled_at = time.monotonic()
        self._running = True
        robot.start_control_loop(self._loop_cb, rate=RATE_HZ)

    def _feedback(self, robot, allowed_status, rx=None):
        rx = self.rx_provider() if rx is None else rx
        times = times_from_rx(rx, time.monotonic())
        if len(times) != 7:
            raise RuntimeError("重力控制的 CAN 反馈轴数不符")
        states = []
        for i, name in enumerate(robot.joint_names):
            if times[i] is None:
                raise RuntimeError(f"J{i+1}真实CAN反馈超过{MAX_RX_AGE_MS}ms未更新或无效")
            raw = robot._motor_map[name].get_state()
            values = snapshot_motor(raw)
            if values is None or abs(values["pos"]) > 12.5:
                raise RuntimeError(f"J{i+1} 重力反馈包含无效数值或位置超出协议范围")
            if raw.status_code not in allowed_status:
                raise RuntimeError(f"J{i+1} 重力驱动状态异常 {raw.status_code}")
            if max(raw.t_mos, raw.t_rotor) >= 65:
                raise RuntimeError(f"J{i+1} 重力过温")
            states.append(raw)
        return tuple(states), rx

    def _model(self, q):
        model = np.asarray(self._gravity_comp_torque(q), dtype=float)
        if model.shape != TORQUE_LIMITS.shape:
            raise RuntimeError("重力模型输出轴数不符")
        invalid = np.flatnonzero(~np.isfinite(model) | (np.abs(model) > TORQUE_LIMITS))
        if invalid.size:
            i = int(invalid[0])
            raise RuntimeError(f"J{i+1} 模型力矩超出电机协议量程或无效: {model[i]:.3f} N·m")
        return model

    def _send(self, robot, command, *, unloading=False):
        fields = ("position", "velocity", "kp", "kd", "tau")
        for i, name in enumerate(robot.joint_names):
            if self._halt.is_set() and not unloading:
                return False
            try:
                robot._motor_map[name].send_mit(*(float(command[field][i]) for field in fields))
            except Exception as exc:
                raise RuntimeError(f"J{i+1} 重力 MIT发送失败: {exc}") from exc
        return True

    def _tick(self, robot, dt):
        began = time.monotonic()
        if self._halt.is_set():
            return
        cycle = float(dt) if self._last_tick is None else began - self._last_tick
        self._last_tick = began
        # Immediately reject real faults. A fresh DISABLED frame may precede
        # the first ENABLED frame; allow it only inside the existing RX window.
        initializing = self._enabled_at is not None and began - self._enabled_at < MAX_RX_AGE_MS / 1000
        states, rx = self._feedback(robot, (0, 1) if initializing else (1,))
        q = np.array([s.pos for s in states])
        model = self._model(q[:6])
        blend, target, kp, kd, tau = self.blend.step(
            began, cycle, q[:6], model, self._hold_kp, self._hold_kd, self._compliant_kp, self._compliant_kd)
        cmd = {"position": [*target, q[6]], "velocity": [0.] * 7,
               "kp": [*kp, float(robot.gripper._mit_kp[0])],
               "kd": [*kd, float(robot.gripper._mit_kd[0])], "tau": [*tau, 0.]}
        if not self._send(robot, cmd):
            return
        # Some drives report much less often than the command rate. Query one
        # axis per tick; never poll or wait synchronously in this callback.
        axis = self._read_axis
        try:
            robot._motor_map[robot.joint_names[axis]].request_feedback()
        except Exception as exc:
            raise RuntimeError(f"J{axis+1} 重力反馈请求发送失败: {exc}") from exc
        self._read_axis = (axis + 1) % 7
        completed = time.monotonic()
        self._last_tau_g = tau
        self.last_mit, self.latest_states = cmd, states
        self._frame = {"blend": blend, "rate_hz": RATE_HZ, "cycle_ms": cycle*1000,
                       "execution_ms": (completed-began)*1000, "completed_at": completed,
                       "rx": rx, "model_nm": model.tolist(), "sent_nm": tau.tolist(),
                       "q_rad": q[:6].tolist(), "velocity_rad_s": [s.vel for s in states[:6]],
                       "driver_feedback_nm": [s.torq for s in states[:6]]}

    def _loop_cb(self, robot, dt):
        try:
            self._tick(robot, dt)
        except Exception as exc:
            self._fault = str(exc)
            # Returning lets the owning panel perform shutdown without an
            # uncaught thread exception; no more commands follow this failure.
            self._halt.set()

    def snapshot(self):
        frame = dict(self._frame)
        completed = frame.pop("completed_at", self._enabled_at)
        return {**frame, "fault": self._fault,
                "age_ms": None if completed is None else (time.monotonic()-completed)*1000}

    def close(self, gentle=True):
        robot = self.rebotarm
        self._halt.set()
        try:
            robot.stop_control_loop()
            thread = getattr(robot, "_ctrl_thread", None)
            if thread is not None and thread.is_alive():
                raise RuntimeError("重力控制线程未退出，跳过平滑卸力")
            last = self.last_mit
            if gentle and self._energized and self._fault is None and last is not None:
                for tick in range(26):
                    states, _ = self._feedback(robot, (1,))
                    if any(abs(s.vel) > (2. if i < 3 else 3.) for i, s in enumerate(states)):
                        raise RuntimeError("卸力中检测到超速，直接失能")
                    u = _smoothstep(tick / 25.)
                    fade = 1. - u
                    cmd = {**last, "position": [(1.-u)*p + u*s.pos for p, s in zip(last["position"], states)]}
                    for field in ("kp", "kd", "tau"):
                        cmd[field] = [fade * val for val in last[field]]
                    self._send(robot, cmd, unloading=True)
                    robot._motor_map[robot.joint_names[tick % 7]].request_feedback()
                    if tick != 25:
                        time.sleep(.02)
        finally:
            self._running = False
            if self._energized:
                failures = []
                for i, name in enumerate(robot.joint_names):
                    try:
                        robot._motor_map[name].disable()
                    except Exception as exc:
                        failures.append(f"J{i+1}: {exc}")
                self._energized = False
                if failures:
                    raise RuntimeError("重力失能发送失败 · " + "；".join(failures))
