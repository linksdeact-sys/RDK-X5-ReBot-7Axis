"""LAN commissioning panel for the official reBot MotorBridge installation."""

import argparse
import collections
import fcntl
import json
import math
import os
import re
import secrets
import signal
import sys
import threading
import time
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit



ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))
from panel.gamepad_input import GamepadReader, SimGamepad, triggers_neutral
from panel.gripper_teleop import GripperTeleop, ENVELOPE_DEG, range_from_points, firmware_timeout_ms, gripper_phase
from panel.usb_reconnect import usb_signature, transport_error, RestartForUsb, RECONNECT_EXIT, supervise
from panel.teleop_motion import CONTROL_REVISION, CONTINUOUS_SPEED
from panel.control_health import watchdog_action, SOFT_PAUSE_MS, HARD_TIMEOUT_MS
from panel.urdf_view import load_view
from panel.camera_preview import CameraPreview
from panel.feedback_state import MAX_RX_AGE_MS, is_fresh, snapshot_motor, times_from_rx
from panel.gripper_watchdog import ensure_gripper_watchdog
from panel.gripper_teleop import MAX_SPEED_RAD_S as GRIPPER_MAX_SPEED_RAD_S
from panel.gripper_teleop import HARD_SPEED_RAD_S as GRIPPER_HARD_SPEED_RAD_S
from panel.gripper_teleop import SOFT_SPEED_RAD_S as GRIPPER_SOFT_SPEED_RAD_S
from panel.gripper_force import ForceCalibration, profile_torque, identity
from panel.gripper_hold import CONTROL_NAME, HOLD_TORQUE_MAX
from panel.gripper_torque import TORQUE_LIMIT_NM
from panel.gripper_contact_motion import policy as contact_motion_policy
from panel.teaching import TeachingLibrary, REVISION as TEACH_REVISION
from panel.teaching_panel import TeachingPanelMixin
from panel.communication_recovery import CommunicationRecoveryMixin
from panel.arm_motion_guard import ArmMotionGuard,startup_scale,GRAVITY_STARTUP_SECONDS
from panel.experience_policy import snapshot as experience_snapshot
LABELS = ["基座旋转", "大臂", "小臂", "腕部俯仰", "腕部滚转", "末端滚转", "夹爪"]
DIRECTIONS = [
    ["俯视逆时针", "俯视顺时针"], ["抬起", "放下"], ["抬起", "放下"],
    ["抬头", "低头"], ["沿输出轴看顺时针", "沿输出轴看逆时针"],
    ["沿输出轴看顺时针", "沿输出轴看逆时针"], ["打开", "闭合"],
]
GRAVITY_PROFILES = {
    "official": {"label": "官方倍率 · 全轴 100%", "scales": [1.0] * 6},
    "measured": {"label": "J4 50% · 其余 100%", "scales": [1.0, 1.0, 1.0, 0.5, 1.0, 1.0]},
}
for percent in range(30, 100, 5):
    if percent != 50:
        GRAVITY_PROFILES[f"j4_{percent}"] = {
            "label": f"J4 {percent}% · 其余 100%",
            "scales": [1.0, 1.0, 1.0, percent / 100.0, 1.0, 1.0],
        }


class Hardware:
    def __init__(self):
        import yaml
        from motorbridge import Controller

        config = yaml.safe_load((ROOT.parent / "config/rebotarm_dm.yaml").read_text())
        self.config = config
        raw_scale = config.get("gravity_compensation", {}).get("tau_scale", 1.0)
        self.gravity_scale = [float(raw_scale)] * 6 if isinstance(raw_scale, (int, float)) else [float(value) for value in raw_scale]
        if len(self.gravity_scale) == 1:
            self.gravity_scale *= 6
        if len(self.gravity_scale) != 6 or not all(math.isfinite(value) for value in self.gravity_scale):
            raise ValueError("gravity_compensation.tau_scale requires six finite values")
        self.controller = Controller.from_dm_device(config["dm_device_type"], str(config["dm_channel"]))
        try:
            from panel.rx_freshness import RxFreshness
            self.rx = RxFreshness()
        except Exception:
            self.controller.close()
            raise
        self.motors = []
        for joint in config["joints"]:
            self.motors.append(self.controller.add_damiao_motor(
                joint["motor_id"], joint["feedback_id"], joint["model"]
            ))
        # Opening a fresh USB handle never resumes an old enabled state.
        for motor in self.motors:
            motor.disable()
        self.gravity([0.0] * 6)
        time.sleep(0.3)

    def poll(self, indices):
        for index in indices:
            self.motors[index].request_feedback()
        time.sleep(0.008)
        self.controller.poll_feedback_once()
        return {index: self.motors[index].get_state() for index in indices}

    def probe(self, index):
        self.motors[index].damiao_get_param_u32(10, 100)

    def gripper_watchdog(self):
        def require_idle():
            states = self.poll(range(7))
            received = times_from_rx(self.rx_meta(), time.monotonic())
            if len(received) != 7 or None in received or any(
                    index not in states or states[index] is None
                    or states[index].status_code != 0
                    or not math.isfinite(states[index].vel) or abs(states[index].vel) > .05
                    for index in range(7)):
                raise RuntimeError("J7 自动配置需七轴新鲜反馈、静止且失能；状态不满足，禁止使能")
        self.last_gripper_watchdog = ensure_gripper_watchdog(self.motors[6], require_idle)
        return self.last_gripper_watchdog["timeout_ms"]

    def enable(self, index):
        from motorbridge import Mode

        self.motors[index].ensure_mode(Mode.MIT, 1000)
        self.motors[index].enable()

    def command(self, index, position, kp, kd, torque=0.0):
        self.motors[index].send_mit(position, 0.0, kp, kd, torque)

    def clear_communication_error(self, index):
        self.motors[index].clear_error()

    def gravity(self, angles):
        import numpy as np
        from reBotArm_control_py.dynamics import load_dynamics_model, compute_generalized_gravity

        if not hasattr(self, "gravity_model"):
            self.gravity_model = load_dynamics_model()
            self.gravity_data = self.gravity_model.createData()
        torque = compute_generalized_gravity(self.gravity_model, np.asarray(angles), self.gravity_data)[:6]
        return (torque * np.asarray(self.gravity_scale)).tolist()

    def rx_meta(self):
        return self.rx.snapshot()

    def begin_gravity(self, scales, gamepad=None, gripper_limits=None, selected_axis=0, *, check_start=None,
                      hold_torque=None, calibration=None, torque_limited=False, teaching=None):
        from panel.gravity_mode import OfficialGravitySession

        self.gravity_session = OfficialGravitySession(self.controller, self.motors, self.config, scales,
                                                      gamepad, gripper_limits, selected_axis, self.rx_meta, check_start,
                                                      hold_torque=hold_torque,calibration=calibration,torque_limited=torque_limited,teaching=teaching)
        self.gravity_session.start()

    def observe_gravity(self):
        # Teleop already reads SDK state in the servo thread. The monitor must
        # not compete for per-motor SDK handle locks a second time at 50 Hz.
        published = getattr(self.gravity_session.control, "latest_states", None)
        states = dict(enumerate(published)) if published is not None else {
            index: motor.get_state() for index, motor in enumerate(self.motors)}
        return (states,
                self.gravity_session.torque, self.gravity_session.running)

    def teleop_status(self):
        control = self.gravity_session.control
        return control.snapshot(), control.stop_requested

    def gravity_status(self):
        return self.gravity_session.status()

    def take_control_records(self):
        return self.gravity_session.control.drain_records()

    def control_watchdog(self):
        return self.gravity_session.control.watchdog_status()

    def pause_control(self):
        self.gravity_session.control.request_timing_pause()

    def end_gravity(self, gentle=True):
        session = getattr(self, "gravity_session", None)
        if session is not None:
            try:
                session.close(gentle)
            finally:
                self.gravity_session = None

    def disable(self, index):
        self.motors[index].disable()

    def close(self):
        self.controller.close()


class Simulation:
    def __init__(self):
        from types import SimpleNamespace

        self.states = [SimpleNamespace(pos=0.0, vel=0.0, torq=0.0, status_code=0,
                                       t_mos=24.0, t_rotor=22.0) for _ in range(7)]
        self.states[6].pos = math.radians(145.3)
        self.gamepad = None
        self.teacher = None
        self.teach_compliance = None
        self.sequence = 0

    def poll(self, indices):
        return {index: self.states[index] for index in indices}

    def probe(self, index):
        pass

    def gripper_watchdog(self):
        return firmware_timeout_ms(4000)

    def enable(self, index):
        self.states[index].status_code = 1

    def command(self, index, position, kp, kd, torque=0.0):
        state = self.states[index]
        delta = (position - state.pos) * 0.28
        state.pos += delta
        state.vel = delta / 0.01
        state.torq = kp * (position - state.pos) + torque

    def gravity(self, angles):
        return [0.0, -1.27, -7.20, -1.98, 0.0, 0.0]

    def rx_meta(self):
        self.sequence += 1
        bad = self.gamepad.snapshot().get("rx_fault_axis") if self.gamepad else None
        return [{"age_ms": 200.0 if i+1 == bad else 0.0, "sequence": self.sequence} for i in range(7)]

    def begin_gravity(self, scales, gamepad=None, gripper_limits=None, selected_axis=0, *, check_start=None,
                      hold_torque=None, calibration=None, torque_limited=False, teaching=None):
        self.gravity_scales = scales
        self.teacher=None
        self.gravity_started = time.monotonic()
        for index in range(7):
            self.enable(index)
        self.gravity_running = True
        self.gamepad = gamepad
        self.motion = None
        self.gripper = GripperTeleop(gripper_limits,hold_torque=hold_torque,calibration=calibration,torque_limited=torque_limited)
        self.calibration = calibration
        self.gripper_command = 0.
        if gamepad:
            from panel.teleop_motion import SmoothDrive
            self.motion = SmoothDrive([s.pos for s in self.states], [(-2.8, 2.8)]*6, selected_axis)
        self.records = []
        self.arm_guard=ArmMotionGuard()
        self.startup_elapsed=0.
        if teaching is not None:
            from panel.teaching import TeachingDirector
            import xml.etree.ElementTree as ET
            root=ET.parse(ROOT.parent/'urdf/DM/urdf/ReBot_Arm_DM.urdf').getroot()
            limits=[]
            for i in range(1,7):
                limit=root.find(f"joint[@name='joint{i}']/limit")
                limits.append((float(limit.attrib['lower']),float(limit.attrib['upper'])))
            self.teacher=TeachingDirector([s.pos for s in self.states],limits+[gripper_limits],teaching["signature"],teaching.get("initial_play"))
            from panel.teach_compliance import TeachCompliance
            self.teach_compliance=TeachCompliance()

    def observe_teaching(self):
        from panel.gripper_hold import GripperCommand
        pad=self.gamepad.snapshot();meta=self.rx_meta()
        if self.teacher.mode=="DRAG" and pad.get("teaching_q") is not None:
            for s,x in zip(self.states,pad["teaching_q"]):s.vel=(x-s.pos)/.02;s.pos=x
        q=[s.pos for s in self.states];v=[s.vel for s in self.states]
        self.arm_guard.update(q[:6],v[:6],self.motion.target[:6],
            [r['sequence'] for r in meta[:6]],time.monotonic(),True,manual_drag=self.teacher.mode=='DRAG')
        if self.arm_guard.active and self.teacher.mode!='HOLD':
            self.teacher.hold(self.motion.target,self.arm_guard.reason)
        target=self.teacher.step(q,v,.02,[r["sequence"] for r in meta],ready=self.startup_elapsed>=.5,grip_contact=self.gripper.torque_ff>0.)
        reference,kp,kd=self.teach_compliance.step(self.teacher.mode,target[:6],q[:6],.02,[120.,120.,120.,18.,18.,18.],[5.,5.,5.,2.,2.,2.])
        target[:6]=reference
        if self.teacher.mode=="DRAG":
            remaining=max(0.,self.gripper.torque_ff-.04)
            self.gripper.hold.cancel("手推示教")
            self.gripper._apply(GripperCommand(kd=.1,tau_ff=remaining,state="HAND_TEACH",status="J7手推示教 · 无主动夹紧力"),self.states[6].vel)
        else:
            error=target[6]-q[6]
            self.gripper.update(position=q[6],velocity=v[6],torque=self.states[6].torq,
                triggers={"rt":min(.4,max(0.,error/.05)) if error>.01 else 0.,"lt":min(.4,max(0.,-error/.05)) if error<-.01 else 0.},
                armed=self.teacher.mode=="REPLAY",hold_allowed=True,dt=.02,
                feedback_sequence=meta[6]["sequence"],feedback_age_ms=meta[6]["age_ms"],allow_grasp=not self.teacher.home_move)
            if self.gripper.speed_hold or self.gripper.command.state=="BLOCKED":self.teacher.hold(target,"J7保护暂停或受阻")
            for i in range(6):
                self.states[i].vel=(target[i]-q[i])/.02
                self.states[i].pos=target[i]
            for _ in range(20):
                s=self.states[6];c=self.gripper.command
                s.torq=c.estimate(s.vel);s.vel+=(s.torq-.65*s.vel)*.001/.025;s.pos+=s.vel*.001
        self.startup_elapsed+=.02
        self.motion.target=target;self.motion.velocity=[*self.teacher.velocity,0.]
        self.motion.status={"DRAG":"七轴手推示教","HOLD":"示教位置保持","REPLAY":"七轴低速回放"}[self.teacher.mode]
        self.motion.driving=self.teacher.mode=="REPLAY";self.motion.need_rearm=False
        return dict(enumerate(self.states)),[t*s*startup_scale(self.startup_elapsed) for t,s in zip(self.gravity([]),self.gravity_scales)],True

    def observe_gravity(self):
        if self.teacher:return self.observe_teaching()
        torque = [value * scale for value, scale in zip(self.gravity([]), self.gravity_scales)]
        if self.motion:
            torque=[t*startup_scale(self.startup_elapsed) for t in torque]
            pad = self.gamepad.snapshot()
            if pad["stop"] or (self.gripper.limits and pad.get("triggers") is None):
                pad = {**pad, "connected": False}
            arm_pad = {**pad, "deadman": False, "intent": [0.]*7} if self.calibration else pad
            neutral=(pad["connected"] and not pad["deadman"] and all(abs(v)<.08 for v in pad["intent"])
                     and triggers_neutral(pad.get("triggers")))
            meta=self.rx_meta()
            self.arm_guard.update([s.pos for s in self.states[:6]],[s.vel for s in self.states[:6]],
                self.motion.target[:6],[r["sequence"] for r in meta[:6]],time.monotonic(),neutral)
            if self.arm_guard.active or self.startup_elapsed<GRAVITY_STARTUP_SECONDS:arm_pad={**arm_pad,"connected":False}
            previous_target=self.motion.target[:]
            self.motion.step(arm_pad, 0.02, [s.pos for s in self.states])
            if self.startup_elapsed<GRAVITY_STARTUP_SECONDS:
                self.motion.status="启动重力补偿渐入中，请继续支撑并保持输入回中"
            j7 = self.states[6]
            if pad.get("gripper_velocity_rad_s") is not None:
                j7.vel=pad["gripper_velocity_rad_s"] # Simulation API only.
            j7_rx = self.rx_meta()[6]
            input_ok = pad["connected"] and not pad["stop"] and (not self.motion.need_rearm or self.arm_guard.active) and pad.get("rx_fault_axis") != 7
            self.gripper_command = self.gripper.update(position=j7.pos, velocity=j7.vel, torque=j7.torq,
                triggers=pad.get("triggers"), armed=input_ok and (pad["deadman"] if self.calibration else self.motion.driving),
                hold_allowed=input_ok, dt=.02,
                feedback_sequence=j7_rx["sequence"], feedback_age_ms=0. if pad.get("rx_fault_axis") == 7 else j7_rx["age_ms"])
            if self.arm_guard.active:
                self.motion.safety_hold(previous_target,self.arm_guard.reason,("arm",self.arm_guard.count))
            elif self.gripper.speed_hold:
                if self.motion.safety_hold(previous_target,self.gripper.pause_reason,self.gripper.pause_count):
                    self.records.append({"event":"local_pause","joint":7,"reason":self.gripper.pause_reason,
                                         "pause_trigger":self.gripper.pause_trigger,
                                         "preserve_arm_support":True,"at":time.monotonic()})
            if self.gripper.limits:
                for _ in range(20):
                    cmd = self.gripper.command
                    j7.torq = cmd.estimate(j7.vel)
                    j7.vel += (j7.torq - .65*j7.vel)*.001/.025
                    j7.pos += j7.vel*.001
                    # Test-only compliant object. Never present in real hardware.
                    contact = pad.get("gripper_object_rad")
                    if contact is not None and j7.pos > contact:
                        j7.pos, j7.vel = contact, 0.
            kp, kd = ([120, 120, 120, 18, 18, 18], [5, 5, 5, 2, 2, 2]) if self.motion.continuous else ([20, 25, 25, 4, 4, 4], [1.5, 2, 2, 1, 0.7, 0.7])
            for _ in range(20):
                for i in range(6):
                    state = self.states[i]
                    servo = kp[i]*(self.motion.target[i]-state.pos) + kd[i]*(self.motion.velocity[i]-state.vel)
                    inertia = [0.08, 0.15, 0.08, 0.015, 0.02, 0.015][i]
                    state.vel += (servo-0.03*state.vel)/inertia*0.001
                    state.pos += state.vel*0.001
            velocity = self.motion.velocity[:]
            velocity[6] = self.gripper.velocity_ref
            self.records.append({"at": time.monotonic(), "q": [s.pos for s in self.states],
                                 "input": self.gamepad.snapshot(), "target": self.motion.target[:],
                                 "mit": {"position": self.motion.target[:], "velocity": velocity,
                                         "kp": kp+[0.0], "kd": kd+[self.gripper.mit_kd if self.gripper.limits else .2],
                                         "tau": torque+[self.gripper.command.tau_ff]}, "rx": self.rx_meta()})
        if self.motion:
            self.startup_elapsed+=.02
        for index in range(6):
            self.states[index].torq = torque[index]
        return self.poll(list(range(7))), torque, self.gravity_running

    def teleop_status(self):
        motion = self.motion.snapshot()
        motion["target_rad"][6] = self.states[6].pos
        return {**motion, "revision": CONTROL_REVISION, "last_tick": time.monotonic(),
                "teaching":self.teacher.snapshot() if self.teacher else None,
                "arm_paused":self.arm_guard.active,"arm_pause_reason":self.arm_guard.reason,
                "gripper_ready": self.gripper.limits is not None, "gripper_status": self.gripper.status,
                "gripper_command_nm": self.gripper.command.tau_ff, "gripper_velocity_ref_rad_s": self.gripper.velocity_ref,
                "gripper_hold_state": self.gripper.command.state,
                "gripper_paused":self.gripper.speed_hold,
                "gripper_pause_reason":self.gripper.pause_reason,
                "gripper_pause_count":self.gripper.pause_count,
                "gripper_pause_trigger":self.gripper.pause_trigger,
                "gripper_contact_motion":self.gripper.contact_motion_snapshot(self.states[6].pos),
                "gripper_close_locked": self.gripper.hold.close_locked,
                "gripper_torque_limit_nm": getattr(self.gripper.hold,"torque_limit",None),
                "gripper_force_target_n": 3. if self.gripper.hold.torque else None,
                "gripper_measured_force_n": None,
                "gripper_calibration": self.calibration.snapshot() if self.calibration else None,
                "gripper_estimated_torque_nm": self.gripper_command,
                "gripper_control": CONTROL_NAME}, self.gamepad.snapshot()["stop"]

    def gravity_status(self):
        return None  # Simulation does not claim real CAN timing.

    def take_control_records(self):
        rows, self.records = self.records, []
        return rows

    def control_watchdog(self):
        return {"age_ms": 0., "phase": "simulated"}

    def pause_control(self):
        self.motion.pause("仿真控制完成心跳短暂超时")

    def end_gravity(self, gentle=True):
        if gentle:
            time.sleep(0.52)
        self.gravity_running = False
        for index in range(7):
            self.disable(index)

    def disable(self, index):
        self.states[index].status_code = 0
        self.states[index].vel = 0.0
        self.states[index].torq = 0.0

    def close(self):
        pass

    def clear_communication_error(self, index):
        if self.states[index].status_code==13:self.states[index].status_code=0


class Panel(TeachingPanelMixin,CommunicationRecoveryMixin):
    def __init__(self, simulate=False, standby=False):
        self.simulate = simulate
        self.standby = standby
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.closing = threading.Event()
        self.pending = None
        self.active = None
        self.last_result = None
        self.error = None
        self.events = collections.deque(maxlen=30)
        self.feedback_at = [None] * 7
        self.last_poll = 0.0
        self.states = [None] * 7
        self.connection_ready = False
        self.connection_state = "STANDBY" if standby else "CONNECTING"
        self.reconnect_requested = threading.Event()
        self.usb_generation = ()
        self.reconnect_count = int(os.environ.get("REBOT_RECONNECT_COUNT", "0"))
        self.connection_session = secrets.token_hex(8)
        self.restart_at = 0.0
        self.gamepad = SimGamepad() if simulate else GamepadReader()
        self.gripper_file = ROOT / ("gripper_limits.sim.json" if simulate else "gripper_limits.json")
        self.gripper_points = {}
        if self.gripper_file.exists():
            try:
                stored = json.loads(self.gripper_file.read_text())
                if range_from_points(stored, complete=False) is None:
                    raise ValueError("文件格式或软件范围不符")
                self.gripper_points = {key: stored[key] for key in ("open", "closed") if stored.get(key) is not None}
            except (OSError, ValueError) as exc:
                self.log(f"J7 端点记录不可用，夹爪保持锁定 · {exc}")
        self.seen = collections.OrderedDict()
        self.force_file = ROOT / ("gripper_hold.sim.json" if simulate else "gripper_hold.json")
        self.force_profile = None
        self.gripper_calibration = None
        if self.force_file.exists():
            try:
                self.force_profile = json.loads(self.force_file.read_text(encoding="utf-8"))
                if profile_torque(self.force_profile, self.gripper_points) is None:
                    self.log("J7 保持力标定无效或端点已变化，只保留速度控制，需单独重做测力")
            except (OSError, ValueError, TypeError):
                self.force_profile = None
                self.log("J7 保持力标定文件不可用，未启用保持")
        self.lease = 0.0
        self.init_recovery()
        self.init_teaching(ROOT)
        self.direction_file = ROOT / ("directions.sim.json" if simulate else "directions.json")
        self.directions = json.loads(self.direction_file.read_text()) if self.direction_file.exists() else {}
        self.worker = None
        if not standby:
            self.worker = threading.Thread(target=self.run, daemon=True)
            self.worker.start()
        else:
            self.log("网页已启动，硬件待手动连接；尚未打开USB2CAN或发送CAN指令")

    def connect_hardware(self, body):
        if body.get("supported") is not True:
            raise ValueError("请先托稳机械臂并勾选确认；连接时会发送七轴失能")
        with self.lock:
            if self.closing.is_set():
                raise RuntimeError("服务正在退出")
            if not self.standby:
                return {"accepted": True, "state": self.connection_state}
            self.standby = False
            self.connection_state = "CONNECTING"
            self.error = None
            self.worker = threading.Thread(target=self.run, daemon=True)
            self.worker.start()
        self.log("用户确认已托稳，开始连接硬件并核验七轴失能；不会自动启动运动")
        return {"accepted": True, "state": "CONNECTING"}

    def connection_lost(self, message):
        with self.lock:
            self.connection_ready = False
            self.connection_state = "WAIT_USB"
            self.error = message
            self.pending = None
            self.feedback_at = [None] * 7
            self.states = [None]*7
            self.last_poll = 0.0
            self.stop.set()
            self.restart_at = time.monotonic()+1.0
            self.reconnect_requested.set()
        self.log("USB连接中断，旧任务已取消，等待自动重连 · " + message)
        print("USB recovery requested · " + message, flush=True)

    def service_usb(self):
        if self.simulate or getattr(self, "standby", False) or self.closing.is_set():
            return
        signature = usb_signature()
        if self.usb_generation and signature != self.usb_generation and not self.reconnect_requested.is_set():
            self.connection_lost("检测到USB拔出或设备重新枚举")
        if self.reconnect_requested.is_set():
            with self.lock:
                self.connection_state = "RECONNECTING" if signature else "WAIT_USB"
            if signature and time.monotonic() >= self.restart_at:
                raise RestartForUsb()

    def verify_disabled(self, hardware):
        # Clear all cached state, send DISABLE, then require new actual feedback.
        # No old position/velocity/controller job survives this handshake.
        with self.lock:
            self.connection_state = "VERIFY_DISABLED"
            self.connection_ready = False
        baseline = [item["sequence"] for item in hardware.rx_meta()]
        previous = list(baseline)
        for index in range(7):
            hardware.disable(index)
        successes = 0
        while not self.closing.is_set() and not self.reconnect_requested.is_set():
            self.service_recovery(hardware)
            metadata = self.refresh(hardware, list(range(7)))
            with self.lock:
                if self.closing.is_set() or self.reconnect_requested.is_set():
                    return False
                safe = all(state is not None and state["status_code"] == 0 and
                             metadata[i]["sequence"] > previous[i] and self.feedback_at[i] is not None
                           for i, state in enumerate(self.states))
                successes = successes+1 if safe else 0
                if safe:
                    previous = [item["sequence"] for item in metadata]
                if successes >= 2:
                    self.connection_ready = True
                    self.connection_state = "READY"
                    self.error = None
            if successes >= 2:
                self.log("七轴新反馈已确认失能，通信已恢复，等待手动启动")
                return True
            # A driver may have remained enabled when the cable was removed.
            for index in range(7):
                if self.states[index] and self.states[index]["status_code"] == 1:
                    hardware.disable(index)
            self.closing.wait(0.10)
        return False

    def log(self, message):
        with self.lock:
            self.events.appendleft({"time": time.time(), "message": message})

    def refresh(self, hardware, indices):
        result = hardware.poll(indices)
        now = time.monotonic()
        metadata = hardware.rx_meta()
        if len(metadata) != len(self.states):
            raise RuntimeError("CAN 反馈轴数与配置不符")
        received = times_from_rx(metadata, now)
        with self.lock:
            if not self.reconnect_requested.is_set():
                for index in indices:
                    self.states[index] = snapshot_motor(result.get(index))
                    if index == 6 and self.states[index] is not None:
                        self.states[index]["raw_pos"] = self.states[index]["pos"]
                        self.states[index]["pos"] = gripper_phase(self.states[index]["pos"])
                    self.feedback_at[index] = received[index]
                self.last_poll = now
        return metadata

    def require_new_feedback(self, hardware, indices, require_heartbeat=True):
        baseline = hardware.rx_meta()
        deadline = time.monotonic() + MAX_RX_AGE_MS / 1000
        while True:
            self.check_stop(require_heartbeat)
            metadata = self.refresh(hardware, indices)
            with self.lock:
                now = time.monotonic()
                missing = [i + 1 for i in indices if metadata[i]["sequence"] <= baseline[i]["sequence"]
                           or not is_fresh(self.states[i], self.feedback_at[i], now)]
            if not missing:
                self.check_stop(require_heartbeat)
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("等待新 CAN 反馈超时：" + "、".join(f"J{i}" for i in missing))
            self.closing.wait(.01)

    def probe(self, hardware, index):
        hardware.probe(index)

    def request_stop(self):
        with self.lock:
            self.motion_request_generation+=1
            self.pending = None
            if not self.standby:
                self.stop.set()

    def save_force_profile(self, calibration):
        payload = calibration.payload()
        with self.lock:
            now = time.monotonic()
            if (not self.connection_ready or self.reconnect_requested.is_set() or
                    any(not is_fresh(s, at, now) or s["status_code"] != 0 or abs(s["vel"]) > .05
                        for s, at in zip(self.states, self.feedback_at))):
                raise RuntimeError("保存力标定需七轴新鲜反馈、静止并失能")
            if profile_torque(payload, self.gripper_points) is None:
                raise RuntimeError("端点/机构已变化或测力记录不完整，未覆盖原力标定")
            temp = self.force_file.with_suffix(".tmp")
            temp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            temp.replace(self.force_file)
            self.force_profile = payload
            with calibration.lock:
                calibration.committed, calibration.commit_requested, calibration.ready = True, False, False
        self.log("J7 名义3N保持参数已独立保存；不是实时测力或峰值限力保证")

    def hold_calibration(self, body):
        action = body.get("action")
        if action == "begin":
            request = {**body, "kind": "gamepad", "joint": 7, "degrees": 1,
                       "gripper_enabled": True, "gripper_calibration": True, "teleop_mode": "continuous", "gripper_policy": "calibrated"}
            return self.submit(request)
        with self.lock:
            calibration = self.gripper_calibration
            if calibration is None:
                raise RuntimeError("请先开始独立测力标定")
            active = self.active
            owns_calibration = active and active.get("gripper_calibration")
            if owns_calibration and body.get("owner") != active["owner"]:
                raise RuntimeError("请在开始此标定的网页上操作；其他页面仍可使用停止按钮")
            if action == "commit":
                if body.get("supported") is not True:
                    raise ValueError("保存前请托稳机械臂并确认可停止失能")
                calibration.payload()
                if owns_calibration and not self.stop.is_set():
                    with calibration.lock:
                        calibration.commit_requested = True
                    self.request_stop()
                    return {"ok": True, "saving_after_disable": True}
                if active or self.pending:
                    raise RuntimeError("请先停止其他任务，再保存已保留的测力记录")
                self.save_force_profile(calibration)
                return {"ok": True, "saved": True}
            if not owns_calibration or self.stop.is_set():
                raise RuntimeError("测力会话未运行或正在停止")
            if action == "abort":
                calibration.abort()
                self.request_stop()
            elif action == "pulse":
                pad = self.gamepad.snapshot()
                if (not pad["connected"] or not pad["deadman"] or pad["stop"] or
                        not triggers_neutral(pad.get("triggers"))):
                    raise RuntimeError("测力脉冲需按住LB、松开LT/RT且输入正常")
                pulse_id = calibration.request_pulse(body.get("torque_nm"))
                return {"ok": True, "pulse_id": pulse_id}
            elif action == "record":
                calibration.record(body.get("pulse_id"), body.get("force_n"))
            else:
                raise ValueError("未知测力操作")
            return {"ok": True}

    def submit(self, body):
        joint = body.get("joint")
        amount = body.get("degrees")
        kind = body.get("kind")
        owner = body.get("owner")
        request_id = body.get("request_id")
        profile = body.get("gravity_profile", "j4_75")
        if type(joint) is not int or not 1 <= joint <= 7:
            raise ValueError("请选择 J1～J7")
        if joint == 7 and kind in ("step", "roundtrip"):
            raise ValueError("J7 不使用普通点动；请通过有独立端点和限矩的夹爪遥操")
        if type(amount) not in (int, float) or not math.isfinite(amount) or not 0 < abs(amount) <= 5:
            raise ValueError("单次测试角度需在 ±0.5°～±5° 内")
        if abs(amount) < 0.5 or kind not in ("step", "roundtrip", "gravity", "gamepad", "teach"):
            raise ValueError("测试参数错误")
        if not isinstance(owner, str) or not 1 <= len(owner) <= 80:
            raise ValueError("页面会话无效")
        if not isinstance(request_id, str) or not re.fullmatch(r"[a-zA-Z0-9_-]{1,80}", request_id):
            raise ValueError("请求编号无效")
        if kind in ("gravity", "gamepad", "teach") and (not isinstance(profile, str) or profile not in GRAVITY_PROFILES):
            raise ValueError("重力配置无效")
        if kind in ("gamepad","teach"):
            if kind=="teach" and (body.get("supported") is not True or body.get("teaching_revision")!=TEACH_REVISION):
                raise ValueError("请确认支撑机械臂/物体，并刷新示教界面")
            if kind=="gamepad" and body.get("safety_revision") != CONTROL_REVISION:
                raise ValueError("旧版手柄入口已停用，请刷新页面使用连续遥操版")
            teleop_mode = body.get("teleop_mode", "continuous")
            if teleop_mode not in ("continuous", "single-axis"):
                raise ValueError("手柄模式无效")
            if joint == 7 and teleop_mode == "single-axis":
                raise ValueError("验证版暂不开放夹爪，请选择J1～J6单轴")
            grip_enabled = body.get("gripper_enabled", False)
            grip_policy = body.get("gripper_policy", "torque-limit")
            if grip_policy not in ("torque-limit", "calibrated"):
                raise ValueError("夹爪策略无效")
            calibration_requested = body.get("gripper_calibration", False)
            if type(calibration_requested) is not bool or (calibration_requested and
                    (body.get("supported") is not True or not grip_enabled or teleop_mode != "continuous")):
                raise ValueError("测力标定需连续模式、启用J7并确认支撑")
            if calibration_requested:
                grip_policy = "calibrated"
            if kind=="teach" and (teleop_mode!="continuous" or grip_enabled is not True or grip_policy!="torque-limit" or calibration_requested):
                raise ValueError("手推示教固定为七轴及2Nm限矩回放，不使用旧测力档")
            if type(grip_enabled) is not bool or (grip_enabled and teleop_mode != "continuous"):
                raise ValueError("夹爪必须在连续模式中单独选择启用")
            pad = self.gamepad.snapshot()
            if kind=="gamepad" and body.get("input_verified") is not True:
                raise ValueError("请先核对手柄输入预览")
            triggers = pad.get("triggers")
            if kind=="gamepad" and (not pad["connected"] or pad["deadman"] or pad["stop"] or any(abs(v) >= 0.10 for v in pad["intent"])
                    or ((grip_enabled or triggers is not None) and not triggers_neutral(triggers))):
                raise RuntimeError("请连接手柄、松开LB并让全部摇杆/扳机回中")
        with self.lock:
            if self.recovery_pending or self.recovery_running:
                raise RuntimeError('通信报警恢复处理中，暂不可启动')
            if not self.connection_ready or self.error or self.reconnect_requested.is_set():
                raise RuntimeError("USB/CAN连接尚未恢复并确认七轴失能，请等待自动重连")
            if self.stop.is_set() or self.closing.is_set():
                raise RuntimeError("停止处理中，等待失能反馈后再启动")
            if request_id in self.seen:
                return self.seen[request_id]
            if self.pending or self.active:
                raise RuntimeError("正在测试，请等待结束或点击停止")
            if any(state and state["status_code"] != 0 for state in self.states):
                raise RuntimeError("当前有电机处于使能或故障状态，请先停止并核查")
            now = time.monotonic()
            required = range(7) if kind in ("gravity", "gamepad", "teach") else (joint - 1,)
            if not all(is_fresh(self.states[i], self.feedback_at[i], now) for i in required):
                raise RuntimeError("需要新鲜的 CAN 状态反馈，寄存器响应不能代替关节反馈")
            if kind in ("gamepad","teach") and any(abs(state["vel"]) > 0.05 for state in self.states):
                raise RuntimeError("请扶稳机械臂，七轴静止后再启动单轴验证")
            if kind in ("gamepad","teach") and grip_enabled and not self.gripper_can_start():
                raise RuntimeError("J7 需记录有效的软件端点，并在端点范围内静止后再启动")
            self.lease = time.monotonic()
            self.pending = {"joint": joint, "degrees": float(amount), "kind": kind,
                            "owner": owner, "id": request_id, "phase": "准备", "target": None,
                            "gravity_profile": profile}
            if kind in ("gamepad","teach"):
                if calibration_requested:
                    self.gripper_calibration = ForceCalibration(self.gripper_points)
                self.pending.update(revision=CONTROL_REVISION, teleop_mode=teleop_mode, gripper_enabled=grip_enabled,
                    gripper_calibration=calibration_requested, gripper_policy=grip_policy,
                    validation="URDF ranges + 0.06rad following lead" if teleop_mode == "continuous" else "single-axis +/-3deg",
                    started_wall=time.time())
            response = {"accepted": True, "id": request_id}
            self.seen[request_id] = response
            while len(self.seen) > 64:
                self.seen.popitem(last=False)
            return response

    def perform_gravity(self, hardware, job):
        initial_play=job.pop('_teaching_play',None)
        job["tau_scale"] = list(GRAVITY_PROFILES[job["gravity_profile"]]["scales"])
        job["axes"] = list(range(1, 8 if job.get("gripper_enabled") else 7))
        job["mode"] = "continuous"
        samples = collections.deque(maxlen=750 if job["kind"] == "gamepad" else 3000)
        startup_samples = []
        protection_events=collections.deque(maxlen=256)
        def collect_control_records():
            rows=hardware.take_control_records()
            protection_events.extend(row for row in rows if row.get("event") in ("local_pause","fault","disable_error"))
            samples.extend(rows)
        error = None
        stop_reason = None
        failure = None
        origin = None
        peak = 0.0
        attempted_start = False
        started_at = time.monotonic()
        overspeed_since = [None] * 7
        pause_latched = False
        try:
            self.check_stop(require_heartbeat=False)
            teleop = job["kind"] in ("gamepad","teach")
            grip_limits = None
            if teleop:
                # Retain the register preflight only while stopped; it cannot
                # substitute for the fresh joint state checked afterwards.
                for index in range(7):
                    self.check_stop(require_heartbeat=False)
                    self.probe(hardware, index)
                if job.get("gripper_enabled"):
                    self.check_stop(require_heartbeat=False)
                    job["gripper_watchdog_ms"] = hardware.gripper_watchdog()
                    job["gripper_watchdog_config"] = getattr(hardware, "last_gripper_watchdog", None)
                    if job["gripper_watchdog_config"] and job["gripper_watchdog_config"]["configured_ram"]:
                        self.log(f"J7 断联超时已自动配置为 {job['gripper_watchdog_ms']:g}ms（仅RAM，已读回验证）")
            self.require_new_feedback(hardware, range(7), require_heartbeat=False)
            with self.lock:
                if any(state is None or state["status_code"] != 0 for state in self.states):
                    raise RuntimeError("持续重力补偿启动前七轴需反馈正常且失能")
                if teleop and any(abs(state["vel"]) > 0.05 for state in self.states):
                    raise RuntimeError("机械臂尚在移动，取消单轴验证启动")
                if teleop and job.get("gripper_enabled"):
                    if not self.gripper_can_start():
                        raise RuntimeError("J7 已不在安全启动范围内")
                    if job["kind"]!="teach" and not triggers_neutral(self.gamepad.snapshot().get("triggers")):
                        raise RuntimeError("J7 启动前必须松开两只扳机")
                    grip_limits = self.gripper_range()
                origin = [state["pos"] for state in self.states[:6]]
                self.active["phase"] = "官方控制器启动"
            self.check_stop(require_heartbeat=False)
            if initial_play is not None:
                self.check_stop(require_heartbeat=True)
                # Recheck fresh positions after all stopped preflights, before
                # enabling. The servo rechecks again after the startup fade.
                from panel.teaching import check_start,check_home_start
                home_limits=None
                if initial_play.get('home'):
                    if self.teaching_signature()!=initial_play['signature']:
                        raise ValueError("机构配置已变化，取消回位")
                    home_limits=self.teaching_limits()
                with self.lock:
                    q=[s['pos'] for s in self.states]
                    v=[s['vel'] for s in self.states];goal=initial_play['_rows'][0]['q']
                    if home_limits:check_home_start(q,v,goal,home_limits)
                    else:check_start(q,v,goal)
            attempted_start = True
            cal = self.gripper_calibration if job.get("gripper_calibration") else None
            hold_tau = None if cal or not grip_limits else profile_torque(self.force_profile, self.gripper_points)
            torque_limited = grip_limits is not None and cal is None and job.get("gripper_policy","torque-limit")=="torque-limit"
            if torque_limited:
                hold_tau=None
            if teleop:
                job["gripper_hold_torque_nm"] = hold_tau
                job["gripper_torque_limit_nm"] = TORQUE_LIMIT_NM if torque_limited else None
            hardware.begin_gravity(job["tau_scale"], self.gamepad if teleop else None,
                                   grip_limits,
                                   None if teleop and job.get("teleop_mode") == "continuous" else job["joint"]-1,
                                   check_start=lambda: self.check_stop(require_heartbeat=False),
                                   hold_torque=hold_tau,calibration=cal,torque_limited=torque_limited,
                                   **({"teaching":{"signature":self.teaching_signature(),"initial_play":initial_play}} if job["kind"]=="teach" else {}))
            started_at = time.monotonic()
            self.log(("七轴手推示教已启动 · " if job["kind"]=="teach" else "手柄控制已启动 · " if teleop else "官方持续重力补偿已启动 · ") + GRAVITY_PROFILES[job["gravity_profile"]]["label"])
            next_sample = started_at
            while True:
                gravity_status = None if teleop else hardware.gravity_status()
                if gravity_status:
                    if gravity_status["fault"]:
                        raise RuntimeError(gravity_status["fault"])
                    age = gravity_status["age_ms"]
                    if age is not None and watchdog_action(age) == "FAULT":
                        failure = {"reason": "纯重力控制完成心跳超时", "control": gravity_status}
                        raise RuntimeError(f"纯重力控制完成心跳异常，年龄 {age:.1f}ms（上限 {HARD_TIMEOUT_MS:g}ms）")
                self.check_stop(require_heartbeat=False)
                now = time.monotonic()
                elapsed = now - started_at
                observed, torque, running = hardware.observe_gravity()
                teleop_state = None
                if teleop:
                    teleop_state, wants_stop = hardware.teleop_status()
                    if job["kind"]=="teach":
                        control=getattr(getattr(hardware,"gravity_session",None),"control",None)
                        self.live_teacher=getattr(control,"teacher",None) if control else getattr(hardware,"teacher",None)
                        self.check_teaching_presence()
                        self.flush_teaching()
                    collect_control_records()
                    if wants_stop:
                        raise InterruptedError("手柄B/Back请求停止")
                if not running:
                    raise RuntimeError((teleop_state or {}).get("fault") or "官方重力控制线程已退出")
                if teleop_state:
                    watchdog = hardware.control_watchdog()
                    action = watchdog_action(watchdog["age_ms"])
                    if action == "FAULT":
                        failure = {"reason": "控制完成心跳超时", **watchdog}
                        raise RuntimeError(f"本地MIT持续卡顿超过250ms，阶段={watchdog['phase']} age={watchdog['age_ms']:.0f}ms")
                    if action == "PAUSE" and not pause_latched:
                        hardware.pause_control()
                        pause_latched = True
                        self.log(f"短暂停顿 {watchdog['age_ms']:.0f}ms：保持、不退出；释放LB并回中后恢复")
                    elif action == "OK":
                        pause_latched = False
                if any(state is None for state in observed.values()):
                    raise RuntimeError("持续重力补偿反馈缺失")
                states = [snapshot_motor(observed[index]) for index in range(7)]
                if states[6] is not None:
                    states[6]["raw_pos"] = states[6]["pos"]
                    states[6]["pos"] = gripper_phase(states[6]["pos"])
                for index, state in enumerate(states):
                    if state is None:
                        raise RuntimeError(f"J{index+1} 反馈包含无效数值")
                    if state["status_code"] != 1 and elapsed > 0.5:
                        raise RuntimeError(f"J{index+1} 驱动状态异常 {state['status_code']}")
                    if max(state["t_mos"], state["t_rotor"]) >= 65:
                        raise RuntimeError(f"J{index+1} 过温")
                    velocity_limit = 2.0 if index < 3 else 3.0
                    local_gripper_pause=(teleop and teleop_state and
                        ((index==6 and job.get("gripper_enabled") and teleop_state.get("gripper_paused"))
                         or (index<6 and teleop_state.get("arm_paused"))))
                    if abs(state["vel"]) > velocity_limit and not local_gripper_pause:
                        if overspeed_since[index] is None:
                            overspeed_since[index] = now
                        if now - overspeed_since[index] >= 0.1:
                            failure = {"joint": index+1, "reason": "持续硬超速",
                                       "velocity_rad_s": state["vel"], "angle_rad": state["pos"], "t": elapsed}
                            raise RuntimeError(f"J{index+1} 持续硬超速 v={state['vel']:+.3f}rad/s")
                    else:
                        overspeed_since[index] = None
                if len(torque) != 6 or any(not math.isfinite(value) or abs(value) > (28 if i < 3 else 10)
                                           for i, value in enumerate(torque)):
                    raise RuntimeError("官方重力力矩超出电机量程")
                metadata = hardware.rx_meta()
                feedback_at = times_from_rx(metadata, time.monotonic())
                if len(feedback_at) != 7:
                    raise RuntimeError("CAN 反馈轴数与配置不符")
                for index, at in enumerate(feedback_at):
                    if at is None:
                        raise RuntimeError(f"J{index+1}真实CAN反馈超过150ms未更新")
                angles = [state["pos"] for state in states[:6]]
                peak = max(peak, *(abs(math.degrees(angle-start)) for angle, start in zip(angles, origin)))
                with self.lock:
                    if self.reconnect_requested.is_set():
                        raise InterruptedError("USB/CAN连接已断开")
                    self.states = states
                    self.feedback_at = feedback_at
                    self.last_poll = time.monotonic()
                    self.active.update({"phase": "持续重力补偿", "elapsed": elapsed,
                                        "progress": 1.0, "scale": 1.0, "torques": torque,
                                        "target": math.degrees(states[job["joint"]-1]["pos"])})
                    if teleop:
                        self.active.update(phase=teleop_state["status"], teleop=teleop_state)
                        self.active["target"] = math.degrees(teleop_state["target_rad"][job["joint"]-1]) if "target_rad" in teleop_state else None
                    elif gravity_status and "blend" in gravity_status:
                        self.active.update(gravity_control=gravity_status, scale=gravity_status["blend"],
                                           phase="重力渐入" if gravity_status["blend"] < 1. else "持续重力补偿")
                if not teleop and now >= next_sample:
                    sample = {"t": elapsed, "q": angles, "command": torque,
                              "velocity": [state["vel"] for state in states[:6]],
                              "feedback": [state["torq"] for state in states[:6]]}
                    if gravity_status:
                        sample["control"] = gravity_status
                        # Keep the traditional report columns aligned to one
                        # completed tick rather than mixed monitor snapshots.
                        if "sent_nm" in gravity_status:
                            sample.update(q=gravity_status["q_rad"], command=gravity_status["sent_nm"],
                                          velocity=gravity_status["velocity_rad_s"],
                                          feedback=gravity_status["driver_feedback_nm"])
                    samples.append(sample)
                    if elapsed <= 1.5:
                        startup_samples.append(sample)
                    next_sample = now + 0.04
                time.sleep(0.02)
        except InterruptedError as exc:
            stop_reason = str(exc)
        except Exception as exc:
            error = str(exc)
            if job['kind']=='teach':self.notify_teaching("示教任务未完成："+error)
        finally:
            if attempted_start:
                with self.lock:
                    self.active["phase"] = "平滑停止" if error is None else "故障失能"
                try:
                    if job["kind"] in ("gamepad","teach") and getattr(hardware, "gravity_session", None):
                        collect_control_records()
                    if job["kind"]=="teach" and self.live_teacher:
                        self.live_teacher.finish(False)
                        self.flush_teaching()
                    hardware.end_gravity(gentle=(error is None and not self.closing.is_set()
                                                 and not self.reconnect_requested.is_set()))
                except Exception as exc:
                    error = f"{error or ''} 停止异常 {exc}".strip()
                    self.request_stop()
            calibration = self.gripper_calibration if job.get("gripper_calibration") else None
            if calibration:
                if calibration.commit_requested and error is None:
                    try:
                        if not self.verify_disabled(hardware):
                            raise RuntimeError("失能反馈未确认，未保存力标定")
                        self.save_force_profile(calibration)
                    except Exception as exc:
                        error = f"力标定未保存：{exc}"
                        self.log(error + "；已有测力记录保留，可停止后重试保存")
                elif not calibration.commit_requested:
                    calibration.abort()
            result = {"id": job["id"], "joint": job["joint"], "kind": job["kind"],
                      "revision": job.get("revision"), "validation": job.get("validation"),
                      "axes": job["axes"], "error": error, "stop_reason": stop_reason,
                      "fault": failure, "tau_scale": job["tau_scale"], "delta": None,
                      "peak": peak, "duration_s": time.monotonic()-started_at}
            with self.lock:
                if job["kind"]=="teach":self.live_teacher=None
                self.last_result = result
                self.active = None
            self.log("持续重力补偿已结束 · " + (error or stop_reason or "已失能"))
            try:
                report_dir = ROOT.parent / "calibration_snapshots"
                report_dir.mkdir(exist_ok=True)
                (report_dir / f"panel_{job['id']}.json").write_text(
                    json.dumps({"job": job, "result": result, "samples": list(samples),
                                "startup_samples": startup_samples,
                                "protection_events":list(protection_events),
                                "sample_limit": samples.maxlen}, ensure_ascii=False, indent=2), encoding="utf-8",
                )
            except OSError as exc:
                self.log(f"重力记录保存失败 · {exc}")

    def heartbeat(self, owner):
        with self.lock:
            job = self.active or self.pending
            if job and job["owner"] == owner:
                self.lease = time.monotonic()

    def gripper_range(self):
        return range_from_points(self.gripper_points)

    def gripper_can_start(self):
        limits = self.gripper_range()
        state = self.states[6]
        return bool(limits and is_fresh(state, self.feedback_at[6], time.monotonic())
                    and state["status_code"] == 0 and abs(state["vel"]) <= .05
                    and limits[0] <= state["pos"] <= limits[1])

    def capture_gripper(self, body):
        end = body.get("end")
        if end not in ("open", "closed"):
            raise ValueError("请选择安全张开或闭合端点")
        with self.lock:
            now = time.monotonic()
            if (not self.connection_ready or self.stop.is_set() or self.active or self.pending or
                    any(not is_fresh(s, at, now) or s["status_code"] != 0
                        for s, at in zip(self.states, self.feedback_at))):
                raise RuntimeError("记录端点前七轴必须全部失能")
            if not is_fresh(self.states[6], self.feedback_at[6], time.monotonic()) or abs(self.states[6]["vel"]) > 0.05:
                raise RuntimeError("夹爪反馈需在线且静止")
            points = {key: value for key, value in self.gripper_points.items()
                      if key in ("open", "closed") and range_from_points({key: value}, complete=False)}
            points[end] = self.states[6]["pos"]
            if range_from_points(points, complete=False) is None:
                raise ValueError("张开和收拢软件端点必须位于 54.3°～236.3°，收拢角大于张开角且至少相差约 1.72°")
            temp = self.gripper_file.with_suffix(".tmp")
            temp.write_text(json.dumps(points, indent=2), encoding="utf-8")
            temp.replace(self.gripper_file)
            self.gripper_points = points
        self.log("已记录夹爪安全端点 · " + end)
        return {"ok": True, "ready": self.gripper_range() is not None}

    def save_direction(self, body):
        joint = body.get("joint")
        direction = body.get("direction")
        if type(joint) is not int or not 1 <= joint <= 7 or direction not in DIRECTIONS[joint - 1]:
            raise ValueError("方向选项无效")
        with self.lock:
            self.directions[str(joint)] = direction
            temp = self.direction_file.with_suffix(".tmp")
            temp.write_text(json.dumps(self.directions, ensure_ascii=False, indent=2), encoding="utf-8")
            temp.replace(self.direction_file)
        self.log(f"已记录 J{joint} 正方向 · {direction}")

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            joints = []
            for index, state in enumerate(self.states):
                at = self.feedback_at[index]
                joints.append({"id": index + 1, "label": LABELS[index],
                               "model": "4340P" if index < 3 else "4310",
                               "online": is_fresh(state, at, now),
                               "feedback_age_ms": round((now - at) * 1000, 1) if at is not None else None,
                               "angle": math.degrees(state["pos"]) if state else None,
                               "raw_angle": math.degrees(state.get("raw_pos", state["pos"])) if state else None,
                               "velocity": state["vel"] if state else None,
                               "torque": state["torq"] if state else None,
                               "status": state["status_code"] if state else None,
                               "mos": state["t_mos"] if state else None,
                               "rotor": state["t_rotor"] if state else None})
            return {"simulate": self.simulate, "joints": joints,
                    "recovery":self.recovery_snapshot(),
                    "connection": {"ready": self.connection_ready, "state": self.connection_state,
                                   "stopping": self.stop.is_set() and self.connection_ready,
                                   "reconnect_count": self.reconnect_count, "session_id": self.connection_session},
                    "active": dict(self.active or self.pending) if self.active or self.pending else None,
                    "result": self.last_result, "error": self.error,
                    "events": list(self.events), "directions": dict(self.directions),
                    "gravity_profiles": GRAVITY_PROFILES,
                    "gamepad_safety_revision": CONTROL_REVISION,
                    "teaching":self.teaching_snapshot(),
                    "control_policy": {"soft_pause_ms": SOFT_PAUSE_MS, "hard_timeout_ms": HARD_TIMEOUT_MS,
                                       "experience":experience_snapshot(),
                                       "arm_motion_event_action":"pause-and-hold",
                                       "gravity_startup_seconds":GRAVITY_STARTUP_SECONDS,
                                       "monitor": "published-state", "log_hz": 25,
                                       "max_speed_rad_s": list(CONTINUOUS_SPEED)},
                    "gamepad": self.gamepad.snapshot(),
                    "gripper": {"points_rad": dict(self.gripper_points), "ready": self.gripper_range() is not None,
                                "can_start": self.gripper_can_start(), "envelope_deg": ENVELOPE_DEG,
                                "max_speed_rad_s": GRIPPER_MAX_SPEED_RAD_S,
                                "hard_speed_rad_s": GRIPPER_HARD_SPEED_RAD_S,
                                "soft_speed_rad_s": GRIPPER_SOFT_SPEED_RAD_S,
                                "speed_event_action":"pause-and-hold-arm",
                                "torque_limit_nm": TORQUE_LIMIT_NM,
                                "contact_motion_policy": contact_motion_policy(),
                                "control": CONTROL_NAME,
                                "hold_calibrated": profile_torque(self.force_profile, self.gripper_points) is not None,
                                "hold_torque_nm": profile_torque(self.force_profile, self.gripper_points),
                                "nominal_force_n": 3. if profile_torque(self.force_profile, self.gripper_points) is not None else None,
                                "measured_force_n": None,
                                "calibration": self.gripper_calibration.snapshot() if self.gripper_calibration else None},
                    "poll_age_ms": (now - self.last_poll) * 1000 if self.last_poll else None}

    def check_stop(self, require_heartbeat=True):
        if self.closing.is_set() or self.stop.is_set() or self.reconnect_requested.is_set():
            raise InterruptedError("已请求停止")
        if require_heartbeat and time.monotonic() - self.lease > 0.8:
            raise InterruptedError("页面连接超时，结束测试")

    def perform(self, hardware, job):
        index = job["joint"] - 1
        enabled = False
        error = None
        interrupted = False
        samples = []
        origin = None
        last_target = None
        kp = 40.0 if index < 3 else (8.0 if index == 6 else 18.0)
        kd = 2.0 if index < 6 else 1.0
        try:
            self.check_stop()
            self.require_new_feedback(hardware, [index])
            with self.lock:
                state = dict(self.states[index])
            if state["status_code"] != 0:
                raise RuntimeError("测试前电机需处于失能状态")
            origin = state["pos"]
            last_target = origin
            self.check_stop()
            enabled = True
            hardware.enable(index)
            delta = math.radians(job["degrees"])
            duration = 8.0 if job["kind"] == "roundtrip" else 5.0
            start = time.monotonic()
            next_tick = start
            next_all_feedback = start
            while time.monotonic() - start < duration:
                self.check_stop()
                if time.monotonic() >= next_all_feedback:
                    self.refresh(hardware, range(7))
                    next_all_feedback = time.monotonic() + .08
                else:
                    self.refresh(hardware, [index])
                self.check_stop()
                elapsed = time.monotonic() - start
                if elapsed < 1.0:
                    phase, fraction, gain = "渐入", 0.0, elapsed
                elif elapsed < 4.0:
                    u = (elapsed - 1.0) / 3.0
                    phase, fraction, gain = "点动", 10*u**3 - 15*u**4 + 6*u**5, 1.0
                elif job["kind"] == "roundtrip" and elapsed < 7.0:
                    u = (elapsed - 4.0) / 3.0
                    phase, fraction, gain = "返回", 1.0 - (10*u**3 - 15*u**4 + 6*u**5), 1.0
                else:
                    phase = "卸力"
                    fraction = 0.0 if job["kind"] == "roundtrip" else 1.0
                    gain = max(0.0, duration - elapsed)
                with self.lock:
                    if not is_fresh(self.states[index], self.feedback_at[index], time.monotonic()):
                        raise RuntimeError(f"J{index+1}真实CAN反馈超过150ms未更新")
                    state = dict(self.states[index])
                    last_target = origin + delta * fraction
                    self.active.update({"phase": phase, "target": math.degrees(last_target),
                                        "origin": math.degrees(origin), "progress": min(1.0, elapsed / duration)})
                if state["status_code"] != 1 and elapsed > 0.3:
                    raise RuntimeError(f"驱动状态异常 {state['status_code']}")
                if abs(state["pos"] - origin) > math.radians(8) or abs(state["vel"]) > 0.5:
                    raise RuntimeError("位移或速度超出小角度测试范围")
                if max(state["t_mos"], state["t_rotor"]) >= 65:
                    raise RuntimeError("电机温度达到测试停止阈值")
                hardware.command(index, last_target, kp * gain, kd)
                samples.append({"t": elapsed, "target": math.degrees(last_target),
                                "actual": math.degrees(state["pos"]), "torque": state["torq"]})
                next_tick = max(next_tick + 0.01, time.monotonic())
                time.sleep(max(0.0, next_tick - time.monotonic()))
        except InterruptedError as exc:
            error, interrupted = str(exc), True
        except Exception as exc:
            error = str(exc)
        finally:
            try:
                if enabled:
                    try:
                        with self.lock:
                            state = self.states[index]
                            unload = (interrupted and not self.closing.is_set()
                                      and not self.reconnect_requested.is_set()
                                      and is_fresh(state, self.feedback_at[index], time.monotonic())
                                      and state["status_code"] == 1)
                            if unload:
                                self.active["phase"] = "停止卸力"
                        if unload:
                            for gain in (0.6, 0.3, 0.0):
                                hardware.command(index, state["pos"], kp * gain, kd)
                                time.sleep(0.03)
                    finally:
                        hardware.disable(index)
            except Exception as exc:
                error = f"{error or ''} 失能确认异常 {exc}".strip()
                self.request_stop()
            finally:
                result = {"id": job["id"], "joint": index + 1, "requested": job["degrees"], "error": error,
                          "delta": samples[-1]["actual"] - math.degrees(origin) if samples else None,
                          "peak": max((abs(row["actual"] - math.degrees(origin)) for row in samples), default=0)}
                with self.lock:
                    self.last_result = result
                    self.active = None
                self.log(f"J{index+1} 测试结束 · {error or '已发送失能'}")
                report_dir = ROOT.parent / "calibration_snapshots"
                report_path = report_dir / f"panel_{job['id']}.json"
                try:
                    report_dir.mkdir(exist_ok=True)
                    report_path.write_text(json.dumps({"job": job, "result": result, "samples": samples},
                                                      ensure_ascii=False, indent=2), encoding="utf-8")
                except OSError as exc:
                    self.log(f"测试记录保存失败 · {exc}")

    def run(self):
        hardware = None
        try:
            if self.standby:
                self.log("网页已启动，硬件待手动连接；尚未打开USB2CAN或发送CAN指令")
                return
            if not self.simulate:
                self.usb_generation = usb_signature()
                if not self.usb_generation:
                    self.connection_lost("USB unavailable：请插入达妙USB2CAN")
                    return
            hardware = Simulation() if self.simulate else Hardware()
            self.log("仿真连接已启动" if self.simulate else "USB2FDCAN 已连接 · 正在核验七轴")
            if not self.verify_disabled(hardware):
                return
            while not self.closing.is_set() and not self.reconnect_requested.is_set():
                if self.service_recovery(hardware):continue
                with self.lock:
                    job = self.pending
                    if job and not self.stop.is_set():
                        self.active, self.pending = job, None
                    else:
                        job = None
                if job:
                    if job["kind"] in ("gravity", "gamepad", "teach"):
                        self.perform_gravity(hardware, job)
                    else:
                        self.perform(hardware, job)
                    continue
                if self.stop.is_set():
                    with self.lock:
                        self.pending = None
                    for index in range(7):
                        hardware.disable(index)
                    with self.lock:
                        if not self.closing.is_set() and not self.reconnect_requested.is_set():
                            self.stop.clear()
                    self.log("七轴失能指令已发送")
                self.refresh(hardware, list(range(7)))
                self.closing.wait(0.035)
        except Exception as exc:
            if not self.simulate and transport_error(exc):
                self.connection_lost(str(exc))
            else:
                with self.lock:
                    self.error = str(exc)
                    self.connection_ready = False
                    self.connection_state = "ERROR"
                self.log(f"连接异常 · {exc}")
        finally:
            if hardware:
                try:
                    for index in range(7):
                        try:
                            hardware.disable(index)
                        except Exception:
                            pass
                finally:
                    try:
                        hardware.close()
                    except Exception as exc:
                        self.log(f"旧USB句柄清理失败，等待进程隔离清理 · {exc}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.127.10")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--standby", action="store_true", help="先只启动网页，用户确认支撑后再连接硬件")
    args = parser.parse_args()
    token = secrets.token_urlsafe(32)
    lock_file = (ROOT / ("panel.sim.lock" if args.simulate else "panel.lock")).open("w")
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Manual standby applies to initial launch. A USB recovery in an already
    # connected session retains its existing disable-only recovery handshake.
    standby = args.standby and int(os.environ.get("REBOT_RECONNECT_COUNT", "0")) == 0
    panel = Panel(args.simulate, standby)
    camera = CameraPreview(args.simulate)
    allowed_host = f"{args.host}:{args.port}"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *values):
            pass

        def send(self, status, body, content_type="application/json"):
            data = json.dumps(body, ensure_ascii=False).encode() if isinstance(body, dict) else body
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.headers.get("Host") != allowed_host:
                self.send(403, {"error": "Host rejected"})
                return
            path = urlsplit(self.path).path
            if path == "/api/camera/frame":
                if self.headers.get("X-Arm-Token") != token:
                    self.send(403, {"error": "Invalid camera request"})
                    return
                try:
                    frame = camera.frame(self.headers.get("X-Camera-Lease"))
                except RuntimeError as exc:
                    self.send(409, {"error": str(exc)})
                    return
                if frame:
                    self.send(200, frame, "image/jpeg")
                else:
                    self.send(204, b"", "image/jpeg")
                return
            if path == "/api/state":
                snapshot = panel.snapshot()
                snapshot["csrf_token"] = token
                self.send(200, snapshot)
                return
            if path == "/api/urdf-view" or path.startswith("/urdf-view/mesh/"):
                try:
                    manifest, assets = load_view(ROOT.parent)
                    if path == "/api/urdf-view":
                        self.send(200, manifest)
                        return
                    index = path.removeprefix("/urdf-view/mesh/")
                    if not re.fullmatch(r"[0-9]{1,3}", index) or int(index) >= len(assets):
                        self.send(404, {"error": "Model asset not found"})
                        return
                    self.send(200, assets[int(index)].read_bytes(), "model/stl")
                except (OSError, KeyError, TypeError, ValueError, SyntaxError) as exc:
                    print(f"URDF viewer unavailable: {exc}", flush=True)
                    self.send(503, {"error": "URDF model unavailable"})
                return
            if path == "/":
                html = (ROOT / "static/index.html").read_text(encoding="utf-8").replace("__TOKEN__", token)
                self.send(200, html.encode(), "text/html; charset=utf-8")
                return
            files = {"/app.js": (ROOT / "static/app.js", "text/javascript; charset=utf-8"),
                     "/urdf-viewer.js": (ROOT / "static/urdf-viewer.js", "text/javascript; charset=utf-8"),
                     "/camera-preview.js": (ROOT / "static/camera-preview.js", "text/javascript; charset=utf-8"),
                     "/teaching.js": (ROOT / "static/teaching.js", "text/javascript; charset=utf-8"),
                     "/style.css": (ROOT / "static/style.css", "text/css; charset=utf-8"),
                     "/zero.png": (ROOT.parent / "official_zero_pose.png", "image/png")}
            if path in files and files[path][0].is_file():
                file, mime = files[path]
                self.send(200, file.read_bytes(), mime)
            else:
                self.send(404, {"error": "Not found"})

        def do_POST(self):
            origin = self.headers.get("Origin")
            if (self.headers.get("Host") != allowed_host or
                    self.headers.get("X-Arm-Token") != token or origin != f"http://{allowed_host}"):
                self.send(403, {"error": "请求来源校验失败"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("请求长度无效")
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError("请求格式无效")
                path = urlsplit(self.path).path
                if path == "/api/test":
                    response = panel.submit(body)
                elif path == "/api/connect":
                    response = panel.connect_hardware(body)
                elif path == "/api/teaching":
                    response = panel.teaching_action(body)
                elif path == "/api/recovery":
                    response = panel.request_recovery(body)
                elif path == "/api/camera/start":
                    response = camera.start()
                elif path == "/api/camera/stop":
                    response = camera.stop(body.get("lease"))
                elif path == "/api/heartbeat":
                    panel.heartbeat(body.get("owner"))
                    response = {"ok": True}
                elif path == "/api/stop":
                    panel.request_stop()
                    response = {"ok": True}
                elif path == "/api/direction":
                    panel.save_direction(body)
                    response = {"ok": True}
                elif path == "/api/gripper/capture":
                    response = panel.capture_gripper(body)
                elif path == "/api/gripper/hold-calibration":
                    response = panel.hold_calibration(body)
                elif path == "/api/sim-input" and args.simulate:
                    panel.gamepad.inject(body)
                    response = {"ok": True}
                else:
                    self.send(404, {"error": "Not found"})
                    return
                self.send(200, response)
            except ValueError as exc:
                self.send(400, {"error": str(exc)})
            except RuntimeError as exc:
                self.send(409, {"error": str(exc)})
            except OSError as exc:
                panel.log(f"保存失败 · {exc}")
                self.send(503, {"error": "保存失败，请检查 RDK 存储状态"})

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    server.service_actions = panel.service_usb
    reconnect_exit = False

    def terminate(signum, frame):
        panel.closing.set()
        panel.stop.set()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    print(f"reBot panel http://{allowed_host}", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except RestartForUsb:
        reconnect_exit = True
    except KeyboardInterrupt:
        pass
    finally:
        panel.closing.set()
        panel.stop.set()
        camera.close()
        if panel.worker is not None:
            panel.worker.join(timeout=5)
        panel.gamepad.close()
        server.server_close()
        if reconnect_exit:
            # The vendor runtime keeps a process-global USB handle even after
            # close(). A fresh child is required after unplug/re-enumeration.
            os._exit(RECONNECT_EXIT)


if __name__ == "__main__":
    if "--simulate" not in sys.argv and os.environ.get("REBOT_PANEL_CHILD") != "1":
        raise SystemExit(supervise(Path(__file__).resolve(), sys.argv[1:]))
    main()
