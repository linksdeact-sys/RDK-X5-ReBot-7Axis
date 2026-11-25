"""Motor-local MIT position/damping loops with bounded single-axis references."""

import collections
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

from reBotArm_control_py.controllers import GravityCompensation
from .feedback_state import times_from_rx
from .gripper_teleop import GripperTeleop, IDLE_KD, gripper_phase
from .teleop_motion import SmoothDrive, CONTROL_REVISION
from .gripper_hold import CONTROL_NAME, GripperCommand
from .arm_motion_guard import ArmMotionGuard,startup_scale,GRAVITY_STARTUP_SECONDS
from .gamepad_input import triggers_neutral

REVISION = CONTROL_REVISION


class GamepadGravityController(GravityCompensation):
    KP = np.array([20., 25., 25., 4., 4., 4.])
    KD = np.array([1.5, 2., 2., 1., 0.7, 0.7])

    def __init__(self, robot, *, input_provider, gripper_limits, selected_axis=0, rx_provider=None,
                 hold_torque=None, calibration=None, torque_limited=False, teaching=None, **kwargs):
        super().__init__(robot, **kwargs)
        if rx_provider is None:
            raise RuntimeError("缺少真实RX时间戳，禁止启动验证")
        self.input_provider = input_provider
        self.rx_provider = rx_provider
        self.selected_axis = selected_axis
        if gripper_limits is not None and selected_axis is not None:
            raise ValueError("J7 遥操只允许在连续模式中启用")
        self.gripper = GripperTeleop(gripper_limits,hold_torque=hold_torque,calibration=calibration,torque_limited=torque_limited)
        self.calibration = calibration
        self.teaching_config=teaching
        self.teacher=None
        self.teach_compliance=None
        self.stop_gripper = threading.Event()
        root = ET.parse("urdf/DM/urdf/ReBot_Arm_DM.urdf").getroot()
        self.limits = []
        for name in robot.arm.joint_names:
            limit = root.find(f"joint[@name='{name}']/limit")
            if limit is None:
                raise ValueError(f"Missing URDF range for {name}")
            self.limits.append((float(limit.attrib["lower"]), float(limit.attrib["upper"])))
        self.motion = None
        self.previous_time = None
        self.stop_requested = False
        self.last_mit = None
        self.filtered_gravity = None
        self.telemetry_lock = threading.Lock()
        # Log at 25 Hz, not at the 100 Hz servo rate. Bounded retained history
        # prevents hundreds of thousands of nested telemetry objects building up.
        self.records = collections.deque(maxlen=750)
        self.next_record = 0.0
        self.latest_states = None
        self.pause_requested = threading.Event()
        self.telemetry = {"status": "正在启动单轴验证", "driving": False, "gripper_ready": False,
                          "revision": REVISION, "selected_axis": None if selected_axis is None else selected_axis+1}
        self.runtime = {"phase": "startup", "last_complete": time.monotonic()}
        self.watchdog_started = time.monotonic()
        self.fault_context = {}
        self.arm_guard=ArmMotionGuard()
        self.startup_elapsed=0.

    def _resolve_params(self, n):
        super()._resolve_params(n)
        # The DM MIT protocol encodes Kd in [0, 5]. The inherited config
        # contains Kd=8 on J1-J3, which would otherwise be silently saturated.
        self._hold_kp = np.clip(self._hold_kp, 0., 500.)
        self._hold_kd = np.clip(self._hold_kd, 0., 5.)
        if self.selected_axis is None:
            # Use the project's existing position-hold gains, not the weak
            # validation gains that could not overcome J2's tracking error.
            self.KP, self.KD = self._hold_kp.copy(), self._hold_kd.copy()
        if not all(np.isfinite(a).all() and np.all(a > 0) for a in (self.KP, self.KD)):
            raise ValueError("遥操Kp/Kd必须为正的有限值")

    def start(self):
        self.rebotarm._ctrl_rate = 100.0
        super().start()
        self.watchdog_started = time.monotonic()

    def _loop_cb(self, robot, dt):
        try:
            self._tick(robot, dt)
        except Exception as exc:
            # Keep the triggering frame, not just the last successful 25 Hz row.
            self.stop_gripper.set()
            with self.telemetry_lock:
                self.telemetry.update(fault=str(exc), status="故障保持，等待失能")
                self.records.append({"event": "fault", "reason": str(exc), "at": time.monotonic(),
                                     "trigger": self.fault_context, "last_mit": self.last_mit})
            if self.gripper.limits is not None:
                try:
                    robot._motor_map[robot.joint_names[6]].disable()
                except Exception as disable_error:
                    with self.telemetry_lock:
                        self.records.append({"event": "disable_error", "joint": 7, "reason": str(disable_error)})
            raise

    def _tick(self, robot, dt):
        tick_start = time.monotonic()
        self.runtime = {**self.runtime, "phase": "feedback", "last_begin": tick_start}
        # MotorBridge already has a background RX thread. Manual draining here
        # competes for recv_lock and is not part of a bounded control tick.
        rx = self.rx_provider()
        self.fault_context = {"rx": rx, "phase": "feedback"}
        rx_times = times_from_rx(rx, tick_start)
        if len(rx_times) != 7:
            raise RuntimeError("真实 CAN 反馈轴数异常")
        if None in rx_times:
            bad = rx_times.index(None) + 1
            raise RuntimeError(f"J{bad}真实CAN反馈超过150ms未更新")
        states = []
        for i, name in enumerate(robot.joint_names):
            self.runtime["phase"] = f"feedback:J{i+1}"
            states.append(robot._motor_map[name].get_state())
        if any(state is None for state in states):
            raise RuntimeError("反馈尚未就绪")
        positions = np.asarray([s.pos for s in states])
        raw_j7_position = float(positions[6])
        positions[6] = gripper_phase(raw_j7_position)
        velocities = np.asarray([s.vel for s in states])
        input_state = self.input_provider()
        self.fault_context = {"rx": rx, "q": positions.tolist(), "velocity": velocities.tolist(),
                              "j7_raw_position": raw_j7_position,
                              "driver_torque": [s.torq for s in states],
                              "status": [s.status_code for s in states], "input": input_state}
        if not np.isfinite(positions).all() or not np.isfinite(velocities).all():
            raise RuntimeError("关节反馈包含无效数值")
        now = time.perf_counter()
        actual_dt = dt if self.previous_time is None else now-self.previous_time
        self.fault_context["cycle_ms"]=actual_dt*1000
        self.previous_time = now
        if self.motion is None:
            self.motion = SmoothDrive(positions, self.limits, self.selected_axis)
            if self.teaching_config is not None:
                from .teaching import TeachingDirector
                self.teacher=TeachingDirector(positions.tolist(),[*self.limits,self.gripper.limits],self.teaching_config["signature"],self.teaching_config.get("initial_play"))
                from .teach_compliance import TeachCompliance
                self.teach_compliance=TeachCompliance()
        if self.teacher is not None:
            # Hand teaching never interprets sticks or requires a controller.
            input_state={**input_state,"connected":True,"deadman":False,"intent":[0.]*7,"triggers":{"lt":0.,"rt":0.}}
        if self.pause_requested.is_set():
            self.pause_requested.clear()
            self.motion.pause("控制完成心跳短暂超时")
            if self.teacher:self.teacher.hold(self.motion.target,"控制完成心跳短暂超时")
        if input_state["stop"]:
            self.stop_requested = True
        if self.stop_requested:
            input_state = {**input_state, "deadman": False, "intent": [0.0]*7}
        grip_enabled = self.gripper.limits is not None
        triggers = input_state.get("triggers")
        if grip_enabled and triggers is None:
            # Capability loss also requires a fresh neutral/deadman handshake.
            input_state = {**input_state, "connected": False}
        neutral=(input_state["connected"] and not input_state["deadman"]
                 and all(abs(v)<.08 for v in input_state["intent"])
                 and (triggers is None or triggers_neutral(triggers)))
        self.arm_guard.update(positions[:6],velocities[:6],self.motion.target[:6],
            [r["sequence"] for r in rx[:6]],tick_start,neutral,
            None if self.motion.continuous else self.motion.origin[:6],
            manual_drag=self.teacher is not None and self.teacher.mode=="DRAG")
        speed_holds=[]
        arm_input = {**input_state, "deadman": False, "intent": [0.]*7} if self.calibration else input_state
        if self.startup_elapsed<GRAVITY_STARTUP_SECONDS or self.arm_guard.active:
            arm_input={**arm_input,"connected":False}
        previous_target=self.motion.target[:]
        self.motion.step(arm_input, actual_dt, positions, speed_holds=speed_holds)
        if self.teacher:
            if self.arm_guard.active and self.teacher.mode!="HOLD":
                self.teacher.hold(previous_target,self.arm_guard.reason)
            targets=self.teacher.step(positions.tolist(),velocities.tolist(),actual_dt,
                [r["sequence"] for r in rx],ready=self.startup_elapsed>=GRAVITY_STARTUP_SECONDS and (self.teacher.mode!="DRAG" or self.gripper.torque_ff<1e-6),grip_contact=self.gripper.torque_ff>0.)
            self.motion.target=targets
            self.motion.velocity=[*self.teacher.velocity,0.]
            self.motion.driving=self.teacher.mode=="REPLAY"
            self.motion.need_rearm=False
            self.motion.status={"DRAG":"七轴手推示教 · J7无主动夹紧力","HOLD":"示教位置保持","REPLAY":"七轴低速回放"}[self.teacher.mode]
            error=float(targets[6]-positions[6])
            triggers={"rt":min(.4,max(0.,error/.05)) if error>.01 else 0.,
                      "lt":min(.4,max(0.,-error/.05)) if error<-.01 else 0.}
        if self.startup_elapsed<GRAVITY_STARTUP_SECONDS:
            self.motion.status="启动重力补偿渐入中，请继续支撑并保持输入回中"
        j7 = states[6]
        if grip_enabled and (j7.status_code not in (0, 1) or
                             not all(np.isfinite(v) for v in (j7.torq, j7.t_mos, j7.t_rotor)) or
                             max(j7.t_mos, j7.t_rotor) >= 65):
            raise RuntimeError("J7 夹爪驱动状态、力矩反馈或温度异常")
        input_ok = (input_state["connected"] and not self.stop_requested and
                    not self.stop_gripper.is_set() and (not self.motion.need_rearm or self.arm_guard.active) and j7.status_code == 1)
        motion_allowed = input_ok and (input_state["deadman"] if self.calibration else self.motion.driving)
        if self.teacher and self.teacher.mode=="DRAG":
            remaining=max(0.,self.gripper.torque_ff-2.*min(actual_dt,.02))
            self.gripper.hold.cancel("七轴手推示教")
            # A prior ceiling latch may only clear after a real manual opening,
            # not merely because the operator changes teaching modes.
            if remaining==0. and self.gripper.hold.close_locked and positions[6]<=self.gripper.hold.lock_position-np.deg2rad(2.):
                self.gripper.hold.close_locked=False
                self.gripper.hold.lock_position=None
            drag_kd=min(.1,max(0.,2.-remaining)/(abs(velocities[6])+.02))
            j7_estimate=self.gripper._apply(GripperCommand(kd=drag_kd,tau_ff=remaining,state="HAND_TEACH",status="J7手推示教 · 无主动夹紧力，物体需托住"),velocities[6])
        else:
            j7_estimate = self.gripper.update(position=positions[6], velocity=velocities[6],
                torque=j7.torq, triggers=triggers,
                armed=motion_allowed, hold_allowed=input_ok,
                dt=actual_dt, feedback_sequence=rx[6]["sequence"], feedback_age_ms=rx[6]["age_ms"],
                allow_grasp=not (self.teacher and self.teacher.home_move))
        if self.arm_guard.active:
            if self.motion.safety_hold(previous_target,self.arm_guard.reason,("arm",self.arm_guard.count)):
                with self.telemetry_lock:
                    self.records.append({"event":"local_pause","axes":self.arm_guard.axes,"at":time.monotonic(),
                        "reason":self.arm_guard.reason,"trigger":self.fault_context,"preserve_arm_support":True})
        elif self.gripper.speed_hold:
            if self.teacher and self.teacher.mode!="HOLD":self.teacher.hold(previous_target,self.gripper.pause_reason)
            if self.motion.safety_hold(previous_target,self.gripper.pause_reason,self.gripper.pause_count):
                with self.telemetry_lock:
                    self.records.append({"event":"local_pause","joint":7,"at":time.monotonic(),
                        "reason":self.gripper.pause_reason,"trigger":self.fault_context,
                        "pause_trigger":self.gripper.pause_trigger,
                        "preserve_arm_support":True,"last_mit":self.last_mit})
        if self.teacher and self.teacher.mode=="REPLAY" and self.gripper.command.state=="BLOCKED":
            self.teacher.hold(previous_target,"J7受阻，回放暂停，未增加夹紧力")
        trajectory_end = time.monotonic()
        self.runtime = {**self.runtime, "phase": "gravity"}
        gravity = self._gravity_comp_torque(positions[:6])
        if not np.isfinite(gravity).all() or np.any(np.abs(gravity) > np.array([28, 28, 28, 10, 10, 10])):
            raise RuntimeError("模型力矩超出协议范围")
        if self.filtered_gravity is None:
            self.filtered_gravity = gravity.copy()
        self.filtered_gravity += min(actual_dt/(0.03+actual_dt), 1.0)*(gravity-self.filtered_gravity)
        gravity_blend=startup_scale(self.startup_elapsed)
        applied_gravity=self.filtered_gravity*gravity_blend
        self.startup_elapsed+=min(actual_dt,.02)
        self._last_tau_g = applied_gravity.copy()
        elapsed = 0.0 if self._transition_started_at is None else now-self._transition_started_at
        blend = self._transition_blend(elapsed)
        kp = (1-blend)*self._hold_kp + blend*self.KP
        kd = (1-blend)*self._hold_kd + blend*self.KD
        if self.teacher:
            reference,kp_values,kd_values=self.teach_compliance.step(self.teacher.mode,
                self.motion.target[:6],positions[:6].tolist(),actual_dt,self._hold_kp,self._hold_kd)
            self.motion.target[:6]=reference
            kp,kd=np.asarray(kp_values),np.asarray(kd_values)
        target = np.asarray(self.motion.target[:6])
        vref = np.asarray(self.motion.velocity[:6])
        self.runtime = {**self.runtime, "phase": "tx"}
        gravity_end = time.monotonic()
        # JointGroup.send_mit suppresses CallError. Direct calls make any
        # failed command visible instead of reporting a successful tick.
        for i, name in enumerate(robot.arm.joint_names):
            try:
                robot._motor_map[name].send_mit(float(target[i]), float(vref[i]),
                    float(kp[i]), float(kd[i]), float(applied_gravity[i]))
            except Exception as exc:
                raise RuntimeError(f"J{i+1} MIT发送失败: {exc}") from exc
        # J7 speed regulation uses the motor's current velocity, not the
        # delayed/cached host measurement. Never issue host-computed torque.
        command = self.gripper.command
        if not grip_enabled or self.stop_gripper.is_set() or self.stop_requested:
            command = GripperCommand(status="J7 停止")
        j7_vref, j7_kd = command.velocity, command.kd
        try:
            robot._motor_map[robot.joint_names[6]].send_mit(*command.mit(raw_j7_position))
        except Exception as exc:
            raise RuntimeError(f"J7 MIT发送失败: {exc}") from exc
        completed = time.monotonic()
        self.latest_states = tuple(states)
        self.runtime = {"phase": "idle", "last_begin": tick_start, "last_complete": completed,
                        "feedback_trajectory_ms": (trajectory_end-tick_start)*1000,
                        "gravity_ms": (gravity_end-trajectory_end)*1000,
                        "tx_ms": (completed-gravity_end)*1000}
        self.last_mit = {"position": [*target, raw_j7_position], "velocity": [*vref, j7_vref],
                         "kp": [*kp, 0.0], "kd": [*kd, j7_kd], "tau": [*applied_gravity, command.tau_ff]}
        estimate = kp*(target-positions[:6])+kd*(vref-velocities[:6])+applied_gravity
        motion = {**self.motion.snapshot(), "gripper_ready": grip_enabled,
                  "teaching":self.teacher.snapshot() if self.teacher else None,
                  "arm_paused":self.arm_guard.active,"arm_pause_reason":self.arm_guard.reason,
                  "arm_pause_axes":self.arm_guard.axes,"gravity_startup_blend":gravity_blend,
                  "gripper_status": self.gripper.status,
                  "gripper_input_allowed":bool(input_ok),"gripper_motion_allowed":bool(motion_allowed),
                  "gripper_command_nm": command.tau_ff, "gripper_velocity_ref_rad_s": j7_vref,
                  "gripper_hold_state": command.state,
                  "gripper_paused":self.gripper.speed_hold,
                  "gripper_pause_reason":self.gripper.pause_reason,
                  "gripper_pause_count":self.gripper.pause_count,
                  "gripper_pause_trigger":self.gripper.pause_trigger,
                  "gripper_contact_motion":self.gripper.contact_motion_snapshot(float(positions[6])),
                  "gripper_close_locked": self.gripper.hold.close_locked,
                  "gripper_torque_limit_nm": getattr(self.gripper.hold,"torque_limit",None),
                  "gripper_force_target_n": 3. if self.gripper.hold.torque else None,
                  "gripper_measured_force_n": None, "gripper_calibration": self.calibration.snapshot() if self.calibration else None,
                  "gripper_estimated_torque_nm": float(j7_estimate), "gripper_control": CONTROL_NAME}
        motion["target_rad"][6] = float(positions[6])
        with self.telemetry_lock:
            self.telemetry = {**motion, "revision": REVISION,
                              "cycle_ms": actual_dt*1000, "last_tick": time.monotonic(),
                              "mit": self.last_mit, "rx": rx, "timing": self.runtime}
            if completed >= self.next_record or actual_dt > 0.025:
                self.next_record = completed+0.04
                self.records.append({"at": completed, "cycle_ms": actual_dt*1000,
                    "j7_raw_position": raw_j7_position,
                    "q": positions.tolist(), "velocity": velocities.tolist(),
                    "feedback": [s.torq for s in states], "status": [s.status_code for s in states],
                    "input": input_state, "target": self.motion.target[:],
                    "reference_velocity": self.motion.velocity[:], "mit": self.last_mit,
                    "estimated_total_torque": estimate.tolist(), "gravity_model": gravity.tolist(),
                    "rx": rx, "motion": motion, "timing": self.runtime})

    def request_timing_pause(self):
        # The monitor never writes targets or sends CAN from another thread.
        # The control tick consumes this latch before integrating any motion.
        self.pause_requested.set()

    def watchdog_status(self):
        # Read NOW, not a telemetry dictionary captured before slow SDK calls.
        runtime = self.runtime
        baseline = max(runtime["last_complete"], self.watchdog_started)
        return {**runtime, "age_ms": (time.monotonic()-baseline)*1000}

    def snapshot(self):
        with self.telemetry_lock:
            return dict(self.telemetry)

    def drain_records(self):
        with self.telemetry_lock:
            rows = list(self.records)
            self.records.clear()
            return rows
