"""Run the upstream GravityCompensation with the panel's existing CAN handles."""

import time

import numpy as np

from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.controllers import GravityCompensation


class OfficialGravitySession:
    def __init__(self, controller, motors, config, scales, gamepad=None, gripper_limits=None,
                 selected_axis=0, rx_provider=None, check_start=None, *, experimental_gravity=False,
                 hold_torque=None, calibration=None, torque_limited=False, teaching=None):
        self.robot = RebotArm("rebotarm_dm.yaml")
        # JointGroup keeps references to these dictionaries. Updating them shares
        # one controller with telemetry and avoids opening the USB adapter twice.
        self.robot._ctrl_map["damiao"] = controller
        self.robot._motor_map.update({
            joint["name"]: motor for joint, motor in zip(config["joints"], motors)
        })
        self.robot._connected = True
        gravity = config.get("gravity_compensation", {})
        self.pure_gravity = gamepad is None
        self.experimental_gravity = self.pure_gravity and experimental_gravity
        control_type, extras = GravityCompensation, {}
        if self.experimental_gravity:
            from .gravity_control import PanelGravityController
            control_type, extras = PanelGravityController, {"rx_provider": rx_provider}
            if check_start is not None:
                extras["check_start"] = check_start
        if gamepad is not None:
            from .gamepad_control import GamepadGravityController
            control_type = GamepadGravityController
            extras = {"input_provider": gamepad.snapshot, "gripper_limits": gripper_limits,
                      "selected_axis": selected_axis, "rx_provider": rx_provider,
                      "hold_torque":hold_torque,"calibration":calibration,"torque_limited":torque_limited}
            if teaching is not None: extras["teaching"] = teaching
        self.control = control_type(
            self.robot,
            kp=float(gravity.get("kp", 2.0)),
            kd=float(gravity.get("kd", 1.0)),
            tau_scale=scales,
            joint_direction=gravity.get("joint_direction", 1.0),
            transition_duration=float(gravity.get("transition_duration", 0.5)),
            **extras,
        )

    def start(self):
        self.control.start()

    @property
    def running(self):
        return self.robot.control_loop_active

    @property
    def torque(self):
        values = self.control.last_tau_g
        return values.tolist() if len(values) == 6 else [0.0] * 6

    def status(self):
        return self.control.snapshot() if self.experimental_gravity else None

    def close(self, gentle=True):
        if self.experimental_gravity:
            self.control.close(gentle)
            return
        commanded_gripper = not self.pure_gravity and self.control.gripper.limits is not None
        if not self.pure_gravity:
            self.control.stop_gripper.set()
        try:
            if commanded_gripper:
                # Stop powered gripping before any slow arm-state queries or
                # arm-only unload. The driver timeout covers a wedged SDK call.
                self.robot.stop_control_loop()
                self.robot._motor_map[self.robot.joint_names[6]].disable()
                thread = getattr(self.robot, "_ctrl_thread", None)
                if thread is not None and thread.is_alive():
                    raise RuntimeError("遥操线程未退出，跳过卸力指令")
            if gentle and self.control._running:
                # Fade from the currently active compliant gains. Upstream end()
                # jumps back to stiff hold gains, which is unsuitable for unload.
                self.robot.stop_control_loop()
                started = self.control._transition_started_at
                elapsed = self.control._transition_duration if started is None else time.perf_counter()-started
                blend = self.control._transition_blend(elapsed)
                positions = self.robot.arm.get_positions().copy()
                gripper_positions = None if commanded_gripper else self.robot.gripper.get_positions().copy()
                torque = np.asarray(self.torque)
                kp = (1-blend)*self.control._hold_kp + blend*self.control._compliant_kp
                kd = (1-blend)*self.control._hold_kd + blend*self.control._compliant_kd
                gripper_kp = self.robot.gripper._mit_kp.copy()
                gripper_kd = self.robot.gripper._mit_kd.copy()
                local = getattr(self.control, "last_mit", None)
                if local is not None:
                    torque = np.asarray(local["tau"][:6])
                    kp, kd = np.asarray(local["kp"][:6]), np.asarray(local["kd"][:6])
                    gripper_kp, gripper_kd = np.asarray(local["kp"][6:]), np.asarray(local["kd"][6:])
                for tick in range(26):
                    u = tick / 25.0
                    fade = 1.0 - u*u*(3.0-2.0*u)
                    self.robot.arm.send_mit(positions, kp=kp*fade, kd=kd*fade, tau=torque*fade)
                    if not commanded_gripper:
                        self.robot.gripper.send_mit(
                            gripper_positions, kp=gripper_kp*fade, kd=gripper_kd*fade,
                            tau=np.zeros(1),
                        )
                    time.sleep(0.02)
        finally:
            self.robot.stop_control_loop()
            self.control._running = False
            self.robot.disable_all()
