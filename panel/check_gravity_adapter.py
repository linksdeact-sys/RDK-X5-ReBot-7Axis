"""Offline integration check with upstream dynamics and fake CAN handles only."""

import time
import json
from itertools import count
from math import radians
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import yaml
from reBotArm_control_py.controllers import GravityCompensation

from panel.gravity_mode import OfficialGravitySession
from panel.gamepad_input import SimGamepad


class FakeMotor:
    def __init__(self, position):
        self.state = SimpleNamespace(pos=position, vel=0.0, torq=0.0, status_code=0,
                                     t_mos=30.0, t_rotor=30.0)
        self.commands = []

    def ensure_mode(self, *args):
        return True

    def get_state(self):
        return self.state

    def request_feedback(self):
        pass

    def enable(self):
        self.state.status_code = 1

    def disable(self):
        self.state.status_code = 0

    def send_mit(self, pos, vel, kp, kd, tau):
        self.commands.append((pos, vel, kp, kd, tau))
        self.state.torq = tau


class FakeController:
    def __init__(self, motors):
        self.motors = motors

    def enable_all(self):
        for motor in self.motors:
            motor.state.status_code = 1

    def disable_all(self):
        for motor in self.motors:
            motor.state.status_code = 0

    def poll_feedback_once(self):
        pass


config = yaml.safe_load(Path("config/rebotarm_dm.yaml").read_text())
with patch("motorbridge.Controller.from_dm_device", side_effect=AssertionError("Real USB must stay closed")):
    baseline_motors = [FakeMotor(0.) for _ in range(7)]
    baseline = OfficialGravitySession(FakeController(baseline_motors), baseline_motors, config, [1.]*6)
    assert type(baseline.control) is GravityCompensation and baseline.status() is None
    baseline.start()
    try:
        time.sleep(.1)
        assert baseline.running and all(m.commands for m in baseline_motors)
    finally:
        baseline.close(gentle=True)
    assert all(m.state.status_code == 0 for m in baseline_motors)
print("PASS production default still uses official pure-gravity controller; no USB opened")

for scales in ([1.0]*6, [1.0, 1.0, 1.0, 0.5, 1.0, 1.0]):
    motors = [FakeMotor(value) for value in (0.0, -0.2, 0.3, 0.1, 0.0, 0.0, -0.1)]
    controller = FakeController(motors)
    sequence = count(1)

    def pure_rx_provider():
        return [{"age_ms": 0., "sequence": next(sequence)} for _ in motors]

    with patch("motorbridge.Controller.from_dm_device", side_effect=AssertionError("Real USB must stay closed")):
        session = OfficialGravitySession(controller, motors, config, scales, rx_provider=pure_rx_provider,
                                         experimental_gravity=True)
        assert session.robot.arm._mm is session.robot._motor_map
        try:
            session.start()
            time.sleep(0.6)
            assert session.running
            assert session.status()["rate_hz"] == 100
            assert all(motor.state.status_code == 1 for motor in motors)
            np.testing.assert_allclose(session.control._tau_scale, scales)
            np.testing.assert_allclose(session.torque, session.control._gravity_comp_torque(
                session.robot.arm.get_positions()))
            assert all(len(motor.commands) > 10 for motor in motors)
            assert all(not motor.commands[0][-1] for motor in motors)
            previous_kp = motors[1].commands[-1][2]
            session.close(gentle=True)
            assert not session.running
            assert all(motor.state.status_code == 0 for motor in motors)
            # Unload must begin at compliant stiffness, never the configured 120.
            assert motors[1].commands[-26][2] <= previous_kp + 1e-6
            assert all(motor.commands[-1][2:] == (0.0, 0.0, 0.0) for motor in motors)
        finally:
            if session.running:
                session.close(gentle=False)
print("PASS upstream controller, shared handles, both profiles, smooth unload, all-seven disable; no USB opened")

motors = [FakeMotor(value) for value in (0.0, -0.2, -0.3, 0.1, 0.0, 0.0, -0.1)]
controller = FakeController(motors)
gamepad = SimGamepad()
rx_fault = [False]*7
rx_sequence = count(1)


def rx_provider():
    seq = next(rx_sequence)
    return [{"age_ms": 200.0 if bad else 0.0, "sequence": seq} for bad in rx_fault]


with patch("motorbridge.Controller.from_dm_device", side_effect=AssertionError("Real USB must stay closed")):
    session = OfficialGravitySession(controller, motors, config, [1, 1, 1, 0.75, 1, 1],
                                     gamepad, None, 0, rx_provider)
    try:
        session.start()
        time.sleep(0.6)
        gamepad.inject({"deadman": True, "intent": [1.0]*7})
        time.sleep(0.3)
        assert session.running
        teleop = session.control.snapshot()
        assert teleop["driving"] and not teleop["gripper_ready"]
        assert teleop["target_rad"][0] > 0
        for i in range(1, 7):
            assert abs(teleop["target_rad"][i]-motors[i].state.pos) < 1e-10
        for i, motor in enumerate(motors[:6]):
            assert all(command[2] > 0 and command[3] > 0 for command in motor.commands)
            assert all(abs(command[1]) <= 0.05+1e-9 for command in motor.commands)
        rows = session.control.drain_records()
        assert rows and all(key in rows[-1] for key in ("input", "mit", "rx", "target", "cycle_ms", "estimated_total_torque"))
        json.dumps(rows)
        gamepad.inject({"connected": False})
        time.sleep(0.6)
        assert session.running and not session.control.snapshot()["driving"]
        gamepad.inject({"stop": True})
        time.sleep(0.03)
        assert session.control.stop_requested
        session.close(gentle=True)
        assert all(motor.state.status_code == 0 for motor in motors)
    finally:
        if session.running:
            session.close(gentle=False)
    gamepad.inject({})
    session = OfficialGravitySession(controller, motors, config, [1, 1, 1, 0.75, 1, 1],
                                     gamepad, None, 0, rx_provider)
    session.start()
    rx_fault[4] = True
    time.sleep(0.1)
    assert not session.running and "J5" in session.control.snapshot()["fault"]
    session.close(gentle=False)
    assert all(motor.state.status_code == 0 for motor in motors)
print("PASS local nonzero Kp/Kd, single-axis references, loss/rearm, RX-stale fault, complete logs and shutdown; no USB opened")

motors = [FakeMotor(value) for value in (0.0, -0.2, -0.3, 0.1, 0.0, 0.0, radians(145.3))]
gripper_motor = motors[6]
rx_fault[:] = [False]*7
gamepad = SimGamepad()
with patch("motorbridge.Controller.from_dm_device", side_effect=AssertionError("Real USB must stay closed")):
    session = OfficialGravitySession(FakeController(motors), motors, config, [1, 1, 1, .75, 1, 1],
                                     gamepad, (radians(60), radians(225)), None, rx_provider)
    try:
        session.start()

        def wait_until(condition):
            deadline = time.monotonic() + 2.
            while not condition():
                assert time.monotonic() < deadline, session.control.snapshot()
                time.sleep(.01)

        wait_until(lambda: session.control.snapshot().get("need_rearm") is False)
        gamepad.inject({"deadman": True, "triggers": {"rt": 1., "lt": 0.}})
        wait_until(lambda: gripper_motor.commands[-1][1] > .02)
        gamepad.inject({"deadman": True, "triggers": {"rt": 1., "lt": 1.}})
        wait_until(lambda: gripper_motor.commands[-1][1] == 0.)
        arm_origin = list(session.control.snapshot()["target_rad"][:6])
        gamepad.inject({"deadman": True, "intent": [1., -1., -1., 1., 1., 1., 0.],
                        "triggers": {"rt": 0., "lt": 1.}})
        wait_until(lambda: gripper_motor.commands[-1][1] < -.02)
        wait_until(lambda: all(abs(a-b) > .001 for a, b in zip(
            session.control.snapshot()["target_rad"][:6], arm_origin)))
        assert session.running and session.control.snapshot()["driving"]
        gamepad.inject({"connected": False})
        wait_until(lambda: gripper_motor.commands[-1][1] == 0.)
        assert all(kp == 0. and 0. < kd <= 1.2 and tau == 0. and abs(v) <= .375
                   for _, v, kp, kd, tau in gripper_motor.commands)
        # The observed speed jump now brakes J7 locally instead of creating a
        # cached-velocity reverse torque or ending an otherwise healthy session.
        gamepad.inject({"deadman": True, "triggers": {"rt": 0., "lt": 1.}})
        gripper_motor.state.vel = -1.106
        wait_until(lambda: gripper_motor.commands[-1][1:]==(0., 0., .4, 0.))
        assert session.running
        gripper_motor.state.vel = -.007
        gamepad.inject({})
        # The subsequent arm unload must not send another J7 command.
        session.control.stop_gripper.set()
        session.robot.stop_control_loop()
        before = len(gripper_motor.commands)
        session.close(gentle=True)
        assert len(gripper_motor.commands) == before
        assert all(m.state.status_code == 0 for m in motors)
    finally:
        if session.running:
            session.close(gentle=False)
    gamepad.inject({})
    gripper_motor.state.vel = -1.6
    session = OfficialGravitySession(FakeController(motors), motors, config, [1, 1, 1, .75, 1, 1],
                                     gamepad, (radians(60), radians(225)), None, rx_provider)
    try:
        session.start()
        wait_until(lambda: session.control.snapshot().get("gripper_paused"))
        assert session.running and all(m.state.status_code==1 for m in motors)
        event = next(row for row in session.control.drain_records() if row.get("event") == "local_pause")
        assert event["trigger"]["velocity"][6]==-1.6 and event["preserve_arm_support"]
        assert not session.control.snapshot().get("fault")
    finally:
        session.close(gentle=False)
print("PASS simultaneous J1-J6 and J7 LT in one session, J7 drive-local velocity, fresh-frame gating, soft brake, RT/LT and unload; no USB opened")
