"""Offline trajectory and inertial-plant tests. Never opens real input/CAN."""

import math
import unittest

from panel.gamepad_input import SimGamepad, map_intent, trigger_intent, triggers_neutral
from panel.teleop_motion import SmoothDrive, CONTINUOUS_SPEED, braking_margin


def pad(intent=None, held=False, connected=True):
    return {"intent": intent or [0.0]*7, "deadman": held, "connected": connected, "stop": False}


class TeleopTests(unittest.TestCase):
    def create(self, axis=0):
        drive = SmoothDrive([0.0]*7, [(-1.0, 1.0)]*6, axis)
        drive.step(pad(), 0.01)
        return drive

    def test_kp20_mapping(self):
        axes = {9: -32767, 10: -32767}
        self.assertEqual(map_intent(axes, {}), [0.0]*7)
        self.assertEqual(map_intent({**axes, 0: 32767}, {})[0], -1.0)
        self.assertEqual(map_intent({**axes, 1: -32767}, {})[1], -1.0)
        self.assertEqual(map_intent({**axes, 9: 32767}, {})[6], 1.0)  # RT closes
        self.assertEqual(map_intent({**axes, 10: 32767}, {})[6], -1.0)  # LT opens
        self.assertEqual(map_intent({9: 32767}, {})[6], 0.0)
        self.assertEqual(map_intent({10: 32767}, {})[6], 0.0)
        self.assertEqual(map_intent({9: 32767, 10: 20000}, {})[6], 0.0)

    def test_trigger_rearm_needs_two_valid_released_values(self):
        self.assertTrue(triggers_neutral({"lt": 0.09, "rt": 0.01}))
        self.assertFalse(triggers_neutral({"lt": 0.10, "rt": 0.0}))
        both = {"lt": 0.8, "rt": 0.5}
        self.assertFalse(triggers_neutral(both))
        self.assertEqual(trigger_intent(both), 0.0)
        for invalid in (None, {}, {"lt": 0.0}, {"lt": True, "rt": 0.},
                        {"lt": float("nan"), "rt": 1.}, {"lt": -0.1, "rt": 1.}):
            with self.subTest(triggers=invalid):
                self.assertFalse(triggers_neutral(invalid))
                self.assertEqual(trigger_intent(invalid), 0.0)

    def test_sim_triggers_use_same_mapping_and_clear_on_disconnect(self):
        gamepad = SimGamepad()
        gamepad.inject({"intent": [1.0]*7})
        self.assertEqual(gamepad.snapshot()["intent"], [1.0]*6 + [0.0])
        self.assertTrue(triggers_neutral(gamepad.snapshot()["triggers"]))
        gamepad.inject({"triggers": {"lt": 0.0, "rt": 0.7}, "deadman": True})
        self.assertEqual(gamepad.snapshot()["intent"][6], 0.7)
        gamepad.inject({"triggers": {"lt": 0.0, "rt": 0.7}, "deadman": True, "connected": False})
        disconnected = gamepad.snapshot()
        self.assertFalse(disconnected["deadman"])
        self.assertIsNone(disconnected["triggers"])
        self.assertEqual(disconnected["intent"], [0.0]*7)
        with self.assertRaises(ValueError):
            gamepad.inject({"triggers": {"rt": 1.0}})

    def test_j6_input_reversed_without_changing_other_axes(self):
        axes = {9: -32767, 10: -32767}
        self.assertEqual(map_intent(axes, {307: 1}), [0., 0., 0., 0., 0., 1., 0.])
        self.assertEqual(map_intent(axes, {308: 1}), [0., 0., 0., 0., 0., -1., 0.])
        self.assertEqual(map_intent(axes, {307: 1, 308: 1}), [0.]*7)

    def test_only_selected_axis_and_bounded_travel(self):
        drive = self.create(3)
        for _ in range(2000):
            drive.step(pad([1.0]*7, True), 0.01)
        for i, value in enumerate(drive.target):
            self.assertLessEqual(abs(value), math.radians(3)+1e-10)
            if i != 3:
                self.assertEqual(value, 0.0)
        self.assertGreater(drive.target[3], 0.04)

    def test_rearm_start_and_reconnect(self):
        drive = SmoothDrive([0.0]*7, [(-1.0, 1.0)]*6, 0)
        for _ in range(100):
            drive.step(pad([1.0]*7, True), 0.01)
        self.assertEqual(drive.target, [0.0]*7)
        drive.step(pad(), 0.01)
        for _ in range(30):
            drive.step(pad([1.0]*7, True), 0.01)
        for _ in range(100):
            drive.step(pad(connected=False), 0.01)
        target = drive.target[:]
        for _ in range(100):
            drive.step(pad([1.0]*7, True), 0.01)
        self.assertEqual(target, drive.target)
        self.assertTrue(drive.need_rearm)

    def test_release_holds_reference(self):
        drive = self.create()
        for _ in range(30):
            drive.step(pad([1.0]*7, True), 0.01)
        for _ in range(100):
            drive.step(pad(), 0.01)
        target = drive.target[:]
        for _ in range(100):
            drive.step(pad(), 0.01)
        self.assertEqual(drive.velocity, [0.0]*7)
        self.assertEqual(drive.target, target)

    def test_timing_pause_and_serious_overrun(self):
        drive = self.create()
        drive.step(pad([1.0]*7, True), 0.01)
        target = drive.target[:]
        drive.step(pad([1.0]*7, True), 0.08)
        self.assertEqual(target, drive.target)
        self.assertTrue(drive.need_rearm)
        self.assertEqual(drive.timing_holds, 1)
        with self.assertRaises(RuntimeError):
            drive.step(pad(), 0.3)

    def test_motor_local_loop_holds_during_host_stall(self):
        drive = self.create()
        q, v = 0.0, 0.0
        command_q, command_v = 0.0, 0.0
        peak_v = 0.0
        # 1 kHz motor loop, 100 Hz host loop, deliberate 100 ms host gap.
        for tick in range(3000):
            if tick % 10 == 0 and not 700 <= tick < 800:
                dt = 0.11 if tick == 800 else 0.01
                drive.step(pad([0.7]+[0.0]*6, tick < 1000), dt)
                command_q, command_v = drive.target[0], drive.velocity[0]
            torque = 20.0*(command_q-q)+1.5*(command_v-v)
            v += (torque-0.03*v)/0.08*0.001
            q += v*0.001
            peak_v = max(peak_v, abs(v))
            self.assertLess(abs(q), 0.07)
        self.assertLess(peak_v, 0.15)
        self.assertLess(abs(v), 1e-4)
        self.assertLess(abs(q-command_q), 1e-4)

    def test_gripper_not_selectable(self):
        with self.assertRaises(ValueError):
            self.create(6)

    def continuous(self, limits=None):
        drive = SmoothDrive([0.0]*7, limits or [(-1., 1.)]*6, None)
        drive.step(pad(), .01, [0.]*7)
        return drive

    def test_continuous_exceeds_three_degrees_without_restarts(self):
        drive = self.continuous()
        for _ in range(700):
            drive.step(pad([1.]*7, True), .01, drive.target[:])
        for i in range(6):
            self.assertGreater(drive.target[i], .5)
        self.assertEqual(drive.target[6], 0.)
        self.assertEqual(drive.snapshot()["mode"], "continuous")

    def test_no_open_loop_windup_when_motor_cannot_follow(self):
        drive = self.continuous()
        for _ in range(2000):
            drive.step(pad([1.]*7, True), .01, [0.]*7)
        self.assertTrue(all(0. < v <= .0600001 for v in drive.target[:6]))
        self.assertTrue(drive.tracking)
        for _ in range(400):
            drive.step(pad([-1.]*7, True), .01, [0.]*7)
        self.assertTrue(all(-.0600001 <= v < 0 for v in drive.target[:6]))

    def test_soft_boundary_only_blocks_outward_joint(self):
        drive = self.continuous([(-.2, .2)]+[(-1., 1.)]*5)
        for _ in range(1000):
            drive.step(pad([1.]*7, True), .01, drive.target[:])
        self.assertLessEqual(drive.target[0], .2)
        self.assertGreater(drive.target[1], .8)
        position = drive.target[0]
        for _ in range(200):
            drive.step(pad([-1.]+[0.]*6, True), .01, drive.target[:])
        self.assertLess(drive.target[0], position-.08)

    def test_center_release_and_reverse_are_continuous(self):
        for release_lb in (False, True):
            drive = self.continuous()
            for _ in range(150):
                drive.step(pad([1.]+[0.]*6, True), .01, drive.target[:])
            origin, previous = drive.target[0], drive.target[0]
            for _ in range(250):
                drive.step(pad(held=not release_lb), .01, drive.target[:])
                self.assertLessEqual(abs(drive.target[0]-previous), CONTINUOUS_SPEED[0]*.01+1e-10)
                previous = drive.target[0]
            self.assertEqual(drive.velocity[0], 0.)
            self.assertLess(drive.target[0]-origin, braking_margin(CONTINUOUS_SPEED[0]))
            for _ in range(250):
                drive.step(pad([-1.]+[0.]*6, True), .01, drive.target[:])
            self.assertLess(drive.target[0], origin-.05)

    def test_short_jitter_does_not_need_lb_rearm(self):
        drive = self.continuous()
        for _ in range(100):
            drive.step(pad([1.]+[0.]*6, True), .01, drive.target[:])
        before = drive.target[0]
        drive.step(pad([1.]+[0.]*6, True), .045, drive.target[:])
        self.assertFalse(drive.need_rearm)
        self.assertLessEqual(drive.target[0]-before, CONTINUOUS_SPEED[0]*.02+1e-10)
        drive.step(pad([1.]+[0.]*6, True), .11, drive.target[:])
        self.assertTrue(drive.need_rearm)

    def test_disconnected_does_not_resume_held_stick(self):
        drive = self.continuous()
        for _ in range(100):
            drive.step(pad([1.]*7, True), .01, drive.target[:])
        for _ in range(200):
            drive.step(pad(connected=False), .01, drive.target[:])
        held = drive.target[:]
        for _ in range(200):
            drive.step(pad([1.]*7, True), .01, drive.target[:])
        self.assertEqual(held, drive.target)

    def test_continuous_requires_actual_feedback(self):
        drive = self.continuous()
        with self.assertRaises(ValueError):
            drive.step(pad(), .01)

    def test_increased_speed_has_same_stop_and_feedback_guards(self):
        drive = self.continuous([(-3., 3.)]*6)
        for _ in range(200):
            drive.step(pad([1.]*7, True), .01, drive.target[:])
            for i in range(6):
                self.assertLessEqual(abs(drive.velocity[i]), CONTINUOUS_SPEED[i]+1e-10)
        for i in range(6):
            self.assertAlmostEqual(drive.velocity[i], CONTINUOUS_SPEED[i], places=3)
        for _ in range(200):
            drive.step(pad(), .01, drive.target[:])
        self.assertEqual(drive.velocity, [0.]*7)

    def test_external_timing_pause_holds_without_disabling_and_requires_rearm(self):
        drive = self.continuous()
        for _ in range(100):
            drive.step(pad([1.]*7, True), .01, drive.target[:])
        held = drive.target[:]
        drive.pause('injected 120ms completion gap')
        for _ in range(100):
            drive.step(pad([1.]*7, True), .01, drive.target[:])
        self.assertEqual(held, drive.target)
        self.assertTrue(drive.need_rearm)
        self.assertEqual(drive.velocity, [0.]*7)
        drive.step(pad(), .01, drive.target[:])
        self.assertFalse(drive.need_rearm)
        for _ in range(100):
            drive.step(pad([1.]*7, True), .01, drive.target[:])
        self.assertGreater(drive.target[0], held[0]+.05)

    def test_continuous_inertial_joint_with_friction_and_gravity_bias(self):
        drive = self.continuous()
        q, velocity, peak = 0., 0., 0.
        for tick in range(10000):
            if tick % 10 == 0:
                measured = [0., q, 0., 0., 0., 0., 0.]
                drive.step(pad([0., 1. if tick < 7000 else 0.]+[0.]*5, True), .01, measured)
            effort = 120*(drive.target[1]-q)+5*(drive.velocity[1]-velocity)-.9
            friction = math.copysign(.6, velocity if abs(velocity) > .001 else effort)
            if abs(velocity) < .001 and abs(effort) < .6:
                velocity = 0.
            else:
                velocity += (effort-friction)/.25*.001
            q += velocity*.001
            peak = max(peak, abs(velocity))
        self.assertGreater(q, .5)
        self.assertLess(peak, CONTINUOUS_SPEED[1]+.03)
        self.assertLess(peak, .5)
        self.assertLess(abs(q-drive.target[1]), math.radians(1))
        self.assertLess(abs(velocity), .002)

    def test_high_speed_boundary_brakes_before_endpoint(self):
        drive = self.continuous()
        saw_slow_wall = False
        for _ in range(2200):
            drive.step(pad([1.]+[0.]*6, True), .01, drive.target[:])
            distance = 1.-drive.target[0]
            self.assertGreaterEqual(distance, -1e-10)
            if distance < .015:
                saw_slow_wall = True
                self.assertLess(abs(drive.velocity[0]), .08)
        self.assertTrue(saw_slow_wall)


if __name__ == "__main__":
    unittest.main()
