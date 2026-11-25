"""Gravity-only regression with synthetic feedback and in-memory MIT writes."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from panel.feedback_state import MAX_RX_AGE_MS
from panel.gravity_control import GravityBlend, PanelGravityController


POSE = np.array([0.1, -0.2, 0.3, 0.05, 0.0, -0.04])
MODEL = np.array([0.0, -4.0, -7.0, -1.5, 0.02, -0.01])
HOLD_KP = np.array([120., 120., 120., 18., 18., 18.])
HOLD_KD = np.array([5., 5., 5., 2., 2., 2.])
FREE_KP = np.full(6, 1.5)
FREE_KD = np.full(6, 1.0)


class GravityBlendTests(unittest.TestCase):
    def apply(self, ramp, now, dt=.01, pose=POSE, model=MODEL):
        return ramp.step(now, dt, pose, model, HOLD_KP, HOLD_KD, FREE_KP, FREE_KD)

    def test_first_tick_holds_measured_pose_without_feedforward_step(self):
        ramp = GravityBlend(.5)
        blend, target, kp, kd, torque = self.apply(ramp, 10.)
        self.assertEqual(blend, 0.)
        np.testing.assert_array_equal(target, POSE)
        np.testing.assert_array_equal(kp, HOLD_KP)
        np.testing.assert_array_equal(kd, HOLD_KD)
        np.testing.assert_array_equal(torque, np.zeros(6))

    def test_transition_is_monotone_and_preserves_steady_model(self):
        ramp = GravityBlend(.5)
        first = self.apply(ramp, 10.)
        moved = POSE + .03
        ticks = [first] + [self.apply(ramp, 10. + .01 * k, pose=moved) for k in range(1, 201)]
        self.assertTrue(all(a[0] <= b[0] for a, b in zip(ticks, ticks[1:])))
        self.assertTrue(all(np.all(np.abs(a[4]) <= np.abs(b[4]) + 1e-12)
                            for a, b in zip(ticks, ticks[1:])))
        blend, target, kp, kd, torque = ticks[-1]
        self.assertAlmostEqual(blend, 1.)
        np.testing.assert_allclose(target, moved)
        np.testing.assert_allclose(kp, FREE_KP)
        np.testing.assert_allclose(kd, FREE_KD)
        np.testing.assert_allclose(torque, MODEL)

    def test_filter_does_not_jump_to_a_new_model_value_after_large_dt(self):
        ramp = GravityBlend(.5, filter_tau=.03)
        for k in range(101):
            self.apply(ramp, 10. + .01 * k, model=np.zeros(6))
        _, _, _, _, torque = self.apply(ramp, 12., dt=1.)
        # The same tick should move toward the request but not bypass smoothing.
        np.testing.assert_array_less(np.abs(torque[1:]), .95 * np.abs(MODEL[1:]))
        self.assertTrue(np.all(torque[1:] * MODEL[1:] > 0))
        _, _, _, _, later = self.apply(ramp, 12.01)
        self.assertTrue(np.all(np.abs(later) >= np.abs(torque)))
        self.assertTrue(np.all(np.abs(later) <= np.abs(MODEL)))


class FakeMotor:
    def __init__(self, axis, position):
        self.state = SimpleNamespace(pos=position, vel=0., torq=0., status_code=1,
                                     t_mos=31., t_rotor=30.)
        self.send_mit = Mock(name=f"J{axis}_send_mit")
        self.request_feedback = Mock(name=f"J{axis}_request_feedback")
        self.ensure_mode = Mock(name=f"J{axis}_ensure_mode")
        self.enable = Mock(name=f"J{axis}_enable", side_effect=lambda: setattr(self.state, "status_code", 1))
        self.disable = Mock(name=f"J{axis}_disable")

    def get_state(self):
        return self.state


class GravityTickTests(unittest.TestCase):
    def setUp(self):
        names = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
        self.motors = [FakeMotor(i + 1, float(q)) for i, q in enumerate([*POSE, -.1])]
        self.robot = SimpleNamespace(
            joint_names=names,
            arm=SimpleNamespace(joint_names=names[:6], num_joints=6, _mit_kp=HOLD_KP,
                                _mit_kd=HOLD_KD),
            gripper=SimpleNamespace(joint_names=names[6:], num_joints=1,
                                     _mit_kp=np.array([8.]), _mit_kd=np.array([1.])),
            has_gripper=True,
            _motor_map=dict(zip(names, self.motors)),
            _ctrl_map={"damiao": SimpleNamespace(poll_feedback_once=self.receive_fresh_frames)},
            start_control_loop=Mock(side_effect=lambda callback, rate: callback(self.robot, 1. / rate)),
            stop_control_loop=Mock(),
        )
        self.rx = [{"sequence": 5, "age_ms": 8.} for _ in names]
        self.controller = PanelGravityController(self.robot, rx_provider=lambda: self.rx)
        self.controller._gravity_comp_torque = Mock(return_value=MODEL.copy())
        self.controller._hold_kp = HOLD_KP.copy()
        self.controller._hold_kd = HOLD_KD.copy()
        self.controller._compliant_kp = FREE_KP.copy()
        self.controller._compliant_kd = FREE_KD.copy()

    def receive_fresh_frames(self):
        for item in self.rx:
            item["sequence"] += 1

    def prepare_start(self):
        for motor in self.motors:
            motor.state.status_code = 0
        # This fixture uses a deterministic model; the upstream integration
        # check separately exercises _resolve_params / Pinocchio.
        self.controller._resolve_params = Mock()
        self.controller._joint_direction = np.ones(6)
        self.controller._tau_scale = np.array([1., 1., 1., .75, 1., 1.])

    def tick(self):
        self.controller._tick(self.robot, .01)

    def assert_no_mit(self):
        for motor in self.motors:
            motor.send_mit.assert_not_called()

    def test_successful_tick_sends_seven_and_publishes_commands(self):
        self.tick()
        for motor in self.motors:
            motor.send_mit.assert_called_once()
        self.assertEqual(self.motors[2].send_mit.call_args.args[-1], 0.)
        self.assertEqual(len(self.controller.latest_states), 7)
        self.assertEqual(len(self.controller.last_mit["tau"]), 7)
        snapshot = self.controller.snapshot()
        self.assertEqual(snapshot["model_nm"], MODEL.tolist())
        self.assertEqual(snapshot["sent_nm"], [0.] * 6)

    def test_stale_axis_stops_before_any_mit_write(self):
        self.rx[2]["age_ms"] = MAX_RX_AGE_MS + 1
        with self.assertRaisesRegex(RuntimeError, r"J3.*CAN"):
            self.tick()
        self.assert_no_mit()
        self.controller._gravity_comp_torque.assert_not_called()

    def test_nonfinite_joint_state_stops_before_any_mit_write(self):
        self.motors[1].state.pos = float("nan")
        with self.assertRaisesRegex(RuntimeError, r"J2"):
            self.tick()
        self.assert_no_mit()

    def test_invalid_model_is_rejected_even_before_ramp_in(self):
        for invalid_torque in (float("nan"), 11.):
            with self.subTest(torque=invalid_torque):
                model = MODEL.copy()
                model[3] = invalid_torque
                self.controller._gravity_comp_torque.return_value = model
                with self.assertRaisesRegex(RuntimeError, r"力矩|模型|J4"):
                    self.tick()
                self.assert_no_mit()

    def test_raw_tx_error_is_not_suppressed_and_stops_later_sends(self):
        self.motors[3].send_mit.side_effect = RuntimeError("fake USB TX failure")
        with self.assertRaisesRegex(RuntimeError, r"J4.*fake USB TX failure"):
            self.tick()
        for motor in self.motors[:4]:
            motor.send_mit.assert_called_once()
        for motor in self.motors[4:]:
            motor.send_mit.assert_not_called()
        self.assertIsNone(self.controller.last_mit)

    def test_start_is_safe_even_if_callback_runs_inside_thread_start(self):
        self.prepare_start()
        self.controller.start()
        self.robot.start_control_loop.assert_called_once()
        for i, motor in enumerate(self.motors):
            motor.enable.assert_called_once()
            pos, vel, kp, _, torque = motor.send_mit.call_args.args
            self.assertEqual(pos, motor.state.pos)
            self.assertEqual(vel, 0.)
            self.assertEqual(kp, HOLD_KP[i] if i < 6 else 8.)
            self.assertEqual(torque, 0.)

    def test_mode_confirmation_failure_prevents_any_enable(self):
        self.prepare_start()
        self.motors[2].ensure_mode.side_effect = RuntimeError("fake mode query timeout")
        with self.assertRaisesRegex(RuntimeError, r"J3.*fake mode query timeout"):
            self.controller.start()
        for motor in self.motors:
            motor.enable.assert_not_called()
        self.assert_no_mit()

    def test_loop_latches_fault_and_cannot_resume_after_fresh_feedback(self):
        self.rx[1]["age_ms"] = MAX_RX_AGE_MS + 1
        self.controller._loop_cb(self.robot, .01)
        self.assertIn("J2", self.controller.snapshot()["fault"])
        self.rx[1]["age_ms"] = 8.
        self.controller._loop_cb(self.robot, .01)
        self.assert_no_mit()

    def test_stale_feedback_on_stop_skips_more_mit_and_disables_all(self):
        self.prepare_start()
        self.controller.start()
        for motor in self.motors:
            motor.send_mit.reset_mock()
        self.rx[0]["age_ms"] = MAX_RX_AGE_MS + 1
        with self.assertRaisesRegex(RuntimeError, r"J1.*CAN"):
            self.controller.close(gentle=True)
        self.robot.stop_control_loop.assert_called_once()
        self.assert_no_mit()
        for motor in self.motors:
            motor.disable.assert_called_once()

    def test_soft_unload_is_monotone_and_disables_every_axis(self):
        self.prepare_start()
        with patch("panel.gravity_control.time.monotonic", return_value=10.) as clock:
            self.controller.start()
            for tick in range(1, 61):
                clock.return_value = 10. + .01 * tick
                self.tick()
            initial = self.controller.last_mit
            self.assertLess(initial["tau"][2], -6.)
            for motor in self.motors:
                motor.send_mit.reset_mock()
            with patch("panel.gravity_control.time.sleep"):
                self.controller.close(gentle=True)
        for i, motor in enumerate(self.motors):
            gains = [record.args[2] for record in motor.send_mit.call_args_list]
            torques = [record.args[-1] for record in motor.send_mit.call_args_list]
            self.assertGreater(len(gains), 1)
            self.assertEqual(gains[0], initial["kp"][i])
            self.assertEqual(gains[-1], 0.)
            self.assertEqual(torques[0], initial["tau"][i])
            self.assertEqual(torques[-1], 0.)
            self.assertTrue(all(a >= b for a, b in zip(gains, gains[1:])))
            self.assertTrue(all(abs(a) >= abs(b) for a, b in zip(torques, torques[1:])))
            motor.disable.assert_called_once()

    def test_disable_failure_does_not_skip_later_axes(self):
        self.prepare_start()
        self.controller.start()
        self.motors[2].disable.side_effect = RuntimeError("fake disable failure")
        with self.assertRaisesRegex(RuntimeError, r"J3.*fake disable failure"):
            self.controller.close(gentle=False)
        for motor in self.motors:
            motor.disable.assert_called_once()


if __name__ == "__main__":
    unittest.main()
