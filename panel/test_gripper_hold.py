"""No USB: explicit grip confirmation, cancellation, bounds and manual force data."""
import math
import unittest

from panel.gripper_teleop import GripperTeleop
from panel.gripper_force import ForceCalibration, profile_torque
from panel.gripper_hold import HOLD_TORQUE_MAX


class HoldTests(unittest.TestCase):
    def setUp(self):
        self.grip = GripperTeleop((1.2, 3.9), hold_torque=.06)
        self.sequence, self.q = 0, 2.

    def tick(self, rt=0., lt=0., *, armed=True, allowed=True, fresh=True, q=None, v=0., tau=.06, dt=.01, age=1., triggers=None):
        if fresh: self.sequence += 1
        self.grip.update(position=self.q if q is None else q, velocity=v, torque=tau,
            triggers={"lt": lt, "rt": rt} if triggers is None else triggers, armed=armed,
            hold_allowed=allowed, dt=dt, feedback_sequence=self.sequence, feedback_age_ms=age)
        return self.grip.command

    def candidate(self):
        for i in range(40):
            self.q = 2.+i*.001
            self.tick(rt=1., v=.1)
        for _ in range(20):
            command = self.tick(rt=1.)
            if command.state == "CANDIDATE": break
        self.assertEqual(command.state, "CANDIDATE")
        self.assertEqual(command.tau_ff, 0.)

    def holding(self):
        self.candidate()
        for _ in range(40): self.tick(armed=False)
        self.assertEqual(self.grip.command.state, "HOLD")
        self.assertAlmostEqual(self.grip.command.tau_ff, .06)

    def test_neutral_without_contact_never_holds(self):
        for _ in range(50): self.tick()
        self.assertEqual(self.grip.command.tau_ff, 0.)
        self.assertNotEqual(self.grip.command.state, "HOLD")

    def test_contact_requires_motion_load_fresh_frames_and_confirmation(self):
        for _ in range(80): self.tick(rt=1.)
        self.assertNotEqual(self.grip.command.state, "CANDIDATE")
        self.setUp()
        for i in range(40):
            self.q = 2.+i*.001
            self.tick(rt=1., v=.1)
        for _ in range(50): self.tick(rt=1., fresh=False)
        self.assertNotEqual(self.grip.command.state, "CANDIDATE")
        self.setUp()
        self.candidate()
        for _ in range(50): self.assertEqual(self.tick(rt=1.).tau_ff, 0.)
        self.assertEqual(self.tick(armed=False).state, "HOLD")

    def test_release_rt_lb_both_preserves_hold_with_bounded_ramp(self):
        self.candidate()
        previous = 0.
        for _ in range(50):
            command = self.tick(armed=False)
            self.assertLessEqual(command.tau_ff-previous, .002+1e-10)
            previous = command.tau_ff
        self.assertAlmostEqual(command.tau_ff, .06)
        for _ in range(200):
            command = self.tick(armed=False)
            self.assertAlmostEqual(command.tau_ff, .06)
            self.assertEqual(command.velocity, 0.)
            self.assertEqual(command.mit(-2.4), (-2.4, 0., 0., .2, .06))

    def test_lt_without_lb_does_not_open_and_with_lb_releases_before_opening(self):
        self.holding()
        self.assertAlmostEqual(self.tick(lt=1., armed=False).tau_ff, .06)
        for _ in range(40):
            cmd = self.tick(lt=1.)
            if cmd.tau_ff > 0: self.assertEqual(cmd.velocity, 0.)
        self.assertEqual(cmd.tau_ff, 0.)
        self.assertLess(cmd.velocity, 0.)

    def test_disconnect_and_invalid_input_clear_hold(self):
        for kwargs in ({"allowed": False},
                       {"triggers": None, "allowed": False}, {"triggers": {"lt": 0., "rt": float("nan")}}):
            self.setUp()
            self.holding()
            self.assertEqual(self.tick(**kwargs).tau_ff, 0.)
            for _ in range(5): self.assertEqual(self.tick(rt=1.).tau_ff, 0.)

    def test_short_control_delay_soft_speed_and_soft_stale_unload_progressively(self):
        for kwargs in ({"dt":.11},{"age":121.},{"v":1.1}):
            self.setUp();self.holding()
            previous=self.grip.command.tau_ff
            cmd=self.tick(**kwargs)
            self.assertTrue(self.grip.speed_hold)
            self.assertGreater(cmd.tau_ff,0.)
            self.assertLess(cmd.tau_ff,previous)
            for _ in range(5):
                previous=cmd.tau_ff
                cmd=self.tick(rt=1.)
                self.assertLessEqual(cmd.tau_ff,previous)
                self.assertEqual(cmd.velocity,0.)

    def test_force_does_not_increase_on_cached_feedback(self):
        self.candidate()
        first = self.tick().tau_ff
        for _ in range(10): self.assertEqual(self.tick(fresh=False).tau_ff, first)

    def test_drift_endpoint_and_candidate_timeout_do_not_regrip(self):
        for q in (2.10, 1.2, 3.9):
            self.setUp()
            self.holding()
            self.assertEqual(self.tick(q=q).tau_ff, 0.)
            self.assertEqual(self.tick(rt=1., q=q).velocity, 0.)
        self.setUp()
        self.candidate()
        for _ in range(310): cmd = self.tick(rt=1.)
        self.assertEqual(cmd.tau_ff, 0.)
        self.assertEqual(cmd.velocity, 0.)

    def test_hard_fault_clears_command_before_raising(self):
        self.holding()
        with self.assertRaises(RuntimeError): self.tick(age=151.)
        self.assertEqual(self.grip.command.tau_ff, 0.)

    def test_dual_trigger_preserves_existing_hold_but_never_confirms_contact(self):
        self.candidate()
        self.assertEqual(self.tick(rt=1., lt=1.).tau_ff, 0.)
        self.tick()
        for _ in range(40): self.tick()
        self.assertAlmostEqual(self.tick(rt=1., lt=1.).tau_ff, .06)

    def test_invalid_hold_torque_rejected_and_uncommissioned_not_holding(self):
        for tau in (True, -.1, .21, float("nan")):
            with self.assertRaises(ValueError): GripperTeleop((1.2, 3.9), hold_torque=tau)
        self.assertEqual(HOLD_TORQUE_MAX, .2)


class ForceCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.points = {"open": 1.2, "closed": 3.9}
        self.cal = ForceCalibration(self.points)
        self.cal.tick(ready=True, safe=True, armed=False, dt=.01, position=2.)

    def pulse(self, tau=.06, *, steady=True):
        pulse = self.cal.request_pulse(tau)
        for _ in range(201):
            self.cal.tick(ready=True, safe=True, armed=True, dt=.01, position=2., command_tau=tau if steady else 0.)
        return pulse

    def test_three_independent_readings_and_endpoint_identity(self):
        for force in (2.9, 3., 3.1): self.cal.record(self.pulse(), force)
        payload = self.cal.payload()
        self.assertEqual(profile_torque(payload, self.points), .06)
        self.assertIsNone(profile_torque(payload, {"open": 1.3, "closed": 3.9}))
        self.assertIsNone(profile_torque({**payload, "hold_torque_nm": 3.}, self.points))
        self.assertEqual(payload["source"], "operator_gauge_single_opening")
        self.assertEqual(len(self.cal.snapshot()["recorded_pulse_ids"]), 3)

    def test_pulse_needs_contact_and_valid_torque(self):
        for tau in (0, .01, .201, .021, True, float("nan")):
            with self.assertRaises(ValueError): self.cal.request_pulse(tau)
        self.cal.tick(ready=False, safe=False, armed=False, dt=.01, position=2.)
        with self.assertRaises(RuntimeError): self.cal.request_pulse(.06)

    def test_deadman_loss_and_unstable_output_cannot_be_recorded(self):
        pulse = self.cal.request_pulse(.06)
        self.cal.tick(ready=True, safe=True, armed=False, dt=.01, position=2.)
        with self.assertRaises(RuntimeError): self.cal.record(pulse, 3.)
        with self.assertRaises(RuntimeError): self.cal.record(self.pulse(steady=False), 3.)

    def test_duplicate_and_out_of_range_measurements_rejected(self):
        pulse = self.pulse()
        for force in (None, 1., 4., True, float("nan")):
            with self.assertRaises(ValueError): self.cal.record(pulse, force)
        self.cal.record(pulse, 3.)
        with self.assertRaises(RuntimeError): self.cal.record(pulse, 3.)
        with self.assertRaises(RuntimeError): self.cal.payload()

    def test_changing_torque_resets_readings_and_abort_prevents_save(self):
        self.cal.record(self.pulse(), 3.)
        self.pulse(.07)
        self.assertEqual(self.cal.snapshot()["readings_n"], [])
        self.cal.abort()
        with self.assertRaises(RuntimeError): self.cal.payload()
