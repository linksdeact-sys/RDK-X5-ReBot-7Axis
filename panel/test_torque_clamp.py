"""No USB: J7 Nm ceiling, progressive RT clamp, and non-ratcheting latch."""
import unittest
from panel.gripper_teleop import GripperTeleop
from panel.gripper_torque import TORQUE_LIMIT_NM, TORQUE_TRIP_NM, CLAMP_SLEW_NM_S
from panel.gripper_hold import HOLD_TRAVEL


class TorqueClampTests(unittest.TestCase):
    def setUp(self):
        self.grip=GripperTeleop((1.2,3.9),torque_limited=True)
        self.seq,self.q=0,2.

    def tick(self,rt=0.,lt=0.,*,armed=True,allowed=True,fresh=True,v=0.,tau=.3,dt=.01):
        if fresh:self.seq+=1
        self.grip.update(position=self.q,velocity=v,torque=tau,
            triggers={"rt":rt,"lt":lt},armed=armed,hold_allowed=allowed,dt=dt,
            feedback_sequence=self.seq,feedback_age_ms=1.)
        return self.grip.command

    def contact(self):
        for i in range(50):
            self.q=2.+.001*i
            self.tick(rt=1.,v=.1)
        for _ in range(16): self.tick(rt=1.,tau=.3)
        self.assertEqual(self.grip.command.state,"CLAMPING")
        self.assertFalse(self.grip.hold.close_locked)
        self.assertGreater(self.grip.command.tau_ff,0.)
        self.assertLess(self.grip.command.tau_ff,.5)

    def cap(self):
        self.contact()
        for _ in range(400): self.tick(rt=1.,tau=.3)
        self.assertEqual(self.grip.command.tau_ff,TORQUE_LIMIT_NM)
        self.assertTrue(self.grip.hold.close_locked)

    def test_nm_limit_not_jaw_force_conversion_and_normal_close_not_two_nm(self):
        self.assertEqual(TORQUE_LIMIT_NM,2.)
        for i in range(100):
            self.q=2.+.001*i
            cmd=self.tick(rt=1.,v=.1)
            self.assertEqual(cmd.tau_ff,0.)
            self.assertLessEqual(cmd.kd*(cmd.velocity+.75),TORQUE_LIMIT_NM)
        self.assertEqual(cmd.kd,1.2)
        self.assertAlmostEqual(cmd.velocity,.375)

    def test_existing_effort_transferred_then_only_held_rt_can_ramp(self):
        self.contact()
        before=self.grip.command.tau_ff
        cmd=self.tick(rt=1.)
        self.assertGreater(cmd.tau_ff,before)
        self.assertLessEqual(cmd.tau_ff-before,CLAMP_SLEW_NM_S*.01+1e-12)
        self.assertEqual(cmd.kd,0.)
        held=cmd.tau_ff
        for _ in range(50): self.assertEqual(self.tick().tau_ff,held)
        for _ in range(50): self.assertEqual(self.tick(rt=1.,armed=False).tau_ff,held)
        self.assertGreater(self.tick(rt=1.).tau_ff,held)

    def test_small_rt_changes_rate_not_force_ceiling(self):
        self.contact()
        start=self.grip.command.tau_ff
        self.assertAlmostEqual(self.tick(rt=.5).tau_ff-start,CLAMP_SLEW_NM_S*.01*.5)

    def test_command_ceiling_latches_and_repeated_rt_never_ratchets(self):
        self.cap()
        for _ in range(4):
            for _ in range(20):
                cmd=self.tick(rt=1.)
                self.assertEqual((cmd.velocity,cmd.kd,cmd.tau_ff),(0.,0.,2.))
            self.tick(armed=False)
            self.assertTrue(self.grip.hold.close_locked)

    def test_feedback_ceiling_can_trip_early_without_promoting_force(self):
        self.contact()
        before=self.grip.command.tau_ff
        cmd=self.tick(rt=1.,tau=TORQUE_TRIP_NM)
        self.assertEqual(cmd.tau_ff,before)
        self.assertTrue(self.grip.hold.close_locked)

    def test_surprise_torque_before_any_travel_latches_with_zero_force(self):
        cmd=self.tick(rt=1.,tau=2.1)
        self.assertTrue(self.grip.hold.close_locked)
        self.assertEqual((cmd.velocity,cmd.tau_ff),(0.,0.))
        self.assertEqual(self.tick(rt=1.).tau_ff,0.)

    def test_cached_frame_never_increases_clamp(self):
        self.contact()
        before=self.grip.command.tau_ff
        for _ in range(100): self.assertEqual(self.tick(rt=1.,fresh=False).tau_ff,before)

    def test_below_limit_no_progress_remains_temporary_not_contact(self):
        for _ in range(160): self.tick(rt=1.,tau=.03,v=-.007326)
        self.assertEqual(self.grip.command.state,"BLOCKED")
        self.assertFalse(self.grip.hold.close_locked)
        self.assertEqual(self.grip.command.tau_ff,0.)
        self.tick()
        for _ in range(10): cmd=self.tick(rt=1.,tau=.03)
        self.assertGreater(cmd.velocity,0.)

    def test_lt_releases_then_only_physical_opening_unlocks(self):
        self.cap()
        anchor=self.q
        for _ in range(110):cmd=self.tick(lt=1.)
        self.assertEqual(cmd.tau_ff,0.)
        self.assertLess(cmd.velocity,0.)
        self.assertTrue(self.grip.hold.close_locked)
        for _ in range(15):self.assertEqual(self.tick(rt=1.).velocity,0.)
        self.q=anchor-HOLD_TRAVEL-.001
        self.tick(lt=1.)
        self.assertFalse(self.grip.hold.close_locked)
        for _ in range(5):cmd=self.tick(rt=1.,v=.01)
        self.assertGreater(cmd.velocity,0.)

    def test_reverse_rt_during_release_cannot_ratchet_or_reapply_force(self):
        self.cap()
        self.tick(lt=1.)
        for _ in range(120):cmd=self.tick(rt=1.)
        self.assertEqual((cmd.velocity,cmd.tau_ff),(0.,0.))
        self.assertTrue(self.grip.hold.close_locked)

    def test_lt_without_lb_does_not_release(self):
        self.cap()
        self.assertEqual(self.tick(lt=1.,armed=False).tau_ff,2.)

    def test_disconnect_cancels_force_and_preserves_ceiling_latch(self):
        self.cap()
        self.assertEqual(self.tick(allowed=False).tau_ff,0.)
        self.assertTrue(self.grip.hold.close_locked)

    def test_drift_pauses_without_abrupt_zero_and_ceiling_event_not_erased(self):
        self.contact()
        self.tick()
        before=self.grip.command.tau_ff
        self.q-=.04
        for _ in range(30):
            cmd=self.tick(tau=2.)
            if cmd.state=="PAUSED":break
            self.assertEqual(cmd.tau_ff,before)
        self.assertEqual(cmd.state,"PAUSED")
        self.assertAlmostEqual(cmd.tau_ff,before-.02)
        self.assertTrue(self.grip.hold.close_locked)

    def test_torque_policy_rejects_legacy_profile_overlay(self):
        with self.assertRaises(ValueError):
            GripperTeleop((1.2,3.9),torque_limited=True,hold_torque=.06)

    def test_software_endpoint_is_not_a_grasp(self):
        self.q=3.9
        for _ in range(30):cmd=self.tick(rt=1.)
        self.assertEqual(cmd.velocity,0.)
        self.assertEqual(cmd.tau_ff,0.)
        self.assertFalse(self.grip.hold.close_locked)
