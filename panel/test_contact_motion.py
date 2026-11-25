"""No hardware: reproduce HOLD compression without promoting clamp effort."""
import math
import unittest
from panel import test_torque_clamp as fixture
from panel.gripper_contact_motion import ContactMotionGuard,SETTLE_TRAVEL_RAD,policy


class ContactMotionTests(unittest.TestCase):
    setUp=fixture.TorqueClampTests.setUp
    tick=fixture.TorqueClampTests.tick
    contact=fixture.TorqueClampTests.contact
    cap=fixture.TorqueClampTests.cap

    def held(self):
        self.contact()
        for _ in range(175):self.tick(rt=1.)
        self.tick()
        return self.grip.command.tau_ff

    def test_recorded_positive_hold_motion_no_longer_cuts_force(self):
        force=self.held()
        self.assertGreater(force,1.)
        # Replay the observed positive velocity and >2deg compression.
        for _ in range(30):
            self.q+=.227106*.01
            cmd=self.tick(v=.227106)
            self.assertEqual(cmd.state,"HOLD")
            self.assertEqual(cmd.tau_ff,force)
            self.assertEqual((cmd.velocity,cmd.kd),(0.,0.))
        for _ in range(100):
            self.assertEqual(self.tick(armed=False).tau_ff,force)
        self.assertFalse(self.grip.speed_hold)

    def test_compression_budget_cannot_ratchet_on_neutral_or_repeated_rt_at_cap(self):
        self.cap();anchor=self.grip.hold.contact_motion.anchor
        self.q+=math.radians(10.)
        for _ in range(25):
            self.tick(rt=1.)
            self.tick(armed=False)
            self.assertEqual(self.grip.hold.contact_motion.anchor,anchor)
            self.assertEqual(self.grip.command.tau_ff,2.)
            self.assertTrue(self.grip.hold.close_locked)
            self.assertFalse(self.grip.speed_hold)
        self.q=anchor+SETTLE_TRAVEL_RAD+.01
        for _ in range(30):
            cmd=self.tick(rt=1.)
            if self.grip.speed_hold:break
        self.assertTrue(self.grip.speed_hold)
        self.assertIn("被动收拢",self.grip.pause_reason)
        self.assertAlmostEqual(cmd.tau_ff,1.98)
        self.assertTrue(self.grip.hold.close_locked)

    def test_reopening_compares_to_compression_peak_and_requires_confirmation(self):
        force=self.held()
        self.q+=math.radians(6.);self.tick()
        self.q-=math.radians(3.) # Still above original anchor, but reopened.
        self.tick();self.assertFalse(self.grip.speed_hold)
        for _ in range(30):
            cmd=self.tick()
            if self.grip.speed_hold:break
            self.assertEqual(cmd.tau_ff,force)
        self.assertTrue(self.grip.speed_hold)
        self.assertIn("反向张开",self.grip.pause_reason)
        detail=self.grip.pause_trigger["contact_motion"]
        self.assertGreater(detail["reopening_deg"],2.)
        self.assertGreaterEqual(detail["fresh_frames"],3)
        self.assertGreaterEqual(detail["suspect_ms"]+1e-8,150.)

    def test_single_reverse_spike_clears_without_releasing(self):
        force=self.held();q=self.q
        self.q-=.04;self.tick();self.tick()
        self.q=q
        for _ in range(40):self.assertEqual(self.tick().tau_ff,force)
        self.assertFalse(self.grip.speed_hold)

    def test_cached_feedback_cannot_confirm_or_move_anchor(self):
        force=self.held();anchor=self.grip.hold.contact_motion.anchor
        self.q-=.04;self.tick()
        for _ in range(40):self.assertEqual(self.tick(fresh=False).tau_ff,force)
        self.assertFalse(self.grip.speed_hold)
        self.assertEqual(self.grip.hold.contact_motion.anchor,anchor)
        self.tick();self.assertFalse(self.grip.speed_hold)
        self.tick();self.assertTrue(self.grip.speed_hold)

    def test_lt_has_priority_over_drift_and_unloads_instead_of_pause(self):
        force=self.held();self.q-=.08
        cmd=self.tick(lt=1.)
        self.assertFalse(self.grip.speed_hold)
        self.assertEqual(cmd.state,"RELEASING")
        self.assertAlmostEqual(cmd.tau_ff,force-.02)

    def test_true_endpoint_still_pauses_without_waiting_for_drift_confirmation(self):
        self.held();self.q=3.9
        self.tick()
        self.assertTrue(self.grip.speed_hold)
        self.assertIn("端点",self.grip.pause_reason)

    def test_hold_capture_does_not_erase_reverse_evidence(self):
        guard=ContactMotionGuard(2.)
        guard.update(2.1,.01,True,True)
        guard.capture_hold(2.05)
        self.assertIsNone(guard.update(2.05,.01,True))
        for _ in range(15):reason=guard.update(2.05,.01,True)
        self.assertIn("反向张开",reason)
        self.assertEqual(policy()['settle_travel_deg'],12.)
