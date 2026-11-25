import unittest
from panel import test_torque_clamp as fixture
from panel.arm_motion_guard import ArmMotionGuard

class ExperienceTests(unittest.TestCase):
    setUp=fixture.TorqueClampTests.setUp
    tick=fixture.TorqueClampTests.tick
    contact=fixture.TorqueClampTests.contact
    def test_sixty_and_ninety_ms_ticks_do_not_release_existing_hold(self):
        self.contact();self.tick()
        force=self.grip.command.tau_ff
        for dt in (.06,.09,.1):
            cmd=self.tick(dt=dt)
            self.assertFalse(self.grip.speed_hold)
            self.assertEqual(cmd.tau_ff,force)
    def test_control_trigger_records_exact_values(self):
        self.contact();self.tick()
        self.tick(dt=.123456)
        self.assertTrue(self.grip.speed_hold)
        self.assertAlmostEqual(self.grip.pause_trigger['cycle_ms'],123.456)
        self.assertEqual(self.grip.pause_trigger['feedback_age_ms'],1.)
    def test_previous_rebound_speed_below_new_soft_gate_does_not_pause(self):
        self.tick(rt=1.,v=-.5348,tau=.1)
        self.assertFalse(self.grip.speed_hold)
        self.tick(rt=1.,v=-.95,tau=.1)
        self.assertTrue(self.grip.speed_hold)
    def test_arm_single_spike_is_ignored_but_persistent_and_hard_events_pause(self):
        guard=ArmMotionGuard();q=[0.]*6;v=[0.]*6;v[3]=-.857
        self.assertFalse(guard.update(q,v,q,[1]*6,1.,False))
        for _ in range(10):self.assertFalse(guard.update(q,v,q,[1]*6,1.1,False))
        self.assertTrue(guard.update(q,v,q,[2]*6,1.1,False))
        guard=ArmMotionGuard();v[3]=-1.6
        self.assertTrue(guard.update(q,v,q,[1]*6,1.,False))
