"""No USB: replay recoverable J7 events without dropping arm support."""
import unittest
from panel.gripper_teleop import GripperTeleop,HARD_SPEED_RAD_S,SOFT_SPEED_RAD_S
from panel.teleop_motion import SmoothDrive
from panel import test_torque_clamp as fixture


class GripperPauseTests(unittest.TestCase):
    setUp=fixture.TorqueClampTests.setUp
    tick=fixture.TorqueClampTests.tick
    contact=fixture.TorqueClampTests.contact
    cap=fixture.TorqueClampTests.cap
    def test_recorded_speeds_and_new_hard_threshold_pause_never_raise(self):
        self.assertEqual(HARD_SPEED_RAD_S,1.5)
        for v in (-1.106228,-1.6,1.6):
            self.setUp();self.contact()
            force=self.grip.command.tau_ff
            cmd=self.tick(rt=1.,v=v)
            self.assertTrue(self.grip.speed_hold)
            self.assertEqual(cmd.state,"PAUSED")
            self.assertEqual(cmd.velocity,0.)
            self.assertAlmostEqual(cmd.tau_ff,force-.02)
            self.assertLessEqual(cmd.tau_ff+cmd.kd*abs(v),2.)
            self.assertIn("硬超速" if abs(v)>1.5 else "软超速",self.grip.pause_reason)

    def test_pause_force_is_monotone_bounded_and_cannot_reapply_with_rt(self):
        self.cap();previous=self.grip.command.tau_ff
        for i in range(150):
            cmd=self.tick(rt=1.,v=-1.6 if i<10 else 0.)
            self.assertTrue(self.grip.speed_hold)
            self.assertLessEqual(cmd.tau_ff,previous)
            self.assertLessEqual(previous-cmd.tau_ff,.02+1e-12)
            self.assertLessEqual(cmd.tau_ff+cmd.kd*(1.6 if i<10 else 0.),2.)
            previous=cmd.tau_ff
        self.assertEqual(cmd.tau_ff,0.)
        self.assertTrue(self.grip.hold.close_locked)

    def test_recovery_requires_new_low_speed_feedback_and_lb_release(self):
        self.tick(rt=1.,v=1.6)
        for _ in range(10):self.tick(rt=1.,v=0.)
        for _ in range(10):self.tick(armed=True,v=0.)
        self.assertTrue(self.grip.speed_hold)
        self.tick(armed=False,fresh=False)
        self.assertTrue(self.grip.speed_hold)
        self.tick(armed=False)
        self.assertFalse(self.grip.speed_hold)
        self.assertEqual(self.grip.command.velocity,0.)

    def test_normal_compression_does_not_trigger_old_two_degree_force_cut(self):
        self.contact()
        for _ in range(10):
            self.q+=.01
            cmd=self.tick(rt=1.,v=.1)
            self.assertEqual(cmd.state,"CLAMPING")
            self.assertFalse(self.grip.speed_hold)
        self.tick() # Capture the final compression position on RT release.
        anchor=self.q
        for _ in range(30):self.tick()
        self.assertFalse(self.grip.speed_hold)
        before=self.grip.command.tau_ff
        self.q=anchor-.04
        for _ in range(30):
            cmd=self.tick()
            if self.grip.speed_hold:break
            self.assertEqual(cmd.tau_ff,before)
        self.assertTrue(self.grip.speed_hold)
        self.assertAlmostEqual(cmd.tau_ff,before-.02)

    def test_true_stale_feedback_and_invalid_data_still_fault_during_pause(self):
        self.tick(rt=1.,v=1.6)
        with self.assertRaises(RuntimeError):
            self.grip.update(position=self.q,velocity=0.,torque=0.,triggers={"rt":0.,"lt":0.},
                armed=False,dt=.01,feedback_sequence=self.seq+1,feedback_age_ms=151.)

    def test_trajectory_freezes_old_safe_reference_and_cannot_resume_held_input(self):
        q=[0.]*6+[2.]
        motion=SmoothDrive(q,[(-2.,2.)]*6,None)
        neutral={"connected":True,"deadman":False,"intent":[0.]*7,"triggers":{"rt":0.,"lt":0.}}
        held={**neutral,"deadman":True,"intent":[1.]*6+[0.]}
        motion.step(neutral,.01,q)
        for _ in range(20):motion.step(held,.01,q)
        frozen=motion.target[:]
        motion.step(held,.01,q)
        self.assertTrue(motion.safety_hold(frozen,"J7超速",1))
        for _ in range(20):
            previous=motion.target[:]
            motion.step(held,.01,q)
            motion.safety_hold(previous,"J7超速",1)
        self.assertEqual(motion.target[:6],frozen[:6])
        self.assertEqual(motion.velocity,[0.]*7)
        for _ in range(20):motion.step(held,.01,q)
        self.assertFalse(motion.driving)
        motion.step(neutral,.01,q)
        for _ in range(10):motion.step(held,.01,q)
        self.assertTrue(motion.driving)
