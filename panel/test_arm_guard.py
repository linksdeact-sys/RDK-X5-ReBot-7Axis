import unittest
from panel.arm_motion_guard import ArmMotionGuard,startup_scale

class ArmGuardTests(unittest.TestCase):
    def test_hand_drag_does_not_apply_teleop_soft_speed_or_tracking_error(self):
        for axis in range(6):
            guard=ArmMotionGuard();v=[0.]*6;v[axis]=-1.49
            for k in range(1,31):
                self.assertFalse(guard.update([.3]*6,v,[0.]*6,[k]*6,k*.01,False,manual_drag=True))
            self.assertEqual(guard.high_frames,[0]*6)
            self.assertEqual(guard.error_since,[None]*6)

    def test_hand_drag_hard_speed_and_counter_integrity_remain(self):
        for speed in (-1.5,1.5):
            guard=ArmMotionGuard()
            self.assertTrue(guard.update([0.]*6,[speed]*6,[0.]*6,[2]*6,1.,False,manual_drag=True))
            with self.assertRaises(RuntimeError):
                guard.update([0.]*6,[0.]*6,[0.]*6,[1]*6,1.01,False,manual_drag=True)

    def test_leaving_hand_drag_starts_new_automatic_confirmation(self):
        guard=ArmMotionGuard()
        for k in range(1,31):guard.update([0.]*6,[-1.]*6,[0.]*6,[k]*6,k*.01,False,manual_drag=True)
        self.assertFalse(guard.update([0.]*6,[-1.]*6,[0.]*6,[31]*6,.31,False))
        self.assertTrue(guard.update([0.]*6,[-1.]*6,[0.]*6,[32]*6,.37,False))

    def test_counter_rollback_is_not_a_recoverable_motion_event(self):
        guard=ArmMotionGuard()
        guard.update([0.]*6,[0.]*6,[0.]*6,[4]*6,1.,True)
        with self.assertRaises(RuntimeError):
            guard.update([0.]*6,[0.]*6,[0.]*6,[3]*6,2.,True)
    def test_all_axes_speed_becomes_pause_not_exception(self):
        for axis in range(6):
            guard=ArmMotionGuard();v=[0.]*6;v[axis]=-.857
            self.assertFalse(guard.update([0.]*6,v,[0.]*6,[1]*6,1.,False))
            self.assertTrue(guard.update([0.]*6,v,[0.]*6,[2]*6,1.06,False))
            self.assertEqual(guard.axes,[axis+1])
            for k in range(3,20):guard.update([0.]*6,[0.]*6,[0.]*6,[k]*6,1.+k*.1,False)
            self.assertTrue(guard.active)
            self.assertFalse(guard.update([0.]*6,[0.]*6,[0.]*6,[20]*6,3.,True))
    def test_repeated_cached_frame_cannot_recover(self):
        guard=ArmMotionGuard()
        guard.update([0.]*6,[1.6]*6,[0.]*6,[1]*6,1.,False)
        for _ in range(10):guard.update([0.]*6,[0.]*6,[0.]*6,[1]*6,2.,True)
        self.assertTrue(guard.active)
    def test_following_error_pauses_after_confirmation(self):
        guard=ArmMotionGuard()
        self.assertFalse(guard.update([.13]*6,[0.]*6,[0.]*6,[1]*6,1.,False))
        self.assertTrue(guard.update([.13]*6,[0.]*6,[0.]*6,[2]*6,1.21,False))
    def test_startup_zero_first_frame_monotone_full_after_half_second(self):
        scales=[startup_scale(i*.01) for i in range(60)]
        self.assertEqual(scales[0],0.)
        self.assertEqual(scales[50],1.)
        self.assertTrue(all(0<=a<=b<=1 for a,b in zip(scales,scales[1:])))
        self.assertLess(max(b-a for a,b in zip(scales,scales[1:])),.031)
