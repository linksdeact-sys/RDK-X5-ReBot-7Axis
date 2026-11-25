"""Regression using fresh and repeated CAN feedback. No hardware access."""
import unittest

from panel.teleop_motion import SmoothDrive
from panel.wrist_speed_guard import WristSpeedGuard


def pad(j4=0., j5=0., held=True):
    return {"intent": [0., 0., 0., j4, j5, 0., 0.], "deadman": held,
            "connected": True, "stop": False}


class WristSpeedGuardTests(unittest.TestCase):
    def test_recorded_isolated_j5_high_sample_never_confirms_duration_by_replay(self):
        guard = WristSpeedGuard(5)
        self.assertFalse(guard.update(velocity=-.462, sequence=524996, received_at=10., neutral=False))
        for _ in range(10):
            self.assertTrue(guard.update(velocity=-.505, sequence=525006, received_at=10.10, neutral=False))
        self.assertTrue(guard.update(velocity=-.420, sequence=525016, received_at=10.20, neutral=True))
        # Re-reading this safe sample cannot release the brake either.
        self.assertTrue(guard.update(velocity=-.420, sequence=525016, received_at=10.20, neutral=True))
        self.assertTrue(guard.update(velocity=-.335, sequence=525026, received_at=10.30, neutral=False))
        self.assertFalse(guard.update(velocity=-.335, sequence=525026, received_at=10.30, neutral=True))

    def test_persistent_high_with_distinct_samples_still_faults(self):
        guard = WristSpeedGuard(4)
        self.assertTrue(guard.update(velocity=.51, sequence=41, received_at=5., neutral=False))
        self.assertTrue(guard.update(velocity=.52, sequence=42, received_at=5.03, neutral=False))
        with self.assertRaisesRegex(RuntimeError, "J4.*持续超速"):
            guard.update(velocity=.51, sequence=49, received_at=5.10, neutral=False)

    def test_severe_single_frame_faults_without_waiting(self):
        with self.assertRaisesRegex(RuntimeError, "J5.*明显超速"):
            WristSpeedGuard(5).update(velocity=-.66, sequence=42, received_at=5., neutral=False)

    def test_one_held_wrist_never_accumulates_a_target_or_stops_other_axis(self):
        measured = [0.] * 7
        motion = SmoothDrive(measured, [(-1., 1.)]*6, None)
        motion.step(pad(held=False), .01, measured)
        for _ in range(30):
            motion.step(pad(j4=.6, j5=1.), .01, measured)
        self.assertGreater(motion.target[4], 0.)
        remaining = motion.target[3]
        measured[4] = .002
        for _ in range(80):
            motion.step(pad(j4=.6, j5=1.), .01, measured, speed_holds=[4])
            self.assertEqual(motion.target[4], .002)
            self.assertEqual(motion.velocity[4], 0.)
        self.assertGreater(motion.target[3], remaining)
        self.assertEqual(motion.snapshot()["speed_hold_axes"], [5])
        motion.step(pad(j4=.6), .01, measured)
        self.assertEqual(motion.snapshot()["speed_hold_axes"], [])
        self.assertEqual(motion.target[4], .002)
