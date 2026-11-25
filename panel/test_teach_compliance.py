import math
import unittest
from panel.teach_compliance import TeachCompliance,policy


class TeachComplianceTests(unittest.TestCase):
    def setUp(self):
        self.c=TeachCompliance();self.q=[0.]*6
        self.kp=[120.,120.,120.,18.,18.,18.];self.kd=[5.,5.,5.,2.,2.,2.]

    def step(self,mode='DRAG',q=None,dt=.01):
        return self.c.step(mode,q or self.q,q or self.q,dt,self.kp,self.kd)

    def settled(self):
        self.step()
        for _ in range(51):self.step()

    def test_first_pose_is_current_not_zero_and_first_gain_is_hold_gain(self):
        q=[.4]*6;target,kp,kd=self.step(q=q)
        self.assertEqual(target,q);self.assertEqual(kp,self.kp);self.assertEqual(kd,self.kd)

    def test_rebot_defaults_and_measured_follow_after_fade(self):
        self.settled();q=[.6]*6
        target,kp,kd=self.step(q=q)
        self.assertEqual(target,q);self.assertEqual(kp,[2.]*6);self.assertEqual(kd,[1.]*6)
        self.assertEqual(policy()['arm_drag_kp'],2.)

    def test_gain_fade_monotone_no_reset_on_each_frame(self):
        values=[self.step()[1][0] for _ in range(60)]
        self.assertTrue(all(a>=b for a,b in zip(values,values[1:])))
        self.assertEqual(values[-1],2.)
        self.assertGreater(values[1],119.)

    def test_hold_fades_from_current_gain_and_reverse_does_not_step(self):
        self.settled()
        _,kp,_=self.step('HOLD');self.assertEqual(kp,[2.]*6)
        for _ in range(12):self.step('HOLD')
        old=self.c.kp[:]
        _,kp,_=self.step('DRAG');self.assertEqual(kp,old)
        for _ in range(51):self.step('DRAG')
        self.assertEqual(self.c.kp,[2.]*6)

    def test_large_tick_does_not_skip_transition(self):
        self.step();self.step(dt=.8)
        self.assertAlmostEqual(self.c.elapsed,.02)
        self.assertGreater(self.c.kp[0],119.)

    def test_replay_does_not_retrigger_hold_blend_every_frame(self):
        self.settled();self.step('HOLD')
        for _ in range(51):self.step('HOLD')
        q=[.1]*6
        target,kp,kd=self.step('REPLAY',q=q)
        self.assertEqual(target,q);self.assertEqual(kp,self.kp);self.assertEqual(kd,self.kd)

    def test_invalid_feedback_or_gains_rejected(self):
        with self.assertRaises(ValueError):self.step('unknown')
        with self.assertRaises(ValueError):self.step(q=[math.nan]*6)
        with self.assertRaises(ValueError):self.c.step('DRAG',self.q,self.q,.01,self.kp,[8.]*6)
