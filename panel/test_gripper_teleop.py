"""No-I/O J7 local-velocity tests with deliberately batched feedback."""
import math
import unittest
from panel.gripper_teleop import (
    BRAKE_KD, CAN_MARGIN_DEG, HARD_SPEED_RAD_S, IDLE_KD, LOCAL_KD,
    MAX_SPEED_RAD_S, MAX_ACCEL_RAD_S2, ZERO_SPEED_TORQUE_ESTIMATE_NM,
    SOFT_SPEED_RAD_S, GripperTeleop, firmware_timeout_ms,
    validate_gripper_range, gripper_phase,
)

OPEN, INWARD, MID = map(math.radians, (60., 225., 140.))


class GripperTeleopTests(unittest.TestCase):
    def setUp(self):
        self.grip, self.seq = GripperTeleop((OPEN, INWARD)), 0

    def tick(self, *, lt=0., rt=0., position=MID, velocity=0., torque=0.,
             armed=True, dt=.01, fresh=True, age=0., triggers=None):
        if fresh:
            self.seq += 1
        return self.grip.update(position=position, velocity=velocity, torque=torque,
            triggers=triggers if triggers is not None else {"lt": lt, "rt": rt},
            armed=armed, dt=dt, feedback_sequence=self.seq, feedback_age_ms=age)

    def test_inner_range_valid_only(self):
        self.assertEqual(CAN_MARGIN_DEG, 5.)
        self.assertEqual(validate_gripper_range({"open": OPEN, "closed": INWARD}), (OPEN, INWARD))
        for points in (None, {}, {"open": OPEN}, {"open": INWARD, "closed": OPEN},
                       {"open": OPEN, "closed": OPEN+.02}, {"open": float("nan"), "closed": INWARD},
                       {"open": OPEN, "closed": "225"},
                       {"open": math.radians(49.3), "closed": math.radians(241.3)}):
            self.assertIsNone(validate_gripper_range(points))

    def test_watchdog_is_still_required(self):
        self.assertEqual(firmware_timeout_ms(4000), 200.)
        for raw in (None, True, 0, 1999, 5001, 2000.0):
            with self.assertRaises(RuntimeError):
                firmware_timeout_ms(raw)

    def test_whole_turn_encoder_equivalent_keeps_taught_arc_without_zero_write(self):
        for deg in (49.3, 76.75, 145.3, 221.83, 241.3):
            self.assertAlmostEqual(math.degrees(gripper_phase(math.radians(deg-360.))), deg)
            self.assertEqual(gripper_phase(math.radians(deg)), math.radians(deg))
        self.assertEqual(gripper_phase(0.), 0.)
        self.assertEqual(gripper_phase(math.radians(245)), math.radians(245))
        self.assertTrue(math.isnan(gripper_phase(float("nan"))))

    def test_uncommissioned_gripper_stays_inactive(self):
        self.grip = GripperTeleop()
        self.assertEqual(self.tick(rt=1.), 0.)
        self.assertEqual(self.grip.velocity_ref, 0.)

    def test_first_frame_no_step_and_rt_lt_direction(self):
        for lt, rt, sign in ((0., 1., 1.), (1., 0., -1.)):
            self.setUp()
            self.tick(lt=lt, rt=rt)
            self.assertEqual(self.grip.velocity_ref, 0.)
            for _ in range(15):
                self.tick(lt=lt, rt=rt)
            self.assertGreater(sign*self.grip.velocity_ref, 0.)
            self.assertLessEqual(abs(self.grip.velocity_ref), MAX_SPEED_RAD_S)
            self.assertEqual(self.grip.mit_kd, LOCAL_KD)
        self.assertAlmostEqual(LOCAL_KD*MAX_SPEED_RAD_S, ZERO_SPEED_TORQUE_ESTIMATE_NM)

    def test_speed_is_1p5x_without_raising_acceleration_or_protection(self):
        self.assertEqual(MAX_SPEED_RAD_S, .25*1.5)
        self.assertEqual(MAX_ACCEL_RAD_S2, .5)
        self.assertEqual(SOFT_SPEED_RAD_S, .9)
        self.assertEqual(HARD_SPEED_RAD_S, 1.5)
        self.assertLess(MAX_SPEED_RAD_S, SOFT_SPEED_RAD_S)
        self.assertEqual(LOCAL_KD, 1.2)

    def test_both_full_triggers_reach_new_speed_with_unchanged_ramp(self):
        for lt, rt, sign in ((1., 0., -1.), (0., 1., 1.)):
            self.setUp()
            previous = 0.
            for _ in range(100):
                self.tick(lt=lt, rt=rt, velocity=sign*.1)
                self.assertLessEqual(abs(self.grip.velocity_ref-previous), MAX_ACCEL_RAD_S2*.01+1e-12)
                previous = self.grip.velocity_ref
            self.assertAlmostEqual(self.grip.velocity_ref, sign*MAX_SPEED_RAD_S)

    def test_same_frame_never_increases_reference(self):
        for _ in range(10):
            self.tick(lt=1.)
        value = self.grip.velocity_ref
        for age in range(10, 120, 10):
            self.tick(lt=1., fresh=False, age=age)
            self.assertEqual(self.grip.velocity_ref, value)
        self.tick(lt=1., fresh=False, age=121.)
        self.assertEqual(self.grip.velocity_ref, 0.)
        self.assertTrue(self.grip.speed_hold)

    def test_one_new_frame_after_gap_has_bounded_step(self):
        self.tick(rt=1.)
        for _ in range(10):
            self.tick(rt=1., fresh=False, age=80.)
        self.tick(rt=1.)
        self.assertLessEqual(self.grip.velocity_ref, .025+1e-12)

    def test_release_dual_trigger_disconnect_reverse_and_late_tick_zero_motion(self):
        for released in ({}, {"armed": False, "rt": 1.}, {"rt": 1., "lt": 1.},
                         {"rt": 1., "dt": .11}, {"triggers": {"rt": float("nan"), "lt": 0.}},
                         {"lt": 1., "fresh": False}):
            self.setUp()
            for _ in range(10):
                self.tick(rt=1.)
            self.tick(**released)
            self.assertEqual(self.grip.velocity_ref, 0.)
            self.assertEqual(self.grip.mit_kd, IDLE_KD)

    def test_recorded_sudden_velocity_brakes_and_requires_real_released_feedback(self):
        for _ in range(10):
            self.tick(lt=1.)
        self.tick(lt=1., velocity=-1.106)
        self.assertTrue(self.grip.speed_hold)
        self.assertEqual(self.grip.velocity_ref, 0.)
        self.assertEqual(self.grip.mit_kd, BRAKE_KD)
        self.tick(velocity=-.1,armed=False)
        for _ in range(10):
            self.tick(velocity=-.1, fresh=False)
        self.assertTrue(self.grip.speed_hold)
        self.tick(lt=1., velocity=-.1)
        self.assertTrue(self.grip.speed_hold)
        self.tick(velocity=-.1,armed=False)
        self.assertFalse(self.grip.speed_hold)

    def test_speed_event_is_local_pause_and_invalid_frame_still_fault(self):
        self.assertEqual(HARD_SPEED_RAD_S, 1.5)
        for speed in (1.1,-1.1,1.6,-1.6):
            self.setUp()
            self.tick(rt=1.,velocity=speed)
            self.assertTrue(self.grip.speed_hold)
            self.assertEqual(self.grip.command.state,"PAUSED")
            self.assertEqual(self.grip.command.velocity,0.)
        for invalid in ({"velocity": float("nan")},
                        {"torque": float("nan")}, {"age": 151.}, {"age": -1.},
                        {"position": math.radians(245.)}):
            with self.assertRaises(RuntimeError):
                self.tick(rt=1., **invalid)

    def test_software_endpoints_never_command_further_outward(self):
        for _ in range(10):
            self.tick(rt=1.)
        self.tick(position=INWARD, rt=1.)
        self.assertEqual(self.grip.velocity_ref, 0.)
        self.tick(position=INWARD, lt=1.)
        self.tick(position=INWARD, lt=1.)
        self.assertLess(self.grip.velocity_ref, 0.)

    def test_stall_in_both_directions_latches_until_true_release(self):
        for lt, rt in ((0., 1.), (1., 0.)):
            self.setUp()
            for _ in range(180):
                self.tick(lt=lt, rt=rt)
            self.assertTrue(self.grip.blocked_direction)
            self.assertEqual(self.grip.velocity_ref, 0.)
            self.tick(lt=1., rt=1.)
            self.tick(lt=lt, rt=rt)
            self.assertTrue(self.grip.blocked_direction)
            self.tick()
            self.assertFalse(self.grip.blocked_direction)

    def test_inertial_plant_with_100ms_batches_and_breakaway_friction(self):
        # Characterization only, not a real gripper inertia identification.
        for inertia in (.004, .015, .025):
            grip = GripperTeleop((OPEN, INWARD))
            q, v, cached_q, cached_v, seq, peak = MID, 0., MID, 0., 1, 0.
            for ms in range(8000):
                if ms % 100 == 0:
                    cached_q, cached_v, seq = q, v, seq+10
                if ms % 10 == 0:
                    held = ms < 6000
                    grip.update(position=cached_q, velocity=cached_v, torque=0.,
                        triggers={"lt": 1. if held else 0., "rt": 0.}, armed=True, dt=.01,
                        feedback_sequence=seq, feedback_age_ms=float(ms % 100))
                effort = grip.mit_kd*(grip.velocity_ref-v)
                if abs(v) < .001 and abs(effort) <= .18:
                    v = 0.
                else:
                    friction = math.copysign(.035, v if abs(v) >= .001 else effort)
                    next_v = v + (effort-friction)*.001/inertia
                    # Capture rest when a finite step crosses zero below the
                    # static breakaway threshold. Coulomb friction cannot make
                    # an otherwise resting plant alternate direction by itself.
                    v = 0. if v*next_v <= 0. and abs(effort) <= .18 else next_v
                q += v*.001
                peak = max(peak, abs(v))
            self.assertLess(q, MID-.2)
            self.assertLess(peak, MAX_SPEED_RAD_S+.03)
            self.assertLess(abs(v), .003)
            self.assertFalse(grip.speed_hold)
