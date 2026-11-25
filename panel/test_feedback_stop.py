"""Motor-free regression of stale feedback and stop ordering."""

from pathlib import Path
import tempfile
from unittest import TestCase
from unittest.mock import Mock, patch
import time
import math

from panel.feedback_state import times_from_rx
from panel.gamepad_input import SimGamepad
from panel.server import Panel
from panel.test_usb_reconnect import FakeHardware


def step_job(request_id="qa-stop"):
    return {"joint": 1, "degrees": 1, "kind": "step", "owner": "qa", "request_id": request_id}


class FeedbackAndStopTests(TestCase):
    def setUp(self):
        with patch("panel.server.GamepadReader", SimGamepad):
            self.panel = Panel(standby=True)
        self.panel.standby = False
        self.panel.connection_ready = True
        self.hardware = FakeHardware()
        for state in self.hardware.states:
            state.status_code = 0
        self.panel.refresh(self.hardware, range(7))

    def test_stop_cancels_pending_and_blocks_subsequent_command(self):
        self.panel.submit(step_job())
        self.assertIsNotNone(self.panel.pending)
        self.panel.request_stop()
        self.assertIsNone(self.panel.pending)
        self.assertTrue(self.panel.snapshot()["connection"]["stopping"])
        with self.assertRaisesRegex(RuntimeError, "停止处理中"):
            self.panel.submit(step_job("second"))

    def test_stop_is_not_cleared_by_disable_handshake(self):
        self.panel.request_stop()
        self.assertTrue(self.panel.verify_disabled(self.hardware))
        self.assertTrue(self.panel.stop.is_set())
        self.assertEqual(self.hardware.enabled, [])

    def test_old_feedback_does_not_admit_new_motion(self):
        self.panel.feedback_at[0] = time.monotonic() - .3
        self.assertFalse(self.panel.snapshot()["joints"][0]["online"])
        with self.assertRaisesRegex(RuntimeError, "新鲜的 CAN"):
            self.panel.submit(step_job())

    def test_j7_equivalent_negative_encoder_phase_keeps_raw_and_taught_range(self):
        self.hardware.states[6].pos = math.radians(-138.17)
        self.panel.gripper_points = {"open": math.radians(76.75), "closed": math.radians(225.79)}
        self.panel.refresh(self.hardware, range(7))
        joint = self.panel.snapshot()["joints"][6]
        self.assertAlmostEqual(joint["angle"], 221.83)
        self.assertAlmostEqual(joint["raw_angle"], -138.17)
        self.assertTrue(self.panel.gripper_can_start())

    def test_preflight_requires_new_rx_sequence_not_just_cached_state(self):
        self.hardware.cached = True
        with self.assertRaisesRegex(RuntimeError, "等待新 CAN 反馈超时：J1"):
            self.panel.require_new_feedback(self.hardware, [0], require_heartbeat=False)
        self.assertEqual(self.hardware.enabled, [])

    def test_feedback_arriving_after_usb_loss_does_not_restore_cache(self):
        poll = self.hardware.poll
        def interrupted_poll(indices):
            result = poll(indices)
            self.panel.connection_lost("USB disconnected")
            return result
        self.hardware.poll = interrupted_poll
        self.panel.refresh(self.hardware, range(7))
        self.assertEqual(self.panel.states, [None] * 7)
        self.assertEqual(self.panel.feedback_at, [None] * 7)

    def test_unsafe_ages_are_not_fresh(self):
        self.assertEqual(times_from_rx([
            {"sequence": 5, "age_ms": float("nan")}, {"sequence": 5, "age_ms": -1},
            {"sequence": 0, "age_ms": 5}, {"sequence": 5, "age_ms": 151},
            {"sequence": 5, "age_ms": 0}], 10), [None, None, None, None, 10])

    def test_hard_fault_does_not_send_unload_position_commands(self):
        def enable(index):
            self.hardware.states[index].status_code = 1
            self.hardware.states[index].t_mos = 70.
        self.hardware.enable = Mock(side_effect=enable)
        self.hardware.command = Mock()
        self.hardware.probe = Mock(side_effect=AssertionError("register transaction inside jog"))
        self.panel.submit(step_job())
        job = self.panel.pending
        self.panel.active, self.panel.pending = job, None
        with tempfile.TemporaryDirectory() as temp:
            with patch("panel.server.ROOT", Path(temp) / "panel"):
                self.panel.perform(self.hardware, job)
        self.hardware.enable.assert_called_once_with(0)
        self.hardware.command.assert_not_called()
        self.hardware.probe.assert_not_called()
        self.assertEqual(self.hardware.disabled, [0])
        self.assertIn("温度", self.panel.last_result["error"])
