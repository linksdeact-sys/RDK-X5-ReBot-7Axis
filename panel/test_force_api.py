"""No-I/O API interlocks and independent force-profile persistence."""
from pathlib import Path
import tempfile
import unittest
import json
from unittest.mock import patch

from panel.server import Panel
from panel.gamepad_input import SimGamepad
from panel.teleop_motion import CONTROL_REVISION
from panel.test_usb_reconnect import FakeHardware
from panel.gripper_force import ForceCalibration, profile_torque


class ForceApiTests(unittest.TestCase):
    def setUp(self):
        with patch("panel.server.GamepadReader", SimGamepad):
            self.panel = Panel(standby=True)
        self.panel.standby = False
        self.panel.connection_ready = True
        self.hw = FakeHardware()
        for state in self.hw.states: state.status_code = 0
        self.hw.states[6].pos = 2.
        self.panel.refresh(self.hw, range(7))
        self.panel.gripper_points = {"open": 1.2, "closed": 3.9}
        self.temp = tempfile.TemporaryDirectory()
        self.panel.force_file = Path(self.temp.name)/"gripper_hold.json"

    def tearDown(self):
        self.panel.gamepad.close()
        self.temp.cleanup()

    def begin(self, **extra):
        return self.panel.hold_calibration({"action": "begin", "owner": "qa", "request_id": "force-qa",
            "input_verified": True, "supported": True, "safety_revision": CONTROL_REVISION, **extra})

    def complete(self):
        cal = ForceCalibration(self.panel.gripper_points)
        cal.tick(ready=True, safe=True, armed=False, dt=.01, position=2.)
        for force in (2.9, 3., 3.1):
            pulse = cal.request_pulse(.06)
            for _ in range(201): cal.tick(ready=True, safe=True, armed=True, dt=.01, position=2., command_tau=.06)
            cal.record(pulse, force)
        return cal

    def test_begin_requires_support_current_revision_idle_and_neutral(self):
        with self.assertRaises(ValueError): self.begin(supported=False)
        with self.assertRaises(ValueError): self.begin(safety_revision="continuous-mit-v4")
        self.panel.gamepad.inject({"deadman": True})
        with self.assertRaises(RuntimeError): self.begin()
        self.panel.gamepad.inject({})
        self.begin()
        self.assertTrue(self.panel.pending["gripper_calibration"])
        self.assertTrue(self.panel.pending["gripper_enabled"])
        self.assertEqual(self.panel.pending["gripper_policy"], "calibrated")
        self.assertEqual(self.hw.enabled, [])

    def test_normal_gripper_defaults_torque_limit_without_force_calibration_or_sdk(self):
        self.panel.submit({"kind":"gamepad", "joint":1, "degrees":1,
            "owner":"qa", "request_id":"light-qa", "input_verified":True,
            "safety_revision":CONTROL_REVISION, "gripper_enabled":True})
        self.assertEqual(self.panel.pending["gripper_policy"], "torque-limit")
        self.assertFalse(self.panel.pending["gripper_calibration"])
        self.assertEqual(self.hw.enabled, [])

    def test_old_newton_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            self.panel.submit({"kind":"gamepad","joint":1,"degrees":1,"owner":"qa",
                "request_id":"old-light","input_verified":True,"safety_revision":CONTROL_REVISION,
                "gripper_enabled":True,"gripper_policy":"light"})

    def test_gripper_policy_is_explicit_and_rejects_unknown_values(self):
        body={"kind":"gamepad", "joint":1, "degrees":1, "owner":"qa",
              "request_id":"mode-qa", "input_verified":True,
              "safety_revision":CONTROL_REVISION, "gripper_enabled":True}
        with self.assertRaises(ValueError):
            self.panel.submit({**body,"gripper_policy":"unlimited"})
        self.assertIsNone(self.panel.pending)
        self.panel.submit({**body,"gripper_policy":"calibrated"})
        self.assertEqual(self.panel.pending["gripper_policy"],"calibrated")
        self.assertEqual(self.hw.enabled, [])

    def test_pulse_needs_owner_deadman_and_contact_no_sdk_call(self):
        self.begin()
        self.panel.active, self.panel.pending = self.panel.pending, None
        body = {"action": "pulse", "owner": "qa", "torque_nm": .06}
        with self.assertRaises(RuntimeError): self.panel.hold_calibration(body)
        self.panel.gamepad.inject({"deadman": True})
        with self.assertRaises(RuntimeError): self.panel.hold_calibration(body)
        self.panel.gripper_calibration.tick(ready=True, safe=True, armed=True, dt=.01, position=2.)
        with self.assertRaises(RuntimeError): self.panel.hold_calibration({**body, "owner": "different"})
        self.assertIn("pulse_id", self.panel.hold_calibration(body))
        self.assertEqual(self.hw.enabled, [])
        self.assertEqual(self.hw.disabled, [])

    def test_commit_is_deferred_until_disable_and_never_saves_on_http_start(self):
        self.begin()
        self.panel.active, self.panel.pending = self.panel.pending, None
        self.panel.gripper_calibration = self.complete()
        with self.assertRaises(ValueError):
            self.panel.hold_calibration({"action": "commit", "owner": "qa"})
        self.panel.hold_calibration({"action": "commit", "owner": "qa", "supported": True})
        self.assertTrue(self.panel.stop.is_set())
        self.assertTrue(self.panel.gripper_calibration.commit_requested)
        self.assertFalse(self.panel.force_file.exists())
        self.assertEqual(self.hw.enabled, [])

    def test_saved_profile_independent_of_endpoints_and_never_real_time_force(self):
        cal = self.complete()
        before = dict(self.panel.gripper_points)
        self.panel.save_force_profile(cal)
        self.assertEqual(self.panel.gripper_points, before)
        self.assertEqual(profile_torque(self.panel.force_profile, before), .06)
        state = self.panel.snapshot()["gripper"]
        self.assertTrue(state["hold_calibrated"])
        self.assertEqual(state["nominal_force_n"], 3.)
        self.assertIsNone(state["measured_force_n"])
        self.panel.gripper_points = {**before, "closed": 3.8}
        self.assertFalse(self.panel.snapshot()["gripper"]["hold_calibrated"])

    def test_save_rejects_enabled_stale_or_changed_endpoints(self):
        cal = self.complete()
        self.panel.states[0]["status_code"] = 1
        with self.assertRaises(RuntimeError): self.panel.save_force_profile(cal)
        self.assertFalse(self.panel.force_file.exists())
        self.panel.states[0]["status_code"] = 0
        self.panel.feedback_at[0] = None
        with self.assertRaises(RuntimeError): self.panel.save_force_profile(cal)
        self.assertFalse(self.panel.force_file.exists())

    def test_save_failure_preserves_readings_for_retry(self):
        cal = self.complete()
        self.panel.force_file = Path(self.temp.name)/"missing"/"hold.json"
        with self.assertRaises(OSError): self.panel.save_force_profile(cal)
        self.assertEqual(len(cal.snapshot()["readings_n"]), 3)
        self.assertFalse(cal.committed)

    def test_abort_preserves_existing_profile(self):
        old = self.complete()
        self.panel.save_force_profile(old)
        original = self.panel.force_file.read_bytes()
        self.begin()
        self.panel.active, self.panel.pending = self.panel.pending, None
        self.panel.hold_calibration({"action": "abort", "owner": "qa"})
        self.assertTrue(self.panel.stop.is_set())
        self.assertEqual(self.panel.force_file.read_bytes(), original)

    def test_restart_loads_parameters_only_and_never_connects_usb(self):
        folder = Path(self.temp.name)
        payload = self.complete().payload()
        (folder/"gripper_hold.json").write_text(json.dumps(payload))
        (folder/"gripper_limits.json").write_text(json.dumps(self.panel.gripper_points))
        with patch("panel.server.ROOT", folder), patch("panel.server.GamepadReader", SimGamepad), \
                patch("panel.server.Hardware", side_effect=AssertionError("NO USB")):
            restarted = Panel(standby=True)
            state = restarted.snapshot()
            self.assertTrue(state["gripper"]["hold_calibrated"])
            self.assertEqual(state["connection"]["state"], "STANDBY")
            self.assertIsNone(state["active"])
            self.assertIsNone(restarted.worker)
            restarted.gamepad.close()

    def test_corrupt_force_file_does_not_destroy_endpoints_or_connect(self):
        folder = Path(self.temp.name)
        (folder/"gripper_hold.json").write_text("not json")
        (folder/"gripper_limits.json").write_text(json.dumps(self.panel.gripper_points))
        with patch("panel.server.ROOT", folder), patch("panel.server.GamepadReader", SimGamepad):
            restarted = Panel(standby=True)
            self.assertEqual(restarted.gripper_points, self.panel.gripper_points)
            self.assertFalse(restarted.snapshot()["gripper"]["hold_calibrated"])
            self.assertIsNone(restarted.worker)
            restarted.gamepad.close()
