"""No hardware: startup RAM repair, read-back failures and disabled-only guards."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call

from panel.gripper_watchdog import ensure_gripper_watchdog
from panel.server import Hardware


class WatchdogStartupTests(unittest.TestCase):
    def motor(self, reads):
        return SimpleNamespace(damiao_get_param_u32=Mock(side_effect=reads),
                               damiao_write_param_u32=Mock())

    def test_disabled_timeout_is_repaired_and_read_back(self):
        motor, guard = self.motor([0, 4000]), Mock()
        result = ensure_gripper_watchdog(motor, guard)
        self.assertEqual(result, {"before_raw": 0, "after_raw": 4000,
                                  "timeout_ms": 200., "configured_ram": True})
        motor.damiao_write_param_u32.assert_called_once_with(9, 4000)
        self.assertEqual(motor.damiao_get_param_u32.call_args_list, [call(9, 100), call(9, 100)])
        self.assertEqual(guard.call_count, 4)

    def test_preserves_valid_setting_without_rewrite(self):
        for raw in (2000, 4000, 5000):
            motor, guard = self.motor([raw]), Mock()
            result = ensure_gripper_watchdog(motor, guard)
            self.assertFalse(result["configured_ram"])
            self.assertEqual(result["after_raw"], raw)
            motor.damiao_write_param_u32.assert_not_called()
            self.assertEqual(guard.call_count, 2)

    def test_out_of_range_setting_is_repaired_not_bypassed(self):
        for raw in (1, 1999, 5001, 0xFFFFFFFF):
            motor = self.motor([raw, 4000])
            self.assertEqual(ensure_gripper_watchdog(motor, Mock())["timeout_ms"], 200.)
            motor.damiao_write_param_u32.assert_called_once_with(9, 4000)

    def test_read_failure_or_bad_type_never_writes(self):
        for raw in (None, True, 0., -1, 0x100000000):
            motor = self.motor([raw])
            with self.assertRaises(RuntimeError):
                ensure_gripper_watchdog(motor, Mock())
            motor.damiao_write_param_u32.assert_not_called()
        motor = self.motor([RuntimeError("read failed")])
        with self.assertRaisesRegex(RuntimeError, "read failed"):
            ensure_gripper_watchdog(motor, Mock())
        motor.damiao_write_param_u32.assert_not_called()

    def test_failed_write_or_mismatched_readback_never_passes(self):
        for after in (0, 3999, 5000, 4000., None, RuntimeError("read timeout")):
            with self.assertRaises(RuntimeError):
                ensure_gripper_watchdog(self.motor([0, after]), Mock())
        motor = self.motor([0])
        motor.damiao_write_param_u32.side_effect = RuntimeError("write failed")
        with self.assertRaisesRegex(RuntimeError, "write failed"):
            ensure_gripper_watchdog(motor, Mock())

    def test_guard_checks_before_read_and_before_write(self):
        for checks in ([RuntimeError("enabled")], [None, RuntimeError("enabled")]):
            motor = self.motor([0, 4000])
            with self.assertRaisesRegex(RuntimeError, "enabled"):
                ensure_gripper_watchdog(motor, Mock(side_effect=checks))
            motor.damiao_write_param_u32.assert_not_called()

    def test_lost_idle_state_after_write_does_not_report_success(self):
        for checks in ([None, None, RuntimeError("changed state")],
                       [None, None, None, RuntimeError("changed state")]):
            with self.assertRaisesRegex(RuntimeError, "changed state"):
                ensure_gripper_watchdog(self.motor([0, 4000]), Mock(side_effect=checks))

    def test_hardware_guard_rejects_enabled_stale_or_moving_axes(self):
        for bad in ("enabled", "moving", "stale", "nan", "missing"):
            states = {i: SimpleNamespace(status_code=0, vel=0.) for i in range(7)}
            metadata = [{"sequence": 1, "age_ms": 1.} for _ in range(7)]
            if bad == "enabled": states[0].status_code = 1
            if bad == "moving": states[3].vel = .1
            if bad == "nan": states[4].vel = float("nan")
            if bad == "missing": states[1] = None
            if bad == "stale": metadata[6]["age_ms"] = 151.
            motor = self.motor([0, 4000])
            hardware = SimpleNamespace(motors=[None]*6+[motor], poll=Mock(return_value=states),
                                       rx_meta=Mock(return_value=metadata))
            with self.assertRaises(RuntimeError):
                Hardware.gripper_watchdog(hardware)
            motor.damiao_write_param_u32.assert_not_called()

    def test_hardware_validates_and_records_ram_repair(self):
        motor = self.motor([0, 4000])
        hardware = SimpleNamespace(motors=[None]*6+[motor], poll=Mock(return_value={
            i: SimpleNamespace(status_code=0, vel=0.) for i in range(7)}),
            rx_meta=Mock(return_value=[{"sequence": 2, "age_ms": 1.} for _ in range(7)]))
        self.assertEqual(Hardware.gripper_watchdog(hardware), 200.)
        self.assertTrue(hardware.last_gripper_watchdog["configured_ram"])
