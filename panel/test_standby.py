"""Standby must serve state without creating any CAN owner or motor worker."""
import threading
import unittest
from unittest.mock import patch, Mock
from panel.server import Panel
from panel.gamepad_input import SimGamepad

class StandbyTests(unittest.TestCase):
    def create(self):
        with patch('panel.server.GamepadReader', SimGamepad):
            return Panel(standby=True)

    def test_standby_never_opens_usb_even_when_device_present(self):
        with patch('panel.server.Hardware', side_effect=AssertionError('NO CAN')) as hw:
            panel = self.create()
            panel.service_usb()
            state = panel.snapshot()
            self.assertEqual(state['connection']['state'], 'STANDBY')
            self.assertFalse(state['connection']['ready'])
            self.assertIsNone(panel.worker)
            self.assertIsNone(state['active'])
            self.assertTrue(all(j['status'] is None and not j['online'] for j in state['joints']))
            hw.assert_not_called()
            panel.gamepad.close()

    def test_cannot_connect_without_support_confirmation(self):
        panel = self.create()
        for body in ({}, {'supported': False}, {'supported': 'true'}):
            with self.assertRaises(ValueError):
                panel.connect_hardware(body)
        self.assertTrue(panel.standby)
        self.assertIsNone(panel.worker)

    def test_confirm_starts_only_one_worker_without_unlocking_motion(self):
        panel = self.create()
        with patch.object(panel, 'run', Mock()) as run:
            panel.connect_hardware({'supported': True})
            panel.worker.join(timeout=1.)
            panel.connect_hardware({'supported': True})
            self.assertEqual(run.call_count, 1)
            self.assertEqual(panel.connection_state, 'CONNECTING')
            self.assertFalse(panel.connection_ready)
            self.assertIsNone(panel.active)

if __name__ == '__main__':
    unittest.main()
