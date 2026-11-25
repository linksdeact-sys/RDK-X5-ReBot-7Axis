"""Offline reconnect tests: fake hardware and child processes, no USB opened."""

import collections
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from panel.server import Panel
from panel.usb_reconnect import usb_signature, RestartForUsb, supervise


def isolated_panel():
    panel = Panel.__new__(Panel)
    panel.lock = threading.RLock()
    panel.closing, panel.stop, panel.reconnect_requested = threading.Event(), threading.Event(), threading.Event()
    panel.connection_ready = False
    panel.connection_state = "CONNECTING"
    panel.simulate = False
    panel.usb_generation = ("port:1:62",)
    panel.pending, panel.active = None, None
    panel.error = None
    panel.states = [None]*7
    panel.feedback_at = [None]*7
    panel.last_poll = 0.0
    panel.events = collections.deque(maxlen=30)
    panel.init_recovery()
    panel.restart_at = 0.0
    return panel


class FakeHardware:
    def __init__(self, cached=False, ignore_disable=False):
        self.sequence = 10
        self.cached, self.ignore_disable = cached, ignore_disable
        self.disabled, self.enabled = [], []
        self.states = [SimpleNamespace(pos=0.0, vel=0.0, torq=0.0, status_code=1,
                                      t_mos=25.0, t_rotor=25.0) for _ in range(7)]

    def disable(self, i):
        self.disabled.append(i)
        if not self.ignore_disable:
            self.states[i].status_code = 0

    def enable(self, i):
        self.enabled.append(i)

    def poll(self, indices):
        if not self.cached:
            self.sequence += 1
        return {i: self.states[i] for i in indices}

    def rx_meta(self):
        return [{"sequence": self.sequence, "age_ms": 0.0} for _ in range(7)]


class ReconnectTests(unittest.TestCase):
    def test_sysfs_detection_and_generation(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            device = root / "1-1.3"
            device.mkdir()
            for field, value in {"idVendor": "34b7", "idProduct": "6877", "busnum": "1", "devnum": "62"}.items():
                (device/field).write_text(value)
            self.assertEqual(usb_signature(root), ("1-1.3:1:62",))
            (device/"devnum").write_text("63")
            self.assertEqual(usb_signature(root), ("1-1.3:1:63",))
            (device/"idVendor").write_text("1234")
            self.assertEqual(usb_signature(root), ())

    def test_disconnect_cancels_pending_invalidates_feedback(self):
        panel = isolated_panel()
        panel.connection_ready = True
        panel.pending = {"id": "must-not-replay"}
        panel.states = [dict(pos=0.0, vel=0.0, status_code=0)]*7
        panel.connection_lost("USB disconnected")
        self.assertFalse(panel.connection_ready)
        self.assertIsNone(panel.pending)
        self.assertEqual(panel.states, [None]*7)
        self.assertTrue(panel.stop.is_set())
        with self.assertRaises(RuntimeError):
            panel.submit({"joint": 1, "degrees": 1, "kind": "step", "owner": "qa", "request_id": "old-job"})

    def test_waits_for_presence_then_requests_fresh_process(self):
        panel = isolated_panel()
        with patch("panel.server.usb_signature", return_value=()):
            panel.service_usb()
        self.assertEqual(panel.connection_state, "WAIT_USB")
        panel.restart_at = 0
        with patch("panel.server.usb_signature", return_value=("port:1:63",)):
            with self.assertRaises(RestartForUsb):
                panel.service_usb()

    def test_requires_two_new_disabled_feedback_snapshots(self):
        panel, hardware = isolated_panel(), FakeHardware()
        self.assertTrue(panel.verify_disabled(hardware))
        self.assertTrue(panel.connection_ready)
        self.assertIsNone(panel.error)
        self.assertEqual(hardware.disabled, list(range(7)))
        self.assertEqual(hardware.enabled, [])
        self.assertIsNone(panel.pending)
        self.assertIsNone(panel.active)

    def test_cached_disabled_values_do_not_unlock(self):
        panel, hardware = isolated_panel(), FakeHardware(cached=True)
        timer = threading.Timer(0.03, panel.closing.set)
        timer.start()
        try:
            self.assertFalse(panel.verify_disabled(hardware))
            self.assertFalse(panel.connection_ready)
            self.assertEqual(hardware.enabled, [])
        finally:
            timer.cancel()

    def test_enabled_feedback_does_not_unlock(self):
        panel, hardware = isolated_panel(), FakeHardware(ignore_disable=True)
        timer = threading.Timer(0.03, panel.closing.set)
        timer.start()
        try:
            self.assertFalse(panel.verify_disabled(hardware))
            self.assertFalse(panel.connection_ready)
            self.assertEqual(hardware.enabled, [])
        finally:
            timer.cancel()

    def test_supervisor_restarts_only_explicit_usb_exit(self):
        first, second = Mock(), Mock()
        first.wait.return_value, second.wait.return_value = 75, 0
        with (patch("panel.usb_reconnect.signal.signal"),
              patch("panel.usb_reconnect.subprocess.Popen", side_effect=[first, second]) as spawn,
              patch("panel.usb_reconnect.threading.Event.wait", return_value=False)):
            self.assertEqual(supervise(Path("server.py"), []), 0)
            self.assertEqual(spawn.call_count, 2)
            self.assertEqual(spawn.call_args_list[0].kwargs["env"]["REBOT_RECONNECT_COUNT"], "0")
            self.assertEqual(spawn.call_args_list[1].kwargs["env"]["REBOT_RECONNECT_COUNT"], "1")
            self.assertEqual(spawn.call_args_list[1].kwargs["env"]["REBOT_PANEL_CHILD"], "1")
        crashed = Mock()
        crashed.wait.return_value = 1
        with patch("panel.usb_reconnect.signal.signal"), patch("panel.usb_reconnect.subprocess.Popen", return_value=crashed) as spawn:
            self.assertEqual(supervise(Path("server.py"), []), 1)
            self.assertEqual(spawn.call_count, 1)


if __name__ == "__main__":
    unittest.main()
