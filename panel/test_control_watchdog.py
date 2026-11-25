"""No hardware. Check authoritative completion time rather than old UI snapshots."""
import time
from types import SimpleNamespace
import unittest
from panel.gamepad_control import GamepadGravityController
from panel.control_health import watchdog_action

class WatchdogTests(unittest.TestCase):
    def check(self, age, phase):
        now = time.monotonic()
        fixture = SimpleNamespace(runtime={'phase': phase, 'last_complete': now-age},
                                  watchdog_started=now-10.)
        return GamepadGravityController.watchdog_status(fixture)

    def test_reads_latest_completed_tick(self):
        state = self.check(.01, 'idle')
        self.assertLess(state['age_ms'], 100)

    def test_real_tx_stall_is_not_ignored(self):
        state = self.check(.15, 'tx')
        self.assertGreater(state['age_ms'], 100)
        self.assertEqual(state['phase'], 'tx')

    def test_startup_before_first_tick_has_bounded_grace(self):
        now = time.monotonic()
        fixture = SimpleNamespace(runtime={'phase': 'startup', 'last_complete': now-2.},
                                  watchdog_started=now)
        state = GamepadGravityController.watchdog_status(fixture)
        self.assertLess(state['age_ms'], 100)

    def test_actual_reported_short_stalls_pause_not_fault(self):
        for age in (102., 199., 210., 250.):
            self.assertEqual(watchdog_action(age), 'PAUSE')
        self.assertEqual(watchdog_action(99.), 'OK')

    def test_persistent_stall_and_invalid_clock_still_fault(self):
        for age in (251., 500., float('nan'), float('inf'), -1.):
            self.assertEqual(watchdog_action(age), 'FAULT')

if __name__ == '__main__':
    unittest.main()
