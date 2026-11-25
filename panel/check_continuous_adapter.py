"""Continuous controller integration test with fake motors. Never opens USB."""
import time
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import yaml

from panel.fake_hardware import FakeMotor, FakeController
from panel.gamepad_input import SimGamepad
from panel.gravity_mode import OfficialGravitySession
from panel.server import Hardware
from panel.control_health import watchdog_action

config = yaml.safe_load(Path('config/rebotarm_dm.yaml').read_text())
motors = [FakeMotor(v) for v in (0., -.5, -.5, 0., 0., 0., -.1)]
controller = FakeController(motors)
# A control tick must NOT acquire the SDK receive drain lock.
controller.poll_feedback_once = lambda: (_ for _ in ()).throw(AssertionError('duplicate RX poll'))
pad = SimGamepad()
def rx():
    return [{'age_ms': 1., 'sequence': 123} for _ in range(7)]
with patch('motorbridge.Controller.from_dm_device', side_effect=AssertionError('no real USB')):
    session = OfficialGravitySession(controller, motors, config, [1, 1, 1, .70, 1, 1], pad, None, None, rx)
    try:
        session.start()
        time.sleep(.6)
        assert session.running, session.control.snapshot()
        assert session.robot.rate == 100.
        np.testing.assert_allclose(session.control.KP, [120, 120, 120, 18, 18, 18])
        np.testing.assert_allclose(session.control.KD, [5, 5, 5, 2, 2, 2])
        pad.inject({'deadman': True, 'intent': [1, -1, -1, 1, 1, 1, 1]})
        time.sleep(.6)
        state = session.control.snapshot()
        assert state['mode'] == 'continuous' and state['driving'], state
        for i in range(6):
            assert abs(state['target_rad'][i]-motors[i].state.pos) > .001
            assert abs(state['target_rad'][i]-motors[i].state.pos) <= .0600001
        assert session.control.watchdog_status()['age_ms'] < 100
        assert 'tx_ms' in state['timing']
        # The monitor must consume the published frame and never get SDK locks.
        read_state = motors[0].get_state
        main_thread = threading.current_thread()
        def monitor_read_guard():
            if threading.current_thread() is main_thread:
                raise AssertionError('monitor SDK read')
            return read_state()
        with patch.object(motors[0], 'get_state', side_effect=monitor_read_guard):
            observed, torque, running = Hardware.observe_gravity(SimpleNamespace(
                gravity_session=session, motors=motors))
            assert running and len(observed) == 7
        blocked, release = threading.Event(), threading.Event()
        original_read = motors[0].get_state
        def stalled_read():
            blocked.set()
            assert release.wait(.23)
            return original_read()
        motors[0].get_state = stalled_read
        assert blocked.wait(.1)
        time.sleep(.12)
        assert watchdog_action(session.control.watchdog_status()['age_ms']) == 'PAUSE'
        session.control.request_timing_pause()
        release.set()
        motors[0].get_state = original_read
        time.sleep(.04)
        assert session.running, session.control.snapshot()
        assert session.control.snapshot()['need_rearm']
        assert all(m.state.status_code == 1 for m in motors)
        # A send failure must propagate, not be swallowed by JointGroup.
        motors[2].send_mit = lambda *a: (_ for _ in ()).throw(RuntimeError('injected TX failure'))
        time.sleep(.06)
        assert not session.running
        assert 'J3 MIT发送失败' in session.control.snapshot()['fault']
    finally:
        session.close(gentle=False)
    assert all(m.state.status_code == 0 for m in motors)
print('PASS six-axis control, monitor cache, injected 120ms stall HOLD without disable, LB rearm, TX fault and disable; NO USB opened')
