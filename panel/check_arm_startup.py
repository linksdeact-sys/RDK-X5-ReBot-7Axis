"""Actual upstream loop, fake motors only: ramp and all-axis recoverable holds."""
import time
from itertools import count
from pathlib import Path
from unittest.mock import patch
import yaml
from panel.fake_hardware import FakeMotor,FakeController
from panel.gamepad_input import SimGamepad
from panel.gravity_mode import OfficialGravitySession

config=yaml.safe_load(Path('config/rebotarm_dm.yaml').read_text())
motors=[FakeMotor(q) for q in (0.,-.2,-.3,.1,0.,0.,2.)]
pad,seq=SimGamepad(),count(1)
def rx():return [{'age_ms':1.,'sequence':next(seq)} for _ in motors]
def wait(predicate,seconds=3):
    end=time.monotonic()+seconds
    while not predicate():
        assert session.running and time.monotonic()<end,session.control.snapshot()
        time.sleep(.01)
with patch('motorbridge.Controller.from_dm_device',side_effect=AssertionError('NO USB')):
    session=OfficialGravitySession(FakeController(motors),motors,config,[1,1,1,.75,1,1],pad,None,None,rx)
    try:
        session.start()
        wait(lambda:session.control.snapshot().get('gravity_startup_blend')==1.)
        rows=session.control.drain_records()
        first=next(r for r in rows if r.get('mit'))
        assert first['mit']['tau'][:6]==[0.]*6
        assert all(first['mit']['kp'][i]>0 and first['mit']['kd'][i]>0 for i in range(6))
        assert abs(motors[3].commands[-1][-1])>.3
        wait(lambda:not session.control.snapshot()['need_rearm'])
        for axis in range(6):
            motors[axis].state.vel=-.857
            wait(lambda:session.control.snapshot().get('arm_paused'))
            frozen=session.control.snapshot()['target_rad'][:6]
            pad.inject({'deadman':True,'intent':[1.]*6+[0.]})
            time.sleep(.08)
            assert session.running and all(m.state.status_code==1 for m in motors)
            assert session.control.snapshot()['target_rad'][:6]==frozen
            assert abs(motors[3].commands[-1][-1])>.3
            motors[axis].state.vel=0.
            time.sleep(.05)
            assert session.control.snapshot()['need_rearm']
            pad.inject({})
            wait(lambda:not session.control.snapshot()['arm_paused'] and not session.control.snapshot()['need_rearm'])
        assert not session.control.snapshot().get('fault')
    finally:session.close(gentle=False)
print('PASS first-frame zero FF, gains retained, smooth startup, J1-J6 -.857 replay -> alive/hold/gravity, neutral rearm; NO USB')
