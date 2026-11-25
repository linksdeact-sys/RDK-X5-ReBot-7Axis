"""Replay recorded hand speeds in the actual upstream loop, NO USB."""
import time
from itertools import count
from pathlib import Path
from unittest.mock import patch
import yaml
from panel.fake_hardware import FakeMotor,FakeController
from panel.gamepad_input import SimGamepad
from panel.gravity_mode import OfficialGravitySession
from panel.teaching import validate


class Motor(FakeMotor):
    def disable(self):self.state.status_code=0


motors=[Motor(q) for q in (0.,-.2,-.3,.1,0.,0.,2.)]
pad=SimGamepad();pad.inject({'connected':False})
counter=count(1)
def rx():return [{'age_ms':1.,'sequence':next(counter)} for _ in motors]
config=yaml.safe_load(Path('config/rebotarm_dm.yaml').read_text())

def wait(predicate,seconds=3):
    deadline=time.monotonic()+seconds
    while not predicate():
        assert session.running and time.monotonic()<deadline,session.control.snapshot()
        time.sleep(.01)

with patch('motorbridge.Controller.from_dm_device',side_effect=AssertionError('NO USB')):
    session=OfficialGravitySession(FakeController(motors),motors,config,[1,1,1,.75,1,1],pad,(1.2,3.9),None,rx,
        torque_limited=True,teaching={'signature':'robot'})
    try:
        session.start();wait(lambda:session.control.teach_compliance is not None and session.control.teach_compliance.elapsed>=.5)
        teacher=session.control.teacher
        teacher.request('record',name='真实手推速度回放')
        wait(lambda:teacher.rows is not None)
        for axis,speed in ((2,-.9304027557),(1,-1.0231990814),(1,-.8815631866)):
            for _ in range(20):
                motors[axis].state.vel=speed;motors[axis].state.pos+=speed*.01
                time.sleep(.01)
                assert teacher.mode=='DRAG' and teacher.rows is not None
                assert not session.control.arm_guard.active
                assert all(m.commands[-1][2:4]==(2.,1.) for m in motors[:6])
            motors[axis].state.vel=0.;time.sleep(.03)
        teacher.request('finish');wait(lambda:teacher.mode=='HOLD')
        record=teacher.take_completed()[0];validate(record,'robot',teacher.limits)
        # The hard gate is intentionally NOT bypassed in hand teaching.
        teacher.request('drag');wait(lambda:teacher.mode=='DRAG' and teacher.factor==0.)
        motors[2].state.vel=-1.5
        wait(lambda:teacher.mode=='HOLD')
        assert session.control.arm_guard.active and session.running
        assert all(m.state.status_code==1 for m in motors)
        assert not session.control.snapshot().get('fault')
    finally:session.close(gentle=False)
print('PASS actual .930/1.023/.882rad/s hand replay keeps recording/Kp2-Kd1; 1.5rad/s still holds, healthy arm remains enabled; NO USB')
