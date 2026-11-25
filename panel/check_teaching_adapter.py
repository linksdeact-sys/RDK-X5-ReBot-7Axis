"""Real upstream loop, fake motors only: seven-axis hand teach and replay."""
import time
import math
from itertools import count
from pathlib import Path
from unittest.mock import patch
import yaml
from panel.fake_hardware import FakeMotor,FakeController
from panel.gamepad_input import SimGamepad
from panel.gravity_mode import OfficialGravitySession
from panel.teaching import validate,prepare_play


class Motor(FakeMotor):
    def disable(self):self.state.status_code=0


motors=[Motor(q) for q in (0.,-.2,-.3,.1,0.,0.,2.)]
pad=SimGamepad();pad.inject({'connected':False})
seq=count(1)
def rx():return [{'age_ms':1.,'sequence':next(seq)} for _ in motors]
config=yaml.safe_load(Path('config/rebotarm_dm.yaml').read_text())

def wait(predicate,seconds=5.,plant=False):
    end=time.monotonic()+seconds
    while not predicate():
        assert session.running and time.monotonic()<end,session.control.snapshot()
        if plant:
            for m in motors[:6]:
                cmd=m.commands[-1];old=m.state.pos;m.state.pos=cmd[0];m.state.vel=(m.state.pos-old)/.01
            m=motors[6];m.state.vel=m.commands[-1][1];m.state.pos+=m.state.vel*.01
        time.sleep(.01)


with patch('motorbridge.Controller.from_dm_device',side_effect=AssertionError('NO USB')):
    session=OfficialGravitySession(FakeController(motors),motors,config,[1,1,1,.75,1,1],pad,(1.2,3.9),None,rx,
        torque_limited=True,teaching={'signature':'robot'})
    try:
        session.start()
        wait(lambda:session.control.teacher and session.control.snapshot().get('gravity_startup_blend')==1.)
        wait(lambda:session.control.teach_compliance.elapsed>=.5)
        teacher=session.control.teacher
        assert all(m.commands[-1][2:4]==(2.,1.) for m in motors[:6])
        assert motors[6].commands[-1][2]==0.
        assert motors[6].commands[-1][-1]==0. and motors[6].commands[-1][1]==0.
        assert motors[6].commands[-1][3]==.1
        teacher.request('record',name='手推七轴')
        for _ in range(90):
            for i,m in enumerate(motors):m.state.pos+=.0005 if i==6 else .0002;m.state.vel=.05 if i==6 else .02
            time.sleep(.01)
            assert all(m.commands[-1][2:4]==(2.,1.) for m in motors[:6])
            assert motors[6].commands[-1][2]==0.
            assert motors[6].commands[-1][-1]==0.
        for m in motors:m.state.vel=0.
        teacher.request('finish')
        wait(lambda:teacher.mode=='HOLD' and teacher.factor==1.)
        record=teacher.take_completed()[0];validate(record,'robot',teacher.limits)
        assert record['samples'][-1]['q'][6]>record['samples'][0]['q'][6]+.02
        assert all(m.commands[-1][2]>0 for m in motors[:6])
        assert motors[6].commands[-1][-1]==0. # No automatic grasp after recording.
        teacher.request('drag');wait(lambda:teacher.mode=='DRAG' and teacher.factor==0.)
        for m,q in zip(motors,record['samples'][0]['q']):m.state.pos=q;m.state.vel=0.
        motors[0].state.pos+=math.radians(14.5)
        motors[6].state.pos+=math.radians(14.5)
        # Let the manual feedback/command cycle observe the new pose before
        # asking for hold; do not simulate an impossible one-frame teleport
        # straight into a stiff reference captured at the previous position.
        wait(lambda:max(abs(m.commands[-1][0]-m.state.pos) for m in motors[:6])<1e-4)
        teacher.request('hold');wait(lambda:teacher.factor==1.)
        teacher.request('play',record=record,confirmed=True)
        wait(lambda:teacher.mode=='REPLAY')
        assert teacher.joining_start and teacher.cursor==0.
        saw_jaw=False
        end=time.monotonic()+10.
        while teacher.mode=='REPLAY':
            assert session.running and time.monotonic()<end,session.control.snapshot()
            for m in motors[:6]:m.state.pos=m.commands[-1][0];m.state.vel=m.commands[-1][1]
            m=motors[6];m.state.vel=m.commands[-1][1];m.state.pos+=m.state.vel*.01
            saw_jaw=saw_jaw or m.state.vel>0.
            assert 0<=m.commands[-1][-1]<=2.
            time.sleep(.01)
        assert saw_jaw and teacher.mode=='HOLD',session.control.snapshot()
        assert all(m.state.status_code==1 for m in motors)
        assert not session.control.snapshot().get('fault')
    finally:session.close(gentle=False)
    assert all(m.state.status_code==0 for m in motors)
    # A new replay session must start controlled, never pass through DRAG or
    # require an operator to manually create HOLD before pressing replay.
    for m,q in zip(motors,record['samples'][0]['q']):m.state.pos=q;m.state.vel=0.
    prepared=prepare_play(record,'robot',teacher.limits)
    session=OfficialGravitySession(FakeController(motors),motors,config,[1,1,1,.75,1,1],pad,(1.2,3.9),None,rx,
        torque_limited=True,teaching={'signature':'robot','initial_play':prepared})
    try:
        session.start()
        wait(lambda:session.control.teacher is not None)
        direct=session.control.teacher
        assert direct.mode=='HOLD'
        wait(lambda:direct.mode=='REPLAY',plant=True)
        assert session.control.teach_compliance.mode=='HOLD'
        wait(lambda:direct.mode=='HOLD',seconds=10.,plant=True)
        assert '回放完成' in direct.reason
        assert all(m.state.status_code==1 for m in motors)
        assert not session.control.snapshot().get('fault')
    finally:session.close(gentle=False)
print('PASS reBot Kp2/Kd1 drag, 14.5deg low-speed join, explicit replay from a fresh controlled session without DRAG, seven-axis replay and end hold; NO USB')
