"""Upstream loop + fake motors only: direct-Nm ceiling, never opens USB."""
import time
from itertools import count
from pathlib import Path
from unittest.mock import patch
import yaml
from panel.fake_hardware import FakeMotor,FakeController
from panel.gamepad_input import SimGamepad
from panel.gravity_mode import OfficialGravitySession

class Motor(FakeMotor):
    def disable(self):self.state.status_code=0
    def send_mit(self,pos,vel,kp,kd,tau):
        super().send_mit(pos,vel,kp,kd,tau)
        self.state.torq=kd*(vel-self.state.vel)+tau

motors=[Motor(q) for q in (0.,-.2,-.3,.1,0.,0.,2.)]
pad,seq=SimGamepad(),count(1)
config=yaml.safe_load(Path('config/rebotarm_dm.yaml').read_text())
object_position=2.08
def rx():return [{'age_ms':1.,'sequence':next(seq)} for _ in motors]
def wait_for(predicate,seconds=7):
    end=time.monotonic()+seconds
    while not predicate():
        assert session.running and time.monotonic()<end,session.control.snapshot()
        m=motors[6];cmd=m.commands[-1] if m.commands else (2.,0.,0.,.2,0.)
        m.state.pos=min(object_position,m.state.pos+max(0.,cmd[1])*.01)
        m.state.vel=0. if m.state.pos>=object_position else max(0.,cmd[1])
        assert 0<=cmd[4]<=2. and cmd[2]==0.
        time.sleep(.01)

with patch('motorbridge.Controller.from_dm_device',side_effect=AssertionError('NO USB')):
    session=OfficialGravitySession(FakeController(motors),motors,config,[1.,1.,1.,.75,1.,1.],
        pad,(1.2,3.9),None,rx,torque_limited=True)
    try:
        session.start()
        wait_for(lambda:session.control.snapshot().get('need_rearm') is False)
        pad.inject({'deadman':True,'triggers':{'rt':1.,'lt':0.}})
        wait_for(lambda:session.control.gripper.command.state=='CLAMPING')
        wait_for(lambda:motors[6].commands[-1][-1]>.5)
        pad.inject({})
        time.sleep(.08)
        held=motors[6].commands[-1][-1]
        assert .5<held<2.
        time.sleep(.1)
        assert motors[6].commands[-1][1:]==(0.,0.,0.,held)
        # Replay positive HOLD compression from the real incident. No RT and
        # no force increase, while the same upstream loop keeps all axes alive.
        for _ in range(30):
            motors[6].state.pos+=.227106*.01
            motors[6].state.vel=.227106
            time.sleep(.01)
            assert session.running and not session.control.gripper.speed_hold
            assert motors[6].commands[-1][1:]==(0.,0.,0.,held)
        motors[6].state.vel=0.
        object_position=motors[6].state.pos
        assert session.control.snapshot()['gripper_contact_motion']['compression_deg']>2.
        pad.inject({'deadman':True,'triggers':{'rt':1.,'lt':0.}})
        wait_for(lambda:session.control.gripper.hold.close_locked)
        capped=motors[6].commands[-1][-1]
        assert 1.98<=capped<=2.
        assert session.control.snapshot()['gripper_torque_limit_nm']==2.
        pad.inject({});time.sleep(.05)
        pad.inject({'deadman':True,'triggers':{'rt':1.,'lt':0.}});time.sleep(.08)
        assert motors[6].commands[-1][1:]==(0.,0.,0.,capped)
        pad.inject({'deadman':True,'triggers':{'lt':1.,'rt':0.}})
        wait_for(lambda:motors[6].commands[-1][1]<0.,seconds=3)
        assert motors[6].commands[-1][-1]==0.
        pad.inject({'connected':False});time.sleep(.05)
        assert motors[6].commands[-1][-1]==0.
    finally:session.close(gentle=False)
    assert all(m.state.status_code==0 for m in motors)
print('PASS direct Nm MIT, progressive RT, recorded HOLD compression without force cut, <=2Nm ceiling, no ratchet, LT unload, disconnect, disable; NO USB')
