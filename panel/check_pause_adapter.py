"""Full upstream loop with fake motors only: J7 pause must retain arm support."""
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
    def send_mit(self,p,v,kp,kd,tau):
        super().send_mit(p,v,kp,kd,tau)
        self.state.torq=kd*(v-self.state.vel)+tau

motors=[Motor(q) for q in (0.,-.2,-.3,.1,0.,0.,2.)]
pad,seq=SimGamepad(),count(1)
config=yaml.safe_load(Path('config/rebotarm_dm.yaml').read_text())
def rx():return [{'age_ms':1.,'sequence':next(seq)} for _ in motors]
def wait(condition,seconds=4):
    end=time.monotonic()+seconds
    while not condition():
        assert session.running and time.monotonic()<end,session.control.snapshot()
        time.sleep(.01)

with patch('motorbridge.Controller.from_dm_device',side_effect=AssertionError('NO USB')):
    session=OfficialGravitySession(FakeController(motors),motors,config,[1.,1.,1.,.75,1.,1.],
        pad,(1.2,3.9),None,rx,torque_limited=True)
    try:
        session.start()
        wait(lambda:session.control.snapshot().get('need_rearm') is False)
        pad.inject({'deadman':True,'triggers':{'rt':1.,'lt':0.}})
        end=time.monotonic()+4.
        while session.control.gripper.command.state!='CLAMPING':
            assert session.running and time.monotonic()<end
            m=motors[6];v=m.commands[-1][1] if m.commands else 0.
            m.state.pos=min(2.08,m.state.pos+max(0.,v)*.01)
            m.state.vel=0. if m.state.pos>=2.08 else v
            time.sleep(.01)
        wait(lambda:motors[6].commands[-1][-1]>.7)
        pad.inject({'deadman':True,'intent':[1.,0.,0.,0.,0.,0.,0.]})
        time.sleep(.15)
        before_force=motors[6].commands[-1][-1]
        motors[6].state.vel=-1.106228
        wait(lambda:session.control.snapshot().get('gripper_paused'))
        assert motors[6].commands[-1][-1]>0. # no step from holding effort to zero
        frozen=session.control.snapshot()['target_rad'][:6]
        for speed in (-.886446,-1.6,1.6,3.2):
            motors[6].state.vel=speed
            time.sleep(.12)
            assert session.running and all(m.state.status_code==1 for m in motors)
            assert session.control.snapshot()['target_rad'][:6]==frozen
            assert not session.control.snapshot().get('fault')
            for i in range(6):
                _,_,kp,kd,tau=motors[i].commands[-1]
                assert kp>0 and kd>0
            assert abs(motors[2].commands[-1][-1])>.5 # gravity feedforward retained
        events=session.control.drain_records()
        assert any(r.get('event')=='local_pause' and r['preserve_arm_support'] for r in events)
        motors[6].state.vel=0.
        pad.inject({'deadman':True,'intent':[1.,0.,0.,0.,0.,0.,0.]})
        time.sleep(.8)
        assert session.control.snapshot()['need_rearm']
        assert session.control.snapshot()['target_rad'][:6]==frozen
        pad.inject({})
        wait(lambda:not session.control.snapshot()['need_rearm'])
        pad.inject({'deadman':True,'intent':[1.,0.,0.,0.,0.,0.,0.]})
        wait(lambda:session.control.snapshot()['target_rad'][0]>frozen[0]+.001)
        # A real driver error is NOT swallowed by the recoverable-pause path.
        motors[6].state.status_code=8
        end=time.monotonic()+2.
        while session.running:
            assert time.monotonic()<end
            time.sleep(.01)
        assert '驱动状态' in session.control.snapshot()['fault']
    finally:session.close(gentle=False)
    assert all(m.state.status_code==0 for m in motors)
print('PASS replay -0.886/-1.106/±1.6/3.2 speeds: job alive, seven enabled, arm references/gains/gravity held, rearm gate; real driver fault retained; NO USB')
