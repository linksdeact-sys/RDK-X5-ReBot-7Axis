"""Full upstream control loop with fake motors: never opens USB or changes a profile."""
import time
from itertools import count
from pathlib import Path
from unittest.mock import patch
import yaml

from panel.fake_hardware import FakeMotor, FakeController
from panel.gamepad_input import SimGamepad
from panel.gravity_mode import OfficialGravitySession


class Motor(FakeMotor):
    def disable(self): self.state.status_code = 0
    def send_mit(self, pos, vel, kp, kd, tau):
        super().send_mit(pos, vel, kp, kd, tau)
        self.state.torq = kd*(vel-self.state.vel)+tau


config = yaml.safe_load(Path('config/rebotarm_dm.yaml').read_text())
motors = [Motor(q) for q in (0., -.2, -.3, .1, 0., 0., 2.)]
pad, seq = SimGamepad(), count(1)
def rx(): return [{"age_ms": 1., "sequence": next(seq)} for _ in motors]

with patch('motorbridge.Controller.from_dm_device', side_effect=AssertionError('NO USB')):
    session = OfficialGravitySession(FakeController(motors), motors, config, [1.,1.,1.,.75,1.,1.],
        pad, (1.2, 3.9), None, rx, hold_torque=.06)
    try:
        session.start()
        until = time.monotonic()+5.
        while session.control.snapshot().get('need_rearm') is not False:
            assert time.monotonic()<until
            time.sleep(.01)
        pad.inject({'deadman':True,'triggers':{'rt':1.,'lt':0.}})
        while session.control.gripper.command.state!='CANDIDATE':
            assert session.running and time.monotonic()<until, session.control.snapshot()
            m=motors[6]
            v=m.commands[-1][1] if m.commands else 0.
            m.state.pos=min(2.08,m.state.pos+v*.01)
            m.state.vel=0. if m.state.pos>=2.08 else v
            time.sleep(.01)
        pad.inject({})
        time.sleep(.4)
        assert session.running
        assert motors[6].commands[-1][1:]==(0.,0.,.2,.06), motors[6].commands[-1]
        state=session.control.snapshot()
        assert state['gripper_hold_state']=='HOLD' and state['gripper_command_nm']==.06
        assert state['gripper_measured_force_n'] is None
        pad.inject({'deadman':True,'intent':[1.,0.,0.,0.,0.,0.,0.]})
        time.sleep(.25)
        assert session.control.snapshot()['target_rad'][0]>.001
        assert motors[6].commands[-1][-1]==.06
        pad.inject({'deadman':True,'triggers':{'lt':1.,'rt':0.}})
        time.sleep(.4)
        assert motors[6].commands[-1][1]<0. and motors[6].commands[-1][-1]==0.
        pad.inject({'connected':False})
        time.sleep(.06)
        assert motors[6].commands[-1][-1]==0.
        assert session.running
    finally:
        session.close(gentle=False)
    assert all(m.state.status_code==0 for m in motors)
print('PASS raw MIT hold torque, release RT/LB, simultaneous arm motion, LT release, HID loss and disable; no USB')
