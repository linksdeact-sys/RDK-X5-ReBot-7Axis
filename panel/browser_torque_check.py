"""8766 simulated motors only. Not a real motor-torque/force measurement."""
import math
from pathlib import Path
from time import monotonic
from playwright.sync_api import expect,sync_playwright

URL='http://192.168.127.10:8766'
with sync_playwright() as pw:
    browser=pw.chromium.launch(channel='msedge',headless=True)
    page=browser.new_page(viewport={'width':1440,'height':1000})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(URL,wait_until='domcontentloaded')
    expect(page.locator('#online-count')).to_have_text('7 / 7 在线')
    def state():
        data=page.request.get(URL+'/api/state').json()
        assert data['simulate'] is True
        return data
    initial=state()
    assert initial['active'] is None
    headers={'Origin':URL,'X-Arm-Token':initial['csrf_token']}
    def pad(**body):
        assert state()['simulate']
        r=page.request.post(URL+'/api/sim-input',data=body,headers=headers)
        assert r.ok,r.text()
    def control(s):return (s.get('active') or {}).get('teleop',{})
    def phase(s):return control(s).get('gripper_hold_state')
    def wait(predicate,seconds=9):
        end=monotonic()+seconds
        while monotonic()<end:
            s=state()
            if predicate(s):return s
            page.wait_for_timeout(30)
        raise AssertionError(str(s.get('active') or s.get('result') or s))
    expect(page.locator('#gripper-policy')).to_have_value('torque-limit')
    expect(page.locator('#gripper-hold-status')).to_contain_text('2.00 N·m')
    assert initial['gripper']['torque_limit_nm']==2.
    assert 'light_reference_n' not in initial['gripper']
    pad(gripper_object_rad=math.radians(initial['joints'][6]['angle'])+.08)
    page.locator('#input-verified').check();page.locator('#gripper-enable').check()
    page.locator('#gamepad-start').click()
    wait(lambda s:s['active'] and not control(s).get('need_rearm',True))
    pad(deadman=True,intent=[1.,0.,0.,0.,0.,0.,0.],triggers={'rt':1.,'lt':0.})
    wait(lambda s:phase(s)=='CLAMPING' and control(s)['gripper_command_nm']>.5)
    pad()
    partial=wait(lambda s:phase(s)=='HOLD' and not control(s)['gripper_close_locked'])
    force=control(partial)['gripper_command_nm']
    assert .5<force<2. and partial['joints'][0]['angle']>initial['joints'][0]['angle']+.1
    page.wait_for_timeout(150)
    assert control(state())['gripper_command_nm']==force
    # Simulate a compliant object settling inward >2deg after RT release.
    # Move the test-only contact plane slowly; the live API has no such input.
    object_position=math.radians(partial['joints'][6]['angle'])
    for step in range(35):
        pad(gripper_object_rad=object_position+(step+1)*.002)
        page.wait_for_timeout(40)
        settling=state()
        assert phase(settling)=='HOLD' and not control(settling)['gripper_paused']
        assert control(settling)['gripper_command_nm']==force
    assert control(state())['gripper_contact_motion']['compression_deg']>2.
    expect(page.locator('#gripper-hold-status')).to_contain_text('12°受限向内压缩')
    pad(deadman=True,triggers={'rt':1.,'lt':0.})
    capped=wait(lambda s:control(s).get('gripper_close_locked'))
    cap=control(capped)['gripper_command_nm'];anchor=capped['joints'][6]['angle']
    assert 1.98<=cap<=2. and control(capped)['gripper_torque_limit_nm']==2.
    assert control(capped)['gripper_measured_force_n'] is None
    for _ in range(3):
        pad();page.wait_for_timeout(100)
        pad(deadman=True,triggers={'rt':1.,'lt':0.});page.wait_for_timeout(100)
        assert control(state())['gripper_command_nm']==cap
        assert control(state())['gripper_velocity_ref_rad_s']==0.
    page.screenshot(path=str(Path('panel/torque-workflow-qa.png').resolve()),full_page=True)
    page.set_viewport_size({'width':390,'height':844})
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
    page.screenshot(path=str(Path('panel/torque-mobile-qa.png').resolve()),full_page=True)
    pad(triggers={'rt':0.,'lt':1.}) # LT without LB must not release.
    page.wait_for_timeout(120)
    assert control(state())['gripper_command_nm']==cap
    pad(deadman=True,triggers={'rt':0.,'lt':1.})
    wait(lambda s:control(s).get('gripper_velocity_ref_rad_s',0)<-.02)
    pad(deadman=True,triggers={'rt':1.,'lt':0.})
    wait(lambda s:phase(s)=='CLOSE_LOCKED')
    assert control(state())['gripper_command_nm']==0.
    pad(deadman=True,triggers={'rt':0.,'lt':1.})
    wait(lambda s:not control(s).get('gripper_close_locked',True) and s['joints'][6]['angle']<anchor-2.)
    pad(stop=True)
    wait(lambda s:not s['active'] and all(j['status']==0 for j in s['joints']))
    pad(gripper_object_rad=None)
    assert not errors,errors
    browser.close()
    print('PASS simulated direct 2Nm ceiling, positive HOLD settling >2deg without force cut, resume, cap lock, LT unload/unlock, simultaneous arm, B, mobile; NO real motion')
