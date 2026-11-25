"""Pause/hold UI acceptance on 8766 simulation only; never real motors."""
import math
from time import monotonic
from pathlib import Path
from playwright.sync_api import expect,sync_playwright

URL='http://192.168.127.10:8766'
with sync_playwright() as pw:
    browser=pw.chromium.launch(channel='msedge',headless=True)
    page=browser.new_page(viewport={'width':1440,'height':1000})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(URL,wait_until='domcontentloaded')
    expect(page.locator('#online-count')).to_have_text('7 / 7 在线')
    def state():
        s=page.request.get(URL+'/api/state').json()
        assert s['simulate'] is True
        return s
    initial=state()
    assert not initial['active'] and initial['gripper']['hard_speed_rad_s']==1.5
    headers={'Origin':URL,'X-Arm-Token':initial['csrf_token']}
    def pad(**body):
        assert state()['simulate']
        r=page.request.post(URL+'/api/sim-input',data=body,headers=headers)
        assert r.ok,r.text()
    def control(s):return (s.get('active') or {}).get('teleop',{})
    def wait(predicate,seconds=8):
        end=monotonic()+seconds
        while monotonic()<end:
            s=state()
            if predicate(s):return s
            page.wait_for_timeout(25)
        raise AssertionError(str(s.get('active') or s.get('result')))
    pad(gripper_object_rad=math.radians(initial['joints'][6]['angle'])+.08)
    page.locator('#input-verified').check();page.locator('#gripper-enable').check()
    page.locator('#gamepad-start').click()
    wait(lambda s:s['active'] and not control(s).get('need_rearm',True))
    pad(deadman=True,triggers={'rt':1.,'lt':0.})
    wait(lambda s:control(s).get('gripper_command_nm',0)>.7)
    pad()
    held=wait(lambda s:control(s).get('gripper_hold_state')=='HOLD')
    force=control(held)['gripper_command_nm']
    pad(deadman=True,intent=[1.,0.,0.,0.,0.,0.,0.],gripper_velocity_rad_s=-1.106228)
    paused=wait(lambda s:control(s).get('gripper_paused'))
    frozen=control(paused)['target_rad'][:6]
    assert 0<control(paused)['gripper_command_nm']<force
    assert all(j['status']==1 for j in paused['joints'])
    assert paused['active']['revision']=='continuous-mit-v11'
    expect(page.locator('#gripper-hold-status')).to_contain_text('当前保护暂停')
    pad(deadman=True,intent=[1.,0.,0.,0.,0.,0.,0.],gripper_velocity_rad_s=-1.6)
    hard=wait(lambda s:'硬超速' in (control(s).get('gripper_pause_reason') or ''))
    page.wait_for_timeout(250)
    s=state()
    assert s['active'] and all(j['status']==1 for j in s['joints'])
    assert control(s)['target_rad'][:6]==frozen
    assert control(s)['need_rearm'] and not control(s)['driving']
    page.screenshot(path=str(Path('panel/pause-workflow-qa.png').resolve()),full_page=True)
    page.set_viewport_size({'width':390,'height':844})
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
    page.screenshot(path=str(Path('panel/pause-mobile-qa.png').resolve()),full_page=True)
    # Remove simulated speed fault but keep stick/LB held: no automatic replay.
    pad(deadman=True,intent=[1.,0.,0.,0.,0.,0.,0.])
    page.wait_for_timeout(650)
    s=state()
    assert s['active'] and all(j['status']==1 for j in s['joints'])
    assert control(s)['need_rearm'] and control(s)['target_rad'][:6]==frozen
    pad()
    wait(lambda s:not control(s).get('need_rearm',True) and not control(s).get('gripper_paused',True))
    pad(deadman=True,intent=[1.,0.,0.,0.,0.,0.,0.])
    wait(lambda s:control(s)['target_rad'][0]>frozen[0]+.001)
    pad(stop=True)
    wait(lambda s:not s['active'] and all(j['status']==0 for j in s['joints']))
    pad(gripper_object_rad=None)
    assert not errors,errors
    browser.close()
    print('PASS simulated speed replay -> ACTIVE pause, seven enabled, frozen arm, gradual J7 unload, 1.5 threshold, neutral/LB rearm, manual B stop, mobile; NO real motion')
