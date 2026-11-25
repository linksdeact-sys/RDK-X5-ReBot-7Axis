"""Hand-teach workflow on 8766 SIMULATION ONLY, never real motors."""
from time import monotonic
from pathlib import Path
from playwright.sync_api import sync_playwright,expect

URL='http://192.168.127.10:8766'
with sync_playwright() as pw:
    browser=pw.chromium.launch(channel='msedge',headless=True)
    page=browser.new_page(viewport={'width':1440,'height':1000})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(URL,wait_until='domcontentloaded')
    expect(page.locator('#online-count')).to_have_text('7 / 7 在线')
    def state():
        s=page.request.get(URL+'/api/state').json();assert s['simulate'];return s
    first=state();headers={'Origin':URL,'X-Arm-Token':first['csrf_token']}
    assert first['teaching']['revision']=='hand-teach-v5-workflow'
    assert first['teaching']['align_deg']==15.
    assert first['teaching']['compliance']['drag_soft_speed_pause'] is False
    assert first['teaching']['compliance']['drag_tracking_error_pause'] is False
    assert first['teaching']['compliance']['hard_speed_rad_s']==1.5
    assert first['teaching']['compliance']['arm_drag_kp']==2.
    assert first['teaching']['compliance']['arm_drag_kd']==1.
    def api(path,body):
        assert state()['simulate']
        r=page.request.post(URL+path,data=body,headers=headers);assert r.ok,r.text()
    def wait(check,seconds=20):
        end=monotonic()+seconds
        while monotonic()<end:
            s=state()
            if check(s):return s
            page.wait_for_timeout(30)
        raise AssertionError(str(s))
    def teach(s):return s['teaching']['state'] or {}
    expect(page.locator('#teach-start')).to_be_disabled()
    page.locator('#teach-supported').check();page.locator('#teach-start').click()
    wait(lambda s:teach(s).get('mode')=='DRAG')
    expect(page.locator('#teach-note')).to_contain_text('Kp=2 / Kd=1')
    q=[j['angle']*3.141592653589793/180 for j in state()['joints']]
    api('/api/sim-input',{'connected':False,'teaching_q':q})
    page.locator('#teach-record').click()
    wait(lambda s:teach(s).get('recording'))
    for k in range(50):
        delta=(k+1)*.0006
        moved=[q[0]+delta,q[1]-delta,q[2]-delta,q[3]+delta,q[4]-delta,q[5]+delta,q[6]+2*delta]
        api('/api/sim-input',{'connected':False,'teaching_q':moved})
        page.wait_for_timeout(35)
        s=state();assert teach(s).get('recording')
        assert s['active']['teleop']['gripper_command_nm']==0.
    page.locator('#teach-finish').click()
    saved=wait(lambda s:teach(s).get('mode')=='HOLD' and teach(s).get('hold_blend')==1. and len(s['teaching']['records'])>len(first['teaching']['records']))
    record=saved['teaching']['records'][0]
    assert record['frames']>30 and len(record['first_q'])==7
    assert saved['active'] and all(j['status']==1 for j in saved['joints'])
    page.locator('#teach-clear').check()
    expect(page.locator('#teach-play')).to_be_enabled() # Old ~3deg offset is now allowed.
    page.locator('#teach-drag').click();wait(lambda s:teach(s).get('mode')=='DRAG' and teach(s).get('hold_blend')==0.)
    target=record['first_q'][:];target[6]+=.29 # >15deg must still be blocked.
    for k in range(30):
        p=[a+(b-a)*(k+1)/30 for a,b in zip(moved,target)]
        api('/api/sim-input',{'connected':False,'teaching_q':p});page.wait_for_timeout(30)
    page.locator('#teach-pause').click()
    wait(lambda s:teach(s).get('mode')=='HOLD' and teach(s).get('hold_blend')==1.)
    page.locator('#teach-clear').check();expect(page.locator('#teach-play')).to_be_disabled()
    expect(page.locator('#teach-align')).to_contain_text('需≤15°')
    page.locator('#teach-drag').click();wait(lambda s:teach(s).get('mode')=='DRAG' and teach(s).get('hold_blend')==0.)
    allowed=record['first_q'][:];allowed[0]+=.253;allowed[6]+=.253 # ~14.5deg in arm and jaw.
    for k in range(30):
        p=[a+(b-a)*(k+1)/30 for a,b in zip(target,allowed)]
        api('/api/sim-input',{'connected':False,'teaching_q':p});page.wait_for_timeout(30)
    page.locator('#teach-pause').click()
    wait(lambda s:teach(s).get('mode')=='HOLD' and teach(s).get('hold_blend')==1.)
    page.locator('#teach-clear').check();expect(page.locator('#teach-play')).to_be_enabled()
    page.locator('#teach-play').click();wait(lambda s:teach(s).get('mode')=='REPLAY')
    wait(lambda s:teach(s).get('joining_start') is True)
    done=wait(lambda s:teach(s).get('mode')=='HOLD' and '回放完成' in (teach(s).get('reason') or ''),30)
    assert done['joints'][6]['angle']>record['first_q'][6]*180/3.141592653589793+1.
    assert done['active'] and all(j['status']==1 for j in done['joints'])
    page.screenshot(path=str(Path('panel/teaching-desktop-qa.png').resolve()),full_page=True)
    page.set_viewport_size({'width':390,'height':844})
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
    page.screenshot(path=str(Path('panel/teaching-mobile-qa.png').resolve()),full_page=True)
    page.locator('#stop').click();wait(lambda s:not s['active'] and all(j['status']==0 for j in s['joints']))
    api('/api/sim-input',{'teaching_q':None})
    assert not errors,errors
    browser.close()
    print('PASS seven-axis teaching; >15deg blocked, ~14.5deg arm/J7 offset allowed then low-speed join before replay; end hold and mobile; SIM ONLY')
