"""Teaching UX regression. 8766 simulation only; no real CAN or USB."""
from time import monotonic
from pathlib import Path
from playwright.sync_api import sync_playwright,expect

URL='http://192.168.127.10:8766'
with sync_playwright() as pw:
    browser=pw.chromium.launch(channel='msedge',headless=True)
    page=browser.new_page(viewport={'width':1440,'height':1100})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(URL,wait_until='domcontentloaded')
    expect(page.locator('#online-count')).to_have_text('7 / 7 在线')
    def state():
        s=page.request.get(URL+'/api/state').json();assert s['simulate'];return s
    first=state();assert not first['active']
    assert first['teaching']['revision']=='hand-teach-v5-workflow'
    headers={'Origin':URL,'X-Arm-Token':first['csrf_token']}
    def api(path,body):
        assert state()['simulate']
        r=page.request.post(URL+path,data=body,headers=headers);assert r.ok,r.text()
    def wait(predicate,seconds=20):
        end=monotonic()+seconds
        while monotonic()<end:
            s=state()
            if predicate(s):return s
            page.wait_for_timeout(30)
        raise AssertionError(str(s))
    def teach(s):return s['teaching']['state'] or {}
    expect(page.locator('#teach-note')).to_contain_text('不需要重新录制')
    expect(page.locator('#teach-start')).to_be_disabled()
    page.locator('#teach-supported').check();page.locator('#teach-start').click()
    wait(lambda s:teach(s).get('mode')=='DRAG' and teach(s).get('hold_blend')==0.)
    q=[j['angle']*3.141592653589793/180 for j in state()['joints']]
    page.locator('#teach-record').click();wait(lambda s:teach(s).get('recording'))
    for k in range(35):
        moved=[x+(k+1)*.0006*(-1 if i in (1,2) else 2 if i==6 else 1) for i,x in enumerate(q)]
        api('/api/sim-input',{'connected':False,'teaching_q':moved})
        page.wait_for_timeout(35)
    page.locator('#teach-finish').click()
    saved=wait(lambda s:teach(s).get('mode')=='HOLD' and s['teaching']['records'][0]['id'] not in [r['id'] for r in first['teaching']['records']])
    record=saved['teaching']['records'][0];assert record['playable']
    page.locator('#teach-files').select_option(record['id'])
    # Replay directly from DRAG, without manually pausing or waiting for HOLD.
    page.locator('#teach-drag').click();wait(lambda s:teach(s).get('mode')=='DRAG' and teach(s).get('hold_blend')==0.)
    api('/api/sim-input',{'connected':False,'teaching_q':None})
    page.locator('#teach-clear').check();expect(page.locator('#teach-play')).to_be_enabled()
    page.locator('#teach-play').click();wait(lambda s:teach(s).get('mode')=='REPLAY')
    page.locator('#teach-pause').click();wait(lambda s:teach(s).get('mode')=='HOLD')
    # Stop session; old record must replay from IDLE without opening hand-teach.
    page.locator('#stop').click();wait(lambda s:not s['active'] and all(j['status']==0 for j in s['joints']))
    expect(page.locator('#teach-note')).not_to_contain_text('用户暂停')
    page.locator('#teach-clear').check();expect(page.locator('#teach-play')).to_be_enabled()
    page.locator('#teach-play').click();wait(lambda s:teach(s).get('mode')=='REPLAY')
    wait(lambda s:teach(s).get('mode')=='HOLD' and '回放完成' in (teach(s).get('reason') or ''),25)
    page.locator('#stop').click();wait(lambda s:not s['active'] and all(j['status']==0 for j in s['joints']))
    # Force one request rejection: its text must survive polling until ack.
    page.route('**/api/teaching',lambda route:route.fulfill(status=400,json={'error':'QA：回放文件校验失败，提示不得消失'}))
    page.locator('#teach-clear').check();page.locator('#teach-play').click()
    expect(page.locator('#teach-error')).to_be_visible()
    page.wait_for_timeout(800)
    expect(page.locator('#teach-error-text')).to_contain_text('提示不得消失')
    assert not state()['active']
    page.unroute('**/api/teaching')
    page.locator('#teach-error-ack').click();expect(page.locator('#teach-error')).to_be_hidden()
    # Metadata from a historical invalid complete recording is rejected visibly.
    bad=next(r for r in state()['teaching']['records'] if r['name']=='QA invalid historical')
    assert bad['complete'] and not bad['playable']
    page.locator('#teach-files').select_option(bad['id']);page.locator('#teach-clear').check()
    expect(page.locator('#teach-play')).to_be_disabled()
    expect(page.locator('#teach-play-reason')).to_contain_text('位置差分')
    expect(page.locator('#teach-align')).to_contain_text('最大误差 J')
    # Backend error persists even after page reload, until explicit ack.
    r=page.request.post(URL+'/api/teaching',data={'action':'play','revision':'hand-teach-v5-workflow','confirmed':False},headers=headers)
    assert r.status==400
    page.reload(wait_until='domcontentloaded')
    expect(page.locator('#teach-error')).to_be_visible()
    page.wait_for_timeout(800);expect(page.locator('#teach-error-text')).to_contain_text('本次回放')
    page.screenshot(path=str(Path('panel/teaching-workflow-desktop.png').resolve()),full_page=True)
    page.set_viewport_size({'width':390,'height':844})
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
    page.screenshot(path=str(Path('panel/teaching-workflow-mobile.png').resolve()),full_page=True)
    page.locator('#teach-error-ack').click();expect(page.locator('#teach-error')).to_be_hidden()
    assert not state()['active'] and not errors,errors
    browser.close()
    print('PASS SIM: record/save, play from DRAG and IDLE without manual HOLD, explicit confirm, end hold, invalid record reason, persistent local/backend errors+ack, mobile; NO real motion')
