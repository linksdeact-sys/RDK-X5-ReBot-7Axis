"""Read-only simulation GET plus mocked recovery POST; never real motors."""
import copy
from playwright.sync_api import sync_playwright,expect

URL='http://192.168.127.10:8766'
with sync_playwright() as pw:
    browser=pw.chromium.launch(channel='msedge',headless=True)
    page=browser.new_page(viewport={'width':1280,'height':1000})
    base=page.request.get(URL+'/api/state').json();assert base['simulate'] and not base['active']
    mock=copy.deepcopy(base);mock['connection'].update(ready=True,state='READY')
    for j in mock['joints']:j.update(status=0,online=True,feedback_age_ms=20.,velocity=0.)
    mock['joints'][6]['status']=13
    mock['recovery'].update(fresh=True,faults=[{'joint':7,'code':13,'name':'通信超时报警'}],targets=[7],can_request=True,all_disabled=False,busy=False,message=None,blocker='J7 通信超时报警（状态13）')
    mock['gripper'].update(can_start=False,ready=True)
    mock['result']={'id':'historical','kind':'teach','joint':1,'axes':list(range(1,8)),'peak':0.,'delta':None,'error':'J1真实CAN反馈超过150ms未更新'}
    posts=[];errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    def api(route):
        if route.request.method=='GET':
            if not route.request.url.endswith('/api/state'):
                route.continue_();return
            route.fulfill(json=mock);return
        posts.append(route.request.url)
        assert route.request.url.endswith('/api/recovery')
        assert route.request.post_data_json['supported'] is True
        mock['joints'][6]['status']=0;mock['gripper']['can_start']=True
        mock['recovery'].update(faults=[],targets=[],can_request=False,all_disabled=True,blocker=None,message='通信报警已解除，七轴新反馈确认失能；等待手动启动，不恢复旧任务')
        route.fulfill(json={'accepted':True})
    page.route('**/api/**',api)
    page.goto(URL,wait_until='domcontentloaded')
    expect(page.locator('#banner')).to_contain_text('七轴反馈已恢复')
    expect(page.locator('#banner')).to_contain_text('J7 通信超时报警')
    expect(page.locator('#banner')).not_to_contain_text('J1真实CAN反馈超过150ms未更新')
    page.locator('#teach-supported').check()
    expect(page.locator('#teach-start')).to_be_disabled()
    expect(page.locator('#teach-start-reason')).to_contain_text('J7')
    expect(page.locator('#recover-communication')).to_be_disabled()
    page.locator('#recovery-supported').check();page.locator('#recover-communication').click()
    expect(page.locator('#recovery-note')).to_contain_text('七轴新反馈确认失能')
    expect(page.locator('#teach-start')).to_be_enabled()
    expect(page.locator('#banner')).to_contain_text('当前七轴反馈已恢复')
    assert posts==[URL+'/api/recovery'] and not errors,(posts,errors)
    mock['joints'][6]['status']=10
    mock['recovery'].update(faults=[{'joint':7,'code':10,'name':'过流'}],can_request=False,blocker='J7过流，不能清除',message=None)
    expect(page.locator('#banner')).to_contain_text('过流')
    expect(page.locator('#recover-communication')).to_be_disabled()
    expect(page.locator('#teach-start')).to_be_disabled()
    page.set_viewport_size({'width':390,'height':844})
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
    browser.close()
    print('PASS SIM/mock: recovered feedback distinct from J7 alarm; explicit supported recovery only, fresh disabled unlock, no automatic start, other faults blocked, mobile')
