"""Simulated historical-result banner; never contacts production port."""
from playwright.sync_api import sync_playwright,expect
URL='http://192.168.127.10:8766'
with sync_playwright() as pw:
    browser=pw.chromium.launch(channel='msedge',headless=True)
    page=browser.new_page();errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    def history(route):
        response=route.fetch();data=response.json()
        assert data['simulate'] is True
        data['result']={'id':'old-job','kind':'gamepad','axes':[1,2,3,4,5,6,7],
            'error':'HISTORICAL_J4_SENTINEL','peak':0,'delta':None}
        route.fulfill(response=response,json=data)
    page.route('**/api/state',history)
    page.goto(URL,wait_until='domcontentloaded')
    expect(page.locator('#online-count')).to_have_text('7 / 7 在线')
    expect(page.locator('#banner')).to_contain_text('上次任务已结束')
    expect(page.locator('#banner')).to_contain_text('HISTORICAL_J4_SENTINEL')
    page.locator('#input-verified').check();page.locator('#gamepad-start').click()
    expect(page.locator('#gamepad-start')).to_have_text('停止手柄控制并失能')
    expect(page.locator('#banner')).not_to_contain_text('HISTORICAL_J4_SENTINEL')
    page.locator('#gamepad-start').click()
    expect(page.locator('#gamepad-start')).to_contain_text('启动 J1～J6 连续遥操')
    expect(page.locator('#banner')).to_contain_text('上次任务已结束')
    assert not errors,errors
    browser.close()
    print('PASS old errors labelled historical when idle, suppressed during new task, no current fault hidden; simulated motors only')
