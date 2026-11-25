"""Browser regression ONLY against simulated motors on port 8766."""
import json
import time
from playwright.sync_api import expect, sync_playwright

BASE = 'http://192.168.127.10:8766'
with sync_playwright() as p:
    browser = p.chromium.launch(channel='msedge', headless=True)
    page = browser.new_page(viewport={'width': 1440, 'height': 1100})
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.goto(BASE)
    expect(page.locator('#online-count')).to_have_text('7 / 7 在线')
    expect(page.locator('#teleop-mode')).to_have_value('continuous')
    def state():
        return page.request.get(BASE+'/api/state').json()
    token = state()['csrf_token']
    headers = {'Origin': BASE, 'X-Arm-Token': token}
    def api(path, body):
        response = page.request.post(BASE+path, data=body, headers=headers)
        assert response.ok, response.text()
    api('/api/sim-input', {'gripper_object_rad': None})
    page.locator('#gravity-profile').select_option('j4_70')
    page.locator('#input-verified').check()
    page.locator('#gamepad-start').click()
    expect(page.locator('#gamepad-start')).to_have_text('停止手柄控制并失能')
    deadline=time.monotonic()+5
    while state()['active']['teleop'].get('need_rearm',True):
        assert time.monotonic()<deadline
        page.wait_for_timeout(40)
    initial = state()
    assert initial['active']['teleop_mode'] == 'continuous'
    assert initial['active']['tau_scale'] == [1, 1, 1, .7, 1, 1]
    assert initial['active']['teleop']['mode'] == 'continuous'
    api('/api/sim-input', {'deadman': True, 'intent': [1, -1, -1, 1, 1, 1, 1]})
    page.wait_for_timeout(2300)
    moving = state()
    for i in range(6):
        assert abs(moving['joints'][i]['angle']-initial['joints'][i]['angle']) > 4, (i, moving)
    assert moving['joints'][6]['angle'] == initial['joints'][6]['angle']
    # Reversal needs no mode restart or neutral/LB rearm.
    api('/api/sim-input', {'deadman': True, 'intent': [-1, 1, 1, -1, -1, -1, 0]})
    page.wait_for_timeout(3000)
    reversed_pose = state()
    assert reversed_pose['joints'][0]['angle'] < moving['joints'][0]['angle']-5
    api('/api/sim-input', {})
    page.wait_for_timeout(1600)
    held = state()
    page.wait_for_timeout(400)
    assert max(abs(a['angle']-b['angle']) for a,b in zip(held['joints'],state()['joints'])) < .01
    api('/api/sim-input', {'connected': False})
    page.wait_for_timeout(250)
    api('/api/sim-input', {'deadman': True, 'intent': [1]*7})
    page.wait_for_timeout(250)
    assert state()['active']['teleop']['need_rearm']
    api('/api/sim-input', {'stop': True})
    deadline = time.monotonic()+5
    while state()['active'] or any(j['status'] != 0 for j in state()['joints']):
        assert time.monotonic() < deadline
        page.wait_for_timeout(100)
    assert all(j['status'] == 0 for j in state()['joints'])
    api('/api/sim-input', {})
    page.locator('#teleop-mode').select_option('single-axis')
    expect(page.locator('#input-verified')).not_to_be_checked()
    assert not errors, errors
    print(json.dumps({'passed':['continuous-default','selected-profile','six-axis-over-three-degrees',
        'gripper-locked','reverse-without-restart','release-hold','reconnect-neutral-gate','B-disable','mode-confirm-reset','no-js-errors']}))
    browser.close()
