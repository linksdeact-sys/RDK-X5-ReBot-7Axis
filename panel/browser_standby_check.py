"""Real page GETs only. Connect-button actions run ONLY on simulation :8766."""
import json
from pathlib import Path
from playwright.sync_api import expect, sync_playwright

ROOT = Path(__file__).resolve().parent
REAL = 'http://192.168.127.10:8765'
SIM = 'http://192.168.127.10:8766'

with sync_playwright() as p:
    browser = p.chromium.launch(channel='msedge', headless=True)
    try:
        real = browser.new_page(viewport={'width': 1440, 'height': 1100})
        errors, real_posts = [], []
        real.on('pageerror', lambda e: errors.append(str(e)))
        real.on('request', lambda r: real_posts.append(r.url) if r.method == 'POST' else None)
        # The app polls continuously; explicit rendered-state readiness is used
        # instead of networkidle, which would never settle here.
        real.goto(REAL, wait_until='domcontentloaded')
        expect(real.locator('#connection')).not_to_contain_text('正在连接')
        state = real.request.get(REAL+'/api/state').json()
        if state['connection']['state'] == 'STANDBY':
            expect(real.locator('#hardware-connect')).to_be_visible()
            expect(real.locator('#connect-hardware')).to_be_disabled()
            expect(real.locator('#positive')).to_be_disabled()
        assert real.evaluate('document.documentElement.scrollWidth <= innerWidth')
        real.screenshot(path=str(ROOT/'standby-desktop.png'), full_page=True)
        real.set_viewport_size({'width': 390, 'height': 844})
        assert real.evaluate('document.documentElement.scrollWidth <= innerWidth')
        real.screenshot(path=str(ROOT/'standby-mobile.png'), full_page=True)
        assert not real_posts, real_posts
        real.close()

        sim = browser.new_page()
        sim.on('pageerror', lambda e: errors.append(str(e)))
        sim.goto(SIM, wait_until='domcontentloaded')
        expect(sim.locator('#hardware-connect')).to_be_visible()
        expect(sim.locator('#connect-hardware')).to_be_disabled()
        baseline = sim.request.get(SIM+'/api/state').json()
        headers = {'Origin': SIM, 'X-Arm-Token': baseline['csrf_token']}
        bad = sim.request.post(SIM+'/api/connect', data={'supported': False}, headers=headers)
        assert bad.status == 400
        assert sim.request.get(SIM+'/api/state').json()['connection']['state'] == 'STANDBY'
        sim.locator('#connect-supported').check()
        expect(sim.locator('#connect-hardware')).to_be_enabled()
        sim.locator('#connect-hardware').click()
        expect(sim.locator('#online-count')).to_have_text('7 / 7 在线')
        expect(sim.locator('#hardware-connect')).to_be_hidden()
        final = sim.request.get(SIM+'/api/state').json()
        assert final['active'] is None
        assert all(j['status'] == 0 for j in final['joints'])
        expect(sim.locator('#gamepad-start')).to_be_disabled()
        assert not errors, errors
        print(json.dumps({'passed':['real-GET-only','desktop-mobile-layout','standby-controls-locked',
            'support-confirm-required','sim-connect-handshake','no-motion-auto-start','no-js-errors']}))
        sim.close()
    finally:
        browser.close()
