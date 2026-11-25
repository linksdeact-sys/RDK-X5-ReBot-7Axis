"""Test reconnect UI using mocked status responses. Sends no motion requests."""
import copy
import json

from playwright.sync_api import expect, sync_playwright

BASE = "http://192.168.127.10:8766"
with sync_playwright() as p:
    browser = p.chromium.launch(channel="msedge", headless=True)
    page = browser.new_page(viewport={"width": 1440, "height": 1100})
    errors, starts = [], []
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.on("request", lambda request: starts.append(request.url) if request.method == "POST" and request.url.endswith("/api/test") else None)
    page.goto(BASE)
    expect(page.locator("#online-count")).to_have_text("7 / 7 在线")
    page.locator("#input-verified").check()
    expect(page.locator("#gamepad-start")).to_be_enabled()
    baseline = page.request.get(BASE+"/api/state").json()
    mocked = copy.deepcopy(baseline)
    mocked["active"] = None
    mocked["error"] = "USB disconnected"
    mocked["connection"].update(ready=False, state="WAIT_USB")
    for joint in mocked["joints"]:
        joint.update(online=False, status=None, angle=None)
    page.route("**/api/state", lambda route: route.fulfill(json=mocked))
    expect(page.locator("#connection")).to_contain_text("等待 USB2CAN 插入")
    expect(page.locator("#input-verified")).not_to_be_checked()
    for button in ("positive", "negative", "roundtrip", "gravity-test", "gamepad-start"):
        expect(page.locator("#"+button)).to_be_disabled()
    mocked = copy.deepcopy(baseline)
    mocked["connection"].update(ready=False, state="VERIFY_DISABLED", session_id="fresh-session")
    expect(page.locator("#connection")).to_contain_text("确认七轴失能")
    expect(page.locator("#positive")).to_be_disabled()
    mocked["connection"].update(ready=True, state="READY")
    mocked["error"] = None
    expect(page.locator("#connection")).to_contain_text("仿真连接")
    expect(page.locator("#positive")).to_be_enabled()
    expect(page.locator("#gamepad-start")).to_be_disabled()
    mocked["connection"]["stopping"] = True
    expect(page.locator("#positive")).to_be_disabled()
    expect(page.locator("#phase")).to_contain_text("正在停止")
    mocked["connection"]["stopping"] = False
    mocked["joints"][0]["online"] = False
    expect(page.locator("#positive")).to_be_disabled()
    expect(page.locator("#online-count")).to_have_text("6 / 7 在线")
    assert not starts, starts
    assert not errors, errors
    print(json.dumps({"passed": ["wait-usb-visible", "stale-feedback-cleared", "controls-locked",
                                  "fresh-disable-handshake-visible", "new-session-requires-confirmation",
                                  "stop-lock", "stale-axis-lock", "no-motion-posts", "no-js-errors"]}))
    browser.close()
