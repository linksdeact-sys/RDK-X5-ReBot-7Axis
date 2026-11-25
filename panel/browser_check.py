"""Exercise motion controls only on the simulated server; inspect hardware read-only."""

import json
import time
from pathlib import Path

from playwright.sync_api import expect, sync_playwright


ROOT = Path(__file__).resolve().parent
SIM = "http://192.168.127.10:8766"
REAL = "http://192.168.127.10:8765"


def wait_idle(page):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        page.wait_for_timeout(200)
        state = page.context.request.get(SIM + "/api/state").json()
        if state["active"] is None and all(joint["status"] == 0 for joint in state["joints"]):
            break
    else:
        raise AssertionError("Simulated motion did not finish")
    expect(page.locator("#positive")).to_be_enabled(timeout=5000)


with sync_playwright() as p:
    browser = p.chromium.launch(channel="msedge", headless=True)
    context = browser.new_context(viewport={"width": 1440, "height": 1100})
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(SIM)
    page.wait_for_load_state("domcontentloaded")
    expect(page.locator("#online-count")).to_have_text("7 / 7 在线")
    expect(page.locator("#positive")).to_be_enabled()
    page.locator("#show-zero").click()
    expect(page.locator("#zero-dialog")).to_be_visible()
    page.keyboard.press("Escape")
    expect(page.locator("#zero-dialog")).not_to_be_visible()
    token = page.locator('meta[name="arm-token"]').get_attribute("content")
    headers = {"Origin": SIM, "X-Arm-Token": token}
    base_job = {"joint": 1, "degrees": 1, "kind": "step", "owner": "qa", "request_id": "qa-invalid"}
    assert context.request.post(SIM + "/api/test", data=base_job).status == 403
    for patch in ({"joint": True}, {"degrees": 99}, {"request_id": "../../bad"}, {"kind": "unknown"},
                  {"kind": "gravity", "gravity_profile": "invalid"},
                  {"kind": "gravity", "gravity_profile": []}):
        assert context.request.post(SIM + "/api/test", data={**base_job, **patch}, headers=headers).status == 400

    page.get_by_role("button", name="选择 J2 大臂", exact=True).click()
    expect(page.locator("#joint-title")).to_have_text("大臂")
    page.get_by_role("button", name="选择 J1 基座旋转", exact=True).first.click()
    page.get_by_role("button", name="1°", exact=True).click()
    page.locator("#roundtrip").click()
    expect(page.locator("#positive")).to_be_disabled()
    busy = context.request.post(SIM + "/api/test", data={**base_job, "request_id": "qa-busy"}, headers=headers)
    assert busy.status == 409
    wait_idle(page)
    result = context.request.get(SIM + "/api/state").json()["result"]
    assert result["error"] is None and result["peak"] > 0.9 and abs(result["delta"]) < 0.1, result

    expect(page.locator("#gravity-profile")).to_have_value("j4_75")
    page.locator("#gravity-profile").select_option("official")
    page.locator("#gravity-test").click()
    expect(page.locator("#positive")).to_be_disabled()
    expect(page.locator("#gravity-test")).to_have_text("停止重力补偿")
    page.wait_for_timeout(13500)
    state = context.request.get(SIM + "/api/state").json()
    assert state["active"]["mode"] == "continuous" and state["active"]["elapsed"] > 12, state["active"]
    assert state["active"]["tau_scale"] == [1.0] * 6
    assert all(joint["status"] == 1 for joint in state["joints"])
    page.close()
    time.sleep(1.4)
    assert context.request.get(SIM + "/api/state").json()["active"]["kind"] == "gravity"
    page = context.new_page()
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(SIM)
    expect(page.locator("#gravity-test")).to_have_text("停止重力补偿")
    page.locator("#gravity-test").click()
    wait_idle(page)
    gravity = context.request.get(SIM + "/api/state").json()["result"]
    assert gravity["kind"] == "gravity" and gravity["error"] is None, gravity
    page.locator("#gravity-profile").select_option("measured")
    page.locator("#gravity-test").click()
    expect(page.locator("#gravity-test")).to_have_text("停止重力补偿")
    state = context.request.get(SIM + "/api/state").json()
    assert state["active"]["tau_scale"][3] == 0.5
    page.keyboard.press("Escape")
    wait_idle(page)

    profiles = context.request.get(SIM + "/api/state").json()["gravity_profiles"]
    assert len(profiles) == 15
    for name, profile in sorted(profiles.items(), key=lambda item: item[1]["scales"][3]):
        page.locator("#gravity-profile").select_option(name)
        expect(page.locator("#gravity-note")).to_contain_text(f"J4 {round(profile['scales'][3]*100)}%")
        page.locator("#gravity-test").click()
        expect(page.locator("#gravity-test")).to_have_text("停止重力补偿")
        expect(page.locator("#gravity-profile")).to_be_disabled()
        deadline = time.monotonic() + 5
        while True:
            state = context.request.get(SIM + "/api/state").json()
            if state["active"] and "tau_scale" in state["active"]:
                break
            assert time.monotonic() < deadline
            page.wait_for_timeout(100)
        assert state["active"]["tau_scale"] == profile["scales"]
        assert all(value == 1.0 for index, value in enumerate(profile["scales"]) if index != 3)
        page.locator("#gravity-test").click()
        wait_idle(page)
        state = context.request.get(SIM + "/api/state").json()
        assert state["result"]["tau_scale"] == profile["scales"] and state["result"]["error"] is None

    page.locator("#direction-options").get_by_role("button", name="俯视逆时针", exact=True).click()
    page.locator("#save-direction").click()
    expect(page.locator("#direction-status")).to_have_text("已记录")
    page.reload()
    page.wait_for_load_state("domcontentloaded")
    expect(page.locator("#direction-status")).to_have_text("已记录")
    page.locator("#negative").click()
    expect(page.locator("#positive")).to_be_disabled()
    page.keyboard.press("Escape")
    wait_idle(page)
    state = context.request.get(SIM + "/api/state").json()
    assert all(joint["status"] == 0 for joint in state["joints"])

    page.route("**/api/state", lambda route: route.abort())
    expect(page.locator("#connection")).to_contain_text("页面重连中", timeout=5000)
    expect(page.locator("#positive")).to_be_disabled()
    page.unroute("**/api/state")
    expect(page.locator("#connection")).to_contain_text("仿真连接", timeout=5000)
    expect(page.locator("#positive")).to_be_enabled()

    page.locator("#positive").click()
    expect(page.locator("#positive")).to_be_disabled()
    page.close()
    time.sleep(1.8)
    state = context.request.get(SIM + "/api/state").json()
    assert state["active"] is None and "超时" in state["result"]["error"], state
    assert all(joint["status"] == 0 for joint in state["joints"])

    real = context.new_page()
    real.on("pageerror", lambda error: errors.append(str(error)))
    real.goto(REAL)
    real.wait_for_load_state("domcontentloaded")
    expect(real.locator("#online-count")).to_have_text("7 / 7 在线", timeout=5000)
    expect(real.locator("#connection")).to_contain_text("RDK 已连接")
    assert real.evaluate("document.documentElement.scrollWidth <= innerWidth")
    real.screenshot(path=str(ROOT / "desktop.png"), full_page=True)
    mobile = browser.new_context(viewport={"width": 390, "height": 844}, device_scale_factor=1)
    mobile_page = mobile.new_page()
    mobile_page.goto(REAL)
    mobile_page.wait_for_load_state("domcontentloaded")
    expect(mobile_page.locator("#online-count")).to_have_text("7 / 7 在线", timeout=5000)
    assert mobile_page.evaluate("document.documentElement.scrollWidth <= innerWidth")
    mobile_page.screenshot(path=str(ROOT / "mobile.png"), full_page=True)
    mobile_page.locator(".direction-panel").scroll_into_view_if_needed()
    assert 0 <= mobile_page.locator("#stop").bounding_box()["y"] < 100
    assert not errors, errors
    mobile.close()
    context.close()
    browser.close()
    print(json.dumps({"passed": ["invalid-input", "csrf", "joint-selection", "roundtrip", "busy-rejection",
                                  "continuous-past-12s", "gravity-survives-page-close", "all-15-j4-profiles",
                                  "direction-save", "escape-stop", "network-recovery", "heartbeat-timeout",
                                  "real-read-only", "desktop-layout", "mobile-layout", "no-js-errors"]}))
