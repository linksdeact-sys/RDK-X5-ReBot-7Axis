"""Exercise only the single-axis validation workflow on simulated motors."""

import json
import time

from playwright.sync_api import expect, sync_playwright

SIM = "http://192.168.127.10:8766"

with sync_playwright() as p:
    browser = p.chromium.launch(channel="msedge", headless=True)
    context = browser.new_context(viewport={"width": 1440, "height": 1100})
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(SIM)
    expect(page.locator("#online-count")).to_have_text("7 / 7 在线")
    token = context.request.get(SIM + "/api/state").json()["csrf_token"]
    headers = {"Origin": SIM, "X-Arm-Token": token}

    def api(path, body):
        response = context.request.post(SIM + path, data=body, headers=headers)
        assert response.ok, response.text()
        return response.json()

    def state():
        return context.request.get(SIM + "/api/state").json()

    def idle(*, controls_available=True):
        deadline = time.monotonic() + 10
        while state()["active"] is not None or any(j["status"] != 0 for j in state()["joints"]):
            assert time.monotonic() < deadline
            page.wait_for_timeout(100)
        if controls_available:
            expect(page.locator("#positive")).to_be_enabled()
        else:
            expect(page.locator("#positive")).to_be_disabled()

    api("/api/sim-input", {})
    page.locator("#teleop-mode").select_option("single-axis")
    body = {"joint": 1, "degrees": 1, "kind": "gamepad", "owner": "qa-local-mit",
            "request_id": f"qa-gate-{time.time_ns()}", "gravity_profile": "j4_75",
            "input_verified": True, "teleop_mode": "single-axis"}
    assert context.request.post(SIM + "/api/test", data=body, headers=headers).status == 400
    assert context.request.post(SIM + "/api/test", data={**body, "safety_revision": "continuous-mit-v3"}, headers=headers).status == 400
    body["safety_revision"] = "continuous-mit-v11"
    assert context.request.post(SIM + "/api/test", data={**body, "joint": 7}, headers=headers).status == 400
    expect(page.locator("#gamepad-start")).to_be_disabled()
    page.locator("#input-verified").check()
    api("/api/sim-input", {"deadman": True})
    assert context.request.post(SIM + "/api/test", data=body, headers=headers).status == 409
    api("/api/sim-input", {})
    expect(page.locator("#gamepad-start")).to_be_enabled()
    page.locator("#gamepad-start").click()
    expect(page.locator("#gamepad-start")).to_have_text("停止手柄控制并失能")
    deadline=time.monotonic()+5
    while state()['active']['teleop'].get('need_rearm',True):
        assert time.monotonic()<deadline
        page.wait_for_timeout(40)
    before = state()
    api("/api/sim-input", {"deadman": True, "intent": [1.0]*7})
    page.wait_for_timeout(2500)
    moving = state()
    assert moving["active"]["revision"] == "continuous-mit-v11"
    assert moving["active"]["tau_scale"] == [1, 1, 1, 0.75, 1, 1]
    assert moving["joints"][0]["angle"] > before["joints"][0]["angle"] + 0.2
    assert moving["joints"][0]["angle"]-before["joints"][0]["angle"] < 3.5
    for i in range(1, 7):
        assert abs(moving["joints"][i]["angle"]-before["joints"][i]["angle"]) < 1e-6
    assert context.request.post(SIM + "/api/gripper/capture", data={"end": "open"}, headers=headers).status == 409
    api("/api/sim-input", {})
    page.wait_for_timeout(1500)
    held = [j["angle"] for j in state()["joints"]]
    page.wait_for_timeout(300)
    assert max(abs(j["angle"]-q) for j, q in zip(state()["joints"], held)) < 0.001
    api("/api/sim-input", {"connected": False})
    page.wait_for_timeout(250)
    assert state()["active"] is not None
    api("/api/sim-input", {"deadman": True, "intent": [1.0]*7})
    page.wait_for_timeout(250)
    assert state()["active"]["teleop"]["need_rearm"]
    assert max(abs(j["angle"]-q) for j, q in zip(state()["joints"], held)) < 0.001
    api("/api/sim-input", {"stop": True})
    idle()
    assert state()["result"]["error"] is None
    api("/api/sim-input", {})
    page.get_by_role("button", name="选择 J7 夹爪", exact=True).first.click()
    expect(page.locator("#gamepad-start")).to_be_disabled()
    expect(page.locator("#gripper-enable")).to_be_disabled()
    expect(page.locator("#positive")).to_be_disabled()
    page.get_by_role("button", name="选择 J4 腕部俯仰", exact=True).first.click()
    page.locator("#gamepad-start").click()
    expect(page.locator("#gamepad-start")).to_have_text("停止手柄控制并失能")
    page.wait_for_timeout(150)
    api("/api/sim-input", {"rx_fault_axis": 4})
    idle(controls_available=False)
    assert "J4真实CAN反馈" in state()["result"]["error"]
    expect(page.locator("#online-count")).to_have_text("6 / 7 在线")
    api("/api/sim-input", {})
    idle()
    assert not errors, errors
    print(json.dumps({"passed": ["old-client-rejected", "gripper-locked", "neutral-LB-gate",
                                  "single-axis-only", "three-degree-bound", "release-hold",
                                  "disconnect-hold", "reconnect-rearm", "B-stop", "RX-stale-fault",
                                  "no-js-errors"]}))
    context.close()
    browser.close()
