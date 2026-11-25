"""Verify the viewer against real URDF assets and mock feedback. Never sends POST."""

import os
from pathlib import Path
from playwright.sync_api import expect, sync_playwright

ROOT = Path(__file__).resolve().parent
BASE = f"http://192.168.127.10:{os.environ.get('REBOT_VIEW_TEST_PORT', '8766')}"

with sync_playwright() as playwright:
    browser = playwright.chromium.launch(channel="msedge", headless=True,
        args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
    page = browser.new_page(viewport={"width": 1440, "height": 1000})
    errors, unexpected_requests = [], []
    page.on("pageerror", lambda error: errors.append(str(error)))

    def read_only(route):
        if route.request.method != "GET" or not route.request.url.startswith(BASE + "/"):
            unexpected_requests.append(route.request.method + " " + route.request.url)
            return route.abort()
        return route.continue_()

    page.route("**/*", read_only)
    state_response = page.request.get(BASE + "/api/state")
    assert state_response.ok
    actual = state_response.json()
    assert not actual["active"]
    manifest = page.request.get(BASE + "/api/urdf-view").json()
    assert manifest["name"] == "ReBot_Arm_DM.urdf"
    assert manifest["meshes"] > 20
    assert [j["id"] for j in manifest["feedback"]] == [1, 2, 3, 4, 5, 6]
    assert page.request.get(BASE + "/urdf-view/mesh/999").status == 404
    state = {**actual, "simulate": True, "joints": [
        {**joint, "online": False, "angle": None, "status": None} for joint in actual["joints"]],
        "connection": {**actual["connection"], "state": "STANDBY", "ready": False},
        "gripper": {"points_rad": {}, "ready": False}}
    page.route("**/api/state", lambda route: route.fulfill(json=state))
    page.goto(BASE, wait_until="domcontentloaded")
    view, canvas = page.locator("#urdf-view"), page.locator("#urdf-canvas")
    gripper_status = page.locator("#urdf-gripper-state")
    j7_readout = page.locator("#joint-angle-7")

    def expect_finger_geometry(finger_mm):
        assert abs(float(view.get_attribute("data-finger-left-mm")) - finger_mm) < 1e-6
        assert abs(float(view.get_attribute("data-finger-right-mm")) + finger_mm) < 1e-6

    def frame_count():
        return int(view.get_attribute("data-frames"))

    def wait_for_new_frame(previous):
        page.wait_for_function("n => Number(document.querySelector('#urdf-view').dataset.frames) > n", arg=previous)

    expect(view).to_have_attribute("data-pose", "reference", timeout=30000)
    expect(page.locator("#arm-diagram")).to_be_hidden()
    expect(gripper_status).to_contain_text("三维仅显示全开参考")
    expect_finger_geometry(28.5)
    page.wait_for_function("Number(document.querySelector('#urdf-view').dataset.frames) >= 1")
    frames = frame_count()
    initial = canvas.screenshot()
    state["connection"].update(state="READY", ready=True)
    for joint, angle in zip(state["joints"], (35, -35, -30, 25, 15, -20, 241.3)):
        joint.update(angle=angle, online=True, status=0)
    expect(view).to_have_attribute("data-pose", "feedback")
    expect(page.locator("#urdf-status")).to_have_text("仿真反馈")
    wait_for_new_frame(frames)
    changed = canvas.screenshot()
    assert initial != changed, "Joint feedback should change the rendered model"

    expect(gripper_status).to_contain_text("收拢限位（J7 241.3°） · 开口 8.5 mm")
    expect_finger_geometry(2.75)
    before_open = frame_count()
    # RDK returns full precision; the display rounds this reading to 49.3°.
    state["joints"][6]["angle"] = 49.25462512398299
    expect(gripper_status).to_contain_text("最大张开（J7 49.3°） · 开口 60 mm")
    expect_finger_geometry(28.5)
    wait_for_new_frame(before_open)
    opened = canvas.screenshot()
    assert opened != changed, "J7 feedback alone must animate the fingers"

    state["joints"][6]["angle"] = 45
    expect(j7_readout).to_have_text("+45.0°")
    expect(gripper_status).to_contain_text("最大张开（J7 49.3°） · 开口 60 mm")
    expect_finger_geometry(28.5)

    before_half = frame_count()
    state["joints"][6]["angle"] = 145.3
    expect(gripper_status).to_contain_text("收拢行程 50% · 估算开口 34.25 mm")
    expect_finger_geometry(15.625)
    wait_for_new_frame(before_half)
    midway = canvas.screenshot()
    assert midway != opened
    assert midway != changed

    state["joints"][6].update(angle=241.3, online=False)
    expect(gripper_status).to_contain_text("等待 J7 反馈 · 保留上次显示（34.25 mm）")
    expect(view).to_have_attribute("data-pose", "feedback")
    expect_finger_geometry(15.625)
    assert canvas.screenshot() == midway

    before_limit = frame_count()
    state["joints"][6]["online"] = True
    expect(gripper_status).to_contain_text("收拢限位（J7 241.3°） · 开口 8.5 mm")
    expect_finger_geometry(2.75)
    wait_for_new_frame(before_limit)
    at_limit = canvas.screenshot()
    assert at_limit != midway

    state["joints"][6]["angle"] = 250
    expect(j7_readout).to_have_text("+250.0°")
    expect(gripper_status).to_contain_text("收拢限位（J7 241.3°） · 开口 8.5 mm")
    expect_finger_geometry(2.75)
    box = canvas.bounding_box()
    page.mouse.move(box["x"] + box["width"]/2, box["y"] + box["height"]/2)
    page.mouse.down()
    page.mouse.move(box["x"] + box["width"]/2 + 50, box["y"] + box["height"]/2 + 20, steps=5)
    page.mouse.up()
    assert canvas.screenshot() != at_limit, "Drag should orbit the model"
    page.get_by_role("button", name="重置视角").click()
    state["joints"][1]["online"] = False
    expect(view).to_have_attribute("data-pose", "stale")
    expect(page.locator("#urdf-status")).to_contain_text("保留最后姿态")
    page.set_viewport_size({"width": 390, "height": 844})
    assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
    page.screenshot(path=str(ROOT / "urdf-view-check.png"), full_page=True)
    canvas.dispatch_event("webglcontextlost")
    expect(page.locator("#arm-diagram")).to_be_visible()
    expect(view).to_be_hidden()
    assert not errors and not unexpected_requests, (errors, unexpected_requests)
    print("PASS: official STL assets, J7 measured 60/8.5 mm endpoints, midpoint and clamping, stale feedback, orbit/reset, mobile, GET only")
    browser.close()
