"""Exercise J7 UI and the fake plant on port 8766 only (seeded QA endpoints)."""
from time import monotonic
from playwright.sync_api import expect, sync_playwright

SIM = "http://192.168.127.10:8766"

with sync_playwright() as pw:
    browser = pw.chromium.launch(channel="msedge", headless=True)
    page = browser.new_page(viewport={"width": 1440, "height": 1000})
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(SIM, wait_until="domcontentloaded")
    expect(page.locator("#online-count")).to_have_text("7 / 7 在线")

    def state():
        value = page.request.get(SIM + "/api/state").json()
        assert value["simulate"]
        return value

    initial = state()
    assert initial["gripper"]["ready"] and initial["gripper"]["can_start"]
    headers = {"Origin": SIM, "X-Arm-Token": initial["csrf_token"]}

    def post(path, body, status=200):
        response = page.request.post(SIM + path, data=body, headers=headers)
        assert response.status == status, response.text()

    post("/api/sim-input", {"gripper_object_rad": None})

    def wait_for(predicate, seconds=5):
        until = monotonic() + seconds
        while monotonic() < until:
            value = state()
            if predicate(value):
                return value
            page.wait_for_timeout(40)
        raise AssertionError(str({"active":value.get("active"),"result":value.get("result")}))

    def effort(data):
        return (data["active"] or {}).get("teleop", {}).get("gripper_velocity_ref_rad_s")

    opt = page.locator("#gripper-enable")
    start = page.locator("#gamepad-start")
    expect(opt).to_be_enabled()
    expect(opt).not_to_be_checked()
    expect(start).to_contain_text("J1～J6")
    post("/api/test", {"kind": "step", "joint": 7, "degrees": 1,
                       "owner": "gripper-qa", "request_id": "gripper-jog"}, status=400)
    opt.check()
    page.locator("#input-verified").check()
    expect(start).to_contain_text("J1～J7")
    post("/api/sim-input", {"triggers": {"lt": 1, "rt": 1}})
    expect(start).to_be_disabled()
    post("/api/sim-input", {})
    expect(start).to_be_enabled()
    start.click()
    expect(start).to_have_text("停止手柄控制并失能")
    wait_for(lambda data: effort(data) == 0 and not data["active"]["teleop"]["need_rearm"])

    post("/api/sim-input", {"deadman": True, "triggers": {"rt": 1, "lt": 0}})
    closing = wait_for(lambda data: (effort(data) or 0) > .02
                       and data["joints"][6]["angle"] > initial["joints"][6]["angle"] + 1.)
    assert 0 < effort(closing) <= .375
    assert closing["active"]["teleop"]["gripper_control"] == "velocity-torque-pause-v6"
    assert closing["active"]["teleop"]["gripper_command_nm"] == 0.
    assert closing["active"]["gripper_watchdog_ms"] == 200.
    assert max(abs(a["angle"] - b["angle"]) for a, b in zip(initial["joints"][:6], closing["joints"][:6])) < .001
    expect(page.locator("#gripper-status")).to_contain_text("J7 收拢")

    post("/api/sim-input", {"deadman": True, "triggers": {"rt": 1, "lt": 1}})
    wait_for(lambda data: effort(data) == 0)
    post("/api/sim-input", {"deadman": True, "triggers": {"rt": 0, "lt": 1}})
    opening = wait_for(lambda data: -.375 <= (effort(data) or 0) < -.02
                        and data["joints"][6]["angle"] < closing["joints"][6]["angle"] - .2)
    assert opening["active"]["gripper_enabled"] is True

    post("/api/sim-input", {"deadman": False, "triggers": {"rt": 1, "lt": 0}})
    wait_for(lambda data: effort(data) == 0)
    post("/api/sim-input", {"connected": False})
    wait_for(lambda data: effort(data) == 0 and data["active"]["teleop"]["need_rearm"])
    post("/api/sim-input", {"deadman": True, "triggers": {"rt": 1, "lt": 0}})
    wait_for(lambda data: data["gamepad"]["deadman"] and effort(data) == 0
             and data["active"]["teleop"]["need_rearm"])
    post("/api/sim-input", {})
    wait_for(lambda data: effort(data) == 0 and not data["active"]["teleop"]["need_rearm"])
    post("/api/sim-input", {"stop": True, "deadman": True, "triggers": {"rt": 1, "lt": 0}})
    wait_for(lambda data: data["active"] is None and all(j["status"] == 0 for j in data["joints"]))
    expect(opt).not_to_be_checked()
    post("/api/sim-input", {})
    assert not errors, errors
    browser.close()
    print("PASS: J7 UI opt-in, no plain jog, dual-trigger gate, RT/LT, LB, disconnect/rearm, B, no JS errors")
