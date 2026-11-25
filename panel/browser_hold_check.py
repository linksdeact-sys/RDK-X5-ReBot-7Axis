"""E2E ONLY on 8766 simulated motors. Entered N values are test data, not real measurements."""
from pathlib import Path
from time import monotonic
from playwright.sync_api import expect, sync_playwright

URL = "http://192.168.127.10:8766"
with sync_playwright() as pw:
    browser = pw.chromium.launch(channel="msedge", headless=True)
    page = browser.new_page(viewport={"width": 1440, "height": 1050})
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(URL, wait_until="domcontentloaded")
    # The panel polls every 200ms, so use rendered readiness, not networkidle.
    expect(page.locator("#online-count")).to_have_text("7 / 7 在线")

    def state():
        data = page.request.get(URL+"/api/state").json()
        assert data["simulate"] is True
        return data

    initial = state()
    assert initial["active"] is None and not initial["gripper"]["hold_calibrated"]
    headers = {"Origin": URL, "X-Arm-Token": initial["csrf_token"]}

    def post(path, body):
        assert state()["simulate"]
        response = page.request.post(URL+path, data=body, headers=headers)
        assert response.ok, response.text()

    def pad(**body): post("/api/sim-input", body)

    def wait(predicate, seconds=7):
        end = monotonic()+seconds
        while monotonic()<end:
            value = state()
            if predicate(value): return value
            page.wait_for_timeout(30)
        raise AssertionError(str(value.get("active") or value.get("result") or value))

    def control(value): return (value.get("active") or {}).get("teleop", {})
    def phase(value): return control(value).get("gripper_hold_state")

    pad(gripper_object_rad=initial["joints"][6]["angle"]*3.141592653589793/180.+.08)
    page.locator("#force-cal-details summary").click()
    page.locator("#input-verified").check()
    page.locator("#force-supported").check()
    page.locator("#force-cal-begin").click()
    wait(lambda s: s["active"] and s["active"].get("gripper_calibration") and not control(s).get("need_rearm", True))
    origins = [j["angle"] for j in state()["joints"][:6]]
    pad(deadman=True, intent=[1.,-1.,-1.,1.,1.,1.,0.], triggers={"rt":1.,"lt":0.})
    wait(lambda s: phase(s)=="CANDIDATE")
    assert max(abs(a-j["angle"]) for a,j in zip(origins,state()["joints"][:6]))<.001
    pad()
    wait(lambda s: s["gripper"]["calibration"]["contact_ready"])
    expect(page.locator("#force-pulse")).to_be_disabled()
    pad(deadman=True)
    page.locator("#force-torque").fill("0.06")
    for index, reading in enumerate(("2.9", "3.0", "3.1"), 1):
        expect(page.locator("#force-pulse")).to_be_enabled()
        page.locator("#force-pulse").click()
        wait(lambda s: bool(s["gripper"]["calibration"]["pulse"]))
        wait(lambda s: control(s).get("gripper_command_nm",0)>.04)
        finished = wait(lambda s: not s["gripper"]["calibration"]["pulse"])
        assert finished["gripper"]["calibration"]["last_pulse"]["complete"]
        assert control(finished)["gripper_command_nm"]==0.
        page.locator("#force-reading").fill(reading)
        page.locator("#force-record").click()
        wait(lambda s: len(s["gripper"]["calibration"]["readings_n"])==index)
    pad()
    page.locator("#force-save").click()
    saved = wait(lambda s: not s["active"] and s["gripper"]["hold_calibrated"] and all(j["status"]==0 for j in s["joints"]))
    assert saved["gripper"]["points_rad"]==initial["gripper"]["points_rad"]
    assert saved["gripper"]["hold_torque_nm"]==.06 and saved["gripper"]["measured_force_n"] is None
    page.locator("#gripper-policy").select_option("calibrated")
    page.locator("#input-verified").check()
    page.locator("#gripper-enable").check()
    page.locator("#gamepad-start").click()
    wait(lambda s: s["active"] and not control(s).get("need_rearm", True))
    angle = state()["joints"][6]["angle"]
    pad(deadman=True,triggers={"rt":0.,"lt":1.})
    wait(lambda s: s["joints"][6]["angle"]<angle-6.)
    pad()
    page.wait_for_timeout(80)
    before_arm = state()["joints"][0]["angle"]
    pad(deadman=True,intent=[1.,0.,0.,0.,0.,0.,0.],triggers={"rt":1.,"lt":0.})
    wait(lambda s: phase(s)=="CANDIDATE")
    assert state()["joints"][0]["angle"]>before_arm+.1
    pad()
    held = wait(lambda s: phase(s)=="HOLD" and abs(control(s)["gripper_command_nm"]-.06)<1e-9)
    assert not held["gamepad"]["deadman"] and control(held)["gripper_measured_force_n"] is None
    held_angle = held["joints"][6]["angle"]
    pad(triggers={"rt":0.,"lt":1.})
    page.wait_for_timeout(250)
    assert phase(state())=="HOLD" and abs(state()["joints"][6]["angle"]-held_angle)<.01
    page.screenshot(path=str(Path("panel/hold-workflow-qa.png").resolve()),full_page=True)
    page.set_viewport_size({"width":390,"height":844})
    page.wait_for_timeout(100)
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
    page.screenshot(path=str(Path("panel/hold-mobile-qa.png").resolve()),full_page=True)
    pad(deadman=True,triggers={"rt":0.,"lt":1.})
    wait(lambda s: control(s).get("gripper_velocity_ref_rad_s",0)<-.02 and control(s)["gripper_command_nm"]==0.)
    pad(stop=True)
    wait(lambda s: not s["active"] and all(j["status"]==0 for j in s["joints"]))
    pad(gripper_object_rad=None)
    page.reload(wait_until="domcontentloaded")
    expect(page.locator("#online-count")).to_have_text("7 / 7 在线")
    assert state()["gripper"]["hold_calibrated"] and state()["active"] is None
    assert not errors, errors
    browser.close()
    print("PASS simulated gauge workflow, three readings, disable-before-save, unchanged endpoints, LB/RT hold, LT release, B, mobile; NOT real 3N verification")
