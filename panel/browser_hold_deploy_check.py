"""Read-only production readiness: block every POST, never connect hardware."""
from playwright.sync_api import expect, sync_playwright

URL = "http://192.168.127.10:8765"
with sync_playwright() as pw:
    browser = pw.chromium.launch(channel="msedge", headless=True)
    page = browser.new_page(viewport={"width":1440,"height":1000})
    writes, errors = [], []
    def readonly(route):
        if route.request.method != "GET":
            writes.append(route.request.url)
            route.abort()
        else: route.continue_()
    page.route("**/api/**", readonly)
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(URL, wait_until="domcontentloaded")
    expect(page.locator("#gripper-hold-status")).to_contain_text("2.00 N·m")
    expect(page.locator("#gripper-hold-status")).to_contain_text("不做换算")
    expect(page.locator("#gripper-policy")).to_have_value("torque-limit")
    expect(page.locator("#experience-policy")).to_contain_text("100ms")
    expect(page.locator("#experience-policy")).to_contain_text("0.90rad/s")
    expect(page.locator("#experience-policy")).to_contain_text("0.80rad/s")
    page.locator("#force-cal-details summary").click()
    expect(page.locator("#force-cal-begin")).to_be_disabled()
    expect(page.locator("#force-torque")).to_have_value("0.02")
    state = page.request.get(URL+"/api/state").json()
    assert state["connection"]["state"]=="STANDBY" and not state["active"]
    assert state['recovery']['revision']=='communication-recovery-v1'
    expect(page.locator('#recover-communication')).to_be_disabled()
    assert state["gamepad_safety_revision"]=="continuous-mit-v11"
    assert state["teaching"]["revision"]=="hand-teach-v5-workflow" and state["teaching"]["state"] is None
    assert state["teaching"]["startup_revision"]=="teach-launch-lock-v1"
    assert state["teaching"]["align_deg"]==15.
    assert state["teaching"]["compliance"]["drag_soft_speed_pause"] is False
    assert state["teaching"]["compliance"]["drag_tracking_error_pause"] is False
    assert state["teaching"]["compliance"]["hard_speed_rad_s"]==1.5
    assert state["teaching"]["compliance"]["arm_drag_kp"]==2.
    assert state["teaching"]["compliance"]["arm_drag_kd"]==1.
    expect(page.locator("#teach-title")).to_have_text("七轴手推示教 · 含夹爪")
    expect(page.locator("#teach-start")).to_be_disabled()
    expect(page.locator("#teach-play")).to_be_disabled()
    expect(page.locator("#teach-note")).not_to_contain_text("用户暂停")
    expect(page.locator("#teach-play-reason")).to_contain_text("先连接机械臂")
    assert state["gripper"]["contact_motion_policy"]["settle_travel_deg"]==12.
    expect(page.locator("#gripper-hold-status")).to_contain_text("12°受限向内压缩")
    assert state["control_policy"]["experience"]["gripper_control_pause_ms"]==100.
    assert state["control_policy"]["experience"]["arm_soft_speed_rad_s"]==.8
    assert state["control_policy"]["gravity_startup_seconds"]==.5
    assert state["control_policy"]["arm_motion_event_action"]=="pause-and-hold"
    assert state["gripper"]["measured_force_n"] is None
    assert state["gripper"]["torque_limit_nm"]==2.
    assert state["gripper"]["hard_speed_rad_s"]==1.5
    assert state["gripper"]["speed_event_action"]=="pause-and-hold-arm"
    assert "light_reference_n" not in state["gripper"]
    assert not writes and not errors, (writes,errors)
    browser.close()
    print("PASS hand-teach-v5 independent playback, explicit gate reason, 15deg window, protections retained, STANDBY; no POST or motion")
