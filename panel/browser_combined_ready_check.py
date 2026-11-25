"""Read-only real-panel check. All POST requests are blocked, no motion starts."""
from playwright.sync_api import expect, sync_playwright

URL = "http://192.168.127.10:8765"

with sync_playwright() as pw:
    browser = pw.chromium.launch(channel="msedge", headless=True)
    page = browser.new_page(viewport={"width": 1440, "height": 1000})
    writes, errors = [], []

    def readonly(route):
        if route.request.method != "GET":
            writes.append(route.request.url)
            route.abort()
        else:
            route.continue_()

    page.route("**/api/**", readonly)
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(URL, wait_until="domcontentloaded")
    # Continuous 200ms polling never reaches networkidle. Wait on rendered state.
    expect(page.locator("#online-count")).to_have_text("7 / 7 在线")
    opt, start = page.locator("#gripper-enable"), page.locator("#gamepad-start")
    expect(opt).to_be_enabled()
    opt.check()  # Browser-local configuration, never click Start.
    page.locator("#input-verified").check()
    expect(start).to_contain_text("J1～J7")
    expect(start).to_be_enabled()
    state = page.request.get(URL+"/api/state").json()
    assert not state["simulate"] and not state["active"]
    assert state["gripper"]["ready"] and state["gripper"]["can_start"]
    assert all(j["online"] and j["status"] == 0 for j in state["joints"])
    assert not writes and not errors, (writes, errors)
    browser.close()
    print("PASS real UI: one J1-J7 start entry, seven online/disabled; no POST or motion")
