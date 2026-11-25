"""Sim-only browser acceptance of persistent seven-axis teaching start."""
from math import radians
from time import monotonic
from playwright.sync_api import sync_playwright, expect

URL="http://192.168.127.10:8766"
with sync_playwright() as pw:
    browser=pw.chromium.launch(channel="msedge")
    page=browser.new_page()
    errors=[];page.on("pageerror",lambda e:errors.append(str(e)))
    page.goto(URL,wait_until="domcontentloaded")
    def state():
        s=page.request.get(URL+"/api/state").json()
        assert s["simulate"] is True
        return s
    initial=state()
    headers={"Origin":URL,"X-Arm-Token":initial["csrf_token"]}
    def api(path,body):
        assert state()["simulate"]
        r=page.request.post(URL+path,data=body,headers=headers)
        assert r.ok,r.text()
    def wait(pred,seconds=12):
        deadline=monotonic()+seconds
        while monotonic()<deadline:
            s=state()
            if pred(s):return s
            page.wait_for_timeout(40)
        raise AssertionError(s["teaching"]["state"] or s["teaching"]["notice"])
    def mode(s):return (s["teaching"]["state"] or {}).get("mode")
    el=page.locator
    expect(el("#online-count")).to_have_text("7 / 7 在线")
    assert not initial["active"]
    expect(el("#teach-home-return")).to_be_disabled()
    page.once("dialog",lambda d:d.accept())
    el("#teach-home-save").click()
    saved=wait(lambda s:s["teaching"]["home"]["q"] is not None)
    origin=saved["teaching"]["home"]["q"]
    assert len(origin)==7 and abs(origin[6]-radians(saved["joints"][6]["angle"]))<1e-9
    assert not saved["active"]
    page.once("dialog",lambda d:d.dismiss());el("#teach-home-return").click()
    assert not state()["active"]
    page.once("dialog",lambda d:d.accept());el("#teach-home-return").click()
    wait(lambda s:mode(s)=="HOLD" and "已到通用起点" in s["teaching"]["state"]["reason"])
    el("#teach-supported").check()
    el("#teach-home-record").click()
    recording=wait(lambda s:(s["teaching"]["state"] or {}).get("recording"))
    assert mode(recording)=="DRAG"
    for i in range(1,26):
        q=origin[:];q[0]+=.003*i;q[6]+=.002*i
        api("/api/sim-input",{"connected":False,"teaching_q":q})
        page.wait_for_timeout(42)
    el("#teach-finish").click()
    completed=wait(lambda s:mode(s)=="HOLD" and len(s["teaching"]["records"])>len(initial["teaching"]["records"]))
    rec=completed["teaching"]["records"][0]
    assert rec["playable"] and max(abs(a-b) for a,b in zip(rec["first_q"],origin))<.018,rec
    assert not completed["teaching"]["home"]["at_start"]
    page.once("dialog",lambda d:d.accept());el("#teach-home-return").click()
    wait(lambda s:(s["teaching"]["state"] or {}).get("returning_home"))
    final=wait(lambda s:mode(s)=="HOLD" and s["teaching"]["home"]["at_start"])
    expect(el("#teach-home-status")).to_contain_text("已到通用起点")
    expect(el("#teach-home-record")).to_be_enabled()
    page.set_viewport_size({"width":390,"height":844})
    assert page.evaluate("document.documentElement.scrollWidth<=innerWidth+1")
    el("#stop").click();wait(lambda s:not s["active"] and all(j["status"]==0 for j in s["joints"]))
    far=state()
    far["teaching"]["home"].update(error_deg=20.2,worst_joint=6,at_start=False,stationary=True,
                                   can_return=False,return_error="J6距起点20.20°，需≤15°")
    page.route("**/api/state",lambda route:route.fulfill(json=far))
    expect(el("#teach-home-status")).to_contain_text("起点已保存，无需重存")
    expect(el("#teach-home-status")).to_contain_text("J6距起点20.20°")
    expect(el("#teach-home-save")).to_have_text("重新标定（覆盖已有起点）")
    expect(el("#teach-home-return")).to_be_disabled()
    assert not errors,errors
    browser.close()
    print("PASS simulation: capture/no motion, confirmation, idle and holding return, J7 arrival, direct recording, saved trajectory, mobile")
