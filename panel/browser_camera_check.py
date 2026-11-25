"""Only the explicitly selected camera is tested; never POST motor endpoints."""
import os
from pathlib import Path
from playwright.sync_api import expect, sync_playwright

PORT = os.environ.get("REBOT_CAMERA_TEST_PORT", "8766")
BASE = f"http://192.168.127.10:{PORT}"

with sync_playwright() as playwright:
    browser = playwright.chromium.launch(channel="msedge", headless=True,
        args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
    try:
        page = browser.new_page(viewport={"width": 1440, "height": 1000})
        errors, posts, frames = [], [], []
        page.on("pageerror", lambda error: errors.append(str(error)))

        def camera_only(route):
            request = route.request
            if request.method == "POST":
                posts.append(request.url)
                if request.url not in (BASE+"/api/camera/start", BASE+"/api/camera/stop"):
                    route.abort()
                    return
            route.continue_()

        page.route("**/*", camera_only)
        page.on("response", lambda response: frames.append(response.status)
                if response.url == BASE+"/api/camera/frame" else None)
        assert page.request.get(BASE+"/api/camera/frame").status == 403
        page.goto(BASE, wait_until="domcontentloaded")
        expect(page.locator("#connection")).not_to_contain_text("正在连接")
        toggle = page.get_by_role("button", name="开启相机")
        canvas = page.locator("#camera-canvas")
        expect(canvas).to_be_hidden()
        assert posts == [] and frames == []
        toggle.click()
        expect(page.locator("#camera-status")).to_contain_text("仿真画面" if PORT == "8766" else "实时", timeout=9000)
        expect(canvas).to_be_visible()
        page.wait_for_function("""() => {
          const cv = document.querySelector('#camera-canvas');
          return cv.getContext('2d').getImageData(0,0,cv.width,cv.height).data[3] === 255;
        }""")
        # In simulation only, keep a layout screenshot with non-private pixels.
        if PORT == "8766":
            page.locator(".vision-grid").screenshot(path=str(Path(__file__).with_name("camera-layout-check.png")))
        page.set_viewport_size({"width": 390, "height": 844})
        assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
        page.get_by_role("button", name="关闭相机").click()
        expect(canvas).to_be_hidden()
        expect(page.locator("#camera-status")).to_have_text("未开启")
        assert 200 in frames and posts == [BASE+"/api/camera/start", BASE+"/api/camera/stop"]
        assert not errors, errors
        print("PASS: no auto-capture, authenticated RGB preview, mobile layout, explicit stop; camera POST only")
    finally:
        browser.close()
