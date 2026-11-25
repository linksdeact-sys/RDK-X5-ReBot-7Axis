(() => {
  const toggle = document.getElementById("camera-toggle");
  const status = document.getElementById("camera-status");
  const placeholder = document.getElementById("camera-placeholder");
  const canvas = document.getElementById("camera-canvas");
  const context = canvas.getContext("2d");
  const token = document.querySelector('meta[name="arm-token"]');
  let lease, generation = 0, timer, starting = false, lastFrame = 0, simulated = false;

  function showPlaceholder(message) {
    canvas.hidden = true;
    context.clearRect(0, 0, canvas.width, canvas.height);
    placeholder.hidden = false;
    placeholder.textContent = message;
  }

  async function postCamera(action, body, keepalive = false) {
    const response = await fetch("/api/camera/" + action, {
      method: "POST", headers: {"X-Arm-Token": token.content, "Content-Type": "application/json"},
      body: JSON.stringify(body), keepalive, signal: AbortSignal.timeout(1600),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "相机请求失败");
    return result;
  }

  function stop(message) {
    ++generation;
    clearTimeout(timer);
    if (lease) postCamera("stop", {lease}, true).catch(() => {});
    lease = null;
    starting = false;
    toggle.disabled = false;
    toggle.textContent = "开启相机";
    status.textContent = message ? "预览已停止" : "未开启";
    showPlaceholder(message || "开启后显示实时画面");
  }

  async function readFrame(run) {
    if (run !== generation || !lease || document.hidden) return;
    try {
      const response = await fetch("/api/camera/frame", {
        headers: {"X-Arm-Token": token.content, "X-Camera-Lease": lease},
        signal: AbortSignal.timeout(1600),
      });
      if (run !== generation) return;
      if (response.status === 204) {
        if (performance.now() - lastFrame > 700) {
          status.textContent = "等待新画面";
          showPlaceholder("等待相机画面…");
        }
      } else if (response.ok) {
        const bitmap = await createImageBitmap(await response.blob());
        try {
          if (run !== generation) return;
          context.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
          canvas.hidden = false;
          placeholder.hidden = true;
          lastFrame = performance.now();
          status.textContent = simulated ? "仿真画面" : "实时 · 640×480";
        } finally {
          bitmap.close();
        }
      } else {
        const {error} = await response.json();
        throw new Error(error || "预览连接中断");
      }
      if (run === generation) timer = setTimeout(() => readFrame(run), 140);
    } catch (error) {
      if (run === generation) stop(error.name === "TimeoutError" ? "预览连接超时" : error.message);
    }
  }

  toggle.addEventListener("click", async () => {
    if (lease) { stop(); return; }
    if (starting || document.hidden) return;
    const run = ++generation;
    starting = true;
    toggle.disabled = true;
    status.textContent = "正在开启";
    showPlaceholder("正在连接 D435i…");
    try {
      const result = await postCamera("start", {});
      if (run !== generation) {
        postCamera("stop", {lease: result.lease}, true).catch(() => {});
        return;
      }
      lease = result.lease;
      simulated = result.simulated;
      starting = false;
      toggle.disabled = false;
      toggle.textContent = "关闭相机";
      lastFrame = performance.now();
      readFrame(run);
    } catch (error) {
      if (run === generation) stop(error.name === "TimeoutError" ? "启动相机超时" : error.message);
    }
  });

  document.addEventListener("visibilitychange", () => {
    if (document.hidden && (lease || starting)) stop();
  });
  window.addEventListener("pagehide", () => {
    if (lease || starting) stop();
  });
})();
