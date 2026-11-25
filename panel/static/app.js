"use strict";
const $ = (id) => document.getElementById(id);
const continuous = job => ["gravity", "gamepad", "teach"].includes(job?.kind);
const GAMEPAD_REVISION = "continuous-mit-v11";
const gripperOptIn = $("gripper-enable");
gripperOptIn.checked = false;
let token = document.querySelector('meta[name="arm-token"]').content;
const owner = globalThis.crypto?.randomUUID?.() || `page-${Date.now()}-${Math.random().toString(16).slice(2)}`;
const labels = ["基座旋转", "大臂", "小臂", "腕部俯仰", "腕部滚转", "末端滚转", "夹爪"];
const directionOptions = [["俯视逆时针", "俯视顺时针"], ["抬起", "放下"], ["抬起", "放下"], ["抬头", "低头"], ["沿输出轴看顺时针", "沿输出轴看逆时针"], ["沿输出轴看顺时针", "沿输出轴看逆时针"], ["打开", "闭合"]];
const hints = ["从上往下观察", "观察大臂抬起 / 放下", "观察小臂抬起 / 放下", "观察末端抬头 / 低头", "沿电机输出轴看向手部", "沿电机输出轴看向夹爪", "观察夹爪打开 / 闭合"];
let selected = 1, step = 3, data = null, online = false, posting = false, chosenDirection = null;
let history = [], lastSuccess = 0, toastTimer;
let connectionSession = null, connectionWasReady = false;
const clock = date => date.toLocaleTimeString("zh-CN", {hour12: false, timeZone: "Asia/Tokyo"});

function toast(message) {
  $("toast").textContent = message;
  $("toast").hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { $("toast").hidden = true; }, 3500);
}

async function post(path, body) {
  const response = await fetch(path, {method: "POST", headers: {"Content-Type": "application/json", "X-Arm-Token": token}, body: JSON.stringify(body), signal: AbortSignal.timeout(3000)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || "操作失败");
  return result;
}

$("recover-communication").addEventListener("click",async()=>{
  try {
    await post('/api/recovery',{supported:$("recovery-supported").checked});
    $("recovery-supported").checked=false;
    await poll();
  } catch(error){toast(error.message);}
});

for (let id = 1; id <= 7; id++) {
  const button = document.createElement("button");
  button.className = "joint-card";
  button.dataset.joint = id;
  button.setAttribute("aria-label", `选择 J${id} ${labels[id - 1]}`);
  button.innerHTML = `<span class="joint-number">J${id}</span><span><span class="joint-name">${labels[id - 1]}</span><span class="joint-model"><i id="dot-${id}"></i>${id < 4 ? "4340P" : "4310"}</span></span><span class="joint-angle" id="joint-angle-${id}">—°</span>`;
  button.addEventListener("click", () => selectJoint(id));
  $("joint-list").append(button);
}

function selectJoint(id) {
  selected = id;
  history = [];
  chosenDirection = data?.directions?.[String(id)] || null;
  document.querySelectorAll("[data-joint]").forEach(el => {
    el.classList.toggle("selected", Number(el.dataset.joint) === id);
    el.setAttribute("aria-pressed", String(Number(el.dataset.joint) === id));
  });
  $("joint-tag").textContent = `J${id}`;
  $("joint-title").textContent = labels[id - 1];
  $("selected-note").textContent = `J${id} · ${labels[id - 1]}`;
  $("view-hint").textContent = hints[id - 1];
  $("direction-options").replaceChildren();
  for (const direction of directionOptions[id - 1]) {
    const button = document.createElement("button");
    button.textContent = direction;
    button.classList.toggle("selected", direction === chosenDirection);
    button.addEventListener("click", () => {
      chosenDirection = direction;
      [...$("direction-options").children].forEach(el => el.classList.toggle("selected", el === button));
      $("save-direction").disabled = !online;
    });
    $("direction-options").append(button);
  }
  render();
  drawChart();
}

document.querySelectorAll(".diagram-joint").forEach(el => {
  el.addEventListener("click", () => selectJoint(Number(el.dataset.joint)));
  el.addEventListener("keydown", event => { if (["Enter", " "].includes(event.key)) {event.preventDefault(); selectJoint(Number(el.dataset.joint));} });
});

document.querySelectorAll("[data-step]").forEach(button => button.addEventListener("click", () => {
  step = Number(button.dataset.step);
  document.querySelectorAll("[data-step]").forEach(el => el.classList.toggle("selected", el === button));
  $("negative-size").textContent = `−${step}°`;
  $("positive-size").textContent = `+${step}°`;
}));

async function move(sign, kind) {
  if (selected === 7 || posting || !online || data?.active) return;
  posting = true;
  render();
  try {
    await post("/api/test", {joint: selected, degrees: sign * step, kind, owner, request_id: `test-${Date.now()}-${Math.random().toString(16).slice(2)}`});
    toast(`J${selected} ${sign > 0 ? "正向" : "负向"}${kind === "roundtrip" ? "往返" : "点动"}已提交`);
    await poll();
  } catch (error) {toast(error.message);}
  finally {posting = false; render();}
}

$("positive").addEventListener("click", () => move(1, "step"));
$("negative").addEventListener("click", () => move(-1, "step"));
$("roundtrip").addEventListener("click", () => move(1, "roundtrip"));
$("gravity-test").addEventListener("click", async () => {
  if (continuous(data?.active)) { $("stop").click(); return; }
  if (posting || !online || data?.active) return;
  posting = true;
  render();
  try {
    await post("/api/test", {joint: selected, degrees: 1, kind: "gravity", owner,
      gravity_profile: $("gravity-profile").value,
      request_id: `gravity-${Date.now()}-${Math.random().toString(16).slice(2)}`});
    toast("持续重力补偿已提交，可随时点击停止");
    await poll();
  } catch (error) { toast(error.message); }
  finally { posting = false; render(); }
});
$("gravity-profile").addEventListener("change", () => { $("input-verified").checked = false; render(); });
$("input-verified").addEventListener("change", render);
$("gripper-policy").addEventListener("change", () => { $("input-verified").checked = false; render(); });
$("connect-supported").addEventListener("change", render);
$("connect-hardware").addEventListener("click", async () => {
  if (posting || !online || !$("connect-supported").checked) return;
  posting = true;
  render();
  try {
    await post("/api/connect", {supported: true});
    $("connect-supported").checked = false;
    toast("正在连接并确认七轴失能，不会自动启动运动");
    await poll();
  } catch (error) { toast(error.message); }
  finally { posting = false; render(); }
});
$("teleop-mode").addEventListener("change", () => {
  $("input-verified").checked = false;
  gripperOptIn.checked = false;
  render();
});
gripperOptIn.addEventListener("change", render);
$("gamepad-start").addEventListener("click", async () => {
  if (data?.active?.kind === "gamepad") { $("stop").click(); return; }
  if (posting || !online || data?.active || $("gamepad-start").disabled) return;
  const gripperEnabled = gripperOptIn.checked && !gripperOptIn.disabled;
  posting = true;
  render();
  try {
    await post("/api/test", {joint: selected, degrees: 1, kind: "gamepad", owner,
      gravity_profile: $("gravity-profile").value, input_verified: $("input-verified").checked,
      teleop_mode: $("teleop-mode").value, safety_revision: GAMEPAD_REVISION,
      gripper_enabled: gripperEnabled,
      gripper_policy: $("gripper-policy").value,
      request_id: `gamepad-${Date.now()}-${Math.random().toString(16).slice(2)}`});
    toast($("teleop-mode").value === "continuous" ? "连续遥操已提交，先松开LB并回中，再按住LB拨杆" : `J${selected} 小范围诊断已提交，目标范围 ±3°`);
    await poll();
  } catch (error) { toast(error.message); }
  finally { posting = false; render(); }
});
for (const end of ["open", "closed"]) $("gripper-" + end).addEventListener("click", async () => {
  if (posting || $("gripper-" + end).disabled) return;
  posting = true;
  render();
  try {
    await post("/api/gripper/capture", {end});
    gripperOptIn.checked = false;
    toast("夹爪安全端点已保存");
    await poll();
  } catch (error) { toast(error.message); }
  finally { posting = false; render(); }
});
$("stop").addEventListener("click", async () => {
  try {await post("/api/stop", {}); toast("停止请求已发送，正在确认失能状态");}
  catch {toast("停止请求发送失败，请使用现场断电手段");}
});

for (const [id, action] of [["force-cal-begin", "begin"], ["force-pulse", "pulse"],
                           ["force-record", "record"], ["force-save", "commit"], ["force-abort", "abort"]]) {
  $(id).addEventListener("click", async () => {
    if (posting || $(id).disabled) return;
    posting = true;
    render();
    const lastPulse = data?.gripper?.calibration?.last_pulse;
    try {
      await post("/api/gripper/hold-calibration", {action, owner,
        supported: $("force-supported").checked, input_verified: $("input-verified").checked,
        safety_revision: GAMEPAD_REVISION, gravity_profile: $("gravity-profile").value,
        request_id: `force-${Date.now()}-${Math.random().toString(16).slice(2)}`,
        torque_nm: Number($("force-torque").value), force_n: Number($("force-reading").value),
        pulse_id: lastPulse?.id});
      if (action === "record") $("force-reading").value = "";
      toast({begin: "测力会话已提交，手臂固定；请先建立接触", pulse: "2秒测力已提交，持续按住LB",
        record: "实测读数已记录", commit: "正在停止并核验失能后保存；请保持支撑", abort: "测力已取消，正在停止失能"}[action]);
      await poll();
    } catch (error) { toast(error.message); }
    finally { posting = false; render(); }
  });
}
for (const id of ["force-supported", "force-torque", "force-reading"]) $(id).addEventListener("input", render);
document.addEventListener("keydown", event => {
  if (event.key === "Escape") {
    event.preventDefault();
    if ($("zero-dialog").open) $("zero-dialog").close();
    $("stop").click();
  }
});
$("save-direction").addEventListener("click", async () => {
  if (!chosenDirection) return;
  try {await post("/api/direction", {joint: selected, direction: chosenDirection}); toast(`已保存 J${selected} 正方向`); await poll();}
  catch (error) {toast(error.message);}
});
$("show-zero").addEventListener("click", () => $("zero-dialog").showModal());
$("close-zero").addEventListener("click", () => $("zero-dialog").close());

function number(value, precision = 2) {return value == null ? "—" : `${value >= 0 ? "+" : ""}${value.toFixed(precision)}`;}
function metric(id, value, unit) {$(id).replaceChildren(document.createTextNode(value)); const small = document.createElement("small"); small.textContent = unit; $(id).append(small);}

function syncGravityProfiles() {
  const select = $("gravity-profile");
  const profiles = data?.gravity_profiles;
  if (!profiles) return;
  const signature = JSON.stringify(profiles);
  if (select.dataset.profiles !== signature) {
    const previous = select.value;
    const options = Object.entries(profiles).sort((a, b) => a[1].scales[3] - b[1].scales[3]);
    select.replaceChildren(...options.map(([key, profile]) => {
      const option = document.createElement("option");
      option.value = key;
      option.textContent = profile.label;
      return option;
    }));
    select.value = previous in profiles ? previous : "j4_75";
    select.dataset.profiles = signature;
  }
  if (continuous(data.active) && data.active.gravity_profile in profiles) {
    select.value = data.active.gravity_profile;
  }
}

function render() {
  const state = data?.joints?.[selected - 1];
  const job = data?.active;
  $("show-zero").disabled = Boolean(job);
  if (job && $("zero-dialog").open) $("zero-dialog").close();
  const linkReady = data?.connection?.ready === true;
  const linkLabels = {STANDBY: "硬件待连接", WAIT_USB: "等待 USB2CAN 插入", RECONNECTING: "USB 自动重连中", VERIFY_DISABLED: "确认七轴失能", CONNECTING: "USB 连接中", ERROR: "USB/CAN 连接异常"};
  const standby = data?.connection?.state === "STANDBY";
  $("hardware-connect").hidden = !standby;
  $("connect-hardware").disabled = !standby || !online || posting || !$("connect-supported").checked;
  const connected = online && linkReady && data?.joints?.some(joint => joint.online);
  $("connection").className = `connection ${connected ? "live" : online ? "" : "offline"}`;
  $("connection").innerHTML = `<i></i>${connected ? data.simulate ? "仿真连接" : "RDK 已连接" : online ? linkLabels[data?.connection?.state] || "核验 CAN 链路" : "页面重连中"}`;
  $("online-count").textContent = `${data?.joints?.filter(joint => joint.online).length || 0} / 7 在线`;
  for (const joint of data?.joints || []) {
    $("joint-angle-" + joint.id).textContent = number(joint.angle, 1) + "°";
    $("dot-" + joint.id).classList.toggle("online", online && joint.online);
  }
  metric("angle", number(state?.angle), "°");
  metric("torque", number(state?.torque), "N·m");
  metric("temperature", state?.mos != null ? `${state.mos.toFixed(0)} / ${state.rotor.toFixed(0)}` : "—", "℃");
  $("motor-state").textContent = !online || !state?.online ? "链路待确认" : state.status === 0 ? "已失能" : state.status === 1 ? "已使能" : `故障 ${state.status}`;
  $("motor-state").className = `state-chip ${state?.status === 1 ? "active" : state?.status > 1 ? "fault" : ""}`;
  const stopping = data?.connection?.stopping === true;
  const blocked = !online || !linkReady || stopping || Boolean(data?.error) || !state?.online || Boolean(job) || posting || data?.recovery?.busy || data?.joints?.some(joint => joint.status != null && joint.status !== 0);
  ["positive", "negative", "roundtrip"].forEach(id => { $(id).disabled = selected === 7 || Boolean(blocked); });
  const gravityActive = continuous(job);
  $("gravity-test").disabled = gravityActive ? !online : Boolean(blocked) || !data?.joints?.every(joint => joint.online);
  $("gravity-test").textContent = gravityActive ? "停止重力补偿" : "启动持续重力补偿";
  $("gravity-profile").disabled = !online || !linkReady || Boolean(job) || posting || Boolean(data?.joints?.some(joint => joint.status != null && joint.status !== 0));
  const selectedProfile = data?.gravity_profiles?.[$("gravity-profile").value];
  const percent = selectedProfile ? Math.round(selectedProfile.scales[3] * 100) : 75;
  $("gravity-note").textContent = `J4 ${percent}%，其余轴 100%。${percent === 100 ? "此档已观察到 J4 自行上抬。" : ""}扶稳后测试，停止并确认失能后换档。`;
  $("save-direction").disabled = !online || !chosenDirection;
  $("direction-instruction").textContent = selected === 7 ? "J7 不支持普通点动；实测角度增大时，夹爪收拢。" : "点击「正向点动」，观察实物动作后选择。";
  $("phase").textContent = job ? `${gravityActive ? "J1～J6" : `J${job.joint}`} · ${job.phase}` : stopping ? "正在停止 · 暂不可启动" : "等待操作";
  $("target").textContent = gravityActive ? `已运行 ${Math.floor(job.elapsed || 0)} 秒` : job?.target != null ? `目标 ${number(job.target)}°` : "目标 —";
  $("progress").style.width = `${100 * (job?.progress || 0)}%`;
  $("job-note").textContent = job ? "本地轨迹执行中，页面心跳保持连接。" : "点动结束后，该轴自动卸力并失能。";
  if (gravityActive) $("job-note").textContent = job.torques ? `力矩 N·m · ${job.torques.map((value, index) => `J${index + 1} ${number(value)}`).join(" / ")}。RDK 本地持续运行。` : "正在启动官方重力控制器，请继续扶稳机械臂。";
  if (job?.kind === "gamepad") {
    $("job-note").textContent = `${job.phase} · 松开 LB 后机械臂减速保持${job.gripper_enabled ? "、J7 零速阻尼" : ""}；B/Back 停止并失能。`;
    $("phase").textContent = `${job.teleop_mode === "continuous" ? `J1～J${job.gripper_enabled ? 7 : 6} 连续遥操` : `J${job.joint} 单轴诊断`} · ${job.phase}`;
    $("target").textContent = job.target != null ? `目标 ${number(job.target)}°` : "准备当前姿态";
    if (job.teleop?.boundary_axes?.length) $("job-note").textContent = `J${job.teleop.boundary_axes.join(" / J")} 接近关节范围边界，可向内离开。B/Back停止并失能。`;
    if (job.teleop?.tracking_axes?.length) $("job-note").textContent = `J${job.teleop.tracking_axes.join(" / J")} 实测跟踪不足，已停止向外累积目标；请回中检查，不要继续加力。`;
  }
  const pad = data?.gamepad;
  $("gamepad-state").textContent = pad?.connected ? `${pad.deadman ? "LB 按下" : "LB 释放"} · ${Math.round(pad.report_age_ms || 0)} ms` : "手柄未就绪";
  $("gamepad-device").textContent = pad ? `${pad.name} · ${pad.error || "输入报告正常"}` : "正在读取输入设备…";
  $("gamepad-preview").replaceChildren(...(pad?.intent || Array(7).fill(0)).map((value, i) => {
    const span = document.createElement("span"); span.textContent = `J${i+1} ${number(value, 2)}`; return span;
  }));
  $("gamepad-raw").textContent = pad ? `轴 ${JSON.stringify(pad.raw_axes)}，按键 ${JSON.stringify(pad.buttons)}` : "—";
  if (job?.kind === "gamepad") $("teleop-mode").value = job.teleop_mode || "single-axis";
  const singleAxis = $("teleop-mode").value === "single-axis";
  const gamepadVersionCurrent = data?.gamepad_safety_revision === GAMEPAD_REVISION;
  const gates=data?.control_policy?.experience;
  $("experience-policy").textContent=gates ? `体验阈值：J7控制延迟 ${gates.gripper_control_pause_ms}ms；J7软速度 ${gates.gripper_soft_speed_rad_s.toFixed(2)}rad/s；手臂软速度 ${gates.arm_soft_speed_rad_s.toFixed(2)}rad/s，${gates.arm_speed_confirm_ms}ms新反馈确认。2 N·m、硬故障和机械行程保护不变。` : "正在读取体验阈值…";
  const allOnline = data?.joints?.every(joint => joint.online) === true;
  const gripper = data?.gripper;
  if (!online || !linkReady || !pad?.connected || singleAxis || !gamepadVersionCurrent ||
      gripper?.ready !== true || (!job && !posting && gripper?.can_start !== true)) {
    gripperOptIn.checked = false;
  }
  gripperOptIn.disabled = singleAxis || !gamepadVersionCurrent || Boolean(blocked) || !allOnline ||
    !pad?.connected || gripper?.ready !== true || gripper?.can_start !== true;
  const triggers = pad?.triggers;
  const triggersNeutral = triggers != null && [triggers.lt, triggers.rt].every(value =>
    Number.isFinite(value) && value >= 0 && value < .10);
  const triggersBlocked = (triggers != null || gripperOptIn.checked) && !triggersNeutral;
  $("teleop-mode").disabled = Boolean(job) || posting;
  $("gamepad-start").textContent = job?.kind === "gamepad" ? "停止手柄控制并失能" : singleAxis ? selected === 7 ? "请选择J1～J6诊断" : `启动 J${selected} 单轴诊断 · ±3°` : `启动 J1～J${gripperOptIn.checked ? 7 : 6} 连续遥操 · J4 ${percent}%`;
  $("gamepad-start").disabled = job?.kind === "gamepad" ? !online : (singleAxis && selected === 7) ||
    !gamepadVersionCurrent || Boolean(blocked) || !allOnline || !pad?.connected || !$("input-verified").checked ||
    pad.deadman || pad.stop || triggersBlocked || pad.intent.some(value => Math.abs(value) >= .10);
  $("input-verified").disabled = Boolean(job);
  const points = gripper?.points_rad || {};
  const endpointDegrees = radians => Number.isFinite(radians) ? `${(radians * 180 / Math.PI).toFixed(1)}°` : "—";
  const endpointSummary = `安全张开端 ${endpointDegrees(points.open)}；安全收拢端 ${endpointDegrees(points.closed)}。`;
  const envelope = gripper?.envelope_deg;
  const envelopeNote = Array.isArray(envelope) && envelope.length === 2 && envelope.every(Number.isFinite) ?
    `端点允许范围：${envelope[0].toFixed(1)}°～${envelope[1].toFixed(1)}°。` : "";
  $("gripper-envelope").textContent = envelopeNote;
  let gripperStatus = "夹爪未启用。";
  if (job?.kind === "gamepad" && job.gripper_enabled) {
    gripperStatus = job.teleop?.gripper_status || (job.teleop?.gripper_ready ? "夹爪遥操已启用。" : "夹爪正在预检。");
  } else if (!gripper?.ready) {
    gripperStatus = "请先记录两个有效的安全端点，扳机不会驱动夹爪。";
  } else if (singleAxis) {
    gripperStatus = "仅连续遥操可启用夹爪。";
  } else if (!job && !gripper.can_start) {
    gripperStatus = "请确认 J7 已失能、反馈有效，且夹爪位于已记录的安全行程内。";
  } else if (gripperOptIn.checked && !triggersNeutral) {
    gripperStatus = triggers == null ? "缺少有效 LT/RT 读数，夹爪不能启动。" : "请释放 LT 和 RT 后再启动。";
  } else if (gripperOptIn.checked) {
    gripperStatus = "已选择本次启用夹爪。";
  }
  $("gripper-status").textContent = `${gripperStatus} ${endpointSummary}`;
  const hold = job?.teleop;
  const contactPolicy = gripper?.contact_motion_policy;
  const contactNote = contactPolicy ? `保持时允许最多${contactPolicy.settle_travel_deg.toFixed(0)}°受限向内压缩，不自动加力；反向张开超过${contactPolicy.reopen_travel_deg.toFixed(0)}°需${contactPolicy.confirm_ms.toFixed(0)}ms新反馈确认。` : "";
  if (job?.kind === "gamepad") $("gripper-policy").value = job.gripper_policy || "torque-limit";
  $("gripper-policy").disabled = Boolean(job) || posting;
  $("gripper-hold-status").textContent = $("gripper-policy").value === "torque-limit" ?
    `电机力矩软件上限 ${(gripper?.torque_limit_nm || 0).toFixed(2)} N·m，不是夹指牛顿值，不做换算。LB＋RT收拢，接触后持续RT才逐步加紧，松RT或LB保持当前力矩；达到上限后RT锁止，LB＋LT先撤力并实际张开才解除。当前前馈 ${(hold?.gripper_command_nm || 0).toFixed(2)} N·m。${contactNote}J7硬超速阈值 ${(gripper?.hard_speed_rad_s || 0).toFixed(1)} rad/s；速度事件暂停遥操并保留手臂支撑，不整机失能。${hold?.gripper_paused ? `当前保护暂停：${hold.gripper_pause_reason}。释放LB、摇杆及扳机回中后恢复。` : hold?.gripper_close_locked ? "当前：收拢已锁止。" : "无需测力标定。"} 软件不保证碰撞峰值，手指远离夹爪。` : gripper?.hold_calibrated ?
    `名义约3N已单点人工标定，保持前馈${gripper.hold_torque_nm.toFixed(3)}Nm；不是当前物体实时受力。${hold?.gripper_hold_state === "HOLD" ? "当前：夹紧保持，LB＋LT释放。" : "接触提示后松RT确认保持。"}` :
    "保持力尚未标定：仍可速度控制，但松手不保持载荷。请用测力工具完成下方独立确认，不需要重测关节。";
  const cal = gripper?.calibration;
  const calJob = job?.kind === "gamepad" && job.gripper_calibration;
  const ownsCal = calJob && job.owner === owner;
  const pulse = cal?.pulse, lastPulse = cal?.last_pulse;
  const recorded = cal?.recorded_pulse_ids || [];
  const tau = Number($("force-torque").value), measured = Number($("force-reading").value);
  $("force-cal-begin").disabled = Boolean(blocked) || !allOnline || !gripper?.can_start ||
    !gamepadVersionCurrent || !$("input-verified").checked || !$("force-supported").checked ||
    !pad?.connected || pad.deadman || pad.stop || !triggersNeutral;
  $("force-pulse").disabled = posting || !ownsCal || !cal?.contact_ready || Boolean(pulse) || recorded.length >= 3 ||
    !pad?.connected || !pad.deadman || pad.stop || !triggersNeutral || !Number.isFinite(tau) || tau < .02 || tau > .20;
  $("force-record").disabled = posting || !ownsCal || Boolean(pulse) || !lastPulse?.complete ||
    recorded.includes(lastPulse.id) || !cal?.contact_ready || recorded.length >= 3 ||
    !Number.isFinite(measured) || measured < 2.5 || measured > 3.5;
  $("force-save").disabled = posting || !cal || cal.aborted || cal.committed || recorded.length !== 3 ||
    Boolean(pulse) || !$("force-supported").checked || !allOnline || Boolean(job && !ownsCal);
  $("force-abort").disabled = posting || !ownsCal;
  $("force-cal-status").textContent = cal ?
    `${cal.committed ? "已停止失能并保存。再次遥操时先张开少许，再收拢建立新接触。" : cal.aborted ? "本次已取消，原力标定未修改。" : calJob && !ownsCal ? "此标定由其他页面启动，本页可使用总停止。" : pulse ? `测力中 ${pulse.elapsed_s.toFixed(1)}/2秒；持续按住LB。` : lastPulse && !lastPulse.complete ? "上次测力中断或力矩未稳定，请重新测量。" : cal.contact_ready ? "接触已确认，可按住LB发送2秒测力。" : "先用LB＋RT收拢，接触提示后松RT。"} 实测记录 ${cal.readings_n.join(" / ") || "—"} N（${recorded.length}/3）。保存会停止并失能，需继续支撑。` :
    "同一开口、同一力矩，三次真实读数均需2.5～3.5N；保存后才启用名义3N保持。";
  ["gripper-open", "gripper-closed"].forEach(id => { $(id).disabled = Boolean(blocked) || !data?.joints?.every(joint => joint.online); });
  const saved = data?.directions?.[String(selected)];
  $("direction-status").textContent = saved ? "已记录" : "待确认";
  $("direction-note").textContent = saved ? `已记录 J${selected} 正方向为「${saved}」。` : "记录用于界面标注，可随时修改。";
  const result = data?.result;
  $("last-joint").textContent = result ? ["gamepad","teach"].includes(result.kind) ? `J1～J${result.axes?.includes(7) ? 7 : 6}` : result.kind === "gravity" ? "J1～J6" : `J${result.joint}` : "—";
  $("peak").textContent = result ? `${result.peak.toFixed(2)}°` : "—";
  $("end-delta").textContent = result?.delta != null ? `${number(result.delta)}°` : "—";
  const currentError=data?.error || job?.teleop?.fault;
  const recovery=data?.recovery;
  const rxRecovered=!job && recovery?.fresh && /CAN.*反馈|反馈.*未更新/.test(result?.error || "");
  const faultSummary=(recovery?.faults || []).map(f=>`J${f.joint} ${f.name}（状态${f.code}）`).join("；");
  const historicalError=!job && result?.error ? rxRecovered ? "上次任务因反馈超时结束；当前七轴反馈已恢复，旧任务不会自动恢复。" : `上次任务已结束：${result.error}` : "";
  const message = !online ? "页面正在重新连接 RDK。若此前已启动运动，请现场确认状态，必要时断电。" : standby ? "网页可用，尚未打开USB2CAN、未发送电机指令。先可靠支撑机械臂，再在下方确认连接。" : currentError || (faultSummary ? `${recovery.fresh?"七轴反馈已恢复。":""}当前拦截：${faultSummary}。可在下方确认支撑后恢复通信报警；其他故障不能用此按钮清除。` : !linkReady ? `${linkLabels[data?.connection?.state] || "正在建立连接"}。等待七轴新的失能反馈，不自动启动运动。` : historicalError || (data?.simulate ? "仿真模式 · 当前使用模拟关节数据" : ""));
  $("banner").hidden = !message;
  $("banner").textContent = message;
  $("banner").className=`banner${rxRecovered && !faultSummary && !currentError?" recovered":""}`;
  $("communication-recovery").hidden=standby || !(faultSummary || recovery?.busy || recovery?.message);
  $("recovery-note").textContent=recovery?.busy ? "正在核验新反馈并恢复通信报警，不会使能。" : faultSummary ? `${recovery?.fresh?"当前角度反馈正常；":"等待反馈恢复；"}${faultSummary}。${recovery?.message || ""}` : recovery?.message || "反馈正常，七轴已失能，等待手动启动。";
  $("recover-communication").disabled=!online || !recovery?.can_request || !$("recovery-supported").checked;
  if (recovery?.busy)$("recovery-supported").checked=false;
  if (window.renderTeaching) window.renderTeaching(data,owner,post,poll,online);
}

let lastEventList = "";
function renderEvents() {
  const container = $("event-list");
  const signature = JSON.stringify(data?.events || []);
  if (signature === lastEventList) return;
  lastEventList = signature;
  container.replaceChildren();
  for (const event of (data?.events || []).slice(0, 3)) {
    const row = document.createElement("div"); row.className = "event-row";
    const time = document.createElement("time"); time.textContent = clock(new Date(event.time * 1000));
    const text = document.createElement("span"); text.textContent = event.message;
    row.append(time, text); container.append(row);
  }
}

let polling = false;
async function poll() {
  if (polling) return;
  polling = true;
  try {
    const response = await fetch("/api/state", {signal: AbortSignal.timeout(1500)});
    if (!response.ok) throw new Error("状态读取失败");
    const previousJob = data?.active;
    const previousResult = data?.result?.id;
    data = await response.json(); online = true; lastSuccess = Date.now();
    if ((previousJob?.kind === "gamepad" && previousJob.id !== data.active?.id) ||
        (data.result?.kind === "gamepad" && data.result.id !== previousResult)) {
      gripperOptIn.checked = false;
    }
    if ((connectionSession && connectionSession !== data.connection?.session_id) ||
        (connectionWasReady && data.connection?.ready !== true)) {
      $("input-verified").checked = false;
      gripperOptIn.checked = false;
      history = [];
    }
    connectionSession = data.connection?.session_id || null;
    connectionWasReady = data.connection?.ready === true;
    if (data.csrf_token && data.csrf_token !== token) {
      token = data.csrf_token;
      document.querySelector('meta[name="arm-token"]').content = token;
    }
    syncGravityProfiles();
    const joint = data.joints[selected - 1];
    if (joint.online && joint.angle != null) history.push({time: Date.now(), actual: joint.angle, target: data.active?.joint === selected ? data.active.target : null});
    history = history.filter(point => point.time > Date.now() - 30000);
    $("last-update").textContent = "数据更新 " + clock(new Date());
    renderEvents();
  } catch {online = false;}
  finally {
    polling = false;
    window.dispatchEvent(new CustomEvent("rebot:feedback", {detail: {
      usable: online && data?.connection?.ready === true,
      simulate: data?.simulate,
      joints: data?.joints,
    }}));
    render();
    drawChart();
  }
}

setInterval(poll, 200);
setInterval(() => {
  if (data?.active?.owner === owner && (!continuous(data.active) || data.active.kind==="teach")) post("/api/heartbeat", {owner}).catch(() => {});
  if (Date.now() - lastSuccess > 2000) {online = false; render();}
}, 200);

function drawChart() {
  const canvas = $("chart"), rect = canvas.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
  if (!rect.width) return;
  const widthPx = Math.round(rect.width * ratio), heightPx = Math.round(rect.height * ratio);
  if (canvas.width !== widthPx || canvas.height !== heightPx) {
    canvas.width = widthPx; canvas.height = heightPx;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  const width = rect.width, height = rect.height, left = 39, right = 9, top = 9, bottom = 22;
  const values = history.flatMap(point => [point.actual, point.target]).filter(value => value != null);
  const low = Math.min(-1, ...values), high = Math.max(1, ...values), margin = Math.max(.4, (high - low) * .12);
  const min = low - margin, max = high + margin;
  const y = value => top + (max - value) / (max - min) * (height - top - bottom);
  const now = Date.now(), x = time => left + (time - now + 30000) / 30000 * (width - left - right);
  ctx.font = '9px Consolas, monospace'; ctx.lineWidth = 1;
  for (let i = 0; i <= 3; i++) {
    const value = min + (max - min) * i / 3, py = y(value);
    ctx.strokeStyle = "#edf1e9"; ctx.beginPath(); ctx.moveTo(left, py); ctx.lineTo(width - right, py); ctx.stroke();
    ctx.fillStyle = "#a2afa1"; ctx.textAlign = "right"; ctx.fillText(value.toFixed(1) + "°", left - 7, py + 3);
  }
  ctx.fillStyle = "#a2afa1"; ctx.textAlign = "left"; ctx.fillText("−30s", left, height - 4); ctx.textAlign = "right"; ctx.fillText("现在", width - right, height - 4);
  for (const [key, color, dash] of [["target", "#a9bca6", [4, 3]], ["actual", "#248868", []]]) {
    ctx.strokeStyle = color; ctx.lineWidth = 1.8; ctx.setLineDash(dash); ctx.beginPath(); let started = false;
    for (const point of history) {if (point[key] == null) {started = false; continue;} if (!started) ctx.moveTo(x(point.time), y(point[key])); else ctx.lineTo(x(point.time), y(point[key])); started = true;}
    ctx.stroke();
  }
  ctx.setLineDash([]);
}
new ResizeObserver(drawChart).observe($("chart"));
selectJoint(1);
poll();
