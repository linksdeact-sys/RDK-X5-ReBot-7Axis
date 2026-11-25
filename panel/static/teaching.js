"use strict";
(() => {
  const REVISION="hand-teach-v5-workflow";
  let context=null,busy=false,listKey="",lastSession=null,localError="";
  const el=id=>document.getElementById(id);
  function showError() {
    const notice=context?.data?.teaching?.notice;
    el("teach-error").hidden=!(localError || notice);
    el("teach-error-text").textContent=localError || notice?.message || "";
  }
  async function action(name) {
    if (!context || busy)return;
    let confirmed=el("teach-clear").checked;
    if (name==="save_home") {
      confirmed=window.confirm(context.data.teaching?.home?.q ? "覆盖通用起点？不会修改旧示教记录或电机零位。" : "将当前七轴姿态保存为通用起点？不会启动运动。");
      if (!confirmed)return;
    } else if (name==="home") {
      confirmed=window.confirm("确认已可靠支撑启动过渡、夹爪空载、整段路径清空，手已离开运动区域？确认后七轴将低速自动回位。");
      if (!confirmed)return;
    }
    busy=true;
    try {
      const {owner,post,poll}=context;
      await post("/api/teaching",{action:name,owner,revision:REVISION,
        supported:name==="play" ? el("teach-clear").checked : el("teach-supported").checked,
        gravity_profile:el("gravity-profile").value,name:el("teach-name").value,
        id:el("teach-files").value,confirmed});
      if (["play","drag","start","home"].includes(name))el("teach-clear").checked=false;
      await poll();
    } catch(error) {localError=error.message;}
    finally {busy=false;showError();}
  }
  for (const [id,name] of Object.entries({"teach-start":"start","teach-drag":"drag","teach-record":"record","teach-finish":"finish","teach-pause":"pause","teach-play":"play","teach-home-save":"save_home","teach-home-return":"home","teach-home-record":"record_home"}))
    el(id).addEventListener("click",()=>action(name));
  el("teach-files").addEventListener("change",()=>{el("teach-clear").checked=false;});
  el("teach-error-ack").addEventListener("click",async()=>{
    if (!context)return;
    try {
      const notice=context.data?.teaching?.notice;
      await context.post("/api/teaching",{action:"ack",notice_id:notice?.id});
      localError="";await context.poll();showError();
    } catch(error) {localError=error.message;showError();}
  });
  window.renderTeaching=(data,owner,post,poll,online)=>{
    context={data,owner,post,poll};const info=data?.teaching,state=info?.state;
    const active=data?.active?.kind==="teach";
    if (lastSession && lastSession!==data?.connection?.session_id) {
      el("teach-supported").checked=false;el("teach-clear").checked=false;
    }
    lastSession=data?.connection?.session_id || lastSession;
    const ready=online && data?.connection?.ready && !data?.connection?.stopping && !data?.error;
    const current=info?.revision===REVISION;
    const idle=ready && !data?.active && data?.joints?.length===7 && data.joints.every(j=>j.online && j.status===0);
    const usable=ready && current && active && state;
    const home=info?.home;
    const homeSafe=ready && current && !busy && home?.stationary && !data?.recovery?.busy;
    el("teach-home-save").disabled=!homeSafe;
    el("teach-home-save").textContent=home?.q ? "重新标定（覆盖已有起点）" : "保存当前姿态为起点";
    el("teach-home-return").disabled=!homeSafe || !home?.can_return || (!active && !data?.gripper?.can_start);
    el("teach-home-record").hidden=!home?.q;
    el("teach-home-record").disabled=!homeSafe || !usable || !home.at_start || !el("teach-supported").checked || info.unsaved>0;
    el("teach-home-status").textContent=state?.returning_home ? "正在低速回位，可随时暂停。" :
      !home?.q ? "尚未标定：将机械臂摆到空载、路径开阔的起始姿态，静止后保存。" :
      home.at_start ? active ? "已到通用起点。确认支撑后可直接开始录制。" : "已在通用起点。点击回到起点建立保持后即可录制。" :
      !homeSafe ? "已保存起点。等待七轴静止、反馈正常且无录制或自动运动。" :
      home.return_error ? `起点已保存，无需重存。${home.return_error}。` :
      home.roll_reentry ? "起点已保存。将先让 J6 向范围内低速退回，其余轴保持；随后七轴回位。" : "已保存起点，可以低速回位。";
    el("teach-start").hidden=active;
    el("teach-start").disabled=busy || !current || !idle || data?.recovery?.busy || !el("teach-supported").checked || !data?.gripper?.ready;
    el("teach-start-reason").textContent=active ? "" : data?.recovery?.blocker || (!ready ? "先连接机械臂并等待反馈" : !el("teach-supported").checked ? "确认可靠支撑后即可进入手推" : "七轴已失能，可进入手推");
    el("teach-drag").hidden=!active;
    el("teach-drag").disabled=busy || !usable || state.recording || state.mode==="DRAG" || state.mode==="STARTUP" || !el("teach-supported").checked;
    el("teach-record").textContent=home?.q ? "自由录制（不使用通用起点）" : "开始录制";
    el("teach-record").disabled=busy || !usable || state.mode!=="DRAG" || state.hold_blend>1e-8 || state.recording || info.unsaved>0;
    el("teach-finish").disabled=busy || !usable || !state.recording;
    el("teach-pause").disabled=busy || !usable;
    el("teach-state").textContent=!active ? "未启动 · 可直接选择记录回放" : state?.returning_home ? "低速回到通用起点" : state?.launch_pending ? "准备回放 · 平滑保持中" :
      state?.joining_start ? "低速接入起点" : ({STARTUP:"补偿渐入中",DRAG:state?.recording?"七轴录制中":"可手推摆放",HOLD:"已暂停并保持",REPLAY:"七轴回放中"}[state?.mode] || "正在准备");
    const records=info?.records || [],key=JSON.stringify(records);
    if (key!==listKey) {
      const selected=el("teach-files").value;el("teach-files").replaceChildren();
      for (const r of records) {
        const option=document.createElement("option");option.value=r.id;
        option.textContent=`${r.name} · ${r.duration_s.toFixed(1)}秒 · ${r.playable?"可回放":r.complete?"待处理 · 不可回放":"草稿 · 不可回放"}`;
        el("teach-files").append(option);
      }
      if (records.some(r=>r.id===selected))el("teach-files").value=selected;
      listKey=key;
    }
    const chosen=records.find(r=>r.id===el("teach-files").value),limit=info?.align_deg ??0;
    let delta=Infinity,worst=-1,diffs=[];
    if (chosen && data?.joints?.length===7 && data.joints.every(j=>j.online)) {
      diffs=data.joints.map((j,i)=>chosen.first_q[i]*180/Math.PI-j.angle);
      delta=Math.max(...diffs.map(Math.abs));worst=diffs.findIndex(v=>Math.abs(v)===delta);
    }
    el("teach-align").textContent=!chosen ? "暂无示教记录。" : !Number.isFinite(delta) ? "等待七轴实时反馈，暂不能计算起点误差。" :
      `最大误差 J${worst+1}：${delta.toFixed(2)}°（需≤${limit}°）。当前${data.joints[worst].angle.toFixed(2)}° → 起点${(chosen.first_q[worst]*180/Math.PI).toFixed(2)}°。`;
    el("teach-axis-errors").textContent=diffs.map((v,i)=>`J${i+1} ${v>=0?"+":""}${v.toFixed(2)}°`).join(" / ");
    let reason="";
    if (!online)reason="网页连接中断，请等待恢复";
    else if (!current)reason="界面版本不一致，请刷新网页";
    else if (data?.recovery?.faults?.length || data?.recovery?.busy)reason=data.recovery.blocker;
    else if (!ready)reason=data?.error || "先连接机械臂，等待七轴反馈";
    else if (data.active && !active)reason="其他任务正在运行，请先安全停止该任务";
    else if (!chosen)reason="先选择一条记录";
    else if (!chosen.compatible)reason="记录的机构或夹爪端点配置不匹配";
    else if (!chosen.playable)reason=chosen.validation_error || "该记录尚未通过校验，不可回放";
    else if (!Number.isFinite(delta))reason="等待七轴实时反馈";
    else if (delta>limit+1e-9)reason=`J${worst+1}偏差${delta.toFixed(2)}°超过${limit}°，请先进入手推调整；调整量见上方`;
    else if (!data.joints.every(j=>Number.isFinite(j.velocity) && Math.abs(j.velocity)<=.05))reason="请停止手推，等待七轴静止";
    else if (active && (!state || state.mode==="STARTUP"))reason="正在启动，请等待补偿渐入完成";
    else if (state?.recording)reason="请先结束并保存当前录制";
    else if (active && (state.mode==="REPLAY" || state.launch_pending))reason="正在执行回放准备或回放；可点击暂停";
    else if (!active && (!idle || !data.gripper?.can_start))reason="启动前需七轴失能且夹爪位于安全范围内";
    else if (busy)reason="指令处理中";
    else if (!el("teach-clear").checked)reason="检查下方安全说明后，勾选本次回放确认";
    el("teach-play").disabled=!!reason;
    el("teach-play-reason").textContent=reason ? "暂不能回放："+reason : "条件通过。点击后将自动准备保持、低速接入起点，再回放；无需先启动手推。";
    el("teach-play").title=reason;
    el("teach-note").textContent=!active ? `已有${records.length}条记录；可直接在下方选择并回放，不需要重新录制。` :
      `${state?.recording?`录制 ${state.recorded_s.toFixed(1)}秒 / ${state.frames}帧`:state?.reason || "手推参数 Kp=2 / Kd=1"}。暂停不等于失能，物体需承接。`;
    showError();
  };
})();
