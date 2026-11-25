"""Seven-axis hand teaching: bounded recorder, validated files, no hardware I/O.

Recording never commands a pose. Playback is a separate explicit action,
accepting a bounded start window and joining its first pose at low speed.
"""
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import tempfile
import threading
import time
from .experience_policy import ARM_IMMEDIATE_SPEED_RAD_S

REVISION = "hand-teach-v5-workflow"
MAX_SECONDS = 120.
MAX_FRAMES = 6001
MAX_GAP = .15
ALIGN_DEG = 15.0
ALIGN_RAD = math.radians(ALIGN_DEG)
ENTRY_RAD = math.radians(2.)
PLAY_SPEED = .15
HOME_REACHED_RAD = math.radians(1.)
HOME_MOVE_SECONDS = 30.
# Local inward recovery on the wrist roll only. The normal URDF range and
# all saved/recorded poses continue to use the original limit.
HOME_ROLL_REENTRY_RAD = math.radians(10.)
HOME_ROLL_NOISE_RAD = math.radians(.5)
HAND_J7_CONFIRM_S = .075
HAND_J7_IMMEDIATE_RAD_S = 2.
HAND_CAPTURE_POLICY = "hand-drag-v2"


def pose(q):
    if not isinstance(q, (list, tuple)) or len(q) != 7 or any(
            type(v) not in (float, int) or not math.isfinite(v) or abs(v)>12.5 for v in q):
        raise ValueError("示教需要七轴有限角度")
    return list(map(float,q))


def validate(record, signature, limits):
    if not isinstance(record,dict) or record.get("version") != 1 or record.get("signature") != signature:
        raise ValueError("示教文件版本或机构/端点配置不匹配")
    rows=record.get("samples")
    if not isinstance(rows,list) or not 2<=len(rows)<=MAX_FRAMES:
        raise ValueError("示教采样数量无效")
    if record.get("complete") is not True:
        raise ValueError("录制未完成，已保留草稿："+(record.get("draft_reason") or "中途结束，不可回放"))
    hand_capture=record.get("capture_policy")==HAND_CAPTURE_POLICY
    excess=[0.]*7
    previous=None
    for frame,row in enumerate(rows,1):
        if not isinstance(row,dict):raise ValueError("示教采样格式无效")
        q=pose(row.get("q"));t=row.get("t")
        if type(t) not in (int,float) or not math.isfinite(t) or not 0<=t<=MAX_SECONDS:
            raise ValueError("示教时间戳无效")
        for axis,(x,(lo,hi)) in enumerate(zip(q,limits),1):
            if not lo<=x<=hi:raise ValueError(f"第{frame}帧 J{axis} 超出当前安全范围")
        if previous:
            h=t-previous["t"]
            if not 0<h<=MAX_GAP:
                raise ValueError(f"第{frame}帧采样间隔{h*1000:.1f}ms异常，需0～150ms，禁止回放")
            speeds=[abs(x-y)/h for x,y in zip(q,previous["q"])]
            for axis,speed in enumerate(speeds):
                nominal=1.5
                # Newly captured hand trajectories have bounded tolerances
                # for sampling quantization/latency, not a new playback speed.
                # Repeated excess accumulates; a large single jump still fails.
                if hand_capture:
                    excess[axis]=max(0.,excess[axis]+(speed-nominal)*h)
                    hard=HAND_J7_IMMEDIATE_RAD_S if axis==6 else nominal
                    invalid=(speed-hard)*h>.01 or excess[axis]>(.04 if axis==6 else .012)
                else:invalid=speed>nominal
                if invalid:
                    raise ValueError(f"第{frame}帧 J{axis+1} 位置差分{speed:.3f}rad/s超出轨迹校验范围，疑似采样跳变，禁止回放（非电机实测速度）")
        elif t!=0:
            raise ValueError("示教必须从零时刻开始")
        previous=row
    if rows[-1]["t"]<.5:
        raise ValueError("示教不足0.5秒，禁止回放")
    return rows


def prepare_play(record, signature, limits):
    rows=validate(record,signature,limits)
    return {"_rows":rows,"_times":[r["t"] for r in rows],"confirmed":True}


def check_start(q, v, first):
    errors=[abs(a-b) for a,b in zip(pose(q),pose(first))]
    axis=errors.index(max(errors))
    if max(errors)>ALIGN_RAD+1e-9:
        raise ValueError(f"J{axis+1}距起点{math.degrees(errors[axis]):.2f}°，需≤{ALIGN_DEG:g}°；请先手推调整")
    if len(v)!=7 or any(not math.isfinite(x) or abs(x)>.05 for x in v):
        raise ValueError("七轴静止后才能回放")
    return max(errors)


def in_limits(q, limits, label="当前姿态"):
    q=pose(q)
    if len(limits)!=7:raise ValueError("七轴行程配置无效")
    for i,(x,(lo,hi)) in enumerate(zip(q,limits),1):
        if not lo<=x<=hi:
            raise ValueError(f"{label} J{i} 为 {math.degrees(x):.2f}°，软件范围 {math.degrees(lo):.2f}°～{math.degrees(hi):.2f}°")
    return q


def prepare_home(q, limits):
    q=in_limits(q,limits,"已保存起点")
    return {"_rows":[{"t":0.,"q":q},{"t":.5,"q":q}],"_times":[0.,.5],"confirmed":True,"home":True}


def check_home_start(q,v,goal,limits):
    """Return total error and optional J6-only, inward-only initial corridor."""
    q=pose(q);goal=in_limits(goal,limits,"已保存起点")
    valid=q[:];lo,hi=limits[5];roll=q[5];corridor=None
    if lo-HOME_ROLL_REENTRY_RAD<=roll<lo:
        valid[5]=lo
        corridor=(roll,lo,1)
    elif hi<roll<=hi+HOME_ROLL_REENTRY_RAD:
        valid[5]=hi
        corridor=(roll,hi,-1)
    in_limits(valid,limits)
    # J7 returns within its calibrated end points and has its own speed and
    # force control. Other joint motion still has the 15-degree entry window.
    check_start(q,v,[*goal[:6],q[6]])
    return max(abs(a-b) for a,b in zip(q,goal)),corridor


class TeachingLibrary:
    def __init__(self, root, signature, limits=None):
        self.root=Path(root);self.signature=signature;self.limits=limits
        self.lock=threading.Lock();self.items=[]
        self.reload()

    def reload(self):
        items=[]
        if self.root.exists():
            for p in sorted(self.root.glob("*.json"),reverse=True)[:100]:
                try:
                    r=self.load(p.stem)
                    rows=r["samples"]
                    issue=None
                    try:validate(r,self.signature,self.limits or [(-12.5,12.5)]*7)
                    except ValueError as exc:issue=r.get("validation_error") or str(exc)
                    items.append({"id":p.stem,"name":r["name"],"duration_s":rows[-1]["t"],
                                  "frames":len(rows),"complete":r.get("complete",False),
                                  "playable":issue is None,"validation_error":issue,
                                  "compatible":r.get("signature")==self.signature,
                                  "first_q":rows[0]["q"],"created_at":r.get("created_at")})
                except (ValueError,OSError,KeyError,TypeError,IndexError):continue
        with self.lock:self.items=items

    def snapshot(self):
        with self.lock:return [dict(item) for item in self.items]

    def load(self, key):
        if not isinstance(key,str) or not re.fullmatch(r"[0-9]{13}-[a-f0-9]{12}",key):
            raise ValueError("无效示教记录ID")
        p=self.root/(key+".json")
        if p.stat().st_size>2_000_000:raise ValueError("示教文件过大")
        envelope=json.loads(p.read_text(encoding="utf-8"))
        if not isinstance(envelope,dict) or not isinstance(envelope.get("record"),dict):raise ValueError("示教文件格式无效")
        r=envelope["record"]
        digest=hashlib.sha256(json.dumps(r,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()
        if envelope.get("sha256")!=digest:raise ValueError("示教文件校验失败")
        return r

    def save(self, record):
        # Called by the panel worker, NEVER by the servo thread.
        if not record.get("samples"):return None
        # Never label an invalid recording as a usable completed trajectory.
        # Preserve its samples in a draft; do not repair/rewrite recorded poses.
        if record.get("complete"):
            try:validate(record,self.signature,self.limits or [(-12.5,12.5)]*7)
            except ValueError as exc:record={**record,"complete":False,"validation_error":str(exc)}
        self.root.mkdir(parents=True,exist_ok=True)
        if len(list(self.root.glob("*.json")))>=100:raise OSError("示教记录已达100条，请先备份整理")
        key=f"{int(time.time()*1000):013d}-{secrets.token_hex(6)}"
        payload=json.dumps(record,sort_keys=True,separators=(",",":"),ensure_ascii=False)
        envelope=json.dumps({"record":record,"sha256":hashlib.sha256(payload.encode()).hexdigest()},ensure_ascii=False)
        if len(envelope.encode())>2_000_000:raise ValueError("示教记录过大")
        tmp=None
        try:
            with tempfile.NamedTemporaryFile(mode="w",encoding="utf-8",dir=self.root,delete=False) as f:
                tmp=f.name;f.write(envelope);f.flush();os.fsync(f.fileno())
            os.replace(tmp,self.root/(key+".json"));tmp=None
        finally:
            if tmp is not None:os.unlink(tmp)
        self.reload();return key


class TeachingDirector:
    """Servo-owned state with a small locked request/completion mailbox."""
    def __init__(self, q, limits, signature, initial_play=None):
        self.limits=limits;self.signature=signature;self.target=pose(q)
        self.velocity=[0.]*6;self.acceleration=[0.]*6
        self.mode="DRAG";self.reason=None;self.factor=0.;self.elapsed=0.
        self.initial_play=initial_play
        if initial_play is not None:self.mode="HOLD";self.factor=1.
        self.rows=None;self.record_name="";self.record_start=0.;self.record_last=None
        self.plan=None;self.plan_times=None;self.cursor=0.;self.rate=.25
        self.joining_start=False
        self.home_move=False;self.home_settled=0;self.home_deadline=0.
        self.home_roll_corridor=None
        self.jaw_fast_since=None;self.jaw_fast_frames=0;self.jaw_sequence=None
        self.lock=threading.Lock();self.pending=None;self.completed=[];self.error=None
        self.state_lock=threading.RLock()
        self.public={"mode":"STARTUP","revision":REVISION};self.last_sequences=None

    def request(self, action, **data):
        if action not in ("record","record_home","finish","hold","drag","play","home","pause"):
            raise ValueError("未知示教动作")
        if action=="play":
            data={**prepare_play(data["record"],self.signature,self.limits),"confirmed":data.get("confirmed")}
        if action=="home":
            data={**prepare_home(data["q"],self.limits),"confirmed":data.get("confirmed")}
        with self.lock:
            if self.pending is not None:raise RuntimeError("上一条示教指令处理中")
            self.pending=(action,data)

    def snapshot(self):
        with self.lock:return dict(self.public)

    def take_completed(self):
        with self.lock:rows,self.completed=self.completed,[]
        return rows

    def finish(self, complete, reason=None):
        with self.state_lock:
            if self.rows is not None:
                record={"version":1,"signature":self.signature,"name":self.record_name,
                        "created_at":time.time(),"complete":bool(complete),"samples":self.rows,
                        "capture_policy":HAND_CAPTURE_POLICY}
                if not complete and reason:record["draft_reason"]=reason
                with self.lock:self.completed.append(record)
                self.rows=None

    def hold(self, q, reason=None):
        self.finish(False,reason);self.mode="HOLD";self.reason=reason
        self.joining_start=False
        self.home_move=False;self.home_settled=0
        self.home_roll_corridor=None
        self.jaw_fast_since=None;self.jaw_fast_frames=0;self.jaw_sequence=None
        self.initial_play=None
        self.target=pose(q);self.velocity=[0.]*6;self.acceleration=[0.]*6

    def _speed_reason(self, v, sequence):
        for axis,value in enumerate(v[:6],1):
            if abs(value)>=ARM_IMMEDIATE_SPEED_RAD_S:return f"J{axis} 速度 {abs(value):.2f} rad/s，已暂停"
        speed=abs(v[6])
        if self.mode!="DRAG":
            self.jaw_fast_since=None;self.jaw_fast_frames=0;self.jaw_sequence=None
            return f"J7 速度 {speed:.2f} rad/s，已暂停" if speed>=ARM_IMMEDIATE_SPEED_RAD_S else None
        if speed>=HAND_J7_IMMEDIATE_RAD_S:return f"J7 手推速度 {speed:.2f} rad/s 超限，已暂停"
        if sequence!=self.jaw_sequence:
            self.jaw_sequence=sequence
            if speed<ARM_IMMEDIATE_SPEED_RAD_S:
                self.jaw_fast_since=None;self.jaw_fast_frames=0
            else:
                if self.jaw_fast_since is None:self.jaw_fast_since=self.elapsed
                self.jaw_fast_frames+=1
                if self.jaw_fast_frames>=3 and self.elapsed-self.jaw_fast_since>=HAND_J7_CONFIRM_S:
                    return f"J7 手推速度持续偏快（{speed:.2f} rad/s），已暂停"
        return None

    def step(self, q, v, dt, sequences, *, ready=True, grip_contact=False):
        with self.state_lock:return self._step(q,v,dt,sequences,ready=ready,grip_contact=grip_contact)

    def _step(self, q, v, dt, sequences, *, ready=True,grip_contact=False):
        q=pose(q);self.elapsed+=dt
        fresh=self.last_sequences is None or all(a>b for a,b in zip(sequences,self.last_sequences))
        if fresh:self.last_sequences=list(sequences)
        with self.lock:command,self.pending=self.pending,None
        # Only this one explicitly confirmed launch may cross STARTUP into
        # replay. A pause/fault/page-loss cancels it, never auto-resumes it.
        if command:self.initial_play=None
        elif ready and self.factor>=.99 and self.initial_play is not None:
            command=("play",self.initial_play);self.initial_play=None
        if command:
            action,data=command
            self.error=None
            try:
                if action in ("record","record_home"):
                    if action=="record_home":
                        if grip_contact or not ready or not fresh or self.mode not in ("HOLD","DRAG") or self.rows is not None or dt>.1:
                            raise ValueError("等待停止运动、夹爪卸力及七轴新反馈后再录制")
                        if check_start(in_limits(q,self.limits),v,in_limits(data["q"],self.limits))>HOME_REACHED_RAD:
                            raise ValueError("尚未到达通用起点，请先回到起点")
                        self.mode="DRAG";self.target=q[:];self.reason=None
                    elif not ready or self.mode!="DRAG" or self.factor>1e-8 or self.rows is not None:
                        raise ValueError("先进入七轴拖动并等待渐入完成")
                    self.rows=[];self.record_start=self.elapsed;self.record_last=None
                    self.record_name=str(data.get("name") or "手推示教")[:60]
                elif action=="finish":
                    if self.rows is None:raise ValueError("当前没有录制")
                    self.finish(True);self.hold(q)
                elif action in ("hold","pause"):self.hold(q,"用户暂停" if action=="pause" else None)
                elif action=="drag":
                    if self.rows is not None or self.mode=="REPLAY":raise ValueError("先结束当前录制或暂停自动运动")
                    self.mode="DRAG";self.reason=None;self.plan=None
                    self.joining_start=False
                elif action in ("play","home"):
                    if self.mode not in ("DRAG","HOLD") or self.rows is not None:
                        raise ValueError("请先结束当前录制或暂停当前回放")
                    rows=data["_rows"]
                    home=bool(data.get("home"))
                    corridor=None
                    if home:
                        if grip_contact:raise ValueError("夹爪正在夹持，请卸载后再回到起点")
                        start_error,corridor=check_home_start(q,v,rows[0]["q"],self.limits)
                    else:start_error=check_start(q,v,rows[0]["q"])
                    if data.get("confirmed") is not True:raise ValueError("请确认手已离开、周围清空并准备承接")
                    if self.mode=="DRAG" or self.factor<.99:
                        self.hold(q,"正在平滑进入保持，随后低速接入起点")
                        self.initial_play=data
                    else:
                        self.plan=rows;self.plan_times=data["_times"];self.cursor=0.
                        self.joining_start=start_error>ENTRY_RAD
                        self.home_move=home;self.home_roll_corridor=corridor;self.home_settled=0
                        if corridor is not None:
                            self.target=q[:]
                            self.velocity=[0.]*6;self.acceleration=[0.]*6
                        self.home_deadline=self.elapsed+HOME_MOVE_SECONDS
                        self.mode="REPLAY";self.reason="低速接入示教起点" if self.joining_start else None
            except (ValueError,KeyError) as exc:self.error=str(exc)
        speed_reason=self._speed_reason(v,sequences[6]) if ready else None
        corridor=self.home_roll_corridor
        limits=list(self.limits)
        pending_reentry=False
        if self.initial_play and self.initial_play.get("home"):
            try:
                _,pending_corridor=check_home_start(q,[0.]*7,self.initial_play["_rows"][0]["q"],self.limits)
                pending_reentry=pending_corridor is not None
            except ValueError:pass
        if corridor is not None:
            initial,boundary,direction=corridor
            lo,hi=limits[5]
            if min((q[5]-boundary)*direction,(self.target[5]-boundary)*direction)>=HOME_ROLL_NOISE_RAD:
                self.home_roll_corridor=corridor=None
            else:
                limits[5]=(initial-HOME_ROLL_NOISE_RAD,hi) if direction>0 else (lo,initial+HOME_ROLL_NOISE_RAD)
        if not ready:
            self.target=q[:]
        elif dt>.1 or speed_reason:
            self.hold(q, speed_reason or "示教控制周期延迟，已暂停")
        elif not pending_reentry and any(not lo<=x<=hi for x,(lo,hi) in zip(q,limits)):
            axis=next(i for i,(x,(lo,hi)) in enumerate(zip(q,limits),1) if not lo<=x<=hi)
            self.hold(q,f"J{axis} 超出当前软件范围，已暂停")
        elif self.home_move and (grip_contact or self.elapsed>=self.home_deadline):
            self.hold(q,"回位遇到夹爪负载或未能及时到位，已暂停")
        elif self.mode=="DRAG":
            self.target=q[:];self.velocity=[0.]*6;self.acceleration=[0.]*6
        elif self.mode=="REPLAY":
            i=max(0,min(len(self.plan)-2,bisect.bisect_right(self.plan_times,self.cursor)-1))
            a,b=self.plan[i:i+2];u=max(0.,min(1.,(self.cursor-a["t"])/(b["t"]-a["t"])))
            goal=[x+(y-x)*u for x,y in zip(a["q"],b["q"])]
            if self.joining_start:
                goal=self.plan[0]["q"][:]
                # The wider acceptance window does NOT jump the command or
                # advance the recorded timeline while approaching its start.
                if max(abs(x-y) for x,y in zip(q,goal))<=ENTRY_RAD and max(abs(x-y) for x,y in zip(self.target[:6],goal[:6]))<.01:
                    self.joining_start=False;self.reason=None
            # Rate-limited reference follows the recorded path, not wall time.
            # Slow the virtual clock whenever the reference or motor lags.
            jaw_ready=abs(q[6]-goal[6])<.04 or (grip_contact and goal[6]>=q[6])
            if not self.home_move and not self.joining_start and jaw_ready and max(abs(x-y) for x,y in zip(self.target,goal))<.025 and max(abs(x-y) for x,y in zip(q[:6],self.target[:6]))<.06:
                self.cursor=min(self.plan_times[-1],self.cursor+min(dt,.02)*self.rate)
            h=min(dt,.02)
            for axis in range(6):
                if corridor is not None and axis!=5:
                    continue
                error=goal[axis]-self.target[axis]
                wish=max(-PLAY_SPEED,min(PLAY_SPEED,error/.1))
                acc=max(-.3,min(.3,(wish-self.velocity[axis])/.1))
                self.acceleration[axis]+=max(-3*h,min(3*h,acc-self.acceleration[axis]))
                self.velocity[axis]=max(-PLAY_SPEED,min(PLAY_SPEED,self.velocity[axis]+self.acceleration[axis]*h))
                move=self.velocity[axis]*h
                if error*move>=0 and abs(move)>=abs(error):
                    self.target[axis]=goal[axis];self.velocity[axis]=self.acceleration[axis]=0.
                else:self.target[axis]+=move
                if corridor is not None:
                    self.target[axis]=min(max(self.target[axis],min(initial,goal[axis])),max(initial,goal[axis]))
                else:self.target[axis]=max(self.limits[axis][0],min(self.limits[axis][1],self.target[axis]))
            self.target[6]=q[6] if corridor is not None else goal[6]
            if self.home_move:
                settled=(all(lo<=x<=hi for x,(lo,hi) in zip(q,self.limits))
                    and max(abs(x-y) for x,y in zip(q,goal))<=HOME_REACHED_RAD
                    and max(abs(x-y) for x,y in zip(self.target[:6],goal[:6]))<.005
                    and max(map(abs,v))<=.05)
                if not settled:self.home_settled=0
                elif fresh:self.home_settled+=1
                if self.home_settled>=3:self.hold(q,"已到通用起点，可以开始录制")
            elif self.cursor>=self.plan_times[-1] and max(abs(x-y) for x,y in zip(q[:6],self.plan[-1]["q"][:6]))<.01 and (abs(q[6]-self.plan[-1]["q"][6])<.02 or grip_contact and self.plan[-1]["q"][6]>=q[6]):
                self.mode="HOLD";self.reason="回放完成，保持当前位置"
                self.velocity=[0.]*6;self.acceleration=[0.]*6
        if self.rows is not None and ready and fresh:
            t=self.elapsed-self.record_start
            if self.record_last is None or t-self.record_last>=.019:
                if self.record_last is not None and t-self.record_last>MAX_GAP:
                    self.hold(q,"录制反馈间隔过大，已保留草稿")
                elif len(self.rows)>=MAX_FRAMES or t>=MAX_SECONDS:
                    self.finish(True);self.hold(q,"已达到120秒录制上限")
                else:
                    self.rows.append({"t":0. if not self.rows else t,"q":q[:]});self.record_last=t
        wish=0. if self.mode=="DRAG" else 1.
        self.factor+=max(-dt/.5,min(dt/.5,wish-self.factor))
        with self.lock:self.public={"revision":REVISION,"mode":self.mode if ready else "STARTUP",
            "recording":self.rows is not None,"frames":len(self.rows) if self.rows is not None else 0,
            "recorded_s":0. if self.rows is None else self.elapsed-self.record_start,
            "reason":self.reason,"error":self.error,"cursor_s":self.cursor,
            "joining_start":self.joining_start,
            "returning_home":self.home_move or bool(self.initial_play and self.initial_play.get("home")),
            "launch_pending":self.initial_play is not None,
            "duration_s":self.plan_times[-1] if self.plan_times else 0.,"hold_blend":self.factor}
        return self.target[:]
