"""HTTP/worker side of hand teaching. No CAN calls, no servo disk writes."""
import hashlib
import json
import math
import os
import secrets
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from .teaching import (TeachingLibrary,REVISION,ALIGN_DEG,HOME_REACHED_RAD,
                       prepare_play,prepare_home,check_start,check_home_start,in_limits)
from .feedback_state import is_fresh
from .teach_compliance import policy as compliance_policy


class TeachingPanelMixin:
    def init_teaching(self, root):
        self.teaching_root=root
        self.live_teacher=None;self.teaching_unsaved=[];self.teaching_save_error=None
        self.teaching_retry_at=0.
        self.teaching_lease_latched=False
        self.teaching_notice=None
        self.teaching_reported_error=None
        # Serializes launch preparation, NOT shared CAN state publication.
        self.teaching_launch_lock=threading.Lock()
        self.teaching_library=TeachingLibrary(root/("teaching_records.sim" if self.simulate else "teaching_records"),self.teaching_signature(),self.teaching_limits(optional=True))
        self.home_file=root/("teaching_home.sim.json" if self.simulate else "teaching_home.json")
        self.teaching_home=None
        if self.home_file.exists():
            try:
                saved=json.loads(self.home_file.read_text(encoding="utf-8"))
                if saved.get("version")!=1 or saved.get("signature")!=self.teaching_library.signature:
                    raise ValueError("通用起点配置不匹配")
                saved["q"]=in_limits(saved["q"],self.teaching_library.limits or [])
                self.teaching_home={**saved,"points":dict(self.gripper_points),"limits":self.teaching_library.limits}
            except (OSError,ValueError,KeyError,TypeError,AttributeError):
                self.log("通用起点不可用，请重新标定")

    def _home_feedback(self, *, active_ok):
        # Caller owns the panel state lock. Copy a complete snapshot.
        if (not self.connection_ready or self.error or self.stop.is_set() or self.closing.is_set()
                or self.reconnect_requested.is_set() or self.pending or self.recovery_pending or self.recovery_running):
            raise RuntimeError("请等待七轴反馈及当前操作结束")
        active=bool(self.active)
        if active:
            teacher=self.live_teacher
            state=teacher.snapshot() if teacher is not None else {}
            if (not active_ok or self.active.get("kind")!="teach" or state.get("mode") not in ("HOLD","DRAG")
                    or state.get("recording") or state.get("launch_pending")):
                raise RuntimeError("请结束录制或暂停当前运动")
        now=time.monotonic()
        if not all(is_fresh(s,t,now) for s,t in zip(self.states,self.feedback_at)):
            raise RuntimeError("等待七轴实时反馈，未执行起点操作")
        if any(s["status_code"]!=(1 if active else 0) or abs(s["vel"])>.05
               or max(s["t_mos"],s["t_rotor"])>=65 for s in self.states):
            raise RuntimeError("七轴需状态正常、温度正常且静止")
        return [s["pos"] for s in self.states],[s["vel"] for s in self.states]

    def _save_home(self, body):
        if body.get("confirmed") is not True:raise ValueError("请确认使用当前姿态保存或覆盖通用起点")
        if not self.teaching_launch_lock.acquire(blocking=False):raise RuntimeError("其他示教操作正在处理")
        try:
            with self.lock:points=dict(self.gripper_points)
            signature=self.teaching_signature();limits=self.teaching_limits()
            with self.lock:
                if points!=self.gripper_points:raise RuntimeError("夹爪端点已变化，请重试")
                q,_=self._home_feedback(active_ok=True)
                payload={"version":1,"signature":signature,"q":in_limits(q,limits)}
            tmp=None
            try:
                with tempfile.NamedTemporaryFile(dir=self.home_file.parent,delete=False,mode="w",encoding="utf-8") as f:
                    tmp=f.name;json.dump(payload,f,allow_nan=False);f.flush();os.fsync(f.fileno())
                os.replace(tmp,self.home_file);tmp=None
            finally:
                if tmp is not None:os.unlink(tmp)
            with self.lock:self.teaching_home={**payload,"points":points,"limits":limits}
            self.log("已保存七轴通用起点（未修改电机零位）")
            return {"ok":True}
        finally:self.teaching_launch_lock.release()

    def _home_plan(self, signature, limits):
        with self.lock:
            home=self.teaching_home
            if not home or home["signature"]!=signature or home["points"]!=self.gripper_points:
                raise ValueError("未标定通用起点，或配置已变化，请重新保存")
            q=home["q"][:]
        return prepare_home(q,limits)

    def teaching_limits(self, optional=False):
        grip=self.gripper_range()
        if grip is None:
            if optional:return None
            raise ValueError("先设置夹爪安全端点")
        try:
            root=ET.parse(self.teaching_root.parent/'urdf/DM/urdf/ReBot_Arm_DM.urdf').getroot()
            limits=[]
            for i in range(1,7):
                node=root.find(f"joint[@name='joint{i}']/limit")
                limits.append((float(node.attrib['lower']),float(node.attrib['upper'])))
        except (OSError,ValueError,AttributeError,KeyError,ET.ParseError) as exc:
            if optional:return None
            raise ValueError("关节范围配置不可用，未启动回放") from exc
        return limits+[grip]

    def notify_teaching(self, message):
        with self.lock:self.teaching_notice={"id":secrets.token_hex(8),"message":message}
        self.log("示教："+message)

    def teaching_signature(self):
        root=self.teaching_root.parent
        digest=hashlib.sha256()
        for rel in ("config/rebotarm_dm.yaml","urdf/DM/urdf/ReBot_Arm_DM.urdf"):
            p=root/rel
            digest.update(p.read_bytes() if p.exists() else rel.encode())
        digest.update(json.dumps(self.gripper_points,sort_keys=True).encode())
        return digest.hexdigest()

    def teaching_action(self, body):
        try:return self._teaching_action(body)
        except (ValueError,RuntimeError,OSError) as exc:
            self.notify_teaching(str(exc))
            raise

    def _teaching_action(self, body):
        action=body.get("action")
        if action=="ack":
            with self.lock:
                if self.teaching_notice and body.get("notice_id")==self.teaching_notice["id"]:
                    self.teaching_notice=None
            return {"ok":True}
        if body.get("revision")!=REVISION:raise ValueError("示教界面已更新，请刷新网页")
        if action=="save_home":return self._save_home(body)
        if action in ("play","home") and body.get("confirmed") is not True:
            raise ValueError("请确认安全支撑、手已离开、路径清空并允许本次"+("回放" if action=="play" else "回位"))
        if action=="record_home" and body.get("supported") is not True:
            raise ValueError("请确认机械臂与物体已托稳，了解录制会切入手推")
        with self.lock:idle=self.active is None
        if action=="start" or action in ("play","home") and idle:
            return self._submit_teaching(body,play=action=="play",home=action=="home")
        with self.lock:
            if not self.active or self.active.get("kind")!="teach" or self.live_teacher is None:
                raise RuntimeError("当前不是示教任务，请先停止其他任务")
            if self.stop.is_set():raise RuntimeError("停止处理中")
            teacher=self.live_teacher
        if action in ("home","record_home"):
            signature=self.teaching_signature()
            if signature!=teacher.signature:raise ValueError("机构配置已变化，请重启示教会话")
            home=self._home_plan(signature,self.teaching_limits())["_rows"][0]["q"]
            with self.lock:
                if self.live_teacher is not teacher:raise RuntimeError("示教会话已变化，请重试")
                q,v=self._home_feedback(active_ok=True)
                if action=="home":error,_=check_home_start(q,v,home,teacher.limits)
                else:error=check_start(in_limits(q,teacher.limits),v,home)
                if action=="record_home" and (error>HOME_REACHED_RAD or self.teaching_unsaved):
                    raise RuntimeError("请先回到通用起点，并确认上一段已保存")
                teacher.request(action,q=home,name=body.get("name"),confirmed=body.get("confirmed"))
                if isinstance(body.get("owner"),str) and 1<=len(body["owner"])<=80:
                    self.active["owner"]=body["owner"]
                    self.lease=time.monotonic();self.teaching_lease_latched=False
            return {"accepted":True}
        if action=="play":
            record=self.teaching_library.load(body.get("id"))
            if record.get("signature")!=self.teaching_signature():raise ValueError("机构/端点配置已变，示教需要重录")
            teacher.request("play",record=record,confirmed=body.get("confirmed"))
        else:
            if action=="record" and self.teaching_unsaved:raise RuntimeError("上段示教尚未保存，请先处理存储问题")
            teacher.request(action,name=body.get("name"))
        with self.lock:
            if isinstance(body.get("owner"),str) and 1<=len(body["owner"])<=80:
                self.active["owner"]=body["owner"]
                self.lease=time.monotonic()
                self.teaching_lease_latched=False
        return {"accepted":True}

    def _submit_teaching(self, body, *, play=False, home=False):
        if not self.teaching_launch_lock.acquire(blocking=False):
            raise RuntimeError("上一条示教启动请求仍在校验，请稍候")
        try:
            with self.lock:
                if self.active or self.pending:raise RuntimeError("其他任务正在启动或运行，请先安全停止")
                points=dict(self.gripper_points)
                generation=self.motion_request_generation
            # All filesystem I/O and trajectory scans stay OUTSIDE self.lock.
            # The idle monitor can keep publishing fresh CAN state throughout.
            signature=self.teaching_signature()
            limits=self.teaching_limits()
            self.teaching_library.signature=signature
            self.teaching_library.limits=limits
            self.teaching_library.reload()
            initial=None
            if play:
                record=self.teaching_library.load(body.get("id"))
                initial=prepare_play(record,signature,limits)
            elif home:
                initial=self._home_plan(signature,limits)
                initial["signature"]=signature
            # No I/O in this short transaction. submit() rechecks the latest
            # feedback, disabled/stationary state, stop and task ownership.
            with self.lock:
                if generation!=self.motion_request_generation:raise RuntimeError('校验期间发生停止或恢复，旧启动请求已取消')
                if points!=self.gripper_points:raise RuntimeError("校验期间夹爪端点已改变，请重新确认")
                if initial is not None:
                    if any(s is None for s in self.states):raise RuntimeError("等待七轴反馈")
                    q,v=self._home_feedback(active_ok=False) if home else ([s['pos'] for s in self.states],[s['vel'] for s in self.states])
                    if home:check_home_start(q,v,initial['_rows'][0]['q'],limits)
                    else:check_start(q,v,initial['_rows'][0]['q'])
                response=self.submit({"kind":"teach","joint":1,"degrees":1,"owner":body.get("owner"),
                    "request_id":body.get("request_id") or "teach-"+secrets.token_hex(12),
                    "gravity_profile":body.get("gravity_profile","j4_75"),"supported":body.get("confirmed") if home else body.get("supported"),
                    "teaching_revision":body.get("revision"),"teleop_mode":"continuous",
                    "gripper_enabled":True,"gripper_policy":"torque-limit"})
                if initial is not None:
                    if not self.pending or self.pending['id']!=response['id']:
                        raise RuntimeError("这条回放请求已处理，请重新确认后再启动")
                    self.pending['_teaching_play']=initial
                self.teaching_lease_latched=False
                self.teaching_reported_error=None
            return response
        finally:self.teaching_launch_lock.release()

    def check_teaching_presence(self):
        teacher=self.live_teacher
        if teacher and not self.teaching_lease_latched and time.monotonic()-self.lease>1.:
            state=teacher.snapshot()
            if state.get("mode") in ("DRAG","REPLAY","STARTUP") or state.get("launch_pending"):
                try:teacher.request("pause")
                except RuntimeError:return
                self.teaching_lease_latched=True
                self.log("示教网页心跳超过1秒，暂停保持，不自动续动")

    def flush_teaching(self):
        teacher=self.live_teacher
        if teacher is None:return
        error=teacher.snapshot().get('error')
        if error and error!=self.teaching_reported_error:self.notify_teaching(error)
        self.teaching_reported_error=error
        self.teaching_unsaved.extend(teacher.take_completed())
        if not self.teaching_unsaved:return
        if time.monotonic()<self.teaching_retry_at:return
        # Bounded memory: one session cannot start another capture while a
        # failed completed record remains unsaved. Files are never overwritten.
        try:
            while self.teaching_unsaved:
                key=self.teaching_library.save(self.teaching_unsaved[0])
                self.teaching_unsaved.pop(0)
                if key:
                    item=next((r for r in self.teaching_library.snapshot() if r['id']==key),None)
                    if item and not item['playable']:self.notify_teaching("已保留待处理记录："+item['validation_error'])
                    else:self.log("七轴示教记录已保存并通过校验 · "+key)
            self.teaching_save_error=None
        except (OSError,ValueError) as exc:
            message=str(exc)
            if message!=self.teaching_save_error:self.log("示教保存失败，草稿仍在内存 · "+message)
            self.teaching_save_error=message
            self.teaching_retry_at=time.monotonic()+2.

    def teaching_snapshot(self):
        home=self.teaching_home
        q=home["q"][:] if home and home["points"]==self.gripper_points else None
        home_info={"q":q,"error_deg":None,"worst_joint":None,"adjust_deg":None,
                   "at_start":False,"stationary":False,"can_return":False,
                   "return_error":None,"roll_reentry":False}
        try:
            measured,_=self._home_feedback(active_ok=True)
            home_info["stationary"]=True
            if q is not None:
                delta=[b-a for a,b in zip(measured,q)]
                axis=max(range(7),key=lambda i:abs(delta[i]))
                error=abs(delta[axis])
                home_info.update(error_deg=math.degrees(error),worst_joint=axis+1,
                                 adjust_deg=math.degrees(delta[axis]),at_start=error<=HOME_REACHED_RAD)
                try:
                    _,corridor=check_home_start(measured,[0.]*7,q,home["limits"])
                    home_info.update(can_return=True,roll_reentry=corridor is not None,
                                     at_start=home_info["at_start"] and corridor is None)
                except ValueError as exc:
                    home_info.update(at_start=False,return_error=str(exc))
        except RuntimeError:pass
        return {"revision":REVISION,"state":self.live_teacher.snapshot() if self.live_teacher else None,
                "home":home_info,
                "startup_revision":"teach-launch-lock-v1",
                "compliance":compliance_policy(),
                "notice":dict(self.teaching_notice) if self.teaching_notice else None,
                "records":self.teaching_library.snapshot(),"save_error":self.teaching_save_error,
                "unsaved":len(self.teaching_unsaved),"max_seconds":120,"align_deg":ALIGN_DEG,"play_speed_rad_s":.15}
