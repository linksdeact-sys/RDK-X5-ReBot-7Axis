"""Explicit idle-only recovery of DM LOST_COMM; never enable or send MIT."""
import time
from .feedback_state import is_fresh

STATUS_NAMES={0:'已失能',1:'已使能',8:'过压',9:'欠压',10:'过流',11:'MOS过温',12:'转子过温',13:'通信超时报警',14:'过载'}


class CommunicationRecoveryMixin:
    def init_recovery(self):
        self.recovery_pending=False;self.recovery_running=False
        self.recovery_message=None
        self.motion_request_generation=0

    def recovery_snapshot(self):
        now=time.monotonic()
        fresh=all(is_fresh(s,at,now) for s,at in zip(self.states,self.feedback_at))
        codes=[s['status_code'] if s else None for s in self.states]
        faults=[{'joint':i+1,'code':c,'name':STATUS_NAMES.get(c,'未知报警')} for i,c in enumerate(codes) if c not in (None,0,1)]
        targets=[f['joint'] for f in faults if f['code']==13]
        busy=self.recovery_pending or self.recovery_running
        disabled=all(c==0 for c in codes)
        allowed=(fresh and bool(targets) and all(c in (0,13) for c in codes)
            and not self.active and not self.pending and not self.stop.is_set()
            and not self.closing.is_set() and not self.reconnect_requested.is_set()
            and self.connection_state in ('READY','VERIFY_DISABLED')
            and all(abs(s['vel'])<=.05 and max(s['t_mos'],s['t_rotor'])<65 for s in self.states))
        blocker=None
        if busy:blocker='正在恢复通信报警，等待新反馈确认失能'
        elif faults:blocker='；'.join(f"J{f['joint']} {f['name']}（状态{f['code']}）" for f in faults)
        elif not fresh:blocker='等待七轴实时反馈恢复'
        elif not self.active and not disabled:blocker='当前仍有电机使能，请先停止并失能'
        return {'revision':'communication-recovery-v1','fresh':fresh,'faults':faults,
            'targets':targets,'busy':busy,'can_request':bool(allowed and not busy),
            'all_disabled':disabled,'blocker':blocker,'message':self.recovery_message}

    def request_recovery(self, body):
        if body.get('supported') is not True:raise ValueError('请先确认机械臂及物体已可靠支撑')
        with self.lock:
            status=self.recovery_snapshot()
            if not status['can_request']:
                raise RuntimeError(status['blocker'] or '需要七轴在线静止、无任务，且只有通信超时报警')
            self.recovery_pending=True
            self.motion_request_generation+=1
            self.recovery_message='已请求恢复通信报警；不会使能或恢复旧任务'
        self.log(self.recovery_message)
        return {'accepted':True}

    def _recovery_safe(self):
        with self.lock:
            if self.active or self.pending or self.stop.is_set() or self.closing.is_set() or self.reconnect_requested.is_set():
                raise RuntimeError('任务/停止/连接状态已变化，取消恢复')
            now=time.monotonic()
            for i,(s,at) in enumerate(zip(self.states,self.feedback_at)):
                if not is_fresh(s,at,now):raise RuntimeError(f'J{i+1}反馈不新鲜，未清报警')
                if s['status_code'] not in (0,13):raise RuntimeError(f"J{i+1}状态{s['status_code']}不允许通信恢复")
                if abs(s['vel'])>.05 or max(s['t_mos'],s['t_rotor'])>=65:raise RuntimeError(f'J{i+1}尚未静止或温度异常')
            return [i for i,s in enumerate(self.states) if s['status_code']==13]

    def service_recovery(self, hardware):
        # Called exclusively by the hardware-owner worker, including while
        # verifying initial disable. HTTP threads only enqueue the request.
        with self.lock:
            if not self.recovery_pending:return False
            self.recovery_pending=False;self.recovery_running=True
        try:
            meta=self.refresh(hardware,range(7));targets=self._recovery_safe()
            # Verify a second set of actual replies before any maintenance TX.
            self.closing.wait(.04)
            newer=self.refresh(hardware,range(7));self._recovery_safe()
            if any(b['sequence']<=a['sequence'] for a,b in zip(meta,newer)):
                raise RuntimeError('没有收到七轴新序号，未清报警')
            targets=[i for i in targets if self.states[i]['status_code']==13]
            for i in targets:
                self._recovery_safe()
                hardware.disable(i)
                self._recovery_safe()
                hardware.clear_communication_error(i)
                hardware.disable(i)
            deadline=time.monotonic()+1.2;good=0;previous=newer
            while time.monotonic()<deadline:
                meta=self.refresh(hardware,range(7));self._recovery_safe()
                good=good+1 if all(s['status_code']==0 for s in self.states) and all(b['sequence']>a['sequence'] for a,b in zip(previous,meta)) else 0
                previous=meta
                if good>=2:break
                self.closing.wait(.04)
            if good<2:raise RuntimeError('驱动器仍回报报警，未解锁；请检查连接/电源')
            self.recovery_message='通信报警已解除，七轴新反馈确认失能；等待手动启动，不恢复旧任务'
        except Exception as exc:
            self.recovery_message='恢复未完成：'+str(exc)
        finally:
            with self.lock:self.recovery_running=False
        self.log(self.recovery_message)
        return True
