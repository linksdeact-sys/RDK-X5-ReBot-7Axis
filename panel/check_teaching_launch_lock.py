"""Read real-sized record files with fake feedback. NEVER create USB/worker."""
import json
import threading
import time
from pathlib import Path
from unittest.mock import patch
from panel.server import Panel,ROOT
from panel.teaching import TeachingLibrary,REVISION

panel=Panel(simulate=True,standby=True)
panel.gripper_points=json.loads((ROOT/'gripper_limits.json').read_text())
panel.teaching_library=TeachingLibrary(ROOT/'teaching_records',panel.teaching_signature(),panel.teaching_limits())
record=next(r for r in panel.teaching_library.snapshot() if r['playable'])
panel.connection_ready=True;panel.connection_state='READY'
panel.states=[dict(pos=q,vel=0.,torq=0.,status_code=0,t_mos=25.,t_rotor=25.) for q in record['first_q']]
panel.feedback_at=[time.monotonic()]*7
done=threading.Event();published=[0]
def publish():
    while not done.is_set():
        with panel.lock:
            panel.feedback_at=[time.monotonic()]*7;published[0]+=1
        done.wait(.005)
thread=threading.Thread(target=publish);thread.start()
try:
    with patch('panel.server.Hardware',side_effect=AssertionError('NO USB')):
        for action in ('start','play'):
            count=published[0];start=time.monotonic()
            response=panel.teaching_action({'action':action,'id':record['id'],'owner':'offline-lock-test',
                'revision':REVISION,'supported':True,'confirmed':True})
            ms=(time.monotonic()-start)*1000
            assert response['accepted'] and panel.pending and panel.worker is None and panel.active is None
            assert action!='play' or panel.pending.get('_teaching_play')
            print(f'PASS {action}: {ms:.1f}ms real-file scan, {published[0]-count} feedback publications; queued only, NO USB')
            with panel.lock:panel.pending=None
finally:
    done.set();thread.join(1.);panel.gamepad.close()
assert not thread.is_alive()
