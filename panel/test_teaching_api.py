"""Linux server/API preflight without worker creation or USB access."""
import time
import unittest
import tempfile
import threading
from pathlib import Path
from unittest.mock import patch
from panel.server import Panel
from panel.teaching import TeachingDirector,TeachingLibrary,REVISION


class TeachingApiTests(unittest.TestCase):
    def setUp(self):
        self.panel=Panel(simulate=True,standby=True)
        self.panel.connection_ready=True;self.panel.connection_state='READY'
        self.panel.gripper_points={'open':1.3,'closed':3.9}
        self.panel.states=[{'pos':0.,'vel':0.,'torq':0.,'status_code':0,'t_mos':25.,'t_rotor':25.} for _ in range(7)]
        self.panel.states[6]['pos']=2.
        self.panel.feedback_at=[time.monotonic()]*7
        self.panel.gamepad.inject({'connected':False})
        self.start={'action':'start','owner':'test','revision':REVISION,'supported':True}
        self.limits=[(-2.,2.)]*6+[(1.3,3.9)]
        self.temp=tempfile.TemporaryDirectory()
        self.panel.teaching_library=TeachingLibrary(Path(self.temp.name),'robot',self.limits)
        self.panel.home_file=Path(self.temp.name)/'home.json'
        self.signature=patch.object(self.panel,'teaching_signature',return_value='robot');self.signature.start()
        self.ranges=patch.object(self.panel,'teaching_limits',return_value=self.limits);self.ranges.start()

    def tearDown(self):
        self.panel.gamepad.close();self.signature.stop();self.ranges.stop();self.temp.cleanup()

    def test_hand_teach_no_controller_no_hardware_open_until_worker(self):
        with patch('panel.server.Hardware',side_effect=AssertionError('NO USB')):
            result=self.panel.teaching_action(self.start)
        self.assertTrue(result['accepted'])
        self.assertEqual(self.panel.pending['kind'],'teach')
        self.assertTrue(self.panel.pending['gripper_enabled'])
        self.assertIsNone(self.panel.active)
        self.assertIsNone(self.panel.worker)

    def test_support_revision_and_fresh_disabled_preflight_required(self):
        for key,value in (('supported',False),('revision','old')):
            with self.assertRaises(ValueError):self.panel.teaching_action({**self.start,key:value})
        self.panel.states[6]['status_code']=1
        with self.assertRaises(RuntimeError):self.panel.teaching_action(self.start)
        self.panel.states[6]['status_code']=0;self.panel.feedback_at[6]-=1.
        with self.assertRaises(RuntimeError):self.panel.teaching_action(self.start)

    def test_no_play_or_record_without_active_teaching(self):
        with self.assertRaises(RuntimeError):self.panel.teaching_action({**self.start,'action':'record'})
        with self.assertRaises(ValueError):self.panel.teaching_action({**self.start,'action':'play','confirmed':False})

    def saved(self,bad=False):
        q=[0.]*6+[2.]
        r={'version':1,'signature':'robot','name':'test','complete':True,'samples':[{'t':i*.02,'q':q[:]} for i in range(31)]}
        if bad:r['samples'][6]['q'][2]=.2
        return self.panel.teaching_library.save(r)

    def test_direct_play_queues_one_explicit_job_without_usb_or_drag(self):
        key=self.saved()
        with patch('panel.server.Hardware',side_effect=AssertionError('NO USB')):
            response=self.panel.teaching_action({**self.start,'action':'play','id':key,'confirmed':True})
        self.assertTrue(response['accepted'])
        self.assertTrue(self.panel.pending['_teaching_play']['confirmed'])
        self.assertIsNone(self.panel.active);self.assertIsNone(self.panel.worker)

    def test_invalid_file_rejected_before_submit_and_error_stays_until_ack(self):
        key=self.saved(bad=True)
        with self.assertRaises(ValueError):self.panel.teaching_action({**self.start,'action':'play','id':key,'confirmed':True})
        self.assertIsNone(self.panel.pending)
        notice=self.panel.teaching_snapshot()['notice']
        for _ in range(4):self.assertEqual(self.panel.teaching_snapshot()['notice'],notice)
        self.panel.teaching_action({'action':'ack','notice_id':'outdated'})
        self.assertEqual(self.panel.teaching_snapshot()['notice'],notice)
        self.panel.teaching_action({'action':'ack','notice_id':notice['id']})
        self.assertIsNone(self.panel.teaching_snapshot()['notice'])

    def test_direct_play_still_requires_alignment_fresh_disabled_and_confirmation(self):
        key=self.saved()
        body={**self.start,'action':'play','id':key,'confirmed':True}
        self.panel.states[0]['pos']=.4
        with self.assertRaises(ValueError):self.panel.teaching_action(body)
        self.panel.states[0]['pos']=0.;self.panel.feedback_at[2]-=1.
        with self.assertRaises(RuntimeError):self.panel.teaching_action(body)
        self.panel.feedback_at=[time.monotonic()]*7;self.panel.states[1]['status_code']=1
        with self.assertRaises(RuntimeError):self.panel.teaching_action(body)
        self.assertIsNone(self.panel.pending)

    def test_page_loss_requests_hold_not_disable_or_replay(self):
        teacher=TeachingDirector([0.]*6+[2.],[(-2.,2.)]*6+[(1.3,3.9)],'robot')
        teacher.step([0.]*6+[2.],[0.]*7,.02,[1]*7)
        self.panel.live_teacher=teacher;self.panel.lease=time.monotonic()-2.
        self.panel.check_teaching_presence()
        self.assertEqual(teacher.pending[0],'pause')
        self.assertFalse(self.panel.stop.is_set())
        teacher.step([0.]*6+[2.],[0.]*7,.02,[2]*7)
        self.assertEqual(teacher.mode,'HOLD')

    def test_slow_file_validation_does_not_block_live_feedback_for_start_or_play(self):
        key=self.saved()
        for action in ('start','play'):
            with self.subTest(action=action):
                done=threading.Event();published=threading.Event()
                def publish():
                    while not done.is_set():
                        with self.panel.lock:
                            self.panel.feedback_at=[time.monotonic()]*7
                            published.set()
                        done.wait(.005)
                thread=threading.Thread(target=publish);thread.start()
                reload=self.panel.teaching_library.reload
                def slow_reload():
                    published.clear()
                    # This times out against the old outer state lock.
                    self.assertTrue(published.wait(.2),'file validation held the CAN state lock')
                    time.sleep(.25)
                    reload()
                try:
                    with patch.object(self.panel.teaching_library,'reload',side_effect=slow_reload),patch('panel.server.Hardware',side_effect=AssertionError('NO USB')):
                        response=self.panel.teaching_action({**self.start,'action':action,'id':key,'confirmed':True})
                    self.assertTrue(response['accepted'])
                    self.assertIsNone(self.panel.active);self.assertIsNone(self.panel.worker)
                    self.panel.pending=None
                finally:done.set();thread.join(1.)
                self.assertFalse(thread.is_alive())

    def test_final_preflight_rejects_feedback_loss_stop_and_config_change_during_scan(self):
        for change in ('feedback','stop','points','active'):
            with self.subTest(change=change):
                self.panel.feedback_at=[time.monotonic()]*7
                self.panel.stop.clear();self.panel.active=None
                self.panel.gripper_points={'open':1.3,'closed':3.9}
                def scan():
                    if change=='feedback':self.panel.feedback_at[3]=time.monotonic()-1.
                    if change=='stop':self.panel.stop.set()
                    if change=='points':self.panel.gripper_points['open']+=.01
                    if change=='active':self.panel.active={'kind':'gamepad'}
                with patch.object(self.panel.teaching_library,'reload',side_effect=scan):
                    with self.assertRaises(RuntimeError):self.panel.teaching_action(self.start)
                self.assertIsNone(self.panel.pending)

    def test_simultaneous_launch_is_rejected_without_waiting_on_state_lock(self):
        self.panel.teaching_launch_lock.acquire()
        try:
            with self.assertRaisesRegex(RuntimeError,'仍在校验'):self.panel.teaching_action(self.start)
            self.assertIsNone(self.panel.pending)
        finally:self.panel.teaching_launch_lock.release()

    def test_home_capture_uses_feedback_not_request_and_return_requires_confirmation(self):
        body={**self.start,'action':'save_home','confirmed':True,'q':[99.]*7}
        self.panel.teaching_action(body)
        self.assertEqual(self.panel.teaching_snapshot()['home']['q'],[0.]*6+[2.])
        self.assertTrue(self.panel.home_file.is_file())
        self.assertIsNone(self.panel.pending)
        with self.assertRaises(ValueError):
            self.panel.teaching_action({**self.start,'action':'home'})
        self.panel.states[0]['pos']=.3
        with self.assertRaisesRegex(ValueError,'15°'):
            self.panel.teaching_action({**self.start,'action':'home','confirmed':True})
        self.panel.states[0]['pos']=.2
        self.panel.teaching_action({**self.start,'action':'home','confirmed':True})
        self.assertTrue(self.panel.pending['_teaching_play']['home'])

    def test_home_cannot_be_captured_from_stale_feedback_or_reused_after_endpoint_change(self):
        body={**self.start,'action':'save_home','confirmed':True}
        self.panel.feedback_at[0]-=1.
        with self.assertRaisesRegex(RuntimeError,'实时反馈'):self.panel.teaching_action(body)
        self.assertFalse(self.panel.home_file.exists())
        self.panel.feedback_at=[time.monotonic()]*7
        self.panel.teaching_action(body)
        self.panel.gripper_points['open']+=.01
        with self.assertRaisesRegex(ValueError,'配置已变化'):
            self.panel.teaching_action({**self.start,'action':'home','confirmed':True})
        self.assertIsNone(self.panel.pending)

    def test_home_preflight_accepts_j6_inward_reentry_and_independent_jaw_travel(self):
        import math
        self.limits[5]=(-3.14,3.14)
        self.panel.states[5]['pos']=math.radians(-174.10)
        self.panel.states[6]['pos']=math.radians(153.01)
        self.panel.teaching_action({**self.start,'action':'save_home','confirmed':True})
        self.panel.states[5]['pos']=math.radians(-189.05)
        self.panel.states[6]['pos']=math.radians(202.8)
        home=self.panel.teaching_snapshot()['home']
        self.assertTrue(home['can_return']);self.assertTrue(home['roll_reentry'])
        self.assertFalse(home['at_start'])
        self.panel.teaching_action({**self.start,'action':'home','confirmed':True})
        self.assertTrue(self.panel.pending['_teaching_play']['home'])
        self.assertIsNone(self.panel.worker)
