"""Recovery uses fake hardware only, no enable/MIT/zero/Flash operations."""
import time
import unittest
from unittest.mock import patch
from panel.server import Panel
from panel.test_usb_reconnect import FakeHardware


class RecoveryHardware(FakeHardware):
    def __init__(self):
        super().__init__();self.cleared=[];self.keep_alarm=False;self.poll_hook=None
        for s in self.states:s.status_code=0
        self.states[6].status_code=13
    def disable(self,i):
        self.disabled.append(i)
        if self.states[i].status_code in (0,1):self.states[i].status_code=0
    def clear_communication_error(self,i):
        self.cleared.append(i)
        if not self.keep_alarm:self.states[i].status_code=0
    def poll(self,indices):
        if self.poll_hook:self.poll_hook()
        return super().poll(indices)


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.p=Panel(simulate=True,standby=True)
        self.p.connection_state='READY';self.p.connection_ready=True
        self.hw=RecoveryHardware();self.p.refresh(self.hw,range(7))
    def tearDown(self):self.p.gamepad.close()
    def test_fresh_angle_and_driver_alarm_are_reported_separately(self):
        s=self.p.snapshot();self.assertTrue(s['recovery']['fresh'])
        self.assertTrue(s['joints'][6]['online']);self.assertEqual(s['joints'][6]['status'],13)
        self.assertEqual(s['recovery']['targets'],[7]);self.assertTrue(s['recovery']['can_request'])
    def test_request_does_not_transmit_and_requires_support(self):
        with self.assertRaises(ValueError):self.p.request_recovery({'supported':False})
        self.p.request_recovery({'supported':True})
        self.assertEqual(self.hw.disabled,[]);self.assertEqual(self.hw.cleared,[])
        self.assertTrue(self.p.recovery_pending)
    def test_recovery_clears_only_timeout_and_verifies_zero_without_enabling(self):
        self.p.request_recovery({'supported':True});self.assertTrue(self.p.service_recovery(self.hw))
        self.assertEqual(self.hw.cleared,[6]);self.assertEqual(self.hw.enabled,[])
        self.assertTrue(self.p.recovery_snapshot()['all_disabled'])
        self.assertIsNone(self.p.active);self.assertIsNone(self.p.pending)
    def test_enabled_other_fault_offline_motion_temperature_busy_are_rejected(self):
        for change in ('enabled','overcurrent','stale','moving','hot','active','pending','stop'):
            with self.subTest(change=change):
                self.hw=RecoveryHardware();self.p.refresh(self.hw,range(7))
                self.p.active=self.p.pending=None;self.p.stop.clear()
                if change=='enabled':self.p.states[0]['status_code']=1
                if change=='overcurrent':self.p.states[0]['status_code']=10
                if change=='stale':self.p.feedback_at[0]-=1.
                if change=='moving':self.p.states[0]['vel']=.1
                if change=='hot':self.p.states[0]['t_mos']=66.
                if change=='active':self.p.active={'kind':'teach'}
                if change=='pending':self.p.pending={'kind':'teach'}
                if change=='stop':self.p.stop.set()
                with self.assertRaises(RuntimeError):self.p.request_recovery({'supported':True})
                self.assertEqual(self.hw.cleared,[])
    def test_no_new_sequence_never_clears_alarm(self):
        self.p.request_recovery({'supported':True});self.hw.cached=True
        self.p.service_recovery(self.hw)
        self.assertEqual(self.hw.cleared,[]);self.assertIn('新序号',self.p.recovery_message)
    def test_stop_or_fault_arriving_after_click_cancels_before_clear(self):
        for stop in (True,False):
            self.p.stop.clear();self.hw=RecoveryHardware();self.p.refresh(self.hw,range(7))
            self.p.request_recovery({'supported':True})
            def changed():
                if stop:self.p.stop.set()
                else:self.hw.states[0].status_code=10
            self.hw.poll_hook=changed;self.p.service_recovery(self.hw)
            self.assertEqual(self.hw.cleared,[]);self.assertFalse(self.p.recovery_running)
    def test_recovery_available_during_initial_disable_verification(self):
        self.p.connection_state='VERIFY_DISABLED';self.p.connection_ready=False
        self.p.request_recovery({'supported':True})
        self.assertTrue(self.p.verify_disabled(self.hw));self.assertEqual(self.hw.cleared,[6])
        self.assertEqual(self.hw.enabled,[])
    def test_recovery_guard_blocks_motion_submission(self):
        self.p.request_recovery({'supported':True})
        with self.assertRaisesRegex(RuntimeError,'恢复处理中'):
            self.p.submit({'joint':1,'degrees':1,'kind':'step','owner':'x','request_id':'test'})
        self.assertIsNone(self.p.pending)
    def test_alarm_not_cleared_is_not_reported_as_success(self):
        self.p.request_recovery({'supported':True});self.hw.keep_alarm=True
        self.p.service_recovery(self.hw)
        self.assertEqual(self.hw.cleared,[6]);self.assertIn('恢复未完成',self.p.recovery_message)
        self.assertFalse(self.p.recovery_snapshot()['all_disabled'])
        self.assertEqual(self.hw.enabled,[])

    def test_recovery_invalidates_a_launch_that_is_still_scanning_files(self):
        from panel.teaching import REVISION
        self.p.teaching_limits=lambda optional=False:[(-2.,2.)]*6+[(1.,4.)]
        def recover_during_scan():
            self.p.request_recovery({'supported':True})
            # Simulate the explicit recovery finishing during the file scan.
            self.p.recovery_pending=False
            self.p.states[6]['status_code']=0
        with patch.object(self.p.teaching_library,'reload',side_effect=recover_during_scan):
            with self.assertRaisesRegex(RuntimeError,'旧启动请求已取消'):
                self.p.teaching_action({'action':'start','revision':REVISION,'owner':'qa','supported':True})
        self.assertIsNone(self.p.pending)
