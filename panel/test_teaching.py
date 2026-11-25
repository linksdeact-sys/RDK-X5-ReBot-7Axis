import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from panel.teaching import TeachingDirector,TeachingLibrary,validate,prepare_play,prepare_home,check_home_start,HOME_MOVE_SECONDS


class TeachingTests(unittest.TestCase):
    def setUp(self):
        self.q=[0.]*6+[2.];self.seq=0;self.limits=[(-2.,2.)]*6+[(1.2,3.9)]
        self.d=TeachingDirector(self.q,self.limits,"robot")

    def tick(self,q=None,fresh=True):
        if fresh:self.seq+=1
        return self.d.step(q or self.q,[0.]*7,.02,[self.seq]*7)

    def recorded(self):
        self.d.request("record",name="七轴示教")
        for i in range(60):
            self.q=[i*.001]*6+[2.+i*.002]
            self.tick()
        self.d.request("finish");self.tick()
        return self.d.take_completed()[0]

    def test_home_reuses_bounded_motion_and_records_measured_first_sample(self):
        home=self.q[:];self.q[0]+=.18;self.q[6]+=.08
        self.d=TeachingDirector(self.q,self.limits,"robot",prepare_home(home,self.limits))
        old=self.q[:]
        target=self.tick()
        self.assertTrue(self.d.snapshot()["returning_home"])
        self.assertLessEqual(abs(target[0]-old[0]),.15*.02)
        for _ in range(250):
            target=self.tick()
            self.q[:6]=target[:6]
            self.q[6]+=max(-.002,min(.002,home[6]-self.q[6]))
            if self.d.mode=="HOLD":break
        self.assertIn("已到通用起点",self.d.reason)
        self.assertFalse(self.d.snapshot()["returning_home"])
        self.d.request("record_home",q=home,name="统一起点")
        self.tick()
        self.assertEqual(self.d.mode,"DRAG")
        self.assertEqual(self.d.rows[0]["q"],self.q)

    def test_home_stall_cannot_count_as_arrival_or_start_recording(self):
        home=self.q[:];self.q[6]+=.05
        self.d=TeachingDirector(self.q,self.limits,"robot",prepare_home(home,self.limits))
        for _ in range(int(HOME_MOVE_SECONDS/.02)+35):self.tick()
        self.assertEqual(self.d.mode,"HOLD")
        self.assertIn("未能及时到位",self.d.reason)
        self.d.request("record_home",q=home)
        self.tick()
        self.assertIn("尚未到达",self.d.error)
        self.assertIsNone(self.d.rows)

    def test_home_keeps_start_window_and_needs_distinct_frames_to_settle(self):
        home=self.q[:];self.q[0]+=.3
        self.d=TeachingDirector(self.q,self.limits,"robot",prepare_home(home,self.limits))
        self.tick()
        self.assertIn("15°",self.d.error)
        self.assertEqual(self.d.mode,"HOLD")
        self.q=home[:]
        self.d.hold(self.q)
        self.d.request("home",q=home,confirmed=True);self.tick()
        for _ in range(8):self.tick(fresh=False)
        self.assertEqual(self.d.mode,"REPLAY")
        self.tick();self.tick()
        self.assertEqual(self.d.mode,"HOLD")

    def test_short_manual_j7_speed_peak_does_not_interrupt_recording(self):
        self.d.request("record");self.tick()
        for _ in range(3):
            self.seq+=1
            self.d.step(self.q,[0.]*6+[1.546],.02,[self.seq]*7)
        self.tick()
        self.assertIsNotNone(self.d.rows)
        self.assertEqual(self.d.mode,"DRAG")

    def test_sustained_or_severe_manual_j7_speed_pauses_and_preserves_reason(self):
        for speed in (1.55,2.01):
            d=TeachingDirector(self.q,self.limits,"robot")
            d.request("record");d.step(self.q,[0.]*7,.01,[1]*7)
            for i in range(2,14):
                d.step(self.q,[0.]*6+[speed],.01,[i]*7)
                if d.mode=="HOLD":break
            self.assertEqual(d.mode,"HOLD")
            draft=d.take_completed()[0]
            self.assertFalse(draft["complete"])
            self.assertIn("J7 手推速度",draft["draft_reason"])

    def test_new_capture_tolerates_small_sampling_jitter_but_not_jump_or_sustained_excess(self):
        for speed,single,passes in ((1.774,True,True),(2.6,True,False),(1.78,False,False)):
            with self.subTest(speed=speed,single=single):
                r=copy.deepcopy(self.recorded())
                q=0.
                for i,row in enumerate(r["samples"]):
                    if i:q+=.02*(speed if not single or i==20 else .25)
                    row["q"][0]=q
                if passes:validate(r,"robot",self.limits)
                else:
                    with self.assertRaises(ValueError):validate(r,"robot",self.limits)
                self.d=TeachingDirector(self.q,self.limits,"robot")

    def test_automatic_home_still_pauses_at_original_j7_threshold(self):
        self.d=TeachingDirector(self.q,self.limits,"robot",prepare_home(self.q,self.limits))
        self.tick()
        self.d.step(self.q,[0.]*6+[1.546],.02,[self.seq+1]*7)
        self.assertEqual(self.d.mode,"HOLD")
        self.assertIn("J7 速度",self.d.reason)

    def test_home_returns_measured_roll_case_inward_with_jaw_following_only_after_reentry(self):
        limits=[(-3.14,3.14)]*6+[(1.3,3.95)]
        start=[0.]*5+[math.radians(-189.05),math.radians(202.8)]
        goal=[.1]*5+[math.radians(-174.10),math.radians(153.01)]
        _,corridor=check_home_start(start,[0.]*7,goal,limits)
        self.assertIsNotNone(corridor)
        self.q=start[:]
        self.d=TeachingDirector(self.q,limits,"robot",prepare_home(goal,limits))
        old_roll=start[5]
        reached=False
        for _ in range(800):
            target=self.tick()
            if self.d.home_roll_corridor:
                self.assertEqual(target[:5],start[:5])
                self.assertEqual(target[6],start[6])
                self.assertGreaterEqual(target[5],old_roll)
                self.assertLessEqual(target[5]-old_roll,.00301)
            else:reached=True
            old_roll=target[5]
            self.q[:6]=target[:6]
            self.q[6]+=max(-.003,min(.003,target[6]-self.q[6]))
            if self.d.mode=="HOLD":break
        self.assertTrue(reached)
        self.assertIn("已到通用起点",self.d.reason)
        self.assertLess(abs(self.q[6]-goal[6]),.018)

    def test_home_rejects_unrelated_out_of_range_poses_and_roll_further_outward(self):
        limits=[(-3.14,3.14)]*6+[(1.3,3.95)]
        q=[0.]*5+[math.radians(-189.05),2.]
        goal=[0.]*5+[math.radians(-174.10),2.]
        for i,value,axis in ((1,-3.2,2),(5,math.radians(-192.),6),(6,4.,7)):
            bad=q[:];bad[i]=value
            with self.assertRaisesRegex(ValueError,f"J{axis}"):check_home_start(bad,[0.]*7,goal,limits)
        self.d=TeachingDirector(q,limits,"robot",prepare_home(goal,limits))
        self.tick(q);q[5]-=math.radians(.7)
        self.tick(q)
        self.assertEqual(self.d.mode,"HOLD")
        self.assertIn("J6",self.d.reason)

    def test_seven_axes_record_finish_holds_without_disabling(self):
        r=self.recorded();validate(r,"robot",self.limits)
        self.assertTrue(r['complete']);self.assertGreater(len(r['samples']),50)
        self.assertGreater(r['samples'][-1]['q'][6],r['samples'][0]['q'][6])
        self.assertEqual(self.d.mode,"HOLD")
        self.assertLess(self.d.factor,1.)
        for _ in range(30):self.tick()
        self.assertAlmostEqual(self.d.factor,1.)

    def test_no_duplicate_cached_samples(self):
        self.d.request('record');self.tick()
        for _ in range(5):self.tick(fresh=False)
        self.assertEqual(len(self.d.rows),1)

    def test_interrupt_preserves_non_replayable_draft(self):
        self.d.request('record')
        for _ in range(30):self.tick()
        self.d.hold(self.q,"故障暂停")
        draft=self.d.take_completed()[0]
        self.assertFalse(draft['complete'])
        with self.assertRaises(ValueError):validate(draft,"robot",self.limits)

    def test_play_requires_confirmation_manual_alignment_and_hold(self):
        r=self.recorded()
        self.q[0]=r['samples'][0]['q'][0]+math.radians(16.)
        self.d.hold(self.q)
        for _ in range(30):self.tick()
        self.d.request('play',record=r,confirmed=True);self.tick()
        self.assertEqual(self.d.mode,'HOLD')
        self.assertIn('15',self.d.error)
        self.q=r['samples'][0]['q'];self.d.hold(self.q)
        self.d.request('play',record=r,confirmed=False);self.tick()
        self.assertEqual(self.d.mode,'HOLD')
        self.assertIn('确认',self.d.error)

    def test_start_window_inclusive_15_degrees_and_rejects_larger(self):
        r=self.recorded()
        for degrees,accepted in ((14.9,True),(15.,True),(-15.,True),(15.01,False)):
            q=r['samples'][0]['q'][:];q[0]+=math.radians(degrees)
            self.q=q;self.d.hold(q)
            for _ in range(30):self.tick()
            self.d.request('play',record=r,confirmed=True);self.tick()
            self.assertEqual(self.d.mode,'REPLAY' if accepted else 'HOLD')

    def test_offset_start_joins_slowly_without_skipping_recorded_time(self):
        r=self.recorded();self.q=r['samples'][0]['q'][:]
        self.q[0]+=math.radians(15.);self.q[6]+=math.radians(15.)
        self.d.hold(self.q)
        for _ in range(30):self.tick()
        old=self.q[:];self.d.request('play',record=r,confirmed=True)
        ref=self.tick()
        self.assertTrue(self.d.joining_start)
        self.assertEqual(self.d.cursor,0.)
        self.assertLessEqual(abs(ref[0]-old[0]),.15*.02)
        # A gripper that has not reached the first pose holds the timeline.
        for _ in range(40):
            self.q[:6]=ref[:6];ref=self.tick()
            self.assertEqual(self.d.cursor,0.)
        self.q=r['samples'][0]['q'][:]
        self.d.target=self.q[:]
        self.tick()
        self.assertFalse(self.d.joining_start)
        self.assertGreater(self.d.cursor,0.)

    def test_low_speed_playback_contains_j7_and_finishes_holding(self):
        r=self.recorded();self.q=r['samples'][0]['q'];self.d.hold(self.q)
        for _ in range(30):self.tick()
        self.d.request('play',record=r,confirmed=True)
        seen=False
        for _ in range(1000):
            self.q=self.tick();seen=seen or self.d.mode=='REPLAY'
            self.assertLessEqual(max(map(abs,self.d.velocity)),.15)
            if seen and self.d.mode=='HOLD':break
        self.assertTrue(seen);self.assertEqual(self.d.mode,'HOLD')
        self.assertGreater(self.q[6],2.1)
        self.assertEqual(self.d.velocity,[0.]*6)

    def test_signature_gap_jump_nonfinite_and_limits_rejected(self):
        r=self.recorded()
        for change in ('signature','gap','jump','nan','limit'):
            bad=copy.deepcopy(r)
            if change=='signature':bad['signature']='changed'
            if change=='gap':bad['samples'][2]['t']+=.2
            if change=='jump':bad['samples'][2]['q'][6]+=.5
            if change=='nan':bad['samples'][2]['q'][0]=math.nan
            if change=='limit':bad['samples'][2]['q'][0]=3.
            with self.assertRaises(ValueError):validate(bad,'robot',self.limits)

    def test_atomic_library_checksum_and_traversal_rejection(self):
        r=self.recorded()
        with tempfile.TemporaryDirectory() as directory:
            lib=TeachingLibrary(directory,'robot');key=lib.save(r)
            self.assertEqual(lib.load(key),r)
            self.assertEqual(lib.snapshot()[0]['frames'],len(r['samples']))
            with self.assertRaises(ValueError):lib.load('../elsewhere')
            p=Path(directory)/(key+'.json');data=json.loads(p.read_text(encoding='utf-8'))
            data['record']['samples'][0]['q'][0]+=.1
            p.write_text(json.dumps(data),encoding='utf-8')
            with self.assertRaises(ValueError):lib.load(key)

    def test_pause_never_automatically_restarts(self):
        r=self.recorded();self.q=r['samples'][0]['q'];self.d.hold(self.q)
        for _ in range(30):self.tick()
        self.d.request('play',record=r,confirmed=True);self.tick()
        self.d.request('pause');self.tick()
        for _ in range(100):self.tick()
        self.assertEqual(self.d.mode,'HOLD')

    def test_direct_launch_waits_for_startup_and_can_be_cancelled(self):
        r=self.recorded();q=r['samples'][0]['q']
        prepared=prepare_play(r,'robot',self.limits)
        for cancel in (False,True):
            d=TeachingDirector(q,self.limits,'robot',prepared)
            self.assertEqual(d.mode,'HOLD')
            for seq in range(1,30):
                d.step(q,[0.]*7,.02,[seq]*7,ready=False)
                self.assertNotEqual(d.mode,'REPLAY')
            if cancel:d.request('pause')
            d.step(q,[0.]*7,.02,[30]*7,ready=True)
            self.assertEqual(d.mode,'HOLD' if cancel else 'REPLAY')
            if cancel:
                for seq in range(31,70):d.step(q,[0.]*7,.02,[seq]*7)
                self.assertEqual(d.mode,'HOLD')

    def test_play_from_drag_automatically_blends_hold_before_replay(self):
        r=self.recorded();self.q=r['samples'][0]['q'][:]
        self.d.request('drag')
        for _ in range(30):self.tick()
        self.d.request('play',record=r,confirmed=True);first=self.tick()
        self.assertEqual(self.d.mode,'HOLD');self.assertTrue(self.d.initial_play)
        self.assertEqual(first,self.q)
        for _ in range(30):self.tick()
        self.assertEqual(self.d.mode,'REPLAY')

    def test_pending_launch_rechecks_alignment_and_is_not_resumed(self):
        r=self.recorded();q=r['samples'][0]['q'][:]
        d=TeachingDirector(q,self.limits,'robot',prepare_play(r,'robot',self.limits))
        moved=q[:];moved[2]+=.4
        d.step(moved,[0.]*7,.02,[1]*7)
        self.assertEqual(d.mode,'HOLD');self.assertIn('J3',d.error)
        d.step(q,[0.]*7,.02,[2]*7)
        self.assertEqual(d.mode,'HOLD')

    def test_invalid_save_is_retained_but_not_advertised_as_replayable(self):
        r=self.recorded();bad=copy.deepcopy(r);bad['samples'][8]['q'][4]+=.06
        with tempfile.TemporaryDirectory() as directory:
            lib=TeachingLibrary(directory,'robot',self.limits);key=lib.save(bad)
            saved=lib.load(key);item=lib.snapshot()[0]
            self.assertFalse(saved['complete']);self.assertFalse(item['playable'])
            self.assertIn('第9帧 J5',item['validation_error'])
            self.assertEqual(saved['samples'],bad['samples'])
            self.assertTrue(bad['complete']) # The caller/original is untouched.

    def test_old_invalid_complete_file_is_reported_without_rewriting(self):
        r=self.recorded()
        with tempfile.TemporaryDirectory() as directory:
            lib=TeachingLibrary(directory,'robot',self.limits);key=lib.save(r)
            p=Path(directory)/(key+'.json')
            data=json.loads(p.read_text(encoding='utf-8'));data['record']['samples'][8]['q'][4]+=.06
            import hashlib
            data['sha256']=hashlib.sha256(json.dumps(data['record'],sort_keys=True,separators=(',',':'),ensure_ascii=False).encode()).hexdigest()
            p.write_text(json.dumps(data),encoding='utf-8');before=p.read_bytes()
            lib.reload();item=lib.snapshot()[0]
            self.assertTrue(item['complete']);self.assertFalse(item['playable'])
            self.assertEqual(p.read_bytes(),before)
