"""Central P1 regression: real bridge methods; no vehicle or ROS master."""
import json
import math
from collections import deque
from pathlib import Path
import unittest

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Gpgga
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import test_runtime_position as fixture


class GlobalAlignmentTests(unittest.TestCase):
    setUp = fixture.RuntimePositionTests.setUp
    bridge = fixture.RuntimePositionTests.bridge
    sample = fixture.RuntimePositionTests.sample

    @staticmethod
    def anchor(b, x, y):
        if b.last_local_odom is None:
            b.last_local_odom = Odometry()
        b.last_local_odom.pose.pose.position.x = x
        b.last_local_odom.pose.pose.position.y = y

    @staticmethod
    def preserving_target(b, angle, delta=(0., 0.)):
        p = b.current_anchor()
        q = b.project_alignment((b.transform_x, b.transform_y, b.transform_yaw), p)
        c, s = math.cos(angle), math.sin(angle)
        return q[0]+delta[0]-c*p[0]+s*p[1], q[1]+delta[1]-s*p[0]-c*p[1], angle

    def test_pure_yaw_preserves_vehicle_position_at_zero_50_500_metres(self):
        for distance in (0., 50., 500.):
            for sign in (-1, 1):
                with self.subTest(distance=distance, sign=sign):
                    b = self.bridge()
                    self.anchor(b, 0., distance)
                    before = b.project_alignment((b.transform_x,b.transform_y,b.transform_yaw),b.current_anchor())
                    target = self.preserving_target(b,b.transform_yaw+sign*math.radians(.7))
                    self.assertTrue(b.move_transform_toward(target,.1,.5,math.radians(8)))
                    after = b.project_alignment((b.transform_x,b.transform_y,b.transform_yaw),b.current_anchor())
                    self.assertLess(math.hypot(after[0]-before[0],after[1]-before[1]),1e-10)

    def test_joint_correction_bounds_car_position_not_transform_translation(self):
        b = self.bridge(); self.anchor(b,0,500)
        target = self.preserving_target(b,b.transform_yaw+.1,(2.,-3.))
        old = b.transform_x,b.transform_y,b.transform_yaw
        self.assertTrue(b.move_transform_toward(target,.1,.5,.2))
        self.assertAlmostEqual(b.correction_diagnostics['actual_position_step_m'],.05)
        self.assertAlmostEqual(b.transform_yaw-old[2],.02)
        self.assertGreater(b.correction_diagnostics['transform_translation_step_m'],1.)

    def test_shortest_yaw_across_pi(self):
        b = self.bridge(); self.anchor(b,50,500)
        b.transform_yaw=math.pi-.01
        target=self.preserving_target(b,-math.pi+.01)
        b.move_transform_toward(target,.1,.5,.1)
        self.assertAlmostEqual(b.normalize_angle(b.transform_yaw-(math.pi-.01)),.01)

    def test_position_only_preserves_yaw_exactly(self):
        b=self.bridge();self.anchor(b,0,500)
        old=b.transform_yaw
        b.move_transform_toward((b.transform_x+1,b.transform_y,old),.1,.1,0.)
        self.assertEqual(b.transform_yaw,old)
        self.assertAlmostEqual(b.correction_diagnostics['actual_position_step_m'],.01)

    def test_negative_zero_long_dt_and_nonfinite_are_deterministic(self):
        for dt in (-1.,0.):
            b=self.bridge();self.anchor(b,0,500)
            old=b.transform_x,b.transform_y,b.transform_yaw
            b.move_transform_toward((200.,30.,.5),dt,.5,.2)
            self.assertEqual((b.transform_x,b.transform_y,b.transform_yaw),old)
        b=self.bridge();self.anchor(b,0,500)
        b.move_transform_toward((200.,30.,.5),100.,.5,.2)
        self.assertAlmostEqual(b.correction_diagnostics['actual_position_step_m'],.25)
        for field in ('dt','position_rate','yaw_rate','target'):
            for bad in (math.nan,math.inf,-math.inf):
                b=self.bridge();self.anchor(b,0,500);old=b.transform_x,b.transform_y,b.transform_yaw
                params=dict(dt=.1,position_rate=.5,yaw_rate=.2,target=(200.,30.,.5))
                params[field]=(bad,30.,.5) if field=='target' else bad
                self.assertFalse(b.move_transform_toward(**params))
                self.assertEqual((b.transform_x,b.transform_y,b.transform_yaw),old)

    def test_transform_error_uses_same_vehicle_point(self):
        b=self.bridge();self.anchor(b,0,500)
        target=self.preserving_target(b,.4)
        error, angle=b.transform_error(target)
        self.assertAlmostEqual(error,0.,places=10)
        self.assertAlmostEqual(angle,.1)

    def test_origin_rebase_does_not_change_correction_or_window_result(self):
        results=[]
        for ox,oy in ((0.,0.),(50.,0.),(0.,500.)):
            b=self.bridge();old=(100.,20.,.3);p=(40.,520.)
            b.transform_x,b.transform_y=b.project_alignment(old,(ox,oy));b.transform_yaw=old[2]
            self.anchor(b,p[0]-ox,p[1]-oy)
            target=self.preserving_target(b,.33,(.2,-.1))
            b.recovery_candidates=deque([(100.,target),(104.,target)])
            stable=b.recovery_window_stable(4)
            b.move_transform_toward(target,.1,.5,.2)
            results.append((b.project_alignment((b.transform_x,b.transform_y,b.transform_yaw),b.current_anchor()),stable))
        for (q,stable) in results[1:]:
            self.assertTrue(stable)
            self.assertAlmostEqual(q[0],results[0][0][0],places=10)
            self.assertAlmostEqual(q[1],results[0][0][1],places=10)

    def test_stability_projects_heading_noise_to_common_anchor(self):
        b=self.bridge();self.anchor(b,0,500)
        targets=[self.preserving_target(b,.3+d) for d in (-.005,0.,.005)]
        self.assertGreater(abs(targets[-1][0]-targets[0][0]),1.)
        b.recovery_candidates=deque(zip((100.,102.,104.),targets))
        self.assertTrue(b.recovery_window_stable(4,.1))
        self.assertLess(b.recovery_diagnostics['position_spread_m'],1e-10)

    def test_stability_rejects_actual_vehicle_position_spread(self):
        b=self.bridge();self.anchor(b,0,500)
        a=self.preserving_target(b,.3);z=self.preserving_target(b,.3,(1.,0.))
        b.recovery_candidates=deque([(100.,a),(102.,z),(104.,a)])
        self.assertFalse(b.recovery_window_stable(4,.1))
        self.assertEqual(b.recovery_diagnostics['reason'],'position_spread')

    def test_local_pose_interpolates_position_and_wrapped_heading(self):
        b=self.bridge()
        b.local_pose_history=deque([(100_000_000_000,(0.,0.,math.radians(179))),
                                    (100_200_000_000,(.16,0.,math.radians(-179)))])
        pose,diagnostic=b.local_pose_at(100_100_000_000)
        self.assertAlmostEqual(pose[0],.08)
        self.assertAlmostEqual(abs(pose[2]),math.pi)
        self.assertEqual(diagnostic['fraction'],.5)

    def test_missing_interpolation_bracket_never_extrapolates(self):
        b=self.bridge();self.sample(b,100.)
        self.assertIsNone(b.local_pose_at(100_010_000_000)[0])
        self.assertIsNone(b.local_pose_at(99_990_000_000)[0])
        b.local_pose_history=deque([(99_900_000_000,(0.,0.,0.)),(100_000_000_000,(.08,0.,0.))])
        self.assertIsNone(b.local_pose_at(99_890_000_000)[0])
        self.assertIsNone(b.local_pose_at(100_010_000_000)[0])

    def test_position_pairing_compensates_vehicle_motion(self):
        b=self.bridge();self.sample(b,100.2,local_x=.16)
        b.local_pose_history=deque([(100_000_000_000,(0.,0.,0.)),(100_200_000_000,(.16,0.,0.))])
        stamp=rospy.Time.from_sec(100.1)
        for msg in (b.last_gps_odom,b.last_fix,b.last_gga):msg.header.stamp=stamp
        b.last_gps_odom.pose.pose.position.x=100.+math.cos(.3)*.08
        b.last_gps_odom.pose.pose.position.y=20.+math.sin(.3)*.08
        self.assertTrue(b.data_quality(self.now,runtime_position=True)[0])
        self.assertAlmostEqual(b.position_pair['local_pose'][0],.08)
        b.update_runtime_position(self.now)
        self.assertLess(abs(b.last_position_error),1e-8)

    def test_heading_uses_its_own_source_time_not_position_time(self):
        self.params['~heading_filter_window_sec']=0.
        b=self.bridge();self.sample(b,100.2,quality=4,covariance=.01,local_x=.16)
        b.local_pose_history=deque([(100_000_000_000,(0.,0.,.1)),(100_200_000_000,(.16,0.,.2))])
        b.last_solution.update(heading_valid=True,heading_fixed=True)
        b.last_heading_yaw=.4;b.last_heading_recv=self.now;b.last_heading_stamp=rospy.Time(100)
        b.last_solution['stamp']=100.
        b.last_solution['stamp_ns']=rospy.Time(100).to_nsec()
        self.assertTrue(b.data_quality(self.now)[0])
        self.assertAlmostEqual(b.candidate_transform(self.now)[2],.3)

    def fixed_sample(self,b,t,bias=0.,local_x=0.,heading=.3):
        self.sample(b,t,quality=4,covariance=.01,bias=bias,local_x=local_x)
        b.last_solution.update(heading_valid=True,heading_fixed=True)
        b.last_heading_yaw=heading;b.last_heading_recv=self.now;b.last_heading_stamp=self.now

    def test_fixed_recovery_at_500m_is_continuous_and_finishes(self):
        b=self.bridge();b.dead_reckoning_budget.last_xy=(500.,0.)
        before=b.project_alignment((b.transform_x,b.transform_y,b.transform_yaw),(500.,0.))
        for i in range(36):
            self.fixed_sample(b,100.+.2*i,local_x=500.,heading=.3+math.radians(.7))
            b.update_guard(self.now)
            after=b.project_alignment((b.transform_x,b.transform_y,b.transform_yaw),(500.,0.))
            self.assertLess(math.hypot(after[0]-before[0],after[1]-before[1]),1e-8)
        self.assertEqual(b.mode,b.TRACKING)
        self.assertFalse(b.alignment_conflict)
        self.assertTrue(b.localization_is_safe(self.now))

    def test_sustained_stable_fixed_conflict_latches_until_restart(self):
        b=self.bridge();old=b.transform_x,b.transform_y,b.transform_yaw
        for i in range(31):
            self.fixed_sample(b,100.+i*.2,bias=4.)
            b.update_guard(self.now)
        self.assertTrue(b.alignment_conflict)
        self.assertFalse(b.localization_is_safe(self.now))
        self.assertEqual((b.transform_x,b.transform_y,b.transform_yaw),old)
        self.fixed_sample(b,107.,bias=0.)
        b.update_guard(self.now)
        self.assertTrue(b.alignment_conflict)
        self.assertFalse(b.localization_is_safe(self.now))

    def test_lone_fixed_or_float_conflict_does_not_latch_new_stop(self):
        b=self.bridge();self.fixed_sample(b,100.,bias=4.);b.update_guard(self.now)
        for i in range(1,40):
            self.sample(b,100.+i*.2,quality=5,bias=4.)
            b.update_guard(self.now)
        self.assertFalse(b.alignment_conflict)
        self.assertTrue(b.localization_is_safe(self.now))

    def test_pair_wait_between_fixed_samples_does_not_erase_conflict(self):
        b=self.bridge()
        for i in range(31):
            self.fixed_sample(b,100.+i*.2,bias=4.)
            b.update_guard(self.now)
            if b.alignment_conflict:
                break
            self.fixed_sample(b,100.+i*.2+.05,bias=4.)
            b.last_local_odom.header.stamp=self.now+rospy.Duration(.01)
            b.update_guard(self.now)
            self.assertEqual(b.runtime_position_reason,'position_source_time_mismatch')
            self.assertIsNotNone(b.alignment_conflict_since)
        self.assertTrue(b.alignment_conflict)
        self.assertFalse(b.localization_is_safe(self.now))

    def test_rejected_data_clears_old_residual_and_marks_invalid(self):
        b=self.bridge();self.fixed_sample(b,100.,bias=.2);b.update_guard(self.now)
        self.assertTrue(b.alignment_error_valid)
        self.sample(b,101.,quality=0);b.update_guard(self.now)
        self.assertFalse(b.alignment_error_valid)
        self.assertIsNone(b.last_position_error)
        self.assertFalse(b.source_pairing['valid'])

    def test_invalid_alignment_parameters_fail_startup(self):
        for key in ('~alignment_conflict_position_m','~alignment_conflict_stable_sec','~pair_wait_timeout_sec'):
            for value in (0.,-1.,math.nan,math.inf):
                self.params[key]=value
                with self.assertRaises(ValueError):self.bridge()
            self.params.pop(key)
        self.params['~pair_wait_timeout_sec']=1.01
        with self.assertRaises(ValueError):self.bridge()

    def test_evening_recorded_transform_step_no_longer_moves_car_metres(self):
        path=Path(__file__).resolve().parents[3]/'tracker_logs/system_run_full_20261020_191857.jsonl'
        if not path.exists():self.skipTest('optional retained evening log unavailable')
        previous=None;chosen=None
        with path.open() as stream:
            for line in stream:
                row=json.loads(line)
                if row.get('record_type')!='sample':continue
                sample=row['data'];g=sample.get('gps_guard') or {}
                if 1792495959.9<sample['stamp']<1792495960.1 and g.get('reason')=='gps_stable':
                    chosen=sample;break
                if g.get('reason')=='gps_recovery_wait':previous=sample
        self.assertIsNotNone(chosen);self.assertIsNotNone(previous)
        def transform(s):
            g=s['gps_guard'];return g['transform_x'],g['transform_y'],math.radians(g['transform_yaw_deg'])
        old,target=transform(previous),transform(chosen)
        p=chosen['local_odom'];anchor=(p['x'],p['y'])
        b=self.bridge();self.anchor(b,*anchor);b.transform_x,b.transform_y,b.transform_yaw=old
        q0=b.project_alignment(old,anchor);q1=b.project_alignment(target,anchor)
        old_jump=math.hypot(q1[0]-q0[0],q1[1]-q0[1])
        self.assertGreater(old_jump,5.)
        b.move_transform_toward(target,.1,.5,math.radians(8))
        q2=b.project_alignment((b.transform_x,b.transform_y,b.transform_yaw),anchor)
        new_step=math.hypot(q2[0]-q0[0],q2[1]-q0[1])
        self.assertLessEqual(new_step,.05+1e-9)
        # Fixed-pose convergence checks the new residual/window semantics too.
        for _ in range(240):b.move_transform_toward(target,.1,.5,math.radians(8))
        self.assertLess(b.transform_error(target)[0],1e-8)
        print('P1 retained-log step: old %.6fm -> anchored %.6fm (0.05m limit)' % (old_jump,new_step))


class BufferedAlignmentTests(unittest.TestCase):
    """Real callbacks with independent source/receive clocks, no ROS master."""
    setUp = fixture.RuntimePositionTests.setUp
    bridge = fixture.RuntimePositionTests.bridge

    def local(self, b, source, received=None):
        self.now = rospy.Time.from_sec(source if received is None else received)
        msg = Odometry()
        msg.header.stamp = rospy.Time.from_sec(source)
        msg.pose.pose.position.x = (source-100.)*.4
        msg.pose.pose.orientation.w = 1.
        b.local_odom_cb(msg)

    def gnss(self, b, source, received=None, quality=4, projection_offset=0.,
             defer_gga=False, defer_solution=False, yaw=.3, heading_fixed=None):
        self.now = rospy.Time.from_sec(source if received is None else received)
        stamp = rospy.Time.from_sec(source)
        gps = Odometry()
        gps.header.stamp = rospy.Time.from_sec(source+projection_offset)
        x = (source+projection_offset-100.)*.4
        gps.pose.pose.position.x = 100.+math.cos(.3)*x
        gps.pose.pose.position.y = 20.+math.sin(.3)*x
        gps.pose.pose.orientation.w = 1.
        b.gps_odom_cb(gps)
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.status.status = 2 if quality in (4,5) else -1
        fix.latitude, fix.longitude = 23.04, 113.39
        fix.position_covariance[0] = fix.position_covariance[4] = .01 if quality==4 else 1.
        fix.position_covariance_type = 1
        b.fix_cb(fix)
        gga = Gpgga()
        gga.header.stamp = stamp
        gga.gps_qual, gga.num_sats, gga.hdop = quality, 12, 1.
        heading_fixed = quality==4 if heading_fixed is None else heading_fixed
        solution = String(data=json.dumps({'stamp':source,'stamp_ns':stamp.to_nsec(),
                                          'heading_valid':heading_fixed,'heading_fixed':heading_fixed}))
        if not defer_gga:
            b.gga_cb(gga)
        if not defer_solution:
            b.solution_cb(solution)
        heading = QuaternionStamped()
        heading.header.stamp = stamp
        heading.quaternion.x, heading.quaternion.y, heading.quaternion.z, heading.quaternion.w = quaternion_from_euler(0,0,yaw)
        if heading_fixed:
            b.heading_cb(heading)
        return gga, solution

    def initialized_tracking(self):
        b = self.bridge()
        b.set_mode(b.TRACKING, 'fixture_fixed')
        self.local(b,100.)
        self.gnss(b,100.)
        b.update_guard(self.now)
        self.assertEqual(b.mode,b.TRACKING)
        return b

    def test_brief_pair_wait_keeps_tracking_transform_and_histories(self):
        b = self.initialized_tracking()
        old = b.transform_x,b.transform_y,b.transform_yaw
        window = list(b.recovery_candidates)
        headings = list(b.heading_transform_history)
        self.gnss(b,100.1)
        b.update_guard(self.now)
        self.assertEqual(b.mode,b.TRACKING)
        self.assertEqual(list(b.recovery_candidates),window)
        self.assertEqual(list(b.heading_transform_history),headings)
        self.assertEqual((b.transform_x,b.transform_y,b.transform_yaw),old)
        self.assertEqual(b.source_stamp_watermarks['gps_odom'],rospy.Time(100).to_nsec())
        self.assertEqual(b.source_stamp_watermarks['heading'],rospy.Time(100).to_nsec())
        self.assertIsNone(b.dead_reckoning_budget.started_at)
        self.local(b,100.12,100.13)
        b.update_guard(self.now)
        self.assertEqual(b.mode,b.TRACKING)
        self.assertEqual(b.last_processed_gps_stamp_ns,rospy.Time.from_sec(100.1).to_nsec())
        self.assertIsNone(b.pair_wait_since)

    def test_10hz_gps_with_120ms_local_delay_keeps_processing(self):
        b = self.bridge()
        b.set_mode(b.TRACKING,'fixture_fixed')
        processed = set()
        for i in range(401):
            now = 100.+i*.02
            self.local(b,now-.12,now)
            if i%5==0:
                self.gnss(b,now,now)
            b.update_guard(self.now)
            if b.last_processed_gps_stamp_ns is not None:
                processed.add(b.last_processed_gps_stamp_ns)
            self.assertEqual(b.mode,b.TRACKING,msg=(i,b.reason,b.source_pairing))
        self.assertGreaterEqual(len(processed),75)
        self.assertLess((self.now.to_nsec()-b.last_processed_gps_stamp_ns)*1e-9,.25)
        self.assertFalse(b.alignment_conflict)

    def test_pending_observation_keeps_its_original_float_quality(self):
        b = self.bridge()
        self.local(b,99.98,100.)
        self.gnss(b,100.,100.,quality=5)
        self.local(b,100.04,100.12)
        self.gnss(b,100.1,100.12,quality=4)
        b.update_guard(self.now)
        self.assertEqual(b.last_processed_gps_stamp_ns,rospy.Time(100).to_nsec())
        self.assertEqual(b.runtime_position_profile,'float')
        self.assertEqual(b.observation_buffer_diagnostics['selected_gga_quality'],5)
        self.assertFalse(b.observation_buffer_diagnostics['selected_heading_fixed'])
        self.assertNotEqual(b.mode,b.TRACKING)
        # Latest callbacks remain available, but cannot backfill old quality.
        self.assertEqual(b.last_gga.gps_qual,4)
        self.assertTrue(b.last_solution['heading_fixed'])
        self.local(b,100.12,100.15)
        b.update_guard(self.now)
        self.assertEqual(b.last_processed_gps_stamp_ns,rospy.Time.from_sec(100.1).to_nsec())

    def test_delayed_callbacks_can_initialize_and_track(self):
        b = self.bridge(initialized=False)
        for i in range(401):
            now = 100.+i*.02
            self.local(b,now-.12,now)
            if i%5==0:
                self.gnss(b,now)
            b.update_guard(self.now)
        self.assertTrue(b.transform_initialized)
        self.assertEqual(b.initialization_precision,'high')
        self.assertEqual(b.mode,b.TRACKING)

    def test_delayed_float_callbacks_keep_position_only_policy(self):
        b = self.bridge()
        old_yaw = b.transform_yaw
        for i in range(651):
            now = 100.+i*.02
            self.local(b,now-.12,now)
            if i%5==0:
                self.gnss(b,now,quality=5)
            b.update_guard(self.now)
        self.assertEqual(b.mode,b.POSITION_TRACKING)
        self.assertGreater(b.runtime_position_credit_count,1)
        self.assertEqual(b.transform_yaw,old_yaw)

    def test_brief_wait_preserves_recovering_window(self):
        b = self.initialized_tracking()
        b.set_mode(b.RECOVERING,'fixture_recovery')
        window = list(b.recovery_candidates)
        self.gnss(b,100.1)
        b.update_guard(self.now)
        self.assertEqual(b.mode,b.RECOVERING)
        self.assertEqual(list(b.recovery_candidates),window)

    def test_stale_heading_is_not_hidden_by_position_bracket_wait(self):
        b = self.initialized_tracking()
        self.gnss(b,100.6)
        b.observation_history['heading'].clear()
        b.last_heading_stamp = rospy.Time(100)
        b.last_heading_recv = rospy.Time(100)
        b.update_guard(self.now)
        self.assertEqual(b.mode,b.DEAD_RECKONING)
        self.assertIsNotNone(b.dead_reckoning_budget.started_at)

    def test_continuous_unmatched_packets_cannot_extend_grace(self):
        b = self.initialized_tracking()
        for i in range(1,8):
            now = 100.+i*.2
            self.local(b,now)
            self.gnss(b,now,projection_offset=-.08)
            b.update_guard(self.now)
            if i==1:
                self.assertEqual(b.mode,b.TRACKING)
            if i>=4:
                self.assertEqual(b.mode,b.DEAD_RECKONING)
        self.assertAlmostEqual(b.dead_reckoning_budget.started_at,100.2)
        self.assertGreater(b.dead_reckoning_budget.distance_m,.4)
        self.assertEqual(b.last_processed_gps_stamp_ns,rospy.Time(100).to_nsec())

    def test_invalid_quality_during_grace_still_leaves_tracking(self):
        b = self.initialized_tracking()
        self.gnss(b,100.1)
        b.update_guard(self.now)
        self.assertEqual(b.mode,b.TRACKING)
        self.gnss(b,100.2,quality=0)
        b.update_guard(self.now)
        self.assertEqual(b.mode,b.DEAD_RECKONING)
        self.assertIsNotNone(b.dead_reckoning_budget.started_at)

    def test_history_is_bounded_and_stale_observation_is_not_applied(self):
        b = self.initialized_tracking()
        for i in range(150):
            self.gnss(b,100.+i*.001)
        self.assertTrue(all(len(h)<=128 for h in b.observation_history.values()))
        self.local(b,102.,102.)
        old = b.last_processed_gps_stamp_ns
        b.update_guard(self.now)
        self.assertEqual(b.last_processed_gps_stamp_ns,old)
        self.assertEqual(b.mode,b.DEAD_RECKONING)

    def test_quality_transition_waits_for_its_own_epoch(self):
        for old_fixed, new_fixed in ((True,False),(False,True)):
            for missing in ('gga','solution','both'):
                with self.subTest(old_fixed=old_fixed,new_fixed=new_fixed,missing=missing):
                    b = self.initialized_tracking() if old_fixed else self.bridge()
                    if not old_fixed:
                        self.local(b,100.)
                        self.gnss(b,100.,quality=5)
                        b.update_guard(self.now)
                    previous_stamp = b.last_processed_gps_stamp_ns
                    old_yaw = b.transform_yaw
                    self.local(b,100.1)
                    gga, solution = self.gnss(b,100.1,quality=4 if new_fixed else 5,
                        defer_gga=missing in ('gga','both'),
                        defer_solution=missing in ('solution','both'), yaw=.3+math.radians(1.5))
                    b.update_guard(self.now)
                    self.assertEqual(b.transform_yaw,old_yaw)
                    if not new_fixed and missing=='solution':
                        # Fully matched Float position needs no new heading or
                        # heading-quality message. It may advance position-only.
                        self.assertEqual(b.last_processed_gps_stamp_ns,rospy.Time.from_sec(100.1).to_nsec())
                    else:
                        self.assertEqual(b.last_processed_gps_stamp_ns,previous_stamp)
                        self.assertEqual(b.observation_buffer_diagnostics['state'],'pair_wait')
                    self.now=rospy.Time.from_sec(100.11)
                    b.gga_cb(gga)
                    b.solution_cb(solution)
                    b.update_guard(self.now)
                    self.assertEqual(b.last_processed_gps_stamp_ns,rospy.Time.from_sec(100.1).to_nsec())
                    if not new_fixed:
                        self.assertEqual(b.transform_yaw,old_yaw)

    def test_missing_quality_times_out_without_rotating_or_consuming(self):
        b=self.initialized_tracking()
        old_yaw=b.transform_yaw
        previous_stamp=b.last_processed_gps_stamp_ns
        self.local(b,100.1)
        self.gnss(b,100.1,defer_gga=True,defer_solution=True,yaw=.4)
        for i in range(1,41):
            self.local(b,100.1+i*.02)
            b.update_guard(self.now)
        self.assertEqual(b.transform_yaw,old_yaw)
        self.assertEqual(b.last_processed_gps_stamp_ns,previous_stamp)
        self.assertEqual(b.mode,b.DEAD_RECKONING)

    def test_solution_identity_is_exact_and_legacy_nearest_quality_is_rejected(self):
        from global_odom_bridge import GlobalOdomBridge
        now=rospy.Time(1792500000,123456789)
        ns=now.to_nsec()
        self.assertTrue(GlobalOdomBridge.same_solution_observation({'stamp':now.to_sec(),'stamp_ns':ns},ns))
        self.assertFalse(GlobalOdomBridge.same_solution_observation({'stamp':now.to_sec(),'stamp_ns':ns-1},ns))
        self.assertFalse(GlobalOdomBridge.same_solution_observation({'stamp':now.to_sec()},ns))
        self.assertFalse(GlobalOdomBridge.same_solution_observation({'stamp':now.to_sec()-.01},ns))


if __name__=='__main__':unittest.main()
