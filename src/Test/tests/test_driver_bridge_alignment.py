"""P1-D/E/F: real G90 parsing -> real bridge callbacks, no master/serial IO.

Projection geometry is synthetic; its Header is copied exactly from the real
driver's fix, as in gps_fix_to_map_odom. Subscriber delivery order is controlled.
"""
import copy
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import test_runtime_position as fixture  # Shared fixture in this central test directory.

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/wheeltec_gps_driver/src'))
from libnmea_navsat_driver.driver import RosNMEADriver
from libnmea_navsat_driver.unicore import crc32


class DriverBridgeAlignmentTests(unittest.TestCase):
    bridge = fixture.RuntimePositionTests.bridge

    def setUp(self):
        fixture.RuntimePositionTests.setUp(self)
        self.params['~rotation']=0.
        self.params['~heading_source_is_north_cw']=True
        self.bias=0.
        self.held=[]
        self.hold_fix=self.hold_gga=self.hold_projection=False
        self.sent_headings=0
        self.sent_solutions=0
        self.last_solution_message=None
        self.b=self.bridge()
        p=patch.object(rospy,'get_rostime',lambda:self.now)
        p.start();self.addCleanup(p.stop)
        def publisher(topic,*args,**kwargs):
            result=Mock()
            callbacks={'/gps/fix':self.fix, '/gnss/gpgga':self.gga,
                       '/gnss/solution_status':self.solution, 'heading':self.heading}
            if topic in callbacks:result.publish.side_effect=callbacks[topic]
            return result
        with patch.object(rospy,'Publisher',side_effect=publisher):
            self.driver=RosNMEADriver()

    def fix(self,msg):
        if self.hold_fix:self.held.append(('fix',copy.deepcopy(msg)))
        else:self.b.fix_cb(msg)
        projection=Odometry()
        projection.header=copy.deepcopy(msg.header)
        x=(msg.header.stamp.to_sec()-100.)*.4
        projection.pose.pose.position.x=100.+math.cos(.3)*x+self.bias
        projection.pose.pose.position.y=20.+math.sin(.3)*x
        projection.pose.pose.orientation.w=1.
        if self.hold_projection:self.held.append(('projection',projection))
        else:self.b.gps_odom_cb(projection)

    def gga(self,msg):
        if self.hold_gga:self.held.append(('gga',copy.deepcopy(msg)))
        else:self.b.gga_cb(msg)

    def solution(self,msg):
        self.sent_solutions+=1
        self.last_solution_message=copy.deepcopy(msg)
        self.b.solution_cb(msg)

    def heading(self,msg):
        self.sent_headings+=1
        self.b.heading_cb(msg)

    def local(self,source,received=None):
        self.now=rospy.Time.from_sec(source if received is None else received)
        msg=Odometry()
        msg.header.stamp=rospy.Time.from_sec(source)
        msg.pose.pose.position.x=(source-100.)*.4
        msg.pose.pose.orientation.w=1.
        self.b.local_odom_cb(msg)

    def emit(self,source,quality=4,heading_type='NARROW_INT',yaw=.3,baseline='1',bad_crc=False):
        self.now=rospy.Time.from_sec(source)
        body='GNGGA,120000.00,2302.4000,N,11323.4000,E,%d,12,0.2,10.0,M,0.0,M,,' % quality
        checksum=0
        for c in body:checksum^=ord(c)
        self.driver.add_sentence('$'+body+'*%02X'%checksum,'gps',timestamp=self.now)
        fields=['0']*35
        fields[0]='NARROW_INT' if quality==4 else 'NARROW_FLOAT'
        fields[2:4]=['23.04','113.39']
        fields[20:24]=[heading_type,baseline,str((90-math.degrees(yaw))%360),'0']
        fields[25]='12'
        raw='#PVTSLNA,95,GPS,UNKNOWN,1,66874000,0,0,18,2;'+','.join(fields)
        self.driver.add_sentence(raw+'*'+('00000000' if bad_crc else '%08x'%crc32(raw[1:])),'gps')

    def tracking(self):
        self.b.set_mode(self.b.TRACKING,'fixture_fixed')
        self.local(100.)
        self.emit(100.)
        self.b.update_guard(self.now)
        self.assertEqual(self.b.mode,self.b.TRACKING)

    def test_real_float_driver_emits_no_heading_and_120ms_delay_still_renews(self):
        for quality in (4,5):
            with self.subTest(position_quality=quality,heading_type='NARROW_FLOAT'):
                self.b=self.bridge()
                self.sent_headings=self.sent_solutions=0
                yaw=self.b.transform_yaw
                for i in range(651):
                    t=100.+i*.02
                    self.local(t-.12,t)
                    if i%5==0:self.emit(t,quality=quality,heading_type='NARROW_FLOAT')
                    self.b.update_guard(self.now)
                self.assertEqual(self.sent_headings,0)
                self.assertEqual(self.sent_solutions,131)
                self.assertGreater(len(self.b.runtime_position_window.history),8)
                self.assertGreaterEqual(self.b.runtime_position_credit_count,2)
                self.assertEqual(self.b.mode,self.b.POSITION_TRACKING)
                self.assertEqual(self.b.transform_yaw,yaw)
                self.assertLess((self.now.to_nsec()-self.b.last_processed_gps_stamp_ns)*1e-9,.25)

    def test_position_and_heading_fixed_states_are_independent(self):
        for quality in (4,5):
            for heading_type in ('NARROW_INT','NARROW_FLOAT'):
                with self.subTest(position_quality=quality,heading_type=heading_type):
                    self.b=self.bridge()
                    self.sent_headings=0
                    self.local(100.)
                    self.emit(100.,quality=quality,heading_type=heading_type)
                    heading_fixed=heading_type=='NARROW_INT'
                    self.assertEqual(self.sent_headings,int(heading_fixed))
                    status=json.loads(self.last_solution_message.data)
                    self.assertEqual(status['position_fixed'],quality==4)
                    self.assertEqual(status['heading_fixed'],heading_fixed)
                    self.assertTrue(self.b.data_quality(self.now,runtime_position=True)[0])
                    self.assertEqual(self.b.data_quality(self.now)[0],quality==4 and heading_fixed)
                    old_yaw=self.b.transform_yaw
                    self.b.update_guard(self.now)
                    if not heading_fixed or quality!=4:
                        self.assertEqual(self.b.transform_yaw,old_yaw)
                        self.assertEqual(self.b.runtime_position_profile,
                                         'fixed_position_only' if quality==4 else 'float')

    def test_real_float_driver_cannot_initialize_without_fixed_heading(self):
        self.b=self.bridge(initialized=False)
        for i in range(151):
            t=100.+i*.1
            self.local(t-.12,t)
            self.emit(t,quality=5,heading_type='NARROW_FLOAT')
            self.b.update_guard(self.now)
        self.assertFalse(self.b.transform_initialized)
        self.assertEqual(self.b.runtime_position_credit_count,0)

    def test_latest_heading_failure_overrides_fresh_old_fixed_cache(self):
        for failure in ('float','invalid','crc'):
            with self.subTest(failure=failure):
                self.setUp_case_tracking()
                self.b.transform_yaw-=.02
                yaw=self.b.transform_yaw
                self.local(100.1)
                self.emit(100.1,quality=4,heading_type='NARROW_FLOAT' if failure=='float' else 'NARROW_INT',
                          baseline='0' if failure=='invalid' else '1',bad_crc=failure=='crc')
                self.assertEqual(self.sent_headings,1)
                self.b.update_guard(self.now)
                self.assertEqual(self.b.transform_yaw,yaw)
                self.assertNotEqual(self.b.mode,self.b.TRACKING)
                self.assertIsNotNone(self.b.dead_reckoning_budget.started_at)
                self.assertFalse(self.b.latest_heading_health(self.now)['valid'])
                self.assertEqual(self.b.runtime_position_credit_count,0)

    def setUp_case_tracking(self):
        # Reuse patched ROS lifecycle, but clear every per-case source cache.
        self.b=self.bridge()
        self.sent_headings=self.sent_solutions=0
        self.tracking()

    def test_late_old_fixed_status_cannot_undo_new_float_failure(self):
        self.tracking()
        old=copy.deepcopy(self.last_solution_message)
        self.local(100.1)
        self.emit(100.1,quality=4,heading_type='NARROW_FLOAT')
        self.b.solution_cb(old)
        yaw=self.b.transform_yaw
        self.b.update_guard(self.now)
        self.assertFalse(self.b.latest_heading_health(self.now)['valid'])
        self.assertNotEqual(self.b.mode,self.b.TRACKING)
        self.assertEqual(self.b.transform_yaw,yaw)

    def test_invalid_health_cannot_be_cleared_by_duplicate_fixed(self):
        self.tracking()
        old=copy.deepcopy(self.last_solution_message)
        self.local(100.1)
        self.b.solution_cb(String(data='{broken'))
        self.b.solution_cb(old)
        self.b.update_guard(self.now)
        self.assertFalse(self.b.latest_heading_health(self.now)['valid'])
        self.assertNotEqual(self.b.mode,self.b.TRACKING)
        self.assertIsNotNone(self.b.dead_reckoning_budget.started_at)

    def test_new_fixed_health_does_not_backfill_old_float_observation(self):
        self.local(99.98,100.)
        self.emit(100.,quality=5,heading_type='NARROW_FLOAT')
        self.local(100.04,100.1)
        self.emit(100.1,quality=4)
        self.b.update_guard(self.now)
        self.assertEqual(self.b.last_processed_gps_stamp_ns,rospy.Time(100).to_nsec())
        self.assertEqual(self.b.runtime_position_profile,'float')
        self.assertEqual(self.b.observation_buffer_diagnostics['selected_gga_quality'],5)
        self.assertNotEqual(self.b.mode,self.b.TRACKING)

    def test_20hz_projection_must_wait_for_its_own_fix_and_gga(self):
        for first,second in ((4,5),(5,4)):
            with self.subTest(first=first,second=second):
                self.setUp_case_tracking()
                self.local(100.05)
                self.hold_projection=True
                self.emit(100.05,quality=first)
                self.hold_projection=False
                self.local(100.10)
                self.hold_fix=self.hold_gga=True
                self.emit(100.10,quality=second)
                yaw=self.b.transform_yaw
                previous=self.b.last_processed_gps_stamp_ns
                self.b.update_guard(self.now)
                self.assertEqual(self.b.last_processed_gps_stamp_ns,previous)
                self.assertEqual(self.b.transform_yaw,yaw)
                self.assertFalse(self.b.data_quality(self.now,runtime_position=True)[0])
                for kind,msg in self.held:
                    if kind=='fix':self.b.fix_cb(msg)
                    elif kind=='gga':self.b.gga_cb(msg)
                self.held=[]
                self.hold_fix=self.hold_gga=False
                self.b.update_guard(self.now)
                self.assertEqual(self.b.last_processed_gps_stamp_ns,rospy.Time.from_sec(100.10).to_nsec())
                self.assertEqual(self.b.source_pairing['gps_odom_ns'],self.b.source_pairing['gps_fix_ns'])

    def test_fix_first_missing_projection_does_not_consume_new_frame(self):
        self.tracking()
        self.local(100.05)
        self.hold_projection=True
        self.emit(100.05,quality=5,heading_type='NARROW_FLOAT')
        previous=self.b.last_processed_gps_stamp_ns
        self.b.update_guard(self.now)
        self.assertEqual(self.b.last_processed_gps_stamp_ns,previous)
        for kind,msg in self.held:
            if kind=='projection':self.b.gps_odom_cb(msg)
        self.held=[]
        self.b.update_guard(self.now)
        self.assertEqual(self.b.last_processed_gps_stamp_ns,rospy.Time.from_sec(100.05).to_nsec())

    def test_bad_quality_or_large_innovation_cannot_renew_without_heading(self):
        for quality,bias in ((0,0.),(5,4.)):
            with self.subTest(quality=quality,bias=bias):
                self.b=self.bridge()
                self.bias=bias
                for i in range(131):
                    t=100.+i*.1
                    self.local(t-.12,t)
                    self.emit(t,quality=quality,heading_type='NARROW_FLOAT')
                    self.b.update_guard(self.now)
                self.assertEqual(self.b.runtime_position_credit_count,0)
                self.assertEqual(self.b.transform_yaw,.3)


if __name__=='__main__':unittest.main()
