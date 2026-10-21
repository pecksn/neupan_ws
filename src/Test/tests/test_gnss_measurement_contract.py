"""Measurement-mode admission and real driver->bridge source-time regression."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/robot_base_driver/scripts'))
sys.path.insert(0, str(ROOT/'src/wheeltec_gps_driver/src'))
from gnss_timebase import GnssTimebase
from test_gnss_observation_time import NS, pvts, nmea, stamp


def metadata(t, family='GGA', mono=0, **changes):
    event = dict(mode='measurement', time_valid=True, clock_verified=True, session_id='test',
        clock_epoch=0, family=family, measurement_stamp_ns=t, host_read_mono_ns=mono,
        observation_id='test:0:{}:{}'.format(family, t))
    event.update(changes)
    return event


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.gate = GnssTimebase('measurement', True)
        self.t = stamp()

    def test_requires_explicit_clock_assertion(self):
        with self.assertRaises(ValueError):
            GnssTimebase('measurement')

    def test_metadata_delay_waits_exact_epoch_never_nearest(self):
        self.assertEqual(self.gate.check('GGA', self.t, self.t, 0, .5)[1], 'position_source_time_mismatch')
        self.gate.receive(metadata(self.t), self.t, 0)
        self.assertTrue(self.gate.check('GGA', self.t, self.t, 0, .5)[0])
        self.assertFalse(self.gate.check('GGA', self.t+1, self.t, 0, .5)[0])

    def test_source_age_and_receive_age_checked_separately(self):
        self.gate.receive(metadata(self.t), self.t, 0)
        self.assertFalse(self.gate.check('GGA', self.t, self.t+600000000, 600000000, .5)[0])
        self.assertEqual(self.gate.metadata['GGA', self.t]['host_read_mono_ns'], 0)

    def test_duplicate_does_not_refresh_receive_or_clear_invalid_latest(self):
        self.gate.receive(metadata(self.t), self.t, 0)
        self.gate.receive(metadata(self.t, mono=100000000), self.t+100000000, 100000000)
        self.assertEqual(self.gate.metadata['GGA', self.t]['host_read_mono_ns'], 0)
        self.gate.receive(metadata(None, time_valid=False), self.t+200000000, 200000000)
        self.gate.receive(metadata(self.t), self.t+300000000, 300000000)
        self.assertFalse(self.gate.check('GGA', self.t, self.t+300000000, 300000000, .5)[0])
        self.gate.receive(metadata(self.t+400000000, mono=400000000), self.t+400000000, 400000000)
        self.assertTrue(self.gate.check('GGA', self.t+400000000, self.t+400000000, 400000000, .5)[0])

    def test_future_invalid_metadata_does_not_poison_watermark(self):
        self.gate.receive(metadata(self.t+10*NS), self.t, 0)
        self.assertNotIn('GGA', self.gate.watermarks)
        self.gate.receive(metadata(self.t+100000000, mono=100000000), self.t+100000000, 100000000)
        self.assertTrue(self.gate.check('GGA', self.t+100000000, self.t+100000000, 100000000, .5)[0])

    def test_clock_step_is_latched_until_restart(self):
        self.gate.receive(metadata(self.t), self.t, 0)
        self.gate.tick(self.t+NS, 100000000)
        self.assertIsNotNone(self.gate.fault)
        self.gate.receive(metadata(self.t+NS, mono=100000000), self.t+NS, 100000000)
        self.assertFalse(self.gate.check('GGA', self.t+NS, self.t+NS, 100000000, .5)[0])

    def test_session_epoch_mode_changes_rejected(self):
        for change in ({'session_id': 'new'}, {'clock_epoch': 1}, {'mode': 'shadow'}, {'clock_verified': False}):
            with self.subTest(change=change):
                gate = GnssTimebase('measurement', True)
                gate.receive(metadata(self.t), self.t, 0)
                gate.receive(metadata(self.t+100000000, **change), self.t+100000000, 100000000)
                self.assertIsNotNone(gate.fault)
        gate = GnssTimebase()
        gate.receive(metadata(self.t), self.t, 0)
        self.assertIsNotNone(gate.fault)

    def test_metadata_memory_bounded(self):
        for i in range(1000):
            self.gate.receive(metadata(self.t+i, mono=i), self.t+i, i)
        self.assertEqual(len(self.gate.metadata), 256)


try:
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf.transformations import quaternion_from_euler
    from libnmea_navsat_driver.driver import RosNMEADriver
    import test_runtime_position as fixture
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'requires ROS1, fake publishers only')
class MeasurementPipelineTests(unittest.TestCase):
    def setUp(self):
        fixture.RuntimePositionTests.setUp(self)
        self.params.update({'~time_mode': 'measurement', '~time_clock_verified': True,
            '~gps_time_mode': 'measurement', '~gps_time_clock_verified': True,
            '~rotation': 0., '~epe_quality5': 1.})  # Synthetic Float accuracy inside existing gate.
        self.epoch = stamp()
        self.now = rospy.Time(self.epoch//NS)
        self.mono = 1000000000
        for p in (patch('time.monotonic_ns', lambda: self.mono),
                  patch.object(rospy, 'get_rostime', lambda: self.now)):
            p.start(); self.addCleanup(p.stop)
        self.b = fixture.RuntimePositionTests.bridge(self)
        def publisher(topic, *a, **k):
            pub = Mock()
            callback = {'/gps/fix': self.fix, '/gnss/gpgga': self.b.gga_cb,
                        '/gnss/solution_status': self.b.solution_cb,
                        'heading': self.b.heading_cb,
                        '/gnss/time_diagnostics': self.b.time_metadata_cb}.get(topic)
            if callback:
                pub.publish.side_effect = callback
            return pub
        with patch.object(rospy, 'Publisher', side_effect=publisher):
            self.driver = RosNMEADriver()

    def fix(self, fix):
        self.b.fix_cb(fix)
        odom = Odometry(); odom.header = copy.deepcopy(fix.header)
        t = (fix.header.stamp.to_nsec()-self.epoch)/NS
        x, y = math.sin(t), 1-math.cos(t)
        odom.pose.pose.position.x = 100+math.cos(.3)*x-math.sin(.3)*y
        odom.pose.pose.position.y = 20+math.sin(.3)*x+math.cos(.3)*y
        odom.pose.pose.orientation.w = 1.
        self.b.gps_odom_cb(odom)

    def local(self, t):
        msg = Odometry(); msg.header.stamp = rospy.Time.from_sec(self.epoch/NS+t)
        msg.pose.pose.position.x, msg.pose.pose.position.y = math.sin(t), 1-math.cos(t)
        q = quaternion_from_euler(0, 0, t)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        self.b.local_odom_cb(msg)

    def send(self, text):
        return self.driver.add_sentence(text, 'gps', host_read_ros_ns=self.now.to_nsec(), host_read_mono_ns=self.mono)

    def test_turning_pose_matches_distinct_position_heading_epochs_under_delay(self):
        for delay in (0, .05, .12, .2):
            with self.subTest(delay=delay):
                self.setUp()
                self.now = rospy.Time.from_sec(self.epoch/NS+.3+delay)
                self.mono = 1000000000+int((.3+delay)*NS)
                for i in range(int(round((.3+delay)/.05))+1):
                    self.local(i*.05)
                self.send(pvts(self.epoch+200000000))
                self.send(nmea('GNGGA,000000.250,2302.4,N,11323.4,E,4,12,1.0,5,M,0,M,,0000'))
                good, reason = self.b.data_quality(self.now)
                self.assertTrue(good, reason)
                self.assertEqual(self.b.position_pair['stamp_ns'], self.epoch+250000000)
                self.assertAlmostEqual(self.b.heading_pair['local_pose'][2], .2, places=5)
                self.assertAlmostEqual(self.b.position_pair['local_pose'][2], .25, places=5)

    def test_invalid_time_and_crc_no_heading_or_fix_now_fallback(self):
        self.send(pvts(self.epoch, state='UNKNOWN'))
        self.assertFalse(self.driver.heading_pub.publish.called)
        self.send(nmea('GNGGA,000000,2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'))
        self.assertFalse(self.driver.fix_pub.publish.called)
        self.send(pvts(self.epoch)[:-8]+'00000000')
        state = json.loads(self.driver.solution_pub.publish.call_args[0][0].data)
        self.assertEqual(state['stamp_ns'], 0)
        self.assertFalse(state['heading_valid'])

    def test_repeated_measurement_does_not_publish_new_fix_or_heading(self):
        self.send(pvts(self.epoch))
        self.send(nmea('GNGGA,000000,2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'))
        self.assertEqual(self.driver.fix_pub.publish.call_count, 1)
        self.send(pvts(self.epoch))
        self.send(nmea('GNGGA,000000,2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'))
        self.assertEqual(self.driver.fix_pub.publish.call_count, 1)
        self.assertEqual(self.driver.heading_pub.publish.call_count, 1)

    def test_rejected_future_epoch_does_not_block_next_valid_fix(self):
        self.send(pvts(self.epoch+86400*NS))
        self.assertFalse(self.driver.heading_pub.publish.called)
        self.send(pvts(self.epoch))
        self.send(nmea('GNGGA,000000,2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'))
        self.assertEqual(self.driver.fix_pub.publish.call_count, 1)
        self.assertEqual(self.driver.heading_pub.publish.call_count, 1)

    def test_clock_step_keeps_budget_and_blocks_safety(self):
        self.b.gnss_timebase.tick(self.now.to_nsec(), self.mono)
        before = copy.deepcopy(vars(self.b.dead_reckoning_budget))
        self.now += rospy.Duration(2)
        self.mono += 100000000
        self.b._timer_cb(None)
        self.assertFalse(self.b.localization_is_safe(self.now))
        self.assertEqual(self.b.dead_reckoning_budget.started_at, before['started_at'])

    def test_four_quality_combinations_keep_time_independent_and_float_position_only(self):
        for position in ('NARROW_INT', 'NARROW_FLOAT'):
            for heading in ('NARROW_INT', 'NARROW_FLOAT'):
                with self.subTest(position=position, heading=heading):
                    self.setUp()
                    self.now += rospy.Duration(.3); self.mono += 300000000
                    for i in range(7):
                        self.local(i*.05)
                    self.send(pvts(self.epoch+200000000, position=position, heading=heading))
                    quality = '4' if position == 'NARROW_INT' else '5'
                    self.send(nmea('GNGGA,000000.250,2302.4,N,11323.4,E,{},12,1,5,M,0,M,,0000'.format(quality)))
                    good, reason = self.b.data_quality(self.now, runtime_position=True)
                    self.assertTrue(good, reason)
                    self.assertEqual(self.driver.heading_pub.publish.called, heading == 'NARROW_INT')
                    self.assertEqual(self.b.transform_yaw, .3)

    def projection(self):
        from gps_fix_to_map_odom import GpsFixToMapOdom
        with patch.object(rospy, 'Publisher', side_effect=lambda *a, **k: Mock()):
            return GpsFixToMapOdom()

    def test_projection_waits_metadata_then_preserves_stamp_and_publishes_once(self):
        self.now += rospy.Duration(.3); self.mono += 300000000
        self.send(pvts(self.epoch+200000000))
        self.send(nmea('GNGGA,000000.250,2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'))
        p = self.projection()
        p.fix_cb(self.driver.fix_pub.publish.call_args[0][0])
        p.timer_cb(None)
        self.assertFalse(p.pub.publish.called)
        p.time_metadata_cb(self.driver.time_diagnostics_pub.publish.call_args[0][0])
        p.timer_cb(None); p.timer_cb(None)
        self.assertEqual(p.pub.publish.call_count, 1)
        self.assertEqual(p.pub.publish.call_args[0][0].header.stamp.to_nsec(), self.epoch+250000000)

    def test_projection_zero_stamp_never_falls_back_to_now(self):
        from sensor_msgs.msg import NavSatFix
        p = self.projection()
        fix = NavSatFix(); fix.latitude, fix.longitude = 23., 113.
        p.fix_cb(fix); p.timer_cb(None)
        self.assertFalse(p.pub.publish.called)


if __name__ == '__main__':
    unittest.main()
