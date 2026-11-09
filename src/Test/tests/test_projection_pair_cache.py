"""Actual projection class, fake clocks/publishers; no ROS master or devices."""
import copy
import unittest
from unittest.mock import patch
import test_gnss_measurement_contract as fixture
from test_gnss_observation_time import NS, pvts, nmea


@unittest.skipUnless(fixture.ROS, 'ROS1 fake messages required')
class PairCacheTests(unittest.TestCase):
    fix = fixture.MeasurementPipelineTests.fix
    projection = fixture.MeasurementPipelineTests.projection
    send = fixture.MeasurementPipelineTests.send

    def setUp(self):
        fixture.MeasurementPipelineTests.setUp(self)
        self.now += fixture.rospy.Duration(.3); self.mono += 300000000
        self.send(pvts(self.epoch+200000000))
        self.send(nmea('GNGGA,000000.250,2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'))
        self.fix_message = self.driver.fix_pub.publish.call_args.args[0]
        self.metadata = self.driver.time_diagnostics_pub.publish.call_args.args[0]

    def test_pending_is_not_a_time_failure_warning(self):
        p = self.projection()
        p.fix_cb(self.fix_message)
        with patch.object(fixture.rospy, 'logwarn_throttle') as warning:
            p.timer_cb(None)
            self.assertFalse(warning.called)
        p.time_metadata_cb(self.metadata); p.timer_cb(None)
        self.assertEqual(p.pub.publish.call_count, 1)

    def test_latest_unpaired_fix_does_not_starve_older_complete_epoch(self):
        p = self.projection()
        p.fix_cb(self.fix_message); p.time_metadata_cb(self.metadata)
        newer = copy.deepcopy(self.fix_message)
        newer.header.stamp += fixture.rospy.Duration(.025)
        p.fix_cb(newer); p.timer_cb(None)
        self.assertEqual(p.pub.publish.call_count, 1)
        self.assertEqual(p.pub.publish.call_args.args[0].header.stamp, self.fix_message.header.stamp)

    def test_metadata_first_and_duplicate_never_republish(self):
        p=self.projection();p.time_metadata_cb(self.metadata)
        p.fix_cb(self.fix_message);p.timer_cb(None)
        p.fix_cb(self.fix_message);p.timer_cb(None)
        self.assertEqual(p.pub.publish.call_count,1)
        self.assertEqual(len(p.pending_fixes),0)

    def test_timeout_rejects_missing_metadata_without_restamping(self):
        p=self.projection();p.fix_cb(self.fix_message)
        arrived=p.pending_fixes[self.fix_message.header.stamp.to_nsec()][1]
        self.now+=fixture.rospy.Duration(.16);self.mono+=160000000
        p.fix_cb(self.fix_message)  # duplicate cannot renew the pairing lease
        self.assertEqual(p.pending_fixes[self.fix_message.header.stamp.to_nsec()][1],arrived)
        with patch.object(fixture.rospy,'logwarn_throttle') as warning:
            p.timer_cb(None)
            self.assertTrue(warning.called)
        self.assertFalse(p.pub.publish.called)
        self.assertEqual(p.pair_state,'pair_metadata_timeout')
        self.assertEqual(len(p.pending_fixes),0)

    def test_pending_capacity_is_bounded_and_session_fault_is_not_pending(self):
        p=self.projection()
        for i in range(100):
            fix=copy.deepcopy(self.fix_message)
            fix.header.stamp+=fixture.rospy.Duration.from_sec(i*.0001)
            p.fix_cb(fix)
        self.assertEqual(len(p.pending_fixes),64)
        p.timebase.fault='time_session_changed_restart_required'
        p.timer_cb(None)
        self.assertFalse(p.pub.publish.called)
        self.assertNotEqual(p.pair_state,'pair_pending')
        self.assertEqual(len(p.pending_fixes),0)

    def test_pairing_does_not_extend_source_freshness(self):
        p=self.projection();p.time_metadata_cb(self.metadata)
        p.fix_cb(self.fix_message)
        self.now+=fixture.rospy.Duration(1.1);self.mono+=1100000000
        p.timer_cb(None)
        self.assertFalse(p.pub.publish.called)
        self.assertNotEqual(p.pair_state,'paired')

