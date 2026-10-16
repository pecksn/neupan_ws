"""ROS1 message-level tests; no ROS master or vehicle nodes are started."""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import rospy
from global_odom_bridge import GlobalOdomBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Gpgga


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.now = rospy.Time(100)
        self.params = {}
        for name in ('init_node', 'Publisher', 'Subscriber', 'Timer'):
            p = patch.object(rospy, name)
            p.start()
            self.addCleanup(p.stop)
        for p in (patch.object(rospy, 'get_param', lambda key, default=None: self.params.get(key, default)),
                  patch.object(rospy.Time, 'now', lambda: self.now)):
            p.start()
            self.addCleanup(p.stop)

    def bridge(self, enabled=False):
        self.params['~low_precision_startup'] = enabled
        return GlobalOdomBridge()

    def sample(self, b, second, quality=5, covariance=16, x=10, heading=True):
        self.now = rospy.Time.from_sec(second)
        local = Odometry()
        local.header.stamp = self.now
        local.pose.pose.orientation.w = 1
        b.last_local_odom = local
        b.last_local_recv = self.now
        b.local_bad_until = rospy.Time(0)
        gps = Odometry()
        gps.header.stamp = self.now
        gps.pose.pose.position.x = x
        gps.pose.pose.position.y = 20
        b.gps_odom_cb(gps)
        fix = NavSatFix()
        fix.header.stamp = self.now
        fix.latitude, fix.longitude = 23.04, 113.39
        fix.status.status = 2 if quality in (4, 5) else (0 if quality == 1 else -1)
        fix.position_covariance[0] = fix.position_covariance[4] = covariance
        b.fix_cb(fix)
        gga = Gpgga()
        gga.header.stamp = self.now
        gga.gps_qual, gga.num_sats, gga.hdop = quality, 12, 1.0
        b.gga_cb(gga)
        b.last_solution = {'stamp': self.now.to_sec(), 'heading_fixed': heading, 'heading_valid': heading}
        b.last_solution_recv = self.now
        b.last_heading_yaw = math.pi / 2
        b.last_heading_recv = self.now
        b.last_heading_stamp = self.now

    def initialize_low(self, b):
        for t in (100, 101, 102):
            self.sample(b, t)
            b.update_guard(self.now)

    def test_default_rejects_float(self):
        b = self.bridge()
        self.initialize_low(b)
        self.assertFalse(b.transform_initialized)
        self.assertFalse(b.localization_is_safe(self.now))

    def test_low_initializes_with_fixed_heading_and_starts_budget(self):
        b = self.bridge(True)
        self.initialize_low(b)
        self.assertTrue(b.transform_initialized)
        self.assertEqual(b.initialization_precision, 'low')
        self.assertEqual(b.mode, b.DEAD_RECKONING)
        self.assertEqual(b.reason, 'low_precision_initialized')
        self.assertEqual(b.dead_reckoning_budget.started_at, 102)
        self.assertAlmostEqual(b.transform_yaw, math.pi / 2)
        self.assertTrue(b.localization_is_safe(self.now))

    def test_single_position_allowed_but_no_fix_not_allowed(self):
        b = self.bridge(True)
        for t in (100, 101, 102):
            self.sample(b, t, quality=1)
            b.update_guard(self.now)
        self.assertTrue(b.transform_initialized)
        b = self.bridge(True)
        self.sample(b, 103, quality=0)
        self.assertFalse(b.data_quality(self.now, low_precision=True)[0])

    def test_requires_fixed_fresh_heading_and_local_odometry(self):
        b = self.bridge(True)
        self.sample(b, 100, heading=False)
        self.assertEqual(b.data_quality(self.now, True)[1], 'heading_solution_unfixed')
        self.sample(b, 100)
        b.last_solution_recv = rospy.Time(96)
        self.assertEqual(b.data_quality(self.now, True)[1], 'heading_solution_stale')
        self.sample(b, 100)
        b.last_heading_recv = rospy.Time(99)
        self.assertEqual(b.data_quality(self.now, True)[1], 'heading_stale')
        self.sample(b, 100)
        b.last_local_recv = rospy.Time(98)
        self.assertEqual(b.data_quality(self.now, True)[1], 'local_odom_stale')

    def test_unbounded_error_and_jitter_rejected(self):
        b = self.bridge(True)
        self.sample(b, 100, covariance=101)
        self.assertFalse(b.data_quality(self.now, True)[0])
        for t, x in ((100, 0), (101, 10), (102, 0)):
            self.sample(b, t, x=x)
            b.update_guard(self.now)
        self.assertFalse(b.transform_initialized)

    def test_coarse_gps_does_not_move_anchor_or_reset_budget(self):
        b = self.bridge(True)
        self.initialize_low(b)
        anchor = b.transform_x, b.transform_y, b.transform_yaw
        self.sample(b, 103, x=30)
        b.update_guard(self.now)
        self.assertEqual(anchor, (b.transform_x, b.transform_y, b.transform_yaw))
        self.assertEqual(b.dead_reckoning_budget.started_at, 102)
        self.assertFalse(b.data_quality(self.now, True)[0])

    def test_high_quality_still_uses_high_precision_initialization(self):
        for enabled in (False, True):
            b = self.bridge(enabled)
            for t in (100, 101, 102):
                self.sample(b, t, quality=4, covariance=.01)
                b.update_guard(self.now)
            self.assertEqual(b.initialization_precision, 'high')
            self.assertEqual(b.mode, b.TRACKING)
            self.assertIsNone(b.dead_reckoning_budget.started_at)

    def test_precise_recovery_restores_high_reference(self):
        b = self.bridge(True)
        self.initialize_low(b)
        for t in (103, 104, 105, 106, 107):
            self.sample(b, t, quality=4, covariance=.01)
            b.update_guard(self.now)
        self.assertEqual(b.initialization_precision, 'low')
        self.assertEqual(b.global_reference_precision, 'high')
        self.assertEqual(b.mode, b.TRACKING)
        self.assertIsNone(b.dead_reckoning_budget.started_at)

    def test_fixed_position_jitter_can_use_explicit_coarse_mode(self):
        for enabled in (False, True):
            b = self.bridge(enabled)
            for t, x in ((100, 9), (101, 11), (102, 9)):
                self.sample(b, t, quality=4, covariance=.01, x=x)
                b.update_guard(self.now)
            self.assertEqual(b.transform_initialized, enabled)
            if enabled:
                self.assertEqual(b.initialization_precision, 'low')

    def test_fault_clears_startup_window(self):
        b = self.bridge(True)
        self.sample(b, 100)
        b.update_guard(self.now)
        self.sample(b, 101, heading=False)
        b.update_guard(self.now)
        self.assertFalse(b.recovery_candidates)
        for t in (102, 103):
            self.sample(b, t)
            b.update_guard(self.now)
        self.assertFalse(b.transform_initialized)
        self.sample(b, 104)
        b.update_guard(self.now)
        self.assertTrue(b.transform_initialized)

    def test_continuation_time_and_distance_limits_remain(self):
        for distance in (False, True):
            b = self.bridge(True)
            self.initialize_low(b)
            if distance:
                b.dead_reckoning_budget.observe((101, 0))
            else:
                self.sample(b, 702)
            self.assertFalse(b.localization_is_safe(self.now))

    def test_invalid_parameter_combination_fails_explicitly(self):
        self.params['~require_heading_fixed'] = False
        with self.assertRaises(ValueError):
            self.bridge(True)


if __name__ == '__main__':
    unittest.main()
