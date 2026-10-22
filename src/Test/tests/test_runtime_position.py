"""Non-Fixed runtime GPS tests; fake messages only, never starts vehicle nodes."""
import json
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'src/robot_base_driver/scripts'))
import rospy
from global_odom_bridge import GlobalOdomBridge, RuntimePositionWindow
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Gpgga


class RuntimePositionTests(unittest.TestCase):
    def setUp(self):
        self.now = rospy.Time(100)
        self.params = {'~allow_nonfixed_runtime': True}
        for name in ('init_node', 'Publisher', 'Subscriber', 'Timer', 'logwarn', 'logwarn_throttle'):
            stub = patch.object(rospy, name)
            stub.start()
            self.addCleanup(stub.stop)
        for stub in (patch.object(rospy, 'get_param', lambda key, default=None: self.params.get(key, default)),
                     patch.object(rospy.Time, 'now', lambda: self.now)):
            stub.start()
            self.addCleanup(stub.stop)

    def bridge(self, initialized=True):
        b = GlobalOdomBridge()
        b.transform_initialized = initialized
        b.transform_x, b.transform_y, b.transform_yaw = 100., 20., .3
        b.initialization_precision = b.global_reference_precision = 'low'
        b.mode = b.DEAD_RECKONING if initialized else b.WAITING
        if initialized:
            b.dead_reckoning_budget.begin(100., (0., 0.))
        return b

    def sample(self, b, t, quality=5, covariance=1., bias=0., local_x=None):
        self.now = rospy.Time.from_sec(t)
        local_x = (t-100.)*.8 if local_x is None else local_x
        local = Odometry()
        local.header.stamp = self.now
        local.pose.pose.position.x = local_x
        local.pose.pose.orientation.w = 1.
        b.last_local_odom, b.last_local_recv = local, self.now
        b.dead_reckoning_budget.observe((local_x, 0.))
        gps = Odometry()
        gps.header.stamp = self.now
        gps.pose.pose.position.x = 100.+math.cos(.3)*local_x+bias
        gps.pose.pose.position.y = 20.+math.sin(.3)*local_x
        b.gps_odom_cb(gps)
        fix = NavSatFix()
        fix.header.stamp = self.now
        fix.latitude, fix.longitude = 23.04, 113.39
        fix.status.status = 2 if quality in (4, 5) else 1 if quality == 2 else 0 if quality == 1 else -1
        fix.position_covariance[0] = fix.position_covariance[4] = covariance
        fix.position_covariance_type = 1
        b.fix_cb(fix)
        gga = Gpgga()
        gga.header.stamp = self.now
        gga.gps_qual, gga.num_sats, gga.hdop = quality, 12, 1.2
        b.gga_cb(gga)
        # No valid or fixed GNSS heading throughout non-Fixed runtime tests.
        b.last_solution = {'stamp': self.now.to_sec(), 'stamp_ns': self.now.to_nsec(), 'heading_fixed': False, 'heading_valid': False}
        b.last_solution_recv = self.now
        # These synchronous fixtures deliberately mutate latest messages after
        # callbacks to test rejection paths. Buffer/arrival-order tests use
        # real immutable callbacks separately in test_global_alignment.py.
        for history in b.observation_history.values():
            history.clear()
        return local_x

    def drive(self, b, until=106., quality=5, covariance=1., bias=0., start=100.):
        for i in range(int(round((until-start)*5))+1):
            t = start+i*.2
            self.sample(b, t, quality, covariance, bias)
            b.update_guard(self.now)

    def test_nonfixed_positions_can_continue_for_500_m_without_fixed(self):
        for quality in (5, 2):
            with self.subTest(quality=quality):
                b = self.bridge()
                for i in range(3126):
                    self.sample(b, 100.+i*.2, quality=quality, bias=.05*math.sin(i*.02))
                    b.update_guard(self.now)
                    self.assertTrue(b.localization_is_safe(self.now))
                self.assertEqual(b.mode, b.POSITION_TRACKING)
                self.assertGreater(b.runtime_position_credit_count, 100)
                self.assertLess(b.dead_reckoning_budget.distance_m, 5.)
                self.assertAlmostEqual(b.transform_yaw, .3)
                self.assertNotEqual(b.global_reference_precision, 'high')

    def test_single_packet_and_repeated_source_do_not_renew_budget(self):
        b = self.bridge()
        self.sample(b, 100.)
        b.update_guard(self.now)
        for i in range(1, 60):
            self.now = rospy.Time.from_sec(100.+i*.1)
            b.last_fix_recv = b.last_gps_recv = b.last_gga_recv = b.last_local_recv = self.now
            b.update_guard(self.now)
        self.assertEqual(b.runtime_position_credit_count, 0)
        self.assertEqual(b.dead_reckoning_budget.started_at, 100.)

    def test_coarse_correction_does_not_claim_fixed_accuracy_or_rotate_yaw(self):
        b = self.bridge()
        self.drive(b, bias=.3)
        self.assertEqual(b.mode, b.POSITION_TRACKING)
        self.assertEqual(b.global_reference_precision, 'runtime_float')
        self.assertEqual(b.initialization_precision, 'low')
        self.assertAlmostEqual(b.transform_yaw, .3)
        self.assertGreater(b.runtime_position_credit_count, 0)
        self.assertLessEqual(b.runtime_position_credit_count, 1)

    def test_large_but_bounded_error_is_corrected_slowly_before_credit(self):
        b = self.bridge()
        previous = b.transform_x
        first_credit_time = None
        for i in range(160):
            t = 100.+i*.2
            self.sample(b, t, bias=2.)
            b.update_guard(self.now)
            self.assertLessEqual(abs(b.transform_x-previous), .1*.2+1.e-8)
            self.assertAlmostEqual(b.transform_yaw, .3)
            previous = b.transform_x
            if b.runtime_position_credit_count and first_credit_time is None:
                first_credit_time = t
                self.assertLessEqual(b.runtime_position_residual_m, .5)
        self.assertIsNotNone(first_credit_time)
        self.assertGreaterEqual(first_credit_time, 120.-1.e-6)

    def test_implausible_innovation_cannot_reset_exhausted_budget(self):
        b = self.bridge()
        b.dead_reckoning_budget.distance_m = 101.
        for i in range(35):
            self.sample(b, 100.+i*.2, bias=3.1)
            b.update_guard(self.now)
        self.assertEqual(b.runtime_position_credit_count, 0)
        self.assertFalse(b.localization_is_safe(self.now))
        self.assertEqual(b.runtime_position_reason, 'position_innovation_too_large')
        self.assertEqual(b.transform_x, 100.)

    def test_qualified_window_can_recover_after_a_budget_stop_without_fixed(self):
        b = self.bridge()
        b.dead_reckoning_budget.distance_m = 101.
        self.assertFalse(b.localization_is_safe(self.now))
        self.drive(b)
        self.assertGreater(b.runtime_position_credit_count, 0)
        self.assertTrue(b.localization_is_safe(self.now))

    def test_no_fix_single_or_unqualified_position_still_stops_at_100_m(self):
        for quality, covariance in ((0, 1.), (1, 1.), (5, 10.), (2, 10.)):
            with self.subTest(quality=quality, covariance=covariance):
                b = self.bridge()
                for i in range(130):
                    self.sample(b, 100.+i, quality=quality, covariance=covariance)
                    b.update_guard(self.now)
                self.assertEqual(b.runtime_position_credit_count, 0)
                self.assertFalse(b.localization_is_safe(self.now))
                self.assertEqual(b.dead_reckoning_budget.limit_reason, 'dead_reckoning_distance_limit')

    def test_outdoor_200m_override_keeps_distance_and_600s_limits(self):
        self.params['~dead_reckoning_max_distance_m'] = 200.
        b = self.bridge()
        budget = b.dead_reckoning_budget
        budget.observe((150., 0.))
        self.assertIsNone(budget.check(300.))
        budget.observe((200., 0.))
        self.assertEqual(budget.check(350.), 'dead_reckoning_distance_limit')
        self.assertEqual(budget.max_duration_sec, 600.)
        b = self.bridge()
        self.assertEqual(b.dead_reckoning_budget.check(700.), 'dead_reckoning_time_limit')

    def test_full_system_200m_override_does_not_change_standalone_default(self):
        root = Path(__file__).resolve().parents[3]
        full = ET.parse(root/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch').getroot()
        core = ET.parse(root/'src/robot_base_driver/launch/navigation_core.launch').getroot()
        name = 'gps_guard_dead_reckoning_max_distance_m'
        self.assertEqual(float(full.find("arg[@name='%s']" % name).get('default')), 200.)
        self.assertEqual(float(core.find("arg[@name='%s']" % name).get('default')), 100.)

    def test_dropout_after_nonfixed_credit_preserves_distance_and_time_limits(self):
        for distance in (True, False):
            b = self.bridge()
            self.drive(b)
            previous = b.runtime_position_credit_count
            last_credit = b.dead_reckoning_budget.started_at
            self.sample(b, 107., quality=0)
            b.update_guard(self.now)
            if distance:
                b.dead_reckoning_budget.distance_m = 100.1
            else:
                self.now = rospy.Time.from_sec(last_credit+601.)
                b.last_local_recv = self.now
            self.assertFalse(b.localization_is_safe(self.now))
            self.assertEqual(b.runtime_position_credit_count, previous)

    def test_bad_covariance_hdop_satellites_and_unknown_covariance_are_rejected(self):
        for field in ('covariance', 'hdop', 'satellites', 'unknown_covariance', 'nan_covariance'):
            b = self.bridge()
            self.sample(b, 100.)
            if field=='covariance':b.last_fix.position_covariance[0] = 4.1
            elif field=='nan_covariance':b.last_fix.position_covariance[0] = float('nan')
            elif field=='hdop':b.last_gga.hdop = 2.6
            elif field=='satellites':b.last_gga.num_sats = 7
            else:b.last_fix.position_covariance_type = 0
            self.assertFalse(b.data_quality(self.now, runtime_position=True)[0], field)

    def test_source_timestamps_future_old_or_mismatched_cannot_credit(self):
        for field in ('future', 'old', 'projection', 'local', 'gga'):
            b = self.bridge()
            self.sample(b, 100.)
            if field=='future':b.last_fix.header.stamp = rospy.Time(101)
            elif field=='old':b.last_fix.header.stamp = rospy.Time(98)
            elif field=='projection':b.last_gps_odom.header.stamp = rospy.Time.from_sec(99.8)
            elif field=='local':b.last_local_odom.header.stamp = rospy.Time.from_sec(99.6)
            else:b.last_gga.header.stamp = rospy.Time(98)
            self.assertFalse(b.data_quality(self.now, runtime_position=True)[0], field)
            b.update_guard(self.now)
            self.assertEqual(b.runtime_position_credit_count, 0)

    def test_callback_pair_wait_does_not_starve_a_good_window(self):
        b = self.bridge()
        previous_stamp = rospy.Time.from_sec(99.8)
        for i in range(35):
            t = 100.+i*.2
            self.sample(b, t)
            b.last_gps_odom.header.stamp = previous_stamp
            b.update_guard(self.now)
            b.last_gps_odom.header.stamp = self.now
            b.update_guard(self.now)
            previous_stamp = self.now
        self.assertGreater(b.runtime_position_credit_count, 0)

    def test_jump_or_dropout_requires_another_full_stability_window(self):
        for dropout in (True, False):
            b = self.bridge()
            self.drive(b, until=104.)
            if dropout:
                self.drive(b, until=111., start=110.)
            else:
                self.drive(b, until=106., start=105., bias=1.5)
            self.assertEqual(b.runtime_position_credit_count, 0)

    def test_lio_staleness_and_fault_still_stop(self):
        for fault in (False, True):
            b = self.bridge()
            self.drive(b)
            if fault:b.local_bad_until = self.now+rospy.Duration(2.)
            else:b.last_local_recv = self.now-rospy.Duration(2.)
            count = b.runtime_position_credit_count
            b.update_guard(self.now)
            self.assertFalse(b.localization_is_safe(self.now))
            self.assertEqual(b.runtime_position_credit_count, count)

    def test_fixed_position_without_good_heading_remains_position_only(self):
        b = self.bridge()
        self.drive(b, quality=4, covariance=.01)
        self.assertEqual(b.mode, b.POSITION_TRACKING)
        self.assertEqual(b.global_reference_precision, 'runtime_fixed_position_only')
        self.assertAlmostEqual(b.transform_yaw, .3)

    def test_full_fixed_recovery_uses_original_heading_recovery_window(self):
        b = self.bridge()
        self.drive(b)
        for i in range(31):
            self.sample(b, 107.+i*.2, quality=4, covariance=.01)
            b.last_solution = {'stamp': self.now.to_sec(), 'stamp_ns': self.now.to_nsec(), 'heading_fixed': True, 'heading_valid': True}
            b.last_heading_yaw, b.last_heading_recv = .3, self.now
            b.last_heading_stamp = self.now
            b.update_guard(self.now)
        self.assertEqual(b.mode, b.TRACKING)
        self.assertEqual(b.global_reference_precision, 'high')
        self.assertIsNone(b.dead_reckoning_budget.started_at)

    def test_runtime_does_not_initialize_without_fixed_heading(self):
        self.params['~low_precision_startup'] = True
        b = self.bridge(initialized=False)
        self.drive(b)
        self.assertFalse(b.transform_initialized)
        self.assertEqual(b.runtime_position_credit_count, 0)

    def test_disabled_runtime_preserves_original_fixed_only_policy(self):
        self.params['~allow_nonfixed_runtime'] = False
        b = self.bridge()
        self.drive(b)
        self.assertEqual(b.mode, b.DEAD_RECKONING)
        self.assertEqual(b.runtime_position_credit_count, 0)

    def test_status_exposes_coarse_precision_and_budget_credits(self):
        b = self.bridge()
        self.drive(b)
        b.publish_status(self.now)
        payload = json.loads(b.status_pub.publish.call_args[0][0].data)
        self.assertEqual(payload['mode'], 'GPS_POSITION_TRACKING')
        self.assertEqual(payload['global_reference_precision'], 'runtime_float')
        self.assertTrue(payload['yaw_frozen'])
        self.assertTrue(payload['runtime_position']['active'])
        self.assertGreater(payload['runtime_position']['budget_renewals'], 0)

    def test_invalid_runtime_parameters_fail_explicitly(self):
        for key, value in (('~runtime_position_max_covariance', float('nan')),
                           ('~runtime_position_stable_sec', 0.), ('~runtime_position_correction_rate', -1.)):
            self.params[key] = value
            with self.assertRaises(ValueError):self.bridge()
            del self.params[key]
        self.params['~guard_enabled'] = False
        with self.assertRaises(ValueError):self.bridge()

    def test_launch_enables_only_neupan_policy_by_default(self):
        root = Path(__file__).resolve().parents[3]
        full = ET.parse(root/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch').getroot()
        nav = ET.parse(root/'src/robot_base_driver/launch/navigation_core.launch').getroot()
        self.assertEqual(full.find("arg[@name='gps_guard_allow_nonfixed_runtime']").get('default'), 'true')
        self.assertEqual(nav.find("arg[@name='gps_guard_allow_nonfixed_runtime']").get('default'), 'false')


class SourceTimeAndTransitionTests(unittest.TestCase):
    setUp = RuntimePositionTests.setUp
    bridge = RuntimePositionTests.bridge
    sample = RuntimePositionTests.sample

    def fixed_sample(self, b, t, local_x=None):
        self.sample(b, t, quality=4, covariance=.01, local_x=local_x)
        b.last_solution = {'stamp': self.now.to_sec(), 'stamp_ns': self.now.to_nsec(), 'heading_fixed': True, 'heading_valid': True}
        b.last_heading_yaw, b.last_heading_recv, b.last_heading_stamp = .3, self.now, self.now

    def tracking_bridge(self):
        b = self.bridge()
        self.fixed_sample(b, 100.)
        b.set_mode(b.TRACKING, 'fixture_fixed')
        b.update_guard(self.now)
        self.assertIsNone(b.dead_reckoning_budget.started_at)
        return b

    def test_fixed_to_pairing_wait_starts_budget_before_early_return(self):
        b = self.tracking_bridge()
        self.sample(b, 101.)
        b.last_gps_odom.header.stamp = rospy.Time.from_sec(100.8)
        b.update_guard(self.now)
        self.assertEqual(b.mode, b.DEAD_RECKONING)
        self.assertEqual(b.dead_reckoning_budget.started_at, 101.)
        self.assertEqual(b.runtime_position_credit_count, 0)

    def test_short_pairing_wait_does_not_starve_startup_window(self):
        for coarse in (False, True):
            with self.subTest(coarse=coarse):
                self.params['~low_precision_startup'] = coarse
                b = self.bridge(initialized=False)
                previous_stamp = rospy.Time.from_sec(99.8)
                for i in range(15):
                    self.fixed_sample(b, 100.+i*.2, local_x=0.)
                    if coarse:
                        b.last_fix.position_covariance[0] = b.last_fix.position_covariance[4] = 16.
                    b.last_gps_odom.header.stamp = previous_stamp
                    b.update_guard(self.now)
                    b.last_gps_odom.header.stamp = self.now
                    b.update_guard(self.now)
                    previous_stamp = self.now
                    if b.transform_initialized:
                        break
                self.assertTrue(b.transform_initialized)
                self.assertEqual(b.initialization_precision, 'low' if coarse else 'high')

    def test_short_pairing_wait_does_not_starve_fixed_recovery_window(self):
        for enabled in (False, True):
            with self.subTest(runtime_enabled=enabled):
                self.params['~allow_nonfixed_runtime'] = enabled
                b = self.bridge()
                previous_stamp = rospy.Time.from_sec(99.8)
                for i in range(25):
                    self.fixed_sample(b, 100.+i*.2, local_x=0.)
                    b.last_gps_odom.header.stamp = previous_stamp
                    b.update_guard(self.now)
                    b.last_gps_odom.header.stamp = self.now
                    b.update_guard(self.now)
                    previous_stamp = self.now
                    if b.mode == b.TRACKING:
                        break
                self.assertEqual(b.mode, b.TRACKING)
                self.assertIsNone(b.dead_reckoning_budget.started_at)

    def test_persistent_pairing_wait_still_expires_after_100_m(self):
        b = self.tracking_bridge()
        for i in range(651):
            t = 101.+i*.2
            self.sample(b, t)
            b.last_gps_odom.header.stamp = rospy.Time.from_sec(t-.2)
            b.update_guard(self.now)
        self.assertFalse(b.localization_is_safe(self.now))
        self.assertEqual(b.dead_reckoning_budget.started_at, 101.)
        self.assertEqual(b.dead_reckoning_budget.limit_reason, 'dead_reckoning_distance_limit')

    def test_stationary_pairing_wait_still_expires_after_600_seconds(self):
        b = self.tracking_bridge()
        for t in (101., 102., 500., 702.):
            self.sample(b, t, local_x=0.)
            b.last_gps_odom.header.stamp = rospy.Time.from_sec(t-.2)
            b.update_guard(self.now)
        self.assertFalse(b.localization_is_safe(self.now))
        self.assertEqual(b.dead_reckoning_budget.started_at, 101.)
        self.assertEqual(b.dead_reckoning_budget.limit_reason, 'dead_reckoning_time_limit')

    def test_existing_position_budget_is_not_restarted_by_pairing_wait(self):
        b = self.bridge()
        b.mode = b.POSITION_TRACKING
        for t in (101., 102., 103.):
            self.sample(b, t)
            b.last_gps_odom.header.stamp = rospy.Time.from_sec(t-.2)
            b.update_guard(self.now)
        self.assertEqual(b.dead_reckoning_budget.started_at, 100.)
        self.assertGreater(b.dead_reckoning_budget.distance_m, 2.)

    def test_fixed_policy_rejects_stale_sources_despite_fresh_receipts(self):
        for field in ('fix', 'projection', 'gga', 'local'):
            with self.subTest(field=field):
                b = self.tracking_bridge()
                self.fixed_sample(b, 102.)
                messages = {'fix':b.last_fix, 'projection':b.last_gps_odom,
                            'gga':b.last_gga, 'local':b.last_local_odom}
                messages[field].header.stamp = rospy.Time(100)
                self.assertFalse(b.data_quality(self.now)[0])
                b.update_guard(self.now)
                self.assertEqual(b.mode, b.DEAD_RECKONING)
                self.assertIsNotNone(b.dead_reckoning_budget.started_at)

    def test_fixed_policy_rejects_future_or_zero_source_stamps(self):
        for stamp in (rospy.Time(0), rospy.Time(103)):
            b = self.bridge()
            self.fixed_sample(b, 101.)
            b.last_fix.header.stamp = stamp
            self.assertFalse(b.data_quality(self.now)[0])

    def test_fixed_policy_checks_pairing_without_runtime_enabled(self):
        self.params['~allow_nonfixed_runtime'] = False
        b = self.tracking_bridge()
        self.fixed_sample(b, 101.)
        b.last_gps_odom.header.stamp = rospy.Time.from_sec(100.8)
        b.update_guard(self.now)
        self.assertEqual(b.mode, b.TRACKING)
        self.assertIsNone(b.dead_reckoning_budget.started_at)
        self.fixed_sample(b, 101.6)
        b.last_gps_odom.header.stamp = rospy.Time.from_sec(101.4)
        b.update_guard(self.now)
        self.assertEqual(b.mode, b.DEAD_RECKONING)
        self.assertEqual(b.dead_reckoning_budget.started_at, 101.)

    def test_republished_fix_cannot_reapply_correction_or_reset_budget(self):
        b = self.tracking_bridge()
        b.transform_x += .1
        b.dead_reckoning_budget.begin(100., (0.,0.))
        b.dead_reckoning_budget.distance_m = 99.
        previous = (b.transform_x,b.transform_y,b.transform_yaw)
        for i in range(1, 20):
            self.now = rospy.Time.from_sec(100.+i*.04)
            b.last_fix_recv = b.last_gps_recv = b.last_gga_recv = b.last_local_recv = self.now
            b.last_heading_recv = b.last_solution_recv = self.now
            b.update_guard(self.now)
        self.assertEqual(previous,(b.transform_x,b.transform_y,b.transform_yaw))
        self.assertEqual(b.dead_reckoning_budget.distance_m,99.)
        self.assertEqual(b.dead_reckoning_budget.started_at,100.)

    def test_repeated_source_does_not_complete_initialization(self):
        self.params['~initial_stable_sec'] = .5
        b = self.bridge(initialized=False)
        self.fixed_sample(b, 100.)
        for i in range(20):
            self.now = rospy.Time.from_sec(100.+i*.04)
            b.last_fix_recv = b.last_gps_recv = b.last_gga_recv = b.last_local_recv = self.now
            b.last_heading_recv = b.last_solution_recv = self.now
            b.update_guard(self.now)
        self.assertFalse(b.transform_initialized)

    def test_repeated_source_does_not_complete_fixed_recovery(self):
        self.params['~recovery_stable_sec'] = .5
        b = self.bridge()
        self.fixed_sample(b,100.)
        for i in range(20):
            self.now = rospy.Time.from_sec(100.+i*.04)
            b.last_fix_recv = b.last_gps_recv = b.last_gga_recv = b.last_local_recv = self.now
            b.last_heading_recv = b.last_solution_recv = self.now
            b.update_guard(self.now)
        self.assertNotEqual(b.mode,b.TRACKING)
        self.assertEqual(b.dead_reckoning_budget.started_at,100.)

    def test_out_of_order_fix_and_gga_do_not_count_as_new_observations(self):
        for field in ('fix', 'gga'):
            b = self.tracking_bridge()
            self.fixed_sample(b,100.2)
            b.update_guard(self.now)
            self.fixed_sample(b,100.3)
            if field=='fix':
                b.last_fix.header.stamp = b.last_gps_odom.header.stamp = rospy.Time.from_sec(100.1)
            else:b.last_gga.header.stamp = rospy.Time.from_sec(100.1)
            self.assertFalse(b.data_quality(self.now)[0],field)

    def test_fixed_heading_requires_source_time_not_just_recent_receipt(self):
        for field in ('heading', 'solution'):
            b = self.bridge()
            self.fixed_sample(b,102.)
            if field=='heading':b.last_heading_stamp = rospy.Time(100)
            else:b.last_solution['stamp'] = 99.
            self.assertFalse(b.data_quality(self.now)[0],field)
            self.assertTrue(b.data_quality(self.now,runtime_position=True)[0])

    def test_malformed_or_future_solution_source_is_rejected(self):
        for stamp in (None,'bad',float('nan'),1.e308,103.):
            b = self.bridge()
            self.fixed_sample(b,101.)
            b.last_solution['stamp'] = stamp
            self.assertFalse(b.data_quality(self.now)[0])

    def test_old_heading_source_cannot_replace_newer_valid_heading(self):
        b = self.tracking_bridge()
        self.fixed_sample(b,100.2)
        b.update_guard(self.now)
        self.fixed_sample(b,100.3)
        b.last_heading_stamp = rospy.Time.from_sec(100.1)
        self.assertFalse(b.data_quality(self.now)[0])

    def test_invalid_future_data_does_not_poison_later_valid_source(self):
        b = self.tracking_bridge()
        self.fixed_sample(b,100.2)
        b.last_fix.header.stamp = b.last_gps_odom.header.stamp = rospy.Time(110)
        self.assertFalse(b.data_quality(self.now)[0])
        self.fixed_sample(b,100.4)
        self.assertTrue(b.data_quality(self.now)[0])


@unittest.skipUnless(os.environ.get('GPS_RUNTIME_ROS_SMOKE') == '1', 'optional isolated ROS node test')
class RuntimeRosProcessTest(unittest.TestCase):
    def test_fixed_pairing_loss_nonfixed_recovery_and_stale_fixed_stop(self):
        import signal
        import socket
        import subprocess
        import tempfile
        import time
        from geometry_msgs.msg import QuaternionStamped
        from std_msgs.msg import Bool, String

        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port), ROS_HOSTNAME='127.0.0.1')
        env['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='gps_runtime_ros_')
        os.environ.update({k:env[k] for k in ('ROS_MASTER_URI', 'ROS_HOSTNAME', 'ROS_LOG_DIR')})
        master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        process = None
        try:
            rospy.init_node('gps_runtime_message_test', anonymous=True, disable_signals=True)
            prefix = '/nonfixed_test/'
            parameters = {'allow_nonfixed_runtime':'true', 'low_precision_startup':'true',
                'initial_stable_sec':'.5', 'runtime_position_stable_sec':'1.5',
                'recovery_stable_sec':'.5',
                'dead_reckoning_max_distance_m':'2.0', 'dead_reckoning_max_duration_sec':'60.',
                'input_topic':prefix+'gps', 'local_topic':prefix+'local', 'fix_topic':prefix+'fix',
                'gga_topic':prefix+'gga', 'heading_topic':prefix+'heading', 'solution_topic':prefix+'solution',
                'output_topic':prefix+'odom', 'status_topic':prefix+'status', 'safety_topic':prefix+'safe'}
            process = subprocess.Popen(['rosrun', 'robot_base_driver', 'global_odom_bridge.py',
                '__name:=nonfixed_runtime_bridge']+['_'+k+':='+v for k,v in parameters.items()],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            statuses, safety, outputs = [], [], []
            subscribers = [rospy.Subscriber(prefix+'status', String, lambda msg:statuses.append(json.loads(msg.data))),
                rospy.Subscriber(prefix+'safe', Bool, lambda msg:safety.append(msg.data)),
                rospy.Subscriber(prefix+'odom', Odometry, outputs.append)]
            pubs = {name:rospy.Publisher(prefix+name, kind, queue_size=10) for name,kind in (
                ('gps',Odometry), ('local',Odometry), ('fix',NavSatFix), ('gga',Gpgga),
                ('heading',QuaternionStamped), ('solution',String))}
            deadline = time.monotonic()+15
            while not all(p.get_num_connections() for p in pubs.values()):
                self.assertIsNone(process.poll())
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.05)
            last_fix = None
            last_emit = 0.
            motion_started = None
            last_credit = 0
            float_start_x = None
            phase = 'fixed_init'
            local_delay = float(os.environ.get('GPS_RUNTIME_LOCAL_DELAY_SEC', '0'))
            mismatch_seen = False
            fixed_transform_x = None
            deadline = time.monotonic()+40
            while time.monotonic()<deadline:
                self.assertIsNone(process.poll(), 'test node exited')
                now = rospy.Time.now()
                monotonic = time.monotonic()
                x = 0. if motion_started is None else .4*(monotonic-motion_started)
                local = Odometry()
                local.header.stamp = now-rospy.Duration(local_delay)
                local.pose.pose.position.x = max(0., x-.4*local_delay)
                local.pose.pose.orientation.w = 1.
                pubs['local'].publish(local)
                if monotonic-last_emit >= .18:
                    last_emit = monotonic
                    if phase != 'stale_fixed':
                        gps = Odometry()
                        # No matching fix exists within 50ms at this 180ms
                        # cadence. An actual one-fix delay is now supported by
                        # the buffer and must not be used to simulate failure.
                        gps.header.stamp = now-rospy.Duration(.08) if phase=='pairing_wait' else now
                        gps.pose.pose.position.x, gps.pose.pose.position.y = 100.+x, 20.
                        fix = NavSatFix()
                        fix.header.stamp = now
                        fix.latitude, fix.longitude, fix.status.status = 23.04, 113.39, 2
                        fixed = phase in ('fixed_init','fixed_recovery')
                        fix.position_covariance[0] = fix.position_covariance[4] = .01 if fixed else 1.
                        fix.position_covariance_type = 1
                        gga = Gpgga()
                        gga.header.stamp = now
                        gga.gps_qual, gga.num_sats, gga.hdop = 4 if fixed else 5, 12, 1.
                        last_fix = gps, fix, gga
                    # The last phase repeatedly sends an old FIXED fix while
                    # heading/solution receptions remain fresh. The common
                    # source-clock guard must still expire its position data.
                    if last_fix:
                        for name,message in zip(('gps','fix','gga'),last_fix):pubs[name].publish(message)
                    heading = QuaternionStamped()
                    heading.header.stamp = now
                    heading.quaternion.w = 1.
                    # Match the real Fixed-gated driver: heading Float/invalid
                    # emits solution status, but NO effective heading message.
                    if phase in ('fixed_init','fixed_recovery','stale_fixed'):
                        pubs['heading'].publish(heading)
                    pubs['solution'].publish(String(data=json.dumps({
                        'stamp': now.to_sec(),
                        'stamp_ns': now.to_nsec(),
                        'position_type':'NARROW_INT' if phase in ('fixed_init','fixed_recovery','stale_fixed') else 'NARROW_FLOAT',
                        'position_fixed':phase in ('fixed_init','fixed_recovery','stale_fixed'),
                        'heading_fixed':phase in ('fixed_init','fixed_recovery','stale_fixed'),
                        'heading_valid':phase in ('fixed_init','fixed_recovery','stale_fixed')})))
                if statuses:
                    state = statuses[-1]
                    runtime = state['runtime_position']
                    if phase=='fixed_init' and state['mode']=='GPS_TRACKING':
                        motion_started = monotonic
                        phase = 'pairing_wait'
                    elif phase=='pairing_wait':
                        mismatch_seen |= runtime['reason']=='position_source_time_mismatch'
                        if state.get('dead_reckoning_limit_reason')=='dead_reckoning_distance_limit' and safety and not safety[-1]:
                            self.assertTrue(mismatch_seen)
                            self.assertFalse(state['localization_safe'])
                            self.assertEqual(state['mode'],'LOCAL_DEAD_RECKONING')
                            self.assertEqual(runtime['budget_renewals'],0)
                            phase = 'float_resume'
                    elif phase=='float_resume' and state['mode']=='GPS_POSITION_TRACKING' and state['localization_safe']:
                        if float_start_x is None:float_start_x = x
                        if x-float_start_x>3.2:
                            self.assertGreaterEqual(runtime['budget_renewals'],2)
                            self.assertEqual(state['global_reference_precision'],'runtime_float')
                            phase = 'fixed_recovery'
                    elif phase=='fixed_recovery' and state['mode']=='GPS_TRACKING':
                        fixed_transform_x = state['transform_x']
                        last_credit = runtime['budget_renewals']
                        phase = 'stale_fixed'
                    elif phase=='stale_fixed' and state.get('dead_reckoning_limit_reason')=='dead_reckoning_distance_limit' and safety and not safety[-1]:
                        self.assertFalse(state['localization_safe'])
                        self.assertEqual(runtime['budget_renewals'], last_credit)
                        self.assertAlmostEqual(state['transform_x'],fixed_transform_x,delta=.05)
                        self.assertTrue(all(math.isfinite(msg.pose.pose.position.x) for msg in outputs))
                        return
                time.sleep(.025)
            self.fail('ROS test timeout: '+str(statuses[-1:]))
        finally:
            rospy.signal_shutdown('isolated runtime GPS test finished')
            for child in (process, master):
                if child is not None and child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
