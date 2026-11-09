"""Configured 1 Hz PVT / 20 Hz GGA regression; no master, devices or live output."""
import unittest
import json
from pathlib import Path
import runpy
from unittest.mock import Mock, patch
import test_gnss_measurement_contract as pipeline
from test_gnss_auto_start import gga
from test_gnss_observation_time import NS, ObservationTiming, pvts, stamp, nmea


class CadenceClockTests(unittest.TestCase):
    def test_recorded_nov6_time_fields_now_qualify_without_explicit_verification(self):
        root = Path(__file__).resolve().parents[3]
        evidence = root/'tracker_logs/review_20261106/startup_diagnosis'
        sentence = runpy.run_path(str(evidence/'replay_clock.py'))['sentence']
        clock = ObservationTiming('measurement', 'auto', lambda: True,
                                  pvt_period_sec=1., gga_period_sec=.05)
        rows = json.loads((evidence/'system_run_full_20261106_154051_time_fixture.json').read_text())
        for row in rows:
            clock.clock_probe = lambda row=row: row.get('host_clock_synchronized') is True
            clock.observe(sentence(row), row['host_read_ros_ns'], row['host_read_mono_ns'])
        self.assertTrue(clock.clock_verified, clock.summary())

    def test_one_hz_fine_and_twenty_hz_gga_qualify_automatically(self):
        clock = ObservationTiming('measurement', 'auto', lambda: True,
                                  pvt_period_sec=1., gga_period_sec=.05)
        epoch = stamp()
        for tick in range(101):
            delta = tick*50000000
            now, mono = epoch+delta+240000000, NS+delta
            if tick % 20 == 0:
                clock.observe(pvts(epoch+delta), now, mono)
            clock.observe(gga(epoch+delta), now, mono)
            if tick < 40:
                self.assertFalse(clock.clock_verified)
        self.assertTrue(clock.clock_verified)

    def test_low_rate_duplicates_and_real_gap_cannot_qualify(self):
        for duplicate in (False, True):
            clock = ObservationTiming('measurement', 'auto', lambda: True,
                                      pvt_period_sec=1., gga_period_sec=.05)
            epoch = stamp()
            for tick in range(141):
                delta = tick*50000000
                now, mono = epoch+delta+240000000, NS+delta
                if tick % 40 == 0:
                    clock.observe(pvts(epoch if duplicate else epoch+delta), now, mono)
                clock.observe(gga(epoch+delta), now, mono)
            self.assertFalse(clock.clock_verified)


@unittest.skipUnless(pipeline.ROS, 'ROS1 fake callbacks required')
class CadencePipelineTests(unittest.TestCase):
    fix = pipeline.MeasurementPipelineTests.fix

    def setUp(self):
        pipeline.MeasurementPipelineTests.setUp(self)
        import test_runtime_position as fixture
        from gps_fix_to_map_odom import GpsFixToMapOdom
        self.params.update({'~pvt_period_sec': 1., '~gga_period_sec': .05,
                            '~gps_time_clock_verified': 'auto'})
        with patch.object(pipeline.rospy, 'Publisher', side_effect=lambda *a, **k: Mock()):
            self.b = fixture.RuntimePositionTests.bridge(self, initialized=False)
        self.driver.observation_timing = ObservationTiming('measurement', 'auto', lambda: True,
                                                         pvt_period_sec=1., gga_period_sec=.05)
        with patch.object(pipeline.rospy, 'Publisher', side_effect=lambda *a, **k: Mock()):
            self.projection = GpsFixToMapOdom()
        from map_odom_align_bridge import MapOdomAlignBridge
        with patch('tf.TransformBroadcaster', return_value=Mock()):
            self.align = MapOdomAlignBridge()
        self.b.pub.publish.side_effect = self.align._global_cb
        self.projection.pub.publish.side_effect = self.b.gps_odom_cb
        def fix(message):
            self.b.fix_cb(message)
            self.projection.fix_cb(message)
        def metadata(message):
            self.b.time_metadata_cb(message)
            self.projection.time_metadata_cb(message)
        self.driver.fix_pub.publish.side_effect = fix
        self.driver.gpgga_pub.publish.side_effect = self.b.gga_cb
        self.driver.heading_pub.publish.side_effect = self.b.heading_cb
        self.driver.solution_pub.publish.side_effect = self.b.solution_cb
        self.driver.time_diagnostics_pub.publish.side_effect = metadata

    def drive(self, end=7., float_after=None, stop_pvt_after=None, duplicate=False):
        rospy = pipeline.rospy
        for tick in range(int(end*50)+1):
            t = tick*.02
            self.now = rospy.Time.from_sec(self.epoch/NS+t)
            self.mono = NS+round(t*NS)
            local = pipeline.Odometry(); local.header.stamp = self.now
            local.pose.pose.orientation.w = 1.
            self.b.local_odom_cb(local)
            self.align._local_cb(local)
            if tick >= 12 and (tick-12) % 50 == 0:
                source = self.epoch if duplicate else self.epoch+(tick-12)*20000000
                if stop_pvt_after is None or t <= stop_pvt_after:
                    self.driver.add_sentence(pvts(source, heading='NARROW_FLOAT' if float_after is not None and t >= float_after else 'NARROW_INT',
                                                 position='NARROW_INT'), 'gps',
                        host_read_ros_ns=self.now.to_nsec(), host_read_mono_ns=self.mono)
            if tick >= 8 and tick % 5 in (0, 2):
                source = self.now.to_nsec()-150000000
                body = gga(source).split('*')[0][1:]
                self.driver.add_sentence(nmea(body+',2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'), 'gps',
                    host_read_ros_ns=self.now.to_nsec(), host_read_mono_ns=self.mono)
            self.projection.timer_cb(None)
            self.b.update_guard(self.now)
            self.b.publish_global_odom(self.now)
            self.align._timer_cb(None)

    def test_real_driver_projection_bridge_initialize_automatically(self):
        self.drive()
        self.assertTrue(self.b.gnss_timebase.clock_ready)
        self.assertTrue(self.b.transform_initialized, self.b.reason)
        self.assertEqual(self.b.mode, self.b.TRACKING, self.b.reason)
        self.assertGreaterEqual(self.driver.heading_pub.publish.call_count, 3)
        self.assertTrue(self.align.br.sendTransform.called)
        self.assertEqual(self.align.br.sendTransform.call_args.args[-2:], ('odom', 'map'))

    def test_float_before_stable_window_cannot_initialize(self):
        self.drive(float_after=3.)
        self.assertFalse(self.b.transform_initialized)

    def test_single_cached_pvt_cannot_initialize(self):
        self.drive(duplicate=True)
        self.assertFalse(self.b.transform_initialized)

    def test_turning_heading_uses_measurement_pose_not_current_pose(self):
        from geometry_msgs.msg import QuaternionStamped
        from tf.transformations import quaternion_from_euler
        rospy = pipeline.rospy
        b = self.b
        for tick in range(115):
            t = tick*.02
            self.now = rospy.Time.from_sec(self.epoch/NS+t)
            local = pipeline.Odometry(); local.header.stamp = self.now
            q = quaternion_from_euler(0, 0, .3*t)
            local.pose.pose.orientation.x, local.pose.pose.orientation.y, local.pose.pose.orientation.z, local.pose.pose.orientation.w = q
            b.local_odom_cb(local)
            if tick >= 12 and (tick-12) % 50 == 0:
                source_t = t-.24
                message = QuaternionStamped()
                message.header.stamp = rospy.Time.from_sec(self.epoch/NS+source_t)
                q = quaternion_from_euler(0, 0, .3*source_t+.4)
                message.quaternion.x, message.quaternion.y, message.quaternion.z, message.quaternion.w = q
                b.heading_cb(message)
                self.assertLessEqual(b.heading_bad_until, self.now)
                # Same-epoch local angle gives .4 rad, using current pose would give .328.
                historical, _ = b.local_pose_at(message.header.stamp.to_nsec())
                self.assertAlmostEqual(b.normalize_angle(b.last_heading_yaw-historical[2]), .4, places=5)

    def test_late_new_packet_is_not_excused_by_one_second_cadence(self):
        self.drive()
        gate = self.b.gnss_timebase
        old = gate.latest['PVTSLNA']
        source = old['measurement_stamp_ns']
        event = dict(old, host_read_ros_ns=source+700000000)
        gate.metadata['PVTSLNA', source] = event
        good, reason = gate.check('PVTSLNA', source, self.now.to_nsec(), self.mono, .5, 1., .3)
        self.assertFalse(good)
        self.assertEqual(reason, 'measurement_transport_late_PVTSLNA')

    def test_true_stream_loss_keeps_existing_continuation_policy(self):
        self.drive(end=10., stop_pvt_after=7.)
        self.assertTrue(self.b.transform_initialized)
        self.assertNotEqual(self.b.mode, self.b.TRACKING)

    def test_new_float_interrupts_fixed_tracking_without_relabeling_old_heading(self):
        self.drive(end=10., float_after=7.)
        self.assertTrue(self.b.transform_initialized)
        self.assertNotEqual(self.b.mode, self.b.TRACKING)
        self.assertFalse(self.b.latest_heading_health(self.now)['valid'])
        self.assertEqual(self.b.last_solution['heading_type'], 'NARROW_FLOAT')


@unittest.skipUnless(pipeline.ROS, 'ROS1 launch parser required')
class CadenceLaunchTests(unittest.TestCase):
    def test_documented_ordinary_and_camera_entries_share_localization_parameters(self):
        import roslaunch
        root=Path(__file__).resolve().parents[3]
        path=root/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch'
        configs=[]
        for extra in ([],['enable_camera_right_path:=true','enable_camera_right_control:=true']):
            config=roslaunch.config.load_config_default([(str(path),[
                'outdoor_mode:=true','low_precision_startup:=true']+extra)],None,verbose=False)
            configs.append({key:value.value for key,value in config.params.items()
                if key.startswith(('/global_odom_bridge/','/gps_fix_to_map_odom/','/nmea_serial_driver_node/'))})
        self.assertTrue(configs[0]['/global_odom_bridge/low_precision_startup'])
        self.assertEqual(configs[0],configs[1])
        commands=(root/'核心启动指令.md').read_text()
        self.assertEqual(commands.count('outdoor_mode:=true low_precision_startup:=true'),2)

    def test_full_system_passes_configured_periods_to_both_driver_and_bridge(self):
        import roslaunch
        root = Path(__file__).resolve().parents[3]
        path = root/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch'
        for period in (1., .1):
            config = roslaunch.config.load_config_default([(str(path), ['outdoor_mode:=true',
                'gps_pvt_period_sec:='+str(period), 'gps_gga_period_sec:=0.05'])], None, verbose=False)
            for node in ('nmea_serial_driver_node', 'global_odom_bridge'):
                self.assertEqual(config.params['/'+node+'/pvt_period_sec'].value, period)
                self.assertEqual(config.params['/'+node+'/gga_period_sec'].value, .05)
                self.assertEqual(config.params['/'+node+'/cadence_margin_sec'].value, .3)


if __name__ == '__main__':
    unittest.main()
