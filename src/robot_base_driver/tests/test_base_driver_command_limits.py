"""Task 02: actual driver callbacks with a mocked serial port, no hardware."""
import math
from pathlib import Path
import random
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import rospy
import base_driver_node
from base_driver_node import RobotBaseDriver
from geometry_msgs.msg import Twist


class DriverCommandLimitsTests(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.serial = MagicMock()
        self.serial.is_open = True
        for target in ('init_node', 'Publisher', 'Subscriber', 'Timer', 'logwarn_throttle',
                       'logfatal', 'loginfo', 'logwarn'):
            stub = patch.object(rospy, target)
            stub.start()
            self.addCleanup(stub.stop)
        for stub in (
                patch.object(rospy, 'get_param', lambda k, d=None: self.params.get(k, d)),
                patch.object(rospy.Time, 'now', lambda: rospy.Time(100)),
                patch.object(base_driver_node.tf, 'TransformBroadcaster'),
                patch.object(base_driver_node.serial, 'Serial', return_value=self.serial)):
            stub.start()
            self.addCleanup(stub.stop)

    def driver(self, **params):
        self.params.update({'~'+k:v for k,v in params.items()})
        return RobotBaseDriver()

    def command(self, driver, speed, steering=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = speed, steering
        driver.cmd_vel_callback(msg)
        return self.packet()

    def packet(self):
        wire = self.serial.write.call_args.args[0]
        self.assertIsInstance(wire, bytes)
        self.assertTrue(wire.endswith(b'\n'))
        prefix, speed, steering = wire.decode().strip().split(',')
        self.assertEqual(prefix, 'CMD')
        speed, steering = float(speed), float(steering)
        self.assertTrue(math.isfinite(speed) and math.isfinite(steering), wire)
        return speed, steering

    def test_each_nonfinite_twist_field_immediately_replaces_motion_with_stop(self):
        driver = self.driver()
        for bad in (math.nan, math.inf, -math.inf):
            for speed, angle in ((bad, .2), (.8, bad), (bad, bad)):
                with self.subTest(speed=speed, angle=angle):
                    self.command(driver, .8, .1)
                    before = self.serial.write.call_count
                    self.assertEqual(self.command(driver, speed, angle), (0., 0.))
                    self.assertEqual(self.serial.write.call_count, before+1)
        rospy.logwarn_throttle.assert_called()

    def test_all_pairs_of_illegal_fields_stop(self):
        driver = self.driver()
        for speed in (math.nan, math.inf, -math.inf):
            for angle in (math.nan, math.inf, -math.inf):
                self.assertEqual(self.command(driver, speed, angle), (0., 0.))

    def test_ordinary_forward_reverse_and_units_are_preserved(self):
        driver = self.driver()
        for speed in (.05, .4, .8, 1., -.05, -.4, -.8, -1.):
            with self.subTest(speed=speed):
                wheel, angle = self.command(driver, speed, .2)
                self.assertEqual(wheel, float('%.2f' % (speed/.1275)))
                self.assertEqual(angle, float('%.2f' % math.degrees(.2)))

    def test_forward_and_reverse_limits_are_independent(self):
        driver = self.driver(max_forward_speed_mps=1.2, max_reverse_speed_mps=.6)
        for speed, limit in ((20., 1.2), (-20., .6)):
            wheel, angle = self.command(driver, speed, .1)
            self.assertLessEqual(abs(wheel)*driver.wheel_radius, limit+1.e-12)
            self.assertGreater(abs(wheel), 0.)
            self.assertEqual(wheel>0, speed>0)

    def test_exact_default_speed_boundaries(self):
        driver = self.driver()
        for speed in (1.5, -1.5):
            wheel, _ = self.command(driver, speed)
            self.assertEqual(wheel, 11.76 if speed>0 else -11.76)

    def test_rounding_does_not_exceed_custom_speed_limits(self):
        driver = self.driver(max_forward_speed_mps=1.234, max_reverse_speed_mps=.987)
        for speed, limit in ((10., 1.234), (-10., .987)):
            wheel, _ = self.command(driver, speed)
            self.assertLessEqual(abs(wheel)*driver.wheel_radius, limit+1.e-12)

    def test_huge_finite_controls_are_clamped_before_conversion(self):
        driver = self.driver()
        for speed in (1.e308, -1.e308):
            for angle in (1.e308, -1.e308):
                wheel, degrees = self.command(driver, speed, angle)
                self.assertLessEqual(abs(wheel)*driver.wheel_radius, 1.5)
                self.assertEqual(degrees, 25. if angle>0 else -25.)

    def test_dead_zone_and_stop_reset_steering(self):
        driver = self.driver()
        for speed in (0., -0., .04999, -.04999):
            self.assertEqual(self.command(driver, speed, .3), (0., 0.))
        for speed in (.05, -.05):
            wheel, angle = self.command(driver, speed, .3)
            self.assertNotEqual(wheel, 0.)
            self.assertNotEqual(angle, 0.)

    def test_original_steering_boundaries_and_stop_behavior(self):
        driver = self.driver()
        for degrees in (25., -25., 25.01, -25.01, 0., 10., -10.):
            _, sent = self.command(driver, .8, math.radians(degrees))
            self.assertEqual(sent, max(-25., min(25., degrees)))
        self.assertEqual(self.command(driver, 0., math.radians(25.)), (0., 0.))

    def test_invalid_configuration_fails_before_opening_serial_or_subscribing(self):
        for name in ('wheel_radius', 'max_forward_speed_mps', 'max_reverse_speed_mps'):
            for bad in (0., -1., math.nan, math.inf, -math.inf, None, 'bad', True):
                with self.subTest(parameter=name, value=bad):
                    self.params = {'~'+name:bad}
                    base_driver_node.serial.Serial.reset_mock()
                    rospy.Subscriber.reset_mock()
                    with self.assertRaises(ValueError):
                        RobotBaseDriver()
                    base_driver_node.serial.Serial.assert_not_called()
                    rospy.Subscriber.assert_not_called()

    def test_unrepresentable_wheel_speed_limit_fails_before_serial(self):
        for params in ({'wheel_radius':1.e-320},
                       {'wheel_radius':1.e308, 'max_forward_speed_mps':1.e-320}):
            with self.subTest(params=params):
                self.params = {'~'+k:v for k,v in params.items()}
                base_driver_node.serial.Serial.reset_mock()
                with self.assertRaises(ValueError):
                    RobotBaseDriver()
                base_driver_node.serial.Serial.assert_not_called()

    def test_valid_radius_and_numeric_parameter_overrides(self):
        driver = self.driver(wheel_radius=.2, max_forward_speed_mps='1.0', max_reverse_speed_mps=.5)
        self.assertEqual(self.command(driver, .8, 0.), (4., 0.))
        self.assertEqual(self.command(driver, -2., 0.), (-2.5, 0.))

    def test_serial_boundary_also_rejects_nonfinite_and_malformed_controls(self):
        driver = self.driver()
        for bad in (math.nan, math.inf, -math.inf, None, 'bad'):
            for wheel, angle in ((bad, 10.), (2., bad), (bad, bad)):
                driver.send_serial_command(wheel, angle)
                self.assertEqual(self.packet(), (0., 0.))

    def test_serial_boundary_clamps_direct_callers(self):
        driver = self.driver(max_forward_speed_mps=1., max_reverse_speed_mps=.5)
        for wheel, angle in ((1.e308, 1.e308), (-1.e308, -1.e308)):
            driver.send_serial_command(wheel, angle)
            sent, degrees = self.packet()
            limit=1. if wheel>0 else .5
            self.assertLessEqual(abs(sent)*driver.wheel_radius, limit+1.e-12)
            self.assertEqual(degrees, 25. if angle>0 else -25.)
        driver.send_serial_command(0., 20.)
        self.assertEqual(self.packet(), (0., 0.))

    def test_motor_alarm_remains_telemetry_only(self):
        driver = self.driver()
        driver.handle_motor_line('MOTOR,LT:8.0,RT:0.0,LMOS:110.0,LCOIL:100.0,'
                                 'RMOS:20.0,RCOIL:20.0,LSTATE:1,RSTATE:1')
        self.assertTrue(driver.motor_alarm_active)
        self.serial.write.assert_not_called()
        self.assertEqual(self.command(driver, .8, .1), (6.27, 5.73))
        self.assertTrue(driver.motor_alarm_active)

    def test_auto_and_manual_selected_commands_share_the_final_gate(self):
        from control_mode_mux import ControlModeMux
        driver = self.driver()
        mux = ControlModeMux()
        for mode in (mux.AUTO, mux.MANUAL):
            mux.mode = mode
            for value in (math.nan, math.inf, -math.inf, 20., -20., .8, -.8, 0.):
                msg = Twist()
                msg.linear.x, msg.angular.z = value, .1
                (mux.auto_callback if mode==mux.AUTO else mux.remote_callback)(msg)
                selected, reason = mux.selected_command(rospy.Time(100))
                self.assertEqual(reason, 'auto_active' if mode==mux.AUTO else 'remote_active')
                driver.cmd_vel_callback(selected)
                wheel, degrees = self.packet()
                if not math.isfinite(value) or value==0.:
                    self.assertEqual((wheel,degrees),(0.,0.))
                else:
                    self.assertLessEqual(abs(wheel)*driver.wheel_radius,1.5)

    def test_existing_localization_stop_is_immediate_and_unchanged(self):
        driver = self.driver(enable_localization_safety_stop=True)
        self.assertEqual(self.packet(), (0., 0.))
        self.assertEqual(self.command(driver, .8, .2), (0., 0.))
        driver.localization_safe = True
        self.assertNotEqual(self.command(driver, -.8, -.2), (0., 0.))
        self.assertEqual(self.command(driver, math.nan, .2), (0., 0.))

    def test_unused_twist_fields_do_not_change_ackermann_interface(self):
        driver = self.driver()
        msg = Twist()
        msg.linear.x=.8
        msg.linear.y=math.nan
        msg.angular.x=math.inf
        msg.angular.z=.1
        driver.cmd_vel_callback(msg)
        self.assertEqual(self.packet(), (6.27, 5.73))

    def test_random_inputs_keep_every_serial_field_finite_and_bounded(self):
        driver = self.driver(max_forward_speed_mps=1.234, max_reverse_speed_mps=.987)
        rng=random.Random(20261019)
        for _ in range(500):
            v=rng.uniform(-1.e5,1.e5);angle=rng.uniform(-1.e5,1.e5)
            wheel,degrees=self.command(driver,v,angle)
            self.assertLessEqual(wheel*driver.wheel_radius,1.234+1.e-12)
            self.assertGreaterEqual(wheel*driver.wheel_radius,-.987-1.e-12)
            self.assertLessEqual(abs(degrees),25.)


class DriverLaunchLimitTests(unittest.TestCase):
    def test_main_entrypoints_pass_defaults_and_overrides_without_starting_nodes(self):
        import roslaunch
        root=Path(__file__).resolve().parents[3]
        entries=(('src/robot_base_driver/launch/driver.launch', []),
                 ('src/robot_base_driver/launch/navigation_core.launch', []),
                 ('src/neupan_ros/example/lxw_car/launch/full_system_run.launch', ['outdoor_mode:=false']),
                 ('src/neupan_ros/example/lxw_car/launch/full_system_run.launch', ['outdoor_mode:=true']))
        for path,mode_args in entries:
            for overrides in ([], ['max_forward_speed_mps:=1.2','max_reverse_speed_mps:=0.6',
                                   'wheel_radius:=0.2']):
                args = mode_args+overrides
                with self.subTest(path=path,args=args):
                    config=roslaunch.config.load_config_default([(str(root/path),args)],None,verbose=False)
                    expected=(1.2,.6,.2) if overrides else (1.5,1.5,.1275)
                    for key,value in zip(('max_forward_speed_mps','max_reverse_speed_mps','wheel_radius'),expected):
                        self.assertAlmostEqual(config.params['/robot_base_driver/'+key].value,value)

    def test_legacy_real_car_entrypoints_inherit_final_driver_protection(self):
        import roslaunch
        root=Path(__file__).resolve().parents[3]
        for path in ('src/neupan_ros/example/lxw_car/launch/neupan_lxwcar.launch',
                     'src/neupan_ros/example/lxw_car/launch/neupan_lxwcar_norviz.launch',
                     'src/neupan_ros/example/lxw_car/launch/outdoor_global_only_test.launch',
                     'src/teb_test/launch/outdoor_teb_test.launch',
                     'src/Test/launch/open_loop_motion_record.launch'):
            with self.subTest(path=path):
                config=roslaunch.config.load_config_default([(str(root/path),[])],None,verbose=False)
                for key in ('max_forward_speed_mps','max_reverse_speed_mps'):
                    self.assertEqual(config.params['/robot_base_driver/'+key].value,1.5)


if __name__ == '__main__':
    unittest.main()
